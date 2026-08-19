"""Vacuum entity for the OpenNeato integration."""

from __future__ import annotations

import asyncio
import logging
from typing import Any

from homeassistant.components.vacuum import (
    StateVacuumEntity,
    VacuumActivity,
    VacuumEntityFeature,
)
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .api import OpenNeatoApiClient
from .const import DOMAIN, FAN_SPEEDS, UISTATE_SUBSTRINGS, is_real_error
from .entity import OpenNeatoEntity

_LOGGER = logging.getLogger(__name__)

# Seconds to let the robot reach a running state before asking it to go home.
# Measured on a Botvac D6: the UI state machine moves through
# STARTHOUSECLEANING into HOUSECLEANINGRUNNING in well under two seconds, and
# SEND_TO_BASE issued before that is dropped.
RETURN_HOME_RESTART_DELAY = 3.0

SUPPORTED_FEATURES = (
    VacuumEntityFeature.START
    | VacuumEntityFeature.STOP
    | VacuumEntityFeature.PAUSE
    | VacuumEntityFeature.RETURN_HOME
    | VacuumEntityFeature.CLEAN_SPOT
    | VacuumEntityFeature.LOCATE
    | VacuumEntityFeature.FAN_SPEED
    | VacuumEntityFeature.SEND_COMMAND
    | VacuumEntityFeature.STATE
)


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up OpenNeato vacuum from a config entry."""
    data = hass.data[DOMAIN][entry.entry_id]
    async_add_entities(
        [
            OpenNeatoVacuum(
                coordinator=data["coordinator"],
                api=data["api"],
                serial=data["serial"],
                model=data["model"],
                sw_version=data["sw_version"],
                fw_version=data["fw_version"],
                host=data["host"],
            )
        ]
    )


class OpenNeatoVacuum(OpenNeatoEntity, StateVacuumEntity):
    """Representation of an OpenNeato vacuum cleaner."""

    _attr_supported_features = SUPPORTED_FEATURES
    _attr_fan_speed_list = FAN_SPEEDS
    _attr_name = None  # Use device name

    def __init__(
        self,
        coordinator,
        api: OpenNeatoApiClient,
        serial: str,
        model: str | None = None,
        sw_version: str | None = None,
        fw_version: str | None = None,
        host: str | None = None,
    ) -> None:
        """Initialize the vacuum entity."""
        super().__init__(
            coordinator, serial,
            model=model, sw_version=sw_version, fw_version=fw_version, host=host,
        )
        self._api = api
        self._attr_unique_id = f"{serial}_vacuum"

    @property
    def activity(self) -> VacuumActivity | None:
        """Return the current vacuum activity."""
        if not self.coordinator.data:
            return None

        state_data = self.coordinator.data.get("state", {})
        charger_data = self.coordinator.data.get("charger", {})
        error_data = self.coordinator.data.get("error", {})

        ui_state = state_data.get("uiState", "")

        # What the robot is *doing* outranks what it is *complaining about*.
        #
        # GetErr latches: it keeps reporting the last fault until something
        # clears it. A robot that failed to undock (282
        # UI_ERROR_NAVIGATION_UndockingFailed), then cleaned anyway, then set
        # off home, still reports 282 the whole way -- so checking the error
        # first painted a perfectly healthy return to base as ERROR on every
        # run. The robot's own state machine had already moved on.
        #
        # A genuine failure still shows: when the robot really is stuck it
        # leaves the running/docking states, no substring matches, and the
        # error check below takes over. The message stays available as the
        # error_message attribute either way.
        #
        # Match using substrings, same as the firmware frontend.
        for substring, activity in UISTATE_SUBSTRINGS:
            if substring in ui_state:
                return activity

        if is_real_error(error_data):
            return VacuumActivity.ERROR

        # For unmapped states, use charger to distinguish docked vs idle
        if charger_data.get("chargingActive") or charger_data.get("extPwrPresent"):
            return VacuumActivity.DOCKED

        return VacuumActivity.IDLE

    @property
    def fan_speed(self) -> str | None:
        """Return the current fan speed."""
        if not self.coordinator.data:
            return None
        settings = self.coordinator.data.get("user_settings", {})
        if settings.get("ecoMode"):
            return "eco"
        if settings.get("intenseClean"):
            return "intense"
        return "normal"

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        """Return extra state attributes."""
        attrs: dict[str, Any] = {}
        if not self.coordinator.data:
            return attrs

        state_data = self.coordinator.data.get("state", {})
        error_data = self.coordinator.data.get("error", {})

        if robot_state := state_data.get("robotState"):
            attrs["robot_state"] = robot_state
        if ui_state := state_data.get("uiState"):
            attrs["ui_state"] = ui_state

        # Keep the two apart: "Returning to base" and "Cleaning complete" are
        # worth surfacing, but calling them errors is what made a normal dock
        # look like a failure.
        if is_real_error(error_data):
            attrs["error_message"] = error_data.get("displayMessage", "Unknown error")
            attrs["error_code"] = error_data.get("errorCode")
        elif error_data.get("hasError"):
            attrs["alert_message"] = error_data.get("displayMessage", "")
            attrs["alert_code"] = error_data.get("errorCode")

        return attrs

    # -- Commands ----------------------------------------------------------

    async def async_start(self, **kwargs: Any) -> None:
        """Start or resume house cleaning."""
        await self._api.clean("house")
        await self.coordinator.async_request_refresh()

    async def async_stop(self, **kwargs: Any) -> None:
        """Stop cleaning.

        Stop ends the run outright. The robot keeps no way back to where it
        started, and the next clean begins a fresh exploration from a new
        origin -- which the LIDAR mapper then has to realign onto the
        accumulated floorplan. If the intent is "come back now", pause and
        return to base instead: that route keeps localisation.
        """
        if self.activity == VacuumActivity.CLEANING:
            _LOGGER.info(
                "Stopping mid-clean discards the robot's localisation; pause "
                "then return to base if you want it home with its frame intact"
            )
        await self._api.clean("stop")
        await self.coordinator.async_request_refresh()

    async def async_pause(self, **kwargs: Any) -> None:
        """Pause cleaning."""
        await self._api.clean("pause")
        await self.coordinator.async_request_refresh()

    async def async_return_to_base(self, **kwargs: Any) -> None:
        """Return to dock, from a stopped robot as well as a running one.

        `dock` sends UIMGR_EVENT_SMARTAPP_SEND_TO_BASE, which the robot only
        acts on while a clean is in progress or paused. After Stop the run is
        over, so the event lands on a robot with nothing to return from and it
        simply sits there -- with, per docs/neato-serial-protocol.md, its
        localisation already discarded and its origin re-zeroed wherever it
        happened to be. Pressing "return to base" then looks broken.

        So when the robot is idle and off the dock, briefly restart cleaning
        to give the state machine a run to end, then send it home. The brush
        spins for a second or two on the way; that is the cost of the button
        doing what it says from any state.

        That recovery is not free: the protocol notes say a bare `Clean` also
        resets the robot's position, so the restart discards localisation just
        as Stop did. There is no lossless way home once a run has been
        stopped -- a robot that comes back beats a preserved frame, and the
        LIDAR mapper realigns the next session anyway. To keep localisation,
        **pause** instead of stopping, then return to base: from PAUSED this
        method sends the event straight through with no restart.
        """
        if self.activity == VacuumActivity.DOCKED:
            _LOGGER.debug("Return to base ignored: already docked")
            return

        if self.activity in (VacuumActivity.CLEANING, VacuumActivity.PAUSED):
            await self._api.clean("dock")
            await self.coordinator.async_request_refresh()
            return

        _LOGGER.info(
            "Robot is %s and off the dock; restarting a clean so it has a run "
            "to return from, then sending it to base",
            self.activity,
        )
        await self._api.clean("house")
        # The state machine needs to actually reach a running state before it
        # will accept the event; sending both back to back is ignored.
        await asyncio.sleep(RETURN_HOME_RESTART_DELAY)
        await self._api.clean("dock")
        await self.coordinator.async_request_refresh()

    async def async_clean_spot(self, **kwargs: Any) -> None:
        """Start spot cleaning."""
        await self._api.clean("spot")
        await self.coordinator.async_request_refresh()

    async def async_locate(self, **kwargs: Any) -> None:
        """Locate the vacuum by playing an alert sound."""
        await self._api.play_sound(19)

    async def async_set_fan_speed(self, fan_speed: str, **kwargs: Any) -> None:
        """Set the fan speed."""
        if fan_speed == "eco":
            await self._api.set_user_setting("EcoMode", "ON")
            await self._api.set_user_setting("IntenseClean", "OFF")
        elif fan_speed == "intense":
            await self._api.set_user_setting("EcoMode", "OFF")
            await self._api.set_user_setting("IntenseClean", "ON")
        else:
            await self._api.set_user_setting("EcoMode", "OFF")
            await self._api.set_user_setting("IntenseClean", "OFF")
        await self.coordinator.async_request_refresh()

    async def async_send_command(
        self, command: str, params: dict[str, Any] | list[Any] | None = None, **kwargs: Any
    ) -> None:
        """Send a raw serial command to the robot."""
        await self._api.send_serial_command(command)
        await self.coordinator.async_request_refresh()
