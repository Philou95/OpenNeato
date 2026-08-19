"""Camera entity exposing the LIDAR map for OpenNeato.

Provides a standard HA camera entity that renders the robot's map as a PNG
image.  Compatible with vacuum-card, picture-entity, and other Lovelace cards
that consume camera entities.

Adaptive content based on robot state:
  - Cleaning: in-progress cleaning session map (path + coverage, polled every 2 s).
  - Manual mode: live 360-degree LIDAR scan (polled every 2 s).
  - Docked / idle: most recent completed cleaning session map (path + coverage).
  - No history: placeholder grid image.

Rendering runs in the executor to avoid blocking the HA event loop.
"""

from __future__ import annotations

import logging
from datetime import timedelta
from typing import Any

from homeassistant.components.camera import Camera
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import CALLBACK_TYPE, HomeAssistant, callback
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.event import async_track_time_interval

from .api import OpenNeatoApiClient
from .const import (
    DOMAIN,
    HISTORY_IMAGE_SIZE,
    HISTORY_POLL_INTERVAL,
    LIDAR_POLL_INTERVAL,
)
from .coordinator import latest_completed_session
from .entity import OpenNeatoEntity
from .history_renderer import (
    parse_session_jsonl,
    render_history_animation,
    render_history_map,
)
from .lidar_renderer import (
    ScanAccumulator,
    render_idle_gif,
    render_idle_image,
    render_lidar_scan,
)

_LOGGER = logging.getLogger(__name__)

# uiState substrings that indicate the robot is cleaning (show history map)
_CLEANING_SUBSTRINGS = {"CLEANINGRUNNING", "CLEANINGPAUSED", "CLEANINGSUSPENDED", "DOCKING"}
# uiState substrings that indicate manual mode (show LIDAR scan)
_MANUAL_SUBSTRINGS = {"MANUALCLEANING"}


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up the OpenNeato map cameras from a config entry."""
    data = hass.data[DOMAIN][entry.entry_id]
    common = {
        "coordinator": data["coordinator"],
        "api": data["api"],
        "serial": data["serial"],
        "model": data["model"],
        "sw_version": data["sw_version"],
        "fw_version": data["fw_version"],
        "host": data["host"],
        # a reload (triggered on options change) is picked up live.
        "options": dict(entry.options),
    }
    async_add_entities(
        [
            OpenNeatoLidarCamera(**common),
            OpenNeatoMotionCamera(**common),
        ]
    )


class OpenNeatoLidarCamera(OpenNeatoEntity, Camera):
    """Camera entity showing the live LIDAR scan or last cleaning session map."""

    # Custom integrations don't read strings.json at runtime, so set the
    # display name directly alongside the translation key (sensors/switches
    # in this codebase do the same).
    _attr_name = "LIDAR map"
    _attr_translation_key = "lidar_map"
    _attr_frame_interval = 5.0
    _attr_content_type = "image/png"

    def __init__(
        self,
        coordinator,
        api: OpenNeatoApiClient,
        serial: str,
        model: str | None = None,
        sw_version: str | None = None,
        fw_version: str | None = None,
        host: str | None = None,
        options: dict[str, Any] | None = None,
    ) -> None:
        """Initialize the LIDAR map camera."""
        OpenNeatoEntity.__init__(
            self, coordinator, serial,
            model=model, sw_version=sw_version, fw_version=fw_version, host=host,
        )
        Camera.__init__(self)
        self._api = api
        self._attr_unique_id = f"{serial}_lidar_map"
        # Floorplan background (None disables it → solid color + grid).

        # LIDAR scan state (manual mode only)
        self._accumulator = ScanAccumulator()
        self._lidar_image: bytes | None = None
        self._lidar_poll_unsub: CALLBACK_TYPE | None = None
        self._lidar_polling_active = False

        # History map state (cleaning + idle)
        self._history_image: bytes | None = None
        self._history_session_name: str | None = None  # track which session is cached
        self._history_poll_unsub: CALLBACK_TYPE | None = None
        self._history_polling_active = False
        # Guards against two concurrent fetches of the same session (e.g.
        # async_added_to_hass and the first coordinator update both firing
        # _check_polling_state before the first fetch finishes). The ESP32
        # bridge reads history off a blocking serial link to the robot, and
        # a second concurrent request for the same file can stall it
        # indefinitely rather than erroring — so this must be prevented
        # client-side rather than relying on the server to reject it.
        self._history_fetch_in_progress = False
        # Name of a completed session we already tried and could not turn
        # into a map (download, parse or render failed, or it carried no
        # pose data). Without this every coordinator tick would re-download
        # the whole JSONL for the same doomed session, forever.
        self._history_failed_session: str | None = None

        # Idle placeholder
        self._idle_image: bytes | None = None

        # Current display mode
        self._map_source: str = "idle"  # "lidar" | "history" | "idle"

        # LIDAR diagnostics exposed as attributes
        self._rotation_speed: float | None = None
        self._valid_points: int | None = None

        # History diagnostics
        self._session_mode: str | None = None
        self._session_duration: int | None = None
        self._session_area: float | None = None

    # ── Properties ───────────────────────────────────────────────────

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        """Return map diagnostics."""
        attrs: dict[str, Any] = {"map_source": self._map_source}
        if self._map_source == "lidar":
            if self._rotation_speed is not None:
                attrs["rotation_speed"] = round(self._rotation_speed, 1)
            if self._valid_points is not None:
                attrs["valid_points"] = self._valid_points
                attrs["scan_quality"] = round(self._valid_points / 360 * 100)
        elif self._map_source == "history":
            if self._history_session_name:
                attrs["session_name"] = self._history_session_name
            if self._session_mode:
                attrs["session_mode"] = self._session_mode
            if self._session_duration is not None:
                attrs["session_duration"] = self._session_duration
            if self._session_area is not None:
                attrs["session_area"] = round(self._session_area, 2)
        return attrs

    # ── Camera interface ─────────────────────────────────────────────

    async def async_camera_image(
        self, width: int | None = None, height: int | None = None
    ) -> bytes | None:
        """Return the current map image based on robot state."""
        # Active LIDAR (manual mode) takes priority
        if self._lidar_polling_active and self._lidar_image is not None:
            return self._lidar_image

        # History map (in-progress cleaning or idle)
        if self._history_image is not None:
            return self._history_image

        # Last resort: LIDAR image from before robot stopped
        if self._lidar_image is not None:
            return self._lidar_image

        # Placeholder
        if self._idle_image is None:
            self._idle_image = await self.hass.async_add_executor_job(
                render_idle_image
            )
        return self._idle_image

    # ── Lifecycle ────────────────────────────────────────────────────

    async def async_added_to_hass(self) -> None:
        """Start listening for coordinator updates."""
        await super().async_added_to_hass()
        # _check_polling_state handles the idle case (loads the latest
        # completed session) and starts LIDAR/history polling otherwise.
        self._check_polling_state()

    async def async_will_remove_from_hass(self) -> None:
        """Clean up the poll timers."""
        self._stop_lidar_polling()
        self._stop_history_polling()
        await super().async_will_remove_from_hass()

    @callback
    def _handle_coordinator_update(self) -> None:
        """React to coordinator data changes — manage polling and map display."""
        self._check_polling_state()
        super()._handle_coordinator_update()

    # ── State detection ──────────────────────────────────────────────

    def _get_ui_state(self) -> str:
        """Return the current uiState string."""
        if not self.coordinator.data:
            return ""
        return self.coordinator.data.get("state", {}).get("uiState", "")

    def _is_robot_cleaning(self) -> bool:
        """Return True if the robot is actively cleaning (show history map)."""
        ui_state = self._get_ui_state()
        return any(sub in ui_state for sub in _CLEANING_SUBSTRINGS)

    def _is_robot_manual(self) -> bool:
        """Return True if the robot is in manual drive mode (show LIDAR)."""
        ui_state = self._get_ui_state()
        return any(sub in ui_state for sub in _MANUAL_SUBSTRINGS)

    # ── Polling lifecycle ────────────────────────────────────────────

    @callback
    def _check_polling_state(self) -> None:
        """Start or stop the appropriate polling based on robot state."""
        cleaning = self._is_robot_cleaning()
        manual = self._is_robot_manual()
        was_cleaning = self._history_polling_active
        was_manual = self._lidar_polling_active

        # Manual mode → LIDAR polling
        if manual and not self._lidar_polling_active:
            self._stop_history_polling()
            self._start_lidar_polling()
        elif not manual and self._lidar_polling_active:
            self._stop_lidar_polling()

        # Cleaning → history map polling (in-progress session)
        if cleaning and not self._history_polling_active:
            self._stop_lidar_polling()
            self._start_history_polling()
        elif not cleaning and self._history_polling_active:
            self._stop_history_polling()

        # Idle: keep the latest completed session map on screen. Re-checks
        # on every coordinator tick so the map appears whenever a new
        # session lands (or after startup if coordinator data arrived
        # after async_added_to_hass already ran).
        if not cleaning and not manual:
            if was_cleaning or was_manual:
                self._history_session_name = None  # force re-fetch
                self._history_failed_session = None  # and give it one retry
            self._map_source = "history"
            latest = latest_completed_session(
                self.coordinator.data.get("history") if self.coordinator.data else None
            )
            latest_name = latest.get("name") if latest else None
            if (
                latest_name
                and latest_name != self._history_session_name
                and latest_name != self._history_failed_session
            ):
                self.hass.async_create_task(
                    self._async_update_history_map(allow_recording=False)
                )

    @callback
    def _start_lidar_polling(self) -> None:
        """Start the periodic LIDAR fetch timer (manual mode)."""
        _LOGGER.debug("Starting LIDAR polling (manual mode)")
        self._lidar_polling_active = True
        self._map_source = "lidar"
        self._lidar_poll_unsub = async_track_time_interval(
            self.hass,
            self._async_poll_lidar,
            timedelta(seconds=LIDAR_POLL_INTERVAL),
        )
        self.hass.async_create_task(self._async_poll_lidar())

    @callback
    def _stop_lidar_polling(self) -> None:
        """Cancel the periodic LIDAR fetch timer."""
        if self._lidar_poll_unsub is not None:
            _LOGGER.debug("Stopping LIDAR polling")
            self._lidar_poll_unsub()
            self._lidar_poll_unsub = None
        self._lidar_polling_active = False

    @callback
    def _start_history_polling(self) -> None:
        """Start polling the in-progress cleaning session map."""
        _LOGGER.debug("Starting history map polling (cleaning in progress)")
        self._history_polling_active = True
        self._map_source = "history"
        self._history_poll_unsub = async_track_time_interval(
            self.hass,
            self._async_poll_history,
            timedelta(seconds=HISTORY_POLL_INTERVAL),
        )
        self.hass.async_create_task(self._async_poll_history())

    @callback
    def _stop_history_polling(self) -> None:
        """Cancel the periodic history map fetch timer."""
        if self._history_poll_unsub is not None:
            _LOGGER.debug("Stopping history map polling")
            self._history_poll_unsub()
            self._history_poll_unsub = None
        self._history_polling_active = False

    async def _async_poll_history(self, _now=None) -> None:
        """Fetch the in-progress recording session and re-render the map."""
        await self._async_update_history_map(allow_recording=True)

    async def _async_poll_lidar(self, _now=None) -> None:
        """Fetch LIDAR data from the ESP32 and re-render the image."""
        try:
            data = await self._api.get_lidar()
        except Exception:
            _LOGGER.debug("LIDAR fetch failed, keeping previous image", exc_info=True)
            return

        points = data.get("points", [])
        self._rotation_speed = data.get("rotationSpeed")
        self._valid_points = data.get("validPoints")
        self._map_source = "lidar"

        # In manual mode the robot is always "moving" when wheels are driven
        self._accumulator.merge(points, True)
        snapshot = self._accumulator.snapshot()

        self._lidar_image = await self.hass.async_add_executor_job(
            render_lidar_scan, snapshot
        )
        self.async_write_ha_state()

    # ── History map ──────────────────────────────────────────────────

    def _get_latest_session(self, allow_recording: bool = False) -> dict[str, Any] | None:
        """Find the most recent session from coordinator data.

        When allow_recording is True, prefer the active recording session
        (in-progress cleaning). When False, skip recording sessions.
        """
        if not self.coordinator.data:
            return None
        history = self.coordinator.data.get("history", [])
        if allow_recording and isinstance(history, list):
            for session in history:
                if (isinstance(session, dict)
                        and session.get("recording")
                        and session.get("name")):
                    return session
        return latest_completed_session(history)

    async def _async_update_history_map(self, allow_recording: bool = False) -> None:
        """Fetch and render a cleaning session map.

        When allow_recording is True, prefer the in-progress recording session.
        When False, only show completed sessions (idle mode).
        """
        if self._history_fetch_in_progress:
            _LOGGER.debug(
                "History map fetch already in progress, skipping overlapping call"
            )
            return

        session_info = self._get_latest_session(allow_recording=allow_recording)
        if not session_info:
            if not self._lidar_polling_active and not self._history_polling_active:
                self._map_source = "idle"
                self.async_write_ha_state()
            return

        session_name = session_info["name"]
        recording = session_info.get("recording", False)

        # Don't re-fetch completed sessions we already have cached.
        # Always re-fetch recording sessions (data is growing).
        if (
            not recording
            and session_name == self._history_session_name
            and self._history_image is not None
        ):
            if not self._lidar_polling_active:
                self._map_source = "history"
                self.async_write_ha_state()
            return

        self._history_fetch_in_progress = True
        succeeded = False
        try:
            _LOGGER.debug("Fetching history session %s for map rendering", session_name)
            try:
                raw_jsonl = await self._api.get_history_session(session_name)
            except Exception:
                _LOGGER.warning(
                    "LIDAR map: failed to fetch session %s", session_name, exc_info=True
                )
                return

            try:
                parsed = await self.hass.async_add_executor_job(parse_session_jsonl, raw_jsonl)
            except Exception:
                _LOGGER.warning(
                    "LIDAR map: failed to parse session %s", session_name, exc_info=True
                )
                return

            if not parsed.get("path"):
                _LOGGER.debug("Session %s has no path data", session_name)
                return

            try:
                self._history_image = await self.hass.async_add_executor_job(
                    render_history_map, parsed, HISTORY_IMAGE_SIZE, recording
                )
            except Exception:
                _LOGGER.warning(
                    "LIDAR map: failed to render session %s", session_name, exc_info=True
                )
                return

            # Cache session identity and extract metadata
            self._history_session_name = session_name
            summary = session_info.get("summary") or parsed.get("summary") or {}
            session_meta = session_info.get("session") or parsed.get("session") or {}
            self._session_mode = session_meta.get("mode") or summary.get("mode")
            self._session_duration = summary.get("duration")
            self._session_area = summary.get("areaCovered")

            if not self._lidar_polling_active:
                self._map_source = "history"
            self.async_write_ha_state()
            succeeded = True
        finally:
            self._history_fetch_in_progress = False
            # A completed session that failed to render looks identical on
            # the next coordinator tick, so remember it instead of pulling
            # the same JSONL off the robot every few seconds forever. A
            # recording session keeps growing, so it stays retryable.
            if not succeeded and not recording:
                self._history_failed_session = session_name


class OpenNeatoMotionCamera(OpenNeatoEntity, Camera):
    """Animated replay of the most recent completed cleaning session.

    Serves an animated GIF time-lapse that plays once then holds on the
    fully-drawn map. Regenerated only when a newer completed session
    appears; mid-clean the entity keeps serving the previous session's
    animation. GIF generation runs in the executor, behind a render
    flag to prevent stampede from coordinator ticks coinciding with
    camera-image requests.
    """

    _attr_name = "Cleaning replay"
    _attr_translation_key = "motion_map"
    _attr_content_type = "image/gif"
    # The GIF only changes when a new completed session appears, so tell
    # HA to re-request the image sparingly (default is 0.5 s = 2 fps of
    # poll traffic even on a static entity).
    _attr_frame_interval = 300.0

    def __init__(
        self,
        coordinator,
        api: OpenNeatoApiClient,
        serial: str,
        model: str | None = None,
        sw_version: str | None = None,
        fw_version: str | None = None,
        host: str | None = None,
        options: dict[str, Any] | None = None,
    ) -> None:
        OpenNeatoEntity.__init__(
            self, coordinator, serial,
            model=model, sw_version=sw_version, fw_version=fw_version, host=host,
        )
        Camera.__init__(self)
        self._api = api
        self._attr_unique_id = f"{serial}_motion_map"
        # Floorplan background (None disables it → solid color + grid).

        self._gif: bytes | None = None
        self._idle_image: bytes | None = None
        self._cached_session_name: str | None = None
        # Completed session we already tried and couldn't animate (fetch,
        # parse or render failed, or it was too short for a GIF). Retrying
        # it on every coordinator tick would re-download the whole session.
        self._failed_session: str | None = None
        self._rendering = False
        self._session_mode: str | None = None
        self._session_duration: int | None = None
        self._session_area: float | None = None

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        attrs: dict[str, Any] = {"map_source": "motion"}
        if self._cached_session_name:
            attrs["session_name"] = self._cached_session_name
        if self._session_mode:
            attrs["session_mode"] = self._session_mode
        if self._session_duration is not None:
            attrs["session_duration"] = self._session_duration
        if self._session_area is not None:
            attrs["session_area"] = round(self._session_area, 2)
        return attrs

    async def async_camera_image(
        self, width: int | None = None, height: int | None = None
    ) -> bytes | None:
        await self._maybe_refresh()
        if self._gif is not None:
            return self._gif
        if self._idle_image is None:
            self._idle_image = await self.hass.async_add_executor_job(render_idle_gif)
        return self._idle_image

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        # Warm the cache on startup so the first preview request returns
        # a rendered GIF instead of the idle placeholder.
        self.hass.async_create_task(self._maybe_refresh())

    def _latest_completed(self) -> dict[str, Any] | None:
        if not self.coordinator.data:
            return None
        return latest_completed_session(self.coordinator.data.get("history", []))

    async def _maybe_refresh(self) -> None:
        """Regenerate the GIF if a newer completed session is available.

        Claims the render slot (`_rendering = True`) synchronously
        before any await so concurrent callers (coordinator update +
        HA image fetch) can't both enter the fetch-and-render path.
        """
        if self._rendering:
            return
        session_info = self._latest_completed()
        if not session_info:
            return
        session_name = session_info["name"]
        if session_name == self._cached_session_name and self._gif is not None:
            return
        if session_name == self._failed_session:
            return

        self._rendering = True
        succeeded = False
        try:
            try:
                raw_jsonl = await self._api.get_history_session(session_name)
            except Exception:
                _LOGGER.warning(
                    "Motion map: failed to fetch session %s",
                    session_name, exc_info=True,
                )
                return

            try:
                parsed = await self.hass.async_add_executor_job(parse_session_jsonl, raw_jsonl)
            except Exception:
                _LOGGER.warning(
                    "Motion map: failed to parse session %s",
                    session_name, exc_info=True,
                )
                return
            if not parsed.get("path"):
                _LOGGER.info(
                    "Motion map: session %s has no usable path data", session_name
                )
                return
            try:
                gif = await self.hass.async_add_executor_job(
                    render_history_animation, parsed
                )
            except Exception:
                _LOGGER.warning(
                    "Motion map: failed to render session %s",
                    session_name, exc_info=True,
                )
                return
            if gif is None:
                _LOGGER.info(
                    "Motion map: session %s too short to animate", session_name
                )
                return
            self._gif = gif
            self._cached_session_name = session_name
            summary = session_info.get("summary") or parsed.get("summary") or {}
            session_meta = session_info.get("session") or parsed.get("session") or {}
            self._session_mode = session_meta.get("mode") or summary.get("mode")
            self._session_duration = summary.get("duration")
            self._session_area = summary.get("areaCovered")
            self.async_write_ha_state()
            succeeded = True
        finally:
            self._rendering = False
            # Same reasoning as the LIDAR camera: don't re-download a
            # session that already proved unrenderable.
            if not succeeded:
                self._failed_session = session_name

    @callback
    def _handle_coordinator_update(self) -> None:
        # Coordinator tick — check for a newer completed session. The
        # actual render happens in the executor via _maybe_refresh.
        self.hass.async_create_task(self._maybe_refresh())
        super()._handle_coordinator_update()