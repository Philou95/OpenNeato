"""Config flow for OpenNeato integration."""

from __future__ import annotations

import logging
from typing import Any

import voluptuous as vol

from homeassistant.config_entries import ConfigFlow, ConfigFlowResult, OptionsFlow
from homeassistant.core import callback
from homeassistant.helpers.aiohttp_client import async_get_clientsession

from .api import OpenNeatoApiClient, OpenNeatoApiError, OpenNeatoConnectionError
from .const import (
    CONF_FLOORPLAN_IMAGE,
    CONF_FLOORPLAN_ORIGIN_X,
    CONF_FLOORPLAN_ORIGIN_Y,
    CONF_FLOORPLAN_ROTATION,
    CONF_FLOORPLAN_SCALE,
    DOMAIN,
    FLOORPLAN_DEFAULT_ROTATION,
    FLOORPLAN_DEFAULT_SCALE,
    FLOORPLAN_SCALE_MAX,
    FLOORPLAN_SCALE_MIN,
    CONF_HOST,
)

_LOGGER = logging.getLogger(__name__)

STEP_USER_DATA_SCHEMA = vol.Schema(
    {
        vol.Required(CONF_HOST): str,
    }
)


class OpenNeatoConfigFlow(ConfigFlow, domain=DOMAIN):
    """Handle a config flow for OpenNeato."""

    VERSION = 1

    async def async_step_user(
        self, user_input: dict[str, Any] | None = None
    ) -> ConfigFlowResult:
        """Handle the initial step."""
        errors: dict[str, str] = {}

        if user_input is not None:
            host = user_input[CONF_HOST]
            session = async_get_clientsession(self.hass)
            api = OpenNeatoApiClient(host, session)

            try:
                firmware_info = await api.get_firmware_version()
                robot_info = await api.get_robot_version()
            except OpenNeatoConnectionError:
                errors["base"] = "cannot_connect"
            except (OpenNeatoApiError, Exception):
                _LOGGER.exception("Unexpected exception during config flow")
                errors["base"] = "unknown"
            else:
                serial = robot_info.get("serialNumber", "")
                model = robot_info.get("modelName")
                software_version = robot_info.get("softwareVersion")
                firmware_version = firmware_info.get("version")

                if not serial:
                    errors["base"] = "unknown"
                    return self.async_show_form(
                        step_id="user",
                        data_schema=STEP_USER_DATA_SCHEMA,
                        errors=errors,
                    )

                await self.async_set_unique_id(serial)
                self._abort_if_unique_id_configured()

                return self.async_create_entry(
                    title=model or f"OpenNeato ({host})",
                    data={
                        CONF_HOST: host,
                        "serial": serial,
                        "model": model,
                        "firmware_version": firmware_version,
                        "software_version": software_version,
                    },
                )

        return self.async_show_form(
            step_id="user",
            data_schema=STEP_USER_DATA_SCHEMA,
            errors=errors,
        )


# ── Floorplan background options ────────────────────────────────────────────

class OpenNeatoOptionsFlowHandler(OptionsFlow):
    """Handle floorplan background options.

    Lets the user point at a local PNG/JPG plan and calibrate its alignment
    (origin, rotation, scale) to the robot's odometry frame. Saved values
    live in entry.options and are read by the camera on every render.
    """

    async def async_step_init(
        self, user_input: dict[str, Any] | None = None
    ) -> ConfigFlowResult:
        """Manage the floorplan options."""
        errors: dict[str, str] = {}

        if user_input is not None:
            path = (user_input.get(CONF_FLOORPLAN_IMAGE) or "").strip()
            # When a path is provided, require it to be absolute — the
            # renderer reads it from disk directly, so a relative path
            # would resolve against HA's working dir (unstable).
            if path and not path.startswith("/"):
                errors[CONF_FLOORPLAN_IMAGE] = "path_must_be_absolute"
            else:
                # Persist; empty path disables the floorplan (renders the
                # solid background again). Defaults are merged in so the
                # user only fills the fields they care about.
                merged = {**self.config_entry.options, **user_input}
                if not path:
                    merged.pop(CONF_FLOORPLAN_IMAGE, None)
                return self.async_create_entry(title="", data=merged)

        return self.async_show_form(
            step_id="init",
            data_schema=self._build_schema(),
            errors=errors,
        )

    def _build_schema(self) -> vol.Schema:
        """Schema pre-filled with current option values (or defaults)."""
        opts = self.config_entry.options
        defaults = {
            CONF_FLOORPLAN_IMAGE: opts.get(CONF_FLOORPLAN_IMAGE, ""),
            CONF_FLOORPLAN_ORIGIN_X: opts.get(CONF_FLOORPLAN_ORIGIN_X, 0.0),
            CONF_FLOORPLAN_ORIGIN_Y: opts.get(CONF_FLOORPLAN_ORIGIN_Y, 0.0),
            CONF_FLOORPLAN_ROTATION: opts.get(
                CONF_FLOORPLAN_ROTATION, FLOORPLAN_DEFAULT_ROTATION
            ),
            CONF_FLOORPLAN_SCALE: opts.get(
                CONF_FLOORPLAN_SCALE, FLOORPLAN_DEFAULT_SCALE
            ),
        }
        return vol.Schema(
            {
                vol.Optional(CONF_FLOORPLAN_IMAGE, default=defaults[CONF_FLOORPLAN_IMAGE]): str,
                vol.Optional(
                    CONF_FLOORPLAN_ORIGIN_X, default=defaults[CONF_FLOORPLAN_ORIGIN_X]
                ): vol.All(vol.Coerce(float), vol.Range(min=-100.0, max=100.0)),
                vol.Optional(
                    CONF_FLOORPLAN_ORIGIN_Y, default=defaults[CONF_FLOORPLAN_ORIGIN_Y]
                ): vol.All(vol.Coerce(float), vol.Range(min=-100.0, max=100.0)),
                vol.Optional(
                    CONF_FLOORPLAN_ROTATION, default=defaults[CONF_FLOORPLAN_ROTATION]
                ): vol.All(vol.Coerce(float), vol.Range(min=-360.0, max=360.0)),
                vol.Optional(
                    CONF_FLOORPLAN_SCALE, default=defaults[CONF_FLOORPLAN_SCALE]
                ): vol.All(
                    vol.Coerce(float),
                    vol.Range(min=FLOORPLAN_SCALE_MIN, max=FLOORPLAN_SCALE_MAX),
                ),
            }
        )


@staticmethod
@callback
def async_get_options_flow(config_entry) -> OpenNeatoOptionsFlowHandler:
    """Get the options flow for this handler."""
    return OpenNeatoOptionsFlowHandler()
