"""The OpenNeato integration."""

from __future__ import annotations

import logging
from pathlib import Path

from homeassistant.components.http import StaticPathConfig
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import Platform
from homeassistant.core import HomeAssistant
from homeassistant.exceptions import ConfigEntryNotReady
from homeassistant.helpers.aiohttp_client import async_get_clientsession
from homeassistant.loader import async_get_integration

from . import websocket
from .api import OpenNeatoApiClient, OpenNeatoConnectionError
from .const import (
    CONF_HOST,
    CONF_MAP_ENABLED,
    DOMAIN,
    MAP_DEFAULT_ENABLED,
)
from .coordinator import OpenNeatoCoordinator
from .http import OpenNeatoMapView
from .lidar_runner import LidarMapRunner

_LOGGER = logging.getLogger(__name__)

# The replay card ships inside the integration rather than as a separate
# HACS resource, so the card and the WebSocket API it talks to can never
# fall out of version sync.
REPLAY_CARD_FILENAME = "openneato-replay-card.js"
REPLAY_CARD_URL = f"/openneato_static/{REPLAY_CARD_FILENAME}"
_FRONTEND_REGISTERED = "frontend_registered"

PLATFORMS: list[Platform] = [
    Platform.VACUUM,
    Platform.SENSOR,
    Platform.BINARY_SENSOR,
    Platform.SWITCH,
    Platform.NUMBER,
    Platform.BUTTON,
    # Platform.CAMERA is deliberately absent. The "LIDAR map" and "Cleaning
    # replay" cameras were server-rendered PNG/GIF snapshots; the canvas card
    # in www/openneato-replay-card.js supersedes both, drawing from the
    # openneato/session(s) websocket API with pan, zoom and scrubbing.
    #
    # camera.py, lidar_renderer.py and history_renderer.py were deleted with
    # it (1347 lines). The one piece still needed, _try_repair_pose, now lives
    # in replay.py.
    Platform.SELECT,
    Platform.TEXT,
    Platform.TIME,
]


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Set up OpenNeato from a config entry."""
    host = entry.data[CONF_HOST]
    session = async_get_clientsession(hass)
    api = OpenNeatoApiClient(host, session)

    _LOGGER.debug("Connecting to OpenNeato at %s", host)
    try:
        firmware_info = await api.get_firmware_version()
        robot_info = await api.get_robot_version()
    except OpenNeatoConnectionError as err:
        _LOGGER.warning("Cannot connect to OpenNeato at %s: %s", host, err)
        raise ConfigEntryNotReady(
            f"Cannot connect to OpenNeato at {host}: {err}"
        ) from err
    except Exception as err:
        _LOGGER.exception("Unexpected error connecting to OpenNeato at %s", host)
        raise ConfigEntryNotReady(
            f"Unexpected error connecting to OpenNeato at {host}: {err}"
        ) from err

    _LOGGER.info(
        "Connected to OpenNeato at %s — %s (%s) firmware %s",
        host,
        robot_info.get("modelName"),
        robot_info.get("serialNumber"),
        firmware_info.get("version"),
    )

    serial = robot_info.get("serialNumber", entry.data.get("serial", "unknown"))
    model = robot_info.get("modelName", entry.data.get("model"))
    sw_version = robot_info.get("softwareVersion", entry.data.get("software_version"))
    fw_version = firmware_info.get("version", entry.data.get("firmware_version"))

    coordinator = OpenNeatoCoordinator(hass, api)
    await coordinator.async_config_entry_first_refresh()

    hass.data.setdefault(DOMAIN, {})
    await _async_register_frontend(hass)

    # Builds a floor plan from the robot's own LIDAR, one cleaning at a time.
    # The accumulated map lives in its own store so it survives restarts and
    # keeps improving instead of being rebuilt from a single run.
    mapper = None
    if entry.options.get(CONF_MAP_ENABLED, MAP_DEFAULT_ENABLED):
        mapper = LidarMapRunner(hass, entry.entry_id, api, coordinator)
        await mapper.async_load()

    hass.data[DOMAIN][entry.entry_id] = {
        "api": api,
        "coordinator": coordinator,
        "mapper": mapper,
        "serial": serial,
        "model": model,
        "sw_version": sw_version,
        "fw_version": fw_version,
        "host": host,
    }

    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)

    # Reload the entry when its options change (e.g. floorplan background
    # calibration) so camera entities pick up the new config without a
    # full HA restart.
    entry.async_on_unload(entry.add_update_listener(async_reload_options))

    return True


async def _async_register_frontend(hass: HomeAssistant) -> None:
    """Serve the replay card and register the API it calls.

    Runs once per HA start, not once per config entry — the static path, the
    WebSocket commands and the frontend script tag are all global.
    """
    if hass.data[DOMAIN].get(_FRONTEND_REGISTERED):
        return
    hass.data[DOMAIN][_FRONTEND_REGISTERED] = True

    websocket.async_register(hass)
    hass.http.register_view(OpenNeatoMapView())

    card_path = Path(__file__).parent / "www" / REPLAY_CARD_FILENAME
    if not await hass.async_add_executor_job(card_path.is_file):
        _LOGGER.warning("Replay card asset missing at %s; card will not load", card_path)
        return

    # Serving the card is a convenience, not a prerequisite for the robot to
    # work — never let a frontend API change take the vacuum down with it.
    try:
        await hass.http.async_register_static_paths(
            [StaticPathConfig(REPLAY_CARD_URL, str(card_path), True)]
        )
        # Deliberately no add_extra_js_url here.
        #
        # The card also has to be declared as a Lovelace resource, because
        # add_extra_js_url alone loses the race against the dashboard's first
        # render. Having *both* is worse than either: the two loaders race, and
        # on the loads where neither import wins before the view is built the
        # file is fetched but never evaluated, so the element is unregistered
        # and the card renders as "Custom element doesn't exist" -- about one
        # reload in three, and more often when reloading quickly.
        #
        # One loader, and the reliable one: the Lovelace resource is awaited
        # before the view builds, which is why every HACS card uses it. The
        # resource URL carries ?v=<manifest version>, so bump the manifest when
        # the card changes or browsers will serve a stale copy.
        integration = await async_get_integration(hass, DOMAIN)
        _LOGGER.debug(
            "Replay card %s served at %s; register it as a Lovelace resource "
            "with the matching ?v= query",
            integration.version or "0",
            REPLAY_CARD_URL,
        )
    except Exception:  # noqa: BLE001
        _LOGGER.exception(
            "Could not auto-register the replay card. Add %s as a Lovelace "
            "resource manually if the card does not appear",
            REPLAY_CARD_URL,
        )
        return
    _LOGGER.debug("Registered replay card at %s", REPLAY_CARD_URL)


async def async_reload_options(hass: HomeAssistant, entry: ConfigEntry) -> None:
    """Reload the config entry to apply updated options."""
    await hass.config_entries.async_reload(entry.entry_id)


async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Unload an OpenNeato config entry."""
    if unload_ok := await hass.config_entries.async_unload_platforms(entry, PLATFORMS):
        stored = hass.data[DOMAIN].pop(entry.entry_id)
        if stored.get("mapper"):
            stored["mapper"].async_unload()
        # Drop this entry's parsed replay sessions so a reload re-fetches
        # instead of serving a stale map from before a recalibration.
        cache = hass.data[DOMAIN].get(websocket.CACHE_KEY)
        if isinstance(cache, dict):
            for key in [k for k in cache if k[0] == entry.entry_id]:
                cache.pop(key, None)
    return unload_ok
