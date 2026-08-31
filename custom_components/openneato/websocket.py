"""WebSocket API backing the interactive replay card.

Two commands: one to list cleaning sessions, one to fetch a parsed session.
The card renders the returned data on a canvas at display refresh rate, so
all the CPU work (downloading, decompressing, building the coverage grid)
stays here on the server and happens exactly once per session.
"""

from __future__ import annotations

import logging
from typing import Any

import voluptuous as vol

from homeassistant.components import websocket_api
from homeassistant.core import HomeAssistant, callback

from .api import OpenNeatoApiError
from .const import (
    CELL_SIZE_M,
    CONF_MAP_ROTATION_OFFSET,
    DOMAIN,
    MAP_DEFAULT_ROTATION_OFFSET,

)
from .lidar_mapper import alignment_key
from .replay import build_replay_session

_LOGGER = logging.getLogger(__name__)

# Parsed sessions are a few hundred KB each; keeping a handful means flipping
# back and forth in the session picker is instant instead of re-downloading
# from the robot every time.
CACHE_KEY = "replay_cache"
# Last session list seen from the robot, so the card still has something
# to show when the bridge is unreachable. A replay is history: it does
# not need the robot awake.
LAST_SESSIONS_KEY = "replay_last_sessions"
_CACHE_MAX = 4


@callback
def async_register(hass: HomeAssistant) -> None:
    """Register the replay WebSocket commands (idempotent)."""
    websocket_api.async_register_command(hass, ws_list_sessions)
    websocket_api.async_register_command(hass, ws_get_session)
    websocket_api.async_register_command(hass, ws_delete_session)


def _resolve_entry(hass: HomeAssistant, entry_id: str | None) -> tuple[str, dict[str, Any]] | None:
    """Return (entry_id, entry_data) for the requested or only OpenNeato entry."""
    entries: dict[str, Any] = hass.data.get(DOMAIN, {})
    # hass.data[DOMAIN] also holds our own module-level scratch keys.
    candidates = {
        key: value
        for key, value in entries.items()
        if isinstance(value, dict) and "api" in value
    }
    if entry_id:
        data = candidates.get(entry_id)
        return (entry_id, data) if data else None
    if len(candidates) == 1:
        return next(iter(candidates.items()))
    return None


def _floorplan_payload(hass: HomeAssistant, entry_id: str) -> dict[str, Any] | None:
    """Expose the background so the card draws the same plan the cameras do.

    A LIDAR-built map wins over a hand-calibrated image: it is drawn in the
    robot's own frame, so its origin and scale are exact rather than fitted,
    and it carries a view rotation derived from the walls, which stands it
    upright however the robot's frame happens to be oriented.

    Either way the stored image is a server-side path the browser can't load,
    so the card fetches it through one of our HTTP views.
    """
    entry = hass.config_entries.async_get_entry(entry_id)
    if entry is None:
        return None

    stored = hass.data.get(DOMAIN, {}).get(entry_id)
    mapper = stored.get("mapper") if isinstance(stored, dict) else None
    if mapper is not None and mapper.sessions:
        cal = mapper.calibration()
        if cal:
            offset = float(
                entry.options.get(CONF_MAP_ROTATION_OFFSET, MAP_DEFAULT_ROTATION_OFFSET)
            )
            return {
                # Busts the browser cache whenever the drawn plan changes --
                # not just when a cleaning is added, since the wall threshold
                # now moves with the map's own distribution.
                "url": f"/api/openneato/map/{entry_id}?v={mapper.render_signature()}",
                "originX": cal["origin_x"],
                "originY": cal["origin_y"],
                "rotation": 0.0,
                "scale": cal["scale"],
                # The grid this plan was drawn on, stated rather than inferred.
                # The card used to read the plan back on the *session's* cell
                # size, which is a different number that merely happened to
                # agree: when the map halved to 2.5 cm and the replay had not,
                # the card sampled every other cell of the plan and drew every
                # wall perforated. Two things that need not agree should not be
                # made to agree by hand.
                "cellSize": CELL_SIZE_M,
                "viewRotation": mapper.view_rotation(offset),
                "generated": True,
                "sessions": mapper.sessions,
            }

    # No hand-calibrated fallback any more: the plan the robot builds for
    # itself is the only one. A static image had to be aligned by hand and
    # went stale the moment the furniture moved.
    return None


@websocket_api.websocket_command(
    {
        vol.Required("type"): "openneato/sessions",
        vol.Optional("entry_id"): str,
    }
)
@websocket_api.async_response
async def ws_list_sessions(
    hass: HomeAssistant,
    connection: websocket_api.ActiveConnection,
    msg: dict[str, Any],
) -> None:
    """List the robot's cleaning sessions, newest first."""
    resolved = _resolve_entry(hass, msg.get("entry_id"))
    if resolved is None:
        connection.send_error(msg["id"], "not_found", "No OpenNeato config entry found")
        return
    entry_id, data = resolved

    coordinator = data["coordinator"]
    history = (coordinator.data or {}).get("history")
    store = hass.data.setdefault(DOMAIN, {}).setdefault(LAST_SESSIONS_KEY, {})

    if isinstance(history, list):
        sessions = [
            {
                "name": item.get("name"),
                "size": item.get("size"),
                "recording": bool(item.get("recording")),
                "session": item.get("session"),
                "summary": item.get("summary"),
            }
            for item in history
            if isinstance(item, dict) and item.get("name")
        ]
        # The firmware returns files in directory order; sort by session start
        # so the picker reads chronologically regardless of filesystem layout.
        sessions.sort(key=lambda s: _session_start(s), reverse=True)
        store[entry_id] = sessions
    else:
        # The robot is unreachable. A replay is history, though -- the parsed
        # sessions are cached here and the floor plan lives in Home Assistant's
        # own storage -- so serve the last list we saw rather than blanking the
        # card. Nothing here needs the robot to be awake.
        sessions = store.get(entry_id)
        if not sessions:
            connection.send_error(
                msg["id"], "unavailable", "No cleaning history available"
            )
            return
        _LOGGER.debug("Replay: robot unreachable, serving %d cached sessions", len(sessions))
        # Whatever was recording is not any more, as far as we can tell.
        sessions = [{**s, "recording": False} for s in sessions]

    connection.send_result(
        msg["id"],
        {
            "entry_id": entry_id,
            "sessions": sessions,
            # Whether any of this came from the robot just now. It has to be
            # said, because the fallback above cannot honestly claim anything
            # is still being recorded -- so a bridge that has gone quiet is
            # reported in exactly the same words as a clean that has just
            # ended, and the card treats those two very differently. The robot
            # here rides out of radio range in one corner of the house on
            # nearly every run, so this is the common case, not the exotic one.
            "robot_available": isinstance(history, list),
            "floorplan": _floorplan_payload(hass, entry_id),
        },
    )


def _session_start(session: dict[str, Any]) -> float:
    """Session start epoch, falling back to the numeric filename prefix."""
    info = session.get("session")
    if isinstance(info, dict) and info.get("time"):
        return float(info["time"])
    try:
        return float(str(session.get("name", "")).split(".", 1)[0])
    except ValueError:
        return 0.0


@websocket_api.websocket_command(
    {
        vol.Required("type"): "openneato/session",
        vol.Required("name"): str,
        vol.Optional("entry_id"): str,
    }
)
@websocket_api.async_response
async def ws_get_session(
    hass: HomeAssistant,
    connection: websocket_api.ActiveConnection,
    msg: dict[str, Any],
) -> None:
    """Download and parse one cleaning session for playback."""
    resolved = _resolve_entry(hass, msg.get("entry_id"))
    if resolved is None:
        connection.send_error(msg["id"], "not_found", "No OpenNeato config entry found")
        return
    entry_id, data = resolved
    name = msg["name"]

    # Serve the run in the map's frame, not the robot's frame of the day.
    # Read before the cache, because it is part of what identifies a parse.
    runner = data.get("mapper")
    align = runner.alignment(name) if runner is not None else None

    # Key the cache on the name the compression cannot change, so a run
    # fetched while it was recording is still a hit once the firmware has
    # renamed it -- and so the card cannot make us download it twice.
    #
    # And on the alignment, because it is an input to the parse and it
    # *appears late*. A run stops recording a minute or so before the map
    # merges it, and a card polling through that window gets a parse made
    # with no alignment at all. Keyed on the name alone that parse is served
    # for good, and the cleaned area sits a hand's width off the walls
    # forever after -- 4.0% of its cells on top of a wall against 1.7% once
    # aligned, measured on the 2026-08-26 run. Until the stable-name key
    # existed the rename to `.hs` happened to flush it; nothing does now, so
    # say what the parse depended on instead of relying on an accident.
    cache: dict[tuple[str, str, Any], dict[str, Any]] = hass.data.setdefault(
        DOMAIN, {}
    ).setdefault(CACHE_KEY, {})
    cache_key = (entry_id, alignment_key(name), align)
    cached = cache.get(cache_key)
    if cached is not None:
        connection.send_result(msg["id"], {**cached, "floorplan": _floorplan_payload(hass, entry_id)})
        return

    resolved = _current_name(data["coordinator"], name)
    if resolved is None:
        # Not on the robot and not in the cache. Deleted, or compressed away
        # while the card was between listings -- neither is a fault, and the
        # card recovers by relisting, so this must not read as a failure.
        _LOGGER.debug("Replay: session %s is no longer on the robot", name)
        connection.send_error(
            msg["id"], "session_gone", f"Session {name} is no longer on the robot"
        )
        return

    try:
        raw = await data["api"].get_history_session(resolved)
    except OpenNeatoApiError as err:
        if err.status == 404:
            # Same situation as the branch above, reached a listing later: the
            # firmware renames a finished run to `.hs` when it compresses it,
            # about a minute after the robot docks, and until the card relists
            # it is asking for a name that no longer exists. The robot answered
            # perfectly well -- it said "not here" -- so this must not reach the
            # card as a fetch failure, which is what made it announce "robot not
            # answering" for the whole minute after every cleaning, while the
            # robot sat on its dock and the map was being rebuilt.
            _LOGGER.debug(
                "Replay: session %s has been renamed under us; the card will "
                "relist", resolved,
            )
            connection.send_error(
                msg["id"], "session_gone", f"Session {resolved} is no longer on the robot"
            )
            return
        _LOGGER.warning("Replay: failed to fetch session %s: %s", resolved, err)
        connection.send_error(msg["id"], "fetch_failed", str(err))
        return
    except Exception as err:  # noqa: BLE001 -- surface any fetch failure to the card
        _LOGGER.warning("Replay: failed to fetch session %s: %s", resolved, err)
        connection.send_error(msg["id"], "fetch_failed", str(err))
        return

    # Coverage-grid construction is CPU-bound; keep it off the event loop.
    try:
        parsed = await hass.async_add_executor_job(
            build_replay_session, raw, resolved, align
        )
    except Exception as err:  # noqa: BLE001
        _LOGGER.exception("Replay: failed to parse session %s", resolved)
        connection.send_error(msg["id"], "parse_failed", str(err))
        return

    if not parsed.get("path"):
        connection.send_error(msg["id"], "empty_session", "Session contains no pose data")
        return

    # Only completed sessions are worth caching -- a recording one grows.
    if not _is_recording(data["coordinator"], resolved):
        if len(cache) >= _CACHE_MAX:
            cache.pop(next(iter(cache)))
        cache[cache_key] = parsed

    connection.send_result(msg["id"], {**parsed, "floorplan": _floorplan_payload(hass, entry_id)})


@websocket_api.websocket_command(
    {
        vol.Required("type"): "openneato/delete_session",
        vol.Required("name"): str,
        vol.Optional("entry_id"): str,
    }
)
@websocket_api.async_response
async def ws_delete_session(
    hass: HomeAssistant,
    connection: websocket_api.ActiveConnection,
    msg: dict[str, Any],
) -> None:
    """Delete one recorded session from the robot.

    For a run that went wrong -- the robot was picked up, the LIDAR was
    blocked, the map came out half-drawn -- so the bad replay stops cluttering
    the picker.

    Note this does NOT unpick the session's contribution to the accumulated
    wall map: `LidarMap` merges hit counts, and once merged a session's cells
    are indistinguishable from every other session's. Deleting a bad session
    stops it being replayed; it does not un-draw its walls.
    """
    resolved = _resolve_entry(hass, msg.get("entry_id"))
    if resolved is None:
        connection.send_error(msg["id"], "not_found", "No OpenNeato config entry found")
        return
    entry_id, data = resolved
    name = _current_name(data["coordinator"], msg["name"])
    if name is None:
        # Already gone. The end state the caller wanted, so refresh and agree
        # rather than reporting a failure to delete what is not there.
        await data["coordinator"].async_request_refresh()
        connection.send_result(msg["id"], {"deleted": msg["name"]})
        return

    # Refuse while the robot is still writing to it: the firmware would be
    # appending to a file we just unlinked.
    if _is_recording(data["coordinator"], name):
        connection.send_error(
            msg["id"], "recording", "That session is still being recorded"
        )
        return

    try:
        await data["api"].delete_history_session(name)
    except Exception as err:  # noqa: BLE001
        _LOGGER.warning("Replay: failed to delete session %s: %s", name, err)
        connection.send_error(msg["id"], "delete_failed", str(err))
        return

    # Drop it from the parsed-session cache so a later request cannot serve
    # a session the robot no longer has.
    # Every parse of it, under whichever alignment it was made with.
    cache = hass.data.setdefault(DOMAIN, {}).setdefault(CACHE_KEY, {})
    key = alignment_key(name)
    for stale in [k for k in cache if k[0] == entry_id and k[1] == key]:
        cache.pop(stale, None)

    # Refresh so the picker's next listing no longer offers it.
    await data["coordinator"].async_request_refresh()

    _LOGGER.info("Replay: deleted session %s", name)
    connection.send_result(msg["id"], {"deleted": name})


def _current_name(coordinator: Any, name: str) -> str | None:
    """What the robot calls this session right now, or None if it has it no more.

    A session is `<epoch>.jsonl` while the robot writes it and becomes
    `<epoch>.jsonl.hs` once the firmware compresses it, minutes after the run
    ends. The card holds whichever name the listing gave it, so every request
    that straddles that rename asks for a file the robot no longer has: the
    fetch 404s, the card blanks the run it had just watched being drawn, and
    the log fills with a failure that is really just a rename.

    Match on the part compression cannot change -- the same key the session
    alignments are stored under.
    """
    history = (coordinator.data or {}).get("history")
    if not isinstance(history, list):
        # No listing to resolve against; the robot is unreachable. Let the
        # fetch itself fail rather than declaring a session gone on no evidence.
        return name
    names = [
        str(item["name"])
        for item in history
        if isinstance(item, dict) and item.get("name")
    ]
    if name in names:
        return name
    key = alignment_key(name)
    return next((n for n in names if alignment_key(n) == key), None)


def _is_recording(coordinator: Any, name: str) -> bool:
    """True while the robot is still appending to this session file."""
    history = (coordinator.data or {}).get("history")
    if not isinstance(history, list):
        return False
    return any(
        isinstance(item, dict) and item.get("name") == name and item.get("recording")
        for item in history
    )
