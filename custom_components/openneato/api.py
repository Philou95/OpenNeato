"""Async HTTP client for the OpenNeato robot API."""

from __future__ import annotations

import json
import logging
import re
import asyncio
from asyncio import Task, ensure_future
from typing import Any

import aiohttp
from asyncio import timeout

from homeassistant.exceptions import HomeAssistantError

from .const import MAX_HISTORY_RESPONSE_BYTES, SESSION_NAME_PATTERN

_LOGGER = logging.getLogger(__name__)

TIMEOUT = 30  # seconds — ESP32 can be slow when serial queue is busy
# Tries at downloading one session before giving up, and the wait between them.
# The bridge writes a pose to SPIFFS every 2 s while cleaning and that is what
# cuts the download short, so the gap is set to land the retry in a different
# phase of that cadence rather than straight back into the same collision.
HISTORY_ATTEMPTS = 3
HISTORY_RETRY_S = 1.5

_SESSION_NAME_RE = re.compile(SESSION_NAME_PATTERN)


class OpenNeatoConnectionError(HomeAssistantError):
    """Error to indicate we cannot connect to the robot."""


class OpenNeatoApiError(HomeAssistantError):
    """Error to indicate a non-connection API failure.

    Carries the HTTP status when there was one, because the callers need to
    tell apart "the robot answered, and the answer was no" from "the robot did
    not answer". A 404 on a session is the first: the file has been renamed
    under us -- the firmware compresses a finished run to `.hs` a minute after
    it ends -- and the cure is to relist, not to tell the user the robot is
    unreachable while it sits on its dock. `None` when the failure had no
    status of its own.
    """

    def __init__(self, message: str, status: int | None = None) -> None:
        super().__init__(message)
        self.status = status


async def _read_json(response: aiohttp.ClientResponse) -> Any:
    """Read a response body and parse it as JSON, tolerating stray non-UTF-8 bytes.

    The firmware's jsonEscape() passes bytes >= 0x20 through verbatim, so the
    /api/version response can contain raw bytes from the robot's smart-battery
    memory (manufacturer name, serial, etc.) that aren't valid UTF-8. Replace
    those bytes rather than raising — losing one glyph is better than failing
    the whole config-entry setup.
    """
    raw = await response.read()
    return json.loads(raw.decode("utf-8", errors="replace"))


class OpenNeatoApiClient:
    """Async HTTP client for OpenNeato."""

    def __init__(self, host: str, session: aiohttp.ClientSession) -> None:
        """Initialize the API client."""
        self._host = host.rstrip("/")
        self._session = session
        self._base_url = f"http://{self._host}"
        # Coalesces concurrent get_history_session() calls for the same
        # filename into a single in-flight request. Both camera entities
        # (LIDAR map + Cleaning replay) independently fetch the same
        # completed session right after the first coordinator refresh, and
        # the ESP32 bridge — which reads history off a blocking serial link
        # to the robot — can stall indefinitely when hit with two
        # overlapping requests for the same file rather than erroring.
        # Entries are removed once the fetch completes, so this only
        # de-duplicates true concurrency, not stale caching (important for
        # in-progress "recording" sessions whose data keeps growing).
        self._history_inflight: dict[str, Task] = {}

        # What we already hold of the session we are following, and its name.
        # A run in progress is fetched again every few seconds for as long as
        # somebody has the map on screen -- around 1200 times over an hour --
        # and downloading the whole file each time is the read pressure under
        # which the bridge loses bytes out of the very file it is writing.
        # Held for one session only: the buffer is replaced, not added to,
        # the moment a different name is asked for.
        self._history_name: str | None = None
        self._history_held = b""

    @property
    def base_url(self) -> str:
        """Return the base URL."""
        return self._base_url

    @property
    def session(self) -> aiohttp.ClientSession:
        """Return the aiohttp session."""
        return self._session

    async def _get(self, path: str) -> dict[str, Any]:
        """Perform a GET request and return parsed JSON."""
        url = f"{self._base_url}{path}"
        _LOGGER.debug("GET %s", url)
        try:
            async with timeout(TIMEOUT):
                async with self._session.get(url) as response:
                    _LOGGER.debug(
                        "GET %s -> %s (%s)",
                        path, response.status, response.content_type,
                    )
                    response.raise_for_status()
                    return await _read_json(response)
        except aiohttp.ClientConnectionError as err:
            _LOGGER.warning("Connection error on GET %s: %s", path, err)
            raise OpenNeatoConnectionError(
                f"Unable to connect to OpenNeato at {self._host}: {err}"
            ) from err
        except aiohttp.ClientResponseError as err:
            _LOGGER.warning("HTTP %s on GET %s: %s", err.status, path, err.message)
            raise OpenNeatoApiError(
                f"API error from {path}: {err.status} {err.message}", err.status
            ) from err
        except TimeoutError as err:
            _LOGGER.warning("Timeout on GET %s (limit %ss)", path, TIMEOUT)
            raise OpenNeatoConnectionError(
                f"Timeout connecting to OpenNeato at {self._host}"
            ) from err

    async def _post(
        self, path: str, params: dict[str, str] | None = None
    ) -> dict[str, Any] | str:
        """Perform a POST request with optional query params."""
        url = f"{self._base_url}{path}"
        _LOGGER.debug("POST %s params=%s", url, params)
        try:
            async with timeout(TIMEOUT):
                async with self._session.post(url, params=params) as response:
                    _LOGGER.debug(
                        "POST %s -> %s (%s)",
                        path, response.status, response.content_type,
                    )
                    response.raise_for_status()
                    content_type = response.content_type or ""
                    if "json" in content_type:
                        return await _read_json(response)
                    return await response.text()
        except aiohttp.ClientConnectionError as err:
            _LOGGER.warning("Connection error on POST %s: %s", path, err)
            raise OpenNeatoConnectionError(
                f"Unable to connect to OpenNeato at {self._host}: {err}"
            ) from err
        except aiohttp.ClientResponseError as err:
            _LOGGER.warning("HTTP %s on POST %s: %s", err.status, path, err.message)
            raise OpenNeatoApiError(
                f"API error from POST {path}: {err.status} {err.message}", err.status
            ) from err
        except TimeoutError as err:
            _LOGGER.warning("Timeout on POST %s (limit %ss)", path, TIMEOUT)
            raise OpenNeatoConnectionError(
                f"Timeout connecting to OpenNeato at {self._host}"
            ) from err

    async def delete_history_session(self, filename: str) -> None:
        """Delete one recorded cleaning session from the bridge.

        The firmware exposes DELETE /api/history/<name>. `filename` comes
        from the /api/history listing and is concatenated into the URL, so
        it is validated against the same strict pattern the download path
        uses — a rogue or MITM'd peer could otherwise aim the delete at an
        unrelated endpoint.
        """
        if not _SESSION_NAME_RE.match(filename):
            raise OpenNeatoApiError(f"Refusing to delete unsafe filename: {filename!r}")

        url = f"{self._base_url}/api/history/{filename}"
        _LOGGER.debug("DELETE %s", url)
        try:
            async with timeout(TIMEOUT):
                async with self._session.delete(url) as response:
                    _LOGGER.debug("DELETE %s -> %s", url, response.status)
                    response.raise_for_status()
        except aiohttp.ClientConnectionError as err:
            _LOGGER.warning("Connection error deleting %s: %s", filename, err)
            raise OpenNeatoConnectionError(
                f"Unable to connect to OpenNeato at {self._host}: {err}"
            ) from err
        except aiohttp.ClientResponseError as err:
            _LOGGER.warning("HTTP %s deleting %s: %s", err.status, filename, err.message)
            raise OpenNeatoApiError(
                f"API error deleting /api/history/{filename}: "
                f"{err.status} {err.message}",
                err.status,
            ) from err
        except TimeoutError as err:
            _LOGGER.warning("Timeout deleting %s (limit %ss)", filename, TIMEOUT)
            raise OpenNeatoConnectionError(
                f"Timeout connecting to OpenNeato at {self._host}"
            ) from err

    async def _put(self, path: str, json_data: dict[str, Any]) -> dict[str, Any]:
        """Perform a PUT request with a JSON body."""
        url = f"{self._base_url}{path}"
        _LOGGER.debug("PUT %s body=%s", url, json_data)
        try:
            async with timeout(TIMEOUT):
                async with self._session.put(url, json=json_data) as response:
                    _LOGGER.debug(
                        "PUT %s -> %s (%s)",
                        path, response.status, response.content_type,
                    )
                    response.raise_for_status()
                    return await _read_json(response)
        except aiohttp.ClientConnectionError as err:
            _LOGGER.warning("Connection error on PUT %s: %s", path, err)
            raise OpenNeatoConnectionError(
                f"Unable to connect to OpenNeato at {self._host}: {err}"
            ) from err
        except aiohttp.ClientResponseError as err:
            _LOGGER.warning("HTTP %s on PUT %s: %s", err.status, path, err.message)
            raise OpenNeatoApiError(
                f"API error from PUT {path}: {err.status} {err.message}", err.status
            ) from err
        except TimeoutError as err:
            _LOGGER.warning("Timeout on PUT %s (limit %ss)", path, TIMEOUT)
            raise OpenNeatoConnectionError(
                f"Timeout connecting to OpenNeato at {self._host}"
            ) from err

    # ── GET endpoints ────────────────────────────────────────────────

    async def get_state(self) -> dict[str, Any]:
        """Get the robot's current state."""
        return await self._get("/api/state")

    async def get_charger(self) -> dict[str, Any]:
        """Get charger / battery information."""
        return await self._get("/api/charger")

    async def get_battery_analog(self) -> dict[str, Any]:
        """Get analog battery readings (voltage, current, temperature)."""
        return await self._get("/api/analog")

    async def get_battery_warranty(self) -> dict[str, Any]:
        """Get battery warranty data (cumulative cycles, runtime)."""
        return await self._get("/api/warranty")

    async def get_error(self) -> dict[str, Any]:
        """Get current error information."""
        return await self._get("/api/error")

    async def get_firmware_version(self) -> dict[str, Any]:
        """Get firmware version info (chip, model, etc.)."""
        return await self._get("/api/firmware/version")

    async def get_robot_version(self) -> dict[str, Any]:
        """Get robot version info (serial, model name, etc.)."""
        return await self._get("/api/version")

    async def get_motors(self) -> dict[str, Any]:
        """Get motor RPM and current readings."""
        return await self._get("/api/motors")

    async def get_system(self) -> dict[str, Any]:
        """Get system information (heap, uptime, RSSI, etc.)."""
        return await self._get("/api/system")

    async def get_user_settings(self) -> dict[str, Any]:
        """Get user-facing settings (eco mode, intense clean, etc.)."""
        return await self._get("/api/user-settings")

    async def get_sensors(self) -> dict[str, Any]:
        """Get digital sensor data (dustbin, bumpers, wheel lift)."""
        return await self._get("/api/sensors")

    async def get_settings(self) -> dict[str, Any]:
        """Get full device settings."""
        return await self._get("/api/settings")

    async def get_history(self) -> list[dict[str, Any]]:
        """Get cleaning history sessions."""
        return await self._get("/api/history")  # type: ignore[return-value]

    async def get_lidar_status(self) -> dict:
        """The bridge's scan counters, and the frame this run is recorded in.

        Cheap on purpose: the handler builds it from variables in RAM, touching
        neither the serial link nor the filesystem. Measured 2026-09-05 over a
        full cleaning: 0.1 ms served, against 332 ms for a live scan.
        """
        return await self._get("/api/lidar/status")

    async def get_lidar_buffer(self, after: int) -> str:
        """Collect the scans the bridge buffered while cleaning.

        Returns NDJSON, one scan per line, or an empty string when there is
        nothing new. `after` is the highest sequence number already held; the
        bridge drops everything up to it, which is what clears the buffer.

        Raises OpenNeatoApiError with status 404 on a bridge too old to have
        the endpoint, which is the caller's signal to sample the old way.
        """
        return await self._get_text(f"/api/lidar/buffer?after={int(after)}")

    async def get_lidar(self) -> dict[str, Any]:
        """Get the latest LDS LIDAR scan (360 points)."""
        return await self._get("/api/lidar")

    async def _get_text(self, path: str) -> str:
        """GET returning raw text, drained to EOF like a session download."""
        url = f"{self._base_url}{path}"
        try:
            async with timeout(TIMEOUT):
                async with self._session.get(url) as response:
                    response.raise_for_status()
                    buf = bytearray()
                    async for chunk in response.content.iter_chunked(65536):
                        buf.extend(chunk)
                        if len(buf) > MAX_HISTORY_RESPONSE_BYTES:
                            raise OpenNeatoApiError(f"{path} exceeds the size cap")
                    return bytes(buf).decode("utf-8", errors="replace")
        except aiohttp.ClientConnectionError as err:
            raise OpenNeatoConnectionError(
                f"Unable to connect to OpenNeato at {self._host}: {err}"
            ) from err
        except aiohttp.ClientResponseError as err:
            raise OpenNeatoApiError(
                f"API error from {path}: {err.status} {err.message}", err.status
            ) from err
        except TimeoutError as err:
            raise OpenNeatoConnectionError(
                f"Timeout connecting to OpenNeato at {self._host}"
            ) from err

    async def get_history_session(self, filename: str) -> str:
        """Download the raw JSONL data for a specific cleaning session.

        Coalesces concurrent requests for the same filename into a single
        in-flight fetch (see `_history_inflight` in __init__) — both camera
        entities can request the same session within milliseconds of each
        other at startup, and the bridge can't reliably serve two
        overlapping requests for the same file.
        """
        task = self._history_inflight.get(filename)
        if task is not None:
            _LOGGER.debug(
                "History fetch for %s already in flight, awaiting it", filename
            )
            return await task

        task = ensure_future(self._fetch_history_session(filename))
        self._history_inflight[filename] = task
        try:
            return await task
        finally:
            self._history_inflight.pop(filename, None)

    async def _fetch_history_session(self, filename: str) -> str:
        """Fetch a session's raw JSONL, retrying a body the bridge cut short.

        The download races the robot's own pose journal. Serving this endpoint
        means reading SPIFFS from the AsyncTCP task while `CleaningHistory`
        writes to it from the loop task every 2 s, with no mutex between them;
        when they collide ESPAsyncWebServer abandons the body mid-send and
        aiohttp raises TransferEncodingError. It only happens while a cleaning
        is running -- at rest the endpoint is solid -- which is exactly when
        somebody is watching the map, so it reached the card as a raw
        "Response payload is not completed" where a map should have been.

        A GET is idempotent and the collision is a matter of timing, so asking
        again a moment later is both safe and usually enough. Deliberately not
        done for the polled endpoints: retrying those would add requests to a
        bridge that is already struggling, which is the wrong direction.
        """
        # Ask only for what has been appended since we last looked. The
        # firmware says where it actually starts serving; anything other than
        # the offset we asked for means it could not honour it, and the body
        # is then the whole file.
        since = len(self._history_held) if self._history_name == filename else 0

        last: Exception | None = None
        for attempt in range(1, HISTORY_ATTEMPTS + 1):
            try:
                served_from, body = await self._fetch_history_once(filename, since)
                return self._join_history(filename, since, served_from, body)
            except aiohttp.ClientPayloadError as err:
                last = err
                if attempt < HISTORY_ATTEMPTS:
                    _LOGGER.debug(
                        "History fetch for %s came back truncated (%s), "
                        "attempt %d of %d",
                        filename, err, attempt, HISTORY_ATTEMPTS,
                    )
                    await asyncio.sleep(HISTORY_RETRY_S)
        _LOGGER.warning(
            "History fetch for %s was cut short %d times: %s",
            filename, HISTORY_ATTEMPTS, last,
        )
        raise OpenNeatoConnectionError(
            f"OpenNeato at {self._host} cut the session download short: {last}"
        ) from last

    def _join_history(
        self, filename: str, since: int, served_from: int, body: bytes
    ) -> str:
        """Join a tail onto what we hold, and keep the result for next time.

        Only whole lines are kept. A body that stops mid-line -- the robot was
        part-way through appending a snapshot when we read -- has to be asked
        for again from the start of that line: committed as it stands, the
        broken line would stay broken for the rest of the run, since every
        later fetch starts after it.
        """
        if since and served_from == since:
            combined = self._history_held + body
        else:
            # An offset the firmware refused, or a firmware too old to know
            # about `since` at all. Either way the body stands alone.
            combined = body

        if len(combined) > MAX_HISTORY_RESPONSE_BYTES:
            self._history_name, self._history_held = None, b""
            raise OpenNeatoApiError(
                f"Session {filename} exceeds size cap "
                f"({MAX_HISTORY_RESPONSE_BYTES} bytes)"
            )

        self._history_name = filename
        self._history_held = combined[: combined.rfind(b"\n") + 1]
        return combined.decode("utf-8", errors="replace")

    async def _fetch_history_once(self, filename: str, since: int = 0) -> tuple[int, bytes]:
        """One attempt at the download.

        `filename` originates from the ESP32's /api/history listing and
        is concatenated into the URL, so we validate it against a strict
        pattern first — a rogue or MITM'd peer could otherwise redirect
        the request to an unrelated endpoint. The response is capped to
        MAX_HISTORY_RESPONSE_BYTES to stop a misbehaving peer from
        OOM'ing HA Core with an unbounded stream.
        """
        if not _SESSION_NAME_RE.match(filename):
            raise OpenNeatoApiError(
                f"Invalid session filename: {filename!r}"
            )
        url = f"{self._base_url}/api/history/{filename}"
        params = {"since": str(since)} if since else None
        _LOGGER.debug("GET %s since=%d", url, since)
        try:
            async with timeout(TIMEOUT):
                async with self._session.get(url, params=params) as response:
                    response.raise_for_status()
                    # Where the body actually starts. Absent on a firmware
                    # that predates the header, which serves whole files --
                    # 0 is the right reading of that.
                    try:
                        served_from = int(response.headers.get("X-Since", 0))
                    except ValueError:
                        served_from = 0
                    # The firmware serves this endpoint as an HTTP chunked
                    # transfer with no Content-Length (beginChunkedResponse
                    # in web_server.cpp). On a chunked aiohttp response,
                    # content.read(n) returns as soon as ANY buffered data is
                    # available — it does NOT block until n bytes or EOF — so
                    # a single bounded read silently truncates the JSONL to
                    # the first chunk, producing a PARTIAL map. Drain the full
                    # stream to EOF (matching the frontend's res.text()),
                    # enforcing the size cap incrementally as we accumulate.
                    buf = bytearray()
                    async for chunk in response.content.iter_chunked(65536):
                        buf.extend(chunk)
                        if len(buf) > MAX_HISTORY_RESPONSE_BYTES:
                            raise OpenNeatoApiError(
                                f"Session {filename} exceeds size cap "
                                f"({MAX_HISTORY_RESPONSE_BYTES} bytes)"
                            )
                    # Firmware emits UTF-8 JSONL; hardcode rather than
                    # call response.get_encoding(), which raises in
                    # modern aiohttp when content was streamed via
                    # response.content (the streaming path doesn't
                    # populate the response's _body buffer that
                    # get_encoding's chardet fallback needs).
                    return served_from, bytes(buf)
        except aiohttp.ClientConnectionError as err:
            raise OpenNeatoConnectionError(
                f"Unable to connect to OpenNeato at {self._host}: {err}"
            ) from err
        except aiohttp.ClientResponseError as err:
            raise OpenNeatoApiError(
                f"API error from /api/history/{filename}: {err.status} {err.message}",
                err.status,
            ) from err
        except TimeoutError as err:
            _LOGGER.warning(
                "Timeout on GET /api/history/%s (limit %ss)", filename, TIMEOUT
            )
            raise OpenNeatoConnectionError(
                f"Timeout connecting to OpenNeato at {self._host}"
            ) from err

    # ── POST endpoints ───────────────────────────────────────────────

    async def clean(self, action: str) -> dict[str, Any] | str:
        """Send a clean command.

        NeatoSerial::clean() recognises "dock", "pause", "stop" and "spot";
        every other value falls through to EVT_START_HOUSE. There is no
        "resume" action -- the robot's own state machine treats a house-clean
        event while paused as a resume, which is why async_start() sends
        "house" in both cases. Passing an unrecognised string here would
        silently start a fresh house clean instead of erroring.
        """
        return await self._post("/api/clean", params={"action": action})

    async def play_sound(self, sound_id: int) -> dict[str, Any] | str:
        """Play a sound by ID (0-20)."""
        return await self._post("/api/sound", params={"id": str(sound_id)})

    async def power(self, action: str) -> dict[str, Any] | str:
        """Send a power command (on, off, standby, shutdown)."""
        return await self._post("/api/power", params={"action": action})

    async def set_user_setting(
        self, key: str, value: str
    ) -> dict[str, Any] | str:
        """Set a single user setting via query params."""
        return await self._post(
            "/api/user-settings", params={"key": key, "value": value}
        )

    async def send_serial_command(self, cmd: str) -> str:
        """Send a raw serial command. Returns plain text."""
        result = await self._post("/api/serial", params={"cmd": cmd})
        return str(result)

    async def clear_errors(self) -> dict[str, Any] | str:
        """Clear all UI errors and alerts."""
        return await self._post("/api/clear-errors")

    async def restart(self) -> dict[str, Any] | str:
        """Restart the ESP32 BRIDGE, not the robot.

        `/api/system/restart` lands on `SystemManager::restart` -> `ESP.restart()`.
        The robot itself is restarted through `/api/power?action=restart`, which
        goes via `TestMode On` + `SetSystemMode` and **re-zeroes the localisation
        frame**: the next session then arrives turned by a quarter. This
        docstring used to say "robot controller" and invited exactly that
        confusion.
        """
        return await self._post("/api/system/restart")

    async def new_battery(self) -> dict[str, Any] | str:
        """Reset battery fuel-gauge calibration after physically replacing the pack."""
        return await self._post("/api/battery/new")

    async def format_fs(self) -> dict[str, Any] | str:
        """Format the filesystem."""
        return await self._post("/api/system/format-fs")

    # ── PUT endpoints ────────────────────────────────────────────────

    async def update_settings(
        self, settings: dict[str, Any]
    ) -> dict[str, Any]:
        """Update device settings (JSON body). Returns full settings."""
        return await self._put("/api/settings", json_data=settings)