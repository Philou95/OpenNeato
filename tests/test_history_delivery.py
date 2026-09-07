"""Offline regression tests: real integration code, simulated HA and transport.

Run: python -m unittest discover -s tests -p 'test_*.py'
Requires aiohttp and Pillow; never contacts Home Assistant or the robot.
"""
import asyncio
import importlib
import json
import sys
import types
import unittest
from unittest.mock import AsyncMock, patch
from pathlib import Path


def stub(name):
    value = types.ModuleType(name)
    sys.modules[name] = value
    return value


stub("homeassistant")
core = stub("homeassistant.core")
core.HomeAssistant = object
core.callback = lambda fn: fn
stub("homeassistant.components")
stub("homeassistant.components.vacuum").VacuumActivity = types.SimpleNamespace(**{
    name: name for name in ("CLEANING", "DOCKED", "IDLE", "PAUSED", "RETURNING", "ERROR")
})
stub("homeassistant.exceptions").HomeAssistantError = type("HomeAssistantError", (Exception,), {})
stub("homeassistant.helpers")
stub("homeassistant.helpers.event").async_track_time_interval = lambda *args: lambda: None


class MemoryStore:
    def __init__(self, hass, version, key):
        self.hass, self.key = hass, key

    async def async_load(self):
        return self.hass.storage.get(self.key)

    def async_delay_save(self, factory, delay):
        self.hass.storage[self.key] = factory()


stub("homeassistant.helpers.storage").Store = MemoryStore
package = stub("openneato_under_test")
package.__path__ = [str(Path(__file__).resolve().parents[1] / "custom_components/openneato")]
api = importlib.import_module("openneato_under_test.api")
runner = importlib.import_module("openneato_under_test.lidar_runner")
BOOT_A, BOOT_B = "0123456789abcdef", "fedcba9876543210"


def scan(seq, boot=BOOT_A, **extra):
    rec = {"seq": seq, "x": 0.5, "y": 0.5, "t": 0, "rpm": 5, "d": [1000]}
    if boot is not None:
        rec["boot"] = boot
    rec.update(extra)
    return json.dumps(rec)


class HistoryCacheTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.client = api.OpenNeatoApiClient("offline.invalid", None)

    async def test_two_sessions_cannot_exchange_prefixes(self):
        for second_prefix in (b'B\n', b'longer second session\n'):
            with self.subTest(second_prefix=second_prefix):
                prefix = b'{"type":"session","mode":"house"}\n'
                tail = b'{"x":2,"y":1,"t":0,"ts":12}\n'
                self.client._join_history("100.jsonl", b"", 0, prefix)
                entered, release = asyncio.Event(), asyncio.Event()

                async def fetch(name, since=0):
                    if name == "100.jsonl":
                        self.assertEqual(since, len(prefix))
                        entered.set()
                        await release.wait()
                        return since, tail
                    return 0, second_prefix

                self.client._fetch_history_once = fetch
                first = asyncio.create_task(self.client.get_history_session("100.jsonl"))
                await entered.wait()
                self.assertEqual(await self.client.get_history_session("200.jsonl"), second_prefix.decode())
                release.set()
                self.assertEqual(await first, (prefix + tail).decode())

    async def test_same_session_requests_are_coalesced(self):
        release = asyncio.Event()
        calls = []

        async def fetch(name, since=0):
            calls.append(name)
            await release.wait()
            return 0, b'whole\n'

        self.client._fetch_history_once = fetch
        first = asyncio.create_task(self.client.get_history_session("100.jsonl"))
        second = asyncio.create_task(self.client.get_history_session("100.jsonl"))
        await asyncio.sleep(0)
        release.set()
        self.assertEqual(await asyncio.gather(first, second), ["whole\n", "whole\n"])
        self.assertEqual(calls, ["100.jsonl"])

    async def test_partial_line_is_requested_again(self):
        self.client._join_history("100.jsonl", b"", 0, b'complete\npart')

        async def fetch(name, since=0):
            self.assertEqual(since, len(b'complete\n'))
            return since, b'partial now complete\n'

        self.client._fetch_history_once = fetch
        self.assertEqual(await self.client.get_history_session("100.jsonl"), "complete\npartial now complete\n")

    def test_refused_offset_replaces_prefix(self):
        self.assertEqual(self.client._join_history("100.jsonl", b'old\n', 0, b'new\n'), "new\n")

    def test_joined_size_cap_is_enforced(self):
        with self.assertRaises(api.OpenNeatoApiError):
            self.client._join_history("100.jsonl", b'x' * api.MAX_HISTORY_RESPONSE_BYTES, api.MAX_HISTORY_RESPONSE_BYTES, b'x')

    async def test_boot_identifier_is_sent_and_validated(self):
        paths = []

        async def get_text(path):
            paths.append(path)
            return ""

        self.client._get_text = get_text
        await self.client.get_lidar_buffer(12, BOOT_A)
        await self.client.get_lidar_buffer(0)
        self.assertEqual(paths, [f"/api/lidar/buffer?after=12&boot={BOOT_A}", "/api/lidar/buffer?after=0"])
        with self.assertRaises(api.OpenNeatoApiError):
            await self.client.get_lidar_buffer(1, "bad&after=900")


class ScanParserTests(unittest.TestCase):
    def test_new_boot_resets_sequence_and_retains_new_scans(self):
        captures, high, boot = runner._parse_scans(scan(1, BOOT_B), 360, BOOT_A)
        self.assertEqual((len(captures), high, boot), (1, 1, BOOT_B))

    def test_repeated_and_partial_batches_are_deduplicated(self):
        batch = "\n".join(scan(seq) for seq in (1, 2, 2, 3))
        captures, high, boot = runner._parse_scans(batch, 2, BOOT_A)
        self.assertEqual((len(captures), high, boot), (1, 3, BOOT_A))

    def test_rejected_geometry_is_still_acknowledged(self):
        captures, high, _ = runner._parse_scans(scan(9, mv=100), 8, BOOT_A)
        self.assertEqual((captures, high), ([], 9))

    def test_mixed_or_invalid_boots_are_not_acknowledged(self):
        for text in (scan(1) + "\n" + scan(2, BOOT_B), scan(1, "invalid")):
            with self.subTest(text=text), self.assertRaises(ValueError):
                runner._parse_scans(text, 0, BOOT_A)

    def test_legacy_firmware_remains_readable(self):
        captures, high, boot = runner._parse_scans(scan(8, None), 7, None)
        self.assertEqual((len(captures), high, boot), (1, 8, None))


class Hass:
    def __init__(self):
        self.storage = {}

    async def async_add_executor_job(self, fn, *args):
        return fn(*args)


class RunnerTests(unittest.IsolatedAsyncioTestCase):
    def make_runner(self, bridge, hass=None):
        coordinator = types.SimpleNamespace(
            data={"state": {"uiState": "UIMGR_STATE_HOUSECLEANINGRUNNING"},
                  "history": [{"name": "100.jsonl", "recording": True}]},
            async_add_listener=lambda fn: lambda: None,
        )
        return runner.LidarMapRunner(hass or Hass(), "offline", bridge, coordinator)

    async def test_filtered_batches_are_live_transport_and_do_not_trigger_serial_fallback(self):
        counter = 0
        async def batch(after, boot):
            nonlocal counter
            counter += 1
            return scan((counter + 1) // 2, mv=10.) if counter % 2 else ""
        bridge = types.SimpleNamespace(get_lidar_buffer=batch, send_serial_command=AsyncMock())
        r = self.make_runner(bridge)
        r._track_new = AsyncMock()
        r._orient_live_from_frame = AsyncMock()
        r._align_live = AsyncMock(return_value=False)
        r._persist_captures = AsyncMock()
        r._last_health = runner.time.monotonic()
        for _ in range(runner.DRAIN_QUIET_TICKS + 2):
            await r._sample_tick()
        self.assertTrue(r._buffer_ok)
        self.assertEqual(r._empty_drains, 0)
        self.assertEqual(r._last_seq, runner.DRAIN_QUIET_TICKS + 2)
        self.assertEqual(r._captures, [])
        r._persist_captures.assert_awaited()
        bridge.send_serial_command.assert_not_awaited()

    async def test_repeated_old_batch_does_not_hide_a_real_stall(self):
        bridge = types.SimpleNamespace(
            get_lidar_buffer=AsyncMock(return_value=scan(7)),
            send_serial_command=AsyncMock(return_value=""), get_lidar=AsyncMock(return_value={}),
        )
        r = self.make_runner(bridge)
        r._buffer_ok = True
        r._scan_boot_id, r._last_seq = BOOT_A, 7
        r._track_new = AsyncMock()
        r._orient_live_from_frame = AsyncMock()
        r._align_live = AsyncMock(return_value=False)
        r._last_health = runner.time.monotonic()
        for _ in range(runner.DRAIN_QUIET_TICKS - 1):
            await r._sample_tick()
        bridge.send_serial_command.assert_not_awaited()
        await r._sample_tick()
        bridge.send_serial_command.assert_awaited()

    async def test_bridge_restart_recovers_all_new_scans(self):
        class Bridge:
            def __init__(self):
                self.after = []

            async def get_lidar_buffer(self, after, boot_id=None):
                self.after.append((after, boot_id))
                cursor = after if boot_id == BOOT_B else 0
                return "\n".join(scan(n, BOOT_B) for n in range(cursor + 1, min(cursor + 4, 20) + 1))

        bridge = Bridge()
        r = self.make_runner(bridge)
        r._last_seq, r._scan_boot_id = 360, BOOT_A
        self.assertEqual(await r._drain_buffer(), 20)
        self.assertEqual((r._last_seq, r._scan_boot_id, len(r._captures)), (20, BOOT_B, 20))
        self.assertEqual(bridge.after[1], (4, BOOT_B))

    async def test_saved_cursor_matches_saved_captures_and_resumes(self):
        hass = Hass()
        r = self.make_runner(None, hass)
        r._session_name, r._scan_boot_id, r._last_seq = "100.jsonl", BOOT_A, 7
        r._captures = [(0.5, 0.5, 0, [(0, 1000)], 5, 0, 0)]
        r._last_persist = runner.time.monotonic() - runner.CAPTURE_PERSIST_S - 1
        await r._persist_captures()
        r._captures.append(r._captures[0])
        restored = self.make_runner(None, hass)
        await restored.async_load()
        restored._start()
        self.assertEqual((restored._scan_boot_id, restored._last_seq), (BOOT_A, 7))
        self.assertEqual(len(restored._captures), 1)

    async def test_tick_is_reserved_while_tracking_awaits(self):
        r = self.make_runner(None)
        r.coordinator.data["state"]["uiState"] = "IDLE"
        entered, release = asyncio.Event(), asyncio.Event()
        calls = []

        async def track():
            calls.append(1)
            entered.set()
            await release.wait()

        r._track_new = track
        first = asyncio.create_task(r._async_tick())
        await entered.wait()
        await r._async_tick()
        self.assertTrue(r._busy)
        self.assertEqual(len(calls), 1)
        release.set()
        await first
        self.assertFalse(r._busy)

    async def test_finish_waits_for_sample_and_tracks_the_final_drain_once(self):
        r = self.make_runner(None)
        r._collecting = True
        r.coordinator.data["state"]["uiState"] = "IDLE"
        entered, release = asyncio.Event(), asyncio.Event()
        consumed, merged = [], []
        r._tracker = types.SimpleNamespace(add_many=consumed.extend)

        async def sample():
            entered.set()
            await release.wait()
            r._captures.append(("in-flight",))

        async def drain():
            r._captures.append(("last-buffered",))

        async def merge():
            merged.append((list(r._captures), r._tracked, list(consumed)))

        r._sample_tick, r._drain_final_buffer, r._merge_run = sample, drain, merge
        tick = asyncio.create_task(r._async_tick())
        await entered.wait()
        finish = asyncio.create_task(r._finish())
        await asyncio.sleep(0)
        await r._finish()  # Duplicate coordinator notification must do nothing.
        await r._async_tick()  # No new work while completion waits for the lock.
        self.assertEqual(merged, [])
        self.assertTrue(r.merging)
        release.set()
        await asyncio.gather(tick, finish)
        captures = [("in-flight",), ("last-buffered",)]
        self.assertEqual(merged, [(captures, 2, captures)])
        self.assertFalse(r.merging)

    async def test_final_drain_waits_for_acknowledged_ring_and_pending_scan(self):
        statuses = iter([{"ring": 2}, {"pending": 1}, {"ring": 0, "pending": 0}])

        async def status():
            return next(statuses)

        r = self.make_runner(types.SimpleNamespace(get_lidar_status=status))
        drains = []

        async def drain():
            drains.append(True)
            return 0

        r._drain_buffer = drain
        with patch.object(runner.asyncio, "sleep", new=unittest.mock.AsyncMock()):
            await r._drain_final_buffer()
        self.assertEqual(len(drains), 3)

    async def test_merge_recomputes_if_tracker_has_not_consumed_every_capture(self):
        r = self.make_runner(None)
        r._captures = [(0.5, 0.5, 0, [(0, 1000)], 5, 0, 0)] * 61
        r._tracked = 60
        r._tracker = types.SimpleNamespace(refused=0, usable=lambda dropped: True)
        r._map = types.SimpleNamespace(as_dict=lambda: {}, merge_session=lambda *args: {"rejected": True})
        r._dump_captures = lambda captures: None
        seen = []

        def build(captures, *, refine, tracker):
            seen.append((len(captures), tracker))
            return {}, {}, {}, None

        with patch.object(runner, "build_session_grids", new=build):
            await r._merge_run()
        self.assertEqual(seen, [(61, None)])


if __name__ == "__main__":
    unittest.main()
