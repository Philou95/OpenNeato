"""Mapping regressions: independent geometry and set-overlap oracles, offline."""
import asyncio
import importlib
import math
import random
import types
import unittest
from unittest.mock import AsyncMock, patch

from test_history_delivery import Hass, runner  # Reuse the offline HA service stubs.

mapper = importlib.import_module("openneato_under_test.lidar_mapper")
slam = importlib.import_module("openneato_under_test.slam")
replay = importlib.import_module("openneato_under_test.replay")


class SetScorer:
    """Slow independent oracle for the accelerated search, including ties."""

    def __init__(self, reference):
        self.reference = set(reference)

    def prepare(self, cells):
        self.cells = set(cells)
        return min(len(self.cells), len(self.reference))

    def count(self, dx, dy):
        shifted = {(x + dx, y + dy) for x, y in self.cells}
        return len(shifted & self.reference)


class OverlapTests(unittest.TestCase):
    def test_exact_scores_with_duplicates_negative_coordinates_and_far_shifts(self):
        rng = random.Random(953)
        for _ in range(30):
            ref = {(rng.randrange(-30, 30), rng.randrange(-20, 20)) for _ in range(100)}
            cells = [(rng.randrange(-30, 30), rng.randrange(-20, 20)) for _ in range(120)]
            cells += cells[:30]
            fast, slow = mapper._OverlapScorer(ref), SetScorer(ref)
            denominator = fast.prepare(cells)
            self.assertEqual(denominator, slow.prepare(cells))
            for dx in (-10**12, -60, -12, 0, 15, 60, 10**12):
                for dy in (-50, -5, 0, 13, 50):
                    with self.subTest(dx=dx, dy=dy):
                        self.assertEqual(fast.count(dx, dy), slow.count(dx, dy))
                        self.assertLessEqual(fast.count(dx, dy), denominator)

    def test_sparse_fallback_and_angle_cache_reset(self):
        ref = {(0, 0), (10**12, 3), (-10**12, -2)}
        scorer = mapper._OverlapScorer(ref)
        self.assertIsNone(scorer._ref_rows)
        for cells in ([(0, 0), (0, 0)], [(0, -3), (10**12, 0)], []):
            scorer.prepare(cells)
            oracle = SetScorer(ref)
            oracle.prepare(cells)
            for dx in (-10**12, 0, 10**12):
                self.assertEqual(scorer.count(dx, 3), oracle.count(dx, 3))
        narrow = mapper._OverlapScorer({(0, 0), (1, 0)})
        narrow.prepare([(0, 0)])
        self.assertEqual(narrow.count(0, 0), 1)
        narrow.prepare([(0, 1)])
        self.assertEqual(narrow.count(0, 0), 0)
        with patch.object(mapper._OverlapScorer, "_MAX_BITS", 1):
            bounded = mapper._OverlapScorer({(0, 0), (1, 1)})
            self.assertIsNone(bounded._ref_rows)
            bounded.prepare([(1, 1)])
            self.assertEqual(bounded.count(0, 0), 1)

    def test_search_matches_set_oracle_for_partial_and_rotated_maps(self):
        ref = {(x, y): 10 for x in range(-25, 35) for y in range(-20, 25)
               if x == -25 or y == -20 or (x == 34 and y < 4) or (y == 24 and x < 7)}
        for quarter, fine in ((0, 0), (1, 13), (3, -22)):
            walls = mapper._rotate_wall_counts(ref, quarter, fine, 3, -2)
            if quarter == 3:
                walls = dict(list(walls.items())[::2])
            expected = mapper.align_to_reference(walls, ref, search_cells=2)
            with patch.object(mapper, "_OverlapScorer", SetScorer):
                actual = mapper.align_to_reference(walls, ref, search_cells=2)
            self.assertEqual(actual, expected)
            self.assertTrue(0 <= actual[3] <= 1)
            self.assertTrue(all(0 <= score <= 1 for score in actual[5]))

    def test_measurable_angle_ignores_hint_without_duplicate_search(self):
        walls = {(x, 0): 10 for x in range(40)}
        with patch.object(mapper, "_measured_turn", return_value=0.0) as measure:
            hinted = mapper.align_to_reference(walls, walls, angle_hint=10)
            self.assertEqual(measure.call_count, 1)
        with patch.object(mapper, "_measured_turn", return_value=0.0):
            self.assertEqual(hinted, mapper.align_to_reference(walls, walls))

    def test_thin_map_retains_both_hint_searches(self):
        walls = {(x, x // 3): 10 for x in range(20)}
        with patch.object(mapper, "_measured_turn", return_value=None) as measure:
            actual = mapper.align_to_reference(walls, walls, search_cells=1, angle_hint=10)
            self.assertEqual(measure.call_count, 3)
            blind = mapper.align_to_reference(walls, walls, 1, None, True)
            aimed = mapper.align_to_reference(walls, walls, 1, 10, True)
        self.assertEqual(actual, aimed if aimed[3] > blind[3] else blind)

    def test_hinted_sweep_keeps_absolute_angle_grid(self):
        walls = {(x, x // 3): 10 for x in range(20)}
        hint = 2.13
        with patch.object(mapper, "_measured_turn", return_value=None), \
             patch.object(mapper, "_rotate_cells_fine", wraps=mapper._rotate_cells_fine) as rotate:
            mapper.align_to_reference(walls, walls, 1, hint, True)
        angles = [call.args[1] for call in rotate.call_args_list]
        expected = [step * mapper.FINE_STEP_DEG for step in range(
            math.ceil((hint-mapper.FINE_HINT_SPAN_DEG)/mapper.FINE_STEP_DEG),
            math.floor((hint+mapper.FINE_HINT_SPAN_DEG)/mapper.FINE_STEP_DEG)+1,
        ) if step != 0]
        self.assertEqual(angles[:4], [hint] * 4)
        self.assertEqual(angles[4:4+len(expected)], expected)

    def test_wrong_hint_cannot_lower_blind_score_on_thin_map(self):
        walls = {(x, x // 3): 10 for x in range(20)}
        with patch.object(mapper, "_measured_turn", return_value=None):
            blind = mapper.align_to_reference(walls, walls, 1)
            hinted = mapper.align_to_reference(walls, walls, 1, 45.)
        self.assertGreaterEqual(hinted[3], blind[3])


class GeometryTests(unittest.TestCase):
    def test_rotated_cell_centres_match_continuous_replay(self):
        cells = [(-41, -7), (-1, 0), (0, 0), (10, 20), (73, -22)]
        for q in range(4):
            for fine in (0, .25, -13, 37, 82):
                norm = [((x+.5)*mapper.CELL_M, (y+.5)*mapper.CELL_M, 0, 0) for x,y in cells]
                transformed = replay._apply_alignment(norm, (q, 3, -2, fine))
                expected = [(math.floor(x/mapper.CELL_M), math.floor(y/mapper.CELL_M))
                            for x,y,_,_ in transformed]
                actual = [(x+3, y-2) for x,y in
                          mapper._rotate_cells_fine(mapper._rotate_cells(cells, q), fine)]
                self.assertEqual(actual, expected, (q, fine))

    def test_rotation_preserves_wall_weight_and_order_independence(self):
        walls = {(x,y): float(1 + (x-y) % 17) for x in range(-20,20) for y in range(-20,20)}
        for q in range(4):
            for fine in (0, 3, 13, 37, 45):
                result = mapper._rotate_wall_counts(walls, q, fine, -7, 2)
                reverse = mapper._rotate_wall_counts(dict(reversed(list(walls.items()))), q, fine, -7, 2)
                self.assertEqual(math.fsum(result.values()), math.fsum(walls.values()))
                self.assertEqual(result, reverse)
                if fine == 45:
                    self.assertLess(len(result), len(walls))

    def test_ambiguous_merge_preserves_map_and_rejection_counter(self):
        walls = {(x,y): 10.0 for x in range(-40,40) for y in range(-40,40)
                 if x in (-40,39) or y in (-40,39)}
        store = mapper.AccumulatedMap({"walls": {f"{x},{y}": w for (x,y),w in walls.items()},
                                       "sessions": 29, "rejects": 2, "floor": ["1,1"]})
        before = store.as_dict()
        result = store.merge_session(walls, {(10,10)}, "new.jsonl", set(walls))
        self.assertTrue(result["ambiguous"])
        self.assertFalse(result["contributed"])
        after = store.as_dict()
        for field in ("walls", "floor", "sessions", "rejects", "quarter_lock"):
            self.assertEqual(after[field], before[field], field)
        self.assertIn("new.jsonl", after["alignments"])

    def test_clear_merge_accumulates_collisions_and_keeps_saved_alignments(self):
        old_aligns = {f"old{n}.jsonl": [1, 2, -3, 13, 4, 5, .7][:n] for n in range(3,8)}
        store = mapper.AccumulatedMap({"walls": {"100,100": 2}, "sessions": 1, "alignments": old_aligns})
        walls = {(x,y): 1.0 for x in range(-6,6) for y in range(-6,6)}
        fit = (1, 2, -3, .9, 45, (.1,.9,.2,.1), False)
        with patch.object(mapper, "align_to_reference", return_value=fit):
            result = store.merge_session(walls, set(), "new.jsonl")
        self.assertTrue(result["contributed"])
        self.assertEqual(math.fsum(store.walls.values()), 2 + len(walls))
        loaded = mapper.AccumulatedMap(store.as_dict())
        for name, align in old_aligns.items():
            self.assertEqual(loaded.as_dict()["alignments"][name], align)
            self.assertEqual(replay._apply_alignment([(1.,2.,30.,4.)], loaded.alignments[name]),
                             replay._apply_alignment([(1.,2.,30.,4.)], align))


class IcpTests(unittest.TestCase):
    def test_rejected_coarse_boundary_skips_unused_fine_search(self):
        def score(walls, x, y, theta, points):
            return -((x - .3)**2 + y*y + theta*theta)
        with patch.object(mapper, "_match_score", side_effect=score) as scoring:
            result = mapper.match_pose({}, 0., 0., 0., [(0, 1000)])
        self.assertTrue(result[3])
        self.assertEqual(scoring.call_count, 125)

    def test_interior_match_still_receives_fine_refinement(self):
        def score(walls, x, y, theta, points):
            return -((x - .025)**2 + y*y + theta*theta)
        with patch.object(mapper, "_match_score", side_effect=score) as scoring:
            result = mapper.match_pose({}, 0., 0., 0., [(0, 1000)])
        self.assertEqual(result, (.025, 0., 0., False))
        self.assertEqual(scoring.call_count, 250)

    def test_drift_stop_is_counted_without_changing_existing_fallback(self):
        tracker = mapper.SessionTracker()
        capture = (0., 0., 0., [(0, 1000)], 5., 0., 0.)
        with patch.object(mapper.SessionTracker, "_close_loops"):
            for _ in range(mapper.MATCH_SEED):
                tracker.add(capture)
            with patch.object(mapper, "match_pose", return_value=(1.,0.,0.,False)):
                tracker.add(capture)
        self.assertEqual(tracker.stopped_at, mapper.MATCH_SEED + 1)
        self.assertFalse(tracker._matching)
        self.assertEqual((tracker._dx, tracker._dy, tracker._dth), (0.,0.,0.))

    def test_quality_is_measured_at_returned_pose_when_iteration_limit_is_hit(self):
        src = [(x*.4,y*.4) for x in range(8) for y in range(5)]
        dst = [(x+.06,y) for x,y in src]
        x, y, th, residual, fit = slam.icp(src, dst, 0, 0, 0, iters=1)
        self.assertAlmostEqual(x, .06)
        self.assertAlmostEqual(y, 0)
        self.assertAlmostEqual(th, 0)
        self.assertAlmostEqual(residual, 0)
        self.assertEqual(fit, 1)

    def test_quality_matches_independent_nearest_neighbour_oracle(self):
        rng = random.Random(41)
        src = [(rng.uniform(-2,2), rng.uniform(-2,2)) for _ in range(150)]
        dst = [(x+.09,y-.04) for x,y in src[:120]]
        for iters in (1, 3, 30):
            x,y,th,res,fit = slam.icp(src,dst,0,0,0,iters=iters)
            c,s = math.cos(th), math.sin(th)
            distances = [min(math.hypot(px*c-py*s+x-qx,px*s+py*c+y-qy) for qx,qy in dst)
                         for px,py in src]
            matched = sorted(d for d in distances if d < slam.MAX_PAIR_M)
            self.assertAlmostEqual(fit, len(matched)/len(src))
            self.assertAlmostEqual(res, slam._median(matched))


class LivePlacementTests(unittest.IsolatedAsyncioTestCase):
    def make_runner(self):
        bridge = types.SimpleNamespace(get_lidar_status=AsyncMock(return_value={
            "frameOffsetKnown": 1, "frameOffset": -88.26,
        }))
        coordinator = types.SimpleNamespace(data={}, async_add_listener=lambda fn: lambda: None)
        result = runner.LidarMapRunner(Hass(), "offline", bridge, coordinator)
        result._tracker = types.SimpleNamespace(placed=25, correction=(0., 0., 0.))
        result._map = types.SimpleNamespace(walls={(0,0): 10}, alignments={})
        result._session_name = "100.jsonl"
        return result

    async def test_frame_preview_rotates_raw_replay_before_25_scans(self):
        r = self.make_runner()
        r._tracker.placed = 1
        r._captures = [object()]
        r.api.get_lidar_status.return_value.update(collecting=1, frameOffsetStatus="initial")
        await r._orient_live_from_frame()
        self.assertIsNone(r._live_align)
        self.assertEqual(r._live_align_at, 0.)
        placement = r.alignment("100.jsonl.hs")
        self.assertEqual(placement[0], 1)
        self.assertAlmostEqual(placement[3], -1.74)
        point = replay._apply_alignment([(1., 0., 0., 0.)], placement)[0]
        self.assertAlmostEqual(point[0], math.cos(math.radians(88.26)))
        self.assertAlmostEqual(point[1], math.sin(math.radians(88.26)))
        self.assertIsNone(r.alignment("99.jsonl"))

    async def test_preview_ignores_inactive_previous_frame_then_retries(self):
        r = self.make_runner()
        r._captures = [object()]
        r.api.get_lidar_status.return_value.update(collecting=0)
        await r._orient_live_from_frame()
        self.assertIsNone(r.alignment("100.jsonl"))
        self.assertFalse(r._frame_hint_done)
        r.api.get_lidar_status.return_value.update(collecting=1, frameOffset=-1.15)
        await r._orient_live_from_frame()
        self.assertAlmostEqual(r.alignment("100.jsonl")[3], 1.15)

    async def test_geometric_then_saved_alignment_take_precedence(self):
        r = self.make_runner()
        await r._frame_angle_hint()
        fit = (0, -4, -4, .6, -.5, (.6,.1,.1,.1), False)
        with patch.object(r, "_fit_live", return_value=fit):
            await r._align_live()
        self.assertEqual(r.alignment("100.jsonl")[:4], (0,-4,-4,-.5))
        saved = (0,-11,-3,-.3,1.,2.,-.17)
        r._map.alignments["100.jsonl"] = saved
        self.assertEqual(r.alignment("100.jsonl.hs"), saved)

    async def test_hint_is_adjusted_for_the_corrected_grid(self):
        r = self.make_runner()
        await r._frame_angle_hint()
        self.assertAlmostEqual(r._corrected_frame_hint((0.,0.,2.)), -3.74)
        self.assertAlmostEqual(r._corrected_frame_hint((0.,0.,92.)), -3.74)

    def test_merge_passes_hint_to_geometric_search(self):
        store = mapper.AccumulatedMap()
        store.walls = {(0,0):10}
        fit = (0,0,0,.9,1.,(.9,.1,.1,.1),False)
        with patch.object(mapper, "align_to_reference", return_value=fit) as search:
            store.merge_session({(0,0):10}, set(), "new.jsonl", angle_hint=1.3)
        self.assertEqual(search.call_args.kwargs["angle_hint"], 1.3)

    async def test_first_attempt_does_not_wait_for_five_minutes_of_uptime(self):
        r = self.make_runner()
        fit = (1, 0, 0, .6, -1.74, (.1,.6,.1,.1), False)
        with patch.object(r, "_fit_live", return_value=fit), patch.object(runner.time, "monotonic", return_value=12.):
            self.assertTrue(await r._align_live())
        self.assertEqual(r._live_align[0], 1)
        r.api.get_lidar_status.assert_awaited_once()

    async def test_undecided_placement_retries_after_thirty_seconds(self):
        r = self.make_runner()
        undecided = (0, 0, 0, .4, 0., (.4,.39,.1,.1), False)
        decided = (1, 0, 0, .6, -1.74, (.1,.6,.1,.1), False)
        with patch.object(r, "_fit_live", side_effect=[undecided, decided]) as fit:
            with patch.object(runner.time, "monotonic", return_value=100.):
                self.assertTrue(await r._align_live())
                self.assertIsNone(r._live_align)
            with patch.object(runner.time, "monotonic", return_value=129.):
                self.assertFalse(await r._align_live())
            with patch.object(runner.time, "monotonic", return_value=130.):
                self.assertTrue(await r._align_live())
            self.assertEqual(fit.call_count, 2)
        self.assertEqual(r._live_align[0], 1)

    async def test_existing_placement_keeps_slower_refresh(self):
        r = self.make_runner()
        r._live_align = (1, 0, 0, -1.74, 0., 0., 0.)
        r._live_align_at = 100.
        with patch.object(runner.time, "monotonic", return_value=130.):
            self.assertFalse(await r._align_live())
        r.api.get_lidar_status.assert_not_awaited()

    async def test_unplaced_failures_back_off_and_allow_late_success(self):
        undecided = (0, 0, 0, .4, 0., (.4,.39,.1,.1), False)
        decided = (1, 0, 0, .6, -1.74, (.1,.6,.1,.1), False)
        for failure in (undecided, None, RuntimeError("fit failed")):
            with self.subTest(failure=failure):
                r = self.make_runner()
                with patch.object(r, "_fit_live", side_effect=[failure] * 5 + [decided]) as fit:
                    for instant in (100., 130., 190., 310., 550., 850.):
                        with patch.object(runner.time, "monotonic", return_value=instant - 1):
                            if instant != 100.:
                                self.assertFalse(await r._align_live())
                        with patch.object(runner.time, "monotonic", return_value=instant):
                            self.assertTrue(await r._align_live())
                    self.assertEqual(fit.call_count, 6)
                self.assertEqual(r._live_align[0], 1)
                self.assertEqual(r._live_attempts, 0)
                with patch.object(runner.time, "monotonic", return_value=1149.):
                    self.assertFalse(await r._align_live())

    async def test_hour_of_ambiguity_limits_ticks_spent_fitting(self):
        r = self.make_runner()
        undecided = (0, 0, 0, .4, 0., (.4,.39,.1,.1), False)
        with patch.object(r, "_fit_live", return_value=undecided) as fit:
            for instant in range(100, 3700):
                with patch.object(runner.time, "monotonic", return_value=float(instant)):
                    await r._align_live()
            self.assertEqual(fit.call_count, 15)
        self.assertIsNone(r._live_align)
        self.assertEqual(r._live_stable, 0)

    async def test_initial_frame_is_cached_instead_of_following_raw_drift(self):
        r = self.make_runner()
        r.api.get_lidar_status.side_effect = [
            {"frameOffsetKnown": 1, "frameOffset": -3.39, "frameOffsetStatus": "initial"},
            {"frameOffsetKnown": 1, "frameOffset": -53.84, "frameOffsetStatus": "initial"},
        ]
        self.assertAlmostEqual(await r._frame_angle_hint(), 3.39)
        self.assertAlmostEqual(await r._frame_angle_hint(), 3.39)
        self.assertEqual(r.api.get_lidar_status.await_count, 1)

    async def test_pending_frame_can_later_use_restored_initial_measurement(self):
        r = self.make_runner()
        r.api.get_lidar_status.side_effect = [
            {"frameOffsetKnown": 0, "frameOffsetStatus": "pending"},
            {"frameOffsetKnown": 1, "frameOffset": -88.26, "frameOffsetStatus": "restored"},
        ]
        self.assertIsNone(await r._frame_angle_hint())
        self.assertFalse(r._frame_hint_done)
        self.assertAlmostEqual(await r._frame_angle_hint(), -1.74)
        self.assertTrue(r._frame_hint_done)

    async def test_invalid_frames_do_not_become_zero_or_poison_cache(self):
        for value in (None, "invalid", float("nan"), float("inf"), -181., 181.):
            r = self.make_runner()
            r.api.get_lidar_status.return_value = {"frameOffsetKnown": 1, "frameOffset": value}
            self.assertIsNone(await r._frame_angle_hint())
            self.assertFalse(r._frame_hint_done)
        for status in ({"frameOffsetKnown": 1},
                       {"frameOffsetKnown": 1, "frameOffset": 0., "frameOffsetStatus": "unknown"}):
            r = self.make_runner()
            r.api.get_lidar_status.return_value = status
            self.assertIsNone(await r._frame_angle_hint())
        r.api.get_lidar_status.return_value = {"frameOffsetKnown": 1, "frameOffset": 0., "frameOffsetStatus": "initial"}
        self.assertEqual(await r._frame_angle_hint(), 0.)
        self.assertTrue(r._frame_hint_done)

    async def test_cancelling_hint_read_releases_alignment_flag(self):
        r = self.make_runner()
        r.api.get_lidar_status.side_effect = asyncio.CancelledError
        with self.assertRaises(asyncio.CancelledError):
            await r._align_live()
        self.assertFalse(r._aligning)


class PoseGraphTests(unittest.TestCase):
    def test_jacobian_blocks_agree_with_finite_differences(self):
        poses = [[.4, -.7, .32], [1.2, .8, -.41]]
        measurement = (.7, -.1, .2)
        residual = slam._residual(poses, 0, 1, measurement)
        for node, is_i in ((0, True), (1, False)):
            block = slam._blocks(*residual[3:], is_i)
            for col in range(3):
                plus, minus = [p[:] for p in poses], [p[:] for p in poses]
                plus[node][col] += 1e-6
                minus[node][col] -= 1e-6
                upper = slam._residual(plus, 0, 1, measurement)
                lower = slam._residual(minus, 0, 1, measurement)
                for row in range(3):
                    self.assertAlmostEqual(block[3*row+col], (upper[row]-lower[row])/2e-6, places=7)

    def test_cached_rotations_follow_each_gauss_seidel_update(self):
        poses = [[0., 0., 0.], [1.2, .1, .1], [2.1, -.2, -.2]]
        edges = [(0, 1, (1., 0., 0.), 1.), (1, 2, (1., 0., 0.), 1.),
                 (0, 2, (2., 0., 0.), .5)]
        original = slam._residual
        def checked(grid, i, j, z, rotations=None, z_rotation=None):
            if rotations is not None:
                for pose, rotation in zip(grid, rotations):
                    self.assertEqual(rotation, (math.cos(pose[2]), math.sin(pose[2])))
                self.assertEqual(z_rotation, (math.cos(z[2]), math.sin(z[2])))
            actual = original(grid, i, j, z, rotations, z_rotation)
            self.assertEqual(actual, original(grid, i, j, z))
            return actual
        stats = {}
        with patch.object(slam, "_residual", side_effect=checked):
            result = slam.optimise(poses, edges, stats=stats)
        self.assertEqual(result[0], poses[0])
        self.assertTrue(stats["converged"])
        self.assertLess(stats["after"]["translation_m"]["rms"], stats["before"]["translation_m"]["rms"])
        self.assertLess(stats["after"]["rotation_deg"]["rms"], stats["before"]["rotation_deg"]["rms"])
        self.assertEqual(result, slam.optimise(poses, edges))
        self.assertEqual(poses[1], [1.2, .1, .1])

    def test_diagnostics_use_separate_units_and_report_iteration_limit(self):
        poses = [[0., 0., 0.], [1., 0., .2]]
        edges = [(0, 1, (1., 0., 0.), 1.)]
        quality = slam.graph_residuals(poses, edges)
        self.assertEqual(quality["translation_m"]["p95"], 0.)
        self.assertAlmostEqual(quality["rotation_deg"]["p95"], math.degrees(.2))
        stats = {}
        self.assertEqual(slam.optimise(poses, edges, sweeps=0, stats=stats), poses)
        self.assertEqual(stats["sweeps"], 0)
        self.assertFalse(stats["converged"])
        self.assertEqual(stats["before"], stats["after"])
        self.assertEqual(slam.graph_residuals([], [])["edges"], 0)


if __name__ == "__main__":
    unittest.main()
