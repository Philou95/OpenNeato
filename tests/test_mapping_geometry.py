"""Mapping regressions: independent geometry and set-overlap oracles, offline."""
import importlib
import math
import random
import unittest
from unittest.mock import patch

from test_history_delivery import runner  # Reuse the offline HA service stubs.

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


if __name__ == "__main__":
    unittest.main()
