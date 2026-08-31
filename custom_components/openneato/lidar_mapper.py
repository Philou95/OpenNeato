"""Build a floor plan from the robot's LIDAR, one cleaning at a time.

Nothing here needs a firmware change. Two endpoints already carry everything
required, and they share a clock:

    POST /api/serial?cmd=GetRobotPos Smooth
        -> Robot Smooth pose: X=.., Y=.., Theta=.., Time=..
    GET  /api/lidar
        -> 360 x {angle, dist (mm), intensity, error}

`Time` is the same clock as the `ts` in session JSONL files, so a scan can be
tied to the recorded path with no wall-clock synchronisation.

Beam geometry, measured rather than assumed (95.3% of returns agree, against
60% for the next-best candidate):

    wx = (x - LIDAR_BEHIND_M * cos(radians(theta))) + d * cos(radians(angle + theta))
    wy = (y - LIDAR_BEHIND_M * sin(radians(theta))) + d * sin(radians(angle + theta))

Two things make this survive across sessions:

* **Re-alignment before merging.** The robot's world frame is anchored on the
  dock and can rotate -- a TestMode cycle was observed to turn it by exactly
  90 degrees. Merging a rotated session into the accumulated grid would ruin
  it, so every new session is first fitted to the stored map over the four
  quarter turns plus a translation.
* **Orientation locked to the walls, not the dock.** The straightening angle
  is derived from the walls themselves, so the map always comes out the same
  way up even if the robot's frame moves under it. It is reported as a *view*
  rotation: the plan itself stays in the robot's frame, because that is the
  frame the path and coverage are drawn in.
"""

from __future__ import annotations

import io
import logging
import math
from typing import Any, Iterable

from PIL import Image, ImageDraw, ImageFont

from . import slam
from .const import CELL_SIZE_M, CLEAN_WIDTH_M as _CLEAN_WIDTH_M

_LOGGER = logging.getLogger(__name__)

# 2.5 cm, measured rather than chosen: the LIDAR is faithful to about a
# centimetre standing still and the reconstructed map to two or two and a half,
# so a finer grid would only draw pose noise. Against the 50 x 29 cm box the
# same captures render 60 x 35 at 5 cm and 55 x 38 at 2.5, halving the
# quantisation on every measurement. Cells cost the square of this, so the grid
# is four times denser -- about 16 000 wall cells and 32 000 floor against a
# MAX_GRID_CELLS of 200 000.
CELL_M = CELL_SIZE_M         # grid resolution — defined once, in const.py
MAX_RANGE_M = 3.5            # returns beyond this are noisy and grazing
# Scales with the cell: a return lands in exactly one cell whatever the grid,
# so quartering the cell area quarters the hits each one collects. Left at 12
# this floor alone would have hidden every wall of a young map.
WALL_MIN_HITS = 3            # floor, and the whole rule for a young map.
                             # Swept over the stored map: at 5 a wall is a
                             # 3571-cell smear, at 20 it thins until it breaks
                             # into pieces. 12 is where the outline is still
                             # continuous but no longer blobby after one or
                             # two cleanings.

# Above that floor the threshold follows the map's own distribution instead of
# staying put, because a fixed count silently rots as cleanings accumulate.
#
# Measured on the real map at four sessions: 13 066 cells, median 4 hits, and
# **23.7% of cells seen exactly once, 45% seen three times or fewer**. Those
# are strays -- a person walking past, a grazing return, a scan merged a few
# centimetres out. A fixed 12 keeps admitting them, so the plan gets noisier
# run after run, which is exactly what gets reported.
#
# Scaling linearly with the session count was tried first and is wrong: it
# assumes every cell is re-seen every session, which partial coverage
# contradicts. At four sessions it demands 48 hits and leaves 1157 cells --
# the outline breaks apart.
#
# A quantile was tried first and had to go. It keeps a stable *share* of the
# map, which self-calibrates only while the map's shape stays the same -- and
# it does not: three quarters of the cells were strays, so the 75th percentile
# was really measuring the noise floor, not the walls. The moment
# scan_free_cells() started deleting those strays the same rule cut into the
# walls instead. Simulated on this map: the outline dropped to 65% of its
# cells after one cleaning, undoing the perforation fixed the same day.
#
# Anchor on the signal instead. A cell counts as wall at a fixed fraction of
# what the best-seen walls score, so the bar rises with the session count --
# which is what the quantile was for -- while how much junk sits underneath
# stops mattering. Simulated over the same three cleanings, the outline holds
# at 90% instead of 65%, and what it does drop is stray.
#
# p95 rather than the maximum: one wildly over-seen cell must not set the bar
# for the map. The fraction is calibrated to agree with the rule it replaces
# on the map as it stands -- 0.209 there -- so today's plan is unchanged and
# only the failure mode differs.
WALL_STRONG_QUANTILE = 0.95
WALL_KEEP_FRACTION = 0.21


def wall_threshold(walls: dict[Any, int]) -> float:
    """Hit count a cell must reach to count as wall, for this map."""
    if not walls:
        return WALL_MIN_HITS
    counts = sorted(walls.values())
    strong = counts[min(len(counts) - 1, int(len(counts) * WALL_STRONG_QUANTILE))]
    return max(WALL_MIN_HITS, WALL_KEEP_FRACTION * strong)


# Movement tolerated between the poses bracketing a scan. Past these the scan
# is dropped outright; below them scan_weight() grades it. They live here
# rather than in the runner because the weighting is what gives them meaning,
# and the runner already imports from this module.
MAX_MOVE_DURING_SCAN_M = 0.12
# Measured on the 2026-08-24 run (481 scans with a pose recorded on each side of
# the scan window): median turn during a scan 1.65 deg, p90 12.6 deg, max 48.7.
# The old 25 deg let a scan through at weight 0.86 that had already smeared a
# point 1 m away by 47 cm -- far more than the 12.5 cm the box over-renders by,
# so the fade was doing nothing. At 5 deg the weighted mean smear at 1 m drops
# from 50 mm to 17 mm while 353 of 481 scans still contribute.
# Tightening MAX_MOVE_DURING_SCAN_M alongside it makes the map *worse* (20 mm):
# `min()` means a tight move limit discards scans the turn limit had kept.
MAX_TURN_DURING_SCAN_DEG = 5.0

# What the robot actually cleans is 319 mm wide -- its own width, not the
# 280 mm of the main brush: the side brush exists precisely to sweep the strip
# beyond the main brush into its path. Measured on Philou's D6.
CLEAN_WIDTH_M = _CLEAN_WIDTH_M
CLEAN_HALF_M = CLEAN_WIDTH_M / 2
# Beyond this, two consecutive poses cannot be joined by a straight swath: the
# robot had time to turn, and painting the chord would invent cleaned floor.
MAX_SWATH_M = 1.5
# Free-space evidence. Where the robot's *centre* went there can be nothing, so
# a narrow band around the path is proof a cell is empty -- unlike the 319 mm
# swath, which laps onto any wall the robot follows. Wall counts on those cells
# are reduced rather than cleared: a wall seen over many cleanings shrugs off one
# stray crossing, while a chair seen once is gone. Applied at most once per cell
# per cleaning, so a run that crosses the same spot repeatedly cannot compound
# its way through a wall. How much is taken off is CARVE_STEP below.
CARVE_HALF_M = 0.05
# Taken off a cell's count each cleaning that sees through it, rather than a
# fraction of it.
#
# It used to be `count * 0.5`, and that is the shape of the bug Philou caught on
# 2026-08-28: **building a wall is linear (+n per cleaning) while erasing it was
# geometric (halved per cleaning), so anything under-observed lost the race by
# construction.** A cell holding a hundred hits banked over twelve cleanings
# fell below the floor in five runs that merely grazed it. Measured on the live
# map: 99% of its cells carried a fractional count, meaning nearly every cell
# had already been halved at least once. Erosion was the normal regime, not the
# exception.
#
# Subtracting makes forgetting symmetric with learning, and 8 keeps the response
# to real change: from the cap below down to the draw threshold is six cleanings,
# against four under the old halving. Slower to forget a chair, far slower to
# eat a wall.
CARVE_STEP = 8.0
# Where the top of the map's distribution is pinned. wall_threshold() returns
# `WALL_KEEP_FRACTION * p95`, so holding p95 still is what holds the bar still,
# and a bar that drifts is what rots the plan run after run.
#
# This used to be a per-cell ceiling -- `min(count + n, 60)` -- and that fixed
# the threshold drifting *up*, which was eroding the map. It bought the erosion
# fix and introduced the thickening Philou reported on 2026-08-31: **a ceiling
# stops the core of a wall growing while the fringe beside it keeps climbing**,
# so the ratio between "seen every cleaning" and "seen twice" closes a little
# more each run until the fringe clears the bar too.
#
# Measured, and it is not subtle. Merging one session with *itself* seven times
# -- identical evidence, a perfect alignment, nothing new learned -- took the
# drawn plan from 2158 cells to 3735 and the walls from 2 cells thick to 3.
# Over seven real cleanings the same inflation ran 2088 -> 4069.
#
# Rescaling the whole grid instead keeps the bar still *and* keeps the ratio,
# because multiplying every cell by the same number changes neither. Replayed
# over the same seven cleanings the plan settles at ~2350 cells and **2 cells
# thick, p90 3** -- as thin as a single cleaning draws it, which is the most the
# data can support. Two things came with it, unasked:
#
#   * the stored grid stops growing (8252 -> 4804 cells), and
#   * the merge overlap *rises* run after run -- 0.85, 0.81, 0.75, 0.72, 0.71
#     under the ceiling, decaying towards the 0.55 that discards the map, against
#     0.85, 0.81, 0.91, 0.88, 0.93, 0.97 rescaled. A cleaner map is easier to
#     fit a cleaning onto, so this moves the map away from the cliff rather
#     than towards it.
#
# The fixed point is worth stating plainly, because it is the property the whole
# rule exists for: a cell settles at `WALL_REFERENCE x (its weight per cleaning)
# / (the p95 cell's weight per cleaning)`. Evidence per opportunity, not evidence
# banked -- which is what a wall is and a passer-by is not.
#
# 60 rather than more for the same reason the ceiling was 60: the rescaling
# fades an unseen cell by roughly a quarter per cleaning, so a departed chair is
# under the bar in about six.
WALL_REFERENCE = 60.0
# Below this a cell is arithmetic dust: it is three and a half fadings under the
# draw threshold and nothing but a fresh sighting can bring it back, which would
# overwrite it anyway. Dropping it is what stops the stored grid -- and the
# .storage file, a megabyte and climbing -- growing forever.
WALL_DUST = 1.5
# Free space read off the beams themselves -- see scan_free_cells().
#
# The guard keeps a beam from rubbing out the surface it just found, and it is
# deliberately one cell and no more. The things worth forgetting are things
# left against a wall, which by definition stand a few centimetres off its
# face: on Philou's map the planks' echo sits 2.5 to 7.5 cm above the wall's
# real face, so a 5 cm guard would shelter the exact band it needs to clear.
# One cell absorbs the quantisation of a single beam, and the real protection
# is elsewhere -- every cell any beam of this run landed on is exempt, and a
# wall face is landed on hundreds of times per run.
FREE_ENDPOINT_GUARD_M = CELL_M
# How many separate scans must see through a cell before its wall count is
# reduced. One stray beam from a bad pose crosses a wall once or twice; real
# open floor is crossed by dozens, so this costs nothing and rules the strays
# out. A cell this run also saw as wall is exempt whatever the count.
FREE_MIN_SCANS = 3
ROBOT_RADIUS_M = CLEAN_HALF_M   # kept for callers that want a footprint radius

# The turret sits at the BACK of the robot while the wheels are on a central
# axle, so the LIDAR is not on the centre of rotation and a scan does not
# originate at the reported pose. Measured 2026-08-24 by turning 178 deg in
# place and fitting the range change against bearing: the range shift is a
# sinusoid of amplitude 205 mm, so the lever arm is half that. The pose is at
# the centre of rotation, not the turret -- x,y held at -0.000/-0.002 through a
# 131 deg in-place turn.
#
# Left uncorrected, every return is displaced by this much along the heading, so
# the same wall is stamped up to 2 x 103 = 206 mm apart depending on which way
# the robot faced. On the calibration box that is most of the error: with the
# 5 deg fade alone it renders 85 x 55 cm, and with this offset as well, 65 x 50.
LIDAR_BEHIND_M = 0.103
# Requested render scale. Both the calibration and the drawing snap it to a
# whole number of pixels per cell -- see plan_step() -- so this is a wish, not
# a guarantee, and 120 is the wish that lands exactly on 3 px at a 2.5 cm grid.
#
# It used to be 100, and that was silently right only while a cell was 5 cm:
# 0.05 x 100 = 5.0 px, so cells laid out in world coordinates happened to
# abut. Halving the grid made a cell 2.5 px, drawn 2 px wide while the next
# one started 2.5 px along -- three of every five boundaries lost a pixel, and
# a wall 82 cells long came out as dashes of at most three. The plan looked
# riddled with holes that were not in the data at all.
RENDER_PX_PER_M = 120
RENDER_PAD_M = 0.3


def plan_step(px_per_m: float) -> int:
    """Whole pixels per cell at this scale -- the plan's unit of layout.

    Everything the plan draws is placed as a multiple of this, from cell
    indices, so cells abut by construction at any scale instead of by the
    arithmetic happening to come out even.
    """
    return max(1, round(CELL_M * px_per_m))

# The map carries exactly three states, and the palette says so plainly:
# black is wall, blue is cleaned floor, and whatever is neither — the gaps in
# the card's lattice and everything outside the plan — is the empty white the
# card paints underneath. Nothing here is a tint or a blend; keep it that way,
# because the whole point is that a glance tells you which of the three a
# square is.
WALL_RGBA = (0, 0, 0, 255)            # black — wall

# Wall straightening.
#
# A cleaning is one pass, and the robot's own pose drifts over it, so a wall
# arrives as a band of cells 15-20 cm wide rather than a line. Drawing the
# cells raw gives the thick blobby outline the map used to have. Fitting
# axis-parallel runs in the straightened frame and collapsing each band to a
# single line is what produces a plan that reads like a drawn one.

# Cap the accumulated grid so a runaway sensor cannot grow it without bound.
MAX_GRID_CELLS = 200_000

# How much of a new session's walls must coincide with the stored map before
# it is trusted enough to merge. Measured separation on real data: an
# identical run scores 99%, a run rotated a quarter turn and shifted still
# scores 95%, while a shape that is not this home (a filled disc) reaches
# only 35%. Anything below this is far more likely to be a bad fit than a
# genuine discovery, and merging it would corrupt the accumulated map.
MERGE_MIN_OVERLAP = 0.55
# Placing the run *in progress* is judged on the margin instead, because the
# overlap of a partial session is meaningless against a threshold tuned for a
# whole one. What is being decided is narrower, too: not "is this session good
# enough to go into the map" but "which of four quarter turns is it in".
#
# Measured by replaying runs 8 and 9 of 2026-08-27 scan by scan against the map
# as it stood before each: from **25 scans** -- about 100 s of cleaning -- the
# right quarter already led by 0.25 to 0.51 and by 2.2 to 4.0 times, and never
# changed for the rest of the run. The thresholds sit below the worst of those
# and far above a coin toss. Both are kept because they fail differently: the
# ratio catches a run whose scores are all low, the margin a symmetric home
# where two quarters both fit well.
LIVE_MIN_MARGIN = 0.15
LIVE_MIN_RATIO = 1.8
# After this many cleanings in a row that will not fit the stored map, it is the
# map that is wrong, not the house: the dock has been moved somewhere the fine
# sweep cannot reach, or the furniture has changed beyond recognition. Without a
# way out the map freezes for good, because every later session is compared
# against the same stale reference and refused in turn -- silently, since a
# rejection only reaches the log. Three is a compromise: fewer and one odd run
# could throw away a good map, more and a real move leaves it stuck for weeks.
MAX_CONSECUTIVE_REJECTS = 3
# Fine angle sweep around the winning quarter turn. +-8 deg covers a dock the
# robot has shoved out of true; 0.5 deg leaves 3.5 cm of residual error 4 m out,
# below what a 5 cm grid can express anyway.
FINE_SPAN_DEG = 8.0
FINE_STEP_DEG = 0.5
FINE_SEARCH_CELLS = 4
# A tilt has to *earn* its place. Two cleanings never cover quite the same
# ground, so the overlap they can reach is capped by that difference rather
# than by any misalignment, and inside that noise a fraction of a degree can
# look like an improvement. Turning a session that did not need turning is how
# a run ends up drawn a couple of degrees off the walls -- seen once, with the
# path crossing 41 wall cells at the angle the sweep chose and none at all
# unturned. A real misalignment is not subtle: the measured corrections ran
# from a 1.24x gain to nearly 4x, so asking for 2% keeps every one of them and
# rejects the noise.
FINE_MIN_GAIN = 0.02


# ── geometry ────────────────────────────────────────────────────────


def project_scan(
    x: float, y: float, theta_deg: float, points: Iterable[tuple[int, int]]
) -> list[tuple[int, int]]:
    """Return the grid cells hit by one scan, in the robot's world frame.

    `points` is (angle_deg, dist_mm). The heading is folded in with the
    angle-addition identity so the inner loop carries no trigonometry.
    """
    tr = math.radians(theta_deg)
    ct, st = math.cos(tr), math.sin(tr)
    # Scans leave from the turret, which trails the pose along the heading.
    ox = x - LIDAR_BEHIND_M * ct
    oy = y - LIDAR_BEHIND_M * st
    inv = 1.0 / CELL_M
    out: list[tuple[int, int]] = []
    for angle, dist_mm in points:
        if not 0 < dist_mm <= MAX_RANGE_M * 1000:
            continue
        d = dist_mm / 1000.0
        ar = math.radians(angle)
        c, s = math.cos(ar), math.sin(ar)
        wx = ox + d * (c * ct - s * st)
        wy = oy + d * (s * ct + c * st)
        out.append((math.floor(wx * inv), math.floor(wy * inv)))
    return out


def scan_free_cells(
    x: float, y: float, theta_deg: float, points: Iterable[tuple[int, int]]
) -> set[tuple[int, int]]:
    """Cells this scan proves are empty: the ones its beams passed through.

    `project_scan` keeps only where each beam *stopped*. Everything it crossed
    on the way is evidence just as strong and was being thrown away, and that
    is why the map had no working memory of removal. The only other free-space
    evidence was the 10 cm band around the robot's centre -- but the centre
    never comes within 16 cm of a surface, so an 11 cm ring around every wall
    could never be revisited. Measured on Philou's map: that band reached
    7 wall cells out of 4587. Anything parked against a wall and later taken
    away stayed drawn for good, and a wall with planks against it kept the
    planks' face instead of its own.

    A beam stops 5 cm short here. Its endpoint is a surface found to within
    the pose error, and without the guard a scan would rub out the very wall
    it just measured.

    A return past MAX_RANGE_M still proves the near 3.5 m empty even though
    `project_scan` discards it as too grazing to place, so it is cast too. A
    *missing* return proves nothing -- dark and glancing surfaces read as
    zero -- and is skipped.
    """
    tr = math.radians(theta_deg)
    ct, st = math.cos(tr), math.sin(tr)
    ox = x - LIDAR_BEHIND_M * ct
    oy = y - LIDAR_BEHIND_M * st
    inv = 1.0 / CELL_M
    out: set[tuple[int, int]] = set()
    for angle, dist_mm in points:
        if dist_mm <= 0:
            continue
        d = min(dist_mm / 1000.0 - FREE_ENDPOINT_GUARD_M, MAX_RANGE_M)
        if d <= 0:
            continue
        ar = math.radians(angle)
        c, s = math.cos(ar), math.sin(ar)
        ux = c * ct - s * st
        uy = s * ct + c * st
        for k in range(int(d / CELL_M)):
            t = k * CELL_M
            out.add((math.floor((ox + t * ux) * inv), math.floor((oy + t * uy) * inv)))
    return out


def stamp_swath(
    x0: float, y0: float, x1: float, y1: float, half: float | None = None
) -> list[tuple[int, int]]:
    """Grid cells swept between two consecutive poses.

    A disc stamped at each pose leaves gaps: poses are 178 mm apart in the
    median but 294 mm at the 90th percentile and up to 1.2 m, so any disc
    narrow enough to be honest about the cleaned width is too narrow to join
    them. Measured on a real run, a 280 mm disc left a gap on 16% of intervals
    while the shipped 450 mm one still left 4%.

    Painting the band the robot swept between the two poses removes that
    trade-off entirely: it is continuous whatever the sampling interval, which
    is also what makes the health throttle harmless.
    """
    inv = 1.0 / CELL_M
    if math.hypot(x1 - x0, y1 - y0) > MAX_SWATH_M:
        # Too far to join honestly; paint the endpoint only.
        x0, y0 = x1, y1
    if half is None:
        half = CLEAN_HALF_M
    lo_i = math.floor((min(x0, x1) - half) * inv)
    hi_i = math.ceil((max(x0, x1) + half) * inv)
    lo_j = math.floor((min(y0, y1) - half) * inv)
    hi_j = math.ceil((max(y0, y1) + half) * inv)
    dx, dy = x1 - x0, y1 - y0
    length2 = dx * dx + dy * dy
    out: list[tuple[int, int]] = []
    for i in range(lo_i, hi_i + 1):
        px = (i + 0.5) * CELL_M
        for j in range(lo_j, hi_j + 1):
            py = (j + 0.5) * CELL_M
            if length2 <= 0.0:
                dist = math.hypot(px - x0, py - y0)
            else:
                t = ((px - x0) * dx + (py - y0) * dy) / length2
                t = min(max(t, 0.0), 1.0)
                dist = math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
            if dist <= half:
                out.append((i, j))
    return out


def stamp_floor(x: float, y: float) -> list[tuple[int, int]]:
    """Grid cells covered by the robot at a single pose."""
    return stamp_swath(x, y, x, y)


def carve_swath(
    x0: float, y0: float, x1: float, y1: float
) -> list[tuple[int, int]]:
    """Cells the robot's centre passed through: proof they are empty."""
    return stamp_swath(x0, y0, x1, y1, half=CARVE_HALF_M)


def _rotate_cells(cells: Iterable[tuple[int, int]], quarter: int) -> list[tuple[int, int]]:
    """Rotate integer cell coordinates by a multiple of 90 degrees."""
    q = quarter % 4
    if q == 0:
        return list(cells)
    if q == 1:
        return [(-cy, cx) for cx, cy in cells]
    if q == 2:
        return [(-cx, -cy) for cx, cy in cells]
    return [(cy, -cx) for cx, cy in cells]


def _rotate_cells_fine(
    cells: Iterable[tuple[int, int]], degrees: float
) -> list[tuple[int, int]]:
    """Rotate cells by an arbitrary angle about the grid origin."""
    if not degrees:
        return list(cells)
    rad = math.radians(degrees)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    return [
        (round(cx * cos_a - cy * sin_a), round(cx * sin_a + cy * cos_a))
        for cx, cy in cells
    ]


def align_to_reference(
    new_walls: dict[tuple[int, int], int],
    ref_walls: dict[tuple[int, int], int],
    search_cells: int = 8,
) -> tuple[int, int, int, float, float, tuple[float, float, float, float]]:
    """Fit a new session's walls onto the accumulated map.

    Tries the four quarter turns, each with a small translation search, then
    refines the angle, and returns
    (quarter, dx, dy, overlap, fine_deg, quarter_scores).
    Overlap is the share of the new session's wall cells that coincide with the
    reference, so 1.0 is perfect.

    `quarter_scores` is what each of the four quarters could reach on the
    coarse search, kept because it answers a different question from the
    overlap: the overlap says how well the session matches the map, the spread
    between the quarters says how sure we are it is *that* quarter and not
    another. The two come apart on a partial session -- a run a quarter of the
    way through overlaps only 0.48-0.67 of the map, well under
    MERGE_MIN_OVERLAP, while still beating the runner-up quarter by 2.2 to 4.0
    times. That is what lets the run in progress be placed on the map before
    there is enough of it to merge. See quarter_margin().

    A quarter turn is not enough on its own. The dock is never square with the
    wall and the robot nudges it while cleaning, so each session also starts a
    few degrees off -- a continuous error, not a right angle. Measured on a real
    map, the quarter-only search accepted 1 deg at 0.70 overlap (merging 7 cm of
    error at 4 m into the map for good) and *rejected* everything past about
    1.5 deg, which freezes the map: every later session compares against the
    same stale reference and is refused in turn.

    So the quarter search is followed by a fine sweep of +-FINE_SPAN_DEG around
    it. It stays a refinement, never a free rotation hunt -- the original worry
    about false matches was about searching all angles, and that is still not
    what happens.
    """
    if not ref_walls or not new_walls:
        return 0, 0, 0, 0.0, 0.0, (0.0, 0.0, 0.0, 0.0)

    ref = set(ref_walls)
    fcx = sum(c[0] for c in ref) / len(ref)
    fcy = sum(c[1] for c in ref) / len(ref)
    best = (0, 0, 0, -1.0)
    # The best each quarter could do, kept so the winner can be compared with
    # the field rather than only with a threshold.
    per_quarter = [0.0, 0.0, 0.0, 0.0]
    for quarter in range(4):
        rotated = _rotate_cells(new_walls, quarter)
        rcx = sum(c[0] for c in rotated) / len(rotated)
        rcy = sum(c[1] for c in rotated) / len(rotated)
        # Two seeds, because either can be the wrong guess:
        #   (0, 0)   the dock has not moved, which is the usual case;
        #   centroid the dock has moved, or the frame origin shifted.
        # Centroid alone fails when one map is a small off-centre piece of the
        # other, since their centres of mass are then nowhere near each other.
        seeds = {(0, 0), (round(fcx - rcx), round(fcy - rcy))}
        for sx, sy in seeds:
            for dx in range(sx - search_cells, sx + search_cells + 1):
                for dy in range(sy - search_cells, sy + search_cells + 1):
                    hit = 0
                    for cx, cy in rotated:
                        if (cx + dx, cy + dy) in ref:
                            hit += 1
                    # Score against the smaller of the two maps. Dividing by
                    # the new session would punish it for covering ground the
                    # stored map has never seen -- a short run recorded first
                    # would then reject every full clean that followed, and
                    # the map could never grow past what it first happened
                    # to see.
                    score = hit / min(len(rotated), len(ref))
                    per_quarter[quarter] = max(per_quarter[quarter], score)
                    if score > best[3]:
                        best = (quarter, dx, dy, score)

    # Handed back as the coarse search left them, before the fine sweep: only
    # the winner gets refined, and comparing a refined score against unrefined
    # ones would read as confidence the search never established.
    scores = (per_quarter[0], per_quarter[1], per_quarter[2], per_quarter[3])

    # Refine the angle around the winning quarter. The translation is searched
    # again but narrowly: rotating about the grid origin shifts a distant map
    # bodily, and that shift has to be absorbed rather than counted as error.
    quarter, dx, dy, score = best
    base = _rotate_cells(new_walls, quarter)
    best_fine = 0.0
    # Beat the untilted fit by a clear margin, not by a rounding error.
    floor_score = score * (1.0 + FINE_MIN_GAIN)
    steps = int(FINE_SPAN_DEG / FINE_STEP_DEG)
    for step in range(-steps, steps + 1):
        fine = step * FINE_STEP_DEG
        if not fine:
            continue
        turned = _rotate_cells_fine(base, fine)
        tcx = sum(c[0] for c in turned) / len(turned)
        tcy = sum(c[1] for c in turned) / len(turned)
        seeds = {(dx, dy), (round(fcx - tcx), round(fcy - tcy))}
        for sx, sy in seeds:
            for ddx in range(sx - FINE_SEARCH_CELLS, sx + FINE_SEARCH_CELLS + 1):
                for ddy in range(sy - FINE_SEARCH_CELLS, sy + FINE_SEARCH_CELLS + 1):
                    hit = 0
                    for cx, cy in turned:
                        if (cx + ddx, cy + ddy) in ref:
                            hit += 1
                    value = hit / min(len(turned), len(ref))
                    if value > max(score, floor_score):
                        score, dx, dy, best_fine = value, ddx, ddy, fine
    return quarter, dx, dy, score, best_fine, scores


def quarter_margin(
    scores: tuple[float, float, float, float], quarter: int
) -> tuple[float, float]:
    """By how much the chosen quarter turn beat the best of the other three.

    Returns (difference, ratio). Both, because they fail differently: the ratio
    catches a fit whose scores are all low, the difference a symmetric home
    where two quarters both match well. An unbeaten quarter -- the others at
    zero -- comes back with an infinite ratio, which is the honest answer.
    """
    others = max(s for q, s in enumerate(scores) if q != quarter)
    if others <= 0.0:
        return scores[quarter], float("inf")
    return scores[quarter] - others, scores[quarter] / others


def manhattan_angle(cells: Iterable[tuple[int, int]]) -> float:
    """Angle, in degrees, between the walls and the axes.

    Assumes a broadly rectangular home: the right angle is the one that makes
    wall cells share an x or a y coordinate as much as possible, because a
    wall collapses into a single histogram bin only when axis-parallel.
    Returns a value in (-45, 45].
    """
    pts = [(cx * CELL_M, cy * CELL_M) for cx, cy in cells]
    if len(pts) < 20:
        return 0.0

    def peakiness(deg: float) -> float:
        r = math.radians(deg)
        c, s = math.cos(r), math.sin(r)
        hx: dict[int, int] = {}
        hy: dict[int, int] = {}
        inv = 1.0 / CELL_M
        for x, y in pts:
            bx = round((x * c - y * s) * inv)
            by = round((x * s + y * c) * inv)
            hx[bx] = hx.get(bx, 0) + 1
            hy[by] = hy.get(by, 0) + 1
        return sum(n * n for n in hx.values()) + sum(n * n for n in hy.values())

    coarse = max(range(90), key=lambda d: peakiness(float(d)))
    best = max(
        (coarse + i * 0.05 for i in range(-20, 21)),
        key=peakiness,
    )
    # A rectangle repeats every quarter turn; report the smallest correction.
    return best - 90.0 if best > 45.0 else best


# ── rendering ───────────────────────────────────────────────────────






NOTICE_RGBA = (210, 120, 120, 90)   # faint enough to read the plan through


def _draw_notice(img: Image.Image, lines: list[str]) -> None:
    """Write a watermark across the plan.

    Pillow's default font is a small bitmap and its scalable loader is not old
    enough to rely on here, so each line is drawn at native size and enlarged.
    Nearest-neighbour on purpose: the card rescales this image again, and a
    smoothed enlargement turns to mush when it does.
    """
    font = ImageFont.load_default()
    scratch = ImageDraw.Draw(Image.new("RGBA", (1, 1)))
    target = img.width * 0.78
    sized = []
    for text in lines:
        left, top, right, bottom = scratch.textbbox((0, 0), text, font=font)
        w, h = max(1, right - left), max(1, bottom - top)
        tile = Image.new("RGBA", (w, h), (0, 0, 0, 0))
        ImageDraw.Draw(tile).text((-left, -top), text, font=font, fill=NOTICE_RGBA)
        scale = max(1, int(target / w))
        sized.append(tile.resize((w * scale, h * scale), Image.NEAREST))
    gap = max(4, img.height // 60)
    total = sum(t.height for t in sized) + gap * (len(sized) - 1)
    y = (img.height - total) // 2
    for tile in sized:
        img.alpha_composite(tile, ((img.width - tile.width) // 2, max(0, y)))
        y += tile.height + gap


def render_plan(
    walls: dict[tuple[int, int], int],
    floor: set[tuple[int, int]],
    px_per_m: int = RENDER_PX_PER_M,
    notice: list[str] | None = None,
) -> tuple[bytes, dict[str, float]] | None:
    """Draw the plan in the robot's frame; return (png, calibration).

    Walls are solid, the traversed interior gets a faint tint, and everything
    else stays transparent so the card's coverage and path read on top. The
    calibration is exact rather than fitted: the image is drawn in world
    coordinates, so its bottom-left corner *is* the origin.
    """
    cal = plan_calibration(walls, floor, px_per_m)
    if cal is None:
        return None
    # Hoisted deliberately. wall_threshold() sorts every count, so calling it
    # from inside the comprehension runs one sort per cell -- O(n^2 log n),
    # which on a 13k-cell map took 131 seconds and pinned the executor thread
    # hard enough to make Home Assistant look like it was restarting.
    threshold = wall_threshold(walls)
    wall_cells = {c for c, n in walls.items() if n >= threshold}
    width, height = cal["width"], cal["height"]
    min_cx, max_cy = cal["min_cx"], cal["max_cy"]

    img = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    step = plan_step(px_per_m)

    def box(cx: int, cy: int):
        # World Y grows up, image rows grow down.
        #
        # Placed from the cell's own index, times a whole number of pixels, so
        # neighbouring cells abut whatever the scale. Deriving the corner from
        # metres instead is what dashed the plan: at 2.5 cm and 100 px/m a cell
        # is 2.5 px, drawn 2 px wide while the next one starts 2.5 px along, so
        # three boundaries in five lost a pixel and every wall came out
        # perforated. PIL's rectangle includes both ends, hence the -1.
        x0 = (cx - min_cx) * step
        y0 = (max_cy - cy - 1) * step
        return [x0, y0, x0 + step - 1, y0 + step - 1]

    # Walls only — the floor is deliberately left out.
    #
    # The card fills the floor itself, square by square, as the replay runs:
    # a square is empty until the robot has been over it, and then it is blue.
    # Painting the accumulated floor here would blue the whole map in before
    # the replay even starts and there would be nothing left to watch.
    #
    # Walls as solid cells.
    #
    # The lattice of small squares the plan is meant to read as is NOT applied
    # here: the card rescales this image by a fractional factor and turns it a
    # couple of degrees to straighten it against the walls, and either of
    # those resamples a fine lattice into ragged clumps. The card screens the
    # lattice on afterwards, in whole device pixels. Keep these cells solid.
    #
    # Fitting straight segments instead was built and dropped: it draws a
    # tidier plan on paper, but one pass of LIDAR has nowhere near the
    # coverage to close a rectilinear outline, so what came out read as
    # scattered strokes.
    for cx, cy in wall_cells:
        draw.rectangle(box(cx, cy), fill=WALL_RGBA)

    if notice:
        _draw_notice(img, notice)

    buf = io.BytesIO()
    img.save(buf, format="PNG", optimize=True)
    return buf.getvalue(), cal


def plan_calibration(
    walls: dict[tuple[int, int], int],
    floor: set[tuple[int, int]],
    px_per_m: int = RENDER_PX_PER_M,
) -> dict[str, float] | None:
    """World-to-pixel mapping the rendered plan will use.

    Split out from the drawing so the card can be told where the plan sits
    without paying for a render. Exact rather than fitted: the image is laid
    out in world coordinates, so its bottom-left corner *is* the origin.
    """
    threshold = wall_threshold(walls)  # hoisted: one sort, not one per cell
    wall_cells = {c for c, n in walls.items() if n >= threshold}
    if not wall_cells:
        return None

    # In cells, not metres. The image is a whole number of cells across and
    # each one a whole number of pixels, so the frame cannot land half a cell
    # off the grid the card reads it back on.
    cells = wall_cells | floor
    pad = max(1, round(RENDER_PAD_M / CELL_M))
    min_cx = min(c[0] for c in cells) - pad
    max_cx = max(c[0] for c in cells) + 1 + pad
    min_cy = min(c[1] for c in cells) - pad
    max_cy = max(c[1] for c in cells) + 1 + pad

    step = plan_step(px_per_m)
    width = max(1, (max_cx - min_cx) * step)
    height = max(1, (max_cy - min_cy) * step)
    if width > 4096 or height > 4096:
        _LOGGER.warning("Generated plan too large (%dx%d); skipping", width, height)
        return None
    return {
        # The scale the image was actually drawn at, which is the snapped one:
        # reporting the requested scale would put the card's cell-by-cell read
        # a fraction of a pixel out per cell and drift it across the plan.
        "scale": step / CELL_M,
        "origin_x": round(min_cx * CELL_M, 3),
        "origin_y": round(min_cy * CELL_M, 3),
        "width": width,
        "height": height,
        "wall_cells": len(wall_cells),
        "floor_cells": len(floor),
        # Cell index of the frame's left and top edges, so the drawing places
        # every cell from its own index instead of re-deriving it from metres.
        "min_cx": min_cx,
        "max_cy": max_cy,
    }


# ── accumulation across sessions ────────────────────────────────────


class AccumulatedMap:
    """Wall hit counts and traversed floor, persisted between cleanings."""

    def __init__(self, data: dict[str, Any] | None = None) -> None:
        data = data or {}
        self.walls: dict[tuple[int, int], int] = {
            _key(k): v for k, v in (data.get("walls") or {}).items()
        }
        self.floor: set[tuple[int, int]] = {
            _key(k) for k in (data.get("floor") or [])
        }
        self.sessions: int = int(data.get("sessions", 0))
        self.rejects: int = int(data.get("rejects", 0))
        # Quarter turn that puts the reference map the right way up, kept so
        # the orientation cannot flip between renders.
        self.quarter_lock: int = int(data.get("quarter_lock", 0))
        # Correction applied to each merged session, keyed by its file name.
        #
        # A session arrives in whatever frame the robot's localisation happened
        # to be in; align_to_reference() fits it onto the accumulated map
        # before merging. That correction used to be computed, used and thrown
        # away -- so the map held straightened walls while the card was still
        # served the session's raw path and coverage. After a localisation
        # loss the two came out a quarter turn apart, which is exactly what a
        # user sees as "the cleaned area does not line up with the walls".
        # Keeping it lets the replay be served in the same frame as the map.
        self.alignments: dict[str, tuple[int, int, int]] = {
            k: tuple(v) for k, v in (data.get("alignments") or {}).items()
        }

    def as_dict(self) -> dict[str, Any]:
        return {
            "walls": {f"{cx},{cy}": n for (cx, cy), n in self.walls.items()},
            "floor": [f"{cx},{cy}" for cx, cy in self.floor],
            "sessions": self.sessions,
            "quarter_lock": self.quarter_lock,
            "rejects": self.rejects,
            "alignments": {k: list(v) for k, v in self.alignments.items()},
        }

    def merge_session(
        self,
        walls: dict[tuple[int, int], int],
        floor: set[tuple[int, int]],
        session_name: str | None = None,
        free: set[tuple[int, int]] | None = None,
        correction: tuple[float, ...] = (0.0, 0.0, 0.0),
        contribute: bool = True,
    ) -> dict[str, Any]:
        """Fold one cleaning into the accumulated map, re-aligning it first.

        `contribute=False` works out where the session sits and remembers it,
        but adds none of its geometry. A spot clean is the case: the robot
        drives itself so its track is trustworthy and worth replaying *on* the
        map, while a square metre of walls has nothing to teach a map built
        from whole-house runs. It must not count as a failure either -- a
        small session naturally overlaps poorly, and three failures in a row
        discard the map.
        """
        report: dict[str, Any] = {
            "session_walls": len(walls), "realigned": False, "contributed": contribute,
        }

        if self.walls:
            quarter, dx, dy, overlap, fine, scores = align_to_reference(
                walls, self.walls
            )
            # Le recouvrement dit si la session ressemble a la carte, la marge
            # si c'est bien ce quart-la et pas un autre. Une fusion refusee
            # avec une marge nette se lit autrement qu'une fusion refusee sur
            # quatre quarts a egalite, et sans ca les deux se ressemblent.
            margin, _ratio = quarter_margin(scores, quarter)
            report.update(
                quarter=quarter, dx=dx, dy=dy, fine=fine, overlap=round(overlap, 3),
                margin=round(margin, 3),
            )
            if overlap < MERGE_MIN_OVERLAP and not contribute:
                # Nothing of it was going into the map anyway, so a poor fit
                # costs nothing and must not be counted against the map. Keep
                # the best transform we found: an approximately placed replay
                # beats one drawn in the robot's own frame.
                report["poor_fit"] = True
                _LOGGER.info(
                    "LIDAR map: session overlaps %.0f%% of the map; placing it "
                    "there anyway and merging nothing", 100 * overlap,
                )
            elif overlap < MERGE_MIN_OVERLAP:
                self.rejects += 1
                report["rejects"] = self.rejects
                if self.rejects < MAX_CONSECUTIVE_REJECTS:
                    # Rather than corrupt a good map with a bad fit, keep what
                    # we have and say so.
                    report["rejected"] = True
                    _LOGGER.warning(
                        "LIDAR map: new session only overlaps %.0f%% of the "
                        "stored map; refusing to merge it (%d in a row, "
                        "starting over at %d)",
                        100 * overlap, self.rejects, MAX_CONSECUTIVE_REJECTS,
                    )
                    return report
                # Three in a row: the stored map is the thing that no longer
                # matches reality. Drop it and let this session be the new one.
                _LOGGER.warning(
                    "LIDAR map: %d sessions in a row would not fit (last one "
                    "%.0f%%); discarding the stored map and starting from this "
                    "cleaning", self.rejects, 100 * overlap,
                )
                report["reset"] = True
                self.walls = {}
                self.floor = set()
                self.alignments = {}
                self.sessions = 0
                self.quarter_lock = 0
                # The transform was fitted against the map just discarded, so
                # it means nothing now: this session becomes the reference in
                # its own frame, untouched.
                quarter = dx = dy = 0
                fine = 0.0
                report.update(quarter=0, dx=0, dy=0, fine=0.0)
            if quarter or dx or dy or fine:
                report["realigned"] = True
                _LOGGER.info(
                    "LIDAR map: session re-aligned by %.1f deg and (%d, %d) cells "
                    "before merging (%.0f%% overlap)",
                    quarter * 90 + fine, dx, dy, 100 * overlap,
                )
            turned = _rotate_cells_fine(_rotate_cells(walls, quarter), fine)
            walls = {
                (cx + dx, cy + dy): n for (cx, cy), n in zip(turned, walls.values())
            }
            floor = {
                (cx + dx, cy + dy)
                for cx, cy in _rotate_cells_fine(_rotate_cells(floor, quarter), fine)
            }
            if free:
                free = {
                    (cx + dx, cy + dy)
                    for cx, cy in _rotate_cells_fine(
                        _rotate_cells(free, quarter), fine
                    )
                }

        if contribute:
            self.rejects = 0
        if session_name:
            # The correction rides with the alignment because it is the other
            # half of the same journey: raw log -> corrected frame -> map.
            # In cells, like dx and dy, and applied before the rotation
            # because that is where scan matching applied it.
            #
            # Seven fields since 2026-08-31: the correction is a rigid
            # transform, and its rotation is the larger half of it. Six-field
            # alignments already stored mean cth = 0, which is what they were
            # replayed as anyway.
            cx = correction[0] / CELL_M
            cy = correction[1] / CELL_M
            cth = correction[2] if len(correction) > 2 else 0.0
            self.alignments[alignment_key(session_name)] = (
                (quarter, dx, dy, fine, cx, cy, cth)
                if self.walls
                else (0, 0, 0, 0.0, cx, cy, cth)
            )

        if not contribute:
            # Placed, remembered, and deliberately not merged.
            report["total_walls"] = len(self.walls)
            report["sessions"] = self.sessions
            return report

        for cell, n in walls.items():
            self.walls[cell] = self.walls.get(cell, 0) + n

        # Then let this run's free space push back on what earlier runs saw.
        # Anything the robot drove through is not there any more, and without
        # this the only way a removed chair leaves the map is by the adaptive
        # threshold slowly outgrowing it -- measured at eleven cleanings.
        if free:
            faded = weakened = 0
            for cell in free:
                previous = self.walls.get(cell)
                if previous is None:
                    continue
                reduced = previous - CARVE_STEP
                if reduced < WALL_MIN_HITS:
                    del self.walls[cell]
                    faded += 1
                else:
                    self.walls[cell] = reduced
                    weakened += 1
            report["carved"] = len(free)
            report["faded"] = faded
            # The ones still on the map but weaker are what a thing recently
            # taken away looks like on its way out; without this the log shows
            # nothing at all until the cell finally drops off.
            report["weakened"] = weakened

        self.floor |= floor
        self.sessions += 1
        report["rescaled"] = self._rescale()

        if len(self.walls) > MAX_GRID_CELLS:
            # Drop the weakest evidence first; real walls are seen repeatedly.
            keep = sorted(self.walls.items(), key=lambda kv: -kv[1])[:MAX_GRID_CELLS]
            self.walls = dict(keep)
            _LOGGER.warning("LIDAR map: grid capped at %d cells", MAX_GRID_CELLS)

        report["total_walls"] = len(self.walls)
        report["sessions"] = self.sessions
        return report

    def _rescale(self) -> float:
        """Pin the top of the distribution, so the draw threshold holds still.

        Returns the factor applied, 1.0 when nothing was needed.

        Multiplying every cell by the same number leaves every ratio between
        them untouched, which is the whole difference from the ceiling this
        replaces: it holds the bar still without also closing the gap between a
        wall and the fringe beside it. See WALL_REFERENCE for the measurements.
        """
        if not self.walls:
            return 1.0
        counts = sorted(self.walls.values())
        p95 = counts[min(len(counts) - 1, int(len(counts) * WALL_STRONG_QUANTILE))]
        if p95 <= WALL_REFERENCE:
            return 1.0
        scale = WALL_REFERENCE / p95
        self.walls = {
            cell: n * scale
            for cell, n in self.walls.items()
            if n * scale >= WALL_DUST
        }
        return scale

    def render_signature(self) -> str:
        """Identifier that changes whenever the drawn plan would change.

        The card's cache-buster used to be the session count alone, which no
        longer holds now that the wall threshold follows the map's own
        distribution: the rule can move -- or the code behind it can -- while
        the session count stands still, and browsers would keep serving the
        stale image. Pairing the count with the threshold and the number of
        cells that clear it makes the URL change exactly when the picture does.
        """
        threshold = wall_threshold(self.walls)
        kept = sum(1 for n in self.walls.values() if n >= threshold)
        return f"{self.sessions}.{threshold}.{kept}"

    def view_rotation(self, user_offset: float = 0.0) -> float:
        """Card rotation that stands the map upright, plus the user's offset.

        The plan itself stays in the robot's frame -- that is the frame the
        path and coverage live in -- so straightening is a property of the
        view, not of the image.
        """
        threshold = wall_threshold(self.walls)  # hoisted: one sort, not one per cell
        wall_cells = [c for c, n in self.walls.items() if n >= threshold]
        skew = manhattan_angle(wall_cells)
        return round((-skew) + self.quarter_lock * 90 + user_offset, 2) % 360


def alignment_key(name: str) -> str:
    """Session name with the compression suffix stripped.

    A session is `<epoch>.jsonl` while the robot is recording it and becomes
    `<epoch>.jsonl.hs` once the firmware compresses it, minutes later. The
    alignment is worked out at merge time, under the recording name, but every
    later request comes in under the compressed one -- so the lookup missed
    every time and the card drew each finished run unaligned, a quarter turn
    off the walls. Key on the part that does not change.
    """
    return name.removesuffix(".hs")


def _key(raw: str) -> tuple[int, int]:
    cx, _, cy = raw.partition(",")
    return int(cx), int(cy)


# ── runtime ─────────────────────────────────────────────────────────


def scan_weight(moved_m: float, turned_deg: float) -> float:
    """How much one scan's returns are worth, from how still the robot was.

    A scan is not instantaneous -- it takes 0.6 to 1.7 s while the robot keeps
    driving -- so the returns are smeared along whatever the robot did during
    it. Rotation dominates, because the smear is the *arc*: 3.5 deg of turn
    displaces a wall point 2 m away by 12 cm, while 3.5 cm of travel displaces
    it by 3.5 cm.

    Measured on a real object with known dimensions: a 50 x 29 cm box came out
    75 x 55 cm on the map, a uniform ~12.5 cm margin on every side. The margin
    did not grow with distance from the map centre (density 20.4 at 0-1 m
    against 19.5 at 3-4 m), which rules out a rotation error *between*
    sessions and points at motion *within* each scan.

    The old filter was binary: keep everything under 12 cm / 25 deg, drop the
    rest. So a scan taken mid-turn counted exactly as much as one taken
    standing still, and since typical rotation sits far below 25 deg almost
    nothing was ever rejected. This grades it instead -- still 1.0 for a
    motionless scan, falling linearly to 0 at the limits, so the accumulated
    map stays on the same scale as everything merged before it.
    """
    if MAX_MOVE_DURING_SCAN_M <= 0 or MAX_TURN_DURING_SCAN_DEG <= 0:
        return 1.0
    move_w = 1.0 - min(1.0, abs(moved_m) / MAX_MOVE_DURING_SCAN_M)
    turn_w = 1.0 - min(1.0, abs(turned_deg) / MAX_TURN_DURING_SCAN_DEG)
    # The worse of the two, not the product: a scan ruined by rotation is not
    # rescued by having barely translated.
    return min(move_w, turn_w)


# -- Scan matching ------------------------------------------------------------
# Odometry drifts, and nothing used to correct it *inside* a session:
# align_to_reference only fits whole sessions to each other. Over a 58 minute
# run the drift is what smears the map -- measured against a 50 x 29 cm box,
# correcting each scan against the map built so far takes it from 65 x 50 cm to
# 60 x 35 (5 cm cells), and to 54 x 34 on a 2 cm grid.
#
# The search is deliberately small and local: this is a nudge onto an existing
# map, not a relocalisation. A run that needs more than MATCH_MAX_DRIFT_M of
# accumulated correction has gone wrong somewhere the matcher cannot fix, so it
# gives up and lets the raw odometry through rather than inventing a pose.
MATCH_SEED = 8               # scans trusted as-is, so there is a map to match against
MATCH_CAP = 6.0              # per-cell score cap: one dense wall must not dominate
MATCH_STRIDE = 2             # every other return is plenty for scoring
MATCH_MAX_DRIFT_M = 0.60     # give up past this much accumulated correction
# (linear step m, steps each way, angular step deg, steps each way)
MATCH_PASSES = ((0.10, 2, 3.0, 2), (0.025, 2, 0.75, 2))


def _match_score(
    walls: dict[tuple[int, int], float],
    x: float,
    y: float,
    theta: float,
    points: list[tuple[int, int]],
) -> float:
    """How well a scan placed at this pose agrees with the map so far."""
    return sum(min(walls.get(cell, 0.0), MATCH_CAP)
               for cell in project_scan(x, y, theta, points))


def match_pose(
    walls: dict[tuple[int, int], float],
    x: float,
    y: float,
    theta: float,
    points: list[tuple[int, int]],
) -> tuple[float, float, float]:
    """Nudge a pose so its scan lands on the map already built.

    Coarse pass then fine pass around the winner, which costs a fraction of a
    single flat search over the same span.
    """
    sparse = points[::MATCH_STRIDE]
    bx, by, bt = x, y, theta
    for step, span, astep, aspan in MATCH_PASSES:
        best_score: float | None = None
        cx, cy, ct = bx, by, bt
        for i in range(-span, span + 1):
            for j in range(-span, span + 1):
                for k in range(-aspan, aspan + 1):
                    px, py, pt = bx + i * step, by + j * step, bt + k * astep
                    score = _match_score(walls, px, py, pt, sparse)
                    if best_score is None or score > best_score:
                        best_score, cx, cy, ct = score, px, py, pt
        bx, by, bt = cx, cy, ct
    return bx, by, bt


def fit_rigid(
    raw: list[tuple[float, float]], fixed: list[tuple[float, float]]
) -> tuple[float, float, float]:
    """Rotation and shift that best carry the raw poses onto the corrected ones.

    Returns (tx, ty, degrees) meaning `p_corrected ~= R(degrees) . p_raw + t`,
    with the rotation about the frame origin so it composes with the quarter
    and fine turns the merge already applies.

    Why a rotation and not just a shift, which is what used to be handed over:
    scan matching applies a *running* correction and the pose graph then moves
    every pose again, and the sum of those is overwhelmingly a rotation of the
    whole run. Averaging it into one translation cannot express that, and the
    error it leaves is not small. Measured over seven cleanings, the median
    distance between where the map places a scan and where the replay draws it:

    | run | mean shift (today) | rigid fit |
    |-----|--------------------|-----------|
    |   6 | 100 mm, 50% > 10 cm |  33 mm, 2.3% |
    |   9 |  45 mm,  0.6%       |  29 mm, 0.2% |
    |  11 |  62 mm,  3.6%       |  22 mm, 0.7% |
    |  14 | 124 mm, 63%         |  33 mm, 3.7% |

    Sixty-four interpolated knots -- 128 numbers instead of three -- reach
    35 mm on run 14, no better than the rigid block. The deformation genuinely
    is a rigid one, so three numbers is not an approximation of the answer, it
    is the answer.
    """
    n = len(raw)
    if n < 2 or n != len(fixed):
        return (0.0, 0.0, 0.0)
    rcx = sum(p[0] for p in raw) / n
    rcy = sum(p[1] for p in raw) / n
    fcx = sum(p[0] for p in fixed) / n
    fcy = sum(p[1] for p in fixed) / n
    sxx = sxy = 0.0
    for (rx, ry), (fx, fy) in zip(raw, fixed):
        ax, ay = rx - rcx, ry - rcy
        bx, by = fx - fcx, fy - fcy
        sxx += ax * bx + ay * by
        sxy += ax * by - ay * bx
    if not sxx and not sxy:
        return (fcx - rcx, fcy - rcy, 0.0)
    theta = math.atan2(sxy, sxx)
    cos_a, sin_a = math.cos(theta), math.sin(theta)
    # t = centroid(fixed) - R . centroid(raw), so the rotation is about the
    # origin and the pair (t, theta) is a transform, not a pair of hints.
    return (
        fcx - (rcx * cos_a - rcy * sin_a),
        fcy - (rcx * sin_a + rcy * cos_a),
        math.degrees(theta),
    )


def _session_poses(captures, match, refine):
    """La passe 1 : recaler chaque scan, puis fermer les boucles.

    Sortie en fonction pour que `build_session_grids` puisse la court-circuiter
    quand un `SessionTracker` a deja fait le travail au fil du menage.
    """
    # Le recalage a besoin d'une carte a laquelle se comparer, donc cette passe
    # construit des murs de travail, jetes ensuite. La projection definitive est
    # refaite en passe 2 depuis les poses retenues. C'est ce decoupage qui
    # permet d'inserer la fermeture de boucle entre les deux -- et, quand elle
    # ne tourne pas, le resultat est identique a l'ancien code puisque la
    # projection est deterministe a poses donnees.
    scratch: dict[tuple[int, int], float] = {}
    scans: list[tuple[float, float, float, Any, float]] = []
    # Running correction: drift accumulates, so each scan starts from the
    # previous scan's answer rather than from raw odometry again.
    dx = dy = dtheta = 0.0
    matching = match
    placed = 0
    # The pose as the robot reported it, kept per placed scan so the correction
    # handed to the replay can be fitted against where the scan actually ended
    # up rather than averaged. See fit_rigid().
    raw_poses: list[tuple[float, float]] = []
    for capture in captures:
        x, y, theta, points = capture[:4]
        weight = scan_weight(capture[5], capture[6]) if len(capture) >= 7 else 1.0
        if weight <= 0:
            continue
        raw_poses.append((x, y))
        x, y, theta = x + dx, y + dy, theta + dtheta
        if matching and placed >= MATCH_SEED:
            mx, my, mtheta = match_pose(scratch, x, y, theta, points)
            dx, dy, dtheta = dx + (mx - x), dy + (my - y), dtheta + (mtheta - theta)
            if math.hypot(dx, dy) > MATCH_MAX_DRIFT_M:
                # Runaway: stop correcting rather than invent a pose.
                _LOGGER.debug("scan matching gave up after %.2f m of drift", math.hypot(dx, dy))
                dx = dy = dtheta = 0.0
                matching = False
            else:
                x, y, theta = mx, my, mtheta
        for cell in project_scan(x, y, theta, points):
            scratch[cell] = scratch.get(cell, 0.0) + weight
        scans.append((x, y, theta, points, weight))
        placed += 1

    # -- fermeture de boucle, entre les deux passes ------------------------
    # Elle rend None quand elle n'a rien a dire -- trop peu de scans, aucun
    # retour sur zone, trop peu de fermetures retenues -- et le dit dans le
    # log. Une exception ne doit pas couter la carte : une session non
    # optimisee vaut infiniment mieux qu'une session perdue.
    if refine and scans:
        try:
            better = slam.refine_poses(
                [(s_[0], s_[1], s_[2], s_[3]) for s_ in scans],
                [(s_[0], s_[1], math.radians(s_[2])) for s_ in scans],
                MAX_RANGE_M,
                LIDAR_BEHIND_M,
            )
        except Exception:
            _LOGGER.exception("SLAM: echec, poses laissees telles quelles")
            better = None
        if better is not None:
            scans = [
                (better[k][0], better[k][1], math.degrees(better[k][2]), s_[3], s_[4])
                for k, s_ in enumerate(scans)
            ]
    # Fitted last, against the poses as the graph left them: the replay has to
    # land on the walls the *projection* used, not on the causal pass.
    correction = fit_rigid(raw_poses, [(s_[0], s_[1]) for s_ in scans])
    return scans, correction


def build_session_grids(
    captures: list[tuple[float, ...]],
    match: bool = True,
    refine: bool = False,
    tracker: SessionTracker | None = None,
) -> tuple[
    dict[tuple[int, int], float],
    set[tuple[int, int]],
    set[tuple[int, int]],
    tuple[float, float, float],
]:
    """Turn a run's captures into wall hit counts and traversed floor.

    Captures may carry the movement measured during the scan as two extra
    fields; when they do, each scan contributes its weight rather than a flat
    1. Older callers passing only (x, y, theta, points) keep the old
    behaviour.

    Each scan is also matched onto the map built from the ones before it,
    which is what stops odometry drift accumulating across a run. Pass
    match=False for the raw-odometry behaviour.

    Also returns the pose correction as a rigid transform -- (tx, ty, degrees),
    metres and degrees. The walls come out in the corrected frame; the replay's
    path and coverage are read from the robot's own log and are still in the raw
    one, so whoever serves them has to be told the difference. Leaving it out
    put the cleaned area 8 cm off the walls on the 2026-08-26 run; averaging it
    into a shift, which is what this returned until 2026-08-31, left the run of
    the 30th 124 mm off. See fit_rigid().

    CPU-bound; call it from the executor.
    """
    # -- le suiveur a-t-il deja tout fait pendant le menage ? --------------
    # Si oui, la passe 1 et l'appariement sont derriere nous : il ne reste que
    # le graphe et la projection. C'est ce qui fait tomber la fusion de cinq
    # minutes a une vingtaine de secondes sur un Raspberry Pi 5.
    if tracker is not None:
        scans = tracker.scans
        better = tracker.refined()
        if better is not None:
            scans = [
                (better[k][0], better[k][1], math.degrees(better[k][2]), s_[3], s_[4])
                for k, s_ in enumerate(scans)
            ]
        # Fitted here rather than read off the tracker, so it is measured
        # against the poses the projection below actually uses. Reading it
        # before the graph ran would describe a frame the map never sees.
        correction = fit_rigid(
            tracker.raw_poses, [(s_[0], s_[1]) for s_ in scans]
        )
    else:
        scans, correction = _session_poses(captures, match, refine)

    # -- passe 2 : la projection -------------------------------------------
    walls: dict[tuple[int, int], float] = {}
    floor: set[tuple[int, int]] = set()
    free: set[tuple[int, int]] = set()
    # How many scans saw through each cell. Counted rather than unioned so a
    # single stray beam cannot clear a wall on its own.
    seen_through: dict[tuple[int, int], int] = {}
    prev: tuple[float, float] | None = None
    for x, y, theta, points, weight in scans:
        for cell in project_scan(x, y, theta, points):
            walls[cell] = walls.get(cell, 0.0) + weight
        for cell in scan_free_cells(x, y, theta, points):
            seen_through[cell] = seen_through.get(cell, 0) + 1
        # Paint from the previous corrected pose, so the cleaned band follows
        # the path the matcher settled on rather than raw odometry.
        if prev is None:
            floor.update(stamp_floor(x, y))
            free.update(carve_swath(x, y, x, y))
        else:
            floor.update(stamp_swath(prev[0], prev[1], x, y))
            free.update(carve_swath(prev[0], prev[1], x, y))
        prev = (x, y)
    # Everything the beams saw through often enough joins what the robot drove
    # over. This is the half that reaches: driving proves a 10 cm band around a
    # path that never comes near a wall, while looking reaches everything in
    # the room the robot can see, which is where anything removed used to be.
    free |= {cell for cell, n in seen_through.items() if n >= FREE_MIN_SCANS}
    # A cell this run saw as wall is not carved by this run -- nor is any cell
    # touching one.
    #
    # Exempting only the exact cells a beam landed on was not enough: a wall is
    # a connected structure, and a cell inside one that this run happened not
    # to hit head-on was being carved by beams grazing past it. Measured over
    # the 2026-08-27 run, that punched **28 holes** straight through walls whose
    # neighbours on both sides stayed drawn -- which is what Philou saw as the
    # wall losing definition.
    #
    # One cell of margin closes every one of them and returns 163 wall cells,
    # for three cells of phantom kept a little longer. Two cells of margin buys
    # no further holes closed and costs more, so the margin is exactly one.
    # ...but "saw as wall" has to mean more than one grazing return.
    #
    # A cell at the tip of a wall is hit from every direction the robot can
    # walk around it, and passed through by the beams that go by on their way
    # to something further off. Protecting it on a single return let the map
    # draw a wall 20 cm thick and 10 cm too long where the navigation says the
    # real one tapers from 9 cm to nothing -- Philou's "the end of the wall
    # looks much thicker than the rest".
    #
    # So a cell keeps its protection only while this run's returns are worth at
    # least as much as the times it was seen through. Measured over the
    # 2026-08-27 run: the tip falls from 20.0 cm to 12.5, the median wall from
    # 10.0 to 7.5 against a real 9.4, the box's outer edge from 35 to 32 cm --
    # and there are *fewer* isolated cells than before, so this trims smear
    # rather than punching holes.
    strong = {
        cell for cell, hits in walls.items() if hits >= seen_through.get(cell, 0)
    }
    free -= {
        (cx + dx, cy + dy)
        for cx, cy in strong
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
    }
    return walls, floor, free, correction


def parse_pose(raw: str) -> tuple[float, float, float, float] | None:
    """Parse 'Robot Smooth pose: X=.., Y=.., Theta=.., Time=..'."""
    try:
        body = raw.split("pose:", 1)[1]
    except IndexError:
        return None
    found: dict[str, float] = {}
    for part in body.split(","):
        key, sep, value = part.partition("=")
        if not sep:
            continue
        try:
            found[key.strip().lower()] = float(value.strip())
        except ValueError:
            return None
    if not {"x", "y", "theta", "time"} <= found.keys():
        return None
    return found["x"], found["y"], found["theta"], found["time"]


def scan_points(payload: dict[str, Any]) -> list[tuple[int, int]]:
    """Valid returns from an /api/lidar payload, as (angle, dist_mm)."""
    return [
        (p["angle"], p["dist"])
        for p in payload.get("points", ())
        if p.get("error") == 0 and 0 < p.get("dist", 0) <= MAX_RANGE_M * 1000
    ]


class SessionTracker:
    """Place les scans et ferme les boucles au fil du menage.

    La passe 1 est causale : `match_pose` ne regarde que les murs deja poses,
    donc la pose d'un scan ne bouge plus une fois placee. L'ICP fait a
    l'arrivee d'un scan rend donc exactement ce qu'il rendrait a la fin --
    verifie sur le run du 27/08 : 9 410 paires appariees dans les deux ordres,
    ecart **0,000000 mm** sur la transformation comme sur le residu.

    L'interet n'est pas seulement d'aller plus vite. Le nombre de paires
    candidates croit comme le **carre** du nombre de scans : 481 scans en
    donnent 9 410, mais 750 scans en donneraient six fois plus, et la fusion
    repasserait a dix minutes sur un Pi 5 -- vingt sur un Pi 4, sur lequel
    tourne une bonne part des installations. Etale au fil de l'eau, le cout
    devient **constant par scan** au lieu de quadratique a la fin.

    ⚠ Le suiveur travaille sur *toutes* les captures. `_drop_slow_scans`
    n'est decide qu'a la fin, sur la mediane du run entier, et retirer un scan
    changerait les poses de tous les suivants -- `match_pose` s'accumule. Le
    contrat est donc : le resultat n'est utilisable que si le filtre ne retire
    rien. C'est le cas quasi systematique (une seule fois sur neuf runs, deux
    scans sur 501), et `usable()` le dit franchement plutot que de rendre un
    resultat approximatif.
    """

    __slots__ = ("_clouds", "_dth", "_dx", "_dy", "_edges", "_matching",
                 "_placed", "_points", "_poses", "_raw", "_scratch",
                 "_weights", "candidates", "matched")

    def __init__(self) -> None:
        self._scratch: dict[tuple[int, int], float] = {}
        self._poses: list[tuple[float, float, float]] = []
        # The pose the robot reported, before any correction, one per placed
        # scan and in the same order as _poses. That pairing is the whole
        # input to fit_rigid().
        self._raw: list[tuple[float, float]] = []
        self._clouds: list[list[tuple[float, float]]] = []
        self._points: list = []
        self._weights: list[float] = []
        self._edges: list[tuple[int, int, tuple[float, float, float], float]] = []
        self._dx = self._dy = self._dth = 0.0
        self._matching = True
        self._placed = 0
        self.candidates = 0
        self.matched = 0

    # ── pendant le menage ────────────────────────────────────────────

    def add(self, capture) -> None:
        """Place un scan et ferme ses retours sur zone avec les precedents.

        Reproduit ligne pour ligne la passe 1 de build_session_grids, puis
        apparie. Tout ecart ici invaliderait l'equivalence demontree.
        """
        x, y, theta, points = capture[:4]
        weight = scan_weight(capture[5], capture[6]) if len(capture) >= 7 else 1.0
        if weight <= 0:
            return
        self._raw.append((x, y))
        x, y, theta = x + self._dx, y + self._dy, theta + self._dth
        if self._matching and self._placed >= MATCH_SEED:
            mx, my, mtheta = match_pose(self._scratch, x, y, theta, points)
            self._dx += mx - x
            self._dy += my - y
            self._dth += mtheta - theta
            if math.hypot(self._dx, self._dy) > MATCH_MAX_DRIFT_M:
                # Runaway: stop correcting rather than invent a pose.
                _LOGGER.debug(
                    "scan matching gave up after %.2f m of drift",
                    math.hypot(self._dx, self._dy),
                )
                self._dx = self._dy = self._dth = 0.0
                self._matching = False
            else:
                x, y, theta = mx, my, mtheta
        for cell in project_scan(x, y, theta, points):
            self._scratch[cell] = self._scratch.get(cell, 0.0) + weight

        k = len(self._poses)
        self._poses.append((x, y, math.radians(theta)))
        self._points.append(points)
        self._weights.append(weight)
        self._clouds.append(
            slam.clouds([(x, y, theta, points)], MAX_RANGE_M, LIDAR_BEHIND_M)[0]
        )
        self._placed += 1
        self._close_loops(k)

    def _close_loops(self, k: int) -> None:
        """Les retours sur zone que ce scan ferme avec les precedents."""
        xk, yk = self._poses[k][0], self._poses[k][1]
        lim = slam.LOOP_MAX_DIST_M * slam.LOOP_MAX_DIST_M
        for j in range(k - slam.LOOP_MIN_GAP + 1):
            dx = self._poses[j][0] - xk
            if dx > slam.LOOP_MAX_DIST_M or dx < -slam.LOOP_MAX_DIST_M:
                continue
            dy = self._poses[j][1] - yk
            if dx * dx + dy * dy > lim:
                continue
            self.candidates += 1
            z0 = slam.relative(self._poses[j], self._poses[k])
            px, py, pth, res, fit = slam.icp(
                self._clouds[k], self._clouds[j], z0[0], z0[1], z0[2]
            )
            if fit < slam.ACCEPT_MIN_FIT or res > slam.ACCEPT_MAX_RES_M:
                continue
            w = min(
                2.0,
                (fit / slam.ACCEPT_MIN_FIT)
                * (slam.ACCEPT_MAX_RES_M / max(res, 1e-3))
                * 0.25,
            )
            self._edges.append((j, k, (px, py, pth), w))
            self.matched += 1

    # ── a la fin ─────────────────────────────────────────────────────

    def usable(self, dropped: int) -> bool:
        """Le suiveur ne vaut que si le filtre de rotation n'a rien retire.

        Retirer un scan changerait la pose de tous les suivants, puisque
        `match_pose` s'accumule -- le resultat serait faux, pas approximatif.
        Le critere n'est donc PAS un compte de poses (add() ecarte deja les
        scans de poids nul, ce qui est un autre filtre), mais bien : combien
        `_drop_slow_scans` a-t-il retire ?
        """
        return dropped == 0 and bool(self._poses)

    def refined(self):
        """Poses optimisees. Rend None -- en disant pourquoi -- s'il n'y a
        pas de quoi contraindre le graphe."""
        n = len(self._poses)
        if n < slam.MIN_SCANS:
            _LOGGER.debug("SLAM: %d scans, trop peu pour fermer une boucle", n)
            return None
        if self.matched < n // 4:
            _LOGGER.info(
                "SLAM: %d fermetures retenues sur %d candidates, trop peu pour "
                "contraindre %d scans -- poses laissees telles quelles",
                self.matched, self.candidates, n,
            )
            return None
        edges = [
            (k, k + 1, slam.relative(self._poses[k], self._poses[k + 1]), 1.0)
            for k in range(n - 1)
        ]
        edges.extend(self._edges)
        refined = slam.optimise(self._poses, edges)
        # Le succes doit se voir autant que l'echec. Sans cette ligne, un
        # suiveur qui a travaille est indiscernable d'un suiveur qui n'a
        # jamais ete alimente -- constate sur le run du 27/08 au soir, ou il a
        # fallu comparer la translation de fusion a un rejeu hors ligne pour
        # savoir laquelle des deux situations on regardait.
        moved = sorted(
            math.hypot(refined[k][0] - self._poses[k][0],
                       refined[k][1] - self._poses[k][1])
            for k in range(n)
        )
        _LOGGER.info(
            "SLAM: %d fermetures sur %d candidates fermees pendant le menage ; "
            "poses deplacees de %.1f cm en median, %.1f cm au pire",
            self.matched, self.candidates, 100 * moved[n // 2], 100 * moved[-1],
        )
        return refined

    @property
    def placed(self) -> int:
        """Combien de scans ont ete places jusqu'ici."""
        return self._placed

    def walls_so_far(self) -> dict[tuple[int, int], float]:
        """Les murs vus depuis le debut du menage, dans le repere du run.

        C'est la copie de `_scratch`, que la passe 1 tient deja a jour pour
        l'appariement : la meme accumulation, cellule par cellule, que la
        passe 2 de `build_session_grids` produirait sur les memes scans. Placer
        la session en cours sur la carte ne coute donc aucune geometrie
        supplementaire -- seulement l'ajustement lui-meme.

        Ce n'est pas tout a fait ce que la fusion verra : elle projettera
        depuis les poses fermees par le graphe, pas depuis les poses causales.
        L'ecart est de quelques centimetres, sans effet sur le choix d'un quart
        de tour.

        ⚠ Le dictionnaire est copie : `add()` tourne dans l'executeur, et le
        rendre tel quel ferait iterer l'appelant sur une structure en cours de
        modification.
        """
        return dict(self._scratch)

    @property
    def scans(self):
        """(x, y, theta_deg, points, weight) par scan place."""
        return [
            (p[0], p[1], math.degrees(p[2]), pts, w)
            for p, pts, w in zip(self._poses, self._points, self._weights)
        ]

    def add_many(self, captures) -> None:
        """Absorbe un lot. Appele depuis l'executeur : un tick de rattrapage
        peut apporter deux douzaines de scans, et ~83 ms de CPU chacun dans la
        boucle d'evenements est exactement ce qui rend la maison lente."""
        for capture in captures:
            self.add(capture)

    @property
    def raw_poses(self) -> list[tuple[float, float]]:
        """Les poses telles que le robot les a rapportees, une par scan place.

        Appariees a `scans` par l'indice : c'est ce que `fit_rigid` demande.
        """
        return list(self._raw)

    @property
    def correction(self) -> tuple[float, float, float]:
        """Le deplacement rigide que le recalage a fait subir au menage.

        (tx, ty, degres), en metres et degres : `pose_carte ~= R(d) . brute + t`.
        Le rejeu la reclame pour dessiner le trajet dans le meme repere que les
        murs.

        Sur les poses causales, donc utilisable **pendant** le menage. La
        fusion, elle, refait l'ajustement contre les poses fermees par le
        graphe -- voir `build_session_grids`.
        """
        return fit_rigid(self._raw, [(p[0], p[1]) for p in self._poses])
