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

    wx = x + d * cos(radians(angle + theta))
    wy = y + d * sin(radians(angle + theta))

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

from PIL import Image, ImageDraw

_LOGGER = logging.getLogger(__name__)

CELL_M = 0.05                # grid resolution
MAX_RANGE_M = 3.5            # returns beyond this are noisy and grazing
WALL_MIN_HITS = 12           # floor, and the whole rule for a young map.
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
# A quantile is self-calibrating: it keeps a stable *share* of the map however
# many cleanings pile up, and rises on its own as noise accumulates. At four
# sessions the 75th percentile lands on 19 -- which is where an earlier
# by-eye sweep had put the threshold after two sessions, so the rule agrees
# with the judgement it replaces.
WALL_KEEP_QUANTILE = 0.75


def wall_threshold(walls: dict[Any, int]) -> int:
    """Hit count a cell must reach to count as wall, for this map."""
    if not walls:
        return WALL_MIN_HITS
    counts = sorted(walls.values())
    idx = min(len(counts) - 1, int(len(counts) * WALL_KEEP_QUANTILE))
    return max(WALL_MIN_HITS, counts[idx])
ROBOT_RADIUS_M = 0.165       # Botvac D6, for stamping visited floor
RENDER_PX_PER_M = 100
RENDER_PAD_M = 0.3

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
    inv = 1.0 / CELL_M
    out: list[tuple[int, int]] = []
    for angle, dist_mm in points:
        if not 0 < dist_mm <= MAX_RANGE_M * 1000:
            continue
        d = dist_mm / 1000.0
        ar = math.radians(angle)
        c, s = math.cos(ar), math.sin(ar)
        wx = x + d * (c * ct - s * st)
        wy = y + d * (s * ct + c * st)
        out.append((math.floor(wx * inv), math.floor(wy * inv)))
    return out


def stamp_floor(x: float, y: float) -> list[tuple[int, int]]:
    """Grid cells covered by the robot's footprint at a pose."""
    inv = 1.0 / CELL_M
    r = math.ceil(ROBOT_RADIUS_M * inv)
    cx, cy = round(x * inv), round(y * inv)
    return [
        (cx + dx, cy + dy)
        for dx in range(-r, r + 1)
        for dy in range(-r, r + 1)
        if dx * dx + dy * dy <= r * r
    ]


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


def align_to_reference(
    new_walls: dict[tuple[int, int], int],
    ref_walls: dict[tuple[int, int], int],
    search_cells: int = 8,
) -> tuple[int, int, int, float]:
    """Fit a new session's walls onto the accumulated map.

    Tries the four quarter turns, each with a small translation search, and
    returns (quarter, dx, dy, overlap). Overlap is the share of the new
    session's wall cells that coincide with the reference, so 1.0 is perfect.

    A quarter turn is the only rotation considered on purpose: the frame moves
    because the dock heading reference is re-zeroed, which lands on right
    angles, and searching arbitrary angles would invite false matches.
    """
    if not ref_walls or not new_walls:
        return 0, 0, 0, 0.0

    ref = set(ref_walls)
    fcx = sum(c[0] for c in ref) / len(ref)
    fcy = sum(c[1] for c in ref) / len(ref)
    best = (0, 0, 0, -1.0)
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
                    if score > best[3]:
                        best = (quarter, dx, dy, score)
    return best


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






def render_plan(
    walls: dict[tuple[int, int], int],
    floor: set[tuple[int, int]],
    px_per_m: int = RENDER_PX_PER_M,
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
    wall_cells = {c for c, n in walls.items() if n >= wall_threshold(walls)}
    min_x, min_y = cal["origin_x"], cal["origin_y"]
    width, height = cal["width"], cal["height"]
    max_y = min_y + height / px_per_m

    img = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    step = CELL_M * px_per_m

    def box(cx: int, cy: int):
        # World Y grows up, image rows grow down.
        x0 = (cx * CELL_M - min_x) * px_per_m
        y0 = (max_y - (cy + 1) * CELL_M) * px_per_m
        return [x0, y0, x0 + step, y0 + step]

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
    wall_cells = {c for c, n in walls.items() if n >= wall_threshold(walls)}
    if not wall_cells:
        return None

    cells = wall_cells | floor
    min_x = min(c[0] for c in cells) * CELL_M - RENDER_PAD_M
    max_x = (max(c[0] for c in cells) + 1) * CELL_M + RENDER_PAD_M
    min_y = min(c[1] for c in cells) * CELL_M - RENDER_PAD_M
    max_y = (max(c[1] for c in cells) + 1) * CELL_M + RENDER_PAD_M

    width = max(1, round((max_x - min_x) * px_per_m))
    height = max(1, round((max_y - min_y) * px_per_m))
    if width > 4096 or height > 4096:
        _LOGGER.warning("Generated plan too large (%dx%d); skipping", width, height)
        return None
    return {
        "scale": float(px_per_m),
        "origin_x": round(min_x, 3),
        "origin_y": round(min_y, 3),
        "width": width,
        "height": height,
        "wall_cells": len(wall_cells),
        "floor_cells": len(floor),
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
            "alignments": {k: list(v) for k, v in self.alignments.items()},
        }

    def merge_session(
        self,
        walls: dict[tuple[int, int], int],
        floor: set[tuple[int, int]],
        session_name: str | None = None,
    ) -> dict[str, Any]:
        """Fold one cleaning into the accumulated map, re-aligning it first."""
        report: dict[str, Any] = {"session_walls": len(walls), "realigned": False}

        if self.walls:
            quarter, dx, dy, overlap = align_to_reference(walls, self.walls)
            report.update(quarter=quarter, dx=dx, dy=dy, overlap=round(overlap, 3))
            if overlap < MERGE_MIN_OVERLAP:
                # Nothing lines up. Rather than corrupt a good map with a bad
                # fit, keep what we have and say so.
                report["rejected"] = True
                _LOGGER.warning(
                    "LIDAR map: new session only overlaps %.0f%% of the stored "
                    "map; refusing to merge it", 100 * overlap,
                )
                return report
            if quarter or dx or dy:
                report["realigned"] = True
                _LOGGER.info(
                    "LIDAR map: session re-aligned by %d deg and (%d, %d) cells "
                    "before merging (%.0f%% overlap)",
                    quarter * 90, dx, dy, 100 * overlap,
                )
            walls = {
                (cx + dx, cy + dy): n
                for (cx, cy), n in zip(_rotate_cells(walls, quarter), walls.values())
            }
            floor = {
                (cx + dx, cy + dy) for cx, cy in _rotate_cells(floor, quarter)
            }

        if session_name:
            self.alignments[session_name] = (quarter, dx, dy) if self.walls else (0, 0, 0)

        for cell, n in walls.items():
            self.walls[cell] = self.walls.get(cell, 0) + n
        self.floor |= floor
        self.sessions += 1

        if len(self.walls) > MAX_GRID_CELLS:
            # Drop the weakest evidence first; real walls are seen repeatedly.
            keep = sorted(self.walls.items(), key=lambda kv: -kv[1])[:MAX_GRID_CELLS]
            self.walls = dict(keep)
            _LOGGER.warning("LIDAR map: grid capped at %d cells", MAX_GRID_CELLS)

        report["total_walls"] = len(self.walls)
        report["sessions"] = self.sessions
        return report

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
        wall_cells = [c for c, n in self.walls.items() if n >= wall_threshold(self.walls)]
        skew = manhattan_angle(wall_cells)
        return round((-skew) + self.quarter_lock * 90 + user_offset, 2) % 360


def _key(raw: str) -> tuple[int, int]:
    cx, _, cy = raw.partition(",")
    return int(cx), int(cy)


# ── runtime ─────────────────────────────────────────────────────────


def build_session_grids(
    captures: list[tuple[float, float, float, list[tuple[int, int]]]]
) -> tuple[dict[tuple[int, int], int], set[tuple[int, int]]]:
    """Turn a run's captures into wall hit counts and traversed floor.

    CPU-bound; call it from the executor.
    """
    walls: dict[tuple[int, int], int] = {}
    floor: set[tuple[int, int]] = set()
    for x, y, theta, points in captures:
        for cell in project_scan(x, y, theta, points):
            walls[cell] = walls.get(cell, 0) + 1
        floor.update(stamp_floor(x, y))
    return walls, floor


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
