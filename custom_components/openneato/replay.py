"""Session parsing for the interactive replay card.

Direct port of `frontend/src/history-data.ts::buildSession`, which differs
from `history_renderer.parse_session_jsonl` in two ways the canvas player
depends on:

  * pose timestamps are normalised against the first retained pose, so the
    scrubber's 0 lines up with the summary duration;
  * recharge markers (which carry no `ts` of their own) are paired with the
    long gaps in the pose timeline, giving each one a start/end window for
    the scrubber's charge segments.

The renderer's parser is left untouched so the static PNG / GIF cameras keep
their current behaviour.

Output is deliberately compact -- flat number arrays instead of dicts, and
coordinates rounded -- because a full session ships over the WebSocket
connection to the browser.
"""

from __future__ import annotations

import json
import logging
import math
from typing import Any

from .const import HISTORY_CELL_SIZE_M, HISTORY_ROBOT_DIAMETER_M
from .history_renderer import _try_repair_pose

_LOGGER = logging.getLogger(__name__)

# Firmware samples poses every ~2s; anything past 15x that is a real pause
# (docking, charging) rather than jitter between snapshots.
POSE_INTERVAL_S = 2.0
GAP_MIN_S = 30.0


def _r(value: float, digits: int = 3) -> float:
    """Round for the wire -- sub-millimetre precision is noise here."""
    return round(float(value), digits)


def build_replay_session(raw: str, name: str = "") -> dict[str, Any]:
    """Parse raw session JSONL into the payload the replay card consumes."""
    lines = [l for l in raw.strip().split("\n") if l.strip()]

    session: dict | None = None
    summary: dict | None = None
    poses: list[dict[str, float]] = []
    # Recharge markers carry no timestamp, so remember the ts of the last
    # pose seen before each one -- that anchors it on the timeline.
    raw_recharges: list[dict[str, float]] = []
    last_pose_ts = 0.0

    for line in lines:
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            repaired = _try_repair_pose(line)
            if repaired is None:
                continue
            obj = repaired

        obj_type = obj.get("type")
        if obj_type == "session":
            session = obj
        elif obj_type == "summary":
            summary = obj
        elif obj_type == "recharge":
            raw_recharges.append(
                {
                    "x": float(obj.get("x", 0)),
                    "y": float(obj.get("y", 0)),
                    "prevTs": last_pose_ts,
                }
            )
        elif "x" in obj and "y" in obj and "type" not in obj:
            last_pose_ts = float(obj.get("ts", 0))
            # Skip the origin pose (all zeros) -- same filter as the frontend.
            if obj.get("x") != 0 or obj.get("y") != 0 or obj.get("t") != 0:
                poses.append(obj)

    if not poses:
        return {
            "name": name,
            "session": session,
            "summary": summary,
            "cellSize": HISTORY_CELL_SIZE_M,
            "bounds": None,
            "duration": 0.0,
            "path": [],
            "coverage": [],
            "recharges": [],
        }

    # `ts` is seconds since boot, not since session start. Normalise so the
    # timeline starts at 0 and matches the summary duration.
    t_origin = float(poses[0].get("ts", 0))
    norm: list[tuple[float, float, float, float]] = [
        (
            float(p.get("x", 0)),
            float(p.get("y", 0)),
            float(p.get("t", 0)),
            max(0.0, float(p.get("ts", 0)) - t_origin),
        )
        for p in poses
    ]

    recharges = _pair_recharges(raw_recharges, norm, t_origin)
    coverage = _coverage_cells(norm)

    pad = HISTORY_ROBOT_DIAMETER_M / 2 + 0.1
    xs = [p[0] for p in norm]
    ys = [p[1] for p in norm]
    bounds = {
        "minX": _r(min(xs) - pad),
        "maxX": _r(max(xs) + pad),
        "minY": _r(min(ys) - pad),
        "maxY": _r(max(ys) + pad),
    }

    # Prefer the firmware's own duration, matching helpers.ts::sessionDuration.
    if summary and float(summary.get("duration", 0) or 0) > 0:
        duration = float(summary["duration"])
    else:
        duration = norm[-1][3]

    path: list[float] = []
    for x, y, t, ts in norm:
        path.extend((_r(x), _r(y), _r(t, 1), _r(ts, 1)))

    _LOGGER.debug(
        "Replay session %s: %d poses, %d coverage cells, %d recharges, %.0fs",
        name, len(norm), len(coverage) // 3, len(recharges), duration,
    )

    return {
        "name": name,
        "session": session,
        "summary": summary,
        "cellSize": HISTORY_CELL_SIZE_M,
        "bounds": bounds,
        "duration": duration,
        "path": path,
        "coverage": coverage,
        "recharges": recharges,
    }


def _pair_recharges(
    raw_recharges: list[dict[str, float]],
    norm: list[tuple[float, float, float, float]],
    t_origin: float,
) -> list[dict[str, float]]:
    """Give each recharge marker a start/end window on the timeline.

    The firmware writes the marker as soon as docking begins but keeps
    emitting snapshots until collection pauses, so the real charge window is
    the long gap a few lines later. Each marker claims its nearest unused
    gap, preferring one that starts after it.
    """
    if not raw_recharges:
        return []

    gaps: list[tuple[float, float]] = []
    for i in range(1, len(norm)):
        if norm[i][3] - norm[i - 1][3] >= GAP_MIN_S:
            gaps.append((norm[i - 1][3], norm[i][3]))

    used: set[int] = set()
    out: list[dict[str, float]] = []
    for r in raw_recharges:
        marker_ts = max(0.0, r["prevTs"] - t_origin)
        best_idx = -1
        best_score = math.inf
        for i, (start, _end) in enumerate(gaps):
            if i in used:
                continue
            # A gap before the marker is penalised 4x -- the pause always
            # follows the marker, so an earlier gap is a much worse match.
            distance = start - marker_ts if start >= marker_ts else (marker_ts - start) * 4
            if distance < best_score:
                best_score = distance
                best_idx = i
        if best_idx >= 0:
            used.add(best_idx)
            start, end = gaps[best_idx]
            out.append({"x": _r(r["x"]), "y": _r(r["y"]), "ts": _r(start, 1), "endTs": _r(end, 1)})
            continue
        # No large gap found -- a single-interval sliver still marks the spot.
        nxt = next((p[3] for p in norm if p[3] > marker_ts), None)
        out.append(
            {
                "x": _r(r["x"]),
                "y": _r(r["y"]),
                "ts": _r(marker_ts, 1),
                "endTs": _r(nxt if nxt is not None else marker_ts + POSE_INTERVAL_S, 1),
            }
        )
    return out


def _coverage_cells(norm: list[tuple[float, float, float, float]]) -> list[float]:
    """Stamp the robot footprint at each pose, keeping the earliest touch ts.

    Returned flat as [cx, cy, ts, cx, cy, ts, ...] so the payload stays small.
    """
    cell = HISTORY_CELL_SIZE_M
    radius_cells = math.ceil(HISTORY_ROBOT_DIAMETER_M / 2 / cell)
    # Precompute the footprint disc once instead of re-testing dx^2+dy^2 per pose.
    disc = [
        (dx, dy)
        for dx in range(-radius_cells, radius_cells + 1)
        for dy in range(-radius_cells, radius_cells + 1)
        if dx * dx + dy * dy <= radius_cells * radius_cells
    ]

    first_ts: dict[tuple[int, int], float] = {}
    for x, y, _t, ts in norm:
        cx = round(x / cell)
        cy = round(y / cell)
        for dx, dy in disc:
            first_ts.setdefault((cx + dx, cy + dy), ts)

    flat: list[float] = []
    for (cx, cy), ts in first_ts.items():
        flat.extend((cx, cy, _r(ts, 1)))
    return flat
