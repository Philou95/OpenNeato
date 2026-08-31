"""Loop closure over a session's poses, in pure Python.

The pipeline already corrects the pose scan by scan (`match_pose`) and session
by session (`align_to_reference`). The middle storey was missing: nothing
corrected the drift accumulated over a full circuit, when the robot sees a wall
from one side at the start of a cleaning and from the other forty minutes later.

This module adds that storey. Every scan is a node; two scans that see the same
place at distant times give a closure constraint, and the error is spread over
the whole trajectory instead of being forced onto the current scan alone.

**No dependencies.** numpy and scipy cannot go into a Home Assistant
integration, so:

  - `cKDTree` -> a hash grid at the search radius. A tree earns its keep when
    the radius is unknown; here it is always `MAX_PAIR_M`, so nine cells are
    enough and building it is linear.
  - SVD -> closed-form 2D Kabsch. Running an SVD on a 2x2 matrix makes no
    sense.
  - `spsolve` -> block Gauss-Seidel relaxation on 3x3 blocks, over-relaxed.
    That is legitimate because the graph is *dense* -- measured, ~5900 edges
    for 479 nodes -- so information crosses the trajectory in a few sweeps.

Measured on 2026-08-27 over three recorded runs, against the reference scipy
version: median difference **0.07 mm** on the poses, identical accept/reject
decisions on the closures, and 13x faster on the optimisation.

What it buys, measured on the most drifted run:

  - the calibration box goes from 20.0 x 62.5 cm **unstable** to 27.5 x 47.5
    stable against a true 29 x 50, and isolated cells from 40 to 14;
  - and above all the **merge overlap** goes from 67.8% to 85.3%. Below
    `MERGE_MIN_OVERLAP` the session is refused, and three refusals in a row
    discard the accumulated map: that session was 3.5 points from the cliff,
    it is now 28.
"""

from __future__ import annotations

import logging
import math

_LOGGER = logging.getLogger(__name__)

MAX_PAIR_M = 0.25          # ICP pairing radius, and the grid's own step
ICP_ITERS = 30
ICP_MIN_PTS = 30           # below this the pairing has nothing to say

# A scan carries ~205 returns, far more than are needed to find a rotation and
# a translation. Measured on the run of the evening of 27/08, 9410 candidates:
# at 204 points the pairing takes 75.7 s for 69.4% overlap, at 102 points it
# takes **34.1 s for 69.5%** -- 2.2x faster and one isolated cell fewer. Going
# lower costs: 68 points return only 67.0%, and it is not the ICP_MIN_PTS guard
# holding it back (making that proportional recovers only 0.7 of a point), it
# is missing information. So 2, not 3.
#
# The stride preserves the angular spread: returns are ordered by angle over
# 360 degrees, so every other one still covers the full turn.
ICP_POINT_STRIDE = 2

LOOP_MIN_GAP = 40          # minimum scans apart to call it a revisit
LOOP_MAX_DIST_M = 1.2      # beyond this the two scans do not see the same thing

# A wrong closure link is worse than no link: it drags the whole trajectory
# towards an invented position. Hence a double rejection, on the residual AND
# on the share of points that find a match.
ACCEPT_MIN_FIT = 0.55
ACCEPT_MAX_RES_M = 0.05

# omega=1.9 is the value that matters: at 1.0 the difference from scipy is
# 5.7 mm in 18.5 s, at 1.9 it is 0.07 mm in 4.4 s. The same optimum comes out
# on a synthetic graph and on the real runs.
OMEGA = 1.9
SWEEPS = 400
HUBER_M = 0.10             # past this a link weighs less -- guard against a false link
DAMPING = 1e-6
TOL_M = 1e-5

MIN_SCANS = 80             # below this there is no loop to close
MAX_SCANS = 1500           # guard: the cost grows with the candidates


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def relative(a, b):
    """Pose of b as seen from a."""
    ca, sa = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (ca * dx + sa * dy, -sa * dx + ca * dy, wrap(b[2] - a[2]))


class _Grid:
    """Points bucketed into cells of side `step`, for a bounded nearest neighbour."""

    __slots__ = ("cells", "step")

    def __init__(self, points, step):
        self.step = step
        self.cells = {}
        inv = 1.0 / step
        for k, (x, y) in enumerate(points):
            c = (math.floor(x * inv), math.floor(y * inv))
            b = self.cells.get(c)
            if b is None:
                self.cells[c] = [(x, y, k)]
            else:
                b.append((x, y, k))

    def nearest(self, x, y, rmax):
        inv = 1.0 / self.step
        cx, cy = math.floor(x * inv), math.floor(y * inv)
        best = rmax * rmax
        bi = -1
        get = self.cells.get
        for ddx in (-1, 0, 1):
            for ddy in (-1, 0, 1):
                b = get((cx + ddx, cy + ddy))
                if b is None:
                    continue
                for px, py, k in b:
                    ex, ey = px - x, py - y
                    d = ex * ex + ey * ey
                    if d < best:
                        best = d
                        bi = k
        return (math.sqrt(best), bi) if bi >= 0 else (None, -1)


def _median(v):
    v = sorted(v)
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


def icp(src, dst, x0, y0, th0, iters=ICP_ITERS, max_pair=MAX_PAIR_M):
    """Bring `src` onto `dst`. Returns (x, y, th, median residual, matched share)."""
    n = len(src)
    if n == 0 or len(dst) < ICP_MIN_PTS:
        return x0, y0, th0, 9.9, 0.0
    g = _Grid(dst, max_pair)
    x, y, th = x0, y0, th0
    res, fit = 9.9, 0.0
    for _ in range(iters):
        c, s = math.cos(th), math.sin(th)
        pts = []
        qts = []
        ds = []
        for px0, py0 in src:
            px = px0 * c - py0 * s + x
            py = px0 * s + py0 * c + y
            d, k = g.nearest(px, py, max_pair)
            if k >= 0:
                pts.append((px, py))
                qts.append(dst[k])
                ds.append(d)
        m = len(pts)
        if m < ICP_MIN_PTS:
            return x, y, th, 9.9, 0.0
        pcx = sum(p[0] for p in pts) / m
        pcy = sum(p[1] for p in pts) / m
        qcx = sum(q[0] for q in qts) / m
        qcy = sum(q[1] for q in qts) / m
        num = den = 0.0
        for (px, py), (qx, qy) in zip(pts, qts):
            ax, ay = px - pcx, py - pcy
            bx, by = qx - qcx, qy - qcy
            num += ax * by - ay * bx
            den += ax * bx + ay * by
        dth = math.atan2(num, den)
        cc, ss = math.cos(dth), math.sin(dth)
        nx = qcx - (cc * pcx - ss * pcy)
        ny = qcy - (ss * pcx + cc * pcy)
        x, y = cc * x - ss * y + nx, ss * x + cc * y + ny
        th += dth
        res = _median(ds)
        fit = m / n
        if abs(dth) < 1e-4 and res < 0.02:
            break
    return x, y, th, res, fit


def _residual(pose_list, i, j, z):
    xi, yi, ti = pose_list[i]
    xj, yj, tj = pose_list[j]
    ci, si = math.cos(ti), math.sin(ti)
    dx, dy = xj - xi, yj - yi
    px = ci * dx + si * dy
    py = -si * dx + ci * dy
    zx, zy, zt = z
    qx, qy = px - zx, py - zy
    cz, sz = math.cos(zt), math.sin(zt)
    return (cz * qx + sz * qy, -sz * qx + cz * qy,
            wrap(tj - ti - zt), ci, si, cz, sz, dx, dy)


def _blocks(ci, si, cz, sz, dx, dy):
    a00, a01 = -ci, -si
    a10, a11 = si, -ci
    c0 = -si * dx + ci * dy
    c1 = -ci * dx - si * dy
    a_00 = cz * a00 + sz * a10
    a_01 = cz * a01 + sz * a11
    a_02 = cz * c0 + sz * c1
    a_10 = -sz * a00 + cz * a10
    a_11 = -sz * a01 + cz * a11
    a_12 = -sz * c0 + cz * c1
    mat_a = (a_00, a_01, a_02, a_10, a_11, a_12, 0.0, 0.0, -1.0)
    mat_b = (-a_00, -a_01, 0.0, -a_10, -a_11, 0.0, 0.0, 0.0, 1.0)
    return mat_a, mat_b


def _solve3(h00, h01, h02, h11, h12, h22, b0, b1, b2, damp):
    """3x3 Cholesky. Returns None rather than an invented direction if it fails."""
    h00 += damp
    h11 += damp
    h22 += damp
    if h00 <= 0.0:
        return None
    l00 = math.sqrt(h00)
    l10 = h01 / l00
    l20 = h02 / l00
    d1 = h11 - l10 * l10
    if d1 <= 0.0:
        return None
    l11 = math.sqrt(d1)
    l21 = (h12 - l20 * l10) / l11
    d2 = h22 - l20 * l20 - l21 * l21
    if d2 <= 0.0:
        return None
    l22 = math.sqrt(d2)
    y0 = -b0 / l00
    y1 = (-b1 - l10 * y0) / l11
    y2 = (-b2 - l20 * y0 - l21 * y1) / l22
    x2 = y2 / l22
    x1 = (y1 - l21 * x2) / l11
    x0 = (y0 - l10 * x1 - l20 * x2) / l00
    return x0, x1, x2


def optimise(poses, edges, sweeps=SWEEPS, fixed=0, huber=HUBER_M, omega=OMEGA):
    """Block Gauss-Seidel relaxation. Returns the list of corrected poses."""
    grid = [[float(p[0]), float(p[1]), float(p[2])] for p in poses]
    n_nodes = len(grid)
    inc = [[] for _ in range(n_nodes)]
    for edge in edges:
        inc[edge[0]].append((edge, True))
        inc[edge[1]].append((edge, False))

    for _ in range(sweeps):
        biggest = 0.0
        for node in range(n_nodes):
            if node == fixed or not inc[node]:
                continue
            h00 = h01 = h02 = h11 = h12 = h22 = 0.0
            b0 = b1 = b2 = 0.0
            for edge, is_i in inc[node]:
                i, j, z, w = edge
                ex, ey, eth, ci, si, cz, sz, dx, dy = _residual(grid, i, j, z)
                nrm = math.hypot(ex, ey)
                k = 1.0 if nrm <= huber else huber / nrm
                ww = w * k
                mat_a, mat_b = _blocks(ci, si, cz, sz, dx, dy)
                j00, j01, j02, j10, j11, j12, j20, j21, j22 = (
                    mat_a if is_i else mat_b
                )
                h00 += ww * (j00 * j00 + j10 * j10 + j20 * j20)
                h01 += ww * (j00 * j01 + j10 * j11 + j20 * j21)
                h02 += ww * (j00 * j02 + j10 * j12 + j20 * j22)
                h11 += ww * (j01 * j01 + j11 * j11 + j21 * j21)
                h12 += ww * (j01 * j02 + j11 * j12 + j21 * j22)
                h22 += ww * (j02 * j02 + j12 * j12 + j22 * j22)
                b0 += ww * (j00 * ex + j10 * ey + j20 * eth)
                b1 += ww * (j01 * ex + j11 * ey + j21 * eth)
                b2 += ww * (j02 * ex + j12 * ey + j22 * eth)
            step = _solve3(h00, h01, h02, h11, h12, h22, b0, b1, b2, DAMPING)
            if step is None:
                continue
            grid[node][0] += omega * step[0]
            grid[node][1] += omega * step[1]
            grid[node][2] = wrap(grid[node][2] + omega * step[2])
            biggest = max(biggest, abs(step[0]), abs(step[1]), abs(step[2]))
        if biggest < TOL_M:
            break
    return grid


def clouds(scans, max_range_m, lidar_behind_m):
    """Each scan as a point cloud, in the robot's frame.

    The LIDAR sits behind the centre of rotation: without that offset the same
    wall is stamped up to 206 mm further away depending on the direction of
    travel.
    """
    out = []
    rad = math.pi / 180.0
    for scan in scans:
        pts = []
        for point in scan[3][::ICP_POINT_STRIDE]:
            ang = point[0]
            dist = point[1] / 1000.0
            if dist <= 0.0 or dist > max_range_m:
                continue
            t = ang * rad
            pts.append((dist * math.cos(t) - lidar_behind_m, dist * math.sin(t)))
        out.append(pts)
    return out


def loop_candidates(poses, min_gap=LOOP_MIN_GAP, max_dist=LOOP_MAX_DIST_M):
    """Scan pairs close in space and far apart in time.

    That is the definition of a revisit: the robot passes where it has already
    been, late enough that its pose has drifted in between.
    """
    lim = max_dist * max_dist
    out = []
    n = len(poses)
    for a in range(n):
        xa, ya = poses[a][0], poses[a][1]
        for b in range(a + min_gap, n):
            dx = poses[b][0] - xa
            if dx > max_dist or dx < -max_dist:
                continue
            dy = poses[b][1] - ya
            d2 = dx * dx + dy * dy
            if d2 <= lim:
                out.append((a, b))
    return out


def refine_poses(scans, poses, max_range_m, lidar_behind_m):
    """Returns the poses corrected by loop closure, or None.

    `scans`: list of (x, y, theta_deg, points), aligned with `poses`.
    `poses`: list of (x, y, theta_rad) as scan-to-map matching established them.

    Returns None -- and says why -- rather than returning the input poses: the
    caller has to be able to tell "nothing to correct" from "corrected".
    """
    n = len(poses)
    if n < MIN_SCANS:
        _LOGGER.debug("SLAM: %d scans, too few to close a loop", n)
        return None
    if n > MAX_SCANS:
        _LOGGER.info("SLAM: %d scans, past the guard of %d", n, MAX_SCANS)
        return None

    cl = clouds(scans, max_range_m, lidar_behind_m)
    cands = loop_candidates(poses)
    if not cands:
        _LOGGER.info("SLAM: no revisit found over %d scans", n)
        return None

    # Odometry: the trajectory as scan-to-map matching established it.
    edges = [
        (k, k + 1, relative(poses[k], poses[k + 1]), 1.0)
        for k in range(n - 1)
    ]
    n_odo = len(edges)
    residuals = []
    for a, b in cands:
        z0 = relative(poses[a], poses[b])
        x, y, th, res, fit = icp(cl[b], cl[a], z0[0], z0[1], z0[2])
        if fit < ACCEPT_MIN_FIT or res > ACCEPT_MAX_RES_M:
            continue
        # Weight: a tight agreement counts for more than a borderline one.
        w = min(
            2.0,
            (fit / ACCEPT_MIN_FIT) * (ACCEPT_MAX_RES_M / max(res, 1e-3)) * 0.25,
        )
        edges.append((a, b, (x, y, th), w))
        residuals.append(res)

    kept = len(edges) - n_odo
    if kept < n // 4:
        _LOGGER.info(
            "SLAM: %d closures kept of %d candidates, too few to constrain "
            "%d scans -- poses left as they are",
            kept, len(cands), n,
        )
        return None

    refined = optimise(poses, edges)
    moved = sorted(
        math.hypot(refined[k][0] - poses[k][0], refined[k][1] - poses[k][1])
        for k in range(n)
    )
    _LOGGER.info(
        "SLAM: %d closures of %d candidates, median residual %.1f cm; "
        "poses moved %.1f cm in the median, %.1f cm at worst",
        kept, len(cands), 100 * _median(residuals),
        100 * moved[n // 2], 100 * moved[-1],
    )
    return refined
