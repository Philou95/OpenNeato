"""Fermeture de boucle sur les poses d'une session, en Python pur.

Le pipeline corrige deja la pose scan par scan (`match_pose`) et session par
session (`align_to_reference`). Il manquait l'etage du milieu : rien ne corrige
la derive accumulee sur un tour complet, quand le robot voit un mur d'un cote
au debut du cycle et de l'autre quarante minutes plus tard.

Ce module ajoute cet etage. Chaque scan est un noeud ; deux scans qui voient le
meme endroit a des instants eloignes donnent une contrainte de fermeture, et
l'erreur est repartie sur toute la trajectoire au lieu d'etre imposee au seul
scan courant.

**Aucune dependance.** numpy et scipy ne peuvent pas entrer dans une integration
Home Assistant, donc :

  - `cKDTree` -> grille de hachage au pas du rayon de recherche. Un arbre sert
    quand le rayon est inconnu ; ici il vaut toujours `MAX_PAIR_M`, donc neuf
    cases suffisent et la construction est lineaire.
  - SVD -> Kabsch 2D en forme fermee. Faire tourner une SVD sur une matrice
    2x2 n'a pas de sens.
  - `spsolve` -> relaxation de Gauss-Seidel par blocs 3x3, sur-relaxee. C'est
    legitime parce que le graphe est *dense* -- mesure, ~5 900 aretes pour 479
    noeuds -- donc l'information traverse la trajectoire en quelques balayages.

Mesure le 2026-08-27 sur trois runs enregistres, contre la version scipy de
reference : ecart median **0,07 mm** sur les poses, decisions d'acceptation des
fermetures identiques, et 13x plus rapide sur l'optimisation.

Ce que ca achete, mesure sur le run le plus derive :

  - le carton etalon passe de 20,0 x 62,5 cm **instable** a 27,5 x 47,5 stable
    contre 29 x 50 reels, cellules isolees de 40 a 14 ;
  - et surtout le **recouvrement de fusion** passe de 67,8 % a 85,3 %. Sous
    `MERGE_MIN_OVERLAP` la session est refusee, et trois refus consecutifs
    effacent la carte accumulee : cette session-la etait a 3,5 points de la
    falaise, elle est maintenant a 28.
"""

from __future__ import annotations

import logging
import math

_LOGGER = logging.getLogger(__name__)

MAX_PAIR_M = 0.25          # rayon d'appariement ICP, et pas de la grille
ICP_ITERS = 30
ICP_MIN_PTS = 30           # sous ca l'appariement n'a rien a dire

LOOP_MIN_GAP = 40          # scans d'ecart minimum pour parler de retour sur zone
LOOP_MAX_DIST_M = 1.2      # au-dela les deux scans ne voient pas la meme chose

# Un lien de fermeture faux est pire que pas de lien : il tire toute la
# trajectoire vers une position inventee. D'ou un double rejet, sur le residu
# ET sur la part de points qui trouvent un correspondant.
ACCEPT_MIN_FIT = 0.55
ACCEPT_MAX_RES_M = 0.05

# omega=1.9 est la valeur qui compte : a 1.0 l'ecart avec scipy est de 5,7 mm
# en 18,5 s, a 1.9 il est de 0,07 mm en 4,4 s. Le meme optimum sort sur un
# graphe synthetique et sur les vrais runs.
OMEGA = 1.9
SWEEPS = 400
HUBER_M = 0.10             # au-dela, un lien pese moins -- pare-fou anti-faux-lien
DAMPING = 1e-6
TOL_M = 1e-5

MIN_SCANS = 80             # sous ca il n'y a pas de boucle a fermer
MAX_SCANS = 1500           # garde-fou : le cout croit avec les candidats


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def relative(a, b):
    """Pose de b vue depuis a."""
    ca, sa = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (ca * dx + sa * dy, -sa * dx + ca * dy, wrap(b[2] - a[2]))


class _Grid:
    """Points ranges par case de cote `pas`, pour un plus-proche-voisin borne."""

    __slots__ = ("cases", "pas")

    def __init__(self, points, pas):
        self.pas = pas
        self.cases = {}
        inv = 1.0 / pas
        for k, (x, y) in enumerate(points):
            c = (math.floor(x * inv), math.floor(y * inv))
            b = self.cases.get(c)
            if b is None:
                self.cases[c] = [(x, y, k)]
            else:
                b.append((x, y, k))

    def nearest(self, x, y, rmax):
        inv = 1.0 / self.pas
        cx, cy = math.floor(x * inv), math.floor(y * inv)
        best = rmax * rmax
        bi = -1
        get = self.cases.get
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
    """Amene `src` sur `dst`. Rend (x, y, th, residu median, part appariee)."""
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
    """Cholesky 3x3. Rend None plutot qu'une direction inventee si ca echoue."""
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
    """Relaxation de Gauss-Seidel par blocs. Rend la liste des poses corrigees."""
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
    """Chaque scan en nuage de points, dans le repere du robot.

    Le LIDAR est en arriere du centre de rotation : sans ce decalage le meme
    mur est stampe jusqu'a 206 mm plus loin selon le sens de marche.
    """
    out = []
    rad = math.pi / 180.0
    for scan in scans:
        pts = []
        for point in scan[3]:
            ang = point[0]
            dist = point[1] / 1000.0
            if dist <= 0.0 or dist > max_range_m:
                continue
            t = ang * rad
            pts.append((dist * math.cos(t) - lidar_behind_m, dist * math.sin(t)))
        out.append(pts)
    return out


def loop_candidates(poses, min_gap=LOOP_MIN_GAP, max_dist=LOOP_MAX_DIST_M):
    """Paires de scans proches dans l'espace et eloignees dans le temps.

    C'est la definition d'un retour sur zone : le robot repasse la ou il est
    deja alle, assez tard pour que sa pose ait derive entre-temps.
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
    """Rend les poses corrigees par fermeture de boucle, ou None.

    `scans` : liste de (x, y, theta_deg, points), alignee sur `poses`.
    `poses` : liste de (x, y, theta_rad) telles que le recalage scan-a-scan
              les a etablies.

    Rend None -- et dit pourquoi -- plutot que de rendre les poses d'entree :
    l'appelant doit pouvoir distinguer << rien a corriger >> de << corrige >>.
    """
    n = len(poses)
    if n < MIN_SCANS:
        _LOGGER.debug("SLAM: %d scans, trop peu pour fermer une boucle", n)
        return None
    if n > MAX_SCANS:
        _LOGGER.info("SLAM: %d scans, au-dela du garde-fou de %d", n, MAX_SCANS)
        return None

    cl = clouds(scans, max_range_m, lidar_behind_m)
    cands = loop_candidates(poses)
    if not cands:
        _LOGGER.info("SLAM: aucun retour sur zone sur %d scans", n)
        return None

    # Odometrie : la trajectoire telle que le recalage scan-a-scan l'a etablie.
    edges = [
        (k, k + 1, relative(poses[k], poses[k + 1]), 1.0)
        for k in range(n - 1)
    ]
    n_odo = len(edges)
    residus = []
    for a, b in cands:
        z0 = relative(poses[a], poses[b])
        x, y, th, res, fit = icp(cl[b], cl[a], z0[0], z0[1], z0[2])
        if fit < ACCEPT_MIN_FIT or res > ACCEPT_MAX_RES_M:
            continue
        # Poids : un accord serre pese plus qu'un accord limite.
        w = min(
            2.0,
            (fit / ACCEPT_MIN_FIT) * (ACCEPT_MAX_RES_M / max(res, 1e-3)) * 0.25,
        )
        edges.append((a, b, (x, y, th), w))
        residus.append(res)

    kept = len(edges) - n_odo
    if kept < n // 4:
        _LOGGER.info(
            "SLAM: %d fermetures retenues sur %d candidates, trop peu pour "
            "contraindre %d scans -- poses laissees telles quelles",
            kept, len(cands), n,
        )
        return None

    refined = optimise(poses, edges)
    moved = sorted(
        math.hypot(refined[k][0] - poses[k][0], refined[k][1] - poses[k][1])
        for k in range(n)
    )
    _LOGGER.info(
        "SLAM: %d fermetures sur %d candidates, residu median %.1f cm ; "
        "poses deplacees de %.1f cm en median, %.1f cm au pire",
        kept, len(cands), 100 * _median(residus),
        100 * moved[n // 2], 100 * moved[-1],
    )
    return refined
