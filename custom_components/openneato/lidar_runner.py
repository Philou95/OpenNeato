"""Drive LIDAR mapping across a cleaning run.

Watches the coordinator for the robot going out to clean, samples pose and
scan while it works, and folds the result into the accumulated map when it
docks. The map improves with every run instead of being rebuilt from one.

The polling budget is deliberately modest. Reading a scan holds the robot's
serial link for roughly 800 ms, and the firmware skips its own 2 s pose
snapshot while a fetch is in flight, so this watches the recording session
file's growth and backs off if the robot's own path log starts thinning.
Measured over a 49-minute run at a 4 s interval, that log stayed at 95-126%
of its normal rate.
"""

from __future__ import annotations

import json
import logging
import math
import time
from functools import partial
from pathlib import Path
from typing import Any

from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.event import async_track_time_interval
from homeassistant.helpers.storage import Store
from datetime import timedelta

from .api import OpenNeatoApiError
from .const import CELL_SIZE_M, DOMAIN
from .lidar_mapper import (
    SessionTracker,
    plan_calibration,
    AccumulatedMap,
    LIVE_MIN_MARGIN,
    LIVE_MIN_RATIO,
    MERGE_MIN_OVERLAP,
    MAX_MOVE_DURING_SCAN_M,
    MAX_TURN_DURING_SCAN_DEG,
    align_to_reference,
    build_session_grids,
    parse_pose,
    quarter_margin,
    MAX_CONSECUTIVE_REJECTS,
    RENDER_PX_PER_M,
    alignment_key,
    render_plan,
    scan_points,
)

_LOGGER = logging.getLogger(__name__)

STORAGE_VERSION = 1
POLL_INTERVAL = 4.0          # seconds between captures
# One run's scans, kept in the config directory so the mapper can be tried
# against real returns instead of synthetic ones -- see _dump_captures(). The
# file's own existence is the switch: it is written once and then never again,
# so this costs a single run. About 1.5 MB for a full cycle; the cap is only a
# runaway guard and a normal run is well under it.
# Entre deux sauvegardes des captures en cours. Voir _persist_captures().
CAPTURE_PERSIST_S = 300.0
# Batches collected per tick. The bridge sends a few scans at a time on
# purpose, so catching up after an outage is spread over several ticks rather
# than blocking one of them.
DRAIN_MAX_BATCHES = 6
# Empty answers in a row before deciding the bridge has stopped buffering.
#
# Generous on purpose. The bridge makes a scan every four to five seconds, this
# ticks at four, and a batch carries up to four -- so empty answers are the
# normal case, not a warning, and a run of five happens on its own. Set to five
# it fired within a minute of a healthy run and dropped back to sampling over
# HTTP, which is the very thing the buffer removes. A minute of true silence is
# a bridge that has stopped; anything less is two clocks sliding past.
DRAIN_QUIET_TICKS = 15
CAPTURE_DUMP_NAME = "openneato_captures.json"
CAPTURE_DUMP_MAX = 3000
# A scan is only geometry if the laser was actually sweeping. Rather than pin a
# nominal speed -- the robot reports 5.03 while docked and the field's unit is
# not documented -- each run is judged against its own median: anything under
# this fraction of it was taken while the LDS was spinning up, stalling or
# coasting, and its 360 "angles" were never swept in one revolution.
MIN_ROTATION_FRACTION = 0.6
MIN_CAPTURES = 60            # below this a run is too thin to be worth merging
BYTES_PER_POSE = 48          # firmware writes ~48 bytes per snapshot, every 2 s
MAX_INTERVAL = 12.0

# Health is judged over the window between two checks, and the control runs
# both ways.
#
# The firmware buffers pose lines and flushes to flash every 30 s, so a *short*
# window catches one flush or three depending on where it lands: over 60 s the
# same healthy robot read 97%, then 51%, then 142%. A 300 s window spans ten
# flushes, so that aliasing is down to a few percent and there is no longer any
# reason to average since the start -- which was the real problem. A cumulative
# average cannot recover: one bad stretch keeps it depressed for the rest of the
# run, so sampling ratcheted down 4 -> 6 -> 9 s and never came back even once
# the robot was logging perfectly again.
#
# Backing off is also partly self-inflicted, which is exactly why the loop has
# to close. Each LIDAR read holds the serial link for around 800 ms and the
# firmware skips its own pose snapshot while a fetch is in flight, so polling
# faster depresses the very number used to decide whether to poll faster.
# Slowing down raises the ratio, which then earns the speed back, and the loop
# settles at the fastest rate the robot can actually sustain.
#
# The check can only change the rate, never stop collection: the measurement is
# too coarse to justify throwing a run away.
HEALTH_EVERY = 300.0
HEALTH_GRACE = 600.0
# Placement de la session en cours sur la carte. Voir _align_live().
#
# 25 scans suffisent a designer le bon quart de tour sur les deux runs
# rejoues ; 40 laisse de la marge sans rien coûter, le premier essai tombant
# alors vers la troisieme minute de menage. On recommence ensuite au meme
# rythme que le controle de sante : l'ajustement coute une a cinq secondes
# d'executeur et fait sauter l'echantillonnage d'un tick, donc douze fois par
# heure est genereux pour un affichage.
LIVE_ALIGN_EVERY = 300.0
LIVE_ALIGN_MIN_SCANS = 40
# ...et on s'arrete des que la reponse se repete, parce qu'elle ne bouge plus.
# Rejoue sur les runs 8 et 9 : le quart de tour est le bon des le premier
# essai, la translation se pose au deuxieme (run 9) ou au troisieme (run 8),
# et les quatre a cinq essais suivants rendent exactement la meme chose.
# S'arreter la ramene le cout d'un run de 35 min de 23 s d'executeur a 7-11 s,
# et un run dont l'ajustement continue de bouger continue d'etre ajuste.
LIVE_ALIGN_STABLE = 2
# ...mais seulement une fois que la session recouvre assez la carte pour que la
# translation veuille dire quelque chose.
#
# ⚠ Deux lectures identiques a cinq minutes d'intervalle ne prouvent pas la
# convergence : elles prouvent que ca n'a pas bouge en cinq minutes. Le
# 31/08/2026 le placement s'est fige sur dy = -5 alors que la fusion allait
# choisir +3 -- huit cellules, 20 cm, un demi-passage de brosse -- et n'y est
# jamais revenu : la zone nettoyee est restee dessinee a cote de ses murs
# pendant tout le reste du menage. La regle avait ete validee sur deux runs,
# et elle generalisait depuis n=2.
#
# Le recouvrement dit quand la translation est determinee, et il le dit sur
# n'importe quel run. Mesure sur le menage du 30/08, ajustement partiel rejoue
# tous les 25 scans :
#
#     scans  50   150   200   300   668
#     recouv 0.38 0.48  0.55  0.72  0.90
#     dy     +1   +2    +3    +3    +3      <- se pose a 0.55
#
# Sous ce seuil la session est un morceau de maison qui glisse le long des murs
# de la carte sans que le score en souffre. C'est le meme seuil que la fusion
# exige avant de croire un ajustement, et pour la meme raison : en dessous,
# l'ajustement n'est pas une mesure.
LIVE_ALIGN_MIN_OVERLAP = MERGE_MIN_OVERLAP
BACKOFF_RATIO = 0.55
RECOVER_RATIO = 0.80   # hysteresis band: below 0.55 slow down, above 0.80 speed up

# uiState substrings, matching camera.py.
CLEANING = ("CLEANINGRUNNING", "CLEANINGPAUSED", "CLEANINGSUSPENDED", "DOCKING")
ACTIVE = ("CLEANINGRUNNING",)
# Modes whose geometry must never reach the accumulated map.
#
# These are substring tests, and "CLEANINGRUNNING" is contained in
# "UIMGR_STATE_MANUALCLEANINGRUNNING" -- so a manual clean has been feeding the
# map all along, without anyone asking for it. Philou drove the robot by hand
# on 2026-08-26 and bumped it into walls several times; the recorded track ran
# 1.5 m through a wall the map puts at 1.35 m, because the wheels turned while
# the chassis did not follow. Odometry taken while a human steers is not
# evidence about where the walls are, and a map is only worth what its worst
# session put in it.
#
# The robot's own pose journal still records the run, so the replay is
# unaffected -- this only keeps it out of the accumulated geometry.
UNMAPPABLE = ("MANUALCLEANING",)
# Modes worth replaying on the map but not worth putting into it. A spot clean
# is driven by the robot, so its track is trustworthy and belongs on screen --
# but a square metre of walls teaches a whole-house map nothing, and its small
# footprint naturally overlaps poorly, which would otherwise count as a failed
# merge. Three of those in a row discard the map.
NO_CONTRIBUTION = ("SPOTCLEANING",)


def _mappable(state: str) -> bool:
    """True while the robot is laying down geometry worth keeping."""
    return (
        any(s in state for s in CLEANING)
        and not any(m in state for m in UNMAPPABLE)
    )


def _parse_scans(text: str) -> tuple[list[tuple], int]:
    """NDJSON from the bridge -> capture tuples. CPU-bound; run in the executor.

    Returns the scans worth keeping and the highest sequence number seen --
    including the ones dropped for smear, or the bridge would resend them for
    ever.
    """
    out: list[tuple] = []
    high = 0
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        high = max(high, int(rec.get("seq", 0)))
        moved = float(rec.get("mv", 0.0))
        turned = float(rec.get("tn", 0.0))
        if moved > MAX_MOVE_DURING_SCAN_M or turned > MAX_TURN_DURING_SCAN_DEG:
            # Smeared across two positions: worse than no scan at all, because
            # it lays walls that were never there.
            continue
        points = [(a, v) for a, v in enumerate(rec.get("d") or []) if v]
        if not points:
            continue
        out.append((
            float(rec.get("x", 0.0)), float(rec.get("y", 0.0)),
            float(rec.get("t", 0.0)), points, float(rec.get("rpm", 0.0)),
            moved, turned,
        ))
    return out, high


class LidarMapRunner:
    """Collects scans during a clean and maintains the accumulated map."""

    def __init__(self, hass: HomeAssistant, entry_id: str, api, coordinator) -> None:
        self.hass = hass
        self.entry_id = entry_id
        self.api = api
        self.coordinator = coordinator
        self._store = Store(hass, STORAGE_VERSION, f"{DOMAIN}_{entry_id}_lidar_map")
        # Les captures d'un run en cours, pour qu'un redemarrage de Home
        # Assistant ne les emporte pas : elles ne vivaient qu'en memoire.
        self._cap_store = Store(hass, STORAGE_VERSION, f"{DOMAIN}_{entry_id}_lidar_captures")
        # Une copie de la carte juste avant une fusion qui la remplacerait.
        self._bak_store = Store(hass, STORAGE_VERSION, f"{DOMAIN}_{entry_id}_lidar_map_backup")
        self._pending_captures: list | None = None
        self._pending_session: str | None = None
        self._last_persist = 0.0
        self._map: AccumulatedMap | None = None
        self._captures: list[tuple[float, float, float, list[tuple[int, int]]]] = []
        # Ferme les boucles au fil du menage plutot qu'en bloc a la fin. Le
        # nombre de paires candidates croit comme le carre du nombre de scans,
        # donc l'etaler transforme un cout quadratique en fin de course en un
        # cout constant par scan -- 5 min 11 s de fusion mesurees sur le
        # Raspberry Pi 5, contre une vingtaine de secondes ainsi.
        self._tracker: SessionTracker | None = None
        self._tracked = 0
        self._tracking = False
        # Ou la session en cours se pose sur la carte, avant que la fusion ne
        # le sache. Voir _align_live().
        self._live_align: tuple[int, int, int, float, float, float, float] | None = None
        self._live_align_at = 0.0
        self._live_stable = 0
        self._aligning = False
        self._collecting = False
        self._unsub_timer = None
        self._unsub_coordinator = None
        self._interval = POLL_INTERVAL
        self._busy = False
        self._last_health = 0.0
        # Collection start, kept apart from the health window anchor: the grace
        # period is about how long the run has been going, not how long since
        # the last check.
        self._collect_start = 0.0
        self._health_start = 0.0
        self._health_ref: dict[str, Any] | None = None
        self._session_name: str | None = None
        self._contributes = True
        # Collecting from the bridge's own buffer: None until we find out,
        # False on a bridge too old to have one.
        self._buffer_ok: bool | None = None
        self._last_seq = 0
        self._empty_drains = 0
        self.last_report: dict[str, Any] = {}

    async def async_load(self) -> None:
        """Restore the accumulated map from storage."""
        data = await self._store.async_load()
        self._map = AccumulatedMap(data)
        if self._map.sessions:
            _LOGGER.info(
                "LIDAR map restored: %d wall cells from %d cleanings",
                len(self._map.walls), self._map.sessions,
            )
        saved = await self._cap_store.async_load()
        if saved and saved.get("captures"):
            self._pending_captures = saved["captures"]
            self._pending_session = saved.get("session")
            _LOGGER.info(
                "LIDAR mapping: %d scans recovered from an interrupted run (%s)",
                len(self._pending_captures), self._pending_session,
            )
        self._unsub_coordinator = self.coordinator.async_add_listener(self._handle_update)

    @callback
    def async_unload(self) -> None:
        self._stop_timer()
        if self._unsub_coordinator:
            self._unsub_coordinator()
            self._unsub_coordinator = None

    # ── lifecycle ───────────────────────────────────────────────────

    @callback
    def _handle_update(self) -> None:
        state = ((self.coordinator.data or {}).get("state") or {}).get("uiState", "")
        cleaning = _mappable(state)
        if cleaning and not self._collecting:
            self._start()
        elif not cleaning and self._collecting:
            self.hass.async_create_task(self._finish())

    def _start(self) -> None:
        state = ((self.coordinator.data or {}).get("state") or {}).get("uiState", "")
        # Decided at the start: by the end the robot says DOCKING and the mode
        # it was cleaning in is no longer readable anywhere.
        self._contributes = not any(m in state for m in NO_CONTRIBUTION)
        self._collecting = True
        self._captures = []
        self._last_persist = time.monotonic()
        self._interval = POLL_INTERVAL
        self._last_health = time.monotonic()
        self._collect_start = time.monotonic()
        self._health_start = time.monotonic()
        self._health_ref = self._recording_session()
        self._session_name = None
        self._note_session_name()
        # Meme session qu'un run que Home Assistant a interrompu : on reprend
        # ou on en etait au lieu de repartir de zero. Le nom vient du fichier
        # que le robot est en train d'ecrire, donc l'egalite suffit a dire que
        # c'est le meme nettoyage et pas le suivant.
        if self._pending_captures and self._pending_session == self._session_name:
            self._captures = [tuple(c) for c in self._pending_captures]
            _LOGGER.info(
                "LIDAR mapping: resuming %s with %d scans already collected",
                self._session_name, len(self._captures),
            )
        elif self._pending_captures:
            _LOGGER.info(
                "LIDAR mapping: dropping %d scans from %s — this is a different run",
                len(self._pending_captures), self._pending_session,
            )
        self._pending_captures = None
        self._pending_session = None
        # Cree apres la reprise eventuelle : le suiveur absorbera les captures
        # restaurees d'un bloc au premier drain, ce qui est exactement ce qu'il
        # aurait fait si elles etaient arrivees une a une.
        self._tracker = SessionTracker()
        self._tracked = 0
        self._tracking = False
        # Remis a zero ici et nulle part ailleurs. Une fusion refusee ne place
        # la session nulle part, et le placement provisoire reste alors le seul
        # que la carte ait pour ce run : le garder vaut mieux que revenir au
        # repere brut du robot.
        self._live_align = None
        self._live_align_at = 0.0
        self._live_stable = 0
        self._start_timer()
        _LOGGER.info("LIDAR mapping: collection started")

    def _start_timer(self) -> None:
        self._stop_timer()
        self._unsub_timer = async_track_time_interval(
            self.hass, self._async_tick, timedelta(seconds=self._interval)
        )

    def _stop_timer(self) -> None:
        if self._unsub_timer:
            self._unsub_timer()
            self._unsub_timer = None

    # ── sampling ────────────────────────────────────────────────────

    async def _track_new(self) -> None:
        """Ferme les boucles des scans arrives depuis le dernier passage.

        Le curseur avance AVANT l'attente : un tick concurrent voit alors une
        tranche vide au lieu de refaire le meme travail. Et si l'executeur
        echoue, le suiveur est abandonne plutot que laisse incomplet -- un
        suiveur a trous rendrait des poses fausses, pas approximatives, et la
        fusion retombe sur le calcul de fin qui, lui, est correct.

        `_aligning` exclut l'autre travail qui touche au suiveur : _align_live()
        copie `_scratch`, et copier un dictionnaire qu'un autre fil est en train
        de remplir leve une RuntimeError. Les deux drapeaux ne sont poses et
        lus que dans la boucle d'evenements, donc l'exclusion est stricte meme
        si le travail lui-meme est dans l'executeur.
        """
        if self._tracker is None or self._tracking or self._aligning:
            return
        new = self._captures[self._tracked:]
        if not new:
            return
        self._tracked = len(self._captures)
        self._tracking = True
        try:
            await self.hass.async_add_executor_job(self._tracker.add_many, new)
        except Exception:
            _LOGGER.exception(
                "SLAM: suivi au fil de l'eau abandonne — la fusion refera le "
                "calcul en fin de run"
            )
            self._tracker = None
        finally:
            self._tracking = False

    @staticmethod
    def _fit_live(tracker: SessionTracker, ref_walls: dict[tuple[int, int], int]):
        """Ajuste les murs vus jusqu'ici sur la carte. Tourne dans l'executeur.

        Prend ce sur quoi il travaille en argument plutot que de le lire sur
        `self` : la fin du menage peut tomber pendant l'attente et remettre le
        suiveur a None, et le fil de l'executeur trouverait alors un attribut
        vide au lieu du travail qu'on lui a confie.
        """
        walls = tracker.walls_so_far()
        if not walls:
            return None
        return align_to_reference(walls, ref_walls)

    async def _align_live(self) -> bool:
        """Place la session en cours sur la carte, sans attendre la fusion.

        Le repere du robot tourne d'un quart de tour d'un menage a l'autre --
        il suit la direction ou le robot se cale en sortant du dock, et sur
        certains cycles il fait un quart de tour de plus avant de commencer.
        La fusion le rattrape et la carte n'en souffre pas ; c'est l'affichage
        qui trinque, la session etant servie dans le repere brut jusqu'a la
        fusion. La zone nettoyee apparait alors en travers des murs pendant
        toute l'heure du menage.

        La rotation, elle, est fixee des le depart : il n'y a donc rien a
        attendre. Rejoue sur les runs 8 et 9 du 27/08, le bon quart de tour se
        detache des 25 scans et ne change plus ensuite.

        Le placement est refait a chaque LIVE_ALIGN_EVERY plutot que fige au
        premier succes : le quart de tour ne bouge pas, mais la translation se
        deplace encore de quelques cellules le temps que le run couvre assez de
        terrain. On s'arrete quand la reponse se repete, pas apres un nombre
        d'essais decide d'avance -- voir LIVE_ALIGN_STABLE.

        Rend True si ce tick a servi a ca -- l'echantillonnage saute alors son
        tour, ce qui coute un scan sur les quelque neuf cents d'un menage.
        """
        if (
            self._aligning
            or self._tracking
            or self._live_stable >= LIVE_ALIGN_STABLE
            or self._tracker is None
            or self._map is None
            # Rien a quoi se raccrocher : la toute premiere carte est ce
            # premier run, dans son propre repere, et il n'y a pas de travers.
            or not self._map.walls
            # Le placement est range sous le nom du fichier de session ; sans
            # lui la carte de rejeu ne saurait pas a quoi il se rapporte.
            or not self._session_name
            or self._tracker.placed < LIVE_ALIGN_MIN_SCANS
        ):
            return False
        now = time.monotonic()
        if now - self._live_align_at < LIVE_ALIGN_EVERY:
            return False
        self._live_align_at = now
        self._aligning = True
        scans = self._tracker.placed
        try:
            fit = await self.hass.async_add_executor_job(
                self._fit_live, self._tracker, self._map.walls
            )
        except Exception as err:  # noqa: BLE001 -- un affichage ne coute pas un run
            _LOGGER.debug("LIDAR mapping: placement provisoire echoue (%s)", err)
            return True
        finally:
            self._aligning = False
        if fit is None:
            return True

        quarter, dx, dy, overlap, fine, scores = fit
        margin, ratio = quarter_margin(scores, quarter)
        if margin < LIVE_MIN_MARGIN or ratio < LIVE_MIN_RATIO:
            # Pas assez tranche pour valoir mieux que le repere brut. Rien de
            # perdu : le prochain essai aura vu plus de terrain.
            _LOGGER.debug(
                "LIDAR mapping: quart de tour indecis apres %d scans "
                "(%s), placement provisoire reporte",
                scans,
                " ".join(f"q{q}={s:.2f}" for q, s in enumerate(scores)),
            )
            return True

        # La meme correction que la fusion range avec l'alignement, et pour la
        # meme raison : les murs sont projetes depuis des poses que le recalage
        # a deja deplacees, le trajet rejoue vient du journal brut du robot.
        cx, cy, cth = (
            self._tracker.correction if self._tracker else (0.0, 0.0, 0.0)
        )
        placement = (
            quarter, dx, dy, fine, cx / CELL_SIZE_M, cy / CELL_SIZE_M, cth,
        )
        first = self._live_align is None
        turned = not first and self._live_align[0] != quarter
        # La correction du recalage bouge de quelques millimetres a chaque
        # scan et ne se repeterait jamais : c'est la pose sur la carte qu'on
        # regarde, pas elle.
        #
        # Et une repetition ne compte pas tant que la session ne recouvre pas
        # assez la carte : sous LIVE_ALIGN_MIN_OVERLAP la translation peut
        # tenir en place cinq minutes durant et se tromper quand meme, faute
        # d'assez de terrain pour la contraindre. Voir la constante.
        if overlap < LIVE_ALIGN_MIN_OVERLAP:
            self._live_stable = 0
        elif not first and self._live_align[:4] == placement[:4]:
            self._live_stable += 1
        else:
            self._live_stable = 1
        self._live_align = placement
        if first or turned:
            _LOGGER.info(
                "LIDAR mapping: session en cours placee sur la carte apres %d "
                "scans — quart %d, decalage (%+d,%+d), %.1f deg ; recouvrement "
                "%.0f%%, marge %.2f (%.1fx)",
                scans, quarter, dx, dy, fine, 100 * overlap, margin, ratio,
            )
        else:
            _LOGGER.debug(
                "LIDAR mapping: placement provisoire revu apres %d scans — "
                "quart %d, decalage (%+d,%+d), recouvrement %.0f%%",
                scans, quarter, dx, dy, 100 * overlap,
            )
        return True

    async def _async_tick(self, _now=None) -> None:
        if self._busy:
            return
        # Avant tout chemin qui peut sortir tot : c'est du travail qui doit se
        # faire a chaque tick, quel que soit l'etat du robot.
        await self._track_new()
        state = ((self.coordinator.data or {}).get("state") or {}).get("uiState", "")
        if not any(s in state for s in ACTIVE) or any(m in state for m in UNMAPPABLE):
            # Paused, recharging, or heading for the dock: no new floor is
            # being laid, so spend nothing on the serial link.
            #
            # Re-anchor the health window while it lasts. A mid-clean recharge
            # can hold the robot on its base for half an hour, and the firmware
            # stops writing poses for the whole of it -- so the growth this
            # check measures collapses to nothing through no fault of the
            # serial link. Left alone it would read that as a sick robot and
            # ratchet the sampling out to MAX_INTERVAL, then take a window per
            # step to climb back: a quarter of an hour of thin sampling on
            # floor the robot is cleaning perfectly well. Counting only the
            # time the robot was actually cleaning is what the measurement
            # meant in the first place.
            self._health_ref = self._recording_session() or self._health_ref
            self._health_start = time.monotonic()
            self._collect_start = max(
                self._collect_start, time.monotonic() - HEALTH_GRACE
            )
            return
        # Before the sampling block, and deliberately so: both of these used to
        # sit after it, and collecting from the bridge's buffer returns early
        # from the try -- so the moment that path started working, the session
        # name was never learned and the health window never advanced. The run
        # of 2026-08-27 merged with no alignment stored at all, which is what
        # draws a replay a quarter turn off the walls.
        self._note_session_name()
        if time.monotonic() - self._last_health >= HEALTH_EVERY:
            self._check_health()

        self._busy = True
        try:
            # Avant l'echantillonnage, et sous le meme drapeau : l'ajustement
            # tient l'executeur une a cinq secondes, et un tick qui echantillonne
            # pendant ce temps-la mettrait deux lectures serie en parallele.
            if await self._align_live():
                return
            # Prefer what the bridge kept for us. It samples on its own loop
            # while cleaning, so a WiFi outage no longer costs the scans taken
            # during it -- and there is no HTTP round trip per scan competing
            # with the firmware's own pose journal for the serial link.
            # Once the bridge is known to buffer, its silence is just "nothing
            # new yet" -- it samples on its own clock, and this tick asking the
            # same question again over HTTP is the round trip the buffer exists
            # to remove. Sampling on every empty answer put *both* paths on the
            # serial link at once and left /api/lidar timing out mid-run, which
            # is the opposite of the point.
            #
            # An unknown bridge still falls through, and a bridge that goes
            # quiet for several ticks is treated as one that has stopped
            # buffering, so a whole run can never be lost to a silent peer.
            if self._buffer_ok is not False:
                got = await self._drain_buffer()
                if got:
                    self._empty_drains = 0
                    return
                if got == 0 and self._buffer_ok:
                    self._empty_drains += 1
                    if self._empty_drains < DRAIN_QUIET_TICKS:
                        return
                    _LOGGER.warning(
                        "LIDAR mapping: the bridge has sent nothing for %d ticks — "
                        "sampling directly again", self._empty_drains,
                    )
                    self._buffer_ok = None
                    self._empty_drains = 0
            # Pose first -- the scan read is the slow half, so this timestamp
            # sits closest to the scan's own instant.
            raw = await self.api.send_serial_command("GetRobotPos Smooth")
            pose = parse_pose(str(raw))
            payload = await self.api.get_lidar()
            points = scan_points(payload)
            # Pose again, after the scan. A scan is a ~800 ms serial round
            # trip, so the pose taken before it is that far stale by the time
            # the beam data actually lands; pairing them projects every return
            # from where the robot *was*. Bracketing the scan gives both ends,
            # and the midpoint is the honest estimate of where it was mid-scan.
            raw_after = await self.api.send_serial_command("GetRobotPos Smooth")
            pose_after = parse_pose(str(raw_after))
            if pose and points:
                x, y, theta, _ts = pose
                moved = turned = 0.0
                if pose_after:
                    x2, y2, theta2, _ts2 = pose_after
                    moved = math.hypot(x2 - x, y2 - y)
                    turned = abs((theta2 - theta + 180.0) % 360.0 - 180.0)
                    if moved > MAX_MOVE_DURING_SCAN_M or turned > MAX_TURN_DURING_SCAN_DEG:
                        # The frame shifted under the beam. One scan smeared
                        # across two positions is worse than no scan at all,
                        # because it lays walls that were never there.
                        _LOGGER.debug(
                            "LIDAR mapping: scan dropped, robot moved %.2f m / %.0f deg during it",
                            moved, turned,
                        )
                        return
                    x = (x + x2) / 2.0
                    y = (y + y2) / 2.0
                    # Half the shortest-arc delta -- the parentheses matter:
                    # `%` binds tighter than `+`, so without them this lands on
                    # theta2 instead of between the two.
                    theta = theta + ((theta2 - theta + 180.0) % 360.0 - 180.0) / 2.0
                self._captures.append(
                    (
                        x,
                        y,
                        theta,
                        points,
                        float(payload.get("rotationSpeed") or 0.0),
                        # Carried so the grid builder can weigh this scan by how
                        # still the robot actually was, instead of treating a
                        # scan taken mid-turn as being worth as much as one
                        # taken standing still.
                        moved,
                        turned,
                        # Link quality where the robot was standing, read from
                        # the coordinator's own poll so it costs no extra
                        # request. The bridge rides on the robot, so this is a
                        # radio survey of the house taken as it cleans.
                        #
                        # Here because of 2026-08-26: one 39 s hole in the
                        # robot's pose log cost 1.6 m of unpainted floor, and
                        # the two worst scan-fetch stalls of that run -- 24 s
                        # and 69 s against a 4 s median -- sat either side of
                        # it. Philou reads the far end of the room as barely
                        # reachable even while the link stays up. It did not
                        # happen again the next run, and the robot's own error
                        # count and brush-stall share were identical across
                        # both, so nothing about the robot explains it.
                        # Without this the next occurrence is just as mute.
                        self._rssi(),
                        self._heap(),
                    )
                )
                await self._persist_captures()
        except Exception as err:  # noqa: BLE001 -- one bad read must never end a run
            _LOGGER.debug("LIDAR mapping: sample failed (%s)", err)
        finally:
            self._busy = False



    def _note_session_name(self) -> None:
        """Learn which file this run is writing to, retrying until it is known.

        The alignment worked out at merge time is stored against this name so
        the card can replay the run in the same frame as the map. It cannot be
        read at the end -- by then the robot no longer reports the file as
        recording -- but reading it once at the start does not work either: the
        coordinator refreshes its history every 30 s, so at the moment
        collection begins its cached copy usually predates the session file and
        the name comes back empty. It worked by timing luck often enough to
        look fine, and when it missed the run was merged with no alignment
        stored and the card drew it a quarter turn off the walls.

        So ask on every tick until the answer arrives, then stop asking. This
        reads the coordinator's cache, not the robot, so it costs nothing.
        """
        if self._session_name:
            return
        name = (self._recording_session() or {}).get("name")
        if name:
            self._session_name = name
            _LOGGER.debug("LIDAR mapping: session file is %s", name)

    def _recording_session(self) -> dict[str, Any] | None:
        for item in (self.coordinator.data or {}).get("history") or ():
            if isinstance(item, dict) and item.get("recording"):
                return item
        return None

    def _check_health(self) -> None:
        """Slow sampling down if the robot's own pose logging is thinning out.

        Compares the recording file's growth against the firmware's 2 s cadence
        over the window since the last check -- long enough that the 30 s flush
        granularity cannot fake a collapse, short enough that a recovery is
        visible. It never stops collection: the measurement is too coarse to
        justify throwing a run away.
        """
        self._last_health = time.monotonic()
        cur = self._recording_session()
        if not cur or not self._health_ref:
            self._health_ref = cur or self._health_ref
            return
        if cur.get("name") != self._health_ref.get("name"):
            # A new file means a new session; re-anchor rather than compare.
            self._health_ref = cur
            self._health_start = time.monotonic()
            return

        now = time.monotonic()
        window = now - self._health_start
        grown = cur.get("size", 0) - self._health_ref.get("size", 0)
        # Re-anchor whatever happens next, so the following window measures the
        # rate that this window's decision produces.
        self._health_ref = cur
        self._health_start = now

        if now - self._collect_start < HEALTH_GRACE:
            return
        expected = window / 2.0 * BYTES_PER_POSE
        if expected <= 0:
            return
        ratio = grown / expected

        before = self._interval
        if grown <= 0:
            # The robot logged nothing at all this window, and that is a
            # different thing from logging thinly. This check exists on the
            # premise that thin logging means our polling is stealing serial
            # bandwidth, so easing off gives it back -- at exactly zero the
            # robot has stopped writing for a reason easing off cannot touch,
            # and the run loses the rest of its resolution for nothing.
            #
            # It is also worth saying out loud. On 2026-08-25 three such
            # windows were the only warning that the bridge's own collection
            # had latched on a serial reply that never came: the session wrote
            # its last pose at 20:47 and sat open, still flagged as recording,
            # 35 minutes later. Reading that as a quiet hold cost the evidence.
            _LOGGER.warning(
                "LIDAR mapping: the robot logged no pose at all over %.0fs — "
                "holding at %.0fs. Its own collection may have stalled.",
                window, self._interval,
            )
            return
        if ratio < BACKOFF_RATIO:
            self._interval = min(MAX_INTERVAL, self._interval * 1.5)
        elif ratio > RECOVER_RATIO:
            self._interval = max(POLL_INTERVAL, self._interval / 1.5)

        if self._interval != before:
            _LOGGER.info(
                "LIDAR mapping: robot pose logging at %.0f%% of normal over %.0fs; "
                "sampling every %.0fs (was %.0fs, %d captures so far)",
                100 * ratio, window, self._interval, before, len(self._captures),
            )
            self._start_timer()
        else:
            _LOGGER.debug(
                "LIDAR mapping: pose logging at %.0f%% over %.0fs, holding %.0fs, %d captures",
                100 * ratio, window, self._interval, len(self._captures),
            )

    # ── completion ──────────────────────────────────────────────────

    async def _finish(self) -> None:
        self._collecting = False
        self._stop_timer()
        captures, self._captures = self._captures, []
        if len(captures) < MIN_CAPTURES:
            _LOGGER.info(
                "LIDAR mapping: only %d captures, too thin to merge", len(captures)
            )
            return

        before_filter = len(captures)
        captures = self._drop_slow_scans(captures)
        dropped = before_filter - len(captures)
        if len(captures) < MIN_CAPTURES:
            _LOGGER.info(
                "LIDAR mapping: only %d captures left after the rotation filter, "
                "too thin to merge", len(captures),
            )
            return

        self._log_link_quality(captures)
        await self.hass.async_add_executor_job(self._dump_captures, captures)

        # Le suiveur n'est valable que si le filtre de rotation n'a rien
        # retire : enlever un scan changerait la pose de tous les suivants,
        # puisque match_pose s'accumule. Cas rare -- une fois sur neuf runs,
        # deux scans sur 501 -- et on le dit au lieu de le taire.
        tracker, self._tracker = self._tracker, None
        if tracker is not None and tracker.refused:
            # Une poignee est normale ; une grosse part dit que le menage s'est
            # passe la ou le recalage ne voit rien -- un couloir, ou la position
            # le long du couloir n'est pas observable. Ca se lit dans le log au
            # lieu de se deviner en rejouant les captures.
            _LOGGER.info(
                "LIDAR mapping: %d scans sur %d laisses a l'odometrie, la "
                "recherche de recalage butait sur le bord de sa fenetre",
                tracker.refused, tracker.placed,
            )
        if tracker is not None and not tracker.usable(dropped):
            _LOGGER.info(
                "SLAM: le filtre de rotation a retire %d scans, le suivi au fil "
                "de l'eau ne vaut plus — recalcul complet", dropped,
            )
            tracker = None
        walls, floor, free, correction = await self.hass.async_add_executor_job(
            # Whole captures now, not c[:4]: the builder weighs each scan by
            # the movement measured during it, which lives in the trailing
            # fields the truncation used to throw away.
            #
            # refine=True : fermeture de boucle sur les poses avant la
            # projection. Elle coute ~85 s dans l'executeur a la fin d'un
            # menage d'une heure, et fait passer le recouvrement de fusion de
            # 67,8 % a 85,3 % sur le run le plus derive, de 65,0 a 74,8 sur
            # celui du 27/08. Sous MERGE_MIN_OVERLAP la session est refusee et
            # trois refus effacent la carte : ca n'achete pas seulement de la
            # nettete, ca eloigne la carte du bord.
            partial(build_session_grids, captures, refine=True, tracker=tracker)
        )
        # La carte telle qu'elle est avant la fusion. merge_session peut la
        # jeter entierement -- au troisieme refus d'affilee il considere que
        # c'est elle qui ne correspond plus a la realite -- et cette decision
        # est irreversible une fois ecrite. Cinq nettoyages de murs accumules
        # meritent une copie avant d'etre effaces sur un jugement automatique.
        before = self._map.as_dict()
        report = await self.hass.async_add_executor_job(
            self._map.merge_session, walls, floor, self._session_name, free,
            correction, self._contributes,
        )
        self.last_report = report
        if report.get("rejected"):
            return

        if report.get("reset"):
            await self._bak_store.async_save(before)
            _LOGGER.warning(
                "LIDAR map: the stored map was discarded — a copy of the "
                "previous one (%d cells, %d cleanings) is in %s_lidar_map_backup",
                len(before.get("walls") or {}), before.get("sessions", 0), DOMAIN,
            )

        await self._store.async_save(self._map.as_dict())
        # Integrees : la copie de travail n'a plus de raison d'etre, et la
        # laisser ferait reprendre un run deja fusionne au prochain demarrage.
        await self._cap_store.async_remove()
        # The carve counts are the only sign free-space evidence did anything;
        # without them a chair fading off the map looks like nothing happened.
        if not report.get("contributed", True):
            _LOGGER.info(
                "LIDAR map: %s placed on the map but not merged into it — "
                "%d wall cells left untouched after %d cleanings",
                self._session_name, report.get("total_walls", 0),
                report.get("sessions", 0),
            )
            return

        _LOGGER.info(
            "LIDAR map updated: %d wall cells after %d cleanings "
            "(%d cells known empty, %d weakened, %d faded out)",
            report.get("total_walls", 0), report.get("sessions", 0),
            report.get("carved", 0), report.get("weakened", 0),
            report.get("faded", 0),
        )

    @staticmethod
    def _log_link_quality(captures: list) -> None:
        """Say how well the bridge was reachable while it worked.

        The radio is the one thing that has cost a run resolution without
        leaving any trace of itself: the pose log simply stops. Stating the
        spread here means a bad run says so at the time, instead of being
        reconstructed from a capture dump days later.
        """
        heaps = sorted(c[8] for c in captures if len(c) > 8 and c[8])
        if len(heaps) >= 5:
            _LOGGER.info(
                "LIDAR mapping: memoire du pont sur le run — median %.0f Ko, "
                "au plus bas %.0f Ko",
                heaps[len(heaps) // 2] / 1024, heaps[0] / 1024,
            )
        vals = [c[7] for c in captures if len(c) > 7 and c[7]]
        if len(vals) < 5:
            return
        vals.sort()
        worst = vals[0]
        median = vals[len(vals) // 2]
        # -75 dBm is where a 2.4 GHz link starts retransmitting in earnest.
        weak = sum(1 for v in vals if v <= -75)
        _LOGGER.info(
            "LIDAR mapping: link over the run — median %.0f dBm, worst %.0f, "
            "%d of %d scans below -75",
            median, worst, weak, len(vals),
        )

    async def _persist_captures(self) -> None:
        """Poser les captures sur le disque de temps en temps.

        Elles ne vivaient qu'en memoire : un redemarrage de Home Assistant au
        milieu d'un nettoyage emportait l'heure de collecte, et le run ne
        laissait aucune trace dans la carte. Le firmware ne conserve jamais un
        scan, donc rien ne pouvait le rattraper apres coup.

        Espacees, parce qu'un run pese pres de deux megaoctets et que Home
        Assistant ecrit souvent sur une carte SD. Le pire cas devient quelques
        minutes de scans perdus au lieu de la totalite.
        """
        now = time.monotonic()
        if now - self._last_persist < CAPTURE_PERSIST_S:
            return
        self._last_persist = now
        payload = {"session": self._session_name, "captures": self._captures}
        self._cap_store.async_delay_save(lambda: payload, 1.0)

    async def _drain_buffer(self) -> int | None:
        """Collect the scans the bridge buffered. None if it has no buffer.

        Repeats until the bridge says it has nothing more, bounded so one tick
        cannot run away: after a long outage there may be dozens waiting, and
        catching up over several ticks is fine.
        """
        total = 0
        for _ in range(DRAIN_MAX_BATCHES):
            try:
                text = await self.api.get_lidar_buffer(self._last_seq)
            except OpenNeatoApiError as err:
                if "404" not in str(err):
                    raise
                if self._buffer_ok is None:
                    _LOGGER.info(
                        "LIDAR mapping: this bridge has no scan buffer — sampling directly"
                    )
                self._buffer_ok = False
                return None
            if not text.strip():
                break
            # A bridge without the buffer does not answer 404: ESPAsyncWebServer
            # matches by prefix, so /api/lidar swallows this path and returns a
            # single scan object with 200. Detected by shape rather than by
            # status -- otherwise the fallback never fires and mapping quietly
            # collects nothing at all.
            if '"seq"' not in text:
                if self._buffer_ok is None:
                    _LOGGER.info(
                        "LIDAR mapping: this bridge has no scan buffer — sampling directly"
                    )
                self._buffer_ok = False
                return None
            # Parsed off the event loop. Each record carries 360 distances, and
            # a catch-up tick can bring two dozen of them: building those lists
            # inline is CPU work in the middle of Home Assistant's loop, which
            # is exactly what makes the rest of the house feel slow.
            scans, high = await self.hass.async_add_executor_job(_parse_scans, text)
            self._last_seq = max(self._last_seq, high)
            for x, y, t, points, rpm, moved, turned in scans:
                self._captures.append(
                    (x, y, t, points, rpm, moved, turned, self._rssi(), self._heap())
                )
                total += 1
                if self._buffer_ok is None:
                    self._buffer_ok = True
                    _LOGGER.info(
                        "LIDAR mapping: collecting from the bridge's own buffer"
                    )
        if total:
            await self._persist_captures()
        return total

    def _rssi(self) -> float:
        """Signal strength from the coordinator's last poll, or 0 if unknown."""
        try:
            return float(((self.coordinator.data or {}).get("system") or {}).get("rssi") or 0.0)
        except (TypeError, ValueError):
            return 0.0

    def _heap(self) -> float:
        """Memoire libre du pont, du meme sondage que le RSSI -- donc gratuit.

        Enregistree ici parce que le pont s'est redemarre pendant les deux
        menages du 27/08, et toujours au meme ENDROIT et non au meme moment :
        a 81 % et 85 % du parcours, la ou la part de scans sous -75 dBm passe
        de 0-4 % a 15-17 %. L'hypothese est qu'un lien faible fait s'empiler
        les tampons AsyncTCP jusqu'a ce que le watchdog memoire du firmware
        tire (sous 16 Ko pendant 30 s). Le heap releve APRES un reboot est
        toujours sain : c'est celui d'AVANT qu'il faut, et il n'existait
        nulle part. Apparie au RSSI du meme scan, il tranchera.
        """
        try:
            return float(((self.coordinator.data or {}).get("system") or {}).get("heap") or 0.0)
        except (TypeError, ValueError):
            return 0.0

    def _dump_captures(self, captures: list) -> None:
        """Keep one run's scans on disk, once, so the mapper can be tested.

        The firmware never persists a LIDAR scan and the robot re-serves a
        frozen frame when it is not cleaning, so nothing about the mapping
        could ever be tried against real returns -- every change had to be
        reasoned about, shipped, and judged a cycle later on the one thing it
        produced. This keeps a single run so the merge can be replayed offline
        as many times as it takes.

        Ecrit a CHAQUE run, en remplacant le precedent. Le fichier ne
        s'ecrivait auparavant que s'il etait absent -- une facon d'economiser un
        peu de disque, mais qui achetait un piege : le 27/08 le dump d'un run
        bloquait celui du suivant, et rien n'aurait signale qu'on rejouait
        l'ancien en croyant analyser le nouveau. 1,4 Mo ne valent pas un
        resultat perime indiscernable d'un resultat frais.

        Pour garder un run en particulier, le renommer : le suivant ne le
        touchera pas. Ne laisse jamais un echec atteindre la fusion : c'est une
        commodite, et la carte compte davantage.
        """
        path = Path(self.hass.config.path(CAPTURE_DUMP_NAME))
        try:
            replaced = path.exists()
            if len(captures) > CAPTURE_DUMP_MAX:
                _LOGGER.debug(
                    "LIDAR mapping: not dumping %d captures, over the %d cap",
                    len(captures), CAPTURE_DUMP_MAX,
                )
                return
            payload = {
                "session": self._session_name,
                "cell_m": CELL_SIZE_M,
                "walls_before": len(self._map.walls),
                "sessions_before": self._map.sessions,
                # Post rotation-filter: exactly what build_session_grids sees,
                # so an offline replay reproduces the run rather than resembling
                # it. (x, y, theta, points, rotationSpeed, moved, turned,
                # rssi, heap)
                "captures": [
                    [c[0], c[1], c[2], [list(p) for p in c[3]], *c[4:]]
                    for c in captures
                ],
            }
            path.write_text(json.dumps(payload), encoding="utf-8")
            _LOGGER.info(
                "LIDAR mapping: kept %d scans of %s in %s (%.1f MB) for offline "
                "testing%s",
                len(captures), self._session_name or "?", path,
                path.stat().st_size / 1e6,
                " — replacing the previous run" if replaced else "",
            )
        except Exception as err:  # noqa: BLE001 -- diagnostics never break a merge
            _LOGGER.warning("LIDAR mapping: could not keep the scans (%s)", err)

    @staticmethod
    def _drop_slow_scans(captures: list) -> list:
        """Discard scans taken while the laser was not sweeping properly.

        Judged against the run's own median rather than a fixed rpm: the
        field's unit is undocumented and the docked reading (5.03) gives no
        usable reference. A run spends most of its time at its normal speed,
        so the median *is* the normal speed, and the outliers below it are the
        spin-ups and stalls whose 360 angles were never swept in one turn.
        """
        speeds = [c[4] for c in captures if len(c) > 4 and c[4] > 0]
        if len(speeds) < 5:
            return captures
        speeds.sort()
        median = speeds[len(speeds) // 2]
        floor_speed = median * MIN_ROTATION_FRACTION
        kept = [c for c in captures if len(c) > 4 and c[4] >= floor_speed]
        if len(kept) < len(captures):
            _LOGGER.info(
                "LIDAR mapping: dropped %d of %d scans below %.1f (median %.1f)",
                len(captures) - len(kept), len(captures), floor_speed, median,
            )
        return kept

    # ── output ──────────────────────────────────────────────────────

    def _stale_notice(self) -> list[str] | None:
        """Watermark for a map that has stopped taking new cleanings.

        A refused merge used to reach nobody: the map simply stopped changing
        and the only trace was a log line. Written across the plan itself, it
        is seen -- and it says what to check, because a dock knocked out of
        true is the usual reason.
        """
        if not self._map or not self._map.rejects:
            return None
        # Unaccented on purpose: Pillow's default bitmap font has no Latin-1
        # glyphs, and "ACTUALISÉE" comes out as "ACTUALIS<box>E". Verified by
        # rendering both. Restoring the accents needs a real font file first.
        return [
            "CARTE NON ACTUALISEE",
            "Verifier que la base n'a pas bouge",
            (
                f"Tentative {self._map.rejects} sur "
                f"{MAX_CONSECUTIVE_REJECTS} avant reinitialisation"
            ),
        ]

    async def async_render(self) -> tuple[bytes, dict[str, float]] | None:
        """Render the accumulated map, or None if nothing is mapped yet."""
        if not self._map or not self._map.walls:
            return None
        return await self.hass.async_add_executor_job(
            render_plan, self._map.walls, self._map.floor,
            RENDER_PX_PER_M, self._stale_notice(),
        )

    def calibration(self) -> dict[str, float] | None:
        """Where the plan sits in the world, without rendering it."""
        if not self._map or not self._map.walls:
            return None
        return plan_calibration(self._map.walls, self._map.floor)

    def alignment(self, session_name: str) -> tuple[int, int, int] | None:
        """How this session was corrected onto the map, if it was merged.

        The card needs it to draw the run in the same frame as the walls;
        without it a session recorded after a localisation loss shows its
        cleaned area a quarter turn off.

        Le run en cours n'est pas encore fusionne et n'a donc rien de range
        sous son nom. Plutot que de le servir dans le repere brut du robot
        pendant toute l'heure du menage, on rend le placement provisoire que
        _align_live() a calcule. Le fusionne passe devant des qu'il existe.
        """
        if not self._map:
            return None
        key = alignment_key(session_name)
        merged = self._map.alignments.get(key)
        if merged is not None:
            return merged
        if (
            self._live_align is not None
            and self._session_name
            and alignment_key(self._session_name) == key
        ):
            return self._live_align
        return None

    def view_rotation(self, user_offset: float = 0.0) -> float:
        """Rotation that stands the map upright, plus the user's offset."""
        return self._map.view_rotation(user_offset) if self._map else user_offset

    def render_signature(self) -> str:
        """Identifier that changes whenever the drawn plan would change."""
        return self._map.render_signature() if self._map else "0"

    @property
    def sessions(self) -> int:
        return self._map.sessions if self._map else 0
