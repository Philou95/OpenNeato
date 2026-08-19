"""Constants for the OpenNeato integration."""

from homeassistant.components.vacuum import VacuumActivity

DOMAIN = "openneato"
CONF_HOST = "host"
DEFAULT_POLL_INTERVAL = 5  # seconds

# ── Floorplan background (history map camera) ──────────────────────────────
# When configured, the history/motion map renders a user-supplied house
# floorplan image as the background instead of the dark solid color. The
# robot's world coordinates (metres, robot odometry frame) are mapped onto
# the image via an origin (x,y metres), a rotation (degrees) and a scale
# (pixels per metre). All four are calibrated manually once.
# Below this much overlap between the plan and the cleaned area, the
# background is effectively invisible (blank image margin is what lands on
# the canvas). Warn with the numbers needed to fix the calibration rather
# than rendering a blank-looking map with nothing in the log.

# The firmware returns uiState as full enum strings like "UIMGR_STATE_HOUSECLEANINGRUNNING".
# We match using substrings (via .includes() style) like the frontend does in dashboard.tsx.
# These substring keys are checked against the raw uiState value.
UISTATE_SUBSTRINGS: list[tuple[str, VacuumActivity]] = [
    ("CLEANINGRUNNING", VacuumActivity.CLEANING),
    ("MANUALCLEANING", VacuumActivity.CLEANING),
    ("CLEANINGPAUSED", VacuumActivity.PAUSED),
    ("CLEANINGSUSPENDED", VacuumActivity.PAUSED),
    ("DOCKING", VacuumActivity.RETURNING),
]

FAN_SPEEDS = ["eco", "normal", "intense"]


def is_real_error(error_data: dict | None) -> bool:
    """True only for a fault, not for the robot's informational alerts.

    `GetErr` reports far more than faults. The firmware groups codes 201-242
    as `UI_ALERT_*` -- "informational", in its own words -- and tags them
    `kind: "warning"`; everything else is `kind: "error"`. Among those alerts
    are 201 `UI_ALERT_RETURN_TO_BASE` and 202 `..._PWR`, which the robot
    raises *every time it heads home at the end of a clean*, alongside
    "Cleaning complete", "Dust bin full" and "Recovering location".

    Reading `hasError` alone therefore painted a normal end-of-cycle dock as a
    fault: the vacuum entity flipped to ERROR and the problem sensor tripped,
    on every single run. Real failures still surface -- 252
    `UI_ERROR_UNABLE_TO_RETURN_TO_BASE` sits outside the alert range and keeps
    `kind: "error"`.

    Firmware predating the `kind` field falls back to the old behaviour rather
    than silently swallowing faults.
    """
    if not error_data or not error_data.get("hasError"):
        return False
    kind = error_data.get("kind")
    if not kind:
        return True
    return kind == "error"


# ── LIDAR map camera ────────────────────────────────────────────────
LIDAR_POLL_INTERVAL = 2  # seconds, only while robot is active

# ── History (cleaning session) map polling ──────────────────────────
# Re-rendering the in-progress session means re-downloading the whole
# growing JSONL each time -- the firmware serves no range/tail API. At
# the LIDAR cadence a ~1h clean would pull ~80 KB roughly 1800 times
# over the ESP32's blocking serial bridge, which starves the rest of
# the API. 30 s still tracks the robot closely enough for a map.
HISTORY_POLL_INTERVAL = 30  # seconds, only while a clean is running
LIDAR_IMAGE_SIZE = 480  # pixels (square)
LIDAR_MAX_RANGE_MM = 5000  # display radius
LIDAR_MAX_DIST_MM = 6000  # reject readings above this
LIDAR_MAX_SCAN_AGE = 5  # keep points from the last N scans
LIDAR_MAX_BRIDGE_GAP = 5  # bridge up to N missing angles
LIDAR_MAX_DIST_JUMP_PCT = 0.3  # 30% — max jump to consider same surface
LIDAR_SMOOTH_WINDOW = 5  # moving-average half-window
LIDAR_MIN_SEGMENT_LEN = 3  # min points to draw a wall segment

# Colors (RGB tuples for PIL)
LIDAR_BG_COLOR = (30, 30, 34)  # #1E1E22
LIDAR_GRID_COLOR = (42, 42, 48)  # #2A2A30
LIDAR_WALL_COLOR = (91, 164, 245)  # #5BA4F5 — desaturated blue, colorblind-safe
LIDAR_ROBOT_COLOR = (138, 138, 142)  # #8A8A8E

# ── History (cleaning session) map ──────────────────────────────────
HISTORY_IMAGE_SIZE = 480  # pixels (square)
HISTORY_ROBOT_DIAMETER_M = 0.33  # Neato Botvac diameter
HISTORY_CELL_SIZE_M = 0.05  # 5cm grid cells for coverage
HISTORY_PAD_PX = 20  # canvas padding
HISTORY_GRID_STEP_M = 0.5  # grid line spacing

HISTORY_BG_COLOR = (30, 30, 34)  # #1E1E22 — same dark bg as LIDAR
HISTORY_GRID_COLOR = (255, 255, 255, 10)  # very subtle white
HISTORY_COVERAGE_COLOR = (52, 199, 89, 38)  # rgba(52, 199, 89, 0.15)
HISTORY_PATH_COLOR = (249, 235, 178, 153)  # rgba(249, 235, 178, 0.6)
HISTORY_START_COLOR = (52, 199, 89, 230)  # green
HISTORY_END_COLOR = (255, 69, 58, 230)  # red
# Warm orange, deliberately distinct from the gold path color so the
# bolt icon reads clearly even when a recharge point sits on top of a
# drawn path segment. Matches vacuum-dashboard conventions for "event".
HISTORY_RECHARGE_COLOR = (255, 160, 51)

# ── Animated motion-replay camera ──────────────────────────────────
# The animation compresses a full session into a short loop regardless
# of the real cleaning duration. Plays once (loop=1) then holds on the
# fully-drawn map so the dashboard settles on a static final image
# rather than spinning forever.
MOTION_FRAMES = 30
MOTION_TOTAL_MS = 7000
MOTION_TAIL_FRAMES = 10  # ~1/3 of the loop holds the final map
# Render-time cap on the path/coverage fed into the GIF encoder — long
# sessions (40+ min, 2000+ poses) would otherwise spend seconds in the
# executor and produce megabyte-scale GIFs. Subsampling preserves the
# first/last pose so start and end markers still sit on real data.
MOTION_MAX_PATH_POINTS = 500

# ── API hardening ───────────────────────────────────────────────────
# Session name flows from the ESP32's /api/history response into the
# next URL path segment, so validate it before concatenation to stop
# a rogue/compromised robot (or LAN MITM over plain HTTP) from
# redirecting requests to other endpoints.
SESSION_NAME_PATTERN = r"^\d+\.jsonl(\.hs)?$"
# Upper bound on /api/history/<name> responses. Real sessions on the
# 1MB SPIFFS history budget cap below this; a larger payload implies a
# misbehaving peer and we refuse to load it into HA Core.
MAX_HISTORY_RESPONSE_BYTES = 2 * 1024 * 1024

# ── Generated LIDAR map ─────────────────────────────────────────────
# The plan is built from the robot's own LIDAR and stays in the robot's
# coordinate frame, so it never needs calibrating. Its orientation is derived
# from the walls rather than the dock, which keeps it upright even when the
# robot's frame is re-zeroed; this offset is added on top for taste.
CONF_MAP_ENABLED = "lidar_map_enabled"
CONF_MAP_ROTATION_OFFSET = "lidar_map_rotation_offset"
MAP_DEFAULT_ENABLED = True
MAP_DEFAULT_ROTATION_OFFSET = 0.0

# Smallest world span the session viewport will fit to. A clean that has only
# just started spans centimetres, and fitting that to the canvas produces an
# absurd pixels-per-metre figure.
MIN_SESSION_SPAN_M = 2.0
