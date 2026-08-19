# Changelog

## 1.21.0

### Added

* **The navigation mode is recorded with each session.** `SetNavigationMode`
existed and `clean()` applied it, but nothing wrote it down, so a replay could
never say whether a run had been Normal, Gentle, Deep or Quick. The firmware
now stamps `nav` into the session header, taken from the same getter `clean()`
uses — so it is the mode the run *actually* used, not whatever the setting
happens to be when the replay is watched later. Sessions recorded before this
simply omit it.

### Changed

* **Everything about a session now lives in the picker's own option text**:
date, clean type, navigation mode, area, distance, duration, battery and
recharges. The separate stats row beside it is gone — it described only the
selected run, and described it twice. Putting the figures in the option text
describes *every* run, so they can be compared without selecting each in turn.
The picker takes the width the row used to occupy. Set `show_picker: false`
and the row comes back carrying everything instead.

### Fixed

* **README told you to install the card by hand.** It said to copy the script
into `/config/www/` and register a Lovelace resource; the integration has been
serving it and registering the script tag itself all along
(`async_register_static_paths` + `add_extra_js_url`). The cache is keyed on the
integration version, so the card refreshes on upgrade.

## 1.20.0

### Changed

* **The wall threshold now follows the map instead of standing still.** A
fixed `WALL_MIN_HITS = 12` rots as cleanings accumulate, and the plan visibly
gets noisier run after run. Measured on a real four-session map: 13 066 cells,
median 4 hits, and **23.7% of cells seen exactly once, 45% seen three times or
fewer**. Those are strays — someone walking past, a grazing return, a scan
merged a few centimetres out — and a fixed count keeps admitting them.

  Scaling linearly with the session count was tried first and is wrong: it
  assumes every cell is re-seen every session, which partial coverage
  contradicts. At four sessions it demands 48 hits and leaves 1157 cells; the
  outline breaks apart.

  A quantile is self-calibrating — it keeps a stable *share* of the map
  however many cleanings pile up, and rises on its own as noise accumulates.
  `WALL_KEEP_QUANTILE = 0.75` lands on 19 at four sessions, which is where an
  earlier by-eye sweep had put the threshold after two, so the rule agrees
  with the judgement it replaces. `WALL_MIN_HITS` stays as the floor, so a
  young map behaves exactly as before.

  Rendered both ways on the same map: **111 336 wall pixels at the old fixed
  12, 87 311 at the adaptive 19 — 21.6% less ink**, all of it low-confidence.

### Fixed

* **The card kept showing a stale plan.** The cache-buster was the session
count alone, so a plan that changed for any other reason — a new threshold, or
the code behind it — kept its old URL and browsers served the image they
already had. It is now a `render_signature()` combining the session count, the
threshold and the number of cells clearing it, so the URL changes exactly when
the picture does.

## 1.19.2

### Fixed

* **Returning to base showed up as an error.** `GetErr` latches: it keeps
reporting the last fault until something clears it, so a complaint raised
early in a run is still in the register at the end of it. The vacuum entity
checked the error *before* the robot's own state, so a normal end-of-cycle
return to base read as `error`.

  The state now wins. `DOCKINGRUNNING` shows `returning`,
  `...CLEANINGRUNNING` shows `cleaning`, and the error check applies only when
  no active state matches. Verified live with 282 latched: `error` before,
  **`returning`** after, message kept as the `error_message` attribute and the
  Error binary sensor still `on`. The vacuum says what the robot is *doing*;
  the error sensor says there is a fault to acknowledge.

  ⚠ **Known trade-off.** While a fault is genuinely blocking, the robot can
  still advertise an active `uiState`, and this ordering shows that state
  rather than the fault — the vacuum reads `returning` where it used to read
  `error`. Nothing is hidden: the Error sensor stays `on` and the text stays
  in `error_message`. Telling a stale latch from a live fault needs freshness
  tracking the coordinator does not do yet.

* **Informational alerts no longer count as problems.** The firmware groups
codes 201-242 as `UI_ALERT_*` — "informational" in its own comment — and tags
them `kind: "warning"`. That family includes 201 `UI_ALERT_RETURN_TO_BASE`,
`Cleaning complete`, `Dust bin full` and `Recovering location`. Reading
`hasError` alone tripped the Error sensor's `problem` device class on all of
them. A shared `is_real_error()` in `const.py` now decides for both platforms,
so they cannot drift apart. Real faults are untouched: 252
`UI_ERROR_UNABLE_TO_RETURN_TO_BASE` sits outside the alert range and still
reports. Firmware predating the `kind` field keeps the old behaviour rather
than silently swallowing faults.

* Alerts are surfaced as `alert_message` / `alert_code` attributes instead of
being labelled errors.

## 1.19.1

### Fixed

* **"Return to base" did nothing on a stopped robot.** `dock` sends
`UIMGR_EVENT_SMARTAPP_SEND_TO_BASE`, which the robot only acts on while a
clean is running or paused. After Stop the run is over, the event lands on a
robot with nothing to return from, and it just sits there — the button looked
broken. It now detects that case and briefly restarts cleaning so the state
machine has a run to end, then sends the robot home. Already docked is a
no-op; running or paused sends the event straight through as before.
  * ⚠ The recovery is not free. Per `docs/neato-serial-protocol.md`, a bare
    `Clean` also resets the robot's position, so the restart discards
    localisation just as Stop did. There is no lossless way home once a run
    has been stopped — a robot that comes back beats a preserved frame, and
    the mapper realigns the next session anyway.
  * **To keep localisation, pause rather than stop**, then return to base:
    from paused, no restart happens. Stopping mid-clean now logs this.

## 1.19.0

### Added

* **Delete button on the replay card.** A run that went wrong — the robot was
lifted, the LIDAR was blocked, the map came out half-drawn — left a bad replay
in the picker with no way to clear it. The bin icon next to the picker deletes
the selected session after a confirmation, then falls back to the newest one
that remains. New `openneato/delete_session` websocket command and
`OpenNeatoApiClient.delete_history_session`, on top of the
`DELETE /api/history/<name>` the firmware already exposed.
  * The filename is validated against the same strict pattern the download
    path uses, so a rogue peer cannot aim the delete at another endpoint —
    `../../settings` is refused before any request leaves Home Assistant.
  * Deleting is refused while the robot is still recording that session.
  * ⚠ **This does not un-draw the session's walls.** The LIDAR mapper merges
    hit counts, and once merged a session's cells are indistinguishable from
    every other session's. Deleting stops a bad run being replayed; it does
    not remove its contribution to the accumulated floorplan.

### Changed

* **The card header is one row**, with the session picker first, then the
stats. The stats scroll inside their own box instead of wrapping, so the
header stays a single line at any width, and an empty `title` collapses so the
picker really is first. **Date, mode and area were dropped from the stats** —
the picker's own label is already `19/08 11:32 — House Clean · 27.25 m²`, so
repeating them beside it spent the row saying the same thing twice. What
remains is what the picker does *not* show: distance, duration, battery and
recharges. Set `show_picker: false` and the three come back, since nothing
else would show them.

## 1.18.0

### Added

* **Six bumper binary sensors** — `lFrontBit`, `rFrontBit`, `lSideBit`,
`rSideBit`, `lLdsBit`, `rLdsBit`. `/api/sensors` returns ten digital bits and
the integration only surfaced four; the other six are the bumper contacts,
which the firmware itself reads as such (`manual_clean_manager.cpp`:
`bumperFrontLeft = d.lFrontBit || d.lLdsBit`). The coordinator already fetched
them on every 5 s poll, so they cost nothing extra. A stuck bumper is a common
reason a robot refuses to start. Shipped disabled by default — they toggle on
every contact during a run — so enable the ones you want to watch.

### Fixed

* **`Battery charge` and `Battery discharge` were labelled mAh but report
mA.** Sampled three times, `dischargeMAH` tracked the instantaneous current
and *decreased* (168 → 151), which a cumulative charge counter cannot do.
Confirmed from the other side minutes later: the moment the robot began
charging, `chargerMAH` jumped to 970 and `dischargeMAH` fell to 0. Both now
carry `SensorDeviceClass.CURRENT` and milliamps. Note they are the rectified
halves of `batteryCurrentMA` and carry no information it does not already
have. **Changing the unit resets these two sensors' long-term statistics.**
* **`Error code` read 200 on a perfectly healthy robot.** 200 is
`UI_ALERT_INVALID`, the robot's documented "no error" sentinel
(`docs/neato-serial-protocol.md`), and is now reported as unknown.
* **`Dock contact` renamed `DC jack`.** It reads `dcJackIn`, the robot's own
barrel jack, measured False while docked with 18.9 V present. The field that
means "on dock" is `extPwrPresent`, already exposed as `External power`. The
translation key is unchanged, so entity IDs are not affected.

### Removed

* **The `LIDAR map` and `Cleaning replay` camera entities.** The canvas replay
card supersedes both: it draws the accumulated LIDAR floorplan and replays a
session over it with pan, zoom and a scrubber, reading the
`openneato/session(s)` websocket commands directly instead of polling a
server-rendered image. `Platform.CAMERA` is gone, and with it `camera.py`,
`lidar_renderer.py` and `history_renderer.py` — 1347 lines. The one helper
still needed, `_try_repair_pose`, moved into `replay.py`.
* **The static floorplan-background option** added in 1.12, along with its
`OpenNeatoOptionsFlowHandler` config-entry options flow. Its removal was never
recorded at the time; noting it here. The LIDAR mapper builds the plan from
the robot's own scans and calibrates itself, so there is nothing left to align
by hand.

## 1.17.4

### Fixed

* **The floorplan vanished from the opening minutes of every clean.** The
in-progress map fitted its viewport to the session bounds, which span only
centimetres in the first seconds, asking for hundreds of pixels per metre: at
30 seconds into a real run the scale reached 830 px/m and the plan had to be
blown up to 4938 px, past the resize ceiling, so it was dropped and the log
filled with "Floorplan resize too large". The viewport is now held open to a
2 m minimum, which caps the scale at 220 px/m. Sessions larger than that are
framed exactly as before.

### Changed

* The dashboard card now uses `rotation: auto`, following the wall-derived
angle the generated map reports instead of the value pinned when the plan was
still a hand-drawn one.

## 1.17.3

### Fixed

* **The grid leaned once the map was straightened.** Grid lines were laid out
along the world axes, which was right while the map itself sat at whatever
angle the dock imposed. With the view now rotated to stand the walls up, the
grid became the only thing still tilted. It is drawn in screen space instead,
anchored on where the world origin lands after pan, zoom and rotation, so it
stays square to the card and still moves with the map rather than swimming
across it. It is also drawn beneath the plan now, not over it.

## 1.17.2

### Fixed

* **A short first run would have locked the map at its own size.** The
re-alignment score divided the number of matching cells by the new session's
cell count, so a session covering ground the stored map had never seen scored
badly for that alone. Record a spot clean first and every full clean after it
would have been refused. The score is now taken against the smaller of the two
maps, so a larger session matching a small reference well is accepted.

* **The alignment search could aim at the wrong place.** It seeded the
translation from the difference between the two maps' centroids, which points
nowhere useful when one map is a small off-centre piece of the other -- the
search then missed the true fit and settled on a wrong quarter turn. It now
tries both the centroid seed and no translation at all, the latter being the
usual case since the dock rarely moves, and keeps whichever fits better.

  Guard still holds either way: an unrelated shape scores 47%, against 100% for
  a genuine match, and the merge threshold sits at 55%.

## 1.17.1

### Fixed

* **Mapping could stop itself silently part-way through a cleaning.** The
health check compared the recording file's growth window to window, but the
firmware buffers pose lines and flushes only every 30 s, so a short window
catches one flush or three: measuring the same healthy robot over three
consecutive minutes gave 97%, then 51%, then 142%. Three low readings in a row
disarmed the sampling timer for the rest of the run, and because only anomalies
were logged, the run simply ended with too few captures to merge and nothing
explaining why.

  The check now averages growth over the whole run rather than the last window,
  ignores the opening minutes where a single flush dominates, logs the healthy
  case at debug, and can only slow sampling down - never stop it. A measurement
  this coarse has no business discarding an hour of collection.

## 1.17.0

### Added

* **Self-building floor plan from the LIDAR.** Every cleaning now samples pose
and scan and folds the result into an accumulated map, so the plan sharpens with
each run instead of being drawn by hand and calibrated by trial. No firmware
change was needed: `/api/lidar` and `GetRobotPos Smooth` over the serial
passthrough already carry everything, and they share a clock. The beam geometry
was measured rather than assumed - 95.3% of returns agree at offset 0, against
60% for the next-best candidate. Putting the grid on the ESP32 was rejected on
the numbers: a 10x10 m grid at 5 cm would take 26-53% of its free heap, on a
device that hung its own history endpoint under memory pressure.

  Two properties make it survive real life. Each session is **re-fitted to the
  stored map** over the four quarter turns before merging, because the robot's
  frame is anchored on the dock and a TestMode cycle was observed to turn it by
  exactly 90 degrees; a session matching less than 55% is refused rather than
  blended in (an identical run scores 99%, a rotated one 95%, a shape that is
  not this home 35%). And the **orientation comes from the walls, not the
  dock**, so the map lands the same way up however the frame moves under it.

  Sampling costs the robot nothing measurable: over a 49-minute run at a 4 s
  interval its own pose logging stayed at 95-126% of normal. The runner watches
  that figure anyway and backs off, then stops, if it drops.

* `rotation: auto` on the replay card follows the wall-derived angle a
generated plan reports. An explicit number still wins.

## 1.16.0

### Fixed

* Map framing only handled quarter turns. `_projection` special-cased 90 and 270
degrees and fitted every other angle to the upright bounding box, so any other
rotation ran off the canvas. It now solves for the scale against the rotated
bounding box: 0, 90, 180 and 270 still fit at exactly the same scale as before,
and the few degrees needed to straighten a skewed map fit too, at about 5% less
zoom. This matters because a LIDAR-built floorplan inherits the robot frame,
which is anchored on the dock rather than the walls -- here 4.9 degrees off.

## 1.15.0

### Fixed

* **Quarter-turn rotation could overflow the canvas** — `rotation: 90` or
`270` spins the map about the canvas centre, but the projection was still
fitted to the unrotated width and height, so on a non-square card the map ran
off the edges. The fit now transposes the box and slides it back onto the
rotation pivot. Square cards were unaffected, which is why this went unnoticed.

## 1.14.0

### Fixed

* **Replay view shifted between cleanings** — the projection was fitted to
each session's own pose bounds, which vary by roughly half a metre from one
clean to the next, so the floorplan (drawn in world coordinates) landed on
different pixels every time and appeared to wander. The viewport now snaps
outward to a 1 m world grid, which absorbs that variation: two consecutive
sessions measured 139.78 px and 150.94 px for the same world point before, and
both land on 155.00 px after. Costs about 14% of zoom (52.5 px/m against 61).
The new `fit` option selects the behaviour — `stable` (default), `plan` (keeps
the whole floorplan in view), or `session` (the old, tighter framing).

## 1.13.0

### Added

* **Interactive cleaning replay card** — a new Lovelace card,
`custom:openneato-replay-card`, that plays a cleaning session back on a canvas
at display refresh rate with play/pause, a scrubber, and drag/wheel pan-zoom.
The existing animated-GIF camera is a 30-frame time-lapse over 7 s (about 4
fps) with the path downsampled to 500 points and no controls, which is as far
as a GIF can go; the card instead ports `renderMap` and `MotionPlayer` from the
robot's own web UI, so the picture and the motion match
`http://neato.local/#/history/...` exactly. Measured at 0.22 ms per frame on a
1700-pose / 9347-cell session, roughly 75x headroom at 60 fps.
* **Replay WebSocket API** — `openneato/sessions` lists cleaning sessions and
`openneato/session` returns one parsed for playback. All the work (download,
decompression, coverage-grid construction) stays on the server and is cached,
so the browser only draws. `replay.py` ports `history-data.ts::buildSession`
rather than reusing the renderer's parser, because the player needs two things
the renderer does not: pose timestamps normalised to the session start, and
recharge markers paired with the pause windows in the pose timeline.
* **Playback speed control** — a button in the replay card cycles through
1x, 2x, 4x, 8x, 16x, 32x and 64x, where 1x is true real time. The web UI fixes
its rate at 8x; a house clean runs close to an hour, so being able to slow a
section down (or race through the dull parts) without editing YAML is worth the
extra button. The `speed` option still sets the starting rate, and a value
outside the presets is kept as-is until the button is used.
* **Floorplan in the replay card** — the calibrated background from the
camera entities is served to the card via `/api/openneato/floorplan/{entry_id}`
and placed with the same origin/rotation/scale maths, so both views agree.

The card is registered automatically; no manual Lovelace resource entry is
needed. The GIF and PNG cameras are unchanged.

## 1.12.1

### Fixed

* **Floorplan wiped out by the map overlays** — every coverage cell, path
segment and marker punched a hole straight through the house plan added in
1.12, leaving flat dark patches exactly where the robot had been. `_render_frame`
drew translucent RGBA fills onto an RGBA canvas, and Pillow only alpha-blends
a draw call when the target image is `RGB` *and* the `ImageDraw` context was
opened in `"RGBA"` mode — on an RGBA target the fill replaces the pixel, alpha
included. The final `paste(..., mask=alpha)` then blended those pixels against
the dark background constant instead of the plan underneath. The canvas is now
`RGB` with `ImageDraw.Draw(img, "RGBA")`, so overlays tint the floorplan
instead of erasing it. The plain dark-background map renders identically to
before.
* **Unrenderable session re-downloaded every 5 s, forever** — both cameras
only recorded the session name after a *successful* render. A session that
failed to download, parse or render — or that carried no pose data, or was
too short to animate — left the cache key unset, so the next coordinator tick
saw "new session" and pulled the whole JSONL again, indefinitely, over the
ESP32's blocking serial bridge. Completed sessions that fail are now
remembered and skipped until a newer one appears; recording sessions stay
retryable because their data is still growing.
* **In-progress map polling hammered the robot** — the cleaning-session map
reused `LIDAR_POLL_INTERVAL` (2 s), and a recording session is re-downloaded
in full every poll by design (the firmware has no range/tail API). A one-hour
clean meant roughly 1800 full downloads of a growing ~80 KB file. Split out
`HISTORY_POLL_INTERVAL` (30 s), which still tracks the robot closely enough
for a map.
* **`last_clean_*` sensors could report a dict** — the
`(s := _latest_summary(d)) and s.get(key)` idiom evaluates to the empty dict
itself when a session carries an empty summary, handing `{}` to Home Assistant
as a duration/distance/area/mode state. Replaced with an explicit
`_summary_value()` helper.
* **Config and options flow showed raw translation keys** — `strings.json` is
the source file consumed by Home Assistant's build step; custom integrations
read `translations/<lang>.json` at runtime, which didn't exist. Added
`translations/en.json`, so the setup dialog, the floorplan options and their
error messages now render as text instead of identifiers like
`path_must_be_absolute`. (Entity names are unaffected — they already fall back
to the `name` attribute, which matches the translations.)
* **Miscalibrated floorplan failed silently** — with a `scale` that doesn't
match the image, the plan is still pasted onto the canvas, but the part that
lands on it is blank margin: the map looks exactly like the plain dark
background, with nothing in the log. The existing guard only caught a plan
that missed the canvas *entirely*, which a wrong scale rarely does. The
renderer now compares the plan's world rectangle against the session's and
warns below 25% overlap, quoting both rectangles and a suggested scale and
origin. (A 480x480 plan configured at the 20 px/m default was being treated as
a 24 m x 24 m area, upscaled 3.2x, and contributing 0% of its actual drawing
to the map.)
* **Origin calibration documented back-to-front** — both the `FloorplanConfig`
docstring and the options-flow help said Origin X/Y anchored the plan's
*top-left* corner. `_draw_floorplan_background` anchors the *bottom-left*
(`top_left_y = anchor_py - height`), because world Y grows upward while canvas
Y grows downward. Corrected the docs to match the code.

### Changed

* **Floorplan path is validated when you save it** — the options flow now
rejects a path with no file behind it instead of accepting it and letting the
renderer silently fall back to the dark background with only a `WARNING` in
the log, which reads as a bad calibration rather than a bad path.
* **`latest_completed_session()` moved to `coordinator.py`** — `sensor.py`
imported it from `camera.py`, which pulled the camera entities and the Pillow
renderers into the sensor platform's import graph. The coordinator is
Pillow-free and already a shared dependency.
* **`api.py::clean()` no longer advertises a `resume` action** — the firmware's
`NeatoSerial::clean()` handles `dock`, `pause`, `stop` and `spot`, and falls
through to `EVT_START_HOUSE` for everything else, so `clean("resume")` would
have silently started a fresh house clean. `async_start()` sending `"house"`
while paused is correct: the robot's own state machine treats it as a resume.
* Dropped a redundant local re-import in `camera.py::_render_animation` and
narrowed `except (OpenNeatoApiError, Exception)` in the config flow to what it
actually was, `except Exception`.

## 1.12

### Added

* **Floorplan background for the map cameras** — the `LIDAR map` and
`Cleaning replay` cameras can now render a user-supplied house floorplan
(local PNG/JPG) as the background instead of the default dark grid.
Configure the image path and calibrate origin (X/Y metres), rotation and
scale (px/m) from **Settings → Devices \& Services → OpenNeato →
Configure**; changes apply on the next render without an HA restart. The
metric grid is hidden while a floorplan is active. Ported the renderer to
Pillow (HA Core already ships it, so no new dependency). Option flow added
via `OpenNeatoOptionsFlowHandler`; options propagate to the cameras through
an entry-reload-on-update listener.

## 1.11



### Added

* **`notify\\\_on\\\_start` switch** — surfaces fork PR #2's "notify on cleaning
started" setting (`ntfyOnStart`). Firmware + frontend already supported
the toggle; the matching HA entity was missing, so the switch row sat
among `notify\\\_on\\\_done`/`error`/`alert`/`docking` with no `start` peer.
* **`ap\\\_fallback\\\_on\\\_disconnect` switch** — upstream firmware #110 added
an `apFallbackOnDisconnect` setting that brings up the captive-portal
AP automatically when WiFi STA drops. Now configurable from HA.
* **`ntfy\\\_topic`, `ntfy\\\_server`, `ntfy\\\_token` text entities** — full ntfy
push-notification configuration is now editable from HA, no need to
open the device web UI. Empty topic disables notifications; empty
server defaults to `ntfy.sh`; empty token is unauthenticated.

### Not done (deliberate deferral)

* **Upstream About metadata not surfaced** — firmware commit bb7b541
added `name`, `repositoryUrl`, `license`, `model` fields to
`/api/firmware/version`. Threading `repository\\\_url` through every
entity constructor for nine platforms (vacuum, sensor, switch,
binary\_sensor, number, button, camera, select, text) just to set a
device-info attribute was too invasive for the value it adds, and the
`name` field already matches our hardcoded `manufacturer="OpenNeato"`.
A cleaner future approach is a dedicated diagnostic sensor that
exposes the full About payload as attributes.
* **Zeroconf discovery not added** — `manifest.json` was eligible for a
`zeroconf` entry, but the firmware (`wifi\\\_manager.cpp`) currently only
registers the generic `\\\_http.\\\_tcp` service. Claiming that service for
the OpenNeato integration would match every random HTTP device on the
LAN. When the firmware advertises a dedicated service type (e.g.
`\\\_openneato.\\\_tcp`), wire this up.

## 1.10

### Fixed

* **Setup crash with new firmware (`utf-8 codec can't decode byte 0xab`)** —
Upstream firmware PR #121 added `smartBattery\\\*` string fields to
`/api/version` that pass raw bytes from the robot's smart-battery
memory through the firmware's `jsonEscape()` (it only escapes bytes
below 0x20). When the pack reports a non-UTF-8 byte, aiohttp's
`response.json()` raised `UnicodeDecodeError` and the config entry
failed setup. Responses are now decoded with `errors="replace"`
before being parsed as JSON so one bad glyph can't take the
integration down.

### Added

* **Battery diagnostics from firmware PR #121** — three new diagnostic
sensors backed by the new `/api/analog` endpoint (battery voltage,
current, external voltage) and two backed by `/api/warranty`
(battery cycles, cumulative cleaning time). The existing
`Battery temperature` sensor was repointed from the now-removed
`/api/charger#battTempC` to `/api/analog#batteryTemperatureC`
(a finer-grained float in °C). A `New battery` button (disabled by
default) calls `POST /api/battery/new` to reset the fuel-gauge
calibration after a pack swap.

## 1.9

### Fixed

* **Cameras stuck on idle placeholder** — `get\\\_history\\\_session` called
`response.get\\\_encoding()` after streaming the body via
`response.content.read()`, which raises `RuntimeError: Cannot compute fallback encoding of a not yet read body` in modern aiohttp (the
streaming path doesn't populate the response's `\\\_body` buffer that
the chardet fallback in `get\\\_encoding` requires). The session
download therefore failed for every fetch, leaving both `LIDAR map`
and `Cleaning replay` on the polar-grid placeholder. Hardcoded UTF-8
decoding instead — the firmware emits UTF-8 JSONL by spec, so the
encoding-detection path was unnecessary.

  This is the root cause that 1.8's WARNING-level logging surfaced.

## 1.8

### Fixed

* **Blank camera diagnostics** — `LIDAR map` and `Cleaning replay` could
stay on the idle polar-grid placeholder even when a valid session
existed, because a Pillow/parser exception inside the executor would
bubble up through `async\\\_create\\\_task` and only surface as an
unhandled-task warning — easy to miss without DEBUG filtering on. The
parse and render steps are now wrapped in `try/except` and log the
traceback at `WARNING` so the failure is visible in the default log.
Fetch-failure logs also moved from DEBUG to WARNING.

## 1.7

### Fixed

* **Camera entity names** — the `LIDAR map` and `Cleaning replay` camera
entities both showed up as the bare device name (e.g. `BotVacD6Connected`)
in the UI, making them indistinguishable. Custom integrations don't read
`strings.json` at runtime, so the translation key alone wasn't enough;
the display name is now set directly alongside it, matching how every
other entity in the integration is declared.
* **LIDAR map stays blank after HA restart** — when Home Assistant started
up with the robot already idle, the LIDAR camera fetched the latest
completed session once and never retried. The integration now re-checks
on every coordinator tick, so the most recent cleaning map appears as
soon as coordinator data is available and refreshes whenever a newer
session lands.

## 1.6

### Added

* **Cleaning replay camera** (`camera.openneato\\\_\\\*\\\_motion\\\_map`, translated as
"Cleaning replay") — standard HA camera entity serving an animated GIF
time-lapse of the most recent completed cleaning session. Plays once,
then holds on the fully-drawn map so dashboards settle on a static
final image. Works with picture-entity, vacuum-card, and other camera
consumers. Regenerated only when a newer completed session lands.
Coverage cells are revealed progressively in sync with path drawing.

### Fixed

* **`/api/history` robustness** — a single heatshrink-corrupted session
file no longer breaks the HA coordinator. Previously, merged JSONL
lines (a rare decompression artifact) produced invalid JSON in the
listing response, causing the entire history fetch to fail and
leaving the LIDAR camera blank and `last\\\_clean\\\_\\\*` sensors as
"unknown". Firmware now validates session/summary metadata structure
before embedding; corrupt metadata is reported as `null`.
* **"Last clean" sensors** — now pick the most recent session by
`summary.time` instead of relying on SPIFFS directory iteration
order, which isn't deterministic. Fixes cases where a stale session
was shown as the latest.

### Security

* Session filenames from the ESP32 listing are now validated against
`\\\\d+\\\\.jsonl(\\\\.hs)?` before being interpolated into the download URL,
and response bodies are capped at 2 MB. Prevents a compromised or
misbehaving peer on LAN from redirecting history requests to
unrelated endpoints or OOM'ing HA Core with an unbounded stream.

### Previous changes bundled in this release (from 2c0adbc)

* Navigation mode select (Normal/Gentle/Deep/Quick)
* Remote syslog switch + syslog server IP text entity
* Wall follower switch migrated to standard `SetUserSettings WallEnable`

## 1.3.1

### Fixed

* **Coordinator resilience** — a single hung endpoint (e.g. `/api/error`
when the robot's serial interface gets stuck on the `GetErr` command)
no longer puts the integration into "requires attention" state. The
coordinator now only fails when ALL critical endpoints (state, charger,
system) time out. Non-critical endpoints fall back to their last-known
value so the integration keeps working during transient hangs.

## 1.3.0

### Added

* **LIDAR map camera entity** — standard HA camera entity (`camera.openneato\\\_\\\*\\\_lidar\\\_map`)
compatible with vacuum-card, picture-entity, and picture-glance cards.
Renders the robot's 360-degree LDS scan as a 480x480 dark-theme PNG with
wall segments, grid rings, and robot indicator. Algorithm ported from the
standalone frontend's lidar-map.tsx (segment detection, gap bridging,
distance smoothing, multi-scan accumulation)
* **Cleaning session history map** — when the robot is docked/idle, the camera
automatically shows the most recent completed cleaning session map with
coverage grid (green), path line (gold), start/end markers, and recharge
bolt icons. Ported from the frontend's history view rendering
* **Adaptive map\_source attribute** — entity exposes `map\\\_source` ("lidar",
"history", or "idle") plus mode-specific diagnostics (rotation\_speed,
scan\_quality for LIDAR; session\_mode, session\_duration, session\_area for
history maps)
* **Self-managed LIDAR polling** — camera fetches `/api/lidar` independently
at 2-second intervals only when the robot is actively cleaning. Stops
polling when idle to avoid wasting ESP32 serial bandwidth. Zero additional
load on the coordinator's 5-second cycle
* **Session JSONL download** — `get\\\_history\\\_session()` API method to fetch
raw JSONL pose data from `/api/history/<file>` with heatshrink corruption
recovery

### Changed

* **No firmware changes required** — uses existing `/api/lidar` and
`/api/history/<file>` endpoints. No flash needed to upgrade from 1.2.0

### Note on camera.py removal in 1.2.0

The previous camera entity (removed in 1.2.0) rendered only cleaning paths
and pulled Pillow as a declared dependency (\~20MB). This new implementation
takes a fundamentally different approach: Pillow is already a core HA
dependency (no manifest entry needed), rendering is split into standalone
modules that run in the executor, and LIDAR polling is self-managed rather
than added to the coordinator. The architecture was evaluated by a
cross-functional review covering ESP32 performance constraints, HA camera
platform conventions, vacuum-card compatibility, and UX considerations.

## 1.2.0

### Added

* **Last clean stats sensors** — 6 new sensors from cleaning history session summaries:

  * Last clean duration, area covered, distance traveled
  * Last clean battery used, cleaning mode, end timestamp
* **Entity translations** — complete `strings.json` with translations for all 35 entities
across sensor, binary\_sensor, switch, button, and number platforms

### Changed

* **Single coordinator** — merged dual fast/slow coordinators into one polling all
endpoints at 5s. The firmware's AsyncCache handles per-endpoint TTL deduplication
(charger 30s, user\_settings 5min), so cache hits return instantly without serial bus access
* **Modernized API client** — replaced deprecated `async\\\_timeout` with `asyncio.timeout`
* **Format filesystem button** — moved to diagnostic category and disabled by default
to prevent accidental data loss
* **Sensor metadata** — added `state\\\_class` to heap and storage sensors for long-term
statistics support

### Removed

* **camera.py** — removed Pillow-based map rendering (\~20MB dependency). Map rendering
belongs in the frontend layer as a Lovelace card (tracked in #37)
* **translations/en.json** — duplicate of strings.json; HA auto-generates translations
for custom components
* **brand/ directory** — not loaded by HA for custom integrations (only for core
integrations in the home-assistant/brands repo)
* **Pillow dependency** — `requirements` is now empty

### Fixed

* **Translation keys** — 29 translation keys were declared in entity code but never
defined in strings.json (dead code). All now properly defined

## 1.1.3

* Custom ntfy server hostname and access token support
* HTTPS with Bearer auth for self-hosted ntfy servers

## 1.1.2

* Revert to standard shared aiohttp session with comprehensive logging

## 1.1.1

* Use dedicated aiohttp session to bypass system proxy
* Increase API timeout to 30s for busy ESP32 serial queue

## 1.1.0

* Address PR review: fix connection handling, response leaks, and compat
* Add local brand images for HA 2026.3+ brands proxy API

## 1.0.2

* Bump version

## 1.0.1

* Fix vacuum state mapping and use project icon

## 1.0.0

* Initial Home Assistant custom integration for OpenNeato

