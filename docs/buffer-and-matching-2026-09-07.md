# Buffer liveness and rejected scan matching

Cycle 1788782979 completed without a bridge restart or storage failure. HA briefly
fell back to direct scanning after reporting 15 empty drains. Its tracker rejected
51 of 527 placed scans at the coarse translation boundary.

## Buffer correction

The liveness counter used the number of **accepted captures**, although the parser
advances the boot-scoped cursor for motion-smeared or empty scans too. A stream of
valid deliveries rejected for quality therefore looked like a stalled bridge.
This is reproducible with fault tests; the saved captures do not contain the
discarded packets needed to prove that it caused this particular recorded warning.

Liveness now follows cursor progress, while geometric quality filtering remains
unchanged. Filtered deliveries establish buffer support, reset the quiet counter
and persist their cursor. Repeated old packets do not count as progress, and a real
stall still triggers the existing fallback. The quiet counter resets per cleaning.

## Matching diagnosis and safe optimization

Replay reproduces all 51 refusals. None has an equally scored interior coarse
candidate. Keeping the rejection protects the map against poorly constrained
translation; enlarging the window or accepting its edge is not justified.

The fine pass previously still ran after a coarse boundary rejection, even though
both callers ignore that pose. It is now skipped, avoiding 125 score evaluations
per refused scan: 6,375 on this cycle. Poses, scratch walls and graph edges are
checked for exact equality against the previous code on three recorded cleanings.
This saves work during collection and offline reconstruction; it does not imply
a measurable reduction of every final merge.

Replay also identifies the drift guard stopping matching at placed scan 445
(capture index 446; two captures had zero weight). The previously accumulated
correction was (−0.075 m, +0.50 m, −8.25°), and the next proposed translation
exceeded 0.60 m. This event was only logged at debug level; it is now explicit,
with a final count of the remaining scans receiving no further matching update.

An experiment retaining the last correction instead of the current raw fallback
avoided the frame jump but reduced overlap with the saved map from 91.83% to
89.17%. Loop-only p95 residuals did not improve either. That experiment is **not
included**. The existing fallback and accepted geometry remain unchanged. Better
handling of a stopped matcher remains an open geometric problem; overlap alone
does not prove physical accuracy. The user considers the current overall map
shape approximately correct, so preserving it is a validation constraint.

Tests exercise filtered deliveries beyond the fallback timeout, true stalls with
repeated packets, omitted fine work on rejected boundaries, preserved fine work
for interior matches, and visibility of the unchanged drift fallback.
