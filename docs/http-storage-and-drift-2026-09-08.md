# HTTP storage diagnostics and local-matching investigation

`GET /api/system` previously queried SPIFFS from the AsyncTCP handler. Filesystem contention could therefore hold up the request handler and other queued work. A 7 September cleaning exhibited a 38.5-second response, but no trace isolates how much of that delay was inside these calls.

Storage counters are now initialized after the filesystem mount and refreshed from loopTask every 30 seconds. Filesystem reads take `FsLock`; the handler reads atomic counters without a filesystem lock or SPIFFS access. Heap, uptime and the other system fields retain their existing sampling behavior. Storage telemetry can lag by one refresh period, plus loop scheduling delay. No route, response field or runtime dependency changes.

Validation: frontend build and embedded-asset generation succeeded; C3 firmware build, repository formatting and clang-tidy passed without defects. Real-device performance during a cleaning must still be checked; idle response times alone cannot establish that the original incident is eliminated.

## Geometry: what is established

Replaying cycle 1788782979 produces 437 matching attempts and 51 boundary rejections. Attempts 409–435 contain 24 rejections, not 27 consecutive rejections. The longest uninterrupted run contains 13. Attempt 436 (placed scan 445, capture index 446) is accepted by the search but proposes accumulated translation `(0.075, 0.600)` m, whose norm is approximately 0.60467 m. This triggers the 0.60 m guard. The previous correction was `(-0.075, 0.500, -8.25 degrees)`.

The triggering scan retains the previous correction; subsequent scans use zero correction. The discontinuity is real. Boundary results alone do not establish whether the missing optimum represents genuine pose error or attraction to an incorrect part of the accumulated grid.

An offline experiment rejects only individual corrections exceeding 0.60 m and allows matching to continue with the previous correction. On the same captures, tracker-based graph refinement and the same pre-cycle reference:

| Metric | Current | Reject excessive step, continue |
|---|---:|---:|
| Drift-limit exceedances | 1 | 11 |
| Matching stop | scan 445 | none |
| Overlap | 91.83% | 88.98% |
| Visible cells | 3295 | 3461 |
| Connected components | 55 | 60 |
| Mean row/column run length | 3.925 | 4.074 |
| Singleton runs | 21.50% | 22.19% |

These metrics are not physical ground truth and depend on thresholds and orientation. Together they provide no sufficient case for deployment. The experiment is excluded. The matching window, drift guard, frameOffset handling and accumulated map are unchanged by the HTTP fix. Further geometry work needs local scan/reference comparisons around the failed episode and validation of the proposed correction, rather than merely avoiding the guard.
