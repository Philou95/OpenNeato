# Settings allocation and history compression

Cycle 1788763156 completed but the bridge restarted in `async_tcp`. The matching
ELF decodes the saved stack through `operator new`, `vector<Field>` growth and
`Settings::toFields()`. Its 71 fields grew to capacity 116, requiring a contiguous
4,176-byte allocation while the previous vector and field strings were alive.
Settings now serialize one field at a time, preserving the same keys and types.
The output allocation and appends are checked; failure discards the response and
returns HTTP 503. This removes the allocation path observed in the dump; it does
not establish the original cause of all heap pressure.

The same cycle reported a zero-byte write for 282 compressed bytes. Free space
alone does not establish the cause. Compression now batches output in a fixed
2 KiB buffer and restarts the entire stream once after failure, using fresh file
handles and encoder state. It never resumes at an uncertain buffered-file offset.
If the retry fails, the raw journal remains available and diagnostics retain the
failure. A complete raw journal can be retried at the next idle boot.

Output is private under `.hs.tmp` until the file is closed and reopened. Size and
an FNV-1a checksum of the stored compressed bytes must match the encoder output,
checked incrementally. Only then is it renamed `.hs` and the raw journal removed.
The checksum detects accidental storage corruption; it is not cryptographic and
does not verify the physical geometry of the recorded trajectory. A power loss
before publication leaves the raw journal intact.

The common compression setup also initializes counters and source size for orphan
sessions, which previously omitted them. Metadata for the selected orphan is read
after finalizing all orphans, avoiding another orphan's summary entering its cache.

Host tests inject JSON allocation failures, escaping cases, short/zero writes,
truncated/extended/corrupt output and read failures. Existing recovery and initial
frame tests remain enabled. Build and clang-tidy checks apply to the firmware;
device deployment and the next complete cleaning are separate validation stages.
