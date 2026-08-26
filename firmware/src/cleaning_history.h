#ifndef CLEANING_HISTORY_H
#define CLEANING_HISTORY_H

#include <Arduino.h>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include "config.h"
#include "data_logger.h"
#include "neato_commands.h"

class NeatoSerial;
class SystemManager;

// Last completed cleaning session stats — populated at end of each session,
// read by NotificationManager to enrich "cleaning done" notifications.
struct LastCleanStats {
    bool valid = false; // True after at least one completed session
    String mode; // "house", "spot", or "manual"
    long durationSec = 0; // Cleaning duration in seconds
    float areaCoveredM2 = 0.0f; // Estimated area in square meters
    float distanceM = 0.0f; // Total distance traveled in meters
    int batteryStart = -1; // Battery % at session start
    int batteryEnd = -1; // Battery % at session end
    int recharges = 0; // Mid-clean recharge count
    // Bumped every time a session is finalized (success or discard) so that
    // NotificationManager can detect when stopCollection's async charger fetch
    // has completed and the stats above reflect the just-ended session.
    uint32_t sessionId = 0;
};

// Session metadata returned by listSessions() — includes the raw JSON of
// the session header line and (if finished) the summary line so the frontend
// can render list cards without fetching each file's full content.
struct HistorySessionInfo {
    String name; // Filename (e.g. "1771683615.jsonl.hs")
    size_t size = 0; // File size in bytes
    bool compressed = false;
    bool recording = false; // True if this is the active recording session
    String session; // Raw JSON of first line ({"type":"session",...})
    String summary; // Raw JSON of last line ({"type":"summary",...}), empty if still recording
};

// One LIDAR scan held for delivery, compacted.
//
// LdsScanData is 5 760 bytes -- 360 points of four ints -- and almost all of
// it is waste for this purpose: the angle is the index, the distance fits a
// uint16, and neither intensity nor errorCode is used to build a map. What is
// left is 744 bytes, which is what makes a useful buffer fit in RAM at all.
struct BufferedScan {
    uint32_t seq = 0;
    uint32_t ts = 0; // robot clock at the scan
    float x = 0.0f, y = 0.0f, theta = 0.0f;
    float rpm = 0.0f;
    // How far the robot moved and turned *during* the scan. A scan takes
    // 0.6 to 1.7 s while the robot keeps driving, so the returns are smeared
    // by whatever it did meanwhile, and the mapper weighs each scan by this.
    // Without it every scan would count the same and the smear filtering
    // would be lost.
    float moved = 0.0f;
    float turned = 0.0f;
    uint16_t dist[360] = {0}; // mm, 0 = no return
};

// Records robot pose data during autonomous cleaning runs and stores each
// session as a JSONL file on SPIFFS. During collection, raw JSONL lines are
// buffered and flushed to /history/<epoch>.jsonl. When cleaning ends, the
// file is compressed to .jsonl.hs via incremental heatshrink encoding
// (non-blocking, spread across tick() calls).
//
// Files are served through the same LogReader/CompressedLogReader/PlainLogReader
// abstractions used by DataLogger, so the web server streaming code is identical.

class CleaningHistory : public LoopTask {
public:
    CleaningHistory(NeatoSerial& neato, DataLogger& logger, SystemManager& sysMgr);

    // -- File management (for API, mirrors DataLogger pattern) ----------------

    std::vector<HistorySessionInfo> listSessions();

    // JSON array served by GET /api/history, rebuilt in the loop task.
    // The HTTP handler runs on the AsyncTCP task; enumerating and
    // decompressing session files there stalls that task while the loop
    // writes to SPIFFS, and ESPAsyncWebServer then aborts the response
    // mid-body (truncated JSON) or the client times out. Serving a
    // pre-built String keeps the handler pure-RAM.
    const String& getListJson();
    void invalidateListJson() { listJsonDirty = true; }

    std::shared_ptr<LogReader> readSession(const String& filename);
    bool deleteSession(const String& filename);
    void deleteAllSessions();

    // Last completed session stats (for notification enrichment)
    const LastCleanStats& getLastCleanStats() const { return lastCleanStats; }

    // -- LIDAR delivery buffer -----------------------------------------------
    //
    // The bridge samples the LIDAR itself while cleaning and holds the scans
    // until Home Assistant collects them, so a WiFi outage costs nothing:
    // nobody but us records a scan, and the firmware never persisted one.
    //
    // Pure RAM, like getListJson(): the batch is built on the loop task and
    // the handler only hands over the finished String. Reading SPIFFS from the
    // AsyncTCP task is what truncated /api/history.
    //
    // `after` is the highest sequence number the caller already has; every
    // scan up to it is dropped. Idempotent -- repeating a request re-sends the
    // same batch, and a scan delivered twice only doubles one weight.
    String takeScanBatch(uint32_t after);

    // Called by WebServer when a clean command is sent via API.
    // Switches to active polling so collection starts immediately
    // instead of waiting for the next idle-interval tick.
    void notifyCleanStart();

    // -- Session import (upload from browser, compress-on-write) ---------------
    // Called by WebServer upload handler. Receives raw JSONL data from the browser,
    // compresses it via heatshrink, and writes directly to /history/<name>.jsonl.hs.

    // Prepare for import. Returns false with an error message if busy or file exists.
    bool beginImport(const String& filename);
    // Feed a chunk of raw JSONL data into the compressor and write to disk.
    bool writeImportChunk(const uint8_t *data, size_t len);
    // Finalize the encoder, flush remaining bytes, close file. Returns true on success.
    bool endImport();
    bool isImporting() const { return importing; }
    const String& getImportError() const { return importError; }

private:
    void tick() override;

    // -- LIDAR delivery buffer ----------------------------------------------
    void sampleScan();          // loop task: ask the robot for a scan
    void serviceScanBuffer();   // loop task: drop acked, refill, build the batch
    void pushScan(const BufferedScan& scan);  // loop task: into RAM, or flash
    bool appendSpill(const BufferedScan& scan); // loop task: write one record
    void refillFromSpill();     // loop task: flash -> RAM when there is room
    void buildBatch();          // loop task: the next batch, as NDJSON
    bool scanPending = false;

    std::deque<BufferedScan> scanRing;   // oldest first
    uint32_t scanSeq = 0;                // last sequence number handed out
    unsigned long lastScanMs = 0;
    // Buffering only starts once someone has actually collected a batch, and
    // stops again if nobody does. A bridge flashed ahead of the integration
    // must not fill its flash for a reader that never comes.
    unsigned long lastDrainMs = 0;
    bool spilling = false;               // overflowing to flash
    size_t spillReadOffset = 0;
    // Handed to the HTTP task, built here.
    String batchJson;
    uint32_t batchFirstSeq = 0;
    uint32_t batchLastSeq = 0;
    volatile uint32_t ackedSeq = 0;

    NeatoSerial& neato;
    DataLogger& dataLogger;
    SystemManager& systemManager;

    // -- Last session stats (survives reset, updated at end of each session) --
    LastCleanStats lastCleanStats;
    uint32_t sessionCounter = 0; // Source of truth for lastCleanStats.sessionId

    // -- State tracking ------------------------------------------------------
    String prevUiState;
    bool collecting = false;
    bool recharging = false;
    bool fetchPending = false;
    unsigned long fetchStartedMs = 0;
    bool recoveryAttempted = false; // Only try orphan recovery once after boot
    size_t snapshotCount = 0;

    // Active session file (open during collection, closed at end)
    File activeFile;
    String activeFilePath; // e.g. "/history/1771683615.jsonl"

    // -- Session metadata ----------------------------------------------------
    String cleanMode;
    time_t sessionStartTime = 0;
    int batteryStart = -1;

    // -- Session accumulators ------------------------------------------------
    int rechargeCount = 0;
    float totalDistance = 0.0f;
    float totalRotation = 0.0f;
    float maxDistFromOrigin = 0.0f;
    int errorsDuringClean = 0;
    bool prevHadError = false;

    // Previous pose for delta calculations
    float prevX = 0.0f;
    float prevY = 0.0f;
    float prevTheta = 0.0f;
    float originX = 0.0f;
    float originY = 0.0f;
    bool hasPrevPose = false;

    // Coarse area coverage — set of visited grid cells
    std::set<uint32_t> visitedCells;

    // -- End-of-session compression (incremental, non-blocking) ---------------
    bool compressing = false;
    File compressSrc;
    File compressDst;
    heatshrink_encoder compressEncoder;
    bool compressInputDone = false;
    String compressSrcPath;
    String compressDstPath;

    bool compressStep(); // Returns true when done

    // -- Collection lifecycle ------------------------------------------------
    void checkState();
    void startCollection(const String& uiState);
    void stopCollection();
    void collectSnapshot();
    void writeLine(const String& line); // Immediate write + flush (headers, summaries)
    void bufferLine(const String& line); // Buffer for deferred flush (pose snapshots)
    void flushWriteBuffer(); // Flush buffered lines to disk
    std::vector<String> writeBuffer;
    unsigned long lastFlushMs = 0;
    // When something last read the session still being written. Poses are
    // buffered in RAM and normally only reach the file every 30s, which is
    // what a live viewer sees as lag. While a reader keeps asking, the flush
    // interval drops; when it stops, buffering goes back to normal on its own
    // so an unwatched clean costs no extra flash writes.
    unsigned long lastWatchedMs = 0;
    bool isWatched() const;
    void writeSessionHeader();
    void writeSessionSummary(int batteryEnd);
    void writeSnapshot(float x, float y, float theta, float time, int brushRPM);
    void updateAccumulators(float x, float y, float theta);
    void resetSession();
    bool replayLine(const String& line);
    bool recoverCollection(const String& uiState);
    void finalizeOrphanSessions();

    // Storage enforcement — delete oldest sessions when budget exceeded
    void enforceLimits();

    // -- Import state (separate from recording compression) -------------------
    bool importing = false;
    File importFile;
    heatshrink_encoder importEncoder;
    String importFilePath; // e.g. "/history/1771683615.jsonl.hs"
    String importError;
    size_t importBytesReceived = 0;

    // Read first and last lines from a session file (decompresses .hs files)
    static void readFirstLastLines(const String& path, bool compressed, String& firstLine, String& lastLine);

    // -- Metadata cache (avoids repeated decompression for listSessions) ------
    // Keyed by filename (e.g. "1771683615.jsonl.hs"). Populated on first list
    // request and after compression/import. Entries are immutable once a session
    // is finalized — invalidated only by delete/deleteAll/enforceLimits.
    struct CachedMeta {
        String session; // Raw JSON of session header line
        String summary; // Raw JSON of summary line
    };
    std::map<String, CachedMeta> metaCache;

    // Pre-serialized /api/history listing (see getListJson()).
    String listJsonCache;
    bool listJsonDirty = true;
    void rebuildListJson();

    // Session/summary JSON captured during stopCollection for cache insertion
    // after compression completes (avoids re-decompressing the just-written file).
    String pendingSessionJson;
    String pendingSummaryJson;

    static bool isCleaningState(const String& uiState);
    static bool isPausedState(const String& uiState);
    static bool isDockingState(const String& uiState);
    static bool isSuspendedState(const String& uiState);
    static String cleanModeFromState(const String& uiState);
};

#endif // CLEANING_HISTORY_H
