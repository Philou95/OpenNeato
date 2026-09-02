#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// SPIFFS is a single device driven from two tasks. AsyncTCP serves session
// downloads (`GET /api/history/<file>`) while loopTask writes a pose snapshot
// every 2 s during a clean. Nothing serialised them, so the driver interleaved
// the two and the *stored* file lost records and ended in torn bytes.
//
// Measured 2026-09-02: the 20:00 session was re-downloaded 433 times by the
// replay card while it recorded, and came back 1345 lines instead of 1826 --
// the last 16 minutes gone, no summary record, binary garbage at the cut. The
// 18:26 session, polled 218 times, survived intact. Same firmware, twice the
// read pressure.
//
// The lock goes on both sides of the task boundary, not on all 83 call sites:
// once at the top of each loop-task tick(), and once in each method a web
// handler can reach -- including LogReader::read(), which AsyncTCP calls
// repeatedly while streaming a chunked response.
//
// Recursive on purpose: the locked methods call each other, and a plain mutex
// would deadlock the first time one of them did.
SemaphoreHandle_t fsMutex();

// Scoped guard. Take it by declaring `FsLock lock;` at the top of a scope.
struct FsLock {
    FsLock() { xSemaphoreTakeRecursive(fsMutex(), portMAX_DELAY); }
    ~FsLock() { xSemaphoreGiveRecursive(fsMutex()); }

    FsLock(const FsLock&) = delete;
    FsLock& operator=(const FsLock&) = delete;
};
