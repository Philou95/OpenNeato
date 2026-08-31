#ifndef ASYNC_CACHE_H
#define ASYNC_CACHE_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <functional>
#include <vector>

// Generic async cache with TTL, request deduplication, and explicit invalidation.
//
// T must be default-constructible. The cache stores the last successful value
// and a timestamp. Subsequent requests within the TTL window return the cached
// value instantly. If a fetch is already in flight, new requesters are queued
// and all receive the same result when it arrives.
//
// Usage:
//   AsyncCache<ChargerData> cache(2000, [this](auto cb) { fetchCharger(cb); });
//   cache.get([](bool ok, const ChargerData& d) { ... });
//
// ── Two tasks, one cache ────────────────────────────────────────────────────
//
// Every member below is touched from **both** the AsyncTCP task and the Arduino
// loop task, and until 2026-08-31 none of it was synchronised:
//
//   * `get()` runs on the AsyncTCP task -- it is called straight out of the
//     HTTP handlers -- and does `waiters.push_back(callback)`;
//   * the fetch completion runs on the loop task, out of `NeatoSerial::tick()`
//     draining the serial queue, and does `std::move(waiters)`.
//
// A push_back that reallocates **copies** the std::functions already in the
// vector. Preempt that mid-copy, let the loop task move the vector out from
// under it, and what the copy constructor reads is a freed `_M_manager`
// pointer. The C3 is single-core, so this needs a context switch inside a
// window of a few instructions -- rare, not impossible.
//
// That is not a theory. The bridge was restarting five to eight times a day
// with a healthy heap and a healthy link, and the core dump of the crash of
// 2026-08-31 17:24 says exactly this:
//
//     crash_task   loopTask        the Arduino loop, not AsyncTCP
//     mcause       1               instruction access fault
//     pc           0x3fcad754      a DRAM address -- it tried to execute data
//     ra           0x420101ba      std::function<void(bool, ErrorData const&)>
//                                  ::function(const&) -- the copy constructor
//
// `std::function<void(bool, const ErrorData&)>` is this class's `Callback` for
// the error cache, and that crash landed as the polling cycle was entering
// /api/error. The task, the corrupted type and the position in the cycle agree.
//
// So: every access to the state below is taken under `mutex`, and **no callback
// is ever invoked while holding it**. Both halves matter. Callbacks are free to
// call `get()` again -- the re-entrancy the old code already worried about --
// and calling one under the lock would deadlock the moment it did.
template<typename T>
class AsyncCache {
public:
    using Callback = std::function<void(bool, const T&)>;
    using FetchFunc = std::function<void(Callback)>;
    using HitFunc = std::function<void(unsigned long)>;

    // ttlMs: how long a cached value is considered fresh (milliseconds)
    // fetchFunc: the async producer — must eventually call its callback exactly once
    // hitFunc: optional callback fired on cache hits (e.g. for logging)
    AsyncCache(unsigned long ttlMs, FetchFunc fetchFunc, HitFunc hitFunc = nullptr) :
        ttl(ttlMs), fetcher(fetchFunc), hitFunc(hitFunc), mutex(xSemaphoreCreateMutex()) {}

    // These live for the lifetime of the device and own a semaphore handle.
    // Deleted rather than defaulted so a copy is a compile error instead of two
    // objects sharing one mutex.
    AsyncCache(const AsyncCache&) = delete;
    AsyncCache& operator=(const AsyncCache&) = delete;

    // Request the value. Returns cached data instantly if fresh, otherwise
    // triggers a fetch (or piggybacks on an in-flight one).
    void get(Callback callback) {
        // Everything decided under the lock, everything *done* outside it.
        bool hit = false;
        unsigned long age = 0;
        T value{};
        std::vector<Callback> abandoned;
        bool startFetch = false;
        unsigned long gen = 0;

        take();
        if (hasValue && (millis() - cachedAt < ttl)) {
            // Cache hit — value is fresh
            hit = true;
            age = millis() - cachedAt;
            value = cached;
        } else {
            // A fetch that never calls back used to wedge this cache for good:
            // `fetching` stayed true, every later get() queued behind it and was
            // never served, and the endpoint answered nothing until the bridge was
            // rebooted. invalidate() does not help — it clears `hasValue`, not
            // this.
            //
            // Seen on 2026-08-28: /api/error timed out 429 times in a row, every
            // polling cycle for hours, while the very same GetErr command answered
            // in 54 ms through the uncached path and POST /api/clear-errors in
            // 148 ms. The serial queue was healthy throughout; only the cache was
            // dead. Which event lost that one callback was never identified, and
            // this makes it not matter: the producer is contracted to call back
            // exactly once, and if it has not by now it never will.
            if (fetching && millis() - fetchStartedAt >= FETCH_GIVE_UP_MS) {
                fetching = false;
                abandoned = std::move(waiters);
                waiters.clear();
            }

            // Add to waiters list
            if (callback)
                waiters.push_back(callback);

            // If a fetch is already in flight, this request will be served when
            // it completes; otherwise trigger a new one.
            if (!fetching) {
                fetching = true;
                fetchStartedAt = millis();
                // Tagged, so a producer abandoned above cannot come back to life
                // later and clear `fetching` out from under the fetch that
                // replaced it.
                gen = ++generation;
                startFetch = true;
            }
            value = cached;
        }
        give();

        if (hit) {
            if (hitFunc)
                hitFunc(age);
            if (callback)
                callback(true, value);
            return;
        }
        for (auto& cb: abandoned) {
            cb(false, value);
        }
        if (startFetch) {
            // Outside the lock as well: the producer enqueues a serial command,
            // and a producer that answered synchronously would otherwise reach
            // complete() with the mutex already held by this very task.
            fetcher([this, gen](bool ok, const T& data) { complete(gen, ok, data); });
        }
    }

    // Mark cached value as stale — next get() will trigger a fresh fetch.
    void invalidate() {
        take();
        hasValue = false;
        give();
    }

    // Check whether a cached value exists (regardless of freshness)
    bool hasCached() {
        take();
        bool has = hasValue;
        give();
        return has;
    }

    // Get the last cached value directly (for non-async access patterns).
    //
    // By value, not by reference: the loop task overwrites `cached` from
    // complete(), and T here holds Strings. Handing a caller a reference into
    // shared state and letting it read the Strings at its leisure is the same
    // class of bug as the one above, one step quieter -- it reads a freed
    // buffer instead of executing one.
    T getCached() {
        take();
        T copy = cached;
        give();
        return copy;
    }

private:
    void take() { xSemaphoreTake(mutex, portMAX_DELAY); }
    void give() { xSemaphoreGive(mutex); }

    // Where the fetch lands. Same rule as get(): state under the lock,
    // callbacks after it.
    void complete(unsigned long gen, bool ok, const T& data) {
        std::vector<Callback> pending;
        T value{};

        take();
        if (gen != generation) {
            give();
            return;
        }
        fetching = false;
        if (ok) {
            cached = data;
            cachedAt = millis();
            hasValue = true;
        }
        value = ok ? cached : data;
        // Moved out rather than iterated in place, so a waiter's callback is
        // free to call get() and queue itself again.
        pending = std::move(waiters);
        waiters.clear();
        give();

        for (auto& cb: pending) {
            cb(ok, value);
        }
    }

    // How long an in-flight fetch is trusted before it is written off. Well
    // clear of anything legitimate: a serial command times out on its own after
    // NEATO_CMD_TIMEOUT_MS (3 s) and the queue holds at most NEATO_QUEUE_MAX_SIZE
    // (16) of them, so even a full queue drains long before this.
    static const unsigned long FETCH_GIVE_UP_MS = 60000;

    unsigned long ttl;
    FetchFunc fetcher;
    HitFunc hitFunc;
    SemaphoreHandle_t mutex;

    T cached;
    unsigned long cachedAt = 0;
    bool hasValue = false;
    bool fetching = false;
    unsigned long fetchStartedAt = 0;
    unsigned long generation = 0;
    std::vector<Callback> waiters;
};

#endif // ASYNC_CACHE_H
