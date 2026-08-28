#ifndef ASYNC_CACHE_H
#define ASYNC_CACHE_H

#include <Arduino.h>
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
        ttl(ttlMs), fetcher(fetchFunc), hitFunc(hitFunc) {}

    // Request the value. Returns cached data instantly if fresh, otherwise
    // triggers a fetch (or piggybacks on an in-flight one).
    void get(Callback callback) {
        // Cache hit — value is fresh
        if (hasValue && (millis() - cachedAt < ttl)) {
            if (hitFunc)
                hitFunc(millis() - cachedAt);
            if (callback)
                callback(true, cached);
            return;
        }

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
            auto abandoned = std::move(waiters);
            waiters.clear();
            for (auto& cb: abandoned) {
                cb(false, cached);
            }
        }

        // Add to waiters list
        if (callback)
            waiters.push_back(callback);

        // If a fetch is already in flight, this request will be served when it completes
        if (fetching)
            return;

        // Trigger a new fetch
        fetching = true;
        fetchStartedAt = millis();
        // Tagged, so a producer abandoned above cannot come back to life later
        // and clear `fetching` out from under the fetch that replaced it.
        unsigned long gen = ++generation;
        fetcher([this, gen](bool ok, const T& data) {
            if (gen != generation)
                return;
            fetching = false;

            if (ok) {
                cached = data;
                cachedAt = millis();
                hasValue = true;
            }

            // Deliver to all waiters — move the vector to avoid re-entrancy issues
            // (a waiter's callback could call get() again)
            auto pending = std::move(waiters);
            waiters.clear();
            for (auto& cb: pending) {
                cb(ok, ok ? cached : data);
            }
        });
    }

    // Mark cached value as stale — next get() will trigger a fresh fetch.
    void invalidate() { hasValue = false; }

    // Check whether a cached value exists (regardless of freshness)
    bool hasCached() const { return hasValue; }

    // Get the last cached value directly (for non-async access patterns)
    const T& getCached() const { return cached; }

private:
    // How long an in-flight fetch is trusted before it is written off. Well
    // clear of anything legitimate: a serial command times out on its own after
    // NEATO_CMD_TIMEOUT_MS (3 s) and the queue holds at most NEATO_QUEUE_MAX_SIZE
    // (16) of them, so even a full queue drains long before this.
    static const unsigned long FETCH_GIVE_UP_MS = 60000;

    unsigned long ttl;
    FetchFunc fetcher;
    HitFunc hitFunc;

    T cached;
    unsigned long cachedAt = 0;
    bool hasValue = false;
    bool fetching = false;
    unsigned long fetchStartedAt = 0;
    unsigned long generation = 0;
    std::vector<Callback> waiters;
};

#endif // ASYNC_CACHE_H
