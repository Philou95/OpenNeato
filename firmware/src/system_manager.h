#ifndef SYSTEM_MANAGER_H
#define SYSTEM_MANAGER_H

#include <Arduino.h>
#include <Preferences.h>
#include <atomic>
#include <esp_task_wdt.h>
#include <functional>
#include "config.h"
#include "json_fields.h"
#include "loop_task.h"

// System health snapshot — returned by SystemManager, serialized by caller
struct SystemHealth : public JsonSerializable {
    size_t heap = 0;
    size_t heapTotal = 0;
    unsigned long uptime = 0;
    int rssi = 0;
    size_t fsUsed = 0;
    size_t fsTotal = 0;
    bool ntpSynced = false;
    time_t time = 0;
    String timeSource;
    String tz;
    String localTime; // DST-aware local time string, e.g. "Sat 17:45:01"
    bool isDst = false; // true when daylight saving time is active
    // What the network watchdog is looking at. Exposed because the failure it
    // guards against is invisible to heap and RSSI: on 2026-08-28 both were
    // healthy while nothing larger than 70 bytes could leave the device.
    unsigned long httpOldestPendingMs = 0;
    unsigned long httpServedFast = 0;
    unsigned long httpServedSlow = 0;
    // Why this boot happened, and how many boots there have been. The bridge
    // restarted 5 to 8 times a day for the week to 2026-08-31 with the heap at
    // 124 KB and the RSSI at -67 dBm right up to the moment it went -- so
    // neither of the two things already reported could say what caused it, and
    // the boot event that could is only written when logging is on, which it is
    // not by default. Here it costs nothing and is always there.
    String resetReason;
    uint32_t bootCount = 0;
    // Smallest free space loopTask's stack has ever had, as returned by
    // uxTaskGetStackHighWaterMark(). Exposed because the bridge spent
    // 2026-09-01 14:34 in a reboot loop from a stack overflow nobody could see
    // coming: the default 8 KB was enough for idle polling and not enough once
    // a cleaning put the LIDAR loop on the same task. A margin that is only ever
    // discovered by exhausting it is not a margin.
    uint32_t loopStackHwm = 0;

    std::vector<Field> toFields() const override;
};

class SystemManager : public LoopTask {
public:
    explicit SystemManager(Preferences& prefs);

    void begin();
    void refreshStorage(); // setup after SPIFFS mount, then loop task only

    // Task Watchdog Timer — must be called from setup() after all slow init,
    // and feedTaskWdt() from every loop() iteration to prevent TWDT reset.
    void initTaskWdt();
    void feedTaskWdt();

    // Best-available epoch (NTP > fallback clock > millis)
    time_t now() const;

    // NTP status
    bool isNtpSynced() const { return ntpSynced; }

    // External clock fallback (e.g. robot clock parsed from GetTime)
    void setFallbackClock(time_t epoch);

    // Apply a timezone string (reconfigures NTP, does NOT store to NVS)
    void applyTimezone(const String& tz);

    // System health snapshot (heap, uptime, RSSI, storage, NTP, time)
    // Caller must supply tz string (owned by SettingsManager, not SystemManager)
    SystemHealth getSystemHealth(const String& tz) const;

    // Deferred restart — schedules a reboot after 500ms so HTTP response can flush
    void restart();

    // Deferred factory reset — clears NVS + WiFi, then restarts
    void factoryReset();

    // Deferred filesystem format — erases all logs/map data, then restarts
    void formatFs();

    // True if a deferred reboot is pending (restart or factory reset)
    bool isRebootPending() const { return pendingRebootAt > 0; }

    // Must be called from loop() — executes deferred reboot when timer expires
    void checkPendingReboot();

    // Callback fired once when NTP first syncs
    using NtpSyncCallback = std::function<void()>;
    void onNtpSync(NtpSyncCallback cb) { ntpSyncCallback = cb; }

private:
    std::atomic<size_t> cachedFsUsed{0};
    std::atomic<size_t> cachedFsTotal{0};
    unsigned long storageSampleAt = 0;
    void tick() override; // Runs every 5000ms — NTP sync detection + heap watchdog

    Preferences& prefs;
    bool ntpSynced = false;
    bool fallbackSet = false;
    time_t fallbackEpoch = 0;
    unsigned long fallbackMillis = 0;

    // Deferred reboot state
    unsigned long pendingRebootAt = 0;
    bool pendingFactoryReset = false;
    bool pendingFormatFs = false;

    // Heap watchdog state
    unsigned long heapLowSince = 0; // millis() when heap first dropped below threshold (0 = healthy)

    // Read once in begin(): esp_reset_reason() is fixed for the life of a boot,
    // and the counter is bumped in NVS there so a restart no one watched still
    // leaves a trace.
    String resetReason;
    uint32_t bootCount = 0;

    // Sampled in tick(), which runs on loopTask -- the task being measured.
    // getSystemHealth() runs on the AsyncTCP task and would otherwise report the
    // web server's stack instead.
    uint32_t loopStackHwm = 0;

    NtpSyncCallback ntpSyncCallback;
};

#endif // SYSTEM_MANAGER_H
