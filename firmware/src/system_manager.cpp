#include "system_manager.h"
#include "web_server.h"
#include <SPIFFS.h>
#include <WiFi.h>
#include <ctime>
#include <esp_idf_version.h>
#include <esp_system.h>

SystemManager::SystemManager(Preferences& prefs) : LoopTask(5000), prefs(prefs) {
    TaskRegistry::add(this);
}

// -- Lifecycle ---------------------------------------------------------------

// Names match DataLogger::logBootEvent() so the two agree when both are read.
static const char *resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:
            return "POWERON";
        case ESP_RST_EXT:
            return "EXT";
        case ESP_RST_SW:
            return "SW_RESET";
        case ESP_RST_PANIC:
            return "PANIC";
        case ESP_RST_INT_WDT:
            return "INT_WDT";
        case ESP_RST_TASK_WDT:
            return "TASK_WDT";
        case ESP_RST_WDT:
            return "WDT";
        case ESP_RST_DEEPSLEEP:
            return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:
            return "BROWNOUT";
        case ESP_RST_SDIO:
            return "SDIO";
        default:
            return "UNKNOWN";
    }
}

void SystemManager::begin() {
    // Why we are running. A brownout, a panic and a watchdog bite all look
    // identical from the outside -- the device is simply back with uptime near
    // zero -- and telling them apart is the whole difference between fixing the
    // power supply and fixing the code.
    resetReason = resetReasonName(esp_reset_reason());
    // No begin()/end() around this: main.cpp opens the namespace once for the
    // lifetime of the device, and closing it here would pull the handle out
    // from under every other manager.
    bootCount = prefs.getUInt(NVS_KEY_BOOT_COUNT, 0) + 1;
    prefs.putUInt(NVS_KEY_BOOT_COUNT, bootCount);
    LOG("SYS", "Boot #%u, reason %s", bootCount, resetReason.c_str());
    // NTP is configured by SettingsManager via applyTimezone() after settings are loaded
    LOG("SYS", "System manager initialized (NTP pending timezone from settings)");
}

// -- Task Watchdog Timer -----------------------------------------------------

void SystemManager::initTaskWdt() {
    // Initialize TWDT with configured timeout. If loop() stops calling
    // feedTaskWdt(), the hardware watchdog resets the ESP32.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
    // ESP-IDF 5.3+ uses config struct API
    const esp_task_wdt_config_t wdtCfg = {
            .timeout_ms = TASK_WDT_TIMEOUT_S * 1000,
            .idle_core_mask = 0,
            .trigger_panic = true,
    };
    esp_task_wdt_init(&wdtCfg);
#else
    esp_task_wdt_init(TASK_WDT_TIMEOUT_S, true); // true = panic (reset) on timeout
#endif
    esp_task_wdt_add(nullptr); // nullptr = subscribe current task (loopTask)
    LOG("SYS", "Task watchdog initialized (%ds timeout)", TASK_WDT_TIMEOUT_S);
}

void SystemManager::feedTaskWdt() {
    esp_task_wdt_reset();
}

void SystemManager::tick() {
    // On loopTask, so nullptr means loopTask -- see the note in the header.
    loopStackHwm = uxTaskGetStackHighWaterMark(nullptr);

    // Detect NTP sync transition
    if (!ntpSynced) {
        time_t t = time(nullptr);
        if (t > 1700000000) {
            ntpSynced = true;
            LOG("SYS", "NTP synced: %ld", static_cast<long>(t));
            if (ntpSyncCallback) {
                ntpSyncCallback();
            }
        }
    }

    // Heap watchdog — restart if free heap stays critically low for too long.
    // This catches scenarios where the web server exhausts sockets/memory and
    // becomes unresponsive (e.g. after a UART desync cascade). A brief dip is
    // tolerated; only sustained low heap triggers the restart.
    size_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < HEAP_WATCHDOG_THRESHOLD) {
        if (heapLowSince == 0) {
            heapLowSince = millis();
            LOG("SYS", "Heap watchdog: low heap detected (%u bytes)", freeHeap);
        } else if (millis() - heapLowSince >= HEAP_WATCHDOG_DURATION_MS) {
            LOG("SYS", "Heap watchdog: heap critically low for %lums (%u bytes), restarting", HEAP_WATCHDOG_DURATION_MS,
                freeHeap);
            ESP.restart();
        }
    } else {
        // Heap recovered — reset the timer
        if (heapLowSince > 0) {
            LOG("SYS", "Heap watchdog: heap recovered (%u bytes)", freeHeap);
            heapLowSince = 0;
        }
    }
}

// -- Time --------------------------------------------------------------------

time_t SystemManager::now() const {
    // Prefer NTP
    time_t t = time(nullptr);
    if (t > 1700000000)
        return t;

    // Fall back to external clock
    if (fallbackSet && fallbackEpoch > 0) {
        return fallbackEpoch + (millis() - fallbackMillis) / 1000;
    }

    // Last resort: uptime as pseudo-timestamp
    return static_cast<time_t>(millis() / 1000);
}

void SystemManager::setFallbackClock(time_t epoch) {
    fallbackEpoch = epoch;
    fallbackMillis = millis();
    fallbackSet = true;
    LOG("SYS", "Fallback clock set: epoch %ld", static_cast<long>(epoch));
}

// -- Timezone ----------------------------------------------------------------

void SystemManager::applyTimezone(const String& tz) {
    configTzTime(tz.c_str(), NTP_SERVER_1, NTP_SERVER_2);
    LOG("SYS", "NTP timezone applied: %s", tz.c_str());
}

// -- Deferred reboot ---------------------------------------------------------

void SystemManager::restart() {
    LOG("SYS", "Restart scheduled");
    pendingRebootAt = millis();
}

void SystemManager::factoryReset() {
    LOG("SYS", "Factory reset scheduled");
    pendingFactoryReset = true;
    pendingRebootAt = millis();
}

void SystemManager::formatFs() {
    LOG("SYS", "Filesystem format scheduled");
    pendingFormatFs = true;
    pendingRebootAt = millis();
}

void SystemManager::checkPendingReboot() {
    if (pendingRebootAt == 0 || millis() - pendingRebootAt < 500)
        return;

    if (pendingFactoryReset) {
        LOG("SYS", "Factory reset: clearing NVS, WiFi credentials, and filesystem...");
        prefs.clear();
        WiFi.disconnect(true, true);
        SPIFFS.format();
        delay(500);
    } else if (pendingFormatFs) {
        LOG("SYS", "Formatting filesystem...");
        SPIFFS.format();
        delay(500);
    }
    LOG("SYS", "Rebooting...");
    ESP.restart();
}

// -- System health -----------------------------------------------------------

std::vector<Field> SystemHealth::toFields() const {
    return {
            {"heap", String(heap), FIELD_INT},
            {"heapTotal", String(heapTotal), FIELD_INT},
            {"uptime", String(uptime), FIELD_INT},
            {"rssi", String(rssi), FIELD_INT},
            {"fsUsed", String(fsUsed), FIELD_INT},
            {"fsTotal", String(fsTotal), FIELD_INT},
            {"ntpSynced", ntpSynced ? "true" : "false", FIELD_BOOL},
            {"time", String(static_cast<long>(time)), FIELD_INT},
            {"timeSource", timeSource, FIELD_STRING},
            {"tz", tz, FIELD_STRING},
            {"localTime", localTime, FIELD_STRING},
            {"isDst", isDst ? "true" : "false", FIELD_BOOL},
            {"httpOldestPendingMs", String(httpOldestPendingMs), FIELD_INT},
            {"httpServedFast", String(httpServedFast), FIELD_INT},
            {"httpServedSlow", String(httpServedSlow), FIELD_INT},
            {"resetReason", resetReason, FIELD_STRING},
            {"bootCount", String(bootCount), FIELD_INT},
            {"loopStackHwm", String(loopStackHwm), FIELD_INT},
    };
}

SystemHealth SystemManager::getSystemHealth(const String& tz) const {
    SystemHealth h;
    h.heap = ESP.getFreeHeap();
    h.heapTotal = ESP.getHeapSize();
    h.httpOldestPendingMs = WebServer::oldestPendingMs();
    h.httpServedFast = WebServer::servedFast();
    h.httpServedSlow = WebServer::servedSlow();
    h.uptime = millis();
    h.rssi = WiFi.RSSI();
    h.fsUsed = SPIFFS.usedBytes();
    h.fsTotal = SPIFFS.totalBytes();
    h.ntpSynced = ntpSynced;
    h.time = now();
    h.timeSource = ntpSynced ? "ntp" : (fallbackSet ? "fallback" : "millis");
    h.tz = tz;
    h.resetReason = resetReason;
    h.bootCount = bootCount;
    h.loopStackHwm = loopStackHwm;

    // Compute DST-aware local time string via localtime_r (same conversion the
    // scheduler uses). The POSIX TZ string applied via configTzTime() handles
    // DST transitions, so this is always correct for the configured timezone.
    if (h.time > 1700000000) {
        struct tm tm;
        localtime_r(&h.time, &tm);
        static const char *DAYS[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
        char buf[20];
        snprintf(buf, sizeof(buf), "%s %02d:%02d:%02d", DAYS[tm.tm_wday], tm.tm_hour, tm.tm_min, tm.tm_sec);
        h.localTime = buf;
        h.isDst = tm.tm_isdst > 0;
    }

    return h;
}
