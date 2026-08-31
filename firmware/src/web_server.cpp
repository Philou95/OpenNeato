#include "web_server.h"
#include "web_assets.h"
#include "neato_serial.h"
#include "data_logger.h"
#include "system_manager.h"
#include "settings_manager.h"
#include "firmware_manager.h"
#include "manual_clean_manager.h"
#include "notification_manager.h"
#include "cleaning_history.h"
#include "wifi_manager.h"
#include "scheduler.h"
#include <SPIFFS.h>
#include <esp_core_dump.h>

unsigned long WebServer::lastApiActivity = 0;
unsigned long WebServer::pendingSince[WebServer::PENDING_SLOTS] = {0};
unsigned long WebServer::fastCount = 0;
unsigned long WebServer::slowCount = 0;
portMUX_TYPE WebServer::pendingMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long WebServer::noteRequest(AsyncWebServerRequest *request) {
    // millis() is 0 for the first millisecond after boot and again every 49
    // days; 1 is indistinguishable in practice and keeps 0 meaning "free".
    unsigned long now = millis();
    unsigned long stamp = now ? now : 1;
    lastApiActivity = now;

    int slot = -1;
    portENTER_CRITICAL(&pendingMux);
    for (uint8_t i = 0; i < PENDING_SLOTS; i++) {
        if (pendingSince[i] == 0) {
            pendingSince[i] = stamp;
            slot = i;
            break;
        }
    }
    portEXIT_CRITICAL(&pendingMux);

    if (slot >= 0) {
        // Fires when the connection closes, which this server does the moment
        // the response is finished — so this is where the answer is counted.
        request->onDisconnect([slot, stamp]() {
            unsigned long took = millis() - stamp;
            portENTER_CRITICAL(&pendingMux);
            pendingSince[slot] = 0;
            if (took >= NET_WDT_SLOW_MS)
                slowCount++;
            else
                fastCount++;
            portEXIT_CRITICAL(&pendingMux);
        });
    }
    return now;
}

unsigned long WebServer::oldestPendingMs() {
    unsigned long now = millis();
    unsigned long oldest = 0;
    portENTER_CRITICAL(&pendingMux);
    for (uint8_t i = 0; i < PENDING_SLOTS; i++) {
        if (pendingSince[i] == 0)
            continue;
        unsigned long age = now - pendingSince[i];
        if (age > oldest)
            oldest = age;
    }
    portEXIT_CRITICAL(&pendingMux);
    return oldest;
}

unsigned long WebServer::servedFast() {
    portENTER_CRITICAL(&pendingMux);
    unsigned long n = fastCount;
    portEXIT_CRITICAL(&pendingMux);
    return n;
}

unsigned long WebServer::servedSlow() {
    portENTER_CRITICAL(&pendingMux);
    unsigned long n = slowCount;
    portEXIT_CRITICAL(&pendingMux);
    return n;
}

void WebServer::resetServedCounts() {
    portENTER_CRITICAL(&pendingMux);
    fastCount = 0;
    slowCount = 0;
    portEXIT_CRITICAL(&pendingMux);
}

WebServer::WebServer(AsyncWebServer& server, NeatoSerial& neato, DataLogger& logger, SystemManager& sys,
                     FirmwareManager& fw, SettingsManager& settings, ManualCleanManager& manual,
                     NotificationManager& notif, CleaningHistory& history, WiFiManager& wifi, Scheduler& scheduler) :
    server(server), neato(neato), logger(logger), sysMgr(sys), fwMgr(fw), settingsMgr(settings), manualMgr(manual),
    notifMgr(notif), historyMgr(history), wifiMgr(wifi), scheduler(scheduler) {}

void WebServer::loggedRoute(const char *path, WebRequestMethodComposite httpMethod, SyncHandler handler) {
    server.on(path, httpMethod, [this, handler](AsyncWebServerRequest *request) {
        unsigned long startMs = noteRequest(request);
        int status = handler(request);
        logger.logRequest(request->method(), request->url().c_str(), status, millis() - startMs);
    });
}

void WebServer::loggedBodyRoute(const char *path, WebRequestMethodComposite httpMethod, BodyHandler handler) {
    server.on(
            path, httpMethod, [](AsyncWebServerRequest *request) { /* handled in body callback */ }, nullptr,
            [this, handler](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t, size_t) {
                unsigned long startMs = noteRequest(request);
                int status = handler(request, data, len);
                logger.logRequest(request->method(), request->url().c_str(), status, millis() - startMs);
            });
}

void WebServer::sendGzipAsset(AsyncWebServerRequest *request, const uint8_t *data, size_t len,
                              const char *contentType) {
    AsyncWebServerResponse *response = request->beginResponse(200, contentType, data, len);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
}

void WebServer::sendError(AsyncWebServerRequest *request, int code, const String& msg) {
    request->send(code, "application/json", fieldsToJson({{"error", msg, FIELD_STRING}}));
}

void WebServer::sendOk(AsyncWebServerRequest *request) {
    request->send(200, "application/json", fieldsToJson({{"ok", "true", FIELD_BOOL}}));
}

void WebServer::begin() {
    // Register all embedded frontend assets from the auto-generated registry
    for (size_t i = 0; i < WEB_ASSETS_COUNT; i++) {
        const WebAsset& asset = WEB_ASSETS[i];
        server.on(asset.path, HTTP_GET, [&asset](AsyncWebServerRequest *request) {
            sendGzipAsset(request, asset.data, asset.length, asset.contentType);
        });
    }

    LOG("WEB", "Registered %u embedded assets", WEB_ASSETS_COUNT);

    registerApiRoutes();
    registerManualRoutes();
    registerLogRoutes();
    registerSystemRoutes();
    registerSettingsRoutes();
    registerFirmwareRoutes();
    registerMapRoutes();
    registerWiFiRoutes();

    LOG("WEB", "Frontend and API routes registered");
}

void WebServer::registerApiRoutes() {
    // -- Sensor query endpoints ----------------------------------------------

    registerGetRoute("/api/version", neato, &NeatoSerial::getVersion, {});
    registerGetRoute("/api/charger", neato, &NeatoSerial::getCharger, {});
    registerGetRoute("/api/analog", neato, &NeatoSerial::getBatteryAnalog, {});
    registerGetRoute("/api/warranty", neato, &NeatoSerial::getBatteryWarranty, {});
    registerGetRoute(
            "/api/motors", neato,
            static_cast<void (NeatoSerial::*)(std::function<void(bool, const MotorData&)>)>(&NeatoSerial::getMotors),
            {});
    registerGetRoute("/api/state", neato, &NeatoSerial::getState, {});
    registerGetRoute("/api/error", neato, &NeatoSerial::getErr, {});
    // Registered before /api/lidar: ESPAsyncWebServer matches routes by prefix,
    // so the shorter path would otherwise swallow this one.
    //
    // Collects the scans the bridge buffered while cleaning. `after` is the
    // highest sequence number the caller already holds; everything up to it is
    // dropped. Pure RAM on this task -- the batch was built on the loop task,
    // because reading SPIFFS here is what truncated /api/history.
    server.on("/api/lidar/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
        request->send(200, "application/json", historyMgr.scanStatusJson());
    });
    server.on("/api/lidar/buffer", HTTP_GET, [this](AsyncWebServerRequest *request) {
        unsigned long startMs = noteRequest(request);
        uint32_t after = 0;
        if (request->hasParam("after"))
            after = strtoul(request->getParam("after")->value().c_str(), nullptr, 10);
        String body = historyMgr.takeScanBatch(after);
        logger.logRequest(HTTP_GET, "/api/lidar/buffer", 200, millis() - startMs);
        request->send(200, "application/x-ndjson", body);
    });
    registerGetRoute("/api/lidar", neato, &NeatoSerial::getLdsScan, {});
    registerGetRoute("/api/user-settings", neato, &NeatoSerial::getUserSettings, {});
    registerGetRoute("/api/sensors", neato,
                     static_cast<void (NeatoSerial::*)(std::function<void(bool, const DigitalSensorData&)>)>(
                             &NeatoSerial::getDigitalSensors),
                     {});

    // -- Action endpoints ----------------------------------------------------
    // All parameterized actions use query strings: resource URL identifies the
    // command, query params carry arguments (mirrors Neato serial protocol).

    registerPostRoute("/api/clean", neato, &NeatoSerial::clean, {"action"});
    registerPostRoute("/api/sound", neato, &NeatoSerial::playSound, {"id"});
    registerPostRoute("/api/power", neato, &NeatoSerial::powerControl, {"action"});
    registerPostRoute("/api/lidar/rotate", neato, &NeatoSerial::setLdsRotation, {"enable"});
    registerPostRoute("/api/user-settings", neato, &NeatoSerial::setUserSetting, {"key", "value"});
    registerPostRoute("/api/clear-errors", neato, &NeatoSerial::clearErrors, {});
    registerPostRoute("/api/battery/new", neato, &NeatoSerial::newBattery, {});
    registerGetRoute("/api/schedule/next", scheduler, &Scheduler::getNextScheduleJson);
    registerPostRoute("/api/schedule/next", scheduler, &Scheduler::requestSkipNextClean);
    registerDeleteRoute("/api/schedule/next", scheduler, &Scheduler::cancelSkipNextClean);

    // Serial endpoint — send arbitrary serial command, returns raw response.
    // Always available (no debug gate — useful for diagnostics without enabling verbose logging).
    // Excluded from public API docs (diagnostics-only passthrough).
    server.on("/api/serial", HTTP_POST, [this](AsyncWebServerRequest *request) {
        unsigned long startMs = noteRequest(request);

        if (!request->hasParam("cmd")) {
            logger.logRequest(HTTP_POST, "/api/serial", 400, millis() - startMs);
            sendError(request, 400, "missing cmd");
            return;
        }
        String cmd = request->getParam("cmd")->value();
        if (cmd.isEmpty()) {
            logger.logRequest(HTTP_POST, "/api/serial", 400, millis() - startMs);
            sendError(request, 400, "empty cmd");
            return;
        }

        auto weak = request->pause();
        bool ok = neato.sendRaw(cmd, [this, weak, startMs](bool /*success*/, const String& response) {
            if (auto req = weak.lock()) {
                unsigned long elapsed = millis() - startMs;
                logger.logRequest(HTTP_POST, "/api/serial", 200, elapsed);
                req->send(200, "text/plain", response);
            }
        });
        if (!ok) {
            logger.logRequest(HTTP_POST, "/api/serial", 503, millis() - startMs);
            sendError(request, 503, "unavailable");
        }
    });

    LOG("WEB", "API routes registered");
}

// -- Manual clean endpoints ---------------------------------------------------

void WebServer::registerManualRoutes() {
    // Register longer paths first — ESPAsyncWebServer matches routes by prefix,
    // so /api/manual would swallow /api/manual/move and /api/manual/motors.

    registerGetRoute("/api/manual/status", manualMgr, &ManualCleanManager::getStatusJson);
    registerPostRoute("/api/manual/move", manualMgr, &ManualCleanManager::move, {"left", "right", "speed"});
    registerPostRoute("/api/manual/motors", manualMgr, &ManualCleanManager::setMotors,
                      {"brush", "vacuum", "sideBrush"});
    registerPostRoute("/api/manual", manualMgr, &ManualCleanManager::enable, {"enable"});

    LOG("WEB", "Manual clean routes registered");
}

// -- Log file endpoints ------------------------------------------------------

// Strip .hs extension so browser saves a plain .jsonl file
static String downloadName(const String& filename) {
    if (filename.endsWith(".hs"))
        return filename.substring(0, filename.length() - 3);
    return filename;
}

static String logListJson(const std::vector<LogFileInfo>& files) {
    String json = "[";
    for (size_t i = 0; i < files.size(); i++) {
        if (i > 0)
            json += ",";
        json += files[i].toJson();
    }
    json += "]";
    return json;
}

void WebServer::registerLogRoutes() {

    // GET /api/logs[/filename] — list logs or download a specific file
    // A single BackwardCompatible handler matches both "/api/logs" and "/api/logs/..."
    // This route uses server.on() directly instead of loggedRoute() because
    // compressed log downloads use chunked streaming (beginChunkedResponse) which
    // must not block — loggedRoute's sync wrapper would block until completion.
    server.on("/api/logs", HTTP_GET, [this](AsyncWebServerRequest *request) {
        unsigned long startMs = millis();
        String filename = request->url().substring(String("/api/logs/").length());

        if (filename.isEmpty()) {
            String json = logListJson(logger.listLogs());
            logger.logRequest(HTTP_GET, "/api/logs", 200, millis() - startMs);
            request->send(200, "application/json", json);
            return;
        }

        // Open log via DataLogger — handles path resolution and transparent decompression
        auto reader = logger.readLog(filename);
        if (!reader) {
            logger.logRequest(HTTP_GET, request->url().c_str(), 404, millis() - startMs);
            sendError(request, 404, "log not found");
            return;
        }

        logger.logRequest(HTTP_GET, request->url().c_str(), 200, millis() - startMs);

        // Stream log content via chunked response — reader handles decompression
        AsyncWebServerResponse *response = request->beginChunkedResponse(
                "application/x-ndjson",
                [reader](uint8_t *buffer, size_t maxLen, size_t) -> size_t { return reader->read(buffer, maxLen); });

        response->addHeader("Content-Disposition", "attachment; filename=\"" + downloadName(filename) + "\"");

        request->send(response);
    });

    // DELETE /api/logs[/filename] — delete all logs or a specific file
    loggedRoute("/api/logs", HTTP_DELETE, [this](AsyncWebServerRequest *request) -> int {
        String filename = request->url().substring(String("/api/logs/").length());

        if (filename.isEmpty()) {
            logger.deleteAllLogs();
            sendOk(request);
            return 200;
        }

        if (logger.deleteLog(filename)) {
            sendOk(request);
            return 200;
        }

        sendError(request, 404, "log not found");
        return 404;
    });

    LOG("WEB", "Log routes registered");
}

// -- System health endpoint ---------------------------------------------------

void WebServer::registerSystemRoutes() {

    // GET /api/system — live system health (heap, uptime, RSSI, storage, NTP)
    loggedRoute("/api/system", HTTP_GET, [this](AsyncWebServerRequest *request) -> int {
        request->send(200, "application/json", sysMgr.getSystemHealth(settingsMgr.get().tz).toJson());
        return 200;
    });


    // GET /api/coredump -- the stored crash dump, verbatim.
    //
    // The boot event carries the registers the summary names, and on RISC-V
    // that is two frames: the faulting PC and whatever `ra` held. Two frames
    // were not enough for the crash of 2026-08-31 23:26 -- `ra` landed inside
    // String::move(), which is reached from 82 call sites in this binary, so
    // the caller is the one thing that matters and the one thing missing.
    //
    // On RISC-V `exc_bt_info` is not a backtrace array but the raw stack of the
    // crashing task, so every return address is already sitting in flash; they
    // only have to be read off the host and resolved against the ELF of the
    // build that crashed.
    //
    // Read-only, and it does not erase. The panic handler overwrites the dump
    // on the next crash and nothing else touches it -- which is exactly why
    // this endpoint could be added *after* the crash it was needed for and
    // still find that crash waiting.
    loggedRoute("/api/coredump", HTTP_GET, [this](AsyncWebServerRequest *request) -> int {
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH && CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
        if (esp_core_dump_image_check() != ESP_OK) {
            sendError(request, 404, "no core dump stored");
            return 404;
        }
        // ~1.1 KB, so heap rather than the AsyncTCP task's stack.
        auto *cd = static_cast<esp_core_dump_summary_t *>(malloc(sizeof(esp_core_dump_summary_t)));
        if (!cd) {
            sendError(request, 507, "out of memory");
            return 507;
        }
        if (esp_core_dump_get_summary(cd) != ESP_OK) {
            free(cd);
            sendError(request, 500, "core dump unreadable");
            return 500;
        }
        std::vector<Field> fields = {
                {"task", String(cd->exc_task), FIELD_STRING},
                {"pc", "0x" + String(cd->exc_pc, HEX), FIELD_STRING},
                {"elf", String(reinterpret_cast<char *>(cd->app_elf_sha256)), FIELD_STRING},
        };
#if CONFIG_IDF_TARGET_ARCH_RISCV
        fields.push_back({"ra", "0x" + String(cd->ex_info.ra, HEX), FIELD_STRING});
        fields.push_back({"sp", "0x" + String(cd->ex_info.sp, HEX), FIELD_STRING});
        fields.push_back({"cause", String(cd->ex_info.mcause), FIELD_INT});
        fields.push_back({"addr", "0x" + String(cd->ex_info.mtval, HEX), FIELD_STRING});
        // The crashing task's stack, lowest address first, starting at `sp`.
        // Hex rather than base64 so it can be read with nothing but a shell.
        String stack;
        stack.reserve(cd->exc_bt_info.dump_size * 2 + 1);
        for (uint32_t i = 0; i < cd->exc_bt_info.dump_size; i++) {
            char b[3];
            snprintf(b, sizeof(b), "%02x", cd->exc_bt_info.stackdump[i]);
            stack += b;
        }
        fields.push_back({"stackSize", String(cd->exc_bt_info.dump_size), FIELD_INT});
        fields.push_back({"stack", stack, FIELD_STRING});
#else
        fields.push_back({"cause", String(cd->ex_info.exc_cause), FIELD_INT});
        fields.push_back({"addr", "0x" + String(cd->ex_info.exc_vaddr, HEX), FIELD_STRING});
        String bt;
        for (uint32_t i = 0; i < cd->exc_bt_info.depth && i < 16; i++)
            bt += (i ? " 0x" : "0x") + String(cd->exc_bt_info.bt[i], HEX);
        fields.push_back({"depth", String(cd->exc_bt_info.depth), FIELD_INT});
        fields.push_back({"corrupted", cd->exc_bt_info.corrupted ? "true" : "false", FIELD_BOOL});
        fields.push_back({"backtrace", bt, FIELD_STRING});
#endif
        free(cd);
        request->send(200, "application/json", fieldsToJson(fields));
        return 200;
#else
        sendError(request, 501, "core dump support not built into this firmware");
        return 501;
#endif
    });
    // Actions defer their reboot for 500ms, allowing the response to flush.
    registerPostRoute("/api/system/restart", sysMgr, &SystemManager::restart);
    registerPostRoute("/api/system/reset", sysMgr, &SystemManager::factoryReset);
    registerPostRoute("/api/system/format-fs", sysMgr, &SystemManager::formatFs);

    LOG("WEB", "System routes registered");
}

// -- Settings endpoint -------------------------------------------------------

void WebServer::registerSettingsRoutes() {

    // GET /api/settings — all user-configurable settings
    registerGetRoute("/api/settings", settingsMgr, &SettingsManager::get);

    // PUT /api/settings — partial update (only fields present are written)
    loggedBodyRoute("/api/settings", HTTP_PUT,
                    [this](AsyncWebServerRequest *request, uint8_t *data, size_t len) -> int {
                        String body = String(reinterpret_cast<const char *>(data), len);
                        ApplyResult result = settingsMgr.apply(body);
                        if (result == APPLY_INVALID) {
                            sendError(request, 400, "Invalid settings");
                            return 400;
                        }
                        if (result == APPLY_CHANGED) {
                            // Push manual clean settings to manager (no reboot needed)
                            const auto& s = settingsMgr.get();
                            manualMgr.setStallThreshold(s.stallThreshold);
                            manualMgr.setBrushRpm(s.brushRpm);
                            manualMgr.setVacuumSpeed(s.vacuumSpeed);
                            manualMgr.setSideBrushPower(s.sideBrushPower);
                        }
                        request->send(200, "application/json", settingsMgr.get().toJson());
                        return 200;
                    });

    // POST /api/notifications/test?topic=<topic> — send a test notification
    loggedRoute("/api/notifications/test", HTTP_POST, [this](AsyncWebServerRequest *request) -> int {
        if (!request->hasParam("topic")) {
            sendError(request, 400, "missing topic");
            return 400;
        }
        String topic = request->getParam("topic")->value();
        if (topic.isEmpty()) {
            sendError(request, 400, "topic cannot be empty");
            return 400;
        }
        notifMgr.sendTestNotification(topic);
        sendOk(request);
        return 200;
    });

    LOG("WEB", "Settings routes registered");
}

// -- Firmware endpoints -------------------------------------------------------

void WebServer::registerFirmwareRoutes() {

    // GET /api/firmware/version — current ESP32 firmware version + chip model + robot support status
    loggedRoute("/api/firmware/version", HTTP_GET, [this](AsyncWebServerRequest *request) -> int {
        std::vector<Field> fields = {
                {"name", "OpenNeato", FIELD_STRING},
                {"version", fwMgr.getFirmwareVersion(), FIELD_STRING},
                {"chip", fwMgr.getChipModel(), FIELD_STRING},
                {"model", neato.getModelName(), FIELD_STRING},
                {"hostname", settingsMgr.get().hostname, FIELD_STRING},
                {"supported", isSupportedModel(neato.getModelName()) ? "true" : "false", FIELD_BOOL},
                {"identifying", neato.isIdentifying() ? "true" : "false", FIELD_BOOL},
                {"repositoryUrl", "https://github.com/renjfk/OpenNeato", FIELD_STRING},
                {"license", "MIT", FIELD_STRING},
        };
        request->send(200, "application/json", fieldsToJson(fields));
        return 200;
    });

    // POST /api/firmware/update?hash=<md5> — single-request firmware upload
    server.on(
            "/api/firmware/update", HTTP_POST,
            // Response handler (called after upload completes)
            [this](AsyncWebServerRequest *request) {
                unsigned long startMs = millis();
                bool ok = fwMgr.getError().isEmpty();

                if (ok) {
                    ok = fwMgr.endUpdate();
                }

                if (!ok) {
                    logger.logRequest(HTTP_POST, "/api/firmware/update", 400, millis() - startMs);
                    sendError(request, 400, fwMgr.getError());
                } else {
                    logger.logRequest(HTTP_POST, "/api/firmware/update", 200, millis() - startMs);
                    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
                    response->addHeader("Connection", "close");
                    request->send(response);
                }
            },
            // Upload handler (called per chunk)
            [this](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len,
                   bool final) {
                // First chunk: initialize update session
                if (!index) {
                    String md5 = request->hasParam("hash") ? request->getParam("hash")->value() : "";
                    if (!fwMgr.beginUpdate(md5)) {
                        return;
                    }
                }

                if (len) {
                    fwMgr.writeChunk(data, len);

                    // Report progress at most once per second
                    static unsigned long lastProgressMs = 0;
                    unsigned long now = millis();
                    if (now - lastProgressMs >= 1000) {
                        auto percent = static_cast<uint8_t>(
                                request->contentLength() > 0 ? static_cast<float>(fwMgr.getProgress()) * 100.0f /
                                                                       static_cast<float>(request->contentLength())
                                                             : 0);
                        LOG("FW", "Progress: %u%% (%zu/%zu bytes)", percent, fwMgr.getProgress(),
                            request->contentLength());
                        lastProgressMs = now;
                    }
                }
            });

    LOG("WEB", "Firmware routes registered");
}

// -- Map data endpoints -------------------------------------------------------

void WebServer::registerMapRoutes() {

    // GET /api/history[/filename] — list sessions, collection status, or download a specific file
    server.on("/api/history", HTTP_GET, [this](AsyncWebServerRequest *request) {
        unsigned long startMs = noteRequest(request);
        String suffix = request->url().substring(String("/api/history/").length());

        if (suffix.isEmpty()) {
            // Serve the listing CleaningHistory pre-built on the loop task.
            // Enumerating and decompressing session files from this AsyncTCP
            // callback stalls it against the loop's SPIFFS writes during a
            // clean, and the server then aborts the body mid-send.
            logger.logRequest(HTTP_GET, "/api/history", 200, millis() - startMs);
            request->send(200, "application/json", historyMgr.getListJson());
            return;
        }

        // Download specific session
        auto reader = historyMgr.readSession(suffix);
        if (!reader) {
            logger.logRequest(HTTP_GET, request->url().c_str(), 404, millis() - startMs);
            sendError(request, 404, "session not found");
            return;
        }

        logger.logRequest(HTTP_GET, request->url().c_str(), 200, millis() - startMs);

        AsyncWebServerResponse *response = request->beginChunkedResponse(
                "application/x-ndjson",
                [reader](uint8_t *buffer, size_t maxLen, size_t) -> size_t { return reader->read(buffer, maxLen); });

        response->addHeader("Content-Disposition", "attachment; filename=\"" + downloadName(suffix) + "\"");

        request->send(response);
    });

    // DELETE /api/history[/filename] — delete one or all sessions
    loggedRoute("/api/history", HTTP_DELETE, [this](AsyncWebServerRequest *request) -> int {
        String filename = request->url().substring(String("/api/history/").length());

        if (filename.isEmpty()) {
            historyMgr.deleteAllSessions();
            sendOk(request);
            return 200;
        }

        if (historyMgr.deleteSession(filename)) {
            sendOk(request);
            return 200;
        }

        sendError(request, 404, "session not found");
        return 404;
    });

    // POST /api/history/import — upload a .jsonl session file, compress and store
    server.on(
            "/api/history/import", HTTP_POST,
            // Response handler (called after upload completes)
            [this](AsyncWebServerRequest *request) {
                unsigned long startMs = millis();
                bool ok = historyMgr.getImportError().isEmpty();

                if (ok) {
                    ok = historyMgr.endImport();
                }

                if (!ok) {
                    logger.logRequest(HTTP_POST, "/api/history/import", 400, millis() - startMs);
                    sendError(request, 400, historyMgr.getImportError());
                } else {
                    logger.logRequest(HTTP_POST, "/api/history/import", 200, millis() - startMs);
                    sendOk(request);
                }
            },
            // Upload handler (called per chunk)
            [this](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len,
                   bool final) {
                // First chunk: initialize import session
                if (!index) {
                    if (!historyMgr.beginImport(filename)) {
                        return;
                    }
                }

                if (len && historyMgr.isImporting()) {
                    historyMgr.writeImportChunk(data, len);
                }
            });

    LOG("WEB", "History routes registered");
}

// -- WiFi management endpoints -----------------------------------------------

void WebServer::registerWiFiRoutes() {
    // GET /api/wifi/status , STA + fallback AP snapshot
    registerGetRoute("/api/wifi/status", wifiMgr, &WiFiManager::getStatus, {});

    // GET /api/wifi/scan , list nearby networks
    registerGetRoute("/api/wifi/scan", wifiMgr, &WiFiManager::scanNetworks, {});

    // POST /api/wifi/connect?ssid=&password= , save credentials and connect.
    // On success the device reboots into normal STA mode; on failure the
    // fallback AP stays up so the user can retry.
    registerPostRoute("/api/wifi/connect", wifiMgr, &WiFiManager::connect, {"ssid", "password"});

    // POST /api/wifi/disconnect , clear credentials and drop the connection
    registerPostRoute("/api/wifi/disconnect", wifiMgr, &WiFiManager::disconnect, {});

    LOG("WEB", "WiFi routes registered");
}
