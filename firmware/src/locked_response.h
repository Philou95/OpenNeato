#ifndef LOCKED_RESPONSE_H
#define LOCKED_RESPONSE_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>

// A string response that can safely be sent from the loop task.
//
// Most of this API answers late: the handler calls request->pause() on the
// AsyncTCP task, the serial command is queued, and the reply is written back
// out of NeatoSerial::tick() -- on the **loop task**. That is the design, and
// ESPAsyncWebServer supports it. What it does not do is guard the response
// object while two tasks are in it:
//
//   * request->send() runs _respond() on the loop task, which fills the TCP
//     send buffer and only then updates _sentLength and _state;
//   * the acknowledgement for those very bytes arrives on the AsyncTCP task,
//     which calls _ack() -- the same buffer-filling code, on the same object.
//
// Preempt the loop task inside client()->write() and the AsyncTCP task finds
// _sentLength still at zero and writes the body a second time. The client then
// receives more bytes than Content-Length promised. Measured on 2026-09-05
// against /api/analog: 12 bursts of 3 concurrent requests, and curl caught one
// of the 36 with `excess = 112, size = 168`. Home Assistant sees it as
// `Data after 'Connection: close'` and drops the reading -- several times an
// hour, on /api/analog and /api/motors.
//
// Same shape as the two races already closed here: AsyncCache's waiter vector
// (2026-08-31) and SPIFFS behind FsLock. Two tasks, one object, no lock.
//
// So the fill is serialised. One mutex for every response of this class rather
// than one each: it is held for a buffer fill and nothing else, and a response
// object per request would mean a semaphore per request. It must be a mutex and
// not a critical section -- client()->write() blocks on the LwIP task, which is
// not allowed inside portENTER_CRITICAL, and cannot deadlock against us because
// the LwIP task never takes this lock.
class LockedResponse : public AsyncWebServerResponse {
public:
    LockedResponse(int code, const char *contentType, const String& content);

    void _respond(AsyncWebServerRequest *request) override;
    size_t _ack(AsyncWebServerRequest *request, size_t len, uint32_t time) override;
    bool _sourceValid() const override { return true; }

private:
    // Fill the client's send buffer with whatever is left of head then body.
    // Callers hold the lock. Mirrors AsyncBasicResponse::write_send_buffs().
    size_t fill(AsyncWebServerRequest *request, size_t len);

    String body;
    String head;
    size_t headWritten = 0;
};

#endif // LOCKED_RESPONSE_H
