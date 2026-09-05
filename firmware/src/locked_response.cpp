#include "locked_response.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace {

    // Shared by every LockedResponse. Created on first use, never destroyed --
    // the web server outlives everything that could want it back.
    SemaphoreHandle_t fillMutex() {
        static SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
        return mutex;
    }

} // namespace

LockedResponse::LockedResponse(int code, const char *contentType, const String& content) : body(content) {
    _code = code;
    _contentType = contentType;
    if (body.length()) {
        _contentLength = body.length();
        if (!_contentType.length())
            _contentType = "text/plain";
    }
    addHeader("Connection", "close", false);
}

void LockedResponse::_respond(AsyncWebServerRequest *request) {
    xSemaphoreTake(fillMutex(), portMAX_DELAY);
    _state = RESPONSE_HEADERS;
    _assembleHead(head, request->version());
    fill(request, 0);
    xSemaphoreGive(fillMutex());
}

size_t LockedResponse::_ack(AsyncWebServerRequest *request, size_t len, uint32_t time) {
    (void) time;
    xSemaphoreTake(fillMutex(), portMAX_DELAY);
    size_t written = fill(request, len);
    xSemaphoreGive(fillMutex());
    return written;
}

size_t LockedResponse::fill(AsyncWebServerRequest *request, size_t len) {
    _ackedLength += len;
    size_t payload = 0;

    if (_state == RESPONSE_HEADERS) {
        size_t written = request->client()->add(head.c_str() + headWritten, head.length() - headWritten);
        _writtenLength += written;
        headWritten += written;
        if (headWritten < head.length()) {
            // The headers did not fit. Push what we have and come back on the
            // next acknowledgement for the rest.
            if (!request->client()->send())
                request->client()->close();
            return written;
        }
        _state = RESPONSE_CONTENT;
        payload += written;
        head = String();
    }

    if (_state == RESPONSE_CONTENT) {
        size_t written = request->client()->write(body.c_str() + _sentLength, body.length() - _sentLength);
        _writtenLength += written;
        _sentLength += written;
        payload += written;
        if (_sentLength >= body.length())
            _state = RESPONSE_END;
    }

    return payload;
}
