#pragma once

#include <Arduino.h>

// Serialize one field at a time: no vector or temporary escaped strings.
// A failed String allocation invalidates the entire response, never a field.
class CheckedJson {
public:
    explicit CheckedJson(String& output) : output(output) {
        output = "";
        valid = output.reserve(2048) && output.concat('{');
    }

    void field(const char *key, const String& value, bool quoted) {
        if (!valid)
            return;
        if (!first)
            append(',');
        first = false;
        append('"');
        append(key);
        append("\":");
        if (quoted) {
            append('"');
            for (size_t i = 0; valid && i < value.length(); ++i) {
                unsigned char c = value[i];
                if (c == '"' || c == '\\') {
                    append('\\');
                    append(static_cast<char>(c));
                } else if (c < 0x20) {
                    char escaped[7];
                    snprintf(escaped, sizeof(escaped), "\\u%04x", c);
                    append(escaped);
                } else {
                    append(static_cast<char>(c));
                }
            }
            append('"');
        } else {
            valid = valid && output.concat(value);
        }
    }

    bool finish() {
        append('}');
        if (!valid)
            output = "";
        return valid;
    }

private:
    String& output;
    bool valid = false;
    bool first = true;
    void append(char c) { valid = valid && output.concat(c); }
    void append(const char *s) { valid = valid && output.concat(s); }
};
