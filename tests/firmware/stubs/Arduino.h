#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

// Small host adapter for exercising the actual recovery implementation.
// File and failure behaviour live in test_history_recovery.cpp.
class String : public std::string {
public:
    using std::string::string;
    String(const std::string& value) : std::string(value) {}
    bool isEmpty() const { return empty(); }
    int indexOf(const char *value) const {
        auto at = find(value);
        return at == npos ? -1 : static_cast<int>(at);
    }
    void trim() {
        auto first = find_first_not_of(" \r\n\t");
        if (first == npos) {
            clear();
            return;
        }
        *this = substr(first, find_last_not_of(" \r\n\t") - first + 1);
    }
};
