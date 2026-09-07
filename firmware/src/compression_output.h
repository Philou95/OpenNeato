#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

// Fixed memory, batched writes, then read-back verification after close.
// The caller must retain the raw source until verification and rename succeed.
struct CompressionOutput {
    uint8_t buffer[2048];
    size_t used = 0;
    size_t size = 0;
    uint32_t hash = 2166136261u;
    size_t verifiedSize = 0;
    uint32_t verifiedHash = 2166136261u;
    size_t lastExpected = 0;
    size_t lastWritten = 0;

    void reset() {
        used = size = verifiedSize = 0;
        hash = verifiedHash = 2166136261u;
        lastExpected = lastWritten = 0;
    }

    static uint32_t updateHash(uint32_t value, const uint8_t *data, size_t count) {
        for (size_t i = 0; i < count; ++i)
            value = (value ^ data[i]) * 16777619u;
        return value;
    }

    template<typename File>
    bool flush(File& file) {
        if (!used)
            return true;
        lastExpected = used;
        size_t written = file.write(buffer, used);
        lastWritten = written;
        size += written;
        if (written != used)
            return false;
        hash = updateHash(hash, buffer, used);
        used = 0;
        return true;
    }

    template<typename File>
    bool append(File& file, const uint8_t *data, size_t count) {
        while (count) {
            size_t n = sizeof(buffer) - used;
            if (n > count)
                n = count;
            memcpy(buffer + used, data, n);
            used += n;
            data += n;
            count -= n;
            if (used == sizeof(buffer) && !flush(file))
                return false;
        }
        return true;
    }

    enum VerifyResult { MORE, VALID, INVALID };

    template<typename File>
    VerifyResult verifyStep(File& file) {
        if (used || file.size() != size)
            return INVALID;
        if (verifiedSize == size)
            return verifiedHash == hash ? VALID : INVALID;
        size_t count = size - verifiedSize;
        if (count > sizeof(buffer))
            count = sizeof(buffer);
        size_t read = file.read(buffer, count);
        if (read != count)
            return INVALID;
        verifiedHash = updateHash(verifiedHash, buffer, read);
        verifiedSize += read;
        return MORE;
    }
};
