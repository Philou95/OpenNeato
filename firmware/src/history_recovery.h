#ifndef HISTORY_RECOVERY_H
#define HISTORY_RECOVERY_H

#include <Arduino.h>
#include <vector>

// Keep the originals until a verified replacement is installed. The backup
// covers the gap between two renames; source markers cover a restart between
// installing the replacement and removing the originals. These markers are
// ordinary JSONL metadata, ignored by pose/replay readers.
namespace HistoryRecovery {

    template<typename Storage>
    bool restore(Storage& fs, const String& target) {
        String backup = target + ".rcb";
        if (!fs.exists(backup))
            return true;
        if (!fs.exists(target))
            return fs.rename(backup, target);
        return fs.remove(backup);
    }

    inline String sourceMarker(const String& source) {
        return "{\"type\":\"recovery_source\",\"path\":\"" + source + "\"}";
    }

    inline uint32_t hashBytes(uint32_t hash, const uint8_t *bytes, size_t size) {
        for (size_t i = 0; i < size; ++i)
            hash = (hash ^ bytes[i]) * 16777619u;
        return hash;
    }

    template<typename File>
    struct Writer {
        File& file;
        uint8_t bytes[512];
        size_t used = 0;
        size_t size = 0;
        uint32_t hash = 2166136261u;

        explicit Writer(File& file) : file(file) {}

        bool flush() {
            if (!used)
                return true;
            if (file.write(bytes, used) != used)
                return false;
            size += used;
            hash = hashBytes(hash, bytes, used);
            used = 0;
            return true;
        }

        bool line(const String& value) {
            // Batch flash writes rather than issuing a SPIFFS write per pose.
            for (size_t i = 0; i <= value.length(); ++i) {
                bytes[used++] = i < value.length() ? static_cast<uint8_t>(value[i]) : '\n';
                if (used == sizeof(bytes) && !flush())
                    return false;
            }
            return true;
        }
    };

    template<typename File>
    bool readLine(File& file, String& line) {
        size_t before = file.position();
        const String raw = file.readStringUntil('\n');
        line = raw;
        line.trim();
        return file.position() > before;
    }

    template<typename Storage, typename Output>
    bool copy(Storage& fs, Output& output, const String& source, bool skipHeader) {
        auto input = fs.open(source, "r");
        if (!input)
            return false;
        size_t expected = input.size();
        while (input.position() < expected) {
            String line;
            if (!readLine(input, line))
                return false;
            if (line.isEmpty() || (skipHeader && line.indexOf("\"type\":\"session\"") >= 0))
                continue;
            if (!output.line(line))
                return false;
        }
        return input.position() == expected;
    }

    template<typename Storage>
    bool verify(Storage& fs, const String& path, size_t size, uint32_t expectedHash) {
        auto file = fs.open(path, "r");
        if (!file || file.size() != size)
            return false;
        uint32_t hash = 2166136261u;
        uint8_t bytes[256];
        size_t read = 0;
        while (read < size) {
            size_t n = file.read(bytes, sizeof(bytes));
            if (!n)
                return false;
            hash = hashBytes(hash, bytes, n);
            read += n;
        }
        return read == size && hash == expectedHash;
    }

    template<typename Storage>
    bool merge(Storage& fs, const std::vector<String>& sources) {
        if (sources.empty())
            return false;
        const String& target = sources.front();
        if (!restore(fs, target))
            return false;
        if (sources.size() == 1)
            return true;

        // Find sources already included by a transaction interrupted during
        // cleanup. Never append those a second time, even if removal fails.
        std::vector<bool> included(sources.size(), false);
        {
            auto file = fs.open(target, "r");
            if (!file)
                return false;
            size_t expected = file.size();
            while (file.position() < expected) {
                String line;
                if (!readLine(file, line))
                    return false;
                for (size_t i = 1; i < sources.size(); ++i) {
                    if (line == sourceMarker(sources[i]))
                        included[i] = true;
                }
            }
        }

        String temporary = target + ".rct";
        auto output = fs.open(temporary, "w");
        if (!output)
            return false;
        Writer<decltype(output)> writer(output);
        bool ok = copy(fs, writer, target, false);
        for (size_t i = 1; ok && i < sources.size(); ++i) {
            if (!included[i]) {
                ok = copy(fs, writer, sources[i], true) && writer.line(sourceMarker(sources[i]));
            }
        }
        ok = ok && writer.flush();
        output.flush();
        output.close();
        if (!ok || !verify(fs, temporary, writer.size, writer.hash)) {
            fs.remove(temporary);
            return false;
        }

        String backup = target + ".rcb";
        if (!fs.rename(target, backup))
            return false;
        if (!fs.rename(temporary, target)) {
            fs.rename(backup, target);
            return false;
        }
        // The verified destination now contains every source and its marker.
        // Interrupted/failed cleanup only leaves redundant copies.
        fs.remove(backup);
        for (size_t i = 1; i < sources.size(); ++i)
            fs.remove(sources[i]);
        return true;
    }

} // namespace HistoryRecovery

#endif // HISTORY_RECOVERY_H
