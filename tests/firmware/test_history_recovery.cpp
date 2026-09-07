#include "history_recovery.h"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>

struct PowerCut {};
struct FakeFs;

struct FakeFile {
    FakeFs *fs = nullptr;
    String path;
    size_t offset = 0;
    bool open = false;
    explicit operator bool() const { return open; }
    size_t position() const { return offset; }
    size_t size() const;
    String readStringUntil(char delimiter);
    size_t read(uint8_t *data, size_t count);
    size_t write(const uint8_t *data, size_t count);
    void flush();
    void close() { open = false; }
};

struct FakeFs {
    std::map<String, String> files;
    unsigned operations = 0, writes = 0, renames = 0;
    unsigned crashAt = 0, shortWriteAt = 0, failRenameAt = 0;
    bool corruptOnFlush = false;
    String failReadPath;

    void mutated() {
        if (++operations == crashAt)
            throw PowerCut{};
    }
    bool exists(const String& name) const { return files.count(name) != 0; }
    FakeFile open(const String& name, const char *mode) {
        if (*mode == 'w') {
            files[name] = "";
            mutated();
        }
        return FakeFile{this, name, 0, exists(name)};
    }
    bool remove(const String& name) {
        bool removed = files.erase(name) != 0;
        if (removed)
            mutated();
        return removed;
    }
    bool rename(const String& from, const String& to) {
        if (++renames == failRenameAt || !exists(from) || exists(to))
            return false;
        files[to] = files.at(from);
        files.erase(from);
        mutated();
        return true;
    }
};

size_t FakeFile::size() const { return fs->files.at(path).size(); }

String FakeFile::readStringUntil(char delimiter) {
    if (path == fs->failReadPath)
        return "";
    const auto& bytes = fs->files.at(path);
    size_t end = bytes.find(delimiter, offset);
    if (end == String::npos)
        end = bytes.size();
    String value = bytes.substr(offset, end - offset);
    offset = end < bytes.size() ? end + 1 : end;
    return value;
}

size_t FakeFile::read(uint8_t *data, size_t count) {
    if (path == fs->failReadPath)
        return 0;
    const auto& bytes = fs->files.at(path);
    size_t n = std::min(count, bytes.size() - offset);
    std::memcpy(data, bytes.data() + offset, n);
    offset += n;
    return n;
}

size_t FakeFile::write(const uint8_t *data, size_t count) {
    size_t n = ++fs->writes == fs->shortWriteAt ? count - 1 : count;
    fs->files[path].append(reinterpret_cast<const char *>(data), n);
    offset += n;
    fs->mutated();
    return n;
}

void FakeFile::flush() {
    if (fs->corruptOnFlush && !fs->files[path].empty())
        fs->files[path][0] ^= 1;
    fs->mutated();
}

const String A = "/history/1788674635.jsonl";
const String B = "/history/1788675000.jsonl";
const String HEADER = "{\"type\":\"session\",\"mode\":\"house\"}\n";
const String POSE_A = "{\"x\":1,\"y\":1,\"t\":0,\"ts\":2}\n";
const String POSE_B = "{\"x\":2,\"y\":2,\"t\":0,\"ts\":4}\n";

String originalA() {
    String text = HEADER + POSE_A;
    // Exercise both full-buffer writes and the final partial buffer.
    for (unsigned i = 0; i < 30; ++i)
        text += "{\"type\":\"metadata\",\"value\":\"additional recording information\"}\n";
    return text;
}

FakeFs initial() {
    FakeFs fs;
    fs.files[A] = originalA();
    fs.files[B] = HEADER + POSE_B;
    return fs;
}

void exactlyOnce(const String& text, const String& part) {
    size_t at = text.find(part);
    assert(at != String::npos);
    assert(text.find(part, at + 1) == String::npos);
}

void verifyResult(const FakeFs& fs) {
    const auto& text = fs.files.at(A);
    exactlyOnce(text, HEADER);
    exactlyOnce(text, POSE_A);
    exactlyOnce(text, POSE_B);
    assert(!fs.exists(B));
}

void retry(FakeFs& fs) {
    fs.crashAt = fs.shortWriteAt = fs.failRenameAt = 0;
    fs.corruptOnFlush = false;
    fs.failReadPath = "";
    assert(HistoryRecovery::restore(fs, A));
    std::vector<String> sources{A};
    if (fs.exists(B))
        sources.push_back(B);
    assert(HistoryRecovery::merge(fs, sources));
    verifyResult(fs);
}

int main() {
    auto complete = initial();
    assert(HistoryRecovery::merge(complete, {A, B}));
    verifyResult(complete);
    // A power interruption after EVERY mutating operation, including both
    // renames and every source deletion, must permit an exact-once retry.
    for (unsigned cut = 1; cut <= complete.operations; ++cut) {
        auto fs = initial();
        fs.crashAt = cut;
        try {
            HistoryRecovery::merge(fs, {A, B});
        } catch (const PowerCut&) {
        }
        retry(fs);
    }
    for (unsigned n = 1; n <= complete.writes; ++n) {
        auto fs = initial();
        fs.shortWriteAt = n;
        assert(!HistoryRecovery::merge(fs, {A, B}));
        assert(fs.files.at(A) == originalA());
        assert(fs.files.at(B) == HEADER + POSE_B);
        retry(fs);
    }
    for (unsigned n = 1; n <= 2; ++n) {
        auto fs = initial();
        fs.failRenameAt = n;
        assert(!HistoryRecovery::merge(fs, {A, B}));
        retry(fs);
    }
    auto corrupt = initial();
    corrupt.corruptOnFlush = true;
    assert(!HistoryRecovery::merge(corrupt, {A, B}));
    retry(corrupt);
    auto unreadable = initial();
    unreadable.failReadPath = B;
    assert(!HistoryRecovery::merge(unreadable, {A, B}));
    retry(unreadable);
    std::cout << "Recovery OK: " << complete.operations << " power cuts, "
              << complete.writes << " short writes, 2 failed renames, corruption and read failure\n";
}
