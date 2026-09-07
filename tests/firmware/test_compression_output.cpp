#include "compression_output.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>
#include <vector>

struct File {
    std::vector<uint8_t> bytes;
    size_t offset = 0;
    size_t writeLimit = 999999;
    size_t readLimit = 999999;
    int writes = 0;
    size_t write(const uint8_t *data, size_t count) {
        ++writes;
        count = std::min(count, writeLimit);
        bytes.insert(bytes.end(), data, data + count);
        return count;
    }
    size_t read(uint8_t *data, size_t count) {
        count = std::min({count, bytes.size() - offset, readLimit});
        std::copy_n(bytes.data() + offset, count, data);
        offset += count;
        return count;
    }
    size_t size() const { return bytes.size(); }
};

static bool verify(CompressionOutput& output, File file) {
    file.offset = 0;
    for (int i = 0; i < 100; ++i) {
        auto result = output.verifyStep(file);
        if (result != CompressionOutput::MORE)
            return result == CompressionOutput::VALID;
    }
    assert(false);
    return false;
}

int main() {
    assert((std::string("/history/1788763156.jsonl") + CompressionOutput::TEMP_SUFFIX).size() <= 31);
    std::vector<uint8_t> input(24379);
    for (size_t i = 0; i < input.size(); ++i)
        input[i] = static_cast<uint8_t>(i * 53);
    CompressionOutput output;
    File file;
    // Encoder-sized chunks, including the 282-byte chunk from the failed cycle.
    for (size_t i = 0; i < input.size(); i += 282)
        assert(output.append(file, input.data() + i, std::min<size_t>(282, input.size() - i)));
    assert(output.flush(file));
    assert(file.writes == 12);
    assert(file.bytes == input);
    CompressionOutput saved = output;
    assert(verify(output, file));

    File corrupt = file;
    corrupt.bytes[4096] ^= 1;
    output = saved;
    assert(!verify(output, corrupt));
    corrupt = file;
    corrupt.bytes.pop_back();
    output = saved;
    assert(!verify(output, corrupt));
    corrupt = file;
    corrupt.bytes.push_back(0);
    output = saved;
    assert(!verify(output, corrupt));
    corrupt = file;
    corrupt.readLimit = 2;
    output = saved;
    assert(!verify(output, corrupt));

    for (size_t limit : {size_t(0), size_t(282), size_t(2047)}) {
        output.reset();
        File failing;
        failing.writeLimit = limit;
        assert(!output.append(failing, input.data(), 2048));
        assert(output.size == limit);
        assert(!verify(output, failing));
        // A retry starts a fresh stream; it does not append at an uncertain offset.
        output.reset();
        File retry;
        assert(output.append(retry, input.data(), input.size()));
        assert(output.flush(retry));
        assert(verify(output, retry));
        assert(retry.bytes == input);
    }
    std::cout << "Compression output: batching, short writes, corruption, truncation, retry OK\n";
}
