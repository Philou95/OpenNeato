#include <cassert>
#include <iostream>
#include "history_frame.h"

int main() {
    float degrees = 0, timestamp = 0;
    assert(restoreInitialFrame("initial", "-3.39", "30.125", false, degrees, timestamp));
    assert(std::fabs(degrees + 3.39f) < 0.001f && timestamp == 30.125f);
    assert(!restoreInitialFrame("initial", "-52.68", "1960", true, degrees, timestamp));
    assert(std::fabs(degrees + 3.39f) < 0.001f && timestamp == 30.125f);
    assert(!restoreInitialFrame("late", "-52.68", "1960", false, degrees, timestamp));
    assert(!restoreInitialFrame(nullptr, nullptr, nullptr, false, degrees, timestamp));
    for (const char *bad: {"", "garbage", "nan", "inf", "181", "2truncated"})
        assert(!restoreInitialFrame("initial", bad, "30", false, degrees, timestamp));
    for (const char *bad: {"", "nan", "-1", "12broken"})
        assert(!restoreInitialFrame("initial", "2", bad, false, degrees, timestamp));
    std::cout << "Initial frame restored; later, missing and malformed measurements refused\n";
}
