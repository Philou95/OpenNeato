#ifndef HISTORY_FRAME_H
#define HISTORY_FRAME_H

#include <cmath>
#include <cstdlib>
#include <cstring>

// A recovered run may contain several orphan measurements. Only the first
// validated initial value is meaningful; never substitute a later drift angle.
inline bool restoreInitialFrame(const char *status, const char *angle, const char *time, bool alreadyKnown,
                                float& degrees, float& timestamp) {
    if (alreadyKnown || !status || strcmp(status, "initial") != 0 || !angle || !time)
        return false;
    char *angleEnd = nullptr;
    char *timeEnd = nullptr;
    float value = strtof(angle, &angleEnd);
    float at = strtof(time, &timeEnd);
    if (angleEnd == angle || *angleEnd || timeEnd == time || *timeEnd || !std::isfinite(value) ||
        std::fabs(value) > 180.0f || !std::isfinite(at) || at < 0.0f)
        return false;
    degrees = value;
    timestamp = at;
    return true;
}

#endif // HISTORY_FRAME_H
