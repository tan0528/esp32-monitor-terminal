#pragma once

#include <cstddef>
#include <cstdint>

#define TWO_PI 6.283185307179586476925286766559

extern uint32_t sourceTestMillis;
void advanceVibrationTestClock(uint32_t milliseconds);

inline uint32_t millis() { return sourceTestMillis; }
inline void delay(uint32_t milliseconds) { advanceVibrationTestClock(milliseconds); }

struct SourceTestSerial
{
    template <typename... Args>
    void printf(const char *, Args...) {}
};
extern SourceTestSerial Serial;

// The firmware routes diagnostics through a serialized sink; the host build only
// needs the symbol so the driver can be linked without the rest of the firmware.
inline void logPrintf(const char *, ...) {}
