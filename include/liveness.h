#pragma once

#include <cstdint>
#include <cstddef>

constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 10000;
constexpr uint32_t TASK_WATCHDOG_TIMEOUT_MS = 15000;

inline bool heartbeatExpired(uint32_t now, uint32_t last)
{
    return static_cast<int32_t>(now - last) > static_cast<int32_t>(HEARTBEAT_TIMEOUT_MS);
}

struct HeartbeatChanges
{
    uint32_t activeMask;
    uint32_t newlyStaleMask;
    uint32_t recoveredMask;
};

// Diagnostic state only. Fresh heartbeats clear an alert on the next observation.
// This monitor never decides whether another task may feed its own watchdog.
class HeartbeatMonitor
{
public:
    HeartbeatChanges update(uint32_t now, const uint32_t *last, std::size_t count)
    {
        uint32_t active = 0;
        for (std::size_t index = 0; index < count && index < 32; ++index)
        {
            if (heartbeatExpired(now, last[index]))
                active |= uint32_t{1} << index;
        }
        const HeartbeatChanges changes{active, active & ~previousMask_, previousMask_ & ~active};
        previousMask_ = active;
        return changes;
    }

private:
    uint32_t previousMask_ = 0;
};
