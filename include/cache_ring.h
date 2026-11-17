#pragma once

#include <cstdint>

// Ring index arithmetic for the offline cache, kept free of Arduino and file I/O
// so the wrap, eviction and corruption-skip cases can be tested on the host.
//
// One slot beyond the capacity is reserved. Without it a full ring would place the
// append position on the oldest committed record, and appending would destroy a
// record that a concurrent Peek is already reading.
namespace cache_ring
{
constexpr uint32_t SLOTS = 65;
constexpr uint32_t CAPACITY = SLOTS - 1;

inline uint32_t tail(uint32_t head, uint32_t count)
{
    return (head + count) % SLOTS;
}

inline bool validIndex(uint32_t head, uint32_t count)
{
    return head < SLOTS && count <= CAPACITY;
}

// Advances the index for one appended record. `overwritten` is set when the ring
// was full and the oldest record was evicted to make room.
inline void append(uint32_t &head, uint32_t &count, bool &overwritten)
{
    overwritten = count == CAPACITY;
    if (overwritten)
        head = (head + 1) % SLOTS;
    else
        ++count;
}

// Removes the oldest record. Returns false when the cache is empty.
inline bool pop(uint32_t &head, uint32_t &count)
{
    if (count == 0)
        return false;
    head = (head + 1) % SLOTS;
    --count;
    return true;
}
}
