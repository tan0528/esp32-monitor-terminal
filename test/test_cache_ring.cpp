#include "cache_ring.h"

#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace
{
std::size_t checks = 0;

void require(bool condition, const char *expression, int line)
{
    ++checks;
    if (!condition)
        throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
}

#define CHECK(condition) require((condition), #condition, __LINE__)

void testGeometry()
{
    CHECK(cache_ring::SLOTS == 65);
    CHECK(cache_ring::CAPACITY == 64);

    // A full ring must leave the append position one slot behind the head, never
    // on it, so an append never overwrites the record a Peek is reading.
    for (uint32_t head = 0; head < cache_ring::SLOTS; ++head)
    {
        const uint32_t appendAt = cache_ring::tail(head, cache_ring::CAPACITY);
        CHECK(appendAt != head);
        CHECK(appendAt == (head + cache_ring::CAPACITY) % cache_ring::SLOTS);
    }
}

void testAppendPop()
{
    uint32_t head = 0;
    uint32_t count = 0;
    bool overwritten = false;

    for (uint32_t index = 0; index < cache_ring::CAPACITY; ++index)
    {
        cache_ring::append(head, count, overwritten);
        CHECK(!overwritten);
        CHECK(head == 0);
        CHECK(count == index + 1);
    }
    CHECK(count == cache_ring::CAPACITY);
    CHECK(cache_ring::tail(head, count) == cache_ring::SLOTS - 1);

    // The next append evicts the oldest instead of growing the count.
    cache_ring::append(head, count, overwritten);
    CHECK(overwritten);
    CHECK(head == 1);
    CHECK(count == cache_ring::CAPACITY);

    CHECK(cache_ring::pop(head, count));
    CHECK(head == 2);
    CHECK(count == cache_ring::CAPACITY - 1);

    // An empty ring must not move the head or wrap the count below zero.
    uint32_t emptyHead = 7;
    uint32_t emptyCount = 0;
    CHECK(!cache_ring::pop(emptyHead, emptyCount));
    CHECK(emptyHead == 7);
    CHECK(emptyCount == 0);
}

void testWrapAroundKeepsEverySlotReachable()
{
    uint32_t head = 0;
    uint32_t count = 0;
    bool overwritten = false;
    bool seen[cache_ring::SLOTS]{};

    // Drive the ring well past one full lap of evictions and record which slots the
    // append position lands on. All slots except the head must be reachable.
    for (uint32_t index = 0; index < cache_ring::SLOTS * 3; ++index)
    {
        const uint32_t appendAt = cache_ring::tail(head, count);
        seen[appendAt] = true;
        CHECK(appendAt < cache_ring::SLOTS);
        CHECK(cache_ring::validIndex(head, count));
        cache_ring::append(head, count, overwritten);
        CHECK(cache_ring::validIndex(head, count));
        CHECK(count <= cache_ring::CAPACITY);
        CHECK(head < cache_ring::SLOTS);
    }

    std::size_t reached = 0;
    for (bool slot : seen) reached += slot ? 1 : 0;
    // The head slot is never an append target, so at most one slot stays untouched.
    CHECK(reached >= cache_ring::SLOTS - 1);
}

void testDrainRestoresEmptyState()
{
    uint32_t head = 0;
    uint32_t count = 0;
    bool overwritten = false;
    for (uint32_t index = 0; index < cache_ring::CAPACITY + 7; ++index)
        cache_ring::append(head, count, overwritten);

    uint32_t popped = 0;
    while (cache_ring::pop(head, count))
        ++popped;
    CHECK(popped == cache_ring::CAPACITY);
    CHECK(count == 0);

    // An empty ring accepts a full capacity again without reporting eviction.
    for (uint32_t index = 0; index < cache_ring::CAPACITY; ++index)
    {
        cache_ring::append(head, count, overwritten);
        CHECK(!overwritten);
    }
    CHECK(count == cache_ring::CAPACITY);
}

void testIndexValidation()
{
    CHECK(cache_ring::validIndex(0, 0));
    CHECK(cache_ring::validIndex(cache_ring::SLOTS - 1, cache_ring::CAPACITY));
    CHECK(!cache_ring::validIndex(cache_ring::SLOTS, 0));
    CHECK(!cache_ring::validIndex(0, cache_ring::CAPACITY + 1));
}
}

int main()
{
    try
    {
        testGeometry();
        testAppendPop();
        testWrapAroundKeepsEverySlotReachable();
        testDrainRestoresEmptyState();
        testIndexValidation();
        std::printf("[PASS] %zu checks: ring geometry, eviction, drain and wrap invariants\n", checks);
    }
    catch (const std::exception &error)
    {
        std::printf("[FAIL] %s\n", error.what());
        return 1;
    }
    return 0;
}
