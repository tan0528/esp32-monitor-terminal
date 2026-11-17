#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

// A clocked register/FIFO model for the real MPU backend, not a replacement driver.
class SourceTestWire
{
public:
    bool failBegin = false;
    bool missing = false;
    bool wrongIdentity = false;
    bool badConfiguration = false;
    bool shortFifoRead = false;
    bool overflow = false;
    bool stalled = false;
    bool fifoResetStuck = false;
    size_t maxRequestBytes = 0;
    size_t packetsGenerated = 0;
    size_t fifoBytesRead = 0;
    size_t maxFifoBytes = 0;
    std::array<uint8_t, 128> registers{};

    SourceTestWire() { reset(); }

    void reset()
    {
        failBegin = missing = wrongIdentity = badConfiguration = false;
        shortFifoRead = overflow = stalled = fifoResetStuck = false;
        maxRequestBytes = packetsGenerated = fifoBytesRead = maxFifoBytes = 0;
        registers.fill(0);
        registers[0x75] = 0x68;
        fifo.clear();
        received.clear();
        transmit.clear();
        elapsed = 0;
    }

    bool begin(int sda, int scl, uint32_t frequency)
    {
        return !failBegin && sda == 21 && scl == 22 && frequency == 400000;
    }
    void setTimeOut(uint16_t) {}
    void beginTransmission(uint8_t address)
    {
        deviceAddress = address;
        transmit.clear();
    }
    size_t write(uint8_t value)
    {
        transmit.push_back(value);
        return 1;
    }
    uint8_t endTransmission(bool)
    {
        if (missing || deviceAddress != 0x68 || transmit.empty()) return 2;
        pointer = transmit[0];
        if (transmit.size() == 2)
        {
            const uint8_t value = transmit[1];
            if (pointer == 0x6b && value == 0x80)
            {
                registers.fill(0);
                registers[0x75] = 0x68;
                fifo.clear();
            }
            else if (pointer == 0x6a && value == 0x04)
            {
                fifo.clear();
                elapsed = 0;
                registers[pointer] = fifoResetStuck ? value : 0;
            }
            else
            {
                registers[pointer] = value;
            }
        }
        return 0;
    }

    size_t requestFrom(uint8_t address, size_t count, bool)
    {
        maxRequestBytes = std::max(maxRequestBytes, count);
        received.clear();
        if (missing || address != 0x68) return 0;
        if (pointer == 0x74)
        {
            if (shortFifoRead) return 0;
            for (size_t i = 0; i < count && !fifo.empty(); ++i)
            {
                received.push_back(fifo.front());
                fifo.pop_front();
                ++fifoBytesRead;
            }
        }
        else if (pointer == 0x72 && count == 2)
        {
            received.push_back(static_cast<uint8_t>(fifo.size() >> 8));
            received.push_back(static_cast<uint8_t>(fifo.size()));
        }
        else
        {
            for (size_t i = 0; i < count; ++i)
            {
                const uint8_t addressRead = static_cast<uint8_t>(pointer + i);
                uint8_t value = registers[addressRead];
                if (addressRead == 0x75 && wrongIdentity) value = 0x70;
                if (addressRead == 0x1a && badConfiguration) value = 0;
                if (addressRead == 0x3a && overflow && packetsGenerated >= 8) value = 0x10;
                received.push_back(value);
            }
        }
        return received.size();
    }
    int read()
    {
        if (received.empty()) return -1;
        const uint8_t value = received.front();
        received.pop_front();
        return value;
    }

    void advance(uint32_t milliseconds)
    {
        if (stalled || registers[0x6a] != 0x40 || registers[0x23] != 0x08)
            return;
        elapsed += milliseconds;
        while (elapsed >= 5)
        {
            elapsed -= 5;
            const int16_t x = (packetsGenerated % 2 == 0) ? 8192 : -8192;
            fifo.push_back(static_cast<uint8_t>(static_cast<uint16_t>(x) >> 8));
            fifo.push_back(static_cast<uint8_t>(x));
            for (size_t i = 0; i < 4; ++i) fifo.push_back(0);
            ++packetsGenerated;
            maxFifoBytes = std::max(maxFifoBytes, fifo.size());
        }
    }

private:
    uint8_t deviceAddress = 0;
    uint8_t pointer = 0;
    uint32_t elapsed = 0;
    std::deque<uint8_t> fifo;
    std::deque<uint8_t> received;
    std::vector<uint8_t> transmit;
};

extern SourceTestWire Wire;
