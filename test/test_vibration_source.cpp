#include "vibration_source.h"
#include <Arduino.h>

#include <cassert>
#include <cmath>
#include <cstdio>

#ifndef TERMINAL_USE_MPU6050
#define TERMINAL_USE_MPU6050 0
#endif

uint32_t sourceTestMillis = 0;
SourceTestSerial Serial;
static size_t callbacks = 0;
static bool cancelImmediately() { return false; }
#if TERMINAL_USE_MPU6050
static bool cancelAfterTenCalls() { return ++callbacks < 10; }
#endif
static bool keepSampling() { ++callbacks; return true; }

#if TERMINAL_USE_MPU6050
#include <Wire.h>
SourceTestWire Wire;
void advanceVibrationTestClock(uint32_t milliseconds)
{
    sourceTestMillis += milliseconds;
    Wire.advance(milliseconds);
}
#else
void advanceVibrationTestClock(uint32_t milliseconds) { sourceTestMillis += milliseconds; }
#endif

int main()
{
    float samples[VIBRATION_SAMPLES]{};
    assert(!readVibrationWindow(nullptr, VIBRATION_SAMPLES, 1.0f));
    assert(!readVibrationWindow(samples, VIBRATION_SAMPLES - 1, 1.0f));
    assert(!readVibrationWindow(samples, VIBRATION_SAMPLES, 1.0f, cancelImmediately));

#if TERMINAL_USE_MPU6050
    Wire.failBegin = true;
    assert(!beginVibrationSource());
    Wire.reset();
    Wire.missing = true;
    assert(!beginVibrationSource());
    assert(!readVibrationWindow(samples, VIBRATION_SAMPLES, 1.0f));
    Wire.reset();
    Wire.wrongIdentity = true;
    assert(!beginVibrationSource());
    Wire.reset();
    Wire.badConfiguration = true;
    assert(!beginVibrationSource());

    Wire.reset();
    assert(beginVibrationSource());
    const uint32_t start = millis();
    assert(readVibrationWindow(samples, VIBRATION_SAMPLES, 100.0f, keepSampling));
    assert(millis() - start >= 1280);
    assert(millis() - start < 1500);
    assert(Wire.packetsGenerated == VIBRATION_SAMPLES);
    assert(Wire.fifoBytesRead == VIBRATION_SAMPLES * 6);
    assert(Wire.maxRequestBytes <= 96);
    assert(Wire.maxFifoBytes < 1024);
    assert(Wire.registers[0x19] == 4 && Wire.registers[0x1a] == 3 && Wire.registers[0x1c] == 8);
    for (size_t i = 0; i < VIBRATION_SAMPLES; ++i)
        assert(samples[i] == (i % 2 == 0 ? 1.0f : -1.0f));
    assert(Wire.registers[0x6a] == 0 && Wire.registers[0x23] == 0);
    assert(callbacks > 100);

    Wire.reset();
    assert(beginVibrationSource());
    Wire.shortFifoRead = true;
    assert(!readVibrationWindow(samples, VIBRATION_SAMPLES, 1.0f));
    Wire.reset();
    assert(beginVibrationSource());
    Wire.overflow = true;
    assert(!readVibrationWindow(samples, VIBRATION_SAMPLES, 1.0f));
    assert(Wire.packetsGenerated >= 8);
    Wire.reset();
    assert(beginVibrationSource());
    Wire.stalled = true;
    const uint32_t timeoutStart = millis();
    assert(!readVibrationWindow(samples, VIBRATION_SAMPLES, 1.0f));
    // The stall budget is 200 ms; the poll interval only adds a few 2 ms steps.
    assert(millis() - timeoutStart > 200 && millis() - timeoutStart < 400);
    Wire.reset();
    assert(beginVibrationSource());
    Wire.fifoResetStuck = true;
    assert(!readVibrationWindow(samples, VIBRATION_SAMPLES, 1.0f));
    Wire.reset();
    assert(beginVibrationSource());
    callbacks = 0;
    assert(!readVibrationWindow(samples, VIBRATION_SAMPLES, 1.0f, cancelAfterTenCalls));
    assert(Wire.registers[0x6a] == 0 && Wire.registers[0x23] == 0);
    std::puts("MPU6050 backend passed FIFO order/scaling/rate, cancellation and bus/read/overflow/timeout failures");
#else
    assert(beginVibrationSource());
    assert(readVibrationWindow(samples, VIBRATION_SAMPLES, 2.0f, keepSampling));
    float repeated[VIBRATION_SAMPLES]{};
    assert(readVibrationWindow(repeated, VIBRATION_SAMPLES, 2.0f));
    double squares = 0;
    for (size_t i = 0; i < VIBRATION_SAMPLES; ++i)
    {
        assert(samples[i] == repeated[i]);
        const float expected = 2.0f * std::sin(6.283185307179586f * 25.0f * i / VIBRATION_SAMPLE_RATE_HZ);
        assert(std::fabs(samples[i] - expected) < 0.0201f);
        squares += samples[i] * samples[i];
    }
    assert(std::fabs(std::sqrt(squares / VIBRATION_SAMPLES) - std::sqrt(2.0)) < 0.01);
    assert(!readVibrationWindow(samples, VIBRATION_SAMPLES, -1.0f));
    assert(!readVibrationWindow(samples, VIBRATION_SAMPLES, NAN));
    std::puts("Synthetic backend passed 256 samples at model Fs=200 Hz, 25 Hz, deterministic noise and argument checks");
#endif
}
