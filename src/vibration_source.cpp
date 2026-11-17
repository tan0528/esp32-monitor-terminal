#include "vibration_source.h"
#include "logging.h"

#include <Arduino.h>
#include <cmath>

#ifndef TERMINAL_USE_MPU6050
#define TERMINAL_USE_MPU6050 0
#endif

#ifndef TERMINAL_SYNTHETIC_NOISE_G
#define TERMINAL_SYNTHETIC_NOISE_G 0.02f
#endif

#if TERMINAL_USE_MPU6050
#include <Wire.h>

namespace
{
// InvenSense RM-MPU-6000A-00 revision 4.2, registers 25, 26, 28, 35, 106–117.
constexpr uint8_t MPU_ADDRESS = 0x68; // AD0 is tied to ground.
constexpr uint8_t SAMPLE_RATE_DIVIDER = 0x19;
constexpr uint8_t CONFIGURATION = 0x1a;
constexpr uint8_t ACCELEROMETER_CONFIGURATION = 0x1c;
constexpr uint8_t FIFO_ENABLE = 0x23;
constexpr uint8_t INTERRUPT_ENABLE = 0x38;
constexpr uint8_t INTERRUPT_STATUS = 0x3a;
constexpr uint8_t USER_CONTROL = 0x6a;
constexpr uint8_t POWER_MANAGEMENT = 0x6b;
constexpr uint8_t FIFO_COUNT = 0x72;
constexpr uint8_t FIFO_DATA = 0x74;
constexpr uint8_t WHO_AM_I = 0x75;
constexpr uint8_t FIFO_OVERFLOW = 0x10;
constexpr uint8_t ACCELEROMETER_FIFO = 0x08;
constexpr uint8_t FIFO_RUNNING = 0x40;
constexpr uint8_t FIFO_RESET = 0x04;
constexpr size_t FIFO_PACKET_BYTES = 6;
constexpr size_t FIFO_CAPACITY_BYTES = 1024;
constexpr size_t MAX_BATCH_SAMPLES = 16;
constexpr float ACCELEROMETER_LSB_PER_G = 8192.0f;
// A 256-sample window at 200 Hz needs 1.28 s. The budget leaves room for I2C
// stalls and a retry inside the 2 s frame period instead of aborting the window.
constexpr uint32_t WINDOW_TIMEOUT_MS = 1600;
constexpr uint32_t FIFO_STALL_TIMEOUT_MS = 200;
// Faster than one ODR sample, so a poll never waits for more than one sample and
// the window does not stretch past the frame period.
constexpr uint32_t POLL_INTERVAL_MS = 2;
bool busReady = false;
bool sourceReady = false;

bool writeRegister(uint8_t address, uint8_t value)
{
    Wire.beginTransmission(MPU_ADDRESS);
    if (Wire.write(address) != 1 || Wire.write(value) != 1)
        return false;
    return Wire.endTransmission(true) == 0;
}

bool readRegisters(uint8_t address, uint8_t *values, size_t count)
{
    Wire.beginTransmission(MPU_ADDRESS);
    if (Wire.write(address) != 1 || Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom(MPU_ADDRESS, count, true) != count)
        return false;
    for (size_t i = 0; i < count; ++i)
    {
        const int value = Wire.read();
        if (value < 0) return false;
        values[i] = static_cast<uint8_t>(value);
    }
    return true;
}

bool registerEquals(uint8_t address, uint8_t expected)
{
    uint8_t value = 0;
    return readRegisters(address, &value, 1) && value == expected;
}

bool stopFifo()
{
    const bool stoppedWrites = writeRegister(FIFO_ENABLE, 0);
    const bool stoppedFifo = writeRegister(USER_CONTROL, 0);
    return stoppedWrites && stoppedFifo;
}

bool failWindow(const char *reason)
{
    sourceReady = false;
    stopFifo();
    logPrintf("[VIBRATION] MPU6050 acquisition invalid: %s\n", reason);
    return false;
}

bool startFifo()
{
    if (!stopFifo() || !writeRegister(USER_CONTROL, FIFO_RESET))
        return false;
    // FIFO_RESET clears itself. Never accept bytes left from a prior window.
    delay(2);
    uint8_t count[2]{};
    uint8_t status = 0;
    return registerEquals(USER_CONTROL, 0) &&
           readRegisters(FIFO_COUNT, count, sizeof(count)) && count[0] == 0 && count[1] == 0 &&
           readRegisters(INTERRUPT_STATUS, &status, 1) &&
           writeRegister(USER_CONTROL, FIFO_RUNNING) &&
           writeRegister(FIFO_ENABLE, ACCELEROMETER_FIFO);
}
}
#endif

const char *vibrationSourceName()
{
#if TERMINAL_USE_MPU6050
    return "MPU6050 X-axis FIFO";
#else
    return "synthetic 25 Hz";
#endif
}

bool beginVibrationSource()
{
#if TERMINAL_USE_MPU6050
    static_assert(VIBRATION_SAMPLE_RATE_HZ == 200.0f, "MPU6050 divider must match the analysis sample rate");
    sourceReady = false;
    if (!busReady)
    {
        busReady = Wire.begin(21, 22, 400000);
        Wire.setTimeOut(40);
    }
    if (!busReady)
        return failWindow("I2C bus initialization failed");
    if (!registerEquals(WHO_AM_I, 0x68))
        return failWindow("I2C probe or WHO_AM_I failed");
    if (!writeRegister(POWER_MANAGEMENT, 0x80))
        return failWindow("device reset failed");
    delay(100);
    // Keep the gyro oscillator available for the PLL and 1 kHz sampling clock.
    if (!writeRegister(POWER_MANAGEMENT, 0x01) ||
        !writeRegister(CONFIGURATION, 0x03) ||
        !writeRegister(SAMPLE_RATE_DIVIDER, 4) ||
        !writeRegister(ACCELEROMETER_CONFIGURATION, 0x08) ||
        !writeRegister(INTERRUPT_ENABLE, FIFO_OVERFLOW) || !stopFifo())
        return failWindow("configuration write failed");
    delay(50);
    if (!registerEquals(WHO_AM_I, 0x68) ||
        !registerEquals(POWER_MANAGEMENT, 0x01) ||
        !registerEquals(CONFIGURATION, 0x03) ||
        !registerEquals(SAMPLE_RATE_DIVIDER, 4) ||
        !registerEquals(ACCELEROMETER_CONFIGURATION, 0x08) ||
        !registerEquals(INTERRUPT_ENABLE, FIFO_OVERFLOW))
        return failWindow("configuration readback failed");
    sourceReady = true;
#endif
    return true;
}

bool readVibrationWindow(float *samples, size_t count, float amplitude, bool (*keepSampling)())
{
    if (samples == nullptr || count != VIBRATION_SAMPLES)
        return false;
    if (keepSampling && !keepSampling())
        return false;

#if TERMINAL_USE_MPU6050
    (void)amplitude; // Simulation controls never replace hardware measurements.
    if (!sourceReady && !beginVibrationSource())
        return false;
    if (!startFifo())
        return failWindow("FIFO reset or enable failed");
    size_t collected = 0;
    const uint32_t startedAt = millis();
    uint32_t lastSampleAt = startedAt;
    uint8_t bytes[MAX_BATCH_SAMPLES * FIFO_PACKET_BYTES]{};
    while (collected < count)
    {
        if (keepSampling && !keepSampling())
        {
            if (!stopFifo()) return failWindow("FIFO stop failed during cancellation");
            return false;
        }
        const uint32_t now = millis();
        if (now - startedAt > WINDOW_TIMEOUT_MS || now - lastSampleAt > FIFO_STALL_TIMEOUT_MS)
            return failWindow("sample timeout or unsupported FIFO implementation");
        uint8_t status = 0;
        uint8_t fifoCount[2]{};
        if (!readRegisters(INTERRUPT_STATUS, &status, 1) ||
            !readRegisters(FIFO_COUNT, fifoCount, sizeof(fifoCount)))
            return failWindow("I2C status read failed");
        const size_t available = (static_cast<size_t>(fifoCount[0]) << 8) | fifoCount[1];
        if ((status & FIFO_OVERFLOW) || available >= FIFO_CAPACITY_BYTES)
            return failWindow("FIFO overflow");
        size_t batch = available / FIFO_PACKET_BYTES;
        if (batch > MAX_BATCH_SAMPLES) batch = MAX_BATCH_SAMPLES;
        if (batch > count - collected) batch = count - collected;
        if (batch == 0)
        {
            // The sensor's clock timestamps samples. This delay only yields while polling.
            delay(POLL_INTERVAL_MS);
            continue;
        }
        if (!readRegisters(FIFO_DATA, bytes, batch * FIFO_PACKET_BYTES))
            return failWindow("I2C FIFO read failed");
        for (size_t i = 0; i < batch; ++i)
        {
            const size_t offset = i * FIFO_PACKET_BYTES;
            const uint16_t encoded = (static_cast<uint16_t>(bytes[offset]) << 8) | bytes[offset + 1];
            const int32_t acceleration = encoded < 0x8000 ? encoded : static_cast<int32_t>(encoded) - 65536;
            if (acceleration >= 32760 || acceleration <= -32760)
                return failWindow("X-axis acceleration saturated the 4 g range");
            samples[collected++] = acceleration / ACCELEROMETER_LSB_PER_G;
        }
        lastSampleAt = millis();
    }
    // Catch an overflow that occurred between the final status and FIFO reads.
    uint8_t status = 0;
    if (!readRegisters(INTERRUPT_STATUS, &status, 1) || (status & FIFO_OVERFLOW))
        return failWindow("final FIFO status failed or overflowed");
    if (!stopFifo())
        return failWindow("FIFO stop failed");
#else
    if (!std::isfinite(amplitude) || amplitude < 0.0f)
        return false;
    constexpr float MODEL_TWO_PI = 6.283185307179586f;
    constexpr float SIGNAL_FREQUENCY_HZ = 25.0f;
    constexpr float noiseAmplitude = TERMINAL_SYNTHETIC_NOISE_G;
    static_assert(noiseAmplitude >= 0.0f, "Synthetic noise amplitude must be nonnegative");
    uint32_t noiseState = 0x6d2b79f5u;
    for (size_t i = 0; i < count; ++i)
    {
        // A fixed seed makes noise and FFT regression checks repeatable across boots.
        noiseState ^= noiseState << 13;
        noiseState ^= noiseState >> 17;
        noiseState ^= noiseState << 5;
        const float noise = (static_cast<float>(noiseState & 0xffffu) / 32767.5f - 1.0f) * noiseAmplitude;
        const float modelTime = static_cast<float>(i) / VIBRATION_SAMPLE_RATE_HZ;
        samples[i] = amplitude * sinf(MODEL_TWO_PI * SIGNAL_FREQUENCY_HZ * modelTime) + noise;
    }
#endif
    return true;
}
