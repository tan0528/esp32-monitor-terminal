#pragma once

#include <cstddef>
#include <cstdint>

// This module has no Arduino dependencies and is shared by firmware and host tests.
constexpr std::size_t TEMPERATURE_WINDOW = 5;
constexpr std::size_t VIBRATION_SAMPLES = 256;
constexpr float VIBRATION_SAMPLE_RATE_HZ = 200.0f;

constexpr float TEMPERATURE_WARNING = 35.0f;
constexpr float TEMPERATURE_FAULT = 45.0f;
constexpr float TEMPERATURE_CLEAR = 33.0f;
constexpr float HUMIDITY_WARNING = 80.0f;
constexpr float HUMIDITY_CLEAR = 75.0f;
constexpr float VIBRATION_WARNING = 1.2f;
constexpr float VIBRATION_FAULT = 1.8f;
constexpr float VIBRATION_CLEAR = 1.0f;

constexpr uint16_t ALARM_TEMPERATURE = 1u << 0;
constexpr uint16_t ALARM_VIBRATION = 1u << 1;
// Any source invalid. ALARM_VIBRATION_INVALID narrows it: bit2 without bit5 is a
// DHT fault, bit2 with bit5 is a vibration-source fault.
constexpr uint16_t ALARM_SENSOR_INVALID = 1u << 2;
constexpr uint16_t ALARM_NETWORK_OFFLINE = 1u << 3;
constexpr uint16_t ALARM_HUMIDITY = 1u << 4;
constexpr uint16_t ALARM_VIBRATION_INVALID = 1u << 5;
// A frame was discarded by the sampling or feature queue during this interval.
constexpr uint16_t ALARM_DATA_LOSS = 1u << 6;

enum class DeviceState : uint8_t { Init, Normal, Warning, Fault, Recovery };

struct FeatureValues
{
    float temperature;
    float humidity;
    float vibrationRms;
    float vibrationPeakToPeak;
    bool valid;
    // Per-source validity. `valid` is their conjunction; the split lets the
    // telemetry say which source failed instead of only that one did.
    bool dhtValid;
    bool vibrationValid;
};

struct VibrationFeatures
{
    float rms = 0.0f;
    float peakToPeak = 0.0f;
    float dominantFrequencyHz = 0.0f;
    bool valid = false;
};

class TemperatureAverage
{
public:
    // Nonfinite values leave the previous window unchanged.
    float add(float temperature);
    float value() const;

private:
    float samples_[TEMPERATURE_WINDOW]{};
    std::size_t next_ = 0;
    std::size_t count_ = 0;
};

// Expects exactly 256 finite samples and a positive finite sample rate.
// RMS/P2P use the original samples. Only the FFT uses a Hann window.
// Frequency is the largest positive-frequency bin, with Fs / 256 resolution.
// A constant window is valid and has zero AC RMS and dominant frequency.
bool analyzeVibration(const float *samples, std::size_t count, float sampleRateHz,
                      VibrationFeatures &result);

// DHT validity controls the temperature window independently of vibration validity.
// The returned valid flag requires both sensor sources to be valid.
FeatureValues calculateFeatures(float temperature, float humidity, bool dhtValid,
                               const VibrationFeatures &vibration, TemperatureAverage &average);

// Adds the conditions the state machine cannot observe: link reachability, which
// source failed, and whether the pipeline dropped frames. `dataLoss` is sticky for
// the caller's reporting interval, not for the whole boot.
uint16_t composeAlarms(uint16_t stateAlarms, bool dhtValid, bool vibrationValid,
                       bool networkOnline, bool dataLoss);

class StateMachine
{
public:
    explicit StateMachine(uint32_t nowMs = 0) : enteredAtMs(nowMs) {}

    // Returns true only when the state changes. The caller owns logging and time.
    bool update(const FeatureValues &frame, uint32_t nowMs);

    DeviceState state = DeviceState::Init;
    uint16_t alarms = 0;
    uint32_t enteredAtMs;

private:
    void setState(DeviceState next, uint32_t nowMs);

    uint8_t startupReadyCount_ = 0;
    uint8_t invalidCount_ = 0;
    uint8_t warningCount_ = 0;
    uint8_t faultCount_ = 0;
    uint8_t safeCount_ = 0;
};
