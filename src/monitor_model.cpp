#include "monitor_model.h"

#include <cmath>
#include <utility>

namespace
{
constexpr float TWO_PI = 6.283185307179586f;

bool validDhtValues(float temperature, float humidity)
{
    return std::isfinite(temperature) && temperature >= -40.0f && temperature <= 80.0f &&
           std::isfinite(humidity) && humidity >= 0.0f && humidity <= 100.0f;
}

bool validVibrationValues(float rms, float peakToPeak)
{
    return std::isfinite(rms) && rms >= 0.0f && std::isfinite(peakToPeak) && peakToPeak >= 0.0f;
}

void countConsecutive(uint8_t &count, bool condition)
{
    if (!condition) count = 0;
    else if (count < 5) ++count;
}

void fft(float (&real)[VIBRATION_SAMPLES], float (&imaginary)[VIBRATION_SAMPLES])
{
    // Bit-reversed input ordering for the iterative radix-2 transform.
    std::size_t reversed = 0;
    for (std::size_t index = 1; index < VIBRATION_SAMPLES; ++index)
    {
        std::size_t bit = VIBRATION_SAMPLES / 2;
        while (reversed & bit)
        {
            reversed ^= bit;
            bit >>= 1;
        }
        reversed ^= bit;
        if (index < reversed)
        {
            std::swap(real[index], real[reversed]);
            std::swap(imaginary[index], imaginary[reversed]);
        }
    }

    for (std::size_t length = 2; length <= VIBRATION_SAMPLES; length *= 2)
    {
        const float angle = -TWO_PI / length;
        const float stepReal = std::cos(angle);
        const float stepImaginary = std::sin(angle);
        for (std::size_t start = 0; start < VIBRATION_SAMPLES; start += length)
        {
            float rotationReal = 1.0f;
            float rotationImaginary = 0.0f;
            for (std::size_t offset = 0; offset < length / 2; ++offset)
            {
                const std::size_t even = start + offset;
                const std::size_t odd = even + length / 2;
                const float oddReal = rotationReal * real[odd] - rotationImaginary * imaginary[odd];
                const float oddImaginary = rotationReal * imaginary[odd] + rotationImaginary * real[odd];
                real[odd] = real[even] - oddReal;
                imaginary[odd] = imaginary[even] - oddImaginary;
                real[even] += oddReal;
                imaginary[even] += oddImaginary;
                const float nextReal = rotationReal * stepReal - rotationImaginary * stepImaginary;
                rotationImaginary = rotationReal * stepImaginary + rotationImaginary * stepReal;
                rotationReal = nextReal;
            }
        }
    }
}
}

float TemperatureAverage::add(float temperature)
{
    if (std::isfinite(temperature))
    {
        samples_[next_] = temperature;
        next_ = (next_ + 1) % TEMPERATURE_WINDOW;
        if (count_ < TEMPERATURE_WINDOW) ++count_;
    }
    return value();
}

float TemperatureAverage::value() const
{
    double sum = 0.0;
    for (std::size_t index = 0; index < count_; ++index) sum += samples_[index];
    return count_ == 0 ? 0.0f : static_cast<float>(sum / count_);
}

bool analyzeVibration(const float *samples, std::size_t count, float sampleRateHz,
                      VibrationFeatures &result)
{
    result = VibrationFeatures{};
    if (samples == nullptr || count != VIBRATION_SAMPLES ||
        !std::isfinite(sampleRateHz) || sampleRateHz <= 0.0f)
        return false;

    double sum = 0.0;
    float lowest = samples[0];
    float highest = samples[0];
    for (std::size_t index = 0; index < count; ++index)
    {
        const float sample = samples[index];
        if (!std::isfinite(sample)) return false;
        sum += sample;
        if (sample < lowest) lowest = sample;
        if (sample > highest) highest = sample;
    }

    const double mean = sum / count;
    double squares = 0.0;
    float real[VIBRATION_SAMPLES];
    float imaginary[VIBRATION_SAMPLES]{};
    for (std::size_t index = 0; index < count; ++index)
    {
        const double centered = samples[index] - mean;
        squares += centered * centered;
        const float hann = 0.5f * (1.0f - std::cos(TWO_PI * index / (count - 1)));
        real[index] = static_cast<float>(centered * hann);
    }

    const float rms = static_cast<float>(std::sqrt(squares / count));
    const float peakToPeak = highest - lowest;
    if (!validVibrationValues(rms, peakToPeak)) return false;

    float dominantFrequencyHz = 0.0f;
    if (squares != 0.0)
    {
        fft(real, imaginary);
        std::size_t strongestBin = 0;
        double strongestPower = 0.0;
        // The zero-frequency bin is deliberately excluded after DC removal.
        for (std::size_t bin = 1; bin <= count / 2; ++bin)
        {
            const double power = static_cast<double>(real[bin]) * real[bin] +
                                 static_cast<double>(imaginary[bin]) * imaginary[bin];
            if (power > strongestPower)
            {
                strongestPower = power;
                strongestBin = bin;
            }
        }
        dominantFrequencyHz = strongestBin * (sampleRateHz / count);
    }

    result.rms = rms;
    result.peakToPeak = peakToPeak;
    result.dominantFrequencyHz = dominantFrequencyHz;
    result.valid = true;
    return true;
}

FeatureValues calculateFeatures(float temperature, float humidity, bool dhtValid,
                               const VibrationFeatures &vibration, TemperatureAverage &average)
{
    FeatureValues result{};
    const bool dhtReady = dhtValid && validDhtValues(temperature, humidity);
    if (dhtReady)
    {
        result.temperature = average.add(temperature);
        result.humidity = humidity;
    }

    const bool vibrationReady = vibration.valid && validVibrationValues(vibration.rms, vibration.peakToPeak);
    if (vibrationReady)
    {
        result.vibrationRms = vibration.rms;
        result.vibrationPeakToPeak = vibration.peakToPeak;
    }
    result.dhtValid = dhtReady;
    result.vibrationValid = vibrationReady;
    result.valid = dhtReady && vibrationReady;
    return result;
}

uint16_t composeAlarms(uint16_t stateAlarms, bool dhtValid, bool vibrationValid,
                       bool networkOnline, bool dataLoss)
{
    uint16_t alarms = stateAlarms;
    if (!networkOnline) alarms |= ALARM_NETWORK_OFFLINE;
    // Reported even when the DHT also failed, so the cloud can always separate a
    // vibration-source fault from a humidity/temperature one.
    if (!vibrationValid) alarms |= ALARM_VIBRATION_INVALID;
    if (dataLoss) alarms |= ALARM_DATA_LOSS;
    if (!dhtValid) alarms |= ALARM_SENSOR_INVALID;
    return alarms;
}

bool StateMachine::update(const FeatureValues &frame, uint32_t nowMs)
{
    const DeviceState previous = state;
    const bool valid = frame.valid && validDhtValues(frame.temperature, frame.humidity) &&
                       validVibrationValues(frame.vibrationRms, frame.vibrationPeakToPeak);
    alarms = 0;
    if (!valid) alarms |= ALARM_SENSOR_INVALID;
    if (valid && frame.temperature >= TEMPERATURE_WARNING) alarms |= ALARM_TEMPERATURE;
    if (valid && frame.humidity >= HUMIDITY_WARNING) alarms |= ALARM_HUMIDITY;
    if (valid && frame.vibrationRms >= VIBRATION_WARNING) alarms |= ALARM_VIBRATION;

    const bool warning = (alarms & (ALARM_TEMPERATURE | ALARM_HUMIDITY | ALARM_VIBRATION)) != 0;
    const bool fault = valid && (frame.temperature >= TEMPERATURE_FAULT || frame.vibrationRms >= VIBRATION_FAULT);
    const bool safe = valid && frame.temperature < TEMPERATURE_CLEAR &&
                      frame.humidity < HUMIDITY_CLEAR && frame.vibrationRms < VIBRATION_CLEAR;

    countConsecutive(startupReadyCount_, valid && !warning);
    countConsecutive(invalidCount_, !valid);
    countConsecutive(warningCount_, warning);
    countConsecutive(faultCount_, fault);
    countConsecutive(safeCount_, safe);

    const bool faultConfirmed = faultCount_ >= 2 || invalidCount_ >= 5;
    const bool warningConfirmed = warningCount_ >= 3 || invalidCount_ >= 3;
    switch (state)
    {
    case DeviceState::Init:
        // Startup values above a threshold must not temporarily report NORMAL.
        if (faultConfirmed) setState(DeviceState::Fault, nowMs);
        else if (warningConfirmed) setState(DeviceState::Warning, nowMs);
        else if (startupReadyCount_ >= 2) setState(DeviceState::Normal, nowMs);
        break;
    case DeviceState::Normal:
        if (faultConfirmed) setState(DeviceState::Fault, nowMs);
        else if (warningConfirmed) setState(DeviceState::Warning, nowMs);
        break;
    case DeviceState::Warning:
        if (faultConfirmed) setState(DeviceState::Fault, nowMs);
        else if (safeCount_ >= 3) setState(DeviceState::Recovery, nowMs);
        break;
    case DeviceState::Fault:
        if (safeCount_ >= 3) setState(DeviceState::Recovery, nowMs);
        break;
    case DeviceState::Recovery:
        if (fault) setState(DeviceState::Fault, nowMs);
        else if (warning || warningConfirmed) setState(DeviceState::Warning, nowMs);
        else if (safeCount_ >= 4) setState(DeviceState::Normal, nowMs);
        break;
    }
    return state != previous;
}

void StateMachine::setState(DeviceState next, uint32_t nowMs)
{
    state = next;
    enteredAtMs = nowMs;
    safeCount_ = 0;
    warningCount_ = 0;
    faultCount_ = 0;
}
