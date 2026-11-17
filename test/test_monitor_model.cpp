#include "monitor_model.h"
#include "liveness.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
std::size_t checks = 0;
bool transitions[5][5]{};

void require(bool condition, const char *expression, int line)
{
    ++checks;
    if (!condition)
        throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
}

#define CHECK(condition) require((condition), #condition, __LINE__)

bool closeTo(float actual, float expected, float tolerance = 0.0001f)
{
    return std::isfinite(actual) && std::fabs(actual - expected) <= tolerance;
}

const FeatureValues SAFE{25.0f, 50.0f, 0.2f, 0.4f, true, true, true};
const FeatureValues WARNING{35.0f, 50.0f, 0.2f, 0.4f, true, true, true};
const FeatureValues FAULT{45.0f, 50.0f, 0.2f, 0.4f, true, true, true};
const FeatureValues INVALID{0.0f, 0.0f, 0.0f, 0.0f, false, false, false};

struct Driver
{
    StateMachine machine{100};
    uint32_t now = 100;

    void step(const FeatureValues &frame, DeviceState expected)
    {
        const DeviceState previous = machine.state;
        const uint32_t entered = machine.enteredAtMs;
        now += 17;
        const bool changed = machine.update(frame, now);
        CHECK(machine.state == expected);
        CHECK(changed == (previous != expected));
        CHECK(machine.enteredAtMs == (changed ? now : entered));
        if (changed)
            transitions[static_cast<std::size_t>(previous)][static_cast<std::size_t>(expected)] = true;
    }

    void normal()
    {
        step(SAFE, DeviceState::Init);
        step(SAFE, DeviceState::Normal);
    }

    void warning()
    {
        normal();
        step(WARNING, DeviceState::Normal);
        step(WARNING, DeviceState::Normal);
        step(WARNING, DeviceState::Warning);
    }

    void recovery()
    {
        warning();
        step(SAFE, DeviceState::Warning);
        step(SAFE, DeviceState::Warning);
        step(SAFE, DeviceState::Recovery);
    }
};

void testStartup()
{
    Driver normal;
    CHECK(normal.machine.state == DeviceState::Init);
    CHECK(normal.machine.enteredAtMs == 100);
    normal.step(SAFE, DeviceState::Init);
    normal.step(INVALID, DeviceState::Init);
    normal.step(SAFE, DeviceState::Init);
    normal.step(SAFE, DeviceState::Normal);

    Driver warning;
    warning.step(WARNING, DeviceState::Init);
    warning.step(WARNING, DeviceState::Init);
    warning.step(WARNING, DeviceState::Warning);

    Driver fault;
    fault.step(FAULT, DeviceState::Init);
    fault.step(FAULT, DeviceState::Fault);

    Driver interruptedFault;
    interruptedFault.step(FAULT, DeviceState::Init);
    interruptedFault.step(SAFE, DeviceState::Init);
    interruptedFault.step(FAULT, DeviceState::Init);
    interruptedFault.step(FAULT, DeviceState::Fault);

    Driver invalid;
    invalid.step(INVALID, DeviceState::Init);
    invalid.step(INVALID, DeviceState::Init);
    invalid.step(INVALID, DeviceState::Warning);
    invalid.step(INVALID, DeviceState::Warning);
    invalid.step(INVALID, DeviceState::Fault);
}

void testWarningAndFaultCounters()
{
    Driver driver;
    driver.normal();
    driver.step(WARNING, DeviceState::Normal);
    driver.step(WARNING, DeviceState::Normal);
    driver.step(SAFE, DeviceState::Normal);
    driver.step(WARNING, DeviceState::Normal);
    driver.step(WARNING, DeviceState::Normal);
    driver.step(WARNING, DeviceState::Warning);
    CHECK(driver.machine.alarms == ALARM_TEMPERATURE);
    driver.step(FAULT, DeviceState::Warning);
    driver.step(WARNING, DeviceState::Warning);
    driver.step(FAULT, DeviceState::Warning);
    driver.step(FAULT, DeviceState::Fault);

    Driver directFault;
    directFault.normal();
    directFault.step(FAULT, DeviceState::Normal);
    directFault.step(FAULT, DeviceState::Fault);

    Driver invalid;
    invalid.normal();
    invalid.step(INVALID, DeviceState::Normal);
    invalid.step(INVALID, DeviceState::Normal);
    invalid.step(SAFE, DeviceState::Normal);
    invalid.step(INVALID, DeviceState::Normal);
    invalid.step(INVALID, DeviceState::Normal);
    invalid.step(INVALID, DeviceState::Warning);
    CHECK(invalid.machine.alarms == ALARM_SENSOR_INVALID);
    invalid.step(INVALID, DeviceState::Warning);
    invalid.step(INVALID, DeviceState::Fault);

    // Numeric conditions on a frame marked invalid cannot create a numeric fault.
    FeatureValues invalidHigh = FAULT;
    invalidHigh.valid = false;
    Driver combinedInvalid;
    combinedInvalid.normal();
    combinedInvalid.step(invalidHigh, DeviceState::Normal);
    combinedInvalid.step(invalidHigh, DeviceState::Normal);
    combinedInvalid.step(invalidHigh, DeviceState::Warning);
    CHECK(combinedInvalid.machine.alarms == ALARM_SENSOR_INVALID);
}

void testThresholdsAndAlarmBits()
{
    FeatureValues humidity = SAFE;
    humidity.humidity = HUMIDITY_WARNING;
    Driver humidityDriver;
    humidityDriver.normal();
    humidityDriver.step(humidity, DeviceState::Normal);
    CHECK(humidityDriver.machine.alarms == ALARM_HUMIDITY);
    humidityDriver.step(humidity, DeviceState::Normal);
    humidityDriver.step(humidity, DeviceState::Warning);
    for (int index = 0; index < 8; ++index)
        humidityDriver.step(humidity, DeviceState::Warning); // No humidity-only FAULT.

    FeatureValues vibration = SAFE;
    vibration.vibrationRms = VIBRATION_WARNING;
    Driver vibrationDriver;
    vibrationDriver.normal();
    vibrationDriver.step(vibration, DeviceState::Normal);
    CHECK(vibrationDriver.machine.alarms == ALARM_VIBRATION);
    vibrationDriver.step(vibration, DeviceState::Normal);
    vibrationDriver.step(vibration, DeviceState::Warning);

    vibration.vibrationRms = VIBRATION_FAULT;
    Driver vibrationFault;
    vibrationFault.normal();
    vibrationFault.step(vibration, DeviceState::Normal);
    vibrationFault.step(vibration, DeviceState::Fault);

    FeatureValues combined = FAULT;
    combined.humidity = HUMIDITY_WARNING;
    combined.vibrationRms = VIBRATION_FAULT;
    Driver allAlarms;
    allAlarms.step(combined, DeviceState::Init);
    CHECK(allAlarms.machine.alarms == (ALARM_TEMPERATURE | ALARM_VIBRATION | ALARM_HUMIDITY));

    FeatureValues below = SAFE;
    below.temperature = TEMPERATURE_WARNING - 0.001f;
    below.humidity = HUMIDITY_WARNING - 0.001f;
    below.vibrationRms = VIBRATION_WARNING - 0.001f;
    Driver belowThreshold;
    belowThreshold.normal();
    for (int index = 0; index < 6; ++index)
        belowThreshold.step(below, DeviceState::Normal);
    CHECK(belowThreshold.machine.alarms == 0);

    // The existing ordinary-warning counter observes the OR of all warning causes.
    Driver alternating;
    alternating.normal();
    alternating.step(WARNING, DeviceState::Normal);
    alternating.step(humidity, DeviceState::Normal);
    vibration.vibrationRms = VIBRATION_WARNING;
    alternating.step(vibration, DeviceState::Warning);
}

void testRecoveryAndHysteresis()
{
    FeatureValues boundary = SAFE;
    Driver driver;
    driver.warning();
    for (int metric = 0; metric < 3; ++metric)
    {
        driver.step(SAFE, DeviceState::Warning);
        driver.step(SAFE, DeviceState::Warning);
        boundary = SAFE;
        if (metric == 0) boundary.temperature = TEMPERATURE_CLEAR;
        if (metric == 1) boundary.humidity = HUMIDITY_CLEAR;
        if (metric == 2) boundary.vibrationRms = VIBRATION_CLEAR;
        driver.step(boundary, DeviceState::Warning); // Clear thresholds are strict '<'.
    }
    driver.step(SAFE, DeviceState::Warning);
    driver.step(SAFE, DeviceState::Warning);
    driver.step(SAFE, DeviceState::Recovery);
    driver.step(SAFE, DeviceState::Recovery);
    driver.step(SAFE, DeviceState::Recovery);
    driver.step(boundary, DeviceState::Recovery); // Interrupts the four-frame recovery count.
    driver.step(SAFE, DeviceState::Recovery);
    driver.step(SAFE, DeviceState::Recovery);
    driver.step(SAFE, DeviceState::Recovery);
    driver.step(SAFE, DeviceState::Normal);

    Driver fault;
    fault.normal();
    fault.step(FAULT, DeviceState::Normal);
    fault.step(FAULT, DeviceState::Fault);
    fault.step(SAFE, DeviceState::Fault);
    fault.step(INVALID, DeviceState::Fault);
    fault.step(SAFE, DeviceState::Fault);
    fault.step(SAFE, DeviceState::Fault);
    fault.step(SAFE, DeviceState::Recovery);
}

void testRecoveryRelapsesAndSaturation()
{
    Driver warning;
    warning.recovery();
    warning.step(WARNING, DeviceState::Warning); // Relapse does not wait three frames.

    Driver fault;
    fault.recovery();
    fault.step(FAULT, DeviceState::Fault); // Relapse does not wait two frames.

    Driver invalid;
    invalid.recovery();
    invalid.step(INVALID, DeviceState::Recovery);
    invalid.step(INVALID, DeviceState::Recovery);
    invalid.step(INVALID, DeviceState::Warning);
    invalid.step(INVALID, DeviceState::Warning);
    invalid.step(INVALID, DeviceState::Fault);
    for (int index = 0; index < 300; ++index)
        invalid.step(INVALID, DeviceState::Fault);
    invalid.step(SAFE, DeviceState::Fault);
    invalid.step(SAFE, DeviceState::Fault);
    invalid.step(SAFE, DeviceState::Recovery);
    for (int index = 0; index < 3; ++index)
        invalid.step(SAFE, DeviceState::Recovery);
    invalid.step(SAFE, DeviceState::Normal);

    Driver heldWarning;
    heldWarning.warning();
    for (int index = 0; index < 300; ++index)
        heldWarning.step(WARNING, DeviceState::Warning);
    heldWarning.step(SAFE, DeviceState::Warning);
    heldWarning.step(SAFE, DeviceState::Warning);
    heldWarning.step(SAFE, DeviceState::Recovery);
}

void testInvalidNumbers()
{
    const float bad[] = {std::numeric_limits<float>::quiet_NaN(),
                         std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()};
    for (float value : bad)
    {
        for (int metric = 0; metric < 4; ++metric)
        {
            FeatureValues invalid = SAFE;
            if (metric == 0) invalid.temperature = value;
            if (metric == 1) invalid.humidity = value;
            if (metric == 2) invalid.vibrationRms = value;
            if (metric == 3) invalid.vibrationPeakToPeak = value;
            Driver driver;
            driver.normal();
            driver.step(invalid, DeviceState::Normal);
            driver.step(invalid, DeviceState::Normal);
            driver.step(invalid, DeviceState::Warning);
            CHECK(driver.machine.alarms == ALARM_SENSOR_INVALID);
        }
    }
}

void testTemperatureAndCombinedValidity()
{
    TemperatureAverage average;
    CHECK(average.value() == 0);
    CHECK(average.add(10) == 10);
    CHECK(average.add(20) == 15);
    CHECK(average.add(30) == 20);
    CHECK(average.add(40) == 25);
    CHECK(average.add(50) == 30);
    CHECK(average.add(60) == 40);
    CHECK(average.add(std::numeric_limits<float>::quiet_NaN()) == 40);
    TemperatureAverage large;
    const float maximum = std::numeric_limits<float>::max();
    CHECK(large.add(maximum) == maximum);
    CHECK(large.add(maximum) == maximum);

    VibrationFeatures vibration;
    vibration.rms = 0.2f;
    vibration.peakToPeak = 0.4f;
    vibration.valid = true;
    TemperatureAverage independent;
    FeatureValues frame = calculateFeatures(10, 50, true, vibration, independent);
    CHECK(frame.valid && frame.temperature == 10);
    frame = calculateFeatures(80, 50, false, vibration, independent);
    CHECK(!frame.valid && independent.value() == 10);
    vibration.valid = false;
    frame = calculateFeatures(20, 50, true, vibration, independent);
    CHECK(!frame.valid && frame.temperature == 15 && independent.value() == 15);
    vibration.valid = true;
    frame = calculateFeatures(30, 50, true, vibration, independent);
    CHECK(frame.valid && frame.temperature == 20);
    const float previous = independent.value();
    frame = calculateFeatures(std::numeric_limits<float>::quiet_NaN(), 50, true, vibration, independent);
    CHECK(!frame.valid && independent.value() == previous);
    frame = calculateFeatures(25, 101, true, vibration, independent);
    CHECK(!frame.valid && independent.value() == previous);
    frame = calculateFeatures(81, 50, true, vibration, independent);
    CHECK(!frame.valid && independent.value() == previous);
    vibration.rms = -1;
    frame = calculateFeatures(25, 50, true, vibration, independent);
    CHECK(!frame.valid);
}

void makeSine(float (&samples)[VIBRATION_SAMPLES], float frequency, float amplitude, float dc)
{
    constexpr double TWO_PI = 6.2831853071795864769;
    for (std::size_t index = 0; index < VIBRATION_SAMPLES; ++index)
        samples[index] = static_cast<float>(dc + amplitude * std::sin(TWO_PI * frequency * index / VIBRATION_SAMPLE_RATE_HZ));
}

void testVibrationMath()
{
    float samples[VIBRATION_SAMPLES];
    VibrationFeatures features;
    makeSine(samples, 25, 2, 15);
    CHECK(analyzeVibration(samples, VIBRATION_SAMPLES, VIBRATION_SAMPLE_RATE_HZ, features));
    CHECK(features.valid);
    CHECK(closeTo(features.rms, std::sqrt(2.0f)));
    CHECK(closeTo(features.peakToPeak, 4));
    CHECK(closeTo(features.dominantFrequencyHz, 25));
    CHECK(samples[0] == 15); // Analysis must not modify the caller's samples.
    CHECK(analyzeVibration(samples, VIBRATION_SAMPLES, 400, features));
    CHECK(closeTo(features.dominantFrequencyHz, 50));
    CHECK(closeTo(features.rms, std::sqrt(2.0f)));

    // The window belongs to spectral analysis; it must not lower the time-domain RMS.
    makeSine(samples, 12.5f, 1, 0);
    CHECK(analyzeVibration(samples, VIBRATION_SAMPLES, VIBRATION_SAMPLE_RATE_HZ, features));
    CHECK(closeTo(features.rms, std::sqrt(0.5f)));
    CHECK(closeTo(features.peakToPeak, 2));
    CHECK(closeTo(features.dominantFrequencyHz, 12.5f));

    for (float &sample : samples) sample = 7;
    CHECK(analyzeVibration(samples, VIBRATION_SAMPLES, VIBRATION_SAMPLE_RATE_HZ, features));
    CHECK(features.valid && features.rms == 0 && features.peakToPeak == 0 && features.dominantFrequencyHz == 0);

    for (std::size_t index = 0; index < VIBRATION_SAMPLES; ++index)
        samples[index] = (index % 2 == 0) ? 3.0f : -3.0f;
    CHECK(analyzeVibration(samples, VIBRATION_SAMPLES, VIBRATION_SAMPLE_RATE_HZ, features));
    CHECK(closeTo(features.rms, 3) && closeTo(features.peakToPeak, 6));
    CHECK(closeTo(features.dominantFrequencyHz, 100));
}

void testFftMixturesAndWindow()
{
    constexpr double TWO_PI = 6.2831853071795864769;
    float samples[VIBRATION_SAMPLES];
    VibrationFeatures features;
    for (std::size_t index = 0; index < VIBRATION_SAMPLES; ++index)
    {
        const double time = index / static_cast<double>(VIBRATION_SAMPLE_RATE_HZ);
        samples[index] = static_cast<float>(30 + 1.5 * std::sin(TWO_PI * 37.5 * time) +
                                           0.4 * std::sin(TWO_PI * 12.5 * time));
    }
    CHECK(analyzeVibration(samples, VIBRATION_SAMPLES, VIBRATION_SAMPLE_RATE_HZ, features));
    CHECK(closeTo(features.rms, std::sqrt((1.5f * 1.5f + 0.4f * 0.4f) / 2)));
    CHECK(closeTo(features.dominantFrequencyHz, 37.5f));

    // A half-bin strong tone loses more peak magnitude with a rectangular window.
    // The Hann transform must select it over the weaker, bin-centred 50 Hz tone.
    constexpr float OFF_BIN_HZ = 26.5f * VIBRATION_SAMPLE_RATE_HZ / VIBRATION_SAMPLES;
    for (std::size_t index = 0; index < VIBRATION_SAMPLES; ++index)
    {
        const double time = index / static_cast<double>(VIBRATION_SAMPLE_RATE_HZ);
        samples[index] = static_cast<float>(100 + std::sin(TWO_PI * OFF_BIN_HZ * time) +
                                           0.75 * std::sin(TWO_PI * 50 * time));
    }
    CHECK(analyzeVibration(samples, VIBRATION_SAMPLES, VIBRATION_SAMPLE_RATE_HZ, features));
    CHECK(closeTo(features.dominantFrequencyHz, OFF_BIN_HZ, 0.391f));
    CHECK(features.rms > 0.85f && features.rms < 0.92f);
}

void testVibrationValidation()
{
    float samples[VIBRATION_SAMPLES]{};
    VibrationFeatures features;
    features.valid = true;
    features.rms = 9;
    CHECK(!analyzeVibration(nullptr, VIBRATION_SAMPLES, 200, features));
    CHECK(!features.valid && features.rms == 0);
    const std::size_t badCounts[] = {0, 1, 32, 255, 257, 512};
    for (std::size_t count : badCounts)
        CHECK(!analyzeVibration(samples, count, 200, features));
    const float badRates[] = {0, -200, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()};
    for (float rate : badRates)
        CHECK(!analyzeVibration(samples, VIBRATION_SAMPLES, rate, features));
    samples[123] = std::numeric_limits<float>::quiet_NaN();
    CHECK(!analyzeVibration(samples, VIBRATION_SAMPLES, 200, features));
    samples[123] = std::numeric_limits<float>::infinity();
    CHECK(!analyzeVibration(samples, VIBRATION_SAMPLES, 200, features));
    samples[123] = std::numeric_limits<float>::max();
    samples[124] = -std::numeric_limits<float>::max();
    CHECK(!analyzeVibration(samples, VIBRATION_SAMPLES, 200, features));
    CHECK(!features.valid);
}

void testHeartbeat()
{
    CHECK(!heartbeatExpired(100, 100));
    CHECK(!heartbeatExpired(10000, 0));
    CHECK(heartbeatExpired(10001, 0));
    CHECK(!heartbeatExpired(100, 101)); // A newer cross-core timestamp is not stale.
    const uint32_t last = std::numeric_limits<uint32_t>::max() - 4000;
    CHECK(!heartbeatExpired(last + 10000, last));
    CHECK(heartbeatExpired(last + 10001, last));
    CHECK(HEARTBEAT_TIMEOUT_MS < TASK_WATCHDOG_TIMEOUT_MS);
}

void checkHeartbeatChanges(const HeartbeatChanges &changes, uint32_t active,
                           uint32_t newlyStale, uint32_t recovered)
{
    CHECK(changes.activeMask == active);
    CHECK(changes.newlyStaleMask == newlyStale);
    CHECK(changes.recoveredMask == recovered);
}

void testHeartbeatRecoveryEpisodes()
{
    HeartbeatMonitor monitor;
    uint32_t last[6]{};
    checkHeartbeatChanges(monitor.update(10000, last, 6), 0, 0, 0);

    for (uint32_t &stamp : last) stamp = 10001;
    last[2] = 0;
    checkHeartbeatChanges(monitor.update(10001, last, 6), 1u << 2, 1u << 2, 0);
    checkHeartbeatChanges(monitor.update(10002, last, 6), 1u << 2, 0, 0);
    last[2] = 10003;
    checkHeartbeatChanges(monitor.update(10003, last, 6), 0, 0, 1u << 2);
    checkHeartbeatChanges(monitor.update(10004, last, 6), 0, 0, 0);

    // The same task can become late again after recovering; it is a fresh episode.
    for (uint32_t &stamp : last) stamp = 20004;
    last[2] = 10003;
    checkHeartbeatChanges(monitor.update(20004, last, 6), 1u << 2, 1u << 2, 0);
    last[2] = 20005;
    checkHeartbeatChanges(monitor.update(20005, last, 6), 0, 0, 1u << 2);

    // One task recovering while another becomes late must not retain the old bit.
    for (uint32_t &stamp : last) stamp = 30000;
    last[0] = 0;
    checkHeartbeatChanges(monitor.update(30000, last, 6), 1u, 1u, 0);
    last[0] = 30001;
    last[5] = 0;
    checkHeartbeatChanges(monitor.update(30001, last, 6), 1u << 5, 1u << 5, 1u);
    last[5] = 30002;
    checkHeartbeatChanges(monitor.update(30002, last, 6), 0, 0, 1u << 5);
    checkHeartbeatChanges(monitor.update(30003, last, 6), 0, 0, 0);
}

void testHeartbeatMonitorWrapAndWidth()
{
    HeartbeatMonitor monitor;
    const uint32_t old = std::numeric_limits<uint32_t>::max() - 4000;
    const uint32_t now = old + 10000;
    uint32_t last[6] = {now, now + 1, old, now, now + 1, now};
    checkHeartbeatChanges(monitor.update(now, last, 6), 0, 0, 0);
    checkHeartbeatChanges(monitor.update(now + 1, last, 6), 1u << 2, 1u << 2, 0);
    last[2] = now + 2;
    checkHeartbeatChanges(monitor.update(now + 2, last, 6), 0, 0, 1u << 2);

    HeartbeatMonitor fullWidth;
    uint32_t allTasks[32];
    for (uint32_t &stamp : allTasks) stamp = 10002; // Concurrent writers may be ahead.
    checkHeartbeatChanges(fullWidth.update(10001, allTasks, 32), 0, 0, 0);
    allTasks[31] = 0;
    checkHeartbeatChanges(fullWidth.update(10001, allTasks, 32), 0x80000000u, 0x80000000u, 0);
    for (uint32_t &stamp : allTasks) stamp = 0;
    checkHeartbeatChanges(fullWidth.update(10001, allTasks, 32), 0xffffffffu, 0x7fffffffu, 0);
    for (uint32_t &stamp : allTasks) stamp = 10002;
    checkHeartbeatChanges(fullWidth.update(10002, allTasks, 32), 0, 0, 0xffffffffu);
}

void testAlarmComposition()
{
    // Both sources healthy and online: the state machine owns every bit.
    CHECK(composeAlarms(0, true, true, true, false) == 0);
    CHECK(composeAlarms(ALARM_TEMPERATURE, true, true, true, false) == ALARM_TEMPERATURE);

    // A vibration-only fault must be distinguishable from a DHT-only fault.
    CHECK(composeAlarms(0, true, false, true, false) == ALARM_VIBRATION_INVALID);
    CHECK(composeAlarms(0, false, true, true, false) == ALARM_SENSOR_INVALID);
    CHECK(composeAlarms(ALARM_SENSOR_INVALID, false, false, true, false) ==
          (ALARM_SENSOR_INVALID | ALARM_VIBRATION_INVALID));

    // Link and data-integrity conditions are independent of the measured values.
    CHECK(composeAlarms(0, true, true, false, false) == ALARM_NETWORK_OFFLINE);
    CHECK(composeAlarms(0, true, true, true, true) == ALARM_DATA_LOSS);
    CHECK(composeAlarms(ALARM_HUMIDITY | ALARM_VIBRATION, false, false, false, true) ==
          (ALARM_HUMIDITY | ALARM_VIBRATION | ALARM_SENSOR_INVALID | ALARM_VIBRATION_INVALID |
           ALARM_NETWORK_OFFLINE | ALARM_DATA_LOSS));
}

void testPerSourceValidity()
{
    TemperatureAverage average;
    VibrationFeatures good;
    good.rms = 0.5f;
    good.peakToPeak = 1.2f;
    good.valid = true;

    // A failed vibration window leaves zeroed values, which must not be read as a
    // valid measurement just because zero passes the numeric range checks.
    const FeatureValues bothGood = calculateFeatures(26, 50, true, good, average);
    CHECK(bothGood.dhtValid && bothGood.vibrationValid && bothGood.valid);

    const FeatureValues dhtOnly = calculateFeatures(26, 50, true, VibrationFeatures{}, average);
    CHECK(dhtOnly.dhtValid && !dhtOnly.vibrationValid && !dhtOnly.valid);
    CHECK(dhtOnly.vibrationRms == 0);

    const FeatureValues vibrationOnly = calculateFeatures(26, 50, false, good, average);
    CHECK(!vibrationOnly.dhtValid && vibrationOnly.vibrationValid && !vibrationOnly.valid);

    // Out-of-range numbers are rejected even when the caller marks the source valid.
    const FeatureValues badRange = calculateFeatures(200, 50, true, good, average);
    CHECK(!badRange.dhtValid && badRange.vibrationValid && !badRange.valid);
    CHECK(calculateFeatures(26, 150, true, good, average).valid == false);

    // A nonfinite reading cannot enter the temperature window either.
    TemperatureAverage isolated;
    calculateFeatures(std::numeric_limits<float>::quiet_NaN(), 50, true, good, isolated);
    CHECK(isolated.value() == 0);
}

void testEveryTransitionObserved()
{
    using State = DeviceState;
    const std::pair<State, State> required[] = {
        {State::Init, State::Normal}, {State::Init, State::Warning}, {State::Init, State::Fault},
        {State::Normal, State::Warning}, {State::Normal, State::Fault},
        {State::Warning, State::Fault}, {State::Warning, State::Recovery},
        {State::Fault, State::Recovery}, {State::Recovery, State::Normal},
        {State::Recovery, State::Warning}, {State::Recovery, State::Fault},
    };
    for (const auto &edge : required)
        CHECK(transitions[static_cast<std::size_t>(edge.first)][static_cast<std::size_t>(edge.second)]);
    std::printf("[PASS] All %zu implemented state-transition edges observed\n", sizeof(required) / sizeof(required[0]));
}
}

int main()
{
    try
    {
        testStartup();
        testWarningAndFaultCounters();
        testThresholdsAndAlarmBits();
        testRecoveryAndHysteresis();
        testRecoveryRelapsesAndSaturation();
        testInvalidNumbers();
        testTemperatureAndCombinedValidity();
        testVibrationMath();
        testFftMixturesAndWindow();
        testVibrationValidation();
        testHeartbeat();
        testHeartbeatRecoveryEpisodes();
        testHeartbeatMonitorWrapAndWidth();
        testAlarmComposition();
        testPerSourceValidity();
        testEveryTransitionObserved();
        std::printf("[PASS] %zu checks: shared state machine, DSP, sensor validity, and heartbeat boundaries\n", checks);
        return 0;
    }
    catch (const std::exception &error)
    {
        std::fprintf(stderr, "[FAIL] %s\n", error.what());
        return 1;
    }
}
