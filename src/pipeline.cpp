#include "terminal.h"
#include "monitor_model.h"
#include "vibration_source.h"

#include <DHT.h>
#include <cmath>

namespace
{
constexpr uint32_t SAMPLE_PERIOD_MS = 2000;
constexpr size_t PIPELINE_QUEUE_DEPTH = 8;
// Resonant point of the passive buzzer; analogWrite's 1 kHz default is too far off.
constexpr uint32_t BUZZER_TONE_HZ = 2731;

enum class SampleMode : uint8_t { Pot, Warning, Fault, InvalidDht };
constexpr const char *MODE_NAMES[] = {"POT", "WARNING", "FAULT", "INVALID_DHT"};
std::atomic<SampleMode> sampleMode{SampleMode::Pot};

struct SensorFrame
{
    uint32_t timestampMs;
    float temperature;
    float humidity;
    float vibration[VIBRATION_SAMPLES];
    bool dhtValid;
    bool vibrationValid;
};

struct FeatureFrame : FeatureValues
{
    uint32_t timestampMs;
    float dominantFrequencyHz;
};

float vibrationAmplitude(SampleMode mode)
{
    switch (mode)
    {
    case SampleMode::Warning: return 2.0f;
    case SampleMode::Fault: return 3.0f;
    case SampleMode::InvalidDht: return 0.0f;
    case SampleMode::Pot: return analogRead(POT_PIN) * (3.0f / 4095.0f);
    }
    return 0.0f;
}

bool keepSampling()
{
    markHeartbeat(TaskId::Sensor);
    return (xEventGroupGetBits(networkEvents) & STOP_SAMPLING_BIT) == 0;
}

void updateAlarmOutputs(const StateMachine &machine)
{
    const uint32_t elapsed = millis() - machine.enteredAtMs;
    bool ledOn = false;
    bool buzzerOn = false;
    switch (machine.state)
    {
    case DeviceState::Init: ledOn = elapsed % 250 < 125; break;
    case DeviceState::Normal: ledOn = elapsed % 2000 < 100; break;
    case DeviceState::Warning:
        ledOn = elapsed % 400 < 200;
        buzzerOn = elapsed % 1200 < 150;
        break;
    case DeviceState::Fault:
        ledOn = true;
        buzzerOn = elapsed % 250 < 125;
        break;
    case DeviceState::Recovery: ledOn = elapsed % 1200 < 600; break;
    }
    digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
    static bool wasBuzzerOn = false;
    if (buzzerOn != wasBuzzerOn)
    {
        // A passive buzzer needs the carrier frequency, not a duty on the 1 kHz
        // analogWrite default, so the tone is driven by an explicit LEDC channel.
        ledcWrite(BUZZER_PIN, buzzerOn ? 128 : 0);
        wasBuzzerOn = buzzerOn;
    }
}

void handleModeButton()
{
    if (ulTaskNotifyTake(pdTRUE, 0) == 0) return;
    constexpr uint32_t DEBOUNCE_MS = 200;
    static uint32_t lastPress = millis() - DEBOUNCE_MS;
    const uint32_t now = millis();
    if (now - lastPress < DEBOUNCE_MS) return;
    lastPress = now;
    const auto current = static_cast<uint8_t>(sampleMode.load(std::memory_order_relaxed));
    const auto next = (current + 1) % (sizeof(MODE_NAMES) / sizeof(MODE_NAMES[0]));
    sampleMode.store(static_cast<SampleMode>(next), std::memory_order_relaxed);
    logPrintf("[TEST] Sample mode: %s\n", MODE_NAMES[next]);
}
}

bool createPipelineQueues()
{
    rawQueue = xQueueCreate(PIPELINE_QUEUE_DEPTH, sizeof(SensorFrame));
    featureQueue = xQueueCreate(PIPELINE_QUEUE_DEPTH, sizeof(FeatureFrame));
    return rawQueue && featureQueue;
}

void sensorTask(void *)
{
    DHT dht(DHT_PIN, DHT22);
    dht.begin();
    analogReadResolution(12);
    markHeartbeat(TaskId::Sensor);
    const bool ready = beginVibrationSource();
    logPrintf("[VIB] source=%s ready=%d Fs=%.1fHz N=%u\n",
         vibrationSourceName(), ready, VIBRATION_SAMPLE_RATE_HZ,
         static_cast<unsigned>(VIBRATION_SAMPLES));
    vTaskDelay(pdMS_TO_TICKS(2200));
    TickType_t nextWake = xTaskGetTickCount();

    for (;;)
    {
        markHeartbeat(TaskId::Sensor);
        if (keepSampling())
        {
            const auto mode = sampleMode.load(std::memory_order_relaxed);
            SensorFrame frame{};
            frame.temperature = dht.readTemperature();
            frame.humidity = dht.readHumidity();
            frame.dhtValid = mode != SampleMode::InvalidDht &&
                             frame.temperature >= -40.0f && frame.temperature <= 80.0f &&
                             frame.humidity >= 0.0f && frame.humidity <= 100.0f;
            frame.vibrationValid = readVibrationWindow(frame.vibration, VIBRATION_SAMPLES,
                                                       vibrationAmplitude(mode), keepSampling);
            // The hardware window takes about 1.3 s, so the timestamp is taken
            // after acquisition to describe the data rather than the request.
            frame.timestampMs = millis();
            if ((xEventGroupGetBits(networkEvents) & STOP_SAMPLING_BIT) == 0)
                sendTracked(QueueId::Raw, &frame, pdMS_TO_TICKS(100));
        }
        // Hardware samples are paced by the accelerometer ODR/FIFO; this is the frame cadence.
        vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

void featureTask(void *)
{
    TemperatureAverage average;
    SensorFrame raw{};
    for (;;)
    {
        markHeartbeat(TaskId::Feature);
        if (xQueueReceive(rawQueue, &raw, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            VibrationFeatures vibration{};
            if (raw.vibrationValid)
                analyzeVibration(raw.vibration, VIBRATION_SAMPLES, VIBRATION_SAMPLE_RATE_HZ, vibration);
            FeatureFrame feature{};
            static_cast<FeatureValues &>(feature) =
                calculateFeatures(raw.temperature, raw.humidity, raw.dhtValid, vibration, average);
            feature.timestampMs = raw.timestampMs;
            feature.dominantFrequencyHz = vibration.dominantFrequencyHz;
            logPrintf("[FEATURE] source=%s Fs=%.1fHz N=%u df=%.5fHz peak=%.3fHz vibValid=%d\n",
                 vibrationSourceName(), VIBRATION_SAMPLE_RATE_HZ,
                 static_cast<unsigned>(VIBRATION_SAMPLES),
                 VIBRATION_SAMPLE_RATE_HZ / VIBRATION_SAMPLES,
                 feature.dominantFrequencyHz, vibration.valid);
            sendTracked(QueueId::Feature, &feature, pdMS_TO_TICKS(100));
        }
    }
}

void controlTask(void *)
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    ledcAttach(BUZZER_PIN, BUZZER_TONE_HZ, 8);
    StateMachine machine(millis());
    FeatureFrame frame{};
    uint32_t sequence = 0;
    uint32_t lastOverflowTotal = queueOverflowTotal();
    for (;;)
    {
        markHeartbeat(TaskId::Control);
        handleModeButton();
        updateAlarmOutputs(machine);
        const EventBits_t events = xEventGroupGetBits(networkEvents);
        if (events & STOP_SAMPLING_BIT)
        {
            xEventGroupSetBits(networkEvents, CONTROL_QUIESCED_BIT);
            vTaskDelay(pdMS_TO_TICKS(25));
            continue;
        }
        if (events & CONTROL_QUIESCED_BIT)
            xEventGroupClearBits(networkEvents, CONTROL_QUIESCED_BIT);

        if (xQueueReceive(featureQueue, &frame, pdMS_TO_TICKS(25)) != pdTRUE) continue;
        const DeviceState previous = machine.state;
        if (machine.update(frame, millis()))
            logPrintf("[STATE] %s -> %s\n", stateToString(previous), stateToString(machine.state));
        const bool online = xEventGroupGetBits(networkEvents) & MQTT_CONNECTED_BIT;
        // Any drop since the previous frame marks this interval, so the cloud can
        // see a loss window instead of only a serial counter nobody reads.
        const uint32_t overflowTotal = queueOverflowTotal();
        const bool dataLoss = overflowTotal != lastOverflowTotal;
        lastOverflowTotal = overflowTotal;
        TelemetryRecord record{};
        record.bootId = bootId;
        record.sequence = ++sequence;
        record.timestampMs = frame.timestampMs;
        record.temperature = frame.temperature;
        record.humidity = frame.humidity;
        record.vibrationRms = frame.vibrationRms;
        record.vibrationPeakToPeak = frame.vibrationPeakToPeak;
        record.dominantFrequencyHz = frame.dominantFrequencyHz;
        record.alarmCode = composeAlarms(machine.alarms, frame.dhtValid, frame.vibrationValid, online, dataLoss);
        record.state = machine.state;
        record.sensorValid = frame.valid;
        logPrintf("[DATA] boot=%lu seq=%lu temp=%.2f hum=%.2f rms=%.3f p2p=%.3f hz=%.2f state=%s alarm=0x%03x valid=%u\n",
             static_cast<unsigned long>(record.bootId), static_cast<unsigned long>(record.sequence),
             record.temperature, record.humidity, record.vibrationRms, record.vibrationPeakToPeak,
             record.dominantFrequencyHz, stateToString(record.state),
             static_cast<unsigned>(record.alarmCode), static_cast<unsigned>(record.sensorValid));
        sendTracked(QueueId::Telemetry, &record, pdMS_TO_TICKS(100));
    }
}
