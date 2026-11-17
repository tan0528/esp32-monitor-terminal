#pragma once

#include <Arduino.h>
#include "monitor_model.h"
#include "logging.h"
#include <atomic>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>

constexpr uint8_t DHT_PIN = 4;
constexpr uint8_t POT_PIN = 34;
constexpr uint8_t LED_PIN = 2;
constexpr uint8_t BUZZER_PIN = 25;
constexpr uint8_t MODE_BUTTON_PIN = 26;
constexpr uint8_t NETWORK_BUTTON_PIN = 27;
constexpr uint8_t RESET_BUTTON_PIN = 33;

constexpr uint32_t CACHE_CAPACITY = 64;
constexpr EventBits_t WIFI_CONNECTED_BIT = BIT0;
constexpr EventBits_t MQTT_CONNECTED_BIT = BIT1;
constexpr EventBits_t FORCED_OFFLINE_BIT = BIT2;
constexpr EventBits_t STORAGE_READY_BIT = BIT3;
constexpr EventBits_t STOP_SAMPLING_BIT = BIT4;
constexpr EventBits_t CONTROL_QUIESCED_BIT = BIT5;
constexpr EventBits_t STORAGE_USABLE_BIT = BIT6;
constexpr uint32_t NETWORK_BUTTON_EVENT = BIT0;
constexpr uint32_t RESET_BUTTON_EVENT = BIT1;

enum class TaskId : uint8_t { Sensor, Feature, Control, Communication, Storage, Monitor, Count };
enum class QueueId : uint8_t { Raw, Feature, Telemetry, StorageCommand, StorageReply, Count };

struct TelemetryRecord
{
    uint32_t bootId;
    uint32_t sequence;
    uint32_t timestampMs;
    float temperature;
    float humidity;
    float vibrationRms;
    float vibrationPeakToPeak;
    float dominantFrequencyHz;
    uint16_t alarmCode;
    DeviceState state;
    uint8_t sensorValid;
};

static_assert(sizeof(TelemetryRecord) == 36, "Update the binary cache version for layout changes");

enum class StorageOp : uint8_t { Append, Peek, Pop };

struct StorageCommand
{
    uint32_t id;
    StorageOp op;
    TelemetryRecord record;
};

struct StorageReply
{
    uint32_t id;
    bool success;
    bool hasRecord;
    // A Pop that matched an already-removed record succeeds without removing again.
    bool removed;
    uint32_t count;
    TelemetryRecord record;
};

extern QueueHandle_t rawQueue, featureQueue, telemetryQueue, storageQueue, storageReplyQueue;
extern EventGroupHandle_t networkEvents;
extern uint32_t bootId;
extern std::atomic<uint32_t> cacheCount, cacheOverwrites, cacheDropped, storageErrors;
extern std::atomic<uint32_t> wifiAttempts, mqttAttempts, publishFailures, replayedRecords;

const char *stateToString(DeviceState state);
void markHeartbeat(TaskId task);
bool sendTracked(QueueId queue, const void *item, TickType_t timeout);
bool createPipelineQueues();

// Serialized log sink. A line that cannot take the lock within the timeout is
// dropped and counted, so a long dump cannot stall a real-time caller.
uint32_t droppedLogLines();

// Frames dropped by the sampling and feature queues since boot.
uint32_t queueOverflowTotal();

void sensorTask(void *);
void featureTask(void *);
void controlTask(void *);
void communicationTask(void *);
void storageTask(void *);
