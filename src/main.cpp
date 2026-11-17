#include "terminal.h"

#include <Preferences.h>
#include "liveness.h"
#include <cstdarg>
#include <esp_idf_version.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/semphr.h>
#include <inttypes.h>

QueueHandle_t rawQueue, featureQueue, telemetryQueue, storageQueue, storageReplyQueue;
EventGroupHandle_t networkEvents;
uint32_t bootId;
std::atomic<uint32_t> cacheCount{0}, cacheOverwrites{0}, cacheDropped{0}, storageErrors{0};
std::atomic<uint32_t> wifiAttempts{0}, mqttAttempts{0}, publishFailures{0}, replayedRecords{0};

namespace
{
constexpr size_t TASK_COUNT = static_cast<size_t>(TaskId::Count);
constexpr size_t QUEUE_COUNT = static_cast<size_t>(QueueId::Count);
constexpr uint32_t LOG_LOCK_TIMEOUT_MS = 10;
TaskHandle_t tasks[TASK_COUNT] = {};
QueueHandle_t queues[QUEUE_COUNT] = {};
std::atomic<uint32_t> heartbeats[TASK_COUNT]{};
std::atomic<uint32_t> queuePeaks[QUEUE_COUNT]{};
std::atomic<uint32_t> queueOverflows[QUEUE_COUNT]{};
SemaphoreHandle_t logMutex = nullptr;
std::atomic<uint32_t> droppedLogs{0};
#if defined(TERMINAL_ENABLE_FAULT_INJECTION) && TERMINAL_ENABLE_FAULT_INJECTION
std::atomic<int> stalledTask{-1};
#endif

struct TaskConfig
{
    const char *name;
    TaskFunction_t entry;
    uint32_t stackBytes;
    UBaseType_t priority;
    BaseType_t core;
};

void monitorTask(void *);
const TaskConfig taskConfigs[] = {
    {"SensorTask", sensorTask, 6144, 3, 1},
    {"FeatureTask", featureTask, 8192, 3, 1},
    {"ControlTask", controlTask, 4096, 4, 1},
    {"CommunicationTask", communicationTask, 6144, 2, 0},
    {"StorageTask", storageTask, 8192, 2, 1},
    {"MonitorTask", monitorTask, 4096, 1, 0},
};
const char *const queueNames[] = {"raw", "feature", "telemetry", "storageCmd", "storageReply"};
static_assert(sizeof(taskConfigs) / sizeof(taskConfigs[0]) == TASK_COUNT, "Task monitor table mismatch");

// A reset triggered by the task watchdog destroys the only record of which task
// was late. The mask is written to NVS when a task first goes late, and reported
// on the next boot so a panic reset can be attributed to a specific task.
void persistStaleMask(uint32_t mask)
{
    Preferences prefs;
    if (!prefs.begin("monitor", false))
        return;
    const uint32_t stored = prefs.getUInt("wd_mask", 0);
    const uint32_t merged = stored | mask;
    if (merged != stored)
        prefs.putUInt("wd_mask", merged);
    prefs.end();
}

void reportLastStaleMask()
{
    Preferences prefs;
    if (!prefs.begin("monitor", false))
        return;
    const uint32_t mask = prefs.getUInt("wd_mask", 0);
    if (mask != 0)
    {
        logPrintf("[BOOT] previous run had late heartbeats mask=0x%02" PRIX32 "\n", mask);
        prefs.putUInt("wd_mask", 0);
    }
    prefs.end();
}

void monitorTask(void *)
{
    HeartbeatMonitor heartbeatMonitor;
    uint32_t reportedStale = 0;
    for (;;)
    {
        markHeartbeat(TaskId::Monitor);
        const uint32_t now = millis();
        const EventBits_t events = xEventGroupGetBits(networkEvents);
        uint32_t observed[static_cast<size_t>(TaskId::Monitor)]{};
        for (size_t i = 0; i < static_cast<size_t>(TaskId::Monitor); ++i)
            observed[i] = heartbeats[i].load();
        const HeartbeatChanges health = heartbeatMonitor.update(now, observed, static_cast<size_t>(TaskId::Monitor));
        if (health.newlyStaleMask != 0)
        {
            logPrintf("[HEALTH] Heartbeat late mask=0x%02" PRIX32 "; individual task watchdogs remain active\n",
                 health.newlyStaleMask);
            const uint32_t next = reportedStale | health.newlyStaleMask;
            if (next != reportedStale)
            {
                reportedStale = next;
                persistStaleMask(reportedStale);
            }
        }
        if (health.recoveredMask != 0)
            logPrintf("[HEALTH] Heartbeat recovered mask=0x%02" PRIX32 "\n", health.recoveredMask);
        logPrintf("[MON] heap=%" PRIu32 " minHeap=%" PRIu32 " cache=%" PRIu32 "/%" PRIu32
             " overwritten=%" PRIu32 " dropped=%" PRIu32 " wifi=%d mqtt=%d forced=%d wifiAttempts=%" PRIu32
             " mqttAttempts=%" PRIu32 " sendFailures=%" PRIu32 " replayed=%" PRIu32
             " storageErrors=%" PRIu32 " logDrops=%" PRIu32 " blocked=0x%02" PRIX32 "\n",
             ESP.getFreeHeap(), ESP.getMinFreeHeap(), cacheCount.load(), CACHE_CAPACITY,
             cacheOverwrites.load(), cacheDropped.load(), (events & WIFI_CONNECTED_BIT) != 0,
             (events & MQTT_CONNECTED_BIT) != 0, (events & FORCED_OFFLINE_BIT) != 0,
             wifiAttempts.load(), mqttAttempts.load(), publishFailures.load(),
             replayedRecords.load(), storageErrors.load(), droppedLogs.load(), health.activeMask);
        for (size_t i = 0; i < TASK_COUNT; ++i)
        {
            const uint32_t age = now - heartbeats[i].load();
            logPrintf("[TASK] %s core=%d heartbeatMs=%" PRIu32 " stackMinBytes=%u\n",
                 taskConfigs[i].name, static_cast<int>(taskConfigs[i].core),
                 static_cast<int32_t>(age) < 0 ? 0U : age,
                 static_cast<unsigned>(uxTaskGetStackHighWaterMark(tasks[i])));
        }
        for (size_t i = 0; i < QUEUE_COUNT; ++i)
        {
            logPrintf("[QUEUE] %s used=%u peak=%" PRIu32 " overflow=%" PRIu32 "\n", queueNames[i],
                 static_cast<unsigned>(uxQueueMessagesWaiting(queues[i])),
                 queuePeaks[i].load(), queueOverflows[i].load());
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// A task waits here until setup has populated every task handle, including the monitor's.
void startTask(void *arg)
{
    auto &config = *static_cast<const TaskConfig *>(arg);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));
    config.entry(nullptr);
}

void configureWatchdog()
{
#if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t config{};
    config.timeout_ms = TASK_WATCHDOG_TIMEOUT_MS;
    config.idle_core_mask = (1U << portNUM_PROCESSORS) - 1U;
    config.trigger_panic = true;
    const esp_err_t initialized = esp_task_wdt_init(&config);
    if (initialized == ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&config));
    else
        ESP_ERROR_CHECK(initialized);
#else
    ESP_ERROR_CHECK(esp_task_wdt_init(TASK_WATCHDOG_TIMEOUT_MS / 1000, true));
#endif
}

uint32_t allocateBootId()
{
    Preferences prefs;
    if (!prefs.begin("monitor", false))
        return 0;
    const uint32_t next = prefs.getUInt("boot_id", 0) + 1;
    const bool written = next != 0 && prefs.putUInt("boot_id", next) == sizeof(next);
    prefs.end();
    if (!written)
        return 0;

    // A write can be lost if power fails before the NVS page commits. Reading the
    // value back with a separate read-only handle proves it reached the flash,
    // otherwise this boot would reuse the previous bootId and collide on the
    // (bootId, sequence) primary key that the cloud de-duplicates with.
    Preferences verify;
    if (!verify.begin("monitor", true))
        return 0;
    const uint32_t stored = verify.getUInt("boot_id", 0);
    verify.end();
    return stored == next ? next : 0;
}

[[noreturn]] void setupFailed(const char *reason)
{
    logPrintf("[FATAL] %s\n", reason);
    for (;;)
        vTaskDelay(pdMS_TO_TICKS(1000));
}

void IRAM_ATTR modeButtonISR()
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(tasks[static_cast<size_t>(TaskId::Control)], &woken);
    if (woken)
        portYIELD_FROM_ISR();
}

void IRAM_ATTR communicationButtonISR(void *arg)
{
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(tasks[static_cast<size_t>(TaskId::Communication)],
                       static_cast<uint32_t>(reinterpret_cast<uintptr_t>(arg)), eSetBits, &woken);
    if (woken)
        portYIELD_FROM_ISR();
}
}

void logPrintf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    if (logMutex == nullptr || xSemaphoreTake(logMutex, pdMS_TO_TICKS(LOG_LOCK_TIMEOUT_MS)) == pdTRUE)
    {
        Serial.vprintf(format, args);
        if (logMutex != nullptr)
            xSemaphoreGive(logMutex);
    }
    else
    {
        ++droppedLogs;
    }
    va_end(args);
}

uint32_t droppedLogLines()
{
    return droppedLogs.load();
}

uint32_t queueOverflowTotal()
{
    // Storage command and reply drops are retries, not lost measurements.
    return queueOverflows[static_cast<size_t>(QueueId::Raw)].load() +
           queueOverflows[static_cast<size_t>(QueueId::Feature)].load() +
           queueOverflows[static_cast<size_t>(QueueId::Telemetry)].load();
}

const char *stateToString(DeviceState state)
{
    switch (state)
    {
    case DeviceState::Init: return "INIT";
    case DeviceState::Normal: return "NORMAL";
    case DeviceState::Warning: return "WARNING";
    case DeviceState::Fault: return "FAULT";
    case DeviceState::Recovery: return "RECOVERY";
    }
    return "UNKNOWN";
}

void markHeartbeat(TaskId task)
{
#if defined(TERMINAL_ENABLE_FAULT_INJECTION) && TERMINAL_ENABLE_FAULT_INJECTION
    if (stalledTask.load() == static_cast<int>(task))
    {
        // Leave the scheduler and idle tasks running while this task stops feeding.
        for (;;)
            vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
    heartbeats[static_cast<size_t>(task)].store(millis());
    ESP_ERROR_CHECK(esp_task_wdt_reset());
}

bool sendTracked(QueueId queue, const void *item, TickType_t timeout)
{
    const size_t i = static_cast<size_t>(queue);
    if (xQueueSend(queues[i], item, timeout) != pdTRUE)
    {
        ++queueOverflows[i];
        return false;
    }

    // A higher-priority consumer can take the item before this measurement.
    uint32_t waiting = uxQueueMessagesWaiting(queues[i]);
    if (waiting == 0)
        waiting = 1;
    uint32_t peak = queuePeaks[i].load();
    while (waiting > peak && !queuePeaks[i].compare_exchange_weak(peak, waiting))
    {
    }
    return true;
}

void setup()
{
    logMutex = xSemaphoreCreateMutex();
    Serial.begin(115200);
    logPrintf("[BOOT] resetReason=%d\n", static_cast<int>(esp_reset_reason()));
    reportLastStaleMask();
    bootId = allocateBootId();
    if (bootId == 0)
        setupFailed("Could not allocate a verified boot ID in NVS");
    logPrintf("[BOOT] boot=%" PRIu32 "\n", bootId);

    networkEvents = xEventGroupCreate();
    telemetryQueue = xQueueCreate(16, sizeof(TelemetryRecord));
    storageQueue = xQueueCreate(4, sizeof(StorageCommand));
    storageReplyQueue = xQueueCreate(4, sizeof(StorageReply));
    if (!networkEvents || !telemetryQueue || !storageQueue || !storageReplyQueue || !createPipelineQueues())
        setupFailed("Could not create FreeRTOS queues or event group");

    queues[static_cast<size_t>(QueueId::Raw)] = rawQueue;
    queues[static_cast<size_t>(QueueId::Feature)] = featureQueue;
    queues[static_cast<size_t>(QueueId::Telemetry)] = telemetryQueue;
    queues[static_cast<size_t>(QueueId::StorageCommand)] = storageQueue;
    queues[static_cast<size_t>(QueueId::StorageReply)] = storageReplyQueue;

    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(NETWORK_BUTTON_PIN, INPUT_PULLUP);
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
    configureWatchdog();
    for (size_t i = 0; i < TASK_COUNT; ++i)
    {
        auto &config = taskConfigs[i];
        if (xTaskCreatePinnedToCore(startTask, config.name, config.stackBytes,
                                   const_cast<TaskConfig *>(&config), config.priority,
                                   &tasks[i], config.core) != pdPASS)
            setupFailed(config.name);
        heartbeats[i].store(millis());
    }
    for (TaskHandle_t task : tasks)
        xTaskNotifyGive(task);
    logPrintf("[WDT] Six application tasks registered on entry; timeout=%" PRIu32 " ms, panic reset enabled\n",
         TASK_WATCHDOG_TIMEOUT_MS);
#if defined(TERMINAL_ENABLE_FAULT_INJECTION) && TERMINAL_ENABLE_FAULT_INJECTION
    logPrintf("[TEST] Watchdog fault injection enabled; serial 0..5\n");
#endif
    attachInterrupt(digitalPinToInterrupt(MODE_BUTTON_PIN), modeButtonISR, FALLING);
    attachInterruptArg(digitalPinToInterrupt(NETWORK_BUTTON_PIN), communicationButtonISR,
                       reinterpret_cast<void *>(NETWORK_BUTTON_EVENT), FALLING);
    attachInterruptArg(digitalPinToInterrupt(RESET_BUTTON_PIN), communicationButtonISR,
                       reinterpret_cast<void *>(RESET_BUTTON_EVENT), FALLING);
}

void loop()
{
    while (Serial.available() > 0)
    {
        const char command = static_cast<char>(Serial.read());
        if (command == 'r' || command == 'R')
        {
            xTaskNotify(tasks[static_cast<size_t>(TaskId::Communication)], RESET_BUTTON_EVENT, eSetBits);
        }
#if defined(TERMINAL_ENABLE_FAULT_INJECTION) && TERMINAL_ENABLE_FAULT_INJECTION
        if (command >= '0' && command <= '5')
        {
            const int index = command - '0';
            logPrintf("[TEST] Stalling %s at next heartbeat\n", taskConfigs[index].name);
            stalledTask.store(index);
        }
#endif
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}
