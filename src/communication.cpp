#include "terminal.h"

#include <WiFi.h>
#include <esp_idf_version.h>
#include <inttypes.h>
#include <mqtt_client.h>

#if !defined(TERMINAL_OFFLINE_ONLY) && __has_include("secrets.h")
#include "secrets.h"
#else
constexpr const char *TB_ACCESS_TOKEN = "";
#endif

#ifndef TB_MQTT_HOST
#define TB_MQTT_HOST "mqtt.eu.thingsboard.cloud"
#endif
#ifndef TB_MQTT_PORT
#define TB_MQTT_PORT 1883
#endif

// "Wokwi-GUEST" only exists inside the simulator. A real board overrides these
// in secrets.h, the same way TB_MQTT_HOST is overridden above.
#ifndef WIFI_SSID
#define WIFI_SSID "Wokwi-GUEST"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

namespace
{
constexpr char MQTT_TOPIC[] = "v1/devices/me/telemetry";
constexpr uint32_t ACK_TIMEOUT_MS = 5000;

bool reached(uint32_t now, uint32_t deadline)
{
    return static_cast<int32_t>(now - deadline) >= 0;
}

struct Backoff
{
    uint32_t deadline = 0;
    uint32_t delayMs = 1000;

    void reset(uint32_t now)
    {
        deadline = now;
        delayMs = 1000;
    }

    void failed(uint32_t now, const char *link)
    {
        deadline = now + delayMs;
        logPrintf("[%s] Retry in %" PRIu32 " ms\n", link, delayMs);
        delayMs = min(delayMs * 2, uint32_t{30000});
    }
};

enum class MqttState : uint8_t { Connecting, Online, Failed };

class TerminalNetwork
{
public:
    void begin()
    {
        WiFi.persistent(false);
        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(false);
        wifiRetry.reset(millis());
        mqttRetry.reset(millis());
        if (TB_ACCESS_TOKEN[0] == '\0')
            logPrintf("[MQTT] Set TB_ACCESS_TOKEN in include/secrets.h to enable publishing\n");
    }

    void pause()
    {
        stopMqtt();
        WiFi.disconnect();
        wifiConnecting = false;
        wifiOnline = false;
        xEventGroupClearBits(networkEvents, WIFI_CONNECTED_BIT);
    }

    void resume(uint32_t now)
    {
        wifiRetry.reset(now);
        mqttRetry.reset(now);
    }

    void poll(uint32_t now)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            if (wifiOnline || (wifiConnecting && reached(now, wifiDeadline)))
            {
                pause();
                wifiRetry.failed(now, "WIFI");
            }
            if (!wifiConnecting && reached(now, wifiRetry.deadline))
            {
                ++wifiAttempts;
                wifiConnecting = true;
                wifiDeadline = now + 8000;
                logPrintf("[WIFI] Connecting\n");
                WiFi.begin(WIFI_SSID, WIFI_PASSWORD, 6);
            }
            return;
        }

        if (!wifiOnline)
        {
            wifiOnline = true;
            wifiConnecting = false;
            resume(now);
            xEventGroupSetBits(networkEvents, WIFI_CONNECTED_BIT);
            logPrintf("[WIFI] Connected, IP=%s\n", WiFi.localIP().toString().c_str());
        }

        if (client)
        {
            const MqttState link = mqttState.load();
            if (link == MqttState::Online && !mqttOnline)
            {
                mqttOnline = true;
                mqttRetry.reset(now);
                xEventGroupSetBits(networkEvents, MQTT_CONNECTED_BIT);
                logPrintf("[MQTT] Connected\n");
            }
            else if (link == MqttState::Failed ||
                     (link == MqttState::Connecting && reached(now, mqttDeadline)))
            {
                failMqtt(now);
            }
        }
        else if (TB_ACCESS_TOKEN[0] != '\0' && reached(now, mqttRetry.deadline))
        {
            startMqtt(now);
        }
    }

    bool online() const { return mqttOnline && mqttState.load() == MqttState::Online; }
    int acknowledgedId() const { return ackId.load(); }

    int publish(const char *payload)
    {
        ackId.store(0);
        return esp_mqtt_client_enqueue(client, MQTT_TOPIC, payload, 0, 1, 0, true);
    }

    void failMqtt(uint32_t now)
    {
        stopMqtt();
        mqttRetry.failed(now, "MQTT");
    }

private:
    esp_mqtt_client_handle_t client = nullptr;
    std::atomic<MqttState> mqttState{MqttState::Connecting};
    std::atomic<int> ackId{0};
    bool wifiConnecting = false;
    bool wifiOnline = false;
    bool mqttOnline = false;
    uint32_t wifiDeadline = 0;
    uint32_t mqttDeadline = 0;
    Backoff wifiRetry;
    Backoff mqttRetry;

    static void onMqttEvent(void *context, esp_event_base_t, int32_t eventId, void *data)
    {
        auto &network = *static_cast<TerminalNetwork *>(context);
        switch (eventId)
        {
        case MQTT_EVENT_CONNECTED:
            network.mqttState.store(MqttState::Online);
            break;
        case MQTT_EVENT_DISCONNECTED:
        case MQTT_EVENT_ERROR:
            network.mqttState.store(MqttState::Failed);
            break;
        case MQTT_EVENT_PUBLISHED:
            network.ackId.store(static_cast<esp_mqtt_event_t *>(data)->msg_id);
            break;
        }
    }

    void startMqtt(uint32_t now)
    {
        ++mqttAttempts;
        logPrintf("[MQTT] Connecting\n");
        esp_mqtt_client_config_t config{};
#if ESP_IDF_VERSION_MAJOR >= 5
        config.broker.address.hostname = TB_MQTT_HOST;
        config.broker.address.port = TB_MQTT_PORT;
        config.broker.address.transport = MQTT_TRANSPORT_OVER_TCP;
        config.credentials.username = TB_ACCESS_TOKEN;
        config.network.disable_auto_reconnect = true;
        config.network.timeout_ms = 2000;
        config.session.keepalive = 15;
#else
        config.host = TB_MQTT_HOST;
        config.port = TB_MQTT_PORT;
        config.transport = MQTT_TRANSPORT_OVER_TCP;
        config.username = TB_ACCESS_TOKEN;
        config.disable_auto_reconnect = true;
        config.network_timeout_ms = 2000;
        config.keepalive = 15;
#endif
        mqttState.store(MqttState::Connecting);
        mqttDeadline = now + 5000;
        client = esp_mqtt_client_init(&config);
        if (!client || esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, onMqttEvent, this) != ESP_OK ||
            esp_mqtt_client_start(client) != ESP_OK)
        {
            failMqtt(now);
        }
    }

    void stopMqtt()
    {
        mqttOnline = false;
        xEventGroupClearBits(networkEvents, MQTT_CONNECTED_BIT);
        if (client)
        {
            esp_mqtt_client_destroy(client);
            client = nullptr;
        }
    }
};

bool storageRequest(StorageOp op, const TelemetryRecord &record, StorageReply &reply)
{
    static uint32_t nextId = 0;
    const StorageCommand command{++nextId, op, record};
    if (!sendTracked(QueueId::StorageCommand, &command, pdMS_TO_TICKS(100)))
        return false;

    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(800);
    for (;;)
    {
        const TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout || xQueueReceive(storageReplyQueue, &reply, timeout - elapsed) != pdTRUE)
        {
            ++storageErrors;
            return false;
        }
        if (reply.id == command.id)
            return reply.success;
    }
}

bool makePayload(const TelemetryRecord &record, bool replayed, char (&payload)[448])
{
    char dhtFields[64] = {};
    char vibrationFields[128] = {};
    // Both sections are omitted rather than published as zero when the matching
    // source failed; ALARM_VIBRATION_INVALID says which source it was.
    if (record.sensorValid)
        snprintf(dhtFields, sizeof(dhtFields), ",\"temperature\":%.3f,\"humidity\":%.3f",
                 record.temperature, record.humidity);
    if (record.bootId == 0)
        snprintf(vibrationFields, sizeof(vibrationFields), ",\"potValue\":%.0f", record.vibrationRms);
    else if ((record.alarmCode & ALARM_VIBRATION_INVALID) == 0)
        // A failed vibration window leaves the fields at zero. Sending that would
        // draw a flat, believable trace, so the fields are omitted instead.
        snprintf(vibrationFields, sizeof(vibrationFields),
                 ",\"vibrationRms\":%.3f,\"vibrationPeakToPeak\":%.3f,\"dominantFrequencyHz\":%.3f",
                 record.vibrationRms, record.vibrationPeakToPeak, record.dominantFrequencyHz);

    const int length = snprintf(payload, sizeof(payload),
        "{\"boot_id\":%" PRIu32 ",\"sequence\":%" PRIu32
        ",\"record_id\":\"%" PRIu32 "-%" PRIu32 "\",\"uptimeMs\":%" PRIu32 ","
        "\"state\":\"%s\",\"alarmCode\":%u,\"valid\":%s,\"replayed\":%s%s%s}",
        record.bootId, record.sequence, record.bootId, record.sequence, record.timestampMs,
        stateToString(record.state), static_cast<unsigned>(record.alarmCode),
        record.sensorValid ? "true" : "false", replayed ? "true" : "false", dhtFields, vibrationFields);
    return length > 0 && static_cast<size_t>(length) < sizeof(payload);
}

bool cacheNext()
{
    TelemetryRecord record;
    if (xQueuePeek(telemetryQueue, &record, 0) != pdTRUE)
        return true;

    StorageReply reply{};
    if (!storageRequest(StorageOp::Append, record, reply))
        return false;
    xQueueReceive(telemetryQueue, &record, 0);
    logPrintf("[CACHE] boot=%" PRIu32 " seq=%" PRIu32 " pending=%" PRIu32 "/%" PRIu32 "\n",
                  record.bootId, record.sequence, reply.count, CACHE_CAPACITY);
    return true;
}

struct InFlight
{
    TelemetryRecord record{};
    int id = 0;
    uint32_t deadline = 0;
    bool replay = false;
    bool acknowledged = false;

    bool start(TerminalNetwork &network, const TelemetryRecord &next, bool fromCache)
    {
        char payload[448];
        if (makePayload(next, fromCache, payload))
            id = network.publish(payload);
        if (id <= 0)
        {
            id = 0;
            ++publishFailures;
            network.failMqtt(millis());
            return false;
        }

        record = next;
        replay = fromCache;
        deadline = millis() + ACK_TIMEOUT_MS;
        return true;
    }

    void settle(TerminalNetwork &network, uint32_t now, bool &cacheLiveFirst)
    {
        if (!id)
            return;
        acknowledged = acknowledged || network.acknowledgedId() == id;
        if (acknowledged)
        {
            if (replay)
            {
                StorageReply reply{};
                if (!storageRequest(StorageOp::Pop, record, reply))
                    return;
                // A Pop retried after a lost reply succeeds without removing the
                // slot again; only an actual removal counts as a replayed frame.
                if (reply.removed)
                    ++replayedRecords;
                logPrintf("[REPLAY] PUBACK boot=%" PRIu32 " seq=%" PRIu32 " remaining=%" PRIu32 "\n",
                     record.bootId, record.sequence, reply.count);
            }
            else
            {
                xQueueReceive(telemetryQueue, &record, 0);
                logPrintf("[MQTT] Live PUBACK boot=%" PRIu32 " seq=%" PRIu32 "\n", record.bootId, record.sequence);
            }
        }
        else if (!network.online() || reached(now, deadline))
        {
            ++publishFailures;
            cacheLiveFirst = !replay;
            if (network.online())
                network.failMqtt(now);
            logPrintf("[MQTT] Unconfirmed boot=%" PRIu32 " seq=%" PRIu32 "; retained for retry\n",
                 record.bootId, record.sequence);
        }
        else
        {
            return;
        }
        *this = {};
    }
};

void restartSafely(TerminalNetwork &network)
{
    logPrintf("[TEST] Software reset requested\n");
    xEventGroupSetBits(networkEvents, STOP_SAMPLING_BIT);
    network.pause();
    const uint32_t deadline = millis() + 5000;
    while (!reached(millis(), deadline))
    {
        markHeartbeat(TaskId::Communication);
        if ((xEventGroupGetBits(networkEvents) & CONTROL_QUIESCED_BIT) &&
            uxQueueMessagesWaiting(telemetryQueue) == 0)
            break;
        cacheNext();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    logPrintf("[TEST] Restarting; cache=%" PRIu32 " queued=%u\n", cacheCount.load(),
                  static_cast<unsigned>(uxQueueMessagesWaiting(telemetryQueue)));
    Serial.flush();
    ESP.restart();
}
}

void communicationTask(void *)
{
    TerminalNetwork network;
    network.begin();
    InFlight flight;
    bool forcedOffline = false;
    bool cacheLiveFirst = false;
    uint32_t lastToggle = millis() - 250;

    for (;;)
    {
        uint32_t buttons = 0;
        xTaskNotifyWait(0, UINT32_MAX, &buttons, pdMS_TO_TICKS(50));
        markHeartbeat(TaskId::Communication);
        const uint32_t now = millis();

        if (buttons & RESET_BUTTON_EVENT)
            restartSafely(network);

        if ((buttons & NETWORK_BUTTON_EVENT) && now - lastToggle >= 250)
        {
            lastToggle = now;
            forcedOffline = !forcedOffline;
            if (forcedOffline)
            {
                xEventGroupSetBits(networkEvents, FORCED_OFFLINE_BIT);
                network.pause();
                logPrintf("[TEST] Forced offline\n");
            }
            else
            {
                xEventGroupClearBits(networkEvents, FORCED_OFFLINE_BIT);
                network.resume(now);
                logPrintf("[TEST] Network enabled\n");
            }
        }

        if (!forcedOffline)
            network.poll(now);
        const EventBits_t storage = xEventGroupGetBits(networkEvents);
        if (!(storage & STORAGE_READY_BIT))
            continue;

        flight.settle(network, now, cacheLiveFirst);
        if (flight.id && !flight.replay)
            continue; // The live frame stays at the head of telemetryQueue until PUBACK or caching.

        if ((storage & STORAGE_USABLE_BIT) &&
            (!network.online() || cacheLiveFirst || flight.replay || cacheCount.load() != 0))
        {
            cacheLiveFirst = !cacheNext();
            if (cacheLiveFirst)
            {
                // When writes fail, still try to drain a readable backlog before going live.
                if (!network.online() || cacheCount.load() == 0)
                {
                    vTaskDelay(pdMS_TO_TICKS(200));
                    continue;
                }
            }
        }

        if (!network.online() || flight.id)
            continue;

        if (cacheCount.load() != 0)
        {
            StorageReply oldest{};
            if (storageRequest(StorageOp::Peek, {}, oldest) && oldest.hasRecord)
                flight.start(network, oldest.record, true);
        }
        else
        {
            TelemetryRecord next;
            if (xQueuePeek(telemetryQueue, &next, 0) == pdTRUE && !flight.start(network, next, false))
                cacheLiveFirst = true;
        }
    }
}
