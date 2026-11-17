#include "terminal.h"
#include "cache_ring.h"

#include <LittleFS.h>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{
constexpr char MOUNT_PATH[] = "/littlefs";
constexpr char DATA_PATH[] = "/littlefs/offline.bin";
constexpr char META_PATH[] = "/littlefs/offline.meta";
constexpr char META_TEMP_PATH[] = "/littlefs/offline.meta.tmp";
constexpr char LEGACY_PATH[] = "/littlefs/offline.jsonl";
constexpr uint32_t CACHE_MAGIC = 0x4d4f4e52; // MONR
// Version 2 adds the dominant frequency to each record.
constexpr uint16_t CACHE_VERSION = 2;
// One slot beyond the capacity keeps a full-ring append off the oldest record.
constexpr uint32_t SLOT_COUNT = cache_ring::SLOTS;
static_assert(SLOT_COUNT == CACHE_CAPACITY + 1, "The ring geometry must match the configured capacity");
static_assert(cache_ring::CAPACITY == CACHE_CAPACITY, "The ring geometry must match the configured capacity");

struct CacheMeta
{
    uint32_t magic = CACHE_MAGIC;
    uint16_t version = CACHE_VERSION;
    uint16_t recordSize = sizeof(TelemetryRecord);
    uint32_t head = 0;
    uint32_t count = 0;
    uint32_t overwritten = 0;
    uint32_t lastBootId = 0;
    uint32_t lastSequence = 0;
};

static_assert(sizeof(CacheMeta) == 28, "Update the cache version if the metadata layout changes");

uint32_t tail(const CacheMeta &meta)
{
    return cache_ring::tail(meta.head, meta.count);
}

void advanceAppend(CacheMeta &meta, const TelemetryRecord &record)
{
    bool overwritten = false;
    cache_ring::append(meta.head, meta.count, overwritten);
    if (overwritten)
        ++meta.overwritten;
    meta.lastBootId = record.bootId;
    meta.lastSequence = record.sequence;
}

bool commitMeta(const CacheMeta &meta)
{
    const int fd = open(META_TEMP_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd < 0)
        return false;

    const bool synced = write(fd, &meta, sizeof(meta)) == sizeof(meta) && fsync(fd) == 0;
    const bool closed = close(fd) == 0;
    // LittleFS atomically replaces the old metadata via rename within the same partition.
    return synced && closed && std::rename(META_TEMP_PATH, META_PATH) == 0;
}

bool readSlot(int fd, uint32_t slot, TelemetryRecord &record)
{
    const auto offset = static_cast<off_t>(slot * sizeof(record));
    return lseek(fd, offset, SEEK_SET) == offset && read(fd, &record, sizeof(record)) == sizeof(record) &&
           record.sequence != 0 && record.sensorValid <= 1 && record.state <= DeviceState::Recovery;
}

// A slot that fails semantic validation would block Peek and Pop forever, which
// stops cached replay and live publishing at the same time. Advancing past it is
// the only recovery the ring allows, so the frame is counted as dropped instead.
bool dropCorruptHead(CacheMeta &meta)
{
    if (meta.count == 0)
        return false;
    CacheMeta next = meta;
    cache_ring::pop(next.head, next.count);
    if (!commitMeta(next))
        return false;
    meta = next;
    ++cacheDropped;
    logPrintf("[STORAGE] Unreadable slot dropped; remaining=%u\n", unsigned(meta.count));
    return true;
}

bool appendRecord(CacheMeta &meta, const TelemetryRecord &record)
{
    if (record.sequence == 0)
        return false;
    // A response can time out after the metadata already committed.
    if (meta.lastBootId == record.bootId && meta.lastSequence == record.sequence)
        return true;

    const int fd = open(DATA_PATH, O_WRONLY);
    if (fd < 0)
        return false;
    const auto offset = static_cast<off_t>(tail(meta) * sizeof(record));
    const bool synced = lseek(fd, offset, SEEK_SET) == offset &&
                        write(fd, &record, sizeof(record)) == sizeof(record) && fsync(fd) == 0;
    const bool closed = close(fd) == 0;
    if (!synced || !closed)
        return false;

    CacheMeta next = meta;
    advanceAppend(next, record);
    if (!commitMeta(next))
        return false;
    meta = next;
    return true;
}

bool peekRecord(CacheMeta &meta, StorageReply &reply)
{
    reply.hasRecord = false;
    for (uint32_t attempts = 0; attempts <= CACHE_CAPACITY && meta.count != 0; ++attempts)
    {
        const int fd = open(DATA_PATH, O_RDONLY);
        if (fd < 0)
            return false;
        reply.hasRecord = readSlot(fd, meta.head, reply.record);
        close(fd);
        if (reply.hasRecord)
            return true;
        if (!dropCorruptHead(meta))
            return false;
    }
    return true;
}

bool popRecord(CacheMeta &meta, const TelemetryRecord &expected, bool &removed)
{
    removed = false;
    if (expected.sequence == 0)
        return false;
    if (meta.count == 0)
        return true;

    const int fd = open(DATA_PATH, O_RDONLY);
    if (fd < 0)
        return false;
    uint32_t index = 0;
    bool readable = true;
    for (; index < meta.count; ++index)
    {
        TelemetryRecord record = {};
        readable = readSlot(fd, (meta.head + index) % SLOT_COUNT, record);
        if (!readable || (record.bootId == expected.bootId && record.sequence == expected.sequence))
            break;
    }
    close(fd);
    if (!readable)
    {
        // An unreadable head must be skipped or the queue never drains. An
        // unreadable middle slot cannot be removed without rewriting the ring, so
        // it is left for the overwrite path and the caller moves on instead of
        // retrying the same search forever.
        if (index != 0)
        {
            logPrintf("[STORAGE] Pop skipped past an unreadable slot at index %u\n", unsigned(index));
            return true;
        }
        return dropCorruptHead(meta);
    }
    // An absent ID was already popped or overwritten. An ID behind the head
    // must wait its turn. Neither case may remove the current head.
    if (index != 0)
        return index == meta.count;

    CacheMeta next = meta;
    cache_ring::pop(next.head, next.count);
    if (!commitMeta(next))
        return false;
    meta = next;
    removed = true;
    return true;
}

bool parseLegacyLine(const char *line, TelemetryRecord &record)
{
    char stateName[9] = {};
    int potValue = 0;
    int end = 0;
    const bool valid = std::sscanf(line, "{\"temperature\":%f,\"humidity\":%f,\"potValue\":%d,"
                                        "\"state\":\"%8[A-Z]\",\"valid\":true}%n",
                                  &record.temperature, &record.humidity, &potValue, stateName, &end) == 4 &&
                       end != 0 && line[end] == '\0';
    if (valid)
    {
        if (!std::isfinite(record.temperature) || !std::isfinite(record.humidity))
            return false;
    }
    else
    {
        end = 0;
        if (std::sscanf(line, "{\"temperature\":null,\"humidity\":null,\"potValue\":%d,"
                              "\"state\":\"%8[A-Z]\",\"valid\":false}%n",
                        &potValue, stateName, &end) != 2 || end == 0 || line[end] != '\0')
            return false;
        record.temperature = 0;
        record.humidity = 0;
    }

    for (uint8_t i = 0; i <= static_cast<uint8_t>(DeviceState::Recovery); ++i)
    {
        const auto state = static_cast<DeviceState>(i);
        if (std::strcmp(stateName, stateToString(state)) == 0)
        {
            record.state = state;
            record.sensorValid = valid;
            // bootId 0 marks legacy records; this field then contains the old ADC value.
            record.vibrationRms = static_cast<float>(potValue);
            return true;
        }
    }
    return false;
}

bool importLegacy(CacheMeta &meta, TelemetryRecord (&slots)[SLOT_COUNT], uint32_t &skipped)
{
    FILE *file = std::fopen(LEGACY_PATH, "r");
    if (file == nullptr)
        return errno == ENOENT;

    char line[194]; // Previous payload buffer was 192 bytes, plus a CRLF terminator.
    uint32_t lineNumber = 0;
    while (std::fgets(line, sizeof(line), file) != nullptr)
    {
        ++lineNumber;
        markHeartbeat(TaskId::Storage);
        if (std::strchr(line, '\n') == nullptr && !std::feof(file))
        {
            int next;
            do
            {
                next = std::fgetc(file);
            } while (next != '\n' && next != EOF);
            ++skipped;
            continue;
        }
        line[std::strcspn(line, "\r\n")] = '\0';
        if (line[0] == '\0')
            continue;

        TelemetryRecord record = {};
        if (!parseLegacyLine(line, record))
        {
            ++skipped;
            continue;
        }
        record.sequence = lineNumber;
        slots[tail(meta)] = record;
        advanceAppend(meta, record);
    }
    const bool readable = std::ferror(file) == 0;
    const bool closed = std::fclose(file) == 0;
    return readable && closed;
}

bool initializeCache(CacheMeta &meta)
{
    TelemetryRecord slots[SLOT_COUNT] = {};
    uint32_t skipped = 0;
    if (!importLegacy(meta, slots, skipped))
        return false;

    const int fd = open(DATA_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd < 0)
        return false;
    const bool synced = write(fd, slots, sizeof(slots)) == sizeof(slots) && fsync(fd) == 0;
    const bool closed = close(fd) == 0;
    if (!synced || !closed || !commitMeta(meta))
        return false;

    // Keep the legacy source for inspection, including lines outside the new capacity.
    if (meta.count != 0 || skipped != 0)
        logPrintf("[STORAGE] JSONL imported=%u overwritten=%u skipped=%u (source retained)\n",
             unsigned(meta.count), unsigned(meta.overwritten), unsigned(skipped));
    return true;
}

// A stale on-disk layout is rebuilt rather than treated as a mount failure, so a
// firmware update that changes the record size keeps the cache usable.
bool resetCache(CacheMeta &meta)
{
    const int fd = open(DATA_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd < 0)
        return false;
    constexpr size_t CHUNK = 256;
    const uint8_t zeros[CHUNK] = {};
    const size_t total = SLOT_COUNT * sizeof(TelemetryRecord);
    bool writtenAll = true;
    for (size_t written = 0; written < total;)
    {
        const size_t chunk = (total - written) < CHUNK ? (total - written) : CHUNK;
        if (write(fd, zeros, chunk) != static_cast<ssize_t>(chunk))
        {
            writtenAll = false;
            break;
        }
        written += chunk;
    }
    const bool synced = writtenAll && fsync(fd) == 0;
    const bool closed = close(fd) == 0;
    meta = CacheMeta{};
    return synced && closed && commitMeta(meta);
}

bool loadCache(CacheMeta &meta)
{
    const int fd = open(META_PATH, O_RDONLY);
    if (fd < 0)
        return errno == ENOENT && initializeCache(meta);

    char extra;
    const bool readMeta = read(fd, &meta, sizeof(meta)) == sizeof(meta) && read(fd, &extra, 1) == 0;
    close(fd);
    if (!readMeta || meta.magic != CACHE_MAGIC)
        return false;
    if (meta.version != CACHE_VERSION || meta.recordSize != sizeof(TelemetryRecord))
    {
        logPrintf("[STORAGE] Cache v%u/%u B is stale, rebuilding as v%u/%u B\n",
             unsigned(meta.version), unsigned(meta.recordSize),
             unsigned(CACHE_VERSION), unsigned(sizeof(TelemetryRecord)));
        return resetCache(meta);
    }
    struct stat dataInfo = {};
    return meta.head < SLOT_COUNT && meta.count <= CACHE_CAPACITY && stat(DATA_PATH, &dataInfo) == 0 &&
           dataInfo.st_size == static_cast<off_t>(SLOT_COUNT * sizeof(TelemetryRecord));
}
}

void storageTask(void *)
{
    CacheMeta meta;
    markHeartbeat(TaskId::Storage);
    const bool mounted = LittleFS.begin(false, MOUNT_PATH);
    const bool ready = mounted && loadCache(meta);
    if (ready)
    {
        cacheCount.store(meta.count);
        cacheOverwrites.store(meta.overwritten);
        logPrintf("[STORAGE] LittleFS ready, recovered=%u/%u overwritten=%u\n",
             unsigned(meta.count), unsigned(CACHE_CAPACITY), unsigned(meta.overwritten));
    }
    else
    {
        storageErrors.fetch_add(1);
        if (mounted)
            logPrintf("[STORAGE] Cache initialization or validation failed\n");
        else
            logPrintf("[STORAGE] LittleFS mount failed, check partition and image\n");
    }
    xEventGroupSetBits(networkEvents, STORAGE_READY_BIT | (ready ? STORAGE_USABLE_BIT : 0));

    while (true)
    {
        StorageCommand command = {};
        if (xQueueReceive(storageQueue, &command, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            StorageReply reply = {};
            reply.id = command.id;
            if (ready)
            {
                switch (command.op)
                {
                case StorageOp::Append:
                    reply.success = appendRecord(meta, command.record);
                    break;
                case StorageOp::Peek:
                    reply.success = peekRecord(meta, reply);
                    break;
                case StorageOp::Pop:
                    reply.success = popRecord(meta, command.record, reply.removed);
                    break;
                }
                reply.count = meta.count;
                cacheCount.store(meta.count);
                cacheOverwrites.store(meta.overwritten);
            }
            if (!reply.success)
                storageErrors.fetch_add(1);
            sendTracked(QueueId::StorageReply, &reply, pdMS_TO_TICKS(100));
        }
        markHeartbeat(TaskId::Storage);
    }
}
