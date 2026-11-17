#pragma once

// Shared declaration for firmware modules and host-test logging adapters.
void logPrintf(const char *format, ...) __attribute__((format(printf, 1, 2)));
