#pragma once

#include "monitor_model.h"

#include <cstddef>

// Call from SensorTask only. The hardware backend measures X-axis acceleration in g.
bool beginVibrationSource();
const char *vibrationSourceName();

// Produces exactly VIBRATION_SAMPLES at VIBRATION_SAMPLE_RATE_HZ.
// Synthetic samples describe model time i/Fs and are generated immediately.
// Hardware samples are clocked by the MPU6050 and drained from its FIFO.
// The callback services the caller and cancels acquisition when it returns false.
// On false the caller must discard the entire window, including partial samples.
bool readVibrationWindow(float *samples, size_t count, float amplitude,
                         bool (*keepSampling)() = nullptr);
