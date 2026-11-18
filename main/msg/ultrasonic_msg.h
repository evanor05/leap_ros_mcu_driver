#pragma once
#include "sample_metadata.h"

#pragma pack(push, 1) // 强制 1 字节对齐
// Legacy UDP telemetry payload. Keep this exact layout for the old wire format.
struct UltrasonicMsg {
    float distance_cm = 0.0f;
    bool is_obstacle_detected = false;
};
#pragma pack(pop)     // 恢复默认对齐

static_assert(sizeof(UltrasonicMsg) == 5, "legacy UDP telemetry layout must stay 5 bytes");

// Internal FreeRTOS queue payload: the reading above plus real sampling metadata.
// It is never serialized into the legacy UDP telemetry packet.
struct UltrasonicStateMsg {
    UltrasonicMsg reading;
    SampleMetadata sample;
};
