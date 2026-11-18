#pragma once

#include <stdint.h>
#include "sample_metadata.h"

struct BatteryMsg {
    SampleMetadata sample;
    float adc_voltage_v = 0.0f;
    float voltage_v = 0.0f;
    uint8_t percentage = 0;
    int raw = 0;
    bool valid = false;
};
