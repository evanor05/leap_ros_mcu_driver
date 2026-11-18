#pragma once
#include "msg/imu_msg.h"
#define ESP_OK 0
#define ESP_FAIL -1
#define pdMS_TO_TICKS(v) (v)
struct Globals { struct { void* imu_state=nullptr; } queues; };
Globals& system_globals();
void xQueueOverwrite(void*,const ImuMsg*);
void vTaskDelay(int);
