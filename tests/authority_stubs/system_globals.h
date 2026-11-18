#pragma once
#include "freertos/FreeRTOS.h"
#include "motion_authority.h"
#include "msg/pid_msg.h"
using QueueHandle_t=void*;
struct Globals {struct {QueueHandle_t imu_state,battery_state,motion_state,speedpid_cmd=nullptr,positionpid_cmd=nullptr;}queues;struct{PidMsg speed_pid,position_pid;}control;};
Globals& system_globals();
int xQueuePeek(QueueHandle_t,void*,int);
inline int xQueueReceive(QueueHandle_t,void*,int){return 0;}
void xQueueOverwrite(QueueHandle_t,const void*);
