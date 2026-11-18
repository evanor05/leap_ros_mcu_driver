#pragma once
#include "motion_safety.h"

using MotionOutputSink = void (*)(int16_t left,int16_t right);
void motion_authority_init(MotionOutputSink sink);
motion_safety::Snapshot motion_status();
bool motion_enable(uint8_t source);
bool motion_reset();
bool motion_arm(uint8_t source);
void motion_estop();
void motion_disarm();
void motion_release(uint8_t source);
void motion_link_lost(uint8_t source);
void motion_link_alive(uint8_t source);
bool motion_submit(MotionMsg command);
bool motion_take(MotionMsg* command);
void motion_inputs(uint32_t faults,bool stopped,bool ready);
void motion_tick();
void motion_watchdog_task(void*);
void motion_complete();
bool motion_request_odometry_reset();
bool motion_take_odometry_reset();
bool motion_commit(void* generation,int16_t left,int16_t right);
