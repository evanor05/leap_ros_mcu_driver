#include "motion_authority.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <cassert>

namespace {
SemaphoreHandle_t mutex = nullptr;
motion_safety::Core core;
MotionMsg pending;
bool has_pending = false;
bool reset_pending = false;
int64_t last_inputs_us = 0;
MotionOutputSink sink = nullptr;

struct Lock {
    Lock() {
        assert(mutex);
        xSemaphoreTake(mutex, portMAX_DELAY);
    }
    ~Lock() { xSemaphoreGive(mutex); }
};

void Brake() { if (sink) sink(0, 0); }
void Clear() {
    has_pending = false;
    Brake();
}
void CheckDeadline() {
    const auto before = core.status.generation;
    const auto now = esp_timer_get_time();
    if (last_inputs_us > 0 && now - last_inputs_us > 40000) {
        core.Inputs(core.status.active_faults | motion_safety::Timing,
                    core.stopped, core.inputs_ready);
    }
    core.Tick(now);
    if (before != core.status.generation) Clear();
}
}  // namespace

void motion_authority_init(MotionOutputSink output) {
    assert(!mutex);
    mutex = xSemaphoreCreateMutex();
    assert(mutex);
    sink = output;
    Brake();
}

motion_safety::Snapshot motion_status() {
    Lock lock;
    return core.status;
}

bool motion_enable(uint8_t source) {
    Lock lock;
    if (reset_pending) return false;
    const auto before = core.status.generation;
    const bool accepted = core.Enable(source, esp_timer_get_time());
    if (before != core.status.generation) Clear();
    return accepted;
}

bool motion_reset() {
    Lock lock;
    const bool accepted = core.Reset();
    if (accepted) Clear();
    return accepted;
}

bool motion_arm(uint8_t source) {
    Lock lock;
    if (reset_pending) return false;
    const auto before = core.status.generation;
    const bool accepted = core.Arm(source, esp_timer_get_time());
    if (before != core.status.generation) Clear();
    return accepted;
}

void motion_estop() {
    Lock lock;
    core.Stop();
    Clear();
}

void motion_disarm() {
    Lock lock;
    core.Disarm();
    Clear();
}

void motion_release(uint8_t source) {
    Lock lock;
    if (core.status.state == motion_safety::State::Armed &&
        source != MOTION_SRC_NONE && core.status.owner == source) {
        core.Disarm();
        Clear();
    }
}

void motion_link_lost(uint8_t source) {
    Lock lock;
    if (source != MOTION_SRC_NONE && core.status.owner == source) {
        core.Disarm(motion_safety::Reason::SourceLost);
        Clear();
    }
}

void motion_link_alive(uint8_t source) {
    Lock lock;
    core.Link(source, esp_timer_get_time());
}

bool motion_submit(MotionMsg command) {
    Lock lock;
    command.received_us = esp_timer_get_time();
    command.authority_generation = core.status.generation;
    const auto before = core.status.generation;
    // An acknowledged action cannot be overwritten by a stream command.
    if (has_pending && (pending.control_mode == 1 || pending.control_mode == 2)) return false;
    if (!core.Accept(command, command.received_us)) {
        if (before != core.status.generation) Clear();
        return false;
    }
    pending = command;
    has_pending = true;
    return true;
}

bool motion_take(MotionMsg* command) {
    Lock lock;
    if (!has_pending) return false;
    has_pending = false;
    if (!core.Allowed(pending.authority_generation, esp_timer_get_time())) return false;
    *command = pending;
    return true;
}

void motion_inputs(uint32_t faults, bool stopped, bool ready) {
    Lock lock;
    const auto before = core.status.generation;
    last_inputs_us = esp_timer_get_time();
    core.Inputs(faults, stopped, ready);
    if (before != core.status.generation) Clear();
}

void motion_tick() {
    Lock lock;
    CheckDeadline();
}

void motion_complete() {
    Lock lock;
    if (core.status.state == motion_safety::State::Armed && core.status.busy) {
        core.Disarm(motion_safety::Reason::Completed);
        Clear();
    }
}

bool motion_request_odometry_reset() {
    Lock lock;
    if (core.status.state != motion_safety::State::Disarmed || !core.stopped) return false;
    reset_pending = true;
    return true;
}

bool motion_take_odometry_reset() {
    Lock lock;
    if (!reset_pending) return false;
    reset_pending = false;
    return core.status.state == motion_safety::State::Disarmed && core.stopped;
}

bool motion_commit(void* generation, int16_t left, int16_t right) {
    Lock lock;
    CheckDeadline();
    const bool allowed = core.Allowed(*static_cast<uint64_t*>(generation), esp_timer_get_time());
    if (sink) sink(allowed ? left : 0, allowed ? right : 0);
    return allowed;
}

void motion_watchdog_task(void*) {
    TickType_t wake = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
        motion_tick();
    }
}
