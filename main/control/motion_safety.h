#pragma once
#include <cmath>
#include <initializer_list>
#include <stdint.h>
#include "msg/motion_msg.h"
#include "motion_config.h"

namespace motion_safety {
enum class State : uint8_t { Disarmed, Armed, Estop, Fault };
enum class Reason : uint8_t { Boot, Enabled, Estop, Released, Timeout, SourceLost,
    Timing, BatteryLow, BatteryStale, ImuStale, InvalidCommand, Completed, Reset };
enum Fault : uint32_t { Timing = 1, BatteryLow = 2, BatteryStale = 4, ImuStale = 8 };
struct Snapshot {
    State state = State::Disarmed;
    Reason reason = Reason::Boot;
    uint8_t owner = MOTION_SRC_NONE;
    uint64_t generation = 1;
    uint32_t faults = 0, active_faults = 0;
    bool busy = false;
};

inline bool ValidCommand(const MotionMsg& c) {
    namespace cfg = leap_config::motion;
    for (float v : {c.target_vx,c.target_vy,c.target_wz,c.target_x,c.target_y,c.target_yaw,c.target_motor_v})
        if (!std::isfinite(v)) return false;
    if (c.control_mode == 0)
        return std::abs(c.target_vx)<=cfg::kMaxLinearVelocityMmS &&
            std::abs(c.target_wz)<=cfg::kMaxAngularVelocityRadS && std::abs(c.target_vy)<=.001f;
    if (c.control_mode == 1) return c.action_id > 0 && std::abs(c.target_x)<=100000 &&
            std::abs(c.target_y)<=100000 && std::abs(c.target_yaw)<=3.141593f;
    if (c.control_mode == 2) return c.action_id > 0 && std::abs(c.target_x)<=5000 && std::abs(c.target_yaw)<=180;
    if (c.control_mode == 3) return c.motor_id>=0 && c.motor_id<2 && std::abs(c.target_motor_v)<=cfg::kTrackWidthMm*cfg::kMaxAngularVelocityRadS;
    return c.control_mode == 6 && std::abs(c.target_vx)<=cfg::kMaxLinearVelocityMmS &&
            std::abs(c.target_vy)<=cfg::kMaxLinearVelocityMmS &&
            std::abs((c.target_vy-c.target_vx)/cfg::kTrackWidthMm)<=cfg::kMaxAngularVelocityRadS;
}

// Pure state machine. The platform serializes every transition and motor commit
// with the same priority-inheriting mutex; it never holds an SMP spinlock in LEDC.
class Core {
 public:
    Snapshot status;
    bool stopped = false, inputs_ready = false;
    void Revoke(State state, Reason reason) {
        ++status.generation; status.state=state; status.reason=reason;
        status.owner=MOTION_SRC_NONE; status.busy=false; deadline_=0; have_command_=false;
    }
    void Stop() { Revoke(State::Estop,Reason::Estop); }
    void Disarm(Reason reason=Reason::Released) {
        if(status.state==State::Estop){Revoke(State::Estop,Reason::Estop);return;}
        Revoke(status.faults?State::Fault:State::Disarmed,reason);
    }
    void Inputs(uint32_t active, bool stationary, bool ready) {
        stopped=stationary; inputs_ready=ready; status.active_faults=active;
        if (active) {
            status.faults |= active;
            if (status.state!=State::Fault) {
                Reason why=active & Timing?Reason::Timing:active & BatteryLow?Reason::BatteryLow:
                    active & BatteryStale?Reason::BatteryStale:Reason::ImuStale;
                Revoke(State::Fault,why);
            }
        }
    }
    bool Reset() {
        if (status.active_faults || !inputs_ready || !stopped) return false;
        status.faults=0; Revoke(State::Disarmed,Reason::Reset); return true;
    }
    bool Arm(uint8_t source,int64_t now) {
        Tick(now);
        if (source==MOTION_SRC_NONE || source>MOTION_SRC_GAMEPAD || !inputs_ready || status.faults ||
            status.state==State::Estop || status.state==State::Fault) return false;
        if (status.state==State::Armed) return status.owner==source; // No lease refresh.
        if (!stopped) return false;
        Revoke(State::Armed,Reason::Enabled); status.owner=source; armed_at_=now; deadline_=now+300000;
        return true;
    }
    // Legacy explicit ENABLE may clear an external stop, never a hardware fault.
    bool Enable(uint8_t source,int64_t now) {
        if (status.faults || status.active_faults) return false;
        if (status.state==State::Estop && !Reset()) return false;
        return Arm(source,now);
    }
    bool Accept(const MotionMsg& c,int64_t now) {
        Tick(now);
        if (status.state!=State::Armed || c.source!=status.owner || c.authority_generation!=status.generation ||
            c.received_us<armed_at_ || c.received_us>now || now-c.received_us>300000) return false;
        if (!ValidCommand(c)) { Disarm(Reason::InvalidCommand); return false; }
        const bool action=c.control_mode==1 || c.control_mode==2;
        if (action && (status.busy || c.action_id<=last_action_[c.source][c.control_mode-1])) return false;
        if (!action && status.busy) return false;
        if (action) last_action_[c.source][c.control_mode-1]=c.action_id;
        status.busy=action; have_command_=true;
        deadline_=c.received_us+(action?15000000LL:300000LL);
        link_at_=c.received_us;
        return true;
    }
    void Link(uint8_t source,int64_t now) { if(source==status.owner) link_at_=now; }
    bool Allowed(uint64_t generation,int64_t now) const {
        return status.state==State::Armed && status.generation==generation && have_command_ &&
            now<=deadline_ && (!status.busy || now-link_at_<=1500000);
    }
    void Tick(int64_t now) {
        if (status.state==State::Armed && have_command_ && (now>deadline_ || (status.busy && now-link_at_>1500000)))
            Disarm(Reason::Timeout);
    }
 private:
    bool have_command_=false;
    int64_t armed_at_=0,deadline_=0,link_at_=0;
    uint64_t last_action_[8][2]={};
};
} // namespace motion_safety
