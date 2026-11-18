#include "motion_safety.h"
#include <cassert>
#include <limits>
using namespace motion_safety;
static MotionMsg Cmd(const Core& c,int64_t at,uint8_t source=MOTION_SRC_MICROROS){
    MotionMsg m;m.source=source;m.received_us=at;m.authority_generation=c.status.generation;m.target_vx=100;return m;
}
int main(){
    Core idle;
    idle.Inputs(0,true,true);assert(idle.Arm(MOTION_SRC_MICROROS,1));
    idle.Tick(10000000);assert(idle.status.state==State::Armed);
    assert(!idle.Allowed(idle.status.generation,10000000));
    MotionMsg limits;limits.control_mode=6;limits.target_vx=500;limits.target_vy=-500;
    assert(!ValidCommand(limits));
    limits.control_mode=1;limits.action_id=1;limits.target_x=1e30f;
    assert(!ValidCommand(limits));
    Core c;
    assert(!c.Arm(MOTION_SRC_MICROROS,100));
    c.Inputs(0,true,true);assert(c.Arm(MOTION_SRC_MICROROS,100));
    assert(!c.Allowed(c.status.generation,101)); // Arm alone never starts a motor.
    auto old=Cmd(c,101);assert(c.Accept(old,101));assert(c.Allowed(old.authority_generation,102));
    c.Stop();c.Disarm();assert(c.status.state==State::Estop);assert(!c.Accept(old,103));assert(!c.Allowed(old.authority_generation,104));
    assert(c.Reset());assert(c.status.state==State::Disarmed);
    assert(!c.Accept(old,105));assert(c.Arm(MOTION_SRC_MICROROS,106));assert(!c.Accept(old,107));
    assert(c.Accept(Cmd(c,108),108));
    assert(!c.Arm(MOTION_SRC_GAMEPAD,109));assert(!c.Accept(Cmd(c,109,MOTION_SRC_GAMEPAD),109));
    c.Disarm();c.Inputs(0,false,true);assert(!c.Arm(MOTION_SRC_GAMEPAD,110));
    c.Inputs(0,true,true);assert(c.Arm(MOTION_SRC_GAMEPAD,111));
    assert(c.Accept(Cmd(c,112,MOTION_SRC_GAMEPAD),112));
    c.Disarm();assert(!c.Accept(Cmd(c,113,MOTION_SRC_MICROROS),113));
    assert(c.Arm(MOTION_SRC_MICROROS,114));
    assert(c.Accept(Cmd(c,115),115));
    // Late traffic must not revive a lease just before the control task notices.
    assert(!c.Accept(Cmd(c,400116),400116));assert(c.status.state==State::Disarmed);
    assert(c.Arm(MOTION_SRC_MICROROS,400117));
    auto bad=Cmd(c,400118);bad.target_vx=std::numeric_limits<float>::quiet_NaN();
    assert(!c.Accept(bad,400118));assert(c.status.state==State::Disarmed);
    for(uint32_t fault : {Timing,BatteryLow,BatteryStale,ImuStale}){
        c.Inputs(fault,true,true);assert(c.status.state==State::Fault);
        assert(!c.Enable(MOTION_SRC_MICROROS,500000));assert(!c.Reset());
        c.Inputs(0,true,true);assert(!c.Enable(MOTION_SRC_MICROROS,500001));
        assert(c.Reset());assert(c.Arm(MOTION_SRC_MICROROS,500002));c.Disarm();
    }
    assert(c.Arm(MOTION_SRC_MICROROS,600000));
    auto action=Cmd(c,600001);action.control_mode=2;action.action_id=1;action.target_x=100;
    assert(c.Accept(action,600001));assert(!c.Accept(action,600002));
    c.Link(MOTION_SRC_MICROROS,1600000);assert(c.Allowed(action.authority_generation,1600001));
    c.Disarm(Reason::Completed);assert(c.Arm(MOTION_SRC_MICROROS,1600002));
    action=Cmd(c,1600003);action.control_mode=2;action.action_id=1;
    assert(!c.Accept(action,1600003)); // Action ID remains consumed across re-arm.
    action.action_id=2;assert(c.Accept(action,1600003));
    c.Tick(3200004);assert(c.status.state==State::Disarmed); // Link timeout.
    assert(c.Arm(MOTION_SRC_MICROROS,3300000));
    auto position=Cmd(c,3300001);position.control_mode=1;position.action_id=1000000000;
    assert(c.Accept(position,3300001));c.Disarm();
    assert(c.Arm(MOTION_SRC_MICROROS,3300002));
    auto relative=Cmd(c,3300003);relative.control_mode=2;relative.action_id=3;
    assert(c.Accept(relative,3300003));c.Disarm();
    // Repeat external stop/reset/arm/source changes 100 times.
    for(int i=0;i<100;++i){
        const uint8_t sources[]={MOTION_SRC_MICROROS,MOTION_SRC_UDP,MOTION_SRC_UART,MOTION_SRC_GAMEPAD,MOTION_SRC_ESPNOW};
        const auto source=sources[i%5];
        c.Stop();assert(c.Enable(source,4000000+i*10));
        auto m=Cmd(c,4000001+i*10,source);assert(c.Accept(m,m.received_us));
        c.Disarm();assert(!c.Allowed(m.authority_generation,m.received_us));
    }
}
