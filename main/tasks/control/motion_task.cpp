#include "motion_config.h"
#include "system_globals.h"
#include "board.h"
#include "msg/imu_msg.h"
#include "msg/battery_msg.h"
#include "esp_timer.h"
#include <cmath>

// The only task allowed to change MotionController state. All actual PWM writes
// go through motion_commit, serialized with stop/reset/arm and fresh time checks.
void motion_task(void*) {
    TickType_t wake=xTaskGetTickCount();
    int64_t last_us=esp_timer_get_time(),low_since=0;
    uint64_t generation=0;
    uint32_t battery_sequence=0;
    bool battery_seen=false;
    MotionMsg active,status;
    ImuMsg imu_msg;
    robot.SetOutputWriter(motion_commit,&generation);
    while(true){
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(leap_config::motion::kControlLoopPeriodMs));
        const int64_t now=esp_timer_get_time(),elapsed=now-last_us;last_us=now;
        const float dt=elapsed*1e-6f;
        uint32_t faults=0;
        if(elapsed<=0 || elapsed>40000){faults|=motion_safety::Timing;wake=xTaskGetTickCount();}
        xQueuePeek(system_globals().queues.imu_state,&imu_msg,0);
        const bool imu_fresh=imu_msg.sample.valid && imu_msg.sample.sample_time_us>0 &&
            now>=imu_msg.sample.sample_time_us && now-imu_msg.sample.sample_time_us<=200000;
        BatteryMsg battery;
        xQueuePeek(system_globals().queues.battery_state,&battery,0);
        if(battery.sample.sequence!=battery_sequence){battery_sequence=battery.sample.sequence;battery_seen=true;}
        const bool battery_fresh=battery.valid && battery.sample.valid && std::isfinite(battery.voltage_v) &&
            battery.sample.sample_time_us>0 && now>=battery.sample.sample_time_us && now-battery.sample.sample_time_us<=2000000;
        if(battery_seen && !battery_fresh) faults|=motion_safety::BatteryStale;
        if(battery_fresh && battery.voltage_v<leap_config::motion::kLowVoltageStopV){
            if(!low_since)low_since=now;
            if(now-low_since>=100000)faults|=motion_safety::BatteryLow;
        }else low_since=0;
        if((active.control_mode==2 || (motion_status().faults & motion_safety::ImuStale)) && !imu_fresh)faults|=motion_safety::ImuStale;
        const bool stopped=std::abs(status.vel_left)<2 && std::abs(status.vel_right)<2;
        motion_inputs(faults,stopped,battery_fresh);
        motion_tick();
        auto safety=motion_status();
        if(safety.generation!=generation){robot.Stop();active={};generation=safety.generation;}
        if(motion_take_odometry_reset())robot.ResetOdometry();
        MotionMsg incoming;
        if(motion_take(&incoming)){
            active=incoming;generation=incoming.authority_generation;
            if(active.control_mode==0)robot.Drive(active.target_vx,0,active.target_wz);
            else if(active.control_mode==1)robot.MoveToPosition(active.target_x,active.target_y,active.target_yaw);
            else if(active.control_mode==2){
                if(!imu_fresh){motion_inputs(faults|motion_safety::ImuStale,stopped,battery_fresh);active={};robot.Stop();}
                else robot.MoveRelative(active.target_x,active.target_yaw);
            }
            else if(active.control_mode==3){robot.Stop();robot.SetMotorTargetVelocity(static_cast<MotorID>(active.motor_id),active.target_motor_v);}
            else if(active.control_mode==6)robot.SetAllMotorTargetsVelocity(active.target_vx,active.target_vy);
        }
        PidMsg gains;
        if(xQueueReceive(system_globals().queues.speedpid_cmd,&gains,0)==pdTRUE &&
           std::isfinite(gains.kp)&&std::isfinite(gains.ki)&&std::isfinite(gains.kd)){
            robot.SetVelocityPidGains(gains.kp,gains.ki,gains.kd);
            system_globals().control.speed_pid=gains;
        }
        if(xQueueReceive(system_globals().queues.positionpid_cmd,&gains,0)==pdTRUE &&
           std::isfinite(gains.kp)&&std::isfinite(gains.ki)&&std::isfinite(gains.kd)){
            robot.SetPositionPidGains(gains.kp,gains.ki,gains.kd);
            system_globals().control.position_pid=gains;
        }
        safety=motion_status();
        robot.Update(dt,imu_fresh?imu_msg.yaw*(3.14159265358979323846f/180):NAN,
                     safety.state==motion_safety::State::Armed && safety.generation==generation);
        if(active.control_mode==2 && !robot.IsBusy()) {motion_complete();active={};}
        robot.GetVelocity(&status.vx,&status.vy,&status.wz);
        robot.GetOdometry(&status.x,&status.y,&status.yaw);
        robot.GetOdometryQuaternion(&status.qw,&status.qx,&status.qy,&status.qz);
        robot.GetAllMotorVelocities(&status.vel_left,&status.vel_right);
        robot.GetAppliedPwm(&status.pwm_left,&status.pwm_right);
        status.control_mode=active.control_mode;status.source=motion_status().owner;
        status.target_vx=active.target_vx;
        status.sample.valid=std::isfinite(status.x)&&std::isfinite(status.y)&&std::isfinite(status.yaw)&&std::isfinite(status.vx)&&std::isfinite(status.wz);
        if(status.sample.valid){status.sample.sample_time_us=now;++status.sample.sequence;}
        xQueueOverwrite(system_globals().queues.motion_state,&status);
    }
}
