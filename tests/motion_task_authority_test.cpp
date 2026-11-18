#include "system_globals.h"
#include "board.h"
#include "msg/imu_msg.h"
#include "msg/battery_msg.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>

using namespace motion_safety;
At8236Motor left_motor,right_motor;
QuadratureEncoder left_encoder,right_encoder;
MotionController robot(left_motor,right_motor,left_encoder,right_encoder);
static ImuMsg imu;
static BatteryMsg battery;
static MotionMsg telemetry;
static Globals globals{{&imu,&battery,&telemetry},{}};
static int64_t now_us=1000000;
static int step=0;
static std::atomic<bool> block_write{false},entered{false},release_write{false};
Globals& system_globals(){return globals;}
void vTaskDelay(TickType_t){}
int64_t esp_timer_get_time(){return now_us;}
TickType_t xTaskGetTickCount(){return now_us/1000;}
int xQueuePeek(QueueHandle_t q,void* out,int){
 if(q==&imu)std::memcpy(out,q,sizeof(imu));else if(q==&battery)std::memcpy(out,q,sizeof(battery));
 else return 0;
 return pdTRUE;
}
void xQueueOverwrite(QueueHandle_t q,const void* value){assert(q==&telemetry);telemetry=*static_cast<const MotionMsg*>(value);}
struct End{};
static bool Submit(uint8_t source=MOTION_SRC_MICROROS,uint8_t mode=0){
 MotionMsg c;c.source=source;c.control_mode=mode;c.target_vx=100;
 if(mode==2){c.action_id=1;c.target_x=100;}
 return motion_submit(c);
}
void vTaskDelayUntil(TickType_t*,TickType_t period){
 now_us+=period*1000;++step;
 imu.sample.valid=true;imu.sample.sample_time_us=now_us;imu.sample.sequence++;
 battery.valid=battery.sample.valid=true;battery.voltage_v=7.4;
 battery.sample.sample_time_us=now_us;battery.sample.sequence++;
 if(step==2){assert(motion_enable(MOTION_SRC_MICROROS));assert(Submit());}
 if(step==3){assert(left_motor.pwm!=0);left_encoder.count=-468;right_encoder.count=468;
   motion_estop();assert(left_motor.pwm==0 && right_motor.pwm==0);assert(!Submit());}
 if(step==4){assert(telemetry.x>20);assert(telemetry.sample.valid);}
 if(step==26){assert(motion_enable(MOTION_SRC_MICROROS));assert(Submit());}
 if(step==27){assert(!Submit(MOTION_SRC_GAMEPAD));motion_release(MOTION_SRC_MICROROS);}
 if(step==28){assert(motion_arm(MOTION_SRC_GAMEPAD));assert(Submit(MOTION_SRC_GAMEPAD));}
 if(step==29){motion_release(MOTION_SRC_GAMEPAD);assert(!Submit());}
 if(step==30){assert(motion_enable(MOTION_SRC_MICROROS));assert(Submit());
    left_encoder.on_read=[](){motion_estop();};}
 if(step==31){assert(left_motor.pwm==0 && right_motor.pwm==0);assert(motion_status().state==State::Estop);}
 if(step>=40 && step<=46)battery.voltage_v=6.;
 if(step==47){assert(motion_status().faults & BatteryLow);assert(!motion_enable(MOTION_SRC_MICROROS));}
 if(step==48){assert(motion_reset());assert(motion_arm(MOTION_SRC_MICROROS));assert(Submit());}
 if(step==49)now_us+=100000;
 if(step==50){assert(motion_status().faults & Timing);assert(!motion_enable(MOTION_SRC_MICROROS));}
 if(step==52){assert(motion_reset());assert(motion_arm(MOTION_SRC_MICROROS));}
 if(step==53){imu.sample.valid=false;assert(Submit(MOTION_SRC_MICROROS,2));}
 if(step==54){imu.sample.valid=false;assert(motion_status().faults & ImuStale);assert(!motion_reset());}
 if(step==56){assert(motion_reset());assert(motion_status().state==State::Disarmed);
   assert(motion_request_odometry_reset());assert(!motion_arm(MOTION_SRC_MICROROS));}
 if(step==57){assert(telemetry.x==0);assert(motion_enable(MOTION_SRC_MICROROS));assert(Submit());}
 if(step==58){left_encoder.on_read=[](){now_us+=400000;};}
 if(step==59){assert(left_motor.pwm==0 && right_motor.pwm==0);assert(motion_status().faults & Timing);}
 if(step==60)assert(motion_status().faults & Timing);
 if(step==61)assert(motion_reset());
 if(step==62)battery.sample.valid=false;
 if(step==63){assert(motion_status().faults & BatteryStale);assert(!motion_reset());}
 if(step==64){assert(motion_reset());assert(!Submit());}
 if(step==65)throw End{};
}
void motion_task(void*);
static void Sink(int16_t left,int16_t right){
 if(left && block_write){entered=true;while(!release_write)std::this_thread::yield();}
 if(left)left_motor.SetSpeed(left);else left_motor.Brake();
 if(right)right_motor.SetSpeed(right);else right_motor.Brake();
}
int main(){
 motion_authority_init(Sink);
 try{motion_task(nullptr);}catch(const End&){}
 assert(step==65);
 motion_estop();motion_complete();motion_disarm();
 assert(motion_status().state==State::Estop);
 assert(motion_reset());
 // An independent watchdog poll stops a stalled motion task.
 assert(motion_enable(MOTION_SRC_MICROROS));assert(Submit());
 uint64_t stalled_generation=motion_status().generation;
 assert(motion_commit(&stalled_generation,80,80));
 now_us+=50000;motion_tick();
 assert(motion_status().faults & Timing);
 assert(left_motor.pwm==0 && right_motor.pwm==0);
 motion_inputs(0,true,true);assert(motion_reset());
 // Test the actual platform wrapper, including stop/write serialization.
 assert(motion_enable(MOTION_SRC_MICROROS));assert(Submit());
 uint64_t generation=motion_status().generation;
 block_write=true;
 std::thread writer([&](){assert(motion_commit(&generation,40,40));});
 while(!entered)std::this_thread::yield();
 std::atomic<bool> stopping{false},stop_finished{false};
 std::thread stop([&](){stopping=true;motion_estop();stop_finished=true;});
 while(!stopping)std::this_thread::yield();
 std::this_thread::sleep_for(std::chrono::milliseconds(10));
 assert(!stop_finished); // Stop cannot return ahead of an in-flight hardware commit.
 release_write=true;writer.join();stop.join();
 assert(left_motor.pwm==0 && right_motor.pwm==0);
 assert(!motion_commit(&generation,100,100));
 assert(left_motor.pwm==0 && right_motor.pwm==0);
 std::cout<<"real motion_task + authority: 65 cycles, fault injection and concurrent stop passed\n";
}
