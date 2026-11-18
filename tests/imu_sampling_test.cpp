#include "system_globals.h"
#include "board.h"
#include <cassert>
#include <vector>
FakeImu imu; Globals globals; std::vector<ImuMsg> samples;
Globals& system_globals(){return globals;}
int64_t esp_timer_get_time(){return (imu.attempt+1)*20000LL;}
void xQueueOverwrite(void*,const ImuMsg* msg){samples.push_back(*msg);}
struct End{};
void vTaskDelay(int){if(samples.size()==4)throw End{};}
void imu_task(void*);
int main(){
 try{imu_task(nullptr);}catch(const End&){}
 assert(samples.size()==4);
 assert(samples[0].sample.valid && samples[0].sample.sequence==1);
 for(int i=1;i<3;++i){
  assert(!samples[i].sample.valid);
  assert(samples[i].sample.sequence==1);
  assert(samples[i].sample.sample_time_us==samples[0].sample.sample_time_us);
 }
 assert(samples[3].sample.valid && samples[3].sample.sequence==2);
 assert(samples[3].sample.sample_time_us>samples[0].sample.sample_time_us);
}
