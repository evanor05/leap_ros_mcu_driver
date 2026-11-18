#include "motion_controller.h"
#include <cassert>
#include <cmath>
#include <limits>
int main() {
 At8236Motor l,r; QuadratureEncoder el,er; MotionController robot(l,r,el,er);
 robot.Drive(100,0,0);
 el.count=-468; er.count=468;
 robot.Update(.02f,0,false);
 float x,y,yaw; robot.GetOdometry(&x,&y,&yaw);
 const float step=leap_config::motion::kWheelDiameterMm*3.14159265358979323846f/10;
 assert(std::abs(x-step)<.001f); assert(l.pwm==0 && r.pwm==0);
 // Movement during stop remains in pose, and resume never replays the drive target.
 el.count-=468;er.count+=468;robot.Update(.04f,0,false);
 robot.GetOdometry(&x,&y,&yaw);assert(std::abs(x-2*step)<.001f);
 robot.Update(.02f,0); assert(l.pwm==0 && r.pwm==0);
 // Integrating the same raw counts at a different dt gives the same pose.
 robot.ResetOdometry();el.count=-468;er.count=468;robot.Update(.1f,0,false);
 robot.GetOdometry(&x,&y,&yaw);assert(std::abs(x-step)<.001f);
 robot.Update(std::numeric_limits<float>::quiet_NaN(),0);assert(l.pwm==0 && r.pwm==0);
 // A curved step uses midpoint heading, not end heading.
 robot.ResetOdometry();er.count=468;robot.Update(.02f,0,false);
 robot.GetOdometry(&x,&y,&yaw);
 float da=step/leap_config::motion::kTrackWidthMm;
 assert(std::abs(x-step*.5f*std::cos(da*.5f))<.001f);
 assert(std::abs(y-step*.5f*std::sin(da*.5f))<.001f);
}
