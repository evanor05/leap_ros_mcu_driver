#pragma once
#include <cmath>
struct FakeImu {
 int attempt=0;
 int Update(){return ++attempt==2 ? -1 : 0;}
 void GetQuaternion(float& w,float& x,float& y,float& z){w=1;x=y=z=0;}
 float GetRoll(){return 0;} float GetPitch(){return 0;} float GetYaw(){return 0;}
 float GetAccX(){return 0;} float GetAccY(){return 0;} float GetAccZ(){return 1;}
 float GetGyroX(){return attempt==3 ? NAN : 0;}
 float GetGyroY(){return 0;} float GetGyroZ(){return 0;}
};
extern FakeImu imu;
