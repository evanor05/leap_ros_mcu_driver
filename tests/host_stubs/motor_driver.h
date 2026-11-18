#pragma once
#include <cstdint>
class At8236Motor { public: int16_t pwm=0; void Brake(){pwm=0;} void SetSpeed(int16_t v){pwm=v;} };
