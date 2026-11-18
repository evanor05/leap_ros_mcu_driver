#pragma once
#include <cstdint>
#include <functional>
class QuadratureEncoder {public:int32_t count=0;std::function<void()> on_read;
 int32_t GetCount(){if(on_read){auto call=on_read;on_read=nullptr;call();}return count;}
 void ResetCount(){count=0;}};
