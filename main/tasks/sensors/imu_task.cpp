#include "system_globals.h"
#include "board.h" // 假设 imu 实例在此声明
#include "msg/imu_msg.h"

#include "esp_timer.h"
#include <cmath>
#include "esp_log.h" // 确保包含了日志打印头文件

void imu_task(void *p) {
    ImuMsg msg;

    while (1) {
        const int64_t sampled_us = esp_timer_get_time();
        if (imu.Update() != ESP_OK) {
            msg.sample.valid = false;
            xQueueOverwrite(system_globals().queues.imu_state, &msg);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        imu.GetQuaternion(msg.qw, msg.qx, msg.qy, msg.qz);
        msg.roll = imu.GetRoll();
        msg.pitch = imu.GetPitch();
        msg.yaw = imu.GetYaw();
        msg.acc_x = imu.GetAccX();
        msg.acc_y = imu.GetAccY();
        msg.acc_z = imu.GetAccZ();
        msg.gyro_x = imu.GetGyroX();
        msg.gyro_y = imu.GetGyroY();
        msg.gyro_z = imu.GetGyroZ();

        msg.sample.valid = std::isfinite(msg.gyro_x) && std::isfinite(msg.gyro_y) &&
            std::isfinite(msg.gyro_z) && std::isfinite(msg.acc_x) &&
            std::isfinite(msg.acc_y) && std::isfinite(msg.acc_z) && std::isfinite(msg.yaw);
        if (msg.sample.valid) {
            msg.sample.sample_time_us = sampled_us;
            ++msg.sample.sequence;
        }
        xQueueOverwrite(system_globals().queues.imu_state, &msg);

        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}
