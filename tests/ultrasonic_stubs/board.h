#pragma once

struct FakeUltrasonic {
    int attempt = 0;

    float GetDistanceCm(float temperature) {
        (void)temperature;
        const float readings[] = {25.0f, -1.0f, -1.0f, 30.0f};
        if (attempt >= 4) {
            return -1.0f;
        }
        return readings[attempt++];
    }
};

extern FakeUltrasonic ultrasonic;
