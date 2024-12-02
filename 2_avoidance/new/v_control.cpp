#include <stdio.h>
#include <iostream>
#include <cmath>

// 保持速度不变的情况下，改变速度的方向朝向目标角度
void v_control(float v, float newv[2], float target_angle) {
    // 将角度从度转换为弧度
    float angle = target_angle * M_PI / 180.0;

    // 计算新的速度分量
    newv[0] = v * cos(angle);
    newv[1] = v * sin(angle);
}

int main() {
    float v = 1.0; // 初始速度大小
    float new_velocity[2];
    float target_angle = 45.0; // 目标角度，单位：度

    v_control(v, new_velocity, target_angle);

    std::cout << "New velocity: (" << new_velocity[0] << ", " << new_velocity[1] << ")" << std::endl;

    return 0;
}