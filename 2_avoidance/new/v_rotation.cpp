#include <stdio.h>
#include <iostream>
#include <cmath>

// old[2]为原始的x,y方向上的速度
// new[2]为旋转angle后的x,y方向上的速度
// angle为旋转的角度（单位：度）

void v_rotation(float old[2], float new_v[2], float angle) {
    // 计算原始速度的大小
    float v = sqrt(old[0] * old[0] + old[1] * old[1]);

    // 计算原始速度的方向
    float alpha = atan2(old[1], old[0]);

    // 将角度从度转换为弧度
    angle = angle * M_PI / 180.0;

    // 计算旋转后的方向
    float theta_new = alpha + angle;

    // 计算旋转后的速度分量
    new_v[0] = v * cos(theta_new);
    new_v[1] = v * sin(theta_new);
}

int main() {
    float old_velocity[2] = {1.0, 1.0};
    float new_velocity[2];
    float angle = 45; // 45 degrees

    v_rotation(old_velocity, new_velocity, angle);

    std::cout << "New velocity: (" << new_velocity[0] << ", " << new_velocity[1] << ")" << std::endl;

    return 0;
}