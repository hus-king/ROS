float calculateDepth(const CSSBox& box, bool isSquare, bool isBlue) {
    float depth_sum = 0;
    int e_index = 0;
    float x_middle = (box.xmin + box.xmax) / 2.0f;
    float y_middle = (box.ymin + box.ymax) / 2.0f;
    float x_smiddle, x_tmiddle;
    float y_smiddle = (box.ymax - box.ymin) / 8.0f;

    // 计算x_smiddle和x_tmiddle（圆环时）
    if (isSquare) {
        x_smiddle = isBlue ? (x_middle + box.xmin) / 2.0f : (x_middle + box.xmax) / 2.0f;
    } else {
        x_tmiddle = isBlue ? (box.xmax - box.xmin) / 3.0f + box.xmin 
                           : 2.0f * (box.xmax - box.xmin) / 3.0f + box.xmin;
        x_smiddle = isBlue ? (x_tmiddle + box.xmin) / 2.0f 
                            : (x_tmiddle + box.xmax) / 2.0f;
    }

    // 第一个循环：四个方向的像素采样
    float x_base = isSquare ? x_middle : x_tmiddle;
    int half_width = static_cast<int>((box.xmax - box.xmin) / 2.0f);
    int half_height = static_cast<int>((box.ymax - box.ymin) / 2.0f);

    for (int j = 0; j < half_height; ++j) {
        for (int i = 0; i < half_width; ++i) {
            auto checkAndAdd = [&](int x, int y) {
                if (depth_msg[x][y] > 0.5f && depth_msg[x][y] < far_thres) {
                    depth_sum += depth_msg[x][y];
                    e_index++;
                }
            };

            checkAndAdd(static_cast<int>(x_base + i), static_cast<int>(y_middle + j));  // 右上
            checkAndAdd(static_cast<int>(x_base + i), static_cast<int>(y_middle - j));  // 右下
            checkAndAdd(static_cast<int>(x_base - i), static_cast<int>(y_middle + j));  // 左上
            checkAndAdd(static_cast<int>(x_base - i), static_cast<int>(y_middle - j));  // 左下

            if (e_index != 0) break;
        }
        if (e_index != 0) break;
    }

    // 方框红色特殊处理：强制初始化平均值
    if (!isSquare && !isBlue) e_index++; // 处理原代码中的e_index++情况
    if (e_index == 0) return 0.0f; // 防止除零

    float depth_average = depth_sum / e_index;

    // 第二个循环：精细区域采样
    int i_start, i_end;
    if (isSquare) {
        i_start = isBlue ? static_cast<int>(x_smiddle) : static_cast<int>(x_middle);
        i_end = isBlue ? static_cast<int>(x_middle) : static_cast<int>(x_smiddle);
    } else {
        i_start = isBlue ? static_cast<int>(x_smiddle) : static_cast<int>(x_tmiddle);
        i_end = isBlue ? static_cast<int>(x_tmiddle) : static_cast<int>(x_smiddle);
    }

    for (int i = i_start; i < i_end; ++i) {
        for (int j = static_cast<int>(y_middle - y_smiddle); 
             j < static_cast<int>(y_middle + y_smiddle); ++j) {
            if (fabs(depth_msg[i][j] - depth_average) >= depth_thres) continue;
            depth_sum += depth_msg[i][j];
            e_index++;
            depth_average = depth_sum / e_index;
        }
    }

    return depth_average;
}

float gain_depth() {
    float z1 = 0.0f, z2 = 0.0f;

    if (TARGET.bigbox.id == 3) { // 方框
        // 处理第一个cssbox
        if (TARGET.cssbox[0].id == 1) { // 蓝色
            z1 = calculateDepth(TARGET.cssbox[0], true, true);
            ROS_ERROR("Z1:%f", z1);
        } else if (TARGET.cssbox[0].id == 0) { // 红色
            z2 = calculateDepth(TARGET.cssbox[0], true, false);
            ROS_ERROR("Z2:%f", z2);
        }

        // 处理第二个cssbox
        if (TARGET.cssbox[1].id == 1) {
            z1 = calculateDepth(TARGET.cssbox[1], true, true);
            ROS_ERROR("Z1:%f", z1);
        } else if (TARGET.cssbox[1].id == 0) {
            z2 = calculateDepth(TARGET.cssbox[1], true, false);
            ROS_ERROR("Z2:%f", z2);
        }
    } else if (TARGET.bigbox.id == 2) { // 圆环
        // 处理第一个cssbox
        if (TARGET.cssbox[0].id == 7) { // 蓝色
            z1 = calculateDepth(TARGET.cssbox[0], false, true);
        } else if (TARGET.cssbox[0].id == 6) { // 红色
            z2 = calculateDepth(TARGET.cssbox[0], false, false);
        }

        // 处理第二个cssbox
        if (TARGET.cssbox[1].id == 7) {
            z1 = calculateDepth(TARGET.cssbox[1], false, true);
        } else if (TARGET.cssbox[1].id == 6) {
            z2 = calculateDepth(TARGET.cssbox[1], false, false);
        }
    }

    // 后续处理逻辑保持不变
    if(z1 - z2 > 0.5) turning = TURNING_R;
    else if(z1 - z2 < -0.5) turning = TURNING_L;
    else turning = TURNING_M;
    ROS_INFO("z1:%f,z2:%f",z1,z2);
    
    float xcen1 = (TARGET.cssbox[1].xmax + TARGET.cssbox[1].xmin)/2.0f;
    float xcen0 = (TARGET.cssbox[0].xmax + TARGET.cssbox[0].xmin)/2.0f;
    float css_dis = abs(xcen0 - xcen1)*(z1+z2)/(fx*2.0f);
    
    if(xcen0 < xcen1)  lean_angle = atan2(z2-z1 , css_dis);
    else if(xcen0 > xcen1) lean_angle = -atan2(z2-z1,css_dis);
    return (z1+z2)/2;
}