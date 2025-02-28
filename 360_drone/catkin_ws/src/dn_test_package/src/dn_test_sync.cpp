#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <camera_processor/PointDepth.h>
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;

// 全局变量定义
darknet_ros_msgs::BoundingBox target_box[2];
int note[2] = {0, 0};
int boxcount = 0;
int no_target_time = 0;
bool find_target = false;
int rand_move_thresh = 100;
float abs_distance = 0.0f;
float arrival_thresh = 0.1;
float just_go_through_thresh = 1.0;
float slanting_move_thresh = 1.5;
double dx = 0.0, dy = 0.0, dz = 0.0;
int pic_size_x = 640, pic_size_y = 480;
float variable_thresh = 0.3;
float dist_thresh = 5.0;
float fx = 606.91064453125;
float fy = 606.9465942382812;
float cx = 317.84600830078125;
float cy = 237.4210662841797;
bool onetouch = true;
float gain_z(float x1, float y1, float x2, float y2);
float depth_msg[640][480];

// 回调函数更新深度信息
void camera_cb(const camera_processor::PointDepth::ConstPtr &msg) {
    for (int y = 0; y < 480; ++y) {
        for (int x = 0; x < 640; ++x) {
            depth_msg[x][y] = msg->depths[y * 640 + x];
        }
    }
    ROS_ERROR("Depth message updated successfully");
}

// 处理识别框的回调函数
void darknet_box_cb_test_bw(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
    float promax = -1.0;
    note[0] = note[1] = 0;
    boxcount = msg->bounding_boxes.size();

    if (boxcount < 2) {
        ROS_ERROR("No available target");
        find_target = false;
        return;
    }

    for (int i = 0; i < boxcount; i++) {
        if (msg->bounding_boxes[i].probability > promax && msg->bounding_boxes[i].id == 0) {
            promax = msg->bounding_boxes[i].probability;
            note[0] = i;
        }
    }

    promax = -1.0;

    for (int i = 0; i < boxcount; i++) {
        if (i != note[0] && msg->bounding_boxes[i].probability > promax && msg->bounding_boxes[i].id == 0) {
            promax = msg->bounding_boxes[i].probability;
            note[1] = i;
        }
    }

    float pic_center[2];
    int temp;
    pic_center[0] = (msg->bounding_boxes[note[0]].xmin + msg->bounding_boxes[note[0]].xmax) / 2.0;
    pic_center[1] = (msg->bounding_boxes[note[1]].xmin + msg->bounding_boxes[note[1]].xmax) / 2.0;

    if (pic_center[0] > pic_center[1]) {
        temp = note[0];
        note[0] = note[1];
        note[1] = temp;
    }
    target_box[0] = msg->bounding_boxes[note[0]];
    target_box[1] = msg->bounding_boxes[note[1]];
    no_target_time = 0;
    find_target = true;
}

// 计算目标位置
void calculate_xyz() {
    //if (!find_target) return;  // 只在找到目标时计算

    float ymin0 = target_box[0].ymin, ymax0 = target_box[0].ymax;
    float ymin1 = target_box[1].ymin, ymax1 = target_box[1].ymax;
    float xmin0 = target_box[0].xmin, xmax0 = target_box[0].xmax;
    float xmin1 = target_box[1].xmin, xmax1 = target_box[1].xmax;
    float xcen0 = (xmax0 + xmin0) / 2.0, xcen1 = (xmax1 + xmin1) / 2.0;
    float ycen0 = (ymax0 + ymin0) / 2.0, ycen1 = (ymax1 + ymin1) / 2.0;
    float pic_center_x = (xcen0 + xcen1) / 2.0, pic_center_y = (ycen0 + ycen1) / 2.0;

    dz = gain_z(xcen0, ycen0, xcen1, ycen1);

    dy = -dz * (pic_center_x - cx) / fx;
    dx = -dz * (pic_center_y - cy) / fy;
    abs_distance = sqrt(dx * dx + dy * dy);
    
    ROS_INFO("Centre Point 2D:x=%f,y=%f\n", pic_center_x, pic_center_y);
    ROS_INFO("Now to target:dx=%f,dy=%f,dz=%f", dx, dy, dz);
    
    return ;
}

// 判断位置是否正确
bool judge_if_position_correct() {
    if (!find_target) return false;  // 只在找到目标时判断

    float ymin0 = target_box[0].ymin, ymax0 = target_box[0].ymax;
    float ymin1 = target_box[1].ymin, ymax1 = target_box[1].ymax;
    float xmin0 = target_box[0].xmin, xmax0 = target_box[0].xmax;
    float xmin1 = target_box[1].xmin, xmax1 = target_box[1].xmax;
    float xcen0 = (xmax0 + xmin0) / 2.0, xcen1 = (xmax1 + xmin1) / 2.0;
    float size0 = (ymax0 - ymin0) * (xmax0 - xmin0);
    float size1 = (ymax1 - ymin1) * (xmax1 - xmin1);

    if ((size0 / size1 > (1 + variable_thresh)) || (size0 / size1 < (1 - variable_thresh))) {
        return false;
    }
    if (((xcen1 - xcen0) / ((xmax1 - xmin1 + xmax0 - xmin0) / 2.0)) < dist_thresh) {
        return false;
    }

    ROS_INFO("Position is correct!");
    return true;
}

// 计算深度
float gain_z(float x1, float y1, float x2, float y2) {
    float z1 = depth_msg[static_cast<int>(x1)][static_cast<int>(y1)];
    float z2 = depth_msg[static_cast<int>(x2)][static_cast<int>(y2)];
    ROS_INFO("z1:%f,z2:%f", z1, z2);
    return (z1 + z2) / 2;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dn_node");
    ros::NodeHandle nh("~");

    // 创建 AsyncSpinner，使用8个线程
    ros::AsyncSpinner spinner(0);
    spinner.start(); // 启动回调处理

    ros::Subscriber camera_sub = nh.subscribe<camera_processor::PointDepth>("/camera_processor/depth/points", 1, camera_cb);
    ros::Subscriber darknet_box_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, darknet_box_cb_test_bw);

    no_target_time = 0;
    find_target = false;

    while (ros::ok()) {
        // 进行其他处理
        calculate_xyz(); // 计算目标位置

        if (no_target_time > rand_move_thresh) {
            // TODO: start random movement as to find target
            continue;
        }

        if (!find_target) {
            no_target_time++;
            continue;
        }

        calculate_xyz();
        
        if (judge_if_position_correct()) {
            if (dz < just_go_through_thresh) {
                // TODO: just go through the circle
            } else if (dz < slanting_move_thresh) {
                // TODO: take one step toward target, in x, y, z
            } else {
                if (abs_distance < arrival_thresh) {
                    // TODO: take one step toward target, in z
                } else {
                    // TODO: take one step toward target, in x, y
                }
            }
        } else {
            // TODO: rotate to fix position
        }
    }

    return 0;
}
