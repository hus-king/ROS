#include <ros/ros.h>

//topic 头文件
#include <iostream>
#include <string>
#include <px4_command/command.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <stdlib.h>
#include <math_utils.h>
#include <algorithm> // 添加此头文件以使用 std::sort
#include <opencv_cpp_yolov5/BoundingBoxes.h>
#include <opencv_cpp_yolov5/BoundingBox.h>
#include <opencv_cpp_yolov5/BoxCenter.h>


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int detect_num;                                                  //darknet发布的检测到的物体数目
darknet_ros_msgs::BoundingBox darknet_box;                       //用于模式4只用识别一张图的情况
darknet_ros_msgs::BoundingBoxes darknet_boxes;                   //用于模式5需要识别三张图的情况
bool land_flag = false;                                                 //悬停标志

float fx=554.3827;                                               //相机内参
float fy=554.3827;
float cx=320;
float cy=240;
// float pic_target[2];                                             //模式4的图像中心ENU坐标
// float abs_distance1=10;                                          //为模式4中穿越2门与识别图像之间的过度而设置的最小距离值

std::vector<float> qr_target_x;                                //图片中心x坐标
std::vector<float> qr_target_y;                                //图片中心y坐标
float qr_cx = -1;
float qr_cy = -1;

string Class_Id_target = -1;
string Class_Id_now = -1;
int door_num = 0;// 或许有用
float door_center[2] = {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};
//--------------------------------------------回调函数--------------------------------------------------
void qrdetector_cb(const opencv_cpp_yolov5::BoxCenter::ConstPtr& msg) {
    darknet_boxes = *msg;// 只是获取消息
}

void confirm_ID(){
    // 识别的二维码ID
    float posibility[10] = {0};
    if(darknet_boxes.bounding_boxes.size() > 0){
        for(int i = 0; i < darknet_boxes.bounding_boxes.size(); i++){
            if(darknet_boxes.bounding_boxes[i].Class_Id < 10 && darknet_boxes.bounding_boxes[i].Class_Id >= 0){
                posibility[darknet_boxes.bounding_boxes[i].Class_Id] = darknet_boxes.bounding_boxes[i].probability;
            }
        }
    }// 去除不是二维码的ID

    // 找到可能性最大的id
    int max_id = 0;
    for(int i = 0; i < 10; i++){
        if(posibility[i] > posibility[max_id]){
            max_id = i;
        }
    }
    Class_Id_target = max_id;
}// 有简化空间



void find_ID() {
    float min_distance = std::numeric_limits<float>::infinity();
    int closest_index = -1;

    // 如果是目标ID，找到其中心
    if (darknet_boxes.bounding_boxes.size() > 0) {
        for (int i = 0; i < darknet_boxes.bounding_boxes.size(); i++) {
            if (darknet_boxes.bounding_boxes[i].Class_Id == Class_Id_target) {
                float center_x = (darknet_boxes.bounding_boxes[i].xmin + darknet_boxes.bounding_boxes[i].xmax) / 2;
                float center_y = (darknet_boxes.bounding_boxes[i].ymin + darknet_boxes.bounding_boxes[i].ymax) / 2;
                float distance = std::sqrt(std::pow(center_x - cx, 2) + std::pow(center_y - cy, 2));

                if (distance < min_distance) {
                    min_distance = distance;
                    closest_index = i;
                }
            }
        }

        if (closest_index != -1) {
            qr_cx = (darknet_boxes.bounding_boxes[closest_index].xmin + darknet_boxes.bounding_boxes[closest_index].xmax) / 2;
            qr_cy = (darknet_boxes.bounding_boxes[closest_index].ymin + darknet_boxes.bounding_boxes[closest_index].ymax) / 2;
            flag_land = 1;
        }
    }
}