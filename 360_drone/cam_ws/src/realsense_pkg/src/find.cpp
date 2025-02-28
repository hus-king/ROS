#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <memory>

// 全局变量，用于存储最新的彩色图像和深度图像
cv::Mat color_image;
cv::Mat depth_image;
cv::Mat depth_image_8u;  // 8位深度图像
cv::Mat depth_image_colormap;  // 伪彩色深度图像
rs2_intrinsics intrinsics;  // 存储深度流的内参
std::shared_ptr<rs2::depth_frame> current_depth_frame;  // 当前深度帧的智能指针

// 初始化RealSense相机配置
rs2::pipeline realsense_pipe;
rs2::config cfg;
rs2::align align_to_color(RS2_STREAM_COLOR);
rs2::colorizer color_map;

// 相机回调函数
void camera_callback(const ros::TimerEvent&) {
    // 获取帧集并对齐深度帧到彩色帧
    rs2::frameset frameset = realsense_pipe.wait_for_frames();
    rs2::frameset aligned_frameset = align_to_color.process(frameset);

    // 获取对齐后的彩色帧和深度帧
    rs2::video_frame color_frame = aligned_frameset.get_color_frame();
    current_depth_frame = std::make_shared<rs2::depth_frame>(aligned_frameset.get_depth_frame());

    // 将彩色帧转换为OpenCV Mat
    const int w = color_frame.as<rs2::video_frame>().get_width();
    const int h = color_frame.as<rs2::video_frame>().get_height();
    color_image = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);

    // 将深度帧转换为OpenCV Mat
    depth_image = cv::Mat(cv::Size(w, h), CV_16UC1, (void*)current_depth_frame->get_data(), cv::Mat::AUTO_STEP);

    // 将深度图像转换为8位单通道，并应用颜色映射
    depth_image.convertTo(depth_image_8u, CV_8UC1, 255.0 / 10000);  // 假设深度值在0-10米之间
    applyColorMap(depth_image_8u, depth_image_colormap, cv::COLORMAP_JET);

    // 显示图像
    cv::imshow("Color Image", color_image);
    cv::imshow("Depth Image", depth_image_colormap);
    cv::waitKey(1);
}

// 无人机方向判断函数
int determine_drone_direction() {
    const int w = depth_image.cols;
    const int h = depth_image.rows;
    float far_distance = 3.0f;  // Maximum depth to consider (in meters)

    // 初始化深度和数组
    std::vector<float> depth_sums(w, 0.0f);
    std::vector<float> depth_aver(w, 0.0f);

    // 遍历每个像素并计算每列的有效深度和
    for (int x = 0; x < w; ++x) {
        int depth_index = 0;
        for (int y = 0; y < h; ++y) {
            float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
            // 获取初始深度值
            float depth_value = current_depth_frame->get_distance(pixel[0], pixel[1]);
            if (depth_value > 0 && depth_value <= far_distance) {
                float point[3];
                rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth_value);  // 获取3D坐标
                depth_sums[x] += point[2];  // 使用Z值作为深度
                depth_index++;
            }
        }
        if (depth_index != 0) {
            depth_aver[x] = depth_sums[x] / depth_index;
        }
    }

    std::cout << "Depth sums: ";
    for (float depth_sum : depth_sums) {
        std::cout << depth_sum << " ";
    }
    std::cout << std::endl;

    // 分析深度和数组，以确定无人机的移动方向
    float left_sum = 0, right_sum = 0;
    for (int i = 1; i < w; ++i) {
        if (depth_aver[i] > depth_aver[i - 1]) right_sum++;
        else if (depth_aver[i] < depth_aver[i - 1]) left_sum++;
    }

    float rate = 0;
    if (right_sum + left_sum != 0) {
        rate = right_sum / (right_sum + left_sum);
    }

    std::cout << "Left sum: " << left_sum << ", Right sum: " << right_sum << std::endl;

    if (rate > 0.6) {
        std::cout << "Drone should move left" << std::endl;
        return 2;  // 左转
    } else if (rate < 0.4) {
        std::cout << "Drone should move right" << std::endl;
        return 1;  // 右转
    } else {
        std::cout << "Drone is on the middle" << std::endl;
        return 0;  // 保持方向
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "realsense_listener");
    ros::NodeHandle nh;

    // 设置相机配置
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    
    // 开始配置并获取相机内参
    rs2::pipeline_profile profile = realsense_pipe.start(cfg);
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    intrinsics = depth_stream.get_intrinsics();  // 获取深度相机的内参

    cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Color Image", cv::WINDOW_AUTOSIZE);

    // 使用定时器定期调用相机回调函数
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), camera_callback);  // 每0.1秒调用一次

    // 主循环
    while (ros::ok()) {
        ros::spinOnce();  // 处理定时器回调
        int direction = determine_drone_direction();  // 确定无人机的方向
        // 这里可以发布ROS消息或者进行其他操作
    }

    return 0;
}
