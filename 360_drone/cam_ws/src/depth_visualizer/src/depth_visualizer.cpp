#include <ros/ros.h>
#include <camera_processor/PointDepth.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// 全局声明 image_pub
image_transport::Publisher image_pub;

// 回调函数，用于处理接收到的深度信息
void depthCallback(const camera_processor::PointDepth::ConstPtr& msg) {
    int width = 640;
    int height = 480;

    cv::Mat depth_image(height, width, CV_32FC1);
    int number = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            depth_image.at<float>(y, x) = msg->depths[y * width + x];
            number++;
        }
    }
    ROS_INFO("%d",number);
    // 归一化深度图像以便可视化
    cv::Mat depth_image_normalized;
    cv::normalize(depth_image, depth_image_normalized, 0, 1, cv::NORM_MINMAX);

    // 转换为可显示的格式
    cv::Mat depth_image_display;
    depth_image_normalized.convertTo(depth_image_display, CV_8UC1, 255);

    // 应用色彩图
    cv::Mat depth_image_colored;
    cv::applyColorMap(depth_image_display, depth_image_colored, cv::COLORMAP_JET);

    // 转换为 ROS 图像消息
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_image_colored).toImageMsg();
    image_pub.publish(image_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "depth_visualizer");
    ros::NodeHandle nh;

    // 创建 ImageTransport 对象
    image_transport::ImageTransport it_(nh);

    // 订阅深度信息
    ros::Subscriber depth_sub = nh.subscribe<camera_processor::PointDepth>("camera_processor/depth/points", 1, depthCallback);

    // 发布图像话题
    image_pub = it_.advertise("camera_processor/depth/image", 1);

    ros::spin();
    return 0;
}
