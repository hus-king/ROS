#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <camera_processor/PointDepth.h>

class PointDepthViewer {
public:
    PointDepthViewer(const std::string& config_path) : it_(nh_) {
        // 订阅彩色图像和深度信息
        color_sub_ = it_.subscribe("/camera_processor/color/image_raw", 1, &PointDepthViewer::colorCallback, this);
        depth_sub_ = nh_.subscribe("/camera_processor/depth/points", 1, &PointDepthViewer::depthCallback, this);

        // 从YAML文件读取点位置
        YAML::Node config = YAML::LoadFile(config_path);
        target_x_ = config["point"]["x"].as<int>();
        target_y_ = config["point"]["y"].as<int>();
    }

    void colorCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            // 在图像中标出点的位置和深度值
            if (!std::isnan(target_z_)) {
                cv::circle(frame, cv::Point(target_x_, target_y_), 5, CV_RGB(255, 0, 0), -1);
                std::string text = "Depth: " + std::to_string(target_z_);
                cv::putText(frame, text, cv::Point(target_x_, target_y_ - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255), 1);
            }

            // 显示图像
            cv::imshow("Color Image", frame);
            cv::waitKey(1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void depthCallback(const camera_processor::PointDepth::ConstPtr& msg) {
        int index = target_y_ * 640 + target_x_;
        if (index < msg->depths.size()) {
            target_z_ = msg->depths[index];
        } else {
            target_z_ = std::nan("");
            ROS_WARN("Point out of depth bounds");
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber color_sub_;
    ros::Subscriber depth_sub_;

    int target_x_;
    int target_y_;
    float target_z_ = std::nan("");
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_depth_viewer");

    // 修改为从参数服务器获取配置文件路径，或使用默认值
    std::string config_path = "/home/nx/cam_ws/src/point_depth_viewer/config/config.yaml"; // 你可以替换成实际路径
    if (argc > 1) {
        config_path = argv[1];
    }

    PointDepthViewer pdv(config_path);
    ros::spin();
    return 0;
}
