#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <librealsense2/rs.hpp>
#include "camera_processor/PointDepth.h"

class CameraProcessor {
public:
    CameraProcessor()
        : it_(nh_) {
        // 发布处理后的彩色图像和深度信息
        color_pub_ = it_.advertise("/camera_processor/color/image_raw", 1);
        depth_pub_ = nh_.advertise<camera_processor::PointDepth>("/camera_processor/depth/points", 1);

        // 初始化相机
        cfg_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
        cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

        try {
            pipe_.start(cfg_);
        } catch (const rs2::error & e) {
            ROS_ERROR("RealSense error calling rs2_pipeline_start: %s", e.what());
            return;
        }

        auto stream_profile = pipe_.get_active_profile().get_stream(RS2_STREAM_COLOR);
        intrinsics_ = stream_profile.as<rs2::video_stream_profile>().get_intrinsics();
    }

    void process() {
        while (ros::ok()) {
            rs2::frameset frameset = pipe_.wait_for_frames();
            rs2::align align_to(RS2_STREAM_COLOR);
            auto aligned_frameset = align_to.process(frameset);

            auto color_frame = aligned_frameset.get_color_frame();
            auto depth_frame = aligned_frameset.get_depth_frame();

            if (!color_frame || !depth_frame) {
                ROS_WARN("Invalid frame received");
                continue;
            }

            // 发布彩色图像
            sensor_msgs::Image color_msg;
            color_msg.header.stamp = ros::Time::now();
            color_msg.height = color_frame.get_height();
            color_msg.width = color_frame.get_width();
            color_msg.encoding = sensor_msgs::image_encodings::RGB8; 
            color_msg.step = color_msg.width * 3;
            color_msg.data.assign(
                reinterpret_cast<const uint8_t*>(color_frame.get_data()),
                reinterpret_cast<const uint8_t*>(color_frame.get_data()) + (color_msg.step * color_msg.height)
            );
            color_pub_.publish(color_msg);

            // 发布深度信息为二维数组
            camera_processor::PointDepth depth_msg;
            depth_msg.depths.resize(640*480);
            for (int y = 0; y < 480; ++y) {
                for (int x = 0; x < 640; ++x) {
                    float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
                    float point[3];
                    rs2_deproject_pixel_to_point(point, &intrinsics_, pixel, depth_frame.get_distance(x, y));
                    depth_msg.depths[y * 640 + x] = point[2]; // 发布z值
                }
            }
            depth_pub_.publish(depth_msg);
            ROS_INFO("success");
            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher color_pub_;
    ros::Publisher depth_pub_;

    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2_intrinsics intrinsics_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_processor");
    CameraProcessor cp;
    cp.process();
    return 0;
}
