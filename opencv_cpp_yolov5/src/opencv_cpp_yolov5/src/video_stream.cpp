#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_stream_publisher");
    ros::NodeHandle nh("~");

    std::string video_path;
    int frame_height, frame_width;

    nh.param("video_path", video_path, std::string("/path/to/video.mp4"));
    nh.param("frame_height", frame_height, 480);
    nh.param("frame_width", frame_width, 640);

    ROS_INFO("video_path: %s", video_path.c_str());
    ROS_INFO("frame_height: %d", frame_height);
    ROS_INFO("frame_width: %d", frame_width);

    cv::VideoCapture cap(video_path);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);

    if (!cap.isOpened()) {
        ROS_ERROR("Could not open video file.");
        return 1;
    }

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("image_raw", 1);
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(20);  // Adjust the rate as needed
    while (nh.ok()) {
        cap >> frame;
        if (!frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        } else {
            ROS_WARN("End of video file reached. restarting...");
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            continue;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}