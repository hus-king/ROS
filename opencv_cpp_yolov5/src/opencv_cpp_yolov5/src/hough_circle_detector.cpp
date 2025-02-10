#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv_cpp_yolov5/CircleInfo.h>
#include <opencv_cpp_yolov5/CircleDetectResult.h>

ros::Publisher circle_pub;
image_transport::Publisher image_pub;
bool pubImg;
double dp, minDist, param1, param2;
int minRadius, maxRadius;

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, dp, minDist,
                     param1, param2, minRadius, maxRadius);

    opencv_cpp_yolov5::CircleDetectResult results;
    results.header = msg->header;
    results.height = msg->height;
    results.width = msg->width;

    ROS_WARN("Found %lu circles", circles.size());
    for (size_t i = 0; i < circles.size(); i++)
    {
        opencv_cpp_yolov5::CircleInfo circle_info;
        circle_info.center_x = circles[i][0];
        circle_info.center_y = circles[i][1];
        circle_info.radius = circles[i][2];
        results.circles.push_back(circle_info);

        if (pubImg)
        {
            cv::circle(cv_ptr->image, cv::Point(circles[i][0], circles[i][1]), 1, cv::Scalar(0, 100, 100), 3);
            cv::circle(cv_ptr->image, cv::Point(circles[i][0], circles[i][1]), circles[i][2], cv::Scalar(255, 0, 255), 3);
        }
    }

    circle_pub.publish(results);

    if (pubImg)
    {
        image_pub.publish(cv_ptr->toImageMsg());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hough_circle_detector_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    nh.param("dp", dp, 1.0);
    nh.param("minDist", minDist, 20.0);
    nh.param("param1", param1, 50.0);
    nh.param("param2", param2, 30.0);
    nh.param("minRadius", minRadius, 0);
    nh.param("maxRadius", maxRadius, 0);
    nh.param("pubImg", pubImg, true);

    ROS_WARN("dp: %f", dp);
    ROS_WARN("minDist: %f", minDist);
    ROS_WARN("param1: %f", param1);
    ROS_WARN("param2: %f", param2);
    ROS_WARN("minRadius: %d", minRadius);
    ROS_WARN("maxRadius: %d", maxRadius);
    ROS_WARN("pubImg: %d", pubImg);

    circle_pub = nh.advertise<opencv_cpp_yolov5::CircleDetectResult>("/opencv_cpp_yolov5/circle_detect_result", 1);
    if (pubImg)
    {
        image_pub = it.advertise("/opencv_cpp_yolov5/circle_detect_result_img", 1);
    }

    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, image_cb);

    ros::spin();
    return 0;
}