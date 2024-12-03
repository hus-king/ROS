#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"

static const std::string RGB_WINDOW = "RGB Image window";

cv::Mat rgbimage;
cv::Mat depthimage;
cv::Rect selectRect;
cv::Point origin;
cv::Rect result;

bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
//bool enable_get_depth = false;

bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;

// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
float center_x;
int detect_flag;

void onMouse(int event, int x, int y, int, void*)
{
    if (select_flag)
    {
        selectRect.x = MIN(origin.x, x);        
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);   
        selectRect.height = abs(y - origin.y);
        selectRect &= cv::Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        bBeginKCF = false;  
        select_flag = true; 
        origin = cv::Point(x, y);       
        selectRect = cv::Rect(x, y, 0, 0);  
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        select_flag = false;
        bRenewROI = true;
    }
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ros::Publisher pub;

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    pub = nh_.advertise<geometry_msgs::Pose>("/vision/target", 1000);
    cv::namedWindow(RGB_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(RGB_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    cv_ptr->image.copyTo(rgbimage);

    cv::setMouseCallback(RGB_WINDOW, onMouse, 0);

    if(bRenewROI)
    {
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        bRenewROI = false;
    }

    if(bBeginKCF)
    {
        result = tracker.update(rgbimage);
        center_x = result.x + result.width/2 - 320.0;
        detect_flag = 1;
        cv::rectangle(rgbimage, result, cv::Scalar( 0, 255, 255 ), 1, 8 );
    }
    else
    {
         cv::rectangle(rgbimage, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);
         detect_flag = 0;
    }
    cv::imshow(RGB_WINDOW, rgbimage);
    cv::waitKey(1);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "kcf_tracker");
  ImageConverter ic;
  
  while(ros::ok())
  {
    ros::spinOnce();

    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.0;
    target_pose.position.y = center_x;
    target_pose.position.z = 0.0;
    target_pose.orientation.w = detect_flag;
    ic.pub.publish(target_pose);
    if (cvWaitKey(33) == 'q')
      break;
  }
  return 0;
}

