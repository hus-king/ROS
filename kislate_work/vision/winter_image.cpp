#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

void boundingBoxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  for (const auto& box : msg->bounding_boxes) {
    ROS_INFO("Class: %s, Probability: %f, BBox: [%d, %d, %d, %d]",
             box.Class.c_str(), box.probability, box.xmin, box.ymin, box.xmax, box.ymax);
  }
}

void captureAndDetect(ros::ServiceClient& client)
{
  std_srvs::Trigger srv;
  if (client.call(srv)) {
    ROS_INFO("Service call successful: %s", srv.response.message.c_str());
  } else {
    ROS_ERROR("Failed to call service capture_and_detect");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_capture_client");
  ros::NodeHandle nh;

  // 创建服务客户端
  ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("capture_and_detect");

  // 订阅 BoundingBox 消息
  ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 1, boundingBoxCallback);

  // 调用服务进行拍照和检测
  captureAndDetect(client);

  ros::spin();
  return 0;
}