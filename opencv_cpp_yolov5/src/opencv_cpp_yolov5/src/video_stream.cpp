#include <ros/ros.h>                 // 包含ROS核心头文件，用于ROS节点的初始化和通信
#include <sensor_msgs/Image.h>       // 包含ROS图像消息的标准头文件
#include <cv_bridge/cv_bridge.h>     // 包含用于OpenCV和ROS桥接的头文件
#include <opencv2/opencv.hpp>        // 包含OpenCV核心头文件，用于图像处理和视频读取

int main(int argc, char** argv) {    // 主函数入口
    ros::init(argc, argv, "video_stream_publisher");  // 初始化ROS节点，并命名为"video_stream_publisher"
    ros::NodeHandle nh("~");          // 创建一个私有命名空间的节点句柄，用于访问参数

    std::string video_path;          // 定义视频文件路径参数
    int frame_height, frame_width;   // 定义视频帧高度和宽度参数

    // 从ROS参数服务器加载参数，如果没有设置，则使用默认值
    nh.param("video_path", video_path, std::string("/path/to/video.mp4"));
    nh.param("frame_height", frame_height, 480);
    nh.param("frame_width", frame_width, 640);

    // 打印参数值到ROS日志（INFO级别）
    ROS_INFO("video_path: %s", video_path.c_str());
    ROS_INFO("frame_height: %d", frame_height);
    ROS_INFO("frame_width: %d", frame_width);

    cv::VideoCapture cap(video_path); // 使用OpenCV打开视频文件
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);  // 设置视频捕获的高度
    cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);    // 设置视频捕获的宽度

    if (!cap.isOpened()) {            // 如果无法打开视频文件
        ROS_ERROR("Could not open video file.");  // 打印错误信息到ROS日志（ERROR级别）
        return 1;                  // 退出程序，返回错误码1
    }

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("image_raw", 1);  // 创建一个发布者，发布话题名为"image_raw"，消息类型为sensor_msgs::Image，队列长度为1

    cv::Mat frame;                 // 定义一个OpenCV的Mat对象，用于存储视频帧
    sensor_msgs::ImagePtr msg;     // 定义一个ROS图像消息指针

    ros::Rate loop_rate(20);       // 设置主循环的频率为每秒20次，可以根据需要调整

    while (nh.ok()) {              // 主循环，持续运行直到节点被关闭
        cap >> frame;              // 从视频捕获中读取一帧到Mat对象

        if (!frame.empty()) {       // 如果帧不为空
            // 使用cv_bridge将OpenCV的Mat转换为ROS的Image消息
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);       // 发布图像消息到话题
            cv::waitKey(1);         // 等待1毫秒，处理视频帧（可选，用于显示或延迟）
        } else {                    // 如果帧为空（视频结束）
            ROS_WARN("End of video file reached. restarting...");  // 打印警告信息到ROS日志（WARN级别）
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);  // 重置视频到第一帧
            continue;               // 跳过剩余循环并重新开始
        }

        ros::spinOnce();            // 处理ROS的回调和订阅
        loop_rate.sleep();          // 根据设定的频率延时，保持循环频率
    }

    return 0;                       // 程序正常结束，返回0
}