// 霍夫圆上基础上修改，无法运行，缺少位置参数
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>


// 这个头文件包含了QRCodeDetector类，用于检测和解码二维码
// 在msg中自定义类型，配置CMakeLists.txt和package.xml，catkin_make后生成
#include <opencv_cpp_yolov5/QRCodeInfo.h>
#include <opencv_cpp_yolov5/QRCodeDetectResult.h>

// 包含YOLO检测器头文件,from yolo_detector.cpp(学长找门代码)
#include <fstream>

#include "yolo_detector.h"

//#include <opencv_cpp_yolov5/BoundingBox.h>
//#include <opencv_cpp_yolov5/BoundingBoxes.h>




// namespace {
//     // 定义一个内联函数 format_yolov5，用于格式化输入图像
//     inline cv::Mat format_yolov5(const cv::Mat &source) {
//         // 获取输入图像的列数（宽度）和行数（高度）
//         int col = source.cols;
//         int row = source.rows;
        
//         // 获取列数和行数中的较大值
//         int _max = std::max(col, row);
        
//         // 创建一个大小为 _max x _max 的黑色图像（所有像素值为0）
//         cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
        
//         // 将输入图像复制到 result 图像的左上角
//         source.copyTo(result(cv::Rect(0, 0, col, row)));
        
//         // 返回格式化后的图像
//         return result;
//     }
// }

// // YoloDetector构造函数，初始化YOLO检测器对象
// YoloDetector::YoloDetector(const std::string& class_list_path, const std::string& yolo_path, 
//     bool use_cuda, float nms_threshold, float confidence_threshold, float score_threshold,
//     float yolo_input_width, float yolo_input_height) : 
//     // 初始化成员变量
//     nms_threshold_(nms_threshold), 
//     confidence_threshold_(confidence_threshold), 
//     score_threshold_(score_threshold),
//     yolo_input_width_(yolo_input_width), 
//     yolo_input_height_(yolo_input_height), 
//     num_classes(get_num_classes(class_list_path)) {
//     // 加载类列表
//     load_class_list(class_list_path);
//     // 加载YOLO模型
//     load_yolo(yolo_path, use_cuda);
//     // 输出初始化完成的信息
//     ROS_INFO("YoloDetector initialized");
// }

// 发布二维码检测结果的ROS话题
ros::Publisher qrcode_pub;
// 发布带有检测结果图像的ROS话题
image_transport::Publisher image_pub;
// 是否发布带有检测结果的图像
bool pubImg;

/*
// 检测回调函数，当接收到图像时调用
double dp, minDist, param1, param2;
// 最小半径和最大半径
int minRadius, maxRadius;
*/





// 图像回调函数，当接收到图像时调用
void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        // 将ROS图像消息转换为OpenCV图像
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        // 如果出现异常，输出错误信息
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 将图像转换为灰度图
    // 灰度图是单通道图像，只有亮度信息，没有颜色信息
    // 二维码检测不需要颜色信息
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    /*
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, dp, minDist,
                     param1, param2, minRadius, maxRadius);
    */

    // 创建QRCodeDetector对象
    cv::QRCodeDetector qrDecoder;
    // 用于存储二维码的顶点
    std::vector<cv::Point> points;
    // 检测并解码二维码
    std::string data = qrDecoder.detectAndDecode(gray, points);

    // 创建二维码检测结果消息
    // 在opencv_cpp_yolov5/msg/QRCodeInfo.msg中定义
    /* 定义为：
    string data
    geometry_msgs/Point[] points
    */
    // geometry_msgs/Point是ROS中的消息类型，用于表示3D空间中的点
    // 由于二维码是2D的，所以只需要两个坐标值
    opencv_cpp_yolov5::QRCodeDetectResult results;
    results.header = msg->header;//消息头
    //消息头指的是消息的元数据，包括时间戳、消息来源、消息序列号等
    results.height = msg->height;//消息高度
    results.width = msg->width;//消息宽度



    // 如果检测到二维码
    if (!data.empty())
    {

        // 创建二维码信息消息
        opencv_cpp_yolov5::QRCodeInfo qrcode_info;
        qrcode_info.data = data;
        qrcode_info.points = points;
        results.qrcodes.push_back(qrcode_info);

        // 如果需要发布带有检测结果的图像
        if (pubImg)
        {
            // 在图像上绘制二维码的顶点
            for (size_t i = 0; i < points.size(); i++)
            {
                cv::line(cv_ptr->image, points[i], points[(i + 1) % points.size()], cv::Scalar(255, 0, 0), 3);
            }
        }
    }

    // 发布二维码检测结果
    qrcode_pub.publish(results);

    // 如果需要发布带有检测结果的图像
    if (pubImg)
    {
        image_pub.publish(cv_ptr->toImageMsg());
    }
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "qrcode_detector_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    // 获取参数pubImg，默认为true
    nh.param("pubImg", pubImg, true);

    ROS_WARN("pubImg: %d", pubImg);

    // 初始化发布二维码检测结果的ROS话题
    qrcode_pub = nh.advertise<opencv_cpp_yolov5::QRCodeDetectResult>("/opencv_cpp_yolov5/qrcode_detect_result", 1);
    // 如果需要发布带有检测结果的图像，初始化相应的ROS话题
    if (pubImg)
    {
        image_pub = it.advertise("/opencv_cpp_yolov5/qrcode_detect_result_img", 1);
    }

    // 订阅摄像头图像话题，并设置回调函数
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, image_cb);

    // 进入ROS事件循环
    ros::spin();
    return 0;
}