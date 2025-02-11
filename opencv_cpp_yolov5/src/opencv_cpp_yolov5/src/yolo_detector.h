#ifndef YOLO_DETECTOR_H_  // 防止头文件重复包含的宏定义（头文件保护）
#define YOLO_DETECTOR_H_

#include <array>  // 包含 C++ 标准库中的 array 容器支持
#include <string>  // 包含 C++ 标准库中的字符串支持
#include <vector>  // 包含 C++ 标准库中的动态数组支持

#include <opencv2/opencv.hpp>  // 包含 OpenCV 核心头文件，用于图像处理

#include <ros/ros.h>  // 包含 ROS 核心头文件，用于 ROS 节点和通信
#include <ros/package.h>  // 包含 ROS 包管理支持
#include <sensor_msgs/image_encodings.h>  // 包含 ROS 图像编码类型支持
#include <image_transport/image_transport.h>  // 包含 ROS 图像传输支持
#include <cv_bridge/cv_bridge.h>  // 包含 ROS 和 OpenCV 之间的图像桥接支持

namespace {  // 匿名命名空间，用于定义全局常量
    constexpr size_t MAX_NUM_CLASSES = 11;  // 定义最大类别数量为 11
    const std::array<cv::Scalar, MAX_NUM_CLASSES> box_colors = {  // 定义用于绘制边界框的颜色数组
        // HACK rescale this array to match `MAX_NUM_CLASSES`  // 提示：需要根据类别数量调整颜色数组
        cv::Scalar(255, 255, 0),  // 颜色 1：黄色
        cv::Scalar(0, 255, 255),  // 颜色 2：青色
        cv::Scalar(255, 0, 255),  // 颜色 3：品红
        cv::Scalar(0, 255, 0),  // 颜色 4：绿色
        cv::Scalar(255, 255, 0),  // 颜色 5：黄色
        cv::Scalar(255, 255, 0),  // 颜色 6：黄色
        cv::Scalar(255, 255, 0),  // 颜色 7：黄色
        cv::Scalar(255, 255, 0),  // 颜色 8：黄色
        cv::Scalar(255, 255, 0),  // 颜色 9：黄色
        cv::Scalar(255, 255, 0),  // 颜色 10：黄色
        cv::Scalar(255, 255, 0),  // 颜色 11：黄色
    };
}

class YoloDetector {  // 定义 YoloDetector 类
public:
    // 构造函数，用于初始化 YOLO 检测器
    YoloDetector(
        const std::string& class_list_path,  // 类标签文件路径
        const std::string& yolo_path,  // YOLO 模型文件路径
        bool use_cuda,  // 是否使用 CUDA 加速
        float nms_threshold,  // 非极大值抑制（NMS）阈值
        float confidence_threshold,  // 置信度阈值
        float score_threshold,  // 分数阈值
        float yolo_input_width,  // YOLO 输入宽度
        float yolo_input_height  // YOLO 输入高度
    );

    // 析构函数，用于释放资源
    ~YoloDetector();

    // 图像回调函数，用于处理 ROS 图像消息并发布检测结果
    void image_cb(
        const sensor_msgs::ImageConstPtr& msg,  // ROS 图像消息指针
        ros::Publisher& bbox_pub,  // 边界框结果发布者
        image_transport::Publisher& image_pub  // 可视化图像发布者
    );

    // 定义检测结果结构体
    struct Detection {
        int class_id;  // 类别 ID
        double confidence;  // 置信度
        cv::Rect box;  // 边界框

        // 默认构造函数
        Detection() {}
        // 带参数的构造函数
        Detection(int class_id, double confidence, cv::Rect box)
            : box(box), class_id(class_id), confidence(confidence) {}
    };

protected:
    // 目标检测核心函数
    void detect(cv::Mat &image, std::vector<Detection> &output);

    // 类别数量（常量）
    const size_t num_classes;
    // 类标签列表（数组）
    std::array<std::string, MAX_NUM_CLASSES> class_list;
    // YOLO 模型（OpenCV DNN 网络）
    cv::dnn::Net yolo;
    // 阈值参数（常量）
    const float nms_threshold_;
    const float confidence_threshold_;
    const float score_threshold_;
    const float yolo_input_width_;
    const float yolo_input_height_;

    // 获取类标签文件中的类别数量
    size_t get_num_classes(const std::string& class_list_path);
    // 加载类标签列表
    void load_class_list(const std::string& class_list_path);
    // 加载 YOLO 模型
    void load_yolo(const std::string &net_path, bool use_cuda);
};

#endif  // YOLO_DETECTOR_H_