// 这个文件好像是自己写的，不是yolov5的源码？
// 非常重要的文件
#ifndef YOLO_DETECTOR_H_
#define YOLO_DETECTOR_H_

#include <array>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace {
    constexpr size_t MAX_NUM_CLASSES = 11;// 11个类别
    const std::array<cv::Scalar, MAX_NUM_CLASSES> box_colors = {
        // HACK rescale this array to match `MAX_NUM_CLASSES`
        // 这是一个颜色数组，用于绘制不同类别的边框
        cv::Scalar(255, 255, 0),
        // 黄色
        cv::Scalar(0, 255, 255),
        // 青色
        cv::Scalar(255, 0, 255),
        // 紫色
        cv::Scalar(0, 255, 0),
        // 绿色
        cv::Scalar(255, 255, 0),
        // 下面都是黄色
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 255, 0),
    };
}

// YoloDetector类，用于检测YOLO目标
class YoloDetector {
public:
    // 构造函数，初始化YOLO检测器对象
    /*
        这些函数说人话就是分别代表：
        1. 类别列表文件路径
        2. YOLO模型文件路径
        3. 是否使用CUDA
        4. NMS阈值
        5. 置信度阈值
        6. 分数阈值
        7. YOLO输入图像宽度
        8. YOLO输入图像高度
    */
    YoloDetector(const std::string& class_list_path, const std::string& yolo_path, 
                bool use_cuda, float nms_threshold, float confidence_threshold, float score_threshold,
                float yolo_input_width, float yolo_input_height);
    
    // 析构函数
    ~YoloDetector();// 注意一下在哪里调用了这个析构函数

    void image_cb(const sensor_msgs::ImageConstPtr& msg, ros::Publisher& bbox_pub, image_transport::Publisher& image_pub);
    // 定义一个检测函数，用于检测目标
    struct Detection {
        int class_id;

        double confidence;
        // cv::Rect是OpenCV中用于表示矩形的类
        cv::Rect box;// 矩形框, 用于表示检测到的目标的位置,这里用的是左上角和右下角的点确定一个矩形框
        /*
            Rect里面有四个参数，分别是x, y, width, height
            声明如下：
        */

        Detection() {}  // no idea whether we should leave default constructor or not
                        // yes, I also have no idea
                        
        // why you need a default constructor here?
        Detection(int class_id, double confidence, cv::Rect box) : box(box), class_id(class_id), confidence(confidence) {}
        // 这里的构造包含参数，关注其实现位置
        /*
            1. 类别ID
            2. 置信度
            3. 矩形框
        */
    };

protected:
    void detect(cv::Mat &image, std::vector<Detection> &output);
    const size_t num_classes;
    std::array<std::string, MAX_NUM_CLASSES> class_list;
    cv::dnn::Net yolo;
    const float nms_threshold_;
    const float confidence_threshold_;
    const float score_threshold_;
    const float yolo_input_width_;
    const float yolo_input_height_;

    size_t get_num_classes(const std::string& class_list_path);
    void load_class_list(const std::string& class_list_path);
    void load_yolo(const std::string &net_path, bool use_cuda);
};

#endif  // YOLO_DETECTOR_H_