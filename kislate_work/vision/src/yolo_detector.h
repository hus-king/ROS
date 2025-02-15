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

    struct Detection {// 结构体，用于保存检测结果，比较重要的一个结构体
        int class_id;

        double confidence;
        // cv::Rect是OpenCV中用于表示矩形的类
        cv::Rect box;// 矩形框, 用于表示检测到的目标的位置,这里用的是左上角和右下角的点确定一个矩形框
        // 注意：只保留左上角的点的坐标
        /*
            Rect里面有四个参数，分别是x, y, width, height
        */

        Detection() {}  // no idea whether we should leave default constructor or not
                        // yes, I also have no idea

        // why you need a default constructor here?

        // The order of parameters in the list can be different from the order in the declaration
        Detection(int class_id, double confidence, cv::Rect box) : box(box), class_id(class_id), confidence(confidence) {}
        // 这里的构造包含参数，关注其实现位置
        /*
            1. 类别ID
            2. 置信度
            3. 矩形框
        */
    };

protected:
    // 定义一个检测函数，用于检测目标
    // 分别是输入图像和输出检测结果
    void detect(cv::Mat &image, std::vector<Detection> &output);

    const size_t num_classes;// 找到你了
    std::array<std::string, MAX_NUM_CLASSES> class_list;// 找到你了
    cv::dnn::Net yolo;// dnn, deep neural network, 深度神经网络
    // 这里的Net是OpenCV中用于表示神经网络的类,其声明如下：
    // (建议折叠)
    /*
        class Net
        {
        public:
            Net();
            Net(const String& model, const String& config = "", const String& framework = "");
            virtual ~Net();
            Net& operator=(const Net& other);
            Net(const Net& other);
            void setPreferableBackend(int backendId);
            void setPreferableTarget(int targetId);
            void setHalideScheduler(const String& scheduler);
            void forward(OutputArrayOfArrays outputBlobs, const std::vector<String>& outBlobNames = std::vector<String>());
            void forward(OutputArrayOfArrays outputBlobs, const std::vector<String>& outBlobNames = std::vector<String>()) const;
            void forward(OutputArrayOfArrays outputBlobs, const std::vector<String>& outBlobNames, int flags);
            void forward(OutputArrayOfArrays outputBlobs, const std::vector<String>& outBlobNames, int flags) const;
            void setInput(const Mat& blob, const String& name = "");
            void setInput(const std::vector<Mat>& blobs, const std::vector<String>& name = std::vector<String>());
            Mat getLayerId(const String& layer);
            std::vector<String> getLayerNames();
            std::vector<String> getUnconnectedOutLayersNames();
            std::vector<String> getUnconnectedOutLayers();
            std::vector<String> getLayerTypes();
            std::vector<String> getLayerNames(const String& type);
            std::vector<int> getLayersShapes(const std::vector<int>& layers, int netInputShape = 0);
            std::vector<int> getLayersShapes(const std::vector<String>& layers, int netInputShape = 0);
            std::vector<int> getLayersShapes(const std::vector<int>& layers, const std::vector<int>& netInputShape);
            std::vector<int> getLayersShapes(const std::vector<String>& layers, const std::vector<int>& netInputShape);
            void setPreferableBackend(int backendId);
            void setPreferableTarget(int targetId);
            void setHalideScheduler(const String& scheduler);
            void forward(OutputArrayOfArrays outputBlobs, const std::vector<String>& outBlobNames = std::vector<String>());
            void forward(OutputArrayOfArrays outputBlobs, const std::vector<String>& outBlobNames = std::vector<String>()) const;
            void forward(OutputArrayOfArrays outputBlobs, const std::vector
    */
    const float nms_threshold_;// 非极大值抑制阈值，用于去除重叠的边框
    const float confidence_threshold_;// 置信度阈值
    const float score_threshold_;// 分数阈值，更进一步的阈值
    const float yolo_input_width_;// YOLO输入图像宽度
    const float yolo_input_height_;// YOLO输入图像高度

    size_t get_num_classes(const std::string& class_list_path);// 获取类别数量
    void load_class_list(const std::string& class_list_path);// 加载类别列表
    void load_yolo(const std::string &net_path, bool use_cuda);// 加载YOLO模型
};

#endif  // YOLO_DETECTOR_H_