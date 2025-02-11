#include <fstream>  // 包含标准文件流库，用于文件操作
#include "yolo_detector.h"  // 包含自定义的 YOLO 检测器头文件，定义了 YoloDetector 类

#include <opencv_cpp_yolov5/BoundingBox.h>  // 包含目标检测结果的自定义消息头文件
#include <opencv_cpp_yolov5/BoundingBoxes.h>  // 包含多个目标检测结果的自定义消息头文件

namespace {
    // 命名空间匿名，其中包含一个辅助函数，用于格式化图像为 YOLOv5 所需的正方形格式
    inline cv::Mat format_yolov5(const cv::Mat &source) {  // 用于将图像调整为 YOLOv5 输入尺寸的辅助函数
        int col = source.cols;  // 获取图像的宽度（列数）
        int row = source.rows;  // 获取图像的高度（行数）
        int _max = std::max(col, row);  // 计算图像的最大边长
        cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);  // 创建一个正方形的黑底矩阵
        source.copyTo(result(cv::Rect(0, 0, col, row)));  // 将原图像拷贝到正方形矩阵的左上角
        return result;  // 返回格式化后的图像
    }
}

YoloDetector::YoloDetector(  // YoloDetector 构造函数，用于初始化 YOLO 检测器
    const std::string& class_list_path, const std::string& yolo_path,
    bool use_cuda /*= false*/, float nms_threshold /*= 0.4*/,
    float confidence_threshold /*= 0.7*/, float score_threshold /*= 0.8*/,
    float yolo_input_width /*= 640.0*/, float yolo_input_height /*= 640.0*/) :
    nms_threshold_(nms_threshold), confidence_threshold_(confidence_threshold), score_threshold_(score_threshold),
    yolo_input_width_(yolo_input_width), yolo_input_height_(yolo_input_height),
    num_classes(get_num_classes(class_list_path)) {  // 初始化成员变量和类数量

    load_class_list(class_list_path);  // 加载类标签列表
    load_yolo(yolo_path, use_cuda);  // 加载 YOLO 模型
    ROS_INFO("YoloDetector initialized");  // 输出初始化完成的日志信息
}

size_t YoloDetector::get_num_classes(const std::string& class_list_path) {  // 获取类的数量
    std::ifstream ifs(class_list_path);  // 打开类标签文件
    if (!ifs.is_open()) {  // 如果文件无法打开
        ROS_ERROR("Failed to open class list file: %s", class_list_path.c_str());  // 输出错误信息
        ros::shutdown();  // 关闭 ROS 节点
    }

    std::string line;  // 用于存储每一行类标签
    size_t i = 0;  // 计数器，用于统计类的数量

    // 遍历文件中的每一行
    while (std::getline(ifs, line)) {
        if (line.empty()) {  // 如果行为空
            continue;  // 跳过
        }
        i++;  // 类的数量加一
    }
    ifs.close();  // 关闭文件
    return i;  // 返回类的数量
}

void YoloDetector::load_class_list(const std::string& class_list_path) {  // 加载类标签列表
    for(int i=0; i<MAX_NUM_CLASSES; i++) {  // 打印原始类标签（调试信息）
        ROS_INFO("original class_list[%d]: %s", i, this->class_list[i].c_str());
    }

    std::ifstream ifs(class_list_path);  // 打开类标签文件
    if (!ifs.is_open()) {  // 如果文件无法打开
        ROS_ERROR("Failed to open class list file: %s", class_list_path.c_str());  // 输出错误信息
        ros::shutdown();  // 关闭 ROS 节点
    }

    std::string line;  // 用于存储每一行类标签
    size_t i = 0;  // 计数器，用于索引类标签数组

    // 遍历文件中的每一行
    while (std::getline(ifs, line)) {
        if (line.empty()) {  // 如果行为空
            continue;  // 跳过
        }
        if (i < MAX_NUM_CLASSES) {  // 如果类的数量未超过最大限制
            class_list[i] = line;  // 存储类标签
            ROS_INFO("Loaded class: %s", line.c_str());  // 输出加载的类标签
        } else {  // 如果类的数量超过最大限制
            ROS_ERROR("Number of classes exceeds the maximum number of classes: %lu", MAX_NUM_CLASSES);
            ros::shutdown();  // 关闭 ROS 节点
        }
        i++;  // 类的数量加一
    }
    ROS_INFO("Loaded %lu classes", i);  // 输出加载的类数量
    ifs.close();  // 关闭文件
}

void YoloDetector::load_yolo(const std::string &net_path, bool use_cuda) {  // 加载 YOLO 模型
    yolo = cv::dnn::readNet(net_path);  // 读取 YOLO 模型
    if (yolo.empty()) {  // 如果模型加载失败
        ROS_ERROR("Failed to load YOLO model: %s", net_path.c_str());  // 输出错误信息
        ros::shutdown();  // 关闭 ROS 节点
    }

    if (use_cuda) {  // 如果使用 CUDA 加速
        yolo.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);  // 设置为 CUDA 后端
        yolo.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);  // 设置为 CUDA 设备
    }
}

void YoloDetector::detect(cv::Mat& image, std::vector<Detection>& output) {  // 目标检测核心函数
    cv::Mat blob;  // 定义用于模型输入的 blob
    auto input_image = format_yolov5(image);  // 格式化输入图像为 YOLOv5 格式

    // 将图像转换为 blob，归一化并调整大小
    cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(yolo_input_width_, yolo_input_height_),
                           cv::Scalar(), true, false);

    yolo.setInput(blob);  // 设置模型的输入
    std::vector<cv::Mat> outputs;  // 定义输出层的结果容器
    yolo.forward(outputs, yolo.getUnconnectedOutLayersNames());  // 执行模型推理，获取输出

    float x_factor = input_image.cols / yolo_input_width_;  // 计算宽度缩放因子
    float y_factor = input_image.rows / yolo_input_height_;  // 计算高度缩放因子

    float *data = (float*) outputs[0].data;  // 获取模型输出的原始数据指针

    static const int dimensions = num_classes + 5;  // 每个检测结果的维数（4个坐标 + 1个置信度 + 类别概率）
    static const int rows = 25200;  // 输出结果的行数

    std::vector<int> class_ids;  // 用于存储检测到的类别ID
    std::vector<float> confidences;  // 用于存储检测结果的置信度
    std::vector<cv::Rect> boxes;  // 用于存储检测到的目标边界框

    // 遍历模型输出的所有行
    for (int i = 0; i < rows; ++i) {
        float confidence = data[4];  // 获取当前行的置信度
        if (confidence >= confidence_threshold_) {  // 如果置信度超过阈值
            float * classes_scores = data + 5;  // 获取类别概率数组
            cv::Mat scores(1, num_classes, CV_32FC1, classes_scores);  // 创建类别概率矩阵
            cv::Point class_id;  // 定义用于存储类别ID的变量
            double max_class_score;  // 定义用于存储最大类别概率的变量
            // 找出类别概率的最大值及其位置
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

            if (max_class_score > score_threshold_) {  // 如果最大类别概率超过阈值
                confidences.push_back(confidence);  // 存储置信度
                class_ids.push_back(class_id.x);  // 存储类别ID
                // 计算边界框的位置和大小
                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];
                int left = int((x - 0.5 * w) * x_factor);  // 边界框左上角的 x 坐标
                int top = int((y - 0.5 * h) * y_factor);  // 边界框左上角的 y 坐标
                int width = int(w * x_factor);  // 边界框的宽度
                int height = int(h * y_factor);  // 边界框的高度
                boxes.push_back(cv::Rect(left, top, width, height));  // 存储边界框
            }
        }
        data += dimensions;  // 指向下一组预测结果
    }

    // 使用非极大值抑制（NMS）过滤边界框
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, nms_result);

    // 根据 NMS 结果重构检测结果
    for (int i=0; i < nms_result.size(); i++) {
        int idx = nms_result[i];
        output.emplace_back(class_ids[idx], confidences[idx], boxes[idx]);
    }
}

void YoloDetector::image_cb(  // 图像回调函数，处理 ROS 图像话题并发布检测结果
    const sensor_msgs::ImageConstPtr& msg, ros::Publisher& bbox_pub, image_transport::Publisher& image_pub) {

    #ifdef RECORD_FPS  // 如果定义了 FPS 记录
    auto start_time = std::chrono::steady_clock::now();  // 记录开始时间
    #endif

    cv_bridge::CvImagePtr cv_ptr;  // 定义用于存储 OpenCV 图像的指针
    try {
        // 将 ROS 图像消息转换为 OpenCV 格式
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());  // 如果转换失败，输出错误信息
        return;
    }

    cv::Mat frame = cv_ptr->image;  // 获取图像数据
    std::vector<Detection> output;  // 用于存储检测结果
    detect(frame, output);  // 调用目标检测函数进行检测

    opencv_cpp_yolov5::BoundingBoxes bbox_msg;  // 定义用于存储检测结果的消息对象
    bbox_msg.header = msg->header;  // 设置消息的头信息

    int detections = output.size();  // 获取检测结果的数量
    for (int i=0; i< detections; i++) {
        auto detection = output[i];  // 获取单个检测结果
        auto box = detection.box;  // 获取边界框
        auto classId = detection.class_id;  // 获取类别ID

        opencv_cpp_yolov5::BoundingBox bbox;  // 定义单个边界框的消息对象
        bbox.Class = class_list[detection.class_id];  // 设置类别名称
        bbox.probability = detection.confidence;  // 设置置信度
        bbox.xmin = box.x;  // 设置边界框左上角的 x 坐标
        bbox.ymin = box.y;  // 设置边界框左上角的 y 坐标
        bbox.xmax = box.x + box.width;  // 设置边界框右下角的 x 坐标
        bbox.ymax = box.y + box.height;  // 设置边界框右下角的 y 坐标
        bbox_msg.bounding_boxes.push_back(bbox);  // 将边界框添加到消息对象中

        // 绘制边界框和类别信息
        const auto color = box_colors[classId % box_colors.size()];  // 获取绘制边界框的颜色
        cv::rectangle(frame, box, color, 3);  // 绘制边界框
        cv::rectangle(frame, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y),
                      color, cv::FILLED);  // 绘制背景填充矩形
        // 构建类别和置信度信息
        std::string label = class_list[classId] + " (" + std::to_string(detection.confidence) + ")";
        // 在图像上绘制类别和置信度信息
        cv::putText(frame, label.c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 0));
    }
    bbox_pub.publish(bbox_msg);  // 发布检测结果消息

    // 将绘制结果后的 OpenCV 图像转换为 ROS 图像消息并发布
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    image_pub.publish(img_msg);

    #ifdef RECORD_FPS  // 如果定义了 FPS 记录
    auto end_time = std::chrono::steady_clock::now();  // 记录结束时间
    std::chrono::duration<double> elapsed_time = end_time - start_time;  // 计算时间差
    float fps = 1.0 / elapsed_time.count();  // 计算 FPS
    ROS_INFO("FPS: %f", fps);  // 输出 FPS 信息
    #endif
}

YoloDetector::~YoloDetector() {  // YoloDetector 析构函数
    ROS_INFO("YoloDetector destroyed");  // 输出销毁信息
}