#include <fstream>

#include "yolo_detector.h"

#include <opencv_cpp_yolov5/BoundingBox.h>
#include <opencv_cpp_yolov5/BoundingBoxes.h>

namespace {// 好家伙，匿名的namespace
    inline cv::Mat format_yolov5(const cv::Mat &source) {
        int col = source.cols;// 获取输入图像的列数（宽度）
        int row = source.rows;// 获取输入图像的行数（高度）
        int _max = std::max(col, row);// 创建一个正方形的图像，挺好

        // 创建一个大小为 _max x _max 的黑色图像（所有像素值为0）
        // zeros函数的声明如下：(目的是创建一个全0的图像)，这里使用的是第一个重载
        /*
            static Mat zeros(int rows, int cols, int type); // 创建一个rows行cols列的全
            static Mat zeros(Size size, int type); // 创建一个size大小的
            static Mat zeros(int ndims, const int* sizes, int type); // 创建一个ndims维度的
            static Mat zeros(const std::vector<int>& sizes, int type) ;// 创建一个sizes维度的
        */
        // CV_8UC3表示8位无符号整数，3通道
        cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);

        // 将输入图像复制到 result 图像的左上角，实现方式是利用ROI，
        // 把source的内容复制到result的ROI区域，ROI位于result的左上角
        // ROI指的是Region of Interest，
        // 中文叫感兴趣区域
        // 在识别中，我们只关心图像的某一部分，而不是整个图像
        // 这个部分就是ROI
        // 一般位于图像的左上角
        source.copyTo(result(cv::Rect(0, 0, col, row)));
        return result;
    }
}

YoloDetector::YoloDetector(const std::string& class_list_path, const std::string& yolo_path, 
                            bool use_cuda=false, float nms_threshold=0.4, float confidence_threshold=0.7, float score_threshold=0.8,
                            float yolo_input_width=640.0, float yolo_input_height=640.0) : 
                            nms_threshold_(nms_threshold), confidence_threshold_(confidence_threshold), score_threshold_(score_threshold),
                            yolo_input_width_(yolo_input_width), yolo_input_height_(yolo_input_height), 
                            num_classes(get_num_classes(class_list_path)) {
    // 这些函数声明下文可见
    load_class_list(class_list_path);
    load_yolo( yolo_path, use_cuda);
    ROS_INFO("YoloDetector initialized");
}// 逆天的构造函数

size_t YoloDetector::get_num_classes(const std::string& class_list_path) {
    // 这里就是获取类别数量的函数
    std::ifstream ifs(class_list_path);// 这是C++的文件输入流，用于读取文件
    if (!ifs.is_open()) {
        ROS_ERROR("Failed to open class list file: %s", class_list_path.c_str());
        ros::shutdown();
    }

    std::string line;// 一行一行读
    size_t i = 0;
    while (std::getline(ifs, line)) {
        if (line.empty()) {
            continue;
        }
        i++;
    }
    ifs.close();
    return i;
}

void YoloDetector::load_class_list(const std::string& class_list_path) {
    // 作用是将string转换为C风格的字符串，目的是为了和C语言的函数兼容？
    for(int i=0;i<MAX_NUM_CLASSES;i++) {
        ROS_INFO("original class_list[%d]: %s", i, this->class_list[i].c_str());
    }
    std::ifstream ifs(class_list_path);
    if (!ifs.is_open()) {
        ROS_ERROR("Failed to open class list file: %s", class_list_path.c_str());
        ros::shutdown();// 关闭节点
    }

    std::string line;
    size_t i = 0;

    while (std::getline(ifs, line)) {
        if (line.empty()) {
            continue;
        }
        if (i < MAX_NUM_CLASSES) {
            class_list[i] = line;
            ROS_INFO("Loaded class: %s", line.c_str());
        } else {
            // 如果类别数量超过了最大数量，就输出错误信息，然后关闭节点
            ROS_ERROR("Number of classes exceeds the maximum number of classes: %lu",MAX_NUM_CLASSES);
            ros::shutdown();
        }
        i++;
    }
    ROS_INFO("Loaded %lu classes", i);
    ifs.close();
}


// 加载YOLO模型， 不知道怎么用
void YoloDetector::load_yolo(const std::string &net_path, bool use_cuda) {
    yolo = cv::dnn::readNet(net_path);
    if (yolo.empty()) {
        ROS_ERROR("Failed to load YOLO model: %s", net_path.c_str());
        ros::shutdown();
    }
    if (use_cuda) {// use？
        yolo.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        yolo.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
}



void YoloDetector::detect(cv::Mat& image, std::vector<Detection>& output) {
    cv::Mat blob;// 创建一个Mat对象blob，用于存放YOLO模型的输入

    auto input_image = format_yolov5(image);// 将输入图像转换为YOLO模型的输入

    // blobFromImage函数的声明如下：
    /*
        void blobFromImage(InputArray image, OutputArray blob, double scalefactor=1.0,
                        const Size& size = Size(), const Scalar& mean = Scalar(),
                        bool swapRB = true, bool crop = false, int ddepth = CV_32F);
    */ // 用于从图像创建4D blob，用于输入神经网络，这里的blob是一个4维的Mat对象
    
    // 每个blob包含一个图像，blob的尺寸是固定的，这里是640x640
    // 把每个像素点的值缩放到0-1之间，因为blob的值是浮点数，范围是0-1，原来输入图像的像素值是0-255
    // 这里的均值是Scalar()，表示没有均值
    cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(yolo_input_width_, yolo_input_height_), cv::Scalar(), true, false);
    //                     输入图像， 输出blob，缩放因子，   输出blob的尺寸，均值，交换RB通道，                            裁剪，输出blob的深度

    // RB通道是指红色和蓝色通道，这里交换是因为OpenCV中的图像是BGR格式，而YOLO模型是RGB格式

    yolo.setInput(blob);// 设置YOLO模型的输入，前面应该加载模型

    std::vector<cv::Mat> outputs; // Mat的vector，用于存放YOLO模型的输出

    yolo.forward(outputs, yolo.getUnconnectedOutLayersNames());// 前向传播，获取YOLO模型的输出
    // 有这几种传播方向：
    // 1. 前向传播：从输入层到输出层， 用于推理
    // 2. 反向传播：从输出层到输入层，用于训练
    // 3. 双向传播：从输入层到输出层，再从输出层到输入层

    // 计算缩放因子，用于将YOLO模型的输出映射到输入图像
    float x_factor = input_image.cols / yolo_input_width_;
    float y_factor = input_image.rows / yolo_input_height_;


    float *data = (float*) outputs[0].data;// 获取YOLO模型的输出数据
    // utputArrayOfArrays类型的outputs是一个vector，里面存放了YOLO模型的输出，可以用的成员函数有data()，size()，empty()等
    // data包含了所有的输出数据，是一个一维数组，每个元素是一个float类型的数值，表示一个目标的信息，包括类别ID，置信度，矩形框的位置等
    // 中心x, 中心y, 宽度, 高度, 置信度, 类别1的概率, 类别2的概率, ...
    // [x, y, w, h, confidence, class1, class2, ...]


    static const int dimensions = num_classes + 5;// 维度是类别数量加5
    // 就如训练时候的class_num + 5

    static const int rows = 25200;// 25200是YOLO模型的输出的行数, 

    // 遍历YOLO模型的输出，获取检测结果，保存到output中（就这三）
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i) {

        // 此时data指向第i个目标的信息
        float confidence = data[4];// 第五个元素
        if (confidence >= confidence_threshold_) {// 置信度大于阈值，开始判断

            float * classes_scores = data + 5;// data+5是类别概率的起始位置

            cv::Mat scores(1, num_classes, CV_32FC1, classes_scores);// 创建一个Mat对象scores，
            //用于存放类别概率, 
            //CV_32FC1表示32位浮点数,  1通道表示灰度图像, 3表示彩色, 4表示带透明度

            cv::Point class_id;
            double max_class_score;// 最大类别概率

            // void minMaxLoc(InputArray src, double* minVal, double* maxVal, Point* minLoc = 0, Point* maxLoc = 0, InputArray mask = noArray());
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);// 获取最大类别概率和类别ID

            if (max_class_score > score_threshold_) {
            
                // 将检测结果保存到output中
                confidences.push_back(confidence);
                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];

                // 计算矩形框的位置
                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);

                // 矩形框的宽度和高度
                int width = int(w * x_factor);
                int height = int(h * y_factor);

                // 将矩形框保存到boxes中
                boxes.push_back(cv::Rect(left, top, width, height));// 这里获得的长度就和原来的图像一样了
            }
        }

        // data指向下一个目标的信息
        data += dimensions;
    }

    std::vector<int> nms_result;// 非极大值抑制的结果（NMS）

    // 非极大值抑制，用于去除重叠的矩形框，只保留置信度最高的矩形框，输出结果保存到nms_result中
    cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, nms_result);

    // 将NMS的结果保存到output中
    for (int i=0;i < nms_result.size(); i++) {
        int idx = nms_result[i];ss
        // 将检测结果保存到output中
        
        // 这里的class_ids[idx]是类别ID，confidences[idx]是置信度，boxes[idx]是矩形框
        output.emplace_back(class_ids[idx], confidences[idx], boxes[idx]);
    }
}


// 检测回调函数，当接收到图像时调用
void YoloDetector::image_cb(const sensor_msgs::ImageConstPtr& msg, ros::Publisher& bbox_pub, image_transport::Publisher& image_pub) {
    //                                                   图像消息     发布检测结果的ROS话题          发布带有检测结果的图像的ROS话题
    #ifdef RECORD_FPS// 记录FPS，用于性能测试
    auto start_time = std::chrono::steady_clock::now();// 记录开始时间
    #endif

    cv_bridge::CvImagePtr cv_ptr;// 创建一个cv_bridge::CvImagePtr对象cv_ptr，用于存放图像消息

    try {// 尝试将图像消息转换为OpenCV图像
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }// 如果出现异常，输出错误信息，然后返回

    cv::Mat frame = cv_ptr->image;// 获取图像
    std::vector<Detection> output;// 创建一个vector，用于存放检测结果
    detect(frame, output);// 检测目标，将结果保存到output中

    opencv_cpp_yolov5::BoundingBoxes bbox_msg;// 创建一个opencv_cpp_yolov5::BoundingBoxes对象bbox_msg，用于发布检测结果
    bbox_msg.header = msg->header;// 消息头

    // 遍历检测结果，将检测结果保存到bbox_msg中，并在图像上绘制矩形框
    int detections = output.size();
    for (int i=0; i< detections; i++) {
        auto detection = output[i];// 获取检测结果
        auto box = detection.box;// 获取矩形框
        auto classId = detection.class_id;// 获取类别ID

        
        opencv_cpp_yolov5::BoundingBox bbox;// 创建一个opencv_cpp_yolov5::BoundingBox对象bbox，用于保存检测结果
        bbox.Class = class_list[detection.class_id];
        bbox.probability = detection.confidence;

        // 矩形框的位置

        // 右上角
        bbox.xmin = box.x;
        bbox.ymin = box.y;

        // 左下角
        bbox.xmax = box.x + box.width;
        bbox.ymax = box.y + box.height;
        bbox_msg.bounding_boxes.push_back(bbox);

        // 绘制矩形框，box_colors在yolo_detector.h中定义,一个匿名的namespace
        // 不理解这里namespace的作用： ？
        const auto color = box_colors[classId % box_colors.size()];// 防止数组越界

        // 绘制矩形框
        // 在frame以box为参数绘制矩形框，颜色是color，线宽是3
        cv::rectangle(frame, box, color, 3);

        // 在矩形框上绘制类别和置信度，画在矩形框的左上角
        cv::rectangle(frame, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);

        // 创建一个字符串label，用于显示类别和置信度
        std::string label = class_list[classId] + " (" + std::to_string(detection.confidence) + ")";

        // 在frame上绘制label，位置是box的左上角，字体是cv::FONT_HERSHEY_SIMPLEX，大小是0.5，颜色是黑色
        cv::putText(frame, label.c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }

    // 发布检测结果
    bbox_pub.publish(bbox_msg);

    // 发布带有检测结果的图像
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    image_pub.publish(img_msg);


    // 如下获取消息
    /*
    #include <ros/ros.h>
    #include <opencv_cpp_yolov5/BoundingBoxes.h>
    #include <opencv_cpp_yolov5/BoundingBox.h>

    void boundingBoxesCallback(const opencv_cpp_yolov5::BoundingBoxes::ConstPtr& msg) {
    ROS_INFO("Received %lu bounding boxes", msg->bounding_boxes.size());

    for (const auto& bbox : msg->bounding_boxes) {
        ROS_INFO("Class: %s, Probability: %f, BBox: [%ld, %ld, %ld, %ld]",
                 bbox.Class.c_str(), bbox.probability, bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax);
    }
    }
    */

    #ifdef RECORD_FPS
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = end_time - start_time;
    float fps = 1.0 / elapsed_time.count();
    ROS_INFO("FPS: %f", fps);
    #endif
}

// 析构函数
YoloDetector::~YoloDetector() {
    ROS_INFO("YoloDetector destroyed");
}