#include <fstream>

#include "yolo_detector.h"

#include <opencv_cpp_yolov5/BoundingBox.h>
#include <opencv_cpp_yolov5/BoundingBoxes.h>

namespace {
    inline cv::Mat format_yolov5(const cv::Mat &source) {
        int col = source.cols;
        int row = source.rows;
        int _max = std::max(col, row);
        cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
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
    load_class_list(class_list_path);
    load_yolo( yolo_path, use_cuda);
    ROS_INFO("YoloDetector initialized");
}

size_t YoloDetector::get_num_classes(const std::string& class_list_path) {
    std::ifstream ifs(class_list_path);
    if (!ifs.is_open()) {
        ROS_ERROR("Failed to open class list file: %s", class_list_path.c_str());
        ros::shutdown();
    }

    std::string line;
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
    for(int i=0;i<MAX_NUM_CLASSES;i++) {
        ROS_INFO("original class_list[%d]: %s", i, this->class_list[i].c_str());
    }
    std::ifstream ifs(class_list_path);
    if (!ifs.is_open()) {
        ROS_ERROR("Failed to open class list file: %s", class_list_path.c_str());
        ros::shutdown();
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
            ROS_ERROR("Number of classes exceeds the maximum number of classes: %lu",MAX_NUM_CLASSES);
            ros::shutdown();
        }
        i++;
    }
    ROS_INFO("Loaded %lu classes", i);
    ifs.close();
}

void YoloDetector::load_yolo(const std::string &net_path, bool use_cuda) {
    yolo = cv::dnn::readNet(net_path);
    if (yolo.empty()) {
        ROS_ERROR("Failed to load YOLO model: %s", net_path.c_str());
        ros::shutdown();
    }
    if (use_cuda) {
        yolo.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        yolo.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
}

void YoloDetector::detect(cv::Mat& image, std::vector<Detection>& output) {
    cv::Mat blob;
    auto input_image = format_yolov5(image);
    cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(yolo_input_width_, yolo_input_height_), cv::Scalar(), true, false);
    yolo.setInput(blob);
    std::vector<cv::Mat> outputs;
    yolo.forward(outputs, yolo.getUnconnectedOutLayersNames());

    float x_factor = input_image.cols / yolo_input_width_;
    float y_factor = input_image.rows / yolo_input_height_;

    float *data = (float*) outputs[0].data;

    static const int dimensions = num_classes + 5;
    static const int rows = 25200;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i) {
        float confidence = data[4];
        if (confidence >= confidence_threshold_) {
            float * classes_scores = data + 5;
            cv::Mat scores(1, num_classes, CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            if (max_class_score > score_threshold_) {
                confidences.push_back(confidence);
                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];
                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
        data += dimensions;
    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, nms_result);
    for (int i=0;i < nms_result.size(); i++) {
        int idx = nms_result[i];
        output.emplace_back(class_ids[idx], confidences[idx], boxes[idx]);
    }
}

void YoloDetector::image_cb(const sensor_msgs::ImageConstPtr& msg, ros::Publisher& bbox_pub, image_transport::Publisher& image_pub) {
    #ifdef RECORD_FPS
    auto start_time = std::chrono::steady_clock::now();
    #endif

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;
    std::vector<Detection> output;
    detect(frame, output);

    opencv_cpp_yolov5::BoundingBoxes bbox_msg;
    bbox_msg.header = msg->header;

    int detections = output.size();
    for (int i=0; i< detections; i++) {
        auto detection = output[i];
        auto box = detection.box;
        auto classId = detection.class_id;

        opencv_cpp_yolov5::BoundingBox bbox;
        bbox.Class = class_list[detection.class_id];
        bbox.probability = detection.confidence;
        bbox.xmin = box.x;
        bbox.ymin = box.y;
        bbox.xmax = box.x + box.width;
        bbox.ymax = box.y + box.height;
        bbox_msg.bounding_boxes.push_back(bbox);

        const auto color = box_colors[classId % box_colors.size()];
        cv::rectangle(frame, box, color, 3);

        cv::rectangle(frame, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
        std::string label = class_list[classId] + " (" + std::to_string(detection.confidence) + ")";
        cv::putText(frame, label.c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }
    bbox_pub.publish(bbox_msg);

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    image_pub.publish(img_msg);

    #ifdef RECORD_FPS
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = end_time - start_time;
    float fps = 1.0 / elapsed_time.count();
    ROS_INFO("FPS: %f", fps);
    #endif
}

YoloDetector::~YoloDetector() {
    ROS_INFO("YoloDetector destroyed");
}