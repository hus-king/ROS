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
    constexpr size_t MAX_NUM_CLASSES = 11;
    const std::array<cv::Scalar, MAX_NUM_CLASSES> box_colors = {
        // HACK rescale this array to match `MAX_NUM_CLASSES`
        cv::Scalar(255, 255, 0),
        cv::Scalar(0, 255, 255),
        cv::Scalar(255, 0, 255),
        cv::Scalar(0, 255, 0),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 255, 0),
    };
}

class YoloDetector {
public:
    YoloDetector(const std::string& class_list_path, const std::string& yolo_path, 
                bool use_cuda, float nms_threshold, float confidence_threshold, float score_threshold,
                float yolo_input_width, float yolo_input_height);
    ~YoloDetector();

    void image_cb(const sensor_msgs::ImageConstPtr& msg, ros::Publisher& bbox_pub, image_transport::Publisher& image_pub);

    struct Detection {
        int class_id;
        double confidence;
        cv::Rect box;

        Detection() {}  // no idea whether we should leave default constructor or not
        Detection(int class_id, double confidence, cv::Rect box) : box(box), class_id(class_id), confidence(confidence) {}
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