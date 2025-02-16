// 必要时借尸还魂
// 算了，还是借尸还魂

#include <opencv_cpp_yolov5/BoundingBox.h>
#include <opencv_cpp_yolov5/BoundingBoxes.h>
#include <opencv_cpp_yolov5/BoxCenter.h>
#include <opencv_cpp_yolov5/detector_conunt.h> // 添加用于发布物体数量的消息类型
#include <std_msgs/Int32.h> // 添加用于发布物体数量的消息类型

#include "yolo_detector.h"


static constexpr float YOLO_INPUT_WIDTH = 640.0;
static constexpr float YOLO_INPUT_HEIGHT = 640.0;

class YoloDetectorPlus : public YoloDetector {
    // 继承
    public:
        YoloDetectorPlus(const std::string& class_list_path, const std::string& yolo_path, 
                    bool use_cuda, float nms_threshold, float confidence_threshold, float score_threshold,
                    float yolo_input_width, float yolo_input_height) : 
                    YoloDetector(class_list_path, yolo_path, use_cuda, nms_threshold, confidence_threshold, score_threshold, yolo_input_width, yolo_input_height) {
            ROS_INFO("YoloDetectorPlus initialized");// ok
        }
    
        // 析构函数
        ~YoloDetectorPlus() {ROS_INFO("YoloDetectorPlus destroyed");}
    
        // detector函数没有重写；


        //重写image_c函数，在发布的消息中加入类别id,并发布物体数量
        void image_cb(const sensor_msgs::ImageConstPtr& msg, ros::Publisher& bbox_pub, image_transport::Publisher& image_pub, ros::Publisher& center_pub, ros::Publisher& conut_pub) {
    
        //
            #ifdef RECORD_FPS// 要使用的话，需要在编译的时候加上这个参数，计算FPS
            auto start_time = std::chrono::steady_clock::now();
            #endif
    
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());// e.what()返回错误信息
                return;
            }
    
            cv::Mat frame = cv_ptr->image;
            std::vector<Detection> output;
            detect(frame, output);
    
            opencv_cpp_yolov5::BoundingBoxes bbox_msg;
            bbox_msg.header = msg->header;

            opencv_cpp_yolov5::detector_conunt count_msg; // 创建一个消息对象
            count_msg.data = output.size(); // 将物体数量赋值给消息对象


            /*
            int blue_ys[2], chess_xs[2];
            int blue_cnt = 0, chess_cnt = 0;
            */
        
        //
            int detections = output.size();

            for (int i=0; i < detections; i++) {
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
                bbox.Class_Id = classId// 加入类别id
                bbox_msg.bounding_boxes.push_back(bbox);
    
                /*
                if (classId == 0) {
                    blue_ys[blue_cnt++] = (bbox.ymin + bbox.ymax) / 2;// 门的蓝色框的中心坐标
                } else if (classId == 1) {
                    chess_xs[chess_cnt++] = (bbox.xmin + bbox.xmax) / 2;// 棋格框的中心坐标
                }
                */

                const auto color = box_colors[classId % box_colors.size()];
                cv::rectangle(frame, box, color, 3);
                cv::rectangle(frame, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
                std::string label = class_list[classId] + " (" + std::to_string(detection.confidence) + ")";
                cv::putText(frame, label.c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            }
            bbox_pub.publish(bbox_msg);
            conut_pub.publish(count_msg); // 发布物体数量
    
        /*
            // TODO we can add another parameter here to ask blues and chesses to be at least `param` separated in the image
            opencv_cpp_yolov5::BoxCenter box_center;
            box_center.header = msg->header;
    
            // In theory, we can indeed achieve this with only the chessboard
            if (blue_cnt == 2 && chess_cnt == 2) {
                box_center.flag = true;  // only when all 4 objects are detected will we consider `box_center` valid
                box_center.y = (blue_ys[0] + blue_ys[1]) / 2;
                box_center.x = (chess_xs[0] + chess_xs[1]) / 2;
    
                cv::circle(frame, cv::Point(box_center.x, box_center.y), 5, cv::Scalar(0, 255, 0), -1);
                cv::putText(frame, "Center", cv::Point(box_center.x + 10, box_center.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
            }
            else {
                box_center.flag = false;
                box_center.x = box_center.y = -1;
            }
    
            // 发布
            center_pub.publish(box_center);
        */
    
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            image_pub.publish(img_msg);// 具体用途不清
        
    
            #ifdef RECORD_FPS
            auto end_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_time = end_time - start_time;
            float fps = 1.0 / elapsed_time.count();
            ROS_INFO("FPS: %f", fps);
            #endif        
        }
    };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_cpp_yolov5_node");
    
    ROS_INFO("start node opencv_cpp_yolov5");

    ros::NodeHandle nh("~");

    std::string yolo_path, class_list_path;
    bool use_cuda = true;
    float score_threshold, nms_threshold, confidence_threshold;

    nh.getParam("yolo_path", yolo_path);
    nh.getParam("class_list_path", class_list_path);
    nh.getParam("use_cuda", use_cuda);
    nh.getParam("score_threshold", score_threshold);
    nh.getParam("nms_threshold", nms_threshold);
    nh.getParam("confidence_threshold", confidence_threshold);

    
    ROS_INFO("yolo_path: %s", yolo_path.c_str());
    ROS_INFO("class_list_path: %s", class_list_path.c_str());
    ROS_INFO("use_cuda: %d", use_cuda);
    ROS_INFO("score_threshold: %f", score_threshold);
    ROS_INFO("nms_threshold: %f", nms_threshold);
    ROS_INFO("confidence_threshold: %f", confidence_threshold);

    YoloDetector yolo_detector(class_list_path, yolo_path, use_cuda, 
                                nms_threshold, confidence_threshold, score_threshold, 
                                YOLO_INPUT_WIDTH, YOLO_INPUT_HEIGHT);
    // 是初始化检测器
    // 创建一个ImageTransport对象it，用于发布和订阅图像消息
    image_transport::ImageTransport it(nh);
    ROS_INFO("initialize image_transport");

    #ifndef DOUBLE_VIDEO_STREAMS// 如果没有定义DOUBLE_VIDEO_STREAMS，就只有一个视频流
    
    // 创建一个Publisher对象image_pub，用于发布图像消息
    image_transport::Publisher image_pub = it.advertise("/opencv_cpp_yolov5/detected_image", 1);
    ROS_INFO("initialize image_pub");

    // 创建一个Publisher对象bbox_pub，用于发布检测结果
    ros::Publisher bbox_pub = nh.advertise<opencv_cpp_yolov5::BoundingBoxes>("/opencv_cpp_yolov5/bounding_boxes", 1);
    ROS_INFO("initialize bbox_pub");

    // 创建一个Publisher对象count_pub，用于发布物体数量
    ros::Publisher count_pub = nh.advertise<std_msgs::Int32>("/opencv_cpp_yolov5/detect_num", 1);
    ROS_INFO("initialize count_pub");

    // 交给image_cb处理
    image_transport::Subscriber sub = it.subscribe("/video_stream_node/image_raw", 1, 
                                boost::bind(&YoloDetectorPlus::image_cb, &yolo_detector, _1, // 用bind强绑，_1占位
                                boost::ref(bbox_pub), boost::ref(image_pub), boost::ref(center_pub), boost::ref(count_pub)));  // 用ref引用  
            
    #else // 如果定义了DOUBLE_VIDEO_STREAMS，就有两个视频流

    // 创建两个Publisher对象，用于发布图像消息
    image_transport::Publisher image_pub_front = it.advertise("/opencv_cpp_yolov5/detected_image_front", 1);
    image_transport::Publisher image_pub_down = it.advertise("/opencv_cpp_yolov5/detected_image_down", 1);
    ROS_INFO("initialize image_pub");

    // 创建两个Publisher对象，用于发布检测结果
    ros::Publisher bbox_pub_front = nh.advertise<opencv_cpp_yolov5::BoundingBoxes>("/opencv_cpp_yolov5/bounding_boxes_front", 1);
    ros::Publisher bbox_pub_down = nh.advertise<opencv_cpp_yolov5::BoundingBoxes>("/opencv_cpp_yolov5/bounding_boxes_down", 1);
    ROS_INFO("initialize bbox_pub");

    // 创建两个Publisher对象，用于发布物体数量
    ros::Publisher count_pub_front = nh.advertise<std_msgs::Int32>("/opencv_cpp_yolov5/detect_num_front", 1);
    ros::Publisher count_pub_down = nh.advertise<std_msgs::Int32>("/opencv_cpp_yolov5/detect_num_down", 1);


    image_transport::Subscriber sub_front = it.subscribe("/usb_cam_front/image_raw", 1,
        boost::bind(image_cb, _1, boost::ref(yolo), boost::ref(class_list), boost::ref(bbox_pub_front), boost::ref(image_pub_front), boost::ref(count_pub_front)
        #ifdef USE_RESNET// resnet意思是ResNet，是一个深度卷积神经网络，用于图像分类
        , boost::ref(resnet)
        #endif
        ));
    image_transport::Subscriber sub_down = it.subscribe("/usb_cam_down/image_raw", 1,
        boost::bind(image_cb, _1, boost::ref(yolo), boost::ref(class_list), boost::ref(bbox_pub_down), boost::ref(image_pub_down), boost::ref(count_pub_down)
        #ifdef USE_RESNET
        , boost::ref(resnet)
        #endif
        ));
    #endif

    ros::spin();// 进入ROS事件循环

    return 0;
}
