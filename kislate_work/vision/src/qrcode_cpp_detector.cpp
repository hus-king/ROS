// 必要时借尸还魂
// 算了，还是借尸还魂

#include <opencv_cpp_yolov5/BoundingBox.h>
#include <opencv_cpp_yolov5/BoundingBoxes.h>
#include <opencv_cpp_yolov5/BoxCenter.h>

#include "yolo_detector.h"


static constexpr float YOLO_INPUT_WIDTH = 640.0;
static constexpr float YOLO_INPUT_HEIGHT = 640.0;

/* 删去了类继承 ，只用下置摄像头*/
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
    // 这里是发布和订阅的地方

    #ifndef DOUBLE_VIDEO_STREAMS// 如果没有定义DOUBLE_VIDEO_STREAMS，就只有一个视频流
    
    // 创建一个Publisher对象image_pub，用于发布图像消息
    image_transport::Publisher image_pub = it.advertise("/opencv_cpp_yolov5/detected_image", 1);
    ROS_INFO("initialize image_pub");
    // 创建一个Publisher对象bbox_pub，用于发布检测结果
    ros::Publisher bbox_pub = nh.advertise<opencv_cpp_yolov5::BoundingBoxes>("/opencv_cpp_yolov5/bounding_boxes", 1);
    ROS_INFO("initialize bbox_pub");
    // 交给image_cb处理
    image_transport::Subscriber sub = it.subscribe("/video_stream_node/image_raw", 1, 
                                boost::bind(&YoloDetectorElite::image_cb, &yolo_detector, _1, // 用bind强绑，_1占位
                                boost::ref(bbox_pub), boost::ref(image_pub), boost::ref(center_pub)));  // 用ref引用  
            
    #else // 如果定义了DOUBLE_VIDEO_STREAMS，就有两个视频流

    // 创建两个Publisher对象，用于发布图像消息
    image_transport::Publisher image_pub_front = it.advertise("/opencv_cpp_yolov5/detected_image_front", 1);
    image_transport::Publisher image_pub_down = it.advertise("/opencv_cpp_yolov5/detected_image_down", 1);
    ROS_INFO("initialize image_pub");

    // 创建两个Publisher对象，用于发布检测结果
    ros::Publisher bbox_pub_front = nh.advertise<opencv_cpp_yolov5::BoundingBoxes>("/opencv_cpp_yolov5/bounding_boxes_front", 1);
    ros::Publisher bbox_pub_down = nh.advertise<opencv_cpp_yolov5::BoundingBoxes>("/opencv_cpp_yolov5/bounding_boxes_down", 1);
    ROS_INFO("initialize bbox_pub");


    image_transport::Subscriber sub_front = it.subscribe("/usb_cam_front/image_raw", 1,
        boost::bind(image_cb, _1, boost::ref(yolo), boost::ref(class_list), boost::ref(bbox_pub_front), boost::ref(image_pub_front)
        #ifdef USE_RESNET// resnet意思是ResNet，是一个深度卷积神经网络，用于图像分类
        , boost::ref(resnet)
        #endif
        ));
    image_transport::Subscriber sub_down = it.subscribe("/usb_cam_down/image_raw", 1,
        boost::bind(image_cb, _1, boost::ref(yolo), boost::ref(class_list), boost::ref(bbox_pub_down), boost::ref(image_pub_down)
        #ifdef USE_RESNET
        , boost::ref(resnet)
        #endif
        ));
    #endif

    ros::spin();// 进入ROS事件循环

    return 0;
}
