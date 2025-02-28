#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

float get_depth_scale(rs2::device dev) {
    for (rs2::sensor& sensor : dev.query_sensors()) {
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

int main() {
    // Initialize camera config
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    rs2::pipeline_profile profile = pipe.start(cfg);

    // Get depth stream intrinsics
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    rs2_intrinsics intrinsics = depth_stream.get_intrinsics();

    // Create align object
    rs2::align align_to_color(RS2_STREAM_COLOR);

    // Create OpenCV windows
    const char* depth_win = "Depth Image";
    const char* color_win = "Color Image";
    cv::namedWindow(depth_win, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(color_win, cv::WINDOW_AUTOSIZE);

    rs2::colorizer color_map;  // Helper to colorize depth images

    float far_distance = 2.0f;  // Maximum depth to consider (in meters)

    while (true) {
        // Get frameset and align depth frame to color frame
        rs2::frameset frameset = pipe.wait_for_frames();
        rs2::frameset aligned_frameset = align_to_color.process(frameset);

        // Get aligned color frame and depth frame
        rs2::video_frame color_frame = aligned_frameset.get_color_frame();
        rs2::depth_frame depth_frame = aligned_frameset.get_depth_frame();

        // Convert color frame to OpenCV Mat
        const int w = color_frame.as<rs2::video_frame>().get_width();
        const int h = color_frame.as<rs2::video_frame>().get_height();
        cv::Mat frame(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

        // Initialize depth sum array
        std::vector<float> depth_sums(w, 0.0f);
        std::vector<float> depth_aver(w, 0.0f);

        // Iterate through each pixel and sum valid depths for each column
        int depth_index = 0;
        for (int x = 0; x < w; ++x) {
            depth_index = 0;
            for (int y = 0; y < h; ++y) {
                float depth_value = depth_frame.get_distance(x, y);
                if (depth_value > 0 && depth_value <= far_distance) {
                    depth_sums[x] += depth_value;
                    depth_index ++;
                }
            }
            if(depth_index != 0 ) {
                depth_aver[x] = depth_sums[x]/depth_index;
            }
        }

        // Display color image
        cv::imshow(color_win, frame);

        // Convert depth frame to OpenCV Mat and apply color map
        rs2::frame depth_colorized = color_map.process(depth_frame);
        cv::Mat depth_mat(cv::Size(w, h), CV_8UC3, (void*)depth_colorized.get_data(), cv::Mat::AUTO_STEP);
        cv::imshow(depth_win, depth_mat);

        // Print depth sums (for debugging)
        std::cout << "Depth sums: ";
        for (float sum : depth_sums) {
            std::cout << sum << " ";
        }
        std::cout << std::endl;

        // Analyze the depth sums array to determine the drone's orientation
        float left_sum = 0, right_sum = 0;

        for (int i = 1; i < w; ++i) {
            if(depth_aver[i]>=depth_aver[i-1]) right_sum++;
            else if(depth_aver[i]<depth_aver[i-1]) left_sum++;
        }

        float rate = 0;
        if( right_sum+left_sum != 0 ) {
            rate = right_sum/(right_sum+left_sum);
        }

        std::cout << "Left sum: " << left_sum << ", Right sum: " << right_sum << std::endl;
        if (rate > 0.6) {
            std::cout << "Drone should move left" << std::endl;
        } else if(rate < 0.4){
            std::cout << "Drone should move right" << std::endl;
        }
        else{
            std::cout << "Drone is on the middle" << std::endl;
        }

        if (cv::waitKey(1) >= 0) break;  // Exit on any key press
    }

    return 0;
}
