#include <opencv2/opencv.hpp>
#include <print>
#include <format>
#include <memory>
#include <string_view>
#include <vector>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

int main(int argc, char *argv[]) {
    // ROS 2 초기화
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("camera_node");
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("/downward_camera/image_raw", qos);

    // OV9281은 mono 카메라이므로 GRAY8 포맷을 명시해 협상 실패를 방지한다.
    // appsink 의 drop=1, max-buffers=1, sync=false 는 라이브 소스에서 백프레셔를 끊어
    // cap.read() 가 영구 블록되는 현상을 막는다.
    // videobalance의 brightness 값을 통해 밝기를 조절합니다 (-1.0 ~ 1.0, 기본값: 0.0)
    // 노출(Gain/대비)을 높이기 위해 contrast 값을 추가로 늘립니다 (기본값: 1.0)
    std::string pipeline_str = "libcamerasrc ! video/x-raw,width=1280,height=720,framerate=120/1 ! videoconvert ! videobalance brightness=0.1 contrast=1.5 ! appsink";
    // std::string pipeline_str =
    //     "libcamerasrc ! "
    //     "video/x-raw,format=GRAY8,width=1280,height=720,framerate=60/1 ! "
    //     "appsink drop=1 max-buffers=1 sync=false";
        
    std::println("test1: libcamera 파이프라인 로드 시도 중...");
    
    // GStreamer 백엔드를 강제 지정하여 엽니다.
    cv::VideoCapture cap(pipeline_str, cv::CAP_GSTREAMER);
    
    std::println("test2: 로드 완료 (혹은 실패)");
    
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "카메라를 열 수 없습니다. 파이프라인을 확인하세요.");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Camera pipeline started via OpenCV(GStreamer). Press Ctrl+C to stop.");

    int frame_count = 0;
    cv::Mat frame;

    std::filesystem::create_directories("frames"); // 프레임 저장용 디렉토리 생성

    while (rclcpp::ok()) {
        // 이미지를 OpenCV 객체(cv::Mat)로 바로 가져옵니다
        if (!cap.read(frame) || frame.empty()) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "프레임을 대기 중입니다...");
            rclcpp::spin_some(node);
            continue;
        }

        // ROS 2 메시지로 변환 후 발행
        std_msgs::msg::Header header;
        header.stamp = node->now();
        // header.frame_id = "camera_link";
        header.frame_id = "camera_optical_link";

        // OV9281은 모노크롬(흑백) 카메라이므로 1채널(GRAY)로 처리해야 합니다.
        cv::Mat gray_frame;
        if (frame.channels() == 3) {
            cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        } else {
            gray_frame = frame;
        }

        // 인코딩을 "bgr8" 대신 "mono8"로 변경
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono8", gray_frame).toImageMsg();
        image_pub->publish(*msg);

        // 10 프레임마다 로컬 이미지 저장
        if (frame_count % 10 == 0) {
            std::string file_name = std::format("./frames/frame_{}.png", frame_count);
            const std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 9};
            cv::imwrite(file_name, frame, params);
        }
        // cv::imshow("Camera Frame", frame);
        frame_count++;
        
        rclcpp::spin_some(node);
    }

    // 자원 해제
    cap.release();
    rclcpp::shutdown();
    return 0;
}
