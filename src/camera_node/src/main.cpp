#include <opencv2/opencv.hpp>
#include <print>
#include <format>
#include <memory>
#include <string_view>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

int main(int argc, char *argv[]) {
    // ROS 2 초기화
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("camera_node");
    auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("/downward_camera/image_raw", 10);

    // OpenCV용 GStreamer 파이프라인 문자열
    // OpenCV가 파이프라인에서 이미지를 꺼내오기 위해서는 반드시 끝에 'appsink'가 명시되어야 합니다.
    // GStreamer 파이프라인의 복잡한 포맷 협상(Negotiation) 문제를 아예 우회하기 위해,
    // OpenCV가 리눅스 범용 카메라 API(V4L2)를 직접 사용하도록 백엔드를 변경합니다.
    // libcamera 버전 충돌(OpenCV와 시스템 간)을 피하기 위해
    // 카메라 ID를 특정하거나, 파이프라인 형태를 좀 더 포괄적으로 잡습니다.
    std::string pipeline_str =
        "libcamerasrc camera-name=ov9281 ! "
        "video/x-raw,width=640,height=400,framerate=30/1 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1";
        
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
        header.frame_id = "camera_link";
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        image_pub->publish(*msg);

        // 10 프레임마다 로컬 이미지 저장
        if (frame_count % 10 == 0) {
            std::string file_name = std::format("frame_{}.png", frame_count);
            const std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 9};
            cv::imwrite(file_name, frame, params);
        }
        frame_count++;
        
        rclcpp::spin_some(node);
    }

    // 자원 해제
    cap.release();
    rclcpp::shutdown();
    return 0;
}
