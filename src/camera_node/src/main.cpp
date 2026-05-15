#include <filesystem>
#include <format>
#include <memory>
#include <opencv2/opencv.hpp>
#include <print>
#include <string_view>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

int main(int argc, char *argv[])
{
    // ROS 2 초기화
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("camera_node");

    // 1. image_transport 설정 (압축 기능을 위해 필요)
    image_transport::ImageTransport it(node);
    auto                       qos       = rclcpp::SensorDataQoS().keep_last(1);
    image_transport::Publisher image_pub = it.advertise("/downward_camera/image_raw", qos.get_rmw_qos_profile());

    // 2. GStreamer 파이프라인 문자열 설정
    // std::string pipeline_str = "libcamerasrc ! video/x-raw,width=1280,height=720,framerate=120/1 ! videoconvert ! videobalance brightness=0.1 contrast=1.5 ! appsink";
    std::string pipeline_str = "libcamerasrc ! video/x-raw,width=1280,height=720,framerate=120/1 ! videoconvert ! appsink";

    cv::VideoCapture cap(pipeline_str, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "카메라를 열 수 없습니다. 파이프라인을 확인하세요.");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Camera pipeline started via OpenCV(GStreamer). Press Ctrl+C to stop.");

    int     frame_count = 0;
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
        image_pub.publish(*msg);

        frame_count++;

        rclcpp::spin_some(node);
    }

    // 자원 해제
    cap.release();
    rclcpp::shutdown();
    return 0;
}
