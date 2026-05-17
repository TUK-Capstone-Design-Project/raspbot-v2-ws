#include <atomic>
#include <cstdio>
#include <filesystem>
#include <format>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <print>
#include <string>
#include <thread>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class RpiCameraNode : public rclcpp::Node
{
public:
    RpiCameraNode() : Node("camera_node2"), running_(false)
    {
        // 생성자에서는 파라미터 초기화만 수행
        width_               = 1280;
        height_              = 720;
        fps_                 = 60;
        new_frame_available_ = false;
    }

    // [중요] shared_from_this()를 안전하게 사용하기 위해 초기화 함수를 따로 만듭니다.
    void setup()
    {
        // 1. image_transport 설정 (이제 객체가 생성된 후이므로 shared_from_this() 사용 가능)
        image_transport::ImageTransport it(shared_from_this());
        auto                            qos = rclcpp::SensorDataQoS().keep_last(1);
        image_pub_                          = it.advertise("/downward_camera/image_raw", qos.get_rmw_qos_profile());

        // 2. 스레드 시작
        running_        = true;
        capture_thread_ = std::thread(&RpiCameraNode::capture_loop, this);
        publish_thread_ = std::thread(&RpiCameraNode::publish_loop, this);

        auto logmsg = std::format("멀티스레드 카메라 노드 초기화 완료 ({}x{} @ {}fps)", width_, height_, fps_);
        RCLCPP_INFO(this->get_logger(), "%s", logmsg.c_str());
    }

    ~RpiCameraNode()
    {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
        if (publish_thread_.joinable()) publish_thread_.join();
    }

private:
    void capture_loop()
    {
        size_t      frame_size = width_ * height_ * 3 / 2; // YUV420
        std::string command    = std::format(
            "rpicam-vid -t 0 --width {} --height {} --framerate {} "
            "--codec yuv420 --nopreview --flush -o -",
            width_, height_, fps_
        );

        FILE *pipe = popen(command.c_str(), "r");
        if (!pipe) {
            RCLCPP_ERROR(this->get_logger(), "rpicam-vid 파이프를 열 수 없습니다.");
            return;
        }

        std::vector<uchar> buffer(frame_size);
        while (rclcpp::ok() && running_) {
            size_t bytes_read = fread(buffer.data(), 1, frame_size, pipe);
            if (bytes_read != frame_size) continue;

            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                // YUV420의 Y채널(처음 W*H 바이트)을 Mono8로 매핑
                cv::Mat gray(height_, width_, CV_8UC1, buffer.data());
                latest_frame_        = gray.clone();
                new_frame_available_ = true;
            }
        }
        pclose(pipe);
    }

    void publish_loop()
    {
        while (rclcpp::ok() && running_) {
            cv::Mat frame_to_publish;
            bool    has_new = false;

            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                if (new_frame_available_) {
                    frame_to_publish     = latest_frame_;
                    new_frame_available_ = false;
                    has_new              = true;
                }
            }

            if (has_new) {
                auto header     = std_msgs::msg::Header();
                header.stamp    = this->now();
                header.frame_id = "camera_optical_link";

                auto msg = cv_bridge::CvImage(header, "mono8", frame_to_publish).toImageMsg();
                image_pub_.publish(*msg);
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        }
    }

    std::atomic<bool>          running_;
    std::thread                capture_thread_;
    std::thread                publish_thread_;
    std::mutex                 frame_mutex_;
    cv::Mat                    latest_frame_;
    bool                       new_frame_available_;
    int                        width_, height_, fps_;
    image_transport::Publisher image_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 1. 노드 객체 생성
    auto node = std::make_shared<RpiCameraNode>();

    // 2. 생성 완료 후 setup() 호출 (shared_ptr이 유효한 상태)
    node->setup();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
