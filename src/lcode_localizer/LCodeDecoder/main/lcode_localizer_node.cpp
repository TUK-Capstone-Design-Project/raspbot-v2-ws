#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "ConfigSettings/ConfigSettings.hpp"
#include "DeBruijnSeq/GenBGArray.hpp"
#include "Digitizer/Digitizer.hpp"
#include "ImgPreprocessor/ImgPreprocessor.hpp"

#define DECODER_DEBUG_MODE // 디버그 모드 비활성화

class LCodeLocalizerNode : public rclcpp::Node
{
public:
    LCodeLocalizerNode() : Node("lcode_localizer_node")
    {
        #ifdef DECODER_DEBUG_MODE
        std::filesystem::create_directories("./resize_images"); // 디버깅용 디렉토리 생성
        std::filesystem::create_directories("./resize_images/success"); // 디버깅용 디렉토리 생성
        std::filesystem::create_directories("./resize_images/failed"); // 디버깅용 디렉토리 생성
        #endif
        config_    = std::make_shared<LCODE::ConfigSettings>();
        ba_        = std::make_unique<LCODE::BGArray>(*config_);
        digitizer_ = std::make_unique<LCODE::Digitizer>(*config_);

        // 2. TF 리스너 초기화 (바퀴 위치를 읽어오기 위함)
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // 캐시 초기값: identity (map == odom). 첫 L-Code 인식 전까지는 이 값이 발행되어
        // Nav2 가 map 프레임이 없어서 죽는 현상을 막는다.
        cached_map_to_odom_.header.frame_id = "map";
        cached_map_to_odom_.child_frame_id  = "odom";
        cached_map_to_odom_.transform.translation.x = 0.0;
        cached_map_to_odom_.transform.translation.y = 0.0;
        cached_map_to_odom_.transform.translation.z = 0.0;
        cached_map_to_odom_.transform.rotation.x = 0.0;
        cached_map_to_odom_.transform.rotation.y = 0.0;
        cached_map_to_odom_.transform.rotation.z = 0.0;
        cached_map_to_odom_.transform.rotation.w = 1.0;

        // 3. TF 발행용 타이머.
        // 새 L-Code 인식 결과가 들어왔을 때만 image_callback 에서 cached_map_to_odom_ 가 갱신되고,
        // 타이머는 절대 새로 계산하지 않고 stamp 만 갈아끼워서 재발행한다.
        // (이전 구현은 매 50ms 마다 last_x/y/th(=마지막 L-Code 인식 결과)와 *현재* odom 으로
        //  map→odom 을 다시 계산했기 때문에, map→base_link 가 항상 last_x/y/th 에 고정되어
        //  Nav2 가 본 로봇은 L-Code 인식이 들어오기 전까진 절대 안 움직이는 것처럼 보였음.)
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), [this]() {
                cached_map_to_odom_.header.stamp = this->get_clock()->now();
                tf_broadcaster_->sendTransform(cached_map_to_odom_);
            }
        );

        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/downward_camera/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&LCodeLocalizerNode::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "L-Code Localizer Node 시작됨 (GPU 가속 대응 완료)");
    }

private:
    std::shared_ptr<LCODE::ConfigSettings> config_;
    std::unique_ptr<LCODE::BGArray>        ba_;
    std::unique_ptr<LCODE::Digitizer>      digitizer_;
    // std::unique_ptr<LCODE::ImgPreprocessor> preprocessor_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr                tf_timer_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;

    double last_x = 0.0, last_y = 0.0, last_th = 0.0;

    // 마지막으로 계산한 map→odom. 새 L-Code 인식이 들어와 publish_all() 이 호출될 때만 갱신되고,
    // 타이머는 stamp 만 갈아끼워서 이걸 그대로 재발행한다. 그 결과 인식 사이엔 map→odom 이
    // 고정되고 odom 의 증분이 그대로 map→base_link 에 반영되어 Nav2 가 로봇 움직임을 실제로 본다.
    geometry_msgs::msg::TransformStamped cached_map_to_odom_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat src;
        try {
            src = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_WARN(this->get_logger(), "cv_bridge 예외 발생: %s", e.what());
            return;
        }
        auto preprocessor_ = std::make_unique<LCODE::ImgPreprocessor>(*config_);

        rclcpp::Time stamp = msg->header.stamp;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "이미지 콜백 호출됨 - 프레임 수신 중");

        cv::resize(src, src, {}, 0.5, 0.5);                  // 1280x720 -> 640x360 (처리 속도 개선)
        cv::rotate(src, src, cv::ROTATE_90_CLOCKWISE);
        static int success_count = 0;
        static int failed_count = 0;
        // 1, 이미지 전처리
        if (!preprocessor_->run(src)) {
        #ifdef DECODER_DEBUG_MODE

            cv::imwrite("./resize_images/failed/" + std::to_string(failed_count++) + ".jpg", src); // 디버깅용 실패 이미지 저장
        #endif
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "L-Code 해독 실패: preprocessor 처리 실패 (마커 미발견 등)");
            return;
        }

        // 2. 디지타이징
        if (!digitizer_->runDigitize(*preprocessor_)) {
        #ifdef DECODER_DEBUG_MODE
            cv::imwrite("./resize_images/failed/" + std::to_string(failed_count++) + ".jpg", src); // 디버깅용 실패 이미지 저장
        #endif
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "L-Code 해독 실패: digitizer 처리 실패");
            return;
        }

        // 3. L-Code에서 좌표와 각도 추출
        double raw_angle    = preprocessor_->camera_up_angle_;
        auto [raw_x, raw_y] = ba_->getPos(digitizer_->digitizied_array, digitizer_->movementIndex);
        if (raw_x == -1 || raw_y == -1) {
        #ifdef DECODER_DEBUG_MODE
            cv::imwrite("./resize_images/failed/" + std::to_string(failed_count++) + ".jpg", src); // 디버깅용 실패 이미지 저장
        #endif
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "L-Code 해독 실패: 유효한 좌표를 얻지 못함");
            return;
        }
        #ifdef DECODER_DEBUG_MODE
        // cv::imwrite("./resize_images/success/" + std::to_string(success_count++) + ".jpg", src); // 디버깅용 성공 이미지 저장
        #endif
        
        // 4. 방향(Heading) 계산 (L-Code에서 준 angle 반영);
        //
        // [컨벤션 정리]
        //   ImgPreprocessor::camera_up_angle_ = atan2(vec.y, vec.x).
        //   OpenCV는 y가 아래로 +이므로 raw_angle은 *CW positive*:
        //     이미지 위쪽 방향(screen up, vec=(0,-1)) → -90°
        //     이미지 오른쪽          (vec=(1,0))    →   0°
        //     이미지 아래쪽          (vec=(0,+1))   → +90°
        //     이미지 왼쪽            (vec=(-1,0))   → ±180°
        //
        // [목표 컨벤션] ROS yaw 표준 (CCW positive, x축=0°)
        //   카메라 오른쪽=0°, 위=+90°, 왼쪽=±180°, 아래=-90°
        //
        // → y축 방향이 반대니까 부호를 반전해야 함 (linear algebra).
        double ros_angle_rad = -raw_angle * (M_PI / 180.0);
        ros_angle_rad        = std::atan2(std::sin(ros_angle_rad), std::cos(ros_angle_rad));

        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        //     "[angle] raw=%.1f° (cv, CW+)  → ros=%.1f° (CCW+)  (%.3f rad)",
        //     raw_angle, ros_angle_rad * 180.0 / M_PI, ros_angle_rad);

        // 5. 최종 좌표 변환: L-Code 좌표 (550×550, 좌상단 원점) -> ROS 좌표 (우상향 X, 상향 Y)

        // 로봇 위치 중앙 이동 보정
        static const double HALF_SIZE = 550.0 * 0.002 / 2.0;
        double              ros_x     = raw_x * 0.002 - HALF_SIZE;         // -0.55 ~ +0.55
        double              ros_y     = (550 - raw_y) * 0.002 - HALF_SIZE; // -0.55 ~ +0.55

        last_x  = ros_x;
        last_y  = ros_y;
        last_th = ros_angle_rad;

        RCLCPP_INFO(this->get_logger(), "L-Code 해독 성공  x: %d, y: %d, angle: %.1f°", raw_x, raw_y, raw_angle);

        // 필요시 map좌표 on
        // RCLCPP_INFO(this->get_logger(), "map 좌표  x: %.3f m, y: %.3f m, angle: %.3f rad (%.1f°)", ros_x, ros_y, ros_angle_rad, -ros_angle_rad * 180.0 / M_PI);

        // 5. 새 인식 결과를 바탕으로 map→odom 을 (재)계산하고 캐시에 박는다.
        //    이후엔 타이머가 같은 transform 을 stamp 만 갱신하며 재발행한다.
        update_map_to_odom(last_x, last_y, last_th, this->get_clock()->now());
    }

    // 한 번의 L-Code 인식 결과로 map→odom 보정값을 새로 계산해 캐시에 저장하고 즉시 1회 발행한다.
    // 이전 구현(publish_all) 과 달리 타이머에서는 호출되지 않는다.
    void update_map_to_odom(double map_x, double map_y, double map_th, const rclcpp::Time &stamp)
    {
        // --- 카메라 → base_link 오프셋 (URDF에서 확인 후 값 입력) ---
        // 카메라가 base_link 기준으로 (cam_offset_x, cam_offset_y) 만큼 떨어져 있음
        // 예: 카메라가 로봇 앞쪽 0.15m, 좌측 0.0m에 있다면
        static const double cam_offset_x = 0.055; // URDF 확인 후 수정
        static const double cam_offset_y = 0.0;  // URDF 확인 후 수정

        // L-Code가 알려준 좌표는 카메라 위치(map 프레임)
        // 로봇 중심(base_link) 위치로 변환
        double cos_th = std::cos(map_th);
        double sin_th = std::sin(map_th);

        double base_x = map_x - (cos_th * cam_offset_x - sin_th * cam_offset_y);
        double base_y = map_y - (sin_th * cam_offset_x + cos_th * cam_offset_y);
        // base_th는 map_th와 동일 (카메라가 로봇과 같은 방향)
        double base_th = map_th;

        // --- 이하 base_x, base_y, base_th를 사용 ---
        geometry_msgs::msg::TransformStamped odom_to_base;
        try {
            if (!tf_buffer_->canTransform("odom", "base_link", tf2::TimePointZero)) {
                return; // 아직 트리가 없으면 종료
            }
            odom_to_base = tf_buffer_->lookupTransform(
                "odom", "base_link", tf2::TimePointZero
            );
        } catch (const tf2::TransformException &ex) {
            RCLCPP_DEBUG(this->get_logger(), "odom->base_link TF 조회 보류 중: %s", ex.what());
            return;
        }

        double odom_x = odom_to_base.transform.translation.x;
        double odom_y = odom_to_base.transform.translation.y;

        tf2::Quaternion odom_q(
            odom_to_base.transform.rotation.x,
            odom_to_base.transform.rotation.y,
            odom_to_base.transform.rotation.z,
            odom_to_base.transform.rotation.w
        );
        double odom_th = tf2::getYaw(odom_q);

        double delta_th = base_th - odom_th;
        double cos_dt   = std::cos(delta_th);
        double sin_dt   = std::sin(delta_th);

        double corr_x = base_x - (cos_dt * odom_x - sin_dt * odom_y);
        double corr_y = base_y - (sin_dt * odom_x + cos_dt * odom_y);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, delta_th);

        // 캐시에 기록 (frame_id/child_frame_id 는 생성자에서 이미 설정됨)
        cached_map_to_odom_.header.stamp            = stamp;
        cached_map_to_odom_.transform.translation.x = corr_x;
        cached_map_to_odom_.transform.translation.y = corr_y;
        cached_map_to_odom_.transform.translation.z = 0.0;
        cached_map_to_odom_.transform.rotation.x    = q.x();
        cached_map_to_odom_.transform.rotation.y    = q.y();
        cached_map_to_odom_.transform.rotation.z    = q.z();
        cached_map_to_odom_.transform.rotation.w    = q.w();

        // 즉시 1회 발행 (타이머 주기를 기다리지 않고 빠르게 반영).
        tf_broadcaster_->sendTransform(cached_map_to_odom_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LCodeLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
