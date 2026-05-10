#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/highgui.hpp>
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

class LCodeLocalizerNode : public rclcpp::Node
{
public:
    LCodeLocalizerNode() : Node("lcode_localizer_node")
    {
        config_    = std::make_shared<LCODE::ConfigSettings>();
        ba_        = std::make_unique<LCODE::BGArray>(*config_);
        digitizer_ = std::make_unique<LCODE::Digitizer>(*config_);

        // 2. TF 리스너 초기화 (바퀴 위치를 읽어오기 위함)
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // 3. TF 발행용 타이머 (카메라 이미지가 멈춰도 TF 트리가 끊어지지 않도록 고정 주기로 발행)
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), [this]() {
                if (initial_pos_found) {
                    publish_all(last_x, last_y, last_th, this->get_clock()->now());
                }
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
    // 초기 위치를 못 찾았더라도 일단 (0,0,0)을 기준으로 map->odom을 발행하도록 강제 활성화합니다.
    // 이렇게 하면 Nav2가 초기에 map 프레임을 찾지 못해 죽는 현상을 방지할 수 있습니다.
    bool   initial_pos_found = true;

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

        if (preprocessor_->run(src)) {
            if (digitizer_->runDigitize(*preprocessor_)) {
                // 반환 형식이 std::tuple<double, double, double>인 경우 예시
                // auto [raw_x, raw_y, raw_angle] = ba_->getPos(digitizer_->digitizied_array, digitizer_->movementIndex);
                auto coords = ba_->getPos(digitizer_->digitizied_array, digitizer_->movementIndex);

                int    raw_x     = coords.first;
                int    raw_y     = coords.second;
                double raw_angle = preprocessor_->camera_up_angle_;

                if (raw_x != -1 && raw_y != -1) {
                    // [수정] L-Code 좌표 (550×550, 좌상단 원점) -> ROS 좌표 (우상향 X, 상향 Y)
                    // 1. 중심 이동: (275, 275) -> (0, 0)
                    // double centered_x = raw_x - 275.0;
                    // double centered_y = raw_y - 275.0;

                    static const double HALF_SIZE = 550.0 * 0.002 / 2.0;

                    // 2. 좌표계 회전: L-Code(→X, ↓Y) -> ROS(→X, ↑Y)
                    // L-Code의 Y축이 아래쪽이므로 반전 필요
                    double ros_x = raw_x * 0.002 - HALF_SIZE;           // -0.55 ~ +0.55
                    double ros_y = (550 - raw_y) * 0.002 - HALF_SIZE;   // -0.55 ~ +0.55

                    // 2. 방향(Heading) 계산 (L-Code에서 준 angle 반영);
                    // last_th = raw_angle;
                    // L-Code 기준 각도를 ROS 라디안 단위로 변환이 필요할 수 있습니다.
                    // caemra_up_angle_ -> cv의 y,x좌표계 반영 ( 0도 부근에서 +, - 교차함)
                    double ros_angle_rad = -raw_angle * (M_PI / 180.0);
                    ros_angle_rad        = std::atan2(std::sin(ros_angle_rad), std::cos(ros_angle_rad));

                    last_x  = ros_x;
                    last_y  = ros_y;
                    last_th = ros_angle_rad;

                    initial_pos_found = true;
                    RCLCPP_INFO(this->get_logger(), "L-Code 해독 성공  x: %d, y: %d, angle: %.1f°", raw_x, raw_y, raw_angle);
                    RCLCPP_INFO(this->get_logger(), "map 좌표  x: %.3f m, y: %.3f m, angle: %.3f rad (%.1f°)", ros_x, ros_y, ros_angle_rad, -ros_angle_rad * 180.0 / M_PI);

                    publish_all(last_x, last_y, last_th, this->get_clock()->now());
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "L-Code 해독 실패: 유효한 좌표를 얻지 못함");
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "L-Code 해독 실패: digitizer 처리 실패");
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "L-Code 해독 실패: preprocessor 처리 실패 (마커 미발견 등)");
        }

        // [중요] 일정한 주기로 TF를 계속 퍼블리시하기 위해 타이머를 사용하므로
        // 여기 있던 퍼블리시 코드는 제거했습니다.
    }

    void publish_all(double map_x, double map_y, double map_th, const rclcpp::Time &stamp)
    {
        // --- 카メ라 → base_link 오프셋 (URDF에서 확인 후 값 입력) ---
        // 카메라가 base_link 기준으로 (cam_offset_x, cam_offset_y) 만큼 떨어져 있음
        // 예: 카메라가 로봇 앞쪽 0.15m, 좌측 0.0m에 있다면
        static const double cam_offset_x = 0.08; // URDF 확인 후 수정
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

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp            = stamp;
        t.header.frame_id         = "map";
        t.child_frame_id          = "odom";
        t.transform.translation.x = corr_x;
        t.transform.translation.y = corr_y;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, delta_th);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LCodeLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
