#ifndef DRIVER_NODE_HPP
#define DRIVER_NODE_HPP

#include "KinematicsInterface.hpp"
#include "LocalizationPauseController.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

class DriverNode : public rclcpp::Node
{
public:
    DriverNode();

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void lcode_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void update_odometry(); // 추가: 오도메트리 계산용 타이머 콜백
    void check_timeout();
    void stop_for_localization(const char *reason, double settle_override_sec = -1.0);
    void update_pause_config();

    // ROS 2 통신
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr lcode_pose_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr             odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr                 localization_abort_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                    tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr                                      timeout_timer_;
    rclcpp::TimerBase::SharedPtr                                      odom_timer_;
    rclcpp::TimerBase::SharedPtr                                      motor_timer_; // 50Hz: KinematicsInterface::tick() 호출
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // 하드웨어 인터페이스
    std::unique_ptr<KinematicsInterface> robot_;

    // 상태 변수 (추측 항법용)
    double       x_, y_, th_;   // 현재 위치 및 방향
    double       vx_, vy_, wz_; // 현재 명령된 속도
    rclcpp::Time last_command_time_;
    rclcpp::Time last_odom_time_;
    double       cmd_timeout_;        // cmd_vel 미수신 허용 시간 (초). 파라미터로 조정.
    bool         timed_out_{ false }; // 타임아웃 stop이 이미 발동됐는지 — 중복 stop 방지.

    LocalizationPauseController pause_controller_;
    bool   periodic_localization_enabled_{ false };
    double localization_distance_interval_m_{ 0.20 };
    double localization_angle_interval_rad_{ 0.523599 };
    double localization_settle_sec_{ 0.30 };
    double localization_wait_timeout_sec_{ 2.0 };
    double post_correction_hold_sec_{ 0.15 };
    double fault_rearm_zero_sec_{ 0.50 };

    // 회전→직진 settle은 기존 파라미터를 유지하되 같은 보정 상태 머신을 사용한다.
    bool   last_was_rotation_{ false };
    double settle_duration_;
    double rotation_speed_threshold_;
    double linear_speed_threshold_;

    bool is_rotation_cmd(const geometry_msgs::msg::Twist &t) const;
    bool is_linear_cmd(const geometry_msgs::msg::Twist &t) const;
    bool is_zero_cmd(const geometry_msgs::msg::Twist &t) const;
    void apply_cmd(const geometry_msgs::msg::Twist &t);
};

#endif
