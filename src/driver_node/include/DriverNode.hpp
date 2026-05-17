#ifndef DRIVER_NODE_HPP
#define DRIVER_NODE_HPP

#include "KinematicsInterface.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class DriverNode : public rclcpp::Node
{
public:
    DriverNode();

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void update_odometry(); // 추가: 오도메트리 계산용 타이머 콜백
    void check_timeout();

    // ROS 2 통신
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr             odom_publisher_;
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
};

#endif