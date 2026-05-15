#include "DriverNode.hpp"
#include <tf2/LinearMath/Quaternion.h>

DriverNode::DriverNode() : Node("driver_node"), x_(0.0), y_(0.0), th_(0.0), vx_(0.0), vy_(0.0), wz_(0.0) {
  robot_ = std::make_unique<KinematicsInterface>("/dev/i2c-1", 0x2B);

  // ── 모터 튜닝 파라미터 (ros2 param set 으로 런타임 조정) ──────────────
  // PWM = round(wheel_speed_m_per_s * speed_scale)
  this->declare_parameter<double>("speed_scale", 40.0);
  // 0 < |PWM| < min_pwm 인 경우 ±min_pwm 으로 끌어올려 정지마찰 보상
  this->declare_parameter<int>("min_pwm", 15);
  // |PWM| < pwm_deadzone 이면 0 으로 무시 (지터 방지)
  this->declare_parameter<int>("pwm_deadzone", 5);
  this->declare_parameter<int>("max_pwm", 255);

  // 초기값 적용
  robot_->speed_scale_ = this->get_parameter("speed_scale").as_double();
  robot_->min_pwm_     = this->get_parameter("min_pwm").as_int();
  robot_->pwm_deadzone_= this->get_parameter("pwm_deadzone").as_int();
  robot_->max_pwm_     = this->get_parameter("max_pwm").as_int();
  RCLCPP_INFO(this->get_logger(),
              "Motor params: speed_scale=%.1f, min_pwm=%d, deadzone=%d, max_pwm=%d",
              robot_->speed_scale_, robot_->min_pwm_,
              robot_->pwm_deadzone_, robot_->max_pwm_);

  // 런타임 변경 (`ros2 param set /driver_node min_pwm 90` 등) 즉시 반영
  param_cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        for (const auto &p : params) {
          if (p.get_name() == "speed_scale")   robot_->speed_scale_ = p.as_double();
          else if (p.get_name() == "min_pwm")      robot_->min_pwm_     = p.as_int();
          else if (p.get_name() == "pwm_deadzone") robot_->pwm_deadzone_= p.as_int();
          else if (p.get_name() == "max_pwm")      robot_->max_pwm_     = p.as_int();
        }
        RCLCPP_INFO(this->get_logger(),
                    "Motor params updated: speed_scale=%.1f, min_pwm=%d, deadzone=%d, max_pwm=%d",
                    robot_->speed_scale_, robot_->min_pwm_,
                    robot_->pwm_deadzone_, robot_->max_pwm_);
        return res;
      });

  // 퍼블리셔 및 TF 브로드캐스터 초기화
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&DriverNode::cmd_vel_callback, this, std::placeholders::_1));

  // 오도메트리 업데이트 타이머 (50Hz)
  last_odom_time_ = this->get_clock()->now();
  odom_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&DriverNode::update_odometry, this));

  last_command_time_ = this->get_clock()->now();
  timeout_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&DriverNode::check_timeout, this));
}

void DriverNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (!robot_->is_connected()) 
  {
    RCLCPP_ERROR(this->get_logger(), "I2C 통신 연결 실패");
    return;
  }
  last_command_time_ = this->get_clock()->now();

  vx_ = msg->linear.x;
  vy_ = msg->linear.y;
  wz_ = msg->angular.z;
  robot_->drive(vx_, vy_, wz_);
}

void DriverNode::update_odometry() {
  auto now = this->get_clock()->now();
  double dt = (now - last_odom_time_).seconds();
  last_odom_time_ = now;

  // 1. 추측 항법(Dead Reckoning) 계산
  //    nav2가 보낸 명령(vx_)이 아니라, KinematicsInterface 내부에서
  //    deadband/min_pwm/max_pwm 보정을 거친 후 *실제로 모터에 들어간 PWM*을
  //    기반으로 메카넘 정기구학으로 역산한 effective 속도를 적분한다.
  //    → cmd_vel은 0.1 m/s인데 PWM이 min_pwm(예: 10)으로 끌어올려져 실제로는
  //      더 빠르게 움직이는 경우 등의 거짓말이 odom에 반영됨.
  double eff_vx, eff_vy, eff_wz;
  robot_->get_effective_velocity(eff_vx, eff_vy, eff_wz);

  double delta_x  = (eff_vx * cos(th_) - eff_vy * sin(th_)) * dt;
  double delta_y  = (eff_vx * sin(th_) + eff_vy * cos(th_)) * dt;
  double delta_th = eff_wz * dt;

  x_  += delta_x;
  y_  += delta_y;
  th_ += delta_th;

  // 2. TF 발행 (odom -> base_link)
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = x_;
  t.transform.translation.y = y_;
  t.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, th_);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(t);

  // 3. Odometry 메시지 발행
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation = t.transform.rotation;

  // twist도 명령값(vx_/vy_/wz_)이 아니라 effective 값을 publish해야
  // 컨트롤러/EKF에서 일관된 속도를 보게 됨.
  odom.twist.twist.linear.x  = eff_vx;
  odom.twist.twist.linear.y  = eff_vy;
  odom.twist.twist.angular.z = eff_wz;

  // 2D 차동/메카넘 로봇: z, roll, pitch는 모르므로 매우 큰 분산으로 무시 신호.
  odom.pose.covariance = {1e-3, 0,    0,    0,    0,    0,
                          0,    1e-3, 0,    0,    0,    0,
                          0,    0,    1e6,  0,    0,    0,
                          0,    0,    0,    1e6,  0,    0,
                          0,    0,    0,    0,    1e6,  0,
                          0,    0,    0,    0,    0,    1e-2};
  odom.twist.covariance = {1e-3, 0,    0,    0,    0,    0,
                           0,    1e-3, 0,    0,    0,    0,
                           0,    0,    1e6,  0,    0,    0,
                           0,    0,    0,    1e6,  0,    0,
                           0,    0,    0,    0,    1e6,  0,
                           0,    0,    0,    0,    0,    1e-2};
  odom_publisher_->publish(odom);
}

void DriverNode::check_timeout() {
  if ((this->get_clock()->now() - last_command_time_).seconds() > TIMEOUT_THRESHOLD_) {
    vx_ = 0.0; vy_ = 0.0; wz_ = 0.0;
    robot_->stop();
  }
}