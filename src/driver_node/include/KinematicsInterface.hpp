#ifndef KINEMATICS_INTERFACE_HPP
#define KINEMATICS_INTERFACE_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

/**
 * @class KinematicsInterface
 * @brief I2C 통신을 통해 4륜 메카넘/옴니 로봇을 제어하는 클래스
 */
class KinematicsInterface {
private:
  int fd_;
  std::string i2c_device_ = "/dev/i2c-1";
  const int i2c_addr_ = 0x2B;

  bool is_simulation_ = false; // 시뮬레이션 모드 플래그

  // 로봇 물리 파라미터 (단위: m)
  const double L_ = 0.15;
  const double W_ = 0.15;
  const double K_ = L_ + W_; // 회전 계수

  // drive() 호출 시 모터에 *실제로 들어간* PWM (clamp/deadband 통과 후) 기반으로
  // 메카넘 정기구학으로 역산한 로봇 좌표계 effective 속도.
  // odom 적분은 명령값(vx/vy/wz)이 아니라 이 값을 써야 거짓말이 줄어듦.
  double effective_vx_ = 0.0;
  double effective_vy_ = 0.0;
  double effective_wz_ = 0.0;

public:
  // ── 모터 튜닝 파라미터 (ROS param 으로 주입; 런타임 조정 가능) ─────────
  // 속도 변환 계수 (m/s -> PWM). PWM = round(wheel_speed * speed_scale_)
  double speed_scale_ = 40.0;

  // |PWM| 가 이 값 미만이면 모터가 못 움직이므로 0으로 무시 (jitter 방지)
  int pwm_deadzone_ = 5;

  // 0 < |PWM| < min_pwm_ 인 경우 부호를 유지한 채 ±min_pwm_ 로 끌어올림
  // → DC 모터의 정지 마찰을 이기는 최소 시동 전압 보상
  int min_pwm_ = 5;

  // 상한 (드라이버 보호)
  int max_pwm_ = 255;
private:

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr sim_pub_;

public:
  /**
   * @brief I2C 장치 초기화 및 연결 확인
   */
  KinematicsInterface(const std::string &device = "/dev/i2c-1",
                      uint8_t addr = 0x2B);

  KinematicsInterface(rclcpp::Node::SharedPtr node, 
                      const std::string &device = "/dev/i2c-1",
                      uint8_t addr = 0x2B);
                      
  /**
   * @brief 객체 소멸 시 로봇 정지 및 I2C 닫기
   */
  ~KinematicsInterface();

  /**
   * @brief 개별 모터 제어 (Low-level)
   * @param id 모터 ID (0~3)
   * @param speed 모터 속도 (-255 ~ 255)
   * @return clamp/deadband/min_pwm 보정 후 *실제로 모터에 들어간* PWM 값.
   *         odom 적분에서 실효 속도 역산에 사용.
   */
  int set_motor(int id, int speed);

  /**
   * @brief 마지막 drive() 호출에서 clamp 후 실제로 적용된 effective 속도 조회.
   *        cmd_vel(명령)이 아니라 모터에 진짜로 박힌 PWM 기반이므로 odom 적분 용도.
   */
  void get_effective_velocity(double &vx, double &vy, double &wz) const;

  /**
   * @brief Nav2/ROS 2 표준 속도 명령 인터페이스 (High-level)
   * @param vx 전진 선속도 (m/s)
   * @param vy 측면 선속도 (m/s)
   * @param wz 각속도 (rad/s)
   */
  void drive(double vx, double vy, double wz);

  /**
   * @brief 모든 구동 정지
   */
  void stop();

  /**
   * @brief I2C 통신 연결 상태 확인
   * @return 연결 성공 시 true
   */
  bool is_connected() const;
};

#endif // KINEMATICS_INTERFACE_HPP