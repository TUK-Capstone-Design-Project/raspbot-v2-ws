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

  // drive() 호출 시 모터에 *실제로 들어간* PWM (clamp/deadband 통과 후) 기반으로
  // 메카넘 정기구학으로 역산한 로봇 좌표계 effective 속도.
  // odom 적분은 명령값(vx/vy/wz)이 아니라 이 값을 써야 거짓말이 줄어듦.
  double effective_vx_ = 0.0;
  double effective_vy_ = 0.0;
  double effective_wz_ = 0.0;

  // 휠별 의도 PWM (drive()가 매번 갱신). tick()이 이 값을 보고 펄스 변조한다.
  // ID 매핑: 0=FL, 1=RL, 2=FR, 3=RR (set_motor와 동일)
  double intended_pwm_[4] = {0.0, 0.0, 0.0, 0.0};

  // 의도 PWM의 선속도/각속도 성분 분리. 변조 모드에서 성분별 게인 가중합에 사용.
  double intended_linear_pwm_[4]  = {0.0, 0.0, 0.0, 0.0};
  double intended_angular_pwm_[4] = {0.0, 0.0, 0.0, 0.0};

  // Bresenham 듀티 변조 누산기. |intended| < min_pwm 일 때만 사용.
  double accumulator_[4] = {0.0, 0.0, 0.0, 0.0};

  // 연속 펄스 잔여 틱 수. 펄스를 여러 틱 연속 ON 유지해서 모터 startup 손실을 줄임.
  int pulse_remaining_[4] = {0, 0, 0, 0};

  // 이전 tick에서 듀티 변조 모드였는지. false→true 전환 시 누산기 kickstart로 첫 펄스 즉시 발사.
  bool was_modulating_[4] = {false, false, false, false};

public:
  // ── 모터 튜닝 파라미터 (ROS param 으로 주입; 런타임 조정 가능) ─────────
  // 회전 계수 K = L + W (바퀴 배치 기반). 로봇이 과회전하면 줄이고, 과소회전하면 늘린다.
  // 캘리브레이션: wz=1.0으로 T초 회전 후 실제 각도 A(rad)를 측정하면 K_new = K_old × (1.0 / (A/T))
  double K_ = 0.327;

  // 속도 변환 계수 (m/s -> PWM). PWM = round(wheel_speed * speed_scale_)
  double speed_scale_ = 40.0;

  // |PWM| 가 이 값 미만이면 모터가 못 움직이므로 0으로 무시 (jitter 방지)
  int pwm_deadzone_ = 5;

  // 0 < |PWM| < min_pwm_ 인 경우 부호를 유지한 채 ±min_pwm_ 로 끌어올림
  // → DC 모터의 정지 마찰을 이기는 최소 시동 전압 보상
  int min_pwm_ = 5;

  // 상한 (드라이버 보호)
  int max_pwm_ = 255;

  // |intended| < min_pwm 일 때 연속 ON 유지할 틱 수. 늘리면 모터 startup 손실 ↓, stutter ↑.
  // 평균 출력(=intended)은 그대로 유지된다 (idle 시간이 비례해서 늘어남).
  int modulation_pulse_ticks_ = 5;

  // 듀티 변조 모드에서 성분별로 적용하는 게인. 모터 비선형성 + 비대칭 startup 손실 보정용.
  // 회전과 선속도의 startup 효율이 매우 다를 수 있어 분리. R = 실제/명령, gain = 1/R.
  // 누산기에 가중합: weighted = linear_pwm × linear_gain + angular_pwm × angular_gain
  double linear_gain_  = 1.5;   // 선속도 성분 게인. 보통 > 1 (translation under-rotation 보정)
  double angular_gain_ = 0.64;  // 각속도 성분 게인. 보통 < 1 (rotation over-rotation 보정)
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
   * @brief 모터 출력 펄스 변조 타이머 콜백 (DriverNode에서 50Hz로 호출).
   *        |intended_pwm| < min_pwm 인 휠은 Bresenham 누산으로 펄스 변조하여
   *        평균 출력이 의도값과 일치하도록 한다. 정지마찰 회피와 저속 정밀도를 동시에 확보.
   */
  void tick();

  /**
   * @brief I2C 통신 연결 상태 확인
   * @return 연결 성공 시 true
   */
  bool is_connected() const;
};

#endif // KINEMATICS_INTERFACE_HPP