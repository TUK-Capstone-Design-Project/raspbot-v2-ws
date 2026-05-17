#include "KinematicsInterface.hpp"

KinematicsInterface::KinematicsInterface(const std::string &device,
                                         uint8_t addr)
    : fd_(-1), i2c_device_(device), i2c_addr_(addr) {

  // 1. I2C 장치 파일 열기
  fd_ = open(i2c_device_.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::cerr << "[ERROR] I2C 장치 열기 실패: " << i2c_device_ << std::endl;
    return;
  }

  // 2. 슬레이브 주소 설정
  if (ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0) {
    std::cerr << "[ERROR] I2C 슬레이브 주소 설정 실패: 0x" << std::hex
              << (int)i2c_addr_ << std::endl;
    close(fd_);
    fd_ = -1;
    return;
  }

  std::cout << "[SUCCESS] 로봇 인터페이스 연결 완료 (Addr: 0x" << std::hex
            << (int)i2c_addr_ << ")" << std::endl;
}

KinematicsInterface::KinematicsInterface(rclcpp::Node::SharedPtr node, 
                                         const std::string &device,
                                         uint8_t addr)
    : fd_(-1), i2c_device_(device), i2c_addr_(addr) {

  // 1. I2C 장치 파일 열기 시도
  fd_ = open(i2c_device_.c_str(), O_RDWR);
  
  if (fd_ < 0) {
    // 장치 열기 실패 시 시뮬레이션 모드로 전환
    is_simulation_ = true;
    RCLCPP_WARN(node->get_logger(), "I2C 장치를 찾을 수 없습니다. 시뮬레이션 모드로 동작합니다.");
    
    // Gazebo 로봇 제어를 위한 토픽 발행기 생성
    sim_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_sim", 10);
  } else {
    // 하드웨어 연결 성공 시 기존 I2C 설정 진행
    if (ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0) {
      is_simulation_ = true;
      close(fd_); fd_ = -1;
    }
  }
}

KinematicsInterface::~KinematicsInterface() {
  stop();
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool KinematicsInterface::is_connected() const { return (fd_ >= 0) || is_simulation_; }

void KinematicsInterface::stop() { drive(0.0, 0.0, 0.0); }

int KinematicsInterface::set_motor(int id, int speed) {
  if (fd_ < 0)
    return 0;

  // ── 데드밴드 + 최소시동 PWM 보정 ──────────────────────────────────────
  int abs_in = std::abs(speed);
  int sign   = (speed > 0) - (speed < 0);
  int out;
  if (abs_in == 0 || abs_in < pwm_deadzone_) {
    out = 0; // 의미있는 명령 아님 → 강제 정지
  } else if (abs_in < min_pwm_) {
    out = sign * min_pwm_; // 정지마찰 이기도록 끌어올림
  } else {
    out = std::max(-max_pwm_, std::min(max_pwm_, speed));
  }

  uint8_t dir = (out < 0) ? 1 : 0;
  uint8_t abs_speed = static_cast<uint8_t>(std::abs(out));
  uint8_t motor_id = static_cast<uint8_t>(id);

  // 야붐(Yahboom) 등의 표준 프로토콜: [Register, ID, Direction, Speed]
  uint8_t buffer[4] = {0x01, motor_id, dir, abs_speed};

  if (write(fd_, buffer, 4) != 4) {
    // 통신 노이즈 시 경고를 출력하려면 주석 해제
    // std::cerr << "[WARN] I2C Write Failed for Motor " << id << std::endl;
  }

  // RPi 5 및 I2C 버스 안정화를 위한 미세 지연
  std::this_thread::sleep_for(std::chrono::microseconds(500));

  return out;
}

void KinematicsInterface::drive(double vx, double vy, double wz) {
  if (is_simulation_) {
    // 시뮬레이션 모드: Gazebo용 Twist 메시지 발행 (펄스 변조 없음)
    geometry_msgs::msg::Twist msg;
    msg.linear.x = vx;
    msg.linear.y = vy;
    msg.angular.z = wz;
    sim_pub_->publish(msg);
    effective_vx_ = vx;
    effective_vy_ = vy;
    effective_wz_ = wz;
    return;
  }

  if (fd_ < 0) {
    effective_vx_ = 0.0;
    effective_vy_ = 0.0;
    effective_wz_ = 0.0;
    return;
  }

  // [메카넘 휠 역기구학 공식]
  // K = L + W (회전 계수). 선속도(vx,vy)와 각속도(wz) 성분을 분리해 저장.
  // 변조 모드에서 성분별로 다른 게인을 적용하기 위함.
  double fl_lin = (vx - vy) * speed_scale_;
  double fr_lin = (vx + vy) * speed_scale_;
  double rl_lin = (vx + vy) * speed_scale_;
  double rr_lin = (vx - vy) * speed_scale_;
  double fl_ang = -K_ * wz * speed_scale_;
  double fr_ang =  K_ * wz * speed_scale_;
  double rl_ang = -K_ * wz * speed_scale_;
  double rr_ang =  K_ * wz * speed_scale_;

  // ID 매핑: 0=FL, 1=RL, 2=FR, 3=RR
  intended_linear_pwm_[0]  = fl_lin;
  intended_linear_pwm_[1]  = rl_lin;
  intended_linear_pwm_[2]  = fr_lin;
  intended_linear_pwm_[3]  = rr_lin;
  intended_angular_pwm_[0] = fl_ang;
  intended_angular_pwm_[1] = rl_ang;
  intended_angular_pwm_[2] = fr_ang;
  intended_angular_pwm_[3] = rr_ang;
  // 연속 모드 판단 및 출력에 쓰이는 raw 합산값
  intended_pwm_[0] = fl_lin + fl_ang;
  intended_pwm_[1] = rl_lin + rl_ang;
  intended_pwm_[2] = fr_lin + fr_ang;
  intended_pwm_[3] = rr_lin + rr_ang;

  // odom용 effective 속도: tick()의 펄스 변조 평균이 의도 PWM과 일치하므로
  // 의도값 그대로 사용. 단, |의도| < deadzone 인 휠은 실제로 못 움직이므로 0으로 반영.
  auto eff_wheel = [this](double p) {
    return (std::abs(p) < pwm_deadzone_) ? 0.0 : p / speed_scale_;
  };
  double wfl = eff_wheel(intended_pwm_[0]);
  double wrl = eff_wheel(intended_pwm_[1]);
  double wfr = eff_wheel(intended_pwm_[2]);
  double wrr = eff_wheel(intended_pwm_[3]);

  // 메카넘 정기구학: 휠 속도 → 로봇 좌표계 속도
  effective_vx_ = ( wfl + wfr + wrl + wrr) / 4.0;
  effective_vy_ = (-wfl + wfr + wrl - wrr) / 4.0;
  effective_wz_ = (-wfl + wfr - wrl + wrr) / (4.0 * K_);
}

void KinematicsInterface::tick() {
  if (is_simulation_ || fd_ < 0) return;

  for (int i = 0; i < 4; ++i) {
    double intended = intended_pwm_[i];
    double abs_intended = std::abs(intended);

    if (abs_intended >= min_pwm_) {
      // 충분히 큰 PWM — 그대로 출력 (연속 모드)
      set_motor(i, static_cast<int>(std::round(intended)));
      accumulator_[i] = 0.0;
      pulse_remaining_[i] = 0;
      was_modulating_[i] = false;
      continue;
    }

    // 변조 모드 후보: 성분별 게인 가중합 계산
    // (deadzone 체크를 weighted 기준으로 해야 게인이 작은 raw intended 도 살릴 수 있다)
    double weighted = intended_linear_pwm_[i]  * linear_gain_
                    + intended_angular_pwm_[i] * angular_gain_;

    // |weighted| > min_pwm 이면 누산기 발산 → ±min_pwm 으로 캡 (실효적 100% 듀티).
    if (std::abs(weighted) > min_pwm_) {
      weighted = (weighted > 0 ? 1 : -1) * static_cast<double>(min_pwm_);
    }
    double abs_weighted = std::abs(weighted);

    if (abs_weighted < pwm_deadzone_) {
      // weighted 도 deadzone 미만이면 진짜 무의미한 명령 — 정지
      set_motor(i, 0);
      accumulator_[i] = 0.0;
      pulse_remaining_[i] = 0;
      was_modulating_[i] = false;
      continue;
    }

    // 변조 모드 진입. 펄스 방향은 weighted 부호 기준 (실제 actuate 방향과 일치).
    int sign = (weighted > 0) ? 1 : -1;

    // Kickstart: 모듈레이션 모드 첫 진입 시 누산기를 threshold로 미리 채워
    // 첫 펄스 발사 지연 (large pulse_ticks에서 수 초 걸릴 수 있음)을 제거.
    if (!was_modulating_[i]) {
      accumulator_[i] = sign * static_cast<double>(min_pwm_) * modulation_pulse_ticks_;
      was_modulating_[i] = true;
    }
    accumulator_[i] += weighted;

    if (pulse_remaining_[i] > 0) {
      // 진행 중인 펄스 트레인 — 계속 ON
      set_motor(i, sign * min_pwm_);
      pulse_remaining_[i]--;
    } else {
      double threshold = static_cast<double>(min_pwm_) * modulation_pulse_ticks_;
      if (std::abs(accumulator_[i]) >= threshold) {
        // 새 펄스 트레인 시작 (첫 틱은 지금 ON, 나머지 (N-1)틱은 다음 호출에서)
        sign = (accumulator_[i] > 0) ? 1 : -1;
        set_motor(i, sign * min_pwm_);
        accumulator_[i] -= sign * threshold;
        pulse_remaining_[i] = modulation_pulse_ticks_ - 1;
      } else {
        set_motor(i, 0);
      }
    }
  }
}

void KinematicsInterface::get_effective_velocity(double &vx, double &vy, double &wz) const {
  vx = effective_vx_;
  vy = effective_vy_;
  wz = effective_wz_;
}
