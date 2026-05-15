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
    // 시뮬레이션 모드: Gazebo용 Twist 메시지 발행
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
  // K = L + W (회전 계수)
  double fl = vx - vy - (K_ * wz);
  double fr = vx + vy + (K_ * wz);
  double rl = vx + vy - (K_ * wz);
  double rr = vx - vy + (K_ * wz);

  // 명령 PWM (round 적용)
  int p_fl_cmd = static_cast<int>(std::round(fl * speed_scale_));
  int p_fr_cmd = static_cast<int>(std::round(fr * speed_scale_));
  int p_rl_cmd = static_cast<int>(std::round(rl * speed_scale_));
  int p_rr_cmd = static_cast<int>(std::round(rr * speed_scale_));

  // 실제로 모터에 들어간 PWM (clamp/deadband/min_pwm 보정 후)
  // ID 매핑: 0=FL, 1=RL, 2=FR, 3=RR
  int p_fl = set_motor(0, p_fl_cmd);
  int p_rl = set_motor(1, p_rl_cmd);
  int p_fr = set_motor(2, p_fr_cmd);
  int p_rr = set_motor(3, p_rr_cmd);

  // PWM → 휠 선속도(m/s) 역산. speed_scale 자체의 비선형성은 못 잡지만,
  // 적어도 deadband/min_pwm 보정으로 인한 양자화는 정직하게 반영됨.
  const double inv_scale = (speed_scale_ != 0.0) ? (1.0 / speed_scale_) : 0.0;
  double wfl = p_fl * inv_scale;
  double wfr = p_fr * inv_scale;
  double wrl = p_rl * inv_scale;
  double wrr = p_rr * inv_scale;

  // 메카넘 정기구학: 휠 속도 → 로봇 좌표계 속도
  effective_vx_ = ( wfl + wfr + wrl + wrr) / 4.0;
  effective_vy_ = (-wfl + wfr + wrl - wrr) / 4.0;
  effective_wz_ = (-wfl + wfr - wrl + wrr) / (4.0 * K_);
}

void KinematicsInterface::get_effective_velocity(double &vx, double &vy, double &wz) const {
  vx = effective_vx_;
  vy = effective_vy_;
  wz = effective_wz_;
}
