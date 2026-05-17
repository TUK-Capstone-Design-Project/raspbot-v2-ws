#include "DriverNode.hpp"
#include <tf2/LinearMath/Quaternion.h>

DriverNode::DriverNode() : Node("driver_node"), x_(0.0), y_(0.0), th_(0.0), vx_(0.0), vy_(0.0), wz_(0.0)
{
    robot_ = std::make_unique<KinematicsInterface>("/dev/i2c-1", 0x2B);

    // ── 모터 튜닝 파라미터 (ros2 param set 으로 런타임 조정) ──────────────
    // PWM = round(wheel_speed_m_per_s * speed_scale)
    // K = L + W (m). 과회전 시 줄이고, 과소회전 시 늘린다. 기본값 0.30
    this->declare_parameter<double>("wheel_rotation_k", 0.327);
    this->declare_parameter<double>("speed_scale", 40.0);
    // 0 < |PWM| < min_pwm 인 경우 ±min_pwm 으로 끌어올려 정지마찰 보상
    this->declare_parameter<int>("min_pwm", 15);
    // |PWM| < pwm_deadzone 이면 0 으로 무시 (지터 방지)
    this->declare_parameter<int>("pwm_deadzone", 1);
    this->declare_parameter<int>("max_pwm", 255);
    // 듀티 변조 시 연속 ON 유지 틱 수 (×100ms). 크면 stutter ↑ 회전량 ↑.
    this->declare_parameter<int>("modulation_pulse_ticks", 5);
    // 변조 모드 게인 (성분별 분리). gain = 1 / (실제/명령 비율).
    // 선속도는 startup 손실 ↑ → gain > 1 로 보정. 각속도는 비선형 과출력 → gain < 1.
    this->declare_parameter<double>("linear_gain", 1.5);
    this->declare_parameter<double>("angular_gain", 0.64);
    // cmd_vel 미수신 허용 시간. Nav2 컨트롤러가 10Hz를 못 맞춰 cmd_vel 간격이
    // 들쭉날쭉할 때 너무 짧으면 stop이 반복 발동되어 끊김 현상이 생긴다.
    this->declare_parameter<double>("cmd_timeout", 0.3);
    // this->declare_parameter<double>("cmd_timeout", 0.1);

    // 회전→직진 전환 settle 파라미터.
    // 제자리 회전을 끝내고 처음으로 선형 이동 명령이 들어오는 순간 짧게 정지해서
    // lcode_localizer 가 map→odom 을 새 인식으로 보정할 시간을 확보한다.
    // 0 이면 비활성화. 카메라 fps 와 인식 지연을 고려해 0.3~0.6s 권장.
    this->declare_parameter<double>("settle_after_rotation", 0);
    this->declare_parameter<double>("rotation_speed_threshold", 0.05); // |wz| (rad/s)
    this->declare_parameter<double>("linear_speed_threshold", 0.02);   // |vx|/|vy| (m/s)

    // 초기값 적용
    robot_->K_                      = this->get_parameter("wheel_rotation_k").as_double();
    robot_->speed_scale_            = this->get_parameter("speed_scale").as_double();
    robot_->min_pwm_                = this->get_parameter("min_pwm").as_int();
    robot_->pwm_deadzone_           = this->get_parameter("pwm_deadzone").as_int();
    robot_->max_pwm_                = this->get_parameter("max_pwm").as_int();
    robot_->modulation_pulse_ticks_ = this->get_parameter("modulation_pulse_ticks").as_int();
    robot_->linear_gain_            = this->get_parameter("linear_gain").as_double();
    robot_->angular_gain_           = this->get_parameter("angular_gain").as_double();
    cmd_timeout_                    = this->get_parameter("cmd_timeout").as_double();
    settle_duration_                = this->get_parameter("settle_after_rotation").as_double();
    rotation_speed_threshold_       = this->get_parameter("rotation_speed_threshold").as_double();
    linear_speed_threshold_         = this->get_parameter("linear_speed_threshold").as_double();
    RCLCPP_INFO(this->get_logger(), "Motor params: K=%.3f, speed_scale=%.1f, min_pwm=%d, deadzone=%d, max_pwm=%d, cmd_timeout=%.2fs, settle=%.2fs", robot_->K_, robot_->speed_scale_, robot_->min_pwm_, robot_->pwm_deadzone_, robot_->max_pwm_, cmd_timeout_, settle_duration_);

    // 런타임 변경 (`ros2 param set /driver_node min_pwm 90` 등) 즉시 반영
    param_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult res;
            res.successful = true;
            for (const auto &p : params) {
                if (p.get_name() == "wheel_rotation_k") robot_->K_ = p.as_double();
                else if (p.get_name() == "speed_scale") robot_->speed_scale_ = p.as_double();
                else if (p.get_name() == "min_pwm") robot_->min_pwm_ = p.as_int();
                else if (p.get_name() == "pwm_deadzone") robot_->pwm_deadzone_ = p.as_int();
                else if (p.get_name() == "max_pwm") robot_->max_pwm_ = p.as_int();
                else if (p.get_name() == "modulation_pulse_ticks") robot_->modulation_pulse_ticks_ = p.as_int();
                else if (p.get_name() == "linear_gain") robot_->linear_gain_ = p.as_double();
                else if (p.get_name() == "angular_gain") robot_->angular_gain_ = p.as_double();
                else if (p.get_name() == "cmd_timeout") cmd_timeout_ = p.as_double();
                else if (p.get_name() == "settle_after_rotation") settle_duration_ = p.as_double();
                else if (p.get_name() == "rotation_speed_threshold") rotation_speed_threshold_ = p.as_double();
                else if (p.get_name() == "linear_speed_threshold") linear_speed_threshold_ = p.as_double();
            }
            RCLCPP_INFO(this->get_logger(), "Motor params updated: K=%.3f, speed_scale=%.1f, min_pwm=%d, deadzone=%d, max_pwm=%d, cmd_timeout=%.2fs, settle=%.2fs", robot_->K_, robot_->speed_scale_, robot_->min_pwm_, robot_->pwm_deadzone_, robot_->max_pwm_, cmd_timeout_, settle_duration_);
            return res;
        }
    );

    // 퍼블리셔 및 TF 브로드캐스터 초기화
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&DriverNode::cmd_vel_callback, this, std::placeholders::_1)
    );

    // 오도메트리 업데이트 타이머 (50Hz)
    last_odom_time_ = this->get_clock()->now();
    odom_timer_     = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&DriverNode::update_odometry, this)
    );

    last_command_time_ = this->get_clock()->now();
    timeout_timer_     = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&DriverNode::check_timeout, this)
    );

    // 모터 출력 타이머 (10Hz, 100ms 주기). drive()가 저장한 의도 PWM을 펄스 변조해 I2C로 전송.
    // 100ms 펄스는 DC 모터의 startup 시정수(수십 ms)를 충분히 초과해 정지마찰을 매번 깰 수 있다.
    // 더 짧으면(20ms) 코일에 전류는 흐르지만 회전 시작 전에 OFF되어 소리만 나고 안 움직임.
    motor_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { robot_->tick(); }
    );
}

bool DriverNode::is_rotation_cmd(const geometry_msgs::msg::Twist &t) const
{
    return std::abs(t.angular.z) >= rotation_speed_threshold_
           && std::abs(t.linear.x) < linear_speed_threshold_
           && std::abs(t.linear.y) < linear_speed_threshold_;
}

bool DriverNode::is_linear_cmd(const geometry_msgs::msg::Twist &t) const
{
    return (std::abs(t.linear.x) >= linear_speed_threshold_
            || std::abs(t.linear.y) >= linear_speed_threshold_)
           && std::abs(t.angular.z) < rotation_speed_threshold_;
}

void DriverNode::apply_cmd(const geometry_msgs::msg::Twist &t)
{
    vx_ = t.linear.x;
    vy_ = t.linear.y;
    wz_ = t.angular.z;
    robot_->drive(vx_, vy_, wz_);
    last_was_rotation_ = is_rotation_cmd(t);
}

void DriverNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!robot_->is_connected()) {
        RCLCPP_ERROR(this->get_logger(), "I2C 통신 연결 실패");
        return;
    }
    auto now           = this->get_clock()->now();
    last_command_time_ = now;
    timed_out_         = false; // 새 명령 도착 — 타임아웃 상태 해제

    // settle 모드 진행 중인 경우: 명령은 보존만 하고 모터에는 보내지 않는다.
    // 단, 새 명령이 다시 회전이면 settle 의미가 없으므로 즉시 종료하고 발행.
    if (settling_) {
        if (is_rotation_cmd(*msg)) {
            settling_ = false;
            RCLCPP_INFO(this->get_logger(), "settle 중 회전 재요청 — 즉시 재개");
            apply_cmd(*msg);
        } else {
            pending_cmd_ = *msg; // 최신 명령만 유지
        }
        return;
    }

    // 회전 → 선형 이동 전환 감지: settle 진입
    if (settle_duration_ > 0.0 && last_was_rotation_ && is_linear_cmd(*msg)) {
        settling_          = true;
        settle_start_time_ = now;
        pending_cmd_       = *msg;
        robot_->stop();
        vx_ = vy_ = wz_ = 0.0;
        last_was_rotation_ = false; // settle 중에는 회전 상태가 아님
        RCLCPP_INFO(this->get_logger(),
                    "회전→직진 전환 감지 — %.2fs settle 시작 (map→odom 보정 대기)",
                    settle_duration_);
        return;
    }

    apply_cmd(*msg);
}

void DriverNode::update_odometry()
{
    auto   now      = this->get_clock()->now();
    double dt       = (now - last_odom_time_).seconds();
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

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    // 2. TF 발행 (odom -> base_link)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp            = now;
    t.header.frame_id         = "odom";
    t.child_frame_id          = "base_link";
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
    odom.header.stamp          = now;
    odom.header.frame_id       = "odom";
    odom.child_frame_id        = "base_link";
    odom.pose.pose.position.x  = x_;
    odom.pose.pose.position.y  = y_;
    odom.pose.pose.orientation = t.transform.rotation;

    // twist도 명령값(vx_/vy_/wz_)이 아니라 effective 값을 publish해야
    // 컨트롤러/EKF에서 일관된 속도를 보게 됨.
    odom.twist.twist.linear.x  = eff_vx;
    odom.twist.twist.linear.y  = eff_vy;
    odom.twist.twist.angular.z = eff_wz;

    // 2D 차동/메카넘 로봇: z, roll, pitch는 모르므로 매우 큰 분산으로 무시 신호.
    odom.pose.covariance  = { 1e-3, 0, 0, 0, 0, 0,
                              0, 1e-3, 0, 0, 0, 0,
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0,
                              0, 0, 0, 0, 1e6, 0,
                              0, 0, 0, 0, 0, 1e-2 };
    odom.twist.covariance = { 1e-3, 0, 0, 0, 0, 0,
                              0, 1e-3, 0, 0, 0, 0,
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0,
                              0, 0, 0, 0, 1e6, 0,
                              0, 0, 0, 0, 0, 1e-2 };
    odom_publisher_->publish(odom);
}

void DriverNode::check_timeout()
{
    auto now = this->get_clock()->now();

    // settle 처리 우선: 시간이 다 되면 그동안 수집한 최신 명령을 적용한다.
    if (settling_) {
        if ((now - settle_start_time_).seconds() >= settle_duration_) {
            settling_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "settle 종료 — pending cmd 발행 (vx=%.2f, vy=%.2f, wz=%.2f)",
                        pending_cmd_.linear.x, pending_cmd_.linear.y, pending_cmd_.angular.z);
            apply_cmd(pending_cmd_);
            last_command_time_ = now; // settle 직후 cmd_timeout으로 또 stop 들어가는 일 방지
        } else {
            last_command_time_ = now; // settle 중엔 timeout 카운트도 멈춰둠
        }
        return; // settle 중/직후엔 아래 timeout 로직 건너뜀
    }

    if (timed_out_) return; // 이미 정지 상태면 I²C로 stop 중복 전송 안 함
    if ((now - last_command_time_).seconds() > cmd_timeout_) {
        vx_ = 0.0;
        vy_ = 0.0;
        wz_ = 0.0;
        robot_->stop();
        timed_out_         = true;
        last_was_rotation_ = false; // 정지 상태에선 회전 이력도 리셋
        RCLCPP_WARN(this->get_logger(), "cmd_vel 미수신 %.2fs 초과 — 모터 정지", cmd_timeout_);
    }
}