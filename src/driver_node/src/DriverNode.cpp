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

    // 주기적 L-code 보정 파라미터. 기본은 비활성화하고 실물 launch에서 켠다.
    this->declare_parameter<bool>("periodic_localization_enabled", false);
    this->declare_parameter<double>("localization_distance_interval_m", 0.20);
    this->declare_parameter<double>("localization_angle_interval_rad", 0.523599);
    this->declare_parameter<double>("localization_settle_sec", 0.30);
    this->declare_parameter<double>("localization_wait_timeout_sec", 2.0);
    this->declare_parameter<double>("post_correction_hold_sec", 0.15);
    this->declare_parameter<double>("fault_rearm_zero_sec", 0.50);

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
    periodic_localization_enabled_  = this->get_parameter("periodic_localization_enabled").as_bool();
    localization_distance_interval_m_ = this->get_parameter("localization_distance_interval_m").as_double();
    localization_angle_interval_rad_ = this->get_parameter("localization_angle_interval_rad").as_double();
    localization_settle_sec_ = this->get_parameter("localization_settle_sec").as_double();
    localization_wait_timeout_sec_ = this->get_parameter("localization_wait_timeout_sec").as_double();
    post_correction_hold_sec_ = this->get_parameter("post_correction_hold_sec").as_double();
    fault_rearm_zero_sec_ = this->get_parameter("fault_rearm_zero_sec").as_double();
    update_pause_config();

    RCLCPP_INFO(
        this->get_logger(),
        "Motor params: K=%.3f, speed_scale=%.1f, min_pwm=%d, deadzone=%d, "
        "max_pwm=%d, cmd_timeout=%.2fs, rotation_settle=%.2fs",
        robot_->K_, robot_->speed_scale_, robot_->min_pwm_, robot_->pwm_deadzone_,
        robot_->max_pwm_, cmd_timeout_, settle_duration_
    );
    RCLCPP_INFO(
        this->get_logger(),
        "Periodic localization: %s, distance=%.3fm, angle=%.3frad, settle=%.2fs, timeout=%.2fs",
        periodic_localization_enabled_ ? "enabled" : "disabled",
        localization_distance_interval_m_, localization_angle_interval_rad_,
        localization_settle_sec_, localization_wait_timeout_sec_
    );

    // 런타임 변경 (`ros2 param set /driver_node min_pwm 90` 등) 즉시 반영
    param_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult res;
            res.successful = true;

            bool periodic_enabled = periodic_localization_enabled_;
            double distance_interval = localization_distance_interval_m_;
            double angle_interval = localization_angle_interval_rad_;
            double settle_sec = localization_settle_sec_;
            double wait_timeout_sec = localization_wait_timeout_sec_;
            double correction_hold_sec = post_correction_hold_sec_;
            double rearm_zero_sec = fault_rearm_zero_sec_;

            for (const auto &p : params) {
                if (p.get_name() == "periodic_localization_enabled") periodic_enabled = p.as_bool();
                else if (p.get_name() == "localization_distance_interval_m") distance_interval = p.as_double();
                else if (p.get_name() == "localization_angle_interval_rad") angle_interval = p.as_double();
                else if (p.get_name() == "localization_settle_sec") settle_sec = p.as_double();
                else if (p.get_name() == "localization_wait_timeout_sec") wait_timeout_sec = p.as_double();
                else if (p.get_name() == "post_correction_hold_sec") correction_hold_sec = p.as_double();
                else if (p.get_name() == "fault_rearm_zero_sec") rearm_zero_sec = p.as_double();
            }
            if (distance_interval <= 0.0 || angle_interval <= 0.0 || settle_sec < 0.0
                || wait_timeout_sec <= 0.0 || correction_hold_sec < 0.0
                || rearm_zero_sec < 0.0) {
                res.successful = false;
                res.reason = "localization intervals/timeouts must be positive";
                return res;
            }

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
            periodic_localization_enabled_ = periodic_enabled;
            localization_distance_interval_m_ = distance_interval;
            localization_angle_interval_rad_ = angle_interval;
            localization_settle_sec_ = settle_sec;
            localization_wait_timeout_sec_ = wait_timeout_sec;
            post_correction_hold_sec_ = correction_hold_sec;
            fault_rearm_zero_sec_ = rearm_zero_sec;
            update_pause_config();
            RCLCPP_INFO(
                this->get_logger(),
                "Parameters updated: periodic_localization=%s, distance=%.3fm, angle=%.3frad",
                periodic_localization_enabled_ ? "enabled" : "disabled",
                localization_distance_interval_m_, localization_angle_interval_rad_
            );
            return res;
        }
    );

    // 퍼블리셔 및 TF 브로드캐스터 초기화
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&DriverNode::cmd_vel_callback, this, std::placeholders::_1)
    );
    lcode_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/lcode/pose", rclcpp::QoS(10).reliable(),
        std::bind(&DriverNode::lcode_pose_callback, this, std::placeholders::_1)
    );
    localization_abort_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "/lcode/localization_abort", rclcpp::QoS(10).reliable()
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

bool DriverNode::is_zero_cmd(const geometry_msgs::msg::Twist &t) const
{
    return std::abs(t.linear.x) < linear_speed_threshold_
           && std::abs(t.linear.y) < linear_speed_threshold_
           && std::abs(t.angular.z) < rotation_speed_threshold_;
}

void DriverNode::update_pause_config()
{
    LocalizationPauseController::Config config;
    config.enabled                     = periodic_localization_enabled_;
    config.distance_interval_m         = localization_distance_interval_m_;
    config.angle_interval_rad          = localization_angle_interval_rad_;
    config.settle_sec                  = localization_settle_sec_;
    config.wait_timeout_sec            = localization_wait_timeout_sec_;
    config.post_correction_hold_sec    = post_correction_hold_sec_;
    config.fault_rearm_zero_sec        = fault_rearm_zero_sec_;
    pause_controller_.set_config(config);
}

void DriverNode::stop_for_localization(const char *reason, double settle_override_sec)
{
    auto now = this->get_clock()->now();
    pause_controller_.start_pause(now.seconds(), settle_override_sec);
    robot_->stop();
    vx_ = vy_ = wz_ = 0.0;
    last_was_rotation_ = false;
    RCLCPP_INFO(
        this->get_logger(),
        "L-code 보정 정지 시작 (%s, distance=%.3fm, angle=%.3frad)",
        reason, pause_controller_.accumulated_distance(), pause_controller_.accumulated_angle()
    );
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

    // 보정 중에는 Nav2 명령을 폐기한다. 보정 후 hold가 끝난 뒤 도착한 새 명령만 통과한다.
    if (!pause_controller_.should_forward_command(is_zero_cmd(*msg), now.seconds())) {
        robot_->stop();
        vx_ = vy_ = wz_ = 0.0;
        return;
    }

    // 기존 회전→직진 settle도 동일한 L-code 보정 상태 머신으로 처리한다.
    if (settle_duration_ > 0.0 && last_was_rotation_ && is_linear_cmd(*msg)) {
        stop_for_localization("rotation-to-linear", settle_duration_);
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

    if (pause_controller_.update_motion(
            eff_vx, eff_vy, eff_wz, dt, now.seconds()
        )) {
        stop_for_localization("periodic-threshold");
    }

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

    auto event = pause_controller_.on_timer(now.seconds());
    if (event == LocalizationPauseController::TimerEvent::WAITING_STARTED) {
        RCLCPP_INFO(this->get_logger(), "정지 안정화 완료 — 새 L-code 인식 대기");
    } else if (event == LocalizationPauseController::TimerEvent::LOCALIZATION_ABORT) {
        robot_->stop();
        vx_ = vy_ = wz_ = 0.0;
        localization_abort_publisher_->publish(std_msgs::msg::Empty());
        RCLCPP_ERROR(
            this->get_logger(),
            "L-code 인식이 %.2fs 안에 성공하지 못해 내비게이션 중단 요청",
            localization_wait_timeout_sec_
        );
    } else if (event == LocalizationPauseController::TimerEvent::FAULT_REARMED) {
        last_command_time_ = now;
        timed_out_ = false;
        RCLCPP_INFO(this->get_logger(), "L-code fault 재무장 완료 — 새 목표 수신 가능");
    }

    if (pause_controller_.state() != LocalizationPauseController::State::PASS_THROUGH) {
        last_command_time_ = now;
        return;
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

void DriverNode::lcode_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (msg->header.frame_id != "map") {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "map 프레임이 아닌 L-code pose 무시: %s", msg->header.frame_id.c_str()
        );
        return;
    }

    const auto event = pause_controller_.on_pose(
        rclcpp::Time(msg->header.stamp).seconds(), this->get_clock()->now().seconds()
    );
    if (event == LocalizationPauseController::PoseEvent::PAUSE_CORRECTION) {
        const double recognition_sec = this->get_clock()->now().seconds()
                                       - pause_controller_.waiting_since();
        robot_->stop();
        vx_ = vy_ = wz_ = 0.0;
        RCLCPP_INFO(
            this->get_logger(),
            "새 L-code 보정 수신 (인식 대기 %.3fs) — %.2fs 후 새 cmd_vel부터 주행 재개",
            recognition_sec, post_correction_hold_sec_
        );
    } else if (event == LocalizationPauseController::PoseEvent::RUNNING_CORRECTION) {
        RCLCPP_DEBUG(this->get_logger(), "주행 중 L-code 보정 성공 — 누적 이동량 초기화");
    }
}
