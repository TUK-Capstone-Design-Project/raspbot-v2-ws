#ifndef LOCALIZATION_PAUSE_CONTROLLER_HPP
#define LOCALIZATION_PAUSE_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <limits>

class LocalizationPauseController
{
public:
    enum class State
    {
        PASS_THROUGH,
        SETTLING,
        WAITING_LCODE,
        POST_CORRECTION_HOLD,
        FAULT_HOLD,
    };

    enum class TimerEvent
    {
        NONE,
        WAITING_STARTED,
        LOCALIZATION_ABORT,
        FAULT_REARMED,
    };

    enum class PoseEvent
    {
        IGNORED,
        RUNNING_CORRECTION,
        PAUSE_CORRECTION,
    };

    struct Config
    {
        bool   enabled{ false };
        double distance_interval_m{ 0.20 };
        double angle_interval_rad{ 0.523599 };
        double settle_sec{ 0.30 };
        double wait_timeout_sec{ 2.0 };
        double post_correction_hold_sec{ 0.0 };
        double fault_rearm_zero_sec{ 0.50 };
    };

    void set_config(const Config &config)
    {
        config_ = config;
    }

    const Config &config() const
    {
        return config_;
    }

    State state() const
    {
        return state_;
    }

    double accumulated_distance() const
    {
        return accumulated_distance_m_;
    }

    double accumulated_angle() const
    {
        return accumulated_angle_rad_;
    }

    double waiting_since() const
    {
        return waiting_since_sec_;
    }

    bool update_motion(double vx, double vy, double wz, double dt, double now_sec)
    {
        if (!config_.enabled || state_ != State::PASS_THROUGH || dt <= 0.0) {
            return false;
        }

        accumulated_distance_m_ += std::hypot(vx, vy) * dt;
        accumulated_angle_rad_ += std::abs(wz) * dt;

        if (accumulated_distance_m_ >= config_.distance_interval_m
            || accumulated_angle_rad_ >= config_.angle_interval_rad) {
            start_pause(now_sec, config_.settle_sec);
            return true;
        }
        return false;
    }

    void start_pause(double now_sec, double settle_override_sec = -1.0)
    {
        if (state_ != State::PASS_THROUGH) return;
        state_             = State::SETTLING;
        state_started_sec_ = now_sec;
        active_settle_sec_ = settle_override_sec >= 0.0
                                 ? settle_override_sec
                                 : config_.settle_sec;
        zero_command_since_sec_ = unset_time();
    }

    PoseEvent on_pose(double pose_stamp_sec, double now_sec)
    {
        if (!std::isfinite(pose_stamp_sec) || pose_stamp_sec <= last_pose_stamp_sec_) {
            return PoseEvent::IGNORED;
        }
        last_pose_stamp_sec_ = pose_stamp_sec;

        if (state_ == State::PASS_THROUGH) {
            reset_accumulators();
            return PoseEvent::RUNNING_CORRECTION;
        }

        if (state_ == State::WAITING_LCODE && pose_stamp_sec >= waiting_since_sec_) {
            reset_accumulators();
            state_             = State::POST_CORRECTION_HOLD;
            state_started_sec_ = now_sec;
            return PoseEvent::PAUSE_CORRECTION;
        }

        return PoseEvent::IGNORED;
    }

    TimerEvent on_timer(double now_sec)
    {
        if (state_ == State::SETTLING
            && now_sec - state_started_sec_ >= active_settle_sec_) {
            state_               = State::WAITING_LCODE;
            state_started_sec_   = now_sec;
            waiting_since_sec_   = now_sec;
            return TimerEvent::WAITING_STARTED;
        }

        if (state_ == State::WAITING_LCODE
            && now_sec - state_started_sec_ >= config_.wait_timeout_sec) {
            state_                   = State::FAULT_HOLD;
            state_started_sec_       = now_sec;
            zero_command_since_sec_  = unset_time();
            return TimerEvent::LOCALIZATION_ABORT;
        }

        if (state_ == State::FAULT_HOLD
            && std::isfinite(zero_command_since_sec_)
            && now_sec - zero_command_since_sec_ >= config_.fault_rearm_zero_sec) {
            state_                   = State::PASS_THROUGH;
            state_started_sec_       = now_sec;
            zero_command_since_sec_  = unset_time();
            reset_accumulators();
            return TimerEvent::FAULT_REARMED;
        }

        return TimerEvent::NONE;
    }

    bool should_forward_command(bool is_zero_command, double now_sec)
    {
        if (state_ == State::PASS_THROUGH) return true;

        if (state_ == State::POST_CORRECTION_HOLD) {
            if (now_sec - state_started_sec_ >= config_.post_correction_hold_sec) {
                state_             = State::PASS_THROUGH;
                state_started_sec_ = now_sec;
                return true;
            }
            return false;
        }

        if (state_ == State::FAULT_HOLD) {
            if (is_zero_command) {
                if (!std::isfinite(zero_command_since_sec_)) {
                    zero_command_since_sec_ = now_sec;
                }
            } else {
                zero_command_since_sec_ = unset_time();
            }
        }

        return false;
    }

    static const char *state_name(State state)
    {
        switch (state) {
        case State::PASS_THROUGH:
            return "PASS_THROUGH";
        case State::SETTLING:
            return "SETTLING";
        case State::WAITING_LCODE:
            return "WAITING_LCODE";
        case State::POST_CORRECTION_HOLD:
            return "POST_CORRECTION_HOLD";
        case State::FAULT_HOLD:
            return "FAULT_HOLD";
        }
        return "UNKNOWN";
    }

private:
    static double unset_time()
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    void reset_accumulators()
    {
        accumulated_distance_m_ = 0.0;
        accumulated_angle_rad_  = 0.0;
    }

    Config config_;
    State  state_{ State::PASS_THROUGH };

    double accumulated_distance_m_{ 0.0 };
    double accumulated_angle_rad_{ 0.0 };
    double state_started_sec_{ 0.0 };
    double active_settle_sec_{ 0.0 };
    double waiting_since_sec_{ unset_time() };
    double zero_command_since_sec_{ unset_time() };
    double last_pose_stamp_sec_{ -std::numeric_limits<double>::infinity() };
};

#endif // LOCALIZATION_PAUSE_CONTROLLER_HPP
