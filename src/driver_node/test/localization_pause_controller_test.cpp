#include "LocalizationPauseController.hpp"

#include <gtest/gtest.h>

namespace
{

LocalizationPauseController make_controller()
{
    LocalizationPauseController controller;
    LocalizationPauseController::Config config;
    config.enabled                  = true;
    config.distance_interval_m      = 0.20;
    config.angle_interval_rad       = 0.523599;
    config.settle_sec               = 0.30;
    config.wait_timeout_sec         = 2.0;
    config.post_correction_hold_sec = 0.15;
    config.fault_rearm_zero_sec     = 0.50;
    controller.set_config(config);
    return controller;
}

TEST(LocalizationPauseControllerTest, RunningCorrectionResetsMotion)
{
    auto controller = make_controller();

    EXPECT_FALSE(controller.update_motion(0.1, 0.0, 0.0, 1.0, 1.0));
    EXPECT_NEAR(controller.accumulated_distance(), 0.1, 1e-9);
    EXPECT_EQ(
        controller.on_pose(1.1, 1.1),
        LocalizationPauseController::PoseEvent::RUNNING_CORRECTION
    );
    EXPECT_DOUBLE_EQ(controller.accumulated_distance(), 0.0);
    EXPECT_DOUBLE_EQ(controller.accumulated_angle(), 0.0);
}

TEST(LocalizationPauseControllerTest, DistanceThresholdWaitsForFreshPose)
{
    auto controller = make_controller();

    EXPECT_TRUE(controller.update_motion(0.2, 0.0, 0.0, 1.0, 1.0));
    EXPECT_EQ(controller.state(), LocalizationPauseController::State::SETTLING);
    EXPECT_EQ(controller.on_timer(1.299), LocalizationPauseController::TimerEvent::NONE);
    EXPECT_EQ(
        controller.on_timer(1.301),
        LocalizationPauseController::TimerEvent::WAITING_STARTED
    );
    EXPECT_EQ(controller.state(), LocalizationPauseController::State::WAITING_LCODE);

    EXPECT_EQ(
        controller.on_pose(1.30, 1.32),
        LocalizationPauseController::PoseEvent::IGNORED
    );
    EXPECT_EQ(
        controller.on_pose(1.32, 1.33),
        LocalizationPauseController::PoseEvent::PAUSE_CORRECTION
    );
    EXPECT_FALSE(controller.should_forward_command(false, 1.40));
    EXPECT_TRUE(controller.should_forward_command(false, 1.49));
}

TEST(LocalizationPauseControllerTest, AngleThresholdTriggersPause)
{
    auto controller = make_controller();

    EXPECT_TRUE(controller.update_motion(0.0, 0.0, 0.523599, 1.0, 1.0));
    EXPECT_EQ(controller.state(), LocalizationPauseController::State::SETTLING);
}

TEST(LocalizationPauseControllerTest, TimeoutAbortsOnceAndRearmsAfterZero)
{
    auto controller = make_controller();

    ASSERT_TRUE(controller.update_motion(0.2, 0.0, 0.0, 1.0, 1.0));
    ASSERT_EQ(
        controller.on_timer(1.31),
        LocalizationPauseController::TimerEvent::WAITING_STARTED
    );
    EXPECT_EQ(
        controller.on_timer(3.32),
        LocalizationPauseController::TimerEvent::LOCALIZATION_ABORT
    );
    EXPECT_EQ(controller.on_timer(3.40), LocalizationPauseController::TimerEvent::NONE);
    EXPECT_FALSE(controller.should_forward_command(false, 3.41));
    EXPECT_FALSE(controller.should_forward_command(true, 3.50));
    EXPECT_EQ(
        controller.on_timer(4.01),
        LocalizationPauseController::TimerEvent::FAULT_REARMED
    );
    EXPECT_EQ(controller.state(), LocalizationPauseController::State::PASS_THROUGH);
}

} // namespace
