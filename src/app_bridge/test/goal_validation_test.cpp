#include <gtest/gtest.h>

#include "app_bridge/GoalValidation.hpp"

using app_bridge::GoalValidation;

TEST(GoalValidationTest, SucceedsInsideToleranceIncludingBoundary)
{
    EXPECT_EQ(
        GoalValidation::evaluate(0.0, 0.0, 0.03, 0.0, 0.03, 0, 2),
        GoalValidation::Decision::SUCCEEDED
    );
}

TEST(GoalValidationTest, RetriesAtMostTwice)
{
    EXPECT_EQ(
        GoalValidation::evaluate(1.0, 2.0, 1.04, 2.0, 0.03, 0, 2),
        GoalValidation::Decision::RETRY
    );
    EXPECT_EQ(
        GoalValidation::evaluate(1.0, 2.0, 1.04, 2.0, 0.03, 1, 2),
        GoalValidation::Decision::RETRY
    );
    EXPECT_EQ(
        GoalValidation::evaluate(1.0, 2.0, 1.04, 2.0, 0.03, 2, 2),
        GoalValidation::Decision::FAILED
    );
}
