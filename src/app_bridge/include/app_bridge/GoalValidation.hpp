#pragma once

#include <cmath>

namespace app_bridge
{

class GoalValidation
{
public:
    enum class Decision
    {
        SUCCEEDED,
        RETRY,
        FAILED,
    };

    static double distance(double target_x, double target_y, double pose_x, double pose_y)
    {
        return std::hypot(target_x - pose_x, target_y - pose_y);
    }

    static Decision evaluate(
        double target_x,
        double target_y,
        double pose_x,
        double pose_y,
        double tolerance_m,
        int retries_completed,
        int max_retries
    )
    {
        if (distance(target_x, target_y, pose_x, pose_y) <= tolerance_m) {
            return Decision::SUCCEEDED;
        }
        return retries_completed < max_retries ? Decision::RETRY : Decision::FAILED;
    }
};

} // namespace app_bridge
