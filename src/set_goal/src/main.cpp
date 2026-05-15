#include <cmath>
#include <future>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <print>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class SetGoalNode : public rclcpp::Node
{
public:
    SetGoalNode() : Node("set_goal_node")
    {
        client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    }

    // 서버 접수 응답(future)을 반환하도록 변경
    std::shared_future<GoalHandleNav::SharedPtr> send_goal(double x, double y, double theta)
    {
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action server not available after waiting");
            return {};
        }

        auto goal_msg                    = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id    = "map";
        goal_msg.pose.header.stamp       = this->now();
        goal_msg.pose.pose.position.x    = x;
        goal_msg.pose.pose.position.y    = y;
        goal_msg.pose.pose.orientation.z = std::sin(theta / 2.0);
        goal_msg.pose.pose.orientation.w = std::cos(theta / 2.0);

        RCLCPP_INFO(this->get_logger(), "Sending goal: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);

        // 콜백 설정 없이 그냥 전송만 하고 future를 반환
        return client_ptr_->async_send_goal(goal_msg);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
};

constexpr auto WORLD_MAX = 550;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 터미널 입력 인자 개수 확인
    if (argc < 3) {
        std::println("Usage: ros2 run set_goal set_goal_node <x> <y> [theta]");
        std::println("Example: ros2 run set_goal set_goal_node 200 300 90");
        return 1;
    }

    // 인자 파싱
    double targetX = std::stod(argv[1]);
    double targetY = std::stod(argv[2]);
    double theta   = 0.0;

    auto nav2X = targetX * 0.002 - WORLD_MAX * 0.002 / 2.0;
    auto nav2Y = (WORLD_MAX - targetY) * 0.002 - WORLD_MAX * 0.002 / 2.0;

    if (argc >= 4) {
        theta = std::stod(argv[3]);
    }

    auto node = std::make_shared<SetGoalNode>();

    // 목표를 전송하고 응답 대기 객체(future)를 받음
    auto future = node->send_goal(nav2X, nav2Y, theta);

    if (future.valid()) {
        // 도착할 때까지(spin)가 아니라, 서버가 명령을 "접수"할 때까지만 대기 (통상 수 밀리초 소요)
        if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto goal_handle = future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(node->get_logger(), "Goal accepted by server! Exiting command.");
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to get response from server");
        }
    }

    // 도착 여부와 상관없이 노드 바로 종료
    rclcpp::shutdown();

    return 0;
}
