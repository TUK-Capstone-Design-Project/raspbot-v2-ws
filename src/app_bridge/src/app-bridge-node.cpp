#include "app_bridge/app-bridge-node.hpp"

#include <cmath>
#include <stdexcept>
#include <tf2/utils.h>

namespace app_bridge
{

// ════════════════════════════════════════════════════════
//  WsSession 구현
// ════════════════════════════════════════════════════════

WsSession::WsSession(tcp::socket socket) : ws_(std::move(socket))
{
    // 핸드셰이크 전에 주소를 저장 (나중에 소켓이 닫히면 접근 불가)
    try {
        auto ep      = beast::get_lowest_layer(ws_).socket().remote_endpoint();
        remote_addr_ = ep.address().to_string() + ":" + std::to_string(ep.port());
    } catch (...) {
        remote_addr_ = "unknown";
    }
}

void WsSession::run(
    std::function<void(const std::string &, Ptr)> on_message,
    std::function<void(Ptr)>                      on_close
)
{
    on_message_ = std::move(on_message);
    on_close_   = std::move(on_close);

    // 텍스트 모드 (JSON)
    ws_.text(true);

    // 비동기 WebSocket 핸드셰이크 수락
    ws_.async_accept(
        beast::bind_front_handler(&WsSession::on_accept, shared_from_this())
    );
}

void WsSession::on_accept(beast::error_code ec)
{
    if (ec) {
        on_close_(shared_from_this());
        return;
    }
    do_read();
}

void WsSession::send(const std::string &data)
{
    auto self = shared_from_this();
    auto buf  = std::make_shared<std::string>(data);

    // post를 통해 strand에서 쓰기 (동시 쓰기 방지)
    boost::asio::post(
        beast::get_lowest_layer(ws_).socket().get_executor(),
        [this, self, buf]() {
            beast::error_code ec;
            ws_.write(boost::asio::buffer(*buf), ec);
            // 실패 시 다음 read에서 감지
        }
    );
}

std::string WsSession::remote_addr() const
{
    return remote_addr_;
}

void WsSession::do_read()
{
    read_buf_.clear();
    ws_.async_read(
        read_buf_,
        beast::bind_front_handler(&WsSession::on_read, shared_from_this())
    );
}

void WsSession::on_read(beast::error_code ec, std::size_t /*bytes_transferred*/)
{
    if (ec) {
        on_close_(shared_from_this());
        return;
    }

    std::string text = beast::buffers_to_string(read_buf_.data());
    if (!text.empty()) {
        on_message_(text, shared_from_this());
    }

    do_read();
}

// ════════════════════════════════════════════════════════
//  AppBridgeNode 구현
// ════════════════════════════════════════════════════════

AppBridgeNode::AppBridgeNode(const rclcpp::NodeOptions &options) : Node("app_bridge_node", options),
                                                                   acceptor_(io_ctx_)
{
    // 파라미터 선언
    this->declare_parameter("port", 5000);
    this->declare_parameter("pose_publish_hz", 2.0);
    this->declare_parameter("final_validation_tolerance_m", 0.03);
    this->declare_parameter("final_validation_timeout_sec", 2.0);
    this->declare_parameter("final_validation_max_retries", 2);
    port_                         = this->get_parameter("port").as_int();
    pose_publish_hz_              = this->get_parameter("pose_publish_hz").as_double();
    final_validation_tolerance_m_ = this->get_parameter("final_validation_tolerance_m").as_double();
    final_validation_timeout_sec_ = this->get_parameter("final_validation_timeout_sec").as_double();
    final_validation_max_retries_ = this->get_parameter("final_validation_max_retries").as_int();

    if (pose_publish_hz_ <= 0.0 || final_validation_tolerance_m_ <= 0.0
        || final_validation_timeout_sec_ <= 0.0 || final_validation_max_retries_ < 0) {
        throw std::invalid_argument("pose/validation parameters must be positive");
    }

    // Nav2 액션 클라이언트
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // TF 리스너
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 로봇 위치 주기적 전송 타이머
    auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / pose_publish_hz_)
    );
    pose_timer_ = this->create_wall_timer(
        period, std::bind(&AppBridgeNode::publish_robot_pose, this)
    );
    validation_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&AppBridgeNode::check_validation_timeout, this)
    );

    lcode_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/lcode/pose", rclcpp::QoS(10).reliable(),
        std::bind(&AppBridgeNode::on_lcode_pose, this, std::placeholders::_1)
    );
    localization_abort_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "/lcode/localization_abort", rclcpp::QoS(10).reliable(),
        std::bind(&AppBridgeNode::on_localization_abort, this, std::placeholders::_1)
    );

    // TCP Acceptor 설정
    tcp::endpoint ep(tcp::v4(), static_cast<unsigned short>(port_));
    acceptor_.open(ep.protocol());
    acceptor_.set_option(tcp::acceptor::reuse_address(true));
    acceptor_.bind(ep);
    acceptor_.listen(boost::asio::socket_base::max_listen_connections);
    start_accept();

    // io_context를 별도 스레드에서 구동
    io_thread_ = std::thread([this]() { io_ctx_.run(); });

    RCLCPP_INFO(
        this->get_logger(),
        "App Bridge started — WebSocket port %d, pose @ %.1f Hz, final tolerance %.3fm",
        port_, pose_publish_hz_, final_validation_tolerance_m_
    );
}

AppBridgeNode::~AppBridgeNode()
{
    io_ctx_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

// ── WebSocket 서버 ─────────────────────────────────────

void AppBridgeNode::start_accept()
{
    acceptor_.async_accept(
        [this](boost::system::error_code ec, tcp::socket socket) {
            if (!ec) {
                auto session = std::make_shared<WsSession>(std::move(socket));
                RCLCPP_INFO(this->get_logger(), "Android connected: %s", session->remote_addr().c_str());

                {
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    clients_.insert(session);
                }

                session->run(
                    [this](const std::string &text, WsSession::Ptr s) {
                        on_client_message(text, s);
                    },
                    [this](WsSession::Ptr s) {
                        on_client_close(s);
                    }
                );
            }
            start_accept();
        }
    );
}

void AppBridgeNode::on_client_message(
    const std::string &text, WsSession::Ptr /*session*/
)
{
    try {
        auto msg  = json::parse(text);
        auto type = msg.value("type", "");

        if (type == "navigate") {
            double x = msg.at("x").get<double>();
            double y = msg.at("y").get<double>();
            send_nav_goal(x, y);
        } else if (type == "cancel") {
            rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_to_cancel;
            {
                std::lock_guard<std::mutex> lock(nav_state_mutex_);
                goal_to_cancel = current_goal_handle_;
                ++nav_generation_;
                goal_request_pending_         = false;
                navigation_active_            = false;
                awaiting_final_validation_     = false;
                localization_failure_reported_ = false;
                current_goal_handle_.reset();
            }
            if (goal_to_cancel) {
                nav_client_->async_cancel_goal(goal_to_cancel);
            }
            RCLCPP_INFO(this->get_logger(), "Navigation cancelled by app");
            broadcast({
                {   "type", "nav_result" },
                { "status",    "canceled" }
            });
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown message type: %s", type.c_str());
        }
    } catch (const json::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
    }
}

void AppBridgeNode::on_client_close(WsSession::Ptr session)
{
    RCLCPP_INFO(this->get_logger(), "Android disconnected: %s", session->remote_addr().c_str());
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.erase(session);
}

void AppBridgeNode::broadcast(const json &msg)
{
    std::string                 data = msg.dump();
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (auto &client : clients_) {
        client->send(data);
    }
}

// ── Nav2 액션 ──────────────────────────────────────────

void AppBridgeNode::send_nav_goal(double x, double y)
{
    std::uint64_t generation;
    {
        std::lock_guard<std::mutex> lock(nav_state_mutex_);
        generation                           = ++nav_generation_;
        target_x_                            = x;
        target_y_                            = y;
        final_validation_retries_completed_  = 0;
        goal_request_pending_                 = false;
        navigation_active_                   = false;
        awaiting_final_validation_            = false;
        localization_failure_reported_        = false;
        current_goal_handle_.reset();
    }

    dispatch_nav_goal(generation, false);
}

void AppBridgeNode::dispatch_nav_goal(std::uint64_t generation, bool is_retry)
{
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
        if (is_retry) {
            broadcast_localization_failed(generation);
        } else {
            bool is_current = false;
            {
                std::lock_guard<std::mutex> lock(nav_state_mutex_);
                is_current = generation == nav_generation_;
            }
            if (is_current) {
                broadcast({
                    {   "type",         "nav_result" },
                    { "status", "server_unavailable" }
                });
            }
        }
        return;
    }

    double x;
    double y;
    {
        std::lock_guard<std::mutex> lock(nav_state_mutex_);
        if (generation != nav_generation_) return;
        x = target_x_;
        y = target_y_;
    }

    // 도착 방향은 강제하지 않음 (yaw_goal_tolerance = π).
    // orientation은 identity를 전달.
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id    = "map";
    goal.pose.header.stamp       = this->now();
    goal.pose.pose.position.x    = x;
    goal.pose.pose.position.y    = y;
    goal.pose.pose.orientation.w = 1.0;

    auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_opts.goal_response_callback =
        [this, generation, is_retry](const auto &goal_handle) {
            on_goal_response(generation, is_retry, goal_handle);
        };
    send_opts.result_callback =
        [this, generation](const auto &result) { on_goal_result(generation, result); };

    {
        std::lock_guard<std::mutex> lock(nav_state_mutex_);
        if (generation != nav_generation_) return;
        goal_request_pending_ = true;
        nav_client_->async_send_goal(goal, send_opts);
    }
    RCLCPP_INFO(
        this->get_logger(), "%sNav goal sent: (%.2f, %.2f)",
        is_retry ? "Retry " : "", x, y
    );
}

void AppBridgeNode::on_goal_response(
    std::uint64_t generation,
    bool is_retry,
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle
)
{
    double x = 0.0;
    double y = 0.0;
    bool stale_goal = false;
    {
        std::lock_guard<std::mutex> lock(nav_state_mutex_);
        if (generation != nav_generation_) {
            stale_goal = true;
        } else {
            goal_request_pending_ = false;
            if (goal_handle) {
                current_goal_handle_ = goal_handle;
                navigation_active_   = true;
                x                     = target_x_;
                y                     = target_y_;
            } else {
                navigation_active_ = false;
            }
        }
    }

    if (stale_goal) {
        if (goal_handle) {
            nav_client_->async_cancel_goal(goal_handle);
        }
        return;
    }

    if (!goal_handle) {
        RCLCPP_WARN(this->get_logger(), "Goal rejected by Nav2");
        if (is_retry) {
            broadcast_localization_failed(generation);
        } else {
            broadcast({
                {   "type", "nav_result" },
                { "status",   "rejected" }
            });
        }
        return;
    }

    if (!is_retry) {
        broadcast({
            { "type", "nav_accepted" },
            {    "x",              x },
            {    "y",              y }
        });
    }
}

void AppBridgeNode::on_goal_result(
    std::uint64_t generation,
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result
)
{
    {
        std::lock_guard<std::mutex> lock(nav_state_mutex_);
        if (generation != nav_generation_) return;

        goal_request_pending_ = false;
        navigation_active_ = false;
        current_goal_handle_.reset();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            validation_started_sec_      = this->get_clock()->now().seconds();
            validation_deadline_sec_     = validation_started_sec_ + final_validation_timeout_sec_;
            awaiting_final_validation_   = true;
        }
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(
            this->get_logger(),
            "Nav2 goal reached — waiting %.2fs for a fresh L-code pose",
            final_validation_timeout_sec_
        );
        return;
    }

    std::string status = "unknown";
    if (result.code == rclcpp_action::ResultCode::ABORTED) status = "aborted";
    else if (result.code == rclcpp_action::ResultCode::CANCELED) status = "canceled";

    RCLCPP_INFO(this->get_logger(), "Navigation result: %s", status.c_str());
    broadcast({
        {   "type", "nav_result" },
        { "status",       status }
    });
}

void AppBridgeNode::on_lcode_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (msg->header.frame_id != "map") return;

    const double pose_stamp = rclcpp::Time(msg->header.stamp).seconds();
    std::uint64_t generation;
    double target_x;
    double target_y;
    int retries_completed;
    GoalValidation::Decision decision;
    double error_m;

    {
        std::lock_guard<std::mutex> lock(nav_state_mutex_);
        if (!awaiting_final_validation_ || pose_stamp < validation_started_sec_) return;

        generation        = nav_generation_;
        target_x          = target_x_;
        target_y          = target_y_;
        retries_completed = final_validation_retries_completed_;
        error_m            = GoalValidation::distance(
            target_x, target_y, msg->pose.position.x, msg->pose.position.y
        );
        decision = GoalValidation::evaluate(
            target_x, target_y, msg->pose.position.x, msg->pose.position.y,
            final_validation_tolerance_m_, retries_completed, final_validation_max_retries_
        );

        awaiting_final_validation_ = false;
        if (decision == GoalValidation::Decision::RETRY) {
            ++final_validation_retries_completed_;
        }
    }

    if (decision == GoalValidation::Decision::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Final L-code validation succeeded (error %.3fm)", error_m);
        broadcast({
            {   "type", "nav_result" },
            { "status",  "succeeded" }
        });
    } else if (decision == GoalValidation::Decision::RETRY) {
        RCLCPP_WARN(
            this->get_logger(),
            "Final position error %.3fm — retrying goal (%d/%d)",
            error_m, retries_completed + 1, final_validation_max_retries_
        );
        dispatch_nav_goal(generation, true);
    } else {
        RCLCPP_ERROR(
            this->get_logger(),
            "Final position error %.3fm after %d retries",
            error_m, retries_completed
        );
        broadcast_localization_failed(generation);
    }
}

void AppBridgeNode::on_localization_abort(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
    bool should_report = false;
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_to_cancel;
    {
        std::lock_guard<std::mutex> lock(nav_state_mutex_);
        if ((goal_request_pending_ || navigation_active_ || awaiting_final_validation_)
            && !localization_failure_reported_) {
            goal_to_cancel                    = current_goal_handle_;
            localization_failure_reported_    = true;
            goal_request_pending_              = false;
            navigation_active_                = false;
            awaiting_final_validation_         = false;
            current_goal_handle_.reset();
            ++nav_generation_; // 내부 cancel 결과와 이전 action 콜백 무효화
            should_report = true;
        }
    }
    if (!should_report) return;

    if (goal_to_cancel) {
        nav_client_->async_cancel_goal(goal_to_cancel);
    }
    RCLCPP_ERROR(this->get_logger(), "L-code localization failed — current Nav2 goal cancelled");
    broadcast({
        {   "type",           "nav_result" },
        { "status", "localization_failed" }
    });
}

void AppBridgeNode::check_validation_timeout()
{
    std::uint64_t generation = 0;
    bool timed_out = false;
    {
        std::lock_guard<std::mutex> lock(nav_state_mutex_);
        const double now = this->get_clock()->now().seconds();
        if (awaiting_final_validation_ && now >= validation_deadline_sec_) {
            generation                    = nav_generation_;
            awaiting_final_validation_    = false;
            timed_out                     = true;
        }
    }
    if (!timed_out) return;

    RCLCPP_ERROR(
        this->get_logger(), "No fresh L-code pose received within %.2fs", final_validation_timeout_sec_
    );
    broadcast_localization_failed(generation);
}

void AppBridgeNode::broadcast_localization_failed(std::uint64_t generation)
{
    {
        std::lock_guard<std::mutex> lock(nav_state_mutex_);
        if (generation != nav_generation_ || localization_failure_reported_) return;
        localization_failure_reported_ = true;
        goal_request_pending_          = false;
        navigation_active_             = false;
        awaiting_final_validation_      = false;
        current_goal_handle_.reset();
    }
    broadcast({
        {   "type",           "nav_result" },
        { "status", "localization_failed" }
    });
}

// ── 로봇 위치 전송 ────────────────────────────────────

void AppBridgeNode::publish_robot_pose()
{
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        if (clients_.empty()) return;
    }

    try {
        auto transform = tf_buffer_->lookupTransform(
            "map", "base_link", tf2::TimePointZero
        );

        auto &t = transform.transform.translation;
        auto &r = transform.transform.rotation;

        double yaw = std::atan2(
            2.0 * (r.w * r.z + r.x * r.y),
            1.0 - 2.0 * (r.y * r.y + r.z * r.z)
        );

        broadcast({
            {  "type", "robot_pose" },
            {     "x",          t.x },
            {     "y",          t.y },
            { "theta",          yaw }
        });
    } catch (const tf2::TransformException &) {
        // TF 아직 준비되지 않음
    }
}

} // namespace app_bridge
