#include "offboard_control_node.hpp" // 包含自己定义的头文件

using namespace std::chrono_literals; // 可以在 .cpp 文件中安全地使用

OffboardControlNode::OffboardControlNode() : Node("offboard_control_node") {
    current_state_ = mavros_msgs::msg::State();
    current_position_.fill(0.0);
    current_velocity_.fill(0.0);
    target_position_ = {0.0, 0.0, 0.0};
    setpoint_counter_ = 0;
    mod_ = 0;

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10, std::bind(&OffboardControlNode::state_cb, this, std::placeholders::_1));

    pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", rclcpp::SensorDataQoS(), std::bind(&OffboardControlNode::pos_cb, this, std::placeholders::_1));

    vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "mavros/local_position/velocity_local", rclcpp::SensorDataQoS(), std::bind(&OffboardControlNode::vel_cb, this, std::placeholders::_1));

    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    last_request_ = this->now();
    timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControlNode::control_loop, this));
    RCLCPP_INFO(this->get_logger(), "OffboardControlNode started.");
}

void OffboardControlNode::state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
}

void OffboardControlNode::pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_position_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
}

void OffboardControlNode::vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    current_velocity_ = {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};
}

void OffboardControlNode::publish_setpoint(double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.pose.position.x = target_position_[0];
    pose.pose.position.y = target_position_[1];
    pose.pose.position.z = target_position_[2];

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    local_pos_pub_->publish(pose);
}

void OffboardControlNode::publish_velocity(double vx, double vy, double vz) {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->now();
    twist.twist.linear.x = vx;
    twist.twist.linear.y = vy;
    twist.twist.linear.z = vz;
    vel_pub_->publish(twist);
}

bool OffboardControlNode::reached_target(double pos_tol) {
    for (int i = 0; i < 3; ++i) {
        if (std::abs(current_position_[i] - target_position_[i]) > pos_tol ||
            std::abs(current_velocity_[i]) > 0.09)
            return false;
    }
    return true;
}

void OffboardControlNode::control_loop() {
    auto now = this->now();

    if (!current_state_.connected) {
        RCLCPP_WARN(this->get_logger(), "等待飞控连接...");
        return;
    }

    if (setpoint_counter_ < 100) {
        publish_setpoint();
        setpoint_counter_++;
        return;
    }

    if (!current_state_.armed &&
        (now - last_request_).seconds() > 5.0 && current_state_.mode == "OFFBOARD") {
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = true;
        arming_client_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "请求解锁...");
        last_request_ = now;
        return;
    }

    if (!current_state_.armed) return;

    switch (mod_) {
        case 0:
            target_position_ = {0.0, 0.0, 1.5};
            publish_setpoint();
            if (reached_target()) {
                RCLCPP_INFO(this->get_logger(), "已起飞至 %.2f 米", target_position_[2]);
                mod_ = 1;
            }
            break;
        case 1:
            target_position_ = {0.3, 0.0, 1.5};
            publish_setpoint();
            if (reached_target()) {
                RCLCPP_INFO(this->get_logger(), "mod = 2");
                mod_ = 2;
            }
            break;
        case 2:
            target_position_ = {0.3, 0.3, 1.5};
            publish_setpoint();
            if (reached_target()) {
                RCLCPP_INFO(this->get_logger(), "mod = 3");
                mod_ = 3;
            }
            break;
        case 3:
            publish_velocity(0.0, 0.0, -0.3);
            if (current_position_[2] < 0.5 && std::abs(current_velocity_[2]) < 0.05) {
                RCLCPP_INFO(this->get_logger(), "即将降落...");
                mod_ = 4;
            }
            break;
        case 4: {
            auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            req->custom_mode = "AUTO.LAND";
            mode_client_->async_send_request(req);
            RCLCPP_INFO(this->get_logger(), "已请求 AUTO.LAND");
            mod_ = 5;
            break;
        }
        default:
            break;
    }

    if (mod_ < 3)
        publish_setpoint();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}