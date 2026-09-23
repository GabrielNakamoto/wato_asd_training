#include "control_node.hpp"
#include <chrono>

ControlNode::ControlNode(): Node("control") {
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr);
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, );
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), );
}

std::optional<geometry_msgs::msg::PointStamped> interpolatePath() {
  for (const auto &pose : current_path_.poses) {
    double dist = std::hypot(
      pose.position.x - robot_odom_.pose.position.x,
      pose.position.y - robot_odom_.pose.position.y
    );

    if (dist > ControlNode::lookahead_distance)
      return pose;
  }

  // we are close to the destination
  return last_pose;
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_) return;

  auto lookahead = interpolatePath();
  if (! lookahead.has_value()) return;

  // compute velocity
  auto final_pose = lookahead.value();
  auto robot_pose = robot_odom_.pose;
  double dyaw = std::atan2(
    final_pose.position.x - robot_pose.position.x,
    final_pose.position.y - robot_pose.position.y
  );

  auto vel_cmd = geometry_msgs::msg::Twist();
  vel_cmd.linear.x = std::cos(dyaw) * ControLNode::linear_speed;
  vel_cmd.linear.y = std::sin(dyaw) * ControLNode::linear_speed;

  // how to determine curve connecting current pose to lookahead?
  // whats the omega factor?

  // vel_cmd.angular.

  // publish velocity command
  // cmd_vel_pub_->publish(vel_cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
