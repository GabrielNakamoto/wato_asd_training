#include "control_node.hpp"
#include <chrono>

ControlNode::ControlNode(): Node("control") {
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ robot_odom_ = *msg; });
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](nav_msgs::msg::Path::SharedPtr msg){ current_path_ = *msg; });
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::interpolatePath() {
  const auto &robot_pos = robot_odom_.pose.pose.position;
  const auto &goal = current_path_.poses.back().pose.position;
  if (std::hypot(goal.x - robot_pos.x, goal.y - robot_pos.y) < ControlNode::goal_tolerance) {
    // stop command
    RCLCPP_INFO(this->get_logger(), "[PURE-PURSUIT] robot within target threshold. stopping");
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return std::nullopt;
  }

  auto last_pose = current_path_.poses[0];
  for (const auto &pose : current_path_.poses) {
    double dist = std::hypot(
      pose.pose.position.x - robot_odom_.pose.pose.position.x,
      pose.pose.position.y - robot_odom_.pose.pose.position.y
    );

    if (dist > ControlNode::lookahead_distance) return last_pose;
    last_pose = pose;
    last_pose.header.stamp = this->get_clock()->now();
  }

  // we are close to the destination
  return last_pose;
}

void ControlNode::controlLoop() {
  if (current_path_.poses.empty()) return;

  auto lookahead = interpolatePath();
  if (! lookahead.has_value()) return;

  // extract robot yaw from quaternion
  const auto robot_pose = robot_odom_.pose.pose;
  const auto q = robot_pose.orientation;
  const double robot_yaw = std::atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z));

  // find chord and radius of curve from current position to target
  // https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
  const auto final_point = lookahead.value().pose.position;
  const double dx = final_point.x - robot_pose.position.x;
  const double dy = final_point.y - robot_pose.position.y;
  const double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
  const double rel_x = final_point.x - robot_pose.position.x;
  const double curvature = 2.0* (local_y / (dx*dx + dy*dy));
  const double omega = curvature * ControlNode::linear_speed;

  // compute velocity
  auto vel_cmd = geometry_msgs::msg::Twist();
  vel_cmd.angular.z = omega;
  vel_cmd.linear.x = ControlNode::linear_speed;

  // publish command
  cmd_vel_pub_->publish(vel_cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
