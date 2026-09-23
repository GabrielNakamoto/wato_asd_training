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
    RCLCPP_INFO(this->get_logger(), "[PURE-PURSUIT] robot within target threshold. stopping");
    // stop command
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return std::nullopt;
  }

  double prox = std::numeric_limits<double>::infinity();
  int start_idx = 0;
  for (int i=0; i<(int)current_path_.poses.size(); ++i) {
    double d = std::hypot(
      current_path_.poses[i].pose.position.x - robot_odom_.pose.pose.position.x,
      current_path_.poses[i].pose.position.y - robot_odom_.pose.pose.position.y
    );

    if (d < prox) {
      prox = d;
      start_idx = i;
    }
  }

  auto last_pose = current_path_.poses[0];
  for (int i=start_idx; i<(int)current_path_.poses.size(); ++i) {
    auto pose = &current_path_.poses[i];
    double d = std::hypot(
      pose->pose.position.x - robot_odom_.pose.pose.position.x,
      pose->pose.position.y - robot_odom_.pose.pose.position.y
    );

    if (d > ControlNode::lookahead_distance ) return last_pose;
    last_pose = *pose;
    last_pose.header.stamp = this->get_clock()->now();
  }

  // we are close to the destination
  return last_pose;
}

void ControlNode::controlLoop() {
  if (current_path_.poses.empty()) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

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
  const double curvature = 2.0* (local_y / (dx*dx + dy*dy));
  const double omega = curvature * ControlNode::linear_speed;

  // compute velocity
  auto vel_cmd = geometry_msgs::msg::Twist();
  vel_cmd.angular.z = omega;
  vel_cmd.linear.x = ControlNode::linear_speed;

  /*
  // if more than 60 degrees off stop and rotate
  const double local_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double heading_error = std::atan2(local_y, local_x);
  if (std::abs(heading_error) > M_PI / 3) {
    vel_cmd.linear.x = 0.0;
    vel_cmd.angular.z = std::copysign(1.0, heading_error);
  }
  */

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
