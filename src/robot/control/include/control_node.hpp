#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "control_core.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    void controlLoop();
    std::optional<geometry_msgs::msg::PoseStamped> interpolatePath();

    nav_msgs::msg::Path current_path_;
    nav_msgs::msg::Odometry robot_odom_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    static constexpr double lookahead_distance = 1.0f;
    static constexpr double goal_tolerance = 0.25;
    static constexpr double linear_speed = 0.9f;
};

#endif
