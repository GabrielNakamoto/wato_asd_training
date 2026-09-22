#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "map_memory_core.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void processCostmap(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap);
    void processOdometry(const std::shared_ptr<nav_msgs::msg::Odometry> odometry);
    void transmitMap();

    static constexpr double distance_threshold = 1.5f;
    static constexpr double map_resolution = 0.1;
    static const size_t map_size = 35; // meters

    // state
    double latest_x = 0.0f, latest_y = 0.0f, latest_yaw = 0.0f;
    double last_x = 0.0f, last_y = 0.0f;
    bool should_update_map_ = false;
    bool costmap_updated_ = false;
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    nav_msgs::msg::Odometry latest_odom_;

    // connections
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
