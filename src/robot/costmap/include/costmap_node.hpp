#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <costmap_core.hpp>

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

  private:
    void processLidar(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // cells per meter
    const static size_t grid_resolution = 10;
    // number of meters per side
    const static size_t grid_size = 10;
    // meter radius of heightened cost/reaction
    const static size_t inflation_radius = 1.75f;
    const static size_t inflation_max_cost = 50;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
};

#endif
