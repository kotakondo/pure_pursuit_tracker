#ifndef PURE_PURSUIT_NODE_HPP
#define PURE_PURSUIT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>
#include <fstream>

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode();
    ~PurePursuitNode();

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlCallback();

    size_t findClosestWaypointIndex();
    size_t findLookaheadWaypointIndex(size_t start_idx, double lookahead_dist);
    double computeDynamicLookahead(double current_speed);
    double computePathCurvature(size_t idx);
    double wrapPi(double angle);
    double getYawFromQuaternion(double qx, double qy, double qz, double qw);

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_actual_trajectory_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_lookahead_point_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_lookahead_marker_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    nav_msgs::msg::Path reference_path_;
    nav_msgs::msg::Path actual_trajectory_;  // ACCUMULATES over time
    nav_msgs::msg::Odometry current_odom_;
    bool odom_received_;
    bool path_received_;
    size_t last_closest_idx_;            // monotonic progress along path

    // Parameters
    double lookahead_distance_;           // L_min
    double k_v_;                          // velocity-dependent lookahead factor
    double max_linear_velocity_;
    double max_angular_velocity_;
    double goal_tolerance_;               // stopping_radius
    double control_frequency_;
    double adaptive_lookahead_distance_;
    double turn_in_place_threshold_;      // stored in radians
    double slow_down_threshold_;          // stored in radians
    double w_smoothing_alpha_;
    double max_lateral_acceleration_;
    double prev_w_command_;

    // CSV logging
    std::ofstream log_file_;
};

#endif // PURE_PURSUIT_NODE_HPP
