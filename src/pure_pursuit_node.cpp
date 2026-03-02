#include <pure_pursuit_tracker/pure_pursuit_node.hpp>
#include <algorithm>
#include <limits>

PurePursuitNode::PurePursuitNode()
    : Node("pure_pursuit_node"),
      odom_received_(false),
      path_received_(false),
      last_closest_idx_(0),
      prev_w_command_(0.0)
{
    // Build odom frame ID from namespace (e.g. "/RR03" -> "RR03/odom")
    {
        std::string ns = this->get_namespace();
        if (!ns.empty() && ns.front() == '/') ns = ns.substr(1);
        odom_frame_id_ = ns.empty() ? "odom" : ns + "/odom";
    }

    // Declare parameters with defaults
    this->declare_parameter("lookahead_distance", 0.8);
    this->declare_parameter("k_v", 0.5);
    this->declare_parameter("max_linear_velocity", 0.5);
    this->declare_parameter("max_angular_velocity", 1.5);
    this->declare_parameter("goal_tolerance", 0.3);
    this->declare_parameter("control_frequency", 20.0);
    this->declare_parameter("adaptive_lookahead_distance", 2.0);
    this->declare_parameter("turn_in_place_threshold_deg", 60.0);
    this->declare_parameter("slow_down_threshold_deg", 30.0);
    this->declare_parameter("w_smoothing_alpha", 0.3);
    this->declare_parameter("max_lateral_acceleration", 0.5);
    this->declare_parameter("enable_curvature_slowdown", true);

    // Get parameters
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    k_v_ = this->get_parameter("k_v").as_double();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    adaptive_lookahead_distance_ = this->get_parameter("adaptive_lookahead_distance").as_double();
    turn_in_place_threshold_ = this->get_parameter("turn_in_place_threshold_deg").as_double() * M_PI / 180.0;
    slow_down_threshold_ = this->get_parameter("slow_down_threshold_deg").as_double() * M_PI / 180.0;
    w_smoothing_alpha_ = this->get_parameter("w_smoothing_alpha").as_double();
    max_lateral_acceleration_ = this->get_parameter("max_lateral_acceleration").as_double();
    enable_curvature_slowdown_ = this->get_parameter("enable_curvature_slowdown").as_bool();

    // Subscribe to /reference_trajectory with QoS: reliable + transient_local (to match publisher)
    rclcpp::QoS path_qos(10);
    path_qos.reliable();
    path_qos.transient_local();

    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "reference_trajectory", path_qos,
        std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

    // Subscribe to odom with default QoS
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));

    // Publishers with default QoS
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_actual_trajectory_ = this->create_publisher<nav_msgs::msg::Path>("actual_trajectory", 10);
    pub_lookahead_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 10);
    pub_lookahead_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_marker", 10);

    // Create wall timer at control_frequency Hz
    int period_ms = static_cast<int>(1000.0 / control_frequency_);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&PurePursuitNode::controlCallback, this));

    // Initialize actual trajectory header
    actual_trajectory_.header.frame_id = odom_frame_id_;

    // Open CSV log file
    log_file_.open("/tmp/pure_pursuit_log.csv", std::ios::out | std::ios::trunc);
    if (log_file_.is_open())
    {
        log_file_ << "time,closest_idx,ref_x,ref_y,robot_x,robot_y,robot_yaw,"
                  << "lookahead_x,lookahead_y,alpha_deg,curvature,v_curv_limit,"
                  << "v_cmd,w_cmd,cross_track_error,w_ff,w_pursuit,w_heading\n";
        RCLCPP_INFO(this->get_logger(), "Logging to /tmp/pure_pursuit_log.csv");
    }

    RCLCPP_INFO(this->get_logger(), "Pure pursuit node initialized");
    RCLCPP_INFO(this->get_logger(), "  lookahead_distance: %.2f", lookahead_distance_);
    RCLCPP_INFO(this->get_logger(), "  k_v: %.2f", k_v_);
    RCLCPP_INFO(this->get_logger(), "  max_linear_velocity: %.2f", max_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "  max_angular_velocity: %.2f", max_angular_velocity_);
    RCLCPP_INFO(this->get_logger(), "  goal_tolerance: %.2f", goal_tolerance_);
    RCLCPP_INFO(this->get_logger(), "  control_frequency: %.1f Hz", control_frequency_);
}

PurePursuitNode::~PurePursuitNode()
{
    if (log_file_.is_open())
    {
        log_file_.flush();
        log_file_.close();
        RCLCPP_INFO(this->get_logger(), "Closed /tmp/pure_pursuit_log.csv");
    }
}

void PurePursuitNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    // Only reset progress when receiving a genuinely new trajectory (different size),
    // not when the same trajectory is re-published periodically.
    bool new_trajectory = !path_received_ || msg->poses.size() != reference_path_.poses.size();

    reference_path_ = *msg;
    path_received_ = true;

    if (new_trajectory)
    {
        last_closest_idx_ = 0;
        RCLCPP_INFO(this->get_logger(), "Received new reference trajectory with %zu waypoints",
                    reference_path_.poses.size());
    }
}

void PurePursuitNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom_ = *msg;
    odom_received_ = true;
}

double PurePursuitNode::getYawFromQuaternion(double qx, double qy, double qz, double qw)
{
    return std::atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz));
}

double PurePursuitNode::wrapPi(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

size_t PurePursuitNode::findClosestWaypointIndex()
{
    if (reference_path_.poses.empty()) return 0;

    // Forward-only search from the last known closest index.
    // Strictly monotonic — never goes backwards. This prevents jumping to
    // the opposite side of self-intersecting paths (e.g. the figure-8 lobe
    // where waypoint 49 is spatially close to waypoints on the return leg).
    const size_t n = reference_path_.poses.size();
    const size_t look_forward = 50;

    size_t start = last_closest_idx_;
    size_t end = std::min(last_closest_idx_ + look_forward, n);

    double rx = current_odom_.pose.pose.position.x;
    double ry = current_odom_.pose.pose.position.y;

    // Scan for the first zero-distance transition in the search window.
    // This marks a turn-in-place cluster; cap the search there so the
    // closest-point can never jump to the spatially-overlapping return leg.
    size_t turn_boundary = end;
    for (size_t i = start; i + 1 < end; ++i)
    {
        double dx = reference_path_.poses[i + 1].pose.position.x -
                    reference_path_.poses[i].pose.position.x;
        double dy = reference_path_.poses[i + 1].pose.position.y -
                    reference_path_.poses[i].pose.position.y;
        if (dx * dx + dy * dy < 1e-6)
        {
            turn_boundary = i + 1;
            break;
        }
    }

    size_t closest_idx = last_closest_idx_;
    double min_dist_sq = std::numeric_limits<double>::max();

    for (size_t i = start; i < turn_boundary; ++i)
    {
        double dx = reference_path_.poses[i].pose.position.x - rx;
        double dy = reference_path_.poses[i].pose.position.y - ry;
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_dist_sq)
        {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }

    // Cross the turn cluster ONLY when the robot is within 0.3 m
    // of the turn-cluster position (the line endpoint + run-out).
    if (turn_boundary < end)
    {
        double tcx = reference_path_.poses[turn_boundary].pose.position.x;
        double tcy = reference_path_.poses[turn_boundary].pose.position.y;
        double dtx = tcx - rx;
        double dty = tcy - ry;
        const double threshold_sq = 0.3 * 0.3;
        if (dtx * dtx + dty * dty < threshold_sq)
        {
            closest_idx = turn_boundary;
            while (closest_idx < n - 1)
            {
                double ndx = reference_path_.poses[closest_idx + 1].pose.position.x -
                             reference_path_.poses[closest_idx].pose.position.x;
                double ndy = reference_path_.poses[closest_idx + 1].pose.position.y -
                             reference_path_.poses[closest_idx].pose.position.y;
                if (ndx * ndx + ndy * ndy < 1e-6)
                    closest_idx++;
                else
                    break;
            }
        }
    }

    last_closest_idx_ = closest_idx;
    return closest_idx;
}

size_t PurePursuitNode::findLookaheadWaypointIndex(size_t start_idx, double lookahead_dist)
{
    if (reference_path_.poses.empty()) return 0;

    size_t lookahead_idx = start_idx;
    double accumulated_dist = 0.0;

    for (size_t i = start_idx; i < reference_path_.poses.size() - 1; ++i)
    {
        // Stop at heading discontinuities (turn-in-place segments).
        // This keeps the goal point pinned at the segment endpoint
        // until the robot actually arrives, instead of letting the
        // lookahead see through the zero-distance turn cluster into
        // the return leg.
        double yaw_i = getYawFromQuaternion(
            reference_path_.poses[i].pose.orientation.x,
            reference_path_.poses[i].pose.orientation.y,
            reference_path_.poses[i].pose.orientation.z,
            reference_path_.poses[i].pose.orientation.w);
        double yaw_next = getYawFromQuaternion(
            reference_path_.poses[i + 1].pose.orientation.x,
            reference_path_.poses[i + 1].pose.orientation.y,
            reference_path_.poses[i + 1].pose.orientation.z,
            reference_path_.poses[i + 1].pose.orientation.w);
        if (std::abs(wrapPi(yaw_next - yaw_i)) > M_PI / 2.0)
        {
            lookahead_idx = i;
            break;
        }

        double dx = reference_path_.poses[i + 1].pose.position.x -
                    reference_path_.poses[i].pose.position.x;
        double dy = reference_path_.poses[i + 1].pose.position.y -
                    reference_path_.poses[i].pose.position.y;
        accumulated_dist += std::sqrt(dx * dx + dy * dy);

        if (accumulated_dist >= lookahead_dist)
        {
            lookahead_idx = i + 1;
            break;
        }
        lookahead_idx = i + 1;
    }

    return lookahead_idx;
}

double PurePursuitNode::computeDynamicLookahead(double current_speed)
{
    return lookahead_distance_ + k_v_ * current_speed;
}

double PurePursuitNode::computePathCurvature(size_t idx)
{
    // Compute curvature from three consecutive waypoints using the Menger curvature formula:
    // kappa = 2 * |cross(P1-P0, P2-P1)| / (|P1-P0| * |P2-P1| * |P2-P0|)
    if (idx == 0 || idx >= reference_path_.poses.size() - 1) return 0.0;

    double x0 = reference_path_.poses[idx - 1].pose.position.x;
    double y0 = reference_path_.poses[idx - 1].pose.position.y;
    double x1 = reference_path_.poses[idx].pose.position.x;
    double y1 = reference_path_.poses[idx].pose.position.y;
    double x2 = reference_path_.poses[idx + 1].pose.position.x;
    double y2 = reference_path_.poses[idx + 1].pose.position.y;

    double ax = x1 - x0, ay = y1 - y0;
    double bx = x2 - x1, by = y2 - y1;

    double cross = std::abs(ax * by - ay * bx);
    double a_len = std::sqrt(ax * ax + ay * ay);
    double b_len = std::sqrt(bx * bx + by * by);
    double cx = x2 - x0, cy = y2 - y0;
    double c_len = std::sqrt(cx * cx + cy * cy);

    double denom = a_len * b_len * c_len;
    if (denom < 1e-6) return 0.0;

    return 2.0 * cross / denom;
}

void PurePursuitNode::controlCallback()
{
    // Guard: need both odom and path data
    if (!odom_received_ || !path_received_ || reference_path_.poses.empty())
    {
        geometry_msgs::msg::Twist zero_twist;
        zero_twist.linear.x = 0.0;
        zero_twist.angular.z = 0.0;
        pub_cmd_vel_->publish(zero_twist);
        return;
    }

    // Get robot position and yaw
    double robot_x = current_odom_.pose.pose.position.x;
    double robot_y = current_odom_.pose.pose.position.y;
    double current_yaw = getYawFromQuaternion(
        current_odom_.pose.pose.orientation.x,
        current_odom_.pose.pose.orientation.y,
        current_odom_.pose.pose.orientation.z,
        current_odom_.pose.pose.orientation.w);

    // Find closest waypoint on trajectory
    size_t closest_idx = findClosestWaypointIndex();

    // Check if we've reached the end of the trajectory
    // Only check goal proximity once the robot has progressed past 90% of the path,
    // to avoid false "goal reached" on closed-loop trajectories where the last waypoint
    // is near the start.
    const auto& last_pose = reference_path_.poses.back().pose.position;
    double dx_to_goal = last_pose.x - robot_x;
    double dy_to_goal = last_pose.y - robot_y;
    double dist_to_goal = std::sqrt(dx_to_goal * dx_to_goal + dy_to_goal * dy_to_goal);

    // When the robot reaches the end of the path, wrap back to the start
    // for continuous looping (infinite laps on closed-loop trajectories).
    size_t goal_check_threshold = static_cast<size_t>(reference_path_.poses.size() * 0.9);
    if (closest_idx >= goal_check_threshold && dist_to_goal < goal_tolerance_)
    {
        last_closest_idx_ = 0;
        closest_idx = 0;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Lap complete, looping.");
    }

    // Compute dynamic lookahead distance: L = L_min + k_v * v (using actual speed)
    double current_speed = std::abs(current_odom_.twist.twist.linear.x);
    double lookahead_dist = computeDynamicLookahead(current_speed);

    // Adaptive lookahead: reduce lookahead distance when approaching goal for graceful stopping.
    // Only apply when past 80% of the path — otherwise on looping/self-intersecting trajectories
    // (figure-8, line) the robot repeatedly passes near the final waypoint and gets its
    // lookahead crushed every lap, causing oscillation or stalling.
    if (closest_idx >= reference_path_.poses.size() * 0.8 &&
        dist_to_goal < adaptive_lookahead_distance_)
    {
        double reduction_factor = dist_to_goal / adaptive_lookahead_distance_;
        lookahead_dist *= reduction_factor;
        lookahead_dist = std::max(lookahead_dist, 0.1);
    }

    // Find lookahead waypoint
    size_t lookahead_idx = findLookaheadWaypointIndex(closest_idx, lookahead_dist);
    const auto& lookahead_pos = reference_path_.poses[lookahead_idx].pose.position;

    // Publish lookahead point (PointStamped, frame_id="odom")
    geometry_msgs::msg::PointStamped lookahead_msg;
    lookahead_msg.header.stamp = this->now();
    lookahead_msg.header.frame_id = odom_frame_id_;
    lookahead_msg.point.x = lookahead_pos.x;
    lookahead_msg.point.y = lookahead_pos.y;
    lookahead_msg.point.z = lookahead_pos.z;
    pub_lookahead_point_->publish(lookahead_msg);

    // Publish lookahead marker for RViz visualization (SPHERE, yellow rgba(1,1,0,0.8))
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = odom_frame_id_;
    marker.ns = "lookahead";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = lookahead_pos.x;
    marker.pose.position.y = lookahead_pos.y;
    marker.pose.position.z = lookahead_pos.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    pub_lookahead_marker_->publish(marker);

    // ============================================================
    // CONTROL LAW (ported from mighty_ws pure_pursuit.cpp)
    // ============================================================

    // Compute heading error to lookahead point
    double dx = lookahead_pos.x - robot_x;
    double dy = lookahead_pos.y - robot_y;
    double heading_to_lookahead = std::atan2(dy, dx);
    double alpha = wrapPi(heading_to_lookahead - current_yaw);
    double abs_alpha = std::abs(alpha);

    // ---- Turn-in-place for large heading errors ----
    if (abs_alpha > turn_in_place_threshold_)
    {
        // Proportional turn rate based on heading error direction
        double w_turn = (alpha / abs_alpha) * max_angular_velocity_ * 0.9;

        // Smooth the angular command
        w_turn = w_smoothing_alpha_ * prev_w_command_ + (1.0 - w_smoothing_alpha_) * w_turn;
        prev_w_command_ = w_turn;

        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = w_turn;
        pub_cmd_vel_->publish(twist);

        // Log turn-in-place
        if (log_file_.is_open())
        {
            const auto& ref = reference_path_.poses[closest_idx].pose.position;
            double cte = std::sqrt((robot_x - ref.x) * (robot_x - ref.x) +
                                   (robot_y - ref.y) * (robot_y - ref.y));
            double t = this->now().seconds();
            log_file_ << t << "," << closest_idx << ","
                      << ref.x << "," << ref.y << ","
                      << robot_x << "," << robot_y << "," << current_yaw * 180.0 / M_PI << ","
                      << lookahead_pos.x << "," << lookahead_pos.y << ","
                      << alpha * 180.0 / M_PI << ",0,0,"
                      << 0.0 << "," << w_turn << "," << cte << "\n";
        }

        // Accumulate actual trajectory: append current robot pose, DO NOT CLEAR
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = odom_frame_id_;
            pose_stamped.pose = current_odom_.pose.pose;
            actual_trajectory_.poses.push_back(pose_stamped);
            actual_trajectory_.header.stamp = this->now();
            pub_actual_trajectory_->publish(actual_trajectory_);
        }
        return;
    }

    // ---- Curvature-based speed limit ----
    double v_curvature_limit = max_linear_velocity_;
    double max_upcoming_curvature = 0.0;
    if (enable_curvature_slowdown_)
    {
        // Scan ahead from the closest waypoint to find the tightest upcoming curve.
        // This lets the robot brake BEFORE entering a corner, not while inside it.
        const size_t curvature_window = 30;  // look ~30 waypoints ahead
        size_t curv_end = std::min(closest_idx + curvature_window, reference_path_.poses.size() - 1);
        for (size_t i = closest_idx; i < curv_end; ++i)
        {
            double k = computePathCurvature(i);
            if (k > max_upcoming_curvature) max_upcoming_curvature = k;
        }

        // Limit speed so the required angular velocity (v * kappa) stays well within
        // max_angular_velocity, leaving headroom for heading corrections.
        // Also apply lateral-acceleration limit for comfort.
        if (max_upcoming_curvature > 0.01)
        {
            double v_omega_limit = (max_angular_velocity_ * 0.7) / max_upcoming_curvature;
            double v_accel_limit = std::sqrt(max_lateral_acceleration_ / max_upcoming_curvature);
            v_curvature_limit = std::min({v_omega_limit, v_accel_limit, max_linear_velocity_});
            v_curvature_limit = std::max(v_curvature_limit, 0.05);
        }
    }

    // ---- Speed reduction based on heading error ----
    double speed_reduction = 1.0;
    if (abs_alpha > slow_down_threshold_)
    {
        // Linear ramp from 1.0 at slow_down_threshold down to 0.0 at turn_in_place_threshold
        speed_reduction = std::max(0.0, 1.0 - (abs_alpha - slow_down_threshold_) /
                                                (turn_in_place_threshold_ - slow_down_threshold_));
    }

    // Also slow down near the goal for smooth stopping.
    // Only apply when past 80% of the path — otherwise closed-loop trajectories
    // (where the last waypoint is near the start) get throttled from the beginning.
    double goal_speed_factor = 1.0;
    if (closest_idx >= reference_path_.poses.size() * 0.8 &&
        dist_to_goal < adaptive_lookahead_distance_)
    {
        goal_speed_factor = std::max(0.1, dist_to_goal / adaptive_lookahead_distance_);
    }

    // Apply all speed factors: curvature limit, heading error reduction, goal proximity
    double v_command = std::min(max_linear_velocity_, v_curvature_limit) * speed_reduction * goal_speed_factor;

    // Ensure minimum velocity when not turning in place (avoid stalling)
    if (v_command < 0.05 && abs_alpha < turn_in_place_threshold_)
    {
        v_command = 0.05;
    }

    // ---- Feedforward: angular velocity from path curvature ----
    // Compute signed curvature at closest waypoint so the robot starts turning
    // as soon as it reaches a curve, BEFORE heading error builds up.
    double w_feedforward = 0.0;
    if (closest_idx > 0 && closest_idx < reference_path_.poses.size() - 1)
    {
        // Signed curvature from cross product of consecutive tangent vectors
        double x0 = reference_path_.poses[closest_idx - 1].pose.position.x;
        double y0 = reference_path_.poses[closest_idx - 1].pose.position.y;
        double x1 = reference_path_.poses[closest_idx].pose.position.x;
        double y1 = reference_path_.poses[closest_idx].pose.position.y;
        double x2 = reference_path_.poses[closest_idx + 1].pose.position.x;
        double y2 = reference_path_.poses[closest_idx + 1].pose.position.y;

        double ax = x1 - x0, ay = y1 - y0;
        double bx = x2 - x1, by = y2 - y1;
        double cross_val = ax * by - ay * bx;  // positive = left turn
        double a_len = std::sqrt(ax * ax + ay * ay);
        double b_len = std::sqrt(bx * bx + by * by);
        double cx = x2 - x0, cy = y2 - y0;
        double c_len = std::sqrt(cx * cx + cy * cy);
        double denom = a_len * b_len * c_len;

        if (denom > 1e-6)
        {
            double signed_curvature = 2.0 * cross_val / denom;
            w_feedforward = v_command * signed_curvature;
        }
    }

    // ---- Feedback: pure pursuit curvature + heading correction ----
    double curvature = (2.0 * std::sin(alpha)) / std::max(lookahead_dist, 0.1);
    double w_pursuit = v_command * curvature;

    double k_yaw = 2.0;
    double w_heading = k_yaw * alpha;

    // Combine: feedforward (anticipate curve) + feedback (correct errors)
    double w_command = w_feedforward + w_pursuit + w_heading;

    // Clamp angular velocity
    w_command = std::clamp(w_command, -max_angular_velocity_, max_angular_velocity_);

    // ---- Smooth angular velocity to reduce jitter ----
    w_command = w_smoothing_alpha_ * prev_w_command_ + (1.0 - w_smoothing_alpha_) * w_command;
    prev_w_command_ = w_command;

    // Publish cmd_vel
    geometry_msgs::msg::Twist twist;
    twist.linear.x = v_command;
    twist.angular.z = w_command;
    pub_cmd_vel_->publish(twist);

    // Log to CSV
    if (log_file_.is_open())
    {
        const auto& ref = reference_path_.poses[closest_idx].pose.position;
        double cte = std::sqrt((robot_x - ref.x) * (robot_x - ref.x) +
                               (robot_y - ref.y) * (robot_y - ref.y));
        double t = this->now().seconds();
        log_file_ << t << "," << closest_idx << ","
                  << ref.x << "," << ref.y << ","
                  << robot_x << "," << robot_y << "," << current_yaw * 180.0 / M_PI << ","
                  << lookahead_pos.x << "," << lookahead_pos.y << ","
                  << alpha * 180.0 / M_PI << "," << max_upcoming_curvature << "," << v_curvature_limit << ","
                  << v_command << "," << w_command << "," << cte
                  << "," << w_feedforward << "," << w_pursuit << "," << w_heading << "\n";
    }

    // Accumulate actual trajectory: append current robot pose, DO NOT CLEAR
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = odom_frame_id_;
        pose_stamped.pose = current_odom_.pose.pose;
        actual_trajectory_.poses.push_back(pose_stamped);
        actual_trajectory_.header.stamp = this->now();
        pub_actual_trajectory_->publish(actual_trajectory_);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
