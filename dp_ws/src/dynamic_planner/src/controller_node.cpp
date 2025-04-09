#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <limits>
#include <vector> 


class PathFollower : public rclcpp::Node
{
public:
  PathFollower()
  : Node("controller_node"),
    current_waypoint_index_(0),
    robot_x_(0.0),
    robot_y_(0.0),
    robot_yaw_(0.0),
    min_front_dist_(std::numeric_limits<double>::infinity()),
    obstacle_detected_(false) 
  {

    this->declare_parameter<double>("goal_tolerance", 0.1);
    this->declare_parameter<double>("obstacle_stop_distance", 0.3);
    this->declare_parameter<double>("angular_gain", 1.0);
    this->declare_parameter<double>("linear_speed", 0.2);
    this->declare_parameter<double>("rotation_threshold", 0.1);
    this->declare_parameter<double>("scan_cone_angle", M_PI / 6.0);

    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    obstacle_stop_distance_ = this->get_parameter("obstacle_stop_distance").as_double();
    angular_gain_ = this->get_parameter("angular_gain").as_double();
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    rotation_threshold_ = this->get_parameter("rotation_threshold").as_double();
    scan_cone_half_angle_ = this->get_parameter("scan_cone_angle").as_double();


    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PathFollower::odomCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&PathFollower::scanCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    replan_pub_ = this->create_publisher<std_msgs::msg::Empty>("/request_replan", 10);

    RCLCPP_INFO(this->get_logger(), "Controller node started.");

    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PathFollower::controlLoop, this));
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    current_path_ = *msg;
    current_waypoint_index_ = 0;
    obstacle_detected_ = false;
    RCLCPP_INFO(this->get_logger(), "Received a new path with %zu poses", current_path_.poses.size());
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    robot_yaw_ = yaw;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    min_front_dist_ = std::numeric_limits<double>::infinity();
    double current_angle = msg->angle_min;

    for (const auto& range : msg->ranges) {
        if (current_angle >= -scan_cone_half_angle_ && current_angle <= scan_cone_half_angle_) {
            if (!std::isinf(range) && !std::isnan(range) && range >= msg->range_min && range <= msg->range_max) {
                if (range < min_front_dist_) {
                    min_front_dist_ = range;
                }
            }
        }
        current_angle += msg->angle_increment;
    }
     // RCLCPP_INFO(this->get_logger(), "Min front distance: %.2f", min_front_dist_);
  }


  void controlLoop()
  {
    if (min_front_dist_ < obstacle_stop_distance_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Obstacle detected at %.2f m! Stopping and requesting replan.", min_front_dist_);
        geometry_msgs::msg::Twist stop_cmd; 
        cmd_pub_->publish(stop_cmd);
        if (!obstacle_detected_) {
            std_msgs::msg::Empty replan_msg;
            replan_pub_->publish(replan_msg);
            current_path_.poses.clear();
            current_waypoint_index_ = 0;
            obstacle_detected_ = true;
        }
        return;
    } else {
        if (obstacle_detected_) {
             RCLCPP_INFO(this->get_logger(), "Obstacle cleared.");
        }
        obstacle_detected_ = false;
    }

    if (current_path_.poses.empty() || current_waypoint_index_ >= current_path_.poses.size()) {
      if (current_path_.poses.empty() && !obstacle_detected_) {
         //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No path to follow or path completed.");
      }
      geometry_msgs::msg::Twist stop_cmd;
      cmd_pub_->publish(stop_cmd);
      return;
    }

    const auto& target_pose = current_path_.poses[current_waypoint_index_].pose;
    double goal_x = target_pose.position.x;
    double goal_y = target_pose.position.y;

    double dx = goal_x - robot_x_;
    double dy = goal_y - robot_y_;
    double dist_to_waypoint = std::sqrt(dx*dx + dy*dy);

    if (dist_to_waypoint < goal_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", current_waypoint_index_);
      current_waypoint_index_++;
      if (current_waypoint_index_ >= current_path_.poses.size()) {
          RCLCPP_INFO(this->get_logger(), "Path completed!");
          geometry_msgs::msg::Twist stop_cmd;
          cmd_pub_->publish(stop_cmd);
      }
      return;
    }

    double desired_heading = std::atan2(dy, dx);
    double yaw_error = desired_heading - robot_yaw_;
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = angular_gain_ * yaw_error;
    cmd.angular.z = std::clamp(cmd.angular.z, -1.0, 1.0);
    if (std::fabs(yaw_error) < rotation_threshold_) {
      cmd.linear.x = linear_speed_;
      cmd.linear.x *= (1.0 - std::fabs(yaw_error) / rotation_threshold_);
    } else {
      cmd.linear.x = 0.0;
    }
    cmd.linear.x = std::clamp(cmd.linear.x, 0.0, linear_speed_);

    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Path current_path_;
  size_t current_waypoint_index_;
  double robot_x_;
  double robot_y_;
  double robot_yaw_;
  double min_front_dist_;
  bool obstacle_detected_;
  double goal_tolerance_;
  double obstacle_stop_distance_;
  double angular_gain_;
  double linear_speed_;
  double rotation_threshold_;
  double scan_cone_half_angle_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}