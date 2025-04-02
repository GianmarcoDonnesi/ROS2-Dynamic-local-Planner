#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <queue>

class PathFollower : public rclcpp::Node
{
public:
  PathFollower()
  : Node("controller_node"),
    current_waypoint_index_(0),
    robot_x_(0.0),
    robot_y_(0.0),
    robot_yaw_(0.0),
    min_front_dist_(std::numeric_limits<double>::infinity())
  {
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PathFollower::odomCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&PathFollower::scanCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

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
    RCLCPP_INFO(this->get_logger(), "Received a path with %zu poses", current_path_.poses.size());
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
    double min_dist = std::numeric_limits<double>::infinity();
    size_t n = msg->ranges.size();
    size_t start_idx = static_cast<size_t>( (msg->angle_min + M_PI/6) / msg->angle_increment );
    size_t end_idx   = static_cast<size_t>( (msg->angle_max - M_PI/6) / msg->angle_increment );
    start_idx = std::min(start_idx, n-1);
    end_idx = std::min(end_idx, n-1);
    for (size_t i = start_idx; i <= end_idx; i++) {
      double r = msg->ranges[i];
      if (!std::isinf(r) && r < min_dist) {
        min_dist = r;
      }
    }
    min_front_dist_ = min_dist;
  }

  void controlLoop()
  {
    if (current_path_.poses.empty() || (current_waypoint_index_ >= current_path_.poses.size())) {
      geometry_msgs::msg::Twist stop;
      cmd_pub_->publish(stop);
      return;
    }

    if (min_front_dist_ < 0.3) {
      RCLCPP_WARN(this->get_logger(), "Obstacle too close, Emergency stop.");
      geometry_msgs::msg::Twist stop;
      cmd_pub_->publish(stop);
      return;
    }

    auto &pose = current_path_.poses[current_waypoint_index_].pose;
    double goal_x = pose.position.x;
    double goal_y = pose.position.y;

    double dx = goal_x - robot_x_;
    double dy = goal_y - robot_y_;
    double dist = std::sqrt(dx*dx + dy*dy);

    if (dist < 0.1) {
      current_waypoint_index_++;
      return;
    }

    double heading = std::atan2(dy, dx);
    double yaw_error = heading - robot_yaw_;
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

    double Kp = 1.0;
    double angular_speed = Kp * yaw_error;
    double linear_speed = 0.2;

    if (std::fabs(yaw_error) > 0.1) {
      linear_speed = 0.0;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_speed;
    cmd.angular.z = angular_speed;
    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Path current_path_;
  size_t current_waypoint_index_;
  double robot_x_;
  double robot_y_;
  double robot_yaw_;
  double min_front_dist_{std::numeric_limits<double>::infinity()};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
