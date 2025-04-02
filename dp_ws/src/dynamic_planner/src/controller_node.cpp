#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
    robot_yaw_(0.0)
  {
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path",
      10,
      std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&PathFollower::odomCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10);

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
  

 
  void controlLoop()
  {
    if (current_path_.poses.empty() || (current_waypoint_index_ >= current_path_.poses.size())) {
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

    if (std::fabs(yaw_error) > 0.3) {
      linear_speed *= 0.5;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_speed;
    cmd.angular.z = angular_speed;
    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Path current_path_;
  size_t current_waypoint_index_;
  double robot_x_;
  double robot_y_;
  double robot_yaw_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
