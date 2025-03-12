#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class DynamicPlannerNode : public rclcpp::Node
{
public:
  DynamicPlannerNode()
  : Node("dp_node") 
  {
   
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      10,
      std::bind(&DynamicPlannerNode::laserCallback, this, std::placeholders::_1));

    local_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/local_cost_map",
      10);

    RCLCPP_INFO(this->get_logger(), "Dynamic Planner Node started.");
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    
    auto local_map = buildLocalMap(scan_msg);

    local_map_pub_->publish(local_map);
  }

  nav_msgs::msg::OccupancyGrid buildLocalMap(const sensor_msgs::msg::LaserScan::SharedPtr &scan_msg)
  {
    nav_msgs::msg::OccupancyGrid grid;

    grid.header.stamp = this->now();
    grid.header.frame_id = "base_link";  

    double resolution = 0.05; 
    int width = 200;        
    int height = 200;        

    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = height;

    
    grid.info.origin.position.x = - (width * resolution) / 2.0;  // e.g. -5.0 m
    grid.info.origin.position.y = - (height * resolution) / 2.0; // e.g. -5.0 m

    
    grid.data.resize(width * height, 0);

   
    for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      double r = scan_msg->ranges[i];

      if (std::isinf(r) || r > 9.9) {
        continue;
      }
     
      double x = r * cos(angle);
      double y = r * sin(angle);

    
      int mx = static_cast<int>((x - grid.info.origin.position.x) / resolution);
      int my = static_cast<int>((y - grid.info.origin.position.y) / resolution);

      if (mx >= 0 && mx < width && my >= 0 && my < height) {
        int index = my * width + mx;
        grid.data[index] = 100;
      }
    }

    return grid;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}