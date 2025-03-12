#include <rclcpp/rclcpp.hpp>

class DynamicPlannerNode : public rclcpp::Node
{
public:
  DynamicPlannerNode()
  : Node("dp_node")
  {
    RCLCPP_INFO(this->get_logger(), "Dynamic Planner Node started.");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
