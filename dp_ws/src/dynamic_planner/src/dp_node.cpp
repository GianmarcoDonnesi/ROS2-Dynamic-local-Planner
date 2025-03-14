#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath> 
#include <limits>
#include <queue>      
#include <vector>     
#include <algorithm>  

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

    global_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/global_cost_map",
      10);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    RCLCPP_INFO(this->get_logger(), "Dynamic Planner Node started.");

    initGlobalMap();

    timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&DynamicPlannerNode::planTimerCallback, this));
  }

private:

  void initGlobalMap()
  {
    double resolution = 0.05;
    int width = 400;
    int height = 400;

    global_map_.header.frame_id = "map";
    global_map_.info.resolution = resolution;
    global_map_.info.width = width;
    global_map_.info.height = height;

    global_map_.info.origin.position.x = -(width * resolution) / 2.0;
    global_map_.info.origin.position.y = -(height * resolution) / 2.0;

    global_map_.data.resize(width * height, 0);
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    auto local_map = buildLocalMap(scan_msg);

    local_map_pub_->publish(local_map);

    double robot_x = 0.0;  
    double robot_y = 0.0;

    updateGlobalMap(local_map, robot_x, robot_y);

    global_map_pub_->publish(global_map_);
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

    grid.info.origin.position.x = - (width * resolution) / 2.0;
    grid.info.origin.position.y = - (height * resolution) / 2.0;

    grid.data.resize(width * height, 0);

    for (size_t i = 0; i < scan_msg->ranges.size(); i++)
    {
      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      double r = scan_msg->ranges[i];

      if (std::isinf(r) || r > 9.9) {
        continue;
      }

      double x = r * std::cos(angle);
      double y = r * std::sin(angle);

      int mx = static_cast<int>((x - grid.info.origin.position.x) / resolution);
      int my = static_cast<int>((y - grid.info.origin.position.y) / resolution);

      if (mx >= 0 && mx < width && my >= 0 && my < height)
      {
        int index = my * width + mx;
        grid.data[index] = 100; 
      }
    }

    return grid;
  }

  void updateGlobalMap(const nav_msgs::msg::OccupancyGrid & local_map, double robot_x, double robot_y)
  {
  
    for (int ly = 0; ly < (int)local_map.info.height; ++ly)
    {
      for (int lx = 0; lx < (int)local_map.info.width; ++lx)
      {
        int local_index = ly * local_map.info.width + lx;
        if (local_map.data[local_index] == 100)
        {
        
          double wx = (lx * local_map.info.resolution) + local_map.info.origin.position.x;
          double wy = (ly * local_map.info.resolution) + local_map.info.origin.position.y;

          double gx = wx + robot_x;
          double gy = wy + robot_y;

          int gx_cell = static_cast<int>((gx - global_map_.info.origin.position.x) / global_map_.info.resolution);
          int gy_cell = static_cast<int>((gy - global_map_.info.origin.position.y) / global_map_.info.resolution);

          if (gx_cell >= 0 && gx_cell < (int)global_map_.info.width &&
              gy_cell >= 0 && gy_cell < (int)global_map_.info.height)
          {
            int global_index = gy_cell * global_map_.info.width + gx_cell;
            global_map_.data[global_index] = 100;
          }
        }
      }
    }

    global_map_.header.stamp = this->now();
  }

  void planTimerCallback()
  {
  
    double start_x = 0.0;
    double start_y = 0.0;
    double goal_x  = 2.0; 
    double goal_y  = 2.0;

    int goal_mx = worldToMapX(goal_x, global_map_);
    int goal_my = worldToMapY(goal_y, global_map_);

    computeDijkstraHeuristicEmpty(goal_mx, goal_my);

    auto path = runAStar(global_map_, start_x, start_y, goal_x, goal_y);
    path.header.stamp = this->now();
    path.header.frame_id = "map";  
    path_pub_->publish(path);
  }

  void computeDijkstraHeuristicEmpty(int goal_mx, int goal_my)
  {
    int width  = global_map_.info.width;
    int height = global_map_.info.height;
    double resolution = global_map_.info.resolution;

    dijkstra_heuristic_.resize(width * height, std::numeric_limits<float>::infinity());

    auto indexOf = [&](int x, int y){ return y * width + x; };
    int goal_index = indexOf(goal_mx, goal_my);

    dijkstra_heuristic_[goal_index] = 0.0f;

    struct Cell {
      int x, y;
      float dist;
      bool operator>(const Cell &other) const { return dist > other.dist; }
    };
    std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> pq;
    pq.push({goal_mx, goal_my, 0.0f});

    std::vector<std::pair<int,int>> neighbors = {
      {1,0}, {-1,0}, {0,1}, {0,-1},
      {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };

    while (!pq.empty())
    {
      Cell current = pq.top();
      pq.pop();
      int cindex = indexOf(current.x, current.y);

      float cdist = dijkstra_heuristic_[cindex];
      if (std::fabs(cdist - current.dist) > 1e-5) {
        continue;
      }

      for (auto &nb : neighbors)
      {
        int nx = current.x + nb.first;
        int ny = current.y + nb.second;
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
          continue;
        }

        float step_cost = std::sqrt(nb.first*nb.first + nb.second*nb.second);
        float new_dist = cdist + step_cost;
        int nindex = indexOf(nx, ny);

        if (new_dist < dijkstra_heuristic_[nindex])
        {
          dijkstra_heuristic_[nindex] = new_dist;
          pq.push({nx, ny, new_dist});
        }
      }
    }

    for (size_t i = 0; i < dijkstra_heuristic_.size(); i++) {
      dijkstra_heuristic_[i] *= resolution;
    }
  }


  nav_msgs::msg::Path runAStar(
    const nav_msgs::msg::OccupancyGrid & map,
    double start_x, double start_y,
    double goal_x,  double goal_y)
  {
    nav_msgs::msg::Path path_result;

    int start_mx = worldToMapX(start_x, map);
    int start_my = worldToMapY(start_y, map);
    int goal_mx  = worldToMapX(goal_x, map);
    int goal_my  = worldToMapY(goal_y, map);

    if (!validCell(start_mx, start_my, map) || !validCell(goal_mx, goal_my, map)) {
      RCLCPP_WARN(this->get_logger(), "Start or Goal out of map bounds.");
      return path_result;
    }

    int width  = map.info.width;
    int height = map.info.height;

    std::vector<float> g_cost(width * height, std::numeric_limits<float>::infinity());
    std::vector<bool> visited(width * height, false);
    std::vector<int> parent(width * height, -1);

    auto indexOf = [&](int x, int y){ return y * width + x; };
    int start_index = indexOf(start_mx, start_my);
    g_cost[start_index] = 0.0f;


    struct Cell {
      int x, y;
      float f;
      bool operator>(const Cell &other) const { return f > other.f; }
    };
    std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> open_list;

    float h_start = dijkstra_heuristic_[start_index];
    open_list.push({start_mx, start_my, h_start});

    //8-direction neighbors
    std::vector<std::pair<int,int>> neighbors = {
      {1,0}, {-1,0}, {0,1}, {0,-1},
      {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };

    bool found_path = false;

    while (!open_list.empty())
    {
      Cell current = open_list.top();
      open_list.pop();
      int cindex = indexOf(current.x, current.y);

      if (visited[cindex]) {
        continue;
      }
      visited[cindex] = true;

      if (current.x == goal_mx && current.y == goal_my) {
        found_path = true;
        break;
      }

      for (auto &nb : neighbors) {
        int nx = current.x + nb.first;
        int ny = current.y + nb.second;
        if (!validCell(nx, ny, map)) {
          continue;
        }
        int nindex = indexOf(nx, ny);
        if (visited[nindex]) {
          continue;
        }
        if (map.data[nindex] == 100) {
          continue;
        }

        float step = std::sqrt(nb.first*nb.first + nb.second*nb.second);
        float cost_to_nb = g_cost[cindex] + step;

        if (cost_to_nb < g_cost[nindex]) {
          g_cost[nindex] = cost_to_nb;
          parent[nindex] = cindex;

          float h = dijkstra_heuristic_[nindex];
          float f = cost_to_nb + h;
          open_list.push({nx, ny, f});
        }
      }
    }

    if (found_path)
    {
      std::vector<int> path_indices;
      int goal_index = indexOf(goal_mx, goal_my);
      int current = goal_index;
      while (current != -1) {
        path_indices.push_back(current);
        current = parent[current];
      }
      std::reverse(path_indices.begin(), path_indices.end());

      for (int idx : path_indices) {
        int x = idx % width;
        int y = idx / width;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = mapToWorldX(x, map);
        pose.pose.position.y = mapToWorldY(y, map);
        pose.pose.orientation.w = 1.0;
        path_result.poses.push_back(pose);
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No path found by A*.");
    }

    return path_result;
  }


  //Helper functions
  bool validCell(int mx, int my, const nav_msgs::msg::OccupancyGrid & map)
  {
    if (mx < 0 || my < 0) return false;
    if (mx >= (int)map.info.width || my >= (int)map.info.height) return false;
    return true;
  }

  int worldToMapX(double wx, const nav_msgs::msg::OccupancyGrid & map)
  {
    double origin_x = map.info.origin.position.x;
    double res = map.info.resolution;
    return static_cast<int>((wx - origin_x) / res);
  }

  int worldToMapY(double wy, const nav_msgs::msg::OccupancyGrid & map)
  {
    double origin_y = map.info.origin.position.y;
    double res = map.info.resolution;
    return static_cast<int>((wy - origin_y) / res);
  }

  double mapToWorldX(int mx, const nav_msgs::msg::OccupancyGrid & map)
  {
    double origin_x = map.info.origin.position.x;
    double res = map.info.resolution;
    return origin_x + (mx + 0.5) * res;
  }

  double mapToWorldY(int my, const nav_msgs::msg::OccupancyGrid & map)
  {
    double origin_y = map.info.origin.position.y;
    double res = map.info.resolution;
    return origin_y + (my + 0.5) * res;
  }


  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  nav_msgs::msg::OccupancyGrid global_map_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<float> dijkstra_heuristic_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}