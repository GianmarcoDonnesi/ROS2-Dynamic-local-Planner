#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>
#include <algorithm>
#include <deque>

const float MAX_DISTANCE = 10.0f;
const float LETHAL_DISTANCE = 0.001f;
const float SAFETY_DISTANCE = 0.15f;

class DynamicPlannerNode : public rclcpp::Node
{
public:
  DynamicPlannerNode()
  : Node("dp_node")
  {
    this->declare_parameter<double>("map_resolution", 0.05);
    this->declare_parameter<int>("global_map_width", 400);
    this->declare_parameter<int>("global_map_height", 400);
    this->declare_parameter<int>("local_map_width", 200);
    this->declare_parameter<int>("local_map_height", 200);

    map_resolution_ = this->get_parameter("map_resolution").as_double();
    global_map_width_ = this->get_parameter("global_map_width").as_int();
    global_map_height_ = this->get_parameter("global_map_height").as_int();
    local_map_width_ = this->get_parameter("local_map_width").as_int();
    local_map_height_ = this->get_parameter("local_map_height").as_int();

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&DynamicPlannerNode::laserCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&DynamicPlannerNode::odomCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&DynamicPlannerNode::goalCallback, this, std::placeholders::_1));

    replan_request_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/request_replan", 10, std::bind(&DynamicPlannerNode::replanRequestCallback, this, std::placeholders::_1));

    global_distance_map_viz_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_distance_map_viz", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    RCLCPP_INFO(this->get_logger(), "Dynamic Planner Node started.");

    initGlobalDistanceMap();
  }

private:
  void planPath();
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void replanRequestCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void initGlobalDistanceMap();
  bool computeLocalDistanceMap(const sensor_msgs::msg::LaserScan::SharedPtr &scan_msg,
                               std::vector<float>& dist_data,
                               nav_msgs::msg::MapMetaData& map_info);
  bool updateGlobalDistanceMap(const std::vector<float>& local_dist_data,
                               const nav_msgs::msg::MapMetaData& local_map_info,
                               const geometry_msgs::msg::Pose& robot_pose_in_map);
  void publishDistanceMapVisualization();
  void computeDijkstraHeuristicEmpty(int goal_mx, int goal_my);
  nav_msgs::msg::Path runAStar(
    const std::vector<float>& distance_map_data,
    const nav_msgs::msg::MapMetaData& map_info,
    double start_x, double start_y,
    double goal_x,  double goal_y);
  bool validCell(int mx, int my, const nav_msgs::msg::MapMetaData & map_info) const;
  int worldToMapX(double wx, const nav_msgs::msg::MapMetaData & map_info) const;
  int worldToMapY(double wy, const nav_msgs::msg::MapMetaData & map_info) const;
  double mapToWorldX(int mx, const nav_msgs::msg::MapMetaData & map_info) const;
  double mapToWorldY(int my, const nav_msgs::msg::MapMetaData & map_info) const;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr replan_request_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_distance_map_viz_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::vector<float> global_distance_data_;
  nav_msgs::msg::MapMetaData global_map_info_;

  std::vector<float> dijkstra_heuristic_;
  geometry_msgs::msg::Pose robot_pose_;
  geometry_msgs::msg::PoseStamped current_goal_;
  rclcpp::Time last_map_update_time_;

  bool pose_received_ = false;
  bool goal_received_ = false;
  bool map_updated_ = false;

  double map_resolution_;
  int global_map_width_;
  int global_map_height_;
  int local_map_width_;
  int local_map_height_;

};

void DynamicPlannerNode::planPath()
{
  if (!goal_received_ || !pose_received_ || !map_updated_) {
    return;
  }

  double start_x = robot_pose_.position.x;
  double start_y = robot_pose_.position.y;
  double goal_x = current_goal_.pose.position.x;
  double goal_y = current_goal_.pose.position.y;

  RCLCPP_INFO(this->get_logger(), "Planning from (%.2f, %.2f) to (%.2f, %.2f)",
              start_x, start_y, goal_x, goal_y);

  int goal_mx = worldToMapX(goal_x, global_map_info_);
  int goal_my = worldToMapY(goal_y, global_map_info_);

  if (!validCell(goal_mx, goal_my, global_map_info_)) {
      RCLCPP_ERROR(this->get_logger(), "Goal is outside map bounds or invalid.");
      return;
  }

  computeDijkstraHeuristicEmpty(goal_mx, goal_my);

  auto path = runAStar(global_distance_data_, global_map_info_, start_x, start_y, goal_x, goal_y);
  path.header.stamp = this->now();
  path.header.frame_id = "map";
  path_pub_->publish(path);
}

void DynamicPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
  pose_received_ = true;
}

void DynamicPlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (msg->header.frame_id != "map") {
        RCLCPP_WARN(this->get_logger(), "Received goal in frame '%s', expected 'map'. Ignoring goal.", msg->header.frame_id.c_str());
        return;
    }
    current_goal_ = *msg;
    goal_received_ = true;
    RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)",
                current_goal_.pose.position.x, current_goal_.pose.position.y);
    planPath(); 
}

void DynamicPlannerNode::replanRequestCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
    RCLCPP_INFO(this->get_logger(), "Replanning requested by controller.");
    planPath();
}

void DynamicPlannerNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  if (!pose_received_) return;

  std::vector<float> local_dist_map_data;
  nav_msgs::msg::MapMetaData local_map_info;
  if (!computeLocalDistanceMap(scan_msg, local_dist_map_data, local_map_info)) {
      RCLCPP_WARN(this->get_logger(), "Failed to compute local distance map.");
      return;
  }

  if(updateGlobalDistanceMap(local_dist_map_data, local_map_info, robot_pose_)) {
      publishDistanceMapVisualization();
  }
  map_updated_ = true;
}

void DynamicPlannerNode::initGlobalDistanceMap()
{
  global_map_info_.resolution = map_resolution_;
  global_map_info_.width = global_map_width_;
  global_map_info_.height = global_map_height_;
  global_map_info_.origin.position.x = -(global_map_width_ * map_resolution_) / 2.0;
  global_map_info_.origin.position.y = -(global_map_height_ * map_resolution_) / 2.0;
  global_map_info_.origin.orientation.w = 1.0;

  global_distance_data_.assign(global_map_width_ * global_map_height_, std::numeric_limits<float>::infinity());
  map_updated_ = true;
}

bool DynamicPlannerNode::computeLocalDistanceMap(const sensor_msgs::msg::LaserScan::SharedPtr &scan_msg, std::vector<float>& dist_data, nav_msgs::msg::MapMetaData& map_info)
{
    map_info.resolution = map_resolution_;
    map_info.width = local_map_width_;
    map_info.height = local_map_height_;
    map_info.origin.position.x = - (local_map_width_ * map_resolution_) / 2.0;
    map_info.origin.position.y = - (local_map_height_ * map_resolution_) / 2.0;
    map_info.origin.orientation.w = 1.0;

    dist_data.assign(local_map_width_ * local_map_height_, std::numeric_limits<float>::infinity());

    std::deque<std::pair<int, int>> queue;

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        double r = scan_msg->ranges[i];

        if (std::isinf(r) || std::isnan(r) || r > scan_msg->range_max || r < scan_msg->range_min) {
            continue;
        }

        double lx = r * std::cos(angle);
        double ly = r * std::sin(angle);

        int mx = static_cast<int>((lx - map_info.origin.position.x) / map_resolution_);
        int my = static_cast<int>((ly - map_info.origin.position.y) / map_resolution_);

        if (mx >= 0 && mx < local_map_width_ && my >= 0 && my < local_map_height_) {
            int index = my * local_map_width_ + mx;
            if (dist_data[index] == std::numeric_limits<float>::infinity()) {
                dist_data[index] = 0.0f;
                queue.push_back({mx, my});
            }
        }
    }

    if (queue.empty()) {
         return true;
    }

    std::vector<std::pair<int, int>> neighbors = { {1,0}, {-1,0}, {0,1}, {0,-1} };
    float step_cost = map_resolution_;

    while(!queue.empty()) {
        std::pair<int, int> current = queue.front();
        queue.pop_front();
        int cx = current.first;
        int cy = current.second;
        int c_idx = cy * local_map_width_ + cx;
        float current_dist = dist_data[c_idx];

        for(const auto& offset : neighbors) {
            int nx = cx + offset.first;
            int ny = cy + offset.second;

            if (nx >= 0 && nx < local_map_width_ && ny >= 0 && ny < local_map_height_) {
                int n_idx = ny * local_map_width_ + nx;
                float new_dist = current_dist + step_cost;

                if (new_dist < dist_data[n_idx]) {
                    dist_data[n_idx] = new_dist;
                    queue.push_back({nx, ny});
                }
            }
        }
    }

     for (float& dist : dist_data) {
         if (dist > MAX_DISTANCE) {
             dist = MAX_DISTANCE;
         }
     }

    return true;
}

bool DynamicPlannerNode::updateGlobalDistanceMap(const std::vector<float>& local_dist_data, const nav_msgs::msg::MapMetaData& local_map_info, const geometry_msgs::msg::Pose& robot_pose_in_map)
{
    tf2::Transform robot_tf;
    tf2::fromMsg(robot_pose_in_map, robot_tf);

    bool map_changed = false;
    int local_width = local_map_info.width;

    for (int ly = 0; ly < (int)local_map_info.height; ++ly) {
        for (int lx = 0; lx < (int)local_map_info.width; ++lx) {
            int local_index = ly * local_width + lx;
            float local_dist = local_dist_data[local_index];

            if (local_dist == std::numeric_limits<float>::infinity()) {
                continue;
            }

            double local_wx = local_map_info.origin.position.x + (lx + 0.5) * local_map_info.resolution;
            double local_wy = local_map_info.origin.position.y + (ly + 0.5) * local_map_info.resolution;
            tf2::Vector3 local_point(local_wx, local_wy, 0.0);
            tf2::Vector3 global_point = robot_tf * local_point;

            int gx_cell = worldToMapX(global_point.x(), global_map_info_);
            int gy_cell = worldToMapY(global_point.y(), global_map_info_);

            if (validCell(gx_cell, gy_cell, global_map_info_)) {
                int global_index = gy_cell * global_map_info_.width + gx_cell;

                if (local_dist < global_distance_data_[global_index]) {
                    global_distance_data_[global_index] = local_dist;
                    map_changed = true;
                }
            }
        }
    }

    if (map_changed) {
         last_map_update_time_ = this->now();
    }
    return map_changed;
}

void DynamicPlannerNode::publishDistanceMapVisualization()
{
      nav_msgs::msg::OccupancyGrid viz_map;
      viz_map.header.stamp = last_map_update_time_;
      viz_map.header.frame_id = "map";
      viz_map.info = global_map_info_;
      viz_map.data.resize(global_distance_data_.size());
      float max_dist_scaled = SAFETY_DISTANCE * 2.0f;

      for (size_t i = 0; i < global_distance_data_.size(); ++i) {
          float dist = global_distance_data_[i];
          if (dist == std::numeric_limits<float>::infinity()) {
              viz_map.data[i] = -1;
          } else if (dist < LETHAL_DISTANCE) {
              viz_map.data[i] = 100;
          } else {
              int8_t cost = static_cast<int8_t>(std::max(0.0, 100.0 * (1.0 - dist / max_dist_scaled)));
              viz_map.data[i] = cost;
          }
      }
      global_distance_map_viz_pub_->publish(viz_map);
}

void DynamicPlannerNode::computeDijkstraHeuristicEmpty(int goal_mx, int goal_my)
{
  int width  = global_map_info_.width;
  int height = global_map_info_.height;
  double resolution = global_map_info_.resolution;

  if(width <= 0 || height <= 0) return;
  dijkstra_heuristic_.assign(width * height, std::numeric_limits<float>::infinity());
  auto indexOf = [&](int x, int y){ return y * width + x; };

  if (!validCell(goal_mx, goal_my, global_map_info_)) {
      RCLCPP_ERROR(get_logger(), "Goal cell (%d, %d) is invalid for Dijkstra.", goal_mx, goal_my);
      return;
  }
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
  float sqrt2 = std::sqrt(2.0f);

  while (!pq.empty()) {
    Cell current = pq.top();
    pq.pop();
    int cindex = indexOf(current.x, current.y);

    if (current.dist > dijkstra_heuristic_[cindex])
      continue;

    for (auto &nb : neighbors) {
      int nx = current.x + nb.first;
      int ny = current.y + nb.second;
      if (!validCell(nx, ny, global_map_info_))
        continue;

      float step_cost = (nb.first != 0 && nb.second != 0) ? sqrt2 : 1.0f;
      float new_dist = current.dist + step_cost;
      int nindex = indexOf(nx, ny);

      if (new_dist < dijkstra_heuristic_[nindex]) {
        dijkstra_heuristic_[nindex] = new_dist;
        pq.push({nx, ny, new_dist});
      }
    }
  }
  for (size_t i = 0; i < dijkstra_heuristic_.size(); i++) {
    if (dijkstra_heuristic_[i] != std::numeric_limits<float>::infinity()){
      dijkstra_heuristic_[i] *= resolution;
    }
  }
}

nav_msgs::msg::Path DynamicPlannerNode::runAStar(
  const std::vector<float>& distance_map_data,
  const nav_msgs::msg::MapMetaData& map_info,
  double start_x, double start_y,
  double goal_x,  double goal_y)
{
  nav_msgs::msg::Path path_result;
  int start_mx = worldToMapX(start_x, map_info);
  int start_my = worldToMapY(start_y, map_info);
  int goal_mx  = worldToMapX(goal_x, map_info);
  int goal_my  = worldToMapY(goal_y, map_info);

  if (!validCell(start_mx, start_my, map_info) || !validCell(goal_mx, goal_my, map_info)) {
    RCLCPP_WARN(this->get_logger(), "Start or Goal out of map bounds.");
    return path_result;
  }

  int width = map_info.width;
  int height = map_info.height;
  if (width <= 0 || height <= 0) return path_result;

  double resolution = map_info.resolution;
  float sqrt2 = std::sqrt(2.0f);

  std::vector<float> g_cost(width * height, std::numeric_limits<float>::infinity());
  std::vector<int> parent(width * height, -1);
  std::vector<bool> closed_list(width * height, false);

  auto indexOf = [&](int x, int y){ return y * width + x; };
  int start_index = indexOf(start_mx, start_my);

  if (start_index < 0 || start_index >= (int)distance_map_data.size()) {
      RCLCPP_ERROR(this->get_logger(), "Start index %d out of bounds for distance map.", start_index);
      return path_result;
  }
  if (distance_map_data[start_index] < LETHAL_DISTANCE) {
       RCLCPP_ERROR(this->get_logger(), "Start cell (%d, %d) is too close to obstacle (dist: %.3f).", start_mx, start_my, distance_map_data[start_index]);
       return path_result;
  }

  g_cost[start_index] = 0.0f;

  struct Cell {
    int index;
    float f;
    bool operator>(const Cell &other) const { return f > other.f; }
  };
  std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> open_list;

   if (dijkstra_heuristic_.size() != (size_t)(width * height)) {
       RCLCPP_ERROR(this->get_logger(), "Heuristic map size mismatch. Recalculating.");
       computeDijkstraHeuristicEmpty(goal_mx, goal_my);
       if(dijkstra_heuristic_.empty() || dijkstra_heuristic_.size() != (size_t)(width*height) ) {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute heuristic correctly.");
            return path_result;
       }
   }
   if (start_index < 0 || start_index >= (int)dijkstra_heuristic_.size()) {
       RCLCPP_ERROR(this->get_logger(), "Start index %d out of bounds for heuristic.", start_index);
       return path_result;
   }

  float h_start = (dijkstra_heuristic_[start_index] == std::numeric_limits<float>::infinity()) ? std::hypot(start_x - goal_x, start_y - goal_y) : dijkstra_heuristic_[start_index];
  open_list.push({start_index, h_start});

  std::vector<std::pair<int,int>> neighbors = {
    {1,0}, {-1,0}, {0,1}, {0,-1},
    {1,1}, {1,-1}, {-1,1}, {-1,-1}
  };
  bool found_path = false;
  int goal_index = indexOf(goal_mx, goal_my);

  while (!open_list.empty()) {
    Cell current_cell = open_list.top();
    open_list.pop();
    int cindex = current_cell.index;

    if (closed_list[cindex]) continue;
    closed_list[cindex] = true;

    if (cindex == goal_index) {
      found_path = true;
      break;
    }

    int cx = cindex % width;
    int cy = cindex / width;

    for (auto &nb_offset : neighbors) {
      int nx = cx + nb_offset.first;
      int ny = cy + nb_offset.second;
      if (!validCell(nx, ny, map_info)) continue;

      int nindex = indexOf(nx, ny);
      if (closed_list[nindex]) continue;

      if (nindex < 0 || nindex >= (int)distance_map_data.size()) continue;
      float distance_to_obstacle = distance_map_data[nindex];

      if (distance_to_obstacle < LETHAL_DISTANCE) continue;

      float step_cost = (nb_offset.first != 0 && nb_offset.second != 0) ? sqrt2 : 1.0f;
      step_cost *= resolution;

      float distance_cost = 0.0f;
      if (distance_to_obstacle < SAFETY_DISTANCE) {
          distance_cost = 5.0f * (SAFETY_DISTANCE - distance_to_obstacle);
      }
      float total_step_cost = step_cost * (1.0f + distance_cost);

      float tentative_g_cost = g_cost[cindex] + total_step_cost;

      if (tentative_g_cost >= g_cost[nindex]) continue;

      parent[nindex] = cindex;
      g_cost[nindex] = tentative_g_cost;

      if (nindex < 0 || nindex >= (int)dijkstra_heuristic_.size()) continue;
      float h_neighbor = (dijkstra_heuristic_[nindex] == std::numeric_limits<float>::infinity()) ?
                  std::hypot(mapToWorldX(nx, map_info) - goal_x, mapToWorldY(ny, map_info) - goal_y) : dijkstra_heuristic_[nindex];
      float f_neighbor = tentative_g_cost + h_neighbor;

      open_list.push({nindex, f_neighbor});
    }
  }

  if (found_path) {
       std::vector<int> path_indices;
       int current_idx = goal_index;
       while (current_idx != -1) {
           path_indices.push_back(current_idx);
           if (current_idx == start_index) break;
           int temp_parent_idx = parent[current_idx];
            if (temp_parent_idx != -1 && (temp_parent_idx < 0 || temp_parent_idx >= (int)parent.size())) {
                 RCLCPP_ERROR(get_logger(), "Path reconstruction error: Invalid parent index %d.", temp_parent_idx);
                 return nav_msgs::msg::Path();
            }
            if (temp_parent_idx != -1 && std::find(path_indices.begin(), path_indices.end(), temp_parent_idx) != path_indices.end()) {
                 RCLCPP_ERROR(get_logger(), "Path reconstruction error: Cycle detected.");
                 return nav_msgs::msg::Path();
            }
            current_idx = temp_parent_idx;
       }
       std::reverse(path_indices.begin(), path_indices.end());

       path_result.poses.reserve(path_indices.size());
       for (int idx : path_indices) {
           int x = idx % width;
           int y = idx / width;
           geometry_msgs::msg::PoseStamped pose;
           pose.header.frame_id = "map";
           pose.header.stamp = this->now();
           pose.pose.position.x = mapToWorldX(x, map_info);
           pose.pose.position.y = mapToWorldY(y, map_info);
           pose.pose.orientation.w = 1.0;
           path_result.poses.push_back(pose);
       }
        RCLCPP_INFO(this->get_logger(), "Path found with %zu poses.", path_result.poses.size());
   } else {
     RCLCPP_WARN(this->get_logger(), "No path found by A*.");
   }
  return path_result;
}

bool DynamicPlannerNode::validCell(int mx, int my, const nav_msgs::msg::MapMetaData & map_info) const
{
  return mx >= 0 && my >= 0 && mx < (int)map_info.width && my < (int)map_info.height;
}

int DynamicPlannerNode::worldToMapX(double wx, const nav_msgs::msg::MapMetaData & map_info) const
{
  return static_cast<int>((wx - map_info.origin.position.x) / map_info.resolution);
}

int DynamicPlannerNode::worldToMapY(double wy, const nav_msgs::msg::MapMetaData & map_info) const
{
  return static_cast<int>((wy - map_info.origin.position.y) / map_info.resolution);
}

double DynamicPlannerNode::mapToWorldX(int mx, const nav_msgs::msg::MapMetaData & map_info) const
{
  return map_info.origin.position.x + (mx + 0.5) * map_info.resolution;
}

double DynamicPlannerNode::mapToWorldY(int my, const nav_msgs::msg::MapMetaData & map_info) const
{
   return map_info.origin.position.y + (my + 0.5) * map_info.resolution;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}