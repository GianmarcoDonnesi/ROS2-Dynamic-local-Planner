# ROS2-Dynamic-local-Planner

<img src="https://github.com/GianmarcoDonnesi/ROS2-Dynamic-local-Planner/blob/main/demo.png" alt="Example Image" width="500"/>

---

## Project Goal
Welcome to the repository containing the code developed for the final project of the Robot Programming course (A.Y. 23/24)
This project implements a **dynamic path planning** system for mobile robots using ROS 2 (Humble Hawksbill). The planner is designed to operate in unknown or partially known environments, updating its internal map based on data received from a Lidar sensor.
The objective is to generate safe paths towards a specified goal while accounting for dynamically detected obstacles. The system also includes a separate controller node to follow the planned path.

---

## Core Features

* **Dynamic Mapping:**
    * Subscribes to `/scan` messages (sensor_msgs/msg/LaserScan).
    * Computes a local **distance map** around the robot using a BFS-based algorithm (Brushfire) to determine the distance to the nearest obstacles.
    * Integrates local distance maps into a global **distance map** (`/global_distance_map_viz`) using the **minimum rule** and TF transformations (`map` -> `odom` -> `base_link`).
* **Path Planning:**
    * Utilizes the **A\*** search algorithm to find the optimal path on the global distance map.
    * Employs a heuristic calculated via the **Dijkstra** algorithm on an empty map (starting from the goal) to guide the A\* search.
    * Cell traversal costs for A\* are derived from the distance to obstacles (higher cost closer to obstacles).
    * Subscribes to the `/goal_pose` topic (geometry_msgs/msg/PoseStamped) to receive navigation goals.
    * Publishes the found path on the `/planned_path` topic (nav_msgs/msg/Path).
* **Control and Replanning:**
    * A separate node (`controller_node`) subscribes to `/planned_path` and `/odom`.
    * Uses a simple P-controller to follow the path waypoint by waypoint.
    * Implements a basic **emergency stop** mechanism based on `/scan` if an obstacle is too close.
    * Sends a **replanning request** to the `dp_node` (on the `/request_replan` topic) when blocked by an obstacle.
* **ROS 2 Integration:** The system is developed as a ROS 2 C++ package (`ament_cmake`) for the Humble Hawksbill distribution on Ubuntu 22.04.

---

## Package Structure (`dynamic_planner`)

* **`dp_node`:** the main node performing dynamic mapping (distance map) and A* planning.
* **`controller_node`:** the node responsible for following the path published by `dp_node` and handling obstacle stops/replanning requests.
* **`tf_broadcaster`:** a simple node publishing a static `map` -> `odom` transform.

---

## System Requirements

* **Operating System:** Ubuntu 22.04 LTS
* **ROS 2 Distribution:** ROS 2 Humble Hawksbill (Desktop Install recommended)
* **Build Tools:** `build-essential`, `cmake`, `git`, `colcon-common-extensions`
    ```bash
    sudo apt update && sudo apt install build-essential cmake git python3-colcon-common-extensions
    ```
* **ROS 2 Dependencies:** Packages needed for building and running (many included with `ros-humble-desktop`):
    ```bash
    sudo apt update && sudo apt install -y \
      ros-humble-rclcpp \
      ros-humble-sensor-msgs \
      ros-humble-nav-msgs \
      ros-humble-geometry-msgs \
      ros-humble-tf2 \
      ros-humble-tf2-ros \
      ros-humble-tf2-geometry-msgs \
      ros-humble-std-msgs \
      ros-humble-rosidl-default-generators \
      ros-humble-turtlebot3-gazebo \
      ros-humble-rviz2 \
      ros-humble-tf2-tools
    ```
---

## Building

1.  **Clone the Repository:**
    ```bash
    git clone <REPOSITORY_URL> 
    # Assuming a ROS 2 workspace at ~/ros2_ws
    cd ~/ros2_ws/src
    ```

2.  **Build the Workspace:**
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install --packages-select dynamic_planner
    # Or build the entire workspace if needed:
    # colcon build --symlink-install
    ```
---

## Running (with TurtleBot3 Gazebo Simulation)

Open several terminals. In **each** terminal, source the ROS 2 environment first:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash # Adjust path to your workspace if different
```

Then follow these steps to correctly launch the simulation and the nodes of the `dynamic_planner` project.

**Steps:**

**Terminal 1 - Start Gazebo Simulation:**
    * Set the robot model and launch Gazebo (e.g., with the `turtlebot3_house` world):
        ```bash
        export TURTLEBOT3_MODEL=burger
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py # Or another world launch file
        ```
    * **Wait** for the Gazebo environment to load completely.

**Terminal 2 - Start TF Broadcaster:**
    * Launch the node that publishes the `map` -> `odom` transform (using simulation time):
        ```bash
        ros2 run dynamic_planner tf_broadcaster --ros-args --param use_sim_time:=true
        ```

**Terminal 3 - Start Planner Node (`dp_node`):**
    * Launch the planning node (using simulation time):
        ```bash
        ros2 run dynamic_planner dp_node --ros-args --param use_sim_time:=true
        ```

**Terminal 4 - Start Controller Node (`controller_node`):**
    * Launch the control node (using simulation time):
        ```bash
        ros2 run dynamic_planner controller_node --ros-args --param use_sim_time:=true
        ```

**Terminal 5 - Start RViz (Optional but Recommended):**
    * Launch the visualization tool:
        ```bash
        rviz2
        ```
    * **Configure RViz:**
        * Set `Global Options` -> `Fixed Frame` to `map`.
        * Add the necessary displays: `TF`, `RobotModel`, `LaserScan` (Topic: `/scan`), `Map` (Topic: `/global_distance_map_viz`, Color Scheme: `costmap`), `Path` (Topic: `/planned_path`).
        * Check the `Status` of each display.

**Terminal 6 (or RViz) - Send Goal:**
    * **After** all nodes are running and RViz is showing initial data:
    * Use the `Nav2 Goal` tool in RViz to graphically set a goal in the `map` frame.
    * Alternatively, from a terminal (remember `Ctrl+C` after the goal is sent/received):
        ```bash
        ros2 topic pub --rate 1 /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: X.X, y: Y.Y, z: 0.0}, orientation: {w: 1.0}}}'
        ```
        *(Replace `X.X` and `Y.Y` with the desired coordinates in the simulated world)*.

---

## **License**
This project is licensed under the [GPL-3.0 License](LICENSE). See the file for more details.
