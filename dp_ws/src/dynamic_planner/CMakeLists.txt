cmake_minimum_required(VERSION 3.8)
project(dynamic_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(dp_node src/dp_node.cpp)
add_executable(tf_broadcaster tf_broadcaster/tf_broadcaster.cpp)
add_executable(controller_node src/controller_node.cpp)

ament_target_dependencies(dp_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

ament_target_dependencies(tf_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
)

ament_target_dependencies(controller_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_geometry_msgs
)

install(TARGETS
  dp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
