cmake_minimum_required(VERSION 3.22)
project(lbr_moveit_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)


add_executable(hello_moveit src/hello_moveit.cpp)

ament_target_dependencies(hello_moveit
  geometry_msgs  
  moveit_ros_planning_interface
  rclcpp
)

add_executable(beer_tapping_node src/beer_tap_motion_planner.cpp)
ament_target_dependencies(beer_tapping_node
  geometry_msgs  
  moveit_ros_planning_interface
  rclcpp
)

add_executable(tapping_node src/tapping_motion_generator_node.cpp)
ament_target_dependencies(tapping_node
  geometry_msgs  
  moveit_ros_planning_interface
  rclcpp
)

add_executable(update_planning_scene src/update_planning_scene.cpp)
ament_target_dependencies(update_planning_scene
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  tf2_geometry_msgs
)


install(DIRECTORY launch scene_meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    hello_moveit
    tapping_node
    beer_tapping_node
    update_planning_scene
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
