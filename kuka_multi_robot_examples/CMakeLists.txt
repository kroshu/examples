cmake_minimum_required(VERSION 3.5)
project(kuka_multi_robot_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(moveit_dual_robot_planning_example src/moveit_dual_robot_planning_example.cpp)
ament_target_dependencies(moveit_dual_robot_planning_example
  moveit_ros_planning_interface
  rclcpp
  trajectory_msgs
)

install(TARGETS
  moveit_dual_robot_planning_example
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config srdf
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)

endif()

ament_package()