cmake_minimum_required(VERSION 3.8)
project(moveit_task_constructor_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mtc_depalletizing_example src/mtc_depalletizing_task_node.cpp)
ament_target_dependencies(mtc_depalletizing_example lifecycle_msgs moveit_task_constructor_core rclcpp)
target_include_directories(mtc_depalletizing_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mtc_depalletizing_example PUBLIC c_std_99 cxx_std_17)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS mtc_depalletizing_example
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)

endif()

ament_package()
