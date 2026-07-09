cmake_minimum_required(VERSION 3.5)
project(kuka_multi_robot_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)

endif()

ament_package()