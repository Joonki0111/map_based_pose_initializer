cmake_minimum_required(VERSION 3.8)
project(map_based_pose_initializer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_auto_mapping_msgs REQUIRED)
find_package(autoware_localization_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_extension REQUIRED)

include_directories(include)

add_library(
  map_based_pose_initializer SHARED
  src/map_based_pose_initializer.cpp
)

ament_target_dependencies(
  map_based_pose_initializer
  rclcpp
  rclcpp_components
  autoware_adapi_v1_msgs
  autoware_auto_mapping_msgs
  autoware_localization_msgs
  lanelet2_core
  lanelet2_extension
)

rclcpp_components_register_nodes(
  map_based_pose_initializer
  "map_based_pose_initializer::MapBasedPoseInitializer"
)

install(TARGETS 
  map_based_pose_initializer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
