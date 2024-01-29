cmake_minimum_required(VERSION 3.8)
project(planner_playground)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(
  include
)

set(dependencies
  ament_cmake
  action_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  geometry_msgs
)

# test helper
add_executable(test_planner_with_rviz_node src/test_planner_with_rviz_node.cpp)
ament_target_dependencies(test_planner_with_rviz_node ${dependencies})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # skip copyright and license
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  test_planner_with_rviz_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
