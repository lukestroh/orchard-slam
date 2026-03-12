cmake_minimum_required(VERSION 3.8)
project(orchard_slam_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

add_executable(imu_covariance_fixer_node src/imu_covariance_fixer_node.cpp)
add_executable(gps_covariance_fixer_node src/gps_covariance_fixer_node.cpp)
ament_target_dependencies(imu_covariance_fixer_node rclcpp sensor_msgs)
ament_target_dependencies(gps_covariance_fixer_node rclcpp sensor_msgs)

install(TARGETS
  imu_covariance_fixer_node
  gps_covariance_fixer_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
