cmake_minimum_required(VERSION 3.8)
project(tutorial_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(move_publisher src/move_publisher.cpp)
ament_target_dependencies(move_publisher rclcpp geometry_msgs)
add_executable(lidar_subscriber src/lidar_subscriber.cpp)
ament_target_dependencies(lidar_subscriber rclcpp sensor_msgs)
add_executable(wall_finder src/wall_finder.cpp)
ament_target_dependencies(wall_finder rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  move_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  lidar_subscriber
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  wall_finder
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
