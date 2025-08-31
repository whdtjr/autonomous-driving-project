cmake_minimum_required(VERSION 3.8)
project(my_cpp_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# (권장) C++ 표준
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 실행 파일들
add_executable(timer_node src/timer_node.cpp)
ament_target_dependencies(timer_node rclcpp)

add_executable(fast_timer_node src/fast_timer_node.cpp)
ament_target_dependencies(fast_timer_node rclcpp)

# 실행 파일 설치
install(TARGETS
  timer_node
  fast_timer_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
