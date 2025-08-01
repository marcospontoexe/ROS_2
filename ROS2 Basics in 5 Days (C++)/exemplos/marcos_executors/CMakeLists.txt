cmake_minimum_required(VERSION 3.8)
project(marcos_executors)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

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

# Build 
add_executable(executor_example_1_node src/executor_example_1.cpp)
ament_target_dependencies(executor_example_1_node rclcpp)

add_executable(executor_example_2_node src/executor_example_2.cpp)
ament_target_dependencies(executor_example_2_node rclcpp nav_msgs)

add_executable(executor_example_3_node src/executor_example_3.cpp)
ament_target_dependencies(executor_example_3_node rclcpp nav_msgs)

add_executable(executor_example_4_node src/executor_example_4.cpp)
ament_target_dependencies(executor_example_4_node rclcpp nav_msgs)

add_executable(executor_example_4_singlethreaded_node src/executor_example_4_singlethreaded.cpp)
ament_target_dependencies(executor_example_4_singlethreaded_node rclcpp nav_msgs)

add_executable(executor_example_5_node src/executor_example_5.cpp)
ament_target_dependencies(executor_example_5_node rclcpp)

add_executable(executor_example_5_reentrant_node src/executor_example_5_reentrant.cpp)
ament_target_dependencies(executor_example_5_reentrant_node rclcpp)

add_executable(executor_example_5_mutualyexclusive_node src/executor_example_5_mutualyexclusive.cpp)
ament_target_dependencies(executor_example_5_mutualyexclusive_node rclcpp)

add_executable(executor_example_5_mutualyexclusive_multiple_node src/executor_example_5_mutualyexclusive_multiple.cpp)
ament_target_dependencies(executor_example_5_mutualyexclusive_multiple_node rclcpp)

# Install
install(TARGETS
   executor_example_1_node
   executor_example_2_node
   executor_example_3_node
   executor_example_4_node
   executor_example_4_singlethreaded_node
   executor_example_5_node
   executor_example_5_reentrant_node
   executor_example_5_mutualyexclusive_node
   executor_example_5_mutualyexclusive_multiple_node
   DESTINATION lib/${PROJECT_NAME}
 )

ament_package()
