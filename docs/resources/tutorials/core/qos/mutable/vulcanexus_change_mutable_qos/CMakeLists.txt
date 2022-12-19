cmake_minimum_required(VERSION 3.8)
project(vulcanexus_change_mutable_qos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(change_mutable_qos_pub1 src/change_mutable_qos_pub1.cpp)
ament_target_dependencies(change_mutable_qos_pub1 rclcpp rmw rmw_fastrtps_cpp std_msgs)

install(TARGETS
change_mutable_qos_pub1
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(change_mutable_qos_pub2 src/change_mutable_qos_pub2.cpp)
ament_target_dependencies(change_mutable_qos_pub2 rclcpp rmw rmw_fastrtps_cpp std_msgs)

install(TARGETS
change_mutable_qos_pub2
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(change_mutable_qos_sub src/change_mutable_qos_sub.cpp)
ament_target_dependencies(change_mutable_qos_sub rclcpp rmw rmw_fastrtps_cpp std_msgs)

install(TARGETS
change_mutable_qos_sub
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
