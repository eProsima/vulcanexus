cmake_minimum_required(VERSION 3.12)

project(demo_keys_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorDataMsg.idl"
)

ament_export_dependencies(rosidl_default_runtime)

# Prepare executables
add_executable(right_sensor src/right_sensor.cpp)
ament_target_dependencies(right_sensor rclcpp)

add_executable(left_sensor src/left_sensor.cpp)
ament_target_dependencies(left_sensor rclcpp)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp)

# Attach to cpp generation target
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Link against the generated source files
target_link_libraries(right_sensor "${cpp_typesupport_target}")
target_link_libraries(left_sensor "${cpp_typesupport_target}")
target_link_libraries(controller "${cpp_typesupport_target}")

# Install
install(TARGETS
  right_sensor
  left_sensor
  controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
