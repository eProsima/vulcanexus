cmake_minimum_required(VERSION 3.5)
project(docs_turtlesim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtlesim_square_move src/SquareMove.cpp)

ament_target_dependencies(turtlesim_square_move "ament_index_cpp" "geometry_msgs" "rclcpp" "std_msgs" "std_srvs" "turtlesim")

target_link_libraries(turtlesim_square_move "${cpp_typesupport_target}")

install(TARGETS turtlesim_square_move
  DESTINATION lib/${PROJECT_NAME})

ament_package()
