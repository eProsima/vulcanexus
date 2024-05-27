cmake_minimum_required(VERSION 3.5)
project(docs_turtlesim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)

# Generate message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Vector3.idl"
  "msg/KeyedTwist.idl"
  "msg/KeyedPose.idl"
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

set(turtlesim_node_SRCS
  src/QtMultiTurtle.cpp
  src/KeyedTurtle.cpp
)

set(turtlesim_node_HDRS
  include/docs_turtlesim/QtMultiTurtle.hpp
)

qt5_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})

# Attach to cpp generation target
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(turtlesim_square_move src/SquareMove.cpp)
ament_target_dependencies(turtlesim_square_move "geometry_msgs" "rclcpp" "std_srvs" "turtlesim")
target_link_libraries(turtlesim_square_move "${cpp_typesupport_target}")

add_executable(turtlesim_multi_qt ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
ament_target_dependencies(turtlesim_multi_qt "ament_index_cpp" "rclcpp")
target_link_libraries(turtlesim_multi_qt Qt5::Widgets "${cpp_typesupport_target}")

add_executable(turtlesim_multi_control src/controller_keyed_turtle.cpp)
ament_target_dependencies(turtlesim_multi_control "ament_index_cpp" "rclcpp")
target_link_libraries(turtlesim_multi_control "${cpp_typesupport_target}")

install(
  TARGETS
  turtlesim_square_move
  turtlesim_multi_qt
  turtlesim_multi_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()
