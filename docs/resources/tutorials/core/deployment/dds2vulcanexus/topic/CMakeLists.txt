# Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#####################################################################
# Project configuration
#####################################################################
cmake_minimum_required(VERSION 3.16.3)
project(dds2vulcanexus)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#####################################################################
# Vulcanexus application
#####################################################################
message(STATUS "Configuring Vulcanexus application...")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(type_files
  "idl/HelloWorld.idl"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${type_files}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(vulcanexus_publisher vulcanexus_app/publisher.cpp)
ament_target_dependencies(vulcanexus_publisher rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vulcanexus_publisher "${cpp_typesupport_target}")

install(TARGETS
  vulcanexus_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

#####################################################################
# Fast DDS application
#####################################################################
message(STATUS "Configuring Fast DDS application...")
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_program(FASTDDSGEN fastddsgen)
set(
  GENERATED_TYPE_SUPPORT_FILES
  ${CMAKE_SOURCE_DIR}/fastdds_app/HelloWorld.h
  ${CMAKE_SOURCE_DIR}/fastdds_app/HelloWorld.cxx
  ${CMAKE_SOURCE_DIR}/fastdds_app/HelloWorldPubSubTypes.h
  ${CMAKE_SOURCE_DIR}/fastdds_app/HelloWorldPubSubTypes.cxx
)
add_custom_command(
  OUTPUT ${GENERATED_TYPE_SUPPORT_FILES}
  COMMAND ${FASTDDSGEN}
  -replace
  -typeros2
  -d ${CMAKE_SOURCE_DIR}/fastdds_app
  ${CMAKE_SOURCE_DIR}/idl/HelloWorld.idl
  DEPENDS ${CMAKE_SOURCE_DIR}/idl/HelloWorld.idl
  COMMENT "Fast DDS type support generation" VERBATIM
)

add_executable(
  fastdds_subscriber
  ${CMAKE_SOURCE_DIR}/fastdds_app/subscriber.cpp
  ${GENERATED_TYPE_SUPPORT_FILES}
)
target_link_libraries(fastdds_subscriber fastrtps fastcdr)
install(TARGETS
  fastdds_subscriber
  DESTINATION lib/${PROJECT_NAME}
)
