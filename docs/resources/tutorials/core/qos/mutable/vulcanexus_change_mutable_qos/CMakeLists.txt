# Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

add_executable(change_mutable_qos_publisher src/change_mutable_qos_publisher.cpp)
ament_target_dependencies(change_mutable_qos_publisher rclcpp rmw rmw_fastrtps_cpp std_msgs)

install(TARGETS
change_mutable_qos_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
src/large_ownership_strength.xml
src/small_ownership_strength.xml
src/subscriber_exclusive_ownership.xml
  DESTINATION profiles/
)

ament_package()
