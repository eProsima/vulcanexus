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

cmake_minimum_required(VERSION 3.8)
project(vulcanexus_base)

set(vulcanexus_base_MAJOR_VERSION 4)
set(vulcanexus_base_MINOR_VERSION 3)
set(vulcanexus_base_PATCH_VERSION 2)
set(vulcanexus_base_MAJOR_VERSION
    ${vulcanexus_base_MAJOR_VERSION}.${vulcanexus_base_MINOR_VERSION}.${vulcanexus_base_PATCH_VERSION})

if (NOT DEFINED ENV{VULCANEXUS_HOME})
    set(VULCANEXUS_HOME ${CMAKE_INSTALL_PREFIX})
else()
    set(VULCANEXUS_HOME $ENV{VULCANEXUS_HOME})
endif()
string(REPLACE ${PROJECT_NAME} "" VULCANEXUS_HOME ${VULCANEXUS_HOME})

# Configure bash script
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/environment/vulcanexus_base-setup.bash.in
    ${CMAKE_INSTALL_PREFIX}/share/vulcanexus_base/environment/vulcanexus_base-setup.bash
    @ONLY)

install(FILES resource/vulcanexus_base DESTINATION share/ament_index/resource_index/packages)
install(FILES resource/package.dsv DESTINATION share/vulcanexus_base)
