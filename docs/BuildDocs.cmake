# Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
#
# This script is a cross-platform version of the Makefile
#
# Usage:
# cmake [-DBINARY_DIR=<path>] [-DDOCS_BUILDER=(html|latex)] [-DCMAKE_INSTALL_PREFIX=<path>] -P BuildDocs.cmake
#
# Preconditions:
# + Overlay the python virtual environment as specified in the README.txt
#

# Look for an executable called sphinx-build
find_program(SPHINX_EXECUTABLE
             NAMES sphinx-build
             DOC "Path to sphinx-build executable")

if(NOT SPHINX_EXECUTABLE)
    message(FATAL_ERROR "Sphinx could not be located")
endif()

# Get version from sphinx's conf.py so that the cmake and the sphinx versions match
file(STRINGS
    conf.py
    LIB_VERSION_TMP
    REGEX "^version = u'[0-9]+.[0-9]+.[0-9]+'"
    )

string(REGEX REPLACE "^version = u'([0-9]+.[0-9]+.[0-9]+)'"
    "\\1"
    LIB_VERSION_STR
    ${LIB_VERSION_TMP}
    )

####################################################################################################
# Build Sphinx documentation
####################################################################################################

set(SPHINX_SOURCE "${CMAKE_SOURCE_DIR}")

if(NOT BINARY_DIR)
    set(BINARY_DIR "${CMAKE_BINARY_DIR}/build")
endif()

if(NOT DOCS_BUILDER)
    set(DOCS_BUILDER html)
endif()

# Generate the sphinx documentation
message(STATUS "Generating documentation with Sphinx")
execute_process(
    COMMAND ${SPHINX_EXECUTABLE}
    -b ${DOCS_BUILDER}
    -d "${BINARY_DIR}/doctrees"
    ${SPHINX_SOURCE}
    ${BINARY_DIR}/${DOCS_BUILDER}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND_ECHO STDOUT
    RESULT_VARIABLE HTML_RESULT) 

if(NOT HTML_RESULT EQUAL 0)
    message(FATAL_ERROR "Sphinx build failed with error ${HTML_RESULT}")
endif()

# Run spelling tests
message(STATUS "Checking spelling with Sphinx")
execute_process(
    COMMAND ${SPHINX_EXECUTABLE}
    -b spelling
    -Q -W --keep-going
    -d "${BINARY_DIR}/doctrees"
    ${SPHINX_SOURCE}
    ${BINARY_DIR}/spelling
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND_ECHO STDOUT
    RESULT_VARIABLE SPELL_RESULT) 

if(NOT SPELL_RESULT EQUAL 0)
    file(GLOB_RECURSE SPELL_ERROR_FILES "${BINARY_DIR}/spelling/*.spelling")
    execute_process(COMMAND ${CMAKE_COMMAND} -E cat ${SPELL_ERROR_FILES})
    message(SEND_ERROR "Sphinx spell failed with error ${SPELL_RESULT}")
endif()

# Install the generated docs
if(CMAKE_INSTALL_PREFIX)
    file(INSTALL
        ${BINARY_DIR}/${DOCS_BUILDER}
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        PATTERN ".buildinfo" EXCLUDE)
endif()
