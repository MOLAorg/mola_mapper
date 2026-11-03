# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

if("$ENV{ROS_VERSION}" STREQUAL "2")
    set(DETECTED_ROS2 TRUE)
endif()

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_mapper LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find dependencies:
find_package(mrpt_maps REQUIRED)
find_package(mola_kernel REQUIRED)
find_package(mola_yaml REQUIRED)
find_package(GTSAM REQUIRED)
find_package(mola_gtsam_factors REQUIRED)
find_package(mola_georeferencing REQUIRED)
find_package(mola_sm_loop_closure REQUIRED)

# -----------------------
# define lib:
set(LIB_SOURCES
  module/src/GlobalGraph.cpp
  module/src/GlobalGraphIO.cpp
  module/src/Mapper.cpp
  module/src/Mapper_LoopClosure.cpp
  module/src/Parameters.cpp
  module/src/register.cpp
)
set(LIB_PUBLIC_HEADERS
  module/include/mola_mapper/GlobalGraph.h
  module/include/mola_mapper/GlobalGraphIO.h
  module/include/mola_mapper/Mapper.h
  module/include/mola_mapper/Parameters.h
)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    gtsam
    mrpt::mrpt_maps
    mola::mola_kernel
    mola::mola_yaml
    mola::mola_gtsam_factors
    mola::mola_georeferencing
    mola::mola_sm_loop_closure
  CMAKE_DEPENDENCIES
    mola_kernel
    mola_yaml
    mrpt_maps
    mola_gtsam_factors
    mola_georeferencing
    mola_sm_loop_closure
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE module/src
)
install(
  DIRECTORY module/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

# Offline tool: re-optimize a saved global graph with other parameters
mola_add_executable(
  TARGET  mola-mapper-reoptimize
  SOURCES apps/mola-mapper-reoptimize.cpp
  LINK_LIBRARIES
    ${PROJECT_NAME}
)

install(
  DIRECTORY
    params
    mola-cli-launchs
  DESTINATION
    share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if(DETECTED_ROS2)
  # find dependencies
  find_package(ament_cmake REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()

    add_subdirectory(test)
  endif()

  ament_package()
endif()
