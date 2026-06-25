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
project(mola_mapper_3d LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find dependencies:
find_package(mrpt-maps REQUIRED)
find_package(mrpt-tclap REQUIRED)
find_package(mola_kernel REQUIRED)
find_package(mola_yaml REQUIRED)
find_package(mp2p_icp REQUIRED)
find_package(mp2p_icp_filters REQUIRED)
find_package(mola_pose_list REQUIRED)
find_package(GTSAM REQUIRED)
find_package(gtsam2mrpt_serial REQUIRED)
find_package(mola_gtsam_factors REQUIRED)
find_package(mola_imu_preintegration REQUIRED)

# -----------------------
# define lib:
set(LIB_SOURCES
  module/src/Mapper3D.cpp
  module/src/Mapper3D_Fusion.cpp
  module/src/Mapper3D_KeyframeIngestion.cpp
  module/src/Mapper3D_SensorCallbacks.cpp
  module/src/WorldModelState.cpp
  module/src/Parameters.cpp
  module/src/register.cpp
)
set(LIB_PUBLIC_HEADERS
  module/include/mola_mapper_3d/Mapper3D.h
  module/include/mola_mapper_3d/Parameters.h
  module/include/mola_mapper_3d/WorldModelState.h
)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    gtsam
    mrpt::maps
    mola::mola_kernel
    mola::mola_yaml
    mola::mp2p_icp
    mola::mp2p_icp_filters
    mola::mola_pose_list
    mola::gtsam2mrpt_serial
    mola::mola_gtsam_factors
    mola::mola_imu_preintegration
  CMAKE_DEPENDENCIES
    mola_kernel
    mola_yaml
    mp2p_icp
    mp2p_icp_filters
    mrpt-maps
    mola_pose_list
    gtsam2mrpt_serial
    mola_gtsam_factors
    mola_imu_preintegration
)

# mola_add_library() expects headers under "ROOT/include", so do this manually here:
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE module/src
)
install(
  DIRECTORY module/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

# CLI apps
# ---------------------------------
find_package(mola_input_rawlog) # optional
find_package(mola_input_rosbag2) # optional

mola_add_executable(
  TARGET  mola-mapper-3d-cli
  SOURCES apps/mola-mapper-3d-cli.cpp
  LINK_LIBRARIES
  ${PROJECT_NAME}
  mrpt::maps
  mrpt::tclap
)

if(mola_input_rawlog_FOUND)
  message(STATUS "mola-mapper-3d-cli: With Rawlog = YES")
  target_link_libraries(mola-mapper-3d-cli mola::mola_input_rawlog)
  target_compile_definitions(mola-mapper-3d-cli PRIVATE HAVE_MOLA_INPUT_RAWLOG)
else()
  message(STATUS "mola-mapper-3d-cli: With Rawlog = NO")
endif()

if(mola_input_rosbag2_FOUND)
  message(STATUS "mola-mapper-3d-cli: With rosbag2 = YES")
  target_link_libraries(mola-mapper-3d-cli mola::mola_input_rosbag2)
  target_compile_definitions(mola-mapper-3d-cli PRIVATE HAVE_MOLA_INPUT_ROSBAG2)
else()
  message(STATUS "mola-mapper-3d-cli: With rosbag2 = NO")
endif()

# Install files:
install(
  DIRECTORY
    params
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
