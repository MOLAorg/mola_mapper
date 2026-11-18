# Unit tests:
mola_add_test(
  TARGET  test-global-graph
  SOURCES test-global-graph.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-lc-wait-idle
  SOURCES test-lc-wait-idle.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-gyro-integration
  SOURCES test-gyro-integration.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-mapper-ingestion
  SOURCES test-mapper-ingestion.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-camera-director
  SOURCES test-camera-director.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-keyframe-map-visualization
  SOURCES test-keyframe-map-visualization.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-loop-odometry-check
  SOURCES test-loop-odometry-check.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

# Loop closures on a simulated run (mola_test_datasets), as local map corrections:
find_package(mola_test_datasets QUIET)
if(mola_test_datasets_FOUND)
  # Install directory of the mola_sm_loop_closure pipelines, so these tests do not
  # need the `ros2` command line tool (not available in some CI environments):
  get_filename_component(MOLA_SM_LC_PREFIX "${mola_sm_loop_closure_DIR}/../../.." ABSOLUTE)
  set(MOLA_SM_LC_PIPELINES_DIR "${MOLA_SM_LC_PREFIX}/share/mola_sm_loop_closure/pipelines")
  mola_add_test(
    TARGET  test-local-map-corrections
    SOURCES test-local-map-corrections.cpp
    LINK_LIBRARIES
      mola::mola_mapper
  )
  mola_add_test(
    TARGET  test-mapper-replay
    SOURCES test-mapper-replay.cpp
    LINK_LIBRARIES
      mola::mola_mapper
  )
  target_compile_definitions(test-mapper-replay PRIVATE
    MOLA_SM_LC_PIPELINES_DIR="${MOLA_SM_LC_PIPELINES_DIR}"
    MOLA_TEST_WAREHOUSE_SIMPLEMAP="${mola_test_datasets_DIR}/../datasets/simplemaps/mvsim-warehouse01.simplemap"
    MOLA_MAPPER_LC_PIPELINE="${PROJECT_SOURCE_DIR}/params/loop-closure-f2f-mapper.yaml"
  )
  target_compile_definitions(test-local-map-corrections PRIVATE
    MOLA_SM_LC_PIPELINES_DIR="${MOLA_SM_LC_PIPELINES_DIR}"
    MOLA_TEST_WAREHOUSE_SIMPLEMAP="${mola_test_datasets_DIR}/../datasets/simplemaps/mvsim-warehouse01.simplemap"
    MOLA_MAPPER_LC_PIPELINE="${PROJECT_SOURCE_DIR}/params/loop-closure-f2f-mapper.yaml"
  )
endif()

mola_add_test(
  TARGET  test-wheel-odometry
  SOURCES test-wheel-odometry.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-loop-consistency
  SOURCES test-loop-consistency.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-graph-visualization
  SOURCES test-graph-visualization.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)
