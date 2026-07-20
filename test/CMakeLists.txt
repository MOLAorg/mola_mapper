# Unit tests:
mola_add_test(
  TARGET  test-mapper-basic
  SOURCES test-mapper-basic.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-navstate-fusion
  SOURCES test-navstate-fusion.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-imu-gnss-fusion
  SOURCES test-imu-gnss-fusion.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-keyframe-ingestion
  SOURCES test-keyframe-ingestion.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-georef-convergence
  SOURCES test-georef-convergence.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-optimizer-thread
  SOURCES test-optimizer-thread.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-highrate-aggregation
  SOURCES test-highrate-aggregation.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-kf-creation-gating
  SOURCES test-kf-creation-gating.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-imu-gravity-filter
  SOURCES test-imu-gravity-filter.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-multisource-georef-stability
  SOURCES test-multisource-georef-stability.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-relocalize-georef
  SOURCES test-relocalize-georef.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)

mola_add_test(
  TARGET  test-predict-twist-filter
  SOURCES test-predict-twist-filter.cpp
  LINK_LIBRARIES
    mola::mola_mapper
)
