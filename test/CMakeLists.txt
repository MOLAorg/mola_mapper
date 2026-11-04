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
