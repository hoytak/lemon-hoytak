INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/lemon)

SET(TESTS
  adaptors_test
  bfs_test
  circulation_test
  counter_test
  dfs_test
  digraph_test
  dijkstra_test
  dim_test
  error_test
  graph_copy_test
  graph_test
  graph_utils_test
  hao_orlin_test
  heap_test
  kruskal_test
  maps_test
  max_matching_test
  radix_sort_test
  path_test
  preflow_test
  random_test
  suurballe_test
  time_measure_test
  unionfind_test)

FOREACH(TEST_NAME ${TESTS})
  ADD_EXECUTABLE(${TEST_NAME} ${TEST_NAME}.cc)
  TARGET_LINK_LIBRARIES(${TEST_NAME} lemon)
  ADD_TEST(${TEST_NAME} ${TEST_NAME})
ENDFOREACH(TEST_NAME)
