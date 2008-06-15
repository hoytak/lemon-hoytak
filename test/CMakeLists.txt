include_directories (${LEMON_SOURCE_DIR})

link_directories (${LEMON_BINARY_DIR}/lemon)

set (TESTS
  bfs_test
  counter_test
  dfs_test
  digraph_test
  dijkstra_test
  dim_test
  error_test
  graph_test
  graph_utils_test
  kruskal_test
  maps_test
  path_test
  random_test
  time_measure_test
  unionfind_test)

foreach (TEST_NAME ${TESTS})
  add_executable (${TEST_NAME} ${TEST_NAME}.cc)
  target_link_libraries (${TEST_NAME} lemon)
  add_test(${TEST_NAME} ${TEST_NAME})
endforeach (TEST_NAME)
