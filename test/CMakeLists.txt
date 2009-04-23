INCLUDE_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}
)

LINK_DIRECTORIES(${PROJECT_BINARY_DIR}/lemon)

SET(TESTS
  adaptors_test
  bfs_test
  circulation_test
  counter_test
  dfs_test
  digraph_test
  dijkstra_test
  dim_test
  edge_set_test
  error_test
  euler_test
  gomory_hu_test
  graph_copy_test
  graph_test
  graph_utils_test
  hao_orlin_test
  heap_test
  kruskal_test
  maps_test
  matching_test
  min_cost_arborescence_test
  min_cost_flow_test
  path_test
  preflow_test
  radix_sort_test
  random_test
  suurballe_test
  time_measure_test
  unionfind_test)

IF(HAVE_LP)
  ADD_EXECUTABLE(lp_test lp_test.cc)
  IF(HAVE_GLPK)
    TARGET_LINK_LIBRARIES(lp_test lemon ${GLPK_LIBRARIES})
  ENDIF(HAVE_GLPK)
  ADD_TEST(lp_test lp_test)

  IF(WIN32 AND HAVE_GLPK)
    GET_TARGET_PROPERTY(TARGET_LOC lp_test LOCATION)
    GET_FILENAME_COMPONENT(TARGET_PATH ${TARGET_LOC} PATH)
    ADD_CUSTOM_COMMAND(TARGET lp_test POST_BUILD
      COMMAND cmake -E copy ${GLPK_BIN_DIR}/glpk.dll ${TARGET_PATH}
      COMMAND cmake -E copy ${GLPK_BIN_DIR}/libltdl3.dll ${TARGET_PATH}
      COMMAND cmake -E copy ${GLPK_BIN_DIR}/zlib1.dll ${TARGET_PATH}
    )
  ENDIF(WIN32 AND HAVE_GLPK)
ENDIF(HAVE_LP)

IF(HAVE_MIP)
  ADD_EXECUTABLE(mip_test mip_test.cc)
  IF(HAVE_GLPK)
    TARGET_LINK_LIBRARIES(mip_test lemon ${GLPK_LIBRARIES})
  ENDIF(HAVE_GLPK)
  ADD_TEST(mip_test mip_test)

  IF(WIN32 AND HAVE_GLPK)
    GET_TARGET_PROPERTY(TARGET_LOC mip_test LOCATION)
    GET_FILENAME_COMPONENT(TARGET_PATH ${TARGET_LOC} PATH)
    ADD_CUSTOM_COMMAND(TARGET mip_test POST_BUILD
      COMMAND cmake -E copy ${GLPK_BIN_DIR}/glpk.dll ${TARGET_PATH}
      COMMAND cmake -E copy ${GLPK_BIN_DIR}/libltdl3.dll ${TARGET_PATH}
      COMMAND cmake -E copy ${GLPK_BIN_DIR}/zlib1.dll ${TARGET_PATH}
    )
  ENDIF(WIN32 AND HAVE_GLPK)
ENDIF(HAVE_MIP)

FOREACH(TEST_NAME ${TESTS})
  ADD_EXECUTABLE(${TEST_NAME} ${TEST_NAME}.cc)
  TARGET_LINK_LIBRARIES(${TEST_NAME} lemon)
  ADD_TEST(${TEST_NAME} ${TEST_NAME})
ENDFOREACH(TEST_NAME)
