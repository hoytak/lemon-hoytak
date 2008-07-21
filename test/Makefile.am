EXTRA_DIST += \
	test/CMakeLists.txt

noinst_HEADERS += \
	test/graph_test.h \
        test/test_tools.h

check_PROGRAMS += \
	test/bfs_test \
        test/counter_test \
	test/dfs_test \
	test/digraph_test \
	test/dijkstra_test \
        test/dim_test \
	test/error_test \
	test/graph_copy_test \
	test/graph_test \
	test/graph_utils_test \
	test/heap_test \
	test/kruskal_test \
        test/maps_test \
        test/random_test \
        test/path_test \
        test/test_tools_fail \
        test/test_tools_pass \
        test/time_measure_test \
	test/unionfind_test

TESTS += $(check_PROGRAMS)
XFAIL_TESTS += test/test_tools_fail$(EXEEXT)

test_bfs_test_SOURCES = test/bfs_test.cc
test_counter_test_SOURCES = test/counter_test.cc
test_dfs_test_SOURCES = test/dfs_test.cc
test_digraph_test_SOURCES = test/digraph_test.cc
test_dijkstra_test_SOURCES = test/dijkstra_test.cc
test_dim_test_SOURCES = test/dim_test.cc
test_error_test_SOURCES = test/error_test.cc
test_graph_copy_test_SOURCES = test/graph_copy_test.cc
test_graph_test_SOURCES = test/graph_test.cc
test_graph_utils_test_SOURCES = test/graph_utils_test.cc
test_heap_test_SOURCES = test/heap_test.cc
test_kruskal_test_SOURCES = test/kruskal_test.cc
test_maps_test_SOURCES = test/maps_test.cc
test_path_test_SOURCES = test/path_test.cc
test_random_test_SOURCES = test/random_test.cc
test_test_tools_fail_SOURCES = test/test_tools_fail.cc
test_test_tools_pass_SOURCES = test/test_tools_pass.cc
test_time_measure_test_SOURCES = test/time_measure_test.cc
test_unionfind_test_SOURCES = test/unionfind_test.cc
