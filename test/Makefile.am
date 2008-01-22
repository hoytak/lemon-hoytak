EXTRA_DIST += \
	test/Makefile

noinst_HEADERS += \
	test/digraph_test.h \
	test/map_test.h \
        test/test_tools.h

check_PROGRAMS += \
	test/digraph_test \
        test/dim_test \
	test/graph_test \
        test/random_test \
        test/test_tools_fail \
        test/test_tools_pass

TESTS += $(check_PROGRAMS)
XFAIL_TESTS += test/test_tools_fail$(EXEEXT)

test_digraph_test_SOURCES = test/digraph_test.cc
test_dim_test_SOURCES = test/dim_test.cc
test_graph_test_SOURCES = test/graph_test.cc
test_random_test_SOURCES = test/random_test.cc
test_test_tools_fail_SOURCES = test/test_tools_fail.cc
test_test_tools_pass_SOURCES = test/test_tools_pass.cc
