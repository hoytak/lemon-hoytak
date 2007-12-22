EXTRA_DIST += \
	test/Makefile

noinst_HEADERS += \
        test/test_tools.h
 
check_PROGRAMS += \
        test/maps_test \
        test/test_tools_fail \
        test/test_tools_pass
 
TESTS += $(check_PROGRAMS)
XFAIL_TESTS += test/test_tools_fail$(EXEEXT)

test_maps_test_SOURCES = test/maps_test.cc
test_test_tools_fail_SOURCES = test/test_tools_fail.cc
test_test_tools_pass_SOURCES = test/test_tools_pass.cc
