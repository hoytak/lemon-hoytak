EXTRA_DIST += \
	lemon/Makefile \
	lemon/lemon.pc.in

pkgconfig_DATA += lemon/lemon.pc

lib_LTLIBRARIES += lemon/libemon.la

lemon_libemon_la_SOURCES =

lemon_libemon_la_CXXFLAGS = $(GLPK_CFLAGS) $(CPLEX_CFLAGS) $(SOPLEX_CXXFLAGS)
lemon_libemon_la_LDFLAGS = $(GLPK_LIBS) $(CPLEX_LIBS) $(SOPLEX_LIBS)

lemon_HEADERS += \
	lemon/list_graph.h

bits_HEADERS +=

concept_HEADERS +=
