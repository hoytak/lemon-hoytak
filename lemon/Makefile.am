EXTRA_DIST += \
	lemon/Makefile \
	lemon/lemon.pc.in

pkgconfig_DATA += lemon/lemon.pc

lib_LTLIBRARIES += lemon/libemon.la

lemon_libemon_la_SOURCES = \
        lemon/base.cc

lemon_libemon_la_CXXFLAGS = $(GLPK_CFLAGS) $(CPLEX_CFLAGS) $(SOPLEX_CXXFLAGS)
lemon_libemon_la_LDFLAGS = $(GLPK_LIBS) $(CPLEX_LIBS) $(SOPLEX_LIBS)

lemon_HEADERS += \
        lemon/concept_check.h \
	lemon/list_graph.h \
        lemon/maps.h \
        lemon/tolerance.h

bits_HEADERS += \
        lemon/bits/invalid.h \
        lemon/bits/utility.h

concept_HEADERS += \
        lemon/concepts/maps.h
