EXTRA_DIST += \
	lemon/Makefile \
	lemon/lemon.pc.in

pkgconfig_DATA += lemon/lemon.pc

lib_LTLIBRARIES += lemon/libemon.la

lemon_libemon_la_SOURCES = \
        lemon/base.cc \
        lemon/random.cc


lemon_libemon_la_CXXFLAGS = $(GLPK_CFLAGS) $(CPLEX_CFLAGS) $(SOPLEX_CXXFLAGS)
lemon_libemon_la_LDFLAGS = $(GLPK_LIBS) $(CPLEX_LIBS) $(SOPLEX_LIBS)

lemon_HEADERS += \
        lemon/concept_check.h \
        lemon/dim2.h \
	lemon/error.h \
	lemon/list_graph.h \
	lemon/maps.h \
	lemon/path.h \
        lemon/random.h \
        lemon/tolerance.h

bits_HEADERS += \
	lemon/bits/alteration_notifier.h \
	lemon/bits/array_map.h \
	lemon/bits/base_extender.h \
	lemon/bits/default_map.h \
	lemon/bits/graph_extender.h \
        lemon/bits/invalid.h \
	lemon/bits/map_extender.h \
	lemon/bits/traits.h \
        lemon/bits/utility.h \
	lemon/bits/vector_map.h

concept_HEADERS += \
	lemon/concept_check.h \
	lemon/concepts/digraph.h \
	lemon/concepts/graph.h \
	lemon/concepts/maps.h \
	lemon/concepts/path.h \
	lemon/concepts/graph_components.h
