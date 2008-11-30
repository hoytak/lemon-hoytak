EXTRA_DIST += \
	lemon/lemon.pc.in \
	lemon/CMakeLists.txt

pkgconfig_DATA += lemon/lemon.pc

lib_LTLIBRARIES += lemon/libemon.la

lemon_libemon_la_SOURCES = \
        lemon/arg_parser.cc \
        lemon/base.cc \
        lemon/color.cc \
        lemon/random.cc

#lemon_libemon_la_CXXFLAGS = $(GLPK_CFLAGS) $(CPLEX_CFLAGS) $(SOPLEX_CXXFLAGS) $(AM_CXXFLAGS)
#lemon_libemon_la_LDFLAGS = $(GLPK_LIBS) $(CPLEX_LIBS) $(SOPLEX_LIBS)

lemon_HEADERS += \
	lemon/adaptors.h \
        lemon/arg_parser.h \
	lemon/assert.h \
        lemon/bfs.h \
        lemon/bin_heap.h \
        lemon/color.h \
	lemon/concept_check.h \
        lemon/counter.h \
	lemon/core.h \
        lemon/dfs.h \
        lemon/dijkstra.h \
        lemon/dim2.h \
        lemon/dimacs.h \
	lemon/elevator.h \
	lemon/error.h \
	lemon/full_graph.h \
        lemon/graph_to_eps.h \
        lemon/grid_graph.h \
	lemon/hypercube_graph.h \
	lemon/kruskal.h \
	lemon/lgf_reader.h \
	lemon/lgf_writer.h \
	lemon/list_graph.h \
	lemon/maps.h \
	lemon/math.h \
	lemon/max_matching.h \
	lemon/nauty_reader.h \
	lemon/path.h \
	lemon/preflow.h \
        lemon/random.h \
	lemon/smart_graph.h \
	lemon/suurballe.h \
        lemon/time_measure.h \
        lemon/tolerance.h \
	lemon/unionfind.h

bits_HEADERS += \
	lemon/bits/alteration_notifier.h \
	lemon/bits/array_map.h \
	lemon/bits/base_extender.h \
        lemon/bits/bezier.h \
	lemon/bits/default_map.h \
        lemon/bits/enable_if.h \
	lemon/bits/graph_adaptor_extender.h \
	lemon/bits/graph_extender.h \
	lemon/bits/map_extender.h \
	lemon/bits/path_dump.h \
	lemon/bits/traits.h \
	lemon/bits/variant.h \
	lemon/bits/vector_map.h

concept_HEADERS += \
	lemon/concepts/digraph.h \
	lemon/concepts/graph.h \
	lemon/concepts/graph_components.h \
	lemon/concepts/heap.h \
	lemon/concepts/maps.h \
	lemon/concepts/path.h
