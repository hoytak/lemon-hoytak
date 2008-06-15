/* -*- C++ -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library
 *
 * Copyright (C) 2003-2008
 * Egervary Jeno Kombinatorikus Optimalizalasi Kutatocsoport
 * (Egervary Research Group on Combinatorial Optimization, EGRES).
 *
 * Permission to use, modify and distribute this software is granted
 * provided that this copyright notice appears in all copies. For
 * precise terms see the accompanying LICENSE file.
 *
 * This software is provided "AS IS" with no warranty of any kind,
 * express or implied, and with no claim as to its suitability for any
 * purpose.
 *
 */

#include <lemon/concepts/digraph.h>
#include <lemon/list_graph.h>
#include <lemon/smart_graph.h>
//#include <lemon/full_graph.h>
//#include <lemon/hypercube_graph.h>

#include "test_tools.h"
#include "graph_test.h"
#include "graph_maps_test.h"

using namespace lemon;
using namespace lemon::concepts;

void check_concepts() {
  { // Checking digraph components
    checkConcept<BaseDigraphComponent, BaseDigraphComponent >();

    checkConcept<IDableDigraphComponent<>, 
      IDableDigraphComponent<> >();

    checkConcept<IterableDigraphComponent<>, 
      IterableDigraphComponent<> >();

    checkConcept<MappableDigraphComponent<>, 
      MappableDigraphComponent<> >();
  }
  { // Checking skeleton digraph
    checkConcept<Digraph, Digraph>();
  }
  { // Checking ListDigraph
    checkConcept<Digraph, ListDigraph>();
    checkConcept<AlterableDigraphComponent<>, ListDigraph>();
    checkConcept<ExtendableDigraphComponent<>, ListDigraph>();
    checkConcept<ClearableDigraphComponent<>, ListDigraph>();
    checkConcept<ErasableDigraphComponent<>, ListDigraph>();
    checkDigraphIterators<ListDigraph>();
  }
  { // Checking SmartDigraph
    checkConcept<Digraph, SmartDigraph>();
    checkConcept<AlterableDigraphComponent<>, SmartDigraph>();
    checkConcept<ExtendableDigraphComponent<>, SmartDigraph>();
    checkConcept<ClearableDigraphComponent<>, SmartDigraph>();
    checkDigraphIterators<SmartDigraph>();
  }
//  { // Checking FullDigraph
//    checkConcept<Digraph, FullDigraph>();
//    checkDigraphIterators<FullDigraph>();
//  }
//  { // Checking HyperCubeDigraph
//    checkConcept<Digraph, HyperCubeDigraph>();
//    checkDigraphIterators<HyperCubeDigraph>();
//  }
}

template <typename Digraph>
void check_graph_validity() {
  TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
  Digraph g;

  Node
    n1 = g.addNode(),
    n2 = g.addNode(),
    n3 = g.addNode();

  Arc
    e1 = g.addArc(n1, n2),
    e2 = g.addArc(n2, n3);

  check(g.valid(n1), "Wrong validity check");
  check(g.valid(e1), "Wrong validity check");

  check(!g.valid(g.nodeFromId(-1)), "Wrong validity check");
  check(!g.valid(g.arcFromId(-1)), "Wrong validity check");
}

template <typename Digraph>
void check_graph_validity_erase() {
  TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
  Digraph g;

  Node
    n1 = g.addNode(),
    n2 = g.addNode(),
    n3 = g.addNode();

  Arc
    e1 = g.addArc(n1, n2),
    e2 = g.addArc(n2, n3);

  check(g.valid(n1), "Wrong validity check");
  check(g.valid(e1), "Wrong validity check");

  g.erase(n1);

  check(!g.valid(n1), "Wrong validity check");
  check(g.valid(n2), "Wrong validity check");
  check(g.valid(n3), "Wrong validity check");
  check(!g.valid(e1), "Wrong validity check");
  check(g.valid(e2), "Wrong validity check");

  check(!g.valid(g.nodeFromId(-1)), "Wrong validity check");
  check(!g.valid(g.arcFromId(-1)), "Wrong validity check");
}

void check_digraphs() {
  { // Checking ListDigraph
    checkDigraph<ListDigraph>();
    checkGraphNodeMap<ListDigraph>();
    checkGraphArcMap<ListDigraph>();

    check_graph_validity_erase<ListDigraph>();
  }
  { // Checking SmartDigraph
    checkDigraph<SmartDigraph>();
    checkGraphNodeMap<SmartDigraph>();
    checkGraphArcMap<SmartDigraph>();

    check_graph_validity<SmartDigraph>();
  }
}

int main() {
  check_concepts();
  check_digraphs();
  return 0;
}
