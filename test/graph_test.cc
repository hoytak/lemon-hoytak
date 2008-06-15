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

#include <lemon/concepts/graph.h>
#include <lemon/list_graph.h>
#include <lemon/smart_graph.h>
// #include <lemon/full_graph.h>
// #include <lemon/grid_graph.h>

#include "test_tools.h"
#include "graph_test.h"
#include "graph_maps_test.h"

using namespace lemon;
using namespace lemon::concepts;

void check_concepts() {
  { // Checking graph components
    checkConcept<BaseGraphComponent, BaseGraphComponent >();

    checkConcept<IDableGraphComponent<>, 
      IDableGraphComponent<> >();

    checkConcept<IterableGraphComponent<>, 
      IterableGraphComponent<> >();

    checkConcept<MappableGraphComponent<>, 
      MappableGraphComponent<> >();
  }
  { // Checking skeleton graph
    checkConcept<Graph, Graph>();
  }
  { // Checking ListGraph
    checkConcept<Graph, ListGraph>();
    checkConcept<AlterableGraphComponent<>, ListGraph>();
    checkConcept<ExtendableGraphComponent<>, ListGraph>();
    checkConcept<ClearableGraphComponent<>, ListGraph>();
    checkConcept<ErasableGraphComponent<>, ListGraph>();
    checkGraphIterators<ListGraph>();
  }
  { // Checking SmartGraph
    checkConcept<Graph, SmartGraph>();
    checkConcept<AlterableGraphComponent<>, SmartGraph>();
    checkConcept<ExtendableGraphComponent<>, SmartGraph>();
    checkConcept<ClearableGraphComponent<>, SmartGraph>();
    checkGraphIterators<SmartGraph>();
  }
//  { // Checking FullGraph
//    checkConcept<Graph, FullGraph>();
//    checkGraphIterators<FullGraph>();
//  }
//  { // Checking GridGraph
//    checkConcept<Graph, GridGraph>();
//    checkGraphIterators<GridGraph>();
//  }
}

template <typename Graph>
void check_graph_validity() {
  TEMPLATE_GRAPH_TYPEDEFS(Graph);
  Graph g;

  Node
    n1 = g.addNode(),
    n2 = g.addNode(),
    n3 = g.addNode();

  Edge
    e1 = g.addEdge(n1, n2),
    e2 = g.addEdge(n2, n3);

  check(g.valid(n1), "Wrong validity check");
  check(g.valid(e1), "Wrong validity check");
  check(g.valid(g.direct(e1, true)), "Wrong validity check");

  check(!g.valid(g.nodeFromId(-1)), "Wrong validity check");
  check(!g.valid(g.edgeFromId(-1)), "Wrong validity check");
  check(!g.valid(g.arcFromId(-1)), "Wrong validity check");
}

template <typename Graph>
void check_graph_validity_erase() {
  TEMPLATE_GRAPH_TYPEDEFS(Graph);
  Graph g;

  Node
    n1 = g.addNode(),
    n2 = g.addNode(),
    n3 = g.addNode();

  Edge
    e1 = g.addEdge(n1, n2),
    e2 = g.addEdge(n2, n3);

  check(g.valid(n1), "Wrong validity check");
  check(g.valid(e1), "Wrong validity check");
  check(g.valid(g.direct(e1, true)), "Wrong validity check");

  g.erase(n1);

  check(!g.valid(n1), "Wrong validity check");
  check(g.valid(n2), "Wrong validity check");
  check(g.valid(n3), "Wrong validity check");
  check(!g.valid(e1), "Wrong validity check");
  check(g.valid(e2), "Wrong validity check");

  check(!g.valid(g.nodeFromId(-1)), "Wrong validity check");
  check(!g.valid(g.edgeFromId(-1)), "Wrong validity check");
  check(!g.valid(g.arcFromId(-1)), "Wrong validity check");
}

// void checkGridGraph(const GridGraph& g, int w, int h) {
//   check(g.width() == w, "Wrong width");
//   check(g.height() == h, "Wrong height");

//   for (int i = 0; i < w; ++i) {
//     for (int j = 0; j < h; ++j) {
//       check(g.col(g(i, j)) == i, "Wrong col");
//       check(g.row(g(i, j)) == j, "Wrong row");
//     }
//   }
  
//   for (int i = 0; i < w; ++i) {
//     for (int j = 0; j < h - 1; ++j) {
//       check(g.source(g.down(g(i, j))) == g(i, j), "Wrong down");
//       check(g.target(g.down(g(i, j))) == g(i, j + 1), "Wrong down");
//     }
//     check(g.down(g(i, h - 1)) == INVALID, "Wrong down");
//   }

//   for (int i = 0; i < w; ++i) {
//     for (int j = 1; j < h; ++j) {
//       check(g.source(g.up(g(i, j))) == g(i, j), "Wrong up");
//       check(g.target(g.up(g(i, j))) == g(i, j - 1), "Wrong up");
//     }
//     check(g.up(g(i, 0)) == INVALID, "Wrong up");
//   }

//   for (int j = 0; j < h; ++j) {
//     for (int i = 0; i < w - 1; ++i) {
//       check(g.source(g.right(g(i, j))) == g(i, j), "Wrong right");
//       check(g.target(g.right(g(i, j))) == g(i + 1, j), "Wrong right");      
//     }
//     check(g.right(g(w - 1, j)) == INVALID, "Wrong right");    
//   }

//   for (int j = 0; j < h; ++j) {
//     for (int i = 1; i < w; ++i) {
//       check(g.source(g.left(g(i, j))) == g(i, j), "Wrong left");
//       check(g.target(g.left(g(i, j))) == g(i - 1, j), "Wrong left");      
//     }
//     check(g.left(g(0, j)) == INVALID, "Wrong left");    
//   }
// }

void check_graphs() {
  { // Checking ListGraph
    checkGraph<ListGraph>();
    checkGraphNodeMap<ListGraph>();
    checkGraphEdgeMap<ListGraph>();

    check_graph_validity_erase<ListGraph>();
  }
  { // Checking SmartGraph
    checkGraph<SmartGraph>();
    checkGraphNodeMap<SmartGraph>();
    checkGraphEdgeMap<SmartGraph>();

    check_graph_validity<SmartGraph>();
  }
//   { // Checking FullGraph
//     FullGraph g(5);
//     checkGraphNodeList(g, 5);
//     checkGraphEdgeList(g, 10);
//   }
//   { // Checking GridGraph
//     GridGraph g(5, 6);
//     checkGraphNodeList(g, 30);
//     checkGraphEdgeList(g, 49);
//     checkGridGraph(g, 5, 6);
//   }
}

int main() {
  check_concepts();
  check_graphs();
  return 0;
}
