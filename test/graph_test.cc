/* -*- mode: C++; indent-tabs-mode: nil; -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library.
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
#include <lemon/grid_graph.h>

#include "test_tools.h"
#include "graph_test.h"

using namespace lemon;
using namespace lemon::concepts;

template <class Graph>
void checkGraph() {
  TEMPLATE_GRAPH_TYPEDEFS(Graph);

  Graph G;
  checkGraphNodeList(G, 0);
  checkGraphEdgeList(G, 0);

  Node
    n1 = G.addNode(),
    n2 = G.addNode(),
    n3 = G.addNode();
  checkGraphNodeList(G, 3);
  checkGraphEdgeList(G, 0);

  Edge e1 = G.addEdge(n1, n2);
  check((G.u(e1) == n1 && G.v(e1) == n2) || (G.u(e1) == n2 && G.v(e1) == n1),
        "Wrong edge");
  checkGraphNodeList(G, 3);
  checkGraphArcList(G, 2);
  checkGraphEdgeList(G, 1);

  checkGraphOutArcList(G, n1, 1);
  checkGraphOutArcList(G, n2, 1);
  checkGraphOutArcList(G, n3, 0);

  checkGraphInArcList(G, n1, 1);
  checkGraphInArcList(G, n2, 1);
  checkGraphInArcList(G, n3, 0);

  checkGraphIncEdgeList(G, n1, 1);
  checkGraphIncEdgeList(G, n2, 1);
  checkGraphIncEdgeList(G, n3, 0);

  checkGraphConArcList(G, 2);
  checkGraphConEdgeList(G, 1);

  Edge e2 = G.addEdge(n2, n1), e3 = G.addEdge(n2, n3);
  checkGraphNodeList(G, 3);
  checkGraphArcList(G, 6);
  checkGraphEdgeList(G, 3);

  checkGraphOutArcList(G, n1, 2);
  checkGraphOutArcList(G, n2, 3);
  checkGraphOutArcList(G, n3, 1);

  checkGraphInArcList(G, n1, 2);
  checkGraphInArcList(G, n2, 3);
  checkGraphInArcList(G, n3, 1);

  checkGraphIncEdgeList(G, n1, 2);
  checkGraphIncEdgeList(G, n2, 3);
  checkGraphIncEdgeList(G, n3, 1);

  checkGraphConArcList(G, 6);
  checkGraphConEdgeList(G, 3);

  checkArcDirections(G);

  checkNodeIds(G);
  checkArcIds(G);
  checkEdgeIds(G);
  checkGraphNodeMap(G);
  checkGraphArcMap(G);
  checkGraphEdgeMap(G);
}

void checkConcepts() {
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
  }
  { // Checking SmartGraph
    checkConcept<Graph, SmartGraph>();
    checkConcept<AlterableGraphComponent<>, SmartGraph>();
    checkConcept<ExtendableGraphComponent<>, SmartGraph>();
    checkConcept<ClearableGraphComponent<>, SmartGraph>();
  }
//  { // Checking FullGraph
//    checkConcept<Graph, FullGraph>();
//  }
  { // Checking GridGraph
    checkConcept<Graph, GridGraph>();
  }
}

template <typename Graph>
void checkGraphValidity() {
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
void checkGraphValidityErase() {
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

void checkGridGraph(const GridGraph& g, int w, int h) {
  check(g.width() == w, "Wrong width");
  check(g.height() == h, "Wrong height");

  for (int i = 0; i < w; ++i) {
    for (int j = 0; j < h; ++j) {
      check(g.col(g(i, j)) == i, "Wrong col");
      check(g.row(g(i, j)) == j, "Wrong row");
    }
  }

  for (int i = 0; i < w; ++i) {
    for (int j = 0; j < h - 1; ++j) {
      check(g.source(g.down(g(i, j))) == g(i, j), "Wrong down");
      check(g.target(g.down(g(i, j))) == g(i, j + 1), "Wrong down");
    }
    check(g.down(g(i, h - 1)) == INVALID, "Wrong down");
  }

  for (int i = 0; i < w; ++i) {
    for (int j = 1; j < h; ++j) {
      check(g.source(g.up(g(i, j))) == g(i, j), "Wrong up");
      check(g.target(g.up(g(i, j))) == g(i, j - 1), "Wrong up");
    }
    check(g.up(g(i, 0)) == INVALID, "Wrong up");
  }

  for (int j = 0; j < h; ++j) {
    for (int i = 0; i < w - 1; ++i) {
      check(g.source(g.right(g(i, j))) == g(i, j), "Wrong right");
      check(g.target(g.right(g(i, j))) == g(i + 1, j), "Wrong right");
    }
    check(g.right(g(w - 1, j)) == INVALID, "Wrong right");
  }

  for (int j = 0; j < h; ++j) {
    for (int i = 1; i < w; ++i) {
      check(g.source(g.left(g(i, j))) == g(i, j), "Wrong left");
      check(g.target(g.left(g(i, j))) == g(i - 1, j), "Wrong left");
    }
    check(g.left(g(0, j)) == INVALID, "Wrong left");
  }

  checkGraphNodeList(g, w*h);
  checkGraphArcList(g, 2*(2*w*h-w-h));
  checkGraphEdgeList(g, 2*w*h-w-h);

  checkGraphOutArcList(g, g(0,0), 2);
  checkGraphOutArcList(g, g(0,1), 3);
  checkGraphOutArcList(g, g(w-2,h-2), 4);

  checkGraphInArcList(g, g(0,0), 2);
  checkGraphInArcList(g, g(0,1), 3);
  checkGraphInArcList(g, g(w-2,h-2), 4);

  checkGraphIncEdgeList(g, g(0,0), 2);
  checkGraphIncEdgeList(g, g(0,1), 3);
  checkGraphIncEdgeList(g, g(w-2,h-2), 4);

  checkGraphConArcList(g, 2*(2*w*h-w-h));
  checkGraphConEdgeList(g, 2*w*h-w-h);

  checkArcDirections(g);

  checkNodeIds(g);
  checkArcIds(g);
  checkEdgeIds(g);
  checkGraphNodeMap(g);
  checkGraphArcMap(g);
  checkGraphEdgeMap(g);
}

void checkGraphs() {
  { // Checking ListGraph
    checkGraph<ListGraph>();
    checkGraphValidityErase<ListGraph>();
  }
  { // Checking SmartGraph
    checkGraph<SmartGraph>();
    checkGraphValidity<SmartGraph>();
  }
//   { // Checking FullGraph
//     FullGraph g(5);
//     checkGraphNodeList(g, 5);
//     checkGraphEdgeList(g, 10);
//   }
  { // Checking GridGraph
    GridGraph g(5, 6);
    checkGridGraph(g, 5, 6);
  }
}

int main() {
  checkConcepts();
  checkGraphs();
  return 0;
}
