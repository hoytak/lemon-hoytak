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
// #include <lemon/grid_graph.h>

#include "test_tools.h"
#include "graph_test.h"

using namespace lemon;
using namespace lemon::concepts;

template <class Graph>
void checkGraphBuild() {
  TEMPLATE_GRAPH_TYPEDEFS(Graph);

  Graph G;
  checkGraphNodeList(G, 0);
  checkGraphEdgeList(G, 0);
  checkGraphArcList(G, 0);

  Node
    n1 = G.addNode(),
    n2 = G.addNode(),
    n3 = G.addNode();
  checkGraphNodeList(G, 3);
  checkGraphEdgeList(G, 0);
  checkGraphArcList(G, 0);

  Edge e1 = G.addEdge(n1, n2);
  check((G.u(e1) == n1 && G.v(e1) == n2) || (G.u(e1) == n2 && G.v(e1) == n1),
        "Wrong edge");

  checkGraphNodeList(G, 3);
  checkGraphEdgeList(G, 1);
  checkGraphArcList(G, 2);

  checkGraphIncEdgeArcLists(G, n1, 1);
  checkGraphIncEdgeArcLists(G, n2, 1);
  checkGraphIncEdgeArcLists(G, n3, 0);

  checkGraphConEdgeList(G, 1);
  checkGraphConArcList(G, 2);

  Edge e2 = G.addEdge(n2, n1),
       e3 = G.addEdge(n2, n3);

  checkGraphNodeList(G, 3);
  checkGraphEdgeList(G, 3);
  checkGraphArcList(G, 6);

  checkGraphIncEdgeArcLists(G, n1, 2);
  checkGraphIncEdgeArcLists(G, n2, 3);
  checkGraphIncEdgeArcLists(G, n3, 1);

  checkGraphConEdgeList(G, 3);
  checkGraphConArcList(G, 6);

  checkArcDirections(G);

  checkNodeIds(G);
  checkArcIds(G);
  checkEdgeIds(G);
  checkGraphNodeMap(G);
  checkGraphArcMap(G);
  checkGraphEdgeMap(G);
}

template <class Graph>
void checkGraphAlter() {
  TEMPLATE_GRAPH_TYPEDEFS(Graph);

  Graph G;
  Node n1 = G.addNode(), n2 = G.addNode(),
       n3 = G.addNode(), n4 = G.addNode();
  Edge e1 = G.addEdge(n1, n2), e2 = G.addEdge(n2, n1),
       e3 = G.addEdge(n2, n3), e4 = G.addEdge(n1, n4),
       e5 = G.addEdge(n4, n3);

  checkGraphNodeList(G, 4);
  checkGraphEdgeList(G, 5);
  checkGraphArcList(G, 10);

  // Check changeU() and changeV()
  if (G.u(e2) == n2) {
    G.changeU(e2, n3);
  } else {
    G.changeV(e2, n3);
  }

  checkGraphNodeList(G, 4);
  checkGraphEdgeList(G, 5);
  checkGraphArcList(G, 10);

  checkGraphIncEdgeArcLists(G, n1, 3);
  checkGraphIncEdgeArcLists(G, n2, 2);
  checkGraphIncEdgeArcLists(G, n3, 3);
  checkGraphIncEdgeArcLists(G, n4, 2);

  checkGraphConEdgeList(G, 5);
  checkGraphConArcList(G, 10);

  if (G.u(e2) == n1) {
    G.changeU(e2, n2);
  } else {
    G.changeV(e2, n2);
  }

  checkGraphNodeList(G, 4);
  checkGraphEdgeList(G, 5);
  checkGraphArcList(G, 10);

  checkGraphIncEdgeArcLists(G, n1, 2);
  checkGraphIncEdgeArcLists(G, n2, 3);
  checkGraphIncEdgeArcLists(G, n3, 3);
  checkGraphIncEdgeArcLists(G, n4, 2);

  checkGraphConEdgeList(G, 5);
  checkGraphConArcList(G, 10);

  // Check contract()
  G.contract(n1, n4, false);

  checkGraphNodeList(G, 3);
  checkGraphEdgeList(G, 5);
  checkGraphArcList(G, 10);

  checkGraphIncEdgeArcLists(G, n1, 4);
  checkGraphIncEdgeArcLists(G, n2, 3);
  checkGraphIncEdgeArcLists(G, n3, 3);

  checkGraphConEdgeList(G, 5);
  checkGraphConArcList(G, 10);

  G.contract(n2, n3);

  checkGraphNodeList(G, 2);
  checkGraphEdgeList(G, 3);
  checkGraphArcList(G, 6);

  checkGraphIncEdgeArcLists(G, n1, 4);
  checkGraphIncEdgeArcLists(G, n2, 2);

  checkGraphConEdgeList(G, 3);
  checkGraphConArcList(G, 6);
}

template <class Graph>
void checkGraphErase() {
  TEMPLATE_GRAPH_TYPEDEFS(Graph);

  Graph G;
  Node n1 = G.addNode(), n2 = G.addNode(),
       n3 = G.addNode(), n4 = G.addNode();
  Edge e1 = G.addEdge(n1, n2), e2 = G.addEdge(n2, n1),
       e3 = G.addEdge(n2, n3), e4 = G.addEdge(n1, n4),
       e5 = G.addEdge(n4, n3);

  // Check edge deletion
  G.erase(e2);

  checkGraphNodeList(G, 4);
  checkGraphEdgeList(G, 4);
  checkGraphArcList(G, 8);

  checkGraphIncEdgeArcLists(G, n1, 2);
  checkGraphIncEdgeArcLists(G, n2, 2);
  checkGraphIncEdgeArcLists(G, n3, 2);
  checkGraphIncEdgeArcLists(G, n4, 2);

  checkGraphConEdgeList(G, 4);
  checkGraphConArcList(G, 8);

  // Check node deletion
  G.erase(n3);

  checkGraphNodeList(G, 3);
  checkGraphEdgeList(G, 2);
  checkGraphArcList(G, 4);

  checkGraphIncEdgeArcLists(G, n1, 2);
  checkGraphIncEdgeArcLists(G, n2, 1);
  checkGraphIncEdgeArcLists(G, n4, 1);

  checkGraphConEdgeList(G, 2);
  checkGraphConArcList(G, 4);
}


template <class Graph>
void checkGraphSnapshot() {
  TEMPLATE_GRAPH_TYPEDEFS(Graph);

  Graph G;
  Node n1 = G.addNode(), n2 = G.addNode(), n3 = G.addNode();
  Edge e1 = G.addEdge(n1, n2), e2 = G.addEdge(n2, n1),
       e3 = G.addEdge(n2, n3);

  checkGraphNodeList(G, 3);
  checkGraphEdgeList(G, 3);
  checkGraphArcList(G, 6);

  typename Graph::Snapshot snapshot(G);

  Node n = G.addNode();
  G.addEdge(n3, n);
  G.addEdge(n, n3);
  G.addEdge(n3, n2);

  checkGraphNodeList(G, 4);
  checkGraphEdgeList(G, 6);
  checkGraphArcList(G, 12);

  snapshot.restore();

  checkGraphNodeList(G, 3);
  checkGraphEdgeList(G, 3);
  checkGraphArcList(G, 6);

  checkGraphIncEdgeArcLists(G, n1, 2);
  checkGraphIncEdgeArcLists(G, n2, 3);
  checkGraphIncEdgeArcLists(G, n3, 1);

  checkGraphConEdgeList(G, 3);
  checkGraphConArcList(G, 6);

  checkNodeIds(G);
  checkEdgeIds(G);
  checkArcIds(G);
  checkGraphNodeMap(G);
  checkGraphEdgeMap(G);
  checkGraphArcMap(G);

  G.addNode();
  snapshot.save(G);

  G.addEdge(G.addNode(), G.addNode());

  snapshot.restore();

  checkGraphNodeList(G, 4);
  checkGraphEdgeList(G, 3);
  checkGraphArcList(G, 6);
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
//    checkGraphIterators<FullGraph>();
//  }
//  { // Checking GridGraph
//    checkConcept<Graph, GridGraph>();
//    checkGraphIterators<GridGraph>();
//  }
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

void checkGraphs() {
  { // Checking ListGraph
    checkGraphBuild<ListGraph>();
    checkGraphAlter<ListGraph>();
    checkGraphErase<ListGraph>();
    checkGraphSnapshot<ListGraph>();
    checkGraphValidityErase<ListGraph>();
  }
  { // Checking SmartGraph
    checkGraphBuild<SmartGraph>();
    checkGraphSnapshot<SmartGraph>();
    checkGraphValidity<SmartGraph>();
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
  checkConcepts();
  checkGraphs();
  return 0;
}
