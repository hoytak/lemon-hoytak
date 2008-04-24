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

#include <lemon/graph_utils.h>

#include "test_tools.h"


using namespace lemon;
using namespace lemon::concepts;

void check_concepts() {

  { // checking digraph components
    checkConcept<BaseGraphComponent, BaseGraphComponent >();

    checkConcept<IDableGraphComponent<>, 
      IDableGraphComponent<> >();

    checkConcept<IterableGraphComponent<>, 
      IterableGraphComponent<> >();

    checkConcept<MappableGraphComponent<>, 
      MappableGraphComponent<> >();

  }
  {
    checkConcept<Graph, ListGraph>();    
    checkConcept<Graph, SmartGraph>();    
//     checkConcept<Graph, FullGraph>();    
//     checkConcept<Graph, Graph>();    
//     checkConcept<Graph, GridGraph>();
  }
}

template <typename Graph>
void check_item_counts(Graph &g, int n, int e) {
  int nn = 0;
  for (typename Graph::NodeIt it(g); it != INVALID; ++it) {
    ++nn;
  }

  check(nn == n, "Wrong node number.");
  //  check(countNodes(g) == n, "Wrong node number.");

  int ee = 0;
  for (typename Graph::ArcIt it(g); it != INVALID; ++it) {
    ++ee;
  }

  check(ee == 2*e, "Wrong arc number.");
  //  check(countArcs(g) == 2*e, "Wrong arc number.");

  int uee = 0;
  for (typename Graph::EdgeIt it(g); it != INVALID; ++it) {
    ++uee;
  }

  check(uee == e, "Wrong edge number.");
  //  check(countEdges(g) == e, "Wrong edge number.");
}

template <typename Graph>
void check_graph_counts() {

  TEMPLATE_GRAPH_TYPEDEFS(Graph);
  Graph g;

  check_item_counts(g,0,0);

  Node
    n1 = g.addNode(),
    n2 = g.addNode(),
    n3 = g.addNode();

  Edge
    e1 = g.addEdge(n1, n2),
    e2 = g.addEdge(n2, n3);

  check_item_counts(g,3,2);
}

template <typename Graph>
void check_graph_validity() {

  TEMPLATE_GRAPH_TYPEDEFS(Graph);
  Graph g;

  check_item_counts(g,0,0);

  Node
    n1 = g.addNode(),
    n2 = g.addNode(),
    n3 = g.addNode();

  Edge
    e1 = g.addEdge(n1, n2),
    e2 = g.addEdge(n2, n3);

  check(g.valid(n1), "Validity check");
  check(g.valid(e1), "Validity check");
  check(g.valid(g.direct(e1, true)), "Validity check");

  check(!g.valid(g.nodeFromId(-1)), "Validity check");
  check(!g.valid(g.edgeFromId(-1)), "Validity check");
  check(!g.valid(g.arcFromId(-1)), "Validity check");
    
}

template <typename Graph>
void check_graph_validity_erase() {

  TEMPLATE_GRAPH_TYPEDEFS(Graph);
  Graph g;

  check_item_counts(g,0,0);

  Node
    n1 = g.addNode(),
    n2 = g.addNode(),
    n3 = g.addNode();

  Edge
    e1 = g.addEdge(n1, n2),
    e2 = g.addEdge(n2, n3);

  check(g.valid(n1), "Validity check");
  check(g.valid(e1), "Validity check");
  check(g.valid(g.direct(e1, true)), "Validity check");

  g.erase(n1);

  check(!g.valid(n1), "Validity check");
  check(g.valid(n2), "Validity check");
  check(g.valid(n3), "Validity check");
  check(!g.valid(e1), "Validity check");
  check(g.valid(e2), "Validity check");

  check(!g.valid(g.nodeFromId(-1)), "Validity check");
  check(!g.valid(g.edgeFromId(-1)), "Validity check");
  check(!g.valid(g.arcFromId(-1)), "Validity check");
    
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

int main() {
  check_concepts();

  check_graph_counts<ListGraph>();
  check_graph_counts<SmartGraph>();

  check_graph_validity_erase<ListGraph>();
  check_graph_validity<SmartGraph>();

//   {
//     FullGraph g(5);
//     check_item_counts(g, 5, 10);
//   }

//   {
//     GridGraph g(5, 6);
//     check_item_counts(g, 30, 49);
//     checkGridGraph(g, 5, 6);
//   }

  std::cout << __FILE__ ": All tests passed.\n";

  return 0;
}
