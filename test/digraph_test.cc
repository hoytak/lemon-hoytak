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

#include <lemon/concepts/digraph.h>
#include <lemon/list_graph.h>
#include <lemon/smart_graph.h>
#include <lemon/full_graph.h>

#include "test_tools.h"
#include "graph_test.h"

using namespace lemon;
using namespace lemon::concepts;

template <class Digraph>
void checkDigraph() {
  TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
  Digraph G;

  checkGraphNodeList(G, 0);
  checkGraphArcList(G, 0);

  Node
    n1 = G.addNode(),
    n2 = G.addNode(),
    n3 = G.addNode();
  checkGraphNodeList(G, 3);
  checkGraphArcList(G, 0);

  Arc a1 = G.addArc(n1, n2);
  check(G.source(a1) == n1 && G.target(a1) == n2, "Wrong arc");
  checkGraphNodeList(G, 3);
  checkGraphArcList(G, 1);

  checkGraphOutArcList(G, n1, 1);
  checkGraphOutArcList(G, n2, 0);
  checkGraphOutArcList(G, n3, 0);

  checkGraphInArcList(G, n1, 0);
  checkGraphInArcList(G, n2, 1);
  checkGraphInArcList(G, n3, 0);

  checkGraphConArcList(G, 1);

  Arc a2 = G.addArc(n2, n1), a3 = G.addArc(n2, n3), a4 = G.addArc(n2, n3);
  checkGraphNodeList(G, 3);
  checkGraphArcList(G, 4);

  checkGraphOutArcList(G, n1, 1);
  checkGraphOutArcList(G, n2, 3);
  checkGraphOutArcList(G, n3, 0);

  checkGraphInArcList(G, n1, 1);
  checkGraphInArcList(G, n2, 1);
  checkGraphInArcList(G, n3, 2);

  checkGraphConArcList(G, 4);

  checkNodeIds(G);
  checkArcIds(G);
  checkGraphNodeMap(G);
  checkGraphArcMap(G);

}

void checkFullDigraph(int num) {
  typedef FullDigraph Digraph;
  DIGRAPH_TYPEDEFS(Digraph);
  Digraph G(num);

  checkGraphNodeList(G, num);
  checkGraphArcList(G, num * num);

  for (NodeIt n(G); n != INVALID; ++n) {
    checkGraphOutArcList(G, n, num);
    checkGraphInArcList(G, n, num);
  }

  checkGraphConArcList(G, num * num);

  checkNodeIds(G);
  checkArcIds(G);
  checkGraphNodeMap(G);
  checkGraphArcMap(G);

  for (int i = 0; i < G.nodeNum(); ++i) {
    check(G.index(G(i)) == i, "Wrong index");
  }

  for (NodeIt s(G); s != INVALID; ++s) {
    for (NodeIt t(G); t != INVALID; ++t) {
      Arc a = G.arc(s, t);
      check(G.source(a) == s && G.target(a) == t, "Wrong arc lookup");
    }
  }

}

void checkConcepts() {
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
  }
  { // Checking SmartDigraph
    checkConcept<Digraph, SmartDigraph>();
    checkConcept<AlterableDigraphComponent<>, SmartDigraph>();
    checkConcept<ExtendableDigraphComponent<>, SmartDigraph>();
    checkConcept<ClearableDigraphComponent<>, SmartDigraph>();
  }
  { // Checking FullDigraph
    checkConcept<Digraph, FullDigraph>();
  }
}

template <typename Digraph>
void checkDigraphValidity() {
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
void checkDigraphValidityErase() {
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

void checkDigraphs() {
  { // Checking ListDigraph
    checkDigraph<ListDigraph>();
    checkDigraphValidityErase<ListDigraph>();
  }
  { // Checking SmartDigraph
    checkDigraph<SmartDigraph>();
    checkDigraphValidity<SmartDigraph>();
  }
  { // Checking FullDigraph
    checkFullDigraph(8);
  }
}

int main() {
  checkDigraphs();
  checkConcepts();
  return 0;
}
