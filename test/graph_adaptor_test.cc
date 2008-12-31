/* -*- mode: C++; indent-tabs-mode: nil; -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library.
 *
 * Copyright (C) 2003-2009
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

#include<iostream>
#include<lemon/concept_check.h>

#include<lemon/list_graph.h>
#include<lemon/smart_graph.h>

#include<lemon/concepts/digraph.h>
#include<lemon/concepts/graph.h>

#include<lemon/adaptors.h>

#include <limits>
#include <lemon/bfs.h>
#include <lemon/path.h>

#include"test/test_tools.h"
#include"test/graph_test.h"

using namespace lemon;

void checkReverseDigraph() {
  checkConcept<concepts::Digraph, ReverseDigraph<concepts::Digraph> >();

  typedef ListDigraph Digraph;
  typedef ReverseDigraph<Digraph> Adaptor;

  Digraph digraph;
  Adaptor adaptor(digraph);

  Digraph::Node n1 = digraph.addNode();
  Digraph::Node n2 = digraph.addNode();
  Digraph::Node n3 = digraph.addNode();

  Digraph::Arc a1 = digraph.addArc(n1, n2);
  Digraph::Arc a2 = digraph.addArc(n1, n3);
  Digraph::Arc a3 = digraph.addArc(n2, n3);

  checkGraphNodeList(adaptor, 3);
  checkGraphArcList(adaptor, 3);
  checkGraphConArcList(adaptor, 3);

  checkGraphOutArcList(adaptor, n1, 0);
  checkGraphOutArcList(adaptor, n2, 1);
  checkGraphOutArcList(adaptor, n3, 2);

  checkGraphInArcList(adaptor, n1, 2);
  checkGraphInArcList(adaptor, n2, 1);
  checkGraphInArcList(adaptor, n3, 0);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);

  for (Adaptor::ArcIt a(adaptor); a != INVALID; ++a) {
    check(adaptor.source(a) == digraph.target(a), "Wrong reverse");
    check(adaptor.target(a) == digraph.source(a), "Wrong reverse");
  }
}

void checkSubDigraph() {
  checkConcept<concepts::Digraph,
    SubDigraph<concepts::Digraph,
    concepts::Digraph::NodeMap<bool>,
    concepts::Digraph::ArcMap<bool> > >();

  typedef ListDigraph Digraph;
  typedef Digraph::NodeMap<bool> NodeFilter;
  typedef Digraph::ArcMap<bool> ArcFilter;
  typedef SubDigraph<Digraph, NodeFilter, ArcFilter> Adaptor;

  Digraph digraph;
  NodeFilter node_filter(digraph);
  ArcFilter arc_filter(digraph);
  Adaptor adaptor(digraph, node_filter, arc_filter);

  Digraph::Node n1 = digraph.addNode();
  Digraph::Node n2 = digraph.addNode();
  Digraph::Node n3 = digraph.addNode();

  Digraph::Arc a1 = digraph.addArc(n1, n2);
  Digraph::Arc a2 = digraph.addArc(n1, n3);
  Digraph::Arc a3 = digraph.addArc(n2, n3);

  node_filter[n1] = node_filter[n2] = node_filter[n3] = true;
  arc_filter[a1] = arc_filter[a2] = arc_filter[a3] = true;

  checkGraphNodeList(adaptor, 3);
  checkGraphArcList(adaptor, 3);
  checkGraphConArcList(adaptor, 3);

  checkGraphOutArcList(adaptor, n1, 2);
  checkGraphOutArcList(adaptor, n2, 1);
  checkGraphOutArcList(adaptor, n3, 0);

  checkGraphInArcList(adaptor, n1, 0);
  checkGraphInArcList(adaptor, n2, 1);
  checkGraphInArcList(adaptor, n3, 2);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);

  arc_filter[a2] = false;

  checkGraphNodeList(adaptor, 3);
  checkGraphArcList(adaptor, 2);
  checkGraphConArcList(adaptor, 2);

  checkGraphOutArcList(adaptor, n1, 1);
  checkGraphOutArcList(adaptor, n2, 1);
  checkGraphOutArcList(adaptor, n3, 0);

  checkGraphInArcList(adaptor, n1, 0);
  checkGraphInArcList(adaptor, n2, 1);
  checkGraphInArcList(adaptor, n3, 1);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);

  node_filter[n1] = false;

  checkGraphNodeList(adaptor, 2);
  checkGraphArcList(adaptor, 1);
  checkGraphConArcList(adaptor, 1);

  checkGraphOutArcList(adaptor, n2, 1);
  checkGraphOutArcList(adaptor, n3, 0);

  checkGraphInArcList(adaptor, n2, 0);
  checkGraphInArcList(adaptor, n3, 1);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);

  node_filter[n1] = node_filter[n2] = node_filter[n3] = false;
  arc_filter[a1] = arc_filter[a2] = arc_filter[a3] = false;

  checkGraphNodeList(adaptor, 0);
  checkGraphArcList(adaptor, 0);
  checkGraphConArcList(adaptor, 0);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
}

void checkFilterNodes1() {
  checkConcept<concepts::Digraph,
    FilterNodes<concepts::Digraph,
      concepts::Digraph::NodeMap<bool> > >();

  typedef ListDigraph Digraph;
  typedef Digraph::NodeMap<bool> NodeFilter;
  typedef FilterNodes<Digraph, NodeFilter> Adaptor;

  Digraph digraph;
  NodeFilter node_filter(digraph);
  Adaptor adaptor(digraph, node_filter);

  Digraph::Node n1 = digraph.addNode();
  Digraph::Node n2 = digraph.addNode();
  Digraph::Node n3 = digraph.addNode();

  Digraph::Arc a1 = digraph.addArc(n1, n2);
  Digraph::Arc a2 = digraph.addArc(n1, n3);
  Digraph::Arc a3 = digraph.addArc(n2, n3);

  node_filter[n1] = node_filter[n2] = node_filter[n3] = true;

  checkGraphNodeList(adaptor, 3);
  checkGraphArcList(adaptor, 3);
  checkGraphConArcList(adaptor, 3);

  checkGraphOutArcList(adaptor, n1, 2);
  checkGraphOutArcList(adaptor, n2, 1);
  checkGraphOutArcList(adaptor, n3, 0);

  checkGraphInArcList(adaptor, n1, 0);
  checkGraphInArcList(adaptor, n2, 1);
  checkGraphInArcList(adaptor, n3, 2);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);

  node_filter[n1] = false;

  checkGraphNodeList(adaptor, 2);
  checkGraphArcList(adaptor, 1);
  checkGraphConArcList(adaptor, 1);

  checkGraphOutArcList(adaptor, n2, 1);
  checkGraphOutArcList(adaptor, n3, 0);

  checkGraphInArcList(adaptor, n2, 0);
  checkGraphInArcList(adaptor, n3, 1);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);

  node_filter[n1] = node_filter[n2] = node_filter[n3] = false;

  checkGraphNodeList(adaptor, 0);
  checkGraphArcList(adaptor, 0);
  checkGraphConArcList(adaptor, 0);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
}

void checkFilterArcs() {
  checkConcept<concepts::Digraph,
    FilterArcs<concepts::Digraph,
    concepts::Digraph::ArcMap<bool> > >();

  typedef ListDigraph Digraph;
  typedef Digraph::ArcMap<bool> ArcFilter;
  typedef FilterArcs<Digraph, ArcFilter> Adaptor;

  Digraph digraph;
  ArcFilter arc_filter(digraph);
  Adaptor adaptor(digraph, arc_filter);

  Digraph::Node n1 = digraph.addNode();
  Digraph::Node n2 = digraph.addNode();
  Digraph::Node n3 = digraph.addNode();

  Digraph::Arc a1 = digraph.addArc(n1, n2);
  Digraph::Arc a2 = digraph.addArc(n1, n3);
  Digraph::Arc a3 = digraph.addArc(n2, n3);

  arc_filter[a1] = arc_filter[a2] = arc_filter[a3] = true;

  checkGraphNodeList(adaptor, 3);
  checkGraphArcList(adaptor, 3);
  checkGraphConArcList(adaptor, 3);

  checkGraphOutArcList(adaptor, n1, 2);
  checkGraphOutArcList(adaptor, n2, 1);
  checkGraphOutArcList(adaptor, n3, 0);

  checkGraphInArcList(adaptor, n1, 0);
  checkGraphInArcList(adaptor, n2, 1);
  checkGraphInArcList(adaptor, n3, 2);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);

  arc_filter[a2] = false;

  checkGraphNodeList(adaptor, 3);
  checkGraphArcList(adaptor, 2);
  checkGraphConArcList(adaptor, 2);

  checkGraphOutArcList(adaptor, n1, 1);
  checkGraphOutArcList(adaptor, n2, 1);
  checkGraphOutArcList(adaptor, n3, 0);

  checkGraphInArcList(adaptor, n1, 0);
  checkGraphInArcList(adaptor, n2, 1);
  checkGraphInArcList(adaptor, n3, 1);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);

  arc_filter[a1] = arc_filter[a2] = arc_filter[a3] = false;

  checkGraphNodeList(adaptor, 3);
  checkGraphArcList(adaptor, 0);
  checkGraphConArcList(adaptor, 0);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
}

void checkUndirector() {
  checkConcept<concepts::Graph, Undirector<concepts::Digraph> >();

  typedef ListDigraph Digraph;
  typedef Undirector<Digraph> Adaptor;

  Digraph digraph;
  Adaptor adaptor(digraph);

  Digraph::Node n1 = digraph.addNode();
  Digraph::Node n2 = digraph.addNode();
  Digraph::Node n3 = digraph.addNode();

  Digraph::Arc a1 = digraph.addArc(n1, n2);
  Digraph::Arc a2 = digraph.addArc(n1, n3);
  Digraph::Arc a3 = digraph.addArc(n2, n3);

  checkGraphNodeList(adaptor, 3);
  checkGraphArcList(adaptor, 6);
  checkGraphEdgeList(adaptor, 3);
  checkGraphConArcList(adaptor, 6);
  checkGraphConEdgeList(adaptor, 3);

  checkGraphOutArcList(adaptor, n1, 2);
  checkGraphOutArcList(adaptor, n2, 2);
  checkGraphOutArcList(adaptor, n3, 2);

  checkGraphInArcList(adaptor, n1, 2);
  checkGraphInArcList(adaptor, n2, 2);
  checkGraphInArcList(adaptor, n3, 2);

  checkGraphIncEdgeList(adaptor, n1, 2);
  checkGraphIncEdgeList(adaptor, n2, 2);
  checkGraphIncEdgeList(adaptor, n3, 2);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);
  checkEdgeIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
  checkGraphEdgeMap(adaptor);

  for (Adaptor::EdgeIt e(adaptor); e != INVALID; ++e) {
    check(adaptor.u(e) == digraph.source(e), "Wrong undir");
    check(adaptor.v(e) == digraph.target(e), "Wrong undir");
  }

}

void checkResidual() {
  checkConcept<concepts::Digraph,
    Residual<concepts::Digraph,
    concepts::Digraph::ArcMap<int>,
    concepts::Digraph::ArcMap<int> > >();

  typedef ListDigraph Digraph;
  typedef Digraph::ArcMap<int> IntArcMap;
  typedef Residual<Digraph, IntArcMap> Adaptor;

  Digraph digraph;
  IntArcMap capacity(digraph), flow(digraph);
  Adaptor adaptor(digraph, capacity, flow);

  Digraph::Node n1 = digraph.addNode();
  Digraph::Node n2 = digraph.addNode();
  Digraph::Node n3 = digraph.addNode();
  Digraph::Node n4 = digraph.addNode();

  Digraph::Arc a1 = digraph.addArc(n1, n2);
  Digraph::Arc a2 = digraph.addArc(n1, n3);
  Digraph::Arc a3 = digraph.addArc(n1, n4);
  Digraph::Arc a4 = digraph.addArc(n2, n3);
  Digraph::Arc a5 = digraph.addArc(n2, n4);
  Digraph::Arc a6 = digraph.addArc(n3, n4);

  capacity[a1] = 8;
  capacity[a2] = 6;
  capacity[a3] = 4;
  capacity[a4] = 4;
  capacity[a5] = 6;
  capacity[a6] = 10;

  for (Adaptor::ArcIt a(adaptor); a != INVALID; ++a) {
    flow[a] = 0;
  }

  checkGraphNodeList(adaptor, 4);
  checkGraphArcList(adaptor, 6);
  checkGraphConArcList(adaptor, 6);

  checkGraphOutArcList(adaptor, n1, 3);
  checkGraphOutArcList(adaptor, n2, 2);
  checkGraphOutArcList(adaptor, n3, 1);
  checkGraphOutArcList(adaptor, n4, 0);

  checkGraphInArcList(adaptor, n1, 0);
  checkGraphInArcList(adaptor, n2, 1);
  checkGraphInArcList(adaptor, n3, 2);
  checkGraphInArcList(adaptor, n4, 3);

  for (Adaptor::ArcIt a(adaptor); a != INVALID; ++a) {
    flow[a] = capacity[a] / 2;
  }

  checkGraphNodeList(adaptor, 4);
  checkGraphArcList(adaptor, 12);
  checkGraphConArcList(adaptor, 12);

  checkGraphOutArcList(adaptor, n1, 3);
  checkGraphOutArcList(adaptor, n2, 3);
  checkGraphOutArcList(adaptor, n3, 3);
  checkGraphOutArcList(adaptor, n4, 3);

  checkGraphInArcList(adaptor, n1, 3);
  checkGraphInArcList(adaptor, n2, 3);
  checkGraphInArcList(adaptor, n3, 3);
  checkGraphInArcList(adaptor, n4, 3);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);

  for (Adaptor::ArcIt a(adaptor); a != INVALID; ++a) {
    flow[a] = capacity[a];
  }

  checkGraphNodeList(adaptor, 4);
  checkGraphArcList(adaptor, 6);
  checkGraphConArcList(adaptor, 6);

  checkGraphOutArcList(adaptor, n1, 0);
  checkGraphOutArcList(adaptor, n2, 1);
  checkGraphOutArcList(adaptor, n3, 2);
  checkGraphOutArcList(adaptor, n4, 3);

  checkGraphInArcList(adaptor, n1, 3);
  checkGraphInArcList(adaptor, n2, 2);
  checkGraphInArcList(adaptor, n3, 1);
  checkGraphInArcList(adaptor, n4, 0);

  for (Adaptor::ArcIt a(adaptor); a != INVALID; ++a) {
    flow[a] = 0;
  }

  int flow_value = 0;
  while (true) {

    Bfs<Adaptor> bfs(adaptor);
    bfs.run(n1, n4);

    if (!bfs.reached(n4)) break;

    Path<Adaptor> p = bfs.path(n4);

    int min = std::numeric_limits<int>::max();
    for (Path<Adaptor>::ArcIt a(p); a != INVALID; ++a) {
      if (adaptor.residualCapacity(a) < min)
        min = adaptor.residualCapacity(a);
    }

    for (Path<Adaptor>::ArcIt a(p); a != INVALID; ++a) {
      adaptor.augment(a, min);
    }
    flow_value += min;
  }

  check(flow_value == 18, "Wrong flow with res graph adaptor");

}

void checkSplitNodes() {
  checkConcept<concepts::Digraph, SplitNodes<concepts::Digraph> >();

  typedef ListDigraph Digraph;
  typedef SplitNodes<Digraph> Adaptor;

  Digraph digraph;
  Adaptor adaptor(digraph);

  Digraph::Node n1 = digraph.addNode();
  Digraph::Node n2 = digraph.addNode();
  Digraph::Node n3 = digraph.addNode();

  Digraph::Arc a1 = digraph.addArc(n1, n2);
  Digraph::Arc a2 = digraph.addArc(n1, n3);
  Digraph::Arc a3 = digraph.addArc(n2, n3);

  checkGraphNodeList(adaptor, 6);
  checkGraphArcList(adaptor, 6);
  checkGraphConArcList(adaptor, 6);

  checkGraphOutArcList(adaptor, adaptor.inNode(n1), 1);
  checkGraphOutArcList(adaptor, adaptor.outNode(n1), 2);
  checkGraphOutArcList(adaptor, adaptor.inNode(n2), 1);
  checkGraphOutArcList(adaptor, adaptor.outNode(n2), 1);
  checkGraphOutArcList(adaptor, adaptor.inNode(n3), 1);
  checkGraphOutArcList(adaptor, adaptor.outNode(n3), 0);

  checkGraphInArcList(adaptor, adaptor.inNode(n1), 0);
  checkGraphInArcList(adaptor, adaptor.outNode(n1), 1);
  checkGraphInArcList(adaptor, adaptor.inNode(n2), 1);
  checkGraphInArcList(adaptor, adaptor.outNode(n2), 1);
  checkGraphInArcList(adaptor, adaptor.inNode(n3), 2);
  checkGraphInArcList(adaptor, adaptor.outNode(n3), 1);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);

  for (Adaptor::ArcIt a(adaptor); a != INVALID; ++a) {
    if (adaptor.origArc(a)) {
      Digraph::Arc oa = a;
      check(adaptor.source(a) == adaptor.outNode(digraph.source(oa)),
            "Wrong split");
      check(adaptor.target(a) == adaptor.inNode(digraph.target(oa)),
            "Wrong split");
    } else {
      Digraph::Node on = a;
      check(adaptor.source(a) == adaptor.inNode(on), "Wrong split");
      check(adaptor.target(a) == adaptor.outNode(on), "Wrong split");
    }
  }
}

void checkSubGraph() {
  checkConcept<concepts::Graph,
    SubGraph<concepts::Graph,
    concepts::Graph::NodeMap<bool>,
    concepts::Graph::EdgeMap<bool> > >();

  typedef ListGraph Graph;
  typedef Graph::NodeMap<bool> NodeFilter;
  typedef Graph::EdgeMap<bool> EdgeFilter;
  typedef SubGraph<Graph, NodeFilter, EdgeFilter> Adaptor;

  Graph graph;
  NodeFilter node_filter(graph);
  EdgeFilter edge_filter(graph);
  Adaptor adaptor(graph, node_filter, edge_filter);

  Graph::Node n1 = graph.addNode();
  Graph::Node n2 = graph.addNode();
  Graph::Node n3 = graph.addNode();
  Graph::Node n4 = graph.addNode();

  Graph::Edge e1 = graph.addEdge(n1, n2);
  Graph::Edge e2 = graph.addEdge(n1, n3);
  Graph::Edge e3 = graph.addEdge(n2, n3);
  Graph::Edge e4 = graph.addEdge(n3, n4);

  node_filter[n1] = node_filter[n2] = node_filter[n3] = node_filter[n4] = true;
  edge_filter[e1] = edge_filter[e2] = edge_filter[e3] = edge_filter[e4] = true;

  checkGraphNodeList(adaptor, 4);
  checkGraphArcList(adaptor, 8);
  checkGraphEdgeList(adaptor, 4);
  checkGraphConArcList(adaptor, 8);
  checkGraphConEdgeList(adaptor, 4);

  checkGraphOutArcList(adaptor, n1, 2);
  checkGraphOutArcList(adaptor, n2, 2);
  checkGraphOutArcList(adaptor, n3, 3);
  checkGraphOutArcList(adaptor, n4, 1);

  checkGraphInArcList(adaptor, n1, 2);
  checkGraphInArcList(adaptor, n2, 2);
  checkGraphInArcList(adaptor, n3, 3);
  checkGraphInArcList(adaptor, n4, 1);

  checkGraphIncEdgeList(adaptor, n1, 2);
  checkGraphIncEdgeList(adaptor, n2, 2);
  checkGraphIncEdgeList(adaptor, n3, 3);
  checkGraphIncEdgeList(adaptor, n4, 1);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);
  checkEdgeIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
  checkGraphEdgeMap(adaptor);

  edge_filter[e2] = false;

  checkGraphNodeList(adaptor, 4);
  checkGraphArcList(adaptor, 6);
  checkGraphEdgeList(adaptor, 3);
  checkGraphConArcList(adaptor, 6);
  checkGraphConEdgeList(adaptor, 3);

  checkGraphOutArcList(adaptor, n1, 1);
  checkGraphOutArcList(adaptor, n2, 2);
  checkGraphOutArcList(adaptor, n3, 2);
  checkGraphOutArcList(adaptor, n4, 1);

  checkGraphInArcList(adaptor, n1, 1);
  checkGraphInArcList(adaptor, n2, 2);
  checkGraphInArcList(adaptor, n3, 2);
  checkGraphInArcList(adaptor, n4, 1);

  checkGraphIncEdgeList(adaptor, n1, 1);
  checkGraphIncEdgeList(adaptor, n2, 2);
  checkGraphIncEdgeList(adaptor, n3, 2);
  checkGraphIncEdgeList(adaptor, n4, 1);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);
  checkEdgeIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
  checkGraphEdgeMap(adaptor);

  node_filter[n1] = false;

  checkGraphNodeList(adaptor, 3);
  checkGraphArcList(adaptor, 4);
  checkGraphEdgeList(adaptor, 2);
  checkGraphConArcList(adaptor, 4);
  checkGraphConEdgeList(adaptor, 2);

  checkGraphOutArcList(adaptor, n2, 1);
  checkGraphOutArcList(adaptor, n3, 2);
  checkGraphOutArcList(adaptor, n4, 1);

  checkGraphInArcList(adaptor, n2, 1);
  checkGraphInArcList(adaptor, n3, 2);
  checkGraphInArcList(adaptor, n4, 1);

  checkGraphIncEdgeList(adaptor, n2, 1);
  checkGraphIncEdgeList(adaptor, n3, 2);
  checkGraphIncEdgeList(adaptor, n4, 1);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);
  checkEdgeIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
  checkGraphEdgeMap(adaptor);

  node_filter[n1] = node_filter[n2] = node_filter[n3] = node_filter[n4] = false;
  edge_filter[e1] = edge_filter[e2] = edge_filter[e3] = edge_filter[e4] = false;

  checkGraphNodeList(adaptor, 0);
  checkGraphArcList(adaptor, 0);
  checkGraphEdgeList(adaptor, 0);
  checkGraphConArcList(adaptor, 0);
  checkGraphConEdgeList(adaptor, 0);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);
  checkEdgeIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
  checkGraphEdgeMap(adaptor);
}

void checkFilterNodes2() {
  checkConcept<concepts::Graph,
    FilterNodes<concepts::Graph,
      concepts::Graph::NodeMap<bool> > >();

  typedef ListGraph Graph;
  typedef Graph::NodeMap<bool> NodeFilter;
  typedef FilterNodes<Graph, NodeFilter> Adaptor;

  Graph graph;
  NodeFilter node_filter(graph);
  Adaptor adaptor(graph, node_filter);

  Graph::Node n1 = graph.addNode();
  Graph::Node n2 = graph.addNode();
  Graph::Node n3 = graph.addNode();
  Graph::Node n4 = graph.addNode();

  Graph::Edge e1 = graph.addEdge(n1, n2);
  Graph::Edge e2 = graph.addEdge(n1, n3);
  Graph::Edge e3 = graph.addEdge(n2, n3);
  Graph::Edge e4 = graph.addEdge(n3, n4);

  node_filter[n1] = node_filter[n2] = node_filter[n3] = node_filter[n4] = true;

  checkGraphNodeList(adaptor, 4);
  checkGraphArcList(adaptor, 8);
  checkGraphEdgeList(adaptor, 4);
  checkGraphConArcList(adaptor, 8);
  checkGraphConEdgeList(adaptor, 4);

  checkGraphOutArcList(adaptor, n1, 2);
  checkGraphOutArcList(adaptor, n2, 2);
  checkGraphOutArcList(adaptor, n3, 3);
  checkGraphOutArcList(adaptor, n4, 1);

  checkGraphInArcList(adaptor, n1, 2);
  checkGraphInArcList(adaptor, n2, 2);
  checkGraphInArcList(adaptor, n3, 3);
  checkGraphInArcList(adaptor, n4, 1);

  checkGraphIncEdgeList(adaptor, n1, 2);
  checkGraphIncEdgeList(adaptor, n2, 2);
  checkGraphIncEdgeList(adaptor, n3, 3);
  checkGraphIncEdgeList(adaptor, n4, 1);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);
  checkEdgeIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
  checkGraphEdgeMap(adaptor);

  node_filter[n1] = false;

  checkGraphNodeList(adaptor, 3);
  checkGraphArcList(adaptor, 4);
  checkGraphEdgeList(adaptor, 2);
  checkGraphConArcList(adaptor, 4);
  checkGraphConEdgeList(adaptor, 2);

  checkGraphOutArcList(adaptor, n2, 1);
  checkGraphOutArcList(adaptor, n3, 2);
  checkGraphOutArcList(adaptor, n4, 1);

  checkGraphInArcList(adaptor, n2, 1);
  checkGraphInArcList(adaptor, n3, 2);
  checkGraphInArcList(adaptor, n4, 1);

  checkGraphIncEdgeList(adaptor, n2, 1);
  checkGraphIncEdgeList(adaptor, n3, 2);
  checkGraphIncEdgeList(adaptor, n4, 1);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);
  checkEdgeIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
  checkGraphEdgeMap(adaptor);

  node_filter[n1] = node_filter[n2] = node_filter[n3] = node_filter[n4] = false;

  checkGraphNodeList(adaptor, 0);
  checkGraphArcList(adaptor, 0);
  checkGraphEdgeList(adaptor, 0);
  checkGraphConArcList(adaptor, 0);
  checkGraphConEdgeList(adaptor, 0);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);
  checkEdgeIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
  checkGraphEdgeMap(adaptor);
}

void checkFilterEdges() {
  checkConcept<concepts::Graph,
    FilterEdges<concepts::Graph,
    concepts::Graph::EdgeMap<bool> > >();

  typedef ListGraph Graph;
  typedef Graph::EdgeMap<bool> EdgeFilter;
  typedef FilterEdges<Graph, EdgeFilter> Adaptor;

  Graph graph;
  EdgeFilter edge_filter(graph);
  Adaptor adaptor(graph, edge_filter);

  Graph::Node n1 = graph.addNode();
  Graph::Node n2 = graph.addNode();
  Graph::Node n3 = graph.addNode();
  Graph::Node n4 = graph.addNode();

  Graph::Edge e1 = graph.addEdge(n1, n2);
  Graph::Edge e2 = graph.addEdge(n1, n3);
  Graph::Edge e3 = graph.addEdge(n2, n3);
  Graph::Edge e4 = graph.addEdge(n3, n4);

  edge_filter[e1] = edge_filter[e2] = edge_filter[e3] = edge_filter[e4] = true;

  checkGraphNodeList(adaptor, 4);
  checkGraphArcList(adaptor, 8);
  checkGraphEdgeList(adaptor, 4);
  checkGraphConArcList(adaptor, 8);
  checkGraphConEdgeList(adaptor, 4);

  checkGraphOutArcList(adaptor, n1, 2);
  checkGraphOutArcList(adaptor, n2, 2);
  checkGraphOutArcList(adaptor, n3, 3);
  checkGraphOutArcList(adaptor, n4, 1);

  checkGraphInArcList(adaptor, n1, 2);
  checkGraphInArcList(adaptor, n2, 2);
  checkGraphInArcList(adaptor, n3, 3);
  checkGraphInArcList(adaptor, n4, 1);

  checkGraphIncEdgeList(adaptor, n1, 2);
  checkGraphIncEdgeList(adaptor, n2, 2);
  checkGraphIncEdgeList(adaptor, n3, 3);
  checkGraphIncEdgeList(adaptor, n4, 1);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);
  checkEdgeIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
  checkGraphEdgeMap(adaptor);

  edge_filter[e2] = false;

  checkGraphNodeList(adaptor, 4);
  checkGraphArcList(adaptor, 6);
  checkGraphEdgeList(adaptor, 3);
  checkGraphConArcList(adaptor, 6);
  checkGraphConEdgeList(adaptor, 3);

  checkGraphOutArcList(adaptor, n1, 1);
  checkGraphOutArcList(adaptor, n2, 2);
  checkGraphOutArcList(adaptor, n3, 2);
  checkGraphOutArcList(adaptor, n4, 1);

  checkGraphInArcList(adaptor, n1, 1);
  checkGraphInArcList(adaptor, n2, 2);
  checkGraphInArcList(adaptor, n3, 2);
  checkGraphInArcList(adaptor, n4, 1);

  checkGraphIncEdgeList(adaptor, n1, 1);
  checkGraphIncEdgeList(adaptor, n2, 2);
  checkGraphIncEdgeList(adaptor, n3, 2);
  checkGraphIncEdgeList(adaptor, n4, 1);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);
  checkEdgeIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
  checkGraphEdgeMap(adaptor);

  edge_filter[e1] = edge_filter[e2] = edge_filter[e3] = edge_filter[e4] = false;

  checkGraphNodeList(adaptor, 4);
  checkGraphArcList(adaptor, 0);
  checkGraphEdgeList(adaptor, 0);
  checkGraphConArcList(adaptor, 0);
  checkGraphConEdgeList(adaptor, 0);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);
  checkEdgeIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);
  checkGraphEdgeMap(adaptor);
}

void checkOrienter() {
  checkConcept<concepts::Digraph,
    Orienter<concepts::Graph, concepts::Graph::EdgeMap<bool> > >();

  typedef ListGraph Graph;
  typedef ListGraph::EdgeMap<bool> DirMap;
  typedef Orienter<Graph> Adaptor;

  Graph graph;
  DirMap dir(graph, true);
  Adaptor adaptor(graph, dir);

  Graph::Node n1 = graph.addNode();
  Graph::Node n2 = graph.addNode();
  Graph::Node n3 = graph.addNode();

  Graph::Edge e1 = graph.addEdge(n1, n2);
  Graph::Edge e2 = graph.addEdge(n1, n3);
  Graph::Edge e3 = graph.addEdge(n2, n3);

  checkGraphNodeList(adaptor, 3);
  checkGraphArcList(adaptor, 3);
  checkGraphConArcList(adaptor, 3);

  {
    dir[e1] = true;
    Adaptor::Node u = adaptor.source(e1);
    Adaptor::Node v = adaptor.target(e1);

    dir[e1] = false;
    check (u == adaptor.target(e1), "Wrong dir");
    check (v == adaptor.source(e1), "Wrong dir");

    check ((u == n1 && v == n2) || (u == n2 && v == n1), "Wrong dir");
    dir[e1] = n1 == u;
  }

  {
    dir[e2] = true;
    Adaptor::Node u = adaptor.source(e2);
    Adaptor::Node v = adaptor.target(e2);

    dir[e2] = false;
    check (u == adaptor.target(e2), "Wrong dir");
    check (v == adaptor.source(e2), "Wrong dir");

    check ((u == n1 && v == n3) || (u == n3 && v == n1), "Wrong dir");
    dir[e2] = n3 == u;
  }

  {
    dir[e3] = true;
    Adaptor::Node u = adaptor.source(e3);
    Adaptor::Node v = adaptor.target(e3);

    dir[e3] = false;
    check (u == adaptor.target(e3), "Wrong dir");
    check (v == adaptor.source(e3), "Wrong dir");

    check ((u == n2 && v == n3) || (u == n3 && v == n2), "Wrong dir");
    dir[e3] = n2 == u;
  }

  checkGraphOutArcList(adaptor, n1, 1);
  checkGraphOutArcList(adaptor, n2, 1);
  checkGraphOutArcList(adaptor, n3, 1);

  checkGraphInArcList(adaptor, n1, 1);
  checkGraphInArcList(adaptor, n2, 1);
  checkGraphInArcList(adaptor, n3, 1);

  checkNodeIds(adaptor);
  checkArcIds(adaptor);

  checkGraphNodeMap(adaptor);
  checkGraphArcMap(adaptor);

}


int main(int, const char **) {

  checkReverseDigraph();
  checkSubDigraph();
  checkFilterNodes1();
  checkFilterArcs();
  checkUndirector();
  checkResidual();
  checkSplitNodes();

  checkSubGraph();
  checkFilterNodes2();
  checkFilterEdges();
  checkOrienter();

  return 0;
}
