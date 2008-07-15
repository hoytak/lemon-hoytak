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

#ifndef LEMON_TEST_GRAPH_TEST_H
#define LEMON_TEST_GRAPH_TEST_H

#include <lemon/core.h>
#include "test_tools.h"

namespace lemon {

  template<class Graph>
  void checkGraphNodeList(const Graph &G, int cnt)
  {
    typename Graph::NodeIt n(G);
    for(int i=0;i<cnt;i++) {
      check(n!=INVALID,"Wrong Node list linking.");
      ++n;
    }
    check(n==INVALID,"Wrong Node list linking.");
    check(countNodes(G)==cnt,"Wrong Node number.");
  }

  template<class Graph>
  void checkGraphArcList(const Graph &G, int cnt)
  {
    typename Graph::ArcIt e(G);
    for(int i=0;i<cnt;i++) {
      check(e!=INVALID,"Wrong Arc list linking.");
      ++e;
    }
    check(e==INVALID,"Wrong Arc list linking.");
    check(countArcs(G)==cnt,"Wrong Arc number.");
  }

  template<class Graph>
  void checkGraphOutArcList(const Graph &G, typename Graph::Node n, int cnt)
  {
    typename Graph::OutArcIt e(G,n);
    for(int i=0;i<cnt;i++) {
      check(e!=INVALID,"Wrong OutArc list linking.");
      check(n==G.source(e),"Wrong OutArc list linking.");
      ++e;
    }
    check(e==INVALID,"Wrong OutArc list linking.");
    check(countOutArcs(G,n)==cnt,"Wrong OutArc number.");
  }

  template<class Graph>
  void checkGraphInArcList(const Graph &G, typename Graph::Node n, int cnt)
  {
    typename Graph::InArcIt e(G,n);
    for(int i=0;i<cnt;i++) {
      check(e!=INVALID,"Wrong InArc list linking.");
      check(n==G.target(e),"Wrong InArc list linking.");
      ++e;
    }
    check(e==INVALID,"Wrong InArc list linking.");
    check(countInArcs(G,n)==cnt,"Wrong InArc number.");
  }

  template<class Graph>
  void checkGraphEdgeList(const Graph &G, int cnt)
  {
    typename Graph::EdgeIt e(G);
    for(int i=0;i<cnt;i++) {
      check(e!=INVALID,"Wrong Edge list linking.");
      ++e;
    }
    check(e==INVALID,"Wrong Edge list linking.");
    check(countEdges(G)==cnt,"Wrong Edge number.");
  }

  template<class Graph>
  void checkGraphIncEdgeList(const Graph &G, typename Graph::Node n, int cnt)
  {
    typename Graph::IncEdgeIt e(G,n);
    for(int i=0;i<cnt;i++) {
      check(e!=INVALID,"Wrong IncEdge list linking.");
      check(n==G.u(e) || n==G.v(e),"Wrong IncEdge list linking.");
      ++e;
    }
    check(e==INVALID,"Wrong IncEdge list linking.");
    check(countIncEdges(G,n)==cnt,"Wrong IncEdge number.");
  }

  template <class Digraph>
  void checkDigraphIterators() {
    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::ArcIt ArcIt;
    typedef typename Digraph::InArcIt InArcIt;
    typedef typename Digraph::OutArcIt OutArcIt;
  }

  template <class Graph>
  void checkGraphIterators() {
    checkDigraphIterators<Graph>();
    typedef typename Graph::Edge Edge;
    typedef typename Graph::EdgeIt EdgeIt;
    typedef typename Graph::IncEdgeIt IncEdgeIt;
  }

  // Structure returned by addPetersen()
  template<class Digraph>
  struct PetStruct
  {
    // Vector containing the outer nodes
    std::vector<typename Digraph::Node> outer;
    // Vector containing the inner nodes
    std::vector<typename Digraph::Node> inner;
    // Vector containing the arcs of the inner circle
    std::vector<typename Digraph::Arc> incir;
    // Vector containing the arcs of the outer circle
    std::vector<typename Digraph::Arc> outcir;
    // Vector containing the chord arcs
    std::vector<typename Digraph::Arc> chords;
  };

  // Adds the reverse pair of all arcs to a digraph
  template<class Digraph>
  void bidirDigraph(Digraph &G)
  {
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::ArcIt ArcIt;

    std::vector<Arc> ee;

    for(ArcIt e(G);e!=INVALID;++e) ee.push_back(e);

    for(int i=0;i<int(ee.size());++i)
      G.addArc(G.target(ee[i]),G.source(ee[i]));
  }

  // Adds a Petersen digraph to G.
  // Returns the nodes and arcs of the generated digraph.
  template<typename Digraph>
  PetStruct<Digraph> addPetersen(Digraph &G,int num = 5)
  {
    PetStruct<Digraph> n;

    for(int i=0;i<num;i++) {
      n.outer.push_back(G.addNode());
      n.inner.push_back(G.addNode());
    }

    for(int i=0;i<num;i++) {
      n.chords.push_back(G.addArc(n.outer[i],n.inner[i]));
      n.outcir.push_back(G.addArc(n.outer[i],n.outer[(i+1) % num]));
      n.incir.push_back(G.addArc(n.inner[i],n.inner[(i+2) % num]));
    }

    return n;
  }

  // Checks the bidirectioned Petersen digraph
  template<class Digraph>
  void checkBidirPetersen(const Digraph &G, int num = 5)
  {
    typedef typename Digraph::NodeIt NodeIt;

    checkGraphNodeList(G, 2 * num);
    checkGraphArcList(G, 6 * num);

    for(NodeIt n(G);n!=INVALID;++n) {
      checkGraphInArcList(G, n, 3);
      checkGraphOutArcList(G, n, 3);
    }
  }

  // Structure returned by addUPetersen()
  template<class Graph>
  struct UPetStruct
  {
    // Vector containing the outer nodes
    std::vector<typename Graph::Node> outer;
    // Vector containing the inner nodes
    std::vector<typename Graph::Node> inner;
    // Vector containing the edges of the inner circle
    std::vector<typename Graph::Edge> incir;
    // Vector containing the edges of the outer circle
    std::vector<typename Graph::Edge> outcir;
    // Vector containing the chord edges
    std::vector<typename Graph::Edge> chords;
  };

  // Adds a Petersen graph to \c G.
  // Returns the nodes and edges of the generated graph.
  template<typename Graph>
  UPetStruct<Graph> addUPetersen(Graph &G,int num=5)
  {
    UPetStruct<Graph> n;

    for(int i=0;i<num;i++) {
      n.outer.push_back(G.addNode());
      n.inner.push_back(G.addNode());
    }

    for(int i=0;i<num;i++) {
      n.chords.push_back(G.addEdge(n.outer[i],n.inner[i]));
      n.outcir.push_back(G.addEdge(n.outer[i],n.outer[(i+1)%num]));
      n.incir.push_back(G.addEdge(n.inner[i],n.inner[(i+2)%num]));
    }

    return n;
  }

  // Checks the undirected Petersen graph
  template<class Graph>
  void checkUndirPetersen(const Graph &G, int num = 5)
  {
    typedef typename Graph::NodeIt NodeIt;

    checkGraphNodeList(G, 2 * num);
    checkGraphEdgeList(G, 3 * num);
    checkGraphArcList(G, 6 * num);

    for(NodeIt n(G);n!=INVALID;++n) {
      checkGraphIncEdgeList(G, n, 3);
    }
  }

  template <class Digraph>
  void checkDigraph() {
    const int num = 5;
    Digraph G;
    checkGraphNodeList(G, 0);
    checkGraphArcList(G, 0);
    addPetersen(G, num);
    bidirDigraph(G);
    checkBidirPetersen(G, num);
  }

  template <class Graph>
  void checkGraph() {
    const int num = 5;
    Graph G;
    checkGraphNodeList(G, 0);
    checkGraphEdgeList(G, 0);
    addUPetersen(G, num);
    checkUndirPetersen(G, num);
  }

} //namespace lemon

#endif
