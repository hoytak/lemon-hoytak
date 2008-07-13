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

#ifndef LEMON_TEST_MAP_TEST_H
#define LEMON_TEST_MAP_TEST_H

#include <vector>
#include <lemon/maps.h>

#include "test_tools.h"

namespace lemon {

  template <typename Graph>
  void checkGraphNodeMap() {
    Graph graph;
    const int num = 16;

    typedef typename Graph::Node Node;

    std::vector<Node> nodes;
    for (int i = 0; i < num; ++i) {
      nodes.push_back(graph.addNode());
    }
    typedef typename Graph::template NodeMap<int> IntNodeMap;
    IntNodeMap map(graph, 42);
    for (int i = 0; i < int(nodes.size()); ++i) {
      check(map[nodes[i]] == 42, "Wrong map constructor.");
    }
    for (int i = 0; i < num; ++i) {
      nodes.push_back(graph.addNode());
      map[nodes.back()] = 23;
      check(map[nodes.back()] == 23, "Wrong operator[].");
    }
    map = constMap<Node>(12);
    for (int i = 0; i < int(nodes.size()); ++i) {
      check(map[nodes[i]] == 12, "Wrong map constructor.");
    }
    graph.clear();
    nodes.clear();
  }

  template <typename Graph>
  void checkGraphArcMap() {
    Graph graph;
    const int num = 16;

    typedef typename Graph::Node Node;
    typedef typename Graph::Arc Arc;

    std::vector<Node> nodes;
    for (int i = 0; i < num; ++i) {
      nodes.push_back(graph.addNode());
    }

    std::vector<Arc> arcs;
    for (int i = 0; i < num; ++i) {
      for (int j = 0; j < i; ++j) {
        arcs.push_back(graph.addArc(nodes[i], nodes[j]));
      }
    }

    typedef typename Graph::template ArcMap<int> IntArcMap;
    IntArcMap map(graph, 42);

    for (int i = 0; i < int(arcs.size()); ++i) {
      check(map[arcs[i]] == 42, "Wrong map constructor.");
    }

    for (int i = 0; i < num; ++i) {
      for (int j = i + 1; j < num; ++j) {
        arcs.push_back(graph.addArc(nodes[i], nodes[j]));
        map[arcs.back()] = 23;
        check(map[arcs.back()] == 23, "Wrong operator[].");
      }
    }
    map = constMap<Arc>(12);
    for (int i = 0; i < int(arcs.size()); ++i) {
      check(map[arcs[i]] == 12, "Wrong map constructor.");
    }
    graph.clear();
    arcs.clear();
  }

  template <typename Graph>
  void checkGraphEdgeMap() {
    Graph graph;
    const int num = 16;

    typedef typename Graph::Node Node;
    typedef typename Graph::Edge Edge;

    std::vector<Node> nodes;
    for (int i = 0; i < num; ++i) {
      nodes.push_back(graph.addNode());
    }

    std::vector<Edge> edges;
    for (int i = 0; i < num; ++i) {
      for (int j = 0; j < i; ++j) {
        edges.push_back(graph.addEdge(nodes[i], nodes[j]));
      }
    }

    typedef typename Graph::template EdgeMap<int> IntEdgeMap;
    IntEdgeMap map(graph, 42);

    for (int i = 0; i < int(edges.size()); ++i) {
      check(map[edges[i]] == 42, "Wrong map constructor.");
    }

    for (int i = 0; i < num; ++i) {
      for (int j = i + 1; j < num; ++j) {
        edges.push_back(graph.addEdge(nodes[i], nodes[j]));
        map[edges.back()] = 23;
        check(map[edges.back()] == 23, "Wrong operator[].");
      }
    }
    map = constMap<Edge>(12);
    for (int i = 0; i < int(edges.size()); ++i) {
      check(map[edges[i]] == 12, "Wrong map constructor.");
    }
    graph.clear();
    edges.clear();
  }

}

#endif
