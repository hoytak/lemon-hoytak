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

#ifndef LEMON_TEST_GRAPH_UTILS_TEST_H
#define LEMON_TEST_GRAPH_UTILS_TEST_H


#include "test_tools.h"
#include <cstdlib>
#include <ctime>

//! \ingroup misc
//! \file
//! \brief Test cases for graph utils.
namespace lemon {
  
  template <typename Digraph>
  void checkDigraphCounters() {
    const int num = 5;
    Digraph digraph;
    addPetersen(digraph, num);
    bidirDigraph(digraph);
    check(countNodes(digraph) == 2*num, "Wrong node number.");
    check(countArcs(digraph) == 6*num, "Wrong arc number.");    
    for (typename Digraph::NodeIt it(digraph); it != INVALID; ++it) {
      check(countOutArcs(digraph, it) == 3, "Wrong out degree number.");
      check(countInArcs(digraph, it) == 3, "Wrong in degree number.");
    }
  }

  template <typename Digraph>
  void checkFindArc() {
    typedef typename Digraph::Node Node;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::ArcIt ArcIt;
    Digraph digraph;
    for (int i = 0; i < 10; ++i) {
      digraph.addNode();
    }
    DescriptorMap<Digraph, Node> nodes(digraph);
    typename DescriptorMap<Digraph, Node>::InverseMap invNodes(nodes);
    for (int i = 0; i < 100; ++i) {
      int src = rnd[invNodes.size()];
      int trg = rnd[invNodes.size()];
      digraph.addArc(invNodes[src], invNodes[trg]);
    }
    typename Digraph::template ArcMap<bool> found(digraph, false);
    DescriptorMap<Digraph, Arc> arcs(digraph);
    for (NodeIt src(digraph); src != INVALID; ++src) {
      for (NodeIt trg(digraph); trg != INVALID; ++trg) {
	for (ConArcIt<Digraph> con(digraph, src, trg); con != INVALID; ++con) {
	  check(digraph.source(con) == src, "Wrong source.");
	  check(digraph.target(con) == trg, "Wrong target.");
	  check(found[con] == false, "The arc found already.");
	  found[con] = true;
	}
      }
    }
    for (ArcIt it(digraph); it != INVALID; ++it) {
      check(found[it] == true, "The arc is not found.");
    }
  }
  
} //namespace lemon


#endif
