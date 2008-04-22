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

#include <iostream>
#include <vector>

#include <lemon/graph_utils.h>

#include <lemon/list_graph.h>
#include <lemon/smart_graph.h>

#include "test_tools.h"
#include "graph_utils_test.h"


using namespace lemon;

template <class Graph>
void checkSnapDeg() 
{
  Graph g;
  typename Graph::Node n1=g.addNode();
  typename Graph::Node n2=g.addNode();
   
  InDegMap<Graph> ind(g);
 
  g.addArc(n1,n2);
  
  typename Graph::Snapshot snap(g);
  
  OutDegMap<Graph> outd(g);
  
  check(ind[n1]==0 && ind[n2]==1, "Wrong InDegMap value.");
  check(outd[n1]==1 && outd[n2]==0, "Wrong OutDegMap value.");

  g.addArc(n1,n2);
  g.addArc(n2,n1);
 
  check(ind[n1]==1 && ind[n2]==2, "Wrong InDegMap value.");
  check(outd[n1]==2 && outd[n2]==1, "Wrong OutDegMap value.");

  snap.restore();

  check(ind[n1]==0 && ind[n2]==1, "Wrong InDegMap value.");
  check(outd[n1]==1 && outd[n2]==0, "Wrong OutDegMap value.");
  
}

int main() {
  ///\file
  { // checking list graph
    checkDigraphCounters<ListDigraph>();
    checkFindArc<ListDigraph>();
  }
  { // checking smart graph
    checkDigraphCounters<SmartDigraph>();
    checkFindArc<SmartDigraph>();
  }
  {
    int num = 5;
    SmartDigraph fg;
    std::vector<SmartDigraph::Node> nodes;
    for (int i = 0; i < num; ++i) {
      nodes.push_back(fg.addNode());
    }
    for (int i = 0; i < num * num; ++i) {
      fg.addArc(nodes[i / num], nodes[i % num]);
    }
    check(countNodes(fg) == num, "FullGraph: wrong node number.");
    check(countArcs(fg) == num*num, "FullGraph: wrong arc number.");
    for (SmartDigraph::NodeIt src(fg); src != INVALID; ++src) {
      for (SmartDigraph::NodeIt trg(fg); trg != INVALID; ++trg) {
	ConArcIt<SmartDigraph> con(fg, src, trg);
	check(con != INVALID, "There is no connecting arc.");
	check(fg.source(con) == src, "Wrong source.");
	check(fg.target(con) == trg, "Wrong target.");
	check(++con == INVALID, "There is more connecting arc.");
      }
    }
    AllArcLookUp<SmartDigraph> el(fg);
    for (SmartDigraph::NodeIt src(fg); src != INVALID; ++src) {
      for (SmartDigraph::NodeIt trg(fg); trg != INVALID; ++trg) {
	SmartDigraph::Arc con = el(src, trg);
	check(con != INVALID, "There is no connecting arc.");
	check(fg.source(con) == src, "Wrong source.");
	check(fg.target(con) == trg, "Wrong target.");
	check(el(src,trg,con) == INVALID, "There is more connecting arc.");
      }
    }
  }

  //check In/OutDegMap (and Snapshot feature)

  checkSnapDeg<ListDigraph>();
  checkSnapDeg<SmartDigraph>();
  
  {
    const int nodeNum = 10;
    const int arcNum = 100;
    ListDigraph digraph;
    InDegMap<ListDigraph> inDeg(digraph);
    OutDegMap<ListDigraph> outDeg(digraph);
    std::vector<ListDigraph::Node> nodes(nodeNum);
    for (int i = 0; i < nodeNum; ++i) {
      nodes[i] = digraph.addNode();
    }
    std::vector<ListDigraph::Arc> arcs(arcNum);
    for (int i = 0; i < arcNum; ++i) {
      arcs[i] = 
	digraph.addArc(nodes[rnd[nodeNum]], nodes[rnd[nodeNum]]);
    }
    for (int i = 0; i < nodeNum; ++i) {
      check(inDeg[nodes[i]] == countInArcs(digraph, nodes[i]), 
	    "Wrong in degree map");
    }
    for (int i = 0; i < nodeNum; ++i) {
      check(outDeg[nodes[i]] == countOutArcs(digraph, nodes[i]), 
	    "Wrong in degree map");
    }
  }

  ///Everything is OK
  std::cout << __FILE__ ": All tests passed.\n";

  return 0;
}
