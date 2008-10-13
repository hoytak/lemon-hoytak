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

#include <iostream>
#include <vector>
#include <queue>
#include <lemon/math.h>
#include <cstdlib>

#include "test_tools.h"
#include <lemon/list_graph.h>
#include <lemon/max_matching.h>

using namespace std;
using namespace lemon;

int main() {

  typedef ListGraph Graph;

  GRAPH_TYPEDEFS(Graph);

  Graph g;
  g.clear();

  std::vector<Graph::Node> nodes;
  for (int i=0; i<13; ++i)
      nodes.push_back(g.addNode());

  g.addEdge(nodes[0], nodes[0]);
  g.addEdge(nodes[6], nodes[10]);
  g.addEdge(nodes[5], nodes[10]);
  g.addEdge(nodes[4], nodes[10]);
  g.addEdge(nodes[3], nodes[11]);
  g.addEdge(nodes[1], nodes[6]);
  g.addEdge(nodes[4], nodes[7]);
  g.addEdge(nodes[1], nodes[8]);
  g.addEdge(nodes[0], nodes[8]);
  g.addEdge(nodes[3], nodes[12]);
  g.addEdge(nodes[6], nodes[9]);
  g.addEdge(nodes[9], nodes[11]);
  g.addEdge(nodes[2], nodes[10]);
  g.addEdge(nodes[10], nodes[8]);
  g.addEdge(nodes[5], nodes[8]);
  g.addEdge(nodes[6], nodes[3]);
  g.addEdge(nodes[0], nodes[5]);
  g.addEdge(nodes[6], nodes[12]);

  MaxMatching<Graph> max_matching(g);
  max_matching.init();
  max_matching.startDense();

  int s=0;
  Graph::NodeMap<Node> mate(g,INVALID);
  max_matching.mateMap(mate);
  for(NodeIt v(g); v!=INVALID; ++v) {
    if ( mate[v]!=INVALID ) ++s;
  }
  int size=int(s/2);  //size will be used as the size of a maxmatching

  for(NodeIt v(g); v!=INVALID; ++v) {
    max_matching.mate(v);
  }

  check ( size == max_matching.size(), "mate() returns a different size matching than max_matching.size()" );

  Graph::NodeMap<MaxMatching<Graph>::DecompType> pos0(g);
  max_matching.decomposition(pos0);

  max_matching.init();
  max_matching.startSparse();
  s=0;
  max_matching.mateMap(mate);
  for(NodeIt v(g); v!=INVALID; ++v) {
    if ( mate[v]!=INVALID ) ++s;
  }
  check ( int(s/2) == size, "The size does not equal!" );

  Graph::NodeMap<MaxMatching<Graph>::DecompType> pos1(g);
  max_matching.decomposition(pos1);

  max_matching.run();
  s=0;
  max_matching.mateMap(mate);
  for(NodeIt v(g); v!=INVALID; ++v) {
    if ( mate[v]!=INVALID ) ++s;
  }
  check ( int(s/2) == size, "The size does not equal!" );

  Graph::NodeMap<MaxMatching<Graph>::DecompType> pos2(g);
  max_matching.decomposition(pos2);

  max_matching.run();
  s=0;
  max_matching.mateMap(mate);
  for(NodeIt v(g); v!=INVALID; ++v) {
    if ( mate[v]!=INVALID ) ++s;
  }
  check ( int(s/2) == size, "The size does not equal!" );

  Graph::NodeMap<MaxMatching<Graph>::DecompType> pos(g);
  max_matching.decomposition(pos);

  bool ismatching=true;
  for(NodeIt v(g); v!=INVALID; ++v) {
    if ( mate[v]!=INVALID ) {
      Node u=mate[v];
      if (mate[u]!=v) ismatching=false;
    }
  }
  check ( ismatching, "It is not a matching!" );

  bool coincide=true;
  for(NodeIt v(g); v!=INVALID; ++v) {
   if ( pos0[v] != pos1[v] || pos1[v]!=pos2[v] || pos2[v]!=pos[v] ) {
     coincide=false;
    }
  }
  check ( coincide, "The decompositions do not coincide! " );

  bool noarc=true;
  for(EdgeIt e(g); e!=INVALID; ++e) {
   if ( (pos[g.v(e)]==max_matching.C &&
         pos[g.u(e)]==max_matching.D) ||
         (pos[g.v(e)]==max_matching.D &&
          pos[g.u(e)]==max_matching.C) )
      noarc=false;
  }
  check ( noarc, "There are arcs between D and C!" );

  bool oddcomp=true;
  Graph::NodeMap<bool> todo(g,true);
  int num_comp=0;
  for(NodeIt v(g); v!=INVALID; ++v) {
   if ( pos[v]==max_matching.D && todo[v] ) {
      int comp_size=1;
      ++num_comp;
      std::queue<Node> Q;
      Q.push(v);
      todo.set(v,false);
      while (!Q.empty()) {
        Node w=Q.front();
        Q.pop();
        for(IncEdgeIt e(g,w); e!=INVALID; ++e) {
          Node u=g.runningNode(e);
          if ( pos[u]==max_matching.D && todo[u] ) {
            ++comp_size;
            Q.push(u);
            todo.set(u,false);
          }
        }
      }
      if ( !(comp_size % 2) ) oddcomp=false;
    }
  }
  check ( oddcomp, "A component of g[D] is not odd." );

  int barrier=0;
  for(NodeIt v(g); v!=INVALID; ++v) {
    if ( pos[v]==max_matching.A ) ++barrier;
  }
  int expected_size=int( countNodes(g)-num_comp+barrier)/2;
  check ( size==expected_size, "The size of the matching is wrong." );

  return 0;
}
