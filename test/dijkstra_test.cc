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

#include <lemon/concepts/digraph.h>
#include <lemon/smart_graph.h>
#include <lemon/list_graph.h>
#include <lemon/graph_utils.h>
#include <lemon/dijkstra.h>
#include <lemon/path.h>

#include "graph_test.h"
#include "test_tools.h"

using namespace lemon;

void checkDijkstraCompile() 
{
  typedef int VType;
  typedef concepts::Digraph Digraph;
  typedef concepts::ReadMap<Digraph::Arc,VType> LengthMap;
  typedef Dijkstra<Digraph, LengthMap> DType;
  
  Digraph G;
  Digraph::Node n;
  Digraph::Arc e;
  VType l;
  bool b;
  DType::DistMap d(G);
  DType::PredMap p(G);
  //  DType::PredNodeMap pn(G);
  LengthMap length;

  DType dijkstra_test(G,length);

  dijkstra_test.run(n);

  l  = dijkstra_test.dist(n);
  e  = dijkstra_test.predArc(n);
  n  = dijkstra_test.predNode(n);
  d  = dijkstra_test.distMap();
  p  = dijkstra_test.predMap();
  //  pn = dijkstra_test.predNodeMap();
  b  = dijkstra_test.reached(n);

  Path<Digraph> pp = dijkstra_test.path(n);
}

void checkDijkstraFunctionCompile() 
{
  typedef int VType;
  typedef concepts::Digraph Digraph;
  typedef Digraph::Arc Arc;
  typedef Digraph::Node Node;
  typedef concepts::ReadMap<Digraph::Arc,VType> LengthMap;
   
  Digraph g;
  dijkstra(g,LengthMap(),Node()).run();
  dijkstra(g,LengthMap()).source(Node()).run();
  dijkstra(g,LengthMap())
    .predMap(concepts::WriteMap<Node,Arc>())
    .distMap(concepts::WriteMap<Node,VType>())
    .run(Node());
}

template <class Digraph>
void checkDijkstra() {    
  TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
  typedef typename Digraph::template ArcMap<int> LengthMap;

  Digraph G;
  Node s, t;
  LengthMap length(G);
  PetStruct<Digraph> ps = addPetersen(G, 5);
   
  for(int i=0;i<5;i++) {
    length[ps.outcir[i]]=4;
    length[ps.incir[i]]=1;
    length[ps.chords[i]]=10;
  }
  s=ps.outer[0];
  t=ps.inner[1];
  
  Dijkstra<Digraph, LengthMap> 
	dijkstra_test(G, length);
  dijkstra_test.run(s);
  
  check(dijkstra_test.dist(t)==13,"Dijkstra found a wrong path.");

  Path<Digraph> p = dijkstra_test.path(t);
  check(p.length()==4,"getPath() found a wrong path.");
  check(checkPath(G, p),"path() found a wrong path.");
  check(pathSource(G, p) == s,"path() found a wrong path.");
  check(pathTarget(G, p) == t,"path() found a wrong path.");
  
  for(ArcIt e(G); e!=INVALID; ++e) {
    Node u=G.source(e);
    Node v=G.target(e);
    check( !dijkstra_test.reached(u) || (dijkstra_test.dist(v) - dijkstra_test.dist(u) <= length[e]),
	   "dist(target)-dist(source)-arc_length= " << dijkstra_test.dist(v) - dijkstra_test.dist(u) - length[e]);
  }

  for(NodeIt v(G); v!=INVALID; ++v){
    check(dijkstra_test.reached(v),"Each node should be reached.");
    if ( dijkstra_test.predArc(v)!=INVALID ) {
      Arc e=dijkstra_test.predArc(v);
      Node u=G.source(e);
      check(u==dijkstra_test.predNode(v),"Wrong tree.");
      check(dijkstra_test.dist(v) - dijkstra_test.dist(u) == length[e],
	    "Wrong distance! Difference: " << std::abs(dijkstra_test.dist(v) - dijkstra_test.dist(u) - length[e]));
    }
  }
  
  {
    NullMap<Node,Arc> myPredMap;
    dijkstra(G,length).predMap(myPredMap).run(s);
  }
}

int main() {
  checkDijkstra<ListDigraph>();
  checkDijkstra<SmartDigraph>();
  return 0;
}
