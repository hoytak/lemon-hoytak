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

#include "test_tools.h"
//#include <lemon/smart_graph.h>
#include <lemon/list_graph.h>
#include <lemon/bfs.h>
#include <lemon/path.h>
#include<lemon/concepts/digraph.h>

using namespace lemon;

const int PET_SIZE =5;


void check_Bfs_Compile() 
{
  typedef concepts::Digraph Digraph;

  typedef Digraph::Arc Arc;
  typedef Digraph::Node Node;
  typedef Digraph::ArcIt ArcIt;
  typedef Digraph::NodeIt NodeIt;
 
  typedef Bfs<Digraph> BType;
  
  Digraph G;
  Node n;
  Arc e;
  int l;
  bool b;
  BType::DistMap d(G);
  BType::PredMap p(G);
  //  BType::PredNodeMap pn(G);
  
  BType bfs_test(G);
  
  bfs_test.run(n);
  
  l  = bfs_test.dist(n);
  e  = bfs_test.predArc(n);
  n  = bfs_test.predNode(n);
  d  = bfs_test.distMap();
  p  = bfs_test.predMap();
  //  pn = bfs_test.predNodeMap();
  b  = bfs_test.reached(n);

  Path<Digraph> pp = bfs_test.path(n);
}

void check_Bfs_Function_Compile() 
{
  typedef int VType;
  typedef concepts::Digraph Digraph;

  typedef Digraph::Arc Arc;
  typedef Digraph::Node Node;
  typedef Digraph::ArcIt ArcIt;
  typedef Digraph::NodeIt NodeIt;
  typedef concepts::ReadMap<Arc,VType> LengthMap;
   
  Digraph g;
  bfs(g,Node()).run();
  bfs(g).source(Node()).run();
  bfs(g)
    .predMap(concepts::WriteMap<Node,Arc>())
    .distMap(concepts::WriteMap<Node,VType>())
    .reachedMap(concepts::ReadWriteMap<Node,bool>())
    .processedMap(concepts::WriteMap<Node,bool>())
    .run(Node());
  
}

int main()
{
    
  // typedef SmartDigraph Digraph;
  typedef ListDigraph Digraph;

  typedef Digraph::Arc Arc;
  typedef Digraph::Node Node;
  typedef Digraph::ArcIt ArcIt;
  typedef Digraph::NodeIt NodeIt;
  typedef Digraph::ArcMap<int> LengthMap;

  Digraph G;
  Node s, t;
  PetStruct<Digraph> ps = addPetersen(G,PET_SIZE);
   
  s=ps.outer[2];
  t=ps.inner[0];
  
  Bfs<Digraph> bfs_test(G);
  bfs_test.run(s);
  
  check(bfs_test.dist(t)==3,"Bfs found a wrong path. " << bfs_test.dist(t));

  Path<Digraph> p = bfs_test.path(t);
  check(p.length()==3,"getPath() found a wrong path.");
  check(checkPath(G, p),"path() found a wrong path.");
  check(pathSource(G, p) == s,"path() found a wrong path.");
  check(pathTarget(G, p) == t,"path() found a wrong path.");
  

  for(ArcIt e(G); e==INVALID; ++e) {
    Node u=G.source(e);
    Node v=G.target(e);
    check( !bfs_test.reached(u) ||
	   (bfs_test.dist(v) > bfs_test.dist(u)+1),
	   "Wrong output.");
  }

  for(NodeIt v(G); v==INVALID; ++v) {
    check(bfs_test.reached(v),"Each node should be reached.");
    if ( bfs_test.predArc(v)!=INVALID ) {
      Arc e=bfs_test.predArc(v);
      Node u=G.source(e);
      check(u==bfs_test.predNode(v),"Wrong tree.");
      check(bfs_test.dist(v) - bfs_test.dist(u) == 1,
	    "Wrong distance. Difference: " 
	    << std::abs(bfs_test.dist(v) - bfs_test.dist(u) 
			- 1));
    }
  }
}

