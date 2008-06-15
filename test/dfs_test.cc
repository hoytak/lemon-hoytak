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
#include <lemon/dfs.h>
#include <lemon/path.h>

#include "graph_test.h"
#include "test_tools.h"

using namespace lemon;

void checkDfsCompile() 
{
  typedef concepts::Digraph Digraph;
  typedef Dfs<Digraph> DType;
  
  Digraph G;
  Digraph::Node n;
  Digraph::Arc e;
  int l;
  bool b;
  DType::DistMap d(G);
  DType::PredMap p(G);
  //  DType::PredNodeMap pn(G);
  
  DType dfs_test(G);
  
  dfs_test.run(n);
  
  l  = dfs_test.dist(n);
  e  = dfs_test.predArc(n);
  n  = dfs_test.predNode(n);
  d  = dfs_test.distMap();
  p  = dfs_test.predMap();
  //  pn = dfs_test.predNodeMap();
  b  = dfs_test.reached(n);

  Path<Digraph> pp = dfs_test.path(n);
}

void checkDfsFunctionCompile() 
{
  typedef int VType;
  typedef concepts::Digraph Digraph;
  typedef Digraph::Arc Arc;
  typedef Digraph::Node Node;
   
  Digraph g;
  dfs(g,Node()).run();
  dfs(g).source(Node()).run();
  dfs(g)
    .predMap(concepts::WriteMap<Node,Arc>())
    .distMap(concepts::WriteMap<Node,VType>())
    .reachedMap(concepts::ReadWriteMap<Node,bool>())
    .processedMap(concepts::WriteMap<Node,bool>())
    .run(Node()); 
}

template <class Digraph>
void checkDfs() {
  TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

  Digraph G;
  Node s, t;
  PetStruct<Digraph> ps = addPetersen(G, 5);
   
  s=ps.outer[2];
  t=ps.inner[0];
  
  Dfs<Digraph> dfs_test(G);
  dfs_test.run(s);  
  
  Path<Digraph> p = dfs_test.path(t);
  check(p.length() == dfs_test.dist(t),"path() found a wrong path.");
  check(checkPath(G, p),"path() found a wrong path.");
  check(pathSource(G, p) == s,"path() found a wrong path.");
  check(pathTarget(G, p) == t,"path() found a wrong path.");
  
  for(NodeIt v(G); v!=INVALID; ++v) {
    check(dfs_test.reached(v),"Each node should be reached.");
    if ( dfs_test.predArc(v)!=INVALID ) {
      Arc e=dfs_test.predArc(v);
      Node u=G.source(e);
      check(u==dfs_test.predNode(v),"Wrong tree.");
      check(dfs_test.dist(v) - dfs_test.dist(u) == 1,
	    "Wrong distance. (" << dfs_test.dist(u) << "->" 
	    <<dfs_test.dist(v) << ')');
    }
  }
}

int main()
{
  checkDfs<ListDigraph>();
  checkDfs<SmartDigraph>();
  return 0;
}
