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
#include <lemon/smart_graph.h>
#include <lemon/list_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/bfs.h>
#include <lemon/path.h>

#include "graph_test.h"
#include "test_tools.h"

using namespace lemon;

char test_lgf[] =
  "@nodes\n"
  "label\n"
  "0\n"
  "1\n"
  "2\n"
  "3\n"
  "4\n"
  "5\n"
  "@arcs\n"
  "     label\n"
  "0 1  0\n"
  "1 2  1\n"
  "2 3  2\n"
  "3 4  3\n"
  "0 3  4\n"
  "0 3  5\n"
  "5 2  6\n"
  "@attributes\n"
  "source 0\n"
  "target 4\n";

void checkBfsCompile()
{
  typedef concepts::Digraph Digraph;
  typedef Bfs<Digraph> BType;

  Digraph G;
  Digraph::Node n;
  Digraph::Arc e;
  int l;
  bool b;
  BType::DistMap d(G);
  BType::PredMap p(G);

  BType bfs_test(G);

  bfs_test.run(n);

  l  = bfs_test.dist(n);
  e  = bfs_test.predArc(n);
  n  = bfs_test.predNode(n);
  d  = bfs_test.distMap();
  p  = bfs_test.predMap();
  b  = bfs_test.reached(n);

  Path<Digraph> pp = bfs_test.path(n);
}

void checkBfsFunctionCompile()
{
  typedef int VType;
  typedef concepts::Digraph Digraph;
  typedef Digraph::Arc Arc;
  typedef Digraph::Node Node;

  Digraph g;
  bool b;
  bfs(g).run(Node());
  b=bfs(g).run(Node(),Node());
  bfs(g).run();
  bfs(g)
    .predMap(concepts::ReadWriteMap<Node,Arc>())
    .distMap(concepts::ReadWriteMap<Node,VType>())
    .reachedMap(concepts::ReadWriteMap<Node,bool>())
    .processedMap(concepts::WriteMap<Node,bool>())
    .run(Node());
  b=bfs(g)
    .predMap(concepts::ReadWriteMap<Node,Arc>())
    .distMap(concepts::ReadWriteMap<Node,VType>())
    .reachedMap(concepts::ReadWriteMap<Node,bool>())
    .processedMap(concepts::WriteMap<Node,bool>())
    .path(concepts::Path<Digraph>())
    .dist(VType())
    .run(Node(),Node());
  bfs(g)
    .predMap(concepts::ReadWriteMap<Node,Arc>())
    .distMap(concepts::ReadWriteMap<Node,VType>())
    .reachedMap(concepts::ReadWriteMap<Node,bool>())
    .processedMap(concepts::WriteMap<Node,bool>())
    .run();
}

template <class Digraph>
void checkBfs() {
  TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

  Digraph G;
  Node s, t;

  std::istringstream input(test_lgf);
  digraphReader(input, G).
    node("source", s).
    node("target", t).
    run();

  Bfs<Digraph> bfs_test(G);
  bfs_test.run(s);

  check(bfs_test.dist(t)==2,"Bfs found a wrong path.");

  Path<Digraph> p = bfs_test.path(t);
  check(p.length()==2,"path() found a wrong path.");
  check(checkPath(G, p),"path() found a wrong path.");
  check(pathSource(G, p) == s,"path() found a wrong path.");
  check(pathTarget(G, p) == t,"path() found a wrong path.");


  for(ArcIt a(G); a!=INVALID; ++a) {
    Node u=G.source(a);
    Node v=G.target(a);
    check( !bfs_test.reached(u) ||
           (bfs_test.dist(v) <= bfs_test.dist(u)+1),
           "Wrong output. " << G.id(u) << "->" << G.id(v));
  }

  for(NodeIt v(G); v!=INVALID; ++v) {
    if (bfs_test.reached(v)) {
      check(v==s || bfs_test.predArc(v)!=INVALID, "Wrong tree.");
      if (bfs_test.predArc(v)!=INVALID ) {
        Arc a=bfs_test.predArc(v);
        Node u=G.source(a);
        check(u==bfs_test.predNode(v),"Wrong tree.");
        check(bfs_test.dist(v) - bfs_test.dist(u) == 1,
              "Wrong distance. Difference: "
              << std::abs(bfs_test.dist(v) - bfs_test.dist(u) - 1));
      }
    }
  }

  {
    NullMap<Node,Arc> myPredMap;
    bfs(G).predMap(myPredMap).run(s);
  }
}

int main()
{
  checkBfs<ListDigraph>();
  checkBfs<SmartDigraph>();
  return 0;
}
