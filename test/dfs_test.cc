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
#include <lemon/dfs.h>
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
  "6\n"
  "@arcs\n"
  "     label\n"
  "0 1  0\n"
  "1 2  1\n"
  "2 3  2\n"
  "1 4  3\n"
  "4 2  4\n"
  "4 5  5\n"
  "5 0  6\n"
  "6 3  7\n"
  "@attributes\n"
  "source 0\n"
  "target 5\n";

void checkDfsCompile()
{
  typedef concepts::Digraph Digraph;
  typedef Dfs<Digraph> DType;
  typedef Digraph::Node Node;
  typedef Digraph::Arc Arc;

  Digraph G;
  Node s, t;
  Arc e;
  int l;
  bool b;
  DType::DistMap d(G);
  DType::PredMap p(G);
  Path<Digraph> pp;

  {
    DType dfs_test(G);

    dfs_test.run(s);
    dfs_test.run(s,t);
    dfs_test.run();

    l  = dfs_test.dist(t);
    e  = dfs_test.predArc(t);
    s  = dfs_test.predNode(t);
    b  = dfs_test.reached(t);
    d  = dfs_test.distMap();
    p  = dfs_test.predMap();
    pp = dfs_test.path(t);
  }
  {
    DType
      ::SetPredMap<concepts::ReadWriteMap<Node,Arc> >
      ::SetDistMap<concepts::ReadWriteMap<Node,int> >
      ::SetReachedMap<concepts::ReadWriteMap<Node,bool> >
      ::SetProcessedMap<concepts::WriteMap<Node,bool> >
      ::SetStandardProcessedMap
      ::Create dfs_test(G);

    dfs_test.run(s);
    dfs_test.run(s,t);
    dfs_test.run();

    l  = dfs_test.dist(t);
    e  = dfs_test.predArc(t);
    s  = dfs_test.predNode(t);
    b  = dfs_test.reached(t);
    pp = dfs_test.path(t);
  }
}

void checkDfsFunctionCompile()
{
  typedef int VType;
  typedef concepts::Digraph Digraph;
  typedef Digraph::Arc Arc;
  typedef Digraph::Node Node;

  Digraph g;
  bool b;
  dfs(g).run(Node());
  b=dfs(g).run(Node(),Node());
  dfs(g).run();
  dfs(g)
    .predMap(concepts::ReadWriteMap<Node,Arc>())
    .distMap(concepts::ReadWriteMap<Node,VType>())
    .reachedMap(concepts::ReadWriteMap<Node,bool>())
    .processedMap(concepts::WriteMap<Node,bool>())
    .run(Node());
  b=dfs(g)
    .predMap(concepts::ReadWriteMap<Node,Arc>())
    .distMap(concepts::ReadWriteMap<Node,VType>())
    .reachedMap(concepts::ReadWriteMap<Node,bool>())
    .processedMap(concepts::WriteMap<Node,bool>())
    .path(concepts::Path<Digraph>())
    .dist(VType())
    .run(Node(),Node());
  dfs(g)
    .predMap(concepts::ReadWriteMap<Node,Arc>())
    .distMap(concepts::ReadWriteMap<Node,VType>())
    .reachedMap(concepts::ReadWriteMap<Node,bool>())
    .processedMap(concepts::WriteMap<Node,bool>())
    .run();
}

template <class Digraph>
void checkDfs() {
  TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

  Digraph G;
  Node s, t;

  std::istringstream input(test_lgf);
  digraphReader(input, G).
    node("source", s).
    node("target", t).
    run();

  Dfs<Digraph> dfs_test(G);
  dfs_test.run(s);

  Path<Digraph> p = dfs_test.path(t);
  check(p.length() == dfs_test.dist(t),"path() found a wrong path.");
  check(checkPath(G, p),"path() found a wrong path.");
  check(pathSource(G, p) == s,"path() found a wrong path.");
  check(pathTarget(G, p) == t,"path() found a wrong path.");

  for(NodeIt v(G); v!=INVALID; ++v) {
    if (dfs_test.reached(v)) {
      check(v==s || dfs_test.predArc(v)!=INVALID, "Wrong tree.");
      if (dfs_test.predArc(v)!=INVALID ) {
        Arc e=dfs_test.predArc(v);
        Node u=G.source(e);
        check(u==dfs_test.predNode(v),"Wrong tree.");
        check(dfs_test.dist(v) - dfs_test.dist(u) == 1,
              "Wrong distance. (" << dfs_test.dist(u) << "->"
              << dfs_test.dist(v) << ")");
      }
    }
  }

  {
    NullMap<Node,Arc> myPredMap;
    dfs(G).predMap(myPredMap).run(s);
  }
}

int main()
{
  checkDfs<ListDigraph>();
  checkDfs<SmartDigraph>();
  return 0;
}
