/* -*- mode: C++; indent-tabs-mode: nil; -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library.
 *
 * Copyright (C) 2003-2009
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

#include <lemon/euler.h>
#include <lemon/list_graph.h>
#include <test/test_tools.h>

using namespace lemon;

template <typename Digraph>
void checkDiEulerIt(const Digraph& g)
{
  typename Digraph::template ArcMap<int> visitationNumber(g, 0);

  DiEulerIt<Digraph> e(g);
  typename Digraph::Node firstNode = g.source(e);
  typename Digraph::Node lastNode = g.target(e);

  for (; e != INVALID; ++e)
  {
    if (e != INVALID)
    {
      lastNode = g.target(e);
    }
    ++visitationNumber[e];
  }

  check(firstNode == lastNode,
      "checkDiEulerIt: first and last node are not the same");

  for (typename Digraph::ArcIt a(g); a != INVALID; ++a)
  {
    check(visitationNumber[a] == 1,
        "checkDiEulerIt: not visited or multiple times visited arc found");
  }
}

template <typename Graph>
void checkEulerIt(const Graph& g)
{
  typename Graph::template EdgeMap<int> visitationNumber(g, 0);

  EulerIt<Graph> e(g);
  typename Graph::Node firstNode = g.u(e);
  typename Graph::Node lastNode = g.v(e);

  for (; e != INVALID; ++e)
  {
    if (e != INVALID)
    {
      lastNode = g.v(e);
    }
    ++visitationNumber[e];
  }

  check(firstNode == lastNode,
      "checkEulerIt: first and last node are not the same");

  for (typename Graph::EdgeIt e(g); e != INVALID; ++e)
  {
    check(visitationNumber[e] == 1,
        "checkEulerIt: not visited or multiple times visited edge found");
  }
}

int main()
{
  typedef ListDigraph Digraph;
  typedef ListGraph Graph;

  Digraph digraphWithEulerianCircuit;
  {
    Digraph& g = digraphWithEulerianCircuit;

    Digraph::Node n0 = g.addNode();
    Digraph::Node n1 = g.addNode();
    Digraph::Node n2 = g.addNode();

    g.addArc(n0, n1);
    g.addArc(n1, n0);
    g.addArc(n1, n2);
    g.addArc(n2, n1);
  }

  Digraph digraphWithoutEulerianCircuit;
  {
    Digraph& g = digraphWithoutEulerianCircuit;

    Digraph::Node n0 = g.addNode();
    Digraph::Node n1 = g.addNode();
    Digraph::Node n2 = g.addNode();

    g.addArc(n0, n1);
    g.addArc(n1, n0);
    g.addArc(n1, n2);
  }

  Graph graphWithEulerianCircuit;
  {
    Graph& g = graphWithEulerianCircuit;

    Graph::Node n0 = g.addNode();
    Graph::Node n1 = g.addNode();
    Graph::Node n2 = g.addNode();

    g.addEdge(n0, n1);
    g.addEdge(n1, n2);
    g.addEdge(n2, n0);
  }

  Graph graphWithoutEulerianCircuit;
  {
    Graph& g = graphWithoutEulerianCircuit;

    Graph::Node n0 = g.addNode();
    Graph::Node n1 = g.addNode();
    Graph::Node n2 = g.addNode();

    g.addEdge(n0, n1);
    g.addEdge(n1, n2);
  }

  checkDiEulerIt(digraphWithEulerianCircuit);

  checkEulerIt(graphWithEulerianCircuit);

  check(eulerian(digraphWithEulerianCircuit),
      "this graph should have an Eulerian circuit");
  check(!eulerian(digraphWithoutEulerianCircuit),
      "this graph should not have an Eulerian circuit");

  check(eulerian(graphWithEulerianCircuit),
      "this graph should have an Eulerian circuit");
  check(!eulerian(graphWithoutEulerianCircuit),
      "this graph should have an Eulerian circuit");

  return 0;
}
