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

#include <lemon/list_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/path.h>
#include <lemon/suurballe.h>

#include "test_tools.h"

using namespace lemon;

char test_lgf[] =
  "@nodes\n"
  "label supply1 supply2 supply3\n"
  "1     0        20      27\n"
  "2     0       -4        0\n"
  "3     0        0        0\n"
  "4     0        0        0\n"
  "5     0        9        0\n"
  "6     0       -6        0\n"
  "7     0        0        0\n"
  "8     0        0        0\n"
  "9     0        3        0\n"
  "10    0       -2        0\n"
  "11    0        0        0\n"
  "12    0       -20     -27\n"
  "@arcs\n"
  "      cost capacity lower1 lower2\n"
  " 1  2  70  11       0      8\n"
  " 1  3 150   3       0      1\n"
  " 1  4  80  15       0      2\n"
  " 2  8  80  12       0      0\n"
  " 3  5 140   5       0      3\n"
  " 4  6  60  10       0      1\n"
  " 4  7  80   2       0      0\n"
  " 4  8 110   3       0      0\n"
  " 5  7  60  14       0      0\n"
  " 5 11 120  12       0      0\n"
  " 6  3   0   3       0      0\n"
  " 6  9 140   4       0      0\n"
  " 6 10  90   8       0      0\n"
  " 7  1  30   5       0      0\n"
  " 8 12  60  16       0      4\n"
  " 9 12  50   6       0      0\n"
  "10 12  70  13       0      5\n"
  "10  2 100   7       0      0\n"
  "10  7  60  10       0      0\n"
  "11 10  20  14       0      6\n"
  "12 11  30  10       0      0\n"
  "@attributes\n"
  "source  1\n"
  "target 12\n"
  "@end\n";

// Check the feasibility of the flow
template <typename Digraph, typename FlowMap>
bool checkFlow( const Digraph& gr, const FlowMap& flow, 
                typename Digraph::Node s, typename Digraph::Node t,
                int value )
{
  TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
  for (ArcIt e(gr); e != INVALID; ++e)
    if (!(flow[e] == 0 || flow[e] == 1)) return false;

  for (NodeIt n(gr); n != INVALID; ++n) {
    int sum = 0;
    for (OutArcIt e(gr, n); e != INVALID; ++e)
      sum += flow[e];
    for (InArcIt e(gr, n); e != INVALID; ++e)
      sum -= flow[e];
    if (n == s && sum != value) return false;
    if (n == t && sum != -value) return false;
    if (n != s && n != t && sum != 0) return false;
  }

  return true;
}

// Check the optimalitiy of the flow
template < typename Digraph, typename CostMap, 
           typename FlowMap, typename PotentialMap >
bool checkOptimality( const Digraph& gr, const CostMap& cost,
                      const FlowMap& flow, const PotentialMap& pi )
{
  // Check the "Complementary Slackness" optimality condition
  TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
  bool opt = true;
  for (ArcIt e(gr); e != INVALID; ++e) {
    typename CostMap::Value red_cost =
      cost[e] + pi[gr.source(e)] - pi[gr.target(e)];
    opt = (flow[e] == 0 && red_cost >= 0) ||
          (flow[e] == 1 && red_cost <= 0);
    if (!opt) break;
  }
  return opt;
}

// Check a path
template <typename Digraph, typename Path>
bool checkPath( const Digraph& gr, const Path& path,
                typename Digraph::Node s, typename Digraph::Node t)
{
  // Check the "Complementary Slackness" optimality condition
  TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
  Node n = s;
  for (int i = 0; i < path.length(); ++i) {
    if (gr.source(path.nth(i)) != n) return false;
    n = gr.target(path.nth(i));
  }
  return n == t;
}


int main()
{
  DIGRAPH_TYPEDEFS(ListDigraph);

  // Read the test digraph
  ListDigraph digraph;
  ListDigraph::ArcMap<int> length(digraph);
  Node source, target;

  std::istringstream input(test_lgf);
  DigraphReader<ListDigraph>(digraph, input).
    arcMap("cost", length).
    node("source", source).
    node("target", target).
    run();
  
  // Find 2 paths
  {
    Suurballe<ListDigraph> suurballe(digraph, length, source, target);
    check(suurballe.run(2) == 2, "Wrong number of paths");
    check(checkFlow(digraph, suurballe.flowMap(), source, target, 2),
          "The flow is not feasible");
    check(suurballe.totalLength() == 510, "The flow is not optimal");
    check(checkOptimality(digraph, length, suurballe.flowMap(), 
                          suurballe.potentialMap()),
          "Wrong potentials");
    for (int i = 0; i < suurballe.pathNum(); ++i)
      check(checkPath(digraph, suurballe.path(i), source, target),
            "Wrong path");
  }

  // Find 3 paths
  {
    Suurballe<ListDigraph> suurballe(digraph, length, source, target);
    check(suurballe.run(3) == 3, "Wrong number of paths");
    check(checkFlow(digraph, suurballe.flowMap(), source, target, 3),
          "The flow is not feasible");
    check(suurballe.totalLength() == 1040, "The flow is not optimal");
    check(checkOptimality(digraph, length, suurballe.flowMap(), 
                          suurballe.potentialMap()),
          "Wrong potentials");
    for (int i = 0; i < suurballe.pathNum(); ++i)
      check(checkPath(digraph, suurballe.path(i), source, target),
            "Wrong path");
  }

  // Find 5 paths (only 3 can be found)
  {
    Suurballe<ListDigraph> suurballe(digraph, length, source, target);
    check(suurballe.run(5) == 3, "Wrong number of paths");
    check(checkFlow(digraph, suurballe.flowMap(), source, target, 3),
          "The flow is not feasible");
    check(suurballe.totalLength() == 1040, "The flow is not optimal");
    check(checkOptimality(digraph, length, suurballe.flowMap(), 
                          suurballe.potentialMap()),
          "Wrong potentials");
    for (int i = 0; i < suurballe.pathNum(); ++i)
      check(checkPath(digraph, suurballe.path(i), source, target),
            "Wrong path");
  }

  return 0;
}
