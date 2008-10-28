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
#include <fstream>

#include <lemon/list_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/path.h>
#include <lemon/suurballe.h>

#include "test_tools.h"

using namespace lemon;

// Checks the feasibility of the flow
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

// Checks the optimalitiy of the flow
template < typename Digraph, typename CostMap, 
           typename FlowMap, typename PotentialMap >
bool checkOptimality( const Digraph& gr, const CostMap& cost,
                      const FlowMap& flow, const PotentialMap& pi )
{
  // Checking the Complementary Slackness optimality condition
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

// Checks a path
template < typename Digraph, typename Path >
bool checkPath( const Digraph& gr, const Path& path,
                typename Digraph::Node s, typename Digraph::Node t)
{
  // Checking the Complementary Slackness optimality condition
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

  // Reading the test digraph
  ListDigraph digraph;
  ListDigraph::ArcMap<int> length(digraph);
  Node source, target;

  std::string fname;
  if(getenv("srcdir"))
    fname = std::string(getenv("srcdir"));
  else fname = ".";
  fname += "/test/min_cost_flow_test.lgf";

  std::ifstream input(fname.c_str());
  check(input, "Input file '" << fname << "' not found");
  DigraphReader<ListDigraph>(digraph, input).
    arcMap("cost", length).
    node("source", source).
    node("target", target).
    run();
  input.close();
  
  // Finding 2 paths
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

  // Finding 3 paths
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

  // Finding 5 paths (only 3 can be found)
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
