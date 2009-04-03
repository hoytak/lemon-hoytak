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

#include <iostream>
#include <fstream>

#include <lemon/list_graph.h>
#include <lemon/lgf_reader.h>

#include <lemon/network_simplex.h>

#include <lemon/concepts/digraph.h>
#include <lemon/concept_check.h>

#include "test_tools.h"

using namespace lemon;

char test_lgf[] =
  "@nodes\n"
  "label  sup1 sup2 sup3\n"
  "    1    20   27    0\n"
  "    2    -4    0    0\n"
  "    3     0    0    0\n"
  "    4     0    0    0\n"
  "    5     9    0    0\n"
  "    6    -6    0    0\n"
  "    7     0    0    0\n"
  "    8     0    0    0\n"
  "    9     3    0    0\n"
  "   10    -2    0    0\n"
  "   11     0    0    0\n"
  "   12   -20  -27    0\n"
  "\n"
  "@arcs\n"
  "       cost  cap low1 low2\n"
  " 1  2    70   11    0    8\n"
  " 1  3   150    3    0    1\n"
  " 1  4    80   15    0    2\n"
  " 2  8    80   12    0    0\n"
  " 3  5   140    5    0    3\n"
  " 4  6    60   10    0    1\n"
  " 4  7    80    2    0    0\n"
  " 4  8   110    3    0    0\n"
  " 5  7    60   14    0    0\n"
  " 5 11   120   12    0    0\n"
  " 6  3     0    3    0    0\n"
  " 6  9   140    4    0    0\n"
  " 6 10    90    8    0    0\n"
  " 7  1    30    5    0    0\n"
  " 8 12    60   16    0    4\n"
  " 9 12    50    6    0    0\n"
  "10 12    70   13    0    5\n"
  "10  2   100    7    0    0\n"
  "10  7    60   10    0    0\n"
  "11 10    20   14    0    6\n"
  "12 11    30   10    0    0\n"
  "\n"
  "@attributes\n"
  "source 1\n"
  "target 12\n";


// Check the interface of an MCF algorithm
template <typename GR, typename Flow, typename Cost>
class McfClassConcept
{
public:

  template <typename MCF>
  struct Constraints {
    void constraints() {
      checkConcept<concepts::Digraph, GR>();

      MCF mcf(g);

      b = mcf.reset()
             .lowerMap(lower)
             .upperMap(upper)
             .capacityMap(upper)
             .boundMaps(lower, upper)
             .costMap(cost)
             .supplyMap(sup)
             .stSupply(n, n, k)
             .run();

      const typename MCF::FlowMap &fm = mcf.flowMap();
      const typename MCF::PotentialMap &pm = mcf.potentialMap();

      v = mcf.totalCost();
      double x = mcf.template totalCost<double>();
      v = mcf.flow(a);
      v = mcf.potential(n);
      mcf.flowMap(flow);
      mcf.potentialMap(pot);

      ignore_unused_variable_warning(fm);
      ignore_unused_variable_warning(pm);
      ignore_unused_variable_warning(x);
    }

    typedef typename GR::Node Node;
    typedef typename GR::Arc Arc;
    typedef concepts::ReadMap<Node, Flow> NM;
    typedef concepts::ReadMap<Arc, Flow> FAM;
    typedef concepts::ReadMap<Arc, Cost> CAM;

    const GR &g;
    const FAM &lower;
    const FAM &upper;
    const CAM &cost;
    const NM &sup;
    const Node &n;
    const Arc &a;
    const Flow &k;
    Flow v;
    bool b;

    typename MCF::FlowMap &flow;
    typename MCF::PotentialMap &pot;
  };

};


// Check the feasibility of the given flow (primal soluiton)
template < typename GR, typename LM, typename UM,
           typename SM, typename FM >
bool checkFlow( const GR& gr, const LM& lower, const UM& upper,
                const SM& supply, const FM& flow )
{
  TEMPLATE_DIGRAPH_TYPEDEFS(GR);

  for (ArcIt e(gr); e != INVALID; ++e) {
    if (flow[e] < lower[e] || flow[e] > upper[e]) return false;
  }

  for (NodeIt n(gr); n != INVALID; ++n) {
    typename SM::Value sum = 0;
    for (OutArcIt e(gr, n); e != INVALID; ++e)
      sum += flow[e];
    for (InArcIt e(gr, n); e != INVALID; ++e)
      sum -= flow[e];
    if (sum != supply[n]) return false;
  }

  return true;
}

// Check the feasibility of the given potentials (dual soluiton)
// using the "Complementary Slackness" optimality condition
template < typename GR, typename LM, typename UM,
           typename CM, typename FM, typename PM >
bool checkPotential( const GR& gr, const LM& lower, const UM& upper,
                     const CM& cost, const FM& flow, const PM& pi )
{
  TEMPLATE_DIGRAPH_TYPEDEFS(GR);

  bool opt = true;
  for (ArcIt e(gr); opt && e != INVALID; ++e) {
    typename CM::Value red_cost =
      cost[e] + pi[gr.source(e)] - pi[gr.target(e)];
    opt = red_cost == 0 ||
          (red_cost > 0 && flow[e] == lower[e]) ||
          (red_cost < 0 && flow[e] == upper[e]);
  }
  return opt;
}

// Run a minimum cost flow algorithm and check the results
template < typename MCF, typename GR,
           typename LM, typename UM,
           typename CM, typename SM >
void checkMcf( const MCF& mcf, bool mcf_result,
               const GR& gr, const LM& lower, const UM& upper,
               const CM& cost, const SM& supply,
               bool result, typename CM::Value total,
               const std::string &test_id = "" )
{
  check(mcf_result == result, "Wrong result " + test_id);
  if (result) {
    check(checkFlow(gr, lower, upper, supply, mcf.flowMap()),
          "The flow is not feasible " + test_id);
    check(mcf.totalCost() == total, "The flow is not optimal " + test_id);
    check(checkPotential(gr, lower, upper, cost, mcf.flowMap(),
                         mcf.potentialMap()),
          "Wrong potentials " + test_id);
  }
}

int main()
{
  // Check the interfaces
  {
    typedef int Flow;
    typedef int Cost;
    // TODO: This typedef should be enabled if the standard maps are
    // reference maps in the graph concepts (See #190).
/**/
    //typedef concepts::Digraph GR;
    typedef ListDigraph GR;
/**/
    checkConcept< McfClassConcept<GR, Flow, Cost>,
                  NetworkSimplex<GR, Flow, Cost> >();
  }

  // Run various MCF tests
  typedef ListDigraph Digraph;
  DIGRAPH_TYPEDEFS(ListDigraph);

  // Read the test digraph
  Digraph gr;
  Digraph::ArcMap<int> c(gr), l1(gr), l2(gr), u(gr);
  Digraph::NodeMap<int> s1(gr), s2(gr), s3(gr);
  ConstMap<Arc, int> cc(1), cu(std::numeric_limits<int>::max());
  Node v, w;

  std::istringstream input(test_lgf);
  DigraphReader<Digraph>(gr, input)
    .arcMap("cost", c)
    .arcMap("cap", u)
    .arcMap("low1", l1)
    .arcMap("low2", l2)
    .nodeMap("sup1", s1)
    .nodeMap("sup2", s2)
    .nodeMap("sup3", s3)
    .node("source", v)
    .node("target", w)
    .run();

  // A. Test NetworkSimplex with the default pivot rule
  {
    NetworkSimplex<Digraph> mcf(gr);

    mcf.upperMap(u).costMap(c);
    checkMcf(mcf, mcf.supplyMap(s1).run(),
             gr, l1, u, c, s1, true,  5240, "#A1");
    checkMcf(mcf, mcf.stSupply(v, w, 27).run(),
             gr, l1, u, c, s2, true,  7620, "#A2");
    mcf.lowerMap(l2);
    checkMcf(mcf, mcf.supplyMap(s1).run(),
             gr, l2, u, c, s1, true,  5970, "#A3");
    checkMcf(mcf, mcf.stSupply(v, w, 27).run(),
             gr, l2, u, c, s2, true,  8010, "#A4");
    mcf.reset();
    checkMcf(mcf, mcf.supplyMap(s1).run(),
             gr, l1, cu, cc, s1, true,  74, "#A5");
    checkMcf(mcf, mcf.lowerMap(l2).stSupply(v, w, 27).run(),
             gr, l2, cu, cc, s2, true,  94, "#A6");
    mcf.reset();
    checkMcf(mcf, mcf.run(),
             gr, l1, cu, cc, s3, true,   0, "#A7");
    checkMcf(mcf, mcf.boundMaps(l2, u).run(),
             gr, l2, u, cc, s3, false,   0, "#A8");
  }

  // B. Test NetworkSimplex with each pivot rule
  {
    NetworkSimplex<Digraph> mcf(gr);
    mcf.supplyMap(s1).costMap(c).capacityMap(u).lowerMap(l2);

    checkMcf(mcf, mcf.run(NetworkSimplex<Digraph>::FIRST_ELIGIBLE),
             gr, l2, u, c, s1, true,  5970, "#B1");
    checkMcf(mcf, mcf.run(NetworkSimplex<Digraph>::BEST_ELIGIBLE),
             gr, l2, u, c, s1, true,  5970, "#B2");
    checkMcf(mcf, mcf.run(NetworkSimplex<Digraph>::BLOCK_SEARCH),
             gr, l2, u, c, s1, true,  5970, "#B3");
    checkMcf(mcf, mcf.run(NetworkSimplex<Digraph>::CANDIDATE_LIST),
             gr, l2, u, c, s1, true,  5970, "#B4");
    checkMcf(mcf, mcf.run(NetworkSimplex<Digraph>::ALTERING_LIST),
             gr, l2, u, c, s1, true,  5970, "#B5");
  }

  return 0;
}
