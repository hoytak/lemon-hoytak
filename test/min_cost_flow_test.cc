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
#include <lemon/smart_graph.h>
#include <lemon/lgf_reader.h>

//#include <lemon/cycle_canceling.h>
//#include <lemon/capacity_scaling.h>
//#include <lemon/cost_scaling.h>
#include <lemon/network_simplex.h>
//#include <lemon/min_cost_flow.h>
//#include <lemon/min_cost_max_flow.h>

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
template <typename GR, typename Value>
class McfClassConcept
{
public:

  template <typename MCF>
  struct Constraints {
    void constraints() {
      checkConcept<concepts::Digraph, GR>();

      MCF mcf_test1(g, lower, upper, cost, sup);
      MCF mcf_test2(g, upper, cost, sup);
      MCF mcf_test3(g, lower, upper, cost, n, n, k);
      MCF mcf_test4(g, upper, cost, n, n, k);

      // TODO: This part should be enabled and the next part
      // should be removed if map copying is supported
/*
      flow = mcf_test1.flowMap();
      mcf_test1.flowMap(flow);

      pot = mcf_test1.potentialMap();
      mcf_test1.potentialMap(pot);
*/
/**/
      const typename MCF::FlowMap &fm =
        mcf_test1.flowMap();
      mcf_test1.flowMap(flow);
      const typename MCF::PotentialMap &pm =
        mcf_test1.potentialMap();
      mcf_test1.potentialMap(pot);
      ignore_unused_variable_warning(fm);
      ignore_unused_variable_warning(pm);
/**/

      mcf_test1.run();

      v = mcf_test1.totalCost();
      v = mcf_test1.flow(a);
      v = mcf_test1.potential(n);
    }

    typedef typename GR::Node Node;
    typedef typename GR::Arc Arc;
    typedef concepts::ReadMap<Node, Value> NM;
    typedef concepts::ReadMap<Arc, Value> AM;

    const GR &g;
    const AM &lower;
    const AM &upper;
    const AM &cost;
    const NM &sup;
    const Node &n;
    const Arc &a;
    const Value &k;
    Value v;

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
// using the Complementary Slackness optimality condition
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
    typedef int Value;
    // This typedef should be enabled if the standard maps are
    // reference maps in the graph concepts
    //typedef concepts::Digraph GR;
    typedef ListDigraph GR;
    typedef concepts::ReadMap<GR::Node, Value> NM;
    typedef concepts::ReadMap<GR::Arc, Value> AM;

    //checkConcept< McfClassConcept<GR, Value>,
    //              CycleCanceling<GR, AM, AM, AM, NM> >();
    //checkConcept< McfClassConcept<GR, Value>,
    //              CapacityScaling<GR, AM, AM, AM, NM> >();
    //checkConcept< McfClassConcept<GR, Value>,
    //              CostScaling<GR, AM, AM, AM, NM> >();
    checkConcept< McfClassConcept<GR, Value>,
                  NetworkSimplex<GR, AM, AM, AM, NM> >();
    //checkConcept< MinCostFlow<GR, Value>,
    //              NetworkSimplex<GR, AM, AM, AM, NM> >();
  }

  // Run various MCF tests
  typedef ListDigraph Digraph;
  DIGRAPH_TYPEDEFS(ListDigraph);

  // Read the test digraph
  Digraph gr;
  Digraph::ArcMap<int> c(gr), l1(gr), l2(gr), u(gr);
  Digraph::NodeMap<int> s1(gr), s2(gr), s3(gr);
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

/*
  // A. Test CapacityScaling with scaling
  {
    CapacityScaling<Digraph> mcf1(gr, u, c, s1);
    CapacityScaling<Digraph> mcf2(gr, u, c, v, w, 27);
    CapacityScaling<Digraph> mcf3(gr, u, c, s3);
    CapacityScaling<Digraph> mcf4(gr, l2, u, c, s1);
    CapacityScaling<Digraph> mcf5(gr, l2, u, c, v, w, 27);
    CapacityScaling<Digraph> mcf6(gr, l2, u, c, s3);

    checkMcf(mcf1, mcf1.run(), gr, l1, u, c, s1, true,  5240, "#A1");
    checkMcf(mcf2, mcf2.run(), gr, l1, u, c, s2, true,  7620, "#A2");
    checkMcf(mcf3, mcf3.run(), gr, l1, u, c, s3, true,     0, "#A3");
    checkMcf(mcf4, mcf4.run(), gr, l2, u, c, s1, true,  5970, "#A4");
    checkMcf(mcf5, mcf5.run(), gr, l2, u, c, s2, true,  8010, "#A5");
    checkMcf(mcf6, mcf6.run(), gr, l2, u, c, s3, false,    0, "#A6");
  }

  // B. Test CapacityScaling without scaling
  {
    CapacityScaling<Digraph> mcf1(gr, u, c, s1);
    CapacityScaling<Digraph> mcf2(gr, u, c, v, w, 27);
    CapacityScaling<Digraph> mcf3(gr, u, c, s3);
    CapacityScaling<Digraph> mcf4(gr, l2, u, c, s1);
    CapacityScaling<Digraph> mcf5(gr, l2, u, c, v, w, 27);
    CapacityScaling<Digraph> mcf6(gr, l2, u, c, s3);

    checkMcf(mcf1, mcf1.run(false), gr, l1, u, c, s1, true,  5240, "#B1");
    checkMcf(mcf2, mcf2.run(false), gr, l1, u, c, s2, true,  7620, "#B2");
    checkMcf(mcf3, mcf3.run(false), gr, l1, u, c, s3, true,     0, "#B3");
    checkMcf(mcf4, mcf4.run(false), gr, l2, u, c, s1, true,  5970, "#B4");
    checkMcf(mcf5, mcf5.run(false), gr, l2, u, c, s2, true,  8010, "#B5");
    checkMcf(mcf6, mcf6.run(false), gr, l2, u, c, s3, false,    0, "#B6");
  }

  // C. Test CostScaling using partial augment-relabel method
  {
    CostScaling<Digraph> mcf1(gr, u, c, s1);
    CostScaling<Digraph> mcf2(gr, u, c, v, w, 27);
    CostScaling<Digraph> mcf3(gr, u, c, s3);
    CostScaling<Digraph> mcf4(gr, l2, u, c, s1);
    CostScaling<Digraph> mcf5(gr, l2, u, c, v, w, 27);
    CostScaling<Digraph> mcf6(gr, l2, u, c, s3);

    checkMcf(mcf1, mcf1.run(), gr, l1, u, c, s1, true,  5240, "#C1");
    checkMcf(mcf2, mcf2.run(), gr, l1, u, c, s2, true,  7620, "#C2");
    checkMcf(mcf3, mcf3.run(), gr, l1, u, c, s3, true,     0, "#C3");
    checkMcf(mcf4, mcf4.run(), gr, l2, u, c, s1, true,  5970, "#C4");
    checkMcf(mcf5, mcf5.run(), gr, l2, u, c, s2, true,  8010, "#C5");
    checkMcf(mcf6, mcf6.run(), gr, l2, u, c, s3, false,    0, "#C6");
  }

  // D. Test CostScaling using push-relabel method
  {
    CostScaling<Digraph> mcf1(gr, u, c, s1);
    CostScaling<Digraph> mcf2(gr, u, c, v, w, 27);
    CostScaling<Digraph> mcf3(gr, u, c, s3);
    CostScaling<Digraph> mcf4(gr, l2, u, c, s1);
    CostScaling<Digraph> mcf5(gr, l2, u, c, v, w, 27);
    CostScaling<Digraph> mcf6(gr, l2, u, c, s3);

    checkMcf(mcf1, mcf1.run(false), gr, l1, u, c, s1, true,  5240, "#D1");
    checkMcf(mcf2, mcf2.run(false), gr, l1, u, c, s2, true,  7620, "#D2");
    checkMcf(mcf3, mcf3.run(false), gr, l1, u, c, s3, true,     0, "#D3");
    checkMcf(mcf4, mcf4.run(false), gr, l2, u, c, s1, true,  5970, "#D4");
    checkMcf(mcf5, mcf5.run(false), gr, l2, u, c, s2, true,  8010, "#D5");
    checkMcf(mcf6, mcf6.run(false), gr, l2, u, c, s3, false,    0, "#D6");
  }
*/

  // E. Test NetworkSimplex with FIRST_ELIGIBLE_PIVOT
  {
    NetworkSimplex<Digraph>::PivotRuleEnum pr =
      NetworkSimplex<Digraph>::FIRST_ELIGIBLE_PIVOT;
    NetworkSimplex<Digraph> mcf1(gr, u, c, s1);
    NetworkSimplex<Digraph> mcf2(gr, u, c, v, w, 27);
    NetworkSimplex<Digraph> mcf3(gr, u, c, s3);
    NetworkSimplex<Digraph> mcf4(gr, l2, u, c, s1);
    NetworkSimplex<Digraph> mcf5(gr, l2, u, c, v, w, 27);
    NetworkSimplex<Digraph> mcf6(gr, l2, u, c, s3);

    checkMcf(mcf1, mcf1.run(pr), gr, l1, u, c, s1, true,  5240, "#E1");
    checkMcf(mcf2, mcf2.run(pr), gr, l1, u, c, s2, true,  7620, "#E2");
    checkMcf(mcf3, mcf3.run(pr), gr, l1, u, c, s3, true,     0, "#E3");
    checkMcf(mcf4, mcf4.run(pr), gr, l2, u, c, s1, true,  5970, "#E4");
    checkMcf(mcf5, mcf5.run(pr), gr, l2, u, c, s2, true,  8010, "#E5");
    checkMcf(mcf6, mcf6.run(pr), gr, l2, u, c, s3, false,    0, "#E6");
  }

  // F. Test NetworkSimplex with BEST_ELIGIBLE_PIVOT
  {
    NetworkSimplex<Digraph>::PivotRuleEnum pr =
      NetworkSimplex<Digraph>::BEST_ELIGIBLE_PIVOT;
    NetworkSimplex<Digraph> mcf1(gr, u, c, s1);
    NetworkSimplex<Digraph> mcf2(gr, u, c, v, w, 27);
    NetworkSimplex<Digraph> mcf3(gr, u, c, s3);
    NetworkSimplex<Digraph> mcf4(gr, l2, u, c, s1);
    NetworkSimplex<Digraph> mcf5(gr, l2, u, c, v, w, 27);
    NetworkSimplex<Digraph> mcf6(gr, l2, u, c, s3);

    checkMcf(mcf1, mcf1.run(pr), gr, l1, u, c, s1, true,  5240, "#F1");
    checkMcf(mcf2, mcf2.run(pr), gr, l1, u, c, s2, true,  7620, "#F2");
    checkMcf(mcf3, mcf3.run(pr), gr, l1, u, c, s3, true,     0, "#F3");
    checkMcf(mcf4, mcf4.run(pr), gr, l2, u, c, s1, true,  5970, "#F4");
    checkMcf(mcf5, mcf5.run(pr), gr, l2, u, c, s2, true,  8010, "#F5");
    checkMcf(mcf6, mcf6.run(pr), gr, l2, u, c, s3, false,    0, "#F6");
  }

  // G. Test NetworkSimplex with BLOCK_SEARCH_PIVOT
  {
    NetworkSimplex<Digraph>::PivotRuleEnum pr =
      NetworkSimplex<Digraph>::BLOCK_SEARCH_PIVOT;
    NetworkSimplex<Digraph> mcf1(gr, u, c, s1);
    NetworkSimplex<Digraph> mcf2(gr, u, c, v, w, 27);
    NetworkSimplex<Digraph> mcf3(gr, u, c, s3);
    NetworkSimplex<Digraph> mcf4(gr, l2, u, c, s1);
    NetworkSimplex<Digraph> mcf5(gr, l2, u, c, v, w, 27);
    NetworkSimplex<Digraph> mcf6(gr, l2, u, c, s3);

    checkMcf(mcf1, mcf1.run(pr), gr, l1, u, c, s1, true,  5240, "#G1");
    checkMcf(mcf2, mcf2.run(pr), gr, l1, u, c, s2, true,  7620, "#G2");
    checkMcf(mcf3, mcf3.run(pr), gr, l1, u, c, s3, true,     0, "#G3");
    checkMcf(mcf4, mcf4.run(pr), gr, l2, u, c, s1, true,  5970, "#G4");
    checkMcf(mcf5, mcf5.run(pr), gr, l2, u, c, s2, true,  8010, "#G5");
    checkMcf(mcf6, mcf6.run(pr), gr, l2, u, c, s3, false,    0, "#G6");
  }

  // H. Test NetworkSimplex with CANDIDATE_LIST_PIVOT
  {
    NetworkSimplex<Digraph>::PivotRuleEnum pr =
      NetworkSimplex<Digraph>::CANDIDATE_LIST_PIVOT;
    NetworkSimplex<Digraph> mcf1(gr, u, c, s1);
    NetworkSimplex<Digraph> mcf2(gr, u, c, v, w, 27);
    NetworkSimplex<Digraph> mcf3(gr, u, c, s3);
    NetworkSimplex<Digraph> mcf4(gr, l2, u, c, s1);
    NetworkSimplex<Digraph> mcf5(gr, l2, u, c, v, w, 27);
    NetworkSimplex<Digraph> mcf6(gr, l2, u, c, s3);

    checkMcf(mcf1, mcf1.run(pr), gr, l1, u, c, s1, true,  5240, "#H1");
    checkMcf(mcf2, mcf2.run(pr), gr, l1, u, c, s2, true,  7620, "#H2");
    checkMcf(mcf3, mcf3.run(pr), gr, l1, u, c, s3, true,     0, "#H3");
    checkMcf(mcf4, mcf4.run(pr), gr, l2, u, c, s1, true,  5970, "#H4");
    checkMcf(mcf5, mcf5.run(pr), gr, l2, u, c, s2, true,  8010, "#H5");
    checkMcf(mcf6, mcf6.run(pr), gr, l2, u, c, s3, false,    0, "#H6");
  }

  // I. Test NetworkSimplex with ALTERING_LIST_PIVOT
  {
    NetworkSimplex<Digraph>::PivotRuleEnum pr =
      NetworkSimplex<Digraph>::ALTERING_LIST_PIVOT;
    NetworkSimplex<Digraph> mcf1(gr, u, c, s1);
    NetworkSimplex<Digraph> mcf2(gr, u, c, v, w, 27);
    NetworkSimplex<Digraph> mcf3(gr, u, c, s3);
    NetworkSimplex<Digraph> mcf4(gr, l2, u, c, s1);
    NetworkSimplex<Digraph> mcf5(gr, l2, u, c, v, w, 27);
    NetworkSimplex<Digraph> mcf6(gr, l2, u, c, s3);

    checkMcf(mcf1, mcf1.run(pr), gr, l1, u, c, s1, true,  5240, "#I1");
    checkMcf(mcf2, mcf2.run(pr), gr, l1, u, c, s2, true,  7620, "#I2");
    checkMcf(mcf3, mcf3.run(pr), gr, l1, u, c, s3, true,     0, "#I3");
    checkMcf(mcf4, mcf4.run(pr), gr, l2, u, c, s1, true,  5970, "#I4");
    checkMcf(mcf5, mcf5.run(pr), gr, l2, u, c, s2, true,  8010, "#I5");
    checkMcf(mcf6, mcf6.run(pr), gr, l2, u, c, s3, false,    0, "#I6");
  }

/*
  // J. Test MinCostFlow
  {
    MinCostFlow<Digraph> mcf1(gr, u, c, s1);
    MinCostFlow<Digraph> mcf2(gr, u, c, v, w, 27);
    MinCostFlow<Digraph> mcf3(gr, u, c, s3);
    MinCostFlow<Digraph> mcf4(gr, l2, u, c, s1);
    MinCostFlow<Digraph> mcf5(gr, l2, u, c, v, w, 27);
    MinCostFlow<Digraph> mcf6(gr, l2, u, c, s3);

    checkMcf(mcf1, mcf1.run(), gr, l1, u, c, s1, true,  5240, "#J1");
    checkMcf(mcf2, mcf2.run(), gr, l1, u, c, s2, true,  7620, "#J2");
    checkMcf(mcf3, mcf3.run(), gr, l1, u, c, s3, true,     0, "#J3");
    checkMcf(mcf4, mcf4.run(), gr, l2, u, c, s1, true,  5970, "#J4");
    checkMcf(mcf5, mcf5.run(), gr, l2, u, c, s2, true,  8010, "#J5");
    checkMcf(mcf6, mcf6.run(), gr, l2, u, c, s3, false,    0, "#J6");
  }
*/
/*
  // K. Test MinCostMaxFlow
  {
    MinCostMaxFlow<Digraph> mcmf(gr, u, c, v, w);
    mcmf.run();
    checkMcf(mcmf, true, gr, l1, u, c, s3, true, 7620, "#K1");
  }
*/

  return 0;
}
