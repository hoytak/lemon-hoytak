/* -*- mode: C++; indent-tabs-mode: nil; -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library.
 *
 * Copyright (C) 2003-2010
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

#include <sstream>
#include <lemon/list_graph.h>
#include <lemon/full_graph.h>
#include <lemon/grid_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/grosso_locatelli_pullan_mc.h>

#include "test_tools.h"

using namespace lemon;

char test_lgf[] =
  "@nodes\n"
  "label max_clique\n"
  "1     0\n"
  "2     0\n"
  "3     0\n"
  "4     1\n"
  "5     1\n"
  "6     1\n"
  "7     1\n"
  "@edges\n"
  "    label\n"
  "1 2     1\n"
  "1 3     2\n"
  "1 4     3\n"
  "1 6     4\n"
  "2 3     5\n"
  "2 5     6\n"
  "2 7     7\n"
  "3 4     8\n"
  "3 5     9\n"
  "4 5    10\n"
  "4 6    11\n"
  "4 7    12\n"
  "5 6    13\n"
  "5 7    14\n"
  "6 7    15\n";
      

// Check with general graphs
template <typename Param>
void checkMaxCliqueGeneral(int max_sel, Param rule) {
  typedef ListGraph GR;
  typedef GrossoLocatelliPullanMc<GR> McAlg;
  typedef McAlg::CliqueNodeIt CliqueIt;
  
  // Basic tests
  {
    GR g;
    GR::NodeMap<bool> map(g);
    McAlg mc(g);
    check(mc.run(max_sel, rule) == 0, "Wrong clique size");
    check(mc.cliqueSize() == 0, "Wrong clique size");
    check(CliqueIt(mc) == INVALID, "Wrong CliqueNodeIt");

    GR::Node u = g.addNode();
    check(mc.run(max_sel, rule) == 1, "Wrong clique size");
    check(mc.cliqueSize() == 1, "Wrong clique size");
    mc.cliqueMap(map);
    check(map[u], "Wrong clique map");
    CliqueIt it1(mc);
    check(static_cast<GR::Node>(it1) == u && ++it1 == INVALID,
          "Wrong CliqueNodeIt");
    
    GR::Node v = g.addNode();
    check(mc.run(max_sel, rule) == 1, "Wrong clique size");
    check(mc.cliqueSize() == 1, "Wrong clique size");
    mc.cliqueMap(map);
    check((map[u] && !map[v]) || (map[v] && !map[u]), "Wrong clique map");
    CliqueIt it2(mc);
    check(it2 != INVALID && ++it2 == INVALID, "Wrong CliqueNodeIt");

    g.addEdge(u, v);
    check(mc.run(max_sel, rule) == 2, "Wrong clique size");
    check(mc.cliqueSize() == 2, "Wrong clique size");
    mc.cliqueMap(map);
    check(map[u] && map[v], "Wrong clique map");
    CliqueIt it3(mc);
    check(it3 != INVALID && ++it3 != INVALID && ++it3 == INVALID,
          "Wrong CliqueNodeIt");
  }

  // Test graph
  {
    GR g;
    GR::NodeMap<bool> max_clique(g);
    GR::NodeMap<bool> map(g);
    std::istringstream input(test_lgf);
    graphReader(g, input)
      .nodeMap("max_clique", max_clique)
      .run();
    
    McAlg mc(g);
    check(mc.run(max_sel, rule) == 4, "Wrong clique size");
    check(mc.cliqueSize() == 4, "Wrong clique size");
    mc.cliqueMap(map);
    for (GR::NodeIt n(g); n != INVALID; ++n) {
      check(map[n] == max_clique[n], "Wrong clique map");
    }
    int cnt = 0;
    for (CliqueIt n(mc); n != INVALID; ++n) {
      cnt++;
      check(map[n] && max_clique[n], "Wrong CliqueNodeIt");
    }
    check(cnt == 4, "Wrong CliqueNodeIt");
  }
}

// Check with full graphs
template <typename Param>
void checkMaxCliqueFullGraph(int max_sel, Param rule) {
  typedef FullGraph GR;
  typedef GrossoLocatelliPullanMc<FullGraph> McAlg;
  typedef McAlg::CliqueNodeIt CliqueIt;
  
  for (int size = 0; size <= 40; size = size * 3 + 1) {
    GR g(size);
    GR::NodeMap<bool> map(g);
    McAlg mc(g);
    check(mc.run(max_sel, rule) == size, "Wrong clique size");
    check(mc.cliqueSize() == size, "Wrong clique size");
    mc.cliqueMap(map);
    for (GR::NodeIt n(g); n != INVALID; ++n) {
      check(map[n], "Wrong clique map");
    }
    int cnt = 0;
    for (CliqueIt n(mc); n != INVALID; ++n) cnt++;
    check(cnt == size, "Wrong CliqueNodeIt");
  }
}

// Check with grid graphs
template <typename Param>
void checkMaxCliqueGridGraph(int max_sel, Param rule) {
  GridGraph g(5, 7);
  GridGraph::NodeMap<char> map(g);
  GrossoLocatelliPullanMc<GridGraph> mc(g);
  check(mc.run(max_sel, rule) == 2, "Wrong clique size");
  check(mc.cliqueSize() == 2, "Wrong clique size");
}


int main() {
  checkMaxCliqueGeneral(50, GrossoLocatelliPullanMc<ListGraph>::RANDOM);
  checkMaxCliqueGeneral(50, GrossoLocatelliPullanMc<ListGraph>::DEGREE_BASED);
  checkMaxCliqueGeneral(50, GrossoLocatelliPullanMc<ListGraph>::PENALTY_BASED);

  checkMaxCliqueFullGraph(50, GrossoLocatelliPullanMc<FullGraph>::RANDOM);
  checkMaxCliqueFullGraph(50, GrossoLocatelliPullanMc<FullGraph>::DEGREE_BASED);
  checkMaxCliqueFullGraph(50, GrossoLocatelliPullanMc<FullGraph>::PENALTY_BASED);
                       
  checkMaxCliqueGridGraph(50, GrossoLocatelliPullanMc<GridGraph>::RANDOM);
  checkMaxCliqueGridGraph(50, GrossoLocatelliPullanMc<GridGraph>::DEGREE_BASED);
  checkMaxCliqueGridGraph(50, GrossoLocatelliPullanMc<GridGraph>::PENALTY_BASED);
                       
  return 0;
}
