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

#include <sstream>

#include <lemon/smart_graph.h>
#include <lemon/hao_orlin.h>

#include <lemon/lgf_reader.h>
#include "test_tools.h"

using namespace lemon;
using namespace std;

const std::string lgf =
  "@nodes\n"
  "label\n"
  "0\n"
  "1\n"
  "2\n"
  "3\n"
  "4\n"
  "5\n"
  "@edges\n"
  "     label  capacity\n"
  "0 1  0      2\n"
  "1 2  1      2\n"
  "2 0  2      2\n"
  "3 4  3      2\n"
  "4 5  4      2\n"
  "5 3  5      2\n"
  "2 3  6      3\n";

int main() {
  SmartGraph graph;
  SmartGraph::EdgeMap<int> capacity(graph);

  istringstream lgfs(lgf);
  graphReader(graph, lgfs).
    edgeMap("capacity", capacity).run();

  HaoOrlin<SmartGraph, SmartGraph::EdgeMap<int> > ho(graph, capacity);
  ho.run();

  check(ho.minCutValue() == 3, "Wrong cut value");

  return 0;
}
