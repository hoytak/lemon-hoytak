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

///\ingroup demos
///\file
///\brief Demonstrating the usage of LEMON's General Flow algorithm
///
/// This demo program reads a general network circulation problem from the
/// file 'circulation-input.lgf', runs the preflow based algorithm for
/// finding a feasible solution and writes the output
/// to 'circulation-input.lgf'
///
/// \include circulation_demo.cc

#include <iostream>

#include "test_tools.h"
#include <lemon/list_graph.h>
#include <lemon/circulation.h>
#include <lemon/lgf_reader.h>

using namespace lemon;

char test_lgf[] =
  "@nodes\n"
  "label delta\n"
  "0     0\n"
  "1     13\n"
  "2     0\n"
  "3     0\n"
  "4     0\n"
  "5     0\n"
  "6     0\n"
  "7     0\n"
  "8     -13\n"
  "9     0\n"
  "@edges\n"
  "    label lo_cap up_cap\n"
  "0 1 0     0      20\n"
  "0 2 1     0      0\n"
  "1 1 2     0      3\n"
  "1 2 3     0      8\n"
  "1 3 4     0      8\n"
  "2 5 5     0      5\n"
  "3 2 6     0      5\n"
  "3 5 7     0      5\n"
  "3 6 8     0      5\n"
  "4 3 9     0      3\n"
  "5 7 10    0      3\n"
  "5 6 11    0      10\n"
  "5 8 12    0      10\n"
  "6 8 13    0      8\n"
  "8 9 14    0      20\n"
  "8 1 15    0      5\n"
  "9 5 16    0      5\n"
  "@attributes\n"
  "source 1\n"
  "sink   8\n";

int main (int, char*[])
{

    typedef ListDigraph Digraph;
    typedef Digraph::Node Node;
    typedef Digraph::NodeIt NodeIt;
    typedef Digraph::Arc Arc;
    typedef Digraph::ArcIt ArcIt;
    typedef Digraph::ArcMap<int> ArcMap;
    typedef Digraph::NodeMap<int> NodeMap;
    typedef Digraph::NodeMap<double> DNodeMap;

    Digraph g;
    ArcMap lo(g);
    ArcMap up(g);
    NodeMap delta(g);
    NodeMap nid(g);
    ArcMap eid(g);
    Node source, sink;
    
    std::istringstream input(test_lgf);
    DigraphReader<Digraph>(g,input).
      arcMap("lo_cap", lo).
      arcMap("up_cap", up).
      nodeMap("delta", delta).
      arcMap("label", eid).
      nodeMap("label", nid).
      node("source",source).
      node("sink",sink).
      run();

    Circulation<Digraph> gen(g,lo,up,delta);
    bool ret=gen.run();
    check(ret,"A feasible solution should have been found.");
    check(gen.checkFlow(), "The found flow is corrupt.");
    
    delta[source]=14;
    delta[sink]=-14;
    
    bool ret2=gen.run();
    check(!ret2,"A feasible solution should not have been found.");
    check(gen.checkBarrier(), "The found barrier is corrupt.");

}
