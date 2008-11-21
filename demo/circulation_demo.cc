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

#include <lemon/list_graph.h>
#include <lemon/circulation.h>
#include <lemon/lgf_reader.h>
#include <lemon/lgf_writer.h>

using namespace lemon;


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
    DNodeMap cx(g);
    DNodeMap cy(g);

    DigraphReader<Digraph>(g,"circulation-input.lgf").
      arcMap("lo_cap", lo).
      arcMap("up_cap", up).
      nodeMap("delta", delta).
      arcMap("label", eid).
      nodeMap("label", nid).
      nodeMap("coordinates_x", cx).
      nodeMap("coordinates_y", cy).
      run();

    Circulation<Digraph> gen(g,lo,up,delta);
    bool ret=gen.run();
    if(ret)
      {
        std::cout << "\nA feasible flow has been found.\n";
        if(!gen.checkFlow()) std::cerr << "Oops!!!\n";
        DigraphWriter<Digraph>(g, "circulation-output.lgf").
          arcMap("lo_cap", lo).
          arcMap("up_cap", up).
          arcMap("flow", gen.flowMap()).
          nodeMap("delta", delta).
          arcMap("label", eid).
          nodeMap("label", nid).
          nodeMap("coordinates_x", cx).
          nodeMap("coordinates_y", cy).
          run();
      }
    else {
      std::cout << "\nThere is no such a flow\n";
      Digraph::NodeMap<int> bar(g);
      gen.barrierMap(bar);
      if(!gen.checkBarrier()) std::cerr << "Dual Oops!!!\n";

      DigraphWriter<Digraph>(g, "circulation-output.lgf").
        arcMap("lo_cap", lo).
        arcMap("up_cap", up).
        nodeMap("barrier", bar).
        nodeMap("delta", delta).
        arcMap("label", eid).
        nodeMap("label", nid).
        nodeMap("coordinates_x", cx).
        nodeMap("coordinates_y", cy).
        run();
    }
  std::cout << "The output is written to file 'circulation-output.lgf'\n\n";

}
