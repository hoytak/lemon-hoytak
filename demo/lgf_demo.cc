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

///\ingroup demos
///\file
///\brief Demonstrating graph input and output
///
/// This program gives an example of how to load a directed graph from
/// an \ref lgf-format "LGF" file with the \ref lemon::DigraphReader
/// "DigraphReader" class.
///
/// The \c "digraph.lgf" file:
/// \include digraph.lgf
///
/// And the program which reads it:
/// \include lgf_demo.cc

#include <iostream>
#include <lemon/smart_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/lgf_writer.h>
#include <lemon/random.h>


using namespace lemon;

int main() {
  SmartDigraph g;
  SmartDigraph::ArcMap<int> cap(g);
  SmartDigraph::Node s, t;

  digraphReader("digraph.lgf", g). // read the directeg graph into g
    arcMap("capacity", cap).       // read the 'capacity' arc map into cap
    node("source", s).             // read 'source' node to s
    node("target", t).             // read 'target' node to t
    run();

  std::cout << "Digraph read from 'digraph.lgf'" << std::endl;
  std::cout << "Number of nodes: " << countNodes(g) << std::endl;
  std::cout << "Number of arcs: " << countArcs(g) << std::endl;

  std::cout << "We can write it to the standard output:" << std::endl;

  digraphWriter(std::cout, g).     // write g to the standard output
    arcMap("capacity", cap).       // write cap into 'capacity'
    node("source", s).             // write s to 'source'
    node("target", t).             // write t to 'target'
    run();

  return 0;
}
