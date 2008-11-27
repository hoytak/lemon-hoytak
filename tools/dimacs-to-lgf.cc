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

///\ingroup tools
///\file
///\brief DIMACS to LGF converter.
///
/// This program converts various DIMACS formats to the LEMON Digraph Format
/// (LGF).
///
/// See
/// \verbatim
///  dimacs-to-lgf --help
/// \endverbatim
/// for more info on usage.
///

#include <iostream>
#include <fstream>
#include <cstring>

#include <lemon/smart_graph.h>
#include <lemon/dimacs.h>
#include <lemon/lgf_writer.h>

#include <lemon/arg_parser.h>

using namespace std;
using namespace lemon;


int main(int argc, const char *argv[]) {
  typedef SmartDigraph Digraph;

  typedef Digraph::Arc Arc;
  typedef Digraph::Node Node;
  typedef Digraph::ArcIt ArcIt;
  typedef Digraph::NodeIt NodeIt;
  typedef Digraph::ArcMap<double> DoubleArcMap;
  typedef Digraph::NodeMap<double> DoubleNodeMap;

  std::string inputName;
  std::string outputName;
  std::string typeName;

  bool mincostflow;
  bool maxflow;
  bool shortestpath;
  bool capacitated;
  bool plain;

  bool version;

  ArgParser ap(argc, argv);
  ap.refOption("-input",
               "use FILE as input instead of standard input",
               inputName).synonym("i", "-input")
    .refOption("-output",
               "use FILE as output instead of standard output",
               outputName).synonym("o", "-output")
    .refOption("-mincostflow",
               "set the type of the digraph to \"mincostflow\" digraph",
               mincostflow)
    .optionGroup("type", "-mincostflow").synonym("mcf", "-mincostflow")
    .refOption("-maxflow",
               "set the type of the digraph to \"maxflow\" digraph",
               maxflow)
    .optionGroup("type", "-maxflow").synonym("mf", "-maxflow")
    .refOption("-shortestpath",
               "set the type of the digraph to \"shortestpath\" digraph",
               shortestpath)
    .optionGroup("type", "-shortestpath").synonym("sp", "-shortestpath")
    .refOption("-capacitated",
               "set the type of the digraph to \"capacitated\" digraph",
               capacitated)
    .optionGroup("type", "-capacitated").synonym("cap", "-capacitated")
    .refOption("-plain",
               "set the type of the digraph to \"plain\" digraph",
               plain)
    .optionGroup("type", "-plain").synonym("pl", "-plain")
    .onlyOneGroup("type")
    .mandatoryGroup("type")
    .refOption("-version", "show version information", version)
    .synonym("v", "-version")
    .run();

  ifstream input;
  if (!inputName.empty()) {
    input.open(inputName.c_str());
    if (!input) {
      cerr << "File open error" << endl;
      return -1;
    }
  }
  istream& is = (inputName.empty() ? cin : input);

  ofstream output;
  if (!outputName.empty()) {
    output.open(outputName.c_str());
    if (!output) {
      cerr << "File open error" << endl;
      return -1;
    }
  }
  ostream& os = (outputName.empty() ? cout : output);

  if (mincostflow) {
    Digraph digraph;
    DoubleArcMap lower(digraph), capacity(digraph), cost(digraph);
    DoubleNodeMap supply(digraph);
    readDimacs(is, digraph, lower, capacity, cost, supply);
    DigraphWriter<Digraph>(digraph, os).
      nodeMap("supply", supply).
      arcMap("lower", lower).
      arcMap("capacity", capacity).
      arcMap("cost", cost).
      run();
  } else if (maxflow) {
    Digraph digraph;
    Node s, t;
    DoubleArcMap capacity(digraph);
    readDimacs(is, digraph, capacity, s, t);
    DigraphWriter<Digraph>(digraph, os).
      arcMap("capacity", capacity).
      node("source", s).
      node("target", t).
      run();
  } else if (shortestpath) {
    Digraph digraph;
    Node s;
    DoubleArcMap capacity(digraph);
    readDimacs(is, digraph, capacity, s);
    DigraphWriter<Digraph>(digraph, os).
      arcMap("capacity", capacity).
      node("source", s).
      run();
  } else if (capacitated) {
    Digraph digraph;
    DoubleArcMap capacity(digraph);
    readDimacs(is, digraph, capacity);
    DigraphWriter<Digraph>(digraph, os).
      arcMap("capacity", capacity).
      run();
  } else if (plain) {
    Digraph digraph;
    readDimacs(is, digraph);
    DigraphWriter<Digraph>(digraph, os).run();
  } else {
    cerr << "Invalid type error" << endl;
    return -1;
  }
  return 0;
}
