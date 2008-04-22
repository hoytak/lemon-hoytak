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
/// This simple demo program gives an example of how to read and write
/// a graph and additional maps (on the nodes or the edges) from/to a
/// stream. 
///
/// \include reader_writer_demo.cc

#include <iostream>
#include <lemon/smart_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/lgf_writer.h>
#include <lemon/random.h>


using namespace lemon;

int main(int argc, const char *argv[]) {
  const int n = argc > 1 ? std::atoi(argv[1]) : 20;
  const int e = argc > 2 ? std::atoi(argv[2]) : static_cast<int>(n * std::log(double(n)));
  const int m = argc > 3 ? std::atoi(argv[3]) : 100;

  SmartDigraph digraph;

  std::stringstream ss;

  try {

    typedef SmartDigraph Digraph;
    typedef Digraph::Node Node;
    typedef Digraph::Arc Arc;
    typedef Digraph::ArcIt ArcIt;

    typedef Digraph::NodeMap<int> PotentialMap;
    typedef Digraph::ArcMap<int> CapacityMap;
    typedef Digraph::ArcMap<std::string> NameMap;

    Digraph digraph;
    PotentialMap potential(digraph);
    CapacityMap capacity(digraph);
    NameMap name(digraph);

    std::vector<Node> nodes;
    for (int i = 0; i < n; ++i) {
      Node node = digraph.addNode();
      potential[node] = rnd[m];
      nodes.push_back(node);
    }

    std::vector<Arc> arcs;
    for (int i = 0; i < e; ++i) {
      int s = rnd[n];
      int t = rnd[n];
      int c = rnd[m];
      Arc arc = digraph.addArc(nodes[s], nodes[t]);
      capacity[arc] = c;
      std::ostringstream os;
      os << "arc \t" << i << std::endl;
      name[arc] = os.str();
      arcs.push_back(arc);
    }


    DigraphWriter<Digraph>(ss, digraph).
      nodeMap("potential", potential).
      arcMap("capacity", capacity).
      arcMap("name", name).
      node("source", nodes[0]).
      node("target", nodes[1]).
      arc("bottleneck", arcs[e / 2]).
      attribute("creator", "lemon library").
      run();

  } catch (DataFormatError& error) {
    std::cerr << error.what() << std::endl;
  }

  try {

    typedef SmartDigraph Digraph;
    typedef Digraph::Node Node;
    typedef Digraph::Arc Arc;
    typedef Digraph::ArcIt ArcIt;

    typedef Digraph::NodeMap<int> LabelMap;
    typedef Digraph::NodeMap<int> PotentialMap;
    typedef Digraph::ArcMap<int> CapacityMap;
    typedef Digraph::ArcMap<std::string> NameMap;

    Digraph digraph;
    LabelMap label(digraph);
    PotentialMap potential(digraph);
    CapacityMap capacity(digraph);
    NameMap name(digraph);

    Node s, t;
    Arc a;
    
    std::string creator;

    for (int i = 0; i < n; ++i) {
      Node node = digraph.addNode();
      label[node] = i;
    }
    
    DigraphReader<Digraph>(ss, digraph).
      useNodes(label).
      nodeMap("potential", potential).
      arcMap("capacity", capacity).
      arcMap("name", name).
      node("source", s).
      node("target", t).
      arc("bottleneck", a).
      attribute("creator", creator).
      run();

    DigraphWriter<Digraph>(std::cout, digraph).
      nodeMap("potential", potential).
      arcMap("capacity", capacity).
      arcMap("name", name).
      node("source", s).
      node("target", t).
      arc("bottleneck", a).
      attribute("creator", creator).
      run();

  } catch (DataFormatError& error) {
    std::cerr << error.what() << std::endl;
  }


  return 0;
}
