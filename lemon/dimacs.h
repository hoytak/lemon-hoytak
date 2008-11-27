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

#ifndef LEMON_DIMACS_H
#define LEMON_DIMACS_H

#include <iostream>
#include <string>
#include <vector>
#include <lemon/maps.h>

/// \ingroup dimacs_group
/// \file
/// \brief DIMACS file format reader.

namespace lemon {

  ///@defgroup dimacs_group DIMACS format
  ///\brief Read and write files in DIMACS format
  ///
  ///Tools to read a digraph from or write it to a file in DIMACS format
  ///data
  ///\ingroup io_group

  /// \addtogroup dimacs_group
  /// @{

  /// DIMACS min cost flow reader function.
  ///
  /// This function reads a min cost flow instance from DIMACS format,
  /// i.e. from DIMACS files having a line starting with
  /// \code
  ///   p min
  /// \endcode
  /// At the beginning \c g is cleared by \c g.clear(). The supply
  /// amount of the nodes are written to \c supply (signed). The
  /// lower bounds, capacities and costs of the arcs are written to
  /// \c lower, \c capacity and \c cost.
  template <typename Digraph, typename LowerMap,
    typename CapacityMap, typename CostMap,
    typename SupplyMap>
  void readDimacsMin( std::istream& is,
                   Digraph &g,
                   LowerMap& lower,
                   CapacityMap& capacity,
                   CostMap& cost,
                   SupplyMap& supply )
  {
    g.clear();
    std::vector<typename Digraph::Node> nodes;
    typename Digraph::Arc e;
    std::string problem, str;
    char c;
    int n, m;
    int i, j;
    typename SupplyMap::Value sup;
    typename CapacityMap::Value low;
    typename CapacityMap::Value cap;
    typename CostMap::Value co;
    while (is >> c) {
      switch (c) {
      case 'c': // comment line
        getline(is, str);
        break;
      case 'p': // problem definition line
        is >> problem >> n >> m;
        getline(is, str);
        if (problem != "min") return;
        nodes.resize(n + 1);
        for (int k = 1; k <= n; ++k) {
          nodes[k] = g.addNode();
          supply.set(nodes[k], 0);
        }
        break;
      case 'n': // node definition line
        is >> i >> sup;
        getline(is, str);
        supply.set(nodes[i], sup);
        break;
      case 'a': // arc (arc) definition line
        is >> i >> j >> low >> cap >> co;
        getline(is, str);
        e = g.addArc(nodes[i], nodes[j]);
        lower.set(e, low);
        if (cap >= 0)
          capacity.set(e, cap);
        else
          capacity.set(e, -1);
        cost.set(e, co);
        break;
      }
    }
  }

  /// DIMACS max flow reader function.
  ///
  /// This function reads a max flow instance from DIMACS format,
  /// i.e. from DIMACS files having a line starting with
  /// \code
  ///   p max
  /// \endcode
  /// At the beginning \c g is cleared by \c g.clear(). The arc
  /// capacities are written to \c capacity and \c s and \c t are
  /// set to the source and the target nodes.
  template<typename Digraph, typename CapacityMap>
  void readDimacsMax(std::istream& is, Digraph &g, CapacityMap& capacity,
                  typename Digraph::Node &s, typename Digraph::Node &t) {
    g.clear();
    std::vector<typename Digraph::Node> nodes;
    typename Digraph::Arc e;
    std::string problem;
    char c, d;
    int n, m;
    int i, j;
    typename CapacityMap::Value _cap;
    std::string str;
    while (is >> c) {
      switch (c) {
      case 'c': // comment line
        getline(is, str);
        break;
      case 'p': // problem definition line
        is >> problem >> n >> m;
        getline(is, str);
        nodes.resize(n + 1);
        for (int k = 1; k <= n; ++k)
          nodes[k] = g.addNode();
        break;
      case 'n': // node definition line
        if (problem == "sp") { // shortest path problem
          is >> i;
          getline(is, str);
          s = nodes[i];
        }
        if (problem == "max") { // max flow problem
          is >> i >> d;
          getline(is, str);
          if (d == 's') s = nodes[i];
          if (d == 't') t = nodes[i];
        }
        break;
      case 'a': // arc (arc) definition line
        if (problem == "max" || problem == "sp") {
          is >> i >> j >> _cap;
          getline(is, str);
          e = g.addArc(nodes[i], nodes[j]);
          capacity.set(e, _cap);
        } else {
          is >> i >> j;
          getline(is, str);
          g.addArc(nodes[i], nodes[j]);
        }
        break;
      }
    }
  }

  /// DIMACS shortest path reader function.
  ///
  /// This function reads a shortest path instance from DIMACS format,
  /// i.e. from DIMACS files having a line starting with
  /// \code
  ///   p sp
  /// \endcode
  /// At the beginning \c g is cleared by \c g.clear(). The arc
  /// capacities are written to \c capacity and \c s is set to the
  /// source node.
  template<typename Digraph, typename CapacityMap>
  void readDimacsSp(std::istream& is, Digraph &g, CapacityMap& capacity,
                  typename Digraph::Node &s) {
    typename Digraph::Node t;
    readDimacsMax(is, g, capacity, s, t);
  }

  /// DIMACS capacitated digraph reader function.
  ///
  /// This function reads an arc capacitated digraph instance from
  /// DIMACS format. At the beginning \c g is cleared by \c g.clear()
  /// and the arc capacities are written to \c capacity.
  template<typename Digraph, typename CapacityMap>
  void readDimacsMax(std::istream& is, Digraph &g, CapacityMap& capacity) {
    typename Digraph::Node u,v;
    readDimacsMax(is, g, capacity, u, v);
  }

  /// DIMACS plain digraph reader function.
  ///
  /// This function reads a digraph without any designated nodes and
  /// maps from DIMACS format, i.e. from DIMACS files having a line
  /// starting with
  /// \code
  ///   p mat
  /// \endcode
  /// At the beginning \c g is cleared by \c g.clear().
  template<typename Digraph>
  void readDimacsMat(std::istream& is, Digraph &g) {
    typename Digraph::Node u,v;
    NullMap<typename Digraph::Arc, int> n;
    readDimacsMax(is, g, n, u, v);
  }

  /// DIMACS plain digraph writer function.
  ///
  /// This function writes a digraph without any designated nodes and
  /// maps into DIMACS format, i.e. into DIMACS file having a line
  /// starting with
  /// \code
  ///   p mat
  /// \endcode
  template<typename Digraph>
  void writeDimacsMat(std::ostream& os, const Digraph &g) {
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::ArcIt ArcIt;

    os << "c matching problem" << std::endl;
    os << "p mat " << g.nodeNum() << " " << g.arcNum() << std::endl;

    typename Digraph::template NodeMap<int> nodes(g);
    int i = 1;
    for(NodeIt v(g); v != INVALID; ++v) {
      nodes.set(v, i);
      ++i;
    }
    for(ArcIt e(g); e != INVALID; ++e) {
      os << "a " << nodes[g.source(e)] << " " << nodes[g.target(e)]
         << std::endl;
    }
  }

  /// @}

} //namespace lemon

#endif //LEMON_DIMACS_H
