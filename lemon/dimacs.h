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
#include <lemon/error.h>

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


  /// DIMACS file type descriptor.
  struct DimacsDescriptor
  {
    ///File type enum
    enum Type
      {
        NONE, MIN, MAX, SP, MAT
      };
    ///The file type
    Type type;
    ///The number of nodes on the graph
    int nodeNum;
    ///The number of edges on the graph
    int edgeNum;
    int lineShift;
    /// Constructor. Sets the type to NONE.
    DimacsDescriptor() : type(NONE) {}
  };

  ///Discover the type of a DIMACS file

  ///It starts seeking the begining of the file for the problem type
  ///and size info. The found date is returned in a special struct
  ///that can be evaluated and passed to the appropriate reader
  ///function.
  DimacsDescriptor dimacsType(std::istream& is)
  {
    DimacsDescriptor r;
    std::string problem,str;
    char c;
    r.lineShift=0;
    while (is >> c)
      switch(c)
        {
        case 'p':
          if(is >> problem >> r.nodeNum >> r.edgeNum)
            {
              getline(is, str);
              r.lineShift++;
              if(problem=="min") r.type=DimacsDescriptor::MIN;
              else if(problem=="max") r.type=DimacsDescriptor::MAX;
              else if(problem=="sp") r.type=DimacsDescriptor::SP;
              else if(problem=="mat") r.type=DimacsDescriptor::MAT;
              else throw FormatError("Unknown problem type");
              return r;
            }
          else
            {
              throw FormatError("Missing or wrong problem type declaration.");
            }
          break;
        case 'c':
          getline(is, str);
          r.lineShift++;
          break;
        default:
          throw FormatError("Unknown DIMACS declaration.");
        }
    throw FormatError("Missing problem type declaration.");
  }



  /// DIMACS min cost flow reader function.
  ///
  /// This function reads a min cost flow instance from DIMACS format,
  /// i.e. from DIMACS files having a line starting with
  /// \code
  ///   p min
  /// \endcode
  /// At the beginning, \c g is cleared by \c g.clear(). The supply
  /// amount of the nodes are written to \c supply (signed). The
  /// lower bounds, capacities and costs of the arcs are written to
  /// \c lower, \c capacity and \c cost.
  ///
  /// If the file type was previously evaluated by dimacsType(), then
  /// the descriptor struct should be given by the \c dest parameter.
  template <typename Digraph, typename LowerMap,
            typename CapacityMap, typename CostMap,
            typename SupplyMap>
  void readDimacsMin(std::istream& is,
                     Digraph &g,
                     LowerMap& lower,
                     CapacityMap& capacity,
                     CostMap& cost,
                     SupplyMap& supply,
                     DimacsDescriptor desc=DimacsDescriptor())
  {
    g.clear();
    std::vector<typename Digraph::Node> nodes;
    typename Digraph::Arc e;
    std::string problem, str;
    char c;
    int i, j;
    if(desc.type==DimacsDescriptor::NONE) desc=dimacsType(is);
    if(desc.type!=DimacsDescriptor::MIN)
      throw FormatError("Problem type mismatch");

    nodes.resize(desc.nodeNum + 1);
    for (int k = 1; k <= desc.nodeNum; ++k) {
      nodes[k] = g.addNode();
      supply.set(nodes[k], 0);
    }

    typename SupplyMap::Value sup;
    typename CapacityMap::Value low;
    typename CapacityMap::Value cap;
    typename CostMap::Value co;
    while (is >> c) {
      switch (c) {
      case 'c': // comment line
        getline(is, str);
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

  template<typename Digraph, typename CapacityMap>
  void _readDimacs(std::istream& is,
                   Digraph &g,
                   CapacityMap& capacity,
                   typename Digraph::Node &s,
                   typename Digraph::Node &t,
                   DimacsDescriptor desc=DimacsDescriptor()) {
    g.clear();
    s=t=INVALID;
    std::vector<typename Digraph::Node> nodes;
    typename Digraph::Arc e;
    char c, d;
    int i, j;
    typename CapacityMap::Value _cap;
    std::string str;
    nodes.resize(desc.nodeNum + 1);
    for (int k = 1; k <= desc.nodeNum; ++k) {
      nodes[k] = g.addNode();
    }

    while (is >> c) {
      switch (c) {
      case 'c': // comment line
        getline(is, str);
        break;
      case 'n': // node definition line
        if (desc.type==DimacsDescriptor::SP) { // shortest path problem
          is >> i;
          getline(is, str);
          s = nodes[i];
        }
        if (desc.type==DimacsDescriptor::MAX) { // max flow problem
          is >> i >> d;
          getline(is, str);
          if (d == 's') s = nodes[i];
          if (d == 't') t = nodes[i];
        }
        break;
      case 'a': // arc (arc) definition line
        if (desc.type==DimacsDescriptor::SP ||
            desc.type==DimacsDescriptor::MAX) {
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

  /// DIMACS max flow reader function.
  ///
  /// This function reads a max flow instance from DIMACS format,
  /// i.e. from DIMACS files having a line starting with
  /// \code
  ///   p max
  /// \endcode
  /// At the beginning, \c g is cleared by \c g.clear(). The arc
  /// capacities are written to \c capacity and \c s and \c t are
  /// set to the source and the target nodes.
  ///
  /// If the file type was previously evaluated by dimacsType(), then
  /// the descriptor struct should be given by the \c dest parameter.
  template<typename Digraph, typename CapacityMap>
  void readDimacsMax(std::istream& is,
                     Digraph &g,
                     CapacityMap& capacity,
                     typename Digraph::Node &s,
                     typename Digraph::Node &t,
                     DimacsDescriptor desc=DimacsDescriptor()) {
    if(desc.type==DimacsDescriptor::NONE) desc=dimacsType(is);
    if(desc.type!=DimacsDescriptor::MAX)
      throw FormatError("Problem type mismatch");
    _readDimacs(is,g,capacity,s,t,desc);
  }

  /// DIMACS shortest path reader function.
  ///
  /// This function reads a shortest path instance from DIMACS format,
  /// i.e. from DIMACS files having a line starting with
  /// \code
  ///   p sp
  /// \endcode
  /// At the beginning, \c g is cleared by \c g.clear(). The arc
  /// lengths are written to \c lenght and \c s is set to the
  /// source node.
  ///
  /// If the file type was previously evaluated by dimacsType(), then
  /// the descriptor struct should be given by the \c dest parameter.
  template<typename Digraph, typename LengthMap>
  void readDimacsSp(std::istream& is,
                    Digraph &g,
                    LengthMap& length,
                    typename Digraph::Node &s,
                    DimacsDescriptor desc=DimacsDescriptor()) {
    typename Digraph::Node t;
    if(desc.type==DimacsDescriptor::NONE) desc=dimacsType(is);
    if(desc.type!=DimacsDescriptor::SP)
      throw FormatError("Problem type mismatch");
    _readDimacs(is, g, length, s, t,desc);
  }

  /// DIMACS capacitated digraph reader function.
  ///
  /// This function reads an arc capacitated digraph instance from
  /// DIMACS 'mat' or 'sp' format.
  /// At the beginning, \c g is cleared by \c g.clear()
  /// and the arc capacities/lengths are written to \c capacity.
  ///
  /// If the file type was previously evaluated by dimacsType(), then
  /// the descriptor struct should be given by the \c dest parameter.
  template<typename Digraph, typename CapacityMap>
  void readDimacsCap(std::istream& is,
                     Digraph &g,
                     CapacityMap& capacity,
                     DimacsDescriptor desc=DimacsDescriptor()) {
    typename Digraph::Node u,v;
    if(desc.type==DimacsDescriptor::NONE) desc=dimacsType(is);
    if(desc.type!=DimacsDescriptor::MAX || desc.type!=DimacsDescriptor::SP)
      throw FormatError("Problem type mismatch");
    _readDimacs(is, g, capacity, u, v, desc);
  }

  /// DIMACS plain digraph reader function.
  ///
  /// This function reads a digraph without any designated nodes and
  /// maps from DIMACS format, i.e. from DIMACS files having a line
  /// starting with
  /// \code
  ///   p mat
  /// \endcode
  /// At the beginning, \c g is cleared by \c g.clear().
  ///
  /// If the file type was previously evaluated by dimacsType(), then
  /// the descriptor struct should be given by the \c dest parameter.
  template<typename Digraph>
  void readDimacsMat(std::istream& is, Digraph &g,
                     DimacsDescriptor desc=DimacsDescriptor()) {
    typename Digraph::Node u,v;
    NullMap<typename Digraph::Arc, int> n;
    if(desc.type==DimacsDescriptor::NONE) desc=dimacsType(is);
    if(desc.type!=DimacsDescriptor::MAT)
      throw FormatError("Problem type mismatch");
    _readDimacs(is, g, n, u, v, desc);
  }

  /// DIMACS plain digraph writer function.
  ///
  /// This function writes a digraph without any designated nodes and
  /// maps into DIMACS format, i.e. into DIMACS file having a line
  /// starting with
  /// \code
  ///   p mat
  /// \endcode
  /// If \c comment is not empty, then it will be printed in the first line
  /// prefixed by 'c'.
  template<typename Digraph>
  void writeDimacsMat(std::ostream& os, const Digraph &g,
                      std::string comment="") {
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::ArcIt ArcIt;

    if(!comment.empty()) 
      os << "c " << comment << std::endl;
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
