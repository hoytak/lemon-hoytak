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

#ifndef HYPERCUBE_GRAPH_H
#define HYPERCUBE_GRAPH_H

#include <iostream>
#include <vector>
#include <lemon/core.h>
#include <lemon/error.h>

#include <lemon/bits/base_extender.h>
#include <lemon/bits/graph_extender.h>

///\ingroup graphs
///\file
///\brief HypercubeDigraph class.

namespace lemon {

  class HypercubeDigraphBase {

  public:

    typedef HypercubeDigraphBase Digraph;

    class Node;
    class Arc;

  public:

    HypercubeDigraphBase() {}

  protected:

    void construct(int dim) {
      _dim = dim;
      _nodeNum = 1 << dim;
    }

  public:

    typedef True NodeNumTag;
    typedef True ArcNumTag;

    int nodeNum() const { return _nodeNum; }
    int arcNum() const { return _nodeNum * _dim; }

    int maxNodeId() const { return nodeNum() - 1; }
    int maxArcId() const { return arcNum() - 1; }

    Node source(Arc e) const {
      return e.id / _dim;
    }

    Node target(Arc e) const {
      return (e.id / _dim) ^ (1 << (e.id % _dim));
    }

    static int id(Node v) { return v.id; }
    static int id(Arc e) { return e.id; }

    static Node nodeFromId(int id) { return Node(id); }

    static Arc arcFromId(int id) { return Arc(id); }

    class Node {
      friend class HypercubeDigraphBase;
    protected:
      int id;
      Node(int _id) { id = _id;}
    public:
      Node() {}
      Node (Invalid) { id = -1; }
      bool operator==(const Node node) const { return id == node.id; }
      bool operator!=(const Node node) const { return id != node.id; }
      bool operator<(const Node node) const { return id < node.id; }
    };

    class Arc {
      friend class HypercubeDigraphBase;
    protected:
      int id;
      Arc(int _id) : id(_id) {}
    public:
      Arc() { }
      Arc (Invalid) { id = -1; }
      bool operator==(const Arc arc) const { return id == arc.id; }
      bool operator!=(const Arc arc) const { return id != arc.id; }
      bool operator<(const Arc arc) const { return id < arc.id; }
    };

    void first(Node& node) const {
      node.id = nodeNum() - 1;
    }

    static void next(Node& node) {
      --node.id;
    }

    void first(Arc& arc) const {
      arc.id = arcNum() - 1;
    }

    static void next(Arc& arc) {
      --arc.id;
    }

    void firstOut(Arc& arc, const Node& node) const {
      arc.id = node.id * _dim;
    }

    void nextOut(Arc& arc) const {
      ++arc.id;
      if (arc.id % _dim == 0) arc.id = -1;
    }

    void firstIn(Arc& arc, const Node& node) const {
      arc.id = (node.id ^ 1) * _dim;
    }

    void nextIn(Arc& arc) const {
      int cnt = arc.id % _dim;
      if ((cnt + 1) % _dim == 0) {
        arc.id = -1;
      } else {
        arc.id = ((arc.id / _dim) ^ ((1 << cnt) * 3)) * _dim + cnt + 1;
      }
    }

    int dimension() const {
      return _dim;
    }

    bool projection(Node node, int n) const {
      return static_cast<bool>(node.id & (1 << n));
    }

    int dimension(Arc arc) const {
      return arc.id % _dim;
    }

    int index(Node node) const {
      return node.id;
    }

    Node operator()(int ix) const {
      return Node(ix);
    }

  private:
    int _dim, _nodeNum;
  };


  typedef DigraphExtender<HypercubeDigraphBase> ExtendedHypercubeDigraphBase;

  /// \ingroup digraphs
  ///
  /// \brief Hypercube digraph class
  ///
  /// This class implements a special digraph type. The nodes of the
  /// digraph are indiced with integers with at most \c dim binary digits.
  /// Two nodes are connected in the digraph if the indices differ only
  /// on one position in the binary form.
  ///
  /// \note The type of the \c ids is chosen to \c int because efficiency
  /// reasons. Thus the maximum dimension of this implementation is 26.
  ///
  /// The digraph type is fully conform to the \ref concepts::Digraph
  /// concept but it does not conform to \ref concepts::Graph.
  class HypercubeDigraph : public ExtendedHypercubeDigraphBase {
  public:

    typedef ExtendedHypercubeDigraphBase Parent;

    /// \brief Construct a hypercube digraph with \c dim dimension.
    ///
    /// Construct a hypercube digraph with \c dim dimension.
    HypercubeDigraph(int dim) { construct(dim); }

    /// \brief Gives back the number of the dimensions.
    ///
    /// Gives back the number of the dimensions.
    int dimension() const {
      return Parent::dimension();
    }

    /// \brief Returns true if the n'th bit of the node is one.
    ///
    /// Returns true if the n'th bit of the node is one.
    bool projection(Node node, int n) const {
      return Parent::projection(node, n);
    }

    /// \brief The dimension id of the arc.
    ///
    /// It returns the dimension id of the arc. It can
    /// be in the \f$ \{0, 1, \dots, dim-1\} \f$ interval.
    int dimension(Arc arc) const {
      return Parent::dimension(arc);
    }

    /// \brief Gives back the index of the node.
    ///
    /// Gives back the index of the node. The lower bits of the
    /// integer describes the node.
    int index(Node node) const {
      return Parent::index(node);
    }

    /// \brief Gives back the node by its index.
    ///
    /// Gives back the node by its index.
    Node operator()(int ix) const {
      return Parent::operator()(ix);
    }

    /// \brief Number of nodes.
    int nodeNum() const { return Parent::nodeNum(); }
    /// \brief Number of arcs.
    int arcNum() const { return Parent::arcNum(); }

    /// \brief Linear combination map.
    ///
    /// It makes possible to give back a linear combination
    /// for each node. This function works like the \c std::accumulate
    /// so it accumulates the \c bf binary function with the \c fv
    /// first value. The map accumulates only on that dimensions where
    /// the node's index is one. The accumulated values should be
    /// given by the \c begin and \c end iterators and the length of this
    /// range should be equal to the dimension number of the digraph.
    ///
    ///\code
    /// const int DIM = 3;
    /// HypercubeDigraph digraph(DIM);
    /// dim2::Point<double> base[DIM];
    /// for (int k = 0; k < DIM; ++k) {
    ///   base[k].x = rnd();
    ///   base[k].y = rnd();
    /// }
    /// HypercubeDigraph::HyperMap<dim2::Point<double> >
    ///   pos(digraph, base, base + DIM, dim2::Point<double>(0.0, 0.0));
    ///\endcode
    ///
    /// \see HypercubeDigraph
    template <typename T, typename BF = std::plus<T> >
    class HyperMap {
    public:

      typedef Node Key;
      typedef T Value;


      /// \brief Constructor for HyperMap.
      ///
      /// Construct a HyperMap for the given digraph. The accumulated values
      /// should be given by the \c begin and \c end iterators and the length
      /// of this range should be equal to the dimension number of the digraph.
      ///
      /// This function accumulates the \c bf binary function with
      /// the \c fv first value. The map accumulates only on that dimensions
      /// where the node's index is one.
      template <typename It>
      HyperMap(const Digraph& digraph, It begin, It end,
               T fv = 0.0, const BF& bf = BF())
        : _graph(digraph), _values(begin, end), _first_value(fv), _bin_func(bf)
      {
        LEMON_ASSERT(_values.size() == digraph.dimension(),
                     "Wrong size of dimension");
      }

      /// \brief Gives back the partial accumulated value.
      ///
      /// Gives back the partial accumulated value.
      Value operator[](Key k) const {
        Value val = _first_value;
        int id = _graph.index(k);
        int n = 0;
        while (id != 0) {
          if (id & 1) {
            val = _bin_func(val, _values[n]);
          }
          id >>= 1;
          ++n;
        }
        return val;
      }

    private:
      const Digraph& _graph;
      std::vector<T> _values;
      T _first_value;
      BF _bin_func;
    };

  };

}

#endif
