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

#ifndef GRID_GRAPH_H
#define GRID_GRAPH_H

#include <iostream>
#include <lemon/core.h>
#include <lemon/assert.h>

#include <lemon/bits/base_extender.h>
#include <lemon/bits/graph_extender.h>

#include <lemon/dim2.h>

///\ingroup graphs
///\file
///\brief GridGraph class.

namespace lemon {

  class GridGraphBase {

  public:

    typedef GridGraphBase Graph;

    class Node;
    class Arc;

  public:

    GridGraphBase() {}

  protected:

    void construct(int w, int h) {
      _height = h; _width = w;
      _nodeNum = h * w; _arcNum = 2 * _nodeNum - w - h;
      _arcLimit = _nodeNum - w;
    }

    Arc _down(Node n) const {
      if (n.id < _nodeNum - _width) {
        return Arc(n.id);
      } else {
        return INVALID;
      }
    }

    Arc _up(Node n) const {
      if (n.id >= _width) {
        return Arc(n.id - _width);
      } else {
        return INVALID;
      }
    }

    Arc _right(Node n) const {
      if (n.id % _width < _width - 1) {
        return _arcLimit + n.id % _width + (n.id / _width) * (_width - 1);
      } else {
        return INVALID;
      }
    }

    Arc _left(Node n) const {
      if (n.id % _width > 0) {
        return _arcLimit + n.id % _width + (n.id / _width) * (_width - 1) - 1;
      } else {
        return INVALID;
      }
    }

  public:

    Node operator()(int i, int j) const {
      LEMON_ASSERT(0 <= i && i < width() &&
                   0 <= j && j < height(), "lemon::GridGraph::IndexError");
      return Node(i + j * _width);
    }

    int row(Node n) const {
      return n.id / _width;
    }

    int col(Node n) const {
      return n.id % _width;
    }

    int width() const {
      return _width;
    }

    int height() const {
      return _height;
    }

    typedef True NodeNumTag;
    typedef True ArcNumTag;

    int nodeNum() const { return _nodeNum; }
    int arcNum() const { return _arcNum; }

    int maxNodeId() const { return nodeNum() - 1; }
    int maxArcId() const { return arcNum() - 1; }

    Node source(Arc e) const {
      if (e.id < _arcLimit) {
        return e.id;
      } else {
        return (e.id - _arcLimit) % (_width - 1) +
          (e.id - _arcLimit) / (_width - 1) * _width;
      }
    }

    Node target(Arc e) const {
      if (e.id < _arcLimit) {
        return e.id + _width;
      } else {
        return (e.id - _arcLimit) % (_width - 1) +
          (e.id - _arcLimit) / (_width - 1) * _width + 1;
      }
    }

    static int id(Node v) { return v.id; }
    static int id(Arc e) { return e.id; }

    static Node nodeFromId(int id) { return Node(id);}

    static Arc arcFromId(int id) { return Arc(id);}

    typedef True FindArcTag;

    Arc findArc(Node u, Node v, Arc prev = INVALID) const {
      if (prev != INVALID) return INVALID;
      if (v.id - u.id == _width) return Arc(u.id);
      if (v.id - u.id == 1 && u.id % _width < _width - 1) {
        return Arc(u.id / _width * (_width - 1) +
                   u.id % _width + _arcLimit);
      }
      return INVALID;
    }

    class Node {
      friend class GridGraphBase;

    protected:
      int id;
      Node(int _id) : id(_id) {}
    public:
      Node() {}
      Node (Invalid) { id = -1; }
      bool operator==(const Node node) const { return id == node.id; }
      bool operator!=(const Node node) const { return id != node.id; }
      bool operator<(const Node node) const { return id < node.id; }
    };

    class Arc {
      friend class GridGraphBase;

    protected:
      int id;
      Arc(int _id) : id(_id) {}
    public:
      Arc() {}
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
      if (node.id < _nodeNum - _width) {
        arc.id = node.id;
      } else if (node.id % _width < _width - 1) {
        arc.id = _arcLimit + node.id % _width +
          (node.id / _width) * (_width - 1);
      } else {
        arc.id = -1;
      }
    }

    void nextOut(Arc& arc) const {
      if (arc.id >= _arcLimit) {
        arc.id = -1;
      } else if (arc.id % _width < _width - 1) {
        arc.id = _arcLimit + arc.id % _width +
          (arc.id / _width) * (_width - 1);
      } else {
        arc.id = -1;
      }
    }

    void firstIn(Arc& arc, const Node& node) const {
      if (node.id >= _width) {
        arc.id = node.id - _width;
      } else if (node.id % _width > 0) {
        arc.id = _arcLimit + node.id % _width +
          (node.id / _width) * (_width - 1) - 1;
      } else {
        arc.id = -1;
      }
    }

    void nextIn(Arc& arc) const {
      if (arc.id >= _arcLimit) {
        arc.id = -1;
      } else if (arc.id % _width > 0) {
        arc.id = _arcLimit + arc.id % _width +
          (arc.id / _width + 1) * (_width - 1) - 1;
      } else {
        arc.id = -1;
      }
    }

  private:
    int _width, _height;
    int _nodeNum, _arcNum;
    int _arcLimit;
  };

  typedef GraphExtender<UndirDigraphExtender<GridGraphBase> >
    ExtendedGridGraphBase;

  /// \ingroup graphs
  ///
  /// \brief Grid graph class
  ///
  /// This class implements a special graph type. The nodes of the
  /// graph can be indiced by two integer \c (i,j) value where \c i
  /// is in the \c [0,width) range and j is in the [0, height) range.
  /// Two nodes are connected in the graph if the indices differ only
  /// on one position and only one is the difference.
  ///
  /// \image html grid_graph.png
  /// \image latex grid_graph.eps "Grid graph" width=\textwidth
  ///
  /// The graph can be indiced in the following way:
  ///\code
  /// GridGraph gr(w, h);
  /// GridGraph::NodeMap<int> val(gr);
  /// for (int i = 0; i < gr.width(); ++i) {
  ///   for (int j = 0; j < gr.height(); ++j) {
  ///     val[gr(i, j)] = i + j;
  ///   }
  /// }
  ///\endcode
  ///
  /// This graph type is fully conform to the \ref concepts::Graph
  /// "Undirected Graph" concept, and it also has an important extra
  /// feature that its maps are real \ref concepts::ReferenceMap
  /// "reference map"s.
  class GridGraph : public ExtendedGridGraphBase {
  public:

    typedef ExtendedGridGraphBase Parent;

    /// \brief Map to get the indices of the nodes as dim2::Point<int>.
    ///
    /// Map to get the indices of the nodes as dim2::Point<int>.
    class IndexMap {
    public:
      /// The key type of the map
      typedef GridGraph::Node Key;
      /// The value type of the map
      typedef dim2::Point<int> Value;

      /// Constructor
      IndexMap(const GridGraph& graph) : _graph(graph) {}

      /// The subscript operator
      Value operator[](const Key& key) const {
        return dim2::Point<int>(_graph.row(key), _graph.col(key));
      }

    private:
      const GridGraph& _graph;
    };

    /// \brief Map to get the row of the nodes.
    ///
    /// Map to get the row of the nodes.
    class RowMap {
    public:
      /// The key type of the map
      typedef GridGraph::Node Key;
      /// The value type of the map
      typedef int Value;

      /// Constructor
      RowMap(const GridGraph& graph) : _graph(graph) {}

      /// The subscript operator
      Value operator[](const Key& key) const {
        return _graph.row(key);
      }

    private:
      const GridGraph& _graph;
    };

    /// \brief Map to get the column of the nodes.
    ///
    /// Map to get the column of the nodes.
    class ColMap {
    public:
      /// The key type of the map
      typedef GridGraph::Node Key;
      /// The value type of the map
      typedef int Value;

      /// Constructor
      ColMap(const GridGraph& graph) : _graph(graph) {}

      /// The subscript operator
      Value operator[](const Key& key) const {
        return _graph.col(key);
      }

    private:
      const GridGraph& _graph;
    };

    /// \brief Constructor
    ///
    /// Constructor.
    /// \param width The width of the grid.
    /// \param height The height of the grid.
    GridGraph(int width, int height) { construct(width, height); }

    /// \brief Resize the graph
    ///
    /// Resize the grid.
    void resize(int width, int height) {
      Parent::notifier(Arc()).clear();
      Parent::notifier(Edge()).clear();
      Parent::notifier(Node()).clear();
      construct(width, height);
      Parent::notifier(Node()).build();
      Parent::notifier(Edge()).build();
      Parent::notifier(Arc()).build();
    }

    /// \brief The node on the given position.
    ///
    /// Gives back the node on the given position.
    Node operator()(int i, int j) const {
      return Parent::operator()(i, j);
    }

    /// \brief Gives back the row index of the node.
    ///
    /// Gives back the row index of the node.
    int row(Node n) const {
      return Parent::row(n);
    }

    /// \brief Gives back the column index of the node.
    ///
    /// Gives back the column index of the node.
    int col(Node n) const {
      return Parent::col(n);
    }

    /// \brief Gives back the width of the grid.
    ///
    /// Gives back the width of the grid.
    int width() const {
      return Parent::width();
    }

    /// \brief Gives back the height of the grid.
    ///
    /// Gives back the height of the grid.
    int height() const {
      return Parent::height();
    }

    /// \brief Gives back the arc goes down from the node.
    ///
    /// Gives back the arc goes down from the node. If there is not
    /// outgoing arc then it gives back \c INVALID.
    Arc down(Node n) const {
      Edge e = _down(n);
      return e != INVALID ? direct(e, true) : INVALID;
    }

    /// \brief Gives back the arc goes up from the node.
    ///
    /// Gives back the arc goes up from the node. If there is not
    /// outgoing arc then it gives back \c INVALID.
    Arc up(Node n) const {
      Edge e = _up(n);
      return e != INVALID ? direct(e, false) : INVALID;
    }

    /// \brief Gives back the arc goes right from the node.
    ///
    /// Gives back the arc goes right from the node. If there is not
    /// outgoing arc then it gives back \c INVALID.
    Arc right(Node n) const {
      Edge e = _right(n);
      return e != INVALID ? direct(e, true) : INVALID;
    }

    /// \brief Gives back the arc goes left from the node.
    ///
    /// Gives back the arc goes left from the node. If there is not
    /// outgoing arc then it gives back \c INVALID.
    Arc left(Node n) const {
      Edge e = _left(n);
      return e != INVALID ? direct(e, false) : INVALID;
    }

  }; // class GridGraph

  /// \brief Index map of the grid graph
  ///
  /// Just returns an IndexMap for the grid graph.
  inline GridGraph::IndexMap indexMap(const GridGraph& graph) {
    return GridGraph::IndexMap(graph);
  }

  /// \brief Row map of the grid graph
  ///
  /// Just returns a RowMap for the grid graph.
  inline GridGraph::RowMap rowMap(const GridGraph& graph) {
    return GridGraph::RowMap(graph);
  }

  /// \brief Column map of the grid graph
  ///
  /// Just returns a ColMap for the grid graph.
  inline GridGraph::ColMap colMap(const GridGraph& graph) {
    return GridGraph::ColMap(graph);
  }
}

#endif // GRID_GRAPH_H
