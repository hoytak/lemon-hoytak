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

#ifndef LEMON_STATIC_GRAPH_H
#define LEMON_STATIC_GRAPH_H

///\ingroup graphs
///\file
///\brief StaticDigraph class.

#include <lemon/core.h>
#include <lemon/bits/graph_extender.h>

namespace lemon {

  class StaticDigraphBase {
  public:

    StaticDigraphBase() 
      : built(false), node_num(0), arc_num(0), 
        node_first_out(NULL), node_first_in(NULL),
        arc_source(NULL), arc_target(NULL), 
        arc_next_in(NULL), arc_next_out(NULL) {}
    
    ~StaticDigraphBase() {
      if (built) {
        delete[] node_first_out;
        delete[] node_first_in;
        delete[] arc_source;
        delete[] arc_target;
        delete[] arc_next_out;
        delete[] arc_next_in;
      }
    }

    class Node {
      friend class StaticDigraphBase;
    protected:
      int id;
      Node(int _id) : id(_id) {}
    public:
      Node() {}
      Node (Invalid) : id(-1) {}
      bool operator==(const Node& node) const { return id == node.id; }
      bool operator!=(const Node& node) const { return id != node.id; }
      bool operator<(const Node& node) const { return id < node.id; }
    };

    class Arc {
      friend class StaticDigraphBase;      
    protected:
      int id;
      Arc(int _id) : id(_id) {}
    public:
      Arc() { }
      Arc (Invalid) : id(-1) {}
      bool operator==(const Arc& arc) const { return id == arc.id; }
      bool operator!=(const Arc& arc) const { return id != arc.id; }
      bool operator<(const Arc& arc) const { return id < arc.id; }
    };

    Node source(const Arc& e) const { return Node(arc_source[e.id]); }
    Node target(const Arc& e) const { return Node(arc_target[e.id]); }

    void first(Node& n) const { n.id = node_num - 1; }
    static void next(Node& n) { --n.id; }

    void first(Arc& e) const { e.id = arc_num - 1; }
    static void next(Arc& e) { --e.id; }

    void firstOut(Arc& e, const Node& n) const { 
      e.id = node_first_out[n.id] != node_first_out[n.id + 1] ? 
        node_first_out[n.id] : -1;
    }
    void nextOut(Arc& e) const { e.id = arc_next_out[e.id]; }

    void firstIn(Arc& e, const Node& n) const { e.id = node_first_in[n.id]; }
    void nextIn(Arc& e) const { e.id = arc_next_in[e.id]; }

    int id(const Node& n) const { return n.id; }
    Node nodeFromId(int id) const { return Node(id); }
    int maxNodeId() const { return node_num - 1; }

    int id(const Arc& e) const { return e.id; }
    Arc arcFromId(int id) const { return Arc(id); }
    int maxArcId() const { return arc_num - 1; }

    typedef True NodeNumTag;
    typedef True ArcNumTag;

    int nodeNum() const { return node_num; }
    int arcNum() const { return arc_num; }

  private:

    template <typename Digraph, typename NodeRefMap>
    class ArcLess {
    public:
      typedef typename Digraph::Arc Arc;

      ArcLess(const Digraph &_graph, const NodeRefMap& _nodeRef) 
        : digraph(_graph), nodeRef(_nodeRef) {}
      
      bool operator()(const Arc& left, const Arc& right) const {
	return nodeRef[digraph.target(left)] < nodeRef[digraph.target(right)];
      }
    private:
      const Digraph& digraph;
      const NodeRefMap& nodeRef;
    };
    
  public:

    typedef True BuildTag;
    
    void clear() {
      if (built) {
        delete[] node_first_out;
        delete[] node_first_in;
        delete[] arc_source;
        delete[] arc_target;
        delete[] arc_next_out;
        delete[] arc_next_in;
      }
      built = false;
      node_num = 0;
      arc_num = 0;
    }
    
    template <typename Digraph, typename NodeRefMap, typename ArcRefMap>
    void build(const Digraph& digraph, NodeRefMap& nodeRef, ArcRefMap& arcRef) {
      typedef typename Digraph::Node GNode;
      typedef typename Digraph::Arc GArc;

      built = true;

      node_num = countNodes(digraph);
      arc_num = countArcs(digraph);

      node_first_out = new int[node_num + 1];
      node_first_in = new int[node_num];

      arc_source = new int[arc_num];
      arc_target = new int[arc_num];
      arc_next_out = new int[arc_num];
      arc_next_in = new int[arc_num];

      int node_index = 0;
      for (typename Digraph::NodeIt n(digraph); n != INVALID; ++n) {
        nodeRef[n] = Node(node_index);
        node_first_in[node_index] = -1;
        ++node_index;
      }

      ArcLess<Digraph, NodeRefMap> arcLess(digraph, nodeRef);

      int arc_index = 0;
      for (typename Digraph::NodeIt n(digraph); n != INVALID; ++n) {
        int source = nodeRef[n].id;
        std::vector<GArc> arcs;
        for (typename Digraph::OutArcIt e(digraph, n); e != INVALID; ++e) {
          arcs.push_back(e);
        }
        if (!arcs.empty()) {
          node_first_out[source] = arc_index;
          std::sort(arcs.begin(), arcs.end(), arcLess);
          for (typename std::vector<GArc>::iterator it = arcs.begin();
               it != arcs.end(); ++it) {
            int target = nodeRef[digraph.target(*it)].id;
            arcRef[*it] = Arc(arc_index);
            arc_source[arc_index] = source; 
            arc_target[arc_index] = target;
            arc_next_in[arc_index] = node_first_in[target];
            node_first_in[target] = arc_index;
            arc_next_out[arc_index] = arc_index + 1;
            ++arc_index;
          }
          arc_next_out[arc_index - 1] = -1;
        } else {
          node_first_out[source] = arc_index;
        }
      }
      node_first_out[node_num] = arc_num;
    }

  protected:

    void fastFirstOut(Arc& e, const Node& n) const {
      e.id = node_first_out[n.id];
    }

    static void fastNextOut(Arc& e) {
      ++e.id;
    }
    void fastLastOut(Arc& e, const Node& n) const {
      e.id = node_first_out[n.id + 1];
    }

  protected:
    bool built;
    int node_num;
    int arc_num;
    int *node_first_out;
    int *node_first_in;
    int *arc_source;
    int *arc_target;
    int *arc_next_in;
    int *arc_next_out;
  };

  typedef DigraphExtender<StaticDigraphBase> ExtendedStaticDigraphBase;


  /// \ingroup graphs
  ///
  /// \brief A static directed graph class.
  ///
  /// \ref StaticDigraph is a highly efficient digraph implementation,
  /// but it is fully static.
  /// It stores only two \c int values for each node and only four \c int
  /// values for each arc. Moreover it provides faster item iteration than
  /// \ref ListDigraph and \ref SmartDigraph, especially using \c OutArcIt
  /// iterators, since its arcs are stored in an appropriate order.
  /// However it only provides build() and clear() functions and does not
  /// support any other modification of the digraph.
  ///
  /// This type fully conforms to the \ref concepts::Digraph "Digraph concept".
  /// Most of its member functions and nested classes are documented
  /// only in the concept class.
  ///
  /// \sa concepts::Digraph
  class StaticDigraph : public ExtendedStaticDigraphBase {
  public:

    typedef ExtendedStaticDigraphBase Parent;
  
  public:
  
    /// \brief Clear the digraph.
    ///
    /// This function erases all nodes and arcs from the digraph.
    void clear() {
      Parent::clear();
    }
    
    /// \brief Build the digraph copying another digraph.
    ///
    /// This function builds the digraph copying another digraph of any
    /// kind. It can be called more than once, but in such case, the whole
    /// structure and all maps will be cleared and rebuilt.
    ///
    /// This method also makes possible to copy a digraph to a StaticDigraph
    /// structure using \ref DigraphCopy.
    /// 
    /// \param digraph An existing digraph to be copied.
    /// \param nodeRef The node references will be copied into this map.
    /// Its key type must be \c Digraph::Node and its value type must be
    /// \c StaticDigraph::Node.
    /// It must conform to the \ref concepts::ReadWriteMap "ReadWriteMap"
    /// concept.
    /// \param arcRef The arc references will be copied into this map.
    /// Its key type must be \c Digraph::Arc and its value type must be
    /// \c StaticDigraph::Arc.
    /// It must conform to the \ref concepts::WriteMap "WriteMap" concept.
    ///
    /// \note If you do not need the arc references, then you could use
    /// \ref NullMap for the last parameter. However the node references
    /// are required by the function itself, thus they must be readable
    /// from the map.
    template <typename Digraph, typename NodeRefMap, typename ArcRefMap>
    void build(const Digraph& digraph, NodeRefMap& nodeRef, ArcRefMap& arcRef) {
      if (built) Parent::clear();
      Parent::build(digraph, nodeRef, arcRef);
    }
  

  protected:

    using Parent::fastFirstOut;
    using Parent::fastNextOut;
    using Parent::fastLastOut;
    
  public:

    class OutArcIt : public Arc {
    public:

      OutArcIt() { }

      OutArcIt(Invalid i) : Arc(i) { }

      OutArcIt(const StaticDigraph& digraph, const Node& node) {
	digraph.fastFirstOut(*this, node);
	digraph.fastLastOut(last, node);
        if (last == *this) *this = INVALID;
      }

      OutArcIt(const StaticDigraph& digraph, const Arc& arc) : Arc(arc) {
        if (arc != INVALID) {
          digraph.fastLastOut(last, digraph.source(arc));
        }
      }

      OutArcIt& operator++() { 
        StaticDigraph::fastNextOut(*this);
        if (last == *this) *this = INVALID;
        return *this; 
      }

    private:
      Arc last;
    };

    Node baseNode(const OutArcIt &arc) const {
      return Parent::source(static_cast<const Arc&>(arc));
    }

    Node runningNode(const OutArcIt &arc) const {
      return Parent::target(static_cast<const Arc&>(arc));
    }

    Node baseNode(const InArcIt &arc) const {
      return Parent::target(static_cast<const Arc&>(arc));
    }

    Node runningNode(const InArcIt &arc) const {
      return Parent::source(static_cast<const Arc&>(arc));
    }

  };

}

#endif
