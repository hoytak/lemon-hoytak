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

#ifndef LEMON_EDGE_SET_H
#define LEMON_EDGE_SET_H

#include <lemon/core.h>
#include <lemon/bits/edge_set_extender.h>

/// \ingroup semi_adaptors
/// \file
/// \brief ArcSet and EdgeSet classes.
///
/// Graphs which use another graph's node-set as own.
namespace lemon {

  template <typename _Graph>
  class ListArcSetBase {
  public:

    typedef _Graph Graph;
    typedef typename Graph::Node Node;
    typedef typename Graph::NodeIt NodeIt;

  protected:

    struct NodeT {
      int first_out, first_in;
      NodeT() : first_out(-1), first_in(-1) {}
    };

    typedef typename ItemSetTraits<Graph, Node>::
    template Map<NodeT>::Type NodesImplBase;

    NodesImplBase* nodes;

    struct ArcT {
      Node source, target;
      int next_out, next_in;
      int prev_out, prev_in;
      ArcT() : prev_out(-1), prev_in(-1) {}
    };

    std::vector<ArcT> arcs;

    int first_arc;
    int first_free_arc;

    const Graph* graph;

    void initalize(const Graph& _graph, NodesImplBase& _nodes) {
      graph = &_graph;
      nodes = &_nodes;
    }

  public:

    class Arc {
      friend class ListArcSetBase<Graph>;
    protected:
      Arc(int _id) : id(_id) {}
      int id;
    public:
      Arc() {}
      Arc(Invalid) : id(-1) {}
      bool operator==(const Arc& arc) const { return id == arc.id; }
      bool operator!=(const Arc& arc) const { return id != arc.id; }
      bool operator<(const Arc& arc) const { return id < arc.id; }
    };

    ListArcSetBase() : first_arc(-1), first_free_arc(-1) {}

    Arc addArc(const Node& u, const Node& v) {
      int n;
      if (first_free_arc == -1) {
        n = arcs.size();
        arcs.push_back(ArcT());
      } else {
        n = first_free_arc;
        first_free_arc = arcs[first_free_arc].next_in;
      }
      arcs[n].next_in = (*nodes)[v].first_in;
      if ((*nodes)[v].first_in != -1) {
        arcs[(*nodes)[v].first_in].prev_in = n;
      }
      (*nodes)[v].first_in = n;
      arcs[n].next_out = (*nodes)[u].first_out;
      if ((*nodes)[u].first_out != -1) {
        arcs[(*nodes)[u].first_out].prev_out = n;
      }
      (*nodes)[u].first_out = n;
      arcs[n].source = u;
      arcs[n].target = v;
      return Arc(n);
    }

    void erase(const Arc& arc) {
      int n = arc.id;
      if (arcs[n].prev_in != -1) {
        arcs[arcs[n].prev_in].next_in = arcs[n].next_in;
      } else {
        (*nodes)[arcs[n].target].first_in = arcs[n].next_in;
      }
      if (arcs[n].next_in != -1) {
        arcs[arcs[n].next_in].prev_in = arcs[n].prev_in;
      }

      if (arcs[n].prev_out != -1) {
        arcs[arcs[n].prev_out].next_out = arcs[n].next_out;
      } else {
        (*nodes)[arcs[n].source].first_out = arcs[n].next_out;
      }
      if (arcs[n].next_out != -1) {
        arcs[arcs[n].next_out].prev_out = arcs[n].prev_out;
      }

    }

    void clear() {
      Node node;
      for (first(node); node != INVALID; next(node)) {
        (*nodes)[node].first_in = -1;
        (*nodes)[node].first_out = -1;
      }
      arcs.clear();
      first_arc = -1;
      first_free_arc = -1;
    }

    void first(Node& node) const {
      graph->first(node);
    }

    void next(Node& node) const {
      graph->next(node);
    }

    void first(Arc& arc) const {
      Node node;
      first(node);
      while (node != INVALID && (*nodes)[node].first_in == -1) {
        next(node);
      }
      arc.id = (node == INVALID) ? -1 : (*nodes)[node].first_in;
    }

    void next(Arc& arc) const {
      if (arcs[arc.id].next_in != -1) {
        arc.id = arcs[arc.id].next_in;
      } else {
        Node node = arcs[arc.id].target;
        next(node);
        while (node != INVALID && (*nodes)[node].first_in == -1) {
          next(node);
        }
        arc.id = (node == INVALID) ? -1 : (*nodes)[node].first_in;
      }
    }

    void firstOut(Arc& arc, const Node& node) const {
      arc.id = (*nodes)[node].first_out;
    }

    void nextOut(Arc& arc) const {
      arc.id = arcs[arc.id].next_out;
    }

    void firstIn(Arc& arc, const Node& node) const {
      arc.id = (*nodes)[node].first_in;
    }

    void nextIn(Arc& arc) const {
      arc.id = arcs[arc.id].next_in;
    }

    int id(const Node& node) const { return graph->id(node); }
    int id(const Arc& arc) const { return arc.id; }

    Node nodeFromId(int ix) const { return graph->nodeFromId(ix); }
    Arc arcFromId(int ix) const { return Arc(ix); }

    int maxNodeId() const { return graph->maxNodeId(); };
    int maxArcId() const { return arcs.size() - 1; }

    Node source(const Arc& arc) const { return arcs[arc.id].source;}
    Node target(const Arc& arc) const { return arcs[arc.id].target;}

    typedef typename ItemSetTraits<Graph, Node>::ItemNotifier NodeNotifier;

    NodeNotifier& notifier(Node) const {
      return graph->notifier(Node());
    }

    template <typename _Value>
    class NodeMap : public Graph::template NodeMap<_Value> {
    public:

      typedef typename _Graph::template NodeMap<_Value> Parent;

      explicit NodeMap(const ListArcSetBase<Graph>& arcset)
        : Parent(*arcset.graph) {}

      NodeMap(const ListArcSetBase<Graph>& arcset, const _Value& value)
        : Parent(*arcset.graph, value) {}

      NodeMap& operator=(const NodeMap& cmap) {
        return operator=<NodeMap>(cmap);
      }

      template <typename CMap>
      NodeMap& operator=(const CMap& cmap) {
        Parent::operator=(cmap);
        return *this;
      }
    };

  };

  /// \ingroup semi_adaptors
  ///
  /// \brief Digraph using a node set of another digraph or graph and
  /// an own arc set.
  ///
  /// This structure can be used to establish another directed graph
  /// over a node set of an existing one. This class uses the same
  /// Node type as the underlying graph, and each valid node of the
  /// original graph is valid in this arc set, therefore the node
  /// objects of the original graph can be used directly with this
  /// class. The node handling functions (id handling, observing, and
  /// iterators) works equivalently as in the original graph.
  ///
  /// This implementation is based on doubly-linked lists, from each
  /// node the outgoing and the incoming arcs make up lists, therefore
  /// one arc can be erased in constant time. It also makes possible,
  /// that node can be removed from the underlying graph, in this case
  /// all arcs incident to the given node is erased from the arc set.
  ///
  /// \param _Graph The type of the graph which shares its node set with
  /// this class. Its interface must conform to the
  /// \ref concepts::Digraph "Digraph" or \ref concepts::Graph "Graph"
  /// concept.
  ///
  /// This class is fully conform to the \ref concepts::Digraph
  /// "Digraph" concept.
  template <typename _Graph>
  class ListArcSet : public ArcSetExtender<ListArcSetBase<_Graph> > {

  public:

    typedef ArcSetExtender<ListArcSetBase<_Graph> > Parent;

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;

    typedef _Graph Graph;


    typedef typename Parent::NodesImplBase NodesImplBase;

    void eraseNode(const Node& node) {
      Arc arc;
      Parent::firstOut(arc, node);
      while (arc != INVALID ) {
        erase(arc);
        Parent::firstOut(arc, node);
      }

      Parent::firstIn(arc, node);
      while (arc != INVALID ) {
        erase(arc);
        Parent::firstIn(arc, node);
      }
    }

    void clearNodes() {
      Parent::clear();
    }

    class NodesImpl : public NodesImplBase {
    public:
      typedef NodesImplBase Parent;

      NodesImpl(const Graph& graph, ListArcSet& arcset)
        : Parent(graph), _arcset(arcset) {}

      virtual ~NodesImpl() {}

    protected:

      virtual void erase(const Node& node) {
        _arcset.eraseNode(node);
        Parent::erase(node);
      }
      virtual void erase(const std::vector<Node>& nodes) {
        for (int i = 0; i < int(nodes.size()); ++i) {
          _arcset.eraseNode(nodes[i]);
        }
        Parent::erase(nodes);
      }
      virtual void clear() {
        _arcset.clearNodes();
        Parent::clear();
      }

    private:
      ListArcSet& _arcset;
    };

    NodesImpl nodes;

  public:

    /// \brief Constructor of the ArcSet.
    ///
    /// Constructor of the ArcSet.
    ListArcSet(const Graph& graph) : nodes(graph, *this) {
      Parent::initalize(graph, nodes);
    }

    /// \brief Add a new arc to the digraph.
    ///
    /// Add a new arc to the digraph with source node \c s
    /// and target node \c t.
    /// \return the new arc.
    Arc addArc(const Node& s, const Node& t) {
      return Parent::addArc(s, t);
    }

    /// \brief Erase an arc from the digraph.
    ///
    /// Erase an arc \c a from the digraph.
    void erase(const Arc& a) {
      return Parent::erase(a);
    }

  };

  template <typename _Graph>
  class ListEdgeSetBase {
  public:

    typedef _Graph Graph;
    typedef typename Graph::Node Node;
    typedef typename Graph::NodeIt NodeIt;

  protected:

    struct NodeT {
      int first_out;
      NodeT() : first_out(-1) {}
    };

    typedef typename ItemSetTraits<Graph, Node>::
    template Map<NodeT>::Type NodesImplBase;

    NodesImplBase* nodes;

    struct ArcT {
      Node target;
      int prev_out, next_out;
      ArcT() : prev_out(-1), next_out(-1) {}
    };

    std::vector<ArcT> arcs;

    int first_arc;
    int first_free_arc;

    const Graph* graph;

    void initalize(const Graph& _graph, NodesImplBase& _nodes) {
      graph = &_graph;
      nodes = &_nodes;
    }

  public:

    class Edge {
      friend class ListEdgeSetBase;
    protected:

      int id;
      explicit Edge(int _id) { id = _id;}

    public:
      Edge() {}
      Edge (Invalid) { id = -1; }
      bool operator==(const Edge& arc) const {return id == arc.id;}
      bool operator!=(const Edge& arc) const {return id != arc.id;}
      bool operator<(const Edge& arc) const {return id < arc.id;}
    };

    class Arc {
      friend class ListEdgeSetBase;
    protected:
      Arc(int _id) : id(_id) {}
      int id;
    public:
      operator Edge() const { return edgeFromId(id / 2); }

      Arc() {}
      Arc(Invalid) : id(-1) {}
      bool operator==(const Arc& arc) const { return id == arc.id; }
      bool operator!=(const Arc& arc) const { return id != arc.id; }
      bool operator<(const Arc& arc) const { return id < arc.id; }
    };

    ListEdgeSetBase() : first_arc(-1), first_free_arc(-1) {}

    Edge addEdge(const Node& u, const Node& v) {
      int n;

      if (first_free_arc == -1) {
        n = arcs.size();
        arcs.push_back(ArcT());
        arcs.push_back(ArcT());
      } else {
        n = first_free_arc;
        first_free_arc = arcs[n].next_out;
      }

      arcs[n].target = u;
      arcs[n | 1].target = v;

      arcs[n].next_out = (*nodes)[v].first_out;
      if ((*nodes)[v].first_out != -1) {
        arcs[(*nodes)[v].first_out].prev_out = n;
      }
      (*nodes)[v].first_out = n;
      arcs[n].prev_out = -1;

      if ((*nodes)[u].first_out != -1) {
        arcs[(*nodes)[u].first_out].prev_out = (n | 1);
      }
      arcs[n | 1].next_out = (*nodes)[u].first_out;
      (*nodes)[u].first_out = (n | 1);
      arcs[n | 1].prev_out = -1;

      return Edge(n / 2);
    }

    void erase(const Edge& arc) {
      int n = arc.id * 2;

      if (arcs[n].next_out != -1) {
        arcs[arcs[n].next_out].prev_out = arcs[n].prev_out;
      }

      if (arcs[n].prev_out != -1) {
        arcs[arcs[n].prev_out].next_out = arcs[n].next_out;
      } else {
        (*nodes)[arcs[n | 1].target].first_out = arcs[n].next_out;
      }

      if (arcs[n | 1].next_out != -1) {
        arcs[arcs[n | 1].next_out].prev_out = arcs[n | 1].prev_out;
      }

      if (arcs[n | 1].prev_out != -1) {
        arcs[arcs[n | 1].prev_out].next_out = arcs[n | 1].next_out;
      } else {
        (*nodes)[arcs[n].target].first_out = arcs[n | 1].next_out;
      }

      arcs[n].next_out = first_free_arc;
      first_free_arc = n;

    }

    void clear() {
      Node node;
      for (first(node); node != INVALID; next(node)) {
        (*nodes)[node].first_out = -1;
      }
      arcs.clear();
      first_arc = -1;
      first_free_arc = -1;
    }

    void first(Node& node) const {
      graph->first(node);
    }

    void next(Node& node) const {
      graph->next(node);
    }

    void first(Arc& arc) const {
      Node node;
      first(node);
      while (node != INVALID && (*nodes)[node].first_out == -1) {
        next(node);
      }
      arc.id = (node == INVALID) ? -1 : (*nodes)[node].first_out;
    }

    void next(Arc& arc) const {
      if (arcs[arc.id].next_out != -1) {
        arc.id = arcs[arc.id].next_out;
      } else {
        Node node = arcs[arc.id ^ 1].target;
        next(node);
        while(node != INVALID && (*nodes)[node].first_out == -1) {
          next(node);
        }
        arc.id = (node == INVALID) ? -1 : (*nodes)[node].first_out;
      }
    }

    void first(Edge& edge) const {
      Node node;
      first(node);
      while (node != INVALID) {
        edge.id = (*nodes)[node].first_out;
        while ((edge.id & 1) != 1) {
          edge.id = arcs[edge.id].next_out;
        }
        if (edge.id != -1) {
          edge.id /= 2;
          return;
        }
        next(node);
      }
      edge.id = -1;
    }

    void next(Edge& edge) const {
      Node node = arcs[edge.id * 2].target;
      edge.id = arcs[(edge.id * 2) | 1].next_out;
      while ((edge.id & 1) != 1) {
        edge.id = arcs[edge.id].next_out;
      }
      if (edge.id != -1) {
        edge.id /= 2;
        return;
      }
      next(node);
      while (node != INVALID) {
        edge.id = (*nodes)[node].first_out;
        while ((edge.id & 1) != 1) {
          edge.id = arcs[edge.id].next_out;
        }
        if (edge.id != -1) {
          edge.id /= 2;
          return;
        }
        next(node);
      }
      edge.id = -1;
    }

    void firstOut(Arc& arc, const Node& node) const {
      arc.id = (*nodes)[node].first_out;
    }

    void nextOut(Arc& arc) const {
      arc.id = arcs[arc.id].next_out;
    }

    void firstIn(Arc& arc, const Node& node) const {
      arc.id = (((*nodes)[node].first_out) ^ 1);
      if (arc.id == -2) arc.id = -1;
    }

    void nextIn(Arc& arc) const {
      arc.id = ((arcs[arc.id ^ 1].next_out) ^ 1);
      if (arc.id == -2) arc.id = -1;
    }

    void firstInc(Edge &arc, bool& dir, const Node& node) const {
      int de = (*nodes)[node].first_out;
      if (de != -1 ) {
        arc.id = de / 2;
        dir = ((de & 1) == 1);
      } else {
        arc.id = -1;
        dir = true;
      }
    }
    void nextInc(Edge &arc, bool& dir) const {
      int de = (arcs[(arc.id * 2) | (dir ? 1 : 0)].next_out);
      if (de != -1 ) {
        arc.id = de / 2;
        dir = ((de & 1) == 1);
      } else {
        arc.id = -1;
        dir = true;
      }
    }

    static bool direction(Arc arc) {
      return (arc.id & 1) == 1;
    }

    static Arc direct(Edge edge, bool dir) {
      return Arc(edge.id * 2 + (dir ? 1 : 0));
    }

    int id(const Node& node) const { return graph->id(node); }
    static int id(Arc e) { return e.id; }
    static int id(Edge e) { return e.id; }

    Node nodeFromId(int id) const { return graph->nodeFromId(id); }
    static Arc arcFromId(int id) { return Arc(id);}
    static Edge edgeFromId(int id) { return Edge(id);}

    int maxNodeId() const { return graph->maxNodeId(); };
    int maxEdgeId() const { return arcs.size() / 2 - 1; }
    int maxArcId() const { return arcs.size()-1; }

    Node source(Arc e) const { return arcs[e.id ^ 1].target; }
    Node target(Arc e) const { return arcs[e.id].target; }

    Node u(Edge e) const { return arcs[2 * e.id].target; }
    Node v(Edge e) const { return arcs[2 * e.id + 1].target; }

    typedef typename ItemSetTraits<Graph, Node>::ItemNotifier NodeNotifier;

    NodeNotifier& notifier(Node) const {
      return graph->notifier(Node());
    }

    template <typename _Value>
    class NodeMap : public Graph::template NodeMap<_Value> {
    public:

      typedef typename _Graph::template NodeMap<_Value> Parent;

      explicit NodeMap(const ListEdgeSetBase<Graph>& arcset)
        : Parent(*arcset.graph) {}

      NodeMap(const ListEdgeSetBase<Graph>& arcset, const _Value& value)
        : Parent(*arcset.graph, value) {}

      NodeMap& operator=(const NodeMap& cmap) {
        return operator=<NodeMap>(cmap);
      }

      template <typename CMap>
      NodeMap& operator=(const CMap& cmap) {
        Parent::operator=(cmap);
        return *this;
      }
    };

  };

  /// \ingroup semi_adaptors
  ///
  /// \brief Graph using a node set of another digraph or graph and an
  /// own edge set.
  ///
  /// This structure can be used to establish another graph over a
  /// node set of an existing one. This class uses the same Node type
  /// as the underlying graph, and each valid node of the original
  /// graph is valid in this arc set, therefore the node objects of
  /// the original graph can be used directly with this class. The
  /// node handling functions (id handling, observing, and iterators)
  /// works equivalently as in the original graph.
  ///
  /// This implementation is based on doubly-linked lists, from each
  /// node the incident edges make up lists, therefore one edge can be
  /// erased in constant time. It also makes possible, that node can
  /// be removed from the underlying graph, in this case all edges
  /// incident to the given node is erased from the arc set.
  ///
  /// \param _Graph The type of the graph which shares its node set
  /// with this class. Its interface must conform to the
  /// \ref concepts::Digraph "Digraph" or \ref concepts::Graph "Graph"
  /// concept.
  ///
  /// This class is fully conform to the \ref concepts::Graph "Graph"
  /// concept.
  template <typename _Graph>
  class ListEdgeSet : public EdgeSetExtender<ListEdgeSetBase<_Graph> > {

  public:

    typedef EdgeSetExtender<ListEdgeSetBase<_Graph> > Parent;

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;
    typedef typename Parent::Edge Edge;

    typedef _Graph Graph;


    typedef typename Parent::NodesImplBase NodesImplBase;

    void eraseNode(const Node& node) {
      Arc arc;
      Parent::firstOut(arc, node);
      while (arc != INVALID ) {
        erase(arc);
        Parent::firstOut(arc, node);
      }

    }

    void clearNodes() {
      Parent::clear();
    }

    class NodesImpl : public NodesImplBase {
    public:
      typedef NodesImplBase Parent;

      NodesImpl(const Graph& graph, ListEdgeSet& arcset)
        : Parent(graph), _arcset(arcset) {}

      virtual ~NodesImpl() {}

    protected:

      virtual void erase(const Node& node) {
        _arcset.eraseNode(node);
        Parent::erase(node);
      }
      virtual void erase(const std::vector<Node>& nodes) {
        for (int i = 0; i < int(nodes.size()); ++i) {
          _arcset.eraseNode(nodes[i]);
        }
        Parent::erase(nodes);
      }
      virtual void clear() {
        _arcset.clearNodes();
        Parent::clear();
      }

    private:
      ListEdgeSet& _arcset;
    };

    NodesImpl nodes;

  public:

    /// \brief Constructor of the EdgeSet.
    ///
    /// Constructor of the EdgeSet.
    ListEdgeSet(const Graph& graph) : nodes(graph, *this) {
      Parent::initalize(graph, nodes);
    }

    /// \brief Add a new edge to the graph.
    ///
    /// Add a new edge to the graph with node \c u
    /// and node \c v endpoints.
    /// \return the new edge.
    Edge addEdge(const Node& u, const Node& v) {
      return Parent::addEdge(u, v);
    }

    /// \brief Erase an edge from the graph.
    ///
    /// Erase the edge \c e from the graph.
    void erase(const Edge& e) {
      return Parent::erase(e);
    }

  };

  template <typename _Graph>
  class SmartArcSetBase {
  public:

    typedef _Graph Graph;
    typedef typename Graph::Node Node;
    typedef typename Graph::NodeIt NodeIt;

  protected:

    struct NodeT {
      int first_out, first_in;
      NodeT() : first_out(-1), first_in(-1) {}
    };

    typedef typename ItemSetTraits<Graph, Node>::
    template Map<NodeT>::Type NodesImplBase;

    NodesImplBase* nodes;

    struct ArcT {
      Node source, target;
      int next_out, next_in;
      ArcT() {}
    };

    std::vector<ArcT> arcs;

    const Graph* graph;

    void initalize(const Graph& _graph, NodesImplBase& _nodes) {
      graph = &_graph;
      nodes = &_nodes;
    }

  public:

    class Arc {
      friend class SmartArcSetBase<Graph>;
    protected:
      Arc(int _id) : id(_id) {}
      int id;
    public:
      Arc() {}
      Arc(Invalid) : id(-1) {}
      bool operator==(const Arc& arc) const { return id == arc.id; }
      bool operator!=(const Arc& arc) const { return id != arc.id; }
      bool operator<(const Arc& arc) const { return id < arc.id; }
    };

    SmartArcSetBase() {}

    Arc addArc(const Node& u, const Node& v) {
      int n = arcs.size();
      arcs.push_back(ArcT());
      arcs[n].next_in = (*nodes)[v].first_in;
      (*nodes)[v].first_in = n;
      arcs[n].next_out = (*nodes)[u].first_out;
      (*nodes)[u].first_out = n;
      arcs[n].source = u;
      arcs[n].target = v;
      return Arc(n);
    }

    void clear() {
      Node node;
      for (first(node); node != INVALID; next(node)) {
        (*nodes)[node].first_in = -1;
        (*nodes)[node].first_out = -1;
      }
      arcs.clear();
    }

    void first(Node& node) const {
      graph->first(node);
    }

    void next(Node& node) const {
      graph->next(node);
    }

    void first(Arc& arc) const {
      arc.id = arcs.size() - 1;
    }

    void next(Arc& arc) const {
      --arc.id;
    }

    void firstOut(Arc& arc, const Node& node) const {
      arc.id = (*nodes)[node].first_out;
    }

    void nextOut(Arc& arc) const {
      arc.id = arcs[arc.id].next_out;
    }

    void firstIn(Arc& arc, const Node& node) const {
      arc.id = (*nodes)[node].first_in;
    }

    void nextIn(Arc& arc) const {
      arc.id = arcs[arc.id].next_in;
    }

    int id(const Node& node) const { return graph->id(node); }
    int id(const Arc& arc) const { return arc.id; }

    Node nodeFromId(int ix) const { return graph->nodeFromId(ix); }
    Arc arcFromId(int ix) const { return Arc(ix); }

    int maxNodeId() const { return graph->maxNodeId(); };
    int maxArcId() const { return arcs.size() - 1; }

    Node source(const Arc& arc) const { return arcs[arc.id].source;}
    Node target(const Arc& arc) const { return arcs[arc.id].target;}

    typedef typename ItemSetTraits<Graph, Node>::ItemNotifier NodeNotifier;

    NodeNotifier& notifier(Node) const {
      return graph->notifier(Node());
    }

    template <typename _Value>
    class NodeMap : public Graph::template NodeMap<_Value> {
    public:

      typedef typename _Graph::template NodeMap<_Value> Parent;

      explicit NodeMap(const SmartArcSetBase<Graph>& arcset)
        : Parent(*arcset.graph) { }

      NodeMap(const SmartArcSetBase<Graph>& arcset, const _Value& value)
        : Parent(*arcset.graph, value) { }

      NodeMap& operator=(const NodeMap& cmap) {
        return operator=<NodeMap>(cmap);
      }

      template <typename CMap>
      NodeMap& operator=(const CMap& cmap) {
        Parent::operator=(cmap);
        return *this;
      }
    };

  };


  /// \ingroup semi_adaptors
  ///
  /// \brief Digraph using a node set of another digraph or graph and
  /// an own arc set.
  ///
  /// This structure can be used to establish another directed graph
  /// over a node set of an existing one. This class uses the same
  /// Node type as the underlying graph, and each valid node of the
  /// original graph is valid in this arc set, therefore the node
  /// objects of the original graph can be used directly with this
  /// class. The node handling functions (id handling, observing, and
  /// iterators) works equivalently as in the original graph.
  ///
  /// \param _Graph The type of the graph which shares its node set with
  /// this class. Its interface must conform to the
  /// \ref concepts::Digraph "Digraph" or \ref concepts::Graph "Graph"
  /// concept.
  ///
  /// This implementation is slightly faster than the \c ListArcSet,
  /// because it uses continuous storage for arcs and it uses just
  /// single-linked lists for enumerate outgoing and incoming
  /// arcs. Therefore the arcs cannot be erased from the arc sets.
  ///
  /// \warning If a node is erased from the underlying graph and this
  /// node is the source or target of one arc in the arc set, then
  /// the arc set is invalidated, and it cannot be used anymore. The
  /// validity can be checked with the \c valid() member function.
  ///
  /// This class is fully conform to the \ref concepts::Digraph
  /// "Digraph" concept.
  template <typename _Graph>
  class SmartArcSet : public ArcSetExtender<SmartArcSetBase<_Graph> > {

  public:

    typedef ArcSetExtender<SmartArcSetBase<_Graph> > Parent;

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;

    typedef _Graph Graph;

  protected:

    typedef typename Parent::NodesImplBase NodesImplBase;

    void eraseNode(const Node& node) {
      if (typename Parent::InArcIt(*this, node) == INVALID &&
          typename Parent::OutArcIt(*this, node) == INVALID) {
        return;
      }
      throw typename NodesImplBase::Notifier::ImmediateDetach();
    }

    void clearNodes() {
      Parent::clear();
    }

    class NodesImpl : public NodesImplBase {
    public:
      typedef NodesImplBase Parent;

      NodesImpl(const Graph& graph, SmartArcSet& arcset)
        : Parent(graph), _arcset(arcset) {}

      virtual ~NodesImpl() {}

      bool attached() const {
        return Parent::attached();
      }

    protected:

      virtual void erase(const Node& node) {
        try {
          _arcset.eraseNode(node);
          Parent::erase(node);
        } catch (const typename NodesImplBase::Notifier::ImmediateDetach&) {
          Parent::clear();
          throw;
        }
      }
      virtual void erase(const std::vector<Node>& nodes) {
        try {
          for (int i = 0; i < int(nodes.size()); ++i) {
            _arcset.eraseNode(nodes[i]);
          }
          Parent::erase(nodes);
        } catch (const typename NodesImplBase::Notifier::ImmediateDetach&) {
          Parent::clear();
          throw;
        }
      }
      virtual void clear() {
        _arcset.clearNodes();
        Parent::clear();
      }

    private:
      SmartArcSet& _arcset;
    };

    NodesImpl nodes;

  public:

    /// \brief Constructor of the ArcSet.
    ///
    /// Constructor of the ArcSet.
    SmartArcSet(const Graph& graph) : nodes(graph, *this) {
      Parent::initalize(graph, nodes);
    }

    /// \brief Add a new arc to the digraph.
    ///
    /// Add a new arc to the digraph with source node \c s
    /// and target node \c t.
    /// \return the new arc.
    Arc addArc(const Node& s, const Node& t) {
      return Parent::addArc(s, t);
    }

    /// \brief Validity check
    ///
    /// This functions gives back false if the ArcSet is
    /// invalidated. It occurs when a node in the underlying graph is
    /// erased and it is not isolated in the ArcSet.
    bool valid() const {
      return nodes.attached();
    }

  };


  template <typename _Graph>
  class SmartEdgeSetBase {
  public:

    typedef _Graph Graph;
    typedef typename Graph::Node Node;
    typedef typename Graph::NodeIt NodeIt;

  protected:

    struct NodeT {
      int first_out;
      NodeT() : first_out(-1) {}
    };

    typedef typename ItemSetTraits<Graph, Node>::
    template Map<NodeT>::Type NodesImplBase;

    NodesImplBase* nodes;

    struct ArcT {
      Node target;
      int next_out;
      ArcT() {}
    };

    std::vector<ArcT> arcs;

    const Graph* graph;

    void initalize(const Graph& _graph, NodesImplBase& _nodes) {
      graph = &_graph;
      nodes = &_nodes;
    }

  public:

    class Edge {
      friend class SmartEdgeSetBase;
    protected:

      int id;
      explicit Edge(int _id) { id = _id;}

    public:
      Edge() {}
      Edge (Invalid) { id = -1; }
      bool operator==(const Edge& arc) const {return id == arc.id;}
      bool operator!=(const Edge& arc) const {return id != arc.id;}
      bool operator<(const Edge& arc) const {return id < arc.id;}
    };

    class Arc {
      friend class SmartEdgeSetBase;
    protected:
      Arc(int _id) : id(_id) {}
      int id;
    public:
      operator Edge() const { return edgeFromId(id / 2); }

      Arc() {}
      Arc(Invalid) : id(-1) {}
      bool operator==(const Arc& arc) const { return id == arc.id; }
      bool operator!=(const Arc& arc) const { return id != arc.id; }
      bool operator<(const Arc& arc) const { return id < arc.id; }
    };

    SmartEdgeSetBase() {}

    Edge addEdge(const Node& u, const Node& v) {
      int n = arcs.size();
      arcs.push_back(ArcT());
      arcs.push_back(ArcT());

      arcs[n].target = u;
      arcs[n | 1].target = v;

      arcs[n].next_out = (*nodes)[v].first_out;
      (*nodes)[v].first_out = n;

      arcs[n | 1].next_out = (*nodes)[u].first_out;
      (*nodes)[u].first_out = (n | 1);

      return Edge(n / 2);
    }

    void clear() {
      Node node;
      for (first(node); node != INVALID; next(node)) {
        (*nodes)[node].first_out = -1;
      }
      arcs.clear();
    }

    void first(Node& node) const {
      graph->first(node);
    }

    void next(Node& node) const {
      graph->next(node);
    }

    void first(Arc& arc) const {
      arc.id = arcs.size() - 1;
    }

    void next(Arc& arc) const {
      --arc.id;
    }

    void first(Edge& arc) const {
      arc.id = arcs.size() / 2 - 1;
    }

    void next(Edge& arc) const {
      --arc.id;
    }

    void firstOut(Arc& arc, const Node& node) const {
      arc.id = (*nodes)[node].first_out;
    }

    void nextOut(Arc& arc) const {
      arc.id = arcs[arc.id].next_out;
    }

    void firstIn(Arc& arc, const Node& node) const {
      arc.id = (((*nodes)[node].first_out) ^ 1);
      if (arc.id == -2) arc.id = -1;
    }

    void nextIn(Arc& arc) const {
      arc.id = ((arcs[arc.id ^ 1].next_out) ^ 1);
      if (arc.id == -2) arc.id = -1;
    }

    void firstInc(Edge &arc, bool& dir, const Node& node) const {
      int de = (*nodes)[node].first_out;
      if (de != -1 ) {
        arc.id = de / 2;
        dir = ((de & 1) == 1);
      } else {
        arc.id = -1;
        dir = true;
      }
    }
    void nextInc(Edge &arc, bool& dir) const {
      int de = (arcs[(arc.id * 2) | (dir ? 1 : 0)].next_out);
      if (de != -1 ) {
        arc.id = de / 2;
        dir = ((de & 1) == 1);
      } else {
        arc.id = -1;
        dir = true;
      }
    }

    static bool direction(Arc arc) {
      return (arc.id & 1) == 1;
    }

    static Arc direct(Edge edge, bool dir) {
      return Arc(edge.id * 2 + (dir ? 1 : 0));
    }

    int id(Node node) const { return graph->id(node); }
    static int id(Arc arc) { return arc.id; }
    static int id(Edge arc) { return arc.id; }

    Node nodeFromId(int id) const { return graph->nodeFromId(id); }
    static Arc arcFromId(int id) { return Arc(id); }
    static Edge edgeFromId(int id) { return Edge(id);}

    int maxNodeId() const { return graph->maxNodeId(); };
    int maxArcId() const { return arcs.size() - 1; }
    int maxEdgeId() const { return arcs.size() / 2 - 1; }

    Node source(Arc e) const { return arcs[e.id ^ 1].target; }
    Node target(Arc e) const { return arcs[e.id].target; }

    Node u(Edge e) const { return arcs[2 * e.id].target; }
    Node v(Edge e) const { return arcs[2 * e.id + 1].target; }

    typedef typename ItemSetTraits<Graph, Node>::ItemNotifier NodeNotifier;

    NodeNotifier& notifier(Node) const {
      return graph->notifier(Node());
    }

    template <typename _Value>
    class NodeMap : public Graph::template NodeMap<_Value> {
    public:

      typedef typename _Graph::template NodeMap<_Value> Parent;

      explicit NodeMap(const SmartEdgeSetBase<Graph>& arcset)
        : Parent(*arcset.graph) { }

      NodeMap(const SmartEdgeSetBase<Graph>& arcset, const _Value& value)
        : Parent(*arcset.graph, value) { }

      NodeMap& operator=(const NodeMap& cmap) {
        return operator=<NodeMap>(cmap);
      }

      template <typename CMap>
      NodeMap& operator=(const CMap& cmap) {
        Parent::operator=(cmap);
        return *this;
      }
    };

  };

  /// \ingroup semi_adaptors
  ///
  /// \brief Graph using a node set of another digraph or graph and an
  /// own edge set.
  ///
  /// This structure can be used to establish another graph over a
  /// node set of an existing one. This class uses the same Node type
  /// as the underlying graph, and each valid node of the original
  /// graph is valid in this arc set, therefore the node objects of
  /// the original graph can be used directly with this class. The
  /// node handling functions (id handling, observing, and iterators)
  /// works equivalently as in the original graph.
  ///
  /// \param _Graph The type of the graph which shares its node set
  /// with this class. Its interface must conform to the
  /// \ref concepts::Digraph "Digraph" or \ref concepts::Graph "Graph"
  ///  concept.
  ///
  /// This implementation is slightly faster than the \c ListEdgeSet,
  /// because it uses continuous storage for edges and it uses just
  /// single-linked lists for enumerate incident edges. Therefore the
  /// edges cannot be erased from the edge sets.
  ///
  /// \warning If a node is erased from the underlying graph and this
  /// node is incident to one edge in the edge set, then the edge set
  /// is invalidated, and it cannot be used anymore. The validity can
  /// be checked with the \c valid() member function.
  ///
  /// This class is fully conform to the \ref concepts::Graph
  /// "Graph" concept.
  template <typename _Graph>
  class SmartEdgeSet : public EdgeSetExtender<SmartEdgeSetBase<_Graph> > {

  public:

    typedef EdgeSetExtender<SmartEdgeSetBase<_Graph> > Parent;

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;
    typedef typename Parent::Edge Edge;

    typedef _Graph Graph;

  protected:

    typedef typename Parent::NodesImplBase NodesImplBase;

    void eraseNode(const Node& node) {
      if (typename Parent::IncEdgeIt(*this, node) == INVALID) {
        return;
      }
      throw typename NodesImplBase::Notifier::ImmediateDetach();
    }

    void clearNodes() {
      Parent::clear();
    }

    class NodesImpl : public NodesImplBase {
    public:
      typedef NodesImplBase Parent;

      NodesImpl(const Graph& graph, SmartEdgeSet& arcset)
        : Parent(graph), _arcset(arcset) {}

      virtual ~NodesImpl() {}

      bool attached() const {
        return Parent::attached();
      }

    protected:

      virtual void erase(const Node& node) {
        try {
          _arcset.eraseNode(node);
          Parent::erase(node);
        } catch (const typename NodesImplBase::Notifier::ImmediateDetach&) {
          Parent::clear();
          throw;
        }
      }
      virtual void erase(const std::vector<Node>& nodes) {
        try {
          for (int i = 0; i < int(nodes.size()); ++i) {
            _arcset.eraseNode(nodes[i]);
          }
          Parent::erase(nodes);
        } catch (const typename NodesImplBase::Notifier::ImmediateDetach&) {
          Parent::clear();
          throw;
        }
      }
      virtual void clear() {
        _arcset.clearNodes();
        Parent::clear();
      }

    private:
      SmartEdgeSet& _arcset;
    };

    NodesImpl nodes;

  public:

    /// \brief Constructor of the EdgeSet.
    ///
    /// Constructor of the EdgeSet.
    SmartEdgeSet(const Graph& graph) : nodes(graph, *this) {
      Parent::initalize(graph, nodes);
    }

    /// \brief Add a new edge to the graph.
    ///
    /// Add a new edge to the graph with node \c u
    /// and node \c v endpoints.
    /// \return the new edge.
    Edge addEdge(const Node& u, const Node& v) {
      return Parent::addEdge(u, v);
    }

    /// \brief Validity check
    ///
    /// This functions gives back false if the EdgeSet is
    /// invalidated. It occurs when a node in the underlying graph is
    /// erased and it is not isolated in the EdgeSet.
    bool valid() const {
      return nodes.attached();
    }

  };

}

#endif
