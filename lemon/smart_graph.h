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

#ifndef LEMON_SMART_GRAPH_H
#define LEMON_SMART_GRAPH_H

///\ingroup graphs
///\file
///\brief SmartDigraph and SmartGraph classes.

#include <vector>

#include <lemon/core.h>
#include <lemon/error.h>
#include <lemon/bits/graph_extender.h>

namespace lemon {

  class SmartDigraph;
  ///Base of SmartDigraph

  ///Base of SmartDigraph
  ///
  class SmartDigraphBase {
  protected:

    struct NodeT
    {
      int first_in, first_out;
      NodeT() {}
    };
    struct ArcT
    {
      int target, source, next_in, next_out;
      ArcT() {}
    };

    std::vector<NodeT> nodes;
    std::vector<ArcT> arcs;

  public:

    typedef SmartDigraphBase Graph;

    class Node;
    class Arc;

  public:

    SmartDigraphBase() : nodes(), arcs() { }
    SmartDigraphBase(const SmartDigraphBase &_g)
      : nodes(_g.nodes), arcs(_g.arcs) { }

    typedef True NodeNumTag;
    typedef True EdgeNumTag;

    int nodeNum() const { return nodes.size(); }
    int arcNum() const { return arcs.size(); }

    int maxNodeId() const { return nodes.size()-1; }
    int maxArcId() const { return arcs.size()-1; }

    Node addNode() {
      int n = nodes.size();
      nodes.push_back(NodeT());
      nodes[n].first_in = -1;
      nodes[n].first_out = -1;
      return Node(n);
    }

    Arc addArc(Node u, Node v) {
      int n = arcs.size();
      arcs.push_back(ArcT());
      arcs[n].source = u._id;
      arcs[n].target = v._id;
      arcs[n].next_out = nodes[u._id].first_out;
      arcs[n].next_in = nodes[v._id].first_in;
      nodes[u._id].first_out = nodes[v._id].first_in = n;

      return Arc(n);
    }

    void clear() {
      arcs.clear();
      nodes.clear();
    }

    Node source(Arc a) const { return Node(arcs[a._id].source); }
    Node target(Arc a) const { return Node(arcs[a._id].target); }

    static int id(Node v) { return v._id; }
    static int id(Arc a) { return a._id; }

    static Node nodeFromId(int id) { return Node(id);}
    static Arc arcFromId(int id) { return Arc(id);}

    bool valid(Node n) const {
      return n._id >= 0 && n._id < static_cast<int>(nodes.size());
    }
    bool valid(Arc a) const {
      return a._id >= 0 && a._id < static_cast<int>(arcs.size());
    }

    class Node {
      friend class SmartDigraphBase;
      friend class SmartDigraph;

    protected:
      int _id;
      explicit Node(int id) : _id(id) {}
    public:
      Node() {}
      Node (Invalid) : _id(-1) {}
      bool operator==(const Node i) const {return _id == i._id;}
      bool operator!=(const Node i) const {return _id != i._id;}
      bool operator<(const Node i) const {return _id < i._id;}
    };


    class Arc {
      friend class SmartDigraphBase;
      friend class SmartDigraph;

    protected:
      int _id;
      explicit Arc(int id) : _id(id) {}
    public:
      Arc() { }
      Arc (Invalid) : _id(-1) {}
      bool operator==(const Arc i) const {return _id == i._id;}
      bool operator!=(const Arc i) const {return _id != i._id;}
      bool operator<(const Arc i) const {return _id < i._id;}
    };

    void first(Node& node) const {
      node._id = nodes.size() - 1;
    }

    static void next(Node& node) {
      --node._id;
    }

    void first(Arc& arc) const {
      arc._id = arcs.size() - 1;
    }

    static void next(Arc& arc) {
      --arc._id;
    }

    void firstOut(Arc& arc, const Node& node) const {
      arc._id = nodes[node._id].first_out;
    }

    void nextOut(Arc& arc) const {
      arc._id = arcs[arc._id].next_out;
    }

    void firstIn(Arc& arc, const Node& node) const {
      arc._id = nodes[node._id].first_in;
    }

    void nextIn(Arc& arc) const {
      arc._id = arcs[arc._id].next_in;
    }

  };

  typedef DigraphExtender<SmartDigraphBase> ExtendedSmartDigraphBase;

  ///\ingroup graphs
  ///
  ///\brief A smart directed graph class.
  ///
  ///This is a simple and fast digraph implementation.
  ///It is also quite memory efficient, but at the price
  ///that <b> it does support only limited (only stack-like)
  ///node and arc deletions</b>.
  ///It conforms to the \ref concepts::Digraph "Digraph concept" with
  ///an important extra feature that its maps are real \ref
  ///concepts::ReferenceMap "reference map"s.
  ///
  ///\sa concepts::Digraph.
  class SmartDigraph : public ExtendedSmartDigraphBase {
  public:

    typedef ExtendedSmartDigraphBase Parent;

  private:

    ///SmartDigraph is \e not copy constructible. Use DigraphCopy() instead.

    ///SmartDigraph is \e not copy constructible. Use DigraphCopy() instead.
    ///
    SmartDigraph(const SmartDigraph &) : ExtendedSmartDigraphBase() {};
    ///\brief Assignment of SmartDigraph to another one is \e not allowed.
    ///Use DigraphCopy() instead.

    ///Assignment of SmartDigraph to another one is \e not allowed.
    ///Use DigraphCopy() instead.
    void operator=(const SmartDigraph &) {}

  public:

    /// Constructor

    /// Constructor.
    ///
    SmartDigraph() {};

    ///Add a new node to the digraph.

    /// \return the new node.
    ///
    Node addNode() { return Parent::addNode(); }

    ///Add a new arc to the digraph.

    ///Add a new arc to the digraph with source node \c s
    ///and target node \c t.
    ///\return the new arc.
    Arc addArc(const Node& s, const Node& t) {
      return Parent::addArc(s, t);
    }

    /// \brief Using this it is possible to avoid the superfluous memory
    /// allocation.

    /// Using this it is possible to avoid the superfluous memory
    /// allocation: if you know that the digraph you want to build will
    /// be very large (e.g. it will contain millions of nodes and/or arcs)
    /// then it is worth reserving space for this amount before starting
    /// to build the digraph.
    /// \sa reserveArc
    void reserveNode(int n) { nodes.reserve(n); };

    /// \brief Using this it is possible to avoid the superfluous memory
    /// allocation.

    /// Using this it is possible to avoid the superfluous memory
    /// allocation: if you know that the digraph you want to build will
    /// be very large (e.g. it will contain millions of nodes and/or arcs)
    /// then it is worth reserving space for this amount before starting
    /// to build the digraph.
    /// \sa reserveNode
    void reserveArc(int m) { arcs.reserve(m); };

    /// \brief Node validity check
    ///
    /// This function gives back true if the given node is valid,
    /// ie. it is a real node of the graph.
    ///
    /// \warning A removed node (using Snapshot) could become valid again
    /// when new nodes are added to the graph.
    bool valid(Node n) const { return Parent::valid(n); }

    /// \brief Arc validity check
    ///
    /// This function gives back true if the given arc is valid,
    /// ie. it is a real arc of the graph.
    ///
    /// \warning A removed arc (using Snapshot) could become valid again
    /// when new arcs are added to the graph.
    bool valid(Arc a) const { return Parent::valid(a); }

    ///Clear the digraph.

    ///Erase all the nodes and arcs from the digraph.
    ///
    void clear() {
      Parent::clear();
    }

    ///Split a node.

    ///This function splits a node. First a new node is added to the digraph,
    ///then the source of each outgoing arc of \c n is moved to this new node.
    ///If \c connect is \c true (this is the default value), then a new arc
    ///from \c n to the newly created node is also added.
    ///\return The newly created node.
    ///
    ///\note The <tt>Arc</tt>s
    ///referencing a moved arc remain
    ///valid. However <tt>InArc</tt>'s and <tt>OutArc</tt>'s
    ///may be invalidated.
    ///\warning This functionality cannot be used together with the Snapshot
    ///feature.
    Node split(Node n, bool connect = true)
    {
      Node b = addNode();
      nodes[b._id].first_out=nodes[n._id].first_out;
      nodes[n._id].first_out=-1;
      for(int i=nodes[b._id].first_out;i!=-1;i++) arcs[i].source=b._id;
      if(connect) addArc(n,b);
      return b;
    }

  public:

    class Snapshot;

  protected:

    void restoreSnapshot(const Snapshot &s)
    {
      while(s.arc_num<arcs.size()) {
        Arc arc = arcFromId(arcs.size()-1);
        Parent::notifier(Arc()).erase(arc);
        nodes[arcs.back().source].first_out=arcs.back().next_out;
        nodes[arcs.back().target].first_in=arcs.back().next_in;
        arcs.pop_back();
      }
      while(s.node_num<nodes.size()) {
        Node node = nodeFromId(nodes.size()-1);
        Parent::notifier(Node()).erase(node);
        nodes.pop_back();
      }
    }

  public:

    ///Class to make a snapshot of the digraph and to restrore to it later.

    ///Class to make a snapshot of the digraph and to restrore to it later.
    ///
    ///The newly added nodes and arcs can be removed using the
    ///restore() function.
    ///\note After you restore a state, you cannot restore
    ///a later state, in other word you cannot add again the arcs deleted
    ///by restore() using another one Snapshot instance.
    ///
    ///\warning If you do not use correctly the snapshot that can cause
    ///either broken program, invalid state of the digraph, valid but
    ///not the restored digraph or no change. Because the runtime performance
    ///the validity of the snapshot is not stored.
    class Snapshot
    {
      SmartDigraph *_graph;
    protected:
      friend class SmartDigraph;
      unsigned int node_num;
      unsigned int arc_num;
    public:
      ///Default constructor.

      ///Default constructor.
      ///To actually make a snapshot you must call save().
      ///
      Snapshot() : _graph(0) {}
      ///Constructor that immediately makes a snapshot

      ///This constructor immediately makes a snapshot of the digraph.
      ///\param graph The digraph we make a snapshot of.
      Snapshot(SmartDigraph &graph) : _graph(&graph) {
        node_num=_graph->nodes.size();
        arc_num=_graph->arcs.size();
      }

      ///Make a snapshot.

      ///Make a snapshot of the digraph.
      ///
      ///This function can be called more than once. In case of a repeated
      ///call, the previous snapshot gets lost.
      ///\param graph The digraph we make the snapshot of.
      void save(SmartDigraph &graph)
      {
        _graph=&graph;
        node_num=_graph->nodes.size();
        arc_num=_graph->arcs.size();
      }

      ///Undo the changes until a snapshot.

      ///Undo the changes until a snapshot created by save().
      ///
      ///\note After you restored a state, you cannot restore
      ///a later state, in other word you cannot add again the arcs deleted
      ///by restore().
      void restore()
      {
        _graph->restoreSnapshot(*this);
      }
    };
  };


  class SmartGraphBase {

  protected:

    struct NodeT {
      int first_out;
    };

    struct ArcT {
      int target;
      int next_out;
    };

    std::vector<NodeT> nodes;
    std::vector<ArcT> arcs;

    int first_free_arc;

  public:

    typedef SmartGraphBase Digraph;

    class Node;
    class Arc;
    class Edge;

    class Node {
      friend class SmartGraphBase;
    protected:

      int _id;
      explicit Node(int id) { _id = id;}

    public:
      Node() {}
      Node (Invalid) { _id = -1; }
      bool operator==(const Node& node) const {return _id == node._id;}
      bool operator!=(const Node& node) const {return _id != node._id;}
      bool operator<(const Node& node) const {return _id < node._id;}
    };

    class Edge {
      friend class SmartGraphBase;
    protected:

      int _id;
      explicit Edge(int id) { _id = id;}

    public:
      Edge() {}
      Edge (Invalid) { _id = -1; }
      bool operator==(const Edge& arc) const {return _id == arc._id;}
      bool operator!=(const Edge& arc) const {return _id != arc._id;}
      bool operator<(const Edge& arc) const {return _id < arc._id;}
    };

    class Arc {
      friend class SmartGraphBase;
    protected:

      int _id;
      explicit Arc(int id) { _id = id;}

    public:
      operator Edge() const { 
        return _id != -1 ? edgeFromId(_id / 2) : INVALID; 
      }

      Arc() {}
      Arc (Invalid) { _id = -1; }
      bool operator==(const Arc& arc) const {return _id == arc._id;}
      bool operator!=(const Arc& arc) const {return _id != arc._id;}
      bool operator<(const Arc& arc) const {return _id < arc._id;}
    };



    SmartGraphBase()
      : nodes(), arcs() {}


    int maxNodeId() const { return nodes.size()-1; }
    int maxEdgeId() const { return arcs.size() / 2 - 1; }
    int maxArcId() const { return arcs.size()-1; }

    Node source(Arc e) const { return Node(arcs[e._id ^ 1].target); }
    Node target(Arc e) const { return Node(arcs[e._id].target); }

    Node u(Edge e) const { return Node(arcs[2 * e._id].target); }
    Node v(Edge e) const { return Node(arcs[2 * e._id + 1].target); }

    static bool direction(Arc e) {
      return (e._id & 1) == 1;
    }

    static Arc direct(Edge e, bool d) {
      return Arc(e._id * 2 + (d ? 1 : 0));
    }

    void first(Node& node) const {
      node._id = nodes.size() - 1;
    }

    void next(Node& node) const {
      --node._id;
    }

    void first(Arc& arc) const {
      arc._id = arcs.size() - 1;
    }

    void next(Arc& arc) const {
      --arc._id;
    }

    void first(Edge& arc) const {
      arc._id = arcs.size() / 2 - 1;
    }

    void next(Edge& arc) const {
      --arc._id;
    }

    void firstOut(Arc &arc, const Node& v) const {
      arc._id = nodes[v._id].first_out;
    }
    void nextOut(Arc &arc) const {
      arc._id = arcs[arc._id].next_out;
    }

    void firstIn(Arc &arc, const Node& v) const {
      arc._id = ((nodes[v._id].first_out) ^ 1);
      if (arc._id == -2) arc._id = -1;
    }
    void nextIn(Arc &arc) const {
      arc._id = ((arcs[arc._id ^ 1].next_out) ^ 1);
      if (arc._id == -2) arc._id = -1;
    }

    void firstInc(Edge &arc, bool& d, const Node& v) const {
      int de = nodes[v._id].first_out;
      if (de != -1) {
        arc._id = de / 2;
        d = ((de & 1) == 1);
      } else {
        arc._id = -1;
        d = true;
      }
    }
    void nextInc(Edge &arc, bool& d) const {
      int de = (arcs[(arc._id * 2) | (d ? 1 : 0)].next_out);
      if (de != -1) {
        arc._id = de / 2;
        d = ((de & 1) == 1);
      } else {
        arc._id = -1;
        d = true;
      }
    }

    static int id(Node v) { return v._id; }
    static int id(Arc e) { return e._id; }
    static int id(Edge e) { return e._id; }

    static Node nodeFromId(int id) { return Node(id);}
    static Arc arcFromId(int id) { return Arc(id);}
    static Edge edgeFromId(int id) { return Edge(id);}

    bool valid(Node n) const {
      return n._id >= 0 && n._id < static_cast<int>(nodes.size());
    }
    bool valid(Arc a) const {
      return a._id >= 0 && a._id < static_cast<int>(arcs.size());
    }
    bool valid(Edge e) const {
      return e._id >= 0 && 2 * e._id < static_cast<int>(arcs.size());
    }

    Node addNode() {
      int n = nodes.size();
      nodes.push_back(NodeT());
      nodes[n].first_out = -1;

      return Node(n);
    }

    Edge addEdge(Node u, Node v) {
      int n = arcs.size();
      arcs.push_back(ArcT());
      arcs.push_back(ArcT());

      arcs[n].target = u._id;
      arcs[n | 1].target = v._id;

      arcs[n].next_out = nodes[v._id].first_out;
      nodes[v._id].first_out = n;

      arcs[n | 1].next_out = nodes[u._id].first_out;
      nodes[u._id].first_out = (n | 1);

      return Edge(n / 2);
    }

    void clear() {
      arcs.clear();
      nodes.clear();
    }

  };

  typedef GraphExtender<SmartGraphBase> ExtendedSmartGraphBase;

  /// \ingroup graphs
  ///
  /// \brief A smart undirected graph class.
  ///
  /// This is a simple and fast graph implementation.
  /// It is also quite memory efficient, but at the price
  /// that <b> it does support only limited (only stack-like)
  /// node and arc deletions</b>.
  /// Except from this it conforms to
  /// the \ref concepts::Graph "Graph concept".
  ///
  /// It also has an
  /// important extra feature that
  /// its maps are real \ref concepts::ReferenceMap "reference map"s.
  ///
  /// \sa concepts::Graph.
  ///
  class SmartGraph : public ExtendedSmartGraphBase {
  private:

    ///SmartGraph is \e not copy constructible. Use GraphCopy() instead.

    ///SmartGraph is \e not copy constructible. Use GraphCopy() instead.
    ///
    SmartGraph(const SmartGraph &) : ExtendedSmartGraphBase() {};

    ///\brief Assignment of SmartGraph to another one is \e not allowed.
    ///Use GraphCopy() instead.

    ///Assignment of SmartGraph to another one is \e not allowed.
    ///Use GraphCopy() instead.
    void operator=(const SmartGraph &) {}

  public:

    typedef ExtendedSmartGraphBase Parent;

    /// Constructor

    /// Constructor.
    ///
    SmartGraph() {}

    ///Add a new node to the graph.

    /// \return the new node.
    ///
    Node addNode() { return Parent::addNode(); }

    ///Add a new edge to the graph.

    ///Add a new edge to the graph with node \c s
    ///and \c t.
    ///\return the new edge.
    Edge addEdge(const Node& s, const Node& t) {
      return Parent::addEdge(s, t);
    }

    /// \brief Node validity check
    ///
    /// This function gives back true if the given node is valid,
    /// ie. it is a real node of the graph.
    ///
    /// \warning A removed node (using Snapshot) could become valid again
    /// when new nodes are added to the graph.
    bool valid(Node n) const { return Parent::valid(n); }

    /// \brief Arc validity check
    ///
    /// This function gives back true if the given arc is valid,
    /// ie. it is a real arc of the graph.
    ///
    /// \warning A removed arc (using Snapshot) could become valid again
    /// when new edges are added to the graph.
    bool valid(Arc a) const { return Parent::valid(a); }

    /// \brief Edge validity check
    ///
    /// This function gives back true if the given edge is valid,
    /// ie. it is a real edge of the graph.
    ///
    /// \warning A removed edge (using Snapshot) could become valid again
    /// when new edges are added to the graph.
    bool valid(Edge e) const { return Parent::valid(e); }

    ///Clear the graph.

    ///Erase all the nodes and edges from the graph.
    ///
    void clear() {
      Parent::clear();
    }

  public:

    class Snapshot;

  protected:

    void saveSnapshot(Snapshot &s)
    {
      s._graph = this;
      s.node_num = nodes.size();
      s.arc_num = arcs.size();
    }

    void restoreSnapshot(const Snapshot &s)
    {
      while(s.arc_num<arcs.size()) {
        int n=arcs.size()-1;
        Edge arc=edgeFromId(n/2);
        Parent::notifier(Edge()).erase(arc);
        std::vector<Arc> dir;
        dir.push_back(arcFromId(n));
        dir.push_back(arcFromId(n-1));
        Parent::notifier(Arc()).erase(dir);
        nodes[arcs[n].target].first_out=arcs[n].next_out;
        nodes[arcs[n-1].target].first_out=arcs[n-1].next_out;
        arcs.pop_back();
        arcs.pop_back();
      }
      while(s.node_num<nodes.size()) {
        int n=nodes.size()-1;
        Node node = nodeFromId(n);
        Parent::notifier(Node()).erase(node);
        nodes.pop_back();
      }
    }

  public:

    ///Class to make a snapshot of the digraph and to restrore to it later.

    ///Class to make a snapshot of the digraph and to restrore to it later.
    ///
    ///The newly added nodes and arcs can be removed using the
    ///restore() function.
    ///
    ///\note After you restore a state, you cannot restore
    ///a later state, in other word you cannot add again the arcs deleted
    ///by restore() using another one Snapshot instance.
    ///
    ///\warning If you do not use correctly the snapshot that can cause
    ///either broken program, invalid state of the digraph, valid but
    ///not the restored digraph or no change. Because the runtime performance
    ///the validity of the snapshot is not stored.
    class Snapshot
    {
      SmartGraph *_graph;
    protected:
      friend class SmartGraph;
      unsigned int node_num;
      unsigned int arc_num;
    public:
      ///Default constructor.

      ///Default constructor.
      ///To actually make a snapshot you must call save().
      ///
      Snapshot() : _graph(0) {}
      ///Constructor that immediately makes a snapshot

      ///This constructor immediately makes a snapshot of the digraph.
      ///\param graph The digraph we make a snapshot of.
      Snapshot(SmartGraph &graph) {
        graph.saveSnapshot(*this);
      }

      ///Make a snapshot.

      ///Make a snapshot of the graph.
      ///
      ///This function can be called more than once. In case of a repeated
      ///call, the previous snapshot gets lost.
      ///\param graph The digraph we make the snapshot of.
      void save(SmartGraph &graph)
      {
        graph.saveSnapshot(*this);
      }

      ///Undo the changes until a snapshot.

      ///Undo the changes until a snapshot created by save().
      ///
      ///\note After you restored a state, you cannot restore
      ///a later state, in other word you cannot add again the arcs deleted
      ///by restore().
      void restore()
      {
        _graph->restoreSnapshot(*this);
      }
    };
  };

} //namespace lemon


#endif //LEMON_SMART_GRAPH_H
