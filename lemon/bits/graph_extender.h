/* -*- C++ -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library
 *
 * Copyright (C) 2003-2007
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

#ifndef LEMON_BITS_GRAPH_EXTENDER_H
#define LEMON_BITS_GRAPH_EXTENDER_H

#include <lemon/bits/invalid.h>

#include <lemon/bits/map_extender.h>
#include <lemon/bits/default_map.h>

#include <lemon/concept_check.h>
#include <lemon/concepts/maps.h>

///\ingroup graphbits
///\file
///\brief Extenders for the digraph types
namespace lemon {

  /// \ingroup graphbits
  ///
  /// \brief Extender for the Digraphs
  template <typename Base>
  class DigraphExtender : public Base {
  public:

    typedef Base Parent;
    typedef DigraphExtender Digraph;

    // Base extensions

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;

    int maxId(Node) const {
      return Parent::maxNodeId();
    }

    int maxId(Arc) const {
      return Parent::maxArcId();
    }

    Node fromId(int id, Node) const {
      return Parent::nodeFromId(id);
    }

    Arc fromId(int id, Arc) const {
      return Parent::arcFromId(id);
    }

    Node oppositeNode(const Node &n, const Arc &e) const {
      if (n == Parent::source(e))
	return Parent::target(e);
      else if(n==Parent::target(e))
	return Parent::source(e);
      else
	return INVALID;
    }

    // Alterable extension

    typedef AlterationNotifier<DigraphExtender, Node> NodeNotifier;
    typedef AlterationNotifier<DigraphExtender, Arc> ArcNotifier;


  protected:

    mutable NodeNotifier node_notifier;
    mutable ArcNotifier arc_notifier;

  public:

    NodeNotifier& notifier(Node) const {
      return node_notifier;
    }
    
    ArcNotifier& notifier(Arc) const {
      return arc_notifier;
    }

    class NodeIt : public Node { 
      const Digraph* digraph;
    public:

      NodeIt() {}

      NodeIt(Invalid i) : Node(i) { }

      explicit NodeIt(const Digraph& _digraph) : digraph(&_digraph) {
	_digraph.first(static_cast<Node&>(*this));
      }

      NodeIt(const Digraph& _digraph, const Node& node) 
	: Node(node), digraph(&_digraph) {}

      NodeIt& operator++() { 
	digraph->next(*this);
	return *this; 
      }

    };


    class ArcIt : public Arc { 
      const Digraph* digraph;
    public:

      ArcIt() { }

      ArcIt(Invalid i) : Arc(i) { }

      explicit ArcIt(const Digraph& _digraph) : digraph(&_digraph) {
	_digraph.first(static_cast<Arc&>(*this));
      }

      ArcIt(const Digraph& _digraph, const Arc& e) : 
	Arc(e), digraph(&_digraph) { }

      ArcIt& operator++() { 
	digraph->next(*this);
	return *this; 
      }

    };


    class OutArcIt : public Arc { 
      const Digraph* digraph;
    public:

      OutArcIt() { }

      OutArcIt(Invalid i) : Arc(i) { }

      OutArcIt(const Digraph& _digraph, const Node& node) 
	: digraph(&_digraph) {
	_digraph.firstOut(*this, node);
      }

      OutArcIt(const Digraph& _digraph, const Arc& arc) 
	: Arc(arc), digraph(&_digraph) {}

      OutArcIt& operator++() { 
	digraph->nextOut(*this);
	return *this; 
      }

    };


    class InArcIt : public Arc { 
      const Digraph* digraph;
    public:

      InArcIt() { }

      InArcIt(Invalid i) : Arc(i) { }

      InArcIt(const Digraph& _digraph, const Node& node) 
	: digraph(&_digraph) {
	_digraph.firstIn(*this, node);
      }

      InArcIt(const Digraph& _digraph, const Arc& arc) : 
	Arc(arc), digraph(&_digraph) {}

      InArcIt& operator++() { 
	digraph->nextIn(*this);
	return *this; 
      }

    };

    /// \brief Base node of the iterator
    ///
    /// Returns the base node (i.e. the source in this case) of the iterator
    Node baseNode(const OutArcIt &e) const {
      return Parent::source(e);
    }
    /// \brief Running node of the iterator
    ///
    /// Returns the running node (i.e. the target in this case) of the
    /// iterator
    Node runningNode(const OutArcIt &e) const {
      return Parent::target(e);
    }

    /// \brief Base node of the iterator
    ///
    /// Returns the base node (i.e. the target in this case) of the iterator
    Node baseNode(const InArcIt &e) const {
      return Parent::target(e);
    }
    /// \brief Running node of the iterator
    ///
    /// Returns the running node (i.e. the source in this case) of the
    /// iterator
    Node runningNode(const InArcIt &e) const {
      return Parent::source(e);
    }

    
    template <typename _Value>
    class NodeMap 
      : public MapExtender<DefaultMap<Digraph, Node, _Value> > {
    public:
      typedef DigraphExtender Digraph;
      typedef MapExtender<DefaultMap<Digraph, Node, _Value> > Parent;

      explicit NodeMap(const Digraph& digraph) 
	: Parent(digraph) {}
      NodeMap(const Digraph& digraph, const _Value& value) 
	: Parent(digraph, value) {}

      NodeMap& operator=(const NodeMap& cmap) {
	return operator=<NodeMap>(cmap);
      }

      template <typename CMap>
      NodeMap& operator=(const CMap& cmap) {
        Parent::operator=(cmap);
	return *this;
      }

    };

    template <typename _Value>
    class ArcMap 
      : public MapExtender<DefaultMap<Digraph, Arc, _Value> > {
    public:
      typedef DigraphExtender Digraph;
      typedef MapExtender<DefaultMap<Digraph, Arc, _Value> > Parent;

      explicit ArcMap(const Digraph& digraph) 
	: Parent(digraph) {}
      ArcMap(const Digraph& digraph, const _Value& value) 
	: Parent(digraph, value) {}

      ArcMap& operator=(const ArcMap& cmap) {
	return operator=<ArcMap>(cmap);
      }

      template <typename CMap>
      ArcMap& operator=(const CMap& cmap) {
        Parent::operator=(cmap);
	return *this;
      }
    };


    Node addNode() {
      Node node = Parent::addNode();
      notifier(Node()).add(node);
      return node;
    }
    
    Arc addArc(const Node& from, const Node& to) {
      Arc arc = Parent::addArc(from, to);
      notifier(Arc()).add(arc);
      return arc;
    }

    void clear() {
      notifier(Arc()).clear();
      notifier(Node()).clear();
      Parent::clear();
    }

    template <typename Digraph, typename NodeRefMap, typename ArcRefMap>
    void build(const Digraph& digraph, NodeRefMap& nodeRef, ArcRefMap& arcRef) {
      Parent::build(digraph, nodeRef, arcRef);
      notifier(Node()).build();
      notifier(Arc()).build();
    }

    void erase(const Node& node) {
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

      notifier(Node()).erase(node);
      Parent::erase(node);
    }
    
    void erase(const Arc& arc) {
      notifier(Arc()).erase(arc);
      Parent::erase(arc);
    }

    DigraphExtender() {
      node_notifier.setContainer(*this);
      arc_notifier.setContainer(*this);
    } 
    

    ~DigraphExtender() {
      arc_notifier.clear();
      node_notifier.clear();
    }
  };

  /// \ingroup graphbits
  ///
  /// \brief Extender for the Graphs
  template <typename Base> 
  class GraphExtender : public Base {
  public:
    
    typedef Base Parent;
    typedef GraphExtender Digraph;

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;
    typedef typename Parent::Edge Edge;

    // Graph extension    

    int maxId(Node) const {
      return Parent::maxNodeId();
    }

    int maxId(Arc) const {
      return Parent::maxArcId();
    }

    int maxId(Edge) const {
      return Parent::maxEdgeId();
    }

    Node fromId(int id, Node) const {
      return Parent::nodeFromId(id);
    }

    Arc fromId(int id, Arc) const {
      return Parent::arcFromId(id);
    }

    Edge fromId(int id, Edge) const {
      return Parent::edgeFromId(id);
    }

    Node oppositeNode(const Node &n, const Edge &e) const {
      if( n == Parent::source(e))
	return Parent::target(e);
      else if( n == Parent::target(e))
	return Parent::source(e);
      else
	return INVALID;
    }

    Arc oppositeArc(const Arc &e) const {
      return Parent::direct(e, !Parent::direction(e));
    }

    using Parent::direct;
    Arc direct(const Edge &ue, const Node &s) const {
      return Parent::direct(ue, Parent::source(ue) == s);
    }

    // Alterable extension

    typedef AlterationNotifier<GraphExtender, Node> NodeNotifier;
    typedef AlterationNotifier<GraphExtender, Arc> ArcNotifier;
    typedef AlterationNotifier<GraphExtender, Edge> EdgeNotifier;


  protected:

    mutable NodeNotifier node_notifier;
    mutable ArcNotifier arc_notifier;
    mutable EdgeNotifier edge_notifier;

  public:

    NodeNotifier& notifier(Node) const {
      return node_notifier;
    }
    
    ArcNotifier& notifier(Arc) const {
      return arc_notifier;
    }

    EdgeNotifier& notifier(Edge) const {
      return edge_notifier;
    }



    class NodeIt : public Node { 
      const Digraph* digraph;
    public:

      NodeIt() {}

      NodeIt(Invalid i) : Node(i) { }

      explicit NodeIt(const Digraph& _digraph) : digraph(&_digraph) {
	_digraph.first(static_cast<Node&>(*this));
      }

      NodeIt(const Digraph& _digraph, const Node& node) 
	: Node(node), digraph(&_digraph) {}

      NodeIt& operator++() { 
	digraph->next(*this);
	return *this; 
      }

    };


    class ArcIt : public Arc { 
      const Digraph* digraph;
    public:

      ArcIt() { }

      ArcIt(Invalid i) : Arc(i) { }

      explicit ArcIt(const Digraph& _digraph) : digraph(&_digraph) {
	_digraph.first(static_cast<Arc&>(*this));
      }

      ArcIt(const Digraph& _digraph, const Arc& e) : 
	Arc(e), digraph(&_digraph) { }

      ArcIt& operator++() { 
	digraph->next(*this);
	return *this; 
      }

    };


    class OutArcIt : public Arc { 
      const Digraph* digraph;
    public:

      OutArcIt() { }

      OutArcIt(Invalid i) : Arc(i) { }

      OutArcIt(const Digraph& _digraph, const Node& node) 
	: digraph(&_digraph) {
	_digraph.firstOut(*this, node);
      }

      OutArcIt(const Digraph& _digraph, const Arc& arc) 
	: Arc(arc), digraph(&_digraph) {}

      OutArcIt& operator++() { 
	digraph->nextOut(*this);
	return *this; 
      }

    };


    class InArcIt : public Arc { 
      const Digraph* digraph;
    public:

      InArcIt() { }

      InArcIt(Invalid i) : Arc(i) { }

      InArcIt(const Digraph& _digraph, const Node& node) 
	: digraph(&_digraph) {
	_digraph.firstIn(*this, node);
      }

      InArcIt(const Digraph& _digraph, const Arc& arc) : 
	Arc(arc), digraph(&_digraph) {}

      InArcIt& operator++() { 
	digraph->nextIn(*this);
	return *this; 
      }

    };


    class EdgeIt : public Parent::Edge { 
      const Digraph* digraph;
    public:

      EdgeIt() { }

      EdgeIt(Invalid i) : Edge(i) { }

      explicit EdgeIt(const Digraph& _digraph) : digraph(&_digraph) {
	_digraph.first(static_cast<Edge&>(*this));
      }

      EdgeIt(const Digraph& _digraph, const Edge& e) : 
	Edge(e), digraph(&_digraph) { }

      EdgeIt& operator++() { 
	digraph->next(*this);
	return *this; 
      }

    };

    class IncArcIt : public Parent::Edge {
      friend class GraphExtender;
      const Digraph* digraph;
      bool direction;
    public:

      IncArcIt() { }

      IncArcIt(Invalid i) : Edge(i), direction(false) { }

      IncArcIt(const Digraph& _digraph, const Node &n) : digraph(&_digraph) {
	_digraph.firstInc(*this, direction, n);
      }

      IncArcIt(const Digraph& _digraph, const Edge &ue, const Node &n)
	: digraph(&_digraph), Edge(ue) {
	direction = (_digraph.source(ue) == n);
      }

      IncArcIt& operator++() {
	digraph->nextInc(*this, direction);
	return *this; 
      }
    };

    /// \brief Base node of the iterator
    ///
    /// Returns the base node (ie. the source in this case) of the iterator
    Node baseNode(const OutArcIt &e) const {
      return Parent::source(static_cast<const Arc&>(e));
    }
    /// \brief Running node of the iterator
    ///
    /// Returns the running node (ie. the target in this case) of the
    /// iterator
    Node runningNode(const OutArcIt &e) const {
      return Parent::target(static_cast<const Arc&>(e));
    }

    /// \brief Base node of the iterator
    ///
    /// Returns the base node (ie. the target in this case) of the iterator
    Node baseNode(const InArcIt &e) const {
      return Parent::target(static_cast<const Arc&>(e));
    }
    /// \brief Running node of the iterator
    ///
    /// Returns the running node (ie. the source in this case) of the
    /// iterator
    Node runningNode(const InArcIt &e) const {
      return Parent::source(static_cast<const Arc&>(e));
    }

    /// Base node of the iterator
    ///
    /// Returns the base node of the iterator
    Node baseNode(const IncArcIt &e) const {
      return e.direction ? source(e) : target(e);
    }
    /// Running node of the iterator
    ///
    /// Returns the running node of the iterator
    Node runningNode(const IncArcIt &e) const {
      return e.direction ? target(e) : source(e);
    }

    // Mappable extension

    template <typename _Value>
    class NodeMap 
      : public MapExtender<DefaultMap<Digraph, Node, _Value> > {
    public:
      typedef GraphExtender Digraph;
      typedef MapExtender<DefaultMap<Digraph, Node, _Value> > Parent;

      NodeMap(const Digraph& digraph) 
	: Parent(digraph) {}
      NodeMap(const Digraph& digraph, const _Value& value) 
	: Parent(digraph, value) {}

      NodeMap& operator=(const NodeMap& cmap) {
	return operator=<NodeMap>(cmap);
      }

      template <typename CMap>
      NodeMap& operator=(const CMap& cmap) {
        Parent::operator=(cmap);
	return *this;
      }

    };

    template <typename _Value>
    class ArcMap 
      : public MapExtender<DefaultMap<Digraph, Arc, _Value> > {
    public:
      typedef GraphExtender Digraph;
      typedef MapExtender<DefaultMap<Digraph, Arc, _Value> > Parent;

      ArcMap(const Digraph& digraph) 
	: Parent(digraph) {}
      ArcMap(const Digraph& digraph, const _Value& value) 
	: Parent(digraph, value) {}

      ArcMap& operator=(const ArcMap& cmap) {
	return operator=<ArcMap>(cmap);
      }

      template <typename CMap>
      ArcMap& operator=(const CMap& cmap) {
        Parent::operator=(cmap);
	return *this;
      }
    };


    template <typename _Value>
    class EdgeMap 
      : public MapExtender<DefaultMap<Digraph, Edge, _Value> > {
    public:
      typedef GraphExtender Digraph;
      typedef MapExtender<DefaultMap<Digraph, Edge, _Value> > Parent;

      EdgeMap(const Digraph& digraph) 
	: Parent(digraph) {}

      EdgeMap(const Digraph& digraph, const _Value& value) 
	: Parent(digraph, value) {}

      EdgeMap& operator=(const EdgeMap& cmap) {
	return operator=<EdgeMap>(cmap);
      }

      template <typename CMap>
      EdgeMap& operator=(const CMap& cmap) {
        Parent::operator=(cmap);
	return *this;
      }

    };

    // Alteration extension

    Node addNode() {
      Node node = Parent::addNode();
      notifier(Node()).add(node);
      return node;
    }

    Edge addEdge(const Node& from, const Node& to) {
      Edge edge = Parent::addEdge(from, to);
      notifier(Edge()).add(edge);
      std::vector<Arc> ev;
      ev.push_back(Parent::direct(edge, true));
      ev.push_back(Parent::direct(edge, false));      
      notifier(Arc()).add(ev);
      return edge;
    }
    
    void clear() {
      notifier(Arc()).clear();
      notifier(Edge()).clear();
      notifier(Node()).clear();
      Parent::clear();
    }

    template <typename Digraph, typename NodeRefMap, typename EdgeRefMap>
    void build(const Digraph& digraph, NodeRefMap& nodeRef, 
               EdgeRefMap& edgeRef) {
      Parent::build(digraph, nodeRef, edgeRef);
      notifier(Node()).build();
      notifier(Edge()).build();
      notifier(Arc()).build();
    }

    void erase(const Node& node) {
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

      notifier(Node()).erase(node);
      Parent::erase(node);
    }

    void erase(const Edge& edge) {
      std::vector<Arc> ev;
      ev.push_back(Parent::direct(edge, true));
      ev.push_back(Parent::direct(edge, false));      
      notifier(Arc()).erase(ev);
      notifier(Edge()).erase(edge);
      Parent::erase(edge);
    }

    GraphExtender() {
      node_notifier.setContainer(*this); 
      arc_notifier.setContainer(*this);
      edge_notifier.setContainer(*this);
    } 

    ~GraphExtender() {
      edge_notifier.clear();
      arc_notifier.clear();
      node_notifier.clear(); 
    } 

  };

}

#endif
