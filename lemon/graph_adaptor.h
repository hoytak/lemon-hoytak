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

#ifndef LEMON_GRAPH_ADAPTOR_H
#define LEMON_GRAPH_ADAPTOR_H

///\ingroup graph_adaptors
///\file
///\brief Several graph adaptors.
///
///This file contains several useful undirected graph adaptor classes.

#include <lemon/core.h>
#include <lemon/maps.h>
#include <lemon/bits/graph_adaptor_extender.h>

namespace lemon {

  /// \brief Base type for the Graph Adaptors
  ///
  /// This is the base type for most of LEMON graph adaptors. 
  /// This class implements a trivial graph adaptor i.e. it only wraps the 
  /// functions and types of the graph. The purpose of this class is to 
  /// make easier implementing graph adaptors. E.g. if an adaptor is 
  /// considered which differs from the wrapped graph only in some of its 
  /// functions or types, then it can be derived from GraphAdaptor, and only 
  /// the differences should be implemented.
  template<typename _Graph>
  class GraphAdaptorBase {
  public:
    typedef _Graph Graph;
    typedef Graph ParentGraph;

  protected:
    Graph* _graph;

    GraphAdaptorBase() : _graph(0) {}

    void setGraph(Graph& graph) { _graph = &graph; }

  public:
    GraphAdaptorBase(Graph& graph) : _graph(&graph) {}
 
    typedef typename Graph::Node Node;
    typedef typename Graph::Arc Arc;
    typedef typename Graph::Edge Edge;
   
    void first(Node& i) const { _graph->first(i); }
    void first(Arc& i) const { _graph->first(i); }
    void first(Edge& i) const { _graph->first(i); }
    void firstIn(Arc& i, const Node& n) const { _graph->firstIn(i, n); }
    void firstOut(Arc& i, const Node& n ) const { _graph->firstOut(i, n); }
    void firstInc(Edge &i, bool &d, const Node &n) const {
      _graph->firstInc(i, d, n);
    }

    void next(Node& i) const { _graph->next(i); }
    void next(Arc& i) const { _graph->next(i); }
    void next(Edge& i) const { _graph->next(i); }
    void nextIn(Arc& i) const { _graph->nextIn(i); }
    void nextOut(Arc& i) const { _graph->nextOut(i); }
    void nextInc(Edge &i, bool &d) const { _graph->nextInc(i, d); }

    Node u(const Edge& e) const { return _graph->u(e); }
    Node v(const Edge& e) const { return _graph->v(e); }

    Node source(const Arc& a) const { return _graph->source(a); }
    Node target(const Arc& a) const { return _graph->target(a); }

    typedef NodeNumTagIndicator<Graph> NodeNumTag;
    int nodeNum() const { return _graph->nodeNum(); }
    
    typedef EdgeNumTagIndicator<Graph> EdgeNumTag;
    int arcNum() const { return _graph->arcNum(); }
    int edgeNum() const { return _graph->edgeNum(); }

    typedef FindEdgeTagIndicator<Graph> FindEdgeTag;
    Arc findArc(const Node& u, const Node& v, const Arc& prev = INVALID) {
      return _graph->findArc(u, v, prev);
    }
    Edge findEdge(const Node& u, const Node& v, const Edge& prev = INVALID) {
      return _graph->findEdge(u, v, prev);
    }
  
    Node addNode() { return _graph->addNode(); }
    Edge addEdge(const Node& u, const Node& v) { return _graph->addEdge(u, v); }

    void erase(const Node& i) { _graph->erase(i); }
    void erase(const Edge& i) { _graph->erase(i); }
  
    void clear() { _graph->clear(); }
    
    bool direction(const Arc& a) const { return _graph->direction(a); }
    Arc direct(const Edge& e, bool d) const { return _graph->direct(e, d); }

    int id(const Node& v) const { return _graph->id(v); }
    int id(const Arc& a) const { return _graph->id(a); }
    int id(const Edge& e) const { return _graph->id(e); }

    Node nodeFromId(int ix) const { return _graph->nodeFromId(ix); }
    Arc arcFromId(int ix) const { return _graph->arcFromId(ix); }
    Edge edgeFromId(int ix) const { return _graph->edgeFromId(ix); }

    int maxNodeId() const { return _graph->maxNodeId(); }
    int maxArcId() const { return _graph->maxArcId(); }
    int maxEdgeId() const { return _graph->maxEdgeId(); }

    typedef typename ItemSetTraits<Graph, Node>::ItemNotifier NodeNotifier;
    NodeNotifier& notifier(Node) const { return _graph->notifier(Node()); } 

    typedef typename ItemSetTraits<Graph, Arc>::ItemNotifier ArcNotifier;
    ArcNotifier& notifier(Arc) const { return _graph->notifier(Arc()); } 

    typedef typename ItemSetTraits<Graph, Edge>::ItemNotifier EdgeNotifier;
    EdgeNotifier& notifier(Edge) const { return _graph->notifier(Edge()); } 

    template <typename _Value>
    class NodeMap : public Graph::template NodeMap<_Value> {
    public:
      typedef typename Graph::template NodeMap<_Value> Parent;
      explicit NodeMap(const GraphAdaptorBase<Graph>& adapter) 
	: Parent(*adapter._graph) {}
      NodeMap(const GraphAdaptorBase<Graph>& adapter, const _Value& value)
	: Parent(*adapter._graph, value) {}

    private:
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
    class ArcMap : public Graph::template ArcMap<_Value> {
    public:
      typedef typename Graph::template ArcMap<_Value> Parent;
      explicit ArcMap(const GraphAdaptorBase<Graph>& adapter) 
	: Parent(*adapter._graph) {}
      ArcMap(const GraphAdaptorBase<Graph>& adapter, const _Value& value)
	: Parent(*adapter._graph, value) {}

    private:
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
    class EdgeMap : public Graph::template EdgeMap<_Value> {
    public:
      typedef typename Graph::template EdgeMap<_Value> Parent;
      explicit EdgeMap(const GraphAdaptorBase<Graph>& adapter) 
	: Parent(*adapter._graph) {}
      EdgeMap(const GraphAdaptorBase<Graph>& adapter, const _Value& value)
	: Parent(*adapter._graph, value) {}

    private:
      EdgeMap& operator=(const EdgeMap& cmap) {
	return operator=<EdgeMap>(cmap);
      }

      template <typename CMap>
      EdgeMap& operator=(const CMap& cmap) {
        Parent::operator=(cmap);
        return *this;
      }
    };

  };

  /// \ingroup graph_adaptors
  ///
  /// \brief Trivial graph adaptor
  ///
  /// This class is an adaptor which does not change the adapted undirected
  /// graph. It can be used only to test the graph adaptors.
  template <typename _Graph>
  class GraphAdaptor 
    : public GraphAdaptorExtender< GraphAdaptorBase<_Graph> > { 
  public:
    typedef _Graph Graph;
    typedef GraphAdaptorExtender<GraphAdaptorBase<_Graph> > Parent;
  protected:
    GraphAdaptor() : Parent() {}

  public:
    explicit GraphAdaptor(Graph& graph) { setGraph(graph); }
  };

  template <typename _Graph, typename NodeFilterMap, 
	    typename EdgeFilterMap, bool checked = true>
  class SubGraphAdaptorBase : public GraphAdaptorBase<_Graph> {
  public:
    typedef _Graph Graph;
    typedef SubGraphAdaptorBase Adaptor;
    typedef GraphAdaptorBase<_Graph> Parent;
  protected:

    NodeFilterMap* _node_filter_map;
    EdgeFilterMap* _edge_filter_map;

    SubGraphAdaptorBase() 
      : Parent(), _node_filter_map(0), _edge_filter_map(0) { }

    void setNodeFilterMap(NodeFilterMap& node_filter_map) {
      _node_filter_map=&node_filter_map;
    }
    void setEdgeFilterMap(EdgeFilterMap& edge_filter_map) {
      _edge_filter_map=&edge_filter_map;
    }

  public:

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;
    typedef typename Parent::Edge Edge;

    void first(Node& i) const { 
      Parent::first(i); 
      while (i!=INVALID && !(*_node_filter_map)[i]) Parent::next(i); 
    }

    void first(Arc& i) const { 
      Parent::first(i); 
      while (i!=INVALID && (!(*_edge_filter_map)[i] 
	     || !(*_node_filter_map)[Parent::source(i)]
	     || !(*_node_filter_map)[Parent::target(i)])) Parent::next(i); 
    }

    void first(Edge& i) const { 
      Parent::first(i); 
      while (i!=INVALID && (!(*_edge_filter_map)[i] 
	     || !(*_node_filter_map)[Parent::u(i)]
	     || !(*_node_filter_map)[Parent::v(i)])) Parent::next(i); 
    }

    void firstIn(Arc& i, const Node& n) const { 
      Parent::firstIn(i, n); 
      while (i!=INVALID && (!(*_edge_filter_map)[i] 
	     || !(*_node_filter_map)[Parent::source(i)])) Parent::nextIn(i); 
    }

    void firstOut(Arc& i, const Node& n) const { 
      Parent::firstOut(i, n); 
      while (i!=INVALID && (!(*_edge_filter_map)[i] 
	     || !(*_node_filter_map)[Parent::target(i)])) Parent::nextOut(i); 
    }

    void firstInc(Edge& i, bool& d, const Node& n) const { 
      Parent::firstInc(i, d, n); 
      while (i!=INVALID && (!(*_edge_filter_map)[i] 
            || !(*_node_filter_map)[Parent::u(i)]
            || !(*_node_filter_map)[Parent::v(i)])) Parent::nextInc(i, d); 
    }

    void next(Node& i) const { 
      Parent::next(i); 
      while (i!=INVALID && !(*_node_filter_map)[i]) Parent::next(i); 
    }

    void next(Arc& i) const { 
      Parent::next(i); 
      while (i!=INVALID && (!(*_edge_filter_map)[i] 
	     || !(*_node_filter_map)[Parent::source(i)]
	     || !(*_node_filter_map)[Parent::target(i)])) Parent::next(i); 
    }

    void next(Edge& i) const { 
      Parent::next(i); 
      while (i!=INVALID && (!(*_edge_filter_map)[i] 
	     || !(*_node_filter_map)[Parent::u(i)]
	     || !(*_node_filter_map)[Parent::v(i)])) Parent::next(i); 
    }

    void nextIn(Arc& i) const { 
      Parent::nextIn(i); 
      while (i!=INVALID && (!(*_edge_filter_map)[i] 
	     || !(*_node_filter_map)[Parent::source(i)])) Parent::nextIn(i); 
    }

    void nextOut(Arc& i) const { 
      Parent::nextOut(i); 
      while (i!=INVALID && (!(*_edge_filter_map)[i] 
	     || !(*_node_filter_map)[Parent::target(i)])) Parent::nextOut(i); 
    }

    void nextInc(Edge& i, bool& d) const { 
      Parent::nextInc(i, d); 
      while (i!=INVALID && (!(*_edge_filter_map)[i]
            || !(*_node_filter_map)[Parent::u(i)]
            || !(*_node_filter_map)[Parent::v(i)])) Parent::nextInc(i, d); 
    }

    /// \brief Hide the given node in the graph.
    ///
    /// This function hides \c n in the graph, i.e. the iteration 
    /// jumps over it. This is done by simply setting the value of \c n  
    /// to be false in the corresponding node-map.
    void hide(const Node& n) const { _node_filter_map->set(n, false); }

    /// \brief Hide the given edge in the graph.
    ///
    /// This function hides \c e in the graph, i.e. the iteration 
    /// jumps over it. This is done by simply setting the value of \c e  
    /// to be false in the corresponding edge-map.
    void hide(const Edge& e) const { _edge_filter_map->set(e, false); }

    /// \brief Unhide the given node in the graph.
    ///
    /// The value of \c n is set to be true in the node-map which stores 
    /// hide information. If \c n was hidden previuosly, then it is shown 
    /// again
     void unHide(const Node& n) const { _node_filter_map->set(n, true); }

    /// \brief Hide the given edge in the graph.
    ///
    /// The value of \c e is set to be true in the edge-map which stores 
    /// hide information. If \c e was hidden previuosly, then it is shown 
    /// again
    void unHide(const Edge& e) const { _edge_filter_map->set(e, true); }

    /// \brief Returns true if \c n is hidden.
    ///
    /// Returns true if \c n is hidden.
    bool hidden(const Node& n) const { return !(*_node_filter_map)[n]; }

    /// \brief Returns true if \c e is hidden.
    ///
    /// Returns true if \c e is hidden.
    bool hidden(const Edge& e) const { return !(*_edge_filter_map)[e]; }

    typedef False NodeNumTag;
    typedef False EdgeNumTag;

    typedef FindEdgeTagIndicator<Graph> FindEdgeTag;
    Arc findArc(const Node& u, const Node& v, 
		  const Arc& prev = INVALID) {
      if (!(*_node_filter_map)[u] || !(*_node_filter_map)[v]) {
        return INVALID;
      }
      Arc arc = Parent::findArc(u, v, prev);
      while (arc != INVALID && !(*_edge_filter_map)[arc]) {
        arc = Parent::findArc(u, v, arc);
      }
      return arc;
    }
    Edge findEdge(const Node& u, const Node& v, 
		  const Edge& prev = INVALID) {
      if (!(*_node_filter_map)[u] || !(*_node_filter_map)[v]) {
        return INVALID;
      }
      Edge edge = Parent::findEdge(u, v, prev);
      while (edge != INVALID && !(*_edge_filter_map)[edge]) {
        edge = Parent::findEdge(u, v, edge);
      }
      return edge;
    }

    template <typename _Value>
    class NodeMap : public SubMapExtender<Adaptor, 
        typename Parent::template NodeMap<_Value> > {
    public:
      typedef _Value Value;
      typedef SubMapExtender<Adaptor, typename Parent::
                             template NodeMap<Value> > MapParent;
    
      NodeMap(const Adaptor& adaptor) 
	: MapParent(adaptor) {}
      NodeMap(const Adaptor& adaptor, const Value& value) 
	: MapParent(adaptor, value) {}

    private:
      NodeMap& operator=(const NodeMap& cmap) {
	return operator=<NodeMap>(cmap);
      }
    
      template <typename CMap>
      NodeMap& operator=(const CMap& cmap) {
        MapParent::operator=(cmap);
	return *this;
      }
    };

    template <typename _Value>
    class ArcMap : public SubMapExtender<Adaptor, 
	typename Parent::template ArcMap<_Value> > {
    public:
      typedef _Value Value;
      typedef SubMapExtender<Adaptor, typename Parent::
                             template ArcMap<Value> > MapParent;
    
      ArcMap(const Adaptor& adaptor) 
	: MapParent(adaptor) {}
      ArcMap(const Adaptor& adaptor, const Value& value) 
	: MapParent(adaptor, value) {}

    private:
      ArcMap& operator=(const ArcMap& cmap) {
	return operator=<ArcMap>(cmap);
      }
    
      template <typename CMap>
      ArcMap& operator=(const CMap& cmap) {
        MapParent::operator=(cmap);
	return *this;
      }
    };

    template <typename _Value>
    class EdgeMap : public SubMapExtender<Adaptor, 
        typename Parent::template EdgeMap<_Value> > {
    public:
      typedef _Value Value;
      typedef SubMapExtender<Adaptor, typename Parent::
                             template EdgeMap<Value> > MapParent;
    
      EdgeMap(const Adaptor& adaptor) 
	: MapParent(adaptor) {}

      EdgeMap(const Adaptor& adaptor, const Value& value) 
	: MapParent(adaptor, value) {}

    private:
      EdgeMap& operator=(const EdgeMap& cmap) {
	return operator=<EdgeMap>(cmap);
      }
    
      template <typename CMap>
      EdgeMap& operator=(const CMap& cmap) {
        MapParent::operator=(cmap);
	return *this;
      }
    };

  };

  template <typename _Graph, typename NodeFilterMap, typename EdgeFilterMap>
  class SubGraphAdaptorBase<_Graph, NodeFilterMap, EdgeFilterMap, false> 
    : public GraphAdaptorBase<_Graph> {
  public:
    typedef _Graph Graph;
    typedef SubGraphAdaptorBase Adaptor;
    typedef GraphAdaptorBase<_Graph> Parent;
  protected:
    NodeFilterMap* _node_filter_map;
    EdgeFilterMap* _edge_filter_map;
    SubGraphAdaptorBase() : Parent(), 
			    _node_filter_map(0), _edge_filter_map(0) { }

    void setNodeFilterMap(NodeFilterMap& node_filter_map) {
      _node_filter_map=&node_filter_map;
    }
    void setEdgeFilterMap(EdgeFilterMap& edge_filter_map) {
      _edge_filter_map=&edge_filter_map;
    }

  public:

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;
    typedef typename Parent::Edge Edge;

    void first(Node& i) const { 
      Parent::first(i); 
      while (i!=INVALID && !(*_node_filter_map)[i]) Parent::next(i); 
    }

    void first(Arc& i) const { 
      Parent::first(i); 
      while (i!=INVALID && !(*_edge_filter_map)[i]) Parent::next(i); 
    }

    void first(Edge& i) const { 
      Parent::first(i); 
      while (i!=INVALID && !(*_edge_filter_map)[i]) Parent::next(i); 
    }

    void firstIn(Arc& i, const Node& n) const { 
      Parent::firstIn(i, n); 
      while (i!=INVALID && !(*_edge_filter_map)[i]) Parent::nextIn(i); 
    }

    void firstOut(Arc& i, const Node& n) const { 
      Parent::firstOut(i, n); 
      while (i!=INVALID && !(*_edge_filter_map)[i]) Parent::nextOut(i); 
    }

    void firstInc(Edge& i, bool& d, const Node& n) const { 
      Parent::firstInc(i, d, n); 
      while (i!=INVALID && !(*_edge_filter_map)[i]) Parent::nextInc(i, d); 
    }

    void next(Node& i) const { 
      Parent::next(i); 
      while (i!=INVALID && !(*_node_filter_map)[i]) Parent::next(i); 
    }
    void next(Arc& i) const { 
      Parent::next(i); 
      while (i!=INVALID && !(*_edge_filter_map)[i]) Parent::next(i); 
    }
    void next(Edge& i) const { 
      Parent::next(i); 
      while (i!=INVALID && !(*_edge_filter_map)[i]) Parent::next(i); 
    }
    void nextIn(Arc& i) const { 
      Parent::nextIn(i); 
      while (i!=INVALID && !(*_edge_filter_map)[i]) Parent::nextIn(i); 
    }

    void nextOut(Arc& i) const { 
      Parent::nextOut(i); 
      while (i!=INVALID && !(*_edge_filter_map)[i]) Parent::nextOut(i); 
    }
    void nextInc(Edge& i, bool& d) const { 
      Parent::nextInc(i, d); 
      while (i!=INVALID && !(*_edge_filter_map)[i]) Parent::nextInc(i, d); 
    }

    /// \brief Hide the given node in the graph.
    ///
    /// This function hides \c n in the graph, i.e. the iteration 
    /// jumps over it. This is done by simply setting the value of \c n  
    /// to be false in the corresponding node-map.
    void hide(const Node& n) const { _node_filter_map->set(n, false); }

    /// \brief Hide the given edge in the graph.
    ///
    /// This function hides \c e in the graph, i.e. the iteration 
    /// jumps over it. This is done by simply setting the value of \c e  
    /// to be false in the corresponding edge-map.
    void hide(const Edge& e) const { _edge_filter_map->set(e, false); }

    /// \brief Unhide the given node in the graph.
    ///
    /// The value of \c n is set to be true in the node-map which stores 
    /// hide information. If \c n was hidden previuosly, then it is shown 
    /// again
     void unHide(const Node& n) const { _node_filter_map->set(n, true); }

    /// \brief Hide the given edge in the graph.
    ///
    /// The value of \c e is set to be true in the edge-map which stores 
    /// hide information. If \c e was hidden previuosly, then it is shown 
    /// again
    void unHide(const Edge& e) const { _edge_filter_map->set(e, true); }

    /// \brief Returns true if \c n is hidden.
    ///
    /// Returns true if \c n is hidden.
    bool hidden(const Node& n) const { return !(*_node_filter_map)[n]; }

    /// \brief Returns true if \c e is hidden.
    ///
    /// Returns true if \c e is hidden.
    bool hidden(const Edge& e) const { return !(*_edge_filter_map)[e]; }

    typedef False NodeNumTag;
    typedef False EdgeNumTag;

    typedef FindEdgeTagIndicator<Graph> FindEdgeTag;
    Arc findArc(const Node& u, const Node& v, 
		  const Arc& prev = INVALID) {
      Arc arc = Parent::findArc(u, v, prev);
      while (arc != INVALID && !(*_edge_filter_map)[arc]) {
        arc = Parent::findArc(u, v, arc);
      }
      return arc;
    }
    Edge findEdge(const Node& u, const Node& v, 
		  const Edge& prev = INVALID) {
      Edge edge = Parent::findEdge(u, v, prev);
      while (edge != INVALID && !(*_edge_filter_map)[edge]) {
        edge = Parent::findEdge(u, v, edge);
      }
      return edge;
    }

    template <typename _Value>
    class NodeMap : public SubMapExtender<Adaptor, 
        typename Parent::template NodeMap<_Value> > {
    public:
      typedef _Value Value;
      typedef SubMapExtender<Adaptor, typename Parent::
                             template NodeMap<Value> > MapParent;
    
      NodeMap(const Adaptor& adaptor) 
	: MapParent(adaptor) {}
      NodeMap(const Adaptor& adaptor, const Value& value) 
	: MapParent(adaptor, value) {}
    
    private:
      NodeMap& operator=(const NodeMap& cmap) {
	return operator=<NodeMap>(cmap);
      }
    
      template <typename CMap>
      NodeMap& operator=(const CMap& cmap) {
        MapParent::operator=(cmap);
	return *this;
      }
    };

    template <typename _Value>
    class ArcMap : public SubMapExtender<Adaptor, 
	typename Parent::template ArcMap<_Value> > {
    public:
      typedef _Value Value;
      typedef SubMapExtender<Adaptor, typename Parent::
                             template ArcMap<Value> > MapParent;
    
      ArcMap(const Adaptor& adaptor) 
	: MapParent(adaptor) {}
      ArcMap(const Adaptor& adaptor, const Value& value) 
	: MapParent(adaptor, value) {}

    private:
      ArcMap& operator=(const ArcMap& cmap) {
	return operator=<ArcMap>(cmap);
      }
    
      template <typename CMap>
      ArcMap& operator=(const CMap& cmap) {
        MapParent::operator=(cmap);
	return *this;
      }
    };

    template <typename _Value>
    class EdgeMap : public SubMapExtender<Adaptor, 
        typename Parent::template EdgeMap<_Value> > {
    public:
      typedef _Value Value;
      typedef SubMapExtender<Adaptor, typename Parent::
                             template EdgeMap<Value> > MapParent;
    
      EdgeMap(const Adaptor& adaptor) 
	: MapParent(adaptor) {}

      EdgeMap(const Adaptor& adaptor, const _Value& value) 
	: MapParent(adaptor, value) {}

    private:
      EdgeMap& operator=(const EdgeMap& cmap) {
	return operator=<EdgeMap>(cmap);
      }
    
      template <typename CMap>
      EdgeMap& operator=(const CMap& cmap) {
        MapParent::operator=(cmap);
	return *this;
      }
    };

  };

  /// \ingroup graph_adaptors
  ///
  /// \brief A graph adaptor for hiding nodes and arcs from an
  /// undirected graph.
  /// 
  /// SubGraphAdaptor shows the graph with filtered node-set and
  /// edge-set. If the \c checked parameter is true then it filters
  /// the edge-set to do not get invalid edges which incident node is
  /// filtered.
  /// 
  /// If the \c checked template parameter is false then we have to
  /// note that the node-iterator cares only the filter on the
  /// node-set, and the edge-iterator cares only the filter on the
  /// edge-set.  This way the edge-map should filter all arcs which
  /// has filtered end node.
  template<typename _Graph, typename NodeFilterMap, 
	   typename EdgeFilterMap, bool checked = true>
  class SubGraphAdaptor : 
    public GraphAdaptorExtender<
    SubGraphAdaptorBase<_Graph, NodeFilterMap, EdgeFilterMap, checked> > {
  public:
    typedef _Graph Graph;
    typedef GraphAdaptorExtender<
      SubGraphAdaptorBase<_Graph, NodeFilterMap, EdgeFilterMap> > Parent;
  protected:
    SubGraphAdaptor() { }
  public:
    SubGraphAdaptor(Graph& _graph, NodeFilterMap& node_filter_map, 
		    EdgeFilterMap& edge_filter_map) { 
      setGraph(_graph);
      setNodeFilterMap(node_filter_map);
      setEdgeFilterMap(edge_filter_map);
    }
  };

  template<typename Graph, typename NodeFilterMap, typename ArcFilterMap>
  SubGraphAdaptor<const Graph, NodeFilterMap, ArcFilterMap>
  subGraphAdaptor(const Graph& graph, 
                   NodeFilterMap& nfm, ArcFilterMap& efm) {
    return SubGraphAdaptor<const Graph, NodeFilterMap, ArcFilterMap>
      (graph, nfm, efm);
  }

  template<typename Graph, typename NodeFilterMap, typename ArcFilterMap>
  SubGraphAdaptor<const Graph, const NodeFilterMap, ArcFilterMap>
  subGraphAdaptor(const Graph& graph, 
                   NodeFilterMap& nfm, ArcFilterMap& efm) {
    return SubGraphAdaptor<const Graph, const NodeFilterMap, ArcFilterMap>
      (graph, nfm, efm);
  }

  template<typename Graph, typename NodeFilterMap, typename ArcFilterMap>
  SubGraphAdaptor<const Graph, NodeFilterMap, const ArcFilterMap>
  subGraphAdaptor(const Graph& graph, 
                   NodeFilterMap& nfm, ArcFilterMap& efm) {
    return SubGraphAdaptor<const Graph, NodeFilterMap, const ArcFilterMap>
      (graph, nfm, efm);
  }

  template<typename Graph, typename NodeFilterMap, typename ArcFilterMap>
  SubGraphAdaptor<const Graph, const NodeFilterMap, const ArcFilterMap>
  subGraphAdaptor(const Graph& graph, 
                   NodeFilterMap& nfm, ArcFilterMap& efm) {
    return SubGraphAdaptor<const Graph, const NodeFilterMap, 
      const ArcFilterMap>(graph, nfm, efm);
  }

  /// \ingroup graph_adaptors
  ///
  /// \brief An adaptor for hiding nodes from an graph.
  ///
  /// An adaptor for hiding nodes from an graph.  This
  /// adaptor specializes SubGraphAdaptor in the way that only the
  /// node-set can be filtered. In usual case the checked parameter is
  /// true, we get the induced subgraph. But if the checked parameter
  /// is false then we can filter only isolated nodes.
  template<typename _Graph, typename _NodeFilterMap, bool checked = true>
  class NodeSubGraphAdaptor : 
    public SubGraphAdaptor<_Graph, _NodeFilterMap, 
			   ConstMap<typename _Graph::Edge, bool>, checked> {
  public:
    typedef _Graph Graph;
    typedef _NodeFilterMap NodeFilterMap;
    typedef SubGraphAdaptor<Graph, NodeFilterMap, 
			    ConstMap<typename Graph::Edge, bool> > Parent;
  protected:
    ConstMap<typename Graph::Edge, bool> const_true_map;

    NodeSubGraphAdaptor() : const_true_map(true) {
      Parent::setEdgeFilterMap(const_true_map);
    }

  public:
    NodeSubGraphAdaptor(Graph& _graph, NodeFilterMap& node_filter_map) : 
      Parent(), const_true_map(true) { 
      Parent::setGraph(_graph);
      Parent::setNodeFilterMap(node_filter_map);
      Parent::setEdgeFilterMap(const_true_map);
    }
  };

  template<typename Graph, typename NodeFilterMap>
  NodeSubGraphAdaptor<const Graph, NodeFilterMap>
  nodeSubGraphAdaptor(const Graph& graph, NodeFilterMap& nfm) {
    return NodeSubGraphAdaptor<const Graph, NodeFilterMap>(graph, nfm);
  }

  template<typename Graph, typename NodeFilterMap>
  NodeSubGraphAdaptor<const Graph, const NodeFilterMap>
  nodeSubGraphAdaptor(const Graph& graph, const NodeFilterMap& nfm) {
    return NodeSubGraphAdaptor<const Graph, const NodeFilterMap>(graph, nfm);
  }

  /// \ingroup graph_adaptors
  ///
  /// \brief An adaptor for hiding edges from an graph.
  ///
  /// \warning Graph adaptors are in even more experimental state
  /// than the other parts of the lib. Use them at you own risk.
  ///
  /// An adaptor for hiding edges from an graph.
  /// This adaptor specializes SubGraphAdaptor in the way that
  /// only the arc-set 
  /// can be filtered.
  template<typename _Graph, typename _EdgeFilterMap>
  class EdgeSubGraphAdaptor : 
    public SubGraphAdaptor<_Graph, ConstMap<typename _Graph::Node,bool>, 
                           _EdgeFilterMap, false> {
  public:
    typedef _Graph Graph;
    typedef _EdgeFilterMap EdgeFilterMap;
    typedef SubGraphAdaptor<Graph, ConstMap<typename Graph::Node,bool>, 
			    EdgeFilterMap, false> Parent;
  protected:
    ConstMap<typename Graph::Node, bool> const_true_map;

    EdgeSubGraphAdaptor() : const_true_map(true) {
      Parent::setNodeFilterMap(const_true_map);
    }

  public:

    EdgeSubGraphAdaptor(Graph& _graph, EdgeFilterMap& edge_filter_map) : 
      Parent(), const_true_map(true) { 
      Parent::setGraph(_graph);
      Parent::setNodeFilterMap(const_true_map);
      Parent::setEdgeFilterMap(edge_filter_map);
    }

  };

  template<typename Graph, typename EdgeFilterMap>
  EdgeSubGraphAdaptor<const Graph, EdgeFilterMap>
  edgeSubGraphAdaptor(const Graph& graph, EdgeFilterMap& efm) {
    return EdgeSubGraphAdaptor<const Graph, EdgeFilterMap>(graph, efm);
  }

  template<typename Graph, typename EdgeFilterMap>
  EdgeSubGraphAdaptor<const Graph, const EdgeFilterMap>
  edgeSubGraphAdaptor(const Graph& graph, const EdgeFilterMap& efm) {
    return EdgeSubGraphAdaptor<const Graph, const EdgeFilterMap>(graph, efm);
  }

  /// \brief Base of direct graph adaptor
  ///
  /// Base class of the direct graph adaptor. All public member
  /// of this class can be used with the DirGraphAdaptor too.
  /// \sa DirGraphAdaptor
  template <typename _Graph, typename _DirectionMap>
  class DirGraphAdaptorBase {
  public:
    
    typedef _Graph Graph;
    typedef _DirectionMap DirectionMap;

    typedef typename Graph::Node Node;
    typedef typename Graph::Edge Arc;
   
    /// \brief Reverse arc
    /// 
    /// It reverse the given arc. It simply negate the direction in the map.
    void reverseArc(const Arc& arc) {
      _direction->set(arc, !(*_direction)[arc]);
    }

    void first(Node& i) const { _graph->first(i); }
    void first(Arc& i) const { _graph->first(i); }
    void firstIn(Arc& i, const Node& n) const {
      bool d;
      _graph->firstInc(i, d, n);
      while (i != INVALID && d == (*_direction)[i]) _graph->nextInc(i, d);
    }
    void firstOut(Arc& i, const Node& n ) const { 
      bool d;
      _graph->firstInc(i, d, n);
      while (i != INVALID && d != (*_direction)[i]) _graph->nextInc(i, d);
    }

    void next(Node& i) const { _graph->next(i); }
    void next(Arc& i) const { _graph->next(i); }
    void nextIn(Arc& i) const {
      bool d = !(*_direction)[i];
      _graph->nextInc(i, d);
      while (i != INVALID && d == (*_direction)[i]) _graph->nextInc(i, d);
    }
    void nextOut(Arc& i) const {
      bool d = (*_direction)[i];
      _graph->nextInc(i, d);
      while (i != INVALID && d != (*_direction)[i]) _graph->nextInc(i, d);
    }

    Node source(const Arc& e) const { 
      return (*_direction)[e] ? _graph->u(e) : _graph->v(e); 
    }
    Node target(const Arc& e) const { 
      return (*_direction)[e] ? _graph->v(e) : _graph->u(e); 
    }

    typedef NodeNumTagIndicator<Graph> NodeNumTag;
    int nodeNum() const { return _graph->nodeNum(); }
    
    typedef EdgeNumTagIndicator<Graph> EdgeNumTag;
    int arcNum() const { return _graph->edgeNum(); }

    typedef FindEdgeTagIndicator<Graph> FindEdgeTag;
    Arc findArc(const Node& u, const Node& v, 
		  const Arc& prev = INVALID) {
      Arc arc = prev;
      bool d = arc == INVALID ? true : (*_direction)[arc];
      if (d) {
        arc = _graph->findEdge(u, v, arc);
        while (arc != INVALID && !(*_direction)[arc]) {
          _graph->findEdge(u, v, arc);
        }
        if (arc != INVALID) return arc;
      }
      _graph->findEdge(v, u, arc);
      while (arc != INVALID && (*_direction)[arc]) {
        _graph->findEdge(u, v, arc);
      }
      return arc;
    }
  
    Node addNode() { 
      return Node(_graph->addNode()); 
    }

    Arc addArc(const Node& u, const Node& v) {
      Arc arc = _graph->addArc(u, v);
      _direction->set(arc, _graph->source(arc) == u);
      return arc; 
    }

    void erase(const Node& i) { _graph->erase(i); }
    void erase(const Arc& i) { _graph->erase(i); }
  
    void clear() { _graph->clear(); }
    
    int id(const Node& v) const { return _graph->id(v); }
    int id(const Arc& e) const { return _graph->id(e); }

    Node nodeFromId(int idx) const { return _graph->nodeFromId(idx); }
    Arc arcFromId(int idx) const { return _graph->edgeFromId(idx); }

    int maxNodeId() const { return _graph->maxNodeId(); }
    int maxArcId() const { return _graph->maxEdgeId(); }

    typedef typename ItemSetTraits<Graph, Node>::ItemNotifier NodeNotifier;
    NodeNotifier& notifier(Node) const { return _graph->notifier(Node()); } 

    typedef typename ItemSetTraits<Graph, Arc>::ItemNotifier ArcNotifier;
    ArcNotifier& notifier(Arc) const { return _graph->notifier(Arc()); } 

    template <typename _Value>
    class NodeMap : public _Graph::template NodeMap<_Value> {
    public:

      typedef typename _Graph::template NodeMap<_Value> Parent;

      explicit NodeMap(const DirGraphAdaptorBase& adapter) 
	: Parent(*adapter._graph) {}

      NodeMap(const DirGraphAdaptorBase& adapter, const _Value& value)
	: Parent(*adapter._graph, value) {}

    private:
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
    class ArcMap : public _Graph::template EdgeMap<_Value> {
    public:

      typedef typename Graph::template EdgeMap<_Value> Parent;

      explicit ArcMap(const DirGraphAdaptorBase& adapter) 
	: Parent(*adapter._graph) { }

      ArcMap(const DirGraphAdaptorBase& adapter, const _Value& value)
	: Parent(*adapter._graph, value) { }

    private:
      ArcMap& operator=(const ArcMap& cmap) {
        return operator=<ArcMap>(cmap);
      }

      template <typename CMap>
      ArcMap& operator=(const CMap& cmap) {
        Parent::operator=(cmap);
        return *this;
      }
    };

    

  protected:
    Graph* _graph;
    DirectionMap* _direction;

    void setDirectionMap(DirectionMap& direction) {
      _direction = &direction;
    }

    void setGraph(Graph& graph) {
      _graph = &graph;
    }

  };


  /// \ingroup graph_adaptors
  ///
  /// \brief A directed graph is made from an graph by an adaptor
  ///
  /// This adaptor gives a direction for each edge in the undirected
  /// graph. The direction of the arcs stored in the
  /// DirectionMap. This map is a bool map on the edges. If
  /// the edge is mapped to true then the direction of the directed
  /// arc will be the same as the default direction of the edge. The
  /// arcs can be easily reverted by the \ref
  /// DirGraphAdaptorBase::reverseArc "reverseArc()" member in the
  /// adaptor.
  ///
  /// It can be used to solve orientation problems on directed graphs.
  /// For example how can we orient an graph to get the minimum
  /// number of strongly connected components. If we orient the arcs with
  /// the dfs algorithm out from the source then we will get such an 
  /// orientation. 
  ///
  /// We use the \ref DfsVisitor "visitor" interface of the 
  /// \ref DfsVisit "dfs" algorithm:
  ///\code
  /// template <typename DirMap>
  /// class OrientVisitor : public DfsVisitor<Graph> {
  /// public:
  ///
  ///   OrientVisitor(const Graph& graph, DirMap& dirMap)
  ///     : _graph(graph), _dirMap(dirMap), _processed(graph, false) {}
  ///
  ///   void discover(const Arc& arc) {
  ///     _processed.set(arc, true);
  ///     _dirMap.set(arc, _graph.direction(arc));
  ///   }
  ///
  ///   void examine(const Arc& arc) {
  ///     if (_processed[arc]) return;
  ///     _processed.set(arc, true);
  ///     _dirMap.set(arc, _graph.direction(arc));
  ///   }  
  /// 
  /// private:
  ///   const Graph& _graph;  
  ///   DirMap& _dirMap;
  ///   Graph::EdgeMap<bool> _processed;
  /// };
  ///\endcode
  ///
  /// And now we can use the orientation:
  ///\code
  /// Graph::EdgeMap<bool> dmap(graph);
  ///
  /// typedef OrientVisitor<Graph::EdgeMap<bool> > Visitor;
  /// Visitor visitor(graph, dmap);
  ///
  /// DfsVisit<Graph, Visitor> dfs(graph, visitor);
  ///
  /// dfs.run();
  ///
  /// typedef DirGraphAdaptor<Graph> DGraph;
  /// DGraph dgraph(graph, dmap);
  ///
  /// LEMON_ASSERT(countStronglyConnectedComponents(dgraph) == 
  ///              countBiArcConnectedComponents(graph), "Wrong Orientation");
  ///\endcode
  ///
  /// The number of the bi-connected components is a lower bound for
  /// the number of the strongly connected components in the directed
  /// graph because if we contract the bi-connected components to
  /// nodes we will get a tree therefore we cannot orient arcs in
  /// both direction between bi-connected components. In the other way
  /// the algorithm will orient one component to be strongly
  /// connected. The two relations proof that the assertion will
  /// be always true and the found solution is optimal.
  ///
  /// \sa DirGraphAdaptorBase
  /// \sa dirGraphAdaptor
  template<typename _Graph, 
           typename DirectionMap = typename _Graph::template EdgeMap<bool> > 
  class DirGraphAdaptor : 
    public DigraphAdaptorExtender<DirGraphAdaptorBase<_Graph, DirectionMap> > {
  public:
    typedef _Graph Graph;
    typedef DigraphAdaptorExtender<
      DirGraphAdaptorBase<_Graph, DirectionMap> > Parent;
  protected:
    DirGraphAdaptor() { }
  public:
    
    /// \brief Constructor of the adaptor
    ///
    /// Constructor of the adaptor
    DirGraphAdaptor(Graph& graph, DirectionMap& direction) { 
      setGraph(graph);
      setDirectionMap(direction);
    }
  };

  /// \brief Just gives back a DirGraphAdaptor
  ///
  /// Just gives back a DirGraphAdaptor
  template<typename Graph, typename DirectionMap>
  DirGraphAdaptor<const Graph, DirectionMap>
  dirGraphAdaptor(const Graph& graph, DirectionMap& dm) {
    return DirGraphAdaptor<const Graph, DirectionMap>(graph, dm);
  }

  template<typename Graph, typename DirectionMap>
  DirGraphAdaptor<const Graph, const DirectionMap>
  dirGraphAdaptor(const Graph& graph, const DirectionMap& dm) {
    return DirGraphAdaptor<const Graph, const DirectionMap>(graph, dm);
  }

}

#endif
