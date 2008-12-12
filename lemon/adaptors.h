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

#ifndef LEMON_ADAPTORS_H
#define LEMON_ADAPTORS_H

/// \ingroup graph_adaptors
/// \file
/// \brief Several graph adaptors
///
/// This file contains several useful adaptors for digraphs and graphs.

#include <lemon/core.h>
#include <lemon/maps.h>
#include <lemon/bits/variant.h>

#include <lemon/bits/graph_adaptor_extender.h>
#include <lemon/tolerance.h>

#include <algorithm>

namespace lemon {

  template<typename _Digraph>
  class DigraphAdaptorBase {
  public:
    typedef _Digraph Digraph;
    typedef DigraphAdaptorBase Adaptor;
    typedef Digraph ParentDigraph;

  protected:
    Digraph* _digraph;
    DigraphAdaptorBase() : _digraph(0) { }
    void setDigraph(Digraph& digraph) { _digraph = &digraph; }

  public:
    DigraphAdaptorBase(Digraph& digraph) : _digraph(&digraph) { }

    typedef typename Digraph::Node Node;
    typedef typename Digraph::Arc Arc;

    void first(Node& i) const { _digraph->first(i); }
    void first(Arc& i) const { _digraph->first(i); }
    void firstIn(Arc& i, const Node& n) const { _digraph->firstIn(i, n); }
    void firstOut(Arc& i, const Node& n ) const { _digraph->firstOut(i, n); }

    void next(Node& i) const { _digraph->next(i); }
    void next(Arc& i) const { _digraph->next(i); }
    void nextIn(Arc& i) const { _digraph->nextIn(i); }
    void nextOut(Arc& i) const { _digraph->nextOut(i); }

    Node source(const Arc& a) const { return _digraph->source(a); }
    Node target(const Arc& a) const { return _digraph->target(a); }

    typedef NodeNumTagIndicator<Digraph> NodeNumTag;
    int nodeNum() const { return _digraph->nodeNum(); }

    typedef ArcNumTagIndicator<Digraph> ArcNumTag;
    int arcNum() const { return _digraph->arcNum(); }

    typedef FindArcTagIndicator<Digraph> FindArcTag;
    Arc findArc(const Node& u, const Node& v, const Arc& prev = INVALID) const {
      return _digraph->findArc(u, v, prev);
    }

    Node addNode() { return _digraph->addNode(); }
    Arc addArc(const Node& u, const Node& v) { return _digraph->addArc(u, v); }

    void erase(const Node& n) { _digraph->erase(n); }
    void erase(const Arc& a) { _digraph->erase(a); }

    void clear() { _digraph->clear(); }

    int id(const Node& n) const { return _digraph->id(n); }
    int id(const Arc& a) const { return _digraph->id(a); }

    Node nodeFromId(int ix) const { return _digraph->nodeFromId(ix); }
    Arc arcFromId(int ix) const { return _digraph->arcFromId(ix); }

    int maxNodeId() const { return _digraph->maxNodeId(); }
    int maxArcId() const { return _digraph->maxArcId(); }

    typedef typename ItemSetTraits<Digraph, Node>::ItemNotifier NodeNotifier;
    NodeNotifier& notifier(Node) const { return _digraph->notifier(Node()); }

    typedef typename ItemSetTraits<Digraph, Arc>::ItemNotifier ArcNotifier;
    ArcNotifier& notifier(Arc) const { return _digraph->notifier(Arc()); }

    template <typename _Value>
    class NodeMap : public Digraph::template NodeMap<_Value> {
    public:

      typedef typename Digraph::template NodeMap<_Value> Parent;

      explicit NodeMap(const Adaptor& adaptor)
        : Parent(*adaptor._digraph) {}

      NodeMap(const Adaptor& adaptor, const _Value& value)
        : Parent(*adaptor._digraph, value) { }

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
    class ArcMap : public Digraph::template ArcMap<_Value> {
    public:

      typedef typename Digraph::template ArcMap<_Value> Parent;

      explicit ArcMap(const Adaptor& adaptor)
        : Parent(*adaptor._digraph) {}

      ArcMap(const Adaptor& adaptor, const _Value& value)
        : Parent(*adaptor._digraph, value) {}

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

  };

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

    typedef ArcNumTagIndicator<Graph> ArcNumTag;
    int arcNum() const { return _graph->arcNum(); }

    typedef EdgeNumTagIndicator<Graph> EdgeNumTag;
    int edgeNum() const { return _graph->edgeNum(); }

    typedef FindArcTagIndicator<Graph> FindArcTag;
    Arc findArc(const Node& u, const Node& v,
                const Arc& prev = INVALID) const {
      return _graph->findArc(u, v, prev);
    }

    typedef FindEdgeTagIndicator<Graph> FindEdgeTag;
    Edge findEdge(const Node& u, const Node& v,
                  const Edge& prev = INVALID) const {
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

  template <typename _Digraph>
  class ReverseDigraphBase : public DigraphAdaptorBase<_Digraph> {
  public:
    typedef _Digraph Digraph;
    typedef DigraphAdaptorBase<_Digraph> Parent;
  protected:
    ReverseDigraphBase() : Parent() { }
  public:
    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;

    void firstIn(Arc& a, const Node& n) const { Parent::firstOut(a, n); }
    void firstOut(Arc& a, const Node& n ) const { Parent::firstIn(a, n); }

    void nextIn(Arc& a) const { Parent::nextOut(a); }
    void nextOut(Arc& a) const { Parent::nextIn(a); }

    Node source(const Arc& a) const { return Parent::target(a); }
    Node target(const Arc& a) const { return Parent::source(a); }

    Arc addArc(const Node& u, const Node& v) { return Parent::addArc(v, u); }

    typedef FindArcTagIndicator<Digraph> FindArcTag;
    Arc findArc(const Node& u, const Node& v,
                const Arc& prev = INVALID) const {
      return Parent::findArc(v, u, prev);
    }

  };

  /// \ingroup graph_adaptors
  ///
  /// \brief A digraph adaptor which reverses the orientation of the arcs.
  ///
  /// ReverseDigraph reverses the arcs in the adapted digraph. The
  /// SubDigraph is conform to the \ref concepts::Digraph
  /// "Digraph concept".
  ///
  /// \tparam _Digraph It must be conform to the \ref concepts::Digraph
  /// "Digraph concept". The type can be specified to be const.
  template<typename _Digraph>
  class ReverseDigraph :
    public DigraphAdaptorExtender<ReverseDigraphBase<_Digraph> > {
  public:
    typedef _Digraph Digraph;
    typedef DigraphAdaptorExtender<
      ReverseDigraphBase<_Digraph> > Parent;
  protected:
    ReverseDigraph() { }
  public:

    /// \brief Constructor
    ///
    /// Creates a reverse digraph adaptor for the given digraph
    explicit ReverseDigraph(Digraph& digraph) {
      Parent::setDigraph(digraph);
    }
  };

  /// \brief Just gives back a reverse digraph adaptor
  ///
  /// Just gives back a reverse digraph adaptor
  template<typename Digraph>
  ReverseDigraph<const Digraph> reverseDigraph(const Digraph& digraph) {
    return ReverseDigraph<const Digraph>(digraph);
  }

  template <typename _Digraph, typename _NodeFilterMap,
            typename _ArcFilterMap, bool _checked = true>
  class SubDigraphBase : public DigraphAdaptorBase<_Digraph> {
  public:
    typedef _Digraph Digraph;
    typedef _NodeFilterMap NodeFilterMap;
    typedef _ArcFilterMap ArcFilterMap;

    typedef SubDigraphBase Adaptor;
    typedef DigraphAdaptorBase<_Digraph> Parent;
  protected:
    NodeFilterMap* _node_filter;
    ArcFilterMap* _arc_filter;
    SubDigraphBase()
      : Parent(), _node_filter(0), _arc_filter(0) { }

    void setNodeFilterMap(NodeFilterMap& node_filter) {
      _node_filter = &node_filter;
    }
    void setArcFilterMap(ArcFilterMap& arc_filter) {
      _arc_filter = &arc_filter;
    }

  public:

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;

    void first(Node& i) const {
      Parent::first(i);
      while (i != INVALID && !(*_node_filter)[i]) Parent::next(i);
    }

    void first(Arc& i) const {
      Parent::first(i);
      while (i != INVALID && (!(*_arc_filter)[i]
                              || !(*_node_filter)[Parent::source(i)]
                              || !(*_node_filter)[Parent::target(i)]))
        Parent::next(i);
    }

    void firstIn(Arc& i, const Node& n) const {
      Parent::firstIn(i, n);
      while (i != INVALID && (!(*_arc_filter)[i]
                              || !(*_node_filter)[Parent::source(i)]))
        Parent::nextIn(i);
    }

    void firstOut(Arc& i, const Node& n) const {
      Parent::firstOut(i, n);
      while (i != INVALID && (!(*_arc_filter)[i]
                              || !(*_node_filter)[Parent::target(i)]))
        Parent::nextOut(i);
    }

    void next(Node& i) const {
      Parent::next(i);
      while (i != INVALID && !(*_node_filter)[i]) Parent::next(i);
    }

    void next(Arc& i) const {
      Parent::next(i);
      while (i != INVALID && (!(*_arc_filter)[i]
                              || !(*_node_filter)[Parent::source(i)]
                              || !(*_node_filter)[Parent::target(i)]))
        Parent::next(i);
    }

    void nextIn(Arc& i) const {
      Parent::nextIn(i);
      while (i != INVALID && (!(*_arc_filter)[i]
                              || !(*_node_filter)[Parent::source(i)]))
        Parent::nextIn(i);
    }

    void nextOut(Arc& i) const {
      Parent::nextOut(i);
      while (i != INVALID && (!(*_arc_filter)[i]
                              || !(*_node_filter)[Parent::target(i)]))
        Parent::nextOut(i);
    }

    void hide(const Node& n) const { _node_filter->set(n, false); }
    void hide(const Arc& a) const { _arc_filter->set(a, false); }

    void unHide(const Node& n) const { _node_filter->set(n, true); }
    void unHide(const Arc& a) const { _arc_filter->set(a, true); }

    bool hidden(const Node& n) const { return !(*_node_filter)[n]; }
    bool hidden(const Arc& a) const { return !(*_arc_filter)[a]; }

    typedef False NodeNumTag;
    typedef False ArcNumTag;

    typedef FindArcTagIndicator<Digraph> FindArcTag;
    Arc findArc(const Node& source, const Node& target,
                const Arc& prev = INVALID) const {
      if (!(*_node_filter)[source] || !(*_node_filter)[target]) {
        return INVALID;
      }
      Arc arc = Parent::findArc(source, target, prev);
      while (arc != INVALID && !(*_arc_filter)[arc]) {
        arc = Parent::findArc(source, target, arc);
      }
      return arc;
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

  };

  template <typename _Digraph, typename _NodeFilterMap, typename _ArcFilterMap>
  class SubDigraphBase<_Digraph, _NodeFilterMap, _ArcFilterMap, false>
    : public DigraphAdaptorBase<_Digraph> {
  public:
    typedef _Digraph Digraph;
    typedef _NodeFilterMap NodeFilterMap;
    typedef _ArcFilterMap ArcFilterMap;

    typedef SubDigraphBase Adaptor;
    typedef DigraphAdaptorBase<Digraph> Parent;
  protected:
    NodeFilterMap* _node_filter;
    ArcFilterMap* _arc_filter;
    SubDigraphBase()
      : Parent(), _node_filter(0), _arc_filter(0) { }

    void setNodeFilterMap(NodeFilterMap& node_filter) {
      _node_filter = &node_filter;
    }
    void setArcFilterMap(ArcFilterMap& arc_filter) {
      _arc_filter = &arc_filter;
    }

  public:

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;

    void first(Node& i) const {
      Parent::first(i);
      while (i!=INVALID && !(*_node_filter)[i]) Parent::next(i);
    }

    void first(Arc& i) const {
      Parent::first(i);
      while (i!=INVALID && !(*_arc_filter)[i]) Parent::next(i);
    }

    void firstIn(Arc& i, const Node& n) const {
      Parent::firstIn(i, n);
      while (i!=INVALID && !(*_arc_filter)[i]) Parent::nextIn(i);
    }

    void firstOut(Arc& i, const Node& n) const {
      Parent::firstOut(i, n);
      while (i!=INVALID && !(*_arc_filter)[i]) Parent::nextOut(i);
    }

    void next(Node& i) const {
      Parent::next(i);
      while (i!=INVALID && !(*_node_filter)[i]) Parent::next(i);
    }
    void next(Arc& i) const {
      Parent::next(i);
      while (i!=INVALID && !(*_arc_filter)[i]) Parent::next(i);
    }
    void nextIn(Arc& i) const {
      Parent::nextIn(i);
      while (i!=INVALID && !(*_arc_filter)[i]) Parent::nextIn(i);
    }

    void nextOut(Arc& i) const {
      Parent::nextOut(i);
      while (i!=INVALID && !(*_arc_filter)[i]) Parent::nextOut(i);
    }

    void hide(const Node& n) const { _node_filter->set(n, false); }
    void hide(const Arc& e) const { _arc_filter->set(e, false); }

    void unHide(const Node& n) const { _node_filter->set(n, true); }
    void unHide(const Arc& e) const { _arc_filter->set(e, true); }

    bool hidden(const Node& n) const { return !(*_node_filter)[n]; }
    bool hidden(const Arc& e) const { return !(*_arc_filter)[e]; }

    typedef False NodeNumTag;
    typedef False ArcNumTag;

    typedef FindArcTagIndicator<Digraph> FindArcTag;
    Arc findArc(const Node& source, const Node& target,
                const Arc& prev = INVALID) const {
      if (!(*_node_filter)[source] || !(*_node_filter)[target]) {
        return INVALID;
      }
      Arc arc = Parent::findArc(source, target, prev);
      while (arc != INVALID && !(*_arc_filter)[arc]) {
        arc = Parent::findArc(source, target, arc);
      }
      return arc;
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

  };

  /// \ingroup graph_adaptors
  ///
  /// \brief An adaptor for hiding nodes and arcs in a digraph
  ///
  /// SubDigraph hides nodes and arcs in a digraph. A bool node map
  /// and a bool arc map must be specified, which define the filters
  /// for nodes and arcs. Just the nodes and arcs with true value are
  /// shown in the subdigraph. The SubDigraph is conform to the \ref
  /// concepts::Digraph "Digraph concept". If the \c _checked parameter
  /// is true, then the arcs incident to filtered nodes are also
  /// filtered out.
  ///
  /// \tparam _Digraph It must be conform to the \ref
  /// concepts::Digraph "Digraph concept". The type can be specified
  /// to const.
  /// \tparam _NodeFilterMap A bool valued node map of the the adapted digraph.
  /// \tparam _ArcFilterMap A bool valued arc map of the the adapted digraph.
  /// \tparam _checked If the parameter is false then the arc filtering
  /// is not checked with respect to node filter. Otherwise, each arc
  /// is automatically filtered, which is incident to a filtered node.
  ///
  /// \see FilterNodes
  /// \see FilterArcs
  template<typename _Digraph,
           typename _NodeFilterMap = typename _Digraph::template NodeMap<bool>,
           typename _ArcFilterMap = typename _Digraph::template ArcMap<bool>,
           bool _checked = true>
  class SubDigraph
    : public DigraphAdaptorExtender<
      SubDigraphBase<_Digraph, _NodeFilterMap, _ArcFilterMap, _checked> > {
  public:
    typedef _Digraph Digraph;
    typedef _NodeFilterMap NodeFilterMap;
    typedef _ArcFilterMap ArcFilterMap;

    typedef DigraphAdaptorExtender<
      SubDigraphBase<Digraph, NodeFilterMap, ArcFilterMap, _checked> >
    Parent;

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;

  protected:
    SubDigraph() { }
  public:

    /// \brief Constructor
    ///
    /// Creates a subdigraph for the given digraph with
    /// given node and arc map filters.
    SubDigraph(Digraph& digraph, NodeFilterMap& node_filter,
               ArcFilterMap& arc_filter) {
      setDigraph(digraph);
      setNodeFilterMap(node_filter);
      setArcFilterMap(arc_filter);
    }

    /// \brief Hides the node of the graph
    ///
    /// This function hides \c n in the digraph, i.e. the iteration
    /// jumps over it. This is done by simply setting the value of \c n
    /// to be false in the corresponding node-map.
    void hide(const Node& n) const { Parent::hide(n); }

    /// \brief Hides the arc of the graph
    ///
    /// This function hides \c a in the digraph, i.e. the iteration
    /// jumps over it. This is done by simply setting the value of \c a
    /// to be false in the corresponding arc-map.
    void hide(const Arc& a) const { Parent::hide(a); }

    /// \brief Unhides the node of the graph
    ///
    /// The value of \c n is set to be true in the node-map which stores
    /// hide information. If \c n was hidden previuosly, then it is shown
    /// again
    void unHide(const Node& n) const { Parent::unHide(n); }

    /// \brief Unhides the arc of the graph
    ///
    /// The value of \c a is set to be true in the arc-map which stores
    /// hide information. If \c a was hidden previuosly, then it is shown
    /// again
    void unHide(const Arc& a) const { Parent::unHide(a); }

    /// \brief Returns true if \c n is hidden.
    ///
    /// Returns true if \c n is hidden.
    ///
    bool hidden(const Node& n) const { return Parent::hidden(n); }

    /// \brief Returns true if \c a is hidden.
    ///
    /// Returns true if \c a is hidden.
    ///
    bool hidden(const Arc& a) const { return Parent::hidden(a); }

  };

  /// \brief Just gives back a subdigraph
  ///
  /// Just gives back a subdigraph
  template<typename Digraph, typename NodeFilterMap, typename ArcFilterMap>
  SubDigraph<const Digraph, NodeFilterMap, ArcFilterMap>
  subDigraph(const Digraph& digraph, NodeFilterMap& nfm, ArcFilterMap& afm) {
    return SubDigraph<const Digraph, NodeFilterMap, ArcFilterMap>
      (digraph, nfm, afm);
  }

  template<typename Digraph, typename NodeFilterMap, typename ArcFilterMap>
  SubDigraph<const Digraph, const NodeFilterMap, ArcFilterMap>
  subDigraph(const Digraph& digraph,
             const NodeFilterMap& nfm, ArcFilterMap& afm) {
    return SubDigraph<const Digraph, const NodeFilterMap, ArcFilterMap>
      (digraph, nfm, afm);
  }

  template<typename Digraph, typename NodeFilterMap, typename ArcFilterMap>
  SubDigraph<const Digraph, NodeFilterMap, const ArcFilterMap>
  subDigraph(const Digraph& digraph,
             NodeFilterMap& nfm, const ArcFilterMap& afm) {
    return SubDigraph<const Digraph, NodeFilterMap, const ArcFilterMap>
      (digraph, nfm, afm);
  }

  template<typename Digraph, typename NodeFilterMap, typename ArcFilterMap>
  SubDigraph<const Digraph, const NodeFilterMap, const ArcFilterMap>
  subDigraph(const Digraph& digraph,
             const NodeFilterMap& nfm, const ArcFilterMap& afm) {
    return SubDigraph<const Digraph, const NodeFilterMap,
      const ArcFilterMap>(digraph, nfm, afm);
  }


  template <typename _Graph, typename _NodeFilterMap,
            typename _EdgeFilterMap, bool _checked = true>
  class SubGraphBase : public GraphAdaptorBase<_Graph> {
  public:
    typedef _Graph Graph;
    typedef _NodeFilterMap NodeFilterMap;
    typedef _EdgeFilterMap EdgeFilterMap;

    typedef SubGraphBase Adaptor;
    typedef GraphAdaptorBase<_Graph> Parent;
  protected:

    NodeFilterMap* _node_filter_map;
    EdgeFilterMap* _edge_filter_map;

    SubGraphBase()
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
                            || !(*_node_filter_map)[Parent::target(i)]))
        Parent::next(i);
    }

    void first(Edge& i) const {
      Parent::first(i);
      while (i!=INVALID && (!(*_edge_filter_map)[i]
                            || !(*_node_filter_map)[Parent::u(i)]
                            || !(*_node_filter_map)[Parent::v(i)]))
        Parent::next(i);
    }

    void firstIn(Arc& i, const Node& n) const {
      Parent::firstIn(i, n);
      while (i!=INVALID && (!(*_edge_filter_map)[i]
                            || !(*_node_filter_map)[Parent::source(i)]))
        Parent::nextIn(i);
    }

    void firstOut(Arc& i, const Node& n) const {
      Parent::firstOut(i, n);
      while (i!=INVALID && (!(*_edge_filter_map)[i]
                            || !(*_node_filter_map)[Parent::target(i)]))
        Parent::nextOut(i);
    }

    void firstInc(Edge& i, bool& d, const Node& n) const {
      Parent::firstInc(i, d, n);
      while (i!=INVALID && (!(*_edge_filter_map)[i]
                            || !(*_node_filter_map)[Parent::u(i)]
                            || !(*_node_filter_map)[Parent::v(i)]))
        Parent::nextInc(i, d);
    }

    void next(Node& i) const {
      Parent::next(i);
      while (i!=INVALID && !(*_node_filter_map)[i]) Parent::next(i);
    }

    void next(Arc& i) const {
      Parent::next(i);
      while (i!=INVALID && (!(*_edge_filter_map)[i]
                            || !(*_node_filter_map)[Parent::source(i)]
                            || !(*_node_filter_map)[Parent::target(i)]))
        Parent::next(i);
    }

    void next(Edge& i) const {
      Parent::next(i);
      while (i!=INVALID && (!(*_edge_filter_map)[i]
                            || !(*_node_filter_map)[Parent::u(i)]
                            || !(*_node_filter_map)[Parent::v(i)]))
        Parent::next(i);
    }

    void nextIn(Arc& i) const {
      Parent::nextIn(i);
      while (i!=INVALID && (!(*_edge_filter_map)[i]
                            || !(*_node_filter_map)[Parent::source(i)]))
        Parent::nextIn(i);
    }

    void nextOut(Arc& i) const {
      Parent::nextOut(i);
      while (i!=INVALID && (!(*_edge_filter_map)[i]
                            || !(*_node_filter_map)[Parent::target(i)]))
        Parent::nextOut(i);
    }

    void nextInc(Edge& i, bool& d) const {
      Parent::nextInc(i, d);
      while (i!=INVALID && (!(*_edge_filter_map)[i]
                            || !(*_node_filter_map)[Parent::u(i)]
                            || !(*_node_filter_map)[Parent::v(i)]))
        Parent::nextInc(i, d);
    }

    void hide(const Node& n) const { _node_filter_map->set(n, false); }
    void hide(const Edge& e) const { _edge_filter_map->set(e, false); }

    void unHide(const Node& n) const { _node_filter_map->set(n, true); }
    void unHide(const Edge& e) const { _edge_filter_map->set(e, true); }

    bool hidden(const Node& n) const { return !(*_node_filter_map)[n]; }
    bool hidden(const Edge& e) const { return !(*_edge_filter_map)[e]; }

    typedef False NodeNumTag;
    typedef False ArcNumTag;
    typedef False EdgeNumTag;

    typedef FindArcTagIndicator<Graph> FindArcTag;
    Arc findArc(const Node& u, const Node& v,
                const Arc& prev = INVALID) const {
      if (!(*_node_filter_map)[u] || !(*_node_filter_map)[v]) {
        return INVALID;
      }
      Arc arc = Parent::findArc(u, v, prev);
      while (arc != INVALID && !(*_edge_filter_map)[arc]) {
        arc = Parent::findArc(u, v, arc);
      }
      return arc;
    }

    typedef FindEdgeTagIndicator<Graph> FindEdgeTag;
    Edge findEdge(const Node& u, const Node& v,
                  const Edge& prev = INVALID) const {
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

  template <typename _Graph, typename _NodeFilterMap, typename _EdgeFilterMap>
  class SubGraphBase<_Graph, _NodeFilterMap, _EdgeFilterMap, false>
    : public GraphAdaptorBase<_Graph> {
  public:
    typedef _Graph Graph;
    typedef _NodeFilterMap NodeFilterMap;
    typedef _EdgeFilterMap EdgeFilterMap;

    typedef SubGraphBase Adaptor;
    typedef GraphAdaptorBase<_Graph> Parent;
  protected:
    NodeFilterMap* _node_filter_map;
    EdgeFilterMap* _edge_filter_map;
    SubGraphBase() : Parent(),
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

    void hide(const Node& n) const { _node_filter_map->set(n, false); }
    void hide(const Edge& e) const { _edge_filter_map->set(e, false); }

    void unHide(const Node& n) const { _node_filter_map->set(n, true); }
    void unHide(const Edge& e) const { _edge_filter_map->set(e, true); }

    bool hidden(const Node& n) const { return !(*_node_filter_map)[n]; }
    bool hidden(const Edge& e) const { return !(*_edge_filter_map)[e]; }

    typedef False NodeNumTag;
    typedef False ArcNumTag;
    typedef False EdgeNumTag;

    typedef FindArcTagIndicator<Graph> FindArcTag;
    Arc findArc(const Node& u, const Node& v,
                const Arc& prev = INVALID) const {
      Arc arc = Parent::findArc(u, v, prev);
      while (arc != INVALID && !(*_edge_filter_map)[arc]) {
        arc = Parent::findArc(u, v, arc);
      }
      return arc;
    }

    typedef FindEdgeTagIndicator<Graph> FindEdgeTag;
    Edge findEdge(const Node& u, const Node& v,
                  const Edge& prev = INVALID) const {
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
  /// \brief A graph adaptor for hiding nodes and edges in an
  /// undirected graph.
  ///
  /// SubGraph hides nodes and edges in a graph. A bool node map and a
  /// bool edge map must be specified, which define the filters for
  /// nodes and edges. Just the nodes and edges with true value are
  /// shown in the subgraph. The SubGraph is conform to the \ref
  /// concepts::Graph "Graph concept". If the \c _checked parameter is
  /// true, then the edges incident to filtered nodes are also
  /// filtered out.
  ///
  /// \tparam _Graph It must be conform to the \ref
  /// concepts::Graph "Graph concept". The type can be specified
  /// to const.
  /// \tparam _NodeFilterMap A bool valued node map of the the adapted graph.
  /// \tparam _EdgeFilterMap A bool valued edge map of the the adapted graph.
  /// \tparam _checked If the parameter is false then the edge filtering
  /// is not checked with respect to node filter. Otherwise, each edge
  /// is automatically filtered, which is incident to a filtered node.
  ///
  /// \see FilterNodes
  /// \see FilterEdges
  template<typename _Graph, typename NodeFilterMap,
           typename EdgeFilterMap, bool _checked = true>
  class SubGraph
    : public GraphAdaptorExtender<
      SubGraphBase<_Graph, NodeFilterMap, EdgeFilterMap, _checked> > {
  public:
    typedef _Graph Graph;
    typedef GraphAdaptorExtender<
      SubGraphBase<_Graph, NodeFilterMap, EdgeFilterMap> > Parent;

    typedef typename Parent::Node Node;
    typedef typename Parent::Edge Edge;

  protected:
    SubGraph() { }
  public:

    /// \brief Constructor
    ///
    /// Creates a subgraph for the given graph with given node and
    /// edge map filters.
    SubGraph(Graph& _graph, NodeFilterMap& node_filter_map,
             EdgeFilterMap& edge_filter_map) {
      setGraph(_graph);
      setNodeFilterMap(node_filter_map);
      setEdgeFilterMap(edge_filter_map);
    }

    /// \brief Hides the node of the graph
    ///
    /// This function hides \c n in the graph, i.e. the iteration
    /// jumps over it. This is done by simply setting the value of \c n
    /// to be false in the corresponding node-map.
    void hide(const Node& n) const { Parent::hide(n); }

    /// \brief Hides the edge of the graph
    ///
    /// This function hides \c e in the graph, i.e. the iteration
    /// jumps over it. This is done by simply setting the value of \c e
    /// to be false in the corresponding edge-map.
    void hide(const Edge& e) const { Parent::hide(e); }

    /// \brief Unhides the node of the graph
    ///
    /// The value of \c n is set to be true in the node-map which stores
    /// hide information. If \c n was hidden previuosly, then it is shown
    /// again
    void unHide(const Node& n) const { Parent::unHide(n); }

    /// \brief Unhides the edge of the graph
    ///
    /// The value of \c e is set to be true in the edge-map which stores
    /// hide information. If \c e was hidden previuosly, then it is shown
    /// again
    void unHide(const Edge& e) const { Parent::unHide(e); }

    /// \brief Returns true if \c n is hidden.
    ///
    /// Returns true if \c n is hidden.
    ///
    bool hidden(const Node& n) const { return Parent::hidden(n); }

    /// \brief Returns true if \c e is hidden.
    ///
    /// Returns true if \c e is hidden.
    ///
    bool hidden(const Edge& e) const { return Parent::hidden(e); }
  };

  /// \brief Just gives back a subgraph
  ///
  /// Just gives back a subgraph
  template<typename Graph, typename NodeFilterMap, typename ArcFilterMap>
  SubGraph<const Graph, NodeFilterMap, ArcFilterMap>
  subGraph(const Graph& graph, NodeFilterMap& nfm, ArcFilterMap& efm) {
    return SubGraph<const Graph, NodeFilterMap, ArcFilterMap>(graph, nfm, efm);
  }

  template<typename Graph, typename NodeFilterMap, typename ArcFilterMap>
  SubGraph<const Graph, const NodeFilterMap, ArcFilterMap>
  subGraph(const Graph& graph,
           const NodeFilterMap& nfm, ArcFilterMap& efm) {
    return SubGraph<const Graph, const NodeFilterMap, ArcFilterMap>
      (graph, nfm, efm);
  }

  template<typename Graph, typename NodeFilterMap, typename ArcFilterMap>
  SubGraph<const Graph, NodeFilterMap, const ArcFilterMap>
  subGraph(const Graph& graph,
           NodeFilterMap& nfm, const ArcFilterMap& efm) {
    return SubGraph<const Graph, NodeFilterMap, const ArcFilterMap>
      (graph, nfm, efm);
  }

  template<typename Graph, typename NodeFilterMap, typename ArcFilterMap>
  SubGraph<const Graph, const NodeFilterMap, const ArcFilterMap>
  subGraph(const Graph& graph,
           const NodeFilterMap& nfm, const ArcFilterMap& efm) {
    return SubGraph<const Graph, const NodeFilterMap, const ArcFilterMap>
      (graph, nfm, efm);
  }

  /// \ingroup graph_adaptors
  ///
  /// \brief An adaptor for hiding nodes from a digraph or a graph.
  ///
  /// FilterNodes adaptor hides nodes in a graph or a digraph. A bool
  /// node map must be specified, which defines the filters for
  /// nodes. Just the unfiltered nodes and the arcs or edges incident
  /// to unfiltered nodes are shown in the subdigraph or subgraph. The
  /// FilterNodes is conform to the \ref concepts::Digraph
  /// "Digraph concept" or \ref concepts::Graph "Graph concept" depending
  /// on the \c _Digraph template parameter. If the \c _checked
  /// parameter is true, then the arc or edges incident to filtered nodes
  /// are also filtered out.
  ///
  /// \tparam _Digraph It must be conform to the \ref
  /// concepts::Digraph "Digraph concept" or \ref concepts::Graph
  /// "Graph concept". The type can be specified to be const.
  /// \tparam _NodeFilterMap A bool valued node map of the the adapted graph.
  /// \tparam _checked If the parameter is false then the arc or edge
  /// filtering is not checked with respect to node filter. In this
  /// case just isolated nodes can be filtered out from the
  /// graph.
#ifdef DOXYGEN
  template<typename _Digraph,
           typename _NodeFilterMap = typename _Digraph::template NodeMap<bool>,
           bool _checked = true>
#else
  template<typename _Digraph,
           typename _NodeFilterMap = typename _Digraph::template NodeMap<bool>,
           bool _checked = true,
           typename Enable = void>
#endif
  class FilterNodes
    : public SubDigraph<_Digraph, _NodeFilterMap,
                        ConstMap<typename _Digraph::Arc, bool>, _checked> {
  public:

    typedef _Digraph Digraph;
    typedef _NodeFilterMap NodeFilterMap;

    typedef SubDigraph<Digraph, NodeFilterMap,
                       ConstMap<typename Digraph::Arc, bool>, _checked>
    Parent;

    typedef typename Parent::Node Node;

  protected:
    ConstMap<typename Digraph::Arc, bool> const_true_map;

    FilterNodes() : const_true_map(true) {
      Parent::setArcFilterMap(const_true_map);
    }

  public:

    /// \brief Constructor
    ///
    /// Creates an adaptor for the given digraph or graph with
    /// given node filter map.
    FilterNodes(Digraph& _digraph, NodeFilterMap& node_filter) :
      Parent(), const_true_map(true) {
      Parent::setDigraph(_digraph);
      Parent::setNodeFilterMap(node_filter);
      Parent::setArcFilterMap(const_true_map);
    }

    /// \brief Hides the node of the graph
    ///
    /// This function hides \c n in the digraph or graph, i.e. the iteration
    /// jumps over it. This is done by simply setting the value of \c n
    /// to be false in the corresponding node map.
    void hide(const Node& n) const { Parent::hide(n); }

    /// \brief Unhides the node of the graph
    ///
    /// The value of \c n is set to be true in the node-map which stores
    /// hide information. If \c n was hidden previuosly, then it is shown
    /// again
    void unHide(const Node& n) const { Parent::unHide(n); }

    /// \brief Returns true if \c n is hidden.
    ///
    /// Returns true if \c n is hidden.
    ///
    bool hidden(const Node& n) const { return Parent::hidden(n); }

  };

  template<typename _Graph, typename _NodeFilterMap, bool _checked>
  class FilterNodes<_Graph, _NodeFilterMap, _checked,
                    typename enable_if<UndirectedTagIndicator<_Graph> >::type>
    : public SubGraph<_Graph, _NodeFilterMap,
                      ConstMap<typename _Graph::Edge, bool>, _checked> {
  public:
    typedef _Graph Graph;
    typedef _NodeFilterMap NodeFilterMap;
    typedef SubGraph<Graph, NodeFilterMap,
                     ConstMap<typename Graph::Edge, bool> > Parent;

    typedef typename Parent::Node Node;
  protected:
    ConstMap<typename Graph::Edge, bool> const_true_map;

    FilterNodes() : const_true_map(true) {
      Parent::setEdgeFilterMap(const_true_map);
    }

  public:

    FilterNodes(Graph& _graph, NodeFilterMap& node_filter_map) :
      Parent(), const_true_map(true) {
      Parent::setGraph(_graph);
      Parent::setNodeFilterMap(node_filter_map);
      Parent::setEdgeFilterMap(const_true_map);
    }

    void hide(const Node& n) const { Parent::hide(n); }
    void unHide(const Node& n) const { Parent::unHide(n); }
    bool hidden(const Node& n) const { return Parent::hidden(n); }

  };


  /// \brief Just gives back a FilterNodes adaptor
  ///
  /// Just gives back a FilterNodes adaptor
  template<typename Digraph, typename NodeFilterMap>
  FilterNodes<const Digraph, NodeFilterMap>
  filterNodes(const Digraph& digraph, NodeFilterMap& nfm) {
    return FilterNodes<const Digraph, NodeFilterMap>(digraph, nfm);
  }

  template<typename Digraph, typename NodeFilterMap>
  FilterNodes<const Digraph, const NodeFilterMap>
  filterNodes(const Digraph& digraph, const NodeFilterMap& nfm) {
    return FilterNodes<const Digraph, const NodeFilterMap>(digraph, nfm);
  }

  /// \ingroup graph_adaptors
  ///
  /// \brief An adaptor for hiding arcs from a digraph.
  ///
  /// FilterArcs adaptor hides arcs in a digraph. A bool arc map must
  /// be specified, which defines the filters for arcs. Just the
  /// unfiltered arcs are shown in the subdigraph. The FilterArcs is
  /// conform to the \ref concepts::Digraph "Digraph concept".
  ///
  /// \tparam _Digraph It must be conform to the \ref concepts::Digraph
  /// "Digraph concept". The type can be specified to be const.
  /// \tparam _ArcFilterMap A bool valued arc map of the the adapted
  /// graph.
  template<typename _Digraph, typename _ArcFilterMap>
  class FilterArcs :
    public SubDigraph<_Digraph, ConstMap<typename _Digraph::Node, bool>,
                      _ArcFilterMap, false> {
  public:
    typedef _Digraph Digraph;
    typedef _ArcFilterMap ArcFilterMap;

    typedef SubDigraph<Digraph, ConstMap<typename Digraph::Node, bool>,
                       ArcFilterMap, false> Parent;

    typedef typename Parent::Arc Arc;

  protected:
    ConstMap<typename Digraph::Node, bool> const_true_map;

    FilterArcs() : const_true_map(true) {
      Parent::setNodeFilterMap(const_true_map);
    }

  public:

    /// \brief Constructor
    ///
    /// Creates a FilterArcs adaptor for the given graph with
    /// given arc map filter.
    FilterArcs(Digraph& digraph, ArcFilterMap& arc_filter)
      : Parent(), const_true_map(true) {
      Parent::setDigraph(digraph);
      Parent::setNodeFilterMap(const_true_map);
      Parent::setArcFilterMap(arc_filter);
    }

    /// \brief Hides the arc of the graph
    ///
    /// This function hides \c a in the graph, i.e. the iteration
    /// jumps over it. This is done by simply setting the value of \c a
    /// to be false in the corresponding arc map.
    void hide(const Arc& a) const { Parent::hide(a); }

    /// \brief Unhides the arc of the graph
    ///
    /// The value of \c a is set to be true in the arc-map which stores
    /// hide information. If \c a was hidden previuosly, then it is shown
    /// again
    void unHide(const Arc& a) const { Parent::unHide(a); }

    /// \brief Returns true if \c a is hidden.
    ///
    /// Returns true if \c a is hidden.
    ///
    bool hidden(const Arc& a) const { return Parent::hidden(a); }

  };

  /// \brief Just gives back an FilterArcs adaptor
  ///
  /// Just gives back an FilterArcs adaptor
  template<typename Digraph, typename ArcFilterMap>
  FilterArcs<const Digraph, ArcFilterMap>
  filterArcs(const Digraph& digraph, ArcFilterMap& afm) {
    return FilterArcs<const Digraph, ArcFilterMap>(digraph, afm);
  }

  template<typename Digraph, typename ArcFilterMap>
  FilterArcs<const Digraph, const ArcFilterMap>
  filterArcs(const Digraph& digraph, const ArcFilterMap& afm) {
    return FilterArcs<const Digraph, const ArcFilterMap>(digraph, afm);
  }

  /// \ingroup graph_adaptors
  ///
  /// \brief An adaptor for hiding edges from a graph.
  ///
  /// FilterEdges adaptor hides edges in a digraph. A bool edge map must
  /// be specified, which defines the filters for edges. Just the
  /// unfiltered edges are shown in the subdigraph. The FilterEdges is
  /// conform to the \ref concepts::Graph "Graph concept".
  ///
  /// \tparam _Graph It must be conform to the \ref concepts::Graph
  /// "Graph concept". The type can be specified to be const.
  /// \tparam _EdgeFilterMap A bool valued edge map of the the adapted
  /// graph.
  template<typename _Graph, typename _EdgeFilterMap>
  class FilterEdges :
    public SubGraph<_Graph, ConstMap<typename _Graph::Node,bool>,
                    _EdgeFilterMap, false> {
  public:
    typedef _Graph Graph;
    typedef _EdgeFilterMap EdgeFilterMap;
    typedef SubGraph<Graph, ConstMap<typename Graph::Node,bool>,
                     EdgeFilterMap, false> Parent;
    typedef typename Parent::Edge Edge;
  protected:
    ConstMap<typename Graph::Node, bool> const_true_map;

    FilterEdges() : const_true_map(true) {
      Parent::setNodeFilterMap(const_true_map);
    }

  public:

    /// \brief Constructor
    ///
    /// Creates a FilterEdges adaptor for the given graph with
    /// given edge map filters.
    FilterEdges(Graph& _graph, EdgeFilterMap& edge_filter_map) :
      Parent(), const_true_map(true) {
      Parent::setGraph(_graph);
      Parent::setNodeFilterMap(const_true_map);
      Parent::setEdgeFilterMap(edge_filter_map);
    }

    /// \brief Hides the edge of the graph
    ///
    /// This function hides \c e in the graph, i.e. the iteration
    /// jumps over it. This is done by simply setting the value of \c e
    /// to be false in the corresponding edge-map.
    void hide(const Edge& e) const { Parent::hide(e); }

    /// \brief Unhides the edge of the graph
    ///
    /// The value of \c e is set to be true in the edge-map which stores
    /// hide information. If \c e was hidden previuosly, then it is shown
    /// again
    void unHide(const Edge& e) const { Parent::unHide(e); }

    /// \brief Returns true if \c e is hidden.
    ///
    /// Returns true if \c e is hidden.
    ///
    bool hidden(const Edge& e) const { return Parent::hidden(e); }

  };

  /// \brief Just gives back a FilterEdges adaptor
  ///
  /// Just gives back a FilterEdges adaptor
  template<typename Graph, typename EdgeFilterMap>
  FilterEdges<const Graph, EdgeFilterMap>
  filterEdges(const Graph& graph, EdgeFilterMap& efm) {
    return FilterEdges<const Graph, EdgeFilterMap>(graph, efm);
  }

  template<typename Graph, typename EdgeFilterMap>
  FilterEdges<const Graph, const EdgeFilterMap>
  filterEdges(const Graph& graph, const EdgeFilterMap& efm) {
    return FilterEdges<const Graph, const EdgeFilterMap>(graph, efm);
  }

  template <typename _Digraph>
  class UndirectorBase {
  public:
    typedef _Digraph Digraph;
    typedef UndirectorBase Adaptor;

    typedef True UndirectedTag;

    typedef typename Digraph::Arc Edge;
    typedef typename Digraph::Node Node;

    class Arc : public Edge {
      friend class UndirectorBase;
    protected:
      bool _forward;

      Arc(const Edge& edge, bool forward) :
        Edge(edge), _forward(forward) {}

    public:
      Arc() {}

      Arc(Invalid) : Edge(INVALID), _forward(true) {}

      bool operator==(const Arc &other) const {
        return _forward == other._forward &&
          static_cast<const Edge&>(*this) == static_cast<const Edge&>(other);
      }
      bool operator!=(const Arc &other) const {
        return _forward != other._forward ||
          static_cast<const Edge&>(*this) != static_cast<const Edge&>(other);
      }
      bool operator<(const Arc &other) const {
        return _forward < other._forward ||
          (_forward == other._forward &&
           static_cast<const Edge&>(*this) < static_cast<const Edge&>(other));
      }
    };



    void first(Node& n) const {
      _digraph->first(n);
    }

    void next(Node& n) const {
      _digraph->next(n);
    }

    void first(Arc& a) const {
      _digraph->first(a);
      a._forward = true;
    }

    void next(Arc& a) const {
      if (a._forward) {
        a._forward = false;
      } else {
        _digraph->next(a);
        a._forward = true;
      }
    }

    void first(Edge& e) const {
      _digraph->first(e);
    }

    void next(Edge& e) const {
      _digraph->next(e);
    }

    void firstOut(Arc& a, const Node& n) const {
      _digraph->firstIn(a, n);
      if( static_cast<const Edge&>(a) != INVALID ) {
        a._forward = false;
      } else {
        _digraph->firstOut(a, n);
        a._forward = true;
      }
    }
    void nextOut(Arc &a) const {
      if (!a._forward) {
        Node n = _digraph->target(a);
        _digraph->nextIn(a);
        if (static_cast<const Edge&>(a) == INVALID ) {
          _digraph->firstOut(a, n);
          a._forward = true;
        }
      }
      else {
        _digraph->nextOut(a);
      }
    }

    void firstIn(Arc &a, const Node &n) const {
      _digraph->firstOut(a, n);
      if (static_cast<const Edge&>(a) != INVALID ) {
        a._forward = false;
      } else {
        _digraph->firstIn(a, n);
        a._forward = true;
      }
    }
    void nextIn(Arc &a) const {
      if (!a._forward) {
        Node n = _digraph->source(a);
        _digraph->nextOut(a);
        if( static_cast<const Edge&>(a) == INVALID ) {
          _digraph->firstIn(a, n);
          a._forward = true;
        }
      }
      else {
        _digraph->nextIn(a);
      }
    }

    void firstInc(Edge &e, bool &d, const Node &n) const {
      d = true;
      _digraph->firstOut(e, n);
      if (e != INVALID) return;
      d = false;
      _digraph->firstIn(e, n);
    }

    void nextInc(Edge &e, bool &d) const {
      if (d) {
        Node s = _digraph->source(e);
        _digraph->nextOut(e);
        if (e != INVALID) return;
        d = false;
        _digraph->firstIn(e, s);
      } else {
        _digraph->nextIn(e);
      }
    }

    Node u(const Edge& e) const {
      return _digraph->source(e);
    }

    Node v(const Edge& e) const {
      return _digraph->target(e);
    }

    Node source(const Arc &a) const {
      return a._forward ? _digraph->source(a) : _digraph->target(a);
    }

    Node target(const Arc &a) const {
      return a._forward ? _digraph->target(a) : _digraph->source(a);
    }

    static Arc direct(const Edge &e, bool d) {
      return Arc(e, d);
    }
    Arc direct(const Edge &e, const Node& n) const {
      return Arc(e, _digraph->source(e) == n);
    }

    static bool direction(const Arc &a) { return a._forward; }

    Node nodeFromId(int ix) const { return _digraph->nodeFromId(ix); }
    Arc arcFromId(int ix) const {
      return direct(_digraph->arcFromId(ix >> 1), bool(ix & 1));
    }
    Edge edgeFromId(int ix) const { return _digraph->arcFromId(ix); }

    int id(const Node &n) const { return _digraph->id(n); }
    int id(const Arc &a) const {
      return  (_digraph->id(a) << 1) | (a._forward ? 1 : 0);
    }
    int id(const Edge &e) const { return _digraph->id(e); }

    int maxNodeId() const { return _digraph->maxNodeId(); }
    int maxArcId() const { return (_digraph->maxArcId() << 1) | 1; }
    int maxEdgeId() const { return _digraph->maxArcId(); }

    Node addNode() { return _digraph->addNode(); }
    Edge addEdge(const Node& u, const Node& v) {
      return _digraph->addArc(u, v);
    }

    void erase(const Node& i) { _digraph->erase(i); }
    void erase(const Edge& i) { _digraph->erase(i); }

    void clear() { _digraph->clear(); }

    typedef NodeNumTagIndicator<Digraph> NodeNumTag;
    int nodeNum() const { return _digraph->nodeNum(); }

    typedef ArcNumTagIndicator<Digraph> ArcNumTag;
    int arcNum() const { return 2 * _digraph->arcNum(); }

    typedef ArcNumTag EdgeNumTag;
    int edgeNum() const { return _digraph->arcNum(); }

    typedef FindArcTagIndicator<Digraph> FindArcTag;
    Arc findArc(Node s, Node t, Arc p = INVALID) const {
      if (p == INVALID) {
        Edge arc = _digraph->findArc(s, t);
        if (arc != INVALID) return direct(arc, true);
        arc = _digraph->findArc(t, s);
        if (arc != INVALID) return direct(arc, false);
      } else if (direction(p)) {
        Edge arc = _digraph->findArc(s, t, p);
        if (arc != INVALID) return direct(arc, true);
        arc = _digraph->findArc(t, s);
        if (arc != INVALID) return direct(arc, false);
      } else {
        Edge arc = _digraph->findArc(t, s, p);
        if (arc != INVALID) return direct(arc, false);
      }
      return INVALID;
    }

    typedef FindArcTag FindEdgeTag;
    Edge findEdge(Node s, Node t, Edge p = INVALID) const {
      if (s != t) {
        if (p == INVALID) {
          Edge arc = _digraph->findArc(s, t);
          if (arc != INVALID) return arc;
          arc = _digraph->findArc(t, s);
          if (arc != INVALID) return arc;
        } else if (_digraph->source(p) == s) {
          Edge arc = _digraph->findArc(s, t, p);
          if (arc != INVALID) return arc;
          arc = _digraph->findArc(t, s);
          if (arc != INVALID) return arc;
        } else {
          Edge arc = _digraph->findArc(t, s, p);
          if (arc != INVALID) return arc;
        }
      } else {
        return _digraph->findArc(s, t, p);
      }
      return INVALID;
    }

  private:

    template <typename _Value>
    class ArcMapBase {
    private:

      typedef typename Digraph::template ArcMap<_Value> MapImpl;

    public:

      typedef typename MapTraits<MapImpl>::ReferenceMapTag ReferenceMapTag;

      typedef _Value Value;
      typedef Arc Key;
      typedef typename MapTraits<MapImpl>::ConstReturnValue ConstReturnValue;
      typedef typename MapTraits<MapImpl>::ReturnValue ReturnValue;
      typedef typename MapTraits<MapImpl>::ConstReturnValue ConstReference;
      typedef typename MapTraits<MapImpl>::ReturnValue Reference;

      ArcMapBase(const Adaptor& adaptor) :
        _forward(*adaptor._digraph), _backward(*adaptor._digraph) {}

      ArcMapBase(const Adaptor& adaptor, const Value& v)
        : _forward(*adaptor._digraph, v), _backward(*adaptor._digraph, v) {}

      void set(const Arc& a, const Value& v) {
        if (direction(a)) {
          _forward.set(a, v);
        } else {
          _backward.set(a, v);
        }
      }

      ConstReturnValue operator[](const Arc& a) const {
        if (direction(a)) {
          return _forward[a];
        } else {
          return _backward[a];
        }
      }

      ReturnValue operator[](const Arc& a) {
        if (direction(a)) {
          return _forward[a];
        } else {
          return _backward[a];
        }
      }

    protected:

      MapImpl _forward, _backward;

    };

  public:

    template <typename _Value>
    class NodeMap : public Digraph::template NodeMap<_Value> {
    public:

      typedef _Value Value;
      typedef typename Digraph::template NodeMap<Value> Parent;

      explicit NodeMap(const Adaptor& adaptor)
        : Parent(*adaptor._digraph) {}

      NodeMap(const Adaptor& adaptor, const _Value& value)
        : Parent(*adaptor._digraph, value) { }

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
    class ArcMap
      : public SubMapExtender<Adaptor, ArcMapBase<_Value> >
    {
    public:
      typedef _Value Value;
      typedef SubMapExtender<Adaptor, ArcMapBase<Value> > Parent;

      explicit ArcMap(const Adaptor& adaptor)
        : Parent(adaptor) {}

      ArcMap(const Adaptor& adaptor, const Value& value)
        : Parent(adaptor, value) {}

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
    class EdgeMap : public Digraph::template ArcMap<_Value> {
    public:

      typedef _Value Value;
      typedef typename Digraph::template ArcMap<Value> Parent;

      explicit EdgeMap(const Adaptor& adaptor)
        : Parent(*adaptor._digraph) {}

      EdgeMap(const Adaptor& adaptor, const Value& value)
        : Parent(*adaptor._digraph, value) {}

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

    typedef typename ItemSetTraits<Digraph, Node>::ItemNotifier NodeNotifier;
    NodeNotifier& notifier(Node) const { return _digraph->notifier(Node()); }

    typedef typename ItemSetTraits<Digraph, Edge>::ItemNotifier EdgeNotifier;
    EdgeNotifier& notifier(Edge) const { return _digraph->notifier(Edge()); }

  protected:

    UndirectorBase() : _digraph(0) {}

    Digraph* _digraph;

    void setDigraph(Digraph& digraph) {
      _digraph = &digraph;
    }

  };

  /// \ingroup graph_adaptors
  ///
  /// \brief Undirect the graph
  ///
  /// This adaptor makes an undirected graph from a directed
  /// graph. All arcs of the underlying digraph will be showed in the
  /// adaptor as an edge. The Orienter adaptor is conform to the \ref
  /// concepts::Graph "Graph concept".
  ///
  /// \tparam _Digraph It must be conform to the \ref
  /// concepts::Digraph "Digraph concept". The type can be specified
  /// to const.
  template<typename _Digraph>
  class Undirector
    : public GraphAdaptorExtender<UndirectorBase<_Digraph> > {
  public:
    typedef _Digraph Digraph;
    typedef GraphAdaptorExtender<UndirectorBase<Digraph> > Parent;
  protected:
    Undirector() { }
  public:

    /// \brief Constructor
    ///
    /// Creates a undirected graph from the given digraph
    Undirector(_Digraph& digraph) {
      setDigraph(digraph);
    }

    /// \brief ArcMap combined from two original ArcMap
    ///
    /// This class adapts two original digraph ArcMap to
    /// get an arc map on the undirected graph.
    template <typename _ForwardMap, typename _BackwardMap>
    class CombinedArcMap {
    public:

      typedef _ForwardMap ForwardMap;
      typedef _BackwardMap BackwardMap;

      typedef typename MapTraits<ForwardMap>::ReferenceMapTag ReferenceMapTag;

      typedef typename ForwardMap::Value Value;
      typedef typename Parent::Arc Key;

      typedef typename MapTraits<ForwardMap>::ReturnValue ReturnValue;
      typedef typename MapTraits<ForwardMap>::ConstReturnValue ConstReturnValue;
      typedef typename MapTraits<ForwardMap>::ReturnValue Reference;
      typedef typename MapTraits<ForwardMap>::ConstReturnValue ConstReference;

      /// \brief Constructor
      ///
      /// Constructor
      CombinedArcMap(ForwardMap& forward, BackwardMap& backward)
        : _forward(&forward), _backward(&backward) {}


      /// \brief Sets the value associated with a key.
      ///
      /// Sets the value associated with a key.
      void set(const Key& e, const Value& a) {
        if (Parent::direction(e)) {
          _forward->set(e, a);
        } else {
          _backward->set(e, a);
        }
      }

      /// \brief Returns the value associated with a key.
      ///
      /// Returns the value associated with a key.
      ConstReturnValue operator[](const Key& e) const {
        if (Parent::direction(e)) {
          return (*_forward)[e];
        } else {
          return (*_backward)[e];
        }
      }

      /// \brief Returns the value associated with a key.
      ///
      /// Returns the value associated with a key.
      ReturnValue operator[](const Key& e) {
        if (Parent::direction(e)) {
          return (*_forward)[e];
        } else {
          return (*_backward)[e];
        }
      }

    protected:

      ForwardMap* _forward;
      BackwardMap* _backward;

    };

    /// \brief Just gives back a combined arc map
    ///
    /// Just gives back a combined arc map
    template <typename ForwardMap, typename BackwardMap>
    static CombinedArcMap<ForwardMap, BackwardMap>
    combinedArcMap(ForwardMap& forward, BackwardMap& backward) {
      return CombinedArcMap<ForwardMap, BackwardMap>(forward, backward);
    }

    template <typename ForwardMap, typename BackwardMap>
    static CombinedArcMap<const ForwardMap, BackwardMap>
    combinedArcMap(const ForwardMap& forward, BackwardMap& backward) {
      return CombinedArcMap<const ForwardMap,
        BackwardMap>(forward, backward);
    }

    template <typename ForwardMap, typename BackwardMap>
    static CombinedArcMap<ForwardMap, const BackwardMap>
    combinedArcMap(ForwardMap& forward, const BackwardMap& backward) {
      return CombinedArcMap<ForwardMap,
        const BackwardMap>(forward, backward);
    }

    template <typename ForwardMap, typename BackwardMap>
    static CombinedArcMap<const ForwardMap, const BackwardMap>
    combinedArcMap(const ForwardMap& forward, const BackwardMap& backward) {
      return CombinedArcMap<const ForwardMap,
        const BackwardMap>(forward, backward);
    }

  };

  /// \brief Just gives back an undirected view of the given digraph
  ///
  /// Just gives back an undirected view of the given digraph
  template<typename Digraph>
  Undirector<const Digraph>
  undirector(const Digraph& digraph) {
    return Undirector<const Digraph>(digraph);
  }

  template <typename _Graph, typename _DirectionMap>
  class OrienterBase {
  public:

    typedef _Graph Graph;
    typedef _DirectionMap DirectionMap;

    typedef typename Graph::Node Node;
    typedef typename Graph::Edge Arc;

    void reverseArc(const Arc& arc) {
      _direction->set(arc, !(*_direction)[arc]);
    }

    void first(Node& i) const { _graph->first(i); }
    void first(Arc& i) const { _graph->first(i); }
    void firstIn(Arc& i, const Node& n) const {
      bool d = true;
      _graph->firstInc(i, d, n);
      while (i != INVALID && d == (*_direction)[i]) _graph->nextInc(i, d);
    }
    void firstOut(Arc& i, const Node& n ) const {
      bool d = true;
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

    typedef EdgeNumTagIndicator<Graph> ArcNumTag;
    int arcNum() const { return _graph->edgeNum(); }

    typedef FindEdgeTagIndicator<Graph> FindArcTag;
    Arc findArc(const Node& u, const Node& v,
                const Arc& prev = INVALID) const {
      Arc arc = _graph->findEdge(u, v, prev);
      while (arc != INVALID && source(arc) != u) {
        arc = _graph->findEdge(u, v, arc);
      }
      return arc;
    }

    Node addNode() {
      return Node(_graph->addNode());
    }

    Arc addArc(const Node& u, const Node& v) {
      Arc arc = _graph->addEdge(u, v);
      _direction->set(arc, _graph->u(arc) == u);
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

      explicit NodeMap(const OrienterBase& adapter)
        : Parent(*adapter._graph) {}

      NodeMap(const OrienterBase& adapter, const _Value& value)
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

      explicit ArcMap(const OrienterBase& adapter)
        : Parent(*adapter._graph) { }

      ArcMap(const OrienterBase& adapter, const _Value& value)
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
  /// \brief Orients the edges of the graph to get a digraph
  ///
  /// This adaptor orients each edge in the undirected graph. The
  /// direction of the arcs stored in an edge node map.  The arcs can
  /// be easily reverted by the \c reverseArc() member function in the
  /// adaptor. The Orienter adaptor is conform to the \ref
  /// concepts::Digraph "Digraph concept".
  ///
  /// \tparam _Graph It must be conform to the \ref concepts::Graph
  /// "Graph concept". The type can be specified to be const.
  /// \tparam _DirectionMap A bool valued edge map of the the adapted
  /// graph.
  ///
  /// \sa orienter
  template<typename _Graph,
           typename DirectionMap = typename _Graph::template EdgeMap<bool> >
  class Orienter :
    public DigraphAdaptorExtender<OrienterBase<_Graph, DirectionMap> > {
  public:
    typedef _Graph Graph;
    typedef DigraphAdaptorExtender<
      OrienterBase<_Graph, DirectionMap> > Parent;
    typedef typename Parent::Arc Arc;
  protected:
    Orienter() { }
  public:

    /// \brief Constructor of the adaptor
    ///
    /// Constructor of the adaptor
    Orienter(Graph& graph, DirectionMap& direction) {
      setGraph(graph);
      setDirectionMap(direction);
    }

    /// \brief Reverse arc
    ///
    /// It reverse the given arc. It simply negate the direction in the map.
    void reverseArc(const Arc& a) {
      Parent::reverseArc(a);
    }
  };

  /// \brief Just gives back a Orienter
  ///
  /// Just gives back a Orienter
  template<typename Graph, typename DirectionMap>
  Orienter<const Graph, DirectionMap>
  orienter(const Graph& graph, DirectionMap& dm) {
    return Orienter<const Graph, DirectionMap>(graph, dm);
  }

  template<typename Graph, typename DirectionMap>
  Orienter<const Graph, const DirectionMap>
  orienter(const Graph& graph, const DirectionMap& dm) {
    return Orienter<const Graph, const DirectionMap>(graph, dm);
  }

  namespace _adaptor_bits {

    template<typename _Digraph,
             typename _CapacityMap = typename _Digraph::template ArcMap<int>,
             typename _FlowMap = _CapacityMap,
             typename _Tolerance = Tolerance<typename _CapacityMap::Value> >
    class ResForwardFilter {
    public:

      typedef _Digraph Digraph;
      typedef _CapacityMap CapacityMap;
      typedef _FlowMap FlowMap;
      typedef _Tolerance Tolerance;

      typedef typename Digraph::Arc Key;
      typedef bool Value;

    private:

      const CapacityMap* _capacity;
      const FlowMap* _flow;
      Tolerance _tolerance;
    public:

      ResForwardFilter(const CapacityMap& capacity, const FlowMap& flow,
                       const Tolerance& tolerance = Tolerance())
        : _capacity(&capacity), _flow(&flow), _tolerance(tolerance) { }

      bool operator[](const typename Digraph::Arc& a) const {
        return _tolerance.positive((*_capacity)[a] - (*_flow)[a]);
      }
    };

    template<typename _Digraph,
             typename _CapacityMap = typename _Digraph::template ArcMap<int>,
             typename _FlowMap = _CapacityMap,
             typename _Tolerance = Tolerance<typename _CapacityMap::Value> >
    class ResBackwardFilter {
    public:

      typedef _Digraph Digraph;
      typedef _CapacityMap CapacityMap;
      typedef _FlowMap FlowMap;
      typedef _Tolerance Tolerance;

      typedef typename Digraph::Arc Key;
      typedef bool Value;

    private:

      const CapacityMap* _capacity;
      const FlowMap* _flow;
      Tolerance _tolerance;

    public:

      ResBackwardFilter(const CapacityMap& capacity, const FlowMap& flow,
                        const Tolerance& tolerance = Tolerance())
        : _capacity(&capacity), _flow(&flow), _tolerance(tolerance) { }

      bool operator[](const typename Digraph::Arc& a) const {
        return _tolerance.positive((*_flow)[a]);
      }
    };

  }

  /// \ingroup graph_adaptors
  ///
  /// \brief An adaptor for composing the residual graph for directed
  /// flow and circulation problems.
  ///
  /// An adaptor for composing the residual graph for directed flow and
  /// circulation problems.  Let \f$ G=(V, A) \f$ be a directed graph
  /// and let \f$ F \f$ be a number type. Let moreover \f$ f,c:A\to F \f$,
  /// be functions on the arc-set.
  ///
  /// Then Residual implements the digraph structure with
  /// node-set \f$ V \f$ and arc-set \f$ A_{forward}\cup A_{backward} \f$,
  /// where \f$ A_{forward}=\{uv : uv\in A, f(uv)<c(uv)\} \f$ and
  /// \f$ A_{backward}=\{vu : uv\in A, f(uv)>0\} \f$, i.e. the so
  /// called residual graph.  When we take the union
  /// \f$ A_{forward}\cup A_{backward} \f$, multiplicities are counted,
  /// i.e.  if an arc is in both \f$ A_{forward} \f$ and
  /// \f$ A_{backward} \f$, then in the adaptor it appears in both
  /// orientation.
  ///
  /// \tparam _Digraph It must be conform to the \ref concepts::Digraph
  /// "Digraph concept". The type is implicitly const.
  /// \tparam _CapacityMap An arc map of some numeric type, it defines
  /// the capacities in the flow problem. The map is implicitly const.
  /// \tparam _FlowMap An arc map of some numeric type, it defines
  /// the capacities in the flow problem.
  /// \tparam _Tolerance Handler for inexact computation.
  template<typename _Digraph,
           typename _CapacityMap = typename _Digraph::template ArcMap<int>,
           typename _FlowMap = _CapacityMap,
           typename _Tolerance = Tolerance<typename _CapacityMap::Value> >
  class Residual :
    public FilterArcs<
    Undirector<const _Digraph>,
    typename Undirector<const _Digraph>::template CombinedArcMap<
      _adaptor_bits::ResForwardFilter<const _Digraph, _CapacityMap,
                                      _FlowMap, _Tolerance>,
      _adaptor_bits::ResBackwardFilter<const _Digraph, _CapacityMap,
                                       _FlowMap, _Tolerance> > >
  {
  public:

    typedef _Digraph Digraph;
    typedef _CapacityMap CapacityMap;
    typedef _FlowMap FlowMap;
    typedef _Tolerance Tolerance;

    typedef typename CapacityMap::Value Value;
    typedef Residual Adaptor;

  protected:

    typedef Undirector<const Digraph> Undirected;

    typedef _adaptor_bits::ResForwardFilter<const Digraph, CapacityMap,
                                            FlowMap, Tolerance> ForwardFilter;

    typedef _adaptor_bits::ResBackwardFilter<const Digraph, CapacityMap,
                                             FlowMap, Tolerance> BackwardFilter;

    typedef typename Undirected::
    template CombinedArcMap<ForwardFilter, BackwardFilter> ArcFilter;

    typedef FilterArcs<Undirected, ArcFilter> Parent;

    const CapacityMap* _capacity;
    FlowMap* _flow;

    Undirected _graph;
    ForwardFilter _forward_filter;
    BackwardFilter _backward_filter;
    ArcFilter _arc_filter;

  public:

    /// \brief Constructor of the residual digraph.
    ///
    /// Constructor of the residual graph. The parameters are the digraph,
    /// the flow map, the capacity map and a tolerance object.
    Residual(const Digraph& digraph, const CapacityMap& capacity,
             FlowMap& flow, const Tolerance& tolerance = Tolerance())
      : Parent(), _capacity(&capacity), _flow(&flow), _graph(digraph),
        _forward_filter(capacity, flow, tolerance),
        _backward_filter(capacity, flow, tolerance),
        _arc_filter(_forward_filter, _backward_filter)
    {
      Parent::setDigraph(_graph);
      Parent::setArcFilterMap(_arc_filter);
    }

    typedef typename Parent::Arc Arc;

    /// \brief Gives back the residual capacity of the arc.
    ///
    /// Gives back the residual capacity of the arc.
    Value residualCapacity(const Arc& a) const {
      if (Undirected::direction(a)) {
        return (*_capacity)[a] - (*_flow)[a];
      } else {
        return (*_flow)[a];
      }
    }

    /// \brief Augment on the given arc in the residual graph.
    ///
    /// Augment on the given arc in the residual graph. It increase
    /// or decrease the flow on the original arc depend on the direction
    /// of the residual arc.
    void augment(const Arc& a, const Value& v) const {
      if (Undirected::direction(a)) {
        _flow->set(a, (*_flow)[a] + v);
      } else {
        _flow->set(a, (*_flow)[a] - v);
      }
    }

    /// \brief Returns the direction of the arc.
    ///
    /// Returns true when the arc is same oriented as the original arc.
    static bool forward(const Arc& a) {
      return Undirected::direction(a);
    }

    /// \brief Returns the direction of the arc.
    ///
    /// Returns true when the arc is opposite oriented as the original arc.
    static bool backward(const Arc& a) {
      return !Undirected::direction(a);
    }

    /// \brief Gives back the forward oriented residual arc.
    ///
    /// Gives back the forward oriented residual arc.
    static Arc forward(const typename Digraph::Arc& a) {
      return Undirected::direct(a, true);
    }

    /// \brief Gives back the backward oriented residual arc.
    ///
    /// Gives back the backward oriented residual arc.
    static Arc backward(const typename Digraph::Arc& a) {
      return Undirected::direct(a, false);
    }

    /// \brief Residual capacity map.
    ///
    /// In generic residual graph the residual capacity can be obtained
    /// as a map.
    class ResidualCapacity {
    protected:
      const Adaptor* _adaptor;
    public:
      /// The Key type
      typedef Arc Key;
      /// The Value type
      typedef typename _CapacityMap::Value Value;

      /// Constructor
      ResidualCapacity(const Adaptor& adaptor) : _adaptor(&adaptor) {}

      /// \e
      Value operator[](const Arc& a) const {
        return _adaptor->residualCapacity(a);
      }

    };

  };

  template <typename _Digraph>
  class SplitNodesBase {
  public:

    typedef _Digraph Digraph;
    typedef DigraphAdaptorBase<const _Digraph> Parent;
    typedef SplitNodesBase Adaptor;

    typedef typename Digraph::Node DigraphNode;
    typedef typename Digraph::Arc DigraphArc;

    class Node;
    class Arc;

  private:

    template <typename T> class NodeMapBase;
    template <typename T> class ArcMapBase;

  public:

    class Node : public DigraphNode {
      friend class SplitNodesBase;
      template <typename T> friend class NodeMapBase;
    private:

      bool _in;
      Node(DigraphNode node, bool in)
        : DigraphNode(node), _in(in) {}

    public:

      Node() {}
      Node(Invalid) : DigraphNode(INVALID), _in(true) {}

      bool operator==(const Node& node) const {
        return DigraphNode::operator==(node) && _in == node._in;
      }

      bool operator!=(const Node& node) const {
        return !(*this == node);
      }

      bool operator<(const Node& node) const {
        return DigraphNode::operator<(node) ||
          (DigraphNode::operator==(node) && _in < node._in);
      }
    };

    class Arc {
      friend class SplitNodesBase;
      template <typename T> friend class ArcMapBase;
    private:
      typedef BiVariant<DigraphArc, DigraphNode> ArcImpl;

      explicit Arc(const DigraphArc& arc) : _item(arc) {}
      explicit Arc(const DigraphNode& node) : _item(node) {}

      ArcImpl _item;

    public:
      Arc() {}
      Arc(Invalid) : _item(DigraphArc(INVALID)) {}

      bool operator==(const Arc& arc) const {
        if (_item.firstState()) {
          if (arc._item.firstState()) {
            return _item.first() == arc._item.first();
          }
        } else {
          if (arc._item.secondState()) {
            return _item.second() == arc._item.second();
          }
        }
        return false;
      }

      bool operator!=(const Arc& arc) const {
        return !(*this == arc);
      }

      bool operator<(const Arc& arc) const {
        if (_item.firstState()) {
          if (arc._item.firstState()) {
            return _item.first() < arc._item.first();
          }
          return false;
        } else {
          if (arc._item.secondState()) {
            return _item.second() < arc._item.second();
          }
          return true;
        }
      }

      operator DigraphArc() const { return _item.first(); }
      operator DigraphNode() const { return _item.second(); }

    };

    void first(Node& n) const {
      _digraph->first(n);
      n._in = true;
    }

    void next(Node& n) const {
      if (n._in) {
        n._in = false;
      } else {
        n._in = true;
        _digraph->next(n);
      }
    }

    void first(Arc& e) const {
      e._item.setSecond();
      _digraph->first(e._item.second());
      if (e._item.second() == INVALID) {
        e._item.setFirst();
        _digraph->first(e._item.first());
      }
    }

    void next(Arc& e) const {
      if (e._item.secondState()) {
        _digraph->next(e._item.second());
        if (e._item.second() == INVALID) {
          e._item.setFirst();
          _digraph->first(e._item.first());
        }
      } else {
        _digraph->next(e._item.first());
      }
    }

    void firstOut(Arc& e, const Node& n) const {
      if (n._in) {
        e._item.setSecond(n);
      } else {
        e._item.setFirst();
        _digraph->firstOut(e._item.first(), n);
      }
    }

    void nextOut(Arc& e) const {
      if (!e._item.firstState()) {
        e._item.setFirst(INVALID);
      } else {
        _digraph->nextOut(e._item.first());
      }
    }

    void firstIn(Arc& e, const Node& n) const {
      if (!n._in) {
        e._item.setSecond(n);
      } else {
        e._item.setFirst();
        _digraph->firstIn(e._item.first(), n);
      }
    }

    void nextIn(Arc& e) const {
      if (!e._item.firstState()) {
        e._item.setFirst(INVALID);
      } else {
        _digraph->nextIn(e._item.first());
      }
    }

    Node source(const Arc& e) const {
      if (e._item.firstState()) {
        return Node(_digraph->source(e._item.first()), false);
      } else {
        return Node(e._item.second(), true);
      }
    }

    Node target(const Arc& e) const {
      if (e._item.firstState()) {
        return Node(_digraph->target(e._item.first()), true);
      } else {
        return Node(e._item.second(), false);
      }
    }

    int id(const Node& n) const {
      return (_digraph->id(n) << 1) | (n._in ? 0 : 1);
    }
    Node nodeFromId(int ix) const {
      return Node(_digraph->nodeFromId(ix >> 1), (ix & 1) == 0);
    }
    int maxNodeId() const {
      return 2 * _digraph->maxNodeId() + 1;
    }

    int id(const Arc& e) const {
      if (e._item.firstState()) {
        return _digraph->id(e._item.first()) << 1;
      } else {
        return (_digraph->id(e._item.second()) << 1) | 1;
      }
    }
    Arc arcFromId(int ix) const {
      if ((ix & 1) == 0) {
        return Arc(_digraph->arcFromId(ix >> 1));
      } else {
        return Arc(_digraph->nodeFromId(ix >> 1));
      }
    }
    int maxArcId() const {
      return std::max(_digraph->maxNodeId() << 1,
                      (_digraph->maxArcId() << 1) | 1);
    }

    static bool inNode(const Node& n) {
      return n._in;
    }

    static bool outNode(const Node& n) {
      return !n._in;
    }

    static bool origArc(const Arc& e) {
      return e._item.firstState();
    }

    static bool bindArc(const Arc& e) {
      return e._item.secondState();
    }

    static Node inNode(const DigraphNode& n) {
      return Node(n, true);
    }

    static Node outNode(const DigraphNode& n) {
      return Node(n, false);
    }

    static Arc arc(const DigraphNode& n) {
      return Arc(n);
    }

    static Arc arc(const DigraphArc& e) {
      return Arc(e);
    }

    typedef True NodeNumTag;
    int nodeNum() const {
      return  2 * countNodes(*_digraph);
    }

    typedef True ArcNumTag;
    int arcNum() const {
      return countArcs(*_digraph) + countNodes(*_digraph);
    }

    typedef True FindArcTag;
    Arc findArc(const Node& u, const Node& v,
                const Arc& prev = INVALID) const {
      if (inNode(u) && outNode(v)) {
        if (static_cast<const DigraphNode&>(u) ==
            static_cast<const DigraphNode&>(v) && prev == INVALID) {
          return Arc(u);
        }
      }
      else if (outNode(u) && inNode(v)) {
        return Arc(::lemon::findArc(*_digraph, u, v, prev));
      }
      return INVALID;
    }

  private:

    template <typename _Value>
    class NodeMapBase
      : public MapTraits<typename Parent::template NodeMap<_Value> > {
      typedef typename Parent::template NodeMap<_Value> NodeImpl;
    public:
      typedef Node Key;
      typedef _Value Value;
      typedef typename MapTraits<NodeImpl>::ReferenceMapTag ReferenceMapTag;
      typedef typename MapTraits<NodeImpl>::ReturnValue ReturnValue;
      typedef typename MapTraits<NodeImpl>::ConstReturnValue ConstReturnValue;
      typedef typename MapTraits<NodeImpl>::ReturnValue Reference;
      typedef typename MapTraits<NodeImpl>::ConstReturnValue ConstReference;

      NodeMapBase(const Adaptor& adaptor)
        : _in_map(*adaptor._digraph), _out_map(*adaptor._digraph) {}
      NodeMapBase(const Adaptor& adaptor, const Value& value)
        : _in_map(*adaptor._digraph, value),
          _out_map(*adaptor._digraph, value) {}

      void set(const Node& key, const Value& val) {
        if (Adaptor::inNode(key)) { _in_map.set(key, val); }
        else {_out_map.set(key, val); }
      }

      ReturnValue operator[](const Node& key) {
        if (Adaptor::inNode(key)) { return _in_map[key]; }
        else { return _out_map[key]; }
      }

      ConstReturnValue operator[](const Node& key) const {
        if (Adaptor::inNode(key)) { return _in_map[key]; }
        else { return _out_map[key]; }
      }

    private:
      NodeImpl _in_map, _out_map;
    };

    template <typename _Value>
    class ArcMapBase
      : public MapTraits<typename Parent::template ArcMap<_Value> > {
      typedef typename Parent::template ArcMap<_Value> ArcImpl;
      typedef typename Parent::template NodeMap<_Value> NodeImpl;
    public:
      typedef Arc Key;
      typedef _Value Value;
      typedef typename MapTraits<ArcImpl>::ReferenceMapTag ReferenceMapTag;
      typedef typename MapTraits<ArcImpl>::ReturnValue ReturnValue;
      typedef typename MapTraits<ArcImpl>::ConstReturnValue ConstReturnValue;
      typedef typename MapTraits<ArcImpl>::ReturnValue Reference;
      typedef typename MapTraits<ArcImpl>::ConstReturnValue ConstReference;

      ArcMapBase(const Adaptor& adaptor)
        : _arc_map(*adaptor._digraph), _node_map(*adaptor._digraph) {}
      ArcMapBase(const Adaptor& adaptor, const Value& value)
        : _arc_map(*adaptor._digraph, value),
          _node_map(*adaptor._digraph, value) {}

      void set(const Arc& key, const Value& val) {
        if (Adaptor::origArc(key)) {
          _arc_map.set(key._item.first(), val);
        } else {
          _node_map.set(key._item.second(), val);
        }
      }

      ReturnValue operator[](const Arc& key) {
        if (Adaptor::origArc(key)) {
          return _arc_map[key._item.first()];
        } else {
          return _node_map[key._item.second()];
        }
      }

      ConstReturnValue operator[](const Arc& key) const {
        if (Adaptor::origArc(key)) {
          return _arc_map[key._item.first()];
        } else {
          return _node_map[key._item.second()];
        }
      }

    private:
      ArcImpl _arc_map;
      NodeImpl _node_map;
    };

  public:

    template <typename _Value>
    class NodeMap
      : public SubMapExtender<Adaptor, NodeMapBase<_Value> >
    {
    public:
      typedef _Value Value;
      typedef SubMapExtender<Adaptor, NodeMapBase<Value> > Parent;

      NodeMap(const Adaptor& adaptor)
        : Parent(adaptor) {}

      NodeMap(const Adaptor& adaptor, const Value& value)
        : Parent(adaptor, value) {}

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
    class ArcMap
      : public SubMapExtender<Adaptor, ArcMapBase<_Value> >
    {
    public:
      typedef _Value Value;
      typedef SubMapExtender<Adaptor, ArcMapBase<Value> > Parent;

      ArcMap(const Adaptor& adaptor)
        : Parent(adaptor) {}

      ArcMap(const Adaptor& adaptor, const Value& value)
        : Parent(adaptor, value) {}

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

    SplitNodesBase() : _digraph(0) {}

    Digraph* _digraph;

    void setDigraph(Digraph& digraph) {
      _digraph = &digraph;
    }

  };

  /// \ingroup graph_adaptors
  ///
  /// \brief Split the nodes of a directed graph
  ///
  /// The SplitNodes adaptor splits each node into an in-node and an
  /// out-node. Formaly, the adaptor replaces each \f$ u \f$ node in
  /// the digraph with two nodes(namely node \f$ u_{in} \f$ and node
  /// \f$ u_{out} \f$). If there is a \f$ (v, u) \f$ arc in the
  /// original digraph the new target of the arc will be \f$ u_{in} \f$
  /// and similarly the source of the original \f$ (u, v) \f$ arc
  /// will be \f$ u_{out} \f$.  The adaptor will add for each node in
  /// the original digraph an additional arc which connects
  /// \f$ (u_{in}, u_{out}) \f$.
  ///
  /// The aim of this class is to run algorithm with node costs if the
  /// algorithm can use directly just arc costs. In this case we should use
  /// a \c SplitNodes and set the node cost of the graph to the
  /// bind arc in the adapted graph.
  ///
  /// \tparam _Digraph It must be conform to the \ref concepts::Digraph
  /// "Digraph concept". The type can be specified to be const.
  template <typename _Digraph>
  class SplitNodes
    : public DigraphAdaptorExtender<SplitNodesBase<const _Digraph> > {
  public:
    typedef _Digraph Digraph;
    typedef DigraphAdaptorExtender<SplitNodesBase<const Digraph> > Parent;

    typedef typename Digraph::Node DigraphNode;
    typedef typename Digraph::Arc DigraphArc;

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;

    /// \brief Constructor of the adaptor.
    ///
    /// Constructor of the adaptor.
    SplitNodes(const Digraph& g) {
      Parent::setDigraph(g);
    }

    /// \brief Returns true when the node is in-node.
    ///
    /// Returns true when the node is in-node.
    static bool inNode(const Node& n) {
      return Parent::inNode(n);
    }

    /// \brief Returns true when the node is out-node.
    ///
    /// Returns true when the node is out-node.
    static bool outNode(const Node& n) {
      return Parent::outNode(n);
    }

    /// \brief Returns true when the arc is arc in the original digraph.
    ///
    /// Returns true when the arc is arc in the original digraph.
    static bool origArc(const Arc& a) {
      return Parent::origArc(a);
    }

    /// \brief Returns true when the arc binds an in-node and an out-node.
    ///
    /// Returns true when the arc binds an in-node and an out-node.
    static bool bindArc(const Arc& a) {
      return Parent::bindArc(a);
    }

    /// \brief Gives back the in-node created from the \c node.
    ///
    /// Gives back the in-node created from the \c node.
    static Node inNode(const DigraphNode& n) {
      return Parent::inNode(n);
    }

    /// \brief Gives back the out-node created from the \c node.
    ///
    /// Gives back the out-node created from the \c node.
    static Node outNode(const DigraphNode& n) {
      return Parent::outNode(n);
    }

    /// \brief Gives back the arc binds the two part of the node.
    ///
    /// Gives back the arc binds the two part of the node.
    static Arc arc(const DigraphNode& n) {
      return Parent::arc(n);
    }

    /// \brief Gives back the arc of the original arc.
    ///
    /// Gives back the arc of the original arc.
    static Arc arc(const DigraphArc& a) {
      return Parent::arc(a);
    }

    /// \brief NodeMap combined from two original NodeMap
    ///
    /// This class adapt two of the original digraph NodeMap to
    /// get a node map on the adapted digraph.
    template <typename InNodeMap, typename OutNodeMap>
    class CombinedNodeMap {
    public:

      typedef Node Key;
      typedef typename InNodeMap::Value Value;

      typedef typename MapTraits<InNodeMap>::ReferenceMapTag ReferenceMapTag;
      typedef typename MapTraits<InNodeMap>::ReturnValue ReturnValue;
      typedef typename MapTraits<InNodeMap>::ConstReturnValue ConstReturnValue;
      typedef typename MapTraits<InNodeMap>::ReturnValue Reference;
      typedef typename MapTraits<InNodeMap>::ConstReturnValue ConstReference;

      /// \brief Constructor
      ///
      /// Constructor.
      CombinedNodeMap(InNodeMap& in_map, OutNodeMap& out_map)
        : _in_map(in_map), _out_map(out_map) {}

      /// \brief The subscript operator.
      ///
      /// The subscript operator.
      Value& operator[](const Key& key) {
        if (Parent::inNode(key)) {
          return _in_map[key];
        } else {
          return _out_map[key];
        }
      }

      /// \brief The const subscript operator.
      ///
      /// The const subscript operator.
      Value operator[](const Key& key) const {
        if (Parent::inNode(key)) {
          return _in_map[key];
        } else {
          return _out_map[key];
        }
      }

      /// \brief The setter function of the map.
      ///
      /// The setter function of the map.
      void set(const Key& key, const Value& value) {
        if (Parent::inNode(key)) {
          _in_map.set(key, value);
        } else {
          _out_map.set(key, value);
        }
      }

    private:

      InNodeMap& _in_map;
      OutNodeMap& _out_map;

    };


    /// \brief Just gives back a combined node map
    ///
    /// Just gives back a combined node map
    template <typename InNodeMap, typename OutNodeMap>
    static CombinedNodeMap<InNodeMap, OutNodeMap>
    combinedNodeMap(InNodeMap& in_map, OutNodeMap& out_map) {
      return CombinedNodeMap<InNodeMap, OutNodeMap>(in_map, out_map);
    }

    template <typename InNodeMap, typename OutNodeMap>
    static CombinedNodeMap<const InNodeMap, OutNodeMap>
    combinedNodeMap(const InNodeMap& in_map, OutNodeMap& out_map) {
      return CombinedNodeMap<const InNodeMap, OutNodeMap>(in_map, out_map);
    }

    template <typename InNodeMap, typename OutNodeMap>
    static CombinedNodeMap<InNodeMap, const OutNodeMap>
    combinedNodeMap(InNodeMap& in_map, const OutNodeMap& out_map) {
      return CombinedNodeMap<InNodeMap, const OutNodeMap>(in_map, out_map);
    }

    template <typename InNodeMap, typename OutNodeMap>
    static CombinedNodeMap<const InNodeMap, const OutNodeMap>
    combinedNodeMap(const InNodeMap& in_map, const OutNodeMap& out_map) {
      return CombinedNodeMap<const InNodeMap,
        const OutNodeMap>(in_map, out_map);
    }

    /// \brief ArcMap combined from an original ArcMap and a NodeMap
    ///
    /// This class adapt an original ArcMap and a NodeMap to get an
    /// arc map on the adapted digraph
    template <typename DigraphArcMap, typename DigraphNodeMap>
    class CombinedArcMap {
    public:

      typedef Arc Key;
      typedef typename DigraphArcMap::Value Value;

      typedef typename MapTraits<DigraphArcMap>::ReferenceMapTag
        ReferenceMapTag;
      typedef typename MapTraits<DigraphArcMap>::ReturnValue
        ReturnValue;
      typedef typename MapTraits<DigraphArcMap>::ConstReturnValue
        ConstReturnValue;
      typedef typename MapTraits<DigraphArcMap>::ReturnValue
        Reference;
      typedef typename MapTraits<DigraphArcMap>::ConstReturnValue
        ConstReference;

      /// \brief Constructor
      ///
      /// Constructor.
      CombinedArcMap(DigraphArcMap& arc_map, DigraphNodeMap& node_map)
        : _arc_map(arc_map), _node_map(node_map) {}

      /// \brief The subscript operator.
      ///
      /// The subscript operator.
      void set(const Arc& arc, const Value& val) {
        if (Parent::origArc(arc)) {
          _arc_map.set(arc, val);
        } else {
          _node_map.set(arc, val);
        }
      }

      /// \brief The const subscript operator.
      ///
      /// The const subscript operator.
      Value operator[](const Key& arc) const {
        if (Parent::origArc(arc)) {
          return _arc_map[arc];
        } else {
          return _node_map[arc];
        }
      }

      /// \brief The const subscript operator.
      ///
      /// The const subscript operator.
      Value& operator[](const Key& arc) {
        if (Parent::origArc(arc)) {
          return _arc_map[arc];
        } else {
          return _node_map[arc];
        }
      }

    private:
      DigraphArcMap& _arc_map;
      DigraphNodeMap& _node_map;
    };

    /// \brief Just gives back a combined arc map
    ///
    /// Just gives back a combined arc map
    template <typename DigraphArcMap, typename DigraphNodeMap>
    static CombinedArcMap<DigraphArcMap, DigraphNodeMap>
    combinedArcMap(DigraphArcMap& arc_map, DigraphNodeMap& node_map) {
      return CombinedArcMap<DigraphArcMap, DigraphNodeMap>(arc_map, node_map);
    }

    template <typename DigraphArcMap, typename DigraphNodeMap>
    static CombinedArcMap<const DigraphArcMap, DigraphNodeMap>
    combinedArcMap(const DigraphArcMap& arc_map, DigraphNodeMap& node_map) {
      return CombinedArcMap<const DigraphArcMap,
        DigraphNodeMap>(arc_map, node_map);
    }

    template <typename DigraphArcMap, typename DigraphNodeMap>
    static CombinedArcMap<DigraphArcMap, const DigraphNodeMap>
    combinedArcMap(DigraphArcMap& arc_map, const DigraphNodeMap& node_map) {
      return CombinedArcMap<DigraphArcMap,
        const DigraphNodeMap>(arc_map, node_map);
    }

    template <typename DigraphArcMap, typename DigraphNodeMap>
    static CombinedArcMap<const DigraphArcMap, const DigraphNodeMap>
    combinedArcMap(const DigraphArcMap& arc_map,
                   const DigraphNodeMap& node_map) {
      return CombinedArcMap<const DigraphArcMap,
        const DigraphNodeMap>(arc_map, node_map);
    }

  };

  /// \brief Just gives back a node splitter
  ///
  /// Just gives back a node splitter
  template<typename Digraph>
  SplitNodes<Digraph>
  splitNodes(const Digraph& digraph) {
    return SplitNodes<Digraph>(digraph);
  }


} //namespace lemon

#endif //LEMON_ADAPTORS_H
