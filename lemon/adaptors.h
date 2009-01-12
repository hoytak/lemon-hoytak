/* -*- mode: C++; indent-tabs-mode: nil; -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library.
 *
 * Copyright (C) 2003-2009
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
/// \brief Adaptor classes for digraphs and graphs
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
  /// \brief Adaptor class for reversing the orientation of the arcs in
  /// a digraph.
  ///
  /// ReverseDigraph can be used for reversing the arcs in a digraph.
  /// It conforms to the \ref concepts::Digraph "Digraph" concept.
  ///
  /// The adapted digraph can also be modified through this adaptor
  /// by adding or removing nodes or arcs, unless the \c GR template
  /// parameter is set to be \c const.
  ///
  /// \tparam GR The type of the adapted digraph.
  /// It must conform to the \ref concepts::Digraph "Digraph" concept.
  /// It can also be specified to be \c const.
  ///
  /// \note The \c Node and \c Arc types of this adaptor and the adapted
  /// digraph are convertible to each other.
  template<typename GR>
#ifdef DOXYGEN
  class ReverseDigraph {
#else
  class ReverseDigraph :
    public DigraphAdaptorExtender<ReverseDigraphBase<GR> > {
#endif
  public:
    /// The type of the adapted digraph.
    typedef GR Digraph;
    typedef DigraphAdaptorExtender<ReverseDigraphBase<GR> > Parent;
  protected:
    ReverseDigraph() { }
  public:

    /// \brief Constructor
    ///
    /// Creates a reverse digraph adaptor for the given digraph.
    explicit ReverseDigraph(Digraph& digraph) {
      Parent::setDigraph(digraph);
    }
  };

  /// \brief Returns a read-only ReverseDigraph adaptor
  ///
  /// This function just returns a read-only \ref ReverseDigraph adaptor.
  /// \ingroup graph_adaptors
  /// \relates ReverseDigraph
  template<typename GR>
  ReverseDigraph<const GR> reverseDigraph(const GR& digraph) {
    return ReverseDigraph<const GR>(digraph);
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

    void status(const Node& n, bool v) const { _node_filter->set(n, v); }
    void status(const Arc& a, bool v) const { _arc_filter->set(a, v); }

    bool status(const Node& n) const { return (*_node_filter)[n]; }
    bool status(const Arc& a) const { return (*_arc_filter)[a]; }

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

    void status(const Node& n, bool v) const { _node_filter->set(n, v); }
    void status(const Arc& a, bool v) const { _arc_filter->set(a, v); }

    bool status(const Node& n) const { return (*_node_filter)[n]; }
    bool status(const Arc& a) const { return (*_arc_filter)[a]; }

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
  /// \brief Adaptor class for hiding nodes and arcs in a digraph
  ///
  /// SubDigraph can be used for hiding nodes and arcs in a digraph.
  /// A \c bool node map and a \c bool arc map must be specified, which
  /// define the filters for nodes and arcs.
  /// Only the nodes and arcs with \c true filter value are
  /// shown in the subdigraph. The arcs that are incident to hidden
  /// nodes are also filtered out.
  /// This adaptor conforms to the \ref concepts::Digraph "Digraph" concept.
  ///
  /// The adapted digraph can also be modified through this adaptor
  /// by adding or removing nodes or arcs, unless the \c GR template
  /// parameter is set to be \c const.
  ///
  /// \tparam GR The type of the adapted digraph.
  /// It must conform to the \ref concepts::Digraph "Digraph" concept.
  /// It can also be specified to be \c const.
  /// \tparam NF The type of the node filter map.
  /// It must be a \c bool (or convertible) node map of the
  /// adapted digraph. The default type is
  /// \ref concepts::Digraph::NodeMap "GR::NodeMap<bool>".
  /// \tparam AF The type of the arc filter map.
  /// It must be \c bool (or convertible) arc map of the
  /// adapted digraph. The default type is
  /// \ref concepts::Digraph::ArcMap "GR::ArcMap<bool>".
  ///
  /// \note The \c Node and \c Arc types of this adaptor and the adapted
  /// digraph are convertible to each other.
  ///
  /// \see FilterNodes
  /// \see FilterArcs
#ifdef DOXYGEN
  template<typename GR, typename NF, typename AF>
  class SubDigraph {
#else
  template<typename GR,
           typename NF = typename GR::template NodeMap<bool>,
           typename AF = typename GR::template ArcMap<bool> >
  class SubDigraph :
    public DigraphAdaptorExtender<SubDigraphBase<GR, NF, AF, true> > {
#endif
  public:
    /// The type of the adapted digraph.
    typedef GR Digraph;
    /// The type of the node filter map.
    typedef NF NodeFilterMap;
    /// The type of the arc filter map.
    typedef AF ArcFilterMap;

    typedef DigraphAdaptorExtender<SubDigraphBase<GR, NF, AF, true> >
      Parent;

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;

  protected:
    SubDigraph() { }
  public:

    /// \brief Constructor
    ///
    /// Creates a subdigraph for the given digraph with the
    /// given node and arc filter maps.
    SubDigraph(Digraph& digraph, NodeFilterMap& node_filter,
               ArcFilterMap& arc_filter) {
      setDigraph(digraph);
      setNodeFilterMap(node_filter);
      setArcFilterMap(arc_filter);
    }

    /// \brief Sets the status of the given node
    ///
    /// This function sets the status of the given node.
    /// It is done by simply setting the assigned value of \c n
    /// to \c v in the node filter map.
    void status(const Node& n, bool v) const { Parent::status(n, v); }

    /// \brief Sets the status of the given arc
    ///
    /// This function sets the status of the given arc.
    /// It is done by simply setting the assigned value of \c a
    /// to \c v in the arc filter map.
    void status(const Arc& a, bool v) const { Parent::status(a, v); }

    /// \brief Returns the status of the given node
    ///
    /// This function returns the status of the given node.
    /// It is \c true if the given node is enabled (i.e. not hidden).
    bool status(const Node& n) const { return Parent::status(n); }

    /// \brief Returns the status of the given arc
    ///
    /// This function returns the status of the given arc.
    /// It is \c true if the given arc is enabled (i.e. not hidden).
    bool status(const Arc& a) const { return Parent::status(a); }

    /// \brief Disables the given node
    ///
    /// This function disables the given node in the subdigraph,
    /// so the iteration jumps over it.
    /// It is the same as \ref status() "status(n, false)".
    void disable(const Node& n) const { Parent::status(n, false); }

    /// \brief Disables the given arc
    ///
    /// This function disables the given arc in the subdigraph,
    /// so the iteration jumps over it.
    /// It is the same as \ref status() "status(a, false)".
    void disable(const Arc& a) const { Parent::status(a, false); }

    /// \brief Enables the given node
    ///
    /// This function enables the given node in the subdigraph.
    /// It is the same as \ref status() "status(n, true)".
    void enable(const Node& n) const { Parent::status(n, true); }

    /// \brief Enables the given arc
    ///
    /// This function enables the given arc in the subdigraph.
    /// It is the same as \ref status() "status(a, true)".
    void enable(const Arc& a) const { Parent::status(a, true); }

  };

  /// \brief Returns a read-only SubDigraph adaptor
  ///
  /// This function just returns a read-only \ref SubDigraph adaptor.
  /// \ingroup graph_adaptors
  /// \relates SubDigraph
  template<typename GR, typename NF, typename AF>
  SubDigraph<const GR, NF, AF>
  subDigraph(const GR& digraph,
             NF& node_filter_map, AF& arc_filter_map) {
    return SubDigraph<const GR, NF, AF>
      (digraph, node_filter_map, arc_filter_map);
  }

  template<typename GR, typename NF, typename AF>
  SubDigraph<const GR, const NF, AF>
  subDigraph(const GR& digraph,
             const NF& node_filter_map, AF& arc_filter_map) {
    return SubDigraph<const GR, const NF, AF>
      (digraph, node_filter_map, arc_filter_map);
  }

  template<typename GR, typename NF, typename AF>
  SubDigraph<const GR, NF, const AF>
  subDigraph(const GR& digraph,
             NF& node_filter_map, const AF& arc_filter_map) {
    return SubDigraph<const GR, NF, const AF>
      (digraph, node_filter_map, arc_filter_map);
  }

  template<typename GR, typename NF, typename AF>
  SubDigraph<const GR, const NF, const AF>
  subDigraph(const GR& digraph,
             const NF& node_filter_map, const AF& arc_filter_map) {
    return SubDigraph<const GR, const NF, const AF>
      (digraph, node_filter_map, arc_filter_map);
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

    void status(const Node& n, bool v) const { _node_filter_map->set(n, v); }
    void status(const Edge& e, bool v) const { _edge_filter_map->set(e, v); }

    bool status(const Node& n) const { return (*_node_filter_map)[n]; }
    bool status(const Edge& e) const { return (*_edge_filter_map)[e]; }

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

    void status(const Node& n, bool v) const { _node_filter_map->set(n, v); }
    void status(const Edge& e, bool v) const { _edge_filter_map->set(e, v); }

    bool status(const Node& n) const { return (*_node_filter_map)[n]; }
    bool status(const Edge& e) const { return (*_edge_filter_map)[e]; }

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
  /// \brief Adaptor class for hiding nodes and edges in an undirected
  /// graph.
  ///
  /// SubGraph can be used for hiding nodes and edges in a graph.
  /// A \c bool node map and a \c bool edge map must be specified, which
  /// define the filters for nodes and edges.
  /// Only the nodes and edges with \c true filter value are
  /// shown in the subgraph. The edges that are incident to hidden
  /// nodes are also filtered out.
  /// This adaptor conforms to the \ref concepts::Graph "Graph" concept.
  ///
  /// The adapted graph can also be modified through this adaptor
  /// by adding or removing nodes or edges, unless the \c GR template
  /// parameter is set to be \c const.
  ///
  /// \tparam GR The type of the adapted graph.
  /// It must conform to the \ref concepts::Graph "Graph" concept.
  /// It can also be specified to be \c const.
  /// \tparam NF The type of the node filter map.
  /// It must be a \c bool (or convertible) node map of the
  /// adapted graph. The default type is
  /// \ref concepts::Graph::NodeMap "GR::NodeMap<bool>".
  /// \tparam EF The type of the edge filter map.
  /// It must be a \c bool (or convertible) edge map of the
  /// adapted graph. The default type is
  /// \ref concepts::Graph::EdgeMap "GR::EdgeMap<bool>".
  ///
  /// \note The \c Node, \c Edge and \c Arc types of this adaptor and the
  /// adapted graph are convertible to each other.
  ///
  /// \see FilterNodes
  /// \see FilterEdges
#ifdef DOXYGEN
  template<typename GR, typename NF, typename EF>
  class SubGraph {
#else
  template<typename GR,
           typename NF = typename GR::template NodeMap<bool>,
           typename EF = typename GR::template EdgeMap<bool> >
  class SubGraph :
    public GraphAdaptorExtender<SubGraphBase<GR, NF, EF, true> > {
#endif
  public:
    /// The type of the adapted graph.
    typedef GR Graph;
    /// The type of the node filter map.
    typedef NF NodeFilterMap;
    /// The type of the edge filter map.
    typedef EF EdgeFilterMap;

    typedef GraphAdaptorExtender< SubGraphBase<GR, NF, EF, true> >
      Parent;

    typedef typename Parent::Node Node;
    typedef typename Parent::Edge Edge;

  protected:
    SubGraph() { }
  public:

    /// \brief Constructor
    ///
    /// Creates a subgraph for the given graph with the given node
    /// and edge filter maps.
    SubGraph(Graph& graph, NodeFilterMap& node_filter_map,
             EdgeFilterMap& edge_filter_map) {
      setGraph(graph);
      setNodeFilterMap(node_filter_map);
      setEdgeFilterMap(edge_filter_map);
    }

    /// \brief Sets the status of the given node
    ///
    /// This function sets the status of the given node.
    /// It is done by simply setting the assigned value of \c n
    /// to \c v in the node filter map.
    void status(const Node& n, bool v) const { Parent::status(n, v); }

    /// \brief Sets the status of the given edge
    ///
    /// This function sets the status of the given edge.
    /// It is done by simply setting the assigned value of \c e
    /// to \c v in the edge filter map.
    void status(const Edge& e, bool v) const { Parent::status(e, v); }

    /// \brief Returns the status of the given node
    ///
    /// This function returns the status of the given node.
    /// It is \c true if the given node is enabled (i.e. not hidden).
    bool status(const Node& n) const { return Parent::status(n); }

    /// \brief Returns the status of the given edge
    ///
    /// This function returns the status of the given edge.
    /// It is \c true if the given edge is enabled (i.e. not hidden).
    bool status(const Edge& e) const { return Parent::status(e); }

    /// \brief Disables the given node
    ///
    /// This function disables the given node in the subdigraph,
    /// so the iteration jumps over it.
    /// It is the same as \ref status() "status(n, false)".
    void disable(const Node& n) const { Parent::status(n, false); }

    /// \brief Disables the given edge
    ///
    /// This function disables the given edge in the subgraph,
    /// so the iteration jumps over it.
    /// It is the same as \ref status() "status(e, false)".
    void disable(const Edge& e) const { Parent::status(e, false); }

    /// \brief Enables the given node
    ///
    /// This function enables the given node in the subdigraph.
    /// It is the same as \ref status() "status(n, true)".
    void enable(const Node& n) const { Parent::status(n, true); }

    /// \brief Enables the given edge
    ///
    /// This function enables the given edge in the subgraph.
    /// It is the same as \ref status() "status(e, true)".
    void enable(const Edge& e) const { Parent::status(e, true); }

  };

  /// \brief Returns a read-only SubGraph adaptor
  ///
  /// This function just returns a read-only \ref SubGraph adaptor.
  /// \ingroup graph_adaptors
  /// \relates SubGraph
  template<typename GR, typename NF, typename EF>
  SubGraph<const GR, NF, EF>
  subGraph(const GR& graph,
           NF& node_filter_map, EF& edge_filter_map) {
    return SubGraph<const GR, NF, EF>
      (graph, node_filter_map, edge_filter_map);
  }

  template<typename GR, typename NF, typename EF>
  SubGraph<const GR, const NF, EF>
  subGraph(const GR& graph,
           const NF& node_filter_map, EF& edge_filter_map) {
    return SubGraph<const GR, const NF, EF>
      (graph, node_filter_map, edge_filter_map);
  }

  template<typename GR, typename NF, typename EF>
  SubGraph<const GR, NF, const EF>
  subGraph(const GR& graph,
           NF& node_filter_map, const EF& edge_filter_map) {
    return SubGraph<const GR, NF, const EF>
      (graph, node_filter_map, edge_filter_map);
  }

  template<typename GR, typename NF, typename EF>
  SubGraph<const GR, const NF, const EF>
  subGraph(const GR& graph,
           const NF& node_filter_map, const EF& edge_filter_map) {
    return SubGraph<const GR, const NF, const EF>
      (graph, node_filter_map, edge_filter_map);
  }


  /// \ingroup graph_adaptors
  ///
  /// \brief Adaptor class for hiding nodes in a digraph or a graph.
  ///
  /// FilterNodes adaptor can be used for hiding nodes in a digraph or a
  /// graph. A \c bool node map must be specified, which defines the filter
  /// for the nodes. Only the nodes with \c true filter value and the
  /// arcs/edges incident to nodes both with \c true filter value are shown
  /// in the subgraph. This adaptor conforms to the \ref concepts::Digraph
  /// "Digraph" concept or the \ref concepts::Graph "Graph" concept
  /// depending on the \c GR template parameter.
  ///
  /// The adapted (di)graph can also be modified through this adaptor
  /// by adding or removing nodes or arcs/edges, unless the \c GR template
  /// parameter is set to be \c const.
  ///
  /// \tparam GR The type of the adapted digraph or graph.
  /// It must conform to the \ref concepts::Digraph "Digraph" concept
  /// or the \ref concepts::Graph "Graph" concept.
  /// It can also be specified to be \c const.
  /// \tparam NF The type of the node filter map.
  /// It must be a \c bool (or convertible) node map of the
  /// adapted (di)graph. The default type is
  /// \ref concepts::Graph::NodeMap "GR::NodeMap<bool>".
  ///
  /// \note The \c Node and <tt>Arc/Edge</tt> types of this adaptor and the
  /// adapted (di)graph are convertible to each other.
#ifdef DOXYGEN
  template<typename GR, typename NF>
  class FilterNodes {
#else
  template<typename GR,
           typename NF = typename GR::template NodeMap<bool>,
           typename Enable = void>
  class FilterNodes :
    public DigraphAdaptorExtender<
      SubDigraphBase<GR, NF, ConstMap<typename GR::Arc, bool>, true> > {
#endif
  public:

    typedef GR Digraph;
    typedef NF NodeFilterMap;

    typedef DigraphAdaptorExtender<
      SubDigraphBase<GR, NF, ConstMap<typename GR::Arc, bool>, true> >
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
    /// Creates a subgraph for the given digraph or graph with the
    /// given node filter map.
    FilterNodes(GR& graph, NodeFilterMap& node_filter) :
      Parent(), const_true_map(true)
    {
      Parent::setDigraph(graph);
      Parent::setNodeFilterMap(node_filter);
      Parent::setArcFilterMap(const_true_map);
    }

    /// \brief Sets the status of the given node
    ///
    /// This function sets the status of the given node.
    /// It is done by simply setting the assigned value of \c n
    /// to \c v in the node filter map.
    void status(const Node& n, bool v) const { Parent::status(n, v); }

    /// \brief Returns the status of the given node
    ///
    /// This function returns the status of the given node.
    /// It is \c true if the given node is enabled (i.e. not hidden).
    bool status(const Node& n) const { return Parent::status(n); }

    /// \brief Disables the given node
    ///
    /// This function disables the given node, so the iteration
    /// jumps over it.
    /// It is the same as \ref status() "status(n, false)".
    void disable(const Node& n) const { Parent::status(n, false); }

    /// \brief Enables the given node
    ///
    /// This function enables the given node.
    /// It is the same as \ref status() "status(n, true)".
    void enable(const Node& n) const { Parent::status(n, true); }

  };

  template<typename GR, typename NF>
  class FilterNodes<GR, NF,
                    typename enable_if<UndirectedTagIndicator<GR> >::type> :
    public GraphAdaptorExtender<
      SubGraphBase<GR, NF, ConstMap<typename GR::Edge, bool>, true> > {

  public:
    typedef GR Graph;
    typedef NF NodeFilterMap;
    typedef GraphAdaptorExtender<
      SubGraphBase<GR, NF, ConstMap<typename GR::Edge, bool>, true> >
      Parent;

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

    void status(const Node& n, bool v) const { Parent::status(n, v); }
    bool status(const Node& n) const { return Parent::status(n); }
    void disable(const Node& n) const { Parent::status(n, false); }
    void enable(const Node& n) const { Parent::status(n, true); }

  };


  /// \brief Returns a read-only FilterNodes adaptor
  ///
  /// This function just returns a read-only \ref FilterNodes adaptor.
  /// \ingroup graph_adaptors
  /// \relates FilterNodes
  template<typename GR, typename NF>
  FilterNodes<const GR, NF>
  filterNodes(const GR& graph, NF& node_filter_map) {
    return FilterNodes<const GR, NF>(graph, node_filter_map);
  }

  template<typename GR, typename NF>
  FilterNodes<const GR, const NF>
  filterNodes(const GR& graph, const NF& node_filter_map) {
    return FilterNodes<const GR, const NF>(graph, node_filter_map);
  }

  /// \ingroup graph_adaptors
  ///
  /// \brief Adaptor class for hiding arcs in a digraph.
  ///
  /// FilterArcs adaptor can be used for hiding arcs in a digraph.
  /// A \c bool arc map must be specified, which defines the filter for
  /// the arcs. Only the arcs with \c true filter value are shown in the
  /// subdigraph. This adaptor conforms to the \ref concepts::Digraph
  /// "Digraph" concept.
  ///
  /// The adapted digraph can also be modified through this adaptor
  /// by adding or removing nodes or arcs, unless the \c GR template
  /// parameter is set to be \c const.
  ///
  /// \tparam GR The type of the adapted digraph.
  /// It must conform to the \ref concepts::Digraph "Digraph" concept.
  /// It can also be specified to be \c const.
  /// \tparam AF The type of the arc filter map.
  /// It must be a \c bool (or convertible) arc map of the
  /// adapted digraph. The default type is
  /// \ref concepts::Digraph::ArcMap "GR::ArcMap<bool>".
  ///
  /// \note The \c Node and \c Arc types of this adaptor and the adapted
  /// digraph are convertible to each other.
#ifdef DOXYGEN
  template<typename GR,
           typename AF>
  class FilterArcs {
#else
  template<typename GR,
           typename AF = typename GR::template ArcMap<bool> >
  class FilterArcs :
    public DigraphAdaptorExtender<
      SubDigraphBase<GR, ConstMap<typename GR::Node, bool>, AF, false> > {
#endif
  public:
    /// The type of the adapted digraph.
    typedef GR Digraph;
    /// The type of the arc filter map.
    typedef AF ArcFilterMap;

    typedef DigraphAdaptorExtender<
      SubDigraphBase<GR, ConstMap<typename GR::Node, bool>, AF, false> >
      Parent;

    typedef typename Parent::Arc Arc;

  protected:
    ConstMap<typename Digraph::Node, bool> const_true_map;

    FilterArcs() : const_true_map(true) {
      Parent::setNodeFilterMap(const_true_map);
    }

  public:

    /// \brief Constructor
    ///
    /// Creates a subdigraph for the given digraph with the given arc
    /// filter map.
    FilterArcs(Digraph& digraph, ArcFilterMap& arc_filter)
      : Parent(), const_true_map(true) {
      Parent::setDigraph(digraph);
      Parent::setNodeFilterMap(const_true_map);
      Parent::setArcFilterMap(arc_filter);
    }

    /// \brief Sets the status of the given arc
    ///
    /// This function sets the status of the given arc.
    /// It is done by simply setting the assigned value of \c a
    /// to \c v in the arc filter map.
    void status(const Arc& a, bool v) const { Parent::status(a, v); }

    /// \brief Returns the status of the given arc
    ///
    /// This function returns the status of the given arc.
    /// It is \c true if the given arc is enabled (i.e. not hidden).
    bool status(const Arc& a) const { return Parent::status(a); }

    /// \brief Disables the given arc
    ///
    /// This function disables the given arc in the subdigraph,
    /// so the iteration jumps over it.
    /// It is the same as \ref status() "status(a, false)".
    void disable(const Arc& a) const { Parent::status(a, false); }

    /// \brief Enables the given arc
    ///
    /// This function enables the given arc in the subdigraph.
    /// It is the same as \ref status() "status(a, true)".
    void enable(const Arc& a) const { Parent::status(a, true); }

  };

  /// \brief Returns a read-only FilterArcs adaptor
  ///
  /// This function just returns a read-only \ref FilterArcs adaptor.
  /// \ingroup graph_adaptors
  /// \relates FilterArcs
  template<typename GR, typename AF>
  FilterArcs<const GR, AF>
  filterArcs(const GR& digraph, AF& arc_filter_map) {
    return FilterArcs<const GR, AF>(digraph, arc_filter_map);
  }

  template<typename GR, typename AF>
  FilterArcs<const GR, const AF>
  filterArcs(const GR& digraph, const AF& arc_filter_map) {
    return FilterArcs<const GR, const AF>(digraph, arc_filter_map);
  }

  /// \ingroup graph_adaptors
  ///
  /// \brief Adaptor class for hiding edges in a graph.
  ///
  /// FilterEdges adaptor can be used for hiding edges in a graph.
  /// A \c bool edge map must be specified, which defines the filter for
  /// the edges. Only the edges with \c true filter value are shown in the
  /// subgraph. This adaptor conforms to the \ref concepts::Graph
  /// "Graph" concept.
  ///
  /// The adapted graph can also be modified through this adaptor
  /// by adding or removing nodes or edges, unless the \c GR template
  /// parameter is set to be \c const.
  ///
  /// \tparam GR The type of the adapted graph.
  /// It must conform to the \ref concepts::Graph "Graph" concept.
  /// It can also be specified to be \c const.
  /// \tparam EF The type of the edge filter map.
  /// It must be a \c bool (or convertible) edge map of the
  /// adapted graph. The default type is
  /// \ref concepts::Graph::EdgeMap "GR::EdgeMap<bool>".
  ///
  /// \note The \c Node, \c Edge and \c Arc types of this adaptor and the
  /// adapted graph are convertible to each other.
#ifdef DOXYGEN
  template<typename GR,
           typename EF>
  class FilterEdges {
#else
  template<typename GR,
           typename EF = typename GR::template EdgeMap<bool> >
  class FilterEdges :
    public GraphAdaptorExtender<
      SubGraphBase<GR, ConstMap<typename GR::Node,bool>, EF, false> > {
#endif
  public:
    /// The type of the adapted graph.
    typedef GR Graph;
    /// The type of the edge filter map.
    typedef EF EdgeFilterMap;

    typedef GraphAdaptorExtender<
      SubGraphBase<GR, ConstMap<typename GR::Node,bool>, EF, false> >
      Parent;

    typedef typename Parent::Edge Edge;

  protected:
    ConstMap<typename Graph::Node, bool> const_true_map;

    FilterEdges() : const_true_map(true) {
      Parent::setNodeFilterMap(const_true_map);
    }

  public:

    /// \brief Constructor
    ///
    /// Creates a subgraph for the given graph with the given edge
    /// filter map.
    FilterEdges(Graph& graph, EdgeFilterMap& edge_filter_map) :
      Parent(), const_true_map(true) {
      Parent::setGraph(graph);
      Parent::setNodeFilterMap(const_true_map);
      Parent::setEdgeFilterMap(edge_filter_map);
    }

    /// \brief Sets the status of the given edge
    ///
    /// This function sets the status of the given edge.
    /// It is done by simply setting the assigned value of \c e
    /// to \c v in the edge filter map.
    void status(const Edge& e, bool v) const { Parent::status(e, v); }

    /// \brief Returns the status of the given edge
    ///
    /// This function returns the status of the given edge.
    /// It is \c true if the given edge is enabled (i.e. not hidden).
    bool status(const Edge& e) const { return Parent::status(e); }

    /// \brief Disables the given edge
    ///
    /// This function disables the given edge in the subgraph,
    /// so the iteration jumps over it.
    /// It is the same as \ref status() "status(e, false)".
    void disable(const Edge& e) const { Parent::status(e, false); }

    /// \brief Enables the given edge
    ///
    /// This function enables the given edge in the subgraph.
    /// It is the same as \ref status() "status(e, true)".
    void enable(const Edge& e) const { Parent::status(e, true); }

  };

  /// \brief Returns a read-only FilterEdges adaptor
  ///
  /// This function just returns a read-only \ref FilterEdges adaptor.
  /// \ingroup graph_adaptors
  /// \relates FilterEdges
  template<typename GR, typename EF>
  FilterEdges<const GR, EF>
  filterEdges(const GR& graph, EF& edge_filter_map) {
    return FilterEdges<const GR, EF>(graph, edge_filter_map);
  }

  template<typename GR, typename EF>
  FilterEdges<const GR, const EF>
  filterEdges(const GR& graph, const EF& edge_filter_map) {
    return FilterEdges<const GR, const EF>(graph, edge_filter_map);
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
  /// \brief Adaptor class for viewing a digraph as an undirected graph.
  ///
  /// Undirector adaptor can be used for viewing a digraph as an undirected
  /// graph. All arcs of the underlying digraph are showed in the
  /// adaptor as an edge (and also as a pair of arcs, of course).
  /// This adaptor conforms to the \ref concepts::Graph "Graph" concept.
  ///
  /// The adapted digraph can also be modified through this adaptor
  /// by adding or removing nodes or edges, unless the \c GR template
  /// parameter is set to be \c const.
  ///
  /// \tparam GR The type of the adapted digraph.
  /// It must conform to the \ref concepts::Digraph "Digraph" concept.
  /// It can also be specified to be \c const.
  ///
  /// \note The \c Node type of this adaptor and the adapted digraph are
  /// convertible to each other, moreover the \c Edge type of the adaptor
  /// and the \c Arc type of the adapted digraph are also convertible to
  /// each other.
  /// (Thus the \c Arc type of the adaptor is convertible to the \c Arc type
  /// of the adapted digraph.)
  template<typename GR>
#ifdef DOXYGEN
  class Undirector {
#else
  class Undirector :
    public GraphAdaptorExtender<UndirectorBase<GR> > {
#endif
  public:
    /// The type of the adapted digraph.
    typedef GR Digraph;
    typedef GraphAdaptorExtender<UndirectorBase<GR> > Parent;
  protected:
    Undirector() { }
  public:

    /// \brief Constructor
    ///
    /// Creates an undirected graph from the given digraph.
    Undirector(Digraph& digraph) {
      setDigraph(digraph);
    }

    /// \brief Arc map combined from two original arc maps
    ///
    /// This map adaptor class adapts two arc maps of the underlying
    /// digraph to get an arc map of the undirected graph.
    /// Its value type is inherited from the first arc map type
    /// (\c %ForwardMap).
    template <typename ForwardMap, typename BackwardMap>
    class CombinedArcMap {
    public:

      /// The key type of the map
      typedef typename Parent::Arc Key;
      /// The value type of the map
      typedef typename ForwardMap::Value Value;

      typedef typename MapTraits<ForwardMap>::ReferenceMapTag ReferenceMapTag;

      typedef typename MapTraits<ForwardMap>::ReturnValue ReturnValue;
      typedef typename MapTraits<ForwardMap>::ConstReturnValue ConstReturnValue;
      typedef typename MapTraits<ForwardMap>::ReturnValue Reference;
      typedef typename MapTraits<ForwardMap>::ConstReturnValue ConstReference;

      /// Constructor
      CombinedArcMap(ForwardMap& forward, BackwardMap& backward)
        : _forward(&forward), _backward(&backward) {}

      /// Sets the value associated with the given key.
      void set(const Key& e, const Value& a) {
        if (Parent::direction(e)) {
          _forward->set(e, a);
        } else {
          _backward->set(e, a);
        }
      }

      /// Returns the value associated with the given key.
      ConstReturnValue operator[](const Key& e) const {
        if (Parent::direction(e)) {
          return (*_forward)[e];
        } else {
          return (*_backward)[e];
        }
      }

      /// Returns a reference to the value associated with the given key.
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

    /// \brief Returns a combined arc map
    ///
    /// This function just returns a combined arc map.
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

  /// \brief Returns a read-only Undirector adaptor
  ///
  /// This function just returns a read-only \ref Undirector adaptor.
  /// \ingroup graph_adaptors
  /// \relates Undirector
  template<typename GR>
  Undirector<const GR> undirector(const GR& digraph) {
    return Undirector<const GR>(digraph);
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
  /// \brief Adaptor class for orienting the edges of a graph to get a digraph
  ///
  /// Orienter adaptor can be used for orienting the edges of a graph to
  /// get a digraph. A \c bool edge map of the underlying graph must be
  /// specified, which define the direction of the arcs in the adaptor.
  /// The arcs can be easily reversed by the \c reverseArc() member function
  /// of the adaptor.
  /// This class conforms to the \ref concepts::Digraph "Digraph" concept.
  ///
  /// The adapted graph can also be modified through this adaptor
  /// by adding or removing nodes or arcs, unless the \c GR template
  /// parameter is set to be \c const.
  ///
  /// \tparam GR The type of the adapted graph.
  /// It must conform to the \ref concepts::Graph "Graph" concept.
  /// It can also be specified to be \c const.
  /// \tparam DM The type of the direction map.
  /// It must be a \c bool (or convertible) edge map of the
  /// adapted graph. The default type is
  /// \ref concepts::Graph::EdgeMap "GR::EdgeMap<bool>".
  ///
  /// \note The \c Node type of this adaptor and the adapted graph are
  /// convertible to each other, moreover the \c Arc type of the adaptor
  /// and the \c Edge type of the adapted graph are also convertible to
  /// each other.
#ifdef DOXYGEN
  template<typename GR,
           typename DM>
  class Orienter {
#else
  template<typename GR,
           typename DM = typename GR::template EdgeMap<bool> >
  class Orienter :
    public DigraphAdaptorExtender<OrienterBase<GR, DM> > {
#endif
  public:

    /// The type of the adapted graph.
    typedef GR Graph;
    /// The type of the direction edge map.
    typedef DM DirectionMap;

    typedef DigraphAdaptorExtender<OrienterBase<GR, DM> > Parent;
    typedef typename Parent::Arc Arc;
  protected:
    Orienter() { }
  public:

    /// \brief Constructor
    ///
    /// Constructor of the adaptor.
    Orienter(Graph& graph, DirectionMap& direction) {
      setGraph(graph);
      setDirectionMap(direction);
    }

    /// \brief Reverses the given arc
    ///
    /// This function reverses the given arc.
    /// It is done by simply negate the assigned value of \c a
    /// in the direction map.
    void reverseArc(const Arc& a) {
      Parent::reverseArc(a);
    }
  };

  /// \brief Returns a read-only Orienter adaptor
  ///
  /// This function just returns a read-only \ref Orienter adaptor.
  /// \ingroup graph_adaptors
  /// \relates Orienter
  template<typename GR, typename DM>
  Orienter<const GR, DM>
  orienter(const GR& graph, DM& direction_map) {
    return Orienter<const GR, DM>(graph, direction_map);
  }

  template<typename GR, typename DM>
  Orienter<const GR, const DM>
  orienter(const GR& graph, const DM& direction_map) {
    return Orienter<const GR, const DM>(graph, direction_map);
  }

  namespace _adaptor_bits {

    template<typename Digraph,
             typename CapacityMap,
             typename FlowMap,
             typename Tolerance>
    class ResForwardFilter {
    public:

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

    template<typename Digraph,
             typename CapacityMap,
             typename FlowMap,
             typename Tolerance>
    class ResBackwardFilter {
    public:

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
  /// \brief Adaptor class for composing the residual digraph for directed
  /// flow and circulation problems.
  ///
  /// ResidualDigraph can be used for composing the \e residual digraph
  /// for directed flow and circulation problems. Let \f$ G=(V, A) \f$
  /// be a directed graph and let \f$ F \f$ be a number type.
  /// Let \f$ flow, cap: A\to F \f$ be functions on the arcs.
  /// This adaptor implements a digraph structure with node set \f$ V \f$
  /// and arc set \f$ A_{forward}\cup A_{backward} \f$,
  /// where \f$ A_{forward}=\{uv : uv\in A, flow(uv)<cap(uv)\} \f$ and
  /// \f$ A_{backward}=\{vu : uv\in A, flow(uv)>0\} \f$, i.e. the so
  /// called residual digraph.
  /// When the union \f$ A_{forward}\cup A_{backward} \f$ is taken,
  /// multiplicities are counted, i.e. the adaptor has exactly
  /// \f$ |A_{forward}| + |A_{backward}|\f$ arcs (it may have parallel
  /// arcs).
  /// This class conforms to the \ref concepts::Digraph "Digraph" concept.
  ///
  /// \tparam GR The type of the adapted digraph.
  /// It must conform to the \ref concepts::Digraph "Digraph" concept.
  /// It is implicitly \c const.
  /// \tparam CM The type of the capacity map.
  /// It must be an arc map of some numerical type, which defines
  /// the capacities in the flow problem. It is implicitly \c const.
  /// The default type is
  /// \ref concepts::Digraph::ArcMap "GR::ArcMap<int>".
  /// \tparam FM The type of the flow map.
  /// It must be an arc map of some numerical type, which defines
  /// the flow values in the flow problem. The default type is \c CM.
  /// \tparam TL The tolerance type for handling inexact computation.
  /// The default tolerance type depends on the value type of the
  /// capacity map.
  ///
  /// \note This adaptor is implemented using Undirector and FilterArcs
  /// adaptors.
  ///
  /// \note The \c Node type of this adaptor and the adapted digraph are
  /// convertible to each other, moreover the \c Arc type of the adaptor
  /// is convertible to the \c Arc type of the adapted digraph.
#ifdef DOXYGEN
  template<typename GR, typename CM, typename FM, typename TL>
  class ResidualDigraph
#else
  template<typename GR,
           typename CM = typename GR::template ArcMap<int>,
           typename FM = CM,
           typename TL = Tolerance<typename CM::Value> >
  class ResidualDigraph :
    public FilterArcs<
      Undirector<const GR>,
      typename Undirector<const GR>::template CombinedArcMap<
        _adaptor_bits::ResForwardFilter<const GR, CM, FM, TL>,
        _adaptor_bits::ResBackwardFilter<const GR, CM, FM, TL> > >
#endif
  {
  public:

    /// The type of the underlying digraph.
    typedef GR Digraph;
    /// The type of the capacity map.
    typedef CM CapacityMap;
    /// The type of the flow map.
    typedef FM FlowMap;
    /// The tolerance type.
    typedef TL Tolerance;

    typedef typename CapacityMap::Value Value;
    typedef ResidualDigraph Adaptor;

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

    /// \brief Constructor
    ///
    /// Constructor of the residual digraph adaptor. The parameters are the
    /// digraph, the capacity map, the flow map, and a tolerance object.
    ResidualDigraph(const Digraph& digraph, const CapacityMap& capacity,
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

    /// \brief Returns the residual capacity of the given arc.
    ///
    /// Returns the residual capacity of the given arc.
    Value residualCapacity(const Arc& a) const {
      if (Undirected::direction(a)) {
        return (*_capacity)[a] - (*_flow)[a];
      } else {
        return (*_flow)[a];
      }
    }

    /// \brief Augments on the given arc in the residual digraph.
    ///
    /// Augments on the given arc in the residual digraph. It increases
    /// or decreases the flow value on the original arc according to the
    /// direction of the residual arc.
    void augment(const Arc& a, const Value& v) const {
      if (Undirected::direction(a)) {
        _flow->set(a, (*_flow)[a] + v);
      } else {
        _flow->set(a, (*_flow)[a] - v);
      }
    }

    /// \brief Returns \c true if the given residual arc is a forward arc.
    ///
    /// Returns \c true if the given residual arc has the same orientation
    /// as the original arc, i.e. it is a so called forward arc.
    static bool forward(const Arc& a) {
      return Undirected::direction(a);
    }

    /// \brief Returns \c true if the given residual arc is a backward arc.
    ///
    /// Returns \c true if the given residual arc has the opposite orientation
    /// than the original arc, i.e. it is a so called backward arc.
    static bool backward(const Arc& a) {
      return !Undirected::direction(a);
    }

    /// \brief Returns the forward oriented residual arc.
    ///
    /// Returns the forward oriented residual arc related to the given
    /// arc of the underlying digraph.
    static Arc forward(const typename Digraph::Arc& a) {
      return Undirected::direct(a, true);
    }

    /// \brief Returns the backward oriented residual arc.
    ///
    /// Returns the backward oriented residual arc related to the given
    /// arc of the underlying digraph.
    static Arc backward(const typename Digraph::Arc& a) {
      return Undirected::direct(a, false);
    }

    /// \brief Residual capacity map.
    ///
    /// This map adaptor class can be used for obtaining the residual
    /// capacities as an arc map of the residual digraph.
    /// Its value type is inherited from the capacity map.
    class ResidualCapacity {
    protected:
      const Adaptor* _adaptor;
    public:
      /// The key type of the map
      typedef Arc Key;
      /// The value type of the map
      typedef typename CapacityMap::Value Value;

      /// Constructor
      ResidualCapacity(const Adaptor& adaptor) : _adaptor(&adaptor) {}

      /// Returns the value associated with the given residual arc
      Value operator[](const Arc& a) const {
        return _adaptor->residualCapacity(a);
      }

    };

    /// \brief Returns a residual capacity map
    ///
    /// This function just returns a residual capacity map.
    ResidualCapacity residualCapacity() const {
      return ResidualCapacity(*this);
    }

  };

  /// \brief Returns a (read-only) Residual adaptor
  ///
  /// This function just returns a (read-only) \ref Residual adaptor.
  /// \ingroup graph_adaptors
  /// \relates Residual
  template<typename GR, typename CM, typename FM>
  ResidualDigraph<GR, CM, FM>
  residualDigraph(const GR& digraph, const CM& capacity_map, FM& flow_map) {
    return ResidualDigraph<GR, CM, FM> (digraph, capacity_map, flow_map);
  }


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
  /// \brief Adaptor class for splitting the nodes of a digraph.
  ///
  /// SplitNodes adaptor can be used for splitting each node into an
  /// \e in-node and an \e out-node in a digraph. Formaly, the adaptor
  /// replaces each node \f$ u \f$ in the digraph with two nodes,
  /// namely node \f$ u_{in} \f$ and node \f$ u_{out} \f$.
  /// If there is a \f$ (v, u) \f$ arc in the original digraph, then the
  /// new target of the arc will be \f$ u_{in} \f$ and similarly the
  /// source of each original \f$ (u, v) \f$ arc will be \f$ u_{out} \f$.
  /// The adaptor adds an additional \e bind \e arc from \f$ u_{in} \f$
  /// to \f$ u_{out} \f$ for each node \f$ u \f$ of the original digraph.
  ///
  /// The aim of this class is running an algorithm with respect to node
  /// costs or capacities if the algorithm considers only arc costs or
  /// capacities directly.
  /// In this case you can use \c SplitNodes adaptor, and set the node
  /// costs/capacities of the original digraph to the \e bind \e arcs
  /// in the adaptor.
  ///
  /// \tparam GR The type of the adapted digraph.
  /// It must conform to the \ref concepts::Digraph "Digraph" concept.
  /// It is implicitly \c const.
  ///
  /// \note The \c Node type of this adaptor is converible to the \c Node
  /// type of the adapted digraph.
  template <typename GR>
#ifdef DOXYGEN
  class SplitNodes {
#else
  class SplitNodes
    : public DigraphAdaptorExtender<SplitNodesBase<const GR> > {
#endif
  public:
    typedef GR Digraph;
    typedef DigraphAdaptorExtender<SplitNodesBase<const GR> > Parent;

    typedef typename Digraph::Node DigraphNode;
    typedef typename Digraph::Arc DigraphArc;

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;

    /// \brief Constructor
    ///
    /// Constructor of the adaptor.
    SplitNodes(const Digraph& g) {
      Parent::setDigraph(g);
    }

    /// \brief Returns \c true if the given node is an in-node.
    ///
    /// Returns \c true if the given node is an in-node.
    static bool inNode(const Node& n) {
      return Parent::inNode(n);
    }

    /// \brief Returns \c true if the given node is an out-node.
    ///
    /// Returns \c true if the given node is an out-node.
    static bool outNode(const Node& n) {
      return Parent::outNode(n);
    }

    /// \brief Returns \c true if the given arc is an original arc.
    ///
    /// Returns \c true if the given arc is one of the arcs in the
    /// original digraph.
    static bool origArc(const Arc& a) {
      return Parent::origArc(a);
    }

    /// \brief Returns \c true if the given arc is a bind arc.
    ///
    /// Returns \c true if the given arc is a bind arc, i.e. it connects
    /// an in-node and an out-node.
    static bool bindArc(const Arc& a) {
      return Parent::bindArc(a);
    }

    /// \brief Returns the in-node created from the given original node.
    ///
    /// Returns the in-node created from the given original node.
    static Node inNode(const DigraphNode& n) {
      return Parent::inNode(n);
    }

    /// \brief Returns the out-node created from the given original node.
    ///
    /// Returns the out-node created from the given original node.
    static Node outNode(const DigraphNode& n) {
      return Parent::outNode(n);
    }

    /// \brief Returns the bind arc that corresponds to the given
    /// original node.
    ///
    /// Returns the bind arc in the adaptor that corresponds to the given
    /// original node, i.e. the arc connecting the in-node and out-node
    /// of \c n.
    static Arc arc(const DigraphNode& n) {
      return Parent::arc(n);
    }

    /// \brief Returns the arc that corresponds to the given original arc.
    ///
    /// Returns the arc in the adaptor that corresponds to the given
    /// original arc.
    static Arc arc(const DigraphArc& a) {
      return Parent::arc(a);
    }

    /// \brief Node map combined from two original node maps
    ///
    /// This map adaptor class adapts two node maps of the original digraph
    /// to get a node map of the split digraph.
    /// Its value type is inherited from the first node map type
    /// (\c InNodeMap).
    template <typename InNodeMap, typename OutNodeMap>
    class CombinedNodeMap {
    public:

      /// The key type of the map
      typedef Node Key;
      /// The value type of the map
      typedef typename InNodeMap::Value Value;

      typedef typename MapTraits<InNodeMap>::ReferenceMapTag ReferenceMapTag;
      typedef typename MapTraits<InNodeMap>::ReturnValue ReturnValue;
      typedef typename MapTraits<InNodeMap>::ConstReturnValue ConstReturnValue;
      typedef typename MapTraits<InNodeMap>::ReturnValue Reference;
      typedef typename MapTraits<InNodeMap>::ConstReturnValue ConstReference;

      /// Constructor
      CombinedNodeMap(InNodeMap& in_map, OutNodeMap& out_map)
        : _in_map(in_map), _out_map(out_map) {}

      /// Returns the value associated with the given key.
      Value operator[](const Key& key) const {
        if (Parent::inNode(key)) {
          return _in_map[key];
        } else {
          return _out_map[key];
        }
      }

      /// Returns a reference to the value associated with the given key.
      Value& operator[](const Key& key) {
        if (Parent::inNode(key)) {
          return _in_map[key];
        } else {
          return _out_map[key];
        }
      }

      /// Sets the value associated with the given key.
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


    /// \brief Returns a combined node map
    ///
    /// This function just returns a combined node map.
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

    /// \brief Arc map combined from an arc map and a node map of the
    /// original digraph.
    ///
    /// This map adaptor class adapts an arc map and a node map of the
    /// original digraph to get an arc map of the split digraph.
    /// Its value type is inherited from the original arc map type
    /// (\c ArcMap).
    template <typename ArcMap, typename NodeMap>
    class CombinedArcMap {
    public:

      /// The key type of the map
      typedef Arc Key;
      /// The value type of the map
      typedef typename ArcMap::Value Value;

      typedef typename MapTraits<ArcMap>::ReferenceMapTag ReferenceMapTag;
      typedef typename MapTraits<ArcMap>::ReturnValue ReturnValue;
      typedef typename MapTraits<ArcMap>::ConstReturnValue ConstReturnValue;
      typedef typename MapTraits<ArcMap>::ReturnValue Reference;
      typedef typename MapTraits<ArcMap>::ConstReturnValue ConstReference;

      /// Constructor
      CombinedArcMap(ArcMap& arc_map, NodeMap& node_map)
        : _arc_map(arc_map), _node_map(node_map) {}

      /// Returns the value associated with the given key.
      Value operator[](const Key& arc) const {
        if (Parent::origArc(arc)) {
          return _arc_map[arc];
        } else {
          return _node_map[arc];
        }
      }

      /// Returns a reference to the value associated with the given key.
      Value& operator[](const Key& arc) {
        if (Parent::origArc(arc)) {
          return _arc_map[arc];
        } else {
          return _node_map[arc];
        }
      }

      /// Sets the value associated with the given key.
      void set(const Arc& arc, const Value& val) {
        if (Parent::origArc(arc)) {
          _arc_map.set(arc, val);
        } else {
          _node_map.set(arc, val);
        }
      }

    private:
      ArcMap& _arc_map;
      NodeMap& _node_map;
    };

    /// \brief Returns a combined arc map
    ///
    /// This function just returns a combined arc map.
    template <typename ArcMap, typename NodeMap>
    static CombinedArcMap<ArcMap, NodeMap>
    combinedArcMap(ArcMap& arc_map, NodeMap& node_map) {
      return CombinedArcMap<ArcMap, NodeMap>(arc_map, node_map);
    }

    template <typename ArcMap, typename NodeMap>
    static CombinedArcMap<const ArcMap, NodeMap>
    combinedArcMap(const ArcMap& arc_map, NodeMap& node_map) {
      return CombinedArcMap<const ArcMap, NodeMap>(arc_map, node_map);
    }

    template <typename ArcMap, typename NodeMap>
    static CombinedArcMap<ArcMap, const NodeMap>
    combinedArcMap(ArcMap& arc_map, const NodeMap& node_map) {
      return CombinedArcMap<ArcMap, const NodeMap>(arc_map, node_map);
    }

    template <typename ArcMap, typename NodeMap>
    static CombinedArcMap<const ArcMap, const NodeMap>
    combinedArcMap(const ArcMap& arc_map, const NodeMap& node_map) {
      return CombinedArcMap<const ArcMap, const NodeMap>(arc_map, node_map);
    }

  };

  /// \brief Returns a (read-only) SplitNodes adaptor
  ///
  /// This function just returns a (read-only) \ref SplitNodes adaptor.
  /// \ingroup graph_adaptors
  /// \relates SplitNodes
  template<typename GR>
  SplitNodes<GR>
  splitNodes(const GR& digraph) {
    return SplitNodes<GR>(digraph);
  }

} //namespace lemon

#endif //LEMON_ADAPTORS_H
