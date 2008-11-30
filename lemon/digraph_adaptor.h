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

#ifndef LEMON_DIGRAPH_ADAPTOR_H
#define LEMON_DIGRAPH_ADAPTOR_H

///\ingroup graph_adaptors
///\file
///\brief Several digraph adaptors.
///
///This file contains several useful digraph adaptor functions.

#include <lemon/core.h>
#include <lemon/maps.h>
#include <lemon/bits/variant.h>

#include <lemon/bits/base_extender.h>
#include <lemon/bits/graph_adaptor_extender.h>
#include <lemon/bits/graph_extender.h>
#include <lemon/tolerance.h>

#include <algorithm>

namespace lemon {

  ///\brief Base type for the Digraph Adaptors
  ///
  ///Base type for the Digraph Adaptors
  ///
  ///This is the base type for most of LEMON digraph adaptors. This
  ///class implements a trivial digraph adaptor i.e. it only wraps the
  ///functions and types of the digraph. The purpose of this class is
  ///to make easier implementing digraph adaptors. E.g. if an adaptor
  ///is considered which differs from the wrapped digraph only in some
  ///of its functions or types, then it can be derived from
  ///DigraphAdaptor, and only the differences should be implemented.
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
    
    typedef EdgeNumTagIndicator<Digraph> EdgeNumTag;
    int arcNum() const { return _digraph->arcNum(); }

    typedef FindEdgeTagIndicator<Digraph> FindEdgeTag;
    Arc findArc(const Node& u, const Node& v, const Arc& prev = INVALID) {
      return _digraph->findArc(u, v, prev);
    }
  
    Node addNode() { return _digraph->addNode(); }
    Arc addArc(const Node& u, const Node& v) { return _digraph->addArc(u, v); }

    void erase(const Node& n) const { _digraph->erase(n); }
    void erase(const Arc& a) const { _digraph->erase(a); }
  
    void clear() const { _digraph->clear(); }
    
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

  ///\ingroup graph_adaptors
  ///
  ///\brief Trivial Digraph Adaptor
  /// 
  /// This class is an adaptor which does not change the adapted
  /// digraph.  It can be used only to test the digraph adaptors.
  template <typename _Digraph>
  class DigraphAdaptor :
    public DigraphAdaptorExtender<DigraphAdaptorBase<_Digraph> > { 
  public:
    typedef _Digraph Digraph;
    typedef DigraphAdaptorExtender<DigraphAdaptorBase<_Digraph> > Parent;
  protected:
    DigraphAdaptor() : Parent() { }

  public:
    explicit DigraphAdaptor(Digraph& digraph) { setDigraph(digraph); }
  };

  /// \brief Just gives back a digraph adaptor
  ///
  /// Just gives back a digraph adaptor which 
  /// should be provide original digraph
  template<typename Digraph>
  DigraphAdaptor<const Digraph>
  digraphAdaptor(const Digraph& digraph) {
    return DigraphAdaptor<const Digraph>(digraph);
  }


  template <typename _Digraph>
  class RevDigraphAdaptorBase : public DigraphAdaptorBase<_Digraph> {
  public:
    typedef _Digraph Digraph;
    typedef DigraphAdaptorBase<_Digraph> Parent;
  protected:
    RevDigraphAdaptorBase() : Parent() { }
  public:
    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;

    void firstIn(Arc& a, const Node& n) const { Parent::firstOut(a, n); }
    void firstOut(Arc& a, const Node& n ) const { Parent::firstIn(a, n); }

    void nextIn(Arc& a) const { Parent::nextOut(a); }
    void nextOut(Arc& a) const { Parent::nextIn(a); }

    Node source(const Arc& a) const { return Parent::target(a); }
    Node target(const Arc& a) const { return Parent::source(a); }

    typedef FindEdgeTagIndicator<Digraph> FindEdgeTag;
    Arc findArc(const Node& u, const Node& v, 
		const Arc& prev = INVALID) {
      return Parent::findArc(v, u, prev);
    }

  };
    

  ///\ingroup graph_adaptors
  ///
  ///\brief A digraph adaptor which reverses the orientation of the arcs.
  ///
  /// If \c g is defined as
  ///\code
  /// ListDigraph g;
  ///\endcode
  /// then
  ///\code
  /// RevDigraphAdaptor<ListDigraph> ga(g);
  ///\endcode
  /// implements the digraph obtained from \c g by 
  /// reversing the orientation of its arcs.
  ///
  /// A good example of using RevDigraphAdaptor is to decide that the
  /// directed graph is wheter strongly connected or not. If from one
  /// node each node is reachable and from each node is reachable this
  /// node then and just then the digraph is strongly
  /// connected. Instead of this condition we use a little bit
  /// different. From one node each node ahould be reachable in the
  /// digraph and in the reversed digraph. Now this condition can be
  /// checked with the Dfs algorithm class and the RevDigraphAdaptor
  /// algorithm class.
  ///
  /// And look at the code:
  ///
  ///\code
  /// bool stronglyConnected(const Digraph& digraph) {
  ///   if (NodeIt(digraph) == INVALID) return true;
  ///   Dfs<Digraph> dfs(digraph);
  ///   dfs.run(NodeIt(digraph));
  ///   for (NodeIt it(digraph); it != INVALID; ++it) {
  ///     if (!dfs.reached(it)) {
  ///       return false;
  ///     }
  ///   }
  ///   typedef RevDigraphAdaptor<const Digraph> RDigraph;
  ///   RDigraph rdigraph(digraph);
  ///   DfsVisit<RDigraph> rdfs(rdigraph);
  ///   rdfs.run(NodeIt(digraph));
  ///   for (NodeIt it(digraph); it != INVALID; ++it) {
  ///     if (!rdfs.reached(it)) {
  ///       return false;
  ///     }
  ///   }
  ///   return true;
  /// }
  ///\endcode
  template<typename _Digraph>
  class RevDigraphAdaptor : 
    public DigraphAdaptorExtender<RevDigraphAdaptorBase<_Digraph> > {
  public:
    typedef _Digraph Digraph;
    typedef DigraphAdaptorExtender<
      RevDigraphAdaptorBase<_Digraph> > Parent;
  protected:
    RevDigraphAdaptor() { }
  public:
    explicit RevDigraphAdaptor(Digraph& digraph) { 
      Parent::setDigraph(digraph); 
    }
  };

  /// \brief Just gives back a reverse digraph adaptor
  ///
  /// Just gives back a reverse digraph adaptor
  template<typename Digraph>
  RevDigraphAdaptor<const Digraph>
  revDigraphAdaptor(const Digraph& digraph) {
    return RevDigraphAdaptor<const Digraph>(digraph);
  }

  template <typename _Digraph, typename _NodeFilterMap, 
	    typename _ArcFilterMap, bool checked = true>
  class SubDigraphAdaptorBase : public DigraphAdaptorBase<_Digraph> {
  public:
    typedef _Digraph Digraph;
    typedef _NodeFilterMap NodeFilterMap;
    typedef _ArcFilterMap ArcFilterMap;

    typedef SubDigraphAdaptorBase Adaptor;
    typedef DigraphAdaptorBase<_Digraph> Parent;
  protected:
    NodeFilterMap* _node_filter;
    ArcFilterMap* _arc_filter;
    SubDigraphAdaptorBase() 
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
	     || !(*_node_filter)[Parent::target(i)])) Parent::next(i); 
    }

    void firstIn(Arc& i, const Node& n) const { 
      Parent::firstIn(i, n); 
      while (i != INVALID && (!(*_arc_filter)[i] 
	     || !(*_node_filter)[Parent::source(i)])) Parent::nextIn(i); 
    }

    void firstOut(Arc& i, const Node& n) const { 
      Parent::firstOut(i, n); 
      while (i != INVALID && (!(*_arc_filter)[i] 
	     || !(*_node_filter)[Parent::target(i)])) Parent::nextOut(i); 
    }

    void next(Node& i) const { 
      Parent::next(i); 
      while (i != INVALID && !(*_node_filter)[i]) Parent::next(i); 
    }

    void next(Arc& i) const { 
      Parent::next(i); 
      while (i != INVALID && (!(*_arc_filter)[i] 
	     || !(*_node_filter)[Parent::source(i)]
	     || !(*_node_filter)[Parent::target(i)])) Parent::next(i); 
    }

    void nextIn(Arc& i) const { 
      Parent::nextIn(i); 
      while (i != INVALID && (!(*_arc_filter)[i] 
	     || !(*_node_filter)[Parent::source(i)])) Parent::nextIn(i); 
    }

    void nextOut(Arc& i) const { 
      Parent::nextOut(i); 
      while (i != INVALID && (!(*_arc_filter)[i] 
	     || !(*_node_filter)[Parent::target(i)])) Parent::nextOut(i); 
    }

    ///\e

    /// This function hides \c n in the digraph, i.e. the iteration 
    /// jumps over it. This is done by simply setting the value of \c n  
    /// to be false in the corresponding node-map.
    void hide(const Node& n) const { _node_filter->set(n, false); }

    ///\e

    /// This function hides \c a in the digraph, i.e. the iteration 
    /// jumps over it. This is done by simply setting the value of \c a
    /// to be false in the corresponding arc-map.
    void hide(const Arc& a) const { _arc_filter->set(a, false); }

    ///\e

    /// The value of \c n is set to be true in the node-map which stores 
    /// hide information. If \c n was hidden previuosly, then it is shown 
    /// again
     void unHide(const Node& n) const { _node_filter->set(n, true); }

    ///\e

    /// The value of \c a is set to be true in the arc-map which stores 
    /// hide information. If \c a was hidden previuosly, then it is shown 
    /// again
    void unHide(const Arc& a) const { _arc_filter->set(a, true); }

    /// Returns true if \c n is hidden.
    
    ///\e
    ///
    bool hidden(const Node& n) const { return !(*_node_filter)[n]; }

    /// Returns true if \c a is hidden.
    
    ///\e
    ///
    bool hidden(const Arc& a) const { return !(*_arc_filter)[a]; }

    typedef False NodeNumTag;
    typedef False EdgeNumTag;

    typedef FindEdgeTagIndicator<Digraph> FindEdgeTag;
    Arc findArc(const Node& source, const Node& target, 
		const Arc& prev = INVALID) {
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
  class SubDigraphAdaptorBase<_Digraph, _NodeFilterMap, _ArcFilterMap, false> 
    : public DigraphAdaptorBase<_Digraph> {
  public:
    typedef _Digraph Digraph;
    typedef _NodeFilterMap NodeFilterMap;
    typedef _ArcFilterMap ArcFilterMap;

    typedef SubDigraphAdaptorBase Adaptor;
    typedef DigraphAdaptorBase<Digraph> Parent;
  protected:
    NodeFilterMap* _node_filter;
    ArcFilterMap* _arc_filter;
    SubDigraphAdaptorBase() 
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

    ///\e

    /// This function hides \c n in the digraph, i.e. the iteration 
    /// jumps over it. This is done by simply setting the value of \c n  
    /// to be false in the corresponding node-map.
    void hide(const Node& n) const { _node_filter->set(n, false); }

    ///\e

    /// This function hides \c e in the digraph, i.e. the iteration 
    /// jumps over it. This is done by simply setting the value of \c e  
    /// to be false in the corresponding arc-map.
    void hide(const Arc& e) const { _arc_filter->set(e, false); }

    ///\e

    /// The value of \c n is set to be true in the node-map which stores 
    /// hide information. If \c n was hidden previuosly, then it is shown 
    /// again
     void unHide(const Node& n) const { _node_filter->set(n, true); }

    ///\e

    /// The value of \c e is set to be true in the arc-map which stores 
    /// hide information. If \c e was hidden previuosly, then it is shown 
    /// again
    void unHide(const Arc& e) const { _arc_filter->set(e, true); }

    /// Returns true if \c n is hidden.
    
    ///\e
    ///
    bool hidden(const Node& n) const { return !(*_node_filter)[n]; }

    /// Returns true if \c n is hidden.
    
    ///\e
    ///
    bool hidden(const Arc& e) const { return !(*_arc_filter)[e]; }

    typedef False NodeNumTag;
    typedef False EdgeNumTag;

    typedef FindEdgeTagIndicator<Digraph> FindEdgeTag;
    Arc findArc(const Node& source, const Node& target, 
		  const Arc& prev = INVALID) {
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
  /// \brief A digraph adaptor for hiding nodes and arcs from a digraph.
  /// 
  /// SubDigraphAdaptor shows the digraph with filtered node-set and 
  /// arc-set. If the \c checked parameter is true then it filters the arcset
  /// to do not get invalid arcs without source or target.
  /// Let \f$ G=(V, A) \f$ be a directed digraph
  /// and suppose that the digraph instance \c g of type ListDigraph
  /// implements \f$ G \f$.
  /// Let moreover \f$ b_V \f$ and \f$ b_A \f$ be bool-valued functions resp.
  /// on the node-set and arc-set.
  /// SubDigraphAdaptor<...>::NodeIt iterates 
  /// on the node-set \f$ \{v\in V : b_V(v)=true\} \f$ and 
  /// SubDigraphAdaptor<...>::ArcIt iterates 
  /// on the arc-set \f$ \{e\in A : b_A(e)=true\} \f$. Similarly, 
  /// SubDigraphAdaptor<...>::OutArcIt and
  /// SubDigraphAdaptor<...>::InArcIt iterates 
  /// only on arcs leaving and entering a specific node which have true value.
  /// 
  /// If the \c checked template parameter is false then we have to
  /// note that the node-iterator cares only the filter on the
  /// node-set, and the arc-iterator cares only the filter on the
  /// arc-set.  This way the arc-map should filter all arcs which's
  /// source or target is filtered by the node-filter.
  ///\code
  /// typedef ListDigraph Digraph;
  /// DIGRAPH_TYPEDEFS(Digraph);
  /// Digraph g;
  /// Node u=g.addNode(); //node of id 0
  /// Node v=g.addNode(); //node of id 1
  /// Arc a=g.addArc(u, v); //arc of id 0
  /// Arc f=g.addArc(v, u); //arc of id 1
  /// BoolNodeMap nm(g, true);
  /// nm.set(u, false);
  /// BoolArcMap am(g, true);
  /// am.set(a, false);
  /// typedef SubDigraphAdaptor<Digraph, BoolNodeMap, BoolArcMap> SubGA;
  /// SubGA ga(g, nm, am);
  /// for (SubGA::NodeIt n(ga); n!=INVALID; ++n)
  ///   std::cout << g.id(n) << std::endl;
  /// std::cout << ":-)" << std::endl;
  /// for (SubGA::ArcIt a(ga); a!=INVALID; ++a) 
  ///   std::cout << g.id(a) << std::endl;
  ///\endcode
  /// The output of the above code is the following.
  ///\code
  /// 1
  /// :-)
  /// 1
  ///\endcode
  /// Note that \c n is of type \c SubGA::NodeIt, but it can be converted to
  /// \c Digraph::Node that is why \c g.id(n) can be applied.
  /// 
  /// For other examples see also the documentation of
  /// NodeSubDigraphAdaptor and ArcSubDigraphAdaptor.
  template<typename _Digraph, 
	   typename _NodeFilterMap = typename _Digraph::template NodeMap<bool>, 
	   typename _ArcFilterMap = typename _Digraph::template ArcMap<bool>, 
	   bool checked = true>
  class SubDigraphAdaptor : 
    public DigraphAdaptorExtender<
    SubDigraphAdaptorBase<_Digraph, _NodeFilterMap, _ArcFilterMap, checked> > {
  public:
    typedef _Digraph Digraph;
    typedef _NodeFilterMap NodeFilterMap;
    typedef _ArcFilterMap ArcFilterMap;

    typedef DigraphAdaptorExtender<
      SubDigraphAdaptorBase<Digraph, NodeFilterMap, ArcFilterMap, checked> >
    Parent;

  protected:
    SubDigraphAdaptor() { }
  public:

    SubDigraphAdaptor(Digraph& digraph, NodeFilterMap& node_filter, 
		      ArcFilterMap& arc_filter) { 
      setDigraph(digraph);
      setNodeFilterMap(node_filter);
      setArcFilterMap(arc_filter);
    }

  };

  /// \brief Just gives back a sub digraph adaptor
  ///
  /// Just gives back a sub digraph adaptor
  template<typename Digraph, typename NodeFilterMap, typename ArcFilterMap>
  SubDigraphAdaptor<const Digraph, NodeFilterMap, ArcFilterMap>
  subDigraphAdaptor(const Digraph& digraph, 
		    NodeFilterMap& nfm, ArcFilterMap& afm) {
    return SubDigraphAdaptor<const Digraph, NodeFilterMap, ArcFilterMap>
      (digraph, nfm, afm);
  }

  template<typename Digraph, typename NodeFilterMap, typename ArcFilterMap>
  SubDigraphAdaptor<const Digraph, const NodeFilterMap, ArcFilterMap>
  subDigraphAdaptor(const Digraph& digraph, 
                   NodeFilterMap& nfm, ArcFilterMap& afm) {
    return SubDigraphAdaptor<const Digraph, const NodeFilterMap, ArcFilterMap>
      (digraph, nfm, afm);
  }

  template<typename Digraph, typename NodeFilterMap, typename ArcFilterMap>
  SubDigraphAdaptor<const Digraph, NodeFilterMap, const ArcFilterMap>
  subDigraphAdaptor(const Digraph& digraph, 
                   NodeFilterMap& nfm, ArcFilterMap& afm) {
    return SubDigraphAdaptor<const Digraph, NodeFilterMap, const ArcFilterMap>
      (digraph, nfm, afm);
  }

  template<typename Digraph, typename NodeFilterMap, typename ArcFilterMap>
  SubDigraphAdaptor<const Digraph, const NodeFilterMap, const ArcFilterMap>
  subDigraphAdaptor(const Digraph& digraph, 
                   NodeFilterMap& nfm, ArcFilterMap& afm) {
    return SubDigraphAdaptor<const Digraph, const NodeFilterMap, 
      const ArcFilterMap>(digraph, nfm, afm);
  }



  ///\ingroup graph_adaptors
  ///
  ///\brief An adaptor for hiding nodes from a digraph.
  ///
  ///An adaptor for hiding nodes from a digraph.  This adaptor
  ///specializes SubDigraphAdaptor in the way that only the node-set
  ///can be filtered. In usual case the checked parameter is true, we
  ///get the induced subgraph. But if the checked parameter is false
  ///then we can filter only isolated nodes.
  template<typename _Digraph, 
	   typename _NodeFilterMap = typename _Digraph::template NodeMap<bool>, 
	   bool checked = true>
  class NodeSubDigraphAdaptor : 
    public SubDigraphAdaptor<_Digraph, _NodeFilterMap, 
			     ConstMap<typename _Digraph::Arc, bool>, checked> {
  public:

    typedef _Digraph Digraph;
    typedef _NodeFilterMap NodeFilterMap;

    typedef SubDigraphAdaptor<Digraph, NodeFilterMap, 
			      ConstMap<typename Digraph::Arc, bool>, checked> 
    Parent;

  protected:
    ConstMap<typename Digraph::Arc, bool> const_true_map;

    NodeSubDigraphAdaptor() : const_true_map(true) {
      Parent::setArcFilterMap(const_true_map);
    }

  public:

    NodeSubDigraphAdaptor(Digraph& _digraph, NodeFilterMap& node_filter) : 
      Parent(), const_true_map(true) { 
      Parent::setDigraph(_digraph);
      Parent::setNodeFilterMap(node_filter);
      Parent::setArcFilterMap(const_true_map);
    }

  };


  /// \brief Just gives back a \c NodeSubDigraphAdaptor
  ///
  /// Just gives back a \c NodeSubDigraphAdaptor
  template<typename Digraph, typename NodeFilterMap>
  NodeSubDigraphAdaptor<const Digraph, NodeFilterMap>
  nodeSubDigraphAdaptor(const Digraph& digraph, NodeFilterMap& nfm) {
    return NodeSubDigraphAdaptor<const Digraph, NodeFilterMap>(digraph, nfm);
  }

  template<typename Digraph, typename NodeFilterMap>
  NodeSubDigraphAdaptor<const Digraph, const NodeFilterMap>
  nodeSubDigraphAdaptor(const Digraph& digraph, const NodeFilterMap& nfm) {
    return NodeSubDigraphAdaptor<const Digraph, const NodeFilterMap>
      (digraph, nfm);
  }

  ///\ingroup graph_adaptors
  ///
  ///\brief An adaptor for hiding arcs from a digraph.
  ///
  ///An adaptor for hiding arcs from a digraph. This adaptor
  ///specializes SubDigraphAdaptor in the way that only the arc-set
  ///can be filtered. The usefulness of this adaptor is demonstrated
  ///in the problem of searching a maximum number of arc-disjoint
  ///shortest paths between two nodes \c s and \c t. Shortest here
  ///means being shortest w.r.t.  non-negative arc-lengths. Note that
  ///the comprehension of the presented solution need's some
  ///elementary knowlarc from combinatorial optimization.
  ///
  ///If a single shortest path is to be searched between \c s and \c
  ///t, then this can be done easily by applying the Dijkstra
  ///algorithm. What happens, if a maximum number of arc-disjoint
  ///shortest paths is to be computed. It can be proved that an arc
  ///can be in a shortest path if and only if it is tight with respect
  ///to the potential function computed by Dijkstra.  Moreover, any
  ///path containing only such arcs is a shortest one.  Thus we have
  ///to compute a maximum number of arc-disjoint paths between \c s
  ///and \c t in the digraph which has arc-set all the tight arcs. The
  ///computation will be demonstrated on the following digraph, which
  ///is read from the dimacs file \c sub_digraph_adaptor_demo.dim.
  ///The full source code is available in \ref
  ///sub_digraph_adaptor_demo.cc.  If you are interested in more demo
  ///programs, you can use \ref dim_to_dot.cc to generate .dot files
  ///from dimacs files.  The .dot file of the following figure was
  ///generated by the demo program \ref dim_to_dot.cc.
  ///
  ///\dot
  ///didigraph lemon_dot_example {
  ///node [ shape=ellipse, fontname=Helvetica, fontsize=10 ];
  ///n0 [ label="0 (s)" ];
  ///n1 [ label="1" ];
  ///n2 [ label="2" ];
  ///n3 [ label="3" ];
  ///n4 [ label="4" ];
  ///n5 [ label="5" ];
  ///n6 [ label="6 (t)" ];
  ///arc [ shape=ellipse, fontname=Helvetica, fontsize=10 ];
  ///n5 ->  n6 [ label="9, length:4" ];
  ///n4 ->  n6 [ label="8, length:2" ];
  ///n3 ->  n5 [ label="7, length:1" ];
  ///n2 ->  n5 [ label="6, length:3" ];
  ///n2 ->  n6 [ label="5, length:5" ];
  ///n2 ->  n4 [ label="4, length:2" ];
  ///n1 ->  n4 [ label="3, length:3" ];
  ///n0 ->  n3 [ label="2, length:1" ];
  ///n0 ->  n2 [ label="1, length:2" ];
  ///n0 ->  n1 [ label="0, length:3" ];
  ///}
  ///\enddot
  ///
  ///\code
  ///Digraph g;
  ///Node s, t;
  ///LengthMap length(g);
  ///
  ///readDimacs(std::cin, g, length, s, t);
  ///
  ///cout << "arcs with lengths (of form id, source--length->target): " << endl;
  ///for(ArcIt e(g); e!=INVALID; ++e) 
  ///  cout << g.id(e) << ", " << g.id(g.source(e)) << "--" 
  ///       << length[e] << "->" << g.id(g.target(e)) << endl;
  ///
  ///cout << "s: " << g.id(s) << " t: " << g.id(t) << endl;
  ///\endcode
  ///Next, the potential function is computed with Dijkstra.
  ///\code
  ///typedef Dijkstra<Digraph, LengthMap> Dijkstra;
  ///Dijkstra dijkstra(g, length);
  ///dijkstra.run(s);
  ///\endcode
  ///Next, we consrtruct a map which filters the arc-set to the tight arcs.
  ///\code
  ///typedef TightArcFilterMap<Digraph, const Dijkstra::DistMap, LengthMap> 
  ///  TightArcFilter;
  ///TightArcFilter tight_arc_filter(g, dijkstra.distMap(), length);
  ///
  ///typedef ArcSubDigraphAdaptor<Digraph, TightArcFilter> SubGA;
  ///SubGA ga(g, tight_arc_filter);
  ///\endcode
  ///Then, the maximum nimber of arc-disjoint \c s-\c t paths are computed 
  ///with a max flow algorithm Preflow.
  ///\code
  ///ConstMap<Arc, int> const_1_map(1);
  ///Digraph::ArcMap<int> flow(g, 0);
  ///
  ///Preflow<SubGA, ConstMap<Arc, int>, Digraph::ArcMap<int> > 
  ///  preflow(ga, const_1_map, s, t);
  ///preflow.run();
  ///\endcode
  ///Last, the output is:
  ///\code  
  ///cout << "maximum number of arc-disjoint shortest path: " 
  ///     << preflow.flowValue() << endl;
  ///cout << "arcs of the maximum number of arc-disjoint shortest s-t paths: " 
  ///     << endl;
  ///for(ArcIt e(g); e!=INVALID; ++e) 
  ///  if (preflow.flow(e))
  ///    cout << " " << g.id(g.source(e)) << "--"
  ///         << length[e] << "->" << g.id(g.target(e)) << endl;
  ///\endcode
  ///The program has the following (expected :-)) output:
  ///\code
  ///arcs with lengths (of form id, source--length->target):
  /// 9, 5--4->6
  /// 8, 4--2->6
  /// 7, 3--1->5
  /// 6, 2--3->5
  /// 5, 2--5->6
  /// 4, 2--2->4
  /// 3, 1--3->4
  /// 2, 0--1->3
  /// 1, 0--2->2
  /// 0, 0--3->1
  ///s: 0 t: 6
  ///maximum number of arc-disjoint shortest path: 2
  ///arcs of the maximum number of arc-disjoint shortest s-t paths:
  /// 9, 5--4->6
  /// 8, 4--2->6
  /// 7, 3--1->5
  /// 4, 2--2->4
  /// 2, 0--1->3
  /// 1, 0--2->2
  ///\endcode
  template<typename _Digraph, typename _ArcFilterMap>
  class ArcSubDigraphAdaptor : 
    public SubDigraphAdaptor<_Digraph, ConstMap<typename _Digraph::Node, bool>, 
			     _ArcFilterMap, false> {
  public:
    typedef _Digraph Digraph;
    typedef _ArcFilterMap ArcFilterMap;

    typedef SubDigraphAdaptor<Digraph, ConstMap<typename Digraph::Node, bool>, 
			      ArcFilterMap, false> Parent;
  protected:
    ConstMap<typename Digraph::Node, bool> const_true_map;

    ArcSubDigraphAdaptor() : const_true_map(true) {
      Parent::setNodeFilterMap(const_true_map);
    }

  public:

    ArcSubDigraphAdaptor(Digraph& digraph, ArcFilterMap& arc_filter) 
      : Parent(), const_true_map(true) { 
      Parent::setDigraph(digraph);
      Parent::setNodeFilterMap(const_true_map);
      Parent::setArcFilterMap(arc_filter);
    }

  };

  /// \brief Just gives back an arc sub digraph adaptor
  ///
  /// Just gives back an arc sub digraph adaptor
  template<typename Digraph, typename ArcFilterMap>
  ArcSubDigraphAdaptor<const Digraph, ArcFilterMap>
  arcSubDigraphAdaptor(const Digraph& digraph, ArcFilterMap& afm) {
    return ArcSubDigraphAdaptor<const Digraph, ArcFilterMap>(digraph, afm);
  }

  template<typename Digraph, typename ArcFilterMap>
  ArcSubDigraphAdaptor<const Digraph, const ArcFilterMap>
  arcSubDigraphAdaptor(const Digraph& digraph, const ArcFilterMap& afm) {
    return ArcSubDigraphAdaptor<const Digraph, const ArcFilterMap>
      (digraph, afm);
  }

  template <typename _Digraph>
  class UndirDigraphAdaptorBase { 
  public:
    typedef _Digraph Digraph;
    typedef UndirDigraphAdaptorBase Adaptor;

    typedef True UndirectedTag;

    typedef typename Digraph::Arc Edge;
    typedef typename Digraph::Node Node;

    class Arc : public Edge {
      friend class UndirDigraphAdaptorBase;
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
    int nodeNum() const { return 2 * _digraph->arcNum(); }
    typedef EdgeNumTagIndicator<Digraph> EdgeNumTag;
    int arcNum() const { return 2 * _digraph->arcNum(); }
    int edgeNum() const { return _digraph->arcNum(); }

    typedef FindEdgeTagIndicator<Digraph> FindEdgeTag;
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

    Edge findEdge(Node s, Node t, Edge p = INVALID) const {
      if (s != t) {
        if (p == INVALID) {
          Edge arc = _digraph->findArc(s, t);
          if (arc != INVALID) return arc;
          arc = _digraph->findArc(t, s);
          if (arc != INVALID) return arc;
        } else if (_digraph->s(p) == s) {
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

      typename MapTraits<MapImpl>::ConstReturnValue 
      operator[](const Arc& a) const { 
	if (direction(a)) {
	  return _forward[a]; 
	} else { 
	  return _backward[a]; 
        }
      }

      typename MapTraits<MapImpl>::ReturnValue 
      operator[](const Arc& a) { 
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

  protected:

    UndirDigraphAdaptorBase() : _digraph(0) {}

    Digraph* _digraph;

    void setDigraph(Digraph& digraph) {
      _digraph = &digraph;
    }
    
  };

  ///\ingroup graph_adaptors
  ///
  /// \brief An graph is made from a directed digraph by an adaptor
  ///
  /// This adaptor makes an undirected graph from a directed
  /// digraph. All arc of the underlying will be showed in the adaptor
  /// as an edge. Let's see an informal example about using
  /// this adaptor:
  ///
  /// There is a network of the streets of a town. Of course there are
  /// some one-way street in the town hence the network is a directed
  /// one. There is a crazy driver who go oppositely in the one-way
  /// street without moral sense. Of course he can pass this streets
  /// slower than the regular way, in fact his speed is half of the
  /// normal speed. How long should he drive to get from a source
  /// point to the target? Let see the example code which calculate it:
  ///
  /// \todo BadCode, SimpleMap does no exists
  ///\code
  /// typedef UndirDigraphAdaptor<Digraph> Graph;
  /// Graph graph(digraph);
  ///
  /// typedef SimpleMap<LengthMap> FLengthMap;
  /// FLengthMap flength(length);
  ///
  /// typedef ScaleMap<LengthMap> RLengthMap;
  /// RLengthMap rlength(length, 2.0);
  ///
  /// typedef Graph::CombinedArcMap<FLengthMap, RLengthMap > ULengthMap;
  /// ULengthMap ulength(flength, rlength);
  /// 
  /// Dijkstra<Graph, ULengthMap> dijkstra(graph, ulength);
  /// std::cout << "Driving time : " << dijkstra.run(src, trg) << std::endl;
  ///\endcode
  ///
  /// The combined arc map makes the length map for the undirected
  /// graph. It is created from a forward and reverse map. The forward
  /// map is created from the original length map with a SimpleMap
  /// adaptor which just makes a read-write map from the reference map
  /// i.e. it forgets that it can be return reference to values. The
  /// reverse map is just the scaled original map with the ScaleMap
  /// adaptor. The combination solves that passing the reverse way
  /// takes double time than the original. To get the driving time we
  /// run the dijkstra algorithm on the graph.
  template<typename _Digraph>
  class UndirDigraphAdaptor 
    : public GraphAdaptorExtender<UndirDigraphAdaptorBase<_Digraph> > {
  public:
    typedef _Digraph Digraph;
    typedef GraphAdaptorExtender<UndirDigraphAdaptorBase<Digraph> > Parent;
  protected:
    UndirDigraphAdaptor() { }
  public:

    /// \brief Constructor
    ///
    /// Constructor
    UndirDigraphAdaptor(_Digraph& _digraph) { 
      setDigraph(_digraph);
    }

    /// \brief ArcMap combined from two original ArcMap
    ///
    /// This class adapts two original digraph ArcMap to
    /// get an arc map on the adaptor.
    template <typename _ForwardMap, typename _BackwardMap>
    class CombinedArcMap {
    public:
      
      typedef _ForwardMap ForwardMap;
      typedef _BackwardMap BackwardMap;

      typedef typename MapTraits<ForwardMap>::ReferenceMapTag ReferenceMapTag;

      typedef typename ForwardMap::Value Value;
      typedef typename Parent::Arc Key;

      /// \brief Constructor      
      ///
      /// Constructor      
      CombinedArcMap() : _forward(0), _backward(0) {}

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
      typename MapTraits<ForwardMap>::ConstReturnValue 
      operator[](const Key& e) const { 
	if (Parent::direction(e)) {
	  return (*_forward)[e]; 
	} else { 
	  return (*_backward)[e]; 
        }
      }

      /// \brief Returns the value associated with a key.
      ///
      /// Returns the value associated with a key.
      typename MapTraits<ForwardMap>::ReturnValue 
      operator[](const Key& e) { 
	if (Parent::direction(e)) {
	  return (*_forward)[e]; 
	} else { 
	  return (*_backward)[e]; 
        }
      }

      /// \brief Sets the forward map
      ///
      /// Sets the forward map
      void setForwardMap(ForwardMap& forward) {
        _forward = &forward;
      }

      /// \brief Sets the backward map
      ///
      /// Sets the backward map
      void setBackwardMap(BackwardMap& backward) {
        _backward = &backward;
      }

    protected:

      ForwardMap* _forward;
      BackwardMap* _backward; 

    };

  };

  /// \brief Just gives back an undir digraph adaptor
  ///
  /// Just gives back an undir digraph adaptor
  template<typename Digraph>
  UndirDigraphAdaptor<const Digraph>
  undirDigraphAdaptor(const Digraph& digraph) {
    return UndirDigraphAdaptor<const Digraph>(digraph);
  }

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

    ResForwardFilter(const Tolerance& tolerance = Tolerance()) 
      : _capacity(0), _flow(0), _tolerance(tolerance)  { }

    void setCapacity(const CapacityMap& capacity) { _capacity = &capacity; }
    void setFlow(const FlowMap& flow) { _flow = &flow; }

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
    ResBackwardFilter(const Tolerance& tolerance = Tolerance())
      : _capacity(0), _flow(0), _tolerance(tolerance) { }

    void setCapacity(const CapacityMap& capacity) { _capacity = &capacity; }
    void setFlow(const FlowMap& flow) { _flow = &flow; }

    bool operator[](const typename Digraph::Arc& a) const {
      return _tolerance.positive((*_flow)[a]);
    }
  };

  
  ///\ingroup graph_adaptors
  ///
  ///\brief An adaptor for composing the residual graph for directed
  ///flow and circulation problems.
  ///
  ///An adaptor for composing the residual graph for directed flow and
  ///circulation problems.  Let \f$ G=(V, A) \f$ be a directed digraph
  ///and let \f$ F \f$ be a number type. Let moreover \f$ f,c:A\to F
  ///\f$, be functions on the arc-set.
  ///
  ///In the appications of ResDigraphAdaptor, \f$ f \f$ usually stands
  ///for a flow and \f$ c \f$ for a capacity function.  Suppose that a
  ///graph instance \c g of type \c ListDigraph implements \f$ G \f$.
  ///
  ///\code 
  ///  ListDigraph g;
  ///\endcode 
  ///
  ///Then ResDigraphAdaptor implements the digraph structure with
  /// node-set \f$ V \f$ and arc-set \f$ A_{forward}\cup A_{backward}
  /// \f$, where \f$ A_{forward}=\{uv : uv\in A, f(uv)<c(uv)\} \f$ and
  /// \f$ A_{backward}=\{vu : uv\in A, f(uv)>0\} \f$, i.e. the so
  /// called residual graph.  When we take the union \f$
  /// A_{forward}\cup A_{backward} \f$, multilicities are counted,
  /// i.e.  if an arc is in both \f$ A_{forward} \f$ and \f$
  /// A_{backward} \f$, then in the adaptor it appears twice. The
  /// following code shows how such an instance can be constructed.
  ///
  ///\code 
  ///  typedef ListDigraph Digraph; 
  ///  IntArcMap f(g), c(g);
  ///  ResDigraphAdaptor<Digraph, int, IntArcMap, IntArcMap> ga(g); 
  ///\endcode
  template<typename _Digraph, 
	   typename _CapacityMap = typename _Digraph::template ArcMap<int>, 
	   typename _FlowMap = _CapacityMap,
           typename _Tolerance = Tolerance<typename _CapacityMap::Value> >
  class ResDigraphAdaptor : 
    public ArcSubDigraphAdaptor< 
    UndirDigraphAdaptor<const _Digraph>, 
    typename UndirDigraphAdaptor<const _Digraph>::template CombinedArcMap<
      ResForwardFilter<const _Digraph, _CapacityMap, _FlowMap>,  
      ResBackwardFilter<const _Digraph, _CapacityMap, _FlowMap> > > {
  public:

    typedef _Digraph Digraph;
    typedef _CapacityMap CapacityMap;
    typedef _FlowMap FlowMap;
    typedef _Tolerance Tolerance;

    typedef typename CapacityMap::Value Value;
    typedef ResDigraphAdaptor Adaptor;

  protected:

    typedef UndirDigraphAdaptor<const Digraph> UndirDigraph;

    typedef ResForwardFilter<const Digraph, CapacityMap, FlowMap> 
    ForwardFilter;

    typedef ResBackwardFilter<const Digraph, CapacityMap, FlowMap> 
    BackwardFilter;

    typedef typename UndirDigraph::
    template CombinedArcMap<ForwardFilter, BackwardFilter> ArcFilter;

    typedef ArcSubDigraphAdaptor<UndirDigraph, ArcFilter> Parent;

    const CapacityMap* _capacity;
    FlowMap* _flow;

    UndirDigraph _graph;
    ForwardFilter _forward_filter;
    BackwardFilter _backward_filter;
    ArcFilter _arc_filter;

    void setCapacityMap(const CapacityMap& capacity) {
      _capacity = &capacity;
      _forward_filter.setCapacity(capacity);
      _backward_filter.setCapacity(capacity);
    }

    void setFlowMap(FlowMap& flow) {
      _flow = &flow;
      _forward_filter.setFlow(flow);
      _backward_filter.setFlow(flow);
    }

  public:

    /// \brief Constructor of the residual digraph.
    ///
    /// Constructor of the residual graph. The parameters are the digraph type,
    /// the flow map, the capacity map and a tolerance object.
    ResDigraphAdaptor(const Digraph& digraph, const CapacityMap& capacity, 
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
    Value rescap(const Arc& arc) const {
      if (UndirDigraph::direction(arc)) {
        return (*_capacity)[arc] - (*_flow)[arc]; 
      } else {
        return (*_flow)[arc];
      }
    } 

    /// \brief Augment on the given arc in the residual digraph.
    ///
    /// Augment on the given arc in the residual digraph. It increase
    /// or decrease the flow on the original arc depend on the direction
    /// of the residual arc.
    void augment(const Arc& e, const Value& a) const {
      if (UndirDigraph::direction(e)) {
        _flow->set(e, (*_flow)[e] + a);
      } else {  
        _flow->set(e, (*_flow)[e] - a);
      }
    }

    /// \brief Returns the direction of the arc.
    ///
    /// Returns true when the arc is same oriented as the original arc.
    static bool forward(const Arc& e) {
      return UndirDigraph::direction(e);
    }

    /// \brief Returns the direction of the arc.
    ///
    /// Returns true when the arc is opposite oriented as the original arc.
    static bool backward(const Arc& e) {
      return !UndirDigraph::direction(e);
    }

    /// \brief Gives back the forward oriented residual arc.
    ///
    /// Gives back the forward oriented residual arc.
    static Arc forward(const typename Digraph::Arc& e) {
      return UndirDigraph::direct(e, true);
    }

    /// \brief Gives back the backward oriented residual arc.
    ///
    /// Gives back the backward oriented residual arc.
    static Arc backward(const typename Digraph::Arc& e) {
      return UndirDigraph::direct(e, false);
    }

    /// \brief Residual capacity map.
    ///
    /// In generic residual digraphs the residual capacity can be obtained 
    /// as a map. 
    class ResCap {
    protected:
      const Adaptor* _adaptor;
    public:
      typedef Arc Key;
      typedef typename _CapacityMap::Value Value;

      ResCap(const Adaptor& adaptor) : _adaptor(&adaptor) {}
      
      Value operator[](const Arc& e) const {
        return _adaptor->rescap(e);
      }
      
    };

  };

  /// \brief Base class for split digraph adaptor
  ///
  /// Base class of split digraph adaptor. In most case you do not need to
  /// use it directly but the documented member functions of this class can 
  /// be used with the SplitDigraphAdaptor class.
  /// \sa SplitDigraphAdaptor
  template <typename _Digraph>
  class SplitDigraphAdaptorBase {
  public:

    typedef _Digraph Digraph;
    typedef DigraphAdaptorBase<const _Digraph> Parent;
    typedef SplitDigraphAdaptorBase Adaptor;

    typedef typename Digraph::Node DigraphNode;
    typedef typename Digraph::Arc DigraphArc;

    class Node;
    class Arc;

  private:

    template <typename T> class NodeMapBase;
    template <typename T> class ArcMapBase;

  public:
    
    class Node : public DigraphNode {
      friend class SplitDigraphAdaptorBase;
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
      friend class SplitDigraphAdaptorBase;
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

    /// \brief Returns true when the node is in-node.
    ///
    /// Returns true when the node is in-node.
    static bool inNode(const Node& n) {
      return n._in;
    }

    /// \brief Returns true when the node is out-node.
    ///
    /// Returns true when the node is out-node.
    static bool outNode(const Node& n) {
      return !n._in;
    }

    /// \brief Returns true when the arc is arc in the original digraph.
    ///
    /// Returns true when the arc is arc in the original digraph.
    static bool origArc(const Arc& e) {
      return e._item.firstState();
    }

    /// \brief Returns true when the arc binds an in-node and an out-node.
    ///
    /// Returns true when the arc binds an in-node and an out-node.
    static bool bindArc(const Arc& e) {
      return e._item.secondState();
    }

    /// \brief Gives back the in-node created from the \c node.
    ///
    /// Gives back the in-node created from the \c node.
    static Node inNode(const DigraphNode& n) {
      return Node(n, true);
    }

    /// \brief Gives back the out-node created from the \c node.
    ///
    /// Gives back the out-node created from the \c node.
    static Node outNode(const DigraphNode& n) {
      return Node(n, false);
    }

    /// \brief Gives back the arc binds the two part of the node.
    /// 
    /// Gives back the arc binds the two part of the node.
    static Arc arc(const DigraphNode& n) {
      return Arc(n);
    }

    /// \brief Gives back the arc of the original arc.
    /// 
    /// Gives back the arc of the original arc.
    static Arc arc(const DigraphArc& e) {
      return Arc(e);
    }

    typedef True NodeNumTag;

    int nodeNum() const {
      return  2 * countNodes(*_digraph);
    }

    typedef True EdgeNumTag;
    int arcNum() const {
      return countArcs(*_digraph) + countNodes(*_digraph);
    }

    typedef True FindEdgeTag;
    Arc findArc(const Node& u, const Node& v, 
		const Arc& prev = INVALID) const {
      if (inNode(u)) {
        if (outNode(v)) {
          if (static_cast<const DigraphNode&>(u) == 
              static_cast<const DigraphNode&>(v) && prev == INVALID) {
            return Arc(u);
          }
        }
      } else {
        if (inNode(v)) {
          return Arc(::lemon::findArc(*_digraph, u, v, prev));
        }
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
      
      NodeMapBase(const Adaptor& adaptor) 
	: _in_map(*adaptor._digraph), _out_map(*adaptor._digraph) {}
      NodeMapBase(const Adaptor& adaptor, const Value& value) 
	: _in_map(*adaptor._digraph, value), 
	  _out_map(*adaptor._digraph, value) {}

      void set(const Node& key, const Value& val) {
	if (Adaptor::inNode(key)) { _in_map.set(key, val); }
	else {_out_map.set(key, val); }
      }
      
      typename MapTraits<NodeImpl>::ReturnValue 
      operator[](const Node& key) {
	if (Adaptor::inNode(key)) { return _in_map[key]; }
	else { return _out_map[key]; }
      }

      typename MapTraits<NodeImpl>::ConstReturnValue
      operator[](const Node& key) const {
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
      
      typename MapTraits<ArcImpl>::ReturnValue
      operator[](const Arc& key) {
	if (Adaptor::origArc(key)) { 
          return _arc_map[key._item.first()];
        } else {
          return _node_map[key._item.second()];
        }
      }

      typename MapTraits<ArcImpl>::ConstReturnValue
      operator[](const Arc& key) const {
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

    SplitDigraphAdaptorBase() : _digraph(0) {}

    Digraph* _digraph;

    void setDigraph(Digraph& digraph) {
      _digraph = &digraph;
    }
    
  };

  /// \ingroup graph_adaptors
  ///
  /// \brief Split digraph adaptor class
  /// 
  /// This is an digraph adaptor which splits all node into an in-node
  /// and an out-node. Formaly, the adaptor replaces each \f$ u \f$
  /// node in the digraph with two node, \f$ u_{in} \f$ node and 
  /// \f$ u_{out} \f$ node. If there is an \f$ (v, u) \f$ arc in the 
  /// original digraph the new target of the arc will be \f$ u_{in} \f$ and
  /// similarly the source of the original \f$ (u, v) \f$ arc will be
  /// \f$ u_{out} \f$.  The adaptor will add for each node in the 
  /// original digraph an additional arc which will connect 
  /// \f$ (u_{in}, u_{out}) \f$.
  ///
  /// The aim of this class is to run algorithm with node costs if the 
  /// algorithm can use directly just arc costs. In this case we should use
  /// a \c SplitDigraphAdaptor and set the node cost of the digraph to the
  /// bind arc in the adapted digraph.
  /// 
  /// By example a maximum flow algoritm can compute how many arc
  /// disjoint paths are in the digraph. But we would like to know how
  /// many node disjoint paths are in the digraph. First we have to
  /// adapt the digraph with the \c SplitDigraphAdaptor. Then run the flow
  /// algorithm on the adapted digraph. The bottleneck of the flow will
  /// be the bind arcs which bounds the flow with the count of the
  /// node disjoint paths.
  ///
  ///\code
  ///
  /// typedef SplitDigraphAdaptor<SmartDigraph> SDigraph;
  ///
  /// SDigraph sdigraph(digraph);
  ///
  /// typedef ConstMap<SDigraph::Arc, int> SCapacity;
  /// SCapacity scapacity(1);
  ///
  /// SDigraph::ArcMap<int> sflow(sdigraph);
  ///
  /// Preflow<SDigraph, SCapacity> 
  ///   spreflow(sdigraph, scapacity, 
  ///            SDigraph::outNode(source), SDigraph::inNode(target));
  ///                                            
  /// spreflow.run();
  ///
  ///\endcode
  ///
  /// The result of the mamixum flow on the original digraph
  /// shows the next figure:
  ///
  /// \image html arc_disjoint.png
  /// \image latex arc_disjoint.eps "Arc disjoint paths" width=\textwidth
  /// 
  /// And the maximum flow on the adapted digraph:
  ///
  /// \image html node_disjoint.png
  /// \image latex node_disjoint.eps "Node disjoint paths" width=\textwidth
  ///
  /// The second solution contains just 3 disjoint paths while the first 4.
  /// The full code can be found in the \ref disjoint_paths_demo.cc demo file.
  ///
  /// This digraph adaptor is fully conform to the 
  /// \ref concepts::Digraph "Digraph" concept and
  /// contains some additional member functions and types. The 
  /// documentation of some member functions may be found just in the
  /// SplitDigraphAdaptorBase class.
  ///
  /// \sa SplitDigraphAdaptorBase
  template <typename _Digraph>
  class SplitDigraphAdaptor 
    : public DigraphAdaptorExtender<SplitDigraphAdaptorBase<_Digraph> > {
  public:
    typedef _Digraph Digraph;
    typedef DigraphAdaptorExtender<SplitDigraphAdaptorBase<Digraph> > Parent;

    typedef typename Parent::Node Node;
    typedef typename Parent::Arc Arc;

    /// \brief Constructor of the adaptor.
    ///
    /// Constructor of the adaptor.
    SplitDigraphAdaptor(Digraph& g) {
      Parent::setDigraph(g);
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


    /// \brief Just gives back a combined node map.
    /// 
    /// Just gives back a combined node map.
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

    /// \brief ArcMap combined from an original ArcMap and NodeMap
    ///
    /// This class adapt an original digraph ArcMap and NodeMap to
    /// get an arc map on the adapted digraph.
    template <typename DigraphArcMap, typename DigraphNodeMap>
    class CombinedArcMap {
    public:
      
      typedef Arc Key;
      typedef typename DigraphArcMap::Value Value;
      
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
                    
    /// \brief Just gives back a combined arc map.
    /// 
    /// Just gives back a combined arc map.
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

  /// \brief Just gives back a split digraph adaptor
  ///
  /// Just gives back a split digraph adaptor
  template<typename Digraph>
  SplitDigraphAdaptor<Digraph>
  splitDigraphAdaptor(const Digraph& digraph) {
    return SplitDigraphAdaptor<Digraph>(digraph);
  }


} //namespace lemon

#endif //LEMON_DIGRAPH_ADAPTOR_H

