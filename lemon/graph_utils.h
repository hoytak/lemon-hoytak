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

#ifndef LEMON_GRAPH_UTILS_H
#define LEMON_GRAPH_UTILS_H

#include <iterator>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

#include <lemon/bits/invalid.h>
#include <lemon/bits/utility.h>
#include <lemon/maps.h>
#include <lemon/bits/traits.h>

#include <lemon/bits/alteration_notifier.h>
#include <lemon/bits/default_map.h>

///\ingroup gutils
///\file
///\brief Digraph utilities.

namespace lemon {

  /// \addtogroup gutils
  /// @{

  ///Creates convenience typedefs for the digraph types and iterators

  ///This \c \#define creates convenience typedefs for the following types
  ///of \c Digraph: \c Node,  \c NodeIt, \c Arc, \c ArcIt, \c InArcIt,
  ///\c OutArcIt
  ///\note If \c G it a template parameter, it should be used in this way.
  ///\code
  ///  GRAPH_TYPEDEFS(typename G);
  ///\endcode
  ///
  ///\warning There are no typedefs for the digraph maps because of the lack of
  ///template typedefs in C++.
#define GRAPH_TYPEDEFS(Digraph)				\
  typedef Digraph::     Node      Node;			\
    typedef Digraph::   NodeIt    NodeIt;			\
    typedef Digraph::   Arc      Arc;			\
    typedef Digraph::   ArcIt    ArcIt;			\
    typedef Digraph:: InArcIt  InArcIt;			\
    typedef Digraph::OutArcIt OutArcIt

  ///Creates convenience typedefs for the graph types and iterators

  ///This \c \#define creates the same convenience typedefs as defined by
  ///\ref GRAPH_TYPEDEFS(Digraph) and three more, namely it creates
  ///\c Edge, \c EdgeIt, \c IncArcIt,
  ///
  ///\note If \c G it a template parameter, it should be used in this way.
  ///\code
  ///  UGRAPH_TYPEDEFS(typename G);
  ///\endcode
  ///
  ///\warning There are no typedefs for the digraph maps because of the lack of
  ///template typedefs in C++.
#define UGRAPH_TYPEDEFS(Digraph)				\
  GRAPH_TYPEDEFS(Digraph);				\
    typedef Digraph:: Edge   Edge;			\
    typedef Digraph:: EdgeIt EdgeIt;			\
    typedef Digraph:: IncArcIt   IncArcIt

  ///\brief Creates convenience typedefs for the bipartite digraph 
  ///types and iterators

  ///This \c \#define creates the same convenience typedefs as defined by
  ///\ref UGRAPH_TYPEDEFS(Digraph) and two more, namely it creates
  ///\c RedIt, \c BlueIt, 
  ///
  ///\note If \c G it a template parameter, it should be used in this way.
  ///\code
  ///  BPUGRAPH_TYPEDEFS(typename G);
  ///\endcode
  ///
  ///\warning There are no typedefs for the digraph maps because of the lack of
  ///template typedefs in C++.
#define BPUGRAPH_TYPEDEFS(Digraph)            \
  UGRAPH_TYPEDEFS(Digraph);		    \
    typedef Digraph::Red Red;             \
    typedef Digraph::Blue Blue;             \
    typedef Digraph::RedIt RedIt;	    \
    typedef Digraph::BlueIt BlueIt

  /// \brief Function to count the items in the digraph.
  ///
  /// This function counts the items (nodes, arcs etc) in the digraph.
  /// The complexity of the function is O(n) because
  /// it iterates on all of the items.

  template <typename Digraph, typename Item>
  inline int countItems(const Digraph& g) {
    typedef typename ItemSetTraits<Digraph, Item>::ItemIt ItemIt;
    int num = 0;
    for (ItemIt it(g); it != INVALID; ++it) {
      ++num;
    }
    return num;
  }

  // Node counting:

  namespace _digraph_utils_bits {
    
    template <typename Digraph, typename Enable = void>
    struct CountNodesSelector {
      static int count(const Digraph &g) {
        return countItems<Digraph, typename Digraph::Node>(g);
      }
    };

    template <typename Digraph>
    struct CountNodesSelector<
      Digraph, typename 
      enable_if<typename Digraph::NodeNumTag, void>::type> 
    {
      static int count(const Digraph &g) {
        return g.nodeNum();
      }
    };    
  }

  /// \brief Function to count the nodes in the digraph.
  ///
  /// This function counts the nodes in the digraph.
  /// The complexity of the function is O(n) but for some
  /// digraph structures it is specialized to run in O(1).
  ///
  /// If the digraph contains a \e nodeNum() member function and a 
  /// \e NodeNumTag tag then this function calls directly the member
  /// function to query the cardinality of the node set.
  template <typename Digraph>
  inline int countNodes(const Digraph& g) {
    return _digraph_utils_bits::CountNodesSelector<Digraph>::count(g);
  }

  namespace _digraph_utils_bits {
    
    template <typename Digraph, typename Enable = void>
    struct CountRedsSelector {
      static int count(const Digraph &g) {
        return countItems<Digraph, typename Digraph::Red>(g);
      }
    };

    template <typename Digraph>
    struct CountRedsSelector<
      Digraph, typename 
      enable_if<typename Digraph::NodeNumTag, void>::type> 
    {
      static int count(const Digraph &g) {
        return g.redNum();
      }
    };    
  }

  /// \brief Function to count the reds in the digraph.
  ///
  /// This function counts the reds in the digraph.
  /// The complexity of the function is O(an) but for some
  /// digraph structures it is specialized to run in O(1).
  ///
  /// If the digraph contains an \e redNum() member function and a 
  /// \e NodeNumTag tag then this function calls directly the member
  /// function to query the cardinality of the A-node set.
  template <typename Digraph>
  inline int countReds(const Digraph& g) {
    return _digraph_utils_bits::CountRedsSelector<Digraph>::count(g);
  }

  namespace _digraph_utils_bits {
    
    template <typename Digraph, typename Enable = void>
    struct CountBluesSelector {
      static int count(const Digraph &g) {
        return countItems<Digraph, typename Digraph::Blue>(g);
      }
    };

    template <typename Digraph>
    struct CountBluesSelector<
      Digraph, typename 
      enable_if<typename Digraph::NodeNumTag, void>::type> 
    {
      static int count(const Digraph &g) {
        return g.blueNum();
      }
    };    
  }

  /// \brief Function to count the blues in the digraph.
  ///
  /// This function counts the blues in the digraph.
  /// The complexity of the function is O(bn) but for some
  /// digraph structures it is specialized to run in O(1).
  ///
  /// If the digraph contains a \e blueNum() member function and a 
  /// \e NodeNumTag tag then this function calls directly the member
  /// function to query the cardinality of the B-node set.
  template <typename Digraph>
  inline int countBlues(const Digraph& g) {
    return _digraph_utils_bits::CountBluesSelector<Digraph>::count(g);
  }


  // Arc counting:

  namespace _digraph_utils_bits {
    
    template <typename Digraph, typename Enable = void>
    struct CountArcsSelector {
      static int count(const Digraph &g) {
        return countItems<Digraph, typename Digraph::Arc>(g);
      }
    };

    template <typename Digraph>
    struct CountArcsSelector<
      Digraph, 
      typename enable_if<typename Digraph::ArcNumTag, void>::type> 
    {
      static int count(const Digraph &g) {
        return g.arcNum();
      }
    };    
  }

  /// \brief Function to count the arcs in the digraph.
  ///
  /// This function counts the arcs in the digraph.
  /// The complexity of the function is O(e) but for some
  /// digraph structures it is specialized to run in O(1).
  ///
  /// If the digraph contains a \e arcNum() member function and a 
  /// \e ArcNumTag tag then this function calls directly the member
  /// function to query the cardinality of the arc set.
  template <typename Digraph>
  inline int countArcs(const Digraph& g) {
    return _digraph_utils_bits::CountArcsSelector<Digraph>::count(g);
  }

  // Undirected arc counting:
  namespace _digraph_utils_bits {
    
    template <typename Digraph, typename Enable = void>
    struct CountEdgesSelector {
      static int count(const Digraph &g) {
        return countItems<Digraph, typename Digraph::Edge>(g);
      }
    };

    template <typename Digraph>
    struct CountEdgesSelector<
      Digraph, 
      typename enable_if<typename Digraph::ArcNumTag, void>::type> 
    {
      static int count(const Digraph &g) {
        return g.edgeNum();
      }
    };    
  }

  /// \brief Function to count the edges in the digraph.
  ///
  /// This function counts the edges in the digraph.
  /// The complexity of the function is O(e) but for some
  /// digraph structures it is specialized to run in O(1).
  ///
  /// If the digraph contains a \e edgeNum() member function and a 
  /// \e ArcNumTag tag then this function calls directly the member
  /// function to query the cardinality of the edge set.
  template <typename Digraph>
  inline int countEdges(const Digraph& g) {
    return _digraph_utils_bits::CountEdgesSelector<Digraph>::count(g);

  }


  template <typename Digraph, typename DegIt>
  inline int countNodeDegree(const Digraph& _g, const typename Digraph::Node& _n) {
    int num = 0;
    for (DegIt it(_g, _n); it != INVALID; ++it) {
      ++num;
    }
    return num;
  }

  /// \brief Function to count the number of the out-arcs from node \c n.
  ///
  /// This function counts the number of the out-arcs from node \c n
  /// in the digraph.  
  template <typename Digraph>
  inline int countOutArcs(const Digraph& _g,  const typename Digraph::Node& _n) {
    return countNodeDegree<Digraph, typename Digraph::OutArcIt>(_g, _n);
  }

  /// \brief Function to count the number of the in-arcs to node \c n.
  ///
  /// This function counts the number of the in-arcs to node \c n
  /// in the digraph.  
  template <typename Digraph>
  inline int countInArcs(const Digraph& _g,  const typename Digraph::Node& _n) {
    return countNodeDegree<Digraph, typename Digraph::InArcIt>(_g, _n);
  }

  /// \brief Function to count the number of the inc-arcs to node \c n.
  ///
  /// This function counts the number of the inc-arcs to node \c n
  /// in the digraph.  
  template <typename Digraph>
  inline int countIncArcs(const Digraph& _g,  const typename Digraph::Node& _n) {
    return countNodeDegree<Digraph, typename Digraph::IncArcIt>(_g, _n);
  }

  namespace _digraph_utils_bits {
    
    template <typename Digraph, typename Enable = void>
    struct FindArcSelector {
      typedef typename Digraph::Node Node;
      typedef typename Digraph::Arc Arc;
      static Arc find(const Digraph &g, Node u, Node v, Arc e) {
        if (e == INVALID) {
          g.firstOut(e, u);
        } else {
          g.nextOut(e);
        }
        while (e != INVALID && g.target(e) != v) {
          g.nextOut(e);
        }
        return e;
      }
    };

    template <typename Digraph>
    struct FindArcSelector<
      Digraph, 
      typename enable_if<typename Digraph::FindArcTag, void>::type> 
    {
      typedef typename Digraph::Node Node;
      typedef typename Digraph::Arc Arc;
      static Arc find(const Digraph &g, Node u, Node v, Arc prev) {
        return g.findArc(u, v, prev);
      }
    };    
  }

  /// \brief Finds an arc between two nodes of a digraph.
  ///
  /// Finds an arc from node \c u to node \c v in digraph \c g.
  ///
  /// If \c prev is \ref INVALID (this is the default value), then
  /// it finds the first arc from \c u to \c v. Otherwise it looks for
  /// the next arc from \c u to \c v after \c prev.
  /// \return The found arc or \ref INVALID if there is no such an arc.
  ///
  /// Thus you can iterate through each arc from \c u to \c v as it follows.
  ///\code
  /// for(Arc e=findArc(g,u,v);e!=INVALID;e=findArc(g,u,v,e)) {
  ///   ...
  /// }
  ///\endcode
  ///
  ///\sa ArcLookUp
  ///\sa AllArcLookUp
  ///\sa DynArcLookUp
  ///\sa ConArcIt
  template <typename Digraph>
  inline typename Digraph::Arc 
  findArc(const Digraph &g, typename Digraph::Node u, typename Digraph::Node v,
           typename Digraph::Arc prev = INVALID) {
    return _digraph_utils_bits::FindArcSelector<Digraph>::find(g, u, v, prev);
  }

  /// \brief Iterator for iterating on arcs connected the same nodes.
  ///
  /// Iterator for iterating on arcs connected the same nodes. It is 
  /// higher level interface for the findArc() function. You can
  /// use it the following way:
  ///\code
  /// for (ConArcIt<Digraph> it(g, src, trg); it != INVALID; ++it) {
  ///   ...
  /// }
  ///\endcode
  /// 
  ///\sa findArc()
  ///\sa ArcLookUp
  ///\sa AllArcLookUp
  ///\sa DynArcLookUp
  ///
  /// \author Balazs Dezso 
  template <typename _Digraph>
  class ConArcIt : public _Digraph::Arc {
  public:

    typedef _Digraph Digraph;
    typedef typename Digraph::Arc Parent;

    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::Node Node;

    /// \brief Constructor.
    ///
    /// Construct a new ConArcIt iterating on the arcs which
    /// connects the \c u and \c v node.
    ConArcIt(const Digraph& g, Node u, Node v) : digraph(g) {
      Parent::operator=(findArc(digraph, u, v));
    }

    /// \brief Constructor.
    ///
    /// Construct a new ConArcIt which continues the iterating from 
    /// the \c e arc.
    ConArcIt(const Digraph& g, Arc e) : Parent(e), digraph(g) {}
    
    /// \brief Increment operator.
    ///
    /// It increments the iterator and gives back the next arc.
    ConArcIt& operator++() {
      Parent::operator=(findArc(digraph, digraph.source(*this), 
				 digraph.target(*this), *this));
      return *this;
    }
  private:
    const Digraph& digraph;
  };

  namespace _digraph_utils_bits {
    
    template <typename Digraph, typename Enable = void>
    struct FindEdgeSelector {
      typedef typename Digraph::Node Node;
      typedef typename Digraph::Edge Edge;
      static Edge find(const Digraph &g, Node u, Node v, Edge e) {
        bool b;
        if (u != v) {
          if (e == INVALID) {
            g.firstInc(e, b, u);
          } else {
            b = g.source(e) == u;
            g.nextInc(e, b);
          }
          while (e != INVALID && (b ? g.target(e) : g.source(e)) != v) {
            g.nextInc(e, b);
          }
        } else {
          if (e == INVALID) {
            g.firstInc(e, b, u);
          } else {
            b = true;
            g.nextInc(e, b);
          }
          while (e != INVALID && (!b || g.target(e) != v)) {
            g.nextInc(e, b);
          }
        }
        return e;
      }
    };

    template <typename Digraph>
    struct FindEdgeSelector<
      Digraph, 
      typename enable_if<typename Digraph::FindArcTag, void>::type> 
    {
      typedef typename Digraph::Node Node;
      typedef typename Digraph::Edge Edge;
      static Edge find(const Digraph &g, Node u, Node v, Edge prev) {
        return g.findEdge(u, v, prev);
      }
    };    
  }

  /// \brief Finds an edge between two nodes of a digraph.
  ///
  /// Finds an edge from node \c u to node \c v in digraph \c g.
  /// If the node \c u and node \c v is equal then each loop arc
  /// will be enumerated.
  ///
  /// If \c prev is \ref INVALID (this is the default value), then
  /// it finds the first arc from \c u to \c v. Otherwise it looks for
  /// the next arc from \c u to \c v after \c prev.
  /// \return The found arc or \ref INVALID if there is no such an arc.
  ///
  /// Thus you can iterate through each arc from \c u to \c v as it follows.
  ///\code
  /// for(Edge e = findEdge(g,u,v); e != INVALID; 
  ///     e = findEdge(g,u,v,e)) {
  ///   ...
  /// }
  ///\endcode
  ///
  ///\sa ConArcIt

  template <typename Digraph>
  inline typename Digraph::Edge 
  findEdge(const Digraph &g, typename Digraph::Node u, typename Digraph::Node v,
            typename Digraph::Edge p = INVALID) {
    return _digraph_utils_bits::FindEdgeSelector<Digraph>::find(g, u, v, p);
  }

  /// \brief Iterator for iterating on edges connected the same nodes.
  ///
  /// Iterator for iterating on edges connected the same nodes. It is 
  /// higher level interface for the findEdge() function. You can
  /// use it the following way:
  ///\code
  /// for (ConEdgeIt<Digraph> it(g, src, trg); it != INVALID; ++it) {
  ///   ...
  /// }
  ///\endcode
  ///
  ///\sa findEdge()
  ///
  /// \author Balazs Dezso 
  template <typename _Digraph>
  class ConEdgeIt : public _Digraph::Edge {
  public:

    typedef _Digraph Digraph;
    typedef typename Digraph::Edge Parent;

    typedef typename Digraph::Edge Edge;
    typedef typename Digraph::Node Node;

    /// \brief Constructor.
    ///
    /// Construct a new ConEdgeIt iterating on the arcs which
    /// connects the \c u and \c v node.
    ConEdgeIt(const Digraph& g, Node u, Node v) : digraph(g) {
      Parent::operator=(findEdge(digraph, u, v));
    }

    /// \brief Constructor.
    ///
    /// Construct a new ConEdgeIt which continues the iterating from 
    /// the \c e arc.
    ConEdgeIt(const Digraph& g, Edge e) : Parent(e), digraph(g) {}
    
    /// \brief Increment operator.
    ///
    /// It increments the iterator and gives back the next arc.
    ConEdgeIt& operator++() {
      Parent::operator=(findEdge(digraph, digraph.source(*this), 
				      digraph.target(*this), *this));
      return *this;
    }
  private:
    const Digraph& digraph;
  };

  /// \brief Copy a map.
  ///
  /// This function copies the \c from map to the \c to map. It uses the
  /// given iterator to iterate on the data structure and it uses the \c ref
  /// mapping to convert the from's keys to the to's keys.
  template <typename To, typename From, 
	    typename ItemIt, typename Ref>	    
  void copyMap(To& to, const From& from, 
	       ItemIt it, const Ref& ref) {
    for (; it != INVALID; ++it) {
      to[ref[it]] = from[it];
    }
  }

  /// \brief Copy the from map to the to map.
  ///
  /// Copy the \c from map to the \c to map. It uses the given iterator
  /// to iterate on the data structure.
  template <typename To, typename From, typename ItemIt>	    
  void copyMap(To& to, const From& from, ItemIt it) {
    for (; it != INVALID; ++it) {
      to[it] = from[it];
    }
  }

  namespace _digraph_utils_bits {

    template <typename Digraph, typename Item, typename RefMap>
    class MapCopyBase {
    public:
      virtual void copy(const Digraph& from, const RefMap& refMap) = 0;
      
      virtual ~MapCopyBase() {}
    };

    template <typename Digraph, typename Item, typename RefMap, 
              typename ToMap, typename FromMap>
    class MapCopy : public MapCopyBase<Digraph, Item, RefMap> {
    public:

      MapCopy(ToMap& tmap, const FromMap& map) 
        : _tmap(tmap), _map(map) {}
      
      virtual void copy(const Digraph& digraph, const RefMap& refMap) {
        typedef typename ItemSetTraits<Digraph, Item>::ItemIt ItemIt;
        for (ItemIt it(digraph); it != INVALID; ++it) {
          _tmap.set(refMap[it], _map[it]);
        }
      }

    private:
      ToMap& _tmap;
      const FromMap& _map;
    };

    template <typename Digraph, typename Item, typename RefMap, typename It>
    class ItemCopy : public MapCopyBase<Digraph, Item, RefMap> {
    public:

      ItemCopy(It& it, const Item& item) : _it(it), _item(item) {}
      
      virtual void copy(const Digraph&, const RefMap& refMap) {
        _it = refMap[_item];
      }

    private:
      It& _it;
      Item _item;
    };

    template <typename Digraph, typename Item, typename RefMap, typename Ref>
    class RefCopy : public MapCopyBase<Digraph, Item, RefMap> {
    public:

      RefCopy(Ref& map) : _map(map) {}
      
      virtual void copy(const Digraph& digraph, const RefMap& refMap) {
        typedef typename ItemSetTraits<Digraph, Item>::ItemIt ItemIt;
        for (ItemIt it(digraph); it != INVALID; ++it) {
          _map.set(it, refMap[it]);
        }
      }

    private:
      Ref& _map;
    };

    template <typename Digraph, typename Item, typename RefMap, 
              typename CrossRef>
    class CrossRefCopy : public MapCopyBase<Digraph, Item, RefMap> {
    public:

      CrossRefCopy(CrossRef& cmap) : _cmap(cmap) {}
      
      virtual void copy(const Digraph& digraph, const RefMap& refMap) {
        typedef typename ItemSetTraits<Digraph, Item>::ItemIt ItemIt;
        for (ItemIt it(digraph); it != INVALID; ++it) {
          _cmap.set(refMap[it], it);
        }
      }

    private:
      CrossRef& _cmap;
    };

    template <typename Digraph, typename Enable = void>
    struct DigraphCopySelector {
      template <typename From, typename NodeRefMap, typename ArcRefMap>
      static void copy(Digraph &to, const From& from,
                       NodeRefMap& nodeRefMap, ArcRefMap& arcRefMap) {
        for (typename From::NodeIt it(from); it != INVALID; ++it) {
          nodeRefMap[it] = to.addNode();
        }
        for (typename From::ArcIt it(from); it != INVALID; ++it) {
          arcRefMap[it] = to.addArc(nodeRefMap[from.source(it)], 
                                          nodeRefMap[from.target(it)]);
        }
      }
    };

    template <typename Digraph>
    struct DigraphCopySelector<
      Digraph, 
      typename enable_if<typename Digraph::BuildTag, void>::type> 
    {
      template <typename From, typename NodeRefMap, typename ArcRefMap>
      static void copy(Digraph &to, const From& from,
                       NodeRefMap& nodeRefMap, ArcRefMap& arcRefMap) {
        to.build(from, nodeRefMap, arcRefMap);
      }
    };

    template <typename Graph, typename Enable = void>
    struct GraphCopySelector {
      template <typename From, typename NodeRefMap, typename EdgeRefMap>
      static void copy(Graph &to, const From& from,
                       NodeRefMap& nodeRefMap, EdgeRefMap& edgeRefMap) {
        for (typename From::NodeIt it(from); it != INVALID; ++it) {
          nodeRefMap[it] = to.addNode();
        }
        for (typename From::EdgeIt it(from); it != INVALID; ++it) {
          edgeRefMap[it] = to.addArc(nodeRefMap[from.source(it)], 
				       nodeRefMap[from.target(it)]);
        }
      }
    };

    template <typename Graph>
    struct GraphCopySelector<
      Graph, 
      typename enable_if<typename Graph::BuildTag, void>::type> 
    {
      template <typename From, typename NodeRefMap, typename EdgeRefMap>
      static void copy(Graph &to, const From& from,
                       NodeRefMap& nodeRefMap, EdgeRefMap& edgeRefMap) {
        to.build(from, nodeRefMap, edgeRefMap);
      }
    };

    template <typename BpGraph, typename Enable = void>
    struct BpGraphCopySelector {
      template <typename From, typename RedRefMap, 
                typename BlueRefMap, typename EdgeRefMap>
      static void copy(BpGraph &to, const From& from,
                       RedRefMap& redRefMap, BlueRefMap& blueRefMap,
                       EdgeRefMap& edgeRefMap) {
        for (typename From::RedIt it(from); it != INVALID; ++it) {
          redRefMap[it] = to.addRed();
        }
        for (typename From::BlueIt it(from); it != INVALID; ++it) {
          blueRefMap[it] = to.addBlue();
        }
        for (typename From::EdgeIt it(from); it != INVALID; ++it) {
          edgeRefMap[it] = to.addArc(redRefMap[from.red(it)], 
                                           blueRefMap[from.blue(it)]);
        }
      }
    };

    template <typename BpGraph>
    struct BpGraphCopySelector<
      BpGraph, 
      typename enable_if<typename BpGraph::BuildTag, void>::type> 
    {
      template <typename From, typename RedRefMap, 
                typename BlueRefMap, typename EdgeRefMap>
      static void copy(BpGraph &to, const From& from,
                       RedRefMap& redRefMap, BlueRefMap& blueRefMap,
                       EdgeRefMap& edgeRefMap) {
        to.build(from, redRefMap, blueRefMap, edgeRefMap);
      }
    };
    

  }

  /// \brief Class to copy a digraph.
  ///
  /// Class to copy a digraph to another digraph (duplicate a digraph). The
  /// simplest way of using it is through the \c copyDigraph() function.
  template <typename To, typename From>
  class DigraphCopy {
  private:

    typedef typename From::Node Node;
    typedef typename From::NodeIt NodeIt;
    typedef typename From::Arc Arc;
    typedef typename From::ArcIt ArcIt;

    typedef typename To::Node TNode;
    typedef typename To::Arc TArc;

    typedef typename From::template NodeMap<TNode> NodeRefMap;
    typedef typename From::template ArcMap<TArc> ArcRefMap;
    
    
  public: 


    /// \brief Constructor for the DigraphCopy.
    ///
    /// It copies the content of the \c _from digraph into the
    /// \c _to digraph.
    DigraphCopy(To& _to, const From& _from) 
      : from(_from), to(_to) {}

    /// \brief Destructor of the DigraphCopy
    ///
    /// Destructor of the DigraphCopy
    ~DigraphCopy() {
      for (int i = 0; i < int(nodeMapCopies.size()); ++i) {
        delete nodeMapCopies[i];
      }
      for (int i = 0; i < int(arcMapCopies.size()); ++i) {
        delete arcMapCopies[i];
      }

    }

    /// \brief Copies the node references into the given map.
    ///
    /// Copies the node references into the given map.
    template <typename NodeRef>
    DigraphCopy& nodeRef(NodeRef& map) {
      nodeMapCopies.push_back(new _digraph_utils_bits::RefCopy<From, Node, 
                              NodeRefMap, NodeRef>(map));
      return *this;
    }

    /// \brief Copies the node cross references into the given map.
    ///
    ///  Copies the node cross references (reverse references) into
    ///  the given map.
    template <typename NodeCrossRef>
    DigraphCopy& nodeCrossRef(NodeCrossRef& map) {
      nodeMapCopies.push_back(new _digraph_utils_bits::CrossRefCopy<From, Node,
                              NodeRefMap, NodeCrossRef>(map));
      return *this;
    }

    /// \brief Make copy of the given map.
    ///
    /// Makes copy of the given map for the newly created digraph. 
    /// The new map's key type is the to digraph's node type,
    /// and the copied map's key type is the from digraph's node
    /// type.  
    template <typename ToMap, typename FromMap>
    DigraphCopy& nodeMap(ToMap& tmap, const FromMap& map) {
      nodeMapCopies.push_back(new _digraph_utils_bits::MapCopy<From, Node, 
                              NodeRefMap, ToMap, FromMap>(tmap, map));
      return *this;
    }

    /// \brief Make a copy of the given node.
    ///
    /// Make a copy of the given node.
    DigraphCopy& node(TNode& tnode, const Node& snode) {
      nodeMapCopies.push_back(new _digraph_utils_bits::ItemCopy<From, Node, 
                              NodeRefMap, TNode>(tnode, snode));
      return *this;
    }

    /// \brief Copies the arc references into the given map.
    ///
    /// Copies the arc references into the given map.
    template <typename ArcRef>
    DigraphCopy& arcRef(ArcRef& map) {
      arcMapCopies.push_back(new _digraph_utils_bits::RefCopy<From, Arc, 
                              ArcRefMap, ArcRef>(map));
      return *this;
    }

    /// \brief Copies the arc cross references into the given map.
    ///
    ///  Copies the arc cross references (reverse references) into
    ///  the given map.
    template <typename ArcCrossRef>
    DigraphCopy& arcCrossRef(ArcCrossRef& map) {
      arcMapCopies.push_back(new _digraph_utils_bits::CrossRefCopy<From, Arc,
                              ArcRefMap, ArcCrossRef>(map));
      return *this;
    }

    /// \brief Make copy of the given map.
    ///
    /// Makes copy of the given map for the newly created digraph. 
    /// The new map's key type is the to digraph's arc type,
    /// and the copied map's key type is the from digraph's arc
    /// type.  
    template <typename ToMap, typename FromMap>
    DigraphCopy& arcMap(ToMap& tmap, const FromMap& map) {
      arcMapCopies.push_back(new _digraph_utils_bits::MapCopy<From, Arc, 
                              ArcRefMap, ToMap, FromMap>(tmap, map));
      return *this;
    }

    /// \brief Make a copy of the given arc.
    ///
    /// Make a copy of the given arc.
    DigraphCopy& arc(TArc& tarc, const Arc& sarc) {
      arcMapCopies.push_back(new _digraph_utils_bits::ItemCopy<From, Arc, 
                              ArcRefMap, TArc>(tarc, sarc));
      return *this;
    }

    /// \brief Executes the copies.
    ///
    /// Executes the copies.
    void run() {
      NodeRefMap nodeRefMap(from);
      ArcRefMap arcRefMap(from);
      _digraph_utils_bits::DigraphCopySelector<To>::
        copy(to, from, nodeRefMap, arcRefMap);
      for (int i = 0; i < int(nodeMapCopies.size()); ++i) {
        nodeMapCopies[i]->copy(from, nodeRefMap);
      }
      for (int i = 0; i < int(arcMapCopies.size()); ++i) {
        arcMapCopies[i]->copy(from, arcRefMap);
      }      
    }

  protected:


    const From& from;
    To& to;

    std::vector<_digraph_utils_bits::MapCopyBase<From, Node, NodeRefMap>* > 
    nodeMapCopies;

    std::vector<_digraph_utils_bits::MapCopyBase<From, Arc, ArcRefMap>* > 
    arcMapCopies;

  };

  /// \brief Copy a digraph to another digraph.
  ///
  /// Copy a digraph to another digraph.
  /// The usage of the function:
  /// 
  ///\code
  /// copyDigraph(trg, src).nodeRef(nr).arcCrossRef(ecr).run();
  ///\endcode
  /// 
  /// After the copy the \c nr map will contain the mapping from the
  /// nodes of the \c from digraph to the nodes of the \c to digraph and
  /// \c ecr will contain the mapping from the arcs of the \c to digraph
  /// to the arcs of the \c from digraph.
  ///
  /// \see DigraphCopy 
  template <typename To, typename From>
  DigraphCopy<To, From> copyDigraph(To& to, const From& from) {
    return DigraphCopy<To, From>(to, from);
  }

  /// \brief Class to copy an graph.
  ///
  /// Class to copy an graph to another digraph (duplicate a digraph).
  /// The simplest way of using it is through the \c copyGraph() function.
  template <typename To, typename From>
  class GraphCopy {
  private:

    typedef typename From::Node Node;
    typedef typename From::NodeIt NodeIt;
    typedef typename From::Arc Arc;
    typedef typename From::ArcIt ArcIt;
    typedef typename From::Edge Edge;
    typedef typename From::EdgeIt EdgeIt;

    typedef typename To::Node TNode;
    typedef typename To::Arc TArc;
    typedef typename To::Edge TEdge;

    typedef typename From::template NodeMap<TNode> NodeRefMap;
    typedef typename From::template EdgeMap<TEdge> EdgeRefMap;

    struct ArcRefMap {
      ArcRefMap(const To& _to, const From& _from,
                 const EdgeRefMap& _edge_ref, const NodeRefMap& _node_ref) 
        : to(_to), from(_from), 
          edge_ref(_edge_ref), node_ref(_node_ref) {}

      typedef typename From::Arc Key;
      typedef typename To::Arc Value;

      Value operator[](const Key& key) const {
        bool forward = 
          (from.direction(key) == 
           (node_ref[from.source(static_cast<const Edge&>(key))] == 
            to.source(edge_ref[static_cast<const Edge&>(key)])));
	return to.direct(edge_ref[key], forward); 
      }
      
      const To& to;
      const From& from;
      const EdgeRefMap& edge_ref;
      const NodeRefMap& node_ref;
    };

    
  public: 


    /// \brief Constructor for the DigraphCopy.
    ///
    /// It copies the content of the \c _from digraph into the
    /// \c _to digraph.
    GraphCopy(To& _to, const From& _from) 
      : from(_from), to(_to) {}

    /// \brief Destructor of the DigraphCopy
    ///
    /// Destructor of the DigraphCopy
    ~GraphCopy() {
      for (int i = 0; i < int(nodeMapCopies.size()); ++i) {
        delete nodeMapCopies[i];
      }
      for (int i = 0; i < int(arcMapCopies.size()); ++i) {
        delete arcMapCopies[i];
      }
      for (int i = 0; i < int(edgeMapCopies.size()); ++i) {
        delete edgeMapCopies[i];
      }

    }

    /// \brief Copies the node references into the given map.
    ///
    /// Copies the node references into the given map.
    template <typename NodeRef>
    GraphCopy& nodeRef(NodeRef& map) {
      nodeMapCopies.push_back(new _digraph_utils_bits::RefCopy<From, Node, 
                              NodeRefMap, NodeRef>(map));
      return *this;
    }

    /// \brief Copies the node cross references into the given map.
    ///
    ///  Copies the node cross references (reverse references) into
    ///  the given map.
    template <typename NodeCrossRef>
    GraphCopy& nodeCrossRef(NodeCrossRef& map) {
      nodeMapCopies.push_back(new _digraph_utils_bits::CrossRefCopy<From, Node,
                              NodeRefMap, NodeCrossRef>(map));
      return *this;
    }

    /// \brief Make copy of the given map.
    ///
    /// Makes copy of the given map for the newly created digraph. 
    /// The new map's key type is the to digraph's node type,
    /// and the copied map's key type is the from digraph's node
    /// type.  
    template <typename ToMap, typename FromMap>
    GraphCopy& nodeMap(ToMap& tmap, const FromMap& map) {
      nodeMapCopies.push_back(new _digraph_utils_bits::MapCopy<From, Node, 
                              NodeRefMap, ToMap, FromMap>(tmap, map));
      return *this;
    }

    /// \brief Make a copy of the given node.
    ///
    /// Make a copy of the given node.
    GraphCopy& node(TNode& tnode, const Node& snode) {
      nodeMapCopies.push_back(new _digraph_utils_bits::ItemCopy<From, Node, 
                              NodeRefMap, TNode>(tnode, snode));
      return *this;
    }

    /// \brief Copies the arc references into the given map.
    ///
    /// Copies the arc references into the given map.
    template <typename ArcRef>
    GraphCopy& arcRef(ArcRef& map) {
      arcMapCopies.push_back(new _digraph_utils_bits::RefCopy<From, Arc, 
                              ArcRefMap, ArcRef>(map));
      return *this;
    }

    /// \brief Copies the arc cross references into the given map.
    ///
    ///  Copies the arc cross references (reverse references) into
    ///  the given map.
    template <typename ArcCrossRef>
    GraphCopy& arcCrossRef(ArcCrossRef& map) {
      arcMapCopies.push_back(new _digraph_utils_bits::CrossRefCopy<From, Arc,
                              ArcRefMap, ArcCrossRef>(map));
      return *this;
    }

    /// \brief Make copy of the given map.
    ///
    /// Makes copy of the given map for the newly created digraph. 
    /// The new map's key type is the to digraph's arc type,
    /// and the copied map's key type is the from digraph's arc
    /// type.  
    template <typename ToMap, typename FromMap>
    GraphCopy& arcMap(ToMap& tmap, const FromMap& map) {
      arcMapCopies.push_back(new _digraph_utils_bits::MapCopy<From, Arc, 
                              ArcRefMap, ToMap, FromMap>(tmap, map));
      return *this;
    }

    /// \brief Make a copy of the given arc.
    ///
    /// Make a copy of the given arc.
    GraphCopy& arc(TArc& tarc, const Arc& sarc) {
      arcMapCopies.push_back(new _digraph_utils_bits::ItemCopy<From, Arc, 
                              ArcRefMap, TArc>(tarc, sarc));
      return *this;
    }

    /// \brief Copies the edge references into the given map.
    ///
    /// Copies the edge references into the given map.
    template <typename EdgeRef>
    GraphCopy& edgeRef(EdgeRef& map) {
      edgeMapCopies.push_back(new _digraph_utils_bits::RefCopy<From, Edge, 
                               EdgeRefMap, EdgeRef>(map));
      return *this;
    }

    /// \brief Copies the edge cross references into the given map.
    ///
    /// Copies the edge cross references (reverse
    /// references) into the given map.
    template <typename EdgeCrossRef>
    GraphCopy& edgeCrossRef(EdgeCrossRef& map) {
      edgeMapCopies.push_back(new _digraph_utils_bits::CrossRefCopy<From, 
                               Edge, EdgeRefMap, EdgeCrossRef>(map));
      return *this;
    }

    /// \brief Make copy of the given map.
    ///
    /// Makes copy of the given map for the newly created digraph. 
    /// The new map's key type is the to digraph's edge type,
    /// and the copied map's key type is the from digraph's edge
    /// type.  
    template <typename ToMap, typename FromMap>
    GraphCopy& edgeMap(ToMap& tmap, const FromMap& map) {
      edgeMapCopies.push_back(new _digraph_utils_bits::MapCopy<From, Edge, 
                               EdgeRefMap, ToMap, FromMap>(tmap, map));
      return *this;
    }

    /// \brief Make a copy of the given edge.
    ///
    /// Make a copy of the given edge.
    GraphCopy& edge(TEdge& tedge, const Edge& sedge) {
      edgeMapCopies.push_back(new _digraph_utils_bits::ItemCopy<From, Edge, 
                               EdgeRefMap, TEdge>(tedge, sedge));
      return *this;
    }

    /// \brief Executes the copies.
    ///
    /// Executes the copies.
    void run() {
      NodeRefMap nodeRefMap(from);
      EdgeRefMap edgeRefMap(from);
      ArcRefMap arcRefMap(to, from, edgeRefMap, nodeRefMap);
      _digraph_utils_bits::GraphCopySelector<To>::
        copy(to, from, nodeRefMap, edgeRefMap);
      for (int i = 0; i < int(nodeMapCopies.size()); ++i) {
        nodeMapCopies[i]->copy(from, nodeRefMap);
      }
      for (int i = 0; i < int(edgeMapCopies.size()); ++i) {
        edgeMapCopies[i]->copy(from, edgeRefMap);
      }
      for (int i = 0; i < int(arcMapCopies.size()); ++i) {
        arcMapCopies[i]->copy(from, arcRefMap);
      }
    }

  private:
    
    const From& from;
    To& to;

    std::vector<_digraph_utils_bits::MapCopyBase<From, Node, NodeRefMap>* > 
    nodeMapCopies;

    std::vector<_digraph_utils_bits::MapCopyBase<From, Arc, ArcRefMap>* > 
    arcMapCopies;

    std::vector<_digraph_utils_bits::MapCopyBase<From, Edge, EdgeRefMap>* > 
    edgeMapCopies;

  };

  /// \brief Copy an graph to another digraph.
  ///
  /// Copy an graph to another digraph.
  /// The usage of the function:
  /// 
  ///\code
  /// copyGraph(trg, src).nodeRef(nr).arcCrossRef(ecr).run();
  ///\endcode
  /// 
  /// After the copy the \c nr map will contain the mapping from the
  /// nodes of the \c from digraph to the nodes of the \c to digraph and
  /// \c ecr will contain the mapping from the arcs of the \c to digraph
  /// to the arcs of the \c from digraph.
  ///
  /// \see GraphCopy 
  template <typename To, typename From>
  GraphCopy<To, From> 
  copyGraph(To& to, const From& from) {
    return GraphCopy<To, From>(to, from);
  }

  /// \brief Class to copy a bipartite digraph.
  ///
  /// Class to copy a bipartite digraph to another digraph
  /// (duplicate a digraph).  The simplest way of using it is through
  /// the \c copyBpGraph() function.
  template <typename To, typename From>
  class BpGraphCopy {
  private:

    typedef typename From::Node Node;
    typedef typename From::Red Red;
    typedef typename From::Blue Blue;
    typedef typename From::NodeIt NodeIt;
    typedef typename From::Arc Arc;
    typedef typename From::ArcIt ArcIt;
    typedef typename From::Edge Edge;
    typedef typename From::EdgeIt EdgeIt;

    typedef typename To::Node TNode;
    typedef typename To::Arc TArc;
    typedef typename To::Edge TEdge;

    typedef typename From::template RedMap<TNode> RedRefMap;
    typedef typename From::template BlueMap<TNode> BlueRefMap;
    typedef typename From::template EdgeMap<TEdge> EdgeRefMap;

    struct NodeRefMap {
      NodeRefMap(const From& _from, const RedRefMap& _red_ref,
                 const BlueRefMap& _blue_ref)
        : from(_from), red_ref(_red_ref), blue_ref(_blue_ref) {}

      typedef typename From::Node Key;
      typedef typename To::Node Value;

      Value operator[](const Key& key) const {
	return from.red(key) ? red_ref[key] : blue_ref[key]; 
      }
      
      const From& from;
      const RedRefMap& red_ref;
      const BlueRefMap& blue_ref;
    };

    struct ArcRefMap {
      ArcRefMap(const To& _to, const From& _from,
                 const EdgeRefMap& _edge_ref, const NodeRefMap& _node_ref) 
        : to(_to), from(_from), 
          edge_ref(_edge_ref), node_ref(_node_ref) {}

      typedef typename From::Arc Key;
      typedef typename To::Arc Value;

      Value operator[](const Key& key) const {
        bool forward = 
          (from.direction(key) == 
           (node_ref[from.source(static_cast<const Edge&>(key))] == 
            to.source(edge_ref[static_cast<const Edge&>(key)])));
	return to.direct(edge_ref[key], forward); 
      }
      
      const To& to;
      const From& from;
      const EdgeRefMap& edge_ref;
      const NodeRefMap& node_ref;
    };
    
  public: 


    /// \brief Constructor for the DigraphCopy.
    ///
    /// It copies the content of the \c _from digraph into the
    /// \c _to digraph.
    BpGraphCopy(To& _to, const From& _from) 
      : from(_from), to(_to) {}

    /// \brief Destructor of the DigraphCopy
    ///
    /// Destructor of the DigraphCopy
    ~BpGraphCopy() {
      for (int i = 0; i < int(redMapCopies.size()); ++i) {
        delete redMapCopies[i];
      }
      for (int i = 0; i < int(blueMapCopies.size()); ++i) {
        delete blueMapCopies[i];
      }
      for (int i = 0; i < int(nodeMapCopies.size()); ++i) {
        delete nodeMapCopies[i];
      }
      for (int i = 0; i < int(arcMapCopies.size()); ++i) {
        delete arcMapCopies[i];
      }
      for (int i = 0; i < int(edgeMapCopies.size()); ++i) {
        delete edgeMapCopies[i];
      }

    }

    /// \brief Copies the A-node references into the given map.
    ///
    /// Copies the A-node references into the given map.
    template <typename RedRef>
    BpGraphCopy& redRef(RedRef& map) {
      redMapCopies.push_back(new _digraph_utils_bits::RefCopy<From, Red, 
                               RedRefMap, RedRef>(map));
      return *this;
    }

    /// \brief Copies the A-node cross references into the given map.
    ///
    /// Copies the A-node cross references (reverse references) into
    /// the given map.
    template <typename RedCrossRef>
    BpGraphCopy& redCrossRef(RedCrossRef& map) {
      redMapCopies.push_back(new _digraph_utils_bits::CrossRefCopy<From, 
                               Red, RedRefMap, RedCrossRef>(map));
      return *this;
    }

    /// \brief Make copy of the given A-node map.
    ///
    /// Makes copy of the given map for the newly created digraph. 
    /// The new map's key type is the to digraph's node type,
    /// and the copied map's key type is the from digraph's node
    /// type.  
    template <typename ToMap, typename FromMap>
    BpGraphCopy& redMap(ToMap& tmap, const FromMap& map) {
      redMapCopies.push_back(new _digraph_utils_bits::MapCopy<From, Red, 
                               RedRefMap, ToMap, FromMap>(tmap, map));
      return *this;
    }

    /// \brief Copies the B-node references into the given map.
    ///
    /// Copies the B-node references into the given map.
    template <typename BlueRef>
    BpGraphCopy& blueRef(BlueRef& map) {
      blueMapCopies.push_back(new _digraph_utils_bits::RefCopy<From, Blue, 
                               BlueRefMap, BlueRef>(map));
      return *this;
    }

    /// \brief Copies the B-node cross references into the given map.
    ///
    ///  Copies the B-node cross references (reverse references) into
    ///  the given map.
    template <typename BlueCrossRef>
    BpGraphCopy& blueCrossRef(BlueCrossRef& map) {
      blueMapCopies.push_back(new _digraph_utils_bits::CrossRefCopy<From, 
                              Blue, BlueRefMap, BlueCrossRef>(map));
      return *this;
    }

    /// \brief Make copy of the given B-node map.
    ///
    /// Makes copy of the given map for the newly created digraph. 
    /// The new map's key type is the to digraph's node type,
    /// and the copied map's key type is the from digraph's node
    /// type.  
    template <typename ToMap, typename FromMap>
    BpGraphCopy& blueMap(ToMap& tmap, const FromMap& map) {
      blueMapCopies.push_back(new _digraph_utils_bits::MapCopy<From, Blue, 
                               BlueRefMap, ToMap, FromMap>(tmap, map));
      return *this;
    }
    /// \brief Copies the node references into the given map.
    ///
    /// Copies the node references into the given map.
    template <typename NodeRef>
    BpGraphCopy& nodeRef(NodeRef& map) {
      nodeMapCopies.push_back(new _digraph_utils_bits::RefCopy<From, Node, 
                              NodeRefMap, NodeRef>(map));
      return *this;
    }

    /// \brief Copies the node cross references into the given map.
    ///
    ///  Copies the node cross references (reverse references) into
    ///  the given map.
    template <typename NodeCrossRef>
    BpGraphCopy& nodeCrossRef(NodeCrossRef& map) {
      nodeMapCopies.push_back(new _digraph_utils_bits::CrossRefCopy<From, Node,
                              NodeRefMap, NodeCrossRef>(map));
      return *this;
    }

    /// \brief Make copy of the given map.
    ///
    /// Makes copy of the given map for the newly created digraph. 
    /// The new map's key type is the to digraph's node type,
    /// and the copied map's key type is the from digraph's node
    /// type.  
    template <typename ToMap, typename FromMap>
    BpGraphCopy& nodeMap(ToMap& tmap, const FromMap& map) {
      nodeMapCopies.push_back(new _digraph_utils_bits::MapCopy<From, Node, 
                              NodeRefMap, ToMap, FromMap>(tmap, map));
      return *this;
    }

    /// \brief Make a copy of the given node.
    ///
    /// Make a copy of the given node.
    BpGraphCopy& node(TNode& tnode, const Node& snode) {
      nodeMapCopies.push_back(new _digraph_utils_bits::ItemCopy<From, Node, 
                              NodeRefMap, TNode>(tnode, snode));
      return *this;
    }

    /// \brief Copies the arc references into the given map.
    ///
    /// Copies the arc references into the given map.
    template <typename ArcRef>
    BpGraphCopy& arcRef(ArcRef& map) {
      arcMapCopies.push_back(new _digraph_utils_bits::RefCopy<From, Arc, 
                              ArcRefMap, ArcRef>(map));
      return *this;
    }

    /// \brief Copies the arc cross references into the given map.
    ///
    ///  Copies the arc cross references (reverse references) into
    ///  the given map.
    template <typename ArcCrossRef>
    BpGraphCopy& arcCrossRef(ArcCrossRef& map) {
      arcMapCopies.push_back(new _digraph_utils_bits::CrossRefCopy<From, Arc,
                              ArcRefMap, ArcCrossRef>(map));
      return *this;
    }

    /// \brief Make copy of the given map.
    ///
    /// Makes copy of the given map for the newly created digraph. 
    /// The new map's key type is the to digraph's arc type,
    /// and the copied map's key type is the from digraph's arc
    /// type.  
    template <typename ToMap, typename FromMap>
    BpGraphCopy& arcMap(ToMap& tmap, const FromMap& map) {
      arcMapCopies.push_back(new _digraph_utils_bits::MapCopy<From, Arc, 
                              ArcRefMap, ToMap, FromMap>(tmap, map));
      return *this;
    }

    /// \brief Make a copy of the given arc.
    ///
    /// Make a copy of the given arc.
    BpGraphCopy& arc(TArc& tarc, const Arc& sarc) {
      arcMapCopies.push_back(new _digraph_utils_bits::ItemCopy<From, Arc, 
                              ArcRefMap, TArc>(tarc, sarc));
      return *this;
    }

    /// \brief Copies the edge references into the given map.
    ///
    /// Copies the edge references into the given map.
    template <typename EdgeRef>
    BpGraphCopy& edgeRef(EdgeRef& map) {
      edgeMapCopies.push_back(new _digraph_utils_bits::RefCopy<From, Edge, 
                               EdgeRefMap, EdgeRef>(map));
      return *this;
    }

    /// \brief Copies the edge cross references into the given map.
    ///
    /// Copies the edge cross references (reverse
    /// references) into the given map.
    template <typename EdgeCrossRef>
    BpGraphCopy& edgeCrossRef(EdgeCrossRef& map) {
      edgeMapCopies.push_back(new _digraph_utils_bits::CrossRefCopy<From, 
                               Edge, EdgeRefMap, EdgeCrossRef>(map));
      return *this;
    }

    /// \brief Make copy of the given map.
    ///
    /// Makes copy of the given map for the newly created digraph. 
    /// The new map's key type is the to digraph's edge type,
    /// and the copied map's key type is the from digraph's edge
    /// type.  
    template <typename ToMap, typename FromMap>
    BpGraphCopy& edgeMap(ToMap& tmap, const FromMap& map) {
      edgeMapCopies.push_back(new _digraph_utils_bits::MapCopy<From, Edge, 
                               EdgeRefMap, ToMap, FromMap>(tmap, map));
      return *this;
    }

    /// \brief Make a copy of the given edge.
    ///
    /// Make a copy of the given edge.
    BpGraphCopy& edge(TEdge& tedge, const Edge& sedge) {
      edgeMapCopies.push_back(new _digraph_utils_bits::ItemCopy<From, Edge, 
                               EdgeRefMap, TEdge>(tedge, sedge));
      return *this;
    }

    /// \brief Executes the copies.
    ///
    /// Executes the copies.
    void run() {
      RedRefMap redRefMap(from);
      BlueRefMap blueRefMap(from);
      NodeRefMap nodeRefMap(from, redRefMap, blueRefMap);
      EdgeRefMap edgeRefMap(from);
      ArcRefMap arcRefMap(to, from, edgeRefMap, nodeRefMap);
      _digraph_utils_bits::BpGraphCopySelector<To>::
        copy(to, from, redRefMap, blueRefMap, edgeRefMap);
      for (int i = 0; i < int(redMapCopies.size()); ++i) {
        redMapCopies[i]->copy(from, redRefMap);
      }
      for (int i = 0; i < int(blueMapCopies.size()); ++i) {
        blueMapCopies[i]->copy(from, blueRefMap);
      }
      for (int i = 0; i < int(nodeMapCopies.size()); ++i) {
        nodeMapCopies[i]->copy(from, nodeRefMap);
      }
      for (int i = 0; i < int(edgeMapCopies.size()); ++i) {
        edgeMapCopies[i]->copy(from, edgeRefMap);
      }
      for (int i = 0; i < int(arcMapCopies.size()); ++i) {
        arcMapCopies[i]->copy(from, arcRefMap);
      }
    }

  private:
    
    const From& from;
    To& to;

    std::vector<_digraph_utils_bits::MapCopyBase<From, Red, RedRefMap>* > 
    redMapCopies;

    std::vector<_digraph_utils_bits::MapCopyBase<From, Blue, BlueRefMap>* > 
    blueMapCopies;

    std::vector<_digraph_utils_bits::MapCopyBase<From, Node, NodeRefMap>* > 
    nodeMapCopies;

    std::vector<_digraph_utils_bits::MapCopyBase<From, Arc, ArcRefMap>* > 
    arcMapCopies;

    std::vector<_digraph_utils_bits::MapCopyBase<From, Edge, EdgeRefMap>* > 
    edgeMapCopies;

  };

  /// \brief Copy a bipartite digraph to another digraph.
  ///
  /// Copy a bipartite digraph to another digraph.
  /// The usage of the function:
  /// 
  ///\code
  /// copyBpGraph(trg, src).redRef(anr).arcCrossRef(ecr).run();
  ///\endcode
  /// 
  /// After the copy the \c nr map will contain the mapping from the
  /// nodes of the \c from digraph to the nodes of the \c to digraph and
  /// \c ecr will contain the mapping from the arcs of the \c to digraph
  /// to the arcs of the \c from digraph.
  ///
  /// \see BpGraphCopy
  template <typename To, typename From>
  BpGraphCopy<To, From> 
  copyBpGraph(To& to, const From& from) {
    return BpGraphCopy<To, From>(to, from);
  }


  /// @}

  /// \addtogroup digraph_maps
  /// @{

  /// Provides an immutable and unique id for each item in the digraph.

  /// The IdMap class provides a unique and immutable id for each item of the
  /// same type (e.g. node) in the digraph. This id is <ul><li>\b unique:
  /// different items (nodes) get different ids <li>\b immutable: the id of an
  /// item (node) does not change (even if you delete other nodes).  </ul>
  /// Through this map you get access (i.e. can read) the inner id values of
  /// the items stored in the digraph. This map can be inverted with its member
  /// class \c InverseMap.
  ///
  template <typename _Digraph, typename _Item>
  class IdMap {
  public:
    typedef _Digraph Digraph;
    typedef int Value;
    typedef _Item Item;
    typedef _Item Key;

    /// \brief Constructor.
    ///
    /// Constructor of the map.
    explicit IdMap(const Digraph& _digraph) : digraph(&_digraph) {}

    /// \brief Gives back the \e id of the item.
    ///
    /// Gives back the immutable and unique \e id of the item.
    int operator[](const Item& item) const { return digraph->id(item);}

    /// \brief Gives back the item by its id.
    ///
    /// Gives back the item by its id.
    Item operator()(int id) { return digraph->fromId(id, Item()); }

  private:
    const Digraph* digraph;

  public:

    /// \brief The class represents the inverse of its owner (IdMap).
    ///
    /// The class represents the inverse of its owner (IdMap).
    /// \see inverse()
    class InverseMap {
    public:

      /// \brief Constructor.
      ///
      /// Constructor for creating an id-to-item map.
      explicit InverseMap(const Digraph& _digraph) : digraph(&_digraph) {}

      /// \brief Constructor.
      ///
      /// Constructor for creating an id-to-item map.
      explicit InverseMap(const IdMap& idMap) : digraph(idMap.digraph) {}

      /// \brief Gives back the given item from its id.
      ///
      /// Gives back the given item from its id.
      /// 
      Item operator[](int id) const { return digraph->fromId(id, Item());}

    private:
      const Digraph* digraph;
    };

    /// \brief Gives back the inverse of the map.
    ///
    /// Gives back the inverse of the IdMap.
    InverseMap inverse() const { return InverseMap(*digraph);} 

  };

  
  /// \brief General invertable digraph-map type.

  /// This type provides simple invertable digraph-maps. 
  /// The InvertableMap wraps an arbitrary ReadWriteMap 
  /// and if a key is set to a new value then store it
  /// in the inverse map.
  ///
  /// The values of the map can be accessed
  /// with stl compatible forward iterator.
  ///
  /// \param _Digraph The digraph type.
  /// \param _Item The item type of the digraph.
  /// \param _Value The value type of the map.
  ///
  /// \see IterableValueMap
  template <typename _Digraph, typename _Item, typename _Value>
  class InvertableMap : protected DefaultMap<_Digraph, _Item, _Value> {
  private:
    
    typedef DefaultMap<_Digraph, _Item, _Value> Map;
    typedef _Digraph Digraph;

    typedef std::map<_Value, _Item> Container;
    Container invMap;    

  public:
 
    /// The key type of InvertableMap (Node, Arc, Edge).
    typedef typename Map::Key Key;
    /// The value type of the InvertableMap.
    typedef typename Map::Value Value;



    /// \brief Constructor.
    ///
    /// Construct a new InvertableMap for the digraph.
    ///
    explicit InvertableMap(const Digraph& digraph) : Map(digraph) {} 

    /// \brief Forward iterator for values.
    ///
    /// This iterator is an stl compatible forward
    /// iterator on the values of the map. The values can
    /// be accessed in the [beginValue, endValue) range.
    ///
    class ValueIterator 
      : public std::iterator<std::forward_iterator_tag, Value> {
      friend class InvertableMap;
    private:
      ValueIterator(typename Container::const_iterator _it) 
        : it(_it) {}
    public:
      
      ValueIterator() {}

      ValueIterator& operator++() { ++it; return *this; }
      ValueIterator operator++(int) { 
        ValueIterator tmp(*this); 
        operator++();
        return tmp; 
      }

      const Value& operator*() const { return it->first; }
      const Value* operator->() const { return &(it->first); }

      bool operator==(ValueIterator jt) const { return it == jt.it; }
      bool operator!=(ValueIterator jt) const { return it != jt.it; }
      
    private:
      typename Container::const_iterator it;
    };

    /// \brief Returns an iterator to the first value.
    ///
    /// Returns an stl compatible iterator to the 
    /// first value of the map. The values of the
    /// map can be accessed in the [beginValue, endValue)
    /// range.
    ValueIterator beginValue() const {
      return ValueIterator(invMap.begin());
    }

    /// \brief Returns an iterator after the last value.
    ///
    /// Returns an stl compatible iterator after the 
    /// last value of the map. The values of the
    /// map can be accessed in the [beginValue, endValue)
    /// range.
    ValueIterator endValue() const {
      return ValueIterator(invMap.end());
    }
    
    /// \brief The setter function of the map.
    ///
    /// Sets the mapped value.
    void set(const Key& key, const Value& val) {
      Value oldval = Map::operator[](key);
      typename Container::iterator it = invMap.find(oldval);
      if (it != invMap.end() && it->second == key) {
	invMap.erase(it);
      }      
      invMap.insert(make_pair(val, key));
      Map::set(key, val);
    }

    /// \brief The getter function of the map.
    ///
    /// It gives back the value associated with the key.
    typename MapTraits<Map>::ConstReturnValue 
    operator[](const Key& key) const {
      return Map::operator[](key);
    }

    /// \brief Gives back the item by its value.
    ///
    /// Gives back the item by its value.
    Key operator()(const Value& key) const {
      typename Container::const_iterator it = invMap.find(key);
      return it != invMap.end() ? it->second : INVALID;
    }

  protected:

    /// \brief Erase the key from the map.
    ///
    /// Erase the key to the map. It is called by the
    /// \c AlterationNotifier.
    virtual void erase(const Key& key) {
      Value val = Map::operator[](key);
      typename Container::iterator it = invMap.find(val);
      if (it != invMap.end() && it->second == key) {
	invMap.erase(it);
      }
      Map::erase(key);
    }

    /// \brief Erase more keys from the map.
    ///
    /// Erase more keys from the map. It is called by the
    /// \c AlterationNotifier.
    virtual void erase(const std::vector<Key>& keys) {
      for (int i = 0; i < int(keys.size()); ++i) {
	Value val = Map::operator[](keys[i]);
	typename Container::iterator it = invMap.find(val);
	if (it != invMap.end() && it->second == keys[i]) {
	  invMap.erase(it);
	}
      }
      Map::erase(keys);
    }

    /// \brief Clear the keys from the map and inverse map.
    ///
    /// Clear the keys from the map and inverse map. It is called by the
    /// \c AlterationNotifier.
    virtual void clear() {
      invMap.clear();
      Map::clear();
    }

  public:

    /// \brief The inverse map type.
    ///
    /// The inverse of this map. The subscript operator of the map
    /// gives back always the item what was last assigned to the value. 
    class InverseMap {
    public:
      /// \brief Constructor of the InverseMap.
      ///
      /// Constructor of the InverseMap.
      explicit InverseMap(const InvertableMap& _inverted) 
        : inverted(_inverted) {}

      /// The value type of the InverseMap.
      typedef typename InvertableMap::Key Value;
      /// The key type of the InverseMap.
      typedef typename InvertableMap::Value Key; 

      /// \brief Subscript operator. 
      ///
      /// Subscript operator. It gives back always the item 
      /// what was last assigned to the value.
      Value operator[](const Key& key) const {
	return inverted(key);
      }
      
    private:
      const InvertableMap& inverted;
    };

    /// \brief It gives back the just readable inverse map.
    ///
    /// It gives back the just readable inverse map.
    InverseMap inverse() const {
      return InverseMap(*this);
    } 


    
  };

  /// \brief Provides a mutable, continuous and unique descriptor for each 
  /// item in the digraph.
  ///
  /// The DescriptorMap class provides a unique and continuous (but mutable)
  /// descriptor (id) for each item of the same type (e.g. node) in the
  /// digraph. This id is <ul><li>\b unique: different items (nodes) get
  /// different ids <li>\b continuous: the range of the ids is the set of
  /// integers between 0 and \c n-1, where \c n is the number of the items of
  /// this type (e.g. nodes) (so the id of a node can change if you delete an
  /// other node, i.e. this id is mutable).  </ul> This map can be inverted
  /// with its member class \c InverseMap.
  ///
  /// \param _Digraph The digraph class the \c DescriptorMap belongs to.
  /// \param _Item The Item is the Key of the Map. It may be Node, Arc or 
  /// Edge.
  template <typename _Digraph, typename _Item>
  class DescriptorMap : protected DefaultMap<_Digraph, _Item, int> {

    typedef _Item Item;
    typedef DefaultMap<_Digraph, _Item, int> Map;

  public:
    /// The digraph class of DescriptorMap.
    typedef _Digraph Digraph;

    /// The key type of DescriptorMap (Node, Arc, Edge).
    typedef typename Map::Key Key;
    /// The value type of DescriptorMap.
    typedef typename Map::Value Value;

    /// \brief Constructor.
    ///
    /// Constructor for descriptor map.
    explicit DescriptorMap(const Digraph& _digraph) : Map(_digraph) {
      Item it;
      const typename Map::Notifier* nf = Map::notifier(); 
      for (nf->first(it); it != INVALID; nf->next(it)) {
	Map::set(it, invMap.size());
	invMap.push_back(it);	
      }      
    }

  protected:

    /// \brief Add a new key to the map.
    ///
    /// Add a new key to the map. It is called by the
    /// \c AlterationNotifier.
    virtual void add(const Item& item) {
      Map::add(item);
      Map::set(item, invMap.size());
      invMap.push_back(item);
    }

    /// \brief Add more new keys to the map.
    ///
    /// Add more new keys to the map. It is called by the
    /// \c AlterationNotifier.
    virtual void add(const std::vector<Item>& items) {
      Map::add(items);
      for (int i = 0; i < int(items.size()); ++i) {
	Map::set(items[i], invMap.size());
	invMap.push_back(items[i]);
      }
    }

    /// \brief Erase the key from the map.
    ///
    /// Erase the key from the map. It is called by the
    /// \c AlterationNotifier.
    virtual void erase(const Item& item) {
      Map::set(invMap.back(), Map::operator[](item));
      invMap[Map::operator[](item)] = invMap.back();
      invMap.pop_back();
      Map::erase(item);
    }

    /// \brief Erase more keys from the map.
    ///
    /// Erase more keys from the map. It is called by the
    /// \c AlterationNotifier.
    virtual void erase(const std::vector<Item>& items) {
      for (int i = 0; i < int(items.size()); ++i) {
	Map::set(invMap.back(), Map::operator[](items[i]));
	invMap[Map::operator[](items[i])] = invMap.back();
	invMap.pop_back();
      }
      Map::erase(items);
    }

    /// \brief Build the unique map.
    ///
    /// Build the unique map. It is called by the
    /// \c AlterationNotifier.
    virtual void build() {
      Map::build();
      Item it;
      const typename Map::Notifier* nf = Map::notifier(); 
      for (nf->first(it); it != INVALID; nf->next(it)) {
	Map::set(it, invMap.size());
	invMap.push_back(it);	
      }      
    }
    
    /// \brief Clear the keys from the map.
    ///
    /// Clear the keys from the map. It is called by the
    /// \c AlterationNotifier.
    virtual void clear() {
      invMap.clear();
      Map::clear();
    }

  public:

    /// \brief Returns the maximal value plus one.
    ///
    /// Returns the maximal value plus one in the map.
    unsigned int size() const {
      return invMap.size();
    }

    /// \brief Swaps the position of the two items in the map.
    ///
    /// Swaps the position of the two items in the map.
    void swap(const Item& p, const Item& q) {
      int pi = Map::operator[](p);
      int qi = Map::operator[](q);
      Map::set(p, qi);
      invMap[qi] = p;
      Map::set(q, pi);
      invMap[pi] = q;
    }

    /// \brief Gives back the \e descriptor of the item.
    ///
    /// Gives back the mutable and unique \e descriptor of the map.
    int operator[](const Item& item) const {
      return Map::operator[](item);
    }

    /// \brief Gives back the item by its descriptor.
    ///
    /// Gives back th item by its descriptor.
    Item operator()(int id) const {
      return invMap[id];
    }
    
  private:

    typedef std::vector<Item> Container;
    Container invMap;

  public:
    /// \brief The inverse map type of DescriptorMap.
    ///
    /// The inverse map type of DescriptorMap.
    class InverseMap {
    public:
      /// \brief Constructor of the InverseMap.
      ///
      /// Constructor of the InverseMap.
      explicit InverseMap(const DescriptorMap& _inverted) 
	: inverted(_inverted) {}


      /// The value type of the InverseMap.
      typedef typename DescriptorMap::Key Value;
      /// The key type of the InverseMap.
      typedef typename DescriptorMap::Value Key; 

      /// \brief Subscript operator. 
      ///
      /// Subscript operator. It gives back the item 
      /// that the descriptor belongs to currently.
      Value operator[](const Key& key) const {
	return inverted(key);
      }

      /// \brief Size of the map.
      ///
      /// Returns the size of the map.
      unsigned int size() const {
	return inverted.size();
      }
      
    private:
      const DescriptorMap& inverted;
    };

    /// \brief Gives back the inverse of the map.
    ///
    /// Gives back the inverse of the map.
    const InverseMap inverse() const {
      return InverseMap(*this);
    }
  };

  /// \brief Returns the source of the given arc.
  ///
  /// The SourceMap gives back the source Node of the given arc. 
  /// \see TargetMap
  /// \author Balazs Dezso
  template <typename Digraph>
  class SourceMap {
  public:

    typedef typename Digraph::Node Value;
    typedef typename Digraph::Arc Key;

    /// \brief Constructor
    ///
    /// Constructor
    /// \param _digraph The digraph that the map belongs to.
    explicit SourceMap(const Digraph& _digraph) : digraph(_digraph) {}

    /// \brief The subscript operator.
    ///
    /// The subscript operator.
    /// \param arc The arc 
    /// \return The source of the arc 
    Value operator[](const Key& arc) const {
      return digraph.source(arc);
    }

  private:
    const Digraph& digraph;
  };

  /// \brief Returns a \ref SourceMap class.
  ///
  /// This function just returns an \ref SourceMap class.
  /// \relates SourceMap
  template <typename Digraph>
  inline SourceMap<Digraph> sourceMap(const Digraph& digraph) {
    return SourceMap<Digraph>(digraph);
  } 

  /// \brief Returns the target of the given arc.
  ///
  /// The TargetMap gives back the target Node of the given arc. 
  /// \see SourceMap
  /// \author Balazs Dezso
  template <typename Digraph>
  class TargetMap {
  public:

    typedef typename Digraph::Node Value;
    typedef typename Digraph::Arc Key;

    /// \brief Constructor
    ///
    /// Constructor
    /// \param _digraph The digraph that the map belongs to.
    explicit TargetMap(const Digraph& _digraph) : digraph(_digraph) {}

    /// \brief The subscript operator.
    ///
    /// The subscript operator.
    /// \param e The arc 
    /// \return The target of the arc 
    Value operator[](const Key& e) const {
      return digraph.target(e);
    }

  private:
    const Digraph& digraph;
  };

  /// \brief Returns a \ref TargetMap class.
  ///
  /// This function just returns a \ref TargetMap class.
  /// \relates TargetMap
  template <typename Digraph>
  inline TargetMap<Digraph> targetMap(const Digraph& digraph) {
    return TargetMap<Digraph>(digraph);
  }

  /// \brief Returns the "forward" directed arc view of an edge.
  ///
  /// Returns the "forward" directed arc view of an edge.
  /// \see BackwardMap
  /// \author Balazs Dezso
  template <typename Digraph>
  class ForwardMap {
  public:

    typedef typename Digraph::Arc Value;
    typedef typename Digraph::Edge Key;

    /// \brief Constructor
    ///
    /// Constructor
    /// \param _digraph The digraph that the map belongs to.
    explicit ForwardMap(const Digraph& _digraph) : digraph(_digraph) {}

    /// \brief The subscript operator.
    ///
    /// The subscript operator.
    /// \param key An edge 
    /// \return The "forward" directed arc view of edge 
    Value operator[](const Key& key) const {
      return digraph.direct(key, true);
    }

  private:
    const Digraph& digraph;
  };

  /// \brief Returns a \ref ForwardMap class.
  ///
  /// This function just returns an \ref ForwardMap class.
  /// \relates ForwardMap
  template <typename Digraph>
  inline ForwardMap<Digraph> forwardMap(const Digraph& digraph) {
    return ForwardMap<Digraph>(digraph);
  }

  /// \brief Returns the "backward" directed arc view of an edge.
  ///
  /// Returns the "backward" directed arc view of an edge.
  /// \see ForwardMap
  /// \author Balazs Dezso
  template <typename Digraph>
  class BackwardMap {
  public:

    typedef typename Digraph::Arc Value;
    typedef typename Digraph::Edge Key;

    /// \brief Constructor
    ///
    /// Constructor
    /// \param _digraph The digraph that the map belongs to.
    explicit BackwardMap(const Digraph& _digraph) : digraph(_digraph) {}

    /// \brief The subscript operator.
    ///
    /// The subscript operator.
    /// \param key An edge 
    /// \return The "backward" directed arc view of edge 
    Value operator[](const Key& key) const {
      return digraph.direct(key, false);
    }

  private:
    const Digraph& digraph;
  };

  /// \brief Returns a \ref BackwardMap class

  /// This function just returns a \ref BackwardMap class.
  /// \relates BackwardMap
  template <typename Digraph>
  inline BackwardMap<Digraph> backwardMap(const Digraph& digraph) {
    return BackwardMap<Digraph>(digraph);
  }

  /// \brief Potential difference map
  ///
  /// If there is an potential map on the nodes then we
  /// can get an arc map as we get the substraction of the
  /// values of the target and source.
  template <typename Digraph, typename NodeMap>
  class PotentialDifferenceMap {
  public:
    typedef typename Digraph::Arc Key;
    typedef typename NodeMap::Value Value;

    /// \brief Constructor
    ///
    /// Contructor of the map
    explicit PotentialDifferenceMap(const Digraph& _digraph, 
                                    const NodeMap& _potential) 
      : digraph(_digraph), potential(_potential) {}

    /// \brief Const subscription operator
    ///
    /// Const subscription operator
    Value operator[](const Key& arc) const {
      return potential[digraph.target(arc)] - potential[digraph.source(arc)];
    }

  private:
    const Digraph& digraph;
    const NodeMap& potential;
  };

  /// \brief Returns a PotentialDifferenceMap.
  ///
  /// This function just returns a PotentialDifferenceMap.
  /// \relates PotentialDifferenceMap
  template <typename Digraph, typename NodeMap>
  PotentialDifferenceMap<Digraph, NodeMap> 
  potentialDifferenceMap(const Digraph& digraph, const NodeMap& potential) {
    return PotentialDifferenceMap<Digraph, NodeMap>(digraph, potential);
  }

  /// \brief Map of the node in-degrees.
  ///
  /// This map returns the in-degree of a node. Once it is constructed,
  /// the degrees are stored in a standard NodeMap, so each query is done
  /// in constant time. On the other hand, the values are updated automatically
  /// whenever the digraph changes.
  ///
  /// \warning Besides addNode() and addArc(), a digraph structure may provide
  /// alternative ways to modify the digraph. The correct behavior of InDegMap
  /// is not guarantied if these additional features are used. For example
  /// the functions \ref ListDigraph::changeSource() "changeSource()",
  /// \ref ListDigraph::changeTarget() "changeTarget()" and
  /// \ref ListDigraph::reverseArc() "reverseArc()"
  /// of \ref ListDigraph will \e not update the degree values correctly.
  ///
  /// \sa OutDegMap

  template <typename _Digraph>
  class InDegMap  
    : protected ItemSetTraits<_Digraph, typename _Digraph::Arc>
      ::ItemNotifier::ObserverBase {

  public:
    
    typedef _Digraph Digraph;
    typedef int Value;
    typedef typename Digraph::Node Key;

    typedef typename ItemSetTraits<_Digraph, typename _Digraph::Arc>
    ::ItemNotifier::ObserverBase Parent;

  private:

    class AutoNodeMap : public DefaultMap<_Digraph, Key, int> {
    public:

      typedef DefaultMap<_Digraph, Key, int> Parent;
      typedef typename Parent::Digraph Digraph;

      AutoNodeMap(const Digraph& digraph) : Parent(digraph, 0) {}
      
      virtual void add(const Key& key) {
	Parent::add(key);
	Parent::set(key, 0);
      }

      virtual void add(const std::vector<Key>& keys) {
	Parent::add(keys);
	for (int i = 0; i < int(keys.size()); ++i) {
	  Parent::set(keys[i], 0);
	}
      }

      virtual void build() {
	Parent::build();
	Key it;
	typename Parent::Notifier* nf = Parent::notifier();
	for (nf->first(it); it != INVALID; nf->next(it)) {
	  Parent::set(it, 0);
	}
      }
    };

  public:

    /// \brief Constructor.
    ///
    /// Constructor for creating in-degree map.
    explicit InDegMap(const Digraph& _digraph) : digraph(_digraph), deg(_digraph) {
      Parent::attach(digraph.notifier(typename _Digraph::Arc()));
      
      for(typename _Digraph::NodeIt it(digraph); it != INVALID; ++it) {
	deg[it] = countInArcs(digraph, it);
      }
    }
    
    /// Gives back the in-degree of a Node.
    int operator[](const Key& key) const {
      return deg[key];
    }

  protected:
    
    typedef typename Digraph::Arc Arc;

    virtual void add(const Arc& arc) {
      ++deg[digraph.target(arc)];
    }

    virtual void add(const std::vector<Arc>& arcs) {
      for (int i = 0; i < int(arcs.size()); ++i) {
        ++deg[digraph.target(arcs[i])];
      }
    }

    virtual void erase(const Arc& arc) {
      --deg[digraph.target(arc)];
    }

    virtual void erase(const std::vector<Arc>& arcs) {
      for (int i = 0; i < int(arcs.size()); ++i) {
        --deg[digraph.target(arcs[i])];
      }
    }

    virtual void build() {
      for(typename _Digraph::NodeIt it(digraph); it != INVALID; ++it) {
	deg[it] = countInArcs(digraph, it);
      }      
    }

    virtual void clear() {
      for(typename _Digraph::NodeIt it(digraph); it != INVALID; ++it) {
	deg[it] = 0;
      }
    }
  private:
    
    const _Digraph& digraph;
    AutoNodeMap deg;
  };

  /// \brief Map of the node out-degrees.
  ///
  /// This map returns the out-degree of a node. Once it is constructed,
  /// the degrees are stored in a standard NodeMap, so each query is done
  /// in constant time. On the other hand, the values are updated automatically
  /// whenever the digraph changes.
  ///
  /// \warning Besides addNode() and addArc(), a digraph structure may provide
  /// alternative ways to modify the digraph. The correct behavior of OutDegMap
  /// is not guarantied if these additional features are used. For example
  /// the functions \ref ListDigraph::changeSource() "changeSource()",
  /// \ref ListDigraph::changeTarget() "changeTarget()" and
  /// \ref ListDigraph::reverseArc() "reverseArc()"
  /// of \ref ListDigraph will \e not update the degree values correctly.
  ///
  /// \sa InDegMap

  template <typename _Digraph>
  class OutDegMap  
    : protected ItemSetTraits<_Digraph, typename _Digraph::Arc>
      ::ItemNotifier::ObserverBase {

  public:

    typedef typename ItemSetTraits<_Digraph, typename _Digraph::Arc>
    ::ItemNotifier::ObserverBase Parent;
    
    typedef _Digraph Digraph;
    typedef int Value;
    typedef typename Digraph::Node Key;

  private:

    class AutoNodeMap : public DefaultMap<_Digraph, Key, int> {
    public:

      typedef DefaultMap<_Digraph, Key, int> Parent;
      typedef typename Parent::Digraph Digraph;

      AutoNodeMap(const Digraph& digraph) : Parent(digraph, 0) {}
      
      virtual void add(const Key& key) {
	Parent::add(key);
	Parent::set(key, 0);
      }
      virtual void add(const std::vector<Key>& keys) {
	Parent::add(keys);
	for (int i = 0; i < int(keys.size()); ++i) {
	  Parent::set(keys[i], 0);
	}
      }
      virtual void build() {
	Parent::build();
	Key it;
	typename Parent::Notifier* nf = Parent::notifier();
	for (nf->first(it); it != INVALID; nf->next(it)) {
	  Parent::set(it, 0);
	}
      }
    };

  public:

    /// \brief Constructor.
    ///
    /// Constructor for creating out-degree map.
    explicit OutDegMap(const Digraph& _digraph) : digraph(_digraph), deg(_digraph) {
      Parent::attach(digraph.notifier(typename _Digraph::Arc()));
      
      for(typename _Digraph::NodeIt it(digraph); it != INVALID; ++it) {
	deg[it] = countOutArcs(digraph, it);
      }
    }

    /// Gives back the out-degree of a Node.
    int operator[](const Key& key) const {
      return deg[key];
    }

  protected:
    
    typedef typename Digraph::Arc Arc;

    virtual void add(const Arc& arc) {
      ++deg[digraph.source(arc)];
    }

    virtual void add(const std::vector<Arc>& arcs) {
      for (int i = 0; i < int(arcs.size()); ++i) {
        ++deg[digraph.source(arcs[i])];
      }
    }

    virtual void erase(const Arc& arc) {
      --deg[digraph.source(arc)];
    }

    virtual void erase(const std::vector<Arc>& arcs) {
      for (int i = 0; i < int(arcs.size()); ++i) {
        --deg[digraph.source(arcs[i])];
      }
    }

    virtual void build() {
      for(typename _Digraph::NodeIt it(digraph); it != INVALID; ++it) {
	deg[it] = countOutArcs(digraph, it);
      }      
    }

    virtual void clear() {
      for(typename _Digraph::NodeIt it(digraph); it != INVALID; ++it) {
	deg[it] = 0;
      }
    }
  private:
    
    const _Digraph& digraph;
    AutoNodeMap deg;
  };


  ///Dynamic arc look up between given endpoints.
  
  ///\ingroup gutils
  ///Using this class, you can find an arc in a digraph from a given
  ///source to a given target in amortized time <em>O(log d)</em>,
  ///where <em>d</em> is the out-degree of the source node.
  ///
  ///It is possible to find \e all parallel arcs between two nodes with
  ///the \c findFirst() and \c findNext() members.
  ///
  ///See the \ref ArcLookUp and \ref AllArcLookUp classes if your
  ///digraph do not changed so frequently.
  ///
  ///This class uses a self-adjusting binary search tree, Sleator's
  ///and Tarjan's Splay tree for guarantee the logarithmic amortized
  ///time bound for arc lookups. This class also guarantees the
  ///optimal time bound in a constant factor for any distribution of
  ///queries.
  ///
  ///\param G The type of the underlying digraph.  
  ///
  ///\sa ArcLookUp  
  ///\sa AllArcLookUp  
  template<class G>
  class DynArcLookUp 
    : protected ItemSetTraits<G, typename G::Arc>::ItemNotifier::ObserverBase
  {
  public:
    typedef typename ItemSetTraits<G, typename G::Arc>
    ::ItemNotifier::ObserverBase Parent;

    GRAPH_TYPEDEFS(typename G);
    typedef G Digraph;

  protected:

    class AutoNodeMap : public DefaultMap<G, Node, Arc> {
    public:

      typedef DefaultMap<G, Node, Arc> Parent;

      AutoNodeMap(const G& digraph) : Parent(digraph, INVALID) {}
      
      virtual void add(const Node& node) {
	Parent::add(node);
	Parent::set(node, INVALID);
      }

      virtual void add(const std::vector<Node>& nodes) {
	Parent::add(nodes);
	for (int i = 0; i < int(nodes.size()); ++i) {
	  Parent::set(nodes[i], INVALID);
	}
      }

      virtual void build() {
	Parent::build();
	Node it;
	typename Parent::Notifier* nf = Parent::notifier();
	for (nf->first(it); it != INVALID; nf->next(it)) {
	  Parent::set(it, INVALID);
	}
      }
    };

    const Digraph &_g;
    AutoNodeMap _head;
    typename Digraph::template ArcMap<Arc> _parent;
    typename Digraph::template ArcMap<Arc> _left;
    typename Digraph::template ArcMap<Arc> _right;
    
    class ArcLess {
      const Digraph &g;
    public:
      ArcLess(const Digraph &_g) : g(_g) {}
      bool operator()(Arc a,Arc b) const 
      {
	return g.target(a)<g.target(b);
      }
    };
    
  public:
    
    ///Constructor

    ///Constructor.
    ///
    ///It builds up the search database.
    DynArcLookUp(const Digraph &g) 
      : _g(g),_head(g),_parent(g),_left(g),_right(g) 
    { 
      Parent::attach(_g.notifier(typename Digraph::Arc()));
      refresh(); 
    }
    
  protected:

    virtual void add(const Arc& arc) {
      insert(arc);
    }

    virtual void add(const std::vector<Arc>& arcs) {
      for (int i = 0; i < int(arcs.size()); ++i) {
	insert(arcs[i]);
      }
    }

    virtual void erase(const Arc& arc) {
      remove(arc);
    }

    virtual void erase(const std::vector<Arc>& arcs) {
      for (int i = 0; i < int(arcs.size()); ++i) {
	remove(arcs[i]);
      }     
    }

    virtual void build() {
      refresh();
    }

    virtual void clear() {
      for(NodeIt n(_g);n!=INVALID;++n) {
	_head.set(n, INVALID);
      }
    }

    void insert(Arc arc) {
      Node s = _g.source(arc);
      Node t = _g.target(arc);
      _left.set(arc, INVALID);
      _right.set(arc, INVALID);
      
      Arc e = _head[s];
      if (e == INVALID) {
	_head.set(s, arc);
	_parent.set(arc, INVALID);
	return;
      }
      while (true) {
	if (t < _g.target(e)) {
	  if (_left[e] == INVALID) {
	    _left.set(e, arc);
	    _parent.set(arc, e);
	    splay(arc);
	    return;
	  } else {
	    e = _left[e];
	  }
	} else {
	  if (_right[e] == INVALID) {
	    _right.set(e, arc);
	    _parent.set(arc, e);
	    splay(arc);
	    return;
	  } else {
	    e = _right[e];
	  }
	}
      }
    }

    void remove(Arc arc) {
      if (_left[arc] == INVALID) {
	if (_right[arc] != INVALID) {
	  _parent.set(_right[arc], _parent[arc]);
	}
	if (_parent[arc] != INVALID) {
	  if (_left[_parent[arc]] == arc) {
	    _left.set(_parent[arc], _right[arc]);
	  } else {
	    _right.set(_parent[arc], _right[arc]);
	  }
	} else {
	  _head.set(_g.source(arc), _right[arc]);
	}
      } else if (_right[arc] == INVALID) {
	_parent.set(_left[arc], _parent[arc]);
	if (_parent[arc] != INVALID) {
	  if (_left[_parent[arc]] == arc) {
	    _left.set(_parent[arc], _left[arc]);
	  } else {
	    _right.set(_parent[arc], _left[arc]);
	  }
	} else {
	  _head.set(_g.source(arc), _left[arc]);
	}
      } else {
	Arc e = _left[arc];
	if (_right[e] != INVALID) {
	  e = _right[e];	  
	  while (_right[e] != INVALID) {
	    e = _right[e];
	  }
	  Arc s = _parent[e];
	  _right.set(_parent[e], _left[e]);
	  if (_left[e] != INVALID) {
	    _parent.set(_left[e], _parent[e]);
	  }
	  
	  _left.set(e, _left[arc]);
	  _parent.set(_left[arc], e);
	  _right.set(e, _right[arc]);
	  _parent.set(_right[arc], e);

	  _parent.set(e, _parent[arc]);
	  if (_parent[arc] != INVALID) {
	    if (_left[_parent[arc]] == arc) {
	      _left.set(_parent[arc], e);
	    } else {
	      _right.set(_parent[arc], e);
	    }
	  }
	  splay(s);
	} else {
	  _right.set(e, _right[arc]);
	  _parent.set(_right[arc], e);

	  if (_parent[arc] != INVALID) {
	    if (_left[_parent[arc]] == arc) {
	      _left.set(_parent[arc], e);
	    } else {
	      _right.set(_parent[arc], e);
	    }
	  } else {
	    _head.set(_g.source(arc), e);
	  }
	}
      }
    }

    Arc refreshRec(std::vector<Arc> &v,int a,int b) 
    {
      int m=(a+b)/2;
      Arc me=v[m];
      if (a < m) {
	Arc left = refreshRec(v,a,m-1);
	_left.set(me, left);
	_parent.set(left, me);
      } else {
	_left.set(me, INVALID);
      }
      if (m < b) {
	Arc right = refreshRec(v,m+1,b);
	_right.set(me, right);
	_parent.set(right, me);
      } else {
	_right.set(me, INVALID);
      }
      return me;
    }

    void refresh() {
      for(NodeIt n(_g);n!=INVALID;++n) {
	std::vector<Arc> v;
	for(OutArcIt e(_g,n);e!=INVALID;++e) v.push_back(e);
	if(v.size()) {
	  std::sort(v.begin(),v.end(),ArcLess(_g));
	  Arc head = refreshRec(v,0,v.size()-1);
	  _head.set(n, head);
	  _parent.set(head, INVALID);
	}
	else _head.set(n, INVALID);
      }
    }

    void zig(Arc v) {        
      Arc w = _parent[v];
      _parent.set(v, _parent[w]);
      _parent.set(w, v);
      _left.set(w, _right[v]);
      _right.set(v, w);
      if (_parent[v] != INVALID) {
	if (_right[_parent[v]] == w) {
	  _right.set(_parent[v], v);
	} else {
	  _left.set(_parent[v], v);
	}
      }
      if (_left[w] != INVALID){
	_parent.set(_left[w], w);
      }
    }

    void zag(Arc v) {        
      Arc w = _parent[v];
      _parent.set(v, _parent[w]);
      _parent.set(w, v);
      _right.set(w, _left[v]);
      _left.set(v, w);
      if (_parent[v] != INVALID){
	if (_left[_parent[v]] == w) {
	  _left.set(_parent[v], v);
	} else {
	  _right.set(_parent[v], v);
	}
      }
      if (_right[w] != INVALID){
	_parent.set(_right[w], w);
      }
    }

    void splay(Arc v) {
      while (_parent[v] != INVALID) {
	if (v == _left[_parent[v]]) {
	  if (_parent[_parent[v]] == INVALID) {
	    zig(v);
	  } else {
	    if (_parent[v] == _left[_parent[_parent[v]]]) {
	      zig(_parent[v]);
	      zig(v);
	    } else {
	      zig(v);
	      zag(v);
	    }
	  }
	} else {
	  if (_parent[_parent[v]] == INVALID) {
	    zag(v);
	  } else {
	    if (_parent[v] == _left[_parent[_parent[v]]]) {
	      zag(v);
	      zig(v);
	    } else {
	      zag(_parent[v]);
	      zag(v);
	    }
	  }
	}
      }
      _head[_g.source(v)] = v;
    }


  public:
    
    ///Find an arc between two nodes.
    
    ///Find an arc between two nodes in time <em>O(</em>log<em>d)</em>, where
    /// <em>d</em> is the number of outgoing arcs of \c s.
    ///\param s The source node
    ///\param t The target node
    ///\return An arc from \c s to \c t if there exists,
    ///\ref INVALID otherwise.
    Arc operator()(Node s, Node t) const
    {
      Arc e = _head[s];
      while (true) {
	if (_g.target(e) == t) {
	  const_cast<DynArcLookUp&>(*this).splay(e);
	  return e;
	} else if (t < _g.target(e)) {
	  if (_left[e] == INVALID) {
	    const_cast<DynArcLookUp&>(*this).splay(e);
	    return INVALID;
	  } else {
	    e = _left[e];
	  }
	} else  {
	  if (_right[e] == INVALID) {
	    const_cast<DynArcLookUp&>(*this).splay(e);
	    return INVALID;
	  } else {
	    e = _right[e];
	  }
	}
      }
    }

    ///Find the first arc between two nodes.
    
    ///Find the first arc between two nodes in time
    /// <em>O(</em>log<em>d)</em>, where <em>d</em> is the number of
    /// outgoing arcs of \c s.  
    ///\param s The source node 
    ///\param t The target node
    ///\return An arc from \c s to \c t if there exists, \ref INVALID
    /// otherwise.
    Arc findFirst(Node s, Node t) const
    {
      Arc e = _head[s];
      Arc r = INVALID;
      while (true) {
	if (_g.target(e) < t) {
	  if (_right[e] == INVALID) {
	    const_cast<DynArcLookUp&>(*this).splay(e);
	    return r;
	  } else {
	    e = _right[e];
	  }
	} else {
	  if (_g.target(e) == t) {
	    r = e;
	  }
	  if (_left[e] == INVALID) {
	    const_cast<DynArcLookUp&>(*this).splay(e);
	    return r;
	  } else {
	    e = _left[e];
	  }
	}
      }
    }

    ///Find the next arc between two nodes.
    
    ///Find the next arc between two nodes in time
    /// <em>O(</em>log<em>d)</em>, where <em>d</em> is the number of
    /// outgoing arcs of \c s.  
    ///\param s The source node 
    ///\param t The target node
    ///\return An arc from \c s to \c t if there exists, \ref INVALID
    /// otherwise.

    ///\note If \c e is not the result of the previous \c findFirst()
    ///operation then the amorized time bound can not be guaranteed.
#ifdef DOXYGEN
    Arc findNext(Node s, Node t, Arc e) const
#else
    Arc findNext(Node, Node t, Arc e) const
#endif
    {
      if (_right[e] != INVALID) {
	e = _right[e];
	while (_left[e] != INVALID) {
	  e = _left[e];
	}
	const_cast<DynArcLookUp&>(*this).splay(e);
      } else {
	while (_parent[e] != INVALID && _right[_parent[e]] ==  e) {
	  e = _parent[e];
	}
	if (_parent[e] == INVALID) {
	  return INVALID;
	} else {
	  e = _parent[e];
	  const_cast<DynArcLookUp&>(*this).splay(e);
	}
      }
      if (_g.target(e) == t) return e;
      else return INVALID;    
    }

  };

  ///Fast arc look up between given endpoints.
  
  ///\ingroup gutils
  ///Using this class, you can find an arc in a digraph from a given
  ///source to a given target in time <em>O(log d)</em>,
  ///where <em>d</em> is the out-degree of the source node.
  ///
  ///It is not possible to find \e all parallel arcs between two nodes.
  ///Use \ref AllArcLookUp for this purpose.
  ///
  ///\warning This class is static, so you should refresh() (or at least
  ///refresh(Node)) this data structure
  ///whenever the digraph changes. This is a time consuming (superlinearly
  ///proportional (<em>O(m</em>log<em>m)</em>) to the number of arcs).
  ///
  ///\param G The type of the underlying digraph.
  ///
  ///\sa DynArcLookUp
  ///\sa AllArcLookUp  
  template<class G>
  class ArcLookUp 
  {
  public:
    GRAPH_TYPEDEFS(typename G);
    typedef G Digraph;

  protected:
    const Digraph &_g;
    typename Digraph::template NodeMap<Arc> _head;
    typename Digraph::template ArcMap<Arc> _left;
    typename Digraph::template ArcMap<Arc> _right;
    
    class ArcLess {
      const Digraph &g;
    public:
      ArcLess(const Digraph &_g) : g(_g) {}
      bool operator()(Arc a,Arc b) const 
      {
	return g.target(a)<g.target(b);
      }
    };
    
  public:
    
    ///Constructor

    ///Constructor.
    ///
    ///It builds up the search database, which remains valid until the digraph
    ///changes.
    ArcLookUp(const Digraph &g) :_g(g),_head(g),_left(g),_right(g) {refresh();}
    
  private:
    Arc refreshRec(std::vector<Arc> &v,int a,int b) 
    {
      int m=(a+b)/2;
      Arc me=v[m];
      _left[me] = a<m?refreshRec(v,a,m-1):INVALID;
      _right[me] = m<b?refreshRec(v,m+1,b):INVALID;
      return me;
    }
  public:
    ///Refresh the data structure at a node.

    ///Build up the search database of node \c n.
    ///
    ///It runs in time <em>O(d</em>log<em>d)</em>, where <em>d</em> is
    ///the number of the outgoing arcs of \c n.
    void refresh(Node n) 
    {
      std::vector<Arc> v;
      for(OutArcIt e(_g,n);e!=INVALID;++e) v.push_back(e);
      if(v.size()) {
	std::sort(v.begin(),v.end(),ArcLess(_g));
	_head[n]=refreshRec(v,0,v.size()-1);
      }
      else _head[n]=INVALID;
    }
    ///Refresh the full data structure.

    ///Build up the full search database. In fact, it simply calls
    ///\ref refresh(Node) "refresh(n)" for each node \c n.
    ///
    ///It runs in time <em>O(m</em>log<em>D)</em>, where <em>m</em> is
    ///the number of the arcs of \c n and <em>D</em> is the maximum
    ///out-degree of the digraph.

    void refresh() 
    {
      for(NodeIt n(_g);n!=INVALID;++n) refresh(n);
    }
    
    ///Find an arc between two nodes.
    
    ///Find an arc between two nodes in time <em>O(</em>log<em>d)</em>, where
    /// <em>d</em> is the number of outgoing arcs of \c s.
    ///\param s The source node
    ///\param t The target node
    ///\return An arc from \c s to \c t if there exists,
    ///\ref INVALID otherwise.
    ///
    ///\warning If you change the digraph, refresh() must be called before using
    ///this operator. If you change the outgoing arcs of
    ///a single node \c n, then
    ///\ref refresh(Node) "refresh(n)" is enough.
    ///
    Arc operator()(Node s, Node t) const
    {
      Arc e;
      for(e=_head[s];
	  e!=INVALID&&_g.target(e)!=t;
	  e = t < _g.target(e)?_left[e]:_right[e]) ;
      return e;
    }

  };

  ///Fast look up of all arcs between given endpoints.
  
  ///\ingroup gutils
  ///This class is the same as \ref ArcLookUp, with the addition
  ///that it makes it possible to find all arcs between given endpoints.
  ///
  ///\warning This class is static, so you should refresh() (or at least
  ///refresh(Node)) this data structure
  ///whenever the digraph changes. This is a time consuming (superlinearly
  ///proportional (<em>O(m</em>log<em>m)</em>) to the number of arcs).
  ///
  ///\param G The type of the underlying digraph.
  ///
  ///\sa DynArcLookUp
  ///\sa ArcLookUp  
  template<class G>
  class AllArcLookUp : public ArcLookUp<G>
  {
    using ArcLookUp<G>::_g;
    using ArcLookUp<G>::_right;
    using ArcLookUp<G>::_left;
    using ArcLookUp<G>::_head;

    GRAPH_TYPEDEFS(typename G);
    typedef G Digraph;
    
    typename Digraph::template ArcMap<Arc> _next;
    
    Arc refreshNext(Arc head,Arc next=INVALID)
    {
      if(head==INVALID) return next;
      else {
	next=refreshNext(_right[head],next);
// 	_next[head]=next;
	_next[head]=( next!=INVALID && _g.target(next)==_g.target(head))
	  ? next : INVALID;
	return refreshNext(_left[head],head);
      }
    }
    
    void refreshNext()
    {
      for(NodeIt n(_g);n!=INVALID;++n) refreshNext(_head[n]);
    }
    
  public:
    ///Constructor

    ///Constructor.
    ///
    ///It builds up the search database, which remains valid until the digraph
    ///changes.
    AllArcLookUp(const Digraph &g) : ArcLookUp<G>(g), _next(g) {refreshNext();}

    ///Refresh the data structure at a node.

    ///Build up the search database of node \c n.
    ///
    ///It runs in time <em>O(d</em>log<em>d)</em>, where <em>d</em> is
    ///the number of the outgoing arcs of \c n.
    
    void refresh(Node n) 
    {
      ArcLookUp<G>::refresh(n);
      refreshNext(_head[n]);
    }
    
    ///Refresh the full data structure.

    ///Build up the full search database. In fact, it simply calls
    ///\ref refresh(Node) "refresh(n)" for each node \c n.
    ///
    ///It runs in time <em>O(m</em>log<em>D)</em>, where <em>m</em> is
    ///the number of the arcs of \c n and <em>D</em> is the maximum
    ///out-degree of the digraph.

    void refresh() 
    {
      for(NodeIt n(_g);n!=INVALID;++n) refresh(_head[n]);
    }
    
    ///Find an arc between two nodes.
    
    ///Find an arc between two nodes.
    ///\param s The source node
    ///\param t The target node
    ///\param prev The previous arc between \c s and \c t. It it is INVALID or
    ///not given, the operator finds the first appropriate arc.
    ///\return An arc from \c s to \c t after \c prev or
    ///\ref INVALID if there is no more.
    ///
    ///For example, you can count the number of arcs from \c u to \c v in the
    ///following way.
    ///\code
    ///AllArcLookUp<ListDigraph> ae(g);
    ///...
    ///int n=0;
    ///for(Arc e=ae(u,v);e!=INVALID;e=ae(u,v,e)) n++;
    ///\endcode
    ///
    ///Finding the first arc take <em>O(</em>log<em>d)</em> time, where
    /// <em>d</em> is the number of outgoing arcs of \c s. Then, the
    ///consecutive arcs are found in constant time.
    ///
    ///\warning If you change the digraph, refresh() must be called before using
    ///this operator. If you change the outgoing arcs of
    ///a single node \c n, then
    ///\ref refresh(Node) "refresh(n)" is enough.
    ///
#ifdef DOXYGEN
    Arc operator()(Node s, Node t, Arc prev=INVALID) const {}
#else
    using ArcLookUp<G>::operator() ;
    Arc operator()(Node s, Node t, Arc prev) const
    {
      return prev==INVALID?(*this)(s,t):_next[prev];
    }
#endif
      
  };

  /// @}

} //END OF NAMESPACE LEMON

#endif
