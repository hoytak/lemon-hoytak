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

#ifndef LEMON_DIJKSTRA_H
#define LEMON_DIJKSTRA_H

///\ingroup shortest_path
///\file
///\brief Dijkstra algorithm.

#include <limits>
#include <lemon/list_graph.h>
#include <lemon/bin_heap.h>
#include <lemon/bits/path_dump.h>
#include <lemon/core.h>
#include <lemon/error.h>
#include <lemon/maps.h>

namespace lemon {

  /// \brief Default operation traits for the Dijkstra algorithm class.
  ///
  /// This operation traits class defines all computational operations and
  /// constants which are used in the Dijkstra algorithm.
  template <typename Value>
  struct DijkstraDefaultOperationTraits {
    /// \brief Gives back the zero value of the type.
    static Value zero() {
      return static_cast<Value>(0);
    }
    /// \brief Gives back the sum of the given two elements.
    static Value plus(const Value& left, const Value& right) {
      return left + right;
    }
    /// \brief Gives back true only if the first value is less than the second.
    static bool less(const Value& left, const Value& right) {
      return left < right;
    }
  };

  /// \brief Widest path operation traits for the Dijkstra algorithm class.
  ///
  /// This operation traits class defines all computational operations and
  /// constants which are used in the Dijkstra algorithm for widest path
  /// computation.
  ///
  /// \see DijkstraDefaultOperationTraits
  template <typename Value>
  struct DijkstraWidestPathOperationTraits {
    /// \brief Gives back the maximum value of the type.
    static Value zero() {
      return std::numeric_limits<Value>::max();
    }
    /// \brief Gives back the minimum of the given two elements.
    static Value plus(const Value& left, const Value& right) {
      return std::min(left, right);
    }
    /// \brief Gives back true only if the first value is less than the second.
    static bool less(const Value& left, const Value& right) {
      return left < right;
    }
  };

  ///Default traits class of Dijkstra class.

  ///Default traits class of Dijkstra class.
  ///\tparam GR The type of the digraph.
  ///\tparam LM The type of the length map.
  template<class GR, class LM>
  struct DijkstraDefaultTraits
  {
    ///The type of the digraph the algorithm runs on.
    typedef GR Digraph;

    ///The type of the map that stores the arc lengths.

    ///The type of the map that stores the arc lengths.
    ///It must meet the \ref concepts::ReadMap "ReadMap" concept.
    typedef LM LengthMap;
    ///The type of the length of the arcs.
    typedef typename LM::Value Value;

    /// Operation traits for Dijkstra algorithm.

    /// This class defines the operations that are used in the algorithm.
    /// \see DijkstraDefaultOperationTraits
    typedef DijkstraDefaultOperationTraits<Value> OperationTraits;

    /// The cross reference type used by the heap.

    /// The cross reference type used by the heap.
    /// Usually it is \c Digraph::NodeMap<int>.
    typedef typename Digraph::template NodeMap<int> HeapCrossRef;
    ///Instantiates a \ref HeapCrossRef.

    ///This function instantiates a \ref HeapCrossRef.
    /// \param g is the digraph, to which we would like to define the
    /// \ref HeapCrossRef.
    static HeapCrossRef *createHeapCrossRef(const Digraph &g)
    {
      return new HeapCrossRef(g);
    }

    ///The heap type used by the Dijkstra algorithm.

    ///The heap type used by the Dijkstra algorithm.
    ///
    ///\sa BinHeap
    ///\sa Dijkstra
    typedef BinHeap<typename LM::Value, HeapCrossRef, std::less<Value> > Heap;
    ///Instantiates a \ref Heap.

    ///This function instantiates a \ref Heap.
    static Heap *createHeap(HeapCrossRef& r)
    {
      return new Heap(r);
    }

    ///\brief The type of the map that stores the predecessor
    ///arcs of the shortest paths.
    ///
    ///The type of the map that stores the predecessor
    ///arcs of the shortest paths.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    typedef typename Digraph::template NodeMap<typename Digraph::Arc> PredMap;
    ///Instantiates a \ref PredMap.

    ///This function instantiates a \ref PredMap.
    ///\param g is the digraph, to which we would like to define the
    ///\ref PredMap.
    ///\todo The digraph alone may be insufficient for the initialization
    static PredMap *createPredMap(const Digraph &g)
    {
      return new PredMap(g);
    }

    ///The type of the map that indicates which nodes are processed.

    ///The type of the map that indicates which nodes are processed.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    ///By default it is a NullMap.
    ///\todo If it is set to a real map,
    ///Dijkstra::processed() should read this.
    typedef NullMap<typename Digraph::Node,bool> ProcessedMap;
    ///Instantiates a \ref ProcessedMap.

    ///This function instantiates a \ref ProcessedMap.
    ///\param g is the digraph, to which
    ///we would like to define the \ref ProcessedMap
#ifdef DOXYGEN
    static ProcessedMap *createProcessedMap(const Digraph &g)
#else
    static ProcessedMap *createProcessedMap(const Digraph &)
#endif
    {
      return new ProcessedMap();
    }

    ///The type of the map that stores the distances of the nodes.

    ///The type of the map that stores the distances of the nodes.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    typedef typename Digraph::template NodeMap<typename LM::Value> DistMap;
    ///Instantiates a \ref DistMap.

    ///This function instantiates a \ref DistMap.
    ///\param g is the digraph, to which we would like to define
    ///the \ref DistMap
    static DistMap *createDistMap(const Digraph &g)
    {
      return new DistMap(g);
    }
  };

  ///%Dijkstra algorithm class.

  /// \ingroup shortest_path
  ///This class provides an efficient implementation of the %Dijkstra algorithm.
  ///
  ///The arc lengths are passed to the algorithm using a
  ///\ref concepts::ReadMap "ReadMap",
  ///so it is easy to change it to any kind of length.
  ///The type of the length is determined by the
  ///\ref concepts::ReadMap::Value "Value" of the length map.
  ///It is also possible to change the underlying priority heap.
  ///
  ///There is also a \ref dijkstra() "function type interface" for the
  ///%Dijkstra algorithm, which is convenient in the simplier cases and
  ///it can be used easier.
  ///
  ///\tparam GR The type of the digraph the algorithm runs on.
  ///The default value is \ref ListDigraph.
  ///The value of GR is not used directly by \ref Dijkstra, it is only
  ///passed to \ref DijkstraDefaultTraits.
  ///\tparam LM A readable arc map that determines the lengths of the
  ///arcs. It is read once for each arc, so the map may involve in
  ///relatively time consuming process to compute the arc lengths if
  ///it is necessary. The default map type is \ref
  ///concepts::Digraph::ArcMap "Digraph::ArcMap<int>".
  ///The value of LM is not used directly by \ref Dijkstra, it is only
  ///passed to \ref DijkstraDefaultTraits.
  ///\tparam TR Traits class to set various data types used by the algorithm.
  ///The default traits class is \ref DijkstraDefaultTraits
  ///"DijkstraDefaultTraits<GR,LM>". See \ref DijkstraDefaultTraits
  ///for the documentation of a Dijkstra traits class.
#ifdef DOXYGEN
  template <typename GR, typename LM, typename TR>
#else
  template <typename GR=ListDigraph,
            typename LM=typename GR::template ArcMap<int>,
            typename TR=DijkstraDefaultTraits<GR,LM> >
#endif
  class Dijkstra {
  public:
    ///\ref Exception for uninitialized parameters.

    ///This error represents problems in the initialization of the
    ///parameters of the algorithm.
    class UninitializedParameter : public lemon::UninitializedParameter {
    public:
      virtual const char* what() const throw() {
        return "lemon::Dijkstra::UninitializedParameter";
      }
    };

    ///The type of the digraph the algorithm runs on.
    typedef typename TR::Digraph Digraph;

    ///The type of the length of the arcs.
    typedef typename TR::LengthMap::Value Value;
    ///The type of the map that stores the arc lengths.
    typedef typename TR::LengthMap LengthMap;
    ///\brief The type of the map that stores the predecessor arcs of the
    ///shortest paths.
    typedef typename TR::PredMap PredMap;
    ///The type of the map that stores the distances of the nodes.
    typedef typename TR::DistMap DistMap;
    ///The type of the map that indicates which nodes are processed.
    typedef typename TR::ProcessedMap ProcessedMap;
    ///The type of the paths.
    typedef PredMapPath<Digraph, PredMap> Path;
    ///The cross reference type used for the current heap.
    typedef typename TR::HeapCrossRef HeapCrossRef;
    ///The heap type used by the algorithm.
    typedef typename TR::Heap Heap;
    ///The operation traits class.
    typedef typename TR::OperationTraits OperationTraits;

    ///The traits class.
    typedef TR Traits;

  private:

    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::OutArcIt OutArcIt;

    //Pointer to the underlying digraph.
    const Digraph *G;
    //Pointer to the length map.
    const LengthMap *length;
    //Pointer to the map of predecessors arcs.
    PredMap *_pred;
    //Indicates if _pred is locally allocated (true) or not.
    bool local_pred;
    //Pointer to the map of distances.
    DistMap *_dist;
    //Indicates if _dist is locally allocated (true) or not.
    bool local_dist;
    //Pointer to the map of processed status of the nodes.
    ProcessedMap *_processed;
    //Indicates if _processed is locally allocated (true) or not.
    bool local_processed;
    //Pointer to the heap cross references.
    HeapCrossRef *_heap_cross_ref;
    //Indicates if _heap_cross_ref is locally allocated (true) or not.
    bool local_heap_cross_ref;
    //Pointer to the heap.
    Heap *_heap;
    //Indicates if _heap is locally allocated (true) or not.
    bool local_heap;

    ///Creates the maps if necessary.
    ///\todo Better memory allocation (instead of new).
    void create_maps()
    {
      if(!_pred) {
        local_pred = true;
        _pred = Traits::createPredMap(*G);
      }
      if(!_dist) {
        local_dist = true;
        _dist = Traits::createDistMap(*G);
      }
      if(!_processed) {
        local_processed = true;
        _processed = Traits::createProcessedMap(*G);
      }
      if (!_heap_cross_ref) {
        local_heap_cross_ref = true;
        _heap_cross_ref = Traits::createHeapCrossRef(*G);
      }
      if (!_heap) {
        local_heap = true;
        _heap = Traits::createHeap(*_heap_cross_ref);
      }
    }

  public:

    typedef Dijkstra Create;

    ///\name Named template parameters

    ///@{

    template <class T>
    struct SetPredMapTraits : public Traits {
      typedef T PredMap;
      static PredMap *createPredMap(const Digraph &)
      {
        throw UninitializedParameter();
      }
    };
    ///\brief \ref named-templ-param "Named parameter" for setting
    ///\ref PredMap type.
    ///
    ///\ref named-templ-param "Named parameter" for setting
    ///\ref PredMap type.
    template <class T>
    struct SetPredMap
      : public Dijkstra< Digraph, LengthMap, SetPredMapTraits<T> > {
      typedef Dijkstra< Digraph, LengthMap, SetPredMapTraits<T> > Create;
    };

    template <class T>
    struct SetDistMapTraits : public Traits {
      typedef T DistMap;
      static DistMap *createDistMap(const Digraph &)
      {
        throw UninitializedParameter();
      }
    };
    ///\brief \ref named-templ-param "Named parameter" for setting
    ///\ref DistMap type.
    ///
    ///\ref named-templ-param "Named parameter" for setting
    ///\ref DistMap type.
    template <class T>
    struct SetDistMap
      : public Dijkstra< Digraph, LengthMap, SetDistMapTraits<T> > {
      typedef Dijkstra< Digraph, LengthMap, SetDistMapTraits<T> > Create;
    };

    template <class T>
    struct SetProcessedMapTraits : public Traits {
      typedef T ProcessedMap;
      static ProcessedMap *createProcessedMap(const Digraph &)
      {
        throw UninitializedParameter();
      }
    };
    ///\brief \ref named-templ-param "Named parameter" for setting
    ///\ref ProcessedMap type.
    ///
    ///\ref named-templ-param "Named parameter" for setting
    ///\ref ProcessedMap type.
    template <class T>
    struct SetProcessedMap
      : public Dijkstra< Digraph, LengthMap, SetProcessedMapTraits<T> > {
      typedef Dijkstra< Digraph, LengthMap, SetProcessedMapTraits<T> > Create;
    };

    struct SetStandardProcessedMapTraits : public Traits {
      typedef typename Digraph::template NodeMap<bool> ProcessedMap;
      static ProcessedMap *createProcessedMap(const Digraph &g)
      {
        return new ProcessedMap(g);
      }
    };
    ///\brief \ref named-templ-param "Named parameter" for setting
    ///\ref ProcessedMap type to be <tt>Digraph::NodeMap<bool></tt>.
    ///
    ///\ref named-templ-param "Named parameter" for setting
    ///\ref ProcessedMap type to be <tt>Digraph::NodeMap<bool></tt>.
    ///If you don't set it explicitly, it will be automatically allocated.
    struct SetStandardProcessedMap
      : public Dijkstra< Digraph, LengthMap, SetStandardProcessedMapTraits > {
      typedef Dijkstra< Digraph, LengthMap, SetStandardProcessedMapTraits >
      Create;
    };

    template <class H, class CR>
    struct SetHeapTraits : public Traits {
      typedef CR HeapCrossRef;
      typedef H Heap;
      static HeapCrossRef *createHeapCrossRef(const Digraph &) {
        throw UninitializedParameter();
      }
      static Heap *createHeap(HeapCrossRef &)
      {
        throw UninitializedParameter();
      }
    };
    ///\brief \ref named-templ-param "Named parameter" for setting
    ///heap and cross reference type
    ///
    ///\ref named-templ-param "Named parameter" for setting heap and cross
    ///reference type.
    template <class H, class CR = typename Digraph::template NodeMap<int> >
    struct SetHeap
      : public Dijkstra< Digraph, LengthMap, SetHeapTraits<H, CR> > {
      typedef Dijkstra< Digraph, LengthMap, SetHeapTraits<H, CR> > Create;
    };

    template <class H, class CR>
    struct SetStandardHeapTraits : public Traits {
      typedef CR HeapCrossRef;
      typedef H Heap;
      static HeapCrossRef *createHeapCrossRef(const Digraph &G) {
        return new HeapCrossRef(G);
      }
      static Heap *createHeap(HeapCrossRef &R)
      {
        return new Heap(R);
      }
    };
    ///\brief \ref named-templ-param "Named parameter" for setting
    ///heap and cross reference type with automatic allocation
    ///
    ///\ref named-templ-param "Named parameter" for setting heap and cross
    ///reference type. It can allocate the heap and the cross reference
    ///object if the cross reference's constructor waits for the digraph as
    ///parameter and the heap's constructor waits for the cross reference.
    template <class H, class CR = typename Digraph::template NodeMap<int> >
    struct SetStandardHeap
      : public Dijkstra< Digraph, LengthMap, SetStandardHeapTraits<H, CR> > {
      typedef Dijkstra< Digraph, LengthMap, SetStandardHeapTraits<H, CR> >
      Create;
    };

    template <class T>
    struct SetOperationTraitsTraits : public Traits {
      typedef T OperationTraits;
    };

    /// \brief \ref named-templ-param "Named parameter" for setting
    ///\ref OperationTraits type
    ///
    ///\ref named-templ-param "Named parameter" for setting
    ///\ref OperationTraits type.
    template <class T>
    struct SetOperationTraits
      : public Dijkstra<Digraph, LengthMap, SetOperationTraitsTraits<T> > {
      typedef Dijkstra<Digraph, LengthMap, SetOperationTraitsTraits<T> >
      Create;
    };

    ///@}

  protected:

    Dijkstra() {}

  public:

    ///Constructor.

    ///Constructor.
    ///\param _g The digraph the algorithm runs on.
    ///\param _length The length map used by the algorithm.
    Dijkstra(const Digraph& _g, const LengthMap& _length) :
      G(&_g), length(&_length),
      _pred(NULL), local_pred(false),
      _dist(NULL), local_dist(false),
      _processed(NULL), local_processed(false),
      _heap_cross_ref(NULL), local_heap_cross_ref(false),
      _heap(NULL), local_heap(false)
    { }

    ///Destructor.
    ~Dijkstra()
    {
      if(local_pred) delete _pred;
      if(local_dist) delete _dist;
      if(local_processed) delete _processed;
      if(local_heap_cross_ref) delete _heap_cross_ref;
      if(local_heap) delete _heap;
    }

    ///Sets the length map.

    ///Sets the length map.
    ///\return <tt> (*this) </tt>
    Dijkstra &lengthMap(const LengthMap &m)
    {
      length = &m;
      return *this;
    }

    ///Sets the map that stores the predecessor arcs.

    ///Sets the map that stores the predecessor arcs.
    ///If you don't use this function before calling \ref run(),
    ///it will allocate one. The destructor deallocates this
    ///automatically allocated map, of course.
    ///\return <tt> (*this) </tt>
    Dijkstra &predMap(PredMap &m)
    {
      if(local_pred) {
        delete _pred;
        local_pred=false;
      }
      _pred = &m;
      return *this;
    }

    ///Sets the map that indicates which nodes are processed.

    ///Sets the map that indicates which nodes are processed.
    ///If you don't use this function before calling \ref run(),
    ///it will allocate one. The destructor deallocates this
    ///automatically allocated map, of course.
    ///\return <tt> (*this) </tt>
    Dijkstra &processedMap(ProcessedMap &m)
    {
      if(local_processed) {
        delete _processed;
        local_processed=false;
      }
      _processed = &m;
      return *this;
    }

    ///Sets the map that stores the distances of the nodes.

    ///Sets the map that stores the distances of the nodes calculated by the
    ///algorithm.
    ///If you don't use this function before calling \ref run(),
    ///it will allocate one. The destructor deallocates this
    ///automatically allocated map, of course.
    ///\return <tt> (*this) </tt>
    Dijkstra &distMap(DistMap &m)
    {
      if(local_dist) {
        delete _dist;
        local_dist=false;
      }
      _dist = &m;
      return *this;
    }

    ///Sets the heap and the cross reference used by algorithm.

    ///Sets the heap and the cross reference used by algorithm.
    ///If you don't use this function before calling \ref run(),
    ///it will allocate one. The destructor deallocates this
    ///automatically allocated heap and cross reference, of course.
    ///\return <tt> (*this) </tt>
    Dijkstra &heap(Heap& hp, HeapCrossRef &cr)
    {
      if(local_heap_cross_ref) {
        delete _heap_cross_ref;
        local_heap_cross_ref=false;
      }
      _heap_cross_ref = &cr;
      if(local_heap) {
        delete _heap;
        local_heap=false;
      }
      _heap = &hp;
      return *this;
    }

  private:

    void finalizeNodeData(Node v,Value dst)
    {
      _processed->set(v,true);
      _dist->set(v, dst);
    }

  public:

    ///\name Execution control
    ///The simplest way to execute the algorithm is to use one of the
    ///member functions called \ref lemon::Dijkstra::run() "run()".
    ///\n
    ///If you need more control on the execution, first you must call
    ///\ref lemon::Dijkstra::init() "init()", then you can add several
    ///source nodes with \ref lemon::Dijkstra::addSource() "addSource()".
    ///Finally \ref lemon::Dijkstra::start() "start()" will perform the
    ///actual path computation.

    ///@{

    ///Initializes the internal data structures.

    ///Initializes the internal data structures.
    ///
    void init()
    {
      create_maps();
      _heap->clear();
      for ( NodeIt u(*G) ; u!=INVALID ; ++u ) {
        _pred->set(u,INVALID);
        _processed->set(u,false);
        _heap_cross_ref->set(u,Heap::PRE_HEAP);
      }
    }

    ///Adds a new source node.

    ///Adds a new source node to the priority heap.
    ///The optional second parameter is the initial distance of the node.
    ///
    ///The function checks if the node has already been added to the heap and
    ///it is pushed to the heap only if either it was not in the heap
    ///or the shortest path found till then is shorter than \c dst.
    void addSource(Node s,Value dst=OperationTraits::zero())
    {
      if(_heap->state(s) != Heap::IN_HEAP) {
        _heap->push(s,dst);
      } else if(OperationTraits::less((*_heap)[s], dst)) {
        _heap->set(s,dst);
        _pred->set(s,INVALID);
      }
    }

    ///Processes the next node in the priority heap

    ///Processes the next node in the priority heap.
    ///
    ///\return The processed node.
    ///
    ///\warning The priority heap must not be empty.
    Node processNextNode()
    {
      Node v=_heap->top();
      Value oldvalue=_heap->prio();
      _heap->pop();
      finalizeNodeData(v,oldvalue);

      for(OutArcIt e(*G,v); e!=INVALID; ++e) {
        Node w=G->target(e);
        switch(_heap->state(w)) {
        case Heap::PRE_HEAP:
          _heap->push(w,OperationTraits::plus(oldvalue, (*length)[e]));
          _pred->set(w,e);
          break;
        case Heap::IN_HEAP:
          {
            Value newvalue = OperationTraits::plus(oldvalue, (*length)[e]);
            if ( OperationTraits::less(newvalue, (*_heap)[w]) ) {
              _heap->decrease(w, newvalue);
              _pred->set(w,e);
            }
          }
          break;
        case Heap::POST_HEAP:
          break;
        }
      }
      return v;
    }

    ///The next node to be processed.

    ///Returns the next node to be processed or \c INVALID if the
    ///priority heap is empty.
    Node nextNode() const
    {
      return !_heap->empty()?_heap->top():INVALID;
    }

    ///\brief Returns \c false if there are nodes
    ///to be processed.
    ///
    ///Returns \c false if there are nodes
    ///to be processed in the priority heap.
    bool emptyQueue() const { return _heap->empty(); }

    ///Returns the number of the nodes to be processed in the priority heap

    ///Returns the number of the nodes to be processed in the priority heap.
    ///
    int queueSize() const { return _heap->size(); }

    ///Executes the algorithm.

    ///Executes the algorithm.
    ///
    ///This method runs the %Dijkstra algorithm from the root node(s)
    ///in order to compute the shortest path to each node.
    ///
    ///The algorithm computes
    ///- the shortest path tree (forest),
    ///- the distance of each node from the root(s).
    ///
    ///\pre init() must be called and at least one root node should be
    ///added with addSource() before using this function.
    ///
    ///\note <tt>d.start()</tt> is just a shortcut of the following code.
    ///\code
    ///  while ( !d.emptyQueue() ) {
    ///    d.processNextNode();
    ///  }
    ///\endcode
    void start()
    {
      while ( !emptyQueue() ) processNextNode();
    }

    ///Executes the algorithm until the given target node is reached.

    ///Executes the algorithm until the given target node is reached.
    ///
    ///This method runs the %Dijkstra algorithm from the root node(s)
    ///in order to compute the shortest path to \c dest.
    ///
    ///The algorithm computes
    ///- the shortest path to \c dest,
    ///- the distance of \c dest from the root(s).
    ///
    ///\pre init() must be called and at least one root node should be
    ///added with addSource() before using this function.
    void start(Node dest)
    {
      while ( !_heap->empty() && _heap->top()!=dest ) processNextNode();
      if ( !_heap->empty() ) finalizeNodeData(_heap->top(),_heap->prio());
    }

    ///Executes the algorithm until a condition is met.

    ///Executes the algorithm until a condition is met.
    ///
    ///This method runs the %Dijkstra algorithm from the root node(s) in
    ///order to compute the shortest path to a node \c v with
    /// <tt>nm[v]</tt> true, if such a node can be found.
    ///
    ///\param nm A \c bool (or convertible) node map. The algorithm
    ///will stop when it reaches a node \c v with <tt>nm[v]</tt> true.
    ///
    ///\return The reached node \c v with <tt>nm[v]</tt> true or
    ///\c INVALID if no such node was found.
    ///
    ///\pre init() must be called and at least one root node should be
    ///added with addSource() before using this function.
    template<class NodeBoolMap>
    Node start(const NodeBoolMap &nm)
    {
      while ( !_heap->empty() && !nm[_heap->top()] ) processNextNode();
      if ( _heap->empty() ) return INVALID;
      finalizeNodeData(_heap->top(),_heap->prio());
      return _heap->top();
    }

    ///Runs the algorithm from the given node.

    ///This method runs the %Dijkstra algorithm from node \c s
    ///in order to compute the shortest path to each node.
    ///
    ///The algorithm computes
    ///- the shortest path tree,
    ///- the distance of each node from the root.
    ///
    ///\note <tt>d.run(s)</tt> is just a shortcut of the following code.
    ///\code
    ///  d.init();
    ///  d.addSource(s);
    ///  d.start();
    ///\endcode
    void run(Node s) {
      init();
      addSource(s);
      start();
    }

    ///Finds the shortest path between \c s and \c t.

    ///This method runs the %Dijkstra algorithm from node \c s
    ///in order to compute the shortest path to \c t.
    ///
    ///\return The length of the shortest <tt>s</tt>--<tt>t</tt> path,
    ///if \c t is reachable form \c s, \c 0 otherwise.
    ///
    ///\note Apart from the return value, <tt>d.run(s,t)</tt> is just a
    ///shortcut of the following code.
    ///\code
    ///  d.init();
    ///  d.addSource(s);
    ///  d.start(t);
    ///\endcode
    Value run(Node s,Node t) {
      init();
      addSource(s);
      start(t);
      return (*_pred)[t]==INVALID?OperationTraits::zero():(*_dist)[t];
    }

    ///@}

    ///\name Query Functions
    ///The result of the %Dijkstra algorithm can be obtained using these
    ///functions.\n
    ///Either \ref lemon::Dijkstra::run() "run()" or
    ///\ref lemon::Dijkstra::start() "start()" must be called before
    ///using them.

    ///@{

    ///The shortest path to a node.

    ///Returns the shortest path to a node.
    ///
    ///\warning \c t should be reachable from the root(s).
    ///
    ///\pre Either \ref run() or \ref start() must be called before
    ///using this function.
    Path path(Node t) const { return Path(*G, *_pred, t); }

    ///The distance of a node from the root(s).

    ///Returns the distance of a node from the root(s).
    ///
    ///\warning If node \c v is not reachable from the root(s), then
    ///the return value of this function is undefined.
    ///
    ///\pre Either \ref run() or \ref start() must be called before
    ///using this function.
    Value dist(Node v) const { return (*_dist)[v]; }

    ///Returns the 'previous arc' of the shortest path tree for a node.

    ///This function returns the 'previous arc' of the shortest path
    ///tree for the node \c v, i.e. it returns the last arc of a
    ///shortest path from the root(s) to \c v. It is \c INVALID if \c v
    ///is not reachable from the root(s) or if \c v is a root.
    ///
    ///The shortest path tree used here is equal to the shortest path
    ///tree used in \ref predNode().
    ///
    ///\pre Either \ref run() or \ref start() must be called before
    ///using this function.
    Arc predArc(Node v) const { return (*_pred)[v]; }

    ///Returns the 'previous node' of the shortest path tree for a node.

    ///This function returns the 'previous node' of the shortest path
    ///tree for the node \c v, i.e. it returns the last but one node
    ///from a shortest path from the root(s) to \c v. It is \c INVALID
    ///if \c v is not reachable from the root(s) or if \c v is a root.
    ///
    ///The shortest path tree used here is equal to the shortest path
    ///tree used in \ref predArc().
    ///
    ///\pre Either \ref run() or \ref start() must be called before
    ///using this function.
    Node predNode(Node v) const { return (*_pred)[v]==INVALID ? INVALID:
                                  G->source((*_pred)[v]); }

    ///\brief Returns a const reference to the node map that stores the
    ///distances of the nodes.
    ///
    ///Returns a const reference to the node map that stores the distances
    ///of the nodes calculated by the algorithm.
    ///
    ///\pre Either \ref run() or \ref init()
    ///must be called before using this function.
    const DistMap &distMap() const { return *_dist;}

    ///\brief Returns a const reference to the node map that stores the
    ///predecessor arcs.
    ///
    ///Returns a const reference to the node map that stores the predecessor
    ///arcs, which form the shortest path tree.
    ///
    ///\pre Either \ref run() or \ref init()
    ///must be called before using this function.
    const PredMap &predMap() const { return *_pred;}

    ///Checks if a node is reachable from the root(s).

    ///Returns \c true if \c v is reachable from the root(s).
    ///\pre Either \ref run() or \ref start()
    ///must be called before using this function.
    bool reached(Node v) const { return (*_heap_cross_ref)[v] !=
                                        Heap::PRE_HEAP; }

    ///Checks if a node is processed.

    ///Returns \c true if \c v is processed, i.e. the shortest
    ///path to \c v has already found.
    ///\pre Either \ref run() or \ref start()
    ///must be called before using this function.
    bool processed(Node v) const { return (*_heap_cross_ref)[v] ==
                                          Heap::POST_HEAP; }

    ///The current distance of a node from the root(s).

    ///Returns the current distance of a node from the root(s).
    ///It may be decreased in the following processes.
    ///\pre \c v should be reached but not processed.
    Value currentDist(Node v) const { return (*_heap)[v]; }

    ///@}
  };


  ///Default traits class of dijkstra() function.

  ///Default traits class of dijkstra() function.
  ///\tparam GR The type of the digraph.
  ///\tparam LM The type of the length map.
  template<class GR, class LM>
  struct DijkstraWizardDefaultTraits
  {
    ///The type of the digraph the algorithm runs on.
    typedef GR Digraph;
    ///The type of the map that stores the arc lengths.

    ///The type of the map that stores the arc lengths.
    ///It must meet the \ref concepts::ReadMap "ReadMap" concept.
    typedef LM LengthMap;
    ///The type of the length of the arcs.
    typedef typename LM::Value Value;

    /// Operation traits for Dijkstra algorithm.

    /// This class defines the operations that are used in the algorithm.
    /// \see DijkstraDefaultOperationTraits
    typedef DijkstraDefaultOperationTraits<Value> OperationTraits;

    /// The cross reference type used by the heap.

    /// The cross reference type used by the heap.
    /// Usually it is \c Digraph::NodeMap<int>.
    typedef typename Digraph::template NodeMap<int> HeapCrossRef;
    ///Instantiates a \ref HeapCrossRef.

    ///This function instantiates a \ref HeapCrossRef.
    /// \param g is the digraph, to which we would like to define the
    /// HeapCrossRef.
    /// \todo The digraph alone may be insufficient for the initialization
    static HeapCrossRef *createHeapCrossRef(const Digraph &g)
    {
      return new HeapCrossRef(g);
    }

    ///The heap type used by the Dijkstra algorithm.

    ///The heap type used by the Dijkstra algorithm.
    ///
    ///\sa BinHeap
    ///\sa Dijkstra
    typedef BinHeap<Value, typename Digraph::template NodeMap<int>,
                    std::less<Value> > Heap;

    ///Instantiates a \ref Heap.

    ///This function instantiates a \ref Heap.
    /// \param r is the HeapCrossRef which is used.
    static Heap *createHeap(HeapCrossRef& r)
    {
      return new Heap(r);
    }

    ///\brief The type of the map that stores the predecessor
    ///arcs of the shortest paths.
    ///
    ///The type of the map that stores the predecessor
    ///arcs of the shortest paths.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    typedef NullMap <typename Digraph::Node,typename Digraph::Arc> PredMap;
    ///Instantiates a \ref PredMap.

    ///This function instantiates a \ref PredMap.
    ///\param g is the digraph, to which we would like to define the
    ///\ref PredMap.
    ///\todo The digraph alone may be insufficient to initialize
#ifdef DOXYGEN
    static PredMap *createPredMap(const Digraph &g)
#else
    static PredMap *createPredMap(const Digraph &)
#endif
    {
      return new PredMap();
    }

    ///The type of the map that indicates which nodes are processed.

    ///The type of the map that indicates which nodes are processed.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    ///By default it is a NullMap.
    ///\todo If it is set to a real map,
    ///Dijkstra::processed() should read this.
    ///\todo named parameter to set this type, function to read and write.
    typedef NullMap<typename Digraph::Node,bool> ProcessedMap;
    ///Instantiates a \ref ProcessedMap.

    ///This function instantiates a \ref ProcessedMap.
    ///\param g is the digraph, to which
    ///we would like to define the \ref ProcessedMap.
#ifdef DOXYGEN
    static ProcessedMap *createProcessedMap(const Digraph &g)
#else
    static ProcessedMap *createProcessedMap(const Digraph &)
#endif
    {
      return new ProcessedMap();
    }

    ///The type of the map that stores the distances of the nodes.

    ///The type of the map that stores the distances of the nodes.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    typedef NullMap<typename Digraph::Node,Value> DistMap;
    ///Instantiates a \ref DistMap.

    ///This function instantiates a \ref DistMap.
    ///\param g is the digraph, to which we would like to define
    ///the \ref DistMap
#ifdef DOXYGEN
    static DistMap *createDistMap(const Digraph &g)
#else
    static DistMap *createDistMap(const Digraph &)
#endif
    {
      return new DistMap();
    }
  };

  /// Default traits class used by \ref DijkstraWizard

  /// To make it easier to use Dijkstra algorithm
  /// we have created a wizard class.
  /// This \ref DijkstraWizard class needs default traits,
  /// as well as the \ref Dijkstra class.
  /// The \ref DijkstraWizardBase is a class to be the default traits of the
  /// \ref DijkstraWizard class.
  /// \todo More named parameters are required...
  template<class GR,class LM>
  class DijkstraWizardBase : public DijkstraWizardDefaultTraits<GR,LM>
  {
    typedef DijkstraWizardDefaultTraits<GR,LM> Base;
  protected:
    //The type of the nodes in the digraph.
    typedef typename Base::Digraph::Node Node;

    //Pointer to the digraph the algorithm runs on.
    void *_g;
    //Pointer to the length map
    void *_length;
    //Pointer to the map of predecessors arcs.
    void *_pred;
    //Pointer to the map of distances.
    void *_dist;
    //Pointer to the source node.
    Node _source;

  public:
    /// Constructor.

    /// This constructor does not require parameters, therefore it initiates
    /// all of the attributes to default values (0, INVALID).
    DijkstraWizardBase() : _g(0), _length(0), _pred(0),
                           _dist(0), _source(INVALID) {}

    /// Constructor.

    /// This constructor requires some parameters,
    /// listed in the parameters list.
    /// Others are initiated to 0.
    /// \param g The digraph the algorithm runs on.
    /// \param l The length map.
    /// \param s The source node.
    DijkstraWizardBase(const GR &g,const LM &l, Node s=INVALID) :
      _g(reinterpret_cast<void*>(const_cast<GR*>(&g))),
      _length(reinterpret_cast<void*>(const_cast<LM*>(&l))),
      _pred(0), _dist(0), _source(s) {}

  };

  /// Auxiliary class for the function type interface of Dijkstra algorithm.

  /// This auxiliary class is created to implement the function type
  /// interface of \ref Dijkstra algorithm. It uses the functions and features
  /// of the plain \ref Dijkstra, but it is much simpler to use it.
  /// It should only be used through the \ref dijkstra() function, which makes
  /// it easier to use the algorithm.
  ///
  /// Simplicity means that the way to change the types defined
  /// in the traits class is based on functions that returns the new class
  /// and not on templatable built-in classes.
  /// When using the plain \ref Dijkstra
  /// the new class with the modified type comes from
  /// the original class by using the ::
  /// operator. In the case of \ref DijkstraWizard only
  /// a function have to be called, and it will
  /// return the needed class.
  ///
  /// It does not have own \ref run() method. When its \ref run() method
  /// is called, it initiates a plain \ref Dijkstra object, and calls the
  /// \ref Dijkstra::run() method of it.
  template<class TR>
  class DijkstraWizard : public TR
  {
    typedef TR Base;

    ///The type of the digraph the algorithm runs on.
    typedef typename TR::Digraph Digraph;

    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::OutArcIt OutArcIt;

    ///The type of the map that stores the arc lengths.
    typedef typename TR::LengthMap LengthMap;
    ///The type of the length of the arcs.
    typedef typename LengthMap::Value Value;
    ///\brief The type of the map that stores the predecessor
    ///arcs of the shortest paths.
    typedef typename TR::PredMap PredMap;
    ///The type of the map that stores the distances of the nodes.
    typedef typename TR::DistMap DistMap;
    ///The type of the map that indicates which nodes are processed.
    typedef typename TR::ProcessedMap ProcessedMap;
    ///The heap type used by the dijkstra algorithm.
    typedef typename TR::Heap Heap;

  public:

    /// Constructor.
    DijkstraWizard() : TR() {}

    /// Constructor that requires parameters.

    /// Constructor that requires parameters.
    /// These parameters will be the default values for the traits class.
    DijkstraWizard(const Digraph &g,const LengthMap &l, Node s=INVALID) :
      TR(g,l,s) {}

    ///Copy constructor
    DijkstraWizard(const TR &b) : TR(b) {}

    ~DijkstraWizard() {}

    ///Runs Dijkstra algorithm from a source node.

    ///Runs Dijkstra algorithm from a source node.
    ///The node can be given with the \ref source() function.
    void run()
    {
      if(Base::_source==INVALID) throw UninitializedParameter();
      Dijkstra<Digraph,LengthMap,TR>
        dij(*reinterpret_cast<const Digraph*>(Base::_g),
            *reinterpret_cast<const LengthMap*>(Base::_length));
      if(Base::_pred) dij.predMap(*reinterpret_cast<PredMap*>(Base::_pred));
      if(Base::_dist) dij.distMap(*reinterpret_cast<DistMap*>(Base::_dist));
      dij.run(Base::_source);
    }

    ///Runs Dijkstra algorithm from the given node.

    ///Runs Dijkstra algorithm from the given node.
    ///\param s is the given source.
    void run(Node s)
    {
      Base::_source=s;
      run();
    }

    /// Sets the source node, from which the Dijkstra algorithm runs.

    /// Sets the source node, from which the Dijkstra algorithm runs.
    /// \param s is the source node.
    DijkstraWizard<TR> &source(Node s)
    {
      Base::_source=s;
      return *this;
    }

    template<class T>
    struct SetPredMapBase : public Base {
      typedef T PredMap;
      static PredMap *createPredMap(const Digraph &) { return 0; };
      SetPredMapBase(const TR &b) : TR(b) {}
    };
    ///\brief \ref named-templ-param "Named parameter"
    ///for setting \ref PredMap object.
    ///
    ///\ref named-templ-param "Named parameter"
    ///for setting \ref PredMap object.
    template<class T>
    DijkstraWizard<SetPredMapBase<T> > predMap(const T &t)
    {
      Base::_pred=reinterpret_cast<void*>(const_cast<T*>(&t));
      return DijkstraWizard<SetPredMapBase<T> >(*this);
    }

    template<class T>
    struct SetProcessedMapBase : public Base {
      typedef T ProcessedMap;
      static ProcessedMap *createProcessedMap(const Digraph &) { return 0; };
      SetProcessedMapBase(const TR &b) : TR(b) {}
    };
    ///\brief \ref named-templ-param "Named parameter"
    ///for setting \ref ProcessedMap object.
    ///
    /// \ref named-templ-param "Named parameter"
    ///for setting \ref ProcessedMap object.
    template<class T>
    DijkstraWizard<SetProcessedMapBase<T> > processedMap(const T &t)
    {
      Base::_processed=reinterpret_cast<void*>(const_cast<T*>(&t));
      return DijkstraWizard<SetProcessedMapBase<T> >(*this);
    }

    template<class T>
    struct SetDistMapBase : public Base {
      typedef T DistMap;
      static DistMap *createDistMap(const Digraph &) { return 0; };
      SetDistMapBase(const TR &b) : TR(b) {}
    };
    ///\brief \ref named-templ-param "Named parameter"
    ///for setting \ref DistMap object.
    ///
    ///\ref named-templ-param "Named parameter"
    ///for setting \ref DistMap object.
    template<class T>
    DijkstraWizard<SetDistMapBase<T> > distMap(const T &t)
    {
      Base::_dist=reinterpret_cast<void*>(const_cast<T*>(&t));
      return DijkstraWizard<SetDistMapBase<T> >(*this);
    }

  };

  ///Function type interface for Dijkstra algorithm.

  /// \ingroup shortest_path
  ///Function type interface for Dijkstra algorithm.
  ///
  ///This function also has several
  ///\ref named-templ-func-param "named parameters",
  ///they are declared as the members of class \ref DijkstraWizard.
  ///The following
  ///example shows how to use these parameters.
  ///\code
  ///  dijkstra(g,length,source).predMap(preds).run();
  ///\endcode
  ///\warning Don't forget to put the \ref DijkstraWizard::run() "run()"
  ///to the end of the parameter list.
  ///\sa DijkstraWizard
  ///\sa Dijkstra
  template<class GR, class LM>
  DijkstraWizard<DijkstraWizardBase<GR,LM> >
  dijkstra(const GR &g,const LM &l,typename GR::Node s=INVALID)
  {
    return DijkstraWizard<DijkstraWizardBase<GR,LM> >(g,l,s);
  }

} //END OF NAMESPACE LEMON

#endif
