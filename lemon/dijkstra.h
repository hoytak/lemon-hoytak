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

#ifndef LEMON_DIJKSTRA_H
#define LEMON_DIJKSTRA_H

///\ingroup shortest_path
///\file
///\brief Dijkstra algorithm.

#include <limits>
#include <lemon/list_graph.h>
#include <lemon/bin_heap.h>
#include <lemon/bits/path_dump.h>
#include <lemon/bits/invalid.h>
#include <lemon/error.h>
#include <lemon/maps.h>

namespace lemon {

  /// \brief Default OperationTraits for the Dijkstra algorithm class.
  ///  
  /// It defines all computational operations and constants which are
  /// used in the Dijkstra algorithm.
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
    /// \brief Gives back true only if the first value less than the second.
    static bool less(const Value& left, const Value& right) {
      return left < right;
    }
  };

  /// \brief Widest path OperationTraits for the Dijkstra algorithm class.
  ///  
  /// It defines all computational operations and constants which are
  /// used in the Dijkstra algorithm for widest path computation.
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
    /// \brief Gives back true only if the first value less than the second.
    static bool less(const Value& left, const Value& right) {
      return left < right;
    }
  };
  
  ///Default traits class of Dijkstra class.

  ///Default traits class of Dijkstra class.
  ///\tparam GR Digraph type.
  ///\tparam LM Type of length map.
  template<class GR, class LM>
  struct DijkstraDefaultTraits
  {
    ///The digraph type the algorithm runs on. 
    typedef GR Digraph;
    ///The type of the map that stores the arc lengths.

    ///The type of the map that stores the arc lengths.
    ///It must meet the \ref concepts::ReadMap "ReadMap" concept.
    typedef LM LengthMap;
    //The type of the length of the arcs.
    typedef typename LM::Value Value;
    /// Operation traits for Dijkstra algorithm.

    /// It defines the used operation by the algorithm.
    /// \see DijkstraDefaultOperationTraits
    typedef DijkstraDefaultOperationTraits<Value> OperationTraits;
    /// The cross reference type used by heap.


    /// The cross reference type used by heap.
    /// Usually it is \c Digraph::NodeMap<int>.
    typedef typename Digraph::template NodeMap<int> HeapCrossRef;
    ///Instantiates a HeapCrossRef.

    ///This function instantiates a \c HeapCrossRef. 
    /// \param G is the digraph, to which we would like to define the 
    /// HeapCrossRef.
    static HeapCrossRef *createHeapCrossRef(const GR &G) 
    {
      return new HeapCrossRef(G);
    }
    
    ///The heap type used by Dijkstra algorithm.

    ///The heap type used by Dijkstra algorithm.
    ///
    ///\sa BinHeap
    ///\sa Dijkstra
    typedef BinHeap<typename LM::Value, HeapCrossRef, std::less<Value> > Heap;

    static Heap *createHeap(HeapCrossRef& R) 
    {
      return new Heap(R);
    }

    ///\brief The type of the map that stores the last
    ///arcs of the shortest paths.
    /// 
    ///The type of the map that stores the last
    ///arcs of the shortest paths.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    ///
    typedef typename Digraph::template NodeMap<typename GR::Arc> PredMap;
    ///Instantiates a PredMap.
 
    ///This function instantiates a \c PredMap. 
    ///\param G is the digraph, to which we would like to define the PredMap.
    ///\todo The digraph alone may be insufficient for the initialization
    static PredMap *createPredMap(const GR &G) 
    {
      return new PredMap(G);
    }

    ///The type of the map that stores whether a nodes is processed.
 
    ///The type of the map that stores whether a nodes is processed.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    ///By default it is a NullMap.
    ///\todo If it is set to a real map,
    ///Dijkstra::processed() should read this.
    ///\todo named parameter to set this type, function to read and write.
    typedef NullMap<typename Digraph::Node,bool> ProcessedMap;
    ///Instantiates a ProcessedMap.
 
    ///This function instantiates a \c ProcessedMap. 
    ///\param g is the digraph, to which
    ///we would like to define the \c ProcessedMap
#ifdef DOXYGEN
    static ProcessedMap *createProcessedMap(const GR &g)
#else
    static ProcessedMap *createProcessedMap(const GR &)
#endif
    {
      return new ProcessedMap();
    }
    ///The type of the map that stores the dists of the nodes.
 
    ///The type of the map that stores the dists of the nodes.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    ///
    typedef typename Digraph::template NodeMap<typename LM::Value> DistMap;
    ///Instantiates a DistMap.
 
    ///This function instantiates a \ref DistMap. 
    ///\param G is the digraph, to which we would like to define the \ref DistMap
    static DistMap *createDistMap(const GR &G)
    {
      return new DistMap(G);
    }
  };
  
  ///%Dijkstra algorithm class.
  
  /// \ingroup shortest_path
  ///This class provides an efficient implementation of %Dijkstra algorithm.
  ///The arc lengths are passed to the algorithm using a
  ///\ref concepts::ReadMap "ReadMap",
  ///so it is easy to change it to any kind of length.
  ///
  ///The type of the length is determined by the
  ///\ref concepts::ReadMap::Value "Value" of the length map.
  ///
  ///It is also possible to change the underlying priority heap.
  ///
  ///\tparam GR The digraph type the algorithm runs on. The default value
  ///is \ref ListDigraph. The value of GR is not used directly by
  ///Dijkstra, it is only passed to \ref DijkstraDefaultTraits.
  ///\tparam LM This read-only ArcMap determines the lengths of the
  ///arcs. It is read once for each arc, so the map may involve in
  ///relatively time consuming process to compute the arc length if
  ///it is necessary. The default map type is \ref
  ///concepts::Digraph::ArcMap "Digraph::ArcMap<int>".  The value
  ///of LM is not used directly by Dijkstra, it is only passed to \ref
  ///DijkstraDefaultTraits.  
  ///\tparam TR Traits class to set
  ///various data types used by the algorithm.  The default traits
  ///class is \ref DijkstraDefaultTraits
  ///"DijkstraDefaultTraits<GR,LM>".  See \ref
  ///DijkstraDefaultTraits for the documentation of a Dijkstra traits
  ///class.

#ifdef DOXYGEN
  template <typename GR, typename LM, typename TR>
#else
  template <typename GR=ListDigraph,
	    typename LM=typename GR::template ArcMap<int>,
	    typename TR=DijkstraDefaultTraits<GR,LM> >
#endif
  class Dijkstra {
  public:
    /**
     * \brief \ref Exception for uninitialized parameters.
     *
     * This error represents problems in the initialization
     * of the parameters of the algorithms.
     */
    class UninitializedParameter : public lemon::UninitializedParameter {
    public:
      virtual const char* what() const throw() {
	return "lemon::Dijkstra::UninitializedParameter";
      }
    };

    typedef TR Traits;
    ///The type of the underlying digraph.
    typedef typename TR::Digraph Digraph;
    ///\e
    typedef typename Digraph::Node Node;
    ///\e
    typedef typename Digraph::NodeIt NodeIt;
    ///\e
    typedef typename Digraph::Arc Arc;
    ///\e
    typedef typename Digraph::OutArcIt OutArcIt;
    
    ///The type of the length of the arcs.
    typedef typename TR::LengthMap::Value Value;
    ///The type of the map that stores the arc lengths.
    typedef typename TR::LengthMap LengthMap;
    ///\brief The type of the map that stores the last
    ///arcs of the shortest paths.
    typedef typename TR::PredMap PredMap;
    ///The type of the map indicating if a node is processed.
    typedef typename TR::ProcessedMap ProcessedMap;
    ///The type of the map that stores the dists of the nodes.
    typedef typename TR::DistMap DistMap;
    ///The cross reference type used for the current heap.
    typedef typename TR::HeapCrossRef HeapCrossRef;
    ///The heap type used by the dijkstra algorithm.
    typedef typename TR::Heap Heap;
    ///The operation traits.
    typedef typename TR::OperationTraits OperationTraits;
  private:
    /// Pointer to the underlying digraph.
    const Digraph *G;
    /// Pointer to the length map
    const LengthMap *length;
    ///Pointer to the map of predecessors arcs.
    PredMap *_pred;
    ///Indicates if \ref _pred is locally allocated (\c true) or not.
    bool local_pred;
    ///Pointer to the map of distances.
    DistMap *_dist;
    ///Indicates if \ref _dist is locally allocated (\c true) or not.
    bool local_dist;
    ///Pointer to the map of processed status of the nodes.
    ProcessedMap *_processed;
    ///Indicates if \ref _processed is locally allocated (\c true) or not.
    bool local_processed;
    ///Pointer to the heap cross references.
    HeapCrossRef *_heap_cross_ref;
    ///Indicates if \ref _heap_cross_ref is locally allocated (\c true) or not.
    bool local_heap_cross_ref;
    ///Pointer to the heap.
    Heap *_heap;
    ///Indicates if \ref _heap is locally allocated (\c true) or not.
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
    
  public :

    typedef Dijkstra Create;
 
    ///\name Named template parameters

    ///@{

    template <class T>
    struct DefPredMapTraits : public Traits {
      typedef T PredMap;
      static PredMap *createPredMap(const Digraph &)
      {
	throw UninitializedParameter();
      }
    };
    ///\ref named-templ-param "Named parameter" for setting PredMap type

    ///\ref named-templ-param "Named parameter" for setting PredMap type
    ///
    template <class T>
    struct DefPredMap 
      : public Dijkstra< Digraph,	LengthMap, DefPredMapTraits<T> > {
      typedef Dijkstra< Digraph,	LengthMap, DefPredMapTraits<T> > Create;
    };
    
    template <class T>
    struct DefDistMapTraits : public Traits {
      typedef T DistMap;
      static DistMap *createDistMap(const Digraph &)
      {
	throw UninitializedParameter();
      }
    };
    ///\ref named-templ-param "Named parameter" for setting DistMap type

    ///\ref named-templ-param "Named parameter" for setting DistMap type
    ///
    template <class T>
    struct DefDistMap 
      : public Dijkstra< Digraph, LengthMap, DefDistMapTraits<T> > { 
      typedef Dijkstra< Digraph, LengthMap, DefDistMapTraits<T> > Create;
    };
    
    template <class T>
    struct DefProcessedMapTraits : public Traits {
      typedef T ProcessedMap;
      static ProcessedMap *createProcessedMap(const Digraph &G) 
      {
	throw UninitializedParameter();
      }
    };
    ///\ref named-templ-param "Named parameter" for setting ProcessedMap type

    ///\ref named-templ-param "Named parameter" for setting ProcessedMap type
    ///
    template <class T>
    struct DefProcessedMap 
      : public Dijkstra< Digraph,	LengthMap, DefProcessedMapTraits<T> > { 
      typedef Dijkstra< Digraph,	LengthMap, DefProcessedMapTraits<T> > Create;
    };
    
    struct DefDigraphProcessedMapTraits : public Traits {
      typedef typename Digraph::template NodeMap<bool> ProcessedMap;
      static ProcessedMap *createProcessedMap(const Digraph &G) 
      {
	return new ProcessedMap(G);
      }
    };
    ///\brief \ref named-templ-param "Named parameter"
    ///for setting the ProcessedMap type to be Digraph::NodeMap<bool>.
    ///
    ///\ref named-templ-param "Named parameter"
    ///for setting the ProcessedMap type to be Digraph::NodeMap<bool>.
    ///If you don't set it explicitely, it will be automatically allocated.
    template <class T>
    struct DefProcessedMapToBeDefaultMap 
      : public Dijkstra< Digraph, LengthMap, DefDigraphProcessedMapTraits> {
      typedef Dijkstra< Digraph, LengthMap, DefDigraphProcessedMapTraits> Create;
    };

    template <class H, class CR>
    struct DefHeapTraits : public Traits {
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
    ///reference type
    ///
    template <class H, class CR = typename Digraph::template NodeMap<int> >
    struct DefHeap
      : public Dijkstra< Digraph,	LengthMap, DefHeapTraits<H, CR> > { 
      typedef Dijkstra< Digraph,	LengthMap, DefHeapTraits<H, CR> > Create;
    };

    template <class H, class CR>
    struct DefStandardHeapTraits : public Traits {
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
    struct DefStandardHeap
      : public Dijkstra< Digraph,	LengthMap, DefStandardHeapTraits<H, CR> > { 
      typedef Dijkstra< Digraph,	LengthMap, DefStandardHeapTraits<H, CR> > 
      Create;
    };

    template <class T>
    struct DefOperationTraitsTraits : public Traits {
      typedef T OperationTraits;
    };
    
    /// \brief \ref named-templ-param "Named parameter" for setting 
    /// OperationTraits type
    ///
    /// \ref named-templ-param "Named parameter" for setting OperationTraits
    /// type
    template <class T>
    struct DefOperationTraits
      : public Dijkstra<Digraph, LengthMap, DefOperationTraitsTraits<T> > {
      typedef Dijkstra<Digraph, LengthMap, DefOperationTraitsTraits<T> >
      Create;
    };
    
    ///@}


  protected:

    Dijkstra() {}

  public:      
    
    ///Constructor.
    
    ///\param _G the digraph the algorithm will run on.
    ///\param _length the length map used by the algorithm.
    Dijkstra(const Digraph& _G, const LengthMap& _length) :
      G(&_G), length(&_length),
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

    ///Sets the map storing the predecessor arcs.

    ///Sets the map storing the predecessor arcs.
    ///If you don't use this function before calling \ref run(),
    ///it will allocate one. The destuctor deallocates this
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

    ///Sets the map storing the distances calculated by the algorithm.

    ///Sets the map storing the distances calculated by the algorithm.
    ///If you don't use this function before calling \ref run(),
    ///it will allocate one. The destuctor deallocates this
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
    ///it will allocate one. The destuctor deallocates this
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

    typedef PredMapPath<Digraph, PredMap> Path;

    ///\name Execution control
    ///The simplest way to execute the algorithm is to use
    ///one of the member functions called \c run(...).
    ///\n
    ///If you need more control on the execution,
    ///first you must call \ref init(), then you can add several source nodes
    ///with \ref addSource().
    ///Finally \ref start() will perform the actual path
    ///computation.

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
    ///
    ///The optional second parameter is the initial distance of the node.
    ///
    ///It checks if the node has already been added to the heap and
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
    ///\warning The priority heap must not be empty!
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

    ///Next node to be processed.
    
    ///Next node to be processed.
    ///
    ///\return The next node to be processed or INVALID if the priority heap
    /// is empty.
    Node nextNode()
    { 
      return !_heap->empty()?_heap->top():INVALID;
    }
 
    ///\brief Returns \c false if there are nodes
    ///to be processed in the priority heap
    ///
    ///Returns \c false if there are nodes
    ///to be processed in the priority heap
    bool emptyQueue() { return _heap->empty(); }
    ///Returns the number of the nodes to be processed in the priority heap

    ///Returns the number of the nodes to be processed in the priority heap
    ///
    int queueSize() { return _heap->size(); }
    
    ///Executes the algorithm.

    ///Executes the algorithm.
    ///
    ///\pre init() must be called and at least one node should be added
    ///with addSource() before using this function.
    ///
    ///This method runs the %Dijkstra algorithm from the root node(s)
    ///in order to
    ///compute the
    ///shortest path to each node. The algorithm computes
    ///- The shortest path tree.
    ///- The distance of each node from the root(s).
    ///
    void start()
    {
      while ( !_heap->empty() ) processNextNode();
    }
    
    ///Executes the algorithm until \c dest is reached.

    ///Executes the algorithm until \c dest is reached.
    ///
    ///\pre init() must be called and at least one node should be added
    ///with addSource() before using this function.
    ///
    ///This method runs the %Dijkstra algorithm from the root node(s)
    ///in order to
    ///compute the
    ///shortest path to \c dest. The algorithm computes
    ///- The shortest path to \c  dest.
    ///- The distance of \c dest from the root(s).
    ///
    void start(Node dest)
    {
      while ( !_heap->empty() && _heap->top()!=dest ) processNextNode();
      if ( !_heap->empty() ) finalizeNodeData(_heap->top(),_heap->prio());
    }
    
    ///Executes the algorithm until a condition is met.

    ///Executes the algorithm until a condition is met.
    ///
    ///\pre init() must be called and at least one node should be added
    ///with addSource() before using this function.
    ///
    ///\param nm must be a bool (or convertible) node map. The algorithm
    ///will stop when it reaches a node \c v with <tt>nm[v]</tt> true.
    ///
    ///\return The reached node \c v with <tt>nm[v]</tt> true or
    ///\c INVALID if no such node was found.
    template<class NodeBoolMap>
    Node start(const NodeBoolMap &nm)
    {
      while ( !_heap->empty() && !nm[_heap->top()] ) processNextNode();
      if ( _heap->empty() ) return INVALID;
      finalizeNodeData(_heap->top(),_heap->prio());
      return _heap->top();
    }
    
    ///Runs %Dijkstra algorithm from node \c s.
    
    ///This method runs the %Dijkstra algorithm from a root node \c s
    ///in order to
    ///compute the
    ///shortest path to each node. The algorithm computes
    ///- The shortest path tree.
    ///- The distance of each node from the root.
    ///
    ///\note d.run(s) is just a shortcut of the following code.
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
    
    ///Finds the shortest path between \c s and \c t.
    ///
    ///\return The length of the shortest s---t path if there exists one,
    ///0 otherwise.
    ///\note Apart from the return value, d.run(s) is
    ///just a shortcut of the following code.
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
    ///Before the use of these functions,
    ///either run() or start() must be called.
    
    ///@{

    ///Gives back the shortest path.
    
    ///Gives back the shortest path.
    ///\pre The \c t should be reachable from the source.
    Path path(Node t) 
    {
      return Path(*G, *_pred, t);
    }

    ///The distance of a node from the root.

    ///Returns the distance of a node from the root.
    ///\pre \ref run() must be called before using this function.
    ///\warning If node \c v in unreachable from the root the return value
    ///of this funcion is undefined.
    Value dist(Node v) const { return (*_dist)[v]; }

    ///The current distance of a node from the root.

    ///Returns the current distance of a node from the root.
    ///It may be decreased in the following processes.
    ///\pre \c node should be reached but not processed
    Value currentDist(Node v) const { return (*_heap)[v]; }

    ///Returns the 'previous arc' of the shortest path tree.

    ///For a node \c v it returns the 'previous arc' of the shortest path tree,
    ///i.e. it returns the last arc of a shortest path from the root to \c
    ///v. It is \ref INVALID
    ///if \c v is unreachable from the root or if \c v=s. The
    ///shortest path tree used here is equal to the shortest path tree used in
    ///\ref predNode().  \pre \ref run() must be called before using
    ///this function.
    Arc predArc(Node v) const { return (*_pred)[v]; }

    ///Returns the 'previous node' of the shortest path tree.

    ///For a node \c v it returns the 'previous node' of the shortest path tree,
    ///i.e. it returns the last but one node from a shortest path from the
    ///root to \c /v. It is INVALID if \c v is unreachable from the root or if
    ///\c v=s. The shortest path tree used here is equal to the shortest path
    ///tree used in \ref predArc().  \pre \ref run() must be called before
    ///using this function.
    Node predNode(Node v) const { return (*_pred)[v]==INVALID ? INVALID:
				  G->source((*_pred)[v]); }
    
    ///Returns a reference to the NodeMap of distances.

    ///Returns a reference to the NodeMap of distances. \pre \ref run() must
    ///be called before using this function.
    const DistMap &distMap() const { return *_dist;}
 
    ///Returns a reference to the shortest path tree map.

    ///Returns a reference to the NodeMap of the arcs of the
    ///shortest path tree.
    ///\pre \ref run() must be called before using this function.
    const PredMap &predMap() const { return *_pred;}
 
    ///Checks if a node is reachable from the root.

    ///Returns \c true if \c v is reachable from the root.
    ///\warning The source nodes are inditated as unreached.
    ///\pre \ref run() must be called before using this function.
    ///
    bool reached(Node v) { return (*_heap_cross_ref)[v] != Heap::PRE_HEAP; }

    ///Checks if a node is processed.

    ///Returns \c true if \c v is processed, i.e. the shortest
    ///path to \c v has already found.
    ///\pre \ref run() must be called before using this function.
    ///
    bool processed(Node v) { return (*_heap_cross_ref)[v] == Heap::POST_HEAP; }
    
    ///@}
  };




 
  ///Default traits class of Dijkstra function.

  ///Default traits class of Dijkstra function.
  ///\tparam GR Digraph type.
  ///\tparam LM Type of length map.
  template<class GR, class LM>
  struct DijkstraWizardDefaultTraits
  {
    ///The digraph type the algorithm runs on. 
    typedef GR Digraph;
    ///The type of the map that stores the arc lengths.

    ///The type of the map that stores the arc lengths.
    ///It must meet the \ref concepts::ReadMap "ReadMap" concept.
    typedef LM LengthMap;
    //The type of the length of the arcs.
    typedef typename LM::Value Value;
    /// Operation traits for Dijkstra algorithm.

    /// It defines the used operation by the algorithm.
    /// \see DijkstraDefaultOperationTraits
    typedef DijkstraDefaultOperationTraits<Value> OperationTraits;
    ///The heap type used by Dijkstra algorithm.

    /// The cross reference type used by heap.

    /// The cross reference type used by heap.
    /// Usually it is \c Digraph::NodeMap<int>.
    typedef typename Digraph::template NodeMap<int> HeapCrossRef;
    ///Instantiates a HeapCrossRef.

    ///This function instantiates a \ref HeapCrossRef. 
    /// \param G is the digraph, to which we would like to define the 
    /// HeapCrossRef.
    /// \todo The digraph alone may be insufficient for the initialization
    static HeapCrossRef *createHeapCrossRef(const GR &G) 
    {
      return new HeapCrossRef(G);
    }
    
    ///The heap type used by Dijkstra algorithm.

    ///The heap type used by Dijkstra algorithm.
    ///
    ///\sa BinHeap
    ///\sa Dijkstra
    typedef BinHeap<typename LM::Value, typename GR::template NodeMap<int>,
		    std::less<Value> > Heap;

    static Heap *createHeap(HeapCrossRef& R) 
    {
      return new Heap(R);
    }

    ///\brief The type of the map that stores the last
    ///arcs of the shortest paths.
    /// 
    ///The type of the map that stores the last
    ///arcs of the shortest paths.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    ///
    typedef NullMap <typename GR::Node,typename GR::Arc> PredMap;
    ///Instantiates a PredMap.
 
    ///This function instantiates a \ref PredMap. 
    ///\param g is the digraph, to which we would like to define the PredMap.
    ///\todo The digraph alone may be insufficient for the initialization
#ifdef DOXYGEN
    static PredMap *createPredMap(const GR &g) 
#else
    static PredMap *createPredMap(const GR &) 
#endif
    {
      return new PredMap();
    }
    ///The type of the map that stores whether a nodes is processed.
 
    ///The type of the map that stores whether a nodes is processed.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    ///By default it is a NullMap.
    ///\todo If it is set to a real map,
    ///Dijkstra::processed() should read this.
    ///\todo named parameter to set this type, function to read and write.
    typedef NullMap<typename Digraph::Node,bool> ProcessedMap;
    ///Instantiates a ProcessedMap.
 
    ///This function instantiates a \ref ProcessedMap. 
    ///\param g is the digraph, to which
    ///we would like to define the \ref ProcessedMap
#ifdef DOXYGEN
    static ProcessedMap *createProcessedMap(const GR &g)
#else
    static ProcessedMap *createProcessedMap(const GR &)
#endif
    {
      return new ProcessedMap();
    }
    ///The type of the map that stores the dists of the nodes.
 
    ///The type of the map that stores the dists of the nodes.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    ///
    typedef NullMap<typename Digraph::Node,typename LM::Value> DistMap;
    ///Instantiates a DistMap.
 
    ///This function instantiates a \ref DistMap. 
    ///\param g is the digraph, to which we would like to define the \ref DistMap
#ifdef DOXYGEN
    static DistMap *createDistMap(const GR &g)
#else
    static DistMap *createDistMap(const GR &)
#endif
    {
      return new DistMap();
    }
  };
  
  /// Default traits used by \ref DijkstraWizard

  /// To make it easier to use Dijkstra algorithm
  ///we have created a wizard class.
  /// This \ref DijkstraWizard class needs default traits,
  ///as well as the \ref Dijkstra class.
  /// The \ref DijkstraWizardBase is a class to be the default traits of the
  /// \ref DijkstraWizard class.
  /// \todo More named parameters are required...
  template<class GR,class LM>
  class DijkstraWizardBase : public DijkstraWizardDefaultTraits<GR,LM>
  {

    typedef DijkstraWizardDefaultTraits<GR,LM> Base;
  protected:
    /// Type of the nodes in the digraph.
    typedef typename Base::Digraph::Node Node;

    /// Pointer to the underlying digraph.
    void *_g;
    /// Pointer to the length map
    void *_length;
    ///Pointer to the map of predecessors arcs.
    void *_pred;
    ///Pointer to the map of distances.
    void *_dist;
    ///Pointer to the source node.
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
    /// \param g is the initial value of  \ref _g
    /// \param l is the initial value of  \ref _length
    /// \param s is the initial value of  \ref _source
    DijkstraWizardBase(const GR &g,const LM &l, Node s=INVALID) :
      _g(reinterpret_cast<void*>(const_cast<GR*>(&g))), 
      _length(reinterpret_cast<void*>(const_cast<LM*>(&l))), 
      _pred(0), _dist(0), _source(s) {}

  };
  
  /// A class to make the usage of Dijkstra algorithm easier

  /// This class is created to make it easier to use Dijkstra algorithm.
  /// It uses the functions and features of the plain \ref Dijkstra,
  /// but it is much simpler to use it.
  ///
  /// Simplicity means that the way to change the types defined
  /// in the traits class is based on functions that returns the new class
  /// and not on templatable built-in classes.
  /// When using the plain \ref Dijkstra
  /// the new class with the modified type comes from
  /// the original class by using the ::
  /// operator. In the case of \ref DijkstraWizard only
  /// a function have to be called and it will
  /// return the needed class.
  ///
  /// It does not have own \ref run method. When its \ref run method is called
  /// it initiates a plain \ref Dijkstra class, and calls the \ref 
  /// Dijkstra::run method of it.
  template<class TR>
  class DijkstraWizard : public TR
  {
    typedef TR Base;

    ///The type of the underlying digraph.
    typedef typename TR::Digraph Digraph;
    //\e
    typedef typename Digraph::Node Node;
    //\e
    typedef typename Digraph::NodeIt NodeIt;
    //\e
    typedef typename Digraph::Arc Arc;
    //\e
    typedef typename Digraph::OutArcIt OutArcIt;
    
    ///The type of the map that stores the arc lengths.
    typedef typename TR::LengthMap LengthMap;
    ///The type of the length of the arcs.
    typedef typename LengthMap::Value Value;
    ///\brief The type of the map that stores the last
    ///arcs of the shortest paths.
    typedef typename TR::PredMap PredMap;
    ///The type of the map that stores the dists of the nodes.
    typedef typename TR::DistMap DistMap;
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

    ///Runs Dijkstra algorithm from a given node.
    
    ///Runs Dijkstra algorithm from a given node.
    ///The node can be given by the \ref source function.
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

    template<class T>
    struct DefPredMapBase : public Base {
      typedef T PredMap;
      static PredMap *createPredMap(const Digraph &) { return 0; };
      DefPredMapBase(const TR &b) : TR(b) {}
    };
    
    ///\brief \ref named-templ-param "Named parameter"
    ///function for setting PredMap type
    ///
    /// \ref named-templ-param "Named parameter"
    ///function for setting PredMap type
    ///
    template<class T>
    DijkstraWizard<DefPredMapBase<T> > predMap(const T &t) 
    {
      Base::_pred=reinterpret_cast<void*>(const_cast<T*>(&t));
      return DijkstraWizard<DefPredMapBase<T> >(*this);
    }
    
    template<class T>
    struct DefDistMapBase : public Base {
      typedef T DistMap;
      static DistMap *createDistMap(const Digraph &) { return 0; };
      DefDistMapBase(const TR &b) : TR(b) {}
    };
    
    ///\brief \ref named-templ-param "Named parameter"
    ///function for setting DistMap type
    ///
    /// \ref named-templ-param "Named parameter"
    ///function for setting DistMap type
    ///
    template<class T>
    DijkstraWizard<DefDistMapBase<T> > distMap(const T &t) 
    {
      Base::_dist=reinterpret_cast<void*>(const_cast<T*>(&t));
      return DijkstraWizard<DefDistMapBase<T> >(*this);
    }
    
    /// Sets the source node, from which the Dijkstra algorithm runs.

    /// Sets the source node, from which the Dijkstra algorithm runs.
    /// \param s is the source node.
    DijkstraWizard<TR> &source(Node s) 
    {
      Base::_source=s;
      return *this;
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
