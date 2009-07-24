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

#ifndef LEMON_BELMANN_FORD_H
#define LEMON_BELMANN_FORD_H

/// \ingroup shortest_path
/// \file
/// \brief Bellman-Ford algorithm.
///

#include <lemon/bits/path_dump.h>
#include <lemon/core.h>
#include <lemon/error.h>
#include <lemon/maps.h>

#include <limits>

namespace lemon {

  /// \brief Default OperationTraits for the BellmanFord algorithm class.
  ///  
  /// It defines all computational operations and constants which are
  /// used in the Bellman-Ford algorithm. The default implementation
  /// is based on the numeric_limits class. If the numeric type does not
  /// have infinity value then the maximum value is used as extremal
  /// infinity value.
  template <
    typename Value, 
    bool has_infinity = std::numeric_limits<Value>::has_infinity>
  struct BellmanFordDefaultOperationTraits {
    /// \brief Gives back the zero value of the type.
    static Value zero() {
      return static_cast<Value>(0);
    }
    /// \brief Gives back the positive infinity value of the type.
    static Value infinity() {
      return std::numeric_limits<Value>::infinity();
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

  template <typename Value>
  struct BellmanFordDefaultOperationTraits<Value, false> {
    static Value zero() {
      return static_cast<Value>(0);
    }
    static Value infinity() {
      return std::numeric_limits<Value>::max();
    }
    static Value plus(const Value& left, const Value& right) {
      if (left == infinity() || right == infinity()) return infinity();
      return left + right;
    }
    static bool less(const Value& left, const Value& right) {
      return left < right;
    }
  };
  
  /// \brief Default traits class of BellmanFord class.
  ///
  /// Default traits class of BellmanFord class.
  /// \param _Digraph Digraph type.
  /// \param _LegthMap Type of length map.
  template<class _Digraph, class _LengthMap>
  struct BellmanFordDefaultTraits {
    /// The digraph type the algorithm runs on. 
    typedef _Digraph Digraph;

    /// \brief The type of the map that stores the arc lengths.
    ///
    /// The type of the map that stores the arc lengths.
    /// It must meet the \ref concepts::ReadMap "ReadMap" concept.
    typedef _LengthMap LengthMap;

    // The type of the length of the arcs.
    typedef typename _LengthMap::Value Value;

    /// \brief Operation traits for Bellman-Ford algorithm.
    ///
    /// It defines the infinity type on the given Value type
    /// and the used operation.
    /// \see BellmanFordDefaultOperationTraits
    typedef BellmanFordDefaultOperationTraits<Value> OperationTraits;
 
    /// \brief The type of the map that stores the last arcs of the 
    /// shortest paths.
    /// 
    /// The type of the map that stores the last
    /// arcs of the shortest paths.
    /// It must meet the \ref concepts::WriteMap "WriteMap" concept.
    ///
    typedef typename Digraph::template NodeMap<typename _Digraph::Arc> PredMap;

    /// \brief Instantiates a PredMap.
    /// 
    /// This function instantiates a \ref PredMap. 
    /// \param digraph is the digraph, to which we would like to define the PredMap.
    static PredMap *createPredMap(const _Digraph& digraph) {
      return new PredMap(digraph);
    }

    /// \brief The type of the map that stores the dists of the nodes.
    ///
    /// The type of the map that stores the dists of the nodes.
    /// It must meet the \ref concepts::WriteMap "WriteMap" concept.
    ///
    typedef typename Digraph::template NodeMap<typename _LengthMap::Value> 
    DistMap;

    /// \brief Instantiates a DistMap.
    ///
    /// This function instantiates a \ref DistMap. 
    /// \param digraph is the digraph, to which we would like to define the 
    /// \ref DistMap
    static DistMap *createDistMap(const _Digraph& digraph) {
      return new DistMap(digraph);
    }

  };
  
  /// \brief %BellmanFord algorithm class.
  ///
  /// \ingroup shortest_path
  /// This class provides an efficient implementation of \c Bellman-Ford 
  /// algorithm. The arc lengths are passed to the algorithm using a
  /// \ref concepts::ReadMap "ReadMap", so it is easy to change it to any 
  /// kind of length.
  ///
  /// The Bellman-Ford algorithm solves the shortest path from one node
  /// problem when the arcs can have negative length but the digraph should
  /// not contain cycles with negative sum of length. If we can assume
  /// that all arc is non-negative in the digraph then the dijkstra algorithm
  /// should be used rather.
  ///
  /// The maximal time complexity of the algorithm is \f$ O(ne) \f$.
  ///
  /// The type of the length is determined by the
  /// \ref concepts::ReadMap::Value "Value" of the length map.
  ///
  /// \param _Digraph The digraph type the algorithm runs on. The default value
  /// is \ref ListDigraph. The value of _Digraph is not used directly by
  /// BellmanFord, it is only passed to \ref BellmanFordDefaultTraits.
  /// \param _LengthMap This read-only ArcMap determines the lengths of the
  /// arcs. The default map type is \ref concepts::Digraph::ArcMap 
  /// "Digraph::ArcMap<int>".  The value of _LengthMap is not used directly 
  /// by BellmanFord, it is only passed to \ref BellmanFordDefaultTraits.  
  /// \param _Traits Traits class to set various data types used by the 
  /// algorithm.  The default traits class is \ref BellmanFordDefaultTraits
  /// "BellmanFordDefaultTraits<_Digraph,_LengthMap>".  See \ref
  /// BellmanFordDefaultTraits for the documentation of a BellmanFord traits
  /// class.
#ifdef DOXYGEN
  template <typename _Digraph, typename _LengthMap, typename _Traits>
#else
  template <typename _Digraph,
	    typename _LengthMap=typename _Digraph::template ArcMap<int>,
	    typename _Traits=BellmanFordDefaultTraits<_Digraph,_LengthMap> >
#endif
  class BellmanFord {
  public:

    typedef _Traits Traits;
    ///The type of the underlying digraph.
    typedef typename _Traits::Digraph Digraph;

    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::OutArcIt OutArcIt;
    
    /// \brief The type of the length of the arcs.
    typedef typename _Traits::LengthMap::Value Value;
    /// \brief The type of the map that stores the arc lengths.
    typedef typename _Traits::LengthMap LengthMap;
    /// \brief The type of the map that stores the last
    /// arcs of the shortest paths.
    typedef typename _Traits::PredMap PredMap;
    /// \brief The type of the map that stores the dists of the nodes.
    typedef typename _Traits::DistMap DistMap;
    /// \brief The operation traits.
    typedef typename _Traits::OperationTraits OperationTraits;
  private:
    /// Pointer to the underlying digraph.
    const Digraph *digraph;
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

    typedef typename Digraph::template NodeMap<bool> MaskMap;
    MaskMap *_mask;

    std::vector<Node> _process;

    /// Creates the maps if necessary.
    void create_maps() {
      if(!_pred) {
	local_pred = true;
	_pred = Traits::createPredMap(*digraph);
      }
      if(!_dist) {
	local_dist = true;
	_dist = Traits::createDistMap(*digraph);
      }
      _mask = new MaskMap(*digraph, false);
    }
    
  public :
 
    typedef BellmanFord Create;

    /// \name Named template parameters

    ///@{

    template <class T>
    struct DefPredMapTraits : public Traits {
      typedef T PredMap;
      static PredMap *createPredMap(const Digraph&) {
        LEMON_ASSERT(false, "PredMap is not initialized");
        return 0; // ignore warnings
      }
    };

    /// \brief \ref named-templ-param "Named parameter" for setting PredMap 
    /// type
    /// \ref named-templ-param "Named parameter" for setting PredMap type
    ///
    template <class T>
    struct SetPredMap 
      : public BellmanFord< Digraph, LengthMap, DefPredMapTraits<T> > {
      typedef BellmanFord< Digraph, LengthMap, DefPredMapTraits<T> > Create;
    };
    
    template <class T>
    struct DefDistMapTraits : public Traits {
      typedef T DistMap;
      static DistMap *createDistMap(const Digraph&) {
        LEMON_ASSERT(false, "DistMap is not initialized");
        return 0; // ignore warnings
      }
    };

    /// \brief \ref named-templ-param "Named parameter" for setting DistMap 
    /// type
    ///
    /// \ref named-templ-param "Named parameter" for setting DistMap type
    ///
    template <class T>
    struct SetDistMap 
      : public BellmanFord< Digraph, LengthMap, DefDistMapTraits<T> > {
      typedef BellmanFord< Digraph, LengthMap, DefDistMapTraits<T> > Create;
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
    struct SetOperationTraits
      : public BellmanFord< Digraph, LengthMap, DefOperationTraitsTraits<T> > {
      typedef BellmanFord< Digraph, LengthMap, DefOperationTraitsTraits<T> >
      Create;
    };
    
    ///@}

  protected:
    
    BellmanFord() {}

  public:      
    
    /// \brief Constructor.
    ///
    /// \param _graph the digraph the algorithm will run on.
    /// \param _length the length map used by the algorithm.
    BellmanFord(const Digraph& _graph, const LengthMap& _length) :
      digraph(&_graph), length(&_length),
      _pred(0), local_pred(false),
      _dist(0), local_dist(false), _mask(0) {}
    
    ///Destructor.
    ~BellmanFord() {
      if(local_pred) delete _pred;
      if(local_dist) delete _dist;
      if(_mask) delete _mask;
    }

    /// \brief Sets the length map.
    ///
    /// Sets the length map.
    /// \return \c (*this)
    BellmanFord &lengthMap(const LengthMap &m) {
      length = &m;
      return *this;
    }

    /// \brief Sets the map storing the predecessor arcs.
    ///
    /// Sets the map storing the predecessor arcs.
    /// If you don't use this function before calling \ref run(),
    /// it will allocate one. The destuctor deallocates this
    /// automatically allocated map, of course.
    /// \return \c (*this)
    BellmanFord &predMap(PredMap &m) {
      if(local_pred) {
	delete _pred;
	local_pred=false;
      }
      _pred = &m;
      return *this;
    }

    /// \brief Sets the map storing the distances calculated by the algorithm.
    ///
    /// Sets the map storing the distances calculated by the algorithm.
    /// If you don't use this function before calling \ref run(),
    /// it will allocate one. The destuctor deallocates this
    /// automatically allocated map, of course.
    /// \return \c (*this)
    BellmanFord &distMap(DistMap &m) {
      if(local_dist) {
	delete _dist;
	local_dist=false;
      }
      _dist = &m;
      return *this;
    }

    /// \name Execution control
    /// The simplest way to execute the algorithm is to use
    /// one of the member functions called \c run(...).
    /// \n
    /// If you need more control on the execution,
    /// first you must call \ref init(), then you can add several source nodes
    /// with \ref addSource().
    /// Finally \ref start() will perform the actual path
    /// computation.

    ///@{

    /// \brief Initializes the internal data structures.
    /// 
    /// Initializes the internal data structures.
    void init(const Value value = OperationTraits::infinity()) {
      create_maps();
      for (NodeIt it(*digraph); it != INVALID; ++it) {
	_pred->set(it, INVALID);
	_dist->set(it, value);
      }
      _process.clear();
      if (OperationTraits::less(value, OperationTraits::infinity())) {
	for (NodeIt it(*digraph); it != INVALID; ++it) {
	  _process.push_back(it);
	  _mask->set(it, true);
	}
      }
    }
    
    /// \brief Adds a new source node.
    ///
    /// Adds a new source node. The optional second parameter is the 
    /// initial distance of the node. It just sets the distance of the 
    /// node to the given value.
    void addSource(Node source, Value dst = OperationTraits::zero()) {
      _dist->set(source, dst);
      if (!(*_mask)[source]) {
	_process.push_back(source);
	_mask->set(source, true);
      }
    }

    /// \brief Executes one round from the Bellman-Ford algorithm.
    ///
    /// If the algoritm calculated the distances in the previous round
    /// exactly for all at most \f$ k \f$ length path lengths then it will
    /// calculate the distances exactly for all at most \f$ k + 1 \f$
    /// length path lengths. With \f$ k \f$ iteration this function
    /// calculates the at most \f$ k \f$ length path lengths.
    ///
    /// \warning The paths with limited arc number cannot be retrieved
    /// easily with \ref path() or \ref predArc() functions. If you
    /// need the shortest path and not just the distance you should store
    /// after each iteration the \ref predMap() map and manually build
    /// the path.
    ///
    /// \return \c true when the algorithm have not found more shorter
    /// paths.
    bool processNextRound() {
      for (int i = 0; i < int(_process.size()); ++i) {
	_mask->set(_process[i], false);
      }
      std::vector<Node> nextProcess;
      std::vector<Value> values(_process.size());
      for (int i = 0; i < int(_process.size()); ++i) {
	values[i] = (*_dist)[_process[i]];
      }
      for (int i = 0; i < int(_process.size()); ++i) {
	for (OutArcIt it(*digraph, _process[i]); it != INVALID; ++it) {
	  Node target = digraph->target(it);
	  Value relaxed = OperationTraits::plus(values[i], (*length)[it]);
	  if (OperationTraits::less(relaxed, (*_dist)[target])) {
	    _pred->set(target, it);
	    _dist->set(target, relaxed);
	    if (!(*_mask)[target]) {
	      _mask->set(target, true);
	      nextProcess.push_back(target);
	    }
	  }	  
	}
      }
      _process.swap(nextProcess);
      return _process.empty();
    }

    /// \brief Executes one weak round from the Bellman-Ford algorithm.
    ///
    /// If the algorithm calculated the distances in the
    /// previous round at least for all at most k length paths then it will
    /// calculate the distances at least for all at most k + 1 length paths.
    /// This function does not make it possible to calculate strictly the
    /// at most k length minimal paths, this is why it is
    /// called just weak round.
    /// \return \c true when the algorithm have not found more shorter paths.
    bool processNextWeakRound() {
      for (int i = 0; i < int(_process.size()); ++i) {
	_mask->set(_process[i], false);
      }
      std::vector<Node> nextProcess;
      for (int i = 0; i < int(_process.size()); ++i) {
	for (OutArcIt it(*digraph, _process[i]); it != INVALID; ++it) {
	  Node target = digraph->target(it);
	  Value relaxed = 
	    OperationTraits::plus((*_dist)[_process[i]], (*length)[it]);
	  if (OperationTraits::less(relaxed, (*_dist)[target])) {
	    _pred->set(target, it);
	    _dist->set(target, relaxed);
	    if (!(*_mask)[target]) {
	      _mask->set(target, true);
	      nextProcess.push_back(target);
	    }
	  }	  
	}
      }
      _process.swap(nextProcess);
      return _process.empty();
    }

    /// \brief Executes the algorithm.
    ///
    /// \pre init() must be called and at least one node should be added
    /// with addSource() before using this function.
    ///
    /// This method runs the %BellmanFord algorithm from the root node(s)
    /// in order to compute the shortest path to each node. The algorithm 
    /// computes 
    /// - The shortest path tree.
    /// - The distance of each node from the root(s).
    void start() {
      int num = countNodes(*digraph) - 1;
      for (int i = 0; i < num; ++i) {
	if (processNextWeakRound()) break;
      }
    }

    /// \brief Executes the algorithm and checks the negative cycles.
    ///
    /// \pre init() must be called and at least one node should be added
    /// with addSource() before using this function. 
    ///
    /// This method runs the %BellmanFord algorithm from the root node(s)
    /// in order to compute the shortest path to each node. The algorithm 
    /// computes 
    /// - The shortest path tree.
    /// - The distance of each node from the root(s).
    /// 
    /// \return \c false if there is a negative cycle in the digraph.
    bool checkedStart() {
      int num = countNodes(*digraph);
      for (int i = 0; i < num; ++i) {
	if (processNextWeakRound()) return true;
      }
      return _process.empty();
    }

    /// \brief Executes the algorithm with path length limit.
    ///
    /// \pre init() must be called and at least one node should be added
    /// with addSource() before using this function.
    ///
    /// This method runs the %BellmanFord algorithm from the root
    /// node(s) in order to compute the shortest path lengths with at
    /// most \c num arc.
    ///
    /// \warning The paths with limited arc number cannot be retrieved
    /// easily with \ref path() or \ref predArc() functions. If you
    /// need the shortest path and not just the distance you should store
    /// after each iteration the \ref predMap() map and manually build
    /// the path.
    ///
    /// The algorithm computes
    /// - The predecessor arc from each node.
    /// - The limited distance of each node from the root(s).
    void limitedStart(int num) {
      for (int i = 0; i < num; ++i) {
	if (processNextRound()) break;
      }
    }
    
    /// \brief Runs %BellmanFord algorithm from node \c s.
    ///    
    /// This method runs the %BellmanFord algorithm from a root node \c s
    /// in order to compute the shortest path to each node. The algorithm 
    /// computes
    /// - The shortest path tree.
    /// - The distance of each node from the root.
    ///
    /// \note d.run(s) is just a shortcut of the following code.
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
    
    /// \brief Runs %BellmanFord algorithm with limited path length 
    /// from node \c s.
    ///    
    /// This method runs the %BellmanFord algorithm from a root node \c s
    /// in order to compute the shortest path with at most \c len arcs 
    /// to each node. The algorithm computes
    /// - The shortest path tree.
    /// - The distance of each node from the root.
    ///
    /// \note d.run(s, num) is just a shortcut of the following code.
    ///\code
    ///  d.init();
    ///  d.addSource(s);
    ///  d.limitedStart(num);
    ///\endcode
    void run(Node s, int num) {
      init();
      addSource(s);
      limitedStart(num);
    }
    
    ///@}

    /// \name Query Functions
    /// The result of the %BellmanFord algorithm can be obtained using these
    /// functions.\n
    /// Before the use of these functions,
    /// either run() or start() must be called.
    
    ///@{

    /// \brief Lemon iterator for get the active nodes.
    ///
    /// Lemon iterator for get the active nodes. This class provides a
    /// common style lemon iterator which gives back a subset of the
    /// nodes. The iterated nodes are active in the algorithm after
    /// the last phase so these should be checked in the next phase to
    /// find augmenting arcs from these.
    class ActiveIt {
    public:

      /// \brief Constructor.
      ///
      /// Constructor for get the nodeset of the variable. 
      ActiveIt(const BellmanFord& algorithm) : _algorithm(&algorithm)
      {
        _index = _algorithm->_process.size() - 1;
      }

      /// \brief Invalid constructor.
      ///
      /// Invalid constructor.
      ActiveIt(Invalid) : _algorithm(0), _index(-1) {}

      /// \brief Conversion to node.
      ///
      /// Conversion to node.
      operator Node() const { 
        return _index >= 0 ? _algorithm->_process[_index] : INVALID;
      }

      /// \brief Increment operator.
      ///
      /// Increment operator.
      ActiveIt& operator++() {
        --_index;
        return *this; 
      }

      bool operator==(const ActiveIt& it) const { 
        return static_cast<Node>(*this) == static_cast<Node>(it); 
      }
      bool operator!=(const ActiveIt& it) const { 
        return static_cast<Node>(*this) != static_cast<Node>(it); 
      }
      bool operator<(const ActiveIt& it) const { 
        return static_cast<Node>(*this) < static_cast<Node>(it); 
      }
      
    private:
      const BellmanFord* _algorithm;
      int _index;
    };

    typedef PredMapPath<Digraph, PredMap> Path;

    /// \brief Gives back the shortest path.
    ///    
    /// Gives back the shortest path.
    /// \pre The \c t should be reachable from the source.
    Path path(Node t) 
    {
      return Path(*digraph, *_pred, t);
    }


    // TODO : implement negative cycle
//     /// \brief Gives back a negative cycle.
//     ///    
//     /// This function gives back a negative cycle.
//     /// If the algorithm have not found yet negative cycle it will give back
//     /// an empty path.
//     Path negativeCycle() {
//       typename Digraph::template NodeMap<int> state(*digraph, 0);
//       for (ActiveIt it(*this); it != INVALID; ++it) {
//         if (state[it] == 0) {
//           for (Node t = it; predArc(t) != INVALID; t = predNode(t)) {
//             if (state[t] == 0) {
//               state[t] = 1;
//             } else if (state[t] == 2) {
//               break;
//             } else {
//               p.clear();
//               typename Path::Builder b(p);
//               b.setStartNode(t);
//               b.pushFront(predArc(t));
//               for(Node s = predNode(t); s != t; s = predNode(s)) {
//                 b.pushFront(predArc(s));
//               }
//               b.commit();
//               return true;
//             }
//           }
//           for (Node t = it; predArc(t) != INVALID; t = predNode(t)) {
//             if (state[t] == 1) {
//               state[t] = 2;
//             } else {
//               break;
//             }
//           }
//         }
//       }
//       return false;
//     }
	  
    /// \brief The distance of a node from the root.
    ///
    /// Returns the distance of a node from the root.
    /// \pre \ref run() must be called before using this function.
    /// \warning If node \c v in unreachable from the root the return value
    /// of this funcion is undefined.
    Value dist(Node v) const { return (*_dist)[v]; }

    /// \brief Returns the 'previous arc' of the shortest path tree.
    ///
    /// For a node \c v it returns the 'previous arc' of the shortest path 
    /// tree, i.e. it returns the last arc of a shortest path from the root 
    /// to \c v. It is \ref INVALID if \c v is unreachable from the root or 
    /// if \c v=s. The shortest path tree used here is equal to the shortest 
    /// path tree used in \ref predNode(). 
    /// \pre \ref run() must be called before using
    /// this function.
    Arc predArc(Node v) const { return (*_pred)[v]; }

    /// \brief Returns the 'previous node' of the shortest path tree.
    ///
    /// For a node \c v it returns the 'previous node' of the shortest path 
    /// tree, i.e. it returns the last but one node from a shortest path from 
    /// the root to \c /v. It is INVALID if \c v is unreachable from the root 
    /// or if \c v=s. The shortest path tree used here is equal to the 
    /// shortest path tree used in \ref predArc().  \pre \ref run() must be 
    /// called before using this function.
    Node predNode(Node v) const { 
      return (*_pred)[v] == INVALID ? INVALID : digraph->source((*_pred)[v]); 
    }
    
    /// \brief Returns a reference to the NodeMap of distances.
    ///
    /// Returns a reference to the NodeMap of distances. \pre \ref run() must
    /// be called before using this function.
    const DistMap &distMap() const { return *_dist;}
 
    /// \brief Returns a reference to the shortest path tree map.
    ///
    /// Returns a reference to the NodeMap of the arcs of the
    /// shortest path tree.
    /// \pre \ref run() must be called before using this function.
    const PredMap &predMap() const { return *_pred; }
 
    /// \brief Checks if a node is reachable from the root.
    ///
    /// Returns \c true if \c v is reachable from the root.
    /// \pre \ref run() must be called before using this function.
    ///
    bool reached(Node v) { return (*_dist)[v] != OperationTraits::infinity(); }
    
    ///@}
  };
 
  /// \brief Default traits class of BellmanFord function.
  ///
  /// Default traits class of BellmanFord function.
  /// \param _Digraph Digraph type.
  /// \param _LengthMap Type of length map.
  template <typename _Digraph, typename _LengthMap>
  struct BellmanFordWizardDefaultTraits {
    /// \brief The digraph type the algorithm runs on. 
    typedef _Digraph Digraph;

    /// \brief The type of the map that stores the arc lengths.
    ///
    /// The type of the map that stores the arc lengths.
    /// It must meet the \ref concepts::ReadMap "ReadMap" concept.
    typedef _LengthMap LengthMap;

    /// \brief The value type of the length map.
    typedef typename _LengthMap::Value Value;

    /// \brief Operation traits for Bellman-Ford algorithm.
    ///
    /// It defines the infinity type on the given Value type
    /// and the used operation.
    /// \see BellmanFordDefaultOperationTraits
    typedef BellmanFordDefaultOperationTraits<Value> OperationTraits;

    /// \brief The type of the map that stores the last
    /// arcs of the shortest paths.
    /// 
    /// The type of the map that stores the last
    /// arcs of the shortest paths.
    /// It must meet the \ref concepts::WriteMap "WriteMap" concept.
    typedef NullMap <typename _Digraph::Node,typename _Digraph::Arc> PredMap;

    /// \brief Instantiates a PredMap.
    /// 
    /// This function instantiates a \ref PredMap. 
    static PredMap *createPredMap(const _Digraph &) {
      return new PredMap();
    }
    /// \brief The type of the map that stores the dists of the nodes.
    ///
    /// The type of the map that stores the dists of the nodes.
    /// It must meet the \ref concepts::WriteMap "WriteMap" concept.
    typedef NullMap<typename Digraph::Node, Value> DistMap;
    /// \brief Instantiates a DistMap.
    ///
    /// This function instantiates a \ref DistMap. 
    static DistMap *createDistMap(const _Digraph &) {
      return new DistMap();
    }
  };
  
  /// \brief Default traits used by \ref BellmanFordWizard
  ///
  /// To make it easier to use BellmanFord algorithm
  /// we have created a wizard class.
  /// This \ref BellmanFordWizard class needs default traits,
  /// as well as the \ref BellmanFord class.
  /// The \ref BellmanFordWizardBase is a class to be the default traits of the
  /// \ref BellmanFordWizard class.
  /// \todo More named parameters are required...
  template<class _Digraph,class _LengthMap>
  class BellmanFordWizardBase 
    : public BellmanFordWizardDefaultTraits<_Digraph,_LengthMap> {

    typedef BellmanFordWizardDefaultTraits<_Digraph,_LengthMap> Base;
  protected:
    /// Type of the nodes in the digraph.
    typedef typename Base::Digraph::Node Node;

    /// Pointer to the underlying digraph.
    void *_graph;
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
    BellmanFordWizardBase() : _graph(0), _length(0), _pred(0),
			   _dist(0), _source(INVALID) {}

    /// Constructor.
    
    /// This constructor requires some parameters,
    /// listed in the parameters list.
    /// Others are initiated to 0.
    /// \param digraph is the initial value of  \ref _graph
    /// \param length is the initial value of  \ref _length
    /// \param source is the initial value of  \ref _source
    BellmanFordWizardBase(const _Digraph& digraph, 
			  const _LengthMap& length, 
			  Node source = INVALID) :
      _graph(reinterpret_cast<void*>(const_cast<_Digraph*>(&digraph))), 
      _length(reinterpret_cast<void*>(const_cast<_LengthMap*>(&length))), 
      _pred(0), _dist(0), _source(source) {}

  };
  
  /// A class to make the usage of BellmanFord algorithm easier

  /// This class is created to make it easier to use BellmanFord algorithm.
  /// It uses the functions and features of the plain \ref BellmanFord,
  /// but it is much simpler to use it.
  ///
  /// Simplicity means that the way to change the types defined
  /// in the traits class is based on functions that returns the new class
  /// and not on templatable built-in classes.
  /// When using the plain \ref BellmanFord
  /// the new class with the modified type comes from
  /// the original class by using the ::
  /// operator. In the case of \ref BellmanFordWizard only
  /// a function have to be called and it will
  /// return the needed class.
  ///
  /// It does not have own \ref run method. When its \ref run method is called
  /// it initiates a plain \ref BellmanFord class, and calls the \ref 
  /// BellmanFord::run method of it.
  template<class _Traits>
  class BellmanFordWizard : public _Traits {
    typedef _Traits Base;

    ///The type of the underlying digraph.
    typedef typename _Traits::Digraph Digraph;

    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::OutArcIt ArcIt;
    
    ///The type of the map that stores the arc lengths.
    typedef typename _Traits::LengthMap LengthMap;

    ///The type of the length of the arcs.
    typedef typename LengthMap::Value Value;

    ///\brief The type of the map that stores the last
    ///arcs of the shortest paths.
    typedef typename _Traits::PredMap PredMap;

    ///The type of the map that stores the dists of the nodes.
    typedef typename _Traits::DistMap DistMap;

  public:
    /// Constructor.
    BellmanFordWizard() : _Traits() {}

    /// \brief Constructor that requires parameters.
    ///
    /// Constructor that requires parameters.
    /// These parameters will be the default values for the traits class.
    BellmanFordWizard(const Digraph& digraph, const LengthMap& length, 
		      Node src = INVALID) 
      : _Traits(digraph, length, src) {}

    /// \brief Copy constructor
    BellmanFordWizard(const _Traits &b) : _Traits(b) {}

    ~BellmanFordWizard() {}

    /// \brief Runs BellmanFord algorithm from a given node.
    ///    
    /// Runs BellmanFord algorithm from a given node.
    /// The node can be given by the \ref source function.
    void run() {
      LEMON_ASSERT(Base::_source != INVALID, "Source node is not given");
      BellmanFord<Digraph,LengthMap,_Traits> 
	bf(*reinterpret_cast<const Digraph*>(Base::_graph), 
           *reinterpret_cast<const LengthMap*>(Base::_length));
      if (Base::_pred) bf.predMap(*reinterpret_cast<PredMap*>(Base::_pred));
      if (Base::_dist) bf.distMap(*reinterpret_cast<DistMap*>(Base::_dist));
      bf.run(Base::_source);
    }

    /// \brief Runs BellmanFord algorithm from the given node.
    ///
    /// Runs BellmanFord algorithm from the given node.
    /// \param src is the given source.
    void run(Node src) {
      Base::_source = src;
      run();
    }

    template<class T>
    struct DefPredMapBase : public Base {
      typedef T PredMap;
      static PredMap *createPredMap(const Digraph &) { return 0; };
      DefPredMapBase(const _Traits &b) : _Traits(b) {}
    };
    
    ///\brief \ref named-templ-param "Named parameter"
    ///function for setting PredMap type
    ///
    /// \ref named-templ-param "Named parameter"
    ///function for setting PredMap type
    ///
    template<class T>
    BellmanFordWizard<DefPredMapBase<T> > predMap(const T &t) 
    {
      Base::_pred=reinterpret_cast<void*>(const_cast<T*>(&t));
      return BellmanFordWizard<DefPredMapBase<T> >(*this);
    }
    
    template<class T>
    struct DefDistMapBase : public Base {
      typedef T DistMap;
      static DistMap *createDistMap(const Digraph &) { return 0; };
      DefDistMapBase(const _Traits &b) : _Traits(b) {}
    };
    
    ///\brief \ref named-templ-param "Named parameter"
    ///function for setting DistMap type
    ///
    /// \ref named-templ-param "Named parameter"
    ///function for setting DistMap type
    ///
    template<class T>
    BellmanFordWizard<DefDistMapBase<T> > distMap(const T &t) {
      Base::_dist=reinterpret_cast<void*>(const_cast<T*>(&t));
      return BellmanFordWizard<DefDistMapBase<T> >(*this);
    }

    template<class T>
    struct DefOperationTraitsBase : public Base {
      typedef T OperationTraits;
      DefOperationTraitsBase(const _Traits &b) : _Traits(b) {}
    };
    
    ///\brief \ref named-templ-param "Named parameter"
    ///function for setting OperationTraits type
    ///
    /// \ref named-templ-param "Named parameter"
    ///function for setting OperationTraits type
    ///
    template<class T>
    BellmanFordWizard<DefOperationTraitsBase<T> > distMap() {
      return BellmanFordWizard<DefDistMapBase<T> >(*this);
    }
    
    /// \brief Sets the source node, from which the BellmanFord algorithm runs.
    ///
    /// Sets the source node, from which the BellmanFord algorithm runs.
    /// \param src is the source node.
    BellmanFordWizard<_Traits>& source(Node src) {
      Base::_source = src;
      return *this;
    }
    
  };
  
  /// \brief Function type interface for BellmanFord algorithm.
  ///
  /// \ingroup shortest_path
  /// Function type interface for BellmanFord algorithm.
  ///
  /// This function also has several \ref named-templ-func-param 
  /// "named parameters", they are declared as the members of class 
  /// \ref BellmanFordWizard.
  /// The following
  /// example shows how to use these parameters.
  ///\code
  /// bellmanford(g,length,source).predMap(preds).run();
  ///\endcode
  /// \warning Don't forget to put the \ref BellmanFordWizard::run() "run()"
  /// to the end of the parameter list.
  /// \sa BellmanFordWizard
  /// \sa BellmanFord
  template<class _Digraph, class _LengthMap>
  BellmanFordWizard<BellmanFordWizardBase<_Digraph,_LengthMap> >
  bellmanFord(const _Digraph& digraph,
	      const _LengthMap& length, 
	      typename _Digraph::Node source = INVALID) {
    return BellmanFordWizard<BellmanFordWizardBase<_Digraph,_LengthMap> >
      (digraph, length, source);
  }

} //END OF NAMESPACE LEMON

#endif

