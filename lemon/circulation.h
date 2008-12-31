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

#ifndef LEMON_CIRCULATION_H
#define LEMON_CIRCULATION_H

#include <lemon/tolerance.h>
#include <lemon/elevator.h>

///\ingroup max_flow
///\file
///\brief Push-relabel algorithm for finding a feasible circulation.
///
namespace lemon {

  /// \brief Default traits class of Circulation class.
  ///
  /// Default traits class of Circulation class.
  /// \tparam _Diraph Digraph type.
  /// \tparam _LCapMap Lower bound capacity map type.
  /// \tparam _UCapMap Upper bound capacity map type.
  /// \tparam _DeltaMap Delta map type.
  template <typename _Diraph, typename _LCapMap,
            typename _UCapMap, typename _DeltaMap>
  struct CirculationDefaultTraits {

    /// \brief The type of the digraph the algorithm runs on.
    typedef _Diraph Digraph;

    /// \brief The type of the map that stores the circulation lower
    /// bound.
    ///
    /// The type of the map that stores the circulation lower bound.
    /// It must meet the \ref concepts::ReadMap "ReadMap" concept.
    typedef _LCapMap LCapMap;

    /// \brief The type of the map that stores the circulation upper
    /// bound.
    ///
    /// The type of the map that stores the circulation upper bound.
    /// It must meet the \ref concepts::ReadMap "ReadMap" concept.
    typedef _UCapMap UCapMap;

    /// \brief The type of the map that stores the lower bound for
    /// the supply of the nodes.
    ///
    /// The type of the map that stores the lower bound for the supply
    /// of the nodes. It must meet the \ref concepts::ReadMap "ReadMap"
    /// concept.
    typedef _DeltaMap DeltaMap;

    /// \brief The type of the flow values.
    typedef typename DeltaMap::Value Value;

    /// \brief The type of the map that stores the flow values.
    ///
    /// The type of the map that stores the flow values.
    /// It must meet the \ref concepts::ReadWriteMap "ReadWriteMap" concept.
    typedef typename Digraph::template ArcMap<Value> FlowMap;

    /// \brief Instantiates a FlowMap.
    ///
    /// This function instantiates a \ref FlowMap.
    /// \param digraph The digraph, to which we would like to define
    /// the flow map.
    static FlowMap* createFlowMap(const Digraph& digraph) {
      return new FlowMap(digraph);
    }

    /// \brief The elevator type used by the algorithm.
    ///
    /// The elevator type used by the algorithm.
    ///
    /// \sa Elevator
    /// \sa LinkedElevator
    typedef lemon::Elevator<Digraph, typename Digraph::Node> Elevator;

    /// \brief Instantiates an Elevator.
    ///
    /// This function instantiates an \ref Elevator.
    /// \param digraph The digraph, to which we would like to define
    /// the elevator.
    /// \param max_level The maximum level of the elevator.
    static Elevator* createElevator(const Digraph& digraph, int max_level) {
      return new Elevator(digraph, max_level);
    }

    /// \brief The tolerance used by the algorithm
    ///
    /// The tolerance used by the algorithm to handle inexact computation.
    typedef lemon::Tolerance<Value> Tolerance;

  };

  /**
     \brief Push-relabel algorithm for the network circulation problem.

     \ingroup max_flow
     This class implements a push-relabel algorithm for the network
     circulation problem.
     It is to find a feasible circulation when lower and upper bounds
     are given for the flow values on the arcs and lower bounds
     are given for the supply values of the nodes.

     The exact formulation of this problem is the following.
     Let \f$G=(V,A)\f$ be a digraph,
     \f$lower, upper: A\rightarrow\mathbf{R}^+_0\f$,
     \f$delta: V\rightarrow\mathbf{R}\f$. Find a feasible circulation
     \f$f: A\rightarrow\mathbf{R}^+_0\f$ so that
     \f[ \sum_{a\in\delta_{out}(v)} f(a) - \sum_{a\in\delta_{in}(v)} f(a)
     \geq delta(v) \quad \forall v\in V, \f]
     \f[ lower(a)\leq f(a) \leq upper(a) \quad \forall a\in A. \f]
     \note \f$delta(v)\f$ specifies a lower bound for the supply of node
     \f$v\f$. It can be either positive or negative, however note that
     \f$\sum_{v\in V}delta(v)\f$ should be zero or negative in order to
     have a feasible solution.

     \note A special case of this problem is when
     \f$\sum_{v\in V}delta(v) = 0\f$. Then the supply of each node \f$v\f$
     will be \e equal \e to \f$delta(v)\f$, if a circulation can be found.
     Thus a feasible solution for the
     \ref min_cost_flow "minimum cost flow" problem can be calculated
     in this way.

     \tparam _Digraph The type of the digraph the algorithm runs on.
     \tparam _LCapMap The type of the lower bound capacity map. The default
     map type is \ref concepts::Digraph::ArcMap "_Digraph::ArcMap<int>".
     \tparam _UCapMap The type of the upper bound capacity map. The default
     map type is \c _LCapMap.
     \tparam _DeltaMap The type of the map that stores the lower bound
     for the supply of the nodes. The default map type is
     \c _Digraph::ArcMap<_UCapMap::Value>.
  */
#ifdef DOXYGEN
template< typename _Digraph,
          typename _LCapMap,
          typename _UCapMap,
          typename _DeltaMap,
          typename _Traits >
#else
template< typename _Digraph,
          typename _LCapMap = typename _Digraph::template ArcMap<int>,
          typename _UCapMap = _LCapMap,
          typename _DeltaMap = typename _Digraph::
                               template NodeMap<typename _UCapMap::Value>,
          typename _Traits=CirculationDefaultTraits<_Digraph, _LCapMap,
                                                    _UCapMap, _DeltaMap> >
#endif
  class Circulation {
  public:

    ///The \ref CirculationDefaultTraits "traits class" of the algorithm.
    typedef _Traits Traits;
    ///The type of the digraph the algorithm runs on.
    typedef typename Traits::Digraph Digraph;
    ///The type of the flow values.
    typedef typename Traits::Value Value;

    /// The type of the lower bound capacity map.
    typedef typename Traits::LCapMap LCapMap;
    /// The type of the upper bound capacity map.
    typedef typename Traits::UCapMap UCapMap;
    /// \brief The type of the map that stores the lower bound for
    /// the supply of the nodes.
    typedef typename Traits::DeltaMap DeltaMap;
    ///The type of the flow map.
    typedef typename Traits::FlowMap FlowMap;

    ///The type of the elevator.
    typedef typename Traits::Elevator Elevator;
    ///The type of the tolerance.
    typedef typename Traits::Tolerance Tolerance;

  private:

    TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

    const Digraph &_g;
    int _node_num;

    const LCapMap *_lo;
    const UCapMap *_up;
    const DeltaMap *_delta;

    FlowMap *_flow;
    bool _local_flow;

    Elevator* _level;
    bool _local_level;

    typedef typename Digraph::template NodeMap<Value> ExcessMap;
    ExcessMap* _excess;

    Tolerance _tol;
    int _el;

  public:

    typedef Circulation Create;

    ///\name Named Template Parameters

    ///@{

    template <typename _FlowMap>
    struct SetFlowMapTraits : public Traits {
      typedef _FlowMap FlowMap;
      static FlowMap *createFlowMap(const Digraph&) {
        LEMON_ASSERT(false, "FlowMap is not initialized");
        return 0; // ignore warnings
      }
    };

    /// \brief \ref named-templ-param "Named parameter" for setting
    /// FlowMap type
    ///
    /// \ref named-templ-param "Named parameter" for setting FlowMap
    /// type.
    template <typename _FlowMap>
    struct SetFlowMap
      : public Circulation<Digraph, LCapMap, UCapMap, DeltaMap,
                           SetFlowMapTraits<_FlowMap> > {
      typedef Circulation<Digraph, LCapMap, UCapMap, DeltaMap,
                          SetFlowMapTraits<_FlowMap> > Create;
    };

    template <typename _Elevator>
    struct SetElevatorTraits : public Traits {
      typedef _Elevator Elevator;
      static Elevator *createElevator(const Digraph&, int) {
        LEMON_ASSERT(false, "Elevator is not initialized");
        return 0; // ignore warnings
      }
    };

    /// \brief \ref named-templ-param "Named parameter" for setting
    /// Elevator type
    ///
    /// \ref named-templ-param "Named parameter" for setting Elevator
    /// type. If this named parameter is used, then an external
    /// elevator object must be passed to the algorithm using the
    /// \ref elevator(Elevator&) "elevator()" function before calling
    /// \ref run() or \ref init().
    /// \sa SetStandardElevator
    template <typename _Elevator>
    struct SetElevator
      : public Circulation<Digraph, LCapMap, UCapMap, DeltaMap,
                           SetElevatorTraits<_Elevator> > {
      typedef Circulation<Digraph, LCapMap, UCapMap, DeltaMap,
                          SetElevatorTraits<_Elevator> > Create;
    };

    template <typename _Elevator>
    struct SetStandardElevatorTraits : public Traits {
      typedef _Elevator Elevator;
      static Elevator *createElevator(const Digraph& digraph, int max_level) {
        return new Elevator(digraph, max_level);
      }
    };

    /// \brief \ref named-templ-param "Named parameter" for setting
    /// Elevator type with automatic allocation
    ///
    /// \ref named-templ-param "Named parameter" for setting Elevator
    /// type with automatic allocation.
    /// The Elevator should have standard constructor interface to be
    /// able to automatically created by the algorithm (i.e. the
    /// digraph and the maximum level should be passed to it).
    /// However an external elevator object could also be passed to the
    /// algorithm with the \ref elevator(Elevator&) "elevator()" function
    /// before calling \ref run() or \ref init().
    /// \sa SetElevator
    template <typename _Elevator>
    struct SetStandardElevator
      : public Circulation<Digraph, LCapMap, UCapMap, DeltaMap,
                       SetStandardElevatorTraits<_Elevator> > {
      typedef Circulation<Digraph, LCapMap, UCapMap, DeltaMap,
                      SetStandardElevatorTraits<_Elevator> > Create;
    };

    /// @}

  protected:

    Circulation() {}

  public:

    /// The constructor of the class.

    /// The constructor of the class.
    /// \param g The digraph the algorithm runs on.
    /// \param lo The lower bound capacity of the arcs.
    /// \param up The upper bound capacity of the arcs.
    /// \param delta The lower bound for the supply of the nodes.
    Circulation(const Digraph &g,const LCapMap &lo,
                const UCapMap &up,const DeltaMap &delta)
      : _g(g), _node_num(),
        _lo(&lo),_up(&up),_delta(&delta),_flow(0),_local_flow(false),
        _level(0), _local_level(false), _excess(0), _el() {}

    /// Destructor.
    ~Circulation() {
      destroyStructures();
    }


  private:

    void createStructures() {
      _node_num = _el = countNodes(_g);

      if (!_flow) {
        _flow = Traits::createFlowMap(_g);
        _local_flow = true;
      }
      if (!_level) {
        _level = Traits::createElevator(_g, _node_num);
        _local_level = true;
      }
      if (!_excess) {
        _excess = new ExcessMap(_g);
      }
    }

    void destroyStructures() {
      if (_local_flow) {
        delete _flow;
      }
      if (_local_level) {
        delete _level;
      }
      if (_excess) {
        delete _excess;
      }
    }

  public:

    /// Sets the lower bound capacity map.

    /// Sets the lower bound capacity map.
    /// \return <tt>(*this)</tt>
    Circulation& lowerCapMap(const LCapMap& map) {
      _lo = &map;
      return *this;
    }

    /// Sets the upper bound capacity map.

    /// Sets the upper bound capacity map.
    /// \return <tt>(*this)</tt>
    Circulation& upperCapMap(const LCapMap& map) {
      _up = &map;
      return *this;
    }

    /// Sets the lower bound map for the supply of the nodes.

    /// Sets the lower bound map for the supply of the nodes.
    /// \return <tt>(*this)</tt>
    Circulation& deltaMap(const DeltaMap& map) {
      _delta = &map;
      return *this;
    }

    /// \brief Sets the flow map.
    ///
    /// Sets the flow map.
    /// If you don't use this function before calling \ref run() or
    /// \ref init(), an instance will be allocated automatically.
    /// The destructor deallocates this automatically allocated map,
    /// of course.
    /// \return <tt>(*this)</tt>
    Circulation& flowMap(FlowMap& map) {
      if (_local_flow) {
        delete _flow;
        _local_flow = false;
      }
      _flow = &map;
      return *this;
    }

    /// \brief Sets the elevator used by algorithm.
    ///
    /// Sets the elevator used by algorithm.
    /// If you don't use this function before calling \ref run() or
    /// \ref init(), an instance will be allocated automatically.
    /// The destructor deallocates this automatically allocated elevator,
    /// of course.
    /// \return <tt>(*this)</tt>
    Circulation& elevator(Elevator& elevator) {
      if (_local_level) {
        delete _level;
        _local_level = false;
      }
      _level = &elevator;
      return *this;
    }

    /// \brief Returns a const reference to the elevator.
    ///
    /// Returns a const reference to the elevator.
    ///
    /// \pre Either \ref run() or \ref init() must be called before
    /// using this function.
    const Elevator& elevator() const {
      return *_level;
    }

    /// \brief Sets the tolerance used by algorithm.
    ///
    /// Sets the tolerance used by algorithm.
    Circulation& tolerance(const Tolerance& tolerance) const {
      _tol = tolerance;
      return *this;
    }

    /// \brief Returns a const reference to the tolerance.
    ///
    /// Returns a const reference to the tolerance.
    const Tolerance& tolerance() const {
      return tolerance;
    }

    /// \name Execution Control
    /// The simplest way to execute the algorithm is to call \ref run().\n
    /// If you need more control on the initial solution or the execution,
    /// first you have to call one of the \ref init() functions, then
    /// the \ref start() function.

    ///@{

    /// Initializes the internal data structures.

    /// Initializes the internal data structures and sets all flow values
    /// to the lower bound.
    void init()
    {
      createStructures();

      for(NodeIt n(_g);n!=INVALID;++n) {
        _excess->set(n, (*_delta)[n]);
      }

      for (ArcIt e(_g);e!=INVALID;++e) {
        _flow->set(e, (*_lo)[e]);
        _excess->set(_g.target(e), (*_excess)[_g.target(e)] + (*_flow)[e]);
        _excess->set(_g.source(e), (*_excess)[_g.source(e)] - (*_flow)[e]);
      }

      // global relabeling tested, but in general case it provides
      // worse performance for random digraphs
      _level->initStart();
      for(NodeIt n(_g);n!=INVALID;++n)
        _level->initAddItem(n);
      _level->initFinish();
      for(NodeIt n(_g);n!=INVALID;++n)
        if(_tol.positive((*_excess)[n]))
          _level->activate(n);
    }

    /// Initializes the internal data structures using a greedy approach.

    /// Initializes the internal data structures using a greedy approach
    /// to construct the initial solution.
    void greedyInit()
    {
      createStructures();

      for(NodeIt n(_g);n!=INVALID;++n) {
        _excess->set(n, (*_delta)[n]);
      }

      for (ArcIt e(_g);e!=INVALID;++e) {
        if (!_tol.positive((*_excess)[_g.target(e)] + (*_up)[e])) {
          _flow->set(e, (*_up)[e]);
          _excess->set(_g.target(e), (*_excess)[_g.target(e)] + (*_up)[e]);
          _excess->set(_g.source(e), (*_excess)[_g.source(e)] - (*_up)[e]);
        } else if (_tol.positive((*_excess)[_g.target(e)] + (*_lo)[e])) {
          _flow->set(e, (*_lo)[e]);
          _excess->set(_g.target(e), (*_excess)[_g.target(e)] + (*_lo)[e]);
          _excess->set(_g.source(e), (*_excess)[_g.source(e)] - (*_lo)[e]);
        } else {
          Value fc = -(*_excess)[_g.target(e)];
          _flow->set(e, fc);
          _excess->set(_g.target(e), 0);
          _excess->set(_g.source(e), (*_excess)[_g.source(e)] - fc);
        }
      }

      _level->initStart();
      for(NodeIt n(_g);n!=INVALID;++n)
        _level->initAddItem(n);
      _level->initFinish();
      for(NodeIt n(_g);n!=INVALID;++n)
        if(_tol.positive((*_excess)[n]))
          _level->activate(n);
    }

    ///Executes the algorithm

    ///This function executes the algorithm.
    ///
    ///\return \c true if a feasible circulation is found.
    ///
    ///\sa barrier()
    ///\sa barrierMap()
    bool start()
    {

      Node act;
      Node bact=INVALID;
      Node last_activated=INVALID;
      while((act=_level->highestActive())!=INVALID) {
        int actlevel=(*_level)[act];
        int mlevel=_node_num;
        Value exc=(*_excess)[act];

        for(OutArcIt e(_g,act);e!=INVALID; ++e) {
          Node v = _g.target(e);
          Value fc=(*_up)[e]-(*_flow)[e];
          if(!_tol.positive(fc)) continue;
          if((*_level)[v]<actlevel) {
            if(!_tol.less(fc, exc)) {
              _flow->set(e, (*_flow)[e] + exc);
              _excess->set(v, (*_excess)[v] + exc);
              if(!_level->active(v) && _tol.positive((*_excess)[v]))
                _level->activate(v);
              _excess->set(act,0);
              _level->deactivate(act);
              goto next_l;
            }
            else {
              _flow->set(e, (*_up)[e]);
              _excess->set(v, (*_excess)[v] + fc);
              if(!_level->active(v) && _tol.positive((*_excess)[v]))
                _level->activate(v);
              exc-=fc;
            }
          }
          else if((*_level)[v]<mlevel) mlevel=(*_level)[v];
        }
        for(InArcIt e(_g,act);e!=INVALID; ++e) {
          Node v = _g.source(e);
          Value fc=(*_flow)[e]-(*_lo)[e];
          if(!_tol.positive(fc)) continue;
          if((*_level)[v]<actlevel) {
            if(!_tol.less(fc, exc)) {
              _flow->set(e, (*_flow)[e] - exc);
              _excess->set(v, (*_excess)[v] + exc);
              if(!_level->active(v) && _tol.positive((*_excess)[v]))
                _level->activate(v);
              _excess->set(act,0);
              _level->deactivate(act);
              goto next_l;
            }
            else {
              _flow->set(e, (*_lo)[e]);
              _excess->set(v, (*_excess)[v] + fc);
              if(!_level->active(v) && _tol.positive((*_excess)[v]))
                _level->activate(v);
              exc-=fc;
            }
          }
          else if((*_level)[v]<mlevel) mlevel=(*_level)[v];
        }

        _excess->set(act, exc);
        if(!_tol.positive(exc)) _level->deactivate(act);
        else if(mlevel==_node_num) {
          _level->liftHighestActiveToTop();
          _el = _node_num;
          return false;
        }
        else {
          _level->liftHighestActive(mlevel+1);
          if(_level->onLevel(actlevel)==0) {
            _el = actlevel;
            return false;
          }
        }
      next_l:
        ;
      }
      return true;
    }

    /// Runs the algorithm.

    /// This function runs the algorithm.
    ///
    /// \return \c true if a feasible circulation is found.
    ///
    /// \note Apart from the return value, c.run() is just a shortcut of
    /// the following code.
    /// \code
    ///   c.greedyInit();
    ///   c.start();
    /// \endcode
    bool run() {
      greedyInit();
      return start();
    }

    /// @}

    /// \name Query Functions
    /// The results of the circulation algorithm can be obtained using
    /// these functions.\n
    /// Either \ref run() or \ref start() should be called before
    /// using them.

    ///@{

    /// \brief Returns the flow on the given arc.
    ///
    /// Returns the flow on the given arc.
    ///
    /// \pre Either \ref run() or \ref init() must be called before
    /// using this function.
    Value flow(const Arc& arc) const {
      return (*_flow)[arc];
    }

    /// \brief Returns a const reference to the flow map.
    ///
    /// Returns a const reference to the arc map storing the found flow.
    ///
    /// \pre Either \ref run() or \ref init() must be called before
    /// using this function.
    const FlowMap& flowMap() const {
      return *_flow;
    }

    /**
       \brief Returns \c true if the given node is in a barrier.

       Barrier is a set \e B of nodes for which

       \f[ \sum_{a\in\delta_{out}(B)} upper(a) -
           \sum_{a\in\delta_{in}(B)} lower(a) < \sum_{v\in B}delta(v) \f]

       holds. The existence of a set with this property prooves that a
       feasible circualtion cannot exist.

       This function returns \c true if the given node is in the found
       barrier. If a feasible circulation is found, the function
       gives back \c false for every node.

       \pre Either \ref run() or \ref init() must be called before
       using this function.

       \sa barrierMap()
       \sa checkBarrier()
    */
    bool barrier(const Node& node) const
    {
      return (*_level)[node] >= _el;
    }

    /// \brief Gives back a barrier.
    ///
    /// This function sets \c bar to the characteristic vector of the
    /// found barrier. \c bar should be a \ref concepts::WriteMap "writable"
    /// node map with \c bool (or convertible) value type.
    ///
    /// If a feasible circulation is found, the function gives back an
    /// empty set, so \c bar[v] will be \c false for all nodes \c v.
    ///
    /// \note This function calls \ref barrier() for each node,
    /// so it runs in \f$O(n)\f$ time.
    ///
    /// \pre Either \ref run() or \ref init() must be called before
    /// using this function.
    ///
    /// \sa barrier()
    /// \sa checkBarrier()
    template<class BarrierMap>
    void barrierMap(BarrierMap &bar) const
    {
      for(NodeIt n(_g);n!=INVALID;++n)
        bar.set(n, (*_level)[n] >= _el);
    }

    /// @}

    /// \name Checker Functions
    /// The feasibility of the results can be checked using
    /// these functions.\n
    /// Either \ref run() or \ref start() should be called before
    /// using them.

    ///@{

    ///Check if the found flow is a feasible circulation

    ///Check if the found flow is a feasible circulation,
    ///
    bool checkFlow() const {
      for(ArcIt e(_g);e!=INVALID;++e)
        if((*_flow)[e]<(*_lo)[e]||(*_flow)[e]>(*_up)[e]) return false;
      for(NodeIt n(_g);n!=INVALID;++n)
        {
          Value dif=-(*_delta)[n];
          for(InArcIt e(_g,n);e!=INVALID;++e) dif-=(*_flow)[e];
          for(OutArcIt e(_g,n);e!=INVALID;++e) dif+=(*_flow)[e];
          if(_tol.negative(dif)) return false;
        }
      return true;
    }

    ///Check whether or not the last execution provides a barrier

    ///Check whether or not the last execution provides a barrier.
    ///\sa barrier()
    ///\sa barrierMap()
    bool checkBarrier() const
    {
      Value delta=0;
      for(NodeIt n(_g);n!=INVALID;++n)
        if(barrier(n))
          delta-=(*_delta)[n];
      for(ArcIt e(_g);e!=INVALID;++e)
        {
          Node s=_g.source(e);
          Node t=_g.target(e);
          if(barrier(s)&&!barrier(t)) delta+=(*_up)[e];
          else if(barrier(t)&&!barrier(s)) delta-=(*_lo)[e];
        }
      return _tol.negative(delta);
    }

    /// @}

  };

}

#endif
