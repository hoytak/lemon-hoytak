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

#ifndef LEMON_CIRCULATION_H
#define LEMON_CIRCULATION_H

#include <iostream>
#include <queue>
#include <lemon/tolerance.h>
#include <lemon/elevator.h>

///\ingroup max_flow
///\file
///\brief Push-prelabel algorithm for finding a feasible circulation.
///
namespace lemon {

  /// \brief Default traits class of Circulation class.
  ///
  /// Default traits class of Circulation class.
  /// \param _Graph Digraph type.
  /// \param _CapacityMap Type of capacity map.
  template <typename _Graph, typename _LCapMap,
            typename _UCapMap, typename _DeltaMap>
  struct CirculationDefaultTraits {

    /// \brief The digraph type the algorithm runs on.
    typedef _Graph Digraph;

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

    /// \brief The type of the map that stores the upper bound of
    /// node excess.
    ///
    /// The type of the map that stores the lower bound of node
    /// excess. It must meet the \ref concepts::ReadMap "ReadMap"
    /// concept.
    typedef _DeltaMap DeltaMap;

    /// \brief The type of the length of the arcs.
    typedef typename DeltaMap::Value Value;

    /// \brief The map type that stores the flow values.
    ///
    /// The map type that stores the flow values.
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

    /// \brief The eleavator type used by Circulation algorithm.
    ///
    /// The elevator type used by Circulation algorithm.
    ///
    /// \sa Elevator
    /// \sa LinkedElevator
    typedef lemon::Elevator<Digraph, typename Digraph::Node> Elevator;

    /// \brief Instantiates an Elevator.
    ///
    /// This function instantiates a \ref Elevator.
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

  ///Push-relabel algorithm for the Network Circulation Problem.

  /**
     \ingroup max_flow
     This class implements a push-relabel algorithm
     or the Network Circulation Problem.
     The exact formulation of this problem is the following.
     \f[\sum_{e\in\rho(v)}x(e)-\sum_{e\in\delta(v)}x(e)\leq
     -delta(v)\quad \forall v\in V \f]
     \f[ lo(e)\leq x(e) \leq up(e) \quad \forall e\in E \f]
  */
  template<class _Graph,
           class _LCapMap=typename _Graph::template ArcMap<int>,
           class _UCapMap=_LCapMap,
           class _DeltaMap=typename _Graph::template NodeMap<
             typename _UCapMap::Value>,
           class _Traits=CirculationDefaultTraits<_Graph, _LCapMap,
                                                  _UCapMap, _DeltaMap> >
  class Circulation {

    typedef _Traits Traits;
    typedef typename Traits::Digraph Digraph;
    TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

    typedef typename Traits::Value Value;

    typedef typename Traits::LCapMap LCapMap;
    typedef typename Traits::UCapMap UCapMap;
    typedef typename Traits::DeltaMap DeltaMap;
    typedef typename Traits::FlowMap FlowMap;
    typedef typename Traits::Elevator Elevator;
    typedef typename Traits::Tolerance Tolerance;

    typedef typename Digraph::template NodeMap<Value> ExcessMap;

    const Digraph &_g;
    int _node_num;

    const LCapMap *_lo;
    const UCapMap *_up;
    const DeltaMap *_delta;

    FlowMap *_flow;
    bool _local_flow;

    Elevator* _level;
    bool _local_level;

    ExcessMap* _excess;

    Tolerance _tol;
    int _el;

  public:

    typedef Circulation Create;

    ///\name Named template parameters

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
    /// type
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
    /// type
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
    /// Elevator type
    ///
    /// \ref named-templ-param "Named parameter" for setting Elevator
    /// type. The Elevator should be standard constructor interface, ie.
    /// the digraph and the maximum level should be passed to it.
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
    /// \param delta The lower bound on node excess.
    Circulation(const Digraph &g,const LCapMap &lo,
                const UCapMap &up,const DeltaMap &delta)
      : _g(g), _node_num(),
        _lo(&lo),_up(&up),_delta(&delta),_flow(0),_local_flow(false),
        _level(0), _local_level(false), _excess(0), _el() {}

    /// Destrcutor.
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
    /// \return \c (*this)
    Circulation& lowerCapMap(const LCapMap& map) {
      _lo = &map;
      return *this;
    }

    /// Sets the upper bound capacity map.

    /// Sets the upper bound capacity map.
    /// \return \c (*this)
    Circulation& upperCapMap(const LCapMap& map) {
      _up = &map;
      return *this;
    }

    /// Sets the lower bound map on excess.

    /// Sets the lower bound map on excess.
    /// \return \c (*this)
    Circulation& deltaMap(const DeltaMap& map) {
      _delta = &map;
      return *this;
    }

    /// Sets the flow map.

    /// Sets the flow map.
    /// \return \c (*this)
    Circulation& flowMap(FlowMap& map) {
      if (_local_flow) {
        delete _flow;
        _local_flow = false;
      }
      _flow = &map;
      return *this;
    }

    /// Returns the flow map.

    /// \return The flow map.
    ///
    const FlowMap& flowMap() {
      return *_flow;
    }

    /// Sets the elevator.

    /// Sets the elevator.
    /// \return \c (*this)
    Circulation& elevator(Elevator& elevator) {
      if (_local_level) {
        delete _level;
        _local_level = false;
      }
      _level = &elevator;
      return *this;
    }

    /// Returns the elevator.

    /// \return The elevator.
    ///
    const Elevator& elevator() {
      return *_level;
    }

    /// Sets the tolerance used by algorithm.

    /// Sets the tolerance used by algorithm.
    ///
    Circulation& tolerance(const Tolerance& tolerance) const {
      _tol = tolerance;
      return *this;
    }

    /// Returns the tolerance used by algorithm.

    /// Returns the tolerance used by algorithm.
    ///
    const Tolerance& tolerance() const {
      return tolerance;
    }

    /// \name Execution control
    /// The simplest way to execute the algorithm is to use one of the
    /// member functions called \c run().
    /// \n
    /// If you need more control on initial solution or execution then
    /// you have to call one \ref init() function and then the start()
    /// function.

    ///@{

    /// Initializes the internal data structures.

    /// Initializes the internal data structures. This function sets
    /// all flow values to the lower bound.
    /// \return This function returns false if the initialization
    /// process found a barrier.
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

    /// Initializes the internal data structures.

    /// Initializes the internal data structures. This functions uses
    /// greedy approach to construct the initial solution.
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

    ///Starts the algorithm

    ///This function starts the algorithm.
    ///\return This function returns true if it found a feasible circulation.
    ///
    ///\sa barrier()
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

    /// Runs the circulation algorithm.

    /// Runs the circulation algorithm.
    /// \note fc.run() is just a shortcut of the following code.
    /// \code
    ///   fc.greedyInit();
    ///   return fc.start();
    /// \endcode
    bool run() {
      greedyInit();
      return start();
    }

    /// @}

    /// \name Query Functions
    /// The result of the %Circulation algorithm can be obtained using
    /// these functions.
    /// \n
    /// Before the use of these functions,
    /// either run() or start() must be called.

    ///@{

    /**
       \brief Returns a barrier
       
       Barrier is a set \e B of nodes for which
       \f[ \sum_{v\in B}-delta(v)<
       \sum_{e\in\rho(B)}lo(e)-\sum_{e\in\delta(B)}up(e) \f]
       holds. The existence of a set with this property prooves that a feasible
       flow cannot exists.
       \sa checkBarrier()
       \sa run()
    */
    template<class GT>
    void barrierMap(GT &bar)
    {
      for(NodeIt n(_g);n!=INVALID;++n)
        bar.set(n, (*_level)[n] >= _el);
    }

    ///Returns true if the node is in the barrier

    ///Returns true if the node is in the barrier
    ///\sa barrierMap()
    bool barrier(const Node& node)
    {
      return (*_level)[node] >= _el;
    }

    /// \brief Returns the flow on the arc.
    ///
    /// Sets the \c flowMap to the flow on the arcs. This method can
    /// be called after the second phase of algorithm.
    Value flow(const Arc& arc) const {
      return (*_flow)[arc];
    }

    /// @}

    /// \name Checker Functions
    /// The feasibility  of the results can be checked using
    /// these functions.
    /// \n
    /// Before the use of these functions,
    /// either run() or start() must be called.

    ///@{

    ///Check if the  \c flow is a feasible circulation
    bool checkFlow() {
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

    ///Check whether or not the last execution provides a barrier
    ///\sa barrier()
    bool checkBarrier()
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
