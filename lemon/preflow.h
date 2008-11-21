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

#ifndef LEMON_PREFLOW_H
#define LEMON_PREFLOW_H

#include <lemon/tolerance.h>
#include <lemon/elevator.h>

/// \file
/// \ingroup max_flow
/// \brief Implementation of the preflow algorithm.

namespace lemon {

  /// \brief Default traits class of Preflow class.
  ///
  /// Default traits class of Preflow class.
  /// \param _Graph Digraph type.
  /// \param _CapacityMap Type of capacity map.
  template <typename _Graph, typename _CapacityMap>
  struct PreflowDefaultTraits {

    /// \brief The digraph type the algorithm runs on.
    typedef _Graph Digraph;

    /// \brief The type of the map that stores the arc capacities.
    ///
    /// The type of the map that stores the arc capacities.
    /// It must meet the \ref concepts::ReadMap "ReadMap" concept.
    typedef _CapacityMap CapacityMap;

    /// \brief The type of the length of the arcs.
    typedef typename CapacityMap::Value Value;

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

    /// \brief The eleavator type used by Preflow algorithm.
    ///
    /// The elevator type used by Preflow algorithm.
    ///
    /// \sa Elevator
    /// \sa LinkedElevator
    typedef LinkedElevator<Digraph, typename Digraph::Node> Elevator;

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


  /// \ingroup max_flow
  ///
  /// \brief %Preflow algorithms class.
  ///
  /// This class provides an implementation of the Goldberg's \e
  /// preflow \e algorithm producing a flow of maximum value in a
  /// digraph. The preflow algorithms are the fastest known max
  /// flow algorithms. The current implementation use a mixture of the
  /// \e "highest label" and the \e "bound decrease" heuristics.
  /// The worst case time complexity of the algorithm is \f$O(n^2\sqrt{e})\f$.
  ///
  /// The algorithm consists from two phases. After the first phase
  /// the maximal flow value and the minimum cut can be obtained. The
  /// second phase constructs the feasible maximum flow on each arc.
  ///
  /// \param _Graph The digraph type the algorithm runs on.
  /// \param _CapacityMap The flow map type.
  /// \param _Traits Traits class to set various data types used by
  /// the algorithm.  The default traits class is \ref
  /// PreflowDefaultTraits.  See \ref PreflowDefaultTraits for the
  /// documentation of a %Preflow traits class.
  ///
  ///\author Jacint Szabo and Balazs Dezso
#ifdef DOXYGEN
  template <typename _Graph, typename _CapacityMap, typename _Traits>
#else
  template <typename _Graph,
            typename _CapacityMap = typename _Graph::template ArcMap<int>,
            typename _Traits = PreflowDefaultTraits<_Graph, _CapacityMap> >
#endif
  class Preflow {
  public:

    typedef _Traits Traits;
    typedef typename Traits::Digraph Digraph;
    typedef typename Traits::CapacityMap CapacityMap;
    typedef typename Traits::Value Value;

    typedef typename Traits::FlowMap FlowMap;
    typedef typename Traits::Elevator Elevator;
    typedef typename Traits::Tolerance Tolerance;

  private:

    TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

    const Digraph& _graph;
    const CapacityMap* _capacity;

    int _node_num;

    Node _source, _target;

    FlowMap* _flow;
    bool _local_flow;

    Elevator* _level;
    bool _local_level;

    typedef typename Digraph::template NodeMap<Value> ExcessMap;
    ExcessMap* _excess;

    Tolerance _tolerance;

    bool _phase;


    void createStructures() {
      _node_num = countNodes(_graph);

      if (!_flow) {
        _flow = Traits::createFlowMap(_graph);
        _local_flow = true;
      }
      if (!_level) {
        _level = Traits::createElevator(_graph, _node_num);
        _local_level = true;
      }
      if (!_excess) {
        _excess = new ExcessMap(_graph);
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

    typedef Preflow Create;

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
      : public Preflow<Digraph, CapacityMap, SetFlowMapTraits<_FlowMap> > {
      typedef Preflow<Digraph, CapacityMap,
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
      : public Preflow<Digraph, CapacityMap, SetElevatorTraits<_Elevator> > {
      typedef Preflow<Digraph, CapacityMap,
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
      : public Preflow<Digraph, CapacityMap,
                       SetStandardElevatorTraits<_Elevator> > {
      typedef Preflow<Digraph, CapacityMap,
                      SetStandardElevatorTraits<_Elevator> > Create;
    };

    /// @}

  protected:

    Preflow() {}

  public:


    /// \brief The constructor of the class.
    ///
    /// The constructor of the class.
    /// \param digraph The digraph the algorithm runs on.
    /// \param capacity The capacity of the arcs.
    /// \param source The source node.
    /// \param target The target node.
    Preflow(const Digraph& digraph, const CapacityMap& capacity,
               Node source, Node target)
      : _graph(digraph), _capacity(&capacity),
        _node_num(0), _source(source), _target(target),
        _flow(0), _local_flow(false),
        _level(0), _local_level(false),
        _excess(0), _tolerance(), _phase() {}

    /// \brief Destrcutor.
    ///
    /// Destructor.
    ~Preflow() {
      destroyStructures();
    }

    /// \brief Sets the capacity map.
    ///
    /// Sets the capacity map.
    /// \return \c (*this)
    Preflow& capacityMap(const CapacityMap& map) {
      _capacity = &map;
      return *this;
    }

    /// \brief Sets the flow map.
    ///
    /// Sets the flow map.
    /// \return \c (*this)
    Preflow& flowMap(FlowMap& map) {
      if (_local_flow) {
        delete _flow;
        _local_flow = false;
      }
      _flow = &map;
      return *this;
    }

    /// \brief Returns the flow map.
    ///
    /// \return The flow map.
    const FlowMap& flowMap() {
      return *_flow;
    }

    /// \brief Sets the elevator.
    ///
    /// Sets the elevator.
    /// \return \c (*this)
    Preflow& elevator(Elevator& elevator) {
      if (_local_level) {
        delete _level;
        _local_level = false;
      }
      _level = &elevator;
      return *this;
    }

    /// \brief Returns the elevator.
    ///
    /// \return The elevator.
    const Elevator& elevator() {
      return *_level;
    }

    /// \brief Sets the source node.
    ///
    /// Sets the source node.
    /// \return \c (*this)
    Preflow& source(const Node& node) {
      _source = node;
      return *this;
    }

    /// \brief Sets the target node.
    ///
    /// Sets the target node.
    /// \return \c (*this)
    Preflow& target(const Node& node) {
      _target = node;
      return *this;
    }

    /// \brief Sets the tolerance used by algorithm.
    ///
    /// Sets the tolerance used by algorithm.
    Preflow& tolerance(const Tolerance& tolerance) const {
      _tolerance = tolerance;
      return *this;
    }

    /// \brief Returns the tolerance used by algorithm.
    ///
    /// Returns the tolerance used by algorithm.
    const Tolerance& tolerance() const {
      return tolerance;
    }

    /// \name Execution control The simplest way to execute the
    /// algorithm is to use one of the member functions called \c
    /// run().
    /// \n
    /// If you need more control on initial solution or
    /// execution then you have to call one \ref init() function and then
    /// the startFirstPhase() and if you need the startSecondPhase().

    ///@{

    /// \brief Initializes the internal data structures.
    ///
    /// Initializes the internal data structures.
    ///
    void init() {
      createStructures();

      _phase = true;
      for (NodeIt n(_graph); n != INVALID; ++n) {
        _excess->set(n, 0);
      }

      for (ArcIt e(_graph); e != INVALID; ++e) {
        _flow->set(e, 0);
      }

      typename Digraph::template NodeMap<bool> reached(_graph, false);

      _level->initStart();
      _level->initAddItem(_target);

      std::vector<Node> queue;
      reached.set(_source, true);

      queue.push_back(_target);
      reached.set(_target, true);
      while (!queue.empty()) {
        _level->initNewLevel();
        std::vector<Node> nqueue;
        for (int i = 0; i < int(queue.size()); ++i) {
          Node n = queue[i];
          for (InArcIt e(_graph, n); e != INVALID; ++e) {
            Node u = _graph.source(e);
            if (!reached[u] && _tolerance.positive((*_capacity)[e])) {
              reached.set(u, true);
              _level->initAddItem(u);
              nqueue.push_back(u);
            }
          }
        }
        queue.swap(nqueue);
      }
      _level->initFinish();

      for (OutArcIt e(_graph, _source); e != INVALID; ++e) {
        if (_tolerance.positive((*_capacity)[e])) {
          Node u = _graph.target(e);
          if ((*_level)[u] == _level->maxLevel()) continue;
          _flow->set(e, (*_capacity)[e]);
          _excess->set(u, (*_excess)[u] + (*_capacity)[e]);
          if (u != _target && !_level->active(u)) {
            _level->activate(u);
          }
        }
      }
    }

    /// \brief Initializes the internal data structures.
    ///
    /// Initializes the internal data structures and sets the initial
    /// flow to the given \c flowMap. The \c flowMap should contain a
    /// flow or at least a preflow, ie. in each node excluding the
    /// target the incoming flow should greater or equal to the
    /// outgoing flow.
    /// \return %False when the given \c flowMap is not a preflow.
    template <typename FlowMap>
    bool flowInit(const FlowMap& flowMap) {
      createStructures();

      for (ArcIt e(_graph); e != INVALID; ++e) {
        _flow->set(e, flowMap[e]);
      }

      for (NodeIt n(_graph); n != INVALID; ++n) {
        Value excess = 0;
        for (InArcIt e(_graph, n); e != INVALID; ++e) {
          excess += (*_flow)[e];
        }
        for (OutArcIt e(_graph, n); e != INVALID; ++e) {
          excess -= (*_flow)[e];
        }
        if (excess < 0 && n != _source) return false;
        _excess->set(n, excess);
      }

      typename Digraph::template NodeMap<bool> reached(_graph, false);

      _level->initStart();
      _level->initAddItem(_target);

      std::vector<Node> queue;
      reached.set(_source, true);

      queue.push_back(_target);
      reached.set(_target, true);
      while (!queue.empty()) {
        _level->initNewLevel();
        std::vector<Node> nqueue;
        for (int i = 0; i < int(queue.size()); ++i) {
          Node n = queue[i];
          for (InArcIt e(_graph, n); e != INVALID; ++e) {
            Node u = _graph.source(e);
            if (!reached[u] &&
                _tolerance.positive((*_capacity)[e] - (*_flow)[e])) {
              reached.set(u, true);
              _level->initAddItem(u);
              nqueue.push_back(u);
            }
          }
          for (OutArcIt e(_graph, n); e != INVALID; ++e) {
            Node v = _graph.target(e);
            if (!reached[v] && _tolerance.positive((*_flow)[e])) {
              reached.set(v, true);
              _level->initAddItem(v);
              nqueue.push_back(v);
            }
          }
        }
        queue.swap(nqueue);
      }
      _level->initFinish();

      for (OutArcIt e(_graph, _source); e != INVALID; ++e) {
        Value rem = (*_capacity)[e] - (*_flow)[e];
        if (_tolerance.positive(rem)) {
          Node u = _graph.target(e);
          if ((*_level)[u] == _level->maxLevel()) continue;
          _flow->set(e, (*_capacity)[e]);
          _excess->set(u, (*_excess)[u] + rem);
          if (u != _target && !_level->active(u)) {
            _level->activate(u);
          }
        }
      }
      for (InArcIt e(_graph, _source); e != INVALID; ++e) {
        Value rem = (*_flow)[e];
        if (_tolerance.positive(rem)) {
          Node v = _graph.source(e);
          if ((*_level)[v] == _level->maxLevel()) continue;
          _flow->set(e, 0);
          _excess->set(v, (*_excess)[v] + rem);
          if (v != _target && !_level->active(v)) {
            _level->activate(v);
          }
        }
      }
      return true;
    }

    /// \brief Starts the first phase of the preflow algorithm.
    ///
    /// The preflow algorithm consists of two phases, this method runs
    /// the first phase. After the first phase the maximum flow value
    /// and a minimum value cut can already be computed, although a
    /// maximum flow is not yet obtained. So after calling this method
    /// \ref flowValue() returns the value of a maximum flow and \ref
    /// minCut() returns a minimum cut.
    /// \pre One of the \ref init() functions should be called.
    void startFirstPhase() {
      _phase = true;

      Node n = _level->highestActive();
      int level = _level->highestActiveLevel();
      while (n != INVALID) {
        int num = _node_num;

        while (num > 0 && n != INVALID) {
          Value excess = (*_excess)[n];
          int new_level = _level->maxLevel();

          for (OutArcIt e(_graph, n); e != INVALID; ++e) {
            Value rem = (*_capacity)[e] - (*_flow)[e];
            if (!_tolerance.positive(rem)) continue;
            Node v = _graph.target(e);
            if ((*_level)[v] < level) {
              if (!_level->active(v) && v != _target) {
                _level->activate(v);
              }
              if (!_tolerance.less(rem, excess)) {
                _flow->set(e, (*_flow)[e] + excess);
                _excess->set(v, (*_excess)[v] + excess);
                excess = 0;
                goto no_more_push_1;
              } else {
                excess -= rem;
                _excess->set(v, (*_excess)[v] + rem);
                _flow->set(e, (*_capacity)[e]);
              }
            } else if (new_level > (*_level)[v]) {
              new_level = (*_level)[v];
            }
          }

          for (InArcIt e(_graph, n); e != INVALID; ++e) {
            Value rem = (*_flow)[e];
            if (!_tolerance.positive(rem)) continue;
            Node v = _graph.source(e);
            if ((*_level)[v] < level) {
              if (!_level->active(v) && v != _target) {
                _level->activate(v);
              }
              if (!_tolerance.less(rem, excess)) {
                _flow->set(e, (*_flow)[e] - excess);
                _excess->set(v, (*_excess)[v] + excess);
                excess = 0;
                goto no_more_push_1;
              } else {
                excess -= rem;
                _excess->set(v, (*_excess)[v] + rem);
                _flow->set(e, 0);
              }
            } else if (new_level > (*_level)[v]) {
              new_level = (*_level)[v];
            }
          }

        no_more_push_1:

          _excess->set(n, excess);

          if (excess != 0) {
            if (new_level + 1 < _level->maxLevel()) {
              _level->liftHighestActive(new_level + 1);
            } else {
              _level->liftHighestActiveToTop();
            }
            if (_level->emptyLevel(level)) {
              _level->liftToTop(level);
            }
          } else {
            _level->deactivate(n);
          }

          n = _level->highestActive();
          level = _level->highestActiveLevel();
          --num;
        }

        num = _node_num * 20;
        while (num > 0 && n != INVALID) {
          Value excess = (*_excess)[n];
          int new_level = _level->maxLevel();

          for (OutArcIt e(_graph, n); e != INVALID; ++e) {
            Value rem = (*_capacity)[e] - (*_flow)[e];
            if (!_tolerance.positive(rem)) continue;
            Node v = _graph.target(e);
            if ((*_level)[v] < level) {
              if (!_level->active(v) && v != _target) {
                _level->activate(v);
              }
              if (!_tolerance.less(rem, excess)) {
                _flow->set(e, (*_flow)[e] + excess);
                _excess->set(v, (*_excess)[v] + excess);
                excess = 0;
                goto no_more_push_2;
              } else {
                excess -= rem;
                _excess->set(v, (*_excess)[v] + rem);
                _flow->set(e, (*_capacity)[e]);
              }
            } else if (new_level > (*_level)[v]) {
              new_level = (*_level)[v];
            }
          }

          for (InArcIt e(_graph, n); e != INVALID; ++e) {
            Value rem = (*_flow)[e];
            if (!_tolerance.positive(rem)) continue;
            Node v = _graph.source(e);
            if ((*_level)[v] < level) {
              if (!_level->active(v) && v != _target) {
                _level->activate(v);
              }
              if (!_tolerance.less(rem, excess)) {
                _flow->set(e, (*_flow)[e] - excess);
                _excess->set(v, (*_excess)[v] + excess);
                excess = 0;
                goto no_more_push_2;
              } else {
                excess -= rem;
                _excess->set(v, (*_excess)[v] + rem);
                _flow->set(e, 0);
              }
            } else if (new_level > (*_level)[v]) {
              new_level = (*_level)[v];
            }
          }

        no_more_push_2:

          _excess->set(n, excess);

          if (excess != 0) {
            if (new_level + 1 < _level->maxLevel()) {
              _level->liftActiveOn(level, new_level + 1);
            } else {
              _level->liftActiveToTop(level);
            }
            if (_level->emptyLevel(level)) {
              _level->liftToTop(level);
            }
          } else {
            _level->deactivate(n);
          }

          while (level >= 0 && _level->activeFree(level)) {
            --level;
          }
          if (level == -1) {
            n = _level->highestActive();
            level = _level->highestActiveLevel();
          } else {
            n = _level->activeOn(level);
          }
          --num;
        }
      }
    }

    /// \brief Starts the second phase of the preflow algorithm.
    ///
    /// The preflow algorithm consists of two phases, this method runs
    /// the second phase. After calling \ref init() and \ref
    /// startFirstPhase() and then \ref startSecondPhase(), \ref
    /// flowMap() return a maximum flow, \ref flowValue() returns the
    /// value of a maximum flow, \ref minCut() returns a minimum cut
    /// \pre The \ref init() and startFirstPhase() functions should be
    /// called before.
    void startSecondPhase() {
      _phase = false;

      typename Digraph::template NodeMap<bool> reached(_graph);
      for (NodeIt n(_graph); n != INVALID; ++n) {
        reached.set(n, (*_level)[n] < _level->maxLevel());
      }

      _level->initStart();
      _level->initAddItem(_source);

      std::vector<Node> queue;
      queue.push_back(_source);
      reached.set(_source, true);

      while (!queue.empty()) {
        _level->initNewLevel();
        std::vector<Node> nqueue;
        for (int i = 0; i < int(queue.size()); ++i) {
          Node n = queue[i];
          for (OutArcIt e(_graph, n); e != INVALID; ++e) {
            Node v = _graph.target(e);
            if (!reached[v] && _tolerance.positive((*_flow)[e])) {
              reached.set(v, true);
              _level->initAddItem(v);
              nqueue.push_back(v);
            }
          }
          for (InArcIt e(_graph, n); e != INVALID; ++e) {
            Node u = _graph.source(e);
            if (!reached[u] &&
                _tolerance.positive((*_capacity)[e] - (*_flow)[e])) {
              reached.set(u, true);
              _level->initAddItem(u);
              nqueue.push_back(u);
            }
          }
        }
        queue.swap(nqueue);
      }
      _level->initFinish();

      for (NodeIt n(_graph); n != INVALID; ++n) {
        if (!reached[n]) {
          _level->dirtyTopButOne(n);
        } else if ((*_excess)[n] > 0 && _target != n) {
          _level->activate(n);
        }
      }

      Node n;
      while ((n = _level->highestActive()) != INVALID) {
        Value excess = (*_excess)[n];
        int level = _level->highestActiveLevel();
        int new_level = _level->maxLevel();

        for (OutArcIt e(_graph, n); e != INVALID; ++e) {
          Value rem = (*_capacity)[e] - (*_flow)[e];
          if (!_tolerance.positive(rem)) continue;
          Node v = _graph.target(e);
          if ((*_level)[v] < level) {
            if (!_level->active(v) && v != _source) {
              _level->activate(v);
            }
            if (!_tolerance.less(rem, excess)) {
              _flow->set(e, (*_flow)[e] + excess);
              _excess->set(v, (*_excess)[v] + excess);
              excess = 0;
              goto no_more_push;
            } else {
              excess -= rem;
              _excess->set(v, (*_excess)[v] + rem);
              _flow->set(e, (*_capacity)[e]);
            }
          } else if (new_level > (*_level)[v]) {
            new_level = (*_level)[v];
          }
        }

        for (InArcIt e(_graph, n); e != INVALID; ++e) {
          Value rem = (*_flow)[e];
          if (!_tolerance.positive(rem)) continue;
          Node v = _graph.source(e);
          if ((*_level)[v] < level) {
            if (!_level->active(v) && v != _source) {
              _level->activate(v);
            }
            if (!_tolerance.less(rem, excess)) {
              _flow->set(e, (*_flow)[e] - excess);
              _excess->set(v, (*_excess)[v] + excess);
              excess = 0;
              goto no_more_push;
            } else {
              excess -= rem;
              _excess->set(v, (*_excess)[v] + rem);
              _flow->set(e, 0);
            }
          } else if (new_level > (*_level)[v]) {
            new_level = (*_level)[v];
          }
        }

      no_more_push:

        _excess->set(n, excess);

        if (excess != 0) {
          if (new_level + 1 < _level->maxLevel()) {
            _level->liftHighestActive(new_level + 1);
          } else {
            // Calculation error
            _level->liftHighestActiveToTop();
          }
          if (_level->emptyLevel(level)) {
            // Calculation error
            _level->liftToTop(level);
          }
        } else {
          _level->deactivate(n);
        }

      }
    }

    /// \brief Runs the preflow algorithm.
    ///
    /// Runs the preflow algorithm.
    /// \note pf.run() is just a shortcut of the following code.
    /// \code
    ///   pf.init();
    ///   pf.startFirstPhase();
    ///   pf.startSecondPhase();
    /// \endcode
    void run() {
      init();
      startFirstPhase();
      startSecondPhase();
    }

    /// \brief Runs the preflow algorithm to compute the minimum cut.
    ///
    /// Runs the preflow algorithm to compute the minimum cut.
    /// \note pf.runMinCut() is just a shortcut of the following code.
    /// \code
    ///   pf.init();
    ///   pf.startFirstPhase();
    /// \endcode
    void runMinCut() {
      init();
      startFirstPhase();
    }

    /// @}

    /// \name Query Functions
    /// The result of the %Preflow algorithm can be obtained using these
    /// functions.\n
    /// Before the use of these functions,
    /// either run() or start() must be called.

    ///@{

    /// \brief Returns the value of the maximum flow.
    ///
    /// Returns the value of the maximum flow by returning the excess
    /// of the target node \c t. This value equals to the value of
    /// the maximum flow already after the first phase.
    Value flowValue() const {
      return (*_excess)[_target];
    }

    /// \brief Returns true when the node is on the source side of minimum cut.
    ///
    /// Returns true when the node is on the source side of minimum
    /// cut. This method can be called both after running \ref
    /// startFirstPhase() and \ref startSecondPhase().
    bool minCut(const Node& node) const {
      return ((*_level)[node] == _level->maxLevel()) == _phase;
    }

    /// \brief Returns a minimum value cut.
    ///
    /// Sets the \c cutMap to the characteristic vector of a minimum value
    /// cut. This method can be called both after running \ref
    /// startFirstPhase() and \ref startSecondPhase(). The result after second
    /// phase could be changed slightly if inexact computation is used.
    /// \pre The \c cutMap should be a bool-valued node-map.
    template <typename CutMap>
    void minCutMap(CutMap& cutMap) const {
      for (NodeIt n(_graph); n != INVALID; ++n) {
        cutMap.set(n, minCut(n));
      }
    }

    /// \brief Returns the flow on the arc.
    ///
    /// Sets the \c flowMap to the flow on the arcs. This method can
    /// be called after the second phase of algorithm.
    Value flow(const Arc& arc) const {
      return (*_flow)[arc];
    }

    /// @}
  };
}

#endif
