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

#ifndef LEMON_NETWORK_SIMPLEX_H
#define LEMON_NETWORK_SIMPLEX_H

/// \ingroup min_cost_flow
///
/// \file
/// \brief Network simplex algorithm for finding a minimum cost flow.

#include <vector>
#include <limits>
#include <algorithm>

#include <lemon/core.h>
#include <lemon/math.h>

namespace lemon {

  /// \addtogroup min_cost_flow
  /// @{

  /// \brief Implementation of the primal network simplex algorithm
  /// for finding a \ref min_cost_flow "minimum cost flow".
  ///
  /// \ref NetworkSimplex implements the primal network simplex algorithm
  /// for finding a \ref min_cost_flow "minimum cost flow".
  ///
  /// \tparam Digraph The digraph type the algorithm runs on.
  /// \tparam LowerMap The type of the lower bound map.
  /// \tparam CapacityMap The type of the capacity (upper bound) map.
  /// \tparam CostMap The type of the cost (length) map.
  /// \tparam SupplyMap The type of the supply map.
  ///
  /// \warning
  /// - Arc capacities and costs should be \e non-negative \e integers.
  /// - Supply values should be \e signed \e integers.
  /// - The value types of the maps should be convertible to each other.
  /// - \c CostMap::Value must be signed type.
  ///
  /// \note \ref NetworkSimplex provides five different pivot rule
  /// implementations that significantly affect the efficiency of the
  /// algorithm.
  /// By default "Block Search" pivot rule is used, which proved to be
  /// by far the most efficient according to our benchmark tests.
  /// However another pivot rule can be selected using \ref run()
  /// function with the proper parameter.
#ifdef DOXYGEN
  template < typename Digraph,
             typename LowerMap,
             typename CapacityMap,
             typename CostMap,
             typename SupplyMap >

#else
  template < typename Digraph,
             typename LowerMap = typename Digraph::template ArcMap<int>,
             typename CapacityMap = typename Digraph::template ArcMap<int>,
             typename CostMap = typename Digraph::template ArcMap<int>,
             typename SupplyMap = typename Digraph::template NodeMap<int> >
#endif
  class NetworkSimplex
  {
    TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

    typedef typename CapacityMap::Value Capacity;
    typedef typename CostMap::Value Cost;
    typedef typename SupplyMap::Value Supply;

    typedef std::vector<Arc> ArcVector;
    typedef std::vector<Node> NodeVector;
    typedef std::vector<int> IntVector;
    typedef std::vector<bool> BoolVector;
    typedef std::vector<Capacity> CapacityVector;
    typedef std::vector<Cost> CostVector;
    typedef std::vector<Supply> SupplyVector;

  public:

    /// The type of the flow map
    typedef typename Digraph::template ArcMap<Capacity> FlowMap;
    /// The type of the potential map
    typedef typename Digraph::template NodeMap<Cost> PotentialMap;

  public:

    /// Enum type for selecting the pivot rule used by \ref run()
    enum PivotRuleEnum {
      FIRST_ELIGIBLE_PIVOT,
      BEST_ELIGIBLE_PIVOT,
      BLOCK_SEARCH_PIVOT,
      CANDIDATE_LIST_PIVOT,
      ALTERING_LIST_PIVOT
    };

  private:

    // State constants for arcs
    enum ArcStateEnum {
      STATE_UPPER = -1,
      STATE_TREE  =  0,
      STATE_LOWER =  1
    };

  private:

    // References for the original data
    const Digraph &_graph;
    const LowerMap *_orig_lower;
    const CapacityMap &_orig_cap;
    const CostMap &_orig_cost;
    const SupplyMap *_orig_supply;
    Node _orig_source;
    Node _orig_target;
    Capacity _orig_flow_value;

    // Result maps
    FlowMap *_flow_map;
    PotentialMap *_potential_map;
    bool _local_flow;
    bool _local_potential;

    // The number of nodes and arcs in the original graph
    int _node_num;
    int _arc_num;

    // Data structures for storing the graph
    IntNodeMap _node_id;
    ArcVector _arc_ref;
    IntVector _source;
    IntVector _target;

    // Node and arc maps
    CapacityVector _cap;
    CostVector _cost;
    CostVector _supply;
    CapacityVector _flow;
    CostVector _pi;

    // Data for storing the spanning tree structure
    IntVector _depth;
    IntVector _parent;
    IntVector _pred;
    IntVector _thread;
    BoolVector _forward;
    IntVector _state;
    int _root;

    // Temporary data used in the current pivot iteration
    int in_arc, join, u_in, v_in, u_out, v_out;
    int first, second, right, last;
    int stem, par_stem, new_stem;
    Capacity delta;

  private:

    /// \brief Implementation of the "First Eligible" pivot rule for the
    /// \ref NetworkSimplex "network simplex" algorithm.
    ///
    /// This class implements the "First Eligible" pivot rule
    /// for the \ref NetworkSimplex "network simplex" algorithm.
    ///
    /// For more information see \ref NetworkSimplex::run().
    class FirstEligiblePivotRule
    {
    private:

      // References to the NetworkSimplex class
      const IntVector  &_source;
      const IntVector  &_target;
      const CostVector &_cost;
      const IntVector  &_state;
      const CostVector &_pi;
      int &_in_arc;
      int _arc_num;

      // Pivot rule data
      int _next_arc;

    public:

      /// Constructor
      FirstEligiblePivotRule(NetworkSimplex &ns) :
        _source(ns._source), _target(ns._target),
        _cost(ns._cost), _state(ns._state), _pi(ns._pi),
        _in_arc(ns.in_arc), _arc_num(ns._arc_num), _next_arc(0)
      {}

      /// Find next entering arc
      bool findEnteringArc() {
        Cost c;
        for (int e = _next_arc; e < _arc_num; ++e) {
          c = _state[e] * (_cost[e] + _pi[_source[e]] - _pi[_target[e]]);
          if (c < 0) {
            _in_arc = e;
            _next_arc = e + 1;
            return true;
          }
        }
        for (int e = 0; e < _next_arc; ++e) {
          c = _state[e] * (_cost[e] + _pi[_source[e]] - _pi[_target[e]]);
          if (c < 0) {
            _in_arc = e;
            _next_arc = e + 1;
            return true;
          }
        }
        return false;
      }

    }; //class FirstEligiblePivotRule


    /// \brief Implementation of the "Best Eligible" pivot rule for the
    /// \ref NetworkSimplex "network simplex" algorithm.
    ///
    /// This class implements the "Best Eligible" pivot rule
    /// for the \ref NetworkSimplex "network simplex" algorithm.
    ///
    /// For more information see \ref NetworkSimplex::run().
    class BestEligiblePivotRule
    {
    private:

      // References to the NetworkSimplex class
      const IntVector  &_source;
      const IntVector  &_target;
      const CostVector &_cost;
      const IntVector  &_state;
      const CostVector &_pi;
      int &_in_arc;
      int _arc_num;

    public:

      /// Constructor
      BestEligiblePivotRule(NetworkSimplex &ns) :
        _source(ns._source), _target(ns._target),
        _cost(ns._cost), _state(ns._state), _pi(ns._pi),
        _in_arc(ns.in_arc), _arc_num(ns._arc_num)
      {}

      /// Find next entering arc
      bool findEnteringArc() {
        Cost c, min = 0;
        for (int e = 0; e < _arc_num; ++e) {
          c = _state[e] * (_cost[e] + _pi[_source[e]] - _pi[_target[e]]);
          if (c < min) {
            min = c;
            _in_arc = e;
          }
        }
        return min < 0;
      }

    }; //class BestEligiblePivotRule


    /// \brief Implementation of the "Block Search" pivot rule for the
    /// \ref NetworkSimplex "network simplex" algorithm.
    ///
    /// This class implements the "Block Search" pivot rule
    /// for the \ref NetworkSimplex "network simplex" algorithm.
    ///
    /// For more information see \ref NetworkSimplex::run().
    class BlockSearchPivotRule
    {
    private:

      // References to the NetworkSimplex class
      const IntVector  &_source;
      const IntVector  &_target;
      const CostVector &_cost;
      const IntVector  &_state;
      const CostVector &_pi;
      int &_in_arc;
      int _arc_num;

      // Pivot rule data
      int _block_size;
      int _next_arc;

    public:

      /// Constructor
      BlockSearchPivotRule(NetworkSimplex &ns) :
        _source(ns._source), _target(ns._target),
        _cost(ns._cost), _state(ns._state), _pi(ns._pi),
        _in_arc(ns.in_arc), _arc_num(ns._arc_num), _next_arc(0)
      {
        // The main parameters of the pivot rule
        const double BLOCK_SIZE_FACTOR = 2.0;
        const int MIN_BLOCK_SIZE = 10;

        _block_size = std::max( int(BLOCK_SIZE_FACTOR * sqrt(_arc_num)),
                                MIN_BLOCK_SIZE );
      }

      /// Find next entering arc
      bool findEnteringArc() {
        Cost c, min = 0;
        int cnt = _block_size;
        int e, min_arc = _next_arc;
        for (e = _next_arc; e < _arc_num; ++e) {
          c = _state[e] * (_cost[e] + _pi[_source[e]] - _pi[_target[e]]);
          if (c < min) {
            min = c;
            min_arc = e;
          }
          if (--cnt == 0) {
            if (min < 0) break;
            cnt = _block_size;
          }
        }
        if (min == 0 || cnt > 0) {
          for (e = 0; e < _next_arc; ++e) {
            c = _state[e] * (_cost[e] + _pi[_source[e]] - _pi[_target[e]]);
            if (c < min) {
              min = c;
              min_arc = e;
            }
            if (--cnt == 0) {
              if (min < 0) break;
              cnt = _block_size;
            }
          }
        }
        if (min >= 0) return false;
        _in_arc = min_arc;
        _next_arc = e;
        return true;
      }

    }; //class BlockSearchPivotRule


    /// \brief Implementation of the "Candidate List" pivot rule for the
    /// \ref NetworkSimplex "network simplex" algorithm.
    ///
    /// This class implements the "Candidate List" pivot rule
    /// for the \ref NetworkSimplex "network simplex" algorithm.
    ///
    /// For more information see \ref NetworkSimplex::run().
    class CandidateListPivotRule
    {
    private:

      // References to the NetworkSimplex class
      const IntVector  &_source;
      const IntVector  &_target;
      const CostVector &_cost;
      const IntVector  &_state;
      const CostVector &_pi;
      int &_in_arc;
      int _arc_num;

      // Pivot rule data
      IntVector _candidates;
      int _list_length, _minor_limit;
      int _curr_length, _minor_count;
      int _next_arc;

    public:

      /// Constructor
      CandidateListPivotRule(NetworkSimplex &ns) :
        _source(ns._source), _target(ns._target),
        _cost(ns._cost), _state(ns._state), _pi(ns._pi),
        _in_arc(ns.in_arc), _arc_num(ns._arc_num), _next_arc(0)
      {
        // The main parameters of the pivot rule
        const double LIST_LENGTH_FACTOR = 1.0;
        const int MIN_LIST_LENGTH = 10;
        const double MINOR_LIMIT_FACTOR = 0.1;
        const int MIN_MINOR_LIMIT = 3;

        _list_length = std::max( int(LIST_LENGTH_FACTOR * sqrt(_arc_num)),
                                 MIN_LIST_LENGTH );
        _minor_limit = std::max( int(MINOR_LIMIT_FACTOR * _list_length),
                                 MIN_MINOR_LIMIT );
        _curr_length = _minor_count = 0;
        _candidates.resize(_list_length);
      }

      /// Find next entering arc
      bool findEnteringArc() {
        Cost min, c;
        int e, min_arc = _next_arc;
        if (_curr_length > 0 && _minor_count < _minor_limit) {
          // Minor iteration: select the best eligible arc from the
          // current candidate list
          ++_minor_count;
          min = 0;
          for (int i = 0; i < _curr_length; ++i) {
            e = _candidates[i];
            c = _state[e] * (_cost[e] + _pi[_source[e]] - _pi[_target[e]]);
            if (c < min) {
              min = c;
              min_arc = e;
            }
            if (c >= 0) {
              _candidates[i--] = _candidates[--_curr_length];
            }
          }
          if (min < 0) {
            _in_arc = min_arc;
            return true;
          }
        }

        // Major iteration: build a new candidate list
        min = 0;
        _curr_length = 0;
        for (e = _next_arc; e < _arc_num; ++e) {
          c = _state[e] * (_cost[e] + _pi[_source[e]] - _pi[_target[e]]);
          if (c < 0) {
            _candidates[_curr_length++] = e;
            if (c < min) {
              min = c;
              min_arc = e;
            }
            if (_curr_length == _list_length) break;
          }
        }
        if (_curr_length < _list_length) {
          for (e = 0; e < _next_arc; ++e) {
            c = _state[e] * (_cost[e] + _pi[_source[e]] - _pi[_target[e]]);
            if (c < 0) {
              _candidates[_curr_length++] = e;
              if (c < min) {
                min = c;
                min_arc = e;
              }
              if (_curr_length == _list_length) break;
            }
          }
        }
        if (_curr_length == 0) return false;
        _minor_count = 1;
        _in_arc = min_arc;
        _next_arc = e;
        return true;
      }

    }; //class CandidateListPivotRule


    /// \brief Implementation of the "Altering Candidate List" pivot rule
    /// for the \ref NetworkSimplex "network simplex" algorithm.
    ///
    /// This class implements the "Altering Candidate List" pivot rule
    /// for the \ref NetworkSimplex "network simplex" algorithm.
    ///
    /// For more information see \ref NetworkSimplex::run().
    class AlteringListPivotRule
    {
    private:

      // References to the NetworkSimplex class
      const IntVector  &_source;
      const IntVector  &_target;
      const CostVector &_cost;
      const IntVector  &_state;
      const CostVector &_pi;
      int &_in_arc;
      int _arc_num;

      // Pivot rule data
      int _block_size, _head_length, _curr_length;
      int _next_arc;
      IntVector _candidates;
      CostVector _cand_cost;

      // Functor class to compare arcs during sort of the candidate list
      class SortFunc
      {
      private:
        const CostVector &_map;
      public:
        SortFunc(const CostVector &map) : _map(map) {}
        bool operator()(int left, int right) {
          return _map[left] > _map[right];
        }
      };

      SortFunc _sort_func;

    public:

      /// Constructor
      AlteringListPivotRule(NetworkSimplex &ns) :
        _source(ns._source), _target(ns._target),
        _cost(ns._cost), _state(ns._state), _pi(ns._pi),
        _in_arc(ns.in_arc), _arc_num(ns._arc_num),
        _next_arc(0), _cand_cost(ns._arc_num), _sort_func(_cand_cost)
      {
        // The main parameters of the pivot rule
        const double BLOCK_SIZE_FACTOR = 1.5;
        const int MIN_BLOCK_SIZE = 10;
        const double HEAD_LENGTH_FACTOR = 0.1;
        const int MIN_HEAD_LENGTH = 3;

        _block_size = std::max( int(BLOCK_SIZE_FACTOR * sqrt(_arc_num)),
                                MIN_BLOCK_SIZE );
        _head_length = std::max( int(HEAD_LENGTH_FACTOR * _block_size),
                                 MIN_HEAD_LENGTH );
        _candidates.resize(_head_length + _block_size);
        _curr_length = 0;
      }

      /// Find next entering arc
      bool findEnteringArc() {
        // Check the current candidate list
        int e;
        for (int i = 0; i < _curr_length; ++i) {
          e = _candidates[i];
          _cand_cost[e] = _state[e] *
            (_cost[e] + _pi[_source[e]] - _pi[_target[e]]);
          if (_cand_cost[e] >= 0) {
            _candidates[i--] = _candidates[--_curr_length];
          }
        }

        // Extend the list
        int cnt = _block_size;
        int last_arc = 0;
        int limit = _head_length;

        for (int e = _next_arc; e < _arc_num; ++e) {
          _cand_cost[e] = _state[e] *
            (_cost[e] + _pi[_source[e]] - _pi[_target[e]]);
          if (_cand_cost[e] < 0) {
            _candidates[_curr_length++] = e;
            last_arc = e;
          }
          if (--cnt == 0) {
            if (_curr_length > limit) break;
            limit = 0;
            cnt = _block_size;
          }
        }
        if (_curr_length <= limit) {
          for (int e = 0; e < _next_arc; ++e) {
            _cand_cost[e] = _state[e] *
              (_cost[e] + _pi[_source[e]] - _pi[_target[e]]);
            if (_cand_cost[e] < 0) {
              _candidates[_curr_length++] = e;
              last_arc = e;
            }
            if (--cnt == 0) {
              if (_curr_length > limit) break;
              limit = 0;
              cnt = _block_size;
            }
          }
        }
        if (_curr_length == 0) return false;
        _next_arc = last_arc + 1;

        // Make heap of the candidate list (approximating a partial sort)
        make_heap( _candidates.begin(), _candidates.begin() + _curr_length,
                   _sort_func );

        // Pop the first element of the heap
        _in_arc = _candidates[0];
        pop_heap( _candidates.begin(), _candidates.begin() + _curr_length,
                  _sort_func );
        _curr_length = std::min(_head_length, _curr_length - 1);
        return true;
      }

    }; //class AlteringListPivotRule

  public:

    /// \brief General constructor (with lower bounds).
    ///
    /// General constructor (with lower bounds).
    ///
    /// \param graph The digraph the algorithm runs on.
    /// \param lower The lower bounds of the arcs.
    /// \param capacity The capacities (upper bounds) of the arcs.
    /// \param cost The cost (length) values of the arcs.
    /// \param supply The supply values of the nodes (signed).
    NetworkSimplex( const Digraph &graph,
                    const LowerMap &lower,
                    const CapacityMap &capacity,
                    const CostMap &cost,
                    const SupplyMap &supply ) :
      _graph(graph), _orig_lower(&lower), _orig_cap(capacity),
      _orig_cost(cost), _orig_supply(&supply),
      _flow_map(NULL), _potential_map(NULL),
      _local_flow(false), _local_potential(false),
      _node_id(graph)
    {}

    /// \brief General constructor (without lower bounds).
    ///
    /// General constructor (without lower bounds).
    ///
    /// \param graph The digraph the algorithm runs on.
    /// \param capacity The capacities (upper bounds) of the arcs.
    /// \param cost The cost (length) values of the arcs.
    /// \param supply The supply values of the nodes (signed).
    NetworkSimplex( const Digraph &graph,
                    const CapacityMap &capacity,
                    const CostMap &cost,
                    const SupplyMap &supply ) :
      _graph(graph), _orig_lower(NULL), _orig_cap(capacity),
      _orig_cost(cost), _orig_supply(&supply),
      _flow_map(NULL), _potential_map(NULL),
      _local_flow(false), _local_potential(false),
      _node_id(graph)
    {}

    /// \brief Simple constructor (with lower bounds).
    ///
    /// Simple constructor (with lower bounds).
    ///
    /// \param graph The digraph the algorithm runs on.
    /// \param lower The lower bounds of the arcs.
    /// \param capacity The capacities (upper bounds) of the arcs.
    /// \param cost The cost (length) values of the arcs.
    /// \param s The source node.
    /// \param t The target node.
    /// \param flow_value The required amount of flow from node \c s
    /// to node \c t (i.e. the supply of \c s and the demand of \c t).
    NetworkSimplex( const Digraph &graph,
                    const LowerMap &lower,
                    const CapacityMap &capacity,
                    const CostMap &cost,
                    Node s, Node t,
                    Capacity flow_value ) :
      _graph(graph), _orig_lower(&lower), _orig_cap(capacity),
      _orig_cost(cost), _orig_supply(NULL),
      _orig_source(s), _orig_target(t), _orig_flow_value(flow_value),
      _flow_map(NULL), _potential_map(NULL),
      _local_flow(false), _local_potential(false),
      _node_id(graph)
    {}

    /// \brief Simple constructor (without lower bounds).
    ///
    /// Simple constructor (without lower bounds).
    ///
    /// \param graph The digraph the algorithm runs on.
    /// \param capacity The capacities (upper bounds) of the arcs.
    /// \param cost The cost (length) values of the arcs.
    /// \param s The source node.
    /// \param t The target node.
    /// \param flow_value The required amount of flow from node \c s
    /// to node \c t (i.e. the supply of \c s and the demand of \c t).
    NetworkSimplex( const Digraph &graph,
                    const CapacityMap &capacity,
                    const CostMap &cost,
                    Node s, Node t,
                    Capacity flow_value ) :
      _graph(graph), _orig_lower(NULL), _orig_cap(capacity),
      _orig_cost(cost), _orig_supply(NULL),
      _orig_source(s), _orig_target(t), _orig_flow_value(flow_value),
      _flow_map(NULL), _potential_map(NULL),
      _local_flow(false), _local_potential(false),
      _node_id(graph)
    {}

    /// Destructor.
    ~NetworkSimplex() {
      if (_local_flow) delete _flow_map;
      if (_local_potential) delete _potential_map;
    }

    /// \brief Set the flow map.
    ///
    /// This function sets the flow map.
    ///
    /// \return <tt>(*this)</tt>
    NetworkSimplex& flowMap(FlowMap &map) {
      if (_local_flow) {
        delete _flow_map;
        _local_flow = false;
      }
      _flow_map = &map;
      return *this;
    }

    /// \brief Set the potential map.
    ///
    /// This function sets the potential map.
    ///
    /// \return <tt>(*this)</tt>
    NetworkSimplex& potentialMap(PotentialMap &map) {
      if (_local_potential) {
        delete _potential_map;
        _local_potential = false;
      }
      _potential_map = &map;
      return *this;
    }

    /// \name Execution control
    /// The algorithm can be executed using the
    /// \ref lemon::NetworkSimplex::run() "run()" function.
    /// @{

    /// \brief Run the algorithm.
    ///
    /// This function runs the algorithm.
    ///
    /// \param pivot_rule The pivot rule that is used during the
    /// algorithm.
    ///
    /// The available pivot rules:
    ///
    /// - FIRST_ELIGIBLE_PIVOT The next eligible arc is selected in
    /// a wraparound fashion in every iteration
    /// (\ref FirstEligiblePivotRule).
    ///
    /// - BEST_ELIGIBLE_PIVOT The best eligible arc is selected in
    /// every iteration (\ref BestEligiblePivotRule).
    ///
    /// - BLOCK_SEARCH_PIVOT A specified number of arcs are examined in
    /// every iteration in a wraparound fashion and the best eligible
    /// arc is selected from this block (\ref BlockSearchPivotRule).
    ///
    /// - CANDIDATE_LIST_PIVOT In a major iteration a candidate list is
    /// built from eligible arcs in a wraparound fashion and in the
    /// following minor iterations the best eligible arc is selected
    /// from this list (\ref CandidateListPivotRule).
    ///
    /// - ALTERING_LIST_PIVOT It is a modified version of the
    /// "Candidate List" pivot rule. It keeps only the several best
    /// eligible arcs from the former candidate list and extends this
    /// list in every iteration (\ref AlteringListPivotRule).
    ///
    /// According to our comprehensive benchmark tests the "Block Search"
    /// pivot rule proved to be the fastest and the most robust on
    /// various test inputs. Thus it is the default option.
    ///
    /// \return \c true if a feasible flow can be found.
    bool run(PivotRuleEnum pivot_rule = BLOCK_SEARCH_PIVOT) {
      return init() && start(pivot_rule);
    }

    /// @}

    /// \name Query Functions
    /// The results of the algorithm can be obtained using these
    /// functions.\n
    /// \ref lemon::NetworkSimplex::run() "run()" must be called before
    /// using them.
    /// @{

    /// \brief Return a const reference to the flow map.
    ///
    /// This function returns a const reference to an arc map storing
    /// the found flow.
    ///
    /// \pre \ref run() must be called before using this function.
    const FlowMap& flowMap() const {
      return *_flow_map;
    }

    /// \brief Return a const reference to the potential map
    /// (the dual solution).
    ///
    /// This function returns a const reference to a node map storing
    /// the found potentials (the dual solution).
    ///
    /// \pre \ref run() must be called before using this function.
    const PotentialMap& potentialMap() const {
      return *_potential_map;
    }

    /// \brief Return the flow on the given arc.
    ///
    /// This function returns the flow on the given arc.
    ///
    /// \pre \ref run() must be called before using this function.
    Capacity flow(const Arc& arc) const {
      return (*_flow_map)[arc];
    }

    /// \brief Return the potential of the given node.
    ///
    /// This function returns the potential of the given node.
    ///
    /// \pre \ref run() must be called before using this function.
    Cost potential(const Node& node) const {
      return (*_potential_map)[node];
    }

    /// \brief Return the total cost of the found flow.
    ///
    /// This function returns the total cost of the found flow.
    /// The complexity of the function is \f$ O(e) \f$.
    ///
    /// \pre \ref run() must be called before using this function.
    Cost totalCost() const {
      Cost c = 0;
      for (ArcIt e(_graph); e != INVALID; ++e)
        c += (*_flow_map)[e] * _orig_cost[e];
      return c;
    }

    /// @}

  private:

    // Initialize internal data structures
    bool init() {
      // Initialize result maps
      if (!_flow_map) {
        _flow_map = new FlowMap(_graph);
        _local_flow = true;
      }
      if (!_potential_map) {
        _potential_map = new PotentialMap(_graph);
        _local_potential = true;
      }

      // Initialize vectors
      _node_num = countNodes(_graph);
      _arc_num = countArcs(_graph);
      int all_node_num = _node_num + 1;
      int all_arc_num = _arc_num + _node_num;

      _arc_ref.resize(_arc_num);
      _source.resize(all_arc_num);
      _target.resize(all_arc_num);

      _cap.resize(all_arc_num);
      _cost.resize(all_arc_num);
      _supply.resize(all_node_num);
      _flow.resize(all_arc_num, 0);
      _pi.resize(all_node_num, 0);

      _depth.resize(all_node_num);
      _parent.resize(all_node_num);
      _pred.resize(all_node_num);
      _forward.resize(all_node_num);
      _thread.resize(all_node_num);
      _state.resize(all_arc_num, STATE_LOWER);

      // Initialize node related data
      bool valid_supply = true;
      if (_orig_supply) {
        Supply sum = 0;
        int i = 0;
        for (NodeIt n(_graph); n != INVALID; ++n, ++i) {
          _node_id[n] = i;
          _supply[i] = (*_orig_supply)[n];
          sum += _supply[i];
        }
        valid_supply = (sum == 0);
      } else {
        int i = 0;
        for (NodeIt n(_graph); n != INVALID; ++n, ++i) {
          _node_id[n] = i;
          _supply[i] = 0;
        }
        _supply[_node_id[_orig_source]] =  _orig_flow_value;
        _supply[_node_id[_orig_target]] = -_orig_flow_value;
      }
      if (!valid_supply) return false;

      // Set data for the artificial root node
      _root = _node_num;
      _depth[_root] = 0;
      _parent[_root] = -1;
      _pred[_root] = -1;
      _thread[_root] = 0;
      _supply[_root] = 0;
      _pi[_root] = 0;

      // Store the arcs in a mixed order
      int k = std::max(int(sqrt(_arc_num)), 10);
      int i = 0;
      for (ArcIt e(_graph); e != INVALID; ++e) {
        _arc_ref[i] = e;
        if ((i += k) >= _arc_num) i = (i % k) + 1;
      }

      // Initialize arc maps
      for (int i = 0; i != _arc_num; ++i) {
        Arc e = _arc_ref[i];
        _source[i] = _node_id[_graph.source(e)];
        _target[i] = _node_id[_graph.target(e)];
        _cost[i] = _orig_cost[e];
        _cap[i] = _orig_cap[e];
      }

      // Remove non-zero lower bounds
      if (_orig_lower) {
        for (int i = 0; i != _arc_num; ++i) {
          Capacity c = (*_orig_lower)[_arc_ref[i]];
          if (c != 0) {
            _cap[i] -= c;
            _supply[_source[i]] -= c;
            _supply[_target[i]] += c;
          }
        }
      }

      // Add artificial arcs and initialize the spanning tree data structure
      Cost max_cost = std::numeric_limits<Cost>::max() / 4;
      Capacity max_cap = std::numeric_limits<Capacity>::max();
      for (int u = 0, e = _arc_num; u != _node_num; ++u, ++e) {
        _thread[u] = u + 1;
        _depth[u] = 1;
        _parent[u] = _root;
        _pred[u] = e;
        if (_supply[u] >= 0) {
          _flow[e] = _supply[u];
          _forward[u] = true;
          _pi[u] = -max_cost;
        } else {
          _flow[e] = -_supply[u];
          _forward[u] = false;
          _pi[u] = max_cost;
        }
        _cost[e] = max_cost;
        _cap[e] = max_cap;
        _state[e] = STATE_TREE;
      }

      return true;
    }

    // Find the join node
    void findJoinNode() {
      int u = _source[in_arc];
      int v = _target[in_arc];
      while (_depth[u] > _depth[v]) u = _parent[u];
      while (_depth[v] > _depth[u]) v = _parent[v];
      while (u != v) {
        u = _parent[u];
        v = _parent[v];
      }
      join = u;
    }

    // Find the leaving arc of the cycle and returns true if the
    // leaving arc is not the same as the entering arc
    bool findLeavingArc() {
      // Initialize first and second nodes according to the direction
      // of the cycle
      if (_state[in_arc] == STATE_LOWER) {
        first  = _source[in_arc];
        second = _target[in_arc];
      } else {
        first  = _target[in_arc];
        second = _source[in_arc];
      }
      delta = _cap[in_arc];
      int result = 0;
      Capacity d;
      int e;

      // Search the cycle along the path form the first node to the root
      for (int u = first; u != join; u = _parent[u]) {
        e = _pred[u];
        d = _forward[u] ? _flow[e] : _cap[e] - _flow[e];
        if (d < delta) {
          delta = d;
          u_out = u;
          result = 1;
        }
      }
      // Search the cycle along the path form the second node to the root
      for (int u = second; u != join; u = _parent[u]) {
        e = _pred[u];
        d = _forward[u] ? _cap[e] - _flow[e] : _flow[e];
        if (d <= delta) {
          delta = d;
          u_out = u;
          result = 2;
        }
      }

      if (result == 1) {
        u_in = first;
        v_in = second;
      } else {
        u_in = second;
        v_in = first;
      }
      return result != 0;
    }

    // Change _flow and _state vectors
    void changeFlow(bool change) {
      // Augment along the cycle
      if (delta > 0) {
        Capacity val = _state[in_arc] * delta;
        _flow[in_arc] += val;
        for (int u = _source[in_arc]; u != join; u = _parent[u]) {
          _flow[_pred[u]] += _forward[u] ? -val : val;
        }
        for (int u = _target[in_arc]; u != join; u = _parent[u]) {
          _flow[_pred[u]] += _forward[u] ? val : -val;
        }
      }
      // Update the state of the entering and leaving arcs
      if (change) {
        _state[in_arc] = STATE_TREE;
        _state[_pred[u_out]] =
          (_flow[_pred[u_out]] == 0) ? STATE_LOWER : STATE_UPPER;
      } else {
        _state[in_arc] = -_state[in_arc];
      }
    }

    // Update _thread and _parent vectors
    void updateThreadParent() {
      int u;
      v_out = _parent[u_out];

      // Handle the case when join and v_out coincide
      bool par_first = false;
      if (join == v_out) {
        for (u = join; u != u_in && u != v_in; u = _thread[u]) ;
        if (u == v_in) {
          par_first = true;
          while (_thread[u] != u_out) u = _thread[u];
          first = u;
        }
      }

      // Find the last successor of u_in (u) and the node after it (right)
      // according to the thread index
      for (u = u_in; _depth[_thread[u]] > _depth[u_in]; u = _thread[u]) ;
      right = _thread[u];
      if (_thread[v_in] == u_out) {
        for (last = u; _depth[last] > _depth[u_out]; last = _thread[last]) ;
        if (last == u_out) last = _thread[last];
      }
      else last = _thread[v_in];

      // Update stem nodes
      _thread[v_in] = stem = u_in;
      par_stem = v_in;
      while (stem != u_out) {
        _thread[u] = new_stem = _parent[stem];

        // Find the node just before the stem node (u) according to
        // the original thread index
        for (u = new_stem; _thread[u] != stem; u = _thread[u]) ;
        _thread[u] = right;

        // Change the parent node of stem and shift stem and par_stem nodes
        _parent[stem] = par_stem;
        par_stem = stem;
        stem = new_stem;

        // Find the last successor of stem (u) and the node after it (right)
        // according to the thread index
        for (u = stem; _depth[_thread[u]] > _depth[stem]; u = _thread[u]) ;
        right = _thread[u];
      }
      _parent[u_out] = par_stem;
      _thread[u] = last;

      if (join == v_out && par_first) {
        if (first != v_in) _thread[first] = right;
      } else {
        for (u = v_out; _thread[u] != u_out; u = _thread[u]) ;
        _thread[u] = right;
      }
    }

    // Update _pred and _forward vectors
    void updatePredArc() {
      int u = u_out, v;
      while (u != u_in) {
        v = _parent[u];
        _pred[u] = _pred[v];
        _forward[u] = !_forward[v];
        u = v;
      }
      _pred[u_in] = in_arc;
      _forward[u_in] = (u_in == _source[in_arc]);
    }

    // Update _depth and _potential vectors
    void updateDepthPotential() {
      _depth[u_in] = _depth[v_in] + 1;
      Cost sigma = _forward[u_in] ?
        _pi[v_in] - _pi[u_in] - _cost[_pred[u_in]] :
        _pi[v_in] - _pi[u_in] + _cost[_pred[u_in]];
      _pi[u_in] += sigma;
      for(int u = _thread[u_in]; _parent[u] != -1; u = _thread[u]) {
        _depth[u] = _depth[_parent[u]] + 1;
        if (_depth[u] <= _depth[u_in]) break;
        _pi[u] += sigma;
      }
    }

    // Execute the algorithm
    bool start(PivotRuleEnum pivot_rule) {
      // Select the pivot rule implementation
      switch (pivot_rule) {
        case FIRST_ELIGIBLE_PIVOT:
          return start<FirstEligiblePivotRule>();
        case BEST_ELIGIBLE_PIVOT:
          return start<BestEligiblePivotRule>();
        case BLOCK_SEARCH_PIVOT:
          return start<BlockSearchPivotRule>();
        case CANDIDATE_LIST_PIVOT:
          return start<CandidateListPivotRule>();
        case ALTERING_LIST_PIVOT:
          return start<AlteringListPivotRule>();
      }
      return false;
    }

    template<class PivotRuleImplementation>
    bool start() {
      PivotRuleImplementation pivot(*this);

      // Execute the network simplex algorithm
      while (pivot.findEnteringArc()) {
        findJoinNode();
        bool change = findLeavingArc();
        changeFlow(change);
        if (change) {
          updateThreadParent();
          updatePredArc();
          updateDepthPotential();
        }
      }

      // Check if the flow amount equals zero on all the artificial arcs
      for (int e = _arc_num; e != _arc_num + _node_num; ++e) {
        if (_flow[e] > 0) return false;
      }

      // Copy flow values to _flow_map
      if (_orig_lower) {
        for (int i = 0; i != _arc_num; ++i) {
          Arc e = _arc_ref[i];
          _flow_map->set(e, (*_orig_lower)[e] + _flow[i]);
        }
      } else {
        for (int i = 0; i != _arc_num; ++i) {
          _flow_map->set(_arc_ref[i], _flow[i]);
        }
      }
      // Copy potential values to _potential_map
      for (NodeIt n(_graph); n != INVALID; ++n) {
        _potential_map->set(n, _pi[_node_id[n]]);
      }

      return true;
    }

  }; //class NetworkSimplex

  ///@}

} //namespace lemon

#endif //LEMON_NETWORK_SIMPLEX_H
