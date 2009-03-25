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
/// \brief Network Simplex algorithm for finding a minimum cost flow.

#include <vector>
#include <limits>
#include <algorithm>

#include <lemon/core.h>
#include <lemon/math.h>

namespace lemon {

  /// \addtogroup min_cost_flow
  /// @{

  /// \brief Implementation of the primal Network Simplex algorithm
  /// for finding a \ref min_cost_flow "minimum cost flow".
  ///
  /// \ref NetworkSimplex implements the primal Network Simplex algorithm
  /// for finding a \ref min_cost_flow "minimum cost flow".
  ///
  /// \tparam GR The digraph type the algorithm runs on.
  /// \tparam V The value type used in the algorithm.
  /// By default it is \c int.
  ///
  /// \warning \c V must be a signed integer type.
  ///
  /// \note %NetworkSimplex provides five different pivot rule
  /// implementations. For more information see \ref PivotRule.
  template <typename GR, typename V = int>
  class NetworkSimplex
  {
  public:

    /// The value type of the algorithm
    typedef V Value;
    /// The type of the flow map
    typedef typename GR::template ArcMap<Value> FlowMap;
    /// The type of the potential map
    typedef typename GR::template NodeMap<Value> PotentialMap;

  public:

    /// \brief Enum type for selecting the pivot rule.
    ///
    /// Enum type for selecting the pivot rule for the \ref run()
    /// function.
    ///
    /// \ref NetworkSimplex provides five different pivot rule
    /// implementations that significantly affect the running time
    /// of the algorithm.
    /// By default \ref BLOCK_SEARCH "Block Search" is used, which
    /// proved to be the most efficient and the most robust on various
    /// test inputs according to our benchmark tests.
    /// However another pivot rule can be selected using the \ref run()
    /// function with the proper parameter.
    enum PivotRule {

      /// The First Eligible pivot rule.
      /// The next eligible arc is selected in a wraparound fashion
      /// in every iteration.
      FIRST_ELIGIBLE,

      /// The Best Eligible pivot rule.
      /// The best eligible arc is selected in every iteration.
      BEST_ELIGIBLE,

      /// The Block Search pivot rule.
      /// A specified number of arcs are examined in every iteration
      /// in a wraparound fashion and the best eligible arc is selected
      /// from this block.
      BLOCK_SEARCH,

      /// The Candidate List pivot rule.
      /// In a major iteration a candidate list is built from eligible arcs
      /// in a wraparound fashion and in the following minor iterations
      /// the best eligible arc is selected from this list.
      CANDIDATE_LIST,

      /// The Altering Candidate List pivot rule.
      /// It is a modified version of the Candidate List method.
      /// It keeps only the several best eligible arcs from the former
      /// candidate list and extends this list in every iteration.
      ALTERING_LIST
    };

  private:

    TEMPLATE_DIGRAPH_TYPEDEFS(GR);

    typedef typename GR::template ArcMap<Value> ValueArcMap;
    typedef typename GR::template NodeMap<Value> ValueNodeMap;

    typedef std::vector<Arc> ArcVector;
    typedef std::vector<Node> NodeVector;
    typedef std::vector<int> IntVector;
    typedef std::vector<bool> BoolVector;
    typedef std::vector<Value> ValueVector;

    // State constants for arcs
    enum ArcStateEnum {
      STATE_UPPER = -1,
      STATE_TREE  =  0,
      STATE_LOWER =  1
    };

  private:

    // Data related to the underlying digraph
    const GR &_graph;
    int _node_num;
    int _arc_num;

    // Parameters of the problem
    ValueArcMap *_plower;
    ValueArcMap *_pupper;
    ValueArcMap *_pcost;
    ValueNodeMap *_psupply;
    bool _pstsup;
    Node _psource, _ptarget;
    Value _pstflow;

    // Result maps
    FlowMap *_flow_map;
    PotentialMap *_potential_map;
    bool _local_flow;
    bool _local_potential;

    // Data structures for storing the digraph
    IntNodeMap _node_id;
    ArcVector _arc_ref;
    IntVector _source;
    IntVector _target;

    // Node and arc data
    ValueVector _cap;
    ValueVector _cost;
    ValueVector _supply;
    ValueVector _flow;
    ValueVector _pi;

    // Data for storing the spanning tree structure
    IntVector _parent;
    IntVector _pred;
    IntVector _thread;
    IntVector _rev_thread;
    IntVector _succ_num;
    IntVector _last_succ;
    IntVector _dirty_revs;
    BoolVector _forward;
    IntVector _state;
    int _root;

    // Temporary data used in the current pivot iteration
    int in_arc, join, u_in, v_in, u_out, v_out;
    int first, second, right, last;
    int stem, par_stem, new_stem;
    Value delta;

  private:

    // Implementation of the First Eligible pivot rule
    class FirstEligiblePivotRule
    {
    private:

      // References to the NetworkSimplex class
      const IntVector  &_source;
      const IntVector  &_target;
      const ValueVector &_cost;
      const IntVector  &_state;
      const ValueVector &_pi;
      int &_in_arc;
      int _arc_num;

      // Pivot rule data
      int _next_arc;

    public:

      // Constructor
      FirstEligiblePivotRule(NetworkSimplex &ns) :
        _source(ns._source), _target(ns._target),
        _cost(ns._cost), _state(ns._state), _pi(ns._pi),
        _in_arc(ns.in_arc), _arc_num(ns._arc_num), _next_arc(0)
      {}

      // Find next entering arc
      bool findEnteringArc() {
        Value c;
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


    // Implementation of the Best Eligible pivot rule
    class BestEligiblePivotRule
    {
    private:

      // References to the NetworkSimplex class
      const IntVector  &_source;
      const IntVector  &_target;
      const ValueVector &_cost;
      const IntVector  &_state;
      const ValueVector &_pi;
      int &_in_arc;
      int _arc_num;

    public:

      // Constructor
      BestEligiblePivotRule(NetworkSimplex &ns) :
        _source(ns._source), _target(ns._target),
        _cost(ns._cost), _state(ns._state), _pi(ns._pi),
        _in_arc(ns.in_arc), _arc_num(ns._arc_num)
      {}

      // Find next entering arc
      bool findEnteringArc() {
        Value c, min = 0;
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


    // Implementation of the Block Search pivot rule
    class BlockSearchPivotRule
    {
    private:

      // References to the NetworkSimplex class
      const IntVector  &_source;
      const IntVector  &_target;
      const ValueVector &_cost;
      const IntVector  &_state;
      const ValueVector &_pi;
      int &_in_arc;
      int _arc_num;

      // Pivot rule data
      int _block_size;
      int _next_arc;

    public:

      // Constructor
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

      // Find next entering arc
      bool findEnteringArc() {
        Value c, min = 0;
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


    // Implementation of the Candidate List pivot rule
    class CandidateListPivotRule
    {
    private:

      // References to the NetworkSimplex class
      const IntVector  &_source;
      const IntVector  &_target;
      const ValueVector &_cost;
      const IntVector  &_state;
      const ValueVector &_pi;
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
        Value min, c;
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


    // Implementation of the Altering Candidate List pivot rule
    class AlteringListPivotRule
    {
    private:

      // References to the NetworkSimplex class
      const IntVector  &_source;
      const IntVector  &_target;
      const ValueVector &_cost;
      const IntVector  &_state;
      const ValueVector &_pi;
      int &_in_arc;
      int _arc_num;

      // Pivot rule data
      int _block_size, _head_length, _curr_length;
      int _next_arc;
      IntVector _candidates;
      ValueVector _cand_cost;

      // Functor class to compare arcs during sort of the candidate list
      class SortFunc
      {
      private:
        const ValueVector &_map;
      public:
        SortFunc(const ValueVector &map) : _map(map) {}
        bool operator()(int left, int right) {
          return _map[left] > _map[right];
        }
      };

      SortFunc _sort_func;

    public:

      // Constructor
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

      // Find next entering arc
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

    /// \brief Constructor.
    ///
    /// Constructor.
    ///
    /// \param graph The digraph the algorithm runs on.
    NetworkSimplex(const GR& graph) :
      _graph(graph),
      _plower(NULL), _pupper(NULL), _pcost(NULL),
      _psupply(NULL), _pstsup(false),
      _flow_map(NULL), _potential_map(NULL),
      _local_flow(false), _local_potential(false),
      _node_id(graph)
    {
      LEMON_ASSERT(std::numeric_limits<Value>::is_integer &&
                   std::numeric_limits<Value>::is_signed,
        "The value type of NetworkSimplex must be a signed integer");
    }

    /// Destructor.
    ~NetworkSimplex() {
      if (_local_flow) delete _flow_map;
      if (_local_potential) delete _potential_map;
    }

    /// \brief Set the lower bounds on the arcs.
    ///
    /// This function sets the lower bounds on the arcs.
    /// If neither this function nor \ref boundMaps() is used before
    /// calling \ref run(), the lower bounds will be set to zero
    /// on all arcs.
    ///
    /// \param map An arc map storing the lower bounds.
    /// Its \c Value type must be convertible to the \c Value type
    /// of the algorithm.
    ///
    /// \return <tt>(*this)</tt>
    template <typename LOWER>
    NetworkSimplex& lowerMap(const LOWER& map) {
      delete _plower;
      _plower = new ValueArcMap(_graph);
      for (ArcIt a(_graph); a != INVALID; ++a) {
        (*_plower)[a] = map[a];
      }
      return *this;
    }

    /// \brief Set the upper bounds (capacities) on the arcs.
    ///
    /// This function sets the upper bounds (capacities) on the arcs.
    /// If none of the functions \ref upperMap(), \ref capacityMap()
    /// and \ref boundMaps() is used before calling \ref run(),
    /// the upper bounds (capacities) will be set to
    /// \c std::numeric_limits<Value>::max() on all arcs.
    ///
    /// \param map An arc map storing the upper bounds.
    /// Its \c Value type must be convertible to the \c Value type
    /// of the algorithm.
    ///
    /// \return <tt>(*this)</tt>
    template<typename UPPER>
    NetworkSimplex& upperMap(const UPPER& map) {
      delete _pupper;
      _pupper = new ValueArcMap(_graph);
      for (ArcIt a(_graph); a != INVALID; ++a) {
        (*_pupper)[a] = map[a];
      }
      return *this;
    }

    /// \brief Set the upper bounds (capacities) on the arcs.
    ///
    /// This function sets the upper bounds (capacities) on the arcs.
    /// It is just an alias for \ref upperMap().
    ///
    /// \return <tt>(*this)</tt>
    template<typename CAP>
    NetworkSimplex& capacityMap(const CAP& map) {
      return upperMap(map);
    }

    /// \brief Set the lower and upper bounds on the arcs.
    ///
    /// This function sets the lower and upper bounds on the arcs.
    /// If neither this function nor \ref lowerMap() is used before
    /// calling \ref run(), the lower bounds will be set to zero
    /// on all arcs.
    /// If none of the functions \ref upperMap(), \ref capacityMap()
    /// and \ref boundMaps() is used before calling \ref run(),
    /// the upper bounds (capacities) will be set to
    /// \c std::numeric_limits<Value>::max() on all arcs.
    ///
    /// \param lower An arc map storing the lower bounds.
    /// \param upper An arc map storing the upper bounds.
    ///
    /// The \c Value type of the maps must be convertible to the
    /// \c Value type of the algorithm.
    ///
    /// \note This function is just a shortcut of calling \ref lowerMap()
    /// and \ref upperMap() separately.
    ///
    /// \return <tt>(*this)</tt>
    template <typename LOWER, typename UPPER>
    NetworkSimplex& boundMaps(const LOWER& lower, const UPPER& upper) {
      return lowerMap(lower).upperMap(upper);
    }

    /// \brief Set the costs of the arcs.
    ///
    /// This function sets the costs of the arcs.
    /// If it is not used before calling \ref run(), the costs
    /// will be set to \c 1 on all arcs.
    ///
    /// \param map An arc map storing the costs.
    /// Its \c Value type must be convertible to the \c Value type
    /// of the algorithm.
    ///
    /// \return <tt>(*this)</tt>
    template<typename COST>
    NetworkSimplex& costMap(const COST& map) {
      delete _pcost;
      _pcost = new ValueArcMap(_graph);
      for (ArcIt a(_graph); a != INVALID; ++a) {
        (*_pcost)[a] = map[a];
      }
      return *this;
    }

    /// \brief Set the supply values of the nodes.
    ///
    /// This function sets the supply values of the nodes.
    /// If neither this function nor \ref stSupply() is used before
    /// calling \ref run(), the supply of each node will be set to zero.
    /// (It makes sense only if non-zero lower bounds are given.)
    ///
    /// \param map A node map storing the supply values.
    /// Its \c Value type must be convertible to the \c Value type
    /// of the algorithm.
    ///
    /// \return <tt>(*this)</tt>
    template<typename SUP>
    NetworkSimplex& supplyMap(const SUP& map) {
      delete _psupply;
      _pstsup = false;
      _psupply = new ValueNodeMap(_graph);
      for (NodeIt n(_graph); n != INVALID; ++n) {
        (*_psupply)[n] = map[n];
      }
      return *this;
    }

    /// \brief Set single source and target nodes and a supply value.
    ///
    /// This function sets a single source node and a single target node
    /// and the required flow value.
    /// If neither this function nor \ref supplyMap() is used before
    /// calling \ref run(), the supply of each node will be set to zero.
    /// (It makes sense only if non-zero lower bounds are given.)
    ///
    /// \param s The source node.
    /// \param t The target node.
    /// \param k The required amount of flow from node \c s to node \c t
    /// (i.e. the supply of \c s and the demand of \c t).
    ///
    /// \return <tt>(*this)</tt>
    NetworkSimplex& stSupply(const Node& s, const Node& t, Value k) {
      delete _psupply;
      _psupply = NULL;
      _pstsup = true;
      _psource = s;
      _ptarget = t;
      _pstflow = k;
      return *this;
    }

    /// \brief Set the flow map.
    ///
    /// This function sets the flow map.
    /// If it is not used before calling \ref run(), an instance will
    /// be allocated automatically. The destructor deallocates this
    /// automatically allocated map, of course.
    ///
    /// \return <tt>(*this)</tt>
    NetworkSimplex& flowMap(FlowMap& map) {
      if (_local_flow) {
        delete _flow_map;
        _local_flow = false;
      }
      _flow_map = &map;
      return *this;
    }

    /// \brief Set the potential map.
    ///
    /// This function sets the potential map, which is used for storing
    /// the dual solution.
    /// If it is not used before calling \ref run(), an instance will
    /// be allocated automatically. The destructor deallocates this
    /// automatically allocated map, of course.
    ///
    /// \return <tt>(*this)</tt>
    NetworkSimplex& potentialMap(PotentialMap& map) {
      if (_local_potential) {
        delete _potential_map;
        _local_potential = false;
      }
      _potential_map = &map;
      return *this;
    }

    /// \name Execution Control
    /// The algorithm can be executed using \ref run().

    /// @{

    /// \brief Run the algorithm.
    ///
    /// This function runs the algorithm.
    /// The paramters can be specified using \ref lowerMap(),
    /// \ref upperMap(), \ref capacityMap(), \ref boundMaps(), 
    /// \ref costMap(), \ref supplyMap() and \ref stSupply()
    /// functions. For example,
    /// \code
    ///   NetworkSimplex<ListDigraph> ns(graph);
    ///   ns.boundMaps(lower, upper).costMap(cost)
    ///     .supplyMap(sup).run();
    /// \endcode
    ///
    /// \param pivot_rule The pivot rule that will be used during the
    /// algorithm. For more information see \ref PivotRule.
    ///
    /// \return \c true if a feasible flow can be found.
    bool run(PivotRule pivot_rule = BLOCK_SEARCH) {
      return init() && start(pivot_rule);
    }

    /// @}

    /// \name Query Functions
    /// The results of the algorithm can be obtained using these
    /// functions.\n
    /// The \ref run() function must be called before using them.

    /// @{

    /// \brief Return the total cost of the found flow.
    ///
    /// This function returns the total cost of the found flow.
    /// The complexity of the function is \f$ O(e) \f$.
    ///
    /// \note The return type of the function can be specified as a
    /// template parameter. For example,
    /// \code
    ///   ns.totalCost<double>();
    /// \endcode
    /// It is useful if the total cost cannot be stored in the \c Value
    /// type of the algorithm, which is the default return type of the
    /// function.
    ///
    /// \pre \ref run() must be called before using this function.
    template <typename Num>
    Num totalCost() const {
      Num c = 0;
      if (_pcost) {
        for (ArcIt e(_graph); e != INVALID; ++e)
          c += (*_flow_map)[e] * (*_pcost)[e];
      } else {
        for (ArcIt e(_graph); e != INVALID; ++e)
          c += (*_flow_map)[e];
      }
      return c;
    }

#ifndef DOXYGEN
    Value totalCost() const {
      return totalCost<Value>();
    }
#endif

    /// \brief Return the flow on the given arc.
    ///
    /// This function returns the flow on the given arc.
    ///
    /// \pre \ref run() must be called before using this function.
    Value flow(const Arc& a) const {
      return (*_flow_map)[a];
    }

    /// \brief Return a const reference to the flow map.
    ///
    /// This function returns a const reference to an arc map storing
    /// the found flow.
    ///
    /// \pre \ref run() must be called before using this function.
    const FlowMap& flowMap() const {
      return *_flow_map;
    }

    /// \brief Return the potential (dual value) of the given node.
    ///
    /// This function returns the potential (dual value) of the
    /// given node.
    ///
    /// \pre \ref run() must be called before using this function.
    Value potential(const Node& n) const {
      return (*_potential_map)[n];
    }

    /// \brief Return a const reference to the potential map
    /// (the dual solution).
    ///
    /// This function returns a const reference to a node map storing
    /// the found potentials, which form the dual solution of the
    /// \ref min_cost_flow "minimum cost flow" problem.
    ///
    /// \pre \ref run() must be called before using this function.
    const PotentialMap& potentialMap() const {
      return *_potential_map;
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
      if (_node_num == 0) return false;

      _arc_ref.resize(_arc_num);
      _source.resize(all_arc_num);
      _target.resize(all_arc_num);

      _cap.resize(all_arc_num);
      _cost.resize(all_arc_num);
      _supply.resize(all_node_num);
      _flow.resize(all_arc_num, 0);
      _pi.resize(all_node_num, 0);

      _parent.resize(all_node_num);
      _pred.resize(all_node_num);
      _forward.resize(all_node_num);
      _thread.resize(all_node_num);
      _rev_thread.resize(all_node_num);
      _succ_num.resize(all_node_num);
      _last_succ.resize(all_node_num);
      _state.resize(all_arc_num, STATE_LOWER);

      // Initialize node related data
      bool valid_supply = true;
      if (!_pstsup && !_psupply) {
        _pstsup = true;
        _psource = _ptarget = NodeIt(_graph);
        _pstflow = 0;
      }
      if (_psupply) {
        Value sum = 0;
        int i = 0;
        for (NodeIt n(_graph); n != INVALID; ++n, ++i) {
          _node_id[n] = i;
          _supply[i] = (*_psupply)[n];
          sum += _supply[i];
        }
        valid_supply = (sum == 0);
      } else {
        int i = 0;
        for (NodeIt n(_graph); n != INVALID; ++n, ++i) {
          _node_id[n] = i;
          _supply[i] = 0;
        }
        _supply[_node_id[_psource]] =  _pstflow;
        _supply[_node_id[_ptarget]]   = -_pstflow;
      }
      if (!valid_supply) return false;

      // Set data for the artificial root node
      _root = _node_num;
      _parent[_root] = -1;
      _pred[_root] = -1;
      _thread[_root] = 0;
      _rev_thread[0] = _root;
      _succ_num[_root] = all_node_num;
      _last_succ[_root] = _root - 1;
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
      if (_pupper && _pcost) {
        for (int i = 0; i != _arc_num; ++i) {
          Arc e = _arc_ref[i];
          _source[i] = _node_id[_graph.source(e)];
          _target[i] = _node_id[_graph.target(e)];
          _cap[i] = (*_pupper)[e];
          _cost[i] = (*_pcost)[e];
        }
      } else {
        for (int i = 0; i != _arc_num; ++i) {
          Arc e = _arc_ref[i];
          _source[i] = _node_id[_graph.source(e)];
          _target[i] = _node_id[_graph.target(e)];
        }
        if (_pupper) {
          for (int i = 0; i != _arc_num; ++i)
            _cap[i] = (*_pupper)[_arc_ref[i]];
        } else {
          Value val = std::numeric_limits<Value>::max();
          for (int i = 0; i != _arc_num; ++i)
            _cap[i] = val;
        }
        if (_pcost) {
          for (int i = 0; i != _arc_num; ++i)
            _cost[i] = (*_pcost)[_arc_ref[i]];
        } else {
          for (int i = 0; i != _arc_num; ++i)
            _cost[i] = 1;
        }
      }

      // Remove non-zero lower bounds
      if (_plower) {
        for (int i = 0; i != _arc_num; ++i) {
          Value c = (*_plower)[_arc_ref[i]];
          if (c != 0) {
            _cap[i] -= c;
            _supply[_source[i]] -= c;
            _supply[_target[i]] += c;
          }
        }
      }

      // Add artificial arcs and initialize the spanning tree data structure
      Value max_cap = std::numeric_limits<Value>::max();
      Value max_cost = std::numeric_limits<Value>::max() / 4;
      for (int u = 0, e = _arc_num; u != _node_num; ++u, ++e) {
        _thread[u] = u + 1;
        _rev_thread[u + 1] = u;
        _succ_num[u] = 1;
        _last_succ[u] = u;
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
      while (u != v) {
        if (_succ_num[u] < _succ_num[v]) {
          u = _parent[u];
        } else {
          v = _parent[v];
        }
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
      Value d;
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
        Value val = _state[in_arc] * delta;
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

    // Update the tree structure
    void updateTreeStructure() {
      int u, w;
      int old_rev_thread = _rev_thread[u_out];
      int old_succ_num = _succ_num[u_out];
      int old_last_succ = _last_succ[u_out];
      v_out = _parent[u_out];

      u = _last_succ[u_in];  // the last successor of u_in
      right = _thread[u];    // the node after it

      // Handle the case when old_rev_thread equals to v_in
      // (it also means that join and v_out coincide)
      if (old_rev_thread == v_in) {
        last = _thread[_last_succ[u_out]];
      } else {
        last = _thread[v_in];
      }

      // Update _thread and _parent along the stem nodes (i.e. the nodes
      // between u_in and u_out, whose parent have to be changed)
      _thread[v_in] = stem = u_in;
      _dirty_revs.clear();
      _dirty_revs.push_back(v_in);
      par_stem = v_in;
      while (stem != u_out) {
        // Insert the next stem node into the thread list
        new_stem = _parent[stem];
        _thread[u] = new_stem;
        _dirty_revs.push_back(u);

        // Remove the subtree of stem from the thread list
        w = _rev_thread[stem];
        _thread[w] = right;
        _rev_thread[right] = w;

        // Change the parent node and shift stem nodes
        _parent[stem] = par_stem;
        par_stem = stem;
        stem = new_stem;

        // Update u and right
        u = _last_succ[stem] == _last_succ[par_stem] ?
          _rev_thread[par_stem] : _last_succ[stem];
        right = _thread[u];
      }
      _parent[u_out] = par_stem;
      _thread[u] = last;
      _rev_thread[last] = u;
      _last_succ[u_out] = u;

      // Remove the subtree of u_out from the thread list except for
      // the case when old_rev_thread equals to v_in
      // (it also means that join and v_out coincide)
      if (old_rev_thread != v_in) {
        _thread[old_rev_thread] = right;
        _rev_thread[right] = old_rev_thread;
      }

      // Update _rev_thread using the new _thread values
      for (int i = 0; i < int(_dirty_revs.size()); ++i) {
        u = _dirty_revs[i];
        _rev_thread[_thread[u]] = u;
      }

      // Update _pred, _forward, _last_succ and _succ_num for the
      // stem nodes from u_out to u_in
      int tmp_sc = 0, tmp_ls = _last_succ[u_out];
      u = u_out;
      while (u != u_in) {
        w = _parent[u];
        _pred[u] = _pred[w];
        _forward[u] = !_forward[w];
        tmp_sc += _succ_num[u] - _succ_num[w];
        _succ_num[u] = tmp_sc;
        _last_succ[w] = tmp_ls;
        u = w;
      }
      _pred[u_in] = in_arc;
      _forward[u_in] = (u_in == _source[in_arc]);
      _succ_num[u_in] = old_succ_num;

      // Set limits for updating _last_succ form v_in and v_out
      // towards the root
      int up_limit_in = -1;
      int up_limit_out = -1;
      if (_last_succ[join] == v_in) {
        up_limit_out = join;
      } else {
        up_limit_in = join;
      }

      // Update _last_succ from v_in towards the root
      for (u = v_in; u != up_limit_in && _last_succ[u] == v_in;
           u = _parent[u]) {
        _last_succ[u] = _last_succ[u_out];
      }
      // Update _last_succ from v_out towards the root
      if (join != old_rev_thread && v_in != old_rev_thread) {
        for (u = v_out; u != up_limit_out && _last_succ[u] == old_last_succ;
             u = _parent[u]) {
          _last_succ[u] = old_rev_thread;
        }
      } else {
        for (u = v_out; u != up_limit_out && _last_succ[u] == old_last_succ;
             u = _parent[u]) {
          _last_succ[u] = _last_succ[u_out];
        }
      }

      // Update _succ_num from v_in to join
      for (u = v_in; u != join; u = _parent[u]) {
        _succ_num[u] += old_succ_num;
      }
      // Update _succ_num from v_out to join
      for (u = v_out; u != join; u = _parent[u]) {
        _succ_num[u] -= old_succ_num;
      }
    }

    // Update potentials
    void updatePotential() {
      Value sigma = _forward[u_in] ?
        _pi[v_in] - _pi[u_in] - _cost[_pred[u_in]] :
        _pi[v_in] - _pi[u_in] + _cost[_pred[u_in]];
      if (_succ_num[u_in] > _node_num / 2) {
        // Update in the upper subtree (which contains the root)
        int before = _rev_thread[u_in];
        int after = _thread[_last_succ[u_in]];
        _thread[before] = after;
        _pi[_root] -= sigma;
        for (int u = _thread[_root]; u != _root; u = _thread[u]) {
          _pi[u] -= sigma;
        }
        _thread[before] = u_in;
      } else {
        // Update in the lower subtree (which has been moved)
        int end = _thread[_last_succ[u_in]];
        for (int u = u_in; u != end; u = _thread[u]) {
          _pi[u] += sigma;
        }
      }
    }

    // Execute the algorithm
    bool start(PivotRule pivot_rule) {
      // Select the pivot rule implementation
      switch (pivot_rule) {
        case FIRST_ELIGIBLE:
          return start<FirstEligiblePivotRule>();
        case BEST_ELIGIBLE:
          return start<BestEligiblePivotRule>();
        case BLOCK_SEARCH:
          return start<BlockSearchPivotRule>();
        case CANDIDATE_LIST:
          return start<CandidateListPivotRule>();
        case ALTERING_LIST:
          return start<AlteringListPivotRule>();
      }
      return false;
    }

    template <typename PivotRuleImpl>
    bool start() {
      PivotRuleImpl pivot(*this);

      // Execute the Network Simplex algorithm
      while (pivot.findEnteringArc()) {
        findJoinNode();
        bool change = findLeavingArc();
        changeFlow(change);
        if (change) {
          updateTreeStructure();
          updatePotential();
        }
      }

      // Check if the flow amount equals zero on all the artificial arcs
      for (int e = _arc_num; e != _arc_num + _node_num; ++e) {
        if (_flow[e] > 0) return false;
      }

      // Copy flow values to _flow_map
      if (_plower) {
        for (int i = 0; i != _arc_num; ++i) {
          Arc e = _arc_ref[i];
          _flow_map->set(e, (*_plower)[e] + _flow[i]);
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
