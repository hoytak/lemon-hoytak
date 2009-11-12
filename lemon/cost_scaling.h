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

#ifndef LEMON_COST_SCALING_H
#define LEMON_COST_SCALING_H

/// \ingroup min_cost_flow_algs
/// \file
/// \brief Cost scaling algorithm for finding a minimum cost flow.

#include <vector>
#include <deque>
#include <limits>

#include <lemon/core.h>
#include <lemon/maps.h>
#include <lemon/math.h>
#include <lemon/adaptors.h>
#include <lemon/circulation.h>
#include <lemon/bellman_ford.h>

namespace lemon {

  /// \addtogroup min_cost_flow_algs
  /// @{

  /// \brief Implementation of the cost scaling algorithm for finding a
  /// minimum cost flow.
  ///
  /// \ref CostScaling implements the cost scaling algorithm performing
  /// augment/push and relabel operations for finding a minimum cost
  /// flow.
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
  /// \note Arc costs are multiplied with the number of nodes during
  /// the algorithm so overflow problems may arise more easily than with
  /// other minimum cost flow algorithms.
  /// If it is available, <tt>long long int</tt> type is used instead of
  /// <tt>long int</tt> in the inside computations.
  ///
  /// \author Peter Kovacs
  template < typename Digraph,
             typename LowerMap = typename Digraph::template ArcMap<int>,
             typename CapacityMap = typename Digraph::template ArcMap<int>,
             typename CostMap = typename Digraph::template ArcMap<int>,
             typename SupplyMap = typename Digraph::template NodeMap<int> >
  class CostScaling
  {
    TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

    typedef typename CapacityMap::Value Capacity;
    typedef typename CostMap::Value Cost;
    typedef typename SupplyMap::Value Supply;
    typedef typename Digraph::template ArcMap<Capacity> CapacityArcMap;
    typedef typename Digraph::template NodeMap<Supply> SupplyNodeMap;

    typedef ResidualDigraph< const Digraph,
                             CapacityArcMap, CapacityArcMap > ResDigraph;
    typedef typename ResDigraph::Arc ResArc;

#if defined __GNUC__ && !defined __STRICT_ANSI__
    typedef long long int LCost;
#else
    typedef long int LCost;
#endif
    typedef typename Digraph::template ArcMap<LCost> LargeCostMap;

  public:

    /// The type of the flow map.
    typedef typename Digraph::template ArcMap<Capacity> FlowMap;
    /// The type of the potential map.
    typedef typename Digraph::template NodeMap<LCost> PotentialMap;

  private:

    /// \brief Map adaptor class for handling residual arc costs.
    ///
    /// Map adaptor class for handling residual arc costs.
    template <typename Map>
    class ResidualCostMap : public MapBase<ResArc, typename Map::Value>
    {
    private:

      const Map &_cost_map;

    public:

      ///\e
      ResidualCostMap(const Map &cost_map) :
        _cost_map(cost_map) {}

      ///\e
      inline typename Map::Value operator[](const ResArc &e) const {
        return ResDigraph::forward(e) ? _cost_map[e] : -_cost_map[e];
      }

    }; //class ResidualCostMap

    /// \brief Map adaptor class for handling reduced arc costs.
    ///
    /// Map adaptor class for handling reduced arc costs.
    class ReducedCostMap : public MapBase<Arc, LCost>
    {
    private:

      const Digraph &_gr;
      const LargeCostMap &_cost_map;
      const PotentialMap &_pot_map;

    public:

      ///\e
      ReducedCostMap( const Digraph &gr,
                      const LargeCostMap &cost_map,
                      const PotentialMap &pot_map ) :
        _gr(gr), _cost_map(cost_map), _pot_map(pot_map) {}

      ///\e
      inline LCost operator[](const Arc &e) const {
        return _cost_map[e] + _pot_map[_gr.source(e)]
                            - _pot_map[_gr.target(e)];
      }

    }; //class ReducedCostMap

  private:

    // The digraph the algorithm runs on
    const Digraph &_graph;
    // The original lower bound map
    const LowerMap *_lower;
    // The modified capacity map
    CapacityArcMap _capacity;
    // The original cost map
    const CostMap &_orig_cost;
    // The scaled cost map
    LargeCostMap _cost;
    // The modified supply map
    SupplyNodeMap _supply;
    bool _valid_supply;

    // Arc map of the current flow
    FlowMap *_flow;
    bool _local_flow;
    // Node map of the current potentials
    PotentialMap *_potential;
    bool _local_potential;

    // The residual cost map
    ResidualCostMap<LargeCostMap> _res_cost;
    // The residual digraph
    ResDigraph *_res_graph;
    // The reduced cost map
    ReducedCostMap *_red_cost;
    // The excess map
    SupplyNodeMap _excess;
    // The epsilon parameter used for cost scaling
    LCost _epsilon;
    // The scaling factor
    int _alpha;

  public:

    /// \brief General constructor (with lower bounds).
    ///
    /// General constructor (with lower bounds).
    ///
    /// \param digraph The digraph the algorithm runs on.
    /// \param lower The lower bounds of the arcs.
    /// \param capacity The capacities (upper bounds) of the arcs.
    /// \param cost The cost (length) values of the arcs.
    /// \param supply The supply values of the nodes (signed).
    CostScaling( const Digraph &digraph,
                 const LowerMap &lower,
                 const CapacityMap &capacity,
                 const CostMap &cost,
                 const SupplyMap &supply ) :
      _graph(digraph), _lower(&lower), _capacity(digraph), _orig_cost(cost),
      _cost(digraph), _supply(digraph), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false), _res_cost(_cost),
      _res_graph(NULL), _red_cost(NULL), _excess(digraph, 0)
    {
      // Check the sum of supply values
      Supply sum = 0;
      for (NodeIt n(_graph); n != INVALID; ++n) sum += _supply[n];
      _valid_supply = sum == 0;
      
      for (ArcIt e(_graph); e != INVALID; ++e) _capacity[e] = capacity[e];
      for (NodeIt n(_graph); n != INVALID; ++n) _supply[n] = supply[n];

      // Remove non-zero lower bounds
      for (ArcIt e(_graph); e != INVALID; ++e) {
        if (lower[e] != 0) {
          _capacity[e] -= lower[e];
          _supply[_graph.source(e)] -= lower[e];
          _supply[_graph.target(e)] += lower[e];
        }
      }
    }
/*
    /// \brief General constructor (without lower bounds).
    ///
    /// General constructor (without lower bounds).
    ///
    /// \param digraph The digraph the algorithm runs on.
    /// \param capacity The capacities (upper bounds) of the arcs.
    /// \param cost The cost (length) values of the arcs.
    /// \param supply The supply values of the nodes (signed).
    CostScaling( const Digraph &digraph,
                 const CapacityMap &capacity,
                 const CostMap &cost,
                 const SupplyMap &supply ) :
      _graph(digraph), _lower(NULL), _capacity(capacity), _orig_cost(cost),
      _cost(digraph), _supply(supply), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false), _res_cost(_cost),
      _res_graph(NULL), _red_cost(NULL), _excess(digraph, 0)
    {
      // Check the sum of supply values
      Supply sum = 0;
      for (NodeIt n(_graph); n != INVALID; ++n) sum += _supply[n];
      _valid_supply = sum == 0;
    }

    /// \brief Simple constructor (with lower bounds).
    ///
    /// Simple constructor (with lower bounds).
    ///
    /// \param digraph The digraph the algorithm runs on.
    /// \param lower The lower bounds of the arcs.
    /// \param capacity The capacities (upper bounds) of the arcs.
    /// \param cost The cost (length) values of the arcs.
    /// \param s The source node.
    /// \param t The target node.
    /// \param flow_value The required amount of flow from node \c s
    /// to node \c t (i.e. the supply of \c s and the demand of \c t).
    CostScaling( const Digraph &digraph,
                 const LowerMap &lower,
                 const CapacityMap &capacity,
                 const CostMap &cost,
                 Node s, Node t,
                 Supply flow_value ) :
      _graph(digraph), _lower(&lower), _capacity(capacity), _orig_cost(cost),
      _cost(digraph), _supply(digraph, 0), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false), _res_cost(_cost),
      _res_graph(NULL), _red_cost(NULL), _excess(digraph, 0)
    {
      // Remove non-zero lower bounds
      _supply[s] =  flow_value;
      _supply[t] = -flow_value;
      for (ArcIt e(_graph); e != INVALID; ++e) {
        if (lower[e] != 0) {
          _capacity[e] -= lower[e];
          _supply[_graph.source(e)] -= lower[e];
          _supply[_graph.target(e)] += lower[e];
        }
      }
      _valid_supply = true;
    }

    /// \brief Simple constructor (without lower bounds).
    ///
    /// Simple constructor (without lower bounds).
    ///
    /// \param digraph The digraph the algorithm runs on.
    /// \param capacity The capacities (upper bounds) of the arcs.
    /// \param cost The cost (length) values of the arcs.
    /// \param s The source node.
    /// \param t The target node.
    /// \param flow_value The required amount of flow from node \c s
    /// to node \c t (i.e. the supply of \c s and the demand of \c t).
    CostScaling( const Digraph &digraph,
                 const CapacityMap &capacity,
                 const CostMap &cost,
                 Node s, Node t,
                 Supply flow_value ) :
      _graph(digraph), _lower(NULL), _capacity(capacity), _orig_cost(cost),
      _cost(digraph), _supply(digraph, 0), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false), _res_cost(_cost),
      _res_graph(NULL), _red_cost(NULL), _excess(digraph, 0)
    {
      _supply[s] =  flow_value;
      _supply[t] = -flow_value;
      _valid_supply = true;
    }
*/
    /// Destructor.
    ~CostScaling() {
      if (_local_flow) delete _flow;
      if (_local_potential) delete _potential;
      delete _res_graph;
      delete _red_cost;
    }

    /// \brief Set the flow map.
    ///
    /// Set the flow map.
    ///
    /// \return \c (*this)
    CostScaling& flowMap(FlowMap &map) {
      if (_local_flow) {
        delete _flow;
        _local_flow = false;
      }
      _flow = &map;
      return *this;
    }

    /// \brief Set the potential map.
    ///
    /// Set the potential map.
    ///
    /// \return \c (*this)
    CostScaling& potentialMap(PotentialMap &map) {
      if (_local_potential) {
        delete _potential;
        _local_potential = false;
      }
      _potential = &map;
      return *this;
    }

    /// \name Execution control

    /// @{

    /// \brief Run the algorithm.
    ///
    /// Run the algorithm.
    ///
    /// \param partial_augment By default the algorithm performs
    /// partial augment and relabel operations in the cost scaling
    /// phases. Set this parameter to \c false for using local push and
    /// relabel operations instead.
    ///
    /// \return \c true if a feasible flow can be found.
    bool run(bool partial_augment = true) {
      if (partial_augment) {
        return init() && startPartialAugment();
      } else {
        return init() && startPushRelabel();
      }
    }

    /// @}

    /// \name Query Functions
    /// The result of the algorithm can be obtained using these
    /// functions.\n
    /// \ref lemon::CostScaling::run() "run()" must be called before
    /// using them.

    /// @{

    /// \brief Return a const reference to the arc map storing the
    /// found flow.
    ///
    /// Return a const reference to the arc map storing the found flow.
    ///
    /// \pre \ref run() must be called before using this function.
    const FlowMap& flowMap() const {
      return *_flow;
    }

    /// \brief Return a const reference to the node map storing the
    /// found potentials (the dual solution).
    ///
    /// Return a const reference to the node map storing the found
    /// potentials (the dual solution).
    ///
    /// \pre \ref run() must be called before using this function.
    const PotentialMap& potentialMap() const {
      return *_potential;
    }

    /// \brief Return the flow on the given arc.
    ///
    /// Return the flow on the given arc.
    ///
    /// \pre \ref run() must be called before using this function.
    Capacity flow(const Arc& arc) const {
      return (*_flow)[arc];
    }

    /// \brief Return the potential of the given node.
    ///
    /// Return the potential of the given node.
    ///
    /// \pre \ref run() must be called before using this function.
    Cost potential(const Node& node) const {
      return (*_potential)[node];
    }

    /// \brief Return the total cost of the found flow.
    ///
    /// Return the total cost of the found flow. The complexity of the
    /// function is \f$ O(e) \f$.
    ///
    /// \pre \ref run() must be called before using this function.
    Cost totalCost() const {
      Cost c = 0;
      for (ArcIt e(_graph); e != INVALID; ++e)
        c += (*_flow)[e] * _orig_cost[e];
      return c;
    }

    /// @}

  private:

    /// Initialize the algorithm.
    bool init() {
      if (!_valid_supply) return false;
      // The scaling factor
      _alpha = 8;

      // Initialize flow and potential maps
      if (!_flow) {
        _flow = new FlowMap(_graph);
        _local_flow = true;
      }
      if (!_potential) {
        _potential = new PotentialMap(_graph);
        _local_potential = true;
      }

      _red_cost = new ReducedCostMap(_graph, _cost, *_potential);
      _res_graph = new ResDigraph(_graph, _capacity, *_flow);

      // Initialize the scaled cost map and the epsilon parameter
      Cost max_cost = 0;
      int node_num = countNodes(_graph);
      for (ArcIt e(_graph); e != INVALID; ++e) {
        _cost[e] = LCost(_orig_cost[e]) * node_num * _alpha;
        if (_orig_cost[e] > max_cost) max_cost = _orig_cost[e];
      }
      _epsilon = max_cost * node_num;

      // Find a feasible flow using Circulation
      Circulation< Digraph, ConstMap<Arc, Capacity>, CapacityArcMap,
                   SupplyMap >
        circulation( _graph, constMap<Arc>(Capacity(0)), _capacity,
                     _supply );
      return circulation.flowMap(*_flow).run();
    }

    /// Execute the algorithm performing partial augmentation and
    /// relabel operations.
    bool startPartialAugment() {
      // Paramters for heuristics
//      const int BF_HEURISTIC_EPSILON_BOUND = 1000;
//      const int BF_HEURISTIC_BOUND_FACTOR  = 3;
      // Maximum augment path length
      const int MAX_PATH_LENGTH = 4;

      // Variables
      typename Digraph::template NodeMap<Arc> pred_arc(_graph);
      typename Digraph::template NodeMap<bool> forward(_graph);
      typename Digraph::template NodeMap<OutArcIt> next_out(_graph);
      typename Digraph::template NodeMap<InArcIt> next_in(_graph);
      typename Digraph::template NodeMap<bool> next_dir(_graph);
      std::deque<Node> active_nodes;
      std::vector<Node> path_nodes;

//      int node_num = countNodes(_graph);
      for ( ; _epsilon >= 1; _epsilon = _epsilon < _alpha && _epsilon > 1 ?
                                        1 : _epsilon / _alpha )
      {
/*
        // "Early Termination" heuristic: use Bellman-Ford algorithm
        // to check if the current flow is optimal
        if (_epsilon <= BF_HEURISTIC_EPSILON_BOUND) {
          typedef ShiftMap< ResidualCostMap<LargeCostMap> > ShiftCostMap;
          ShiftCostMap shift_cost(_res_cost, 1);
          BellmanFord<ResDigraph, ShiftCostMap> bf(*_res_graph, shift_cost);
          bf.init(0);
          bool done = false;
          int K = int(BF_HEURISTIC_BOUND_FACTOR * sqrt(node_num));
          for (int i = 0; i < K && !done; ++i)
            done = bf.processNextWeakRound();
          if (done) break;
        }
*/
        // Saturate arcs not satisfying the optimality condition
        Capacity delta;
        for (ArcIt e(_graph); e != INVALID; ++e) {
          if (_capacity[e] - (*_flow)[e] > 0 && (*_red_cost)[e] < 0) {
            delta = _capacity[e] - (*_flow)[e];
            _excess[_graph.source(e)] -= delta;
            _excess[_graph.target(e)] += delta;
            (*_flow)[e] = _capacity[e];
          }
          if ((*_flow)[e] > 0 && -(*_red_cost)[e] < 0) {
            _excess[_graph.target(e)] -= (*_flow)[e];
            _excess[_graph.source(e)] += (*_flow)[e];
            (*_flow)[e] = 0;
          }
        }

        // Find active nodes (i.e. nodes with positive excess)
        for (NodeIt n(_graph); n != INVALID; ++n) {
          if (_excess[n] > 0) active_nodes.push_back(n);
        }

        // Initialize the next arc maps
        for (NodeIt n(_graph); n != INVALID; ++n) {
          next_out[n] = OutArcIt(_graph, n);
          next_in[n] = InArcIt(_graph, n);
          next_dir[n] = true;
        }

        // Perform partial augment and relabel operations
        while (active_nodes.size() > 0) {
          // Select an active node (FIFO selection)
          if (_excess[active_nodes[0]] <= 0) {
            active_nodes.pop_front();
            continue;
          }
          Node start = active_nodes[0];
          path_nodes.clear();
          path_nodes.push_back(start);

          // Find an augmenting path from the start node
          Node u, tip = start;
          LCost min_red_cost;
          while ( _excess[tip] >= 0 &&
                  int(path_nodes.size()) <= MAX_PATH_LENGTH )
          {
            if (next_dir[tip]) {
              for (OutArcIt e = next_out[tip]; e != INVALID; ++e) {
                if (_capacity[e] - (*_flow)[e] > 0 && (*_red_cost)[e] < 0) {
                  u = _graph.target(e);
                  pred_arc[u] = e;
                  forward[u] = true;
                  next_out[tip] = e;
                  tip = u;
                  path_nodes.push_back(tip);
                  goto next_step;
                }
              }
              next_dir[tip] = false;
            }
            for (InArcIt e = next_in[tip]; e != INVALID; ++e) {
              if ((*_flow)[e] > 0 && -(*_red_cost)[e] < 0) {
                u = _graph.source(e);
                pred_arc[u] = e;
                forward[u] = false;
                next_in[tip] = e;
                tip = u;
                path_nodes.push_back(tip);
                goto next_step;
              }
            }

            // Relabel tip node
            min_red_cost = std::numeric_limits<LCost>::max() / 2;
            for (OutArcIt oe(_graph, tip); oe != INVALID; ++oe) {
              if ( _capacity[oe] - (*_flow)[oe] > 0 &&
                   (*_red_cost)[oe] < min_red_cost )
                min_red_cost = (*_red_cost)[oe];
            }
            for (InArcIt ie(_graph, tip); ie != INVALID; ++ie) {
              if ((*_flow)[ie] > 0 && -(*_red_cost)[ie] < min_red_cost)
                min_red_cost = -(*_red_cost)[ie];
            }
            (*_potential)[tip] -= min_red_cost + _epsilon;

            // Reset the next arc maps
            next_out[tip] = OutArcIt(_graph, tip);
            next_in[tip] = InArcIt(_graph, tip);
            next_dir[tip] = true;

            // Step back
            if (tip != start) {
              path_nodes.pop_back();
              tip = path_nodes[path_nodes.size()-1];
            }

          next_step:
            continue;
          }

          // Augment along the found path (as much flow as possible)
          Capacity delta;
          for (int i = 1; i < int(path_nodes.size()); ++i) {
            u = path_nodes[i];
            delta = forward[u] ?
              _capacity[pred_arc[u]] - (*_flow)[pred_arc[u]] :
              (*_flow)[pred_arc[u]];
            delta = std::min(delta, _excess[path_nodes[i-1]]);
            (*_flow)[pred_arc[u]] += forward[u] ? delta : -delta;
            _excess[path_nodes[i-1]] -= delta;
            _excess[u] += delta;
            if (_excess[u] > 0 && _excess[u] <= delta) active_nodes.push_back(u);
          }
        }
      }

      // Compute node potentials for the original costs
      ResidualCostMap<CostMap> res_cost(_orig_cost);
      BellmanFord< ResDigraph, ResidualCostMap<CostMap> >
        bf(*_res_graph, res_cost);
      bf.init(0); bf.start();
      for (NodeIt n(_graph); n != INVALID; ++n)
        (*_potential)[n] = bf.dist(n);

      // Handle non-zero lower bounds
      if (_lower) {
        for (ArcIt e(_graph); e != INVALID; ++e)
          (*_flow)[e] += (*_lower)[e];
      }
      return true;
    }

    /// Execute the algorithm performing push and relabel operations.
    bool startPushRelabel() {
      // Paramters for heuristics
//      const int BF_HEURISTIC_EPSILON_BOUND = 1000;
//      const int BF_HEURISTIC_BOUND_FACTOR  = 3;

      typename Digraph::template NodeMap<bool> hyper(_graph, false);
      typename Digraph::template NodeMap<Arc> pred_arc(_graph);
      typename Digraph::template NodeMap<bool> forward(_graph);
      typename Digraph::template NodeMap<OutArcIt> next_out(_graph);
      typename Digraph::template NodeMap<InArcIt> next_in(_graph);
      typename Digraph::template NodeMap<bool> next_dir(_graph);
      std::deque<Node> active_nodes;

//      int node_num = countNodes(_graph);
      for ( ; _epsilon >= 1; _epsilon = _epsilon < _alpha && _epsilon > 1 ?
                                        1 : _epsilon / _alpha )
      {
/*
        // "Early Termination" heuristic: use Bellman-Ford algorithm
        // to check if the current flow is optimal
        if (_epsilon <= BF_HEURISTIC_EPSILON_BOUND) {
          typedef ShiftMap< ResidualCostMap<LargeCostMap> > ShiftCostMap;
          ShiftCostMap shift_cost(_res_cost, 1);
          BellmanFord<ResDigraph, ShiftCostMap> bf(*_res_graph, shift_cost);
          bf.init(0);
          bool done = false;
          int K = int(BF_HEURISTIC_BOUND_FACTOR * sqrt(node_num));
          for (int i = 0; i < K && !done; ++i)
            done = bf.processNextWeakRound();
          if (done) break;
        }
*/

        // Saturate arcs not satisfying the optimality condition
        Capacity delta;
        for (ArcIt e(_graph); e != INVALID; ++e) {
          if (_capacity[e] - (*_flow)[e] > 0 && (*_red_cost)[e] < 0) {
            delta = _capacity[e] - (*_flow)[e];
            _excess[_graph.source(e)] -= delta;
            _excess[_graph.target(e)] += delta;
            (*_flow)[e] = _capacity[e];
          }
          if ((*_flow)[e] > 0 && -(*_red_cost)[e] < 0) {
            _excess[_graph.target(e)] -= (*_flow)[e];
            _excess[_graph.source(e)] += (*_flow)[e];
            (*_flow)[e] = 0;
          }
        }

        // Find active nodes (i.e. nodes with positive excess)
        for (NodeIt n(_graph); n != INVALID; ++n) {
          if (_excess[n] > 0) active_nodes.push_back(n);
        }

        // Initialize the next arc maps
        for (NodeIt n(_graph); n != INVALID; ++n) {
          next_out[n] = OutArcIt(_graph, n);
          next_in[n] = InArcIt(_graph, n);
          next_dir[n] = true;
        }

        // Perform push and relabel operations
        while (active_nodes.size() > 0) {
          // Select an active node (FIFO selection)
          Node n = active_nodes[0], t;
          bool relabel_enabled = true;

          // Perform push operations if there are admissible arcs
          if (_excess[n] > 0 && next_dir[n]) {
            OutArcIt e = next_out[n];
            for ( ; e != INVALID; ++e) {
              if (_capacity[e] - (*_flow)[e] > 0 && (*_red_cost)[e] < 0) {
                delta = std::min(_capacity[e] - (*_flow)[e], _excess[n]);
                t = _graph.target(e);

                // Push-look-ahead heuristic
                Capacity ahead = -_excess[t];
                for (OutArcIt oe(_graph, t); oe != INVALID; ++oe) {
                  if (_capacity[oe] - (*_flow)[oe] > 0 && (*_red_cost)[oe] < 0)
                    ahead += _capacity[oe] - (*_flow)[oe];
                }
                for (InArcIt ie(_graph, t); ie != INVALID; ++ie) {
                  if ((*_flow)[ie] > 0 && -(*_red_cost)[ie] < 0)
                    ahead += (*_flow)[ie];
                }
                if (ahead < 0) ahead = 0;

                // Push flow along the arc
                if (ahead < delta) {
                  (*_flow)[e] += ahead;
                  _excess[n] -= ahead;
                  _excess[t] += ahead;
                  active_nodes.push_front(t);
                  hyper[t] = true;
                  relabel_enabled = false;
                  break;
                } else {
                  (*_flow)[e] += delta;
                  _excess[n] -= delta;
                  _excess[t] += delta;
                  if (_excess[t] > 0 && _excess[t] <= delta)
                    active_nodes.push_back(t);
                }

                if (_excess[n] == 0) break;
              }
            }
            if (e != INVALID) {
              next_out[n] = e;
            } else {
              next_dir[n] = false;
            }
          }

          if (_excess[n] > 0 && !next_dir[n]) {
            InArcIt e = next_in[n];
            for ( ; e != INVALID; ++e) {
              if ((*_flow)[e] > 0 && -(*_red_cost)[e] < 0) {
                delta = std::min((*_flow)[e], _excess[n]);
                t = _graph.source(e);

                // Push-look-ahead heuristic
                Capacity ahead = -_excess[t];
                for (OutArcIt oe(_graph, t); oe != INVALID; ++oe) {
                  if (_capacity[oe] - (*_flow)[oe] > 0 && (*_red_cost)[oe] < 0)
                    ahead += _capacity[oe] - (*_flow)[oe];
                }
                for (InArcIt ie(_graph, t); ie != INVALID; ++ie) {
                  if ((*_flow)[ie] > 0 && -(*_red_cost)[ie] < 0)
                    ahead += (*_flow)[ie];
                }
                if (ahead < 0) ahead = 0;

                // Push flow along the arc
                if (ahead < delta) {
                  (*_flow)[e] -= ahead;
                  _excess[n] -= ahead;
                  _excess[t] += ahead;
                  active_nodes.push_front(t);
                  hyper[t] = true;
                  relabel_enabled = false;
                  break;
                } else {
                  (*_flow)[e] -= delta;
                  _excess[n] -= delta;
                  _excess[t] += delta;
                  if (_excess[t] > 0 && _excess[t] <= delta)
                    active_nodes.push_back(t);
                }

                if (_excess[n] == 0) break;
              }
            }
            next_in[n] = e;
          }

          // Relabel the node if it is still active (or hyper)
          if (relabel_enabled && (_excess[n] > 0 || hyper[n])) {
            LCost min_red_cost = std::numeric_limits<LCost>::max() / 2;
            for (OutArcIt oe(_graph, n); oe != INVALID; ++oe) {
              if ( _capacity[oe] - (*_flow)[oe] > 0 &&
                   (*_red_cost)[oe] < min_red_cost )
                min_red_cost = (*_red_cost)[oe];
            }
            for (InArcIt ie(_graph, n); ie != INVALID; ++ie) {
              if ((*_flow)[ie] > 0 && -(*_red_cost)[ie] < min_red_cost)
                min_red_cost = -(*_red_cost)[ie];
            }
            (*_potential)[n] -= min_red_cost + _epsilon;
            hyper[n] = false;

            // Reset the next arc maps
            next_out[n] = OutArcIt(_graph, n);
            next_in[n] = InArcIt(_graph, n);
            next_dir[n] = true;
          }

          // Remove nodes that are not active nor hyper
          while ( active_nodes.size() > 0 &&
                  _excess[active_nodes[0]] <= 0 &&
                  !hyper[active_nodes[0]] ) {
            active_nodes.pop_front();
          }
        }
      }

      // Compute node potentials for the original costs
      ResidualCostMap<CostMap> res_cost(_orig_cost);
      BellmanFord< ResDigraph, ResidualCostMap<CostMap> >
        bf(*_res_graph, res_cost);
      bf.init(0); bf.start();
      for (NodeIt n(_graph); n != INVALID; ++n)
        (*_potential)[n] = bf.dist(n);

      // Handle non-zero lower bounds
      if (_lower) {
        for (ArcIt e(_graph); e != INVALID; ++e)
          (*_flow)[e] += (*_lower)[e];
      }
      return true;
    }

  }; //class CostScaling

  ///@}

} //namespace lemon

#endif //LEMON_COST_SCALING_H
