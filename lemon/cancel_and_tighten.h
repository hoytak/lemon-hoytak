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

#ifndef LEMON_CANCEL_AND_TIGHTEN_H
#define LEMON_CANCEL_AND_TIGHTEN_H

/// \ingroup min_cost_flow
///
/// \file
/// \brief Cancel and Tighten algorithm for finding a minimum cost flow.

#include <vector>

#include <lemon/circulation.h>
#include <lemon/bellman_ford.h>
#include <lemon/howard.h>
#include <lemon/adaptors.h>
#include <lemon/tolerance.h>
#include <lemon/math.h>

#include <lemon/static_graph.h>

namespace lemon {

  /// \addtogroup min_cost_flow
  /// @{

  /// \brief Implementation of the Cancel and Tighten algorithm for
  /// finding a minimum cost flow.
  ///
  /// \ref CancelAndTighten implements the Cancel and Tighten algorithm for
  /// finding a minimum cost flow.
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
  /// \author Peter Kovacs
  template < typename Digraph,
             typename LowerMap = typename Digraph::template ArcMap<int>,
             typename CapacityMap = typename Digraph::template ArcMap<int>,
             typename CostMap = typename Digraph::template ArcMap<int>,
             typename SupplyMap = typename Digraph::template NodeMap<int> >
  class CancelAndTighten
  {
    TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

    typedef typename CapacityMap::Value Capacity;
    typedef typename CostMap::Value Cost;
    typedef typename SupplyMap::Value Supply;
    typedef typename Digraph::template ArcMap<Capacity> CapacityArcMap;
    typedef typename Digraph::template NodeMap<Supply> SupplyNodeMap;

    typedef ResidualDigraph< const Digraph,
      CapacityArcMap, CapacityArcMap > ResDigraph;

  public:

    /// The type of the flow map.
    typedef typename Digraph::template ArcMap<Capacity> FlowMap;
    /// The type of the potential map.
    typedef typename Digraph::template NodeMap<Cost> PotentialMap;

  private:

    /// \brief Map adaptor class for handling residual arc costs.
    ///
    /// Map adaptor class for handling residual arc costs.
    class ResidualCostMap : public MapBase<typename ResDigraph::Arc, Cost>
    {
      typedef typename ResDigraph::Arc Arc;
      
    private:

      const CostMap &_cost_map;

    public:

      ///\e
      ResidualCostMap(const CostMap &cost_map) : _cost_map(cost_map) {}

      ///\e
      Cost operator[](const Arc &e) const {
        return ResDigraph::forward(e) ? _cost_map[e] : -_cost_map[e];
      }

    }; //class ResidualCostMap

    /// \brief Map adaptor class for handling reduced arc costs.
    ///
    /// Map adaptor class for handling reduced arc costs.
    class ReducedCostMap : public MapBase<Arc, Cost>
    {
    private:

      const Digraph &_gr;
      const CostMap &_cost_map;
      const PotentialMap &_pot_map;

    public:

      ///\e
      ReducedCostMap( const Digraph &gr,
                      const CostMap &cost_map,
                      const PotentialMap &pot_map ) :
        _gr(gr), _cost_map(cost_map), _pot_map(pot_map) {}

      ///\e
      inline Cost operator[](const Arc &e) const {
        return _cost_map[e] + _pot_map[_gr.source(e)]
                            - _pot_map[_gr.target(e)];
      }

    }; //class ReducedCostMap

    struct BFOperationTraits {
      static double zero() { return 0; }

      static double infinity() {
        return std::numeric_limits<double>::infinity();
      }

      static double plus(const double& left, const double& right) {
        return left + right;
      }

      static bool less(const double& left, const double& right) {
        return left + 1e-6 < right;
      }
    }; // class BFOperationTraits

  private:

    // The digraph the algorithm runs on
    const Digraph &_graph;
    // The original lower bound map
    const LowerMap *_lower;
    // The modified capacity map
    CapacityArcMap _capacity;
    // The original cost map
    const CostMap &_cost;
    // The modified supply map
    SupplyNodeMap _supply;
    bool _valid_supply;

    // Arc map of the current flow
    FlowMap *_flow;
    bool _local_flow;
    // Node map of the current potentials
    PotentialMap *_potential;
    bool _local_potential;

    // The residual digraph
    ResDigraph *_res_graph;
    // The residual cost map
    ResidualCostMap _res_cost;

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
    CancelAndTighten( const Digraph &digraph,
                      const LowerMap &lower,
                      const CapacityMap &capacity,
                      const CostMap &cost,
                      const SupplyMap &supply ) :
      _graph(digraph), _lower(&lower), _capacity(digraph), _cost(cost),
      _supply(digraph), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false),
      _res_graph(NULL), _res_cost(_cost)
    {
      // Check the sum of supply values
      Supply sum = 0;
      for (NodeIt n(_graph); n != INVALID; ++n) {
        _supply[n] = supply[n];
        sum += _supply[n];
      }
      _valid_supply = sum == 0;

      // Remove non-zero lower bounds
      for (ArcIt e(_graph); e != INVALID; ++e) {
        _capacity[e] = capacity[e];
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
    CancelAndTighten( const Digraph &digraph,
                      const CapacityMap &capacity,
                      const CostMap &cost,
                      const SupplyMap &supply ) :
      _graph(digraph), _lower(NULL), _capacity(capacity), _cost(cost),
      _supply(supply), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false),
      _res_graph(NULL), _res_cost(_cost)
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
    CancelAndTighten( const Digraph &digraph,
                      const LowerMap &lower,
                      const CapacityMap &capacity,
                      const CostMap &cost,
                      Node s, Node t,
                      Supply flow_value ) :
      _graph(digraph), _lower(&lower), _capacity(capacity), _cost(cost),
      _supply(digraph, 0), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false),
      _res_graph(NULL), _res_cost(_cost)
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
    CancelAndTighten( const Digraph &digraph,
                      const CapacityMap &capacity,
                      const CostMap &cost,
                      Node s, Node t,
                      Supply flow_value ) :
      _graph(digraph), _lower(NULL), _capacity(capacity), _cost(cost),
      _supply(digraph, 0), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false),
      _res_graph(NULL), _res_cost(_cost)
    {
      _supply[s] =  flow_value;
      _supply[t] = -flow_value;
      _valid_supply = true;
    }
*/
    /// Destructor.
    ~CancelAndTighten() {
      if (_local_flow) delete _flow;
      if (_local_potential) delete _potential;
      delete _res_graph;
    }

    /// \brief Set the flow map.
    ///
    /// Set the flow map.
    ///
    /// \return \c (*this)
    CancelAndTighten& flowMap(FlowMap &map) {
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
    CancelAndTighten& potentialMap(PotentialMap &map) {
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
    /// \return \c true if a feasible flow can be found.
    bool run() {
      return init() && start();
    }

    /// @}

    /// \name Query Functions
    /// The result of the algorithm can be obtained using these
    /// functions.\n
    /// \ref lemon::CancelAndTighten::run() "run()" must be called before
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
        c += (*_flow)[e] * _cost[e];
      return c;
    }

    /// @}

  private:

    /// Initialize the algorithm.
    bool init() {
      if (!_valid_supply) return false;

      // Initialize flow and potential maps
      if (!_flow) {
        _flow = new FlowMap(_graph);
        _local_flow = true;
      }
      if (!_potential) {
        _potential = new PotentialMap(_graph);
        _local_potential = true;
      }

      _res_graph = new ResDigraph(_graph, _capacity, *_flow);

      // Find a feasible flow using Circulation
      Circulation< Digraph, ConstMap<Arc, Capacity>,
                   CapacityArcMap, SupplyMap >
        circulation( _graph, constMap<Arc>(Capacity(0)),
                     _capacity, _supply );
      return circulation.flowMap(*_flow).run();
    }

    bool start() {
      const double LIMIT_FACTOR = 0.01;
      const int MIN_LIMIT = 3;

      typedef typename Digraph::template NodeMap<double> FloatPotentialMap;
      typedef typename Digraph::template NodeMap<int> LevelMap;
      typedef typename Digraph::template NodeMap<bool> BoolNodeMap;
      typedef typename Digraph::template NodeMap<Node> PredNodeMap;
      typedef typename Digraph::template NodeMap<Arc> PredArcMap;
      typedef typename ResDigraph::template ArcMap<double> ResShiftCostMap;
      FloatPotentialMap pi(_graph);
      LevelMap level(_graph);
      BoolNodeMap reached(_graph);
      BoolNodeMap processed(_graph);
      PredNodeMap pred_node(_graph);
      PredArcMap pred_arc(_graph);
      int node_num = countNodes(_graph);
      typedef std::pair<Arc, bool> pair;
      std::vector<pair> stack(node_num);
      std::vector<Node> proc_vector(node_num);
      ResShiftCostMap shift_cost(*_res_graph);

      Tolerance<double> tol;
      tol.epsilon(1e-6);

      Timer t1, t2, t3;
      t1.reset();
      t2.reset();
      t3.reset();

      // Initialize epsilon and the node potentials
      double epsilon = 0;
      for (ArcIt e(_graph); e != INVALID; ++e) {
        if (_capacity[e] - (*_flow)[e] > 0 && _cost[e] < -epsilon)
          epsilon = -_cost[e];
        else if ((*_flow)[e] > 0 && _cost[e] > epsilon)
          epsilon = _cost[e];
      }
      for (NodeIt v(_graph); v != INVALID; ++v) {
        pi[v] = 0;
      }

      // Start phases
      int limit = int(LIMIT_FACTOR * node_num);
      if (limit < MIN_LIMIT) limit = MIN_LIMIT;
      int iter = limit;
      while (epsilon * node_num >= 1) {
        t1.start();
        // Find and cancel cycles in the admissible digraph using DFS
        for (NodeIt n(_graph); n != INVALID; ++n) {
          reached[n] = false;
          processed[n] = false;
        }
        int stack_head = -1;
        int proc_head = -1;

        for (NodeIt start(_graph); start != INVALID; ++start) {
          if (reached[start]) continue;

          // New start node
          reached[start] = true;
          pred_arc[start] = INVALID;
          pred_node[start] = INVALID;

          // Find the first admissible residual outgoing arc
          double p = pi[start];
          Arc e;
          _graph.firstOut(e, start);
          while ( e != INVALID && (_capacity[e] - (*_flow)[e] == 0 ||
                  !tol.negative(_cost[e] + p - pi[_graph.target(e)])) )
            _graph.nextOut(e);
          if (e != INVALID) {
            stack[++stack_head] = pair(e, true);
            goto next_step_1;
          }
          _graph.firstIn(e, start);
          while ( e != INVALID && ((*_flow)[e] == 0 ||
                  !tol.negative(-_cost[e] + p - pi[_graph.source(e)])) )
            _graph.nextIn(e);
          if (e != INVALID) {
            stack[++stack_head] = pair(e, false);
            goto next_step_1;
          }
          processed[start] = true;
          proc_vector[++proc_head] = start;
          continue;
        next_step_1:

          while (stack_head >= 0) {
            Arc se = stack[stack_head].first;
            bool sf = stack[stack_head].second;
            Node u, v;
            if (sf) {
              u = _graph.source(se);
              v = _graph.target(se);
            } else {
              u = _graph.target(se);
              v = _graph.source(se);
            }

            if (!reached[v]) {
              // A new node is reached
              reached[v] = true;
              pred_node[v] = u;
              pred_arc[v] = se;
              // Find the first admissible residual outgoing arc
              double p = pi[v];
              Arc e;
              _graph.firstOut(e, v);
              while ( e != INVALID && (_capacity[e] - (*_flow)[e] == 0 ||
                      !tol.negative(_cost[e] + p - pi[_graph.target(e)])) )
                _graph.nextOut(e);
              if (e != INVALID) {
                stack[++stack_head] = pair(e, true);
                goto next_step_2;
              }
              _graph.firstIn(e, v);
              while ( e != INVALID && ((*_flow)[e] == 0 ||
                      !tol.negative(-_cost[e] + p - pi[_graph.source(e)])) )
                _graph.nextIn(e);
              stack[++stack_head] = pair(e, false);
            next_step_2: ;
            } else {
              if (!processed[v]) {
                // A cycle is found
                Node n, w = u;
                Capacity d, delta = sf ? _capacity[se] - (*_flow)[se] :
                                         (*_flow)[se];
                for (n = u; n != v; n = pred_node[n]) {
                  d = _graph.target(pred_arc[n]) == n ?
                      _capacity[pred_arc[n]] - (*_flow)[pred_arc[n]] :
                      (*_flow)[pred_arc[n]];
                  if (d <= delta) {
                    delta = d;
                    w = pred_node[n];
                  }
                }

/*
                std::cout << "CYCLE FOUND: ";
                if (sf)
                  std::cout << _cost[se] + pi[_graph.source(se)] - pi[_graph.target(se)];
                else
                  std::cout << _graph.id(se) << ":" << -(_cost[se] + pi[_graph.source(se)] - pi[_graph.target(se)]);
                for (n = u; n != v; n = pred_node[n]) {
                  if (_graph.target(pred_arc[n]) == n)
                    std::cout << " " << _cost[pred_arc[n]] + pi[_graph.source(pred_arc[n])] - pi[_graph.target(pred_arc[n])];
                  else
                    std::cout << " " << -(_cost[pred_arc[n]] + pi[_graph.source(pred_arc[n])] - pi[_graph.target(pred_arc[n])]);
                }
                std::cout << "\n";
*/
                // Augment along the cycle
                (*_flow)[se] = sf ? (*_flow)[se] + delta :
                                    (*_flow)[se] - delta;
                for (n = u; n != v; n = pred_node[n]) {
                  if (_graph.target(pred_arc[n]) == n)
                    (*_flow)[pred_arc[n]] += delta;
                  else
                    (*_flow)[pred_arc[n]] -= delta;
                }
                for (n = u; stack_head > 0 && n != w; n = pred_node[n]) {
                  --stack_head;
                  reached[n] = false;
                }
                u = w;
              }
              v = u;

              // Find the next admissible residual outgoing arc
              double p = pi[v];
              Arc e = stack[stack_head].first;
              if (!stack[stack_head].second) {
                _graph.nextIn(e);
                goto in_arc_3;
              }
              _graph.nextOut(e);
              while ( e != INVALID && (_capacity[e] - (*_flow)[e] == 0 ||
                      !tol.negative(_cost[e] + p - pi[_graph.target(e)])) )
                _graph.nextOut(e);
              if (e != INVALID) {
                stack[stack_head] = pair(e, true);
                goto next_step_3;
              }
              _graph.firstIn(e, v);
            in_arc_3:
              while ( e != INVALID && ((*_flow)[e] == 0 ||
                      !tol.negative(-_cost[e] + p - pi[_graph.source(e)])) )
                _graph.nextIn(e);
              stack[stack_head] = pair(e, false);
            next_step_3: ;
            }

            while (stack_head >= 0 && stack[stack_head].first == INVALID) {
              processed[v] = true;
              proc_vector[++proc_head] = v;
              if (--stack_head >= 0) {
                v = stack[stack_head].second ?
                    _graph.source(stack[stack_head].first) :
                    _graph.target(stack[stack_head].first);
                // Find the next admissible residual outgoing arc
                double p = pi[v];
                Arc e = stack[stack_head].first;
                if (!stack[stack_head].second) {
                  _graph.nextIn(e);
                  goto in_arc_4;
                }
                _graph.nextOut(e);
                while ( e != INVALID && (_capacity[e] - (*_flow)[e] == 0 ||
                        !tol.negative(_cost[e] + p - pi[_graph.target(e)])) )
                  _graph.nextOut(e);
                if (e != INVALID) {
                  stack[stack_head] = pair(e, true);
                  goto next_step_4;
                }
                _graph.firstIn(e, v);
              in_arc_4:
                while ( e != INVALID && ((*_flow)[e] == 0 ||
                        !tol.negative(-_cost[e] + p - pi[_graph.source(e)])) )
                  _graph.nextIn(e);
                stack[stack_head] = pair(e, false);
              next_step_4: ;
              }
            }
          }
        }
        t1.stop();

        // Tighten potentials and epsilon
        if (--iter > 0) {
          // Compute levels
          t2.start();
          for (int i = proc_head; i >= 0; --i) {
            Node v = proc_vector[i];
            double p = pi[v];
            int l = 0;
            for (InArcIt e(_graph, v); e != INVALID; ++e) {
              Node u = _graph.source(e);
              if ( _capacity[e] - (*_flow)[e] > 0 &&
                   tol.negative(_cost[e] + pi[u] - p) &&
                   level[u] + 1 > l ) l = level[u] + 1;
            }
            for (OutArcIt e(_graph, v); e != INVALID; ++e) {
              Node u = _graph.target(e);
              if ( (*_flow)[e] > 0 &&
                   tol.negative(-_cost[e] + pi[u] - p) &&
                   level[u] + 1 > l ) l = level[u] + 1;
            }
            level[v] = l;
          }

          // Modify potentials
          double p, q = -1;
          for (ArcIt e(_graph); e != INVALID; ++e) {
            Node u = _graph.source(e);
            Node v = _graph.target(e);
            if (_capacity[e] - (*_flow)[e] > 0 && level[u] - level[v] > 0) {
              p = (_cost[e] + pi[u] - pi[v] + epsilon) /
                  (level[u] - level[v] + 1);
              if (q < 0 || p < q) q = p;
            }
            else if ((*_flow)[e] > 0 && level[v] - level[u] > 0) {
              p = (-_cost[e] - pi[u] + pi[v] + epsilon) /
                  (level[v] - level[u] + 1);
              if (q < 0 || p < q) q = p;
            }
          }
          for (NodeIt v(_graph); v != INVALID; ++v) {
            pi[v] -= q * level[v];
          }

          // Modify epsilon
          epsilon = 0;
          for (ArcIt e(_graph); e != INVALID; ++e) {
            double curr = _cost[e] + pi[_graph.source(e)]
                                   - pi[_graph.target(e)];
            if (_capacity[e] - (*_flow)[e] > 0 && curr < -epsilon)
              epsilon = -curr;
            else if ((*_flow)[e] > 0 && curr > epsilon)
              epsilon = curr;
          }
          t2.stop();
        } else {
          // Set epsilon to the minimum cycle mean
          t3.start();

/**/
          StaticDigraph static_graph;
          typename ResDigraph::template NodeMap<typename StaticDigraph::Node> node_ref(*_res_graph);
          typename ResDigraph::template ArcMap<typename StaticDigraph::Arc> arc_ref(*_res_graph);
          static_graph.build(*_res_graph, node_ref, arc_ref);
          typename StaticDigraph::template NodeMap<double> static_pi(static_graph);
          typename StaticDigraph::template ArcMap<double> static_cost(static_graph);

          for (typename ResDigraph::ArcIt e(*_res_graph); e != INVALID; ++e)
            static_cost[arc_ref[e]] = _res_cost[e];

          Howard<StaticDigraph, typename StaticDigraph::template ArcMap<double> >
            mmc(static_graph, static_cost);
          mmc.findMinMean();
          epsilon = -mmc.cycleMean();
/**/

/*
          Howard<ResDigraph, ResidualCostMap> mmc(*_res_graph, _res_cost);
          mmc.findMinMean();
          epsilon = -mmc.cycleMean();
*/

          // Compute feasible potentials for the current epsilon
          for (typename StaticDigraph::ArcIt e(static_graph); e != INVALID; ++e)
            static_cost[e] += epsilon;
          typename BellmanFord<StaticDigraph, typename StaticDigraph::template ArcMap<double> >::
            template SetDistMap<typename StaticDigraph::template NodeMap<double> >::
            template SetOperationTraits<BFOperationTraits>::Create
              bf(static_graph, static_cost);
          bf.distMap(static_pi).init(0);
          bf.start();
          for (NodeIt n(_graph); n != INVALID; ++n)
            pi[n] = static_pi[node_ref[n]];
          
/*
          for (typename ResDigraph::ArcIt e(*_res_graph); e != INVALID; ++e)
            shift_cost[e] = _res_cost[e] + epsilon;
          typename BellmanFord<ResDigraph, ResShiftCostMap>::
            template SetDistMap<FloatPotentialMap>::
            template SetOperationTraits<BFOperationTraits>::Create
              bf(*_res_graph, shift_cost);
          bf.distMap(pi).init(0);
          bf.start();
*/

          iter = limit;
          t3.stop();
        }
      }

//      std::cout << t1.realTime() << " " << t2.realTime() << " " << t3.realTime() << "\n";

      // Handle non-zero lower bounds
      if (_lower) {
        for (ArcIt e(_graph); e != INVALID; ++e)
          (*_flow)[e] += (*_lower)[e];
      }
      return true;
    }

  }; //class CancelAndTighten

  ///@}

} //namespace lemon

#endif //LEMON_CANCEL_AND_TIGHTEN_H
