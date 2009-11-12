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

#ifndef LEMON_CYCLE_CANCELING_H
#define LEMON_CYCLE_CANCELING_H

/// \ingroup min_cost_flow
///
/// \file
/// \brief Cycle-canceling algorithm for finding a minimum cost flow.

#include <vector>
#include <lemon/adaptors.h>
#include <lemon/path.h>

#include <lemon/circulation.h>
#include <lemon/bellman_ford.h>
#include <lemon/howard.h>

namespace lemon {

  /// \addtogroup min_cost_flow
  /// @{

  /// \brief Implementation of a cycle-canceling algorithm for
  /// finding a minimum cost flow.
  ///
  /// \ref CycleCanceling implements a cycle-canceling algorithm for
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
  /// \note By default the \ref BellmanFord "Bellman-Ford" algorithm is
  /// used for negative cycle detection with limited iteration number.
  /// However \ref CycleCanceling also provides the "Minimum Mean
  /// Cycle-Canceling" algorithm, which is \e strongly \e polynomial,
  /// but rather slower in practice.
  /// To use this version of the algorithm, call \ref run() with \c true
  /// parameter.
  ///
  /// \author Peter Kovacs
  template < typename Digraph,
             typename LowerMap = typename Digraph::template ArcMap<int>,
             typename CapacityMap = typename Digraph::template ArcMap<int>,
             typename CostMap = typename Digraph::template ArcMap<int>,
             typename SupplyMap = typename Digraph::template NodeMap<int> >
  class CycleCanceling
  {
    TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

    typedef typename CapacityMap::Value Capacity;
    typedef typename CostMap::Value Cost;
    typedef typename SupplyMap::Value Supply;
    typedef typename Digraph::template ArcMap<Capacity> CapacityArcMap;
    typedef typename Digraph::template NodeMap<Supply> SupplyNodeMap;

    typedef ResidualDigraph< const Digraph,
      CapacityArcMap, CapacityArcMap > ResDigraph;
    typedef typename ResDigraph::Node ResNode;
    typedef typename ResDigraph::NodeIt ResNodeIt;
    typedef typename ResDigraph::Arc ResArc;
    typedef typename ResDigraph::ArcIt ResArcIt;

  public:

    /// The type of the flow map.
    typedef typename Digraph::template ArcMap<Capacity> FlowMap;
    /// The type of the potential map.
    typedef typename Digraph::template NodeMap<Cost> PotentialMap;

  private:

    /// \brief Map adaptor class for handling residual arc costs.
    ///
    /// Map adaptor class for handling residual arc costs.
    class ResidualCostMap : public MapBase<ResArc, Cost>
    {
    private:

      const CostMap &_cost_map;

    public:

      ///\e
      ResidualCostMap(const CostMap &cost_map) : _cost_map(cost_map) {}

      ///\e
      Cost operator[](const ResArc &e) const {
        return ResDigraph::forward(e) ? _cost_map[e] : -_cost_map[e];
      }

    }; //class ResidualCostMap

  private:

    // The maximum number of iterations for the first execution of the
    // Bellman-Ford algorithm. It should be at least 2.
    static const int BF_FIRST_LIMIT  = 2;
    // The iteration limit for the Bellman-Ford algorithm is multiplied
    // by BF_LIMIT_FACTOR/100 in every round.
    static const int BF_LIMIT_FACTOR = 150;

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
    CycleCanceling( const Digraph &digraph,
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
    CycleCanceling( const Digraph &digraph,
                    const CapacityMap &capacity,
                    const CostMap &cost,
                    const SupplyMap &supply ) :
      _graph(digraph), _lower(NULL), _capacity(capacity), _cost(cost),
      _supply(supply), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false), _res_graph(NULL),
      _res_cost(_cost)
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
    CycleCanceling( const Digraph &digraph,
                    const LowerMap &lower,
                    const CapacityMap &capacity,
                    const CostMap &cost,
                    Node s, Node t,
                    Supply flow_value ) :
      _graph(digraph), _lower(&lower), _capacity(capacity), _cost(cost),
      _supply(digraph, 0), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false), _res_graph(NULL),
      _res_cost(_cost)
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
    CycleCanceling( const Digraph &digraph,
                    const CapacityMap &capacity,
                    const CostMap &cost,
                    Node s, Node t,
                    Supply flow_value ) :
      _graph(digraph), _lower(NULL), _capacity(capacity), _cost(cost),
      _supply(digraph, 0), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false), _res_graph(NULL),
      _res_cost(_cost)
    {
      _supply[s] =  flow_value;
      _supply[t] = -flow_value;
      _valid_supply = true;
    }
*/
    /// Destructor.
    ~CycleCanceling() {
      if (_local_flow) delete _flow;
      if (_local_potential) delete _potential;
      delete _res_graph;
    }

    /// \brief Set the flow map.
    ///
    /// Set the flow map.
    ///
    /// \return \c (*this)
    CycleCanceling& flowMap(FlowMap &map) {
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
    CycleCanceling& potentialMap(PotentialMap &map) {
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
    /// \param min_mean_cc Set this parameter to \c true to run the
    /// "Minimum Mean Cycle-Canceling" algorithm, which is strongly
    /// polynomial, but rather slower in practice.
    ///
    /// \return \c true if a feasible flow can be found.
    bool run(bool min_mean_cc = false) {
      return init() && start(min_mean_cc);
    }

    /// @}

    /// \name Query Functions
    /// The result of the algorithm can be obtained using these
    /// functions.\n
    /// \ref lemon::CycleCanceling::run() "run()" must be called before
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

      // Initializing flow and potential maps
      if (!_flow) {
        _flow = new FlowMap(_graph);
        _local_flow = true;
      }
      if (!_potential) {
        _potential = new PotentialMap(_graph);
        _local_potential = true;
      }

      _res_graph = new ResDigraph(_graph, _capacity, *_flow);

      // Finding a feasible flow using Circulation
      Circulation< Digraph, ConstMap<Arc, Capacity>, CapacityArcMap,
                   SupplyMap >
        circulation( _graph, constMap<Arc>(Capacity(0)), _capacity,
                     _supply );
      return circulation.flowMap(*_flow).run();
    }

    bool start(bool min_mean_cc) {
      if (min_mean_cc)
        startMinMean();
      else
        start();

      // Handling non-zero lower bounds
      if (_lower) {
        for (ArcIt e(_graph); e != INVALID; ++e)
          (*_flow)[e] += (*_lower)[e];
      }
      return true;
    }

    /// \brief Execute the algorithm using \ref BellmanFord.
    ///
    /// Execute the algorithm using the \ref BellmanFord
    /// "Bellman-Ford" algorithm for negative cycle detection with
    /// successively larger limit for the number of iterations.
    void start() {
      typename BellmanFord<ResDigraph, ResidualCostMap>::PredMap pred(*_res_graph);
      typename ResDigraph::template NodeMap<int> visited(*_res_graph);
      std::vector<ResArc> cycle;
      int node_num = countNodes(_graph);

      int length_bound = BF_FIRST_LIMIT;
      bool optimal = false;
      while (!optimal) {
        BellmanFord<ResDigraph, ResidualCostMap> bf(*_res_graph, _res_cost);
        bf.predMap(pred);
        bf.init(0);
        int iter_num = 0;
        bool cycle_found = false;
        while (!cycle_found) {
          int curr_iter_num = iter_num + length_bound <= node_num ?
                              length_bound : node_num - iter_num;
          iter_num += curr_iter_num;
          int real_iter_num = curr_iter_num;
          for (int i = 0; i < curr_iter_num; ++i) {
            if (bf.processNextWeakRound()) {
              real_iter_num = i;
              break;
            }
          }
          if (real_iter_num < curr_iter_num) {
            // Optimal flow is found
            optimal = true;
            // Setting node potentials
            for (NodeIt n(_graph); n != INVALID; ++n)
              (*_potential)[n] = bf.dist(n);
            break;
          } else {
            // Searching for node disjoint negative cycles
            for (ResNodeIt n(*_res_graph); n != INVALID; ++n)
              visited[n] = 0;
            int id = 0;
            for (ResNodeIt n(*_res_graph); n != INVALID; ++n) {
              if (visited[n] > 0) continue;
              visited[n] = ++id;
              ResNode u = pred[n] == INVALID ?
                          INVALID : _res_graph->source(pred[n]);
              while (u != INVALID && visited[u] == 0) {
                visited[u] = id;
                u = pred[u] == INVALID ?
                    INVALID : _res_graph->source(pred[u]);
              }
              if (u != INVALID && visited[u] == id) {
                // Finding the negative cycle
                cycle_found = true;
                cycle.clear();
                ResArc e = pred[u];
                cycle.push_back(e);
                Capacity d = _res_graph->residualCapacity(e);
                while (_res_graph->source(e) != u) {
                  cycle.push_back(e = pred[_res_graph->source(e)]);
                  if (_res_graph->residualCapacity(e) < d)
                    d = _res_graph->residualCapacity(e);
                }

                // Augmenting along the cycle
                for (int i = 0; i < int(cycle.size()); ++i)
                  _res_graph->augment(cycle[i], d);
              }
            }
          }

          if (!cycle_found)
            length_bound = length_bound * BF_LIMIT_FACTOR / 100;
        }
      }
    }

    /// \brief Execute the algorithm using \ref Howard.
    ///
    /// Execute the algorithm using \ref Howard for negative
    /// cycle detection.
    void startMinMean() {
      typedef Path<ResDigraph> ResPath;
      Howard<ResDigraph, ResidualCostMap> mmc(*_res_graph, _res_cost);
      ResPath cycle;

      mmc.cycle(cycle);
      if (mmc.findMinMean()) {
        while (mmc.cycleLength() < 0) {
          // Finding the cycle
          mmc.findCycle();

          // Finding the largest flow amount that can be augmented
          // along the cycle
          Capacity delta = 0;
          for (typename ResPath::ArcIt e(cycle); e != INVALID; ++e) {
            if (delta == 0 || _res_graph->residualCapacity(e) < delta)
              delta = _res_graph->residualCapacity(e);
          }

          // Augmenting along the cycle
          for (typename ResPath::ArcIt e(cycle); e != INVALID; ++e)
            _res_graph->augment(e, delta);

          // Finding the minimum cycle mean for the modified residual
          // digraph
          if (!mmc.findMinMean()) break;
        }
      }

      // Computing node potentials
      BellmanFord<ResDigraph, ResidualCostMap> bf(*_res_graph, _res_cost);
      bf.init(0); bf.start();
      for (NodeIt n(_graph); n != INVALID; ++n)
        (*_potential)[n] = bf.dist(n);
    }

  }; //class CycleCanceling

  ///@}

} //namespace lemon

#endif //LEMON_CYCLE_CANCELING_H
