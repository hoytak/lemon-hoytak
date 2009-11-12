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

#ifndef LEMON_CAPACITY_SCALING_H
#define LEMON_CAPACITY_SCALING_H

/// \ingroup min_cost_flow
///
/// \file
/// \brief Capacity scaling algorithm for finding a minimum cost flow.

#include <vector>
#include <lemon/bin_heap.h>

namespace lemon {

  /// \addtogroup min_cost_flow
  /// @{

  /// \brief Implementation of the capacity scaling algorithm for
  /// finding a minimum cost flow.
  ///
  /// \ref CapacityScaling implements the capacity scaling version
  /// of the successive shortest path algorithm for finding a minimum
  /// cost flow.
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
  class CapacityScaling
  {
    TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

    typedef typename CapacityMap::Value Capacity;
    typedef typename CostMap::Value Cost;
    typedef typename SupplyMap::Value Supply;
    typedef typename Digraph::template ArcMap<Capacity> CapacityArcMap;
    typedef typename Digraph::template NodeMap<Supply> SupplyNodeMap;
    typedef typename Digraph::template NodeMap<Arc> PredMap;

  public:

    /// The type of the flow map.
    typedef typename Digraph::template ArcMap<Capacity> FlowMap;
    /// The type of the potential map.
    typedef typename Digraph::template NodeMap<Cost> PotentialMap;

  private:

    /// \brief Special implementation of the \ref Dijkstra algorithm
    /// for finding shortest paths in the residual network.
    ///
    /// \ref ResidualDijkstra is a special implementation of the
    /// \ref Dijkstra algorithm for finding shortest paths in the
    /// residual network of the digraph with respect to the reduced arc
    /// costs and modifying the node potentials according to the
    /// distance of the nodes.
    class ResidualDijkstra
    {
      typedef typename Digraph::template NodeMap<int> HeapCrossRef;
      typedef BinHeap<Cost, HeapCrossRef> Heap;

    private:

      // The digraph the algorithm runs on
      const Digraph &_graph;

      // The main maps
      const FlowMap &_flow;
      const CapacityArcMap &_res_cap;
      const CostMap &_cost;
      const SupplyNodeMap &_excess;
      PotentialMap &_potential;

      // The distance map
      PotentialMap _dist;
      // The pred arc map
      PredMap &_pred;
      // The processed (i.e. permanently labeled) nodes
      std::vector<Node> _proc_nodes;

    public:

      /// Constructor.
      ResidualDijkstra( const Digraph &digraph,
                        const FlowMap &flow,
                        const CapacityArcMap &res_cap,
                        const CostMap &cost,
                        const SupplyMap &excess,
                        PotentialMap &potential,
                        PredMap &pred ) :
        _graph(digraph), _flow(flow), _res_cap(res_cap), _cost(cost),
        _excess(excess), _potential(potential), _dist(digraph),
        _pred(pred)
      {}

      /// Run the algorithm from the given source node.
      Node run(Node s, Capacity delta = 1) {
        HeapCrossRef heap_cross_ref(_graph, Heap::PRE_HEAP);
        Heap heap(heap_cross_ref);
        heap.push(s, 0);
        _pred[s] = INVALID;
        _proc_nodes.clear();

        // Processing nodes
        while (!heap.empty() && _excess[heap.top()] > -delta) {
          Node u = heap.top(), v;
          Cost d = heap.prio() + _potential[u], nd;
          _dist[u] = heap.prio();
          heap.pop();
          _proc_nodes.push_back(u);

          // Traversing outgoing arcs
          for (OutArcIt e(_graph, u); e != INVALID; ++e) {
            if (_res_cap[e] >= delta) {
              v = _graph.target(e);
              switch(heap.state(v)) {
              case Heap::PRE_HEAP:
                heap.push(v, d + _cost[e] - _potential[v]);
                _pred[v] = e;
                break;
              case Heap::IN_HEAP:
                nd = d + _cost[e] - _potential[v];
                if (nd < heap[v]) {
                  heap.decrease(v, nd);
                  _pred[v] = e;
                }
                break;
              case Heap::POST_HEAP:
                break;
              }
            }
          }

          // Traversing incoming arcs
          for (InArcIt e(_graph, u); e != INVALID; ++e) {
            if (_flow[e] >= delta) {
              v = _graph.source(e);
              switch(heap.state(v)) {
              case Heap::PRE_HEAP:
                heap.push(v, d - _cost[e] - _potential[v]);
                _pred[v] = e;
                break;
              case Heap::IN_HEAP:
                nd = d - _cost[e] - _potential[v];
                if (nd < heap[v]) {
                  heap.decrease(v, nd);
                  _pred[v] = e;
                }
                break;
              case Heap::POST_HEAP:
                break;
              }
            }
          }
        }
        if (heap.empty()) return INVALID;

        // Updating potentials of processed nodes
        Node t = heap.top();
        Cost t_dist = heap.prio();
        for (int i = 0; i < int(_proc_nodes.size()); ++i)
          _potential[_proc_nodes[i]] += _dist[_proc_nodes[i]] - t_dist;

        return t;
      }

    }; //class ResidualDijkstra

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

    // The residual capacity map
    CapacityArcMap _res_cap;
    // The excess map
    SupplyNodeMap _excess;
    // The excess nodes (i.e. nodes with positive excess)
    std::vector<Node> _excess_nodes;
    // The deficit nodes (i.e. nodes with negative excess)
    std::vector<Node> _deficit_nodes;

    // The delta parameter used for capacity scaling
    Capacity _delta;
    // The maximum number of phases
    int _phase_num;

    // The pred arc map
    PredMap _pred;
    // Implementation of the Dijkstra algorithm for finding augmenting
    // shortest paths in the residual network
    ResidualDijkstra *_dijkstra;

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
    CapacityScaling( const Digraph &digraph,
                     const LowerMap &lower,
                     const CapacityMap &capacity,
                     const CostMap &cost,
                     const SupplyMap &supply ) :
      _graph(digraph), _lower(&lower), _capacity(digraph), _cost(cost),
      _supply(digraph), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false),
      _res_cap(digraph), _excess(digraph), _pred(digraph), _dijkstra(NULL)
    {
      Supply sum = 0;
      for (NodeIt n(_graph); n != INVALID; ++n) {
        _supply[n] = supply[n];
        _excess[n] = supply[n];
        sum += supply[n];
      }
      _valid_supply = sum == 0;
      for (ArcIt a(_graph); a != INVALID; ++a) {
        _capacity[a] = capacity[a];
        _res_cap[a] = capacity[a];
      }

      // Remove non-zero lower bounds
      typename LowerMap::Value lcap;
      for (ArcIt e(_graph); e != INVALID; ++e) {
        if ((lcap = lower[e]) != 0) {
          _capacity[e] -= lcap;
          _res_cap[e] -= lcap;
          _supply[_graph.source(e)] -= lcap;
          _supply[_graph.target(e)] += lcap;
          _excess[_graph.source(e)] -= lcap;
          _excess[_graph.target(e)] += lcap;
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
    CapacityScaling( const Digraph &digraph,
                     const CapacityMap &capacity,
                     const CostMap &cost,
                     const SupplyMap &supply ) :
      _graph(digraph), _lower(NULL), _capacity(capacity), _cost(cost),
      _supply(supply), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false),
      _res_cap(capacity), _excess(supply), _pred(digraph), _dijkstra(NULL)
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
    CapacityScaling( const Digraph &digraph,
                     const LowerMap &lower,
                     const CapacityMap &capacity,
                     const CostMap &cost,
                     Node s, Node t,
                     Supply flow_value ) :
      _graph(digraph), _lower(&lower), _capacity(capacity), _cost(cost),
      _supply(digraph, 0), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false),
      _res_cap(capacity), _excess(digraph, 0), _pred(digraph), _dijkstra(NULL)
    {
      // Remove non-zero lower bounds
      _supply[s] = _excess[s] =  flow_value;
      _supply[t] = _excess[t] = -flow_value;
      typename LowerMap::Value lcap;
      for (ArcIt e(_graph); e != INVALID; ++e) {
        if ((lcap = lower[e]) != 0) {
          _capacity[e] -= lcap;
          _res_cap[e] -= lcap;
          _supply[_graph.source(e)] -= lcap;
          _supply[_graph.target(e)] += lcap;
          _excess[_graph.source(e)] -= lcap;
          _excess[_graph.target(e)] += lcap;
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
    CapacityScaling( const Digraph &digraph,
                     const CapacityMap &capacity,
                     const CostMap &cost,
                     Node s, Node t,
                     Supply flow_value ) :
      _graph(digraph), _lower(NULL), _capacity(capacity), _cost(cost),
      _supply(digraph, 0), _flow(NULL), _local_flow(false),
      _potential(NULL), _local_potential(false),
      _res_cap(capacity), _excess(digraph, 0), _pred(digraph), _dijkstra(NULL)
    {
      _supply[s] = _excess[s] =  flow_value;
      _supply[t] = _excess[t] = -flow_value;
      _valid_supply = true;
    }
*/
    /// Destructor.
    ~CapacityScaling() {
      if (_local_flow) delete _flow;
      if (_local_potential) delete _potential;
      delete _dijkstra;
    }

    /// \brief Set the flow map.
    ///
    /// Set the flow map.
    ///
    /// \return \c (*this)
    CapacityScaling& flowMap(FlowMap &map) {
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
    CapacityScaling& potentialMap(PotentialMap &map) {
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
    /// This function runs the algorithm.
    ///
    /// \param scaling Enable or disable capacity scaling.
    /// If the maximum arc capacity and/or the amount of total supply
    /// is rather small, the algorithm could be slightly faster without
    /// scaling.
    ///
    /// \return \c true if a feasible flow can be found.
    bool run(bool scaling = true) {
      return init(scaling) && start();
    }

    /// @}

    /// \name Query Functions
    /// The results of the algorithm can be obtained using these
    /// functions.\n
    /// \ref lemon::CapacityScaling::run() "run()" must be called before
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
    bool init(bool scaling) {
      if (!_valid_supply) return false;

      // Initializing maps
      if (!_flow) {
        _flow = new FlowMap(_graph);
        _local_flow = true;
      }
      if (!_potential) {
        _potential = new PotentialMap(_graph);
        _local_potential = true;
      }
      for (ArcIt e(_graph); e != INVALID; ++e) (*_flow)[e] = 0;
      for (NodeIt n(_graph); n != INVALID; ++n) (*_potential)[n] = 0;

      _dijkstra = new ResidualDijkstra( _graph, *_flow, _res_cap, _cost,
                                        _excess, *_potential, _pred );

      // Initializing delta value
      if (scaling) {
        // With scaling
        Supply max_sup = 0, max_dem = 0;
        for (NodeIt n(_graph); n != INVALID; ++n) {
          if ( _supply[n] > max_sup) max_sup =  _supply[n];
          if (-_supply[n] > max_dem) max_dem = -_supply[n];
        }
        Capacity max_cap = 0;
        for (ArcIt e(_graph); e != INVALID; ++e) {
          if (_capacity[e] > max_cap) max_cap = _capacity[e];
        }
        max_sup = std::min(std::min(max_sup, max_dem), max_cap);
        _phase_num = 0;
        for (_delta = 1; 2 * _delta <= max_sup; _delta *= 2)
          ++_phase_num;
      } else {
        // Without scaling
        _delta = 1;
      }

      return true;
    }

    bool start() {
      if (_delta > 1)
        return startWithScaling();
      else
        return startWithoutScaling();
    }

    /// Execute the capacity scaling algorithm.
    bool startWithScaling() {
      // Processing capacity scaling phases
      Node s, t;
      int phase_cnt = 0;
      int factor = 4;
      while (true) {
        // Saturating all arcs not satisfying the optimality condition
        for (ArcIt e(_graph); e != INVALID; ++e) {
          Node u = _graph.source(e), v = _graph.target(e);
          Cost c = _cost[e] + (*_potential)[u] - (*_potential)[v];
          if (c < 0 && _res_cap[e] >= _delta) {
            _excess[u] -= _res_cap[e];
            _excess[v] += _res_cap[e];
            (*_flow)[e] = _capacity[e];
            _res_cap[e] = 0;
          }
          else if (c > 0 && (*_flow)[e] >= _delta) {
            _excess[u] += (*_flow)[e];
            _excess[v] -= (*_flow)[e];
            (*_flow)[e] = 0;
            _res_cap[e] = _capacity[e];
          }
        }

        // Finding excess nodes and deficit nodes
        _excess_nodes.clear();
        _deficit_nodes.clear();
        for (NodeIt n(_graph); n != INVALID; ++n) {
          if (_excess[n] >=  _delta) _excess_nodes.push_back(n);
          if (_excess[n] <= -_delta) _deficit_nodes.push_back(n);
        }
        int next_node = 0, next_def_node = 0;

        // Finding augmenting shortest paths
        while (next_node < int(_excess_nodes.size())) {
          // Checking deficit nodes
          if (_delta > 1) {
            bool delta_deficit = false;
            for ( ; next_def_node < int(_deficit_nodes.size());
                    ++next_def_node ) {
              if (_excess[_deficit_nodes[next_def_node]] <= -_delta) {
                delta_deficit = true;
                break;
              }
            }
            if (!delta_deficit) break;
          }

          // Running Dijkstra
          s = _excess_nodes[next_node];
          if ((t = _dijkstra->run(s, _delta)) == INVALID) {
            if (_delta > 1) {
              ++next_node;
              continue;
            }
            return false;
          }

          // Augmenting along a shortest path from s to t.
          Capacity d = std::min(_excess[s], -_excess[t]);
          Node u = t;
          Arc e;
          if (d > _delta) {
            while ((e = _pred[u]) != INVALID) {
              Capacity rc;
              if (u == _graph.target(e)) {
                rc = _res_cap[e];
                u = _graph.source(e);
              } else {
                rc = (*_flow)[e];
                u = _graph.target(e);
              }
              if (rc < d) d = rc;
            }
          }
          u = t;
          while ((e = _pred[u]) != INVALID) {
            if (u == _graph.target(e)) {
              (*_flow)[e] += d;
              _res_cap[e] -= d;
              u = _graph.source(e);
            } else {
              (*_flow)[e] -= d;
              _res_cap[e] += d;
              u = _graph.target(e);
            }
          }
          _excess[s] -= d;
          _excess[t] += d;

          if (_excess[s] < _delta) ++next_node;
        }

        if (_delta == 1) break;
        if (++phase_cnt > _phase_num / 4) factor = 2;
        _delta = _delta <= factor ? 1 : _delta / factor;
      }

      // Handling non-zero lower bounds
      if (_lower) {
        for (ArcIt e(_graph); e != INVALID; ++e)
          (*_flow)[e] += (*_lower)[e];
      }
      return true;
    }

    /// Execute the successive shortest path algorithm.
    bool startWithoutScaling() {
      // Finding excess nodes
      for (NodeIt n(_graph); n != INVALID; ++n)
        if (_excess[n] > 0) _excess_nodes.push_back(n);
      if (_excess_nodes.size() == 0) return true;
      int next_node = 0;

      // Finding shortest paths
      Node s, t;
      while ( _excess[_excess_nodes[next_node]] > 0 ||
              ++next_node < int(_excess_nodes.size()) )
      {
        // Running Dijkstra
        s = _excess_nodes[next_node];
        if ((t = _dijkstra->run(s)) == INVALID) return false;

        // Augmenting along a shortest path from s to t
        Capacity d = std::min(_excess[s], -_excess[t]);
        Node u = t;
        Arc e;
        if (d > 1) {
          while ((e = _pred[u]) != INVALID) {
            Capacity rc;
            if (u == _graph.target(e)) {
              rc = _res_cap[e];
              u = _graph.source(e);
            } else {
              rc = (*_flow)[e];
              u = _graph.target(e);
            }
            if (rc < d) d = rc;
          }
        }
        u = t;
        while ((e = _pred[u]) != INVALID) {
          if (u == _graph.target(e)) {
            (*_flow)[e] += d;
            _res_cap[e] -= d;
            u = _graph.source(e);
          } else {
            (*_flow)[e] -= d;
            _res_cap[e] += d;
            u = _graph.target(e);
          }
        }
        _excess[s] -= d;
        _excess[t] += d;
      }

      // Handling non-zero lower bounds
      if (_lower) {
        for (ArcIt e(_graph); e != INVALID; ++e)
          (*_flow)[e] += (*_lower)[e];
      }
      return true;
    }

  }; //class CapacityScaling

  ///@}

} //namespace lemon

#endif //LEMON_CAPACITY_SCALING_H
