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

#ifndef LEMON_SUURBALLE_H
#define LEMON_SUURBALLE_H

///\ingroup shortest_path
///\file
///\brief An algorithm for finding arc-disjoint paths between two
/// nodes having minimum total length.

#include <vector>
#include <lemon/bin_heap.h>
#include <lemon/path.h>

namespace lemon {

  /// \addtogroup shortest_path
  /// @{

  /// \brief Algorithm for finding arc-disjoint paths between two nodes
  /// having minimum total length.
  ///
  /// \ref lemon::Suurballe "Suurballe" implements an algorithm for
  /// finding arc-disjoint paths having minimum total length (cost)
  /// from a given source node to a given target node in a digraph.
  ///
  /// In fact, this implementation is the specialization of the
  /// \ref CapacityScaling "successive shortest path" algorithm.
  ///
  /// \tparam Digraph The digraph type the algorithm runs on.
  /// The default value is \c ListDigraph.
  /// \tparam LengthMap The type of the length (cost) map.
  /// The default value is <tt>Digraph::ArcMap<int></tt>.
  ///
  /// \warning Length values should be \e non-negative \e integers.
  ///
  /// \note For finding node-disjoint paths this algorithm can be used
  /// with \ref SplitDigraphAdaptor.
#ifdef DOXYGEN
  template <typename Digraph, typename LengthMap>
#else
  template < typename Digraph = ListDigraph,
             typename LengthMap = typename Digraph::template ArcMap<int> >
#endif
  class Suurballe
  {
    TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

    typedef typename LengthMap::Value Length;
    typedef ConstMap<Arc, int> ConstArcMap;
    typedef typename Digraph::template NodeMap<Arc> PredMap;

  public:

    /// The type of the flow map.
    typedef typename Digraph::template ArcMap<int> FlowMap;
    /// The type of the potential map.
    typedef typename Digraph::template NodeMap<Length> PotentialMap;
    /// The type of the path structures.
    typedef SimplePath<Digraph> Path;

  private:
  
    /// \brief Special implementation of the Dijkstra algorithm
    /// for finding shortest paths in the residual network.
    ///
    /// \ref ResidualDijkstra is a special implementation of the
    /// \ref Dijkstra algorithm for finding shortest paths in the
    /// residual network of the digraph with respect to the reduced arc
    /// lengths and modifying the node potentials according to the
    /// distance of the nodes.
    class ResidualDijkstra
    {
      typedef typename Digraph::template NodeMap<int> HeapCrossRef;
      typedef BinHeap<Length, HeapCrossRef> Heap;

    private:

      // The digraph the algorithm runs on
      const Digraph &_graph;

      // The main maps
      const FlowMap &_flow;
      const LengthMap &_length;
      PotentialMap &_potential;

      // The distance map
      PotentialMap _dist;
      // The pred arc map
      PredMap &_pred;
      // The processed (i.e. permanently labeled) nodes
      std::vector<Node> _proc_nodes;
      
      Node _s;
      Node _t;

    public:

      /// Constructor.
      ResidualDijkstra( const Digraph &digraph,
                        const FlowMap &flow,
                        const LengthMap &length,
                        PotentialMap &potential,
                        PredMap &pred,
                        Node s, Node t ) :
        _graph(digraph), _flow(flow), _length(length), _potential(potential),
        _dist(digraph), _pred(pred), _s(s), _t(t) {}

      /// \brief Run the algorithm. It returns \c true if a path is found
      /// from the source node to the target node.
      bool run() {
        HeapCrossRef heap_cross_ref(_graph, Heap::PRE_HEAP);
        Heap heap(heap_cross_ref);
        heap.push(_s, 0);
        _pred[_s] = INVALID;
        _proc_nodes.clear();

        // Process nodes
        while (!heap.empty() && heap.top() != _t) {
          Node u = heap.top(), v;
          Length d = heap.prio() + _potential[u], nd;
          _dist[u] = heap.prio();
          heap.pop();
          _proc_nodes.push_back(u);

          // Traverse outgoing arcs
          for (OutArcIt e(_graph, u); e != INVALID; ++e) {
            if (_flow[e] == 0) {
              v = _graph.target(e);
              switch(heap.state(v)) {
              case Heap::PRE_HEAP:
                heap.push(v, d + _length[e] - _potential[v]);
                _pred[v] = e;
                break;
              case Heap::IN_HEAP:
                nd = d + _length[e] - _potential[v];
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

          // Traverse incoming arcs
          for (InArcIt e(_graph, u); e != INVALID; ++e) {
            if (_flow[e] == 1) {
              v = _graph.source(e);
              switch(heap.state(v)) {
              case Heap::PRE_HEAP:
                heap.push(v, d - _length[e] - _potential[v]);
                _pred[v] = e;
                break;
              case Heap::IN_HEAP:
                nd = d - _length[e] - _potential[v];
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
        if (heap.empty()) return false;

        // Update potentials of processed nodes
        Length t_dist = heap.prio();
        for (int i = 0; i < int(_proc_nodes.size()); ++i)
          _potential[_proc_nodes[i]] += _dist[_proc_nodes[i]] - t_dist;
        return true;
      }

    }; //class ResidualDijkstra

  private:

    // The digraph the algorithm runs on
    const Digraph &_graph;
    // The length map
    const LengthMap &_length;
    
    // Arc map of the current flow
    FlowMap *_flow;
    bool _local_flow;
    // Node map of the current potentials
    PotentialMap *_potential;
    bool _local_potential;

    // The source node
    Node _source;
    // The target node
    Node _target;

    // Container to store the found paths
    std::vector< SimplePath<Digraph> > paths;
    int _path_num;

    // The pred arc map
    PredMap _pred;
    // Implementation of the Dijkstra algorithm for finding augmenting
    // shortest paths in the residual network
    ResidualDijkstra *_dijkstra;

  public:

    /// \brief Constructor.
    ///
    /// Constructor.
    ///
    /// \param digraph The digraph the algorithm runs on.
    /// \param length The length (cost) values of the arcs.
    /// \param s The source node.
    /// \param t The target node.
    Suurballe( const Digraph &digraph,
               const LengthMap &length,
               Node s, Node t ) :
      _graph(digraph), _length(length), _flow(0), _local_flow(false),
      _potential(0), _local_potential(false), _source(s), _target(t),
      _pred(digraph) {}

    /// Destructor.
    ~Suurballe() {
      if (_local_flow) delete _flow;
      if (_local_potential) delete _potential;
      delete _dijkstra;
    }

    /// \brief Set the flow map.
    ///
    /// This function sets the flow map.
    ///
    /// The found flow contains only 0 and 1 values. It is the union of
    /// the found arc-disjoint paths.
    ///
    /// \return \c (*this)
    Suurballe& flowMap(FlowMap &map) {
      if (_local_flow) {
        delete _flow;
        _local_flow = false;
      }
      _flow = &map;
      return *this;
    }

    /// \brief Set the potential map.
    ///
    /// This function sets the potential map.
    ///
    /// The potentials provide the dual solution of the underlying 
    /// minimum cost flow problem.
    ///
    /// \return \c (*this)
    Suurballe& potentialMap(PotentialMap &map) {
      if (_local_potential) {
        delete _potential;
        _local_potential = false;
      }
      _potential = &map;
      return *this;
    }

    /// \name Execution control
    /// The simplest way to execute the algorithm is to call the run()
    /// function.
    /// \n
    /// If you only need the flow that is the union of the found
    /// arc-disjoint paths, you may call init() and findFlow().

    /// @{

    /// \brief Run the algorithm.
    ///
    /// This function runs the algorithm.
    ///
    /// \param k The number of paths to be found.
    ///
    /// \return \c k if there are at least \c k arc-disjoint paths from
    /// \c s to \c t in the digraph. Otherwise it returns the number of
    /// arc-disjoint paths found.
    ///
    /// \note Apart from the return value, <tt>s.run(k)</tt> is just a
    /// shortcut of the following code.
    /// \code
    ///   s.init();
    ///   s.findFlow(k);
    ///   s.findPaths();
    /// \endcode
    int run(int k = 2) {
      init();
      findFlow(k);
      findPaths();
      return _path_num;
    }

    /// \brief Initialize the algorithm.
    ///
    /// This function initializes the algorithm.
    void init() {
      // Initialize maps
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

      _dijkstra = new ResidualDijkstra( _graph, *_flow, _length, 
                                        *_potential, _pred,
                                        _source, _target );
    }

    /// \brief Execute the successive shortest path algorithm to find
    /// an optimal flow.
    ///
    /// This function executes the successive shortest path algorithm to
    /// find a minimum cost flow, which is the union of \c k or less
    /// arc-disjoint paths.
    ///
    /// \return \c k if there are at least \c k arc-disjoint paths from
    /// \c s to \c t in the digraph. Otherwise it returns the number of
    /// arc-disjoint paths found.
    ///
    /// \pre \ref init() must be called before using this function.
    int findFlow(int k = 2) {
      // Find shortest paths
      _path_num = 0;
      while (_path_num < k) {
        // Run Dijkstra
        if (!_dijkstra->run()) break;
        ++_path_num;

        // Set the flow along the found shortest path
        Node u = _target;
        Arc e;
        while ((e = _pred[u]) != INVALID) {
          if (u == _graph.target(e)) {
            (*_flow)[e] = 1;
            u = _graph.source(e);
          } else {
            (*_flow)[e] = 0;
            u = _graph.target(e);
          }
        }
      }
      return _path_num;
    }
    
    /// \brief Compute the paths from the flow.
    ///
    /// This function computes the paths from the flow.
    ///
    /// \pre \ref init() and \ref findFlow() must be called before using
    /// this function.
    void findPaths() {
      // Create the residual flow map (the union of the paths not found
      // so far)
      FlowMap res_flow(_graph);
      for(ArcIt a(_graph); a != INVALID; ++a) res_flow[a] = (*_flow)[a];

      paths.clear();
      paths.resize(_path_num);
      for (int i = 0; i < _path_num; ++i) {
        Node n = _source;
        while (n != _target) {
          OutArcIt e(_graph, n);
          for ( ; res_flow[e] == 0; ++e) ;
          n = _graph.target(e);
          paths[i].addBack(e);
          res_flow[e] = 0;
        }
      }
    }

    /// @}

    /// \name Query Functions
    /// The results of the algorithm can be obtained using these
    /// functions.
    /// \n The algorithm should be executed before using them.

    /// @{

    /// \brief Return a const reference to the arc map storing the
    /// found flow.
    ///
    /// This function returns a const reference to the arc map storing
    /// the flow that is the union of the found arc-disjoint paths.
    ///
    /// \pre \ref run() or \ref findFlow() must be called before using
    /// this function.
    const FlowMap& flowMap() const {
      return *_flow;
    }

    /// \brief Return a const reference to the node map storing the
    /// found potentials (the dual solution).
    ///
    /// This function returns a const reference to the node map storing
    /// the found potentials that provide the dual solution of the
    /// underlying minimum cost flow problem.
    ///
    /// \pre \ref run() or \ref findFlow() must be called before using
    /// this function.
    const PotentialMap& potentialMap() const {
      return *_potential;
    }

    /// \brief Return the flow on the given arc.
    ///
    /// This function returns the flow on the given arc.
    /// It is \c 1 if the arc is involved in one of the found paths,
    /// otherwise it is \c 0.
    ///
    /// \pre \ref run() or \ref findFlow() must be called before using
    /// this function.
    int flow(const Arc& arc) const {
      return (*_flow)[arc];
    }

    /// \brief Return the potential of the given node.
    ///
    /// This function returns the potential of the given node.
    ///
    /// \pre \ref run() or \ref findFlow() must be called before using
    /// this function.
    Length potential(const Node& node) const {
      return (*_potential)[node];
    }

    /// \brief Return the total length (cost) of the found paths (flow).
    ///
    /// This function returns the total length (cost) of the found paths
    /// (flow). The complexity of the function is \f$ O(e) \f$.
    ///
    /// \pre \ref run() or \ref findFlow() must be called before using
    /// this function.
    Length totalLength() const {
      Length c = 0;
      for (ArcIt e(_graph); e != INVALID; ++e)
        c += (*_flow)[e] * _length[e];
      return c;
    }

    /// \brief Return the number of the found paths.
    ///
    /// This function returns the number of the found paths.
    ///
    /// \pre \ref run() or \ref findFlow() must be called before using
    /// this function.
    int pathNum() const {
      return _path_num;
    }

    /// \brief Return a const reference to the specified path.
    ///
    /// This function returns a const reference to the specified path.
    ///
    /// \param i The function returns the \c i-th path.
    /// \c i must be between \c 0 and <tt>%pathNum()-1</tt>.
    ///
    /// \pre \ref run() or \ref findPaths() must be called before using
    /// this function.
    Path path(int i) const {
      return paths[i];
    }

    /// @}

  }; //class Suurballe

  ///@}

} //namespace lemon

#endif //LEMON_SUURBALLE_H
