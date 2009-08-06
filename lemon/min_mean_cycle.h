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

#ifndef LEMON_MIN_MEAN_CYCLE_H
#define LEMON_MIN_MEAN_CYCLE_H

/// \ingroup shortest_path
///
/// \file
/// \brief Howard's algorithm for finding a minimum mean cycle.

#include <vector>
#include <lemon/core.h>
#include <lemon/path.h>
#include <lemon/tolerance.h>
#include <lemon/connectivity.h>

namespace lemon {

  /// \addtogroup shortest_path
  /// @{

  /// \brief Implementation of Howard's algorithm for finding a minimum
  /// mean cycle.
  ///
  /// \ref MinMeanCycle implements Howard's algorithm for finding a
  /// directed cycle of minimum mean length (cost) in a digraph.
  ///
  /// \tparam GR The type of the digraph the algorithm runs on.
  /// \tparam LEN The type of the length map. The default
  /// map type is \ref concepts::Digraph::ArcMap "GR::ArcMap<int>".
  ///
  /// \warning \c LEN::Value must be convertible to \c double.
#ifdef DOXYGEN
  template <typename GR, typename LEN>
#else
  template < typename GR,
             typename LEN = typename GR::template ArcMap<int> >
#endif
  class MinMeanCycle
  {
  public:
  
    /// The type of the digraph the algorithm runs on
    typedef GR Digraph;
    /// The type of the length map
    typedef LEN LengthMap;
    /// The type of the arc lengths
    typedef typename LengthMap::Value Value;
    /// The type of the paths
    typedef lemon::Path<Digraph> Path;

  private:

    TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
  
    // The digraph the algorithm runs on
    const Digraph &_gr;
    // The length of the arcs
    const LengthMap &_length;

    // Data for the found cycles
    bool _curr_found, _best_found;
    Value _curr_length, _best_length;
    int _curr_size, _best_size;
    Node _curr_node, _best_node;

    Path *_cycle_path;
    bool _local_path;

    // Internal data used by the algorithm
    typename Digraph::template NodeMap<Arc> _policy;
    typename Digraph::template NodeMap<bool> _reached;
    typename Digraph::template NodeMap<int> _level;
    typename Digraph::template NodeMap<double> _dist;

    // Data for storing the strongly connected components
    int _comp_num;
    typename Digraph::template NodeMap<int> _comp;
    std::vector<std::vector<Node> > _comp_nodes;
    std::vector<Node>* _nodes;
    typename Digraph::template NodeMap<std::vector<Arc> > _in_arcs;
    
    // Queue used for BFS search
    std::vector<Node> _queue;
    int _qfront, _qback;
    
    Tolerance<double> _tol;

  public:

    /// \brief Constructor.
    ///
    /// The constructor of the class.
    ///
    /// \param digraph The digraph the algorithm runs on.
    /// \param length The lengths (costs) of the arcs.
    MinMeanCycle( const Digraph &digraph,
                  const LengthMap &length ) :
      _gr(digraph), _length(length), _cycle_path(NULL), _local_path(false),
      _policy(digraph), _reached(digraph), _level(digraph), _dist(digraph),
      _comp(digraph), _in_arcs(digraph)
    {}

    /// Destructor.
    ~MinMeanCycle() {
      if (_local_path) delete _cycle_path;
    }

    /// \brief Set the path structure for storing the found cycle.
    ///
    /// This function sets an external path structure for storing the
    /// found cycle.
    ///
    /// If you don't call this function before calling \ref run() or
    /// \ref findMinMean(), it will allocate a local \ref Path "path"
    /// structure. The destuctor deallocates this automatically
    /// allocated object, of course.
    ///
    /// \note The algorithm calls only the \ref lemon::Path::addBack()
    /// "addBack()" function of the given path structure.
    ///
    /// \return <tt>(*this)</tt>
    ///
    /// \sa cycle()
    MinMeanCycle& cyclePath(Path &path) {
      if (_local_path) {
        delete _cycle_path;
        _local_path = false;
      }
      _cycle_path = &path;
      return *this;
    }

    /// \name Execution control
    /// The simplest way to execute the algorithm is to call the \ref run()
    /// function.\n
    /// If you only need the minimum mean length, you may call
    /// \ref findMinMean().

    /// @{

    /// \brief Run the algorithm.
    ///
    /// This function runs the algorithm.
    /// It can be called more than once (e.g. if the underlying digraph
    /// and/or the arc lengths have been modified).
    ///
    /// \return \c true if a directed cycle exists in the digraph.
    ///
    /// \note <tt>mmc.run()</tt> is just a shortcut of the following code.
    /// \code
    ///   return mmc.findMinMean() && mmc.findCycle();
    /// \endcode
    bool run() {
      return findMinMean() && findCycle();
    }

    /// \brief Find the minimum cycle mean.
    ///
    /// This function finds the minimum mean length of the directed
    /// cycles in the digraph.
    ///
    /// \return \c true if a directed cycle exists in the digraph.
    bool findMinMean() {
      // Initialize and find strongly connected components
      init();
      findComponents();
      
      // Find the minimum cycle mean in the components
      for (int comp = 0; comp < _comp_num; ++comp) {
        // Find the minimum mean cycle in the current component
        if (!buildPolicyGraph(comp)) continue;
        while (true) {
          findPolicyCycle();
          if (!computeNodeDistances()) break;
        }
        // Update the best cycle (global minimum mean cycle)
        if ( !_best_found || (_curr_found &&
             _curr_length * _best_size < _best_length * _curr_size) ) {
          _best_found = true;
          _best_length = _curr_length;
          _best_size = _curr_size;
          _best_node = _curr_node;
        }
      }
      return _best_found;
    }

    /// \brief Find a minimum mean directed cycle.
    ///
    /// This function finds a directed cycle of minimum mean length
    /// in the digraph using the data computed by findMinMean().
    ///
    /// \return \c true if a directed cycle exists in the digraph.
    ///
    /// \pre \ref findMinMean() must be called before using this function.
    bool findCycle() {
      if (!_best_found) return false;
      _cycle_path->addBack(_policy[_best_node]);
      for ( Node v = _best_node;
            (v = _gr.target(_policy[v])) != _best_node; ) {
        _cycle_path->addBack(_policy[v]);
      }
      return true;
    }

    /// @}

    /// \name Query Functions
    /// The results of the algorithm can be obtained using these
    /// functions.\n
    /// The algorithm should be executed before using them.

    /// @{

    /// \brief Return the total length of the found cycle.
    ///
    /// This function returns the total length of the found cycle.
    ///
    /// \pre \ref run() or \ref findMinMean() must be called before
    /// using this function.
    Value cycleLength() const {
      return _best_length;
    }

    /// \brief Return the number of arcs on the found cycle.
    ///
    /// This function returns the number of arcs on the found cycle.
    ///
    /// \pre \ref run() or \ref findMinMean() must be called before
    /// using this function.
    int cycleArcNum() const {
      return _best_size;
    }

    /// \brief Return the mean length of the found cycle.
    ///
    /// This function returns the mean length of the found cycle.
    ///
    /// \note <tt>alg.cycleMean()</tt> is just a shortcut of the
    /// following code.
    /// \code
    ///   return static_cast<double>(alg.cycleLength()) / alg.cycleArcNum();
    /// \endcode
    ///
    /// \pre \ref run() or \ref findMinMean() must be called before
    /// using this function.
    double cycleMean() const {
      return static_cast<double>(_best_length) / _best_size;
    }

    /// \brief Return the found cycle.
    ///
    /// This function returns a const reference to the path structure
    /// storing the found cycle.
    ///
    /// \pre \ref run() or \ref findCycle() must be called before using
    /// this function.
    ///
    /// \sa cyclePath()
    const Path& cycle() const {
      return *_cycle_path;
    }

    ///@}

  private:

    // Initialize
    void init() {
      _tol.epsilon(1e-6);
      if (!_cycle_path) {
        _local_path = true;
        _cycle_path = new Path;
      }
      _queue.resize(countNodes(_gr));
      _best_found = false;
      _best_length = 0;
      _best_size = 1;
      _cycle_path->clear();
    }
    
    // Find strongly connected components and initialize _comp_nodes
    // and _in_arcs
    void findComponents() {
      _comp_num = stronglyConnectedComponents(_gr, _comp);
      _comp_nodes.resize(_comp_num);
      if (_comp_num == 1) {
        _comp_nodes[0].clear();
        for (NodeIt n(_gr); n != INVALID; ++n) {
          _comp_nodes[0].push_back(n);
          _in_arcs[n].clear();
          for (InArcIt a(_gr, n); a != INVALID; ++a) {
            _in_arcs[n].push_back(a);
          }
        }
      } else {
        for (int i = 0; i < _comp_num; ++i)
          _comp_nodes[i].clear();
        for (NodeIt n(_gr); n != INVALID; ++n) {
          int k = _comp[n];
          _comp_nodes[k].push_back(n);
          _in_arcs[n].clear();
          for (InArcIt a(_gr, n); a != INVALID; ++a) {
            if (_comp[_gr.source(a)] == k) _in_arcs[n].push_back(a);
          }
        }
      }
    }

    // Build the policy graph in the given strongly connected component
    // (the out-degree of every node is 1)
    bool buildPolicyGraph(int comp) {
      _nodes = &(_comp_nodes[comp]);
      if (_nodes->size() < 1 ||
          (_nodes->size() == 1 && _in_arcs[(*_nodes)[0]].size() == 0)) {
        return false;
      }
      for (int i = 0; i < int(_nodes->size()); ++i) {
        _dist[(*_nodes)[i]] = std::numeric_limits<double>::max();
      }
      Node u, v;
      Arc e;
      for (int i = 0; i < int(_nodes->size()); ++i) {
        v = (*_nodes)[i];
        for (int j = 0; j < int(_in_arcs[v].size()); ++j) {
          e = _in_arcs[v][j];
          u = _gr.source(e);
          if (_length[e] < _dist[u]) {
            _dist[u] = _length[e];
            _policy[u] = e;
          }
        }
      }
      return true;
    }

    // Find the minimum mean cycle in the policy graph
    void findPolicyCycle() {
      for (int i = 0; i < int(_nodes->size()); ++i) {
        _level[(*_nodes)[i]] = -1;
      }
      Value clength;
      int csize;
      Node u, v;
      _curr_found = false;
      for (int i = 0; i < int(_nodes->size()); ++i) {
        u = (*_nodes)[i];
        if (_level[u] >= 0) continue;
        for (; _level[u] < 0; u = _gr.target(_policy[u])) {
          _level[u] = i;
        }
        if (_level[u] == i) {
          // A cycle is found
          clength = _length[_policy[u]];
          csize = 1;
          for (v = u; (v = _gr.target(_policy[v])) != u; ) {
            clength += _length[_policy[v]];
            ++csize;
          }
          if ( !_curr_found ||
               (clength * _curr_size < _curr_length * csize) ) {
            _curr_found = true;
            _curr_length = clength;
            _curr_size = csize;
            _curr_node = u;
          }
        }
      }
    }

    // Contract the policy graph and compute node distances
    bool computeNodeDistances() {
      // Find the component of the main cycle and compute node distances
      // using reverse BFS
      for (int i = 0; i < int(_nodes->size()); ++i) {
        _reached[(*_nodes)[i]] = false;
      }
      double curr_mean = double(_curr_length) / _curr_size;
      _qfront = _qback = 0;
      _queue[0] = _curr_node;
      _reached[_curr_node] = true;
      _dist[_curr_node] = 0;
      Node u, v;
      Arc e;
      while (_qfront <= _qback) {
        v = _queue[_qfront++];
        for (int j = 0; j < int(_in_arcs[v].size()); ++j) {
          e = _in_arcs[v][j];
          u = _gr.source(e);
          if (_policy[u] == e && !_reached[u]) {
            _reached[u] = true;
            _dist[u] = _dist[v] + _length[e] - curr_mean;
            _queue[++_qback] = u;
          }
        }
      }

      // Connect all other nodes to this component and compute node
      // distances using reverse BFS
      _qfront = 0;
      while (_qback < int(_nodes->size())-1) {
        v = _queue[_qfront++];
        for (int j = 0; j < int(_in_arcs[v].size()); ++j) {
          e = _in_arcs[v][j];
          u = _gr.source(e);
          if (!_reached[u]) {
            _reached[u] = true;
            _policy[u] = e;
            _dist[u] = _dist[v] + _length[e] - curr_mean;
            _queue[++_qback] = u;
          }
        }
      }

      // Improve node distances
      bool improved = false;
      for (int i = 0; i < int(_nodes->size()); ++i) {
        v = (*_nodes)[i];
        for (int j = 0; j < int(_in_arcs[v].size()); ++j) {
          e = _in_arcs[v][j];
          u = _gr.source(e);
          double delta = _dist[v] + _length[e] - curr_mean;
          if (_tol.less(delta, _dist[u])) {
            _dist[u] = delta;
            _policy[u] = e;
            improved = true;
          }
        }
      }
      return improved;
    }

  }; //class MinMeanCycle

  ///@}

} //namespace lemon

#endif //LEMON_MIN_MEAN_CYCLE_H
