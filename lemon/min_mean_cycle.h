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

    // The total length of the found cycle
    Value _cycle_length;
    // The number of arcs on the found cycle
    int _cycle_size;
    // The found cycle
    Path *_cycle_path;

    bool _local_path;
    bool _cycle_found;
    Node _cycle_node;

    typename Digraph::template NodeMap<bool> _reached;
    typename Digraph::template NodeMap<double> _dist;
    typename Digraph::template NodeMap<Arc> _policy;

    typename Digraph::template NodeMap<int> _comp;
    int _comp_num;

    std::vector<Node> _nodes;
    std::vector<Arc> _arcs;
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
      _gr(digraph), _length(length), _cycle_length(0), _cycle_size(-1),
      _cycle_path(NULL), _local_path(false), _reached(digraph),
      _dist(digraph), _policy(digraph), _comp(digraph)
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
      // Initialize
      _tol.epsilon(1e-6);
      if (!_cycle_path) {
        _local_path = true;
        _cycle_path = new Path;
      }
      _cycle_path->clear();
      _cycle_found = false;

      // Find the minimum cycle mean in the components
      _comp_num = stronglyConnectedComponents(_gr, _comp);
      for (int comp = 0; comp < _comp_num; ++comp) {
        if (!initCurrentComponent(comp)) continue;
        while (true) {
          if (!findPolicyCycles()) break;
          contractPolicyGraph(comp);
          if (!computeNodeDistances()) break;
        }
      }
      return _cycle_found;
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
      if (!_cycle_found) return false;
      _cycle_path->addBack(_policy[_cycle_node]);
      for ( Node v = _cycle_node;
            (v = _gr.target(_policy[v])) != _cycle_node; ) {
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
    /// \pre \ref run() or \ref findCycle() must be called before
    /// using this function.
    Value cycleLength() const {
      return _cycle_length;
    }

    /// \brief Return the number of arcs on the found cycle.
    ///
    /// This function returns the number of arcs on the found cycle.
    ///
    /// \pre \ref run() or \ref findCycle() must be called before
    /// using this function.
    int cycleArcNum() const {
      return _cycle_size;
    }

    /// \brief Return the mean length of the found cycle.
    ///
    /// This function returns the mean length of the found cycle.
    ///
    /// \note <tt>mmc.cycleMean()</tt> is just a shortcut of the
    /// following code.
    /// \code
    ///   return double(mmc.cycleLength()) / mmc.cycleArcNum();
    /// \endcode
    ///
    /// \pre \ref run() or \ref findMinMean() must be called before
    /// using this function.
    double cycleMean() const {
      return double(_cycle_length) / _cycle_size;
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

    // Initialize the internal data structures for the current strongly
    // connected component and create the policy graph.
    // The policy graph can be represented by the _policy map because
    // the out-degree of every node is 1.
    bool initCurrentComponent(int comp) {
      // Find the nodes of the current component
      _nodes.clear();
      for (NodeIt n(_gr); n != INVALID; ++n) {
        if (_comp[n] == comp) _nodes.push_back(n);
      }
      if (_nodes.size() <= 1) return false;
      // Find the arcs of the current component
      _arcs.clear();
      for (ArcIt e(_gr); e != INVALID; ++e) {
        if ( _comp[_gr.source(e)] == comp &&
             _comp[_gr.target(e)] == comp )
          _arcs.push_back(e);
      }
      // Initialize _reached, _dist, _policy maps
      for (int i = 0; i < int(_nodes.size()); ++i) {
        _reached[_nodes[i]] = false;
        _policy[_nodes[i]] = INVALID;
      }
      Node u; Arc e;
      for (int j = 0; j < int(_arcs.size()); ++j) {
        e = _arcs[j];
        u = _gr.source(e);
        if (!_reached[u] || _length[e] < _dist[u]) {
          _dist[u] = _length[e];
          _policy[u] = e;
          _reached[u] = true;
        }
      }
      return true;
    }

    // Find all cycles in the policy graph.
    // Set _cycle_found to true if a cycle is found and set
    // _cycle_length, _cycle_size, _cycle_node to represent the minimum
    // mean cycle in the policy graph.
    bool findPolicyCycles() {
      typename Digraph::template NodeMap<int> level(_gr, -1);
      bool curr_cycle_found = false;
      Value clength;
      int csize;
      int path_cnt = 0;
      Node u, v;
      // Searching for cycles
      for (int i = 0; i < int(_nodes.size()); ++i) {
        if (level[_nodes[i]] < 0) {
          u = _nodes[i];
          level[u] = path_cnt;
          while (level[u = _gr.target(_policy[u])] < 0)
            level[u] = path_cnt;
          if (level[u] == path_cnt) {
            // A cycle is found
            curr_cycle_found = true;
            clength = _length[_policy[u]];
            csize = 1;
            for (v = u; (v = _gr.target(_policy[v])) != u; ) {
              clength += _length[_policy[v]];
              ++csize;
            }
            if ( !_cycle_found ||
                 clength * _cycle_size < _cycle_length * csize ) {
              _cycle_found = true;
              _cycle_length = clength;
              _cycle_size = csize;
              _cycle_node = u;
            }
          }
          ++path_cnt;
        }
      }
      return curr_cycle_found;
    }

    // Contract the policy graph to be connected by cutting all cycles
    // except for the main cycle (i.e. the minimum mean cycle).
    void contractPolicyGraph(int comp) {
      // Find the component of the main cycle using reverse BFS search
      typename Digraph::template NodeMap<int> found(_gr, false);
      std::deque<Node> queue;
      queue.push_back(_cycle_node);
      found[_cycle_node] = true;
      Node u, v;
      while (!queue.empty()) {
        v = queue.front(); queue.pop_front();
        for (InArcIt e(_gr, v); e != INVALID; ++e) {
          u = _gr.source(e);
          if (_policy[u] == e && !found[u]) {
            found[u] = true;
            queue.push_back(u);
          }
        }
      }
      // Connect all other nodes to this component using reverse BFS search
      queue.clear();
      for (int i = 0; i < int(_nodes.size()); ++i)
        if (found[_nodes[i]]) queue.push_back(_nodes[i]);
      int found_cnt = queue.size();
      while (found_cnt < int(_nodes.size())) {
        v = queue.front(); queue.pop_front();
        for (InArcIt e(_gr, v); e != INVALID; ++e) {
          u = _gr.source(e);
          if (_comp[u] == comp && !found[u]) {
            found[u] = true;
            ++found_cnt;
            _policy[u] = e;
            queue.push_back(u);
          }
        }
      }
    }

    // Compute node distances in the policy graph and update the
    // policy graph if the node distances can be improved.
    bool computeNodeDistances() {
      // Compute node distances using reverse BFS search
      double cycle_mean = double(_cycle_length) / _cycle_size;
      typename Digraph::template NodeMap<int> found(_gr, false);
      std::deque<Node> queue;
      queue.push_back(_cycle_node);
      found[_cycle_node] = true;
      _dist[_cycle_node] = 0;
      Node u, v;
      while (!queue.empty()) {
        v = queue.front(); queue.pop_front();
        for (InArcIt e(_gr, v); e != INVALID; ++e) {
          u = _gr.source(e);
          if (_policy[u] == e && !found[u]) {
            found[u] = true;
            _dist[u] = _dist[v] + _length[e] - cycle_mean;
            queue.push_back(u);
          }
        }
      }
      // Improving node distances
      bool improved = false;
      for (int j = 0; j < int(_arcs.size()); ++j) {
        Arc e = _arcs[j];
        u = _gr.source(e); v = _gr.target(e);
        double delta = _dist[v] + _length[e] - cycle_mean;
        if (_tol.less(delta, _dist[u])) {
          improved = true;
          _dist[u] = delta;
          _policy[u] = e;
        }
      }
      return improved;
    }

  }; //class MinMeanCycle

  ///@}

} //namespace lemon

#endif //LEMON_MIN_MEAN_CYCLE_H
