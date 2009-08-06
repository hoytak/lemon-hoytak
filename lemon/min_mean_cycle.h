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

  /// \brief Default traits class of MinMeanCycle class.
  ///
  /// Default traits class of MinMeanCycle class.
  /// \tparam GR The type of the digraph.
  /// \tparam LEN The type of the length map.
  /// It must conform to the \ref concepts::ReadMap "ReadMap" concept.
#ifdef DOXYGEN
  template <typename GR, typename LEN>
#else
  template <typename GR, typename LEN,
    bool integer = std::numeric_limits<typename LEN::Value>::is_integer>
#endif
  struct MinMeanCycleDefaultTraits
  {
    /// The type of the digraph
    typedef GR Digraph;
    /// The type of the length map
    typedef LEN LengthMap;
    /// The type of the arc lengths
    typedef typename LengthMap::Value Value;

    /// \brief The large value type used for internal computations
    ///
    /// The large value type used for internal computations.
    /// It is \c long \c long if the \c Value type is integer,
    /// otherwise it is \c double.
    /// \c Value must be convertible to \c LargeValue.
    typedef double LargeValue;

    /// The tolerance type used for internal computations
    typedef lemon::Tolerance<LargeValue> Tolerance;

    /// \brief The path type of the found cycles
    ///
    /// The path type of the found cycles.
    /// It must conform to the \ref lemon::concepts::Path "Path" concept
    /// and it must have an \c addBack() function.
    typedef lemon::Path<Digraph> Path;
  };

  // Default traits class for integer value types
  template <typename GR, typename LEN>
  struct MinMeanCycleDefaultTraits<GR, LEN, true>
  {
    typedef GR Digraph;
    typedef LEN LengthMap;
    typedef typename LengthMap::Value Value;
#ifdef LEMON_HAVE_LONG_LONG
    typedef long long LargeValue;
#else
    typedef long LargeValue;
#endif
    typedef lemon::Tolerance<LargeValue> Tolerance;
    typedef lemon::Path<Digraph> Path;
  };


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
#ifdef DOXYGEN
  template <typename GR, typename LEN, typename TR>
#else
  template < typename GR,
             typename LEN = typename GR::template ArcMap<int>,
             typename TR = MinMeanCycleDefaultTraits<GR, LEN> >
#endif
  class MinMeanCycle
  {
  public:
  
    /// The type of the digraph
    typedef typename TR::Digraph Digraph;
    /// The type of the length map
    typedef typename TR::LengthMap LengthMap;
    /// The type of the arc lengths
    typedef typename TR::Value Value;

    /// \brief The large value type
    ///
    /// The large value type used for internal computations.
    /// Using the \ref MinMeanCycleDefaultTraits "default traits class",
    /// it is \c long \c long if the \c Value type is integer,
    /// otherwise it is \c double.
    typedef typename TR::LargeValue LargeValue;

    /// The tolerance type
    typedef typename TR::Tolerance Tolerance;

    /// \brief The path type of the found cycles
    ///
    /// The path type of the found cycles.
    /// Using the \ref MinMeanCycleDefaultTraits "default traits class",
    /// it is \ref lemon::Path "Path<Digraph>".
    typedef typename TR::Path Path;

    /// The \ref MinMeanCycleDefaultTraits "traits class" of the algorithm
    typedef TR Traits;

  private:

    TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
  
    // The digraph the algorithm runs on
    const Digraph &_gr;
    // The length of the arcs
    const LengthMap &_length;

    // Data for the found cycles
    bool _curr_found, _best_found;
    LargeValue _curr_length, _best_length;
    int _curr_size, _best_size;
    Node _curr_node, _best_node;

    Path *_cycle_path;
    bool _local_path;

    // Internal data used by the algorithm
    typename Digraph::template NodeMap<Arc> _policy;
    typename Digraph::template NodeMap<bool> _reached;
    typename Digraph::template NodeMap<int> _level;
    typename Digraph::template NodeMap<LargeValue> _dist;

    // Data for storing the strongly connected components
    int _comp_num;
    typename Digraph::template NodeMap<int> _comp;
    std::vector<std::vector<Node> > _comp_nodes;
    std::vector<Node>* _nodes;
    typename Digraph::template NodeMap<std::vector<Arc> > _in_arcs;
    
    // Queue used for BFS search
    std::vector<Node> _queue;
    int _qfront, _qback;

    Tolerance _tolerance;
  
  public:
  
    /// \name Named Template Parameters
    /// @{

    template <typename T>
    struct SetLargeValueTraits : public Traits {
      typedef T LargeValue;
      typedef lemon::Tolerance<T> Tolerance;
    };

    /// \brief \ref named-templ-param "Named parameter" for setting
    /// \c LargeValue type.
    ///
    /// \ref named-templ-param "Named parameter" for setting \c LargeValue
    /// type. It is used for internal computations in the algorithm.
    template <typename T>
    struct SetLargeValue
      : public MinMeanCycle<GR, LEN, SetLargeValueTraits<T> > {
      typedef MinMeanCycle<GR, LEN, SetLargeValueTraits<T> > Create;
    };

    template <typename T>
    struct SetPathTraits : public Traits {
      typedef T Path;
    };

    /// \brief \ref named-templ-param "Named parameter" for setting
    /// \c %Path type.
    ///
    /// \ref named-templ-param "Named parameter" for setting the \c %Path
    /// type of the found cycles.
    /// It must conform to the \ref lemon::concepts::Path "Path" concept
    /// and it must have an \c addBack() function.
    template <typename T>
    struct SetPath
      : public MinMeanCycle<GR, LEN, SetPathTraits<T> > {
      typedef MinMeanCycle<GR, LEN, SetPathTraits<T> > Create;
    };
    
    /// @}

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
    MinMeanCycle& cycle(Path &path) {
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
    LargeValue cycleLength() const {
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
    const Path& cycle() const {
      return *_cycle_path;
    }

    ///@}

  private:

    // Initialize
    void init() {
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
        _dist[(*_nodes)[i]] = std::numeric_limits<LargeValue>::max();
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
      LargeValue clength;
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
            _dist[u] = _dist[v] + _length[e] * _curr_size - _curr_length;
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
            _dist[u] = _dist[v] + _length[e] * _curr_size - _curr_length;
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
          LargeValue delta = _dist[v] + _length[e] * _curr_size - _curr_length;
          if (_tolerance.less(delta, _dist[u])) {
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
