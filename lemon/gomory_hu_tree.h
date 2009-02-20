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

#ifndef LEMON_GOMORY_HU_TREE_H
#define LEMON_GOMORY_HU_TREE_H

#include <limits>

#include <lemon/preflow.h>
#include <lemon/concept_check.h>
#include <lemon/concepts/maps.h>

/// \ingroup min_cut
/// \file 
/// \brief Gomory-Hu cut tree in graphs.

namespace lemon {

  /// \ingroup min_cut
  ///
  /// \brief Gomory-Hu cut tree algorithm
  ///
  /// The Gomory-Hu tree is a tree on the nodeset of the digraph, but it
  /// may contain arcs which are not in the original digraph. It helps
  /// to calculate the minimum cut between all pairs of nodes, because
  /// the minimum capacity arc on the tree path between two nodes has
  /// the same weight as the minimum cut in the digraph between these
  /// nodes. Moreover this arc separates the nodes to two parts which
  /// determine this minimum cut.
  /// 
  /// The algorithm calculates \e n-1 distinict minimum cuts with
  /// preflow algorithm, therefore the algorithm has
  /// \f$(O(n^3\sqrt{e})\f$ overall time complexity. It calculates a
  /// rooted Gomory-Hu tree, the structure of the tree and the weights
  /// can be obtained with \c predNode() and \c predValue()
  /// functions. The \c minCutValue() and \c minCutMap() calculates
  /// the minimum cut and the minimum cut value between any two node
  /// in the digraph.
  template <typename _Graph, 
	    typename _Capacity = typename _Graph::template EdgeMap<int> >
  class GomoryHuTree {
  public:

    /// The graph type
    typedef _Graph Graph;
    /// The capacity on edges
    typedef _Capacity Capacity;
    /// The value type of capacities
    typedef typename Capacity::Value Value;
    
  private:

    TEMPLATE_GRAPH_TYPEDEFS(Graph);

    const Graph& _graph;
    const Capacity& _capacity;

    Node _root;
    typename Graph::template NodeMap<Node>* _pred;
    typename Graph::template NodeMap<Value>* _weight;
    typename Graph::template NodeMap<int>* _order;

    void createStructures() {
      if (!_pred) {
	_pred = new typename Graph::template NodeMap<Node>(_graph);
      }
      if (!_weight) {
	_weight = new typename Graph::template NodeMap<Value>(_graph);
      }
      if (!_order) {
	_order = new typename Graph::template NodeMap<int>(_graph);
      }
    }

    void destroyStructures() {
      if (_pred) {
	delete _pred;
      }
      if (_weight) {
	delete _weight;
      }
      if (_order) {
	delete _order;
      }
    }
  
  public:

    /// \brief Constructor
    ///
    /// Constructor
    /// \param graph The graph type.
    /// \param capacity The capacity map.
    GomoryHuTree(const Graph& graph, const Capacity& capacity) 
      : _graph(graph), _capacity(capacity),
	_pred(0), _weight(0), _order(0) 
    {
      checkConcept<concepts::ReadMap<Edge, Value>, Capacity>();
    }


    /// \brief Destructor
    ///
    /// Destructor
    ~GomoryHuTree() {
      destroyStructures();
    }

    /// \brief Initializes the internal data structures.
    ///
    /// Initializes the internal data structures.
    ///
    void init() {
      createStructures();

      _root = NodeIt(_graph);
      for (NodeIt n(_graph); n != INVALID; ++n) {
	_pred->set(n, _root);
	_order->set(n, -1);
      }
      _pred->set(_root, INVALID);
      _weight->set(_root, std::numeric_limits<Value>::max()); 
    }


    /// \brief Starts the algorithm
    ///
    /// Starts the algorithm.
    void start() {
      Preflow<Graph, Capacity> fa(_graph, _capacity, _root, INVALID);

      for (NodeIt n(_graph); n != INVALID; ++n) {
	if (n == _root) continue;

	Node pn = (*_pred)[n];
	fa.source(n);
	fa.target(pn);

	fa.runMinCut();

	_weight->set(n, fa.flowValue());

	for (NodeIt nn(_graph); nn != INVALID; ++nn) {
	  if (nn != n && fa.minCut(nn) && (*_pred)[nn] == pn) {
	    _pred->set(nn, n);
	  }
	}
	if ((*_pred)[pn] != INVALID && fa.minCut((*_pred)[pn])) {
	  _pred->set(n, (*_pred)[pn]);
	  _pred->set(pn, n);
	  _weight->set(n, (*_weight)[pn]);
	  _weight->set(pn, fa.flowValue());	
	}
      }

      _order->set(_root, 0);
      int index = 1;

      for (NodeIt n(_graph); n != INVALID; ++n) {
	std::vector<Node> st;
	Node nn = n;
	while ((*_order)[nn] == -1) {
	  st.push_back(nn);
	  nn = (*_pred)[nn];
	}
	while (!st.empty()) {
	  _order->set(st.back(), index++);
	  st.pop_back();
	}
      }
    }

    /// \brief Runs the Gomory-Hu algorithm.  
    ///
    /// Runs the Gomory-Hu algorithm.
    /// \note gh.run() is just a shortcut of the following code.
    /// \code
    ///   ght.init();
    ///   ght.start();
    /// \endcode
    void run() {
      init();
      start();
    }

    /// \brief Returns the predecessor node in the Gomory-Hu tree.
    ///
    /// Returns the predecessor node in the Gomory-Hu tree. If the node is
    /// the root of the Gomory-Hu tree, then it returns \c INVALID.
    Node predNode(const Node& node) {
      return (*_pred)[node];
    }

    /// \brief Returns the weight of the predecessor arc in the
    /// Gomory-Hu tree.
    ///
    /// Returns the weight of the predecessor arc in the Gomory-Hu
    /// tree.  If the node is the root of the Gomory-Hu tree, the
    /// result is undefined.
    Value predValue(const Node& node) {
      return (*_weight)[node];
    }

    /// \brief Returns the minimum cut value between two nodes
    ///
    /// Returns the minimum cut value between two nodes. The
    /// algorithm finds the nearest common ancestor in the Gomory-Hu
    /// tree and calculates the minimum weight arc on the paths to
    /// the ancestor.
    Value minCutValue(const Node& s, const Node& t) const {
      Node sn = s, tn = t;
      Value value = std::numeric_limits<Value>::max();
      
      while (sn != tn) {
	if ((*_order)[sn] < (*_order)[tn]) {
	  if ((*_weight)[tn] < value) value = (*_weight)[tn];
	  tn = (*_pred)[tn];
	} else {
	  if ((*_weight)[sn] < value) value = (*_weight)[sn];
	  sn = (*_pred)[sn];
	}
      }
      return value;
    }

    /// \brief Returns the minimum cut between two nodes
    ///
    /// Returns the minimum cut value between two nodes. The
    /// algorithm finds the nearest common ancestor in the Gomory-Hu
    /// tree and calculates the minimum weight arc on the paths to
    /// the ancestor. Then it sets all nodes to the cut determined by
    /// this arc. The \c cutMap should be \ref concepts::ReadWriteMap
    /// "ReadWriteMap".
    template <typename CutMap>
    Value minCutMap(const Node& s, const Node& t, CutMap& cutMap) const {
      Node sn = s, tn = t;

      Node rn = INVALID;
      Value value = std::numeric_limits<Value>::max();
      
      while (sn != tn) {
	if ((*_order)[sn] < (*_order)[tn]) {
	  if ((*_weight)[tn] < value) {
	    rn = tn;
	    value = (*_weight)[tn];
	  }
	  tn = (*_pred)[tn];
	} else {
	  if ((*_weight)[sn] < value) {
	    rn = sn;
	    value = (*_weight)[sn];
	  }
	  sn = (*_pred)[sn];
	}
      }

      typename Graph::template NodeMap<bool> reached(_graph, false);
      reached.set(_root, true);
      cutMap.set(_root, false);
      reached.set(rn, true);
      cutMap.set(rn, true);

      for (NodeIt n(_graph); n != INVALID; ++n) {
	std::vector<Node> st;
	Node nn = n;
	while (!reached[nn]) {
	  st.push_back(nn);
	  nn = (*_pred)[nn];
	}
	while (!st.empty()) {
	  cutMap.set(st.back(), cutMap[nn]);
	  st.pop_back();
	}
      }
      
      return value;
    }

  };

}

#endif
