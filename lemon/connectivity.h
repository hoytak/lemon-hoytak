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

#ifndef LEMON_CONNECTIVITY_H
#define LEMON_CONNECTIVITY_H

#include <lemon/dfs.h>
#include <lemon/bfs.h>
#include <lemon/core.h>
#include <lemon/maps.h>
#include <lemon/adaptors.h>

#include <lemon/concepts/digraph.h>
#include <lemon/concepts/graph.h>
#include <lemon/concept_check.h>

#include <stack>
#include <functional>

/// \ingroup connectivity
/// \file
/// \brief Connectivity algorithms
///
/// Connectivity algorithms

namespace lemon {

  /// \ingroup connectivity
  ///
  /// \brief Check whether the given undirected graph is connected.
  ///
  /// Check whether the given undirected graph is connected.
  /// \param graph The undirected graph.
  /// \return %True when there is path between any two nodes in the graph.
  /// \note By definition, the empty graph is connected.
  template <typename Graph>
  bool connected(const Graph& graph) {
    checkConcept<concepts::Graph, Graph>();
    typedef typename Graph::NodeIt NodeIt;
    if (NodeIt(graph) == INVALID) return true;
    Dfs<Graph> dfs(graph);
    dfs.run(NodeIt(graph));
    for (NodeIt it(graph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        return false;
      }
    }
    return true;
  }

  /// \ingroup connectivity
  ///
  /// \brief Count the number of connected components of an undirected graph
  ///
  /// Count the number of connected components of an undirected graph
  ///
  /// \param graph The graph. It must be undirected.
  /// \return The number of components
  /// \note By definition, the empty graph consists
  /// of zero connected components.
  template <typename Graph>
  int countConnectedComponents(const Graph &graph) {
    checkConcept<concepts::Graph, Graph>();
    typedef typename Graph::Node Node;
    typedef typename Graph::Arc Arc;

    typedef NullMap<Node, Arc> PredMap;
    typedef NullMap<Node, int> DistMap;

    int compNum = 0;
    typename Bfs<Graph>::
      template SetPredMap<PredMap>::
      template SetDistMap<DistMap>::
      Create bfs(graph);

    PredMap predMap;
    bfs.predMap(predMap);

    DistMap distMap;
    bfs.distMap(distMap);

    bfs.init();
    for(typename Graph::NodeIt n(graph); n != INVALID; ++n) {
      if (!bfs.reached(n)) {
        bfs.addSource(n);
        bfs.start();
        ++compNum;
      }
    }
    return compNum;
  }

  /// \ingroup connectivity
  ///
  /// \brief Find the connected components of an undirected graph
  ///
  /// Find the connected components of an undirected graph.
  ///
  /// \param graph The graph. It must be undirected.
  /// \retval compMap A writable node map. The values will be set from 0 to
  /// the number of the connected components minus one. Each values of the map
  /// will be set exactly once, the values of a certain component will be
  /// set continuously.
  /// \return The number of components
  ///
  template <class Graph, class NodeMap>
  int connectedComponents(const Graph &graph, NodeMap &compMap) {
    checkConcept<concepts::Graph, Graph>();
    typedef typename Graph::Node Node;
    typedef typename Graph::Arc Arc;
    checkConcept<concepts::WriteMap<Node, int>, NodeMap>();

    typedef NullMap<Node, Arc> PredMap;
    typedef NullMap<Node, int> DistMap;

    int compNum = 0;
    typename Bfs<Graph>::
      template SetPredMap<PredMap>::
      template SetDistMap<DistMap>::
      Create bfs(graph);

    PredMap predMap;
    bfs.predMap(predMap);

    DistMap distMap;
    bfs.distMap(distMap);

    bfs.init();
    for(typename Graph::NodeIt n(graph); n != INVALID; ++n) {
      if(!bfs.reached(n)) {
        bfs.addSource(n);
        while (!bfs.emptyQueue()) {
          compMap.set(bfs.nextNode(), compNum);
          bfs.processNextNode();
        }
        ++compNum;
      }
    }
    return compNum;
  }

  namespace _connectivity_bits {

    template <typename Digraph, typename Iterator >
    struct LeaveOrderVisitor : public DfsVisitor<Digraph> {
    public:
      typedef typename Digraph::Node Node;
      LeaveOrderVisitor(Iterator it) : _it(it) {}

      void leave(const Node& node) {
        *(_it++) = node;
      }

    private:
      Iterator _it;
    };

    template <typename Digraph, typename Map>
    struct FillMapVisitor : public DfsVisitor<Digraph> {
    public:
      typedef typename Digraph::Node Node;
      typedef typename Map::Value Value;

      FillMapVisitor(Map& map, Value& value)
        : _map(map), _value(value) {}

      void reach(const Node& node) {
        _map.set(node, _value);
      }
    private:
      Map& _map;
      Value& _value;
    };

    template <typename Digraph, typename ArcMap>
    struct StronglyConnectedCutArcsVisitor : public DfsVisitor<Digraph> {
    public:
      typedef typename Digraph::Node Node;
      typedef typename Digraph::Arc Arc;

      StronglyConnectedCutArcsVisitor(const Digraph& digraph,
                                      ArcMap& cutMap,
                                      int& cutNum)
        : _digraph(digraph), _cutMap(cutMap), _cutNum(cutNum),
          _compMap(digraph, -1), _num(-1) {
      }

      void start(const Node&) {
        ++_num;
      }

      void reach(const Node& node) {
        _compMap.set(node, _num);
      }

      void examine(const Arc& arc) {
         if (_compMap[_digraph.source(arc)] !=
             _compMap[_digraph.target(arc)]) {
           _cutMap.set(arc, true);
           ++_cutNum;
         }
      }
    private:
      const Digraph& _digraph;
      ArcMap& _cutMap;
      int& _cutNum;

      typename Digraph::template NodeMap<int> _compMap;
      int _num;
    };

  }


  /// \ingroup connectivity
  ///
  /// \brief Check whether the given directed graph is strongly connected.
  ///
  /// Check whether the given directed graph is strongly connected. The
  /// graph is strongly connected when any two nodes of the graph are
  /// connected with directed paths in both direction.
  /// \return %False when the graph is not strongly connected.
  /// \see connected
  ///
  /// \note By definition, the empty graph is strongly connected.
  template <typename Digraph>
  bool stronglyConnected(const Digraph& digraph) {
    checkConcept<concepts::Digraph, Digraph>();

    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;

    typename Digraph::Node source = NodeIt(digraph);
    if (source == INVALID) return true;

    using namespace _connectivity_bits;

    typedef DfsVisitor<Digraph> Visitor;
    Visitor visitor;

    DfsVisit<Digraph, Visitor> dfs(digraph, visitor);
    dfs.init();
    dfs.addSource(source);
    dfs.start();

    for (NodeIt it(digraph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        return false;
      }
    }

    typedef ReverseDigraph<const Digraph> RDigraph;
    typedef typename RDigraph::NodeIt RNodeIt;
    RDigraph rdigraph(digraph);

    typedef DfsVisitor<Digraph> RVisitor;
    RVisitor rvisitor;

    DfsVisit<RDigraph, RVisitor> rdfs(rdigraph, rvisitor);
    rdfs.init();
    rdfs.addSource(source);
    rdfs.start();

    for (RNodeIt it(rdigraph); it != INVALID; ++it) {
      if (!rdfs.reached(it)) {
        return false;
      }
    }

    return true;
  }

  /// \ingroup connectivity
  ///
  /// \brief Count the strongly connected components of a directed graph
  ///
  /// Count the strongly connected components of a directed graph.
  /// The strongly connected components are the classes of an
  /// equivalence relation on the nodes of the graph. Two nodes are in
  /// the same class if they are connected with directed paths in both
  /// direction.
  ///
  /// \param digraph The graph.
  /// \return The number of components
  /// \note By definition, the empty graph has zero
  /// strongly connected components.
  template <typename Digraph>
  int countStronglyConnectedComponents(const Digraph& digraph) {
    checkConcept<concepts::Digraph, Digraph>();

    using namespace _connectivity_bits;

    typedef typename Digraph::Node Node;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::ArcIt ArcIt;

    typedef std::vector<Node> Container;
    typedef typename Container::iterator Iterator;

    Container nodes(countNodes(digraph));
    typedef LeaveOrderVisitor<Digraph, Iterator> Visitor;
    Visitor visitor(nodes.begin());

    DfsVisit<Digraph, Visitor> dfs(digraph, visitor);
    dfs.init();
    for (NodeIt it(digraph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        dfs.addSource(it);
        dfs.start();
      }
    }

    typedef typename Container::reverse_iterator RIterator;
    typedef ReverseDigraph<const Digraph> RDigraph;

    RDigraph rdigraph(digraph);

    typedef DfsVisitor<Digraph> RVisitor;
    RVisitor rvisitor;

    DfsVisit<RDigraph, RVisitor> rdfs(rdigraph, rvisitor);

    int compNum = 0;

    rdfs.init();
    for (RIterator it = nodes.rbegin(); it != nodes.rend(); ++it) {
      if (!rdfs.reached(*it)) {
        rdfs.addSource(*it);
        rdfs.start();
        ++compNum;
      }
    }
    return compNum;
  }

  /// \ingroup connectivity
  ///
  /// \brief Find the strongly connected components of a directed graph
  ///
  /// Find the strongly connected components of a directed graph.  The
  /// strongly connected components are the classes of an equivalence
  /// relation on the nodes of the graph. Two nodes are in
  /// relationship when there are directed paths between them in both
  /// direction. In addition, the numbering of components will satisfy
  /// that there is no arc going from a higher numbered component to
  /// a lower.
  ///
  /// \param digraph The digraph.
  /// \retval compMap A writable node map. The values will be set from 0 to
  /// the number of the strongly connected components minus one. Each value
  /// of the map will be set exactly once, the values of a certain component
  /// will be set continuously.
  /// \return The number of components
  ///
  template <typename Digraph, typename NodeMap>
  int stronglyConnectedComponents(const Digraph& digraph, NodeMap& compMap) {
    checkConcept<concepts::Digraph, Digraph>();
    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    checkConcept<concepts::WriteMap<Node, int>, NodeMap>();

    using namespace _connectivity_bits;

    typedef std::vector<Node> Container;
    typedef typename Container::iterator Iterator;

    Container nodes(countNodes(digraph));
    typedef LeaveOrderVisitor<Digraph, Iterator> Visitor;
    Visitor visitor(nodes.begin());

    DfsVisit<Digraph, Visitor> dfs(digraph, visitor);
    dfs.init();
    for (NodeIt it(digraph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        dfs.addSource(it);
        dfs.start();
      }
    }

    typedef typename Container::reverse_iterator RIterator;
    typedef ReverseDigraph<const Digraph> RDigraph;

    RDigraph rdigraph(digraph);

    int compNum = 0;

    typedef FillMapVisitor<RDigraph, NodeMap> RVisitor;
    RVisitor rvisitor(compMap, compNum);

    DfsVisit<RDigraph, RVisitor> rdfs(rdigraph, rvisitor);

    rdfs.init();
    for (RIterator it = nodes.rbegin(); it != nodes.rend(); ++it) {
      if (!rdfs.reached(*it)) {
        rdfs.addSource(*it);
        rdfs.start();
        ++compNum;
      }
    }
    return compNum;
  }

  /// \ingroup connectivity
  ///
  /// \brief Find the cut arcs of the strongly connected components.
  ///
  /// Find the cut arcs of the strongly connected components.
  /// The strongly connected components are the classes of an equivalence
  /// relation on the nodes of the graph. Two nodes are in relationship
  /// when there are directed paths between them in both direction.
  /// The strongly connected components are separated by the cut arcs.
  ///
  /// \param graph The graph.
  /// \retval cutMap A writable node map. The values will be set true when the
  /// arc is a cut arc.
  ///
  /// \return The number of cut arcs
  template <typename Digraph, typename ArcMap>
  int stronglyConnectedCutArcs(const Digraph& graph, ArcMap& cutMap) {
    checkConcept<concepts::Digraph, Digraph>();
    typedef typename Digraph::Node Node;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::NodeIt NodeIt;
    checkConcept<concepts::WriteMap<Arc, bool>, ArcMap>();

    using namespace _connectivity_bits;

    typedef std::vector<Node> Container;
    typedef typename Container::iterator Iterator;

    Container nodes(countNodes(graph));
    typedef LeaveOrderVisitor<Digraph, Iterator> Visitor;
    Visitor visitor(nodes.begin());

    DfsVisit<Digraph, Visitor> dfs(graph, visitor);
    dfs.init();
    for (NodeIt it(graph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        dfs.addSource(it);
        dfs.start();
      }
    }

    typedef typename Container::reverse_iterator RIterator;
    typedef ReverseDigraph<const Digraph> RDigraph;

    RDigraph rgraph(graph);

    int cutNum = 0;

    typedef StronglyConnectedCutArcsVisitor<RDigraph, ArcMap> RVisitor;
    RVisitor rvisitor(rgraph, cutMap, cutNum);

    DfsVisit<RDigraph, RVisitor> rdfs(rgraph, rvisitor);

    rdfs.init();
    for (RIterator it = nodes.rbegin(); it != nodes.rend(); ++it) {
      if (!rdfs.reached(*it)) {
        rdfs.addSource(*it);
        rdfs.start();
      }
    }
    return cutNum;
  }

  namespace _connectivity_bits {

    template <typename Digraph>
    class CountBiNodeConnectedComponentsVisitor : public DfsVisitor<Digraph> {
    public:
      typedef typename Digraph::Node Node;
      typedef typename Digraph::Arc Arc;
      typedef typename Digraph::Edge Edge;

      CountBiNodeConnectedComponentsVisitor(const Digraph& graph, int &compNum)
        : _graph(graph), _compNum(compNum),
          _numMap(graph), _retMap(graph), _predMap(graph), _num(0) {}

      void start(const Node& node) {
        _predMap.set(node, INVALID);
      }

      void reach(const Node& node) {
        _numMap.set(node, _num);
        _retMap.set(node, _num);
        ++_num;
      }

      void discover(const Arc& edge) {
        _predMap.set(_graph.target(edge), _graph.source(edge));
      }

      void examine(const Arc& edge) {
        if (_graph.source(edge) == _graph.target(edge) &&
            _graph.direction(edge)) {
          ++_compNum;
          return;
        }
        if (_predMap[_graph.source(edge)] == _graph.target(edge)) {
          return;
        }
        if (_retMap[_graph.source(edge)] > _numMap[_graph.target(edge)]) {
          _retMap.set(_graph.source(edge), _numMap[_graph.target(edge)]);
        }
      }

      void backtrack(const Arc& edge) {
        if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
          _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
        }
        if (_numMap[_graph.source(edge)] <= _retMap[_graph.target(edge)]) {
          ++_compNum;
        }
      }

    private:
      const Digraph& _graph;
      int& _compNum;

      typename Digraph::template NodeMap<int> _numMap;
      typename Digraph::template NodeMap<int> _retMap;
      typename Digraph::template NodeMap<Node> _predMap;
      int _num;
    };

    template <typename Digraph, typename ArcMap>
    class BiNodeConnectedComponentsVisitor : public DfsVisitor<Digraph> {
    public:
      typedef typename Digraph::Node Node;
      typedef typename Digraph::Arc Arc;
      typedef typename Digraph::Edge Edge;

      BiNodeConnectedComponentsVisitor(const Digraph& graph,
                                       ArcMap& compMap, int &compNum)
        : _graph(graph), _compMap(compMap), _compNum(compNum),
          _numMap(graph), _retMap(graph), _predMap(graph), _num(0) {}

      void start(const Node& node) {
        _predMap.set(node, INVALID);
      }

      void reach(const Node& node) {
        _numMap.set(node, _num);
        _retMap.set(node, _num);
        ++_num;
      }

      void discover(const Arc& edge) {
        Node target = _graph.target(edge);
        _predMap.set(target, edge);
        _edgeStack.push(edge);
      }

      void examine(const Arc& edge) {
        Node source = _graph.source(edge);
        Node target = _graph.target(edge);
        if (source == target && _graph.direction(edge)) {
          _compMap.set(edge, _compNum);
          ++_compNum;
          return;
        }
        if (_numMap[target] < _numMap[source]) {
          if (_predMap[source] != _graph.oppositeArc(edge)) {
            _edgeStack.push(edge);
          }
        }
        if (_predMap[source] != INVALID &&
            target == _graph.source(_predMap[source])) {
          return;
        }
        if (_retMap[source] > _numMap[target]) {
          _retMap.set(source, _numMap[target]);
        }
      }

      void backtrack(const Arc& edge) {
        Node source = _graph.source(edge);
        Node target = _graph.target(edge);
        if (_retMap[source] > _retMap[target]) {
          _retMap.set(source, _retMap[target]);
        }
        if (_numMap[source] <= _retMap[target]) {
          while (_edgeStack.top() != edge) {
            _compMap.set(_edgeStack.top(), _compNum);
            _edgeStack.pop();
          }
          _compMap.set(edge, _compNum);
          _edgeStack.pop();
          ++_compNum;
        }
      }

    private:
      const Digraph& _graph;
      ArcMap& _compMap;
      int& _compNum;

      typename Digraph::template NodeMap<int> _numMap;
      typename Digraph::template NodeMap<int> _retMap;
      typename Digraph::template NodeMap<Arc> _predMap;
      std::stack<Edge> _edgeStack;
      int _num;
    };


    template <typename Digraph, typename NodeMap>
    class BiNodeConnectedCutNodesVisitor : public DfsVisitor<Digraph> {
    public:
      typedef typename Digraph::Node Node;
      typedef typename Digraph::Arc Arc;
      typedef typename Digraph::Edge Edge;

      BiNodeConnectedCutNodesVisitor(const Digraph& graph, NodeMap& cutMap,
                                     int& cutNum)
        : _graph(graph), _cutMap(cutMap), _cutNum(cutNum),
          _numMap(graph), _retMap(graph), _predMap(graph), _num(0) {}

      void start(const Node& node) {
        _predMap.set(node, INVALID);
        rootCut = false;
      }

      void reach(const Node& node) {
        _numMap.set(node, _num);
        _retMap.set(node, _num);
        ++_num;
      }

      void discover(const Arc& edge) {
        _predMap.set(_graph.target(edge), _graph.source(edge));
      }

      void examine(const Arc& edge) {
        if (_graph.source(edge) == _graph.target(edge) &&
            _graph.direction(edge)) {
          if (!_cutMap[_graph.source(edge)]) {
            _cutMap.set(_graph.source(edge), true);
            ++_cutNum;
          }
          return;
        }
        if (_predMap[_graph.source(edge)] == _graph.target(edge)) return;
        if (_retMap[_graph.source(edge)] > _numMap[_graph.target(edge)]) {
          _retMap.set(_graph.source(edge), _numMap[_graph.target(edge)]);
        }
      }

      void backtrack(const Arc& edge) {
        if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
          _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
        }
        if (_numMap[_graph.source(edge)] <= _retMap[_graph.target(edge)]) {
          if (_predMap[_graph.source(edge)] != INVALID) {
            if (!_cutMap[_graph.source(edge)]) {
              _cutMap.set(_graph.source(edge), true);
              ++_cutNum;
            }
          } else if (rootCut) {
            if (!_cutMap[_graph.source(edge)]) {
              _cutMap.set(_graph.source(edge), true);
              ++_cutNum;
            }
          } else {
            rootCut = true;
          }
        }
      }

    private:
      const Digraph& _graph;
      NodeMap& _cutMap;
      int& _cutNum;

      typename Digraph::template NodeMap<int> _numMap;
      typename Digraph::template NodeMap<int> _retMap;
      typename Digraph::template NodeMap<Node> _predMap;
      std::stack<Edge> _edgeStack;
      int _num;
      bool rootCut;
    };

  }

  template <typename Graph>
  int countBiNodeConnectedComponents(const Graph& graph);

  /// \ingroup connectivity
  ///
  /// \brief Checks the graph is bi-node-connected.
  ///
  /// This function checks that the undirected graph is bi-node-connected
  /// graph. The graph is bi-node-connected if any two undirected edge is
  /// on same circle.
  ///
  /// \param graph The graph.
  /// \return %True when the graph bi-node-connected.
  template <typename Graph>
  bool biNodeConnected(const Graph& graph) {
    return countBiNodeConnectedComponents(graph) <= 1;
  }

  /// \ingroup connectivity
  ///
  /// \brief Count the biconnected components.
  ///
  /// This function finds the bi-node-connected components in an undirected
  /// graph. The biconnected components are the classes of an equivalence
  /// relation on the undirected edges. Two undirected edge is in relationship
  /// when they are on same circle.
  ///
  /// \param graph The graph.
  /// \return The number of components.
  template <typename Graph>
  int countBiNodeConnectedComponents(const Graph& graph) {
    checkConcept<concepts::Graph, Graph>();
    typedef typename Graph::NodeIt NodeIt;

    using namespace _connectivity_bits;

    typedef CountBiNodeConnectedComponentsVisitor<Graph> Visitor;

    int compNum = 0;
    Visitor visitor(graph, compNum);

    DfsVisit<Graph, Visitor> dfs(graph, visitor);
    dfs.init();

    for (NodeIt it(graph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        dfs.addSource(it);
        dfs.start();
      }
    }
    return compNum;
  }

  /// \ingroup connectivity
  ///
  /// \brief Find the bi-node-connected components.
  ///
  /// This function finds the bi-node-connected components in an undirected
  /// graph. The bi-node-connected components are the classes of an equivalence
  /// relation on the undirected edges. Two undirected edge are in relationship
  /// when they are on same circle.
  ///
  /// \param graph The graph.
  /// \retval compMap A writable uedge map. The values will be set from 0
  /// to the number of the biconnected components minus one. Each values
  /// of the map will be set exactly once, the values of a certain component
  /// will be set continuously.
  /// \return The number of components.
  ///
  template <typename Graph, typename EdgeMap>
  int biNodeConnectedComponents(const Graph& graph,
                                EdgeMap& compMap) {
    checkConcept<concepts::Graph, Graph>();
    typedef typename Graph::NodeIt NodeIt;
    typedef typename Graph::Edge Edge;
    checkConcept<concepts::WriteMap<Edge, int>, EdgeMap>();

    using namespace _connectivity_bits;

    typedef BiNodeConnectedComponentsVisitor<Graph, EdgeMap> Visitor;

    int compNum = 0;
    Visitor visitor(graph, compMap, compNum);

    DfsVisit<Graph, Visitor> dfs(graph, visitor);
    dfs.init();

    for (NodeIt it(graph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        dfs.addSource(it);
        dfs.start();
      }
    }
    return compNum;
  }

  /// \ingroup connectivity
  ///
  /// \brief Find the bi-node-connected cut nodes.
  ///
  /// This function finds the bi-node-connected cut nodes in an undirected
  /// graph. The bi-node-connected components are the classes of an equivalence
  /// relation on the undirected edges. Two undirected edges are in
  /// relationship when they are on same circle. The biconnected components
  /// are separted by nodes which are the cut nodes of the components.
  ///
  /// \param graph The graph.
  /// \retval cutMap A writable edge map. The values will be set true when
  /// the node separate two or more components.
  /// \return The number of the cut nodes.
  template <typename Graph, typename NodeMap>
  int biNodeConnectedCutNodes(const Graph& graph, NodeMap& cutMap) {
    checkConcept<concepts::Graph, Graph>();
    typedef typename Graph::Node Node;
    typedef typename Graph::NodeIt NodeIt;
    checkConcept<concepts::WriteMap<Node, bool>, NodeMap>();

    using namespace _connectivity_bits;

    typedef BiNodeConnectedCutNodesVisitor<Graph, NodeMap> Visitor;

    int cutNum = 0;
    Visitor visitor(graph, cutMap, cutNum);

    DfsVisit<Graph, Visitor> dfs(graph, visitor);
    dfs.init();

    for (NodeIt it(graph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        dfs.addSource(it);
        dfs.start();
      }
    }
    return cutNum;
  }

  namespace _connectivity_bits {

    template <typename Digraph>
    class CountBiEdgeConnectedComponentsVisitor : public DfsVisitor<Digraph> {
    public:
      typedef typename Digraph::Node Node;
      typedef typename Digraph::Arc Arc;
      typedef typename Digraph::Edge Edge;

      CountBiEdgeConnectedComponentsVisitor(const Digraph& graph, int &compNum)
        : _graph(graph), _compNum(compNum),
          _numMap(graph), _retMap(graph), _predMap(graph), _num(0) {}

      void start(const Node& node) {
        _predMap.set(node, INVALID);
      }

      void reach(const Node& node) {
        _numMap.set(node, _num);
        _retMap.set(node, _num);
        ++_num;
      }

      void leave(const Node& node) {
        if (_numMap[node] <= _retMap[node]) {
          ++_compNum;
        }
      }

      void discover(const Arc& edge) {
        _predMap.set(_graph.target(edge), edge);
      }

      void examine(const Arc& edge) {
        if (_predMap[_graph.source(edge)] == _graph.oppositeArc(edge)) {
          return;
        }
        if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
          _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
        }
      }

      void backtrack(const Arc& edge) {
        if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
          _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
        }
      }

    private:
      const Digraph& _graph;
      int& _compNum;

      typename Digraph::template NodeMap<int> _numMap;
      typename Digraph::template NodeMap<int> _retMap;
      typename Digraph::template NodeMap<Arc> _predMap;
      int _num;
    };

    template <typename Digraph, typename NodeMap>
    class BiEdgeConnectedComponentsVisitor : public DfsVisitor<Digraph> {
    public:
      typedef typename Digraph::Node Node;
      typedef typename Digraph::Arc Arc;
      typedef typename Digraph::Edge Edge;

      BiEdgeConnectedComponentsVisitor(const Digraph& graph,
                                       NodeMap& compMap, int &compNum)
        : _graph(graph), _compMap(compMap), _compNum(compNum),
          _numMap(graph), _retMap(graph), _predMap(graph), _num(0) {}

      void start(const Node& node) {
        _predMap.set(node, INVALID);
      }

      void reach(const Node& node) {
        _numMap.set(node, _num);
        _retMap.set(node, _num);
        _nodeStack.push(node);
        ++_num;
      }

      void leave(const Node& node) {
        if (_numMap[node] <= _retMap[node]) {
          while (_nodeStack.top() != node) {
            _compMap.set(_nodeStack.top(), _compNum);
            _nodeStack.pop();
          }
          _compMap.set(node, _compNum);
          _nodeStack.pop();
          ++_compNum;
        }
      }

      void discover(const Arc& edge) {
        _predMap.set(_graph.target(edge), edge);
      }

      void examine(const Arc& edge) {
        if (_predMap[_graph.source(edge)] == _graph.oppositeArc(edge)) {
          return;
        }
        if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
          _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
        }
      }

      void backtrack(const Arc& edge) {
        if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
          _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
        }
      }

    private:
      const Digraph& _graph;
      NodeMap& _compMap;
      int& _compNum;

      typename Digraph::template NodeMap<int> _numMap;
      typename Digraph::template NodeMap<int> _retMap;
      typename Digraph::template NodeMap<Arc> _predMap;
      std::stack<Node> _nodeStack;
      int _num;
    };


    template <typename Digraph, typename ArcMap>
    class BiEdgeConnectedCutEdgesVisitor : public DfsVisitor<Digraph> {
    public:
      typedef typename Digraph::Node Node;
      typedef typename Digraph::Arc Arc;
      typedef typename Digraph::Edge Edge;

      BiEdgeConnectedCutEdgesVisitor(const Digraph& graph,
                                     ArcMap& cutMap, int &cutNum)
        : _graph(graph), _cutMap(cutMap), _cutNum(cutNum),
          _numMap(graph), _retMap(graph), _predMap(graph), _num(0) {}

      void start(const Node& node) {
        _predMap[node] = INVALID;
      }

      void reach(const Node& node) {
        _numMap.set(node, _num);
        _retMap.set(node, _num);
        ++_num;
      }

      void leave(const Node& node) {
        if (_numMap[node] <= _retMap[node]) {
          if (_predMap[node] != INVALID) {
            _cutMap.set(_predMap[node], true);
            ++_cutNum;
          }
        }
      }

      void discover(const Arc& edge) {
        _predMap.set(_graph.target(edge), edge);
      }

      void examine(const Arc& edge) {
        if (_predMap[_graph.source(edge)] == _graph.oppositeArc(edge)) {
          return;
        }
        if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
          _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
        }
      }

      void backtrack(const Arc& edge) {
        if (_retMap[_graph.source(edge)] > _retMap[_graph.target(edge)]) {
          _retMap.set(_graph.source(edge), _retMap[_graph.target(edge)]);
        }
      }

    private:
      const Digraph& _graph;
      ArcMap& _cutMap;
      int& _cutNum;

      typename Digraph::template NodeMap<int> _numMap;
      typename Digraph::template NodeMap<int> _retMap;
      typename Digraph::template NodeMap<Arc> _predMap;
      int _num;
    };
  }

  template <typename Graph>
  int countBiEdgeConnectedComponents(const Graph& graph);

  /// \ingroup connectivity
  ///
  /// \brief Checks that the graph is bi-edge-connected.
  ///
  /// This function checks that the graph is bi-edge-connected. The undirected
  /// graph is bi-edge-connected when any two nodes are connected with two
  /// edge-disjoint paths.
  ///
  /// \param graph The undirected graph.
  /// \return The number of components.
  template <typename Graph>
  bool biEdgeConnected(const Graph& graph) {
    return countBiEdgeConnectedComponents(graph) <= 1;
  }

  /// \ingroup connectivity
  ///
  /// \brief Count the bi-edge-connected components.
  ///
  /// This function count the bi-edge-connected components in an undirected
  /// graph. The bi-edge-connected components are the classes of an equivalence
  /// relation on the nodes. Two nodes are in relationship when they are
  /// connected with at least two edge-disjoint paths.
  ///
  /// \param graph The undirected graph.
  /// \return The number of components.
  template <typename Graph>
  int countBiEdgeConnectedComponents(const Graph& graph) {
    checkConcept<concepts::Graph, Graph>();
    typedef typename Graph::NodeIt NodeIt;

    using namespace _connectivity_bits;

    typedef CountBiEdgeConnectedComponentsVisitor<Graph> Visitor;

    int compNum = 0;
    Visitor visitor(graph, compNum);

    DfsVisit<Graph, Visitor> dfs(graph, visitor);
    dfs.init();

    for (NodeIt it(graph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        dfs.addSource(it);
        dfs.start();
      }
    }
    return compNum;
  }

  /// \ingroup connectivity
  ///
  /// \brief Find the bi-edge-connected components.
  ///
  /// This function finds the bi-edge-connected components in an undirected
  /// graph. The bi-edge-connected components are the classes of an equivalence
  /// relation on the nodes. Two nodes are in relationship when they are
  /// connected at least two edge-disjoint paths.
  ///
  /// \param graph The graph.
  /// \retval compMap A writable node map. The values will be set from 0 to
  /// the number of the biconnected components minus one. Each values
  /// of the map will be set exactly once, the values of a certain component
  /// will be set continuously.
  /// \return The number of components.
  ///
  template <typename Graph, typename NodeMap>
  int biEdgeConnectedComponents(const Graph& graph, NodeMap& compMap) {
    checkConcept<concepts::Graph, Graph>();
    typedef typename Graph::NodeIt NodeIt;
    typedef typename Graph::Node Node;
    checkConcept<concepts::WriteMap<Node, int>, NodeMap>();

    using namespace _connectivity_bits;

    typedef BiEdgeConnectedComponentsVisitor<Graph, NodeMap> Visitor;

    int compNum = 0;
    Visitor visitor(graph, compMap, compNum);

    DfsVisit<Graph, Visitor> dfs(graph, visitor);
    dfs.init();

    for (NodeIt it(graph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        dfs.addSource(it);
        dfs.start();
      }
    }
    return compNum;
  }

  /// \ingroup connectivity
  ///
  /// \brief Find the bi-edge-connected cut edges.
  ///
  /// This function finds the bi-edge-connected components in an undirected
  /// graph. The bi-edge-connected components are the classes of an equivalence
  /// relation on the nodes. Two nodes are in relationship when they are
  /// connected with at least two edge-disjoint paths. The bi-edge-connected
  /// components are separted by edges which are the cut edges of the
  /// components.
  ///
  /// \param graph The graph.
  /// \retval cutMap A writable node map. The values will be set true when the
  /// edge is a cut edge.
  /// \return The number of cut edges.
  template <typename Graph, typename EdgeMap>
  int biEdgeConnectedCutEdges(const Graph& graph, EdgeMap& cutMap) {
    checkConcept<concepts::Graph, Graph>();
    typedef typename Graph::NodeIt NodeIt;
    typedef typename Graph::Edge Edge;
    checkConcept<concepts::WriteMap<Edge, bool>, EdgeMap>();

    using namespace _connectivity_bits;

    typedef BiEdgeConnectedCutEdgesVisitor<Graph, EdgeMap> Visitor;

    int cutNum = 0;
    Visitor visitor(graph, cutMap, cutNum);

    DfsVisit<Graph, Visitor> dfs(graph, visitor);
    dfs.init();

    for (NodeIt it(graph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        dfs.addSource(it);
        dfs.start();
      }
    }
    return cutNum;
  }


  namespace _connectivity_bits {

    template <typename Digraph, typename IntNodeMap>
    class TopologicalSortVisitor : public DfsVisitor<Digraph> {
    public:
      typedef typename Digraph::Node Node;
      typedef typename Digraph::Arc edge;

      TopologicalSortVisitor(IntNodeMap& order, int num)
        : _order(order), _num(num) {}

      void leave(const Node& node) {
        _order.set(node, --_num);
      }

    private:
      IntNodeMap& _order;
      int _num;
    };

  }

  /// \ingroup connectivity
  ///
  /// \brief Sort the nodes of a DAG into topolgical order.
  ///
  /// Sort the nodes of a DAG into topolgical order.
  ///
  /// \param graph The graph. It must be directed and acyclic.
  /// \retval order A writable node map. The values will be set from 0 to
  /// the number of the nodes in the graph minus one. Each values of the map
  /// will be set exactly once, the values  will be set descending order.
  ///
  /// \see checkedTopologicalSort
  /// \see dag
  template <typename Digraph, typename NodeMap>
  void topologicalSort(const Digraph& graph, NodeMap& order) {
    using namespace _connectivity_bits;

    checkConcept<concepts::Digraph, Digraph>();
    checkConcept<concepts::WriteMap<typename Digraph::Node, int>, NodeMap>();

    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;

    TopologicalSortVisitor<Digraph, NodeMap>
      visitor(order, countNodes(graph));

    DfsVisit<Digraph, TopologicalSortVisitor<Digraph, NodeMap> >
      dfs(graph, visitor);

    dfs.init();
    for (NodeIt it(graph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        dfs.addSource(it);
        dfs.start();
      }
    }
  }

  /// \ingroup connectivity
  ///
  /// \brief Sort the nodes of a DAG into topolgical order.
  ///
  /// Sort the nodes of a DAG into topolgical order. It also checks
  /// that the given graph is DAG.
  ///
  /// \param digraph The graph. It must be directed and acyclic.
  /// \retval order A readable - writable node map. The values will be set
  /// from 0 to the number of the nodes in the graph minus one. Each values
  /// of the map will be set exactly once, the values will be set descending
  /// order.
  /// \return %False when the graph is not DAG.
  ///
  /// \see topologicalSort
  /// \see dag
  template <typename Digraph, typename NodeMap>
  bool checkedTopologicalSort(const Digraph& digraph, NodeMap& order) {
    using namespace _connectivity_bits;

    checkConcept<concepts::Digraph, Digraph>();
    checkConcept<concepts::ReadWriteMap<typename Digraph::Node, int>,
      NodeMap>();

    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;

    for (NodeIt it(digraph); it != INVALID; ++it) {
      order.set(it, -1);
    }

    TopologicalSortVisitor<Digraph, NodeMap>
      visitor(order, countNodes(digraph));

    DfsVisit<Digraph, TopologicalSortVisitor<Digraph, NodeMap> >
      dfs(digraph, visitor);

    dfs.init();
    for (NodeIt it(digraph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        dfs.addSource(it);
        while (!dfs.emptyQueue()) {
           Arc arc = dfs.nextArc();
           Node target = digraph.target(arc);
           if (dfs.reached(target) && order[target] == -1) {
             return false;
           }
           dfs.processNextArc();
         }
      }
    }
    return true;
  }

  /// \ingroup connectivity
  ///
  /// \brief Check that the given directed graph is a DAG.
  ///
  /// Check that the given directed graph is a DAG. The DAG is
  /// an Directed Acyclic Digraph.
  /// \return %False when the graph is not DAG.
  /// \see acyclic
  template <typename Digraph>
  bool dag(const Digraph& digraph) {

    checkConcept<concepts::Digraph, Digraph>();

    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;

    typedef typename Digraph::template NodeMap<bool> ProcessedMap;

    typename Dfs<Digraph>::template SetProcessedMap<ProcessedMap>::
      Create dfs(digraph);

    ProcessedMap processed(digraph);
    dfs.processedMap(processed);

    dfs.init();
    for (NodeIt it(digraph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        dfs.addSource(it);
        while (!dfs.emptyQueue()) {
          Arc edge = dfs.nextArc();
          Node target = digraph.target(edge);
          if (dfs.reached(target) && !processed[target]) {
            return false;
          }
          dfs.processNextArc();
        }
      }
    }
    return true;
  }

  /// \ingroup connectivity
  ///
  /// \brief Check that the given undirected graph is acyclic.
  ///
  /// Check that the given undirected graph acyclic.
  /// \param graph The undirected graph.
  /// \return %True when there is no circle in the graph.
  /// \see dag
  template <typename Graph>
  bool acyclic(const Graph& graph) {
    checkConcept<concepts::Graph, Graph>();
    typedef typename Graph::Node Node;
    typedef typename Graph::NodeIt NodeIt;
    typedef typename Graph::Arc Arc;
    Dfs<Graph> dfs(graph);
    dfs.init();
    for (NodeIt it(graph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        dfs.addSource(it);
        while (!dfs.emptyQueue()) {
          Arc edge = dfs.nextArc();
          Node source = graph.source(edge);
          Node target = graph.target(edge);
          if (dfs.reached(target) &&
              dfs.predArc(source) != graph.oppositeArc(edge)) {
            return false;
          }
          dfs.processNextArc();
        }
      }
    }
    return true;
  }

  /// \ingroup connectivity
  ///
  /// \brief Check that the given undirected graph is tree.
  ///
  /// Check that the given undirected graph is tree.
  /// \param graph The undirected graph.
  /// \return %True when the graph is acyclic and connected.
  template <typename Graph>
  bool tree(const Graph& graph) {
    checkConcept<concepts::Graph, Graph>();
    typedef typename Graph::Node Node;
    typedef typename Graph::NodeIt NodeIt;
    typedef typename Graph::Arc Arc;
    Dfs<Graph> dfs(graph);
    dfs.init();
    dfs.addSource(NodeIt(graph));
    while (!dfs.emptyQueue()) {
      Arc edge = dfs.nextArc();
      Node source = graph.source(edge);
      Node target = graph.target(edge);
      if (dfs.reached(target) &&
          dfs.predArc(source) != graph.oppositeArc(edge)) {
        return false;
      }
      dfs.processNextArc();
    }
    for (NodeIt it(graph); it != INVALID; ++it) {
      if (!dfs.reached(it)) {
        return false;
      }
    }
    return true;
  }

  namespace _connectivity_bits {

    template <typename Digraph>
    class BipartiteVisitor : public BfsVisitor<Digraph> {
    public:
      typedef typename Digraph::Arc Arc;
      typedef typename Digraph::Node Node;

      BipartiteVisitor(const Digraph& graph, bool& bipartite)
        : _graph(graph), _part(graph), _bipartite(bipartite) {}

      void start(const Node& node) {
        _part[node] = true;
      }
      void discover(const Arc& edge) {
        _part.set(_graph.target(edge), !_part[_graph.source(edge)]);
      }
      void examine(const Arc& edge) {
        _bipartite = _bipartite &&
          _part[_graph.target(edge)] != _part[_graph.source(edge)];
      }

    private:

      const Digraph& _graph;
      typename Digraph::template NodeMap<bool> _part;
      bool& _bipartite;
    };

    template <typename Digraph, typename PartMap>
    class BipartitePartitionsVisitor : public BfsVisitor<Digraph> {
    public:
      typedef typename Digraph::Arc Arc;
      typedef typename Digraph::Node Node;

      BipartitePartitionsVisitor(const Digraph& graph,
                                 PartMap& part, bool& bipartite)
        : _graph(graph), _part(part), _bipartite(bipartite) {}

      void start(const Node& node) {
        _part.set(node, true);
      }
      void discover(const Arc& edge) {
        _part.set(_graph.target(edge), !_part[_graph.source(edge)]);
      }
      void examine(const Arc& edge) {
        _bipartite = _bipartite &&
          _part[_graph.target(edge)] != _part[_graph.source(edge)];
      }

    private:

      const Digraph& _graph;
      PartMap& _part;
      bool& _bipartite;
    };
  }

  /// \ingroup connectivity
  ///
  /// \brief Check if the given undirected graph is bipartite or not
  ///
  /// The function checks if the given undirected \c graph graph is bipartite
  /// or not. The \ref Bfs algorithm is used to calculate the result.
  /// \param graph The undirected graph.
  /// \return %True if \c graph is bipartite, %false otherwise.
  /// \sa bipartitePartitions
  template<typename Graph>
  inline bool bipartite(const Graph &graph){
    using namespace _connectivity_bits;

    checkConcept<concepts::Graph, Graph>();

    typedef typename Graph::NodeIt NodeIt;
    typedef typename Graph::ArcIt ArcIt;

    bool bipartite = true;

    BipartiteVisitor<Graph>
      visitor(graph, bipartite);
    BfsVisit<Graph, BipartiteVisitor<Graph> >
      bfs(graph, visitor);
    bfs.init();
    for(NodeIt it(graph); it != INVALID; ++it) {
      if(!bfs.reached(it)){
        bfs.addSource(it);
        while (!bfs.emptyQueue()) {
          bfs.processNextNode();
          if (!bipartite) return false;
        }
      }
    }
    return true;
  }

  /// \ingroup connectivity
  ///
  /// \brief Check if the given undirected graph is bipartite or not
  ///
  /// The function checks if the given undirected graph is bipartite
  /// or not. The  \ref  Bfs  algorithm  is   used  to  calculate the result.
  /// During the execution, the \c partMap will be set as the two
  /// partitions of the graph.
  /// \param graph The undirected graph.
  /// \retval partMap A writable bool map of nodes. It will be set as the
  /// two partitions of the graph.
  /// \return %True if \c graph is bipartite, %false otherwise.
  template<typename Graph, typename NodeMap>
  inline bool bipartitePartitions(const Graph &graph, NodeMap &partMap){
    using namespace _connectivity_bits;

    checkConcept<concepts::Graph, Graph>();

    typedef typename Graph::Node Node;
    typedef typename Graph::NodeIt NodeIt;
    typedef typename Graph::ArcIt ArcIt;

    bool bipartite = true;

    BipartitePartitionsVisitor<Graph, NodeMap>
      visitor(graph, partMap, bipartite);
    BfsVisit<Graph, BipartitePartitionsVisitor<Graph, NodeMap> >
      bfs(graph, visitor);
    bfs.init();
    for(NodeIt it(graph); it != INVALID; ++it) {
      if(!bfs.reached(it)){
        bfs.addSource(it);
        while (!bfs.emptyQueue()) {
          bfs.processNextNode();
          if (!bipartite) return false;
        }
      }
    }
    return true;
  }

  /// \brief Returns true when there are not loop edges in the graph.
  ///
  /// Returns true when there are not loop edges in the graph.
  template <typename Digraph>
  bool loopFree(const Digraph& digraph) {
    for (typename Digraph::ArcIt it(digraph); it != INVALID; ++it) {
      if (digraph.source(it) == digraph.target(it)) return false;
    }
    return true;
  }

  /// \brief Returns true when there are not parallel edges in the graph.
  ///
  /// Returns true when there are not parallel edges in the graph.
  template <typename Digraph>
  bool parallelFree(const Digraph& digraph) {
    typename Digraph::template NodeMap<bool> reached(digraph, false);
    for (typename Digraph::NodeIt n(digraph); n != INVALID; ++n) {
      for (typename Digraph::OutArcIt a(digraph, n); a != INVALID; ++a) {
        if (reached[digraph.target(a)]) return false;
        reached.set(digraph.target(a), true);
      }
      for (typename Digraph::OutArcIt a(digraph, n); a != INVALID; ++a) {
        reached.set(digraph.target(a), false);
      }
    }
    return true;
  }

  /// \brief Returns true when there are not loop edges and parallel
  /// edges in the graph.
  ///
  /// Returns true when there are not loop edges and parallel edges in
  /// the graph.
  template <typename Digraph>
  bool simpleDigraph(const Digraph& digraph) {
    typename Digraph::template NodeMap<bool> reached(digraph, false);
    for (typename Digraph::NodeIt n(digraph); n != INVALID; ++n) {
      reached.set(n, true);
      for (typename Digraph::OutArcIt a(digraph, n); a != INVALID; ++a) {
        if (reached[digraph.target(a)]) return false;
        reached.set(digraph.target(a), true);
      }
      for (typename Digraph::OutArcIt a(digraph, n); a != INVALID; ++a) {
        reached.set(digraph.target(a), false);
      }
      reached.set(n, false);
    }
    return true;
  }

} //namespace lemon

#endif //LEMON_CONNECTIVITY_H
