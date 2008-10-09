/* -*- mode: C++; indent-tabs-mode: nil; -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library.
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

///\ingroup graph_concepts
///\file
///\brief The concept of graph components.


#ifndef LEMON_CONCEPT_GRAPH_COMPONENTS_H
#define LEMON_CONCEPT_GRAPH_COMPONENTS_H

#include <lemon/core.h>
#include <lemon/concepts/maps.h>

#include <lemon/bits/alteration_notifier.h>

namespace lemon {
  namespace concepts {

    /// \brief Skeleton class for graph Node and Arc types
    ///
    /// This class describes the interface of Node and Arc (and Edge
    /// in undirected graphs) subtypes of graph types.
    ///
    /// \note This class is a template class so that we can use it to
    /// create graph skeleton classes. The reason for this is than Node
    /// and Arc types should \em not derive from the same base class.
    /// For Node you should instantiate it with character 'n' and for Arc
    /// with 'a'.

#ifndef DOXYGEN
    template <char _selector = '0'>
#endif
    class GraphItem {
    public:
      /// \brief Default constructor.
      ///
      /// \warning The default constructor is not required to set
      /// the item to some well-defined value. So you should consider it
      /// as uninitialized.
      GraphItem() {}
      /// \brief Copy constructor.
      ///
      /// Copy constructor.
      ///
      GraphItem(const GraphItem &) {}
      /// \brief Invalid constructor \& conversion.
      ///
      /// This constructor initializes the item to be invalid.
      /// \sa Invalid for more details.
      GraphItem(Invalid) {}
      /// \brief Assign operator for nodes.
      ///
      /// The nodes are assignable.
      ///
      GraphItem& operator=(GraphItem const&) { return *this; }
      /// \brief Equality operator.
      ///
      /// Two iterators are equal if and only if they represents the
      /// same node in the graph or both are invalid.
      bool operator==(GraphItem) const { return false; }
      /// \brief Inequality operator.
      ///
      /// \sa operator==(const Node& n)
      ///
      bool operator!=(GraphItem) const { return false; }

      /// \brief Artificial ordering operator.
      ///
      /// To allow the use of graph descriptors as key type in std::map or
      /// similar associative container we require this.
      ///
      /// \note This operator only have to define some strict ordering of
      /// the items; this order has nothing to do with the iteration
      /// ordering of the items.
      bool operator<(GraphItem) const { return false; }

      template<typename _GraphItem>
      struct Constraints {
        void constraints() {
          _GraphItem i1;
          _GraphItem i2 = i1;
          _GraphItem i3 = INVALID;

          i1 = i2 = i3;

          bool b;
          //          b = (ia == ib) && (ia != ib) && (ia < ib);
          b = (ia == ib) && (ia != ib);
          b = (ia == INVALID) && (ib != INVALID);
          b = (ia < ib);
        }

        const _GraphItem &ia;
        const _GraphItem &ib;
      };
    };

    /// \brief An empty base directed graph class.
    ///
    /// This class provides the minimal set of features needed for a
    /// directed graph structure. All digraph concepts have to be
    /// conform to this base directed graph. It just provides types
    /// for nodes and arcs and functions to get the source and the
    /// target of the arcs.
    class BaseDigraphComponent {
    public:

      typedef BaseDigraphComponent Digraph;

      /// \brief Node class of the digraph.
      ///
      /// This class represents the Nodes of the digraph.
      ///
      typedef GraphItem<'n'> Node;

      /// \brief Arc class of the digraph.
      ///
      /// This class represents the Arcs of the digraph.
      ///
      typedef GraphItem<'e'> Arc;

      /// \brief Gives back the target node of an arc.
      ///
      /// Gives back the target node of an arc.
      ///
      Node target(const Arc&) const { return INVALID;}

      /// \brief Gives back the source node of an arc.
      ///
      /// Gives back the source node of an arc.
      ///
      Node source(const Arc&) const { return INVALID;}

      /// \brief Gives back the opposite node on the given arc.
      ///
      /// Gives back the opposite node on the given arc.
      Node oppositeNode(const Node&, const Arc&) const {
        return INVALID;
      }

      template <typename _Digraph>
      struct Constraints {
        typedef typename _Digraph::Node Node;
        typedef typename _Digraph::Arc Arc;

        void constraints() {
          checkConcept<GraphItem<'n'>, Node>();
          checkConcept<GraphItem<'a'>, Arc>();
          {
            Node n;
            Arc e(INVALID);
            n = digraph.source(e);
            n = digraph.target(e);
            n = digraph.oppositeNode(n, e);
          }
        }

        const _Digraph& digraph;
      };
    };

    /// \brief An empty base undirected graph class.
    ///
    /// This class provides the minimal set of features needed for an
    /// undirected graph structure. All undirected graph concepts have
    /// to be conform to this base graph. It just provides types for
    /// nodes, arcs and edges and functions to get the
    /// source and the target of the arcs and edges,
    /// conversion from arcs to edges and function to get
    /// both direction of the edges.
    class BaseGraphComponent : public BaseDigraphComponent {
    public:
      typedef BaseDigraphComponent::Node Node;
      typedef BaseDigraphComponent::Arc Arc;
      /// \brief Undirected arc class of the graph.
      ///
      /// This class represents the edges of the graph.
      /// The undirected graphs can be used as a directed graph which
      /// for each arc contains the opposite arc too so the graph is
      /// bidirected. The edge represents two opposite
      /// directed arcs.
      class Edge : public GraphItem<'u'> {
      public:
        typedef GraphItem<'u'> Parent;
        /// \brief Default constructor.
        ///
        /// \warning The default constructor is not required to set
        /// the item to some well-defined value. So you should consider it
        /// as uninitialized.
        Edge() {}
        /// \brief Copy constructor.
        ///
        /// Copy constructor.
        ///
        Edge(const Edge &) : Parent() {}
        /// \brief Invalid constructor \& conversion.
        ///
        /// This constructor initializes the item to be invalid.
        /// \sa Invalid for more details.
        Edge(Invalid) {}
        /// \brief Converter from arc to edge.
        ///
        /// Besides the core graph item functionality each arc should
        /// be convertible to the represented edge.
        Edge(const Arc&) {}
        /// \brief Assign arc to edge.
        ///
        /// Besides the core graph item functionality each arc should
        /// be convertible to the represented edge.
        Edge& operator=(const Arc&) { return *this; }
      };

      /// \brief Returns the direction of the arc.
      ///
      /// Returns the direction of the arc. Each arc represents an
      /// edge with a direction. It gives back the
      /// direction.
      bool direction(const Arc&) const { return true; }

      /// \brief Returns the directed arc.
      ///
      /// Returns the directed arc from its direction and the
      /// represented edge.
      Arc direct(const Edge&, bool) const { return INVALID;}

      /// \brief Returns the directed arc.
      ///
      /// Returns the directed arc from its source and the
      /// represented edge.
      Arc direct(const Edge&, const Node&) const { return INVALID;}

      /// \brief Returns the opposite arc.
      ///
      /// Returns the opposite arc. It is the arc representing the
      /// same edge and has opposite direction.
      Arc oppositeArc(const Arc&) const { return INVALID;}

      /// \brief Gives back one ending of an edge.
      ///
      /// Gives back one ending of an edge.
      Node u(const Edge&) const { return INVALID;}

      /// \brief Gives back the other ending of an edge.
      ///
      /// Gives back the other ending of an edge.
      Node v(const Edge&) const { return INVALID;}

      template <typename _Graph>
      struct Constraints {
        typedef typename _Graph::Node Node;
        typedef typename _Graph::Arc Arc;
        typedef typename _Graph::Edge Edge;

        void constraints() {
          checkConcept<BaseDigraphComponent, _Graph>();
          checkConcept<GraphItem<'u'>, Edge>();
          {
            Node n;
            Edge ue(INVALID);
            Arc e;
            n = graph.u(ue);
            n = graph.v(ue);
            e = graph.direct(ue, true);
            e = graph.direct(ue, n);
            e = graph.oppositeArc(e);
            ue = e;
            bool d = graph.direction(e);
            ignore_unused_variable_warning(d);
          }
        }

        const _Graph& graph;
      };

    };

    /// \brief An empty idable base digraph class.
    ///
    /// This class provides beside the core digraph features
    /// core id functions for the digraph structure.
    /// The most of the base digraphs should be conform to this concept.
    /// The id's are unique and immutable.
    template <typename _Base = BaseDigraphComponent>
    class IDableDigraphComponent : public _Base {
    public:

      typedef _Base Base;
      typedef typename Base::Node Node;
      typedef typename Base::Arc Arc;

      /// \brief Gives back an unique integer id for the Node.
      ///
      /// Gives back an unique integer id for the Node.
      ///
      int id(const Node&) const { return -1;}

      /// \brief Gives back the node by the unique id.
      ///
      /// Gives back the node by the unique id.
      /// If the digraph does not contain node with the given id
      /// then the result of the function is undetermined.
      Node nodeFromId(int) const { return INVALID;}

      /// \brief Gives back an unique integer id for the Arc.
      ///
      /// Gives back an unique integer id for the Arc.
      ///
      int id(const Arc&) const { return -1;}

      /// \brief Gives back the arc by the unique id.
      ///
      /// Gives back the arc by the unique id.
      /// If the digraph does not contain arc with the given id
      /// then the result of the function is undetermined.
      Arc arcFromId(int) const { return INVALID;}

      /// \brief Gives back an integer greater or equal to the maximum
      /// Node id.
      ///
      /// Gives back an integer greater or equal to the maximum Node
      /// id.
      int maxNodeId() const { return -1;}

      /// \brief Gives back an integer greater or equal to the maximum
      /// Arc id.
      ///
      /// Gives back an integer greater or equal to the maximum Arc
      /// id.
      int maxArcId() const { return -1;}

      template <typename _Digraph>
      struct Constraints {

        void constraints() {
          checkConcept<Base, _Digraph >();
          typename _Digraph::Node node;
          int nid = digraph.id(node);
          nid = digraph.id(node);
          node = digraph.nodeFromId(nid);
          typename _Digraph::Arc arc;
          int eid = digraph.id(arc);
          eid = digraph.id(arc);
          arc = digraph.arcFromId(eid);

          nid = digraph.maxNodeId();
          ignore_unused_variable_warning(nid);
          eid = digraph.maxArcId();
          ignore_unused_variable_warning(eid);
        }

        const _Digraph& digraph;
      };
    };

    /// \brief An empty idable base undirected graph class.
    ///
    /// This class provides beside the core undirected graph features
    /// core id functions for the undirected graph structure.  The
    /// most of the base undirected graphs should be conform to this
    /// concept.  The id's are unique and immutable.
    template <typename _Base = BaseGraphComponent>
    class IDableGraphComponent : public IDableDigraphComponent<_Base> {
    public:

      typedef _Base Base;
      typedef typename Base::Edge Edge;

      using IDableDigraphComponent<_Base>::id;

      /// \brief Gives back an unique integer id for the Edge.
      ///
      /// Gives back an unique integer id for the Edge.
      ///
      int id(const Edge&) const { return -1;}

      /// \brief Gives back the edge by the unique id.
      ///
      /// Gives back the edge by the unique id.  If the
      /// graph does not contain arc with the given id then the
      /// result of the function is undetermined.
      Edge edgeFromId(int) const { return INVALID;}

      /// \brief Gives back an integer greater or equal to the maximum
      /// Edge id.
      ///
      /// Gives back an integer greater or equal to the maximum Edge
      /// id.
      int maxEdgeId() const { return -1;}

      template <typename _Graph>
      struct Constraints {

        void constraints() {
          checkConcept<Base, _Graph >();
          checkConcept<IDableDigraphComponent<Base>, _Graph >();
          typename _Graph::Edge edge;
          int ueid = graph.id(edge);
          ueid = graph.id(edge);
          edge = graph.edgeFromId(ueid);
          ueid = graph.maxEdgeId();
          ignore_unused_variable_warning(ueid);
        }

        const _Graph& graph;
      };
    };

    /// \brief Skeleton class for graph NodeIt and ArcIt
    ///
    /// Skeleton class for graph NodeIt and ArcIt.
    ///
    template <typename _Graph, typename _Item>
    class GraphItemIt : public _Item {
    public:
      /// \brief Default constructor.
      ///
      /// @warning The default constructor sets the iterator
      /// to an undefined value.
      GraphItemIt() {}
      /// \brief Copy constructor.
      ///
      /// Copy constructor.
      ///
      GraphItemIt(const GraphItemIt& ) {}
      /// \brief Sets the iterator to the first item.
      ///
      /// Sets the iterator to the first item of \c the graph.
      ///
      explicit GraphItemIt(const _Graph&) {}
      /// \brief Invalid constructor \& conversion.
      ///
      /// This constructor initializes the item to be invalid.
      /// \sa Invalid for more details.
      GraphItemIt(Invalid) {}
      /// \brief Assign operator for items.
      ///
      /// The items are assignable.
      ///
      GraphItemIt& operator=(const GraphItemIt&) { return *this; }
      /// \brief Next item.
      ///
      /// Assign the iterator to the next item.
      ///
      GraphItemIt& operator++() { return *this; }
      /// \brief Equality operator
      ///
      /// Two iterators are equal if and only if they point to the
      /// same object or both are invalid.
      bool operator==(const GraphItemIt&) const { return true;}
      /// \brief Inequality operator
      ///
      /// \sa operator==(Node n)
      ///
      bool operator!=(const GraphItemIt&) const { return true;}

      template<typename _GraphItemIt>
      struct Constraints {
        void constraints() {
          _GraphItemIt it1(g);
          _GraphItemIt it2;

          it2 = ++it1;
          ++it2 = it1;
          ++(++it1);

          _Item bi = it1;
          bi = it2;
        }
        _Graph& g;
      };
    };

    /// \brief Skeleton class for graph InArcIt and OutArcIt
    ///
    /// \note Because InArcIt and OutArcIt may not inherit from the same
    /// base class, the _selector is a additional template parameter. For
    /// InArcIt you should instantiate it with character 'i' and for
    /// OutArcIt with 'o'.
    template <typename _Graph,
              typename _Item = typename _Graph::Arc,
              typename _Base = typename _Graph::Node,
              char _selector = '0'>
    class GraphIncIt : public _Item {
    public:
      /// \brief Default constructor.
      ///
      /// @warning The default constructor sets the iterator
      /// to an undefined value.
      GraphIncIt() {}
      /// \brief Copy constructor.
      ///
      /// Copy constructor.
      ///
      GraphIncIt(GraphIncIt const& gi) : _Item(gi) {}
      /// \brief Sets the iterator to the first arc incoming into or outgoing
      /// from the node.
      ///
      /// Sets the iterator to the first arc incoming into or outgoing
      /// from the node.
      ///
      explicit GraphIncIt(const _Graph&, const _Base&) {}
      /// \brief Invalid constructor \& conversion.
      ///
      /// This constructor initializes the item to be invalid.
      /// \sa Invalid for more details.
      GraphIncIt(Invalid) {}
      /// \brief Assign operator for iterators.
      ///
      /// The iterators are assignable.
      ///
      GraphIncIt& operator=(GraphIncIt const&) { return *this; }
      /// \brief Next item.
      ///
      /// Assign the iterator to the next item.
      ///
      GraphIncIt& operator++() { return *this; }

      /// \brief Equality operator
      ///
      /// Two iterators are equal if and only if they point to the
      /// same object or both are invalid.
      bool operator==(const GraphIncIt&) const { return true;}

      /// \brief Inequality operator
      ///
      /// \sa operator==(Node n)
      ///
      bool operator!=(const GraphIncIt&) const { return true;}

      template <typename _GraphIncIt>
      struct Constraints {
        void constraints() {
          checkConcept<GraphItem<_selector>, _GraphIncIt>();
          _GraphIncIt it1(graph, node);
          _GraphIncIt it2;

          it2 = ++it1;
          ++it2 = it1;
          ++(++it1);
          _Item e = it1;
          e = it2;

        }

        _Item arc;
        _Base node;
        _Graph graph;
        _GraphIncIt it;
      };
    };


    /// \brief An empty iterable digraph class.
    ///
    /// This class provides beside the core digraph features
    /// iterator based iterable interface for the digraph structure.
    /// This concept is part of the Digraph concept.
    template <typename _Base = BaseDigraphComponent>
    class IterableDigraphComponent : public _Base {

    public:

      typedef _Base Base;
      typedef typename Base::Node Node;
      typedef typename Base::Arc Arc;

      typedef IterableDigraphComponent Digraph;

      /// \name Base iteration
      ///
      /// This interface provides functions for iteration on digraph items
      ///
      /// @{

      /// \brief Gives back the first node in the iterating order.
      ///
      /// Gives back the first node in the iterating order.
      ///
      void first(Node&) const {}

      /// \brief Gives back the next node in the iterating order.
      ///
      /// Gives back the next node in the iterating order.
      ///
      void next(Node&) const {}

      /// \brief Gives back the first arc in the iterating order.
      ///
      /// Gives back the first arc in the iterating order.
      ///
      void first(Arc&) const {}

      /// \brief Gives back the next arc in the iterating order.
      ///
      /// Gives back the next arc in the iterating order.
      ///
      void next(Arc&) const {}


      /// \brief Gives back the first of the arcs point to the given
      /// node.
      ///
      /// Gives back the first of the arcs point to the given node.
      ///
      void firstIn(Arc&, const Node&) const {}

      /// \brief Gives back the next of the arcs points to the given
      /// node.
      ///
      /// Gives back the next of the arcs points to the given node.
      ///
      void nextIn(Arc&) const {}

      /// \brief Gives back the first of the arcs start from the
      /// given node.
      ///
      /// Gives back the first of the arcs start from the given node.
      ///
      void firstOut(Arc&, const Node&) const {}

      /// \brief Gives back the next of the arcs start from the given
      /// node.
      ///
      /// Gives back the next of the arcs start from the given node.
      ///
      void nextOut(Arc&) const {}

      /// @}

      /// \name Class based iteration
      ///
      /// This interface provides functions for iteration on digraph items
      ///
      /// @{

      /// \brief This iterator goes through each node.
      ///
      /// This iterator goes through each node.
      ///
      typedef GraphItemIt<Digraph, Node> NodeIt;

      /// \brief This iterator goes through each node.
      ///
      /// This iterator goes through each node.
      ///
      typedef GraphItemIt<Digraph, Arc> ArcIt;

      /// \brief This iterator goes trough the incoming arcs of a node.
      ///
      /// This iterator goes trough the \e inccoming arcs of a certain node
      /// of a digraph.
      typedef GraphIncIt<Digraph, Arc, Node, 'i'> InArcIt;

      /// \brief This iterator goes trough the outgoing arcs of a node.
      ///
      /// This iterator goes trough the \e outgoing arcs of a certain node
      /// of a digraph.
      typedef GraphIncIt<Digraph, Arc, Node, 'o'> OutArcIt;

      /// \brief The base node of the iterator.
      ///
      /// Gives back the base node of the iterator.
      /// It is always the target of the pointed arc.
      Node baseNode(const InArcIt&) const { return INVALID; }

      /// \brief The running node of the iterator.
      ///
      /// Gives back the running node of the iterator.
      /// It is always the source of the pointed arc.
      Node runningNode(const InArcIt&) const { return INVALID; }

      /// \brief The base node of the iterator.
      ///
      /// Gives back the base node of the iterator.
      /// It is always the source of the pointed arc.
      Node baseNode(const OutArcIt&) const { return INVALID; }

      /// \brief The running node of the iterator.
      ///
      /// Gives back the running node of the iterator.
      /// It is always the target of the pointed arc.
      Node runningNode(const OutArcIt&) const { return INVALID; }

      /// @}

      template <typename _Digraph>
      struct Constraints {
        void constraints() {
          checkConcept<Base, _Digraph>();

          {
            typename _Digraph::Node node(INVALID);
            typename _Digraph::Arc arc(INVALID);
            {
              digraph.first(node);
              digraph.next(node);
            }
            {
              digraph.first(arc);
              digraph.next(arc);
            }
            {
              digraph.firstIn(arc, node);
              digraph.nextIn(arc);
            }
            {
              digraph.firstOut(arc, node);
              digraph.nextOut(arc);
            }
          }

          {
            checkConcept<GraphItemIt<_Digraph, typename _Digraph::Arc>,
              typename _Digraph::ArcIt >();
            checkConcept<GraphItemIt<_Digraph, typename _Digraph::Node>,
              typename _Digraph::NodeIt >();
            checkConcept<GraphIncIt<_Digraph, typename _Digraph::Arc,
              typename _Digraph::Node, 'i'>, typename _Digraph::InArcIt>();
            checkConcept<GraphIncIt<_Digraph, typename _Digraph::Arc,
              typename _Digraph::Node, 'o'>, typename _Digraph::OutArcIt>();

            typename _Digraph::Node n;
            typename _Digraph::InArcIt ieit(INVALID);
            typename _Digraph::OutArcIt oeit(INVALID);
            n = digraph.baseNode(ieit);
            n = digraph.runningNode(ieit);
            n = digraph.baseNode(oeit);
            n = digraph.runningNode(oeit);
            ignore_unused_variable_warning(n);
          }
        }

        const _Digraph& digraph;

      };
    };

    /// \brief An empty iterable undirected graph class.
    ///
    /// This class provides beside the core graph features iterator
    /// based iterable interface for the undirected graph structure.
    /// This concept is part of the Graph concept.
    template <typename _Base = BaseGraphComponent>
    class IterableGraphComponent : public IterableDigraphComponent<_Base> {
    public:

      typedef _Base Base;
      typedef typename Base::Node Node;
      typedef typename Base::Arc Arc;
      typedef typename Base::Edge Edge;


      typedef IterableGraphComponent Graph;

      /// \name Base iteration
      ///
      /// This interface provides functions for iteration on graph items
      /// @{

      using IterableDigraphComponent<_Base>::first;
      using IterableDigraphComponent<_Base>::next;

      /// \brief Gives back the first edge in the iterating
      /// order.
      ///
      /// Gives back the first edge in the iterating order.
      ///
      void first(Edge&) const {}

      /// \brief Gives back the next edge in the iterating
      /// order.
      ///
      /// Gives back the next edge in the iterating order.
      ///
      void next(Edge&) const {}


      /// \brief Gives back the first of the edges from the
      /// given node.
      ///
      /// Gives back the first of the edges from the given
      /// node. The bool parameter gives back that direction which
      /// gives a good direction of the edge so the source of the
      /// directed arc is the given node.
      void firstInc(Edge&, bool&, const Node&) const {}

      /// \brief Gives back the next of the edges from the
      /// given node.
      ///
      /// Gives back the next of the edges from the given
      /// node. The bool parameter should be used as the \c firstInc()
      /// use it.
      void nextInc(Edge&, bool&) const {}

      using IterableDigraphComponent<_Base>::baseNode;
      using IterableDigraphComponent<_Base>::runningNode;

      /// @}

      /// \name Class based iteration
      ///
      /// This interface provides functions for iteration on graph items
      ///
      /// @{

      /// \brief This iterator goes through each node.
      ///
      /// This iterator goes through each node.
      typedef GraphItemIt<Graph, Edge> EdgeIt;
      /// \brief This iterator goes trough the incident arcs of a
      /// node.
      ///
      /// This iterator goes trough the incident arcs of a certain
      /// node of a graph.
      typedef GraphIncIt<Graph, Edge, Node, 'u'> IncEdgeIt;
      /// \brief The base node of the iterator.
      ///
      /// Gives back the base node of the iterator.
      Node baseNode(const IncEdgeIt&) const { return INVALID; }

      /// \brief The running node of the iterator.
      ///
      /// Gives back the running node of the iterator.
      Node runningNode(const IncEdgeIt&) const { return INVALID; }

      /// @}

      template <typename _Graph>
      struct Constraints {
        void constraints() {
          checkConcept<IterableDigraphComponent<Base>, _Graph>();

          {
            typename _Graph::Node node(INVALID);
            typename _Graph::Edge edge(INVALID);
            bool dir;
            {
              graph.first(edge);
              graph.next(edge);
            }
            {
              graph.firstInc(edge, dir, node);
              graph.nextInc(edge, dir);
            }

          }

          {
            checkConcept<GraphItemIt<_Graph, typename _Graph::Edge>,
              typename _Graph::EdgeIt >();
            checkConcept<GraphIncIt<_Graph, typename _Graph::Edge,
              typename _Graph::Node, 'u'>, typename _Graph::IncEdgeIt>();

            typename _Graph::Node n;
            typename _Graph::IncEdgeIt ueit(INVALID);
            n = graph.baseNode(ueit);
            n = graph.runningNode(ueit);
          }
        }

        const _Graph& graph;

      };
    };

    /// \brief An empty alteration notifier digraph class.
    ///
    /// This class provides beside the core digraph features alteration
    /// notifier interface for the digraph structure.  This implements
    /// an observer-notifier pattern for each digraph item. More
    /// obsevers can be registered into the notifier and whenever an
    /// alteration occured in the digraph all the observers will
    /// notified about it.
    template <typename _Base = BaseDigraphComponent>
    class AlterableDigraphComponent : public _Base {
    public:

      typedef _Base Base;
      typedef typename Base::Node Node;
      typedef typename Base::Arc Arc;


      /// The node observer registry.
      typedef AlterationNotifier<AlterableDigraphComponent, Node>
      NodeNotifier;
      /// The arc observer registry.
      typedef AlterationNotifier<AlterableDigraphComponent, Arc>
      ArcNotifier;

      /// \brief Gives back the node alteration notifier.
      ///
      /// Gives back the node alteration notifier.
      NodeNotifier& notifier(Node) const {
        return NodeNotifier();
      }

      /// \brief Gives back the arc alteration notifier.
      ///
      /// Gives back the arc alteration notifier.
      ArcNotifier& notifier(Arc) const {
        return ArcNotifier();
      }

      template <typename _Digraph>
      struct Constraints {
        void constraints() {
          checkConcept<Base, _Digraph>();
          typename _Digraph::NodeNotifier& nn
            = digraph.notifier(typename _Digraph::Node());

          typename _Digraph::ArcNotifier& en
            = digraph.notifier(typename _Digraph::Arc());

          ignore_unused_variable_warning(nn);
          ignore_unused_variable_warning(en);
        }

        const _Digraph& digraph;

      };

    };

    /// \brief An empty alteration notifier undirected graph class.
    ///
    /// This class provides beside the core graph features alteration
    /// notifier interface for the graph structure.  This implements
    /// an observer-notifier pattern for each graph item. More
    /// obsevers can be registered into the notifier and whenever an
    /// alteration occured in the graph all the observers will
    /// notified about it.
    template <typename _Base = BaseGraphComponent>
    class AlterableGraphComponent : public AlterableDigraphComponent<_Base> {
    public:

      typedef _Base Base;
      typedef typename Base::Edge Edge;


      /// The arc observer registry.
      typedef AlterationNotifier<AlterableGraphComponent, Edge>
      EdgeNotifier;

      /// \brief Gives back the arc alteration notifier.
      ///
      /// Gives back the arc alteration notifier.
      EdgeNotifier& notifier(Edge) const {
        return EdgeNotifier();
      }

      template <typename _Graph>
      struct Constraints {
        void constraints() {
          checkConcept<AlterableGraphComponent<Base>, _Graph>();
          typename _Graph::EdgeNotifier& uen
            = graph.notifier(typename _Graph::Edge());
          ignore_unused_variable_warning(uen);
        }

        const _Graph& graph;

      };

    };

    /// \brief Class describing the concept of graph maps
    ///
    /// This class describes the common interface of the graph maps
    /// (NodeMap, ArcMap), that is maps that can be used to
    /// associate data to graph descriptors (nodes or arcs).
    template <typename _Graph, typename _Item, typename _Value>
    class GraphMap : public ReadWriteMap<_Item, _Value> {
    public:

      typedef ReadWriteMap<_Item, _Value> Parent;

      /// The graph type of the map.
      typedef _Graph Graph;
      /// The key type of the map.
      typedef _Item Key;
      /// The value type of the map.
      typedef _Value Value;

      /// \brief Construct a new map.
      ///
      /// Construct a new map for the graph.
      explicit GraphMap(const Graph&) {}
      /// \brief Construct a new map with default value.
      ///
      /// Construct a new map for the graph and initalise the values.
      GraphMap(const Graph&, const Value&) {}

    private:
      /// \brief Copy constructor.
      ///
      /// Copy Constructor.
      GraphMap(const GraphMap&) : Parent() {}

      /// \brief Assign operator.
      ///
      /// Assign operator. It does not mofify the underlying graph,
      /// it just iterates on the current item set and set the  map
      /// with the value returned by the assigned map.
      template <typename CMap>
      GraphMap& operator=(const CMap&) {
        checkConcept<ReadMap<Key, Value>, CMap>();
        return *this;
      }

    public:
      template<typename _Map>
      struct Constraints {
        void constraints() {
          checkConcept<ReadWriteMap<Key, Value>, _Map >();
          // Construction with a graph parameter
          _Map a(g);
          // Constructor with a graph and a default value parameter
          _Map a2(g,t);
          // Copy constructor.
          // _Map b(c);

          // ReadMap<Key, Value> cmap;
          // b = cmap;

          ignore_unused_variable_warning(a);
          ignore_unused_variable_warning(a2);
          // ignore_unused_variable_warning(b);
        }

        const _Map &c;
        const Graph &g;
        const typename GraphMap::Value &t;
      };

    };

    /// \brief An empty mappable digraph class.
    ///
    /// This class provides beside the core digraph features
    /// map interface for the digraph structure.
    /// This concept is part of the Digraph concept.
    template <typename _Base = BaseDigraphComponent>
    class MappableDigraphComponent : public _Base  {
    public:

      typedef _Base Base;
      typedef typename Base::Node Node;
      typedef typename Base::Arc Arc;

      typedef MappableDigraphComponent Digraph;

      /// \brief ReadWrite map of the nodes.
      ///
      /// ReadWrite map of the nodes.
      ///
      template <typename _Value>
      class NodeMap : public GraphMap<Digraph, Node, _Value> {
      public:
        typedef GraphMap<MappableDigraphComponent, Node, _Value> Parent;

        /// \brief Construct a new map.
        ///
        /// Construct a new map for the digraph.
        explicit NodeMap(const MappableDigraphComponent& digraph)
          : Parent(digraph) {}

        /// \brief Construct a new map with default value.
        ///
        /// Construct a new map for the digraph and initalise the values.
        NodeMap(const MappableDigraphComponent& digraph, const _Value& value)
          : Parent(digraph, value) {}

      private:
        /// \brief Copy constructor.
        ///
        /// Copy Constructor.
        NodeMap(const NodeMap& nm) : Parent(nm) {}

        /// \brief Assign operator.
        ///
        /// Assign operator.
        template <typename CMap>
        NodeMap& operator=(const CMap&) {
          checkConcept<ReadMap<Node, _Value>, CMap>();
          return *this;
        }

      };

      /// \brief ReadWrite map of the arcs.
      ///
      /// ReadWrite map of the arcs.
      ///
      template <typename _Value>
      class ArcMap : public GraphMap<Digraph, Arc, _Value> {
      public:
        typedef GraphMap<MappableDigraphComponent, Arc, _Value> Parent;

        /// \brief Construct a new map.
        ///
        /// Construct a new map for the digraph.
        explicit ArcMap(const MappableDigraphComponent& digraph)
          : Parent(digraph) {}

        /// \brief Construct a new map with default value.
        ///
        /// Construct a new map for the digraph and initalise the values.
        ArcMap(const MappableDigraphComponent& digraph, const _Value& value)
          : Parent(digraph, value) {}

      private:
        /// \brief Copy constructor.
        ///
        /// Copy Constructor.
        ArcMap(const ArcMap& nm) : Parent(nm) {}

        /// \brief Assign operator.
        ///
        /// Assign operator.
        template <typename CMap>
        ArcMap& operator=(const CMap&) {
          checkConcept<ReadMap<Arc, _Value>, CMap>();
          return *this;
        }

      };


      template <typename _Digraph>
      struct Constraints {

        struct Dummy {
          int value;
          Dummy() : value(0) {}
          Dummy(int _v) : value(_v) {}
        };

        void constraints() {
          checkConcept<Base, _Digraph>();
          { // int map test
            typedef typename _Digraph::template NodeMap<int> IntNodeMap;
            checkConcept<GraphMap<_Digraph, typename _Digraph::Node, int>,
              IntNodeMap >();
          } { // bool map test
            typedef typename _Digraph::template NodeMap<bool> BoolNodeMap;
            checkConcept<GraphMap<_Digraph, typename _Digraph::Node, bool>,
              BoolNodeMap >();
          } { // Dummy map test
            typedef typename _Digraph::template NodeMap<Dummy> DummyNodeMap;
            checkConcept<GraphMap<_Digraph, typename _Digraph::Node, Dummy>,
              DummyNodeMap >();
          }

          { // int map test
            typedef typename _Digraph::template ArcMap<int> IntArcMap;
            checkConcept<GraphMap<_Digraph, typename _Digraph::Arc, int>,
              IntArcMap >();
          } { // bool map test
            typedef typename _Digraph::template ArcMap<bool> BoolArcMap;
            checkConcept<GraphMap<_Digraph, typename _Digraph::Arc, bool>,
              BoolArcMap >();
          } { // Dummy map test
            typedef typename _Digraph::template ArcMap<Dummy> DummyArcMap;
            checkConcept<GraphMap<_Digraph, typename _Digraph::Arc, Dummy>,
              DummyArcMap >();
          }
        }

        _Digraph& digraph;
      };
    };

    /// \brief An empty mappable base bipartite graph class.
    ///
    /// This class provides beside the core graph features
    /// map interface for the graph structure.
    /// This concept is part of the Graph concept.
    template <typename _Base = BaseGraphComponent>
    class MappableGraphComponent : public MappableDigraphComponent<_Base>  {
    public:

      typedef _Base Base;
      typedef typename Base::Edge Edge;

      typedef MappableGraphComponent Graph;

      /// \brief ReadWrite map of the edges.
      ///
      /// ReadWrite map of the edges.
      ///
      template <typename _Value>
      class EdgeMap : public GraphMap<Graph, Edge, _Value> {
      public:
        typedef GraphMap<MappableGraphComponent, Edge, _Value> Parent;

        /// \brief Construct a new map.
        ///
        /// Construct a new map for the graph.
        explicit EdgeMap(const MappableGraphComponent& graph)
          : Parent(graph) {}

        /// \brief Construct a new map with default value.
        ///
        /// Construct a new map for the graph and initalise the values.
        EdgeMap(const MappableGraphComponent& graph, const _Value& value)
          : Parent(graph, value) {}

      private:
        /// \brief Copy constructor.
        ///
        /// Copy Constructor.
        EdgeMap(const EdgeMap& nm) : Parent(nm) {}

        /// \brief Assign operator.
        ///
        /// Assign operator.
        template <typename CMap>
        EdgeMap& operator=(const CMap&) {
          checkConcept<ReadMap<Edge, _Value>, CMap>();
          return *this;
        }

      };


      template <typename _Graph>
      struct Constraints {

        struct Dummy {
          int value;
          Dummy() : value(0) {}
          Dummy(int _v) : value(_v) {}
        };

        void constraints() {
          checkConcept<MappableGraphComponent<Base>, _Graph>();

          { // int map test
            typedef typename _Graph::template EdgeMap<int> IntEdgeMap;
            checkConcept<GraphMap<_Graph, typename _Graph::Edge, int>,
              IntEdgeMap >();
          } { // bool map test
            typedef typename _Graph::template EdgeMap<bool> BoolEdgeMap;
            checkConcept<GraphMap<_Graph, typename _Graph::Edge, bool>,
              BoolEdgeMap >();
          } { // Dummy map test
            typedef typename _Graph::template EdgeMap<Dummy> DummyEdgeMap;
            checkConcept<GraphMap<_Graph, typename _Graph::Edge, Dummy>,
              DummyEdgeMap >();
          }
        }

        _Graph& graph;
      };
    };

    /// \brief An empty extendable digraph class.
    ///
    /// This class provides beside the core digraph features digraph
    /// extendable interface for the digraph structure.  The main
    /// difference between the base and this interface is that the
    /// digraph alterations should handled already on this level.
    template <typename _Base = BaseDigraphComponent>
    class ExtendableDigraphComponent : public _Base {
    public:
      typedef _Base Base;

      typedef typename _Base::Node Node;
      typedef typename _Base::Arc Arc;

      /// \brief Adds a new node to the digraph.
      ///
      /// Adds a new node to the digraph.
      ///
      Node addNode() {
        return INVALID;
      }

      /// \brief Adds a new arc connects the given two nodes.
      ///
      /// Adds a new arc connects the the given two nodes.
      Arc addArc(const Node&, const Node&) {
        return INVALID;
      }

      template <typename _Digraph>
      struct Constraints {
        void constraints() {
          checkConcept<Base, _Digraph>();
          typename _Digraph::Node node_a, node_b;
          node_a = digraph.addNode();
          node_b = digraph.addNode();
          typename _Digraph::Arc arc;
          arc = digraph.addArc(node_a, node_b);
        }

        _Digraph& digraph;
      };
    };

    /// \brief An empty extendable base undirected graph class.
    ///
    /// This class provides beside the core undirected graph features
    /// core undircted graph extend interface for the graph structure.
    /// The main difference between the base and this interface is
    /// that the graph alterations should handled already on this
    /// level.
    template <typename _Base = BaseGraphComponent>
    class ExtendableGraphComponent : public _Base {
    public:

      typedef _Base Base;
      typedef typename _Base::Node Node;
      typedef typename _Base::Edge Edge;

      /// \brief Adds a new node to the graph.
      ///
      /// Adds a new node to the graph.
      ///
      Node addNode() {
        return INVALID;
      }

      /// \brief Adds a new arc connects the given two nodes.
      ///
      /// Adds a new arc connects the the given two nodes.
      Edge addArc(const Node&, const Node&) {
        return INVALID;
      }

      template <typename _Graph>
      struct Constraints {
        void constraints() {
          checkConcept<Base, _Graph>();
          typename _Graph::Node node_a, node_b;
          node_a = graph.addNode();
          node_b = graph.addNode();
          typename _Graph::Edge edge;
          edge = graph.addEdge(node_a, node_b);
        }

        _Graph& graph;
      };
    };

    /// \brief An empty erasable digraph class.
    ///
    /// This class provides beside the core digraph features core erase
    /// functions for the digraph structure. The main difference between
    /// the base and this interface is that the digraph alterations
    /// should handled already on this level.
    template <typename _Base = BaseDigraphComponent>
    class ErasableDigraphComponent : public _Base {
    public:

      typedef _Base Base;
      typedef typename Base::Node Node;
      typedef typename Base::Arc Arc;

      /// \brief Erase a node from the digraph.
      ///
      /// Erase a node from the digraph. This function should
      /// erase all arcs connecting to the node.
      void erase(const Node&) {}

      /// \brief Erase an arc from the digraph.
      ///
      /// Erase an arc from the digraph.
      ///
      void erase(const Arc&) {}

      template <typename _Digraph>
      struct Constraints {
        void constraints() {
          checkConcept<Base, _Digraph>();
          typename _Digraph::Node node;
          digraph.erase(node);
          typename _Digraph::Arc arc;
          digraph.erase(arc);
        }

        _Digraph& digraph;
      };
    };

    /// \brief An empty erasable base undirected graph class.
    ///
    /// This class provides beside the core undirected graph features
    /// core erase functions for the undirceted graph structure. The
    /// main difference between the base and this interface is that
    /// the graph alterations should handled already on this level.
    template <typename _Base = BaseGraphComponent>
    class ErasableGraphComponent : public _Base {
    public:

      typedef _Base Base;
      typedef typename Base::Node Node;
      typedef typename Base::Edge Edge;

      /// \brief Erase a node from the graph.
      ///
      /// Erase a node from the graph. This function should erase
      /// arcs connecting to the node.
      void erase(const Node&) {}

      /// \brief Erase an arc from the graph.
      ///
      /// Erase an arc from the graph.
      ///
      void erase(const Edge&) {}

      template <typename _Graph>
      struct Constraints {
        void constraints() {
          checkConcept<Base, _Graph>();
          typename _Graph::Node node;
          graph.erase(node);
          typename _Graph::Edge edge;
          graph.erase(edge);
        }

        _Graph& graph;
      };
    };

    /// \brief An empty clearable base digraph class.
    ///
    /// This class provides beside the core digraph features core clear
    /// functions for the digraph structure. The main difference between
    /// the base and this interface is that the digraph alterations
    /// should handled already on this level.
    template <typename _Base = BaseDigraphComponent>
    class ClearableDigraphComponent : public _Base {
    public:

      typedef _Base Base;

      /// \brief Erase all nodes and arcs from the digraph.
      ///
      /// Erase all nodes and arcs from the digraph.
      ///
      void clear() {}

      template <typename _Digraph>
      struct Constraints {
        void constraints() {
          checkConcept<Base, _Digraph>();
          digraph.clear();
        }

        _Digraph digraph;
      };
    };

    /// \brief An empty clearable base undirected graph class.
    ///
    /// This class provides beside the core undirected graph features
    /// core clear functions for the undirected graph structure. The
    /// main difference between the base and this interface is that
    /// the graph alterations should handled already on this level.
    template <typename _Base = BaseGraphComponent>
    class ClearableGraphComponent : public ClearableDigraphComponent<_Base> {
    public:

      typedef _Base Base;

      template <typename _Graph>
      struct Constraints {
        void constraints() {
          checkConcept<ClearableGraphComponent<Base>, _Graph>();
        }

        _Graph graph;
      };
    };

  }

}

#endif
