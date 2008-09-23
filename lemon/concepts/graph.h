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
///\brief The concept of Undirected Graphs.

#ifndef LEMON_CONCEPT_GRAPH_H
#define LEMON_CONCEPT_GRAPH_H

#include <lemon/concepts/graph_components.h>
#include <lemon/concepts/graph.h>
#include <lemon/core.h>

namespace lemon {
  namespace concepts {

    /// \ingroup graph_concepts
    ///
    /// \brief Class describing the concept of Undirected Graphs.
    ///
    /// This class describes the common interface of all Undirected
    /// Graphs.
    ///
    /// As all concept describing classes it provides only interface
    /// without any sensible implementation. So any algorithm for
    /// undirected graph should compile with this class, but it will not
    /// run properly, of course.
    ///
    /// The LEMON undirected graphs also fulfill the concept of
    /// directed graphs (\ref lemon::concepts::Digraph "Digraph
    /// Concept"). Each edges can be seen as two opposite
    /// directed arc and consequently the undirected graph can be
    /// seen as the direceted graph of these directed arcs. The
    /// Graph has the Edge inner class for the edges and
    /// the Arc type for the directed arcs. The Arc type is
    /// convertible to Edge or inherited from it so from a directed
    /// arc we can get the represented edge.
    ///
    /// In the sense of the LEMON each edge has a default
    /// direction (it should be in every computer implementation,
    /// because the order of edge's nodes defines an
    /// orientation). With the default orientation we can define that
    /// the directed arc is forward or backward directed. With the \c
    /// direction() and \c direct() function we can get the direction
    /// of the directed arc and we can direct an edge.
    ///
    /// The EdgeIt is an iterator for the edges. We can use
    /// the EdgeMap to map values for the edges. The InArcIt and
    /// OutArcIt iterates on the same edges but with opposite
    /// direction. The IncEdgeIt iterates also on the same edges
    /// as the OutArcIt and InArcIt but it is not convertible to Arc just
    /// to Edge.
    class Graph {
    public:
      /// \brief The undirected graph should be tagged by the
      /// UndirectedTag.
      ///
      /// The undirected graph should be tagged by the UndirectedTag. This
      /// tag helps the enable_if technics to make compile time
      /// specializations for undirected graphs.
      typedef True UndirectedTag;

      /// \brief The base type of node iterators,
      /// or in other words, the trivial node iterator.
      ///
      /// This is the base type of each node iterator,
      /// thus each kind of node iterator converts to this.
      /// More precisely each kind of node iterator should be inherited
      /// from the trivial node iterator.
      class Node {
      public:
        /// Default constructor

        /// @warning The default constructor sets the iterator
        /// to an undefined value.
        Node() { }
        /// Copy constructor.

        /// Copy constructor.
        ///
        Node(const Node&) { }

        /// Invalid constructor \& conversion.

        /// This constructor initializes the iterator to be invalid.
        /// \sa Invalid for more details.
        Node(Invalid) { }
        /// Equality operator

        /// Two iterators are equal if and only if they point to the
        /// same object or both are invalid.
        bool operator==(Node) const { return true; }

        /// Inequality operator

        /// \sa operator==(Node n)
        ///
        bool operator!=(Node) const { return true; }

        /// Artificial ordering operator.

        /// To allow the use of graph descriptors as key type in std::map or
        /// similar associative container we require this.
        ///
        /// \note This operator only have to define some strict ordering of
        /// the items; this order has nothing to do with the iteration
        /// ordering of the items.
        bool operator<(Node) const { return false; }

      };

      /// This iterator goes through each node.

      /// This iterator goes through each node.
      /// Its usage is quite simple, for example you can count the number
      /// of nodes in graph \c g of type \c Graph like this:
      ///\code
      /// int count=0;
      /// for (Graph::NodeIt n(g); n!=INVALID; ++n) ++count;
      ///\endcode
      class NodeIt : public Node {
      public:
        /// Default constructor

        /// @warning The default constructor sets the iterator
        /// to an undefined value.
        NodeIt() { }
        /// Copy constructor.

        /// Copy constructor.
        ///
        NodeIt(const NodeIt& n) : Node(n) { }
        /// Invalid constructor \& conversion.

        /// Initialize the iterator to be invalid.
        /// \sa Invalid for more details.
        NodeIt(Invalid) { }
        /// Sets the iterator to the first node.

        /// Sets the iterator to the first node of \c g.
        ///
        NodeIt(const Graph&) { }
        /// Node -> NodeIt conversion.

        /// Sets the iterator to the node of \c the graph pointed by
        /// the trivial iterator.
        /// This feature necessitates that each time we
        /// iterate the arc-set, the iteration order is the same.
        NodeIt(const Graph&, const Node&) { }
        /// Next node.

        /// Assign the iterator to the next node.
        ///
        NodeIt& operator++() { return *this; }
      };


      /// The base type of the edge iterators.

      /// The base type of the edge iterators.
      ///
      class Edge {
      public:
        /// Default constructor

        /// @warning The default constructor sets the iterator
        /// to an undefined value.
        Edge() { }
        /// Copy constructor.

        /// Copy constructor.
        ///
        Edge(const Edge&) { }
        /// Initialize the iterator to be invalid.

        /// Initialize the iterator to be invalid.
        ///
        Edge(Invalid) { }
        /// Equality operator

        /// Two iterators are equal if and only if they point to the
        /// same object or both are invalid.
        bool operator==(Edge) const { return true; }
        /// Inequality operator

        /// \sa operator==(Edge n)
        ///
        bool operator!=(Edge) const { return true; }

        /// Artificial ordering operator.

        /// To allow the use of graph descriptors as key type in std::map or
        /// similar associative container we require this.
        ///
        /// \note This operator only have to define some strict ordering of
        /// the items; this order has nothing to do with the iteration
        /// ordering of the items.
        bool operator<(Edge) const { return false; }
      };

      /// This iterator goes through each edge.

      /// This iterator goes through each edge of a graph.
      /// Its usage is quite simple, for example you can count the number
      /// of edges in a graph \c g of type \c Graph as follows:
      ///\code
      /// int count=0;
      /// for(Graph::EdgeIt e(g); e!=INVALID; ++e) ++count;
      ///\endcode
      class EdgeIt : public Edge {
      public:
        /// Default constructor

        /// @warning The default constructor sets the iterator
        /// to an undefined value.
        EdgeIt() { }
        /// Copy constructor.

        /// Copy constructor.
        ///
        EdgeIt(const EdgeIt& e) : Edge(e) { }
        /// Initialize the iterator to be invalid.

        /// Initialize the iterator to be invalid.
        ///
        EdgeIt(Invalid) { }
        /// This constructor sets the iterator to the first edge.

        /// This constructor sets the iterator to the first edge.
        EdgeIt(const Graph&) { }
        /// Edge -> EdgeIt conversion

        /// Sets the iterator to the value of the trivial iterator.
        /// This feature necessitates that each time we
        /// iterate the edge-set, the iteration order is the
        /// same.
        EdgeIt(const Graph&, const Edge&) { }
        /// Next edge

        /// Assign the iterator to the next edge.
        EdgeIt& operator++() { return *this; }
      };

      /// \brief This iterator goes trough the incident undirected
      /// arcs of a node.
      ///
      /// This iterator goes trough the incident edges
      /// of a certain node of a graph. You should assume that the
      /// loop arcs will be iterated twice.
      ///
      /// Its usage is quite simple, for example you can compute the
      /// degree (i.e. count the number of incident arcs of a node \c n
      /// in graph \c g of type \c Graph as follows.
      ///
      ///\code
      /// int count=0;
      /// for(Graph::IncEdgeIt e(g, n); e!=INVALID; ++e) ++count;
      ///\endcode
      class IncEdgeIt : public Edge {
      public:
        /// Default constructor

        /// @warning The default constructor sets the iterator
        /// to an undefined value.
        IncEdgeIt() { }
        /// Copy constructor.

        /// Copy constructor.
        ///
        IncEdgeIt(const IncEdgeIt& e) : Edge(e) { }
        /// Initialize the iterator to be invalid.

        /// Initialize the iterator to be invalid.
        ///
        IncEdgeIt(Invalid) { }
        /// This constructor sets the iterator to first incident arc.

        /// This constructor set the iterator to the first incident arc of
        /// the node.
        IncEdgeIt(const Graph&, const Node&) { }
        /// Edge -> IncEdgeIt conversion

        /// Sets the iterator to the value of the trivial iterator \c e.
        /// This feature necessitates that each time we
        /// iterate the arc-set, the iteration order is the same.
        IncEdgeIt(const Graph&, const Edge&) { }
        /// Next incident arc

        /// Assign the iterator to the next incident arc
        /// of the corresponding node.
        IncEdgeIt& operator++() { return *this; }
      };

      /// The directed arc type.

      /// The directed arc type. It can be converted to the
      /// edge or it should be inherited from the undirected
      /// arc.
      class Arc : public Edge {
      public:
        /// Default constructor

        /// @warning The default constructor sets the iterator
        /// to an undefined value.
        Arc() { }
        /// Copy constructor.

        /// Copy constructor.
        ///
        Arc(const Arc& e) : Edge(e) { }
        /// Initialize the iterator to be invalid.

        /// Initialize the iterator to be invalid.
        ///
        Arc(Invalid) { }
        /// Equality operator

        /// Two iterators are equal if and only if they point to the
        /// same object or both are invalid.
        bool operator==(Arc) const { return true; }
        /// Inequality operator

        /// \sa operator==(Arc n)
        ///
        bool operator!=(Arc) const { return true; }

        /// Artificial ordering operator.

        /// To allow the use of graph descriptors as key type in std::map or
        /// similar associative container we require this.
        ///
        /// \note This operator only have to define some strict ordering of
        /// the items; this order has nothing to do with the iteration
        /// ordering of the items.
        bool operator<(Arc) const { return false; }

      };
      /// This iterator goes through each directed arc.

      /// This iterator goes through each arc of a graph.
      /// Its usage is quite simple, for example you can count the number
      /// of arcs in a graph \c g of type \c Graph as follows:
      ///\code
      /// int count=0;
      /// for(Graph::ArcIt e(g); e!=INVALID; ++e) ++count;
      ///\endcode
      class ArcIt : public Arc {
      public:
        /// Default constructor

        /// @warning The default constructor sets the iterator
        /// to an undefined value.
        ArcIt() { }
        /// Copy constructor.

        /// Copy constructor.
        ///
        ArcIt(const ArcIt& e) : Arc(e) { }
        /// Initialize the iterator to be invalid.

        /// Initialize the iterator to be invalid.
        ///
        ArcIt(Invalid) { }
        /// This constructor sets the iterator to the first arc.

        /// This constructor sets the iterator to the first arc of \c g.
        ///@param g the graph
        ArcIt(const Graph &g) { ignore_unused_variable_warning(g); }
        /// Arc -> ArcIt conversion

        /// Sets the iterator to the value of the trivial iterator \c e.
        /// This feature necessitates that each time we
        /// iterate the arc-set, the iteration order is the same.
        ArcIt(const Graph&, const Arc&) { }
        ///Next arc

        /// Assign the iterator to the next arc.
        ArcIt& operator++() { return *this; }
      };

      /// This iterator goes trough the outgoing directed arcs of a node.

      /// This iterator goes trough the \e outgoing arcs of a certain node
      /// of a graph.
      /// Its usage is quite simple, for example you can count the number
      /// of outgoing arcs of a node \c n
      /// in graph \c g of type \c Graph as follows.
      ///\code
      /// int count=0;
      /// for (Graph::OutArcIt e(g, n); e!=INVALID; ++e) ++count;
      ///\endcode

      class OutArcIt : public Arc {
      public:
        /// Default constructor

        /// @warning The default constructor sets the iterator
        /// to an undefined value.
        OutArcIt() { }
        /// Copy constructor.

        /// Copy constructor.
        ///
        OutArcIt(const OutArcIt& e) : Arc(e) { }
        /// Initialize the iterator to be invalid.

        /// Initialize the iterator to be invalid.
        ///
        OutArcIt(Invalid) { }
        /// This constructor sets the iterator to the first outgoing arc.

        /// This constructor sets the iterator to the first outgoing arc of
        /// the node.
        ///@param n the node
        ///@param g the graph
        OutArcIt(const Graph& n, const Node& g) {
          ignore_unused_variable_warning(n);
          ignore_unused_variable_warning(g);
        }
        /// Arc -> OutArcIt conversion

        /// Sets the iterator to the value of the trivial iterator.
        /// This feature necessitates that each time we
        /// iterate the arc-set, the iteration order is the same.
        OutArcIt(const Graph&, const Arc&) { }
        ///Next outgoing arc

        /// Assign the iterator to the next
        /// outgoing arc of the corresponding node.
        OutArcIt& operator++() { return *this; }
      };

      /// This iterator goes trough the incoming directed arcs of a node.

      /// This iterator goes trough the \e incoming arcs of a certain node
      /// of a graph.
      /// Its usage is quite simple, for example you can count the number
      /// of outgoing arcs of a node \c n
      /// in graph \c g of type \c Graph as follows.
      ///\code
      /// int count=0;
      /// for(Graph::InArcIt e(g, n); e!=INVALID; ++e) ++count;
      ///\endcode

      class InArcIt : public Arc {
      public:
        /// Default constructor

        /// @warning The default constructor sets the iterator
        /// to an undefined value.
        InArcIt() { }
        /// Copy constructor.

        /// Copy constructor.
        ///
        InArcIt(const InArcIt& e) : Arc(e) { }
        /// Initialize the iterator to be invalid.

        /// Initialize the iterator to be invalid.
        ///
        InArcIt(Invalid) { }
        /// This constructor sets the iterator to first incoming arc.

        /// This constructor set the iterator to the first incoming arc of
        /// the node.
        ///@param n the node
        ///@param g the graph
        InArcIt(const Graph& g, const Node& n) {
          ignore_unused_variable_warning(n);
          ignore_unused_variable_warning(g);
        }
        /// Arc -> InArcIt conversion

        /// Sets the iterator to the value of the trivial iterator \c e.
        /// This feature necessitates that each time we
        /// iterate the arc-set, the iteration order is the same.
        InArcIt(const Graph&, const Arc&) { }
        /// Next incoming arc

        /// Assign the iterator to the next inarc of the corresponding node.
        ///
        InArcIt& operator++() { return *this; }
      };

      /// \brief Read write map of the nodes to type \c T.
      ///
      /// ReadWrite map of the nodes to type \c T.
      /// \sa Reference
      template<class T>
      class NodeMap : public ReadWriteMap< Node, T >
      {
      public:

        ///\e
        NodeMap(const Graph&) { }
        ///\e
        NodeMap(const Graph&, T) { }

      private:
        ///Copy constructor
        NodeMap(const NodeMap& nm) : ReadWriteMap< Node, T >(nm) { }
        ///Assignment operator
        template <typename CMap>
        NodeMap& operator=(const CMap&) {
          checkConcept<ReadMap<Node, T>, CMap>();
          return *this;
        }
      };

      /// \brief Read write map of the directed arcs to type \c T.
      ///
      /// Reference map of the directed arcs to type \c T.
      /// \sa Reference
      template<class T>
      class ArcMap : public ReadWriteMap<Arc,T>
      {
      public:

        ///\e
        ArcMap(const Graph&) { }
        ///\e
        ArcMap(const Graph&, T) { }
      private:
        ///Copy constructor
        ArcMap(const ArcMap& em) : ReadWriteMap<Arc,T>(em) { }
        ///Assignment operator
        template <typename CMap>
        ArcMap& operator=(const CMap&) {
          checkConcept<ReadMap<Arc, T>, CMap>();
          return *this;
        }
      };

      /// Read write map of the edges to type \c T.

      /// Reference map of the arcs to type \c T.
      /// \sa Reference
      template<class T>
      class EdgeMap : public ReadWriteMap<Edge,T>
      {
      public:

        ///\e
        EdgeMap(const Graph&) { }
        ///\e
        EdgeMap(const Graph&, T) { }
      private:
        ///Copy constructor
        EdgeMap(const EdgeMap& em) : ReadWriteMap<Edge,T>(em) {}
        ///Assignment operator
        template <typename CMap>
        EdgeMap& operator=(const CMap&) {
          checkConcept<ReadMap<Edge, T>, CMap>();
          return *this;
        }
      };

      /// \brief Direct the given edge.
      ///
      /// Direct the given edge. The returned arc source
      /// will be the given node.
      Arc direct(const Edge&, const Node&) const {
        return INVALID;
      }

      /// \brief Direct the given edge.
      ///
      /// Direct the given edge. The returned arc
      /// represents the given edge and the direction comes
      /// from the bool parameter. The source of the edge and
      /// the directed arc is the same when the given bool is true.
      Arc direct(const Edge&, bool) const {
        return INVALID;
      }

      /// \brief Returns true if the arc has default orientation.
      ///
      /// Returns whether the given directed arc is same orientation as
      /// the corresponding edge's default orientation.
      bool direction(Arc) const { return true; }

      /// \brief Returns the opposite directed arc.
      ///
      /// Returns the opposite directed arc.
      Arc oppositeArc(Arc) const { return INVALID; }

      /// \brief Opposite node on an arc
      ///
      /// \return the opposite of the given Node on the given Edge
      Node oppositeNode(Node, Edge) const { return INVALID; }

      /// \brief First node of the edge.
      ///
      /// \return the first node of the given Edge.
      ///
      /// Naturally edges don't have direction and thus
      /// don't have source and target node. But we use these two methods
      /// to query the two nodes of the arc. The direction of the arc
      /// which arises this way is called the inherent direction of the
      /// edge, and is used to define the "default" direction
      /// of the directed versions of the arcs.
      /// \sa direction
      Node u(Edge) const { return INVALID; }

      /// \brief Second node of the edge.
      Node v(Edge) const { return INVALID; }

      /// \brief Source node of the directed arc.
      Node source(Arc) const { return INVALID; }

      /// \brief Target node of the directed arc.
      Node target(Arc) const { return INVALID; }

      /// \brief Returns the id of the node.
      int id(Node) const { return -1; }

      /// \brief Returns the id of the edge.
      int id(Edge) const { return -1; }

      /// \brief Returns the id of the arc.
      int id(Arc) const { return -1; }

      /// \brief Returns the node with the given id.
      ///
      /// \pre The argument should be a valid node id in the graph.
      Node nodeFromId(int) const { return INVALID; }

      /// \brief Returns the edge with the given id.
      ///
      /// \pre The argument should be a valid edge id in the graph.
      Edge edgeFromId(int) const { return INVALID; }

      /// \brief Returns the arc with the given id.
      ///
      /// \pre The argument should be a valid arc id in the graph.
      Arc arcFromId(int) const { return INVALID; }

      /// \brief Returns an upper bound on the node IDs.
      int maxNodeId() const { return -1; }

      /// \brief Returns an upper bound on the edge IDs.
      int maxEdgeId() const { return -1; }

      /// \brief Returns an upper bound on the arc IDs.
      int maxArcId() const { return -1; }

      void first(Node&) const {}
      void next(Node&) const {}

      void first(Edge&) const {}
      void next(Edge&) const {}

      void first(Arc&) const {}
      void next(Arc&) const {}

      void firstOut(Arc&, Node) const {}
      void nextOut(Arc&) const {}

      void firstIn(Arc&, Node) const {}
      void nextIn(Arc&) const {}

      void firstInc(Edge &, bool &, const Node &) const {}
      void nextInc(Edge &, bool &) const {}

      // The second parameter is dummy.
      Node fromId(int, Node) const { return INVALID; }
      // The second parameter is dummy.
      Edge fromId(int, Edge) const { return INVALID; }
      // The second parameter is dummy.
      Arc fromId(int, Arc) const { return INVALID; }

      // Dummy parameter.
      int maxId(Node) const { return -1; }
      // Dummy parameter.
      int maxId(Edge) const { return -1; }
      // Dummy parameter.
      int maxId(Arc) const { return -1; }

      /// \brief Base node of the iterator
      ///
      /// Returns the base node (the source in this case) of the iterator
      Node baseNode(OutArcIt e) const {
        return source(e);
      }
      /// \brief Running node of the iterator
      ///
      /// Returns the running node (the target in this case) of the
      /// iterator
      Node runningNode(OutArcIt e) const {
        return target(e);
      }

      /// \brief Base node of the iterator
      ///
      /// Returns the base node (the target in this case) of the iterator
      Node baseNode(InArcIt e) const {
        return target(e);
      }
      /// \brief Running node of the iterator
      ///
      /// Returns the running node (the source in this case) of the
      /// iterator
      Node runningNode(InArcIt e) const {
        return source(e);
      }

      /// \brief Base node of the iterator
      ///
      /// Returns the base node of the iterator
      Node baseNode(IncEdgeIt) const {
        return INVALID;
      }

      /// \brief Running node of the iterator
      ///
      /// Returns the running node of the iterator
      Node runningNode(IncEdgeIt) const {
        return INVALID;
      }

      template <typename _Graph>
      struct Constraints {
        void constraints() {
          checkConcept<IterableGraphComponent<>, _Graph>();
          checkConcept<IDableGraphComponent<>, _Graph>();
          checkConcept<MappableGraphComponent<>, _Graph>();
        }
      };

    };

  }

}

#endif
