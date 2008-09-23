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

#ifndef LEMON_CONCEPT_DIGRAPH_H
#define LEMON_CONCEPT_DIGRAPH_H

///\ingroup graph_concepts
///\file
///\brief The concept of directed graphs.

#include <lemon/core.h>
#include <lemon/concepts/maps.h>
#include <lemon/concept_check.h>
#include <lemon/concepts/graph_components.h>

namespace lemon {
  namespace concepts {

    /// \ingroup graph_concepts
    ///
    /// \brief Class describing the concept of directed graphs.
    ///
    /// This class describes the \ref concept "concept" of the
    /// immutable directed digraphs.
    ///
    /// Note that actual digraph implementation like @ref ListDigraph or
    /// @ref SmartDigraph may have several additional functionality.
    ///
    /// \sa concept
    class Digraph {
    private:
      ///Digraphs are \e not copy constructible. Use DigraphCopy() instead.

      ///Digraphs are \e not copy constructible. Use DigraphCopy() instead.
      ///
      Digraph(const Digraph &) {};
      ///\brief Assignment of \ref Digraph "Digraph"s to another ones are
      ///\e not allowed. Use DigraphCopy() instead.

      ///Assignment of \ref Digraph "Digraph"s to another ones are
      ///\e not allowed.  Use DigraphCopy() instead.

      void operator=(const Digraph &) {}
    public:
      ///\e

      /// Defalult constructor.

      /// Defalult constructor.
      ///
      Digraph() { }
      /// Class for identifying a node of the digraph

      /// This class identifies a node of the digraph. It also serves
      /// as a base class of the node iterators,
      /// thus they will convert to this type.
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

        /// To allow the use of digraph descriptors as key type in std::map or
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
      /// of nodes in digraph \c g of type \c Digraph like this:
      ///\code
      /// int count=0;
      /// for (Digraph::NodeIt n(g); n!=INVALID; ++n) ++count;
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
        NodeIt(const Digraph&) { }
        /// Node -> NodeIt conversion.

        /// Sets the iterator to the node of \c the digraph pointed by
        /// the trivial iterator.
        /// This feature necessitates that each time we
        /// iterate the arc-set, the iteration order is the same.
        NodeIt(const Digraph&, const Node&) { }
        /// Next node.

        /// Assign the iterator to the next node.
        ///
        NodeIt& operator++() { return *this; }
      };


      /// Class for identifying an arc of the digraph

      /// This class identifies an arc of the digraph. It also serves
      /// as a base class of the arc iterators,
      /// thus they will convert to this type.
      class Arc {
      public:
        /// Default constructor

        /// @warning The default constructor sets the iterator
        /// to an undefined value.
        Arc() { }
        /// Copy constructor.

        /// Copy constructor.
        ///
        Arc(const Arc&) { }
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

        /// To allow the use of digraph descriptors as key type in std::map or
        /// similar associative container we require this.
        ///
        /// \note This operator only have to define some strict ordering of
        /// the items; this order has nothing to do with the iteration
        /// ordering of the items.
        bool operator<(Arc) const { return false; }
      };

      /// This iterator goes trough the outgoing arcs of a node.

      /// This iterator goes trough the \e outgoing arcs of a certain node
      /// of a digraph.
      /// Its usage is quite simple, for example you can count the number
      /// of outgoing arcs of a node \c n
      /// in digraph \c g of type \c Digraph as follows.
      ///\code
      /// int count=0;
      /// for (Digraph::OutArcIt e(g, n); e!=INVALID; ++e) ++count;
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
        OutArcIt(const Digraph&, const Node&) { }
        /// Arc -> OutArcIt conversion

        /// Sets the iterator to the value of the trivial iterator.
        /// This feature necessitates that each time we
        /// iterate the arc-set, the iteration order is the same.
        OutArcIt(const Digraph&, const Arc&) { }
        ///Next outgoing arc

        /// Assign the iterator to the next
        /// outgoing arc of the corresponding node.
        OutArcIt& operator++() { return *this; }
      };

      /// This iterator goes trough the incoming arcs of a node.

      /// This iterator goes trough the \e incoming arcs of a certain node
      /// of a digraph.
      /// Its usage is quite simple, for example you can count the number
      /// of outgoing arcs of a node \c n
      /// in digraph \c g of type \c Digraph as follows.
      ///\code
      /// int count=0;
      /// for(Digraph::InArcIt e(g, n); e!=INVALID; ++e) ++count;
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
        InArcIt(const Digraph&, const Node&) { }
        /// Arc -> InArcIt conversion

        /// Sets the iterator to the value of the trivial iterator \c e.
        /// This feature necessitates that each time we
        /// iterate the arc-set, the iteration order is the same.
        InArcIt(const Digraph&, const Arc&) { }
        /// Next incoming arc

        /// Assign the iterator to the next inarc of the corresponding node.
        ///
        InArcIt& operator++() { return *this; }
      };
      /// This iterator goes through each arc.

      /// This iterator goes through each arc of a digraph.
      /// Its usage is quite simple, for example you can count the number
      /// of arcs in a digraph \c g of type \c Digraph as follows:
      ///\code
      /// int count=0;
      /// for(Digraph::ArcIt e(g); e!=INVALID; ++e) ++count;
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
        ///@param g the digraph
        ArcIt(const Digraph& g) { ignore_unused_variable_warning(g); }
        /// Arc -> ArcIt conversion

        /// Sets the iterator to the value of the trivial iterator \c e.
        /// This feature necessitates that each time we
        /// iterate the arc-set, the iteration order is the same.
        ArcIt(const Digraph&, const Arc&) { }
        ///Next arc

        /// Assign the iterator to the next arc.
        ArcIt& operator++() { return *this; }
      };
      ///Gives back the target node of an arc.

      ///Gives back the target node of an arc.
      ///
      Node target(Arc) const { return INVALID; }
      ///Gives back the source node of an arc.

      ///Gives back the source node of an arc.
      ///
      Node source(Arc) const { return INVALID; }

      /// \brief Returns the ID of the node.
      int id(Node) const { return -1; }

      /// \brief Returns the ID of the arc.
      int id(Arc) const { return -1; }

      /// \brief Returns the node with the given ID.
      ///
      /// \pre The argument should be a valid node ID in the graph.
      Node nodeFromId(int) const { return INVALID; }

      /// \brief Returns the arc with the given ID.
      ///
      /// \pre The argument should be a valid arc ID in the graph.
      Arc arcFromId(int) const { return INVALID; }

      /// \brief Returns an upper bound on the node IDs.
      int maxNodeId() const { return -1; }

      /// \brief Returns an upper bound on the arc IDs.
      int maxArcId() const { return -1; }

      void first(Node&) const {}
      void next(Node&) const {}

      void first(Arc&) const {}
      void next(Arc&) const {}


      void firstIn(Arc&, const Node&) const {}
      void nextIn(Arc&) const {}

      void firstOut(Arc&, const Node&) const {}
      void nextOut(Arc&) const {}

      // The second parameter is dummy.
      Node fromId(int, Node) const { return INVALID; }
      // The second parameter is dummy.
      Arc fromId(int, Arc) const { return INVALID; }

      // Dummy parameter.
      int maxId(Node) const { return -1; }
      // Dummy parameter.
      int maxId(Arc) const { return -1; }

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

      /// \brief The opposite node on the given arc.
      ///
      /// Gives back the opposite node on the given arc.
      Node oppositeNode(const Node&, const Arc&) const { return INVALID; }

      /// \brief Read write map of the nodes to type \c T.
      ///
      /// ReadWrite map of the nodes to type \c T.
      /// \sa Reference
      template<class T>
      class NodeMap : public ReadWriteMap< Node, T > {
      public:

        ///\e
        NodeMap(const Digraph&) { }
        ///\e
        NodeMap(const Digraph&, T) { }

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

      /// \brief Read write map of the arcs to type \c T.
      ///
      /// Reference map of the arcs to type \c T.
      /// \sa Reference
      template<class T>
      class ArcMap : public ReadWriteMap<Arc,T> {
      public:

        ///\e
        ArcMap(const Digraph&) { }
        ///\e
        ArcMap(const Digraph&, T) { }
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

      template <typename _Digraph>
      struct Constraints {
        void constraints() {
          checkConcept<IterableDigraphComponent<>, _Digraph>();
          checkConcept<IDableDigraphComponent<>, _Digraph>();
          checkConcept<MappableDigraphComponent<>, _Digraph>();
        }
      };

    };

  } //namespace concepts
} //namespace lemon



#endif // LEMON_CONCEPT_DIGRAPH_H
