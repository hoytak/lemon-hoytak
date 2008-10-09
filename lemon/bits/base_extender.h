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

#ifndef LEMON_BITS_BASE_EXTENDER_H
#define LEMON_BITS_BASE_EXTENDER_H

#include <lemon/core.h>
#include <lemon/error.h>

#include <lemon/bits/map_extender.h>
#include <lemon/bits/default_map.h>

#include <lemon/concept_check.h>
#include <lemon/concepts/maps.h>

//\ingroup digraphbits
//\file
//\brief Extenders for the digraph types
namespace lemon {

  // \ingroup digraphbits
  //
  // \brief BaseDigraph to BaseGraph extender
  template <typename Base>
  class UndirDigraphExtender : public Base {

  public:

    typedef Base Parent;
    typedef typename Parent::Arc Edge;
    typedef typename Parent::Node Node;

    typedef True UndirectedTag;

    class Arc : public Edge {
      friend class UndirDigraphExtender;

    protected:
      bool forward;

      Arc(const Edge &ue, bool _forward) :
        Edge(ue), forward(_forward) {}

    public:
      Arc() {}

      // Invalid arc constructor
      Arc(Invalid i) : Edge(i), forward(true) {}

      bool operator==(const Arc &that) const {
        return forward==that.forward && Edge(*this)==Edge(that);
      }
      bool operator!=(const Arc &that) const {
        return forward!=that.forward || Edge(*this)!=Edge(that);
      }
      bool operator<(const Arc &that) const {
        return forward<that.forward ||
          (!(that.forward<forward) && Edge(*this)<Edge(that));
      }
    };

    // First node of the edge
    Node u(const Edge &e) const {
      return Parent::source(e);
    }

    // Source of the given arc
    Node source(const Arc &e) const {
      return e.forward ? Parent::source(e) : Parent::target(e);
    }

    // Second node of the edge
    Node v(const Edge &e) const {
      return Parent::target(e);
    }

    // Target of the given arc
    Node target(const Arc &e) const {
      return e.forward ? Parent::target(e) : Parent::source(e);
    }

    // \brief Directed arc from an edge.
    //
    // Returns a directed arc corresponding to the specified edge.
    // If the given bool is true, the first node of the given edge and
    // the source node of the returned arc are the same.
    static Arc direct(const Edge &e, bool d) {
      return Arc(e, d);
    }

    // Returns whether the given directed arc has the same orientation
    // as the corresponding edge.
    static bool direction(const Arc &a) { return a.forward; }

    using Parent::first;
    using Parent::next;

    void first(Arc &e) const {
      Parent::first(e);
      e.forward=true;
    }

    void next(Arc &e) const {
      if( e.forward ) {
        e.forward = false;
      }
      else {
        Parent::next(e);
        e.forward = true;
      }
    }

    void firstOut(Arc &e, const Node &n) const {
      Parent::firstIn(e,n);
      if( Edge(e) != INVALID ) {
        e.forward = false;
      }
      else {
        Parent::firstOut(e,n);
        e.forward = true;
      }
    }
    void nextOut(Arc &e) const {
      if( ! e.forward ) {
        Node n = Parent::target(e);
        Parent::nextIn(e);
        if( Edge(e) == INVALID ) {
          Parent::firstOut(e, n);
          e.forward = true;
        }
      }
      else {
        Parent::nextOut(e);
      }
    }

    void firstIn(Arc &e, const Node &n) const {
      Parent::firstOut(e,n);
      if( Edge(e) != INVALID ) {
        e.forward = false;
      }
      else {
        Parent::firstIn(e,n);
        e.forward = true;
      }
    }
    void nextIn(Arc &e) const {
      if( ! e.forward ) {
        Node n = Parent::source(e);
        Parent::nextOut(e);
        if( Edge(e) == INVALID ) {
          Parent::firstIn(e, n);
          e.forward = true;
        }
      }
      else {
        Parent::nextIn(e);
      }
    }

    void firstInc(Edge &e, bool &d, const Node &n) const {
      d = true;
      Parent::firstOut(e, n);
      if (e != INVALID) return;
      d = false;
      Parent::firstIn(e, n);
    }

    void nextInc(Edge &e, bool &d) const {
      if (d) {
        Node s = Parent::source(e);
        Parent::nextOut(e);
        if (e != INVALID) return;
        d = false;
        Parent::firstIn(e, s);
      } else {
        Parent::nextIn(e);
      }
    }

    Node nodeFromId(int ix) const {
      return Parent::nodeFromId(ix);
    }

    Arc arcFromId(int ix) const {
      return direct(Parent::arcFromId(ix >> 1), bool(ix & 1));
    }

    Edge edgeFromId(int ix) const {
      return Parent::arcFromId(ix);
    }

    int id(const Node &n) const {
      return Parent::id(n);
    }

    int id(const Edge &e) const {
      return Parent::id(e);
    }

    int id(const Arc &e) const {
      return 2 * Parent::id(e) + int(e.forward);
    }

    int maxNodeId() const {
      return Parent::maxNodeId();
    }

    int maxArcId() const {
      return 2 * Parent::maxArcId() + 1;
    }

    int maxEdgeId() const {
      return Parent::maxArcId();
    }

    int arcNum() const {
      return 2 * Parent::arcNum();
    }

    int edgeNum() const {
      return Parent::arcNum();
    }

    Arc findArc(Node s, Node t, Arc p = INVALID) const {
      if (p == INVALID) {
        Edge arc = Parent::findArc(s, t);
        if (arc != INVALID) return direct(arc, true);
        arc = Parent::findArc(t, s);
        if (arc != INVALID) return direct(arc, false);
      } else if (direction(p)) {
        Edge arc = Parent::findArc(s, t, p);
        if (arc != INVALID) return direct(arc, true);
        arc = Parent::findArc(t, s);
        if (arc != INVALID) return direct(arc, false);
      } else {
        Edge arc = Parent::findArc(t, s, p);
        if (arc != INVALID) return direct(arc, false);
      }
      return INVALID;
    }

    Edge findEdge(Node s, Node t, Edge p = INVALID) const {
      if (s != t) {
        if (p == INVALID) {
          Edge arc = Parent::findArc(s, t);
          if (arc != INVALID) return arc;
          arc = Parent::findArc(t, s);
          if (arc != INVALID) return arc;
        } else if (Parent::s(p) == s) {
          Edge arc = Parent::findArc(s, t, p);
          if (arc != INVALID) return arc;
          arc = Parent::findArc(t, s);
          if (arc != INVALID) return arc;
        } else {
          Edge arc = Parent::findArc(t, s, p);
          if (arc != INVALID) return arc;
        }
      } else {
        return Parent::findArc(s, t, p);
      }
      return INVALID;
    }
  };

  template <typename Base>
  class BidirBpGraphExtender : public Base {
  public:
    typedef Base Parent;
    typedef BidirBpGraphExtender Digraph;

    typedef typename Parent::Node Node;
    typedef typename Parent::Edge Edge;


    using Parent::first;
    using Parent::next;

    using Parent::id;

    class Red : public Node {
      friend class BidirBpGraphExtender;
    public:
      Red() {}
      Red(const Node& node) : Node(node) {
        LEMON_DEBUG(Parent::red(node) || node == INVALID,
                    typename Parent::NodeSetError());
      }
      Red& operator=(const Node& node) {
        LEMON_DEBUG(Parent::red(node) || node == INVALID,
                    typename Parent::NodeSetError());
        Node::operator=(node);
        return *this;
      }
      Red(Invalid) : Node(INVALID) {}
      Red& operator=(Invalid) {
        Node::operator=(INVALID);
        return *this;
      }
    };

    void first(Red& node) const {
      Parent::firstRed(static_cast<Node&>(node));
    }
    void next(Red& node) const {
      Parent::nextRed(static_cast<Node&>(node));
    }

    int id(const Red& node) const {
      return Parent::redId(node);
    }

    class Blue : public Node {
      friend class BidirBpGraphExtender;
    public:
      Blue() {}
      Blue(const Node& node) : Node(node) {
        LEMON_DEBUG(Parent::blue(node) || node == INVALID,
                    typename Parent::NodeSetError());
      }
      Blue& operator=(const Node& node) {
        LEMON_DEBUG(Parent::blue(node) || node == INVALID,
                    typename Parent::NodeSetError());
        Node::operator=(node);
        return *this;
      }
      Blue(Invalid) : Node(INVALID) {}
      Blue& operator=(Invalid) {
        Node::operator=(INVALID);
        return *this;
      }
    };

    void first(Blue& node) const {
      Parent::firstBlue(static_cast<Node&>(node));
    }
    void next(Blue& node) const {
      Parent::nextBlue(static_cast<Node&>(node));
    }

    int id(const Blue& node) const {
      return Parent::redId(node);
    }

    Node source(const Edge& arc) const {
      return red(arc);
    }
    Node target(const Edge& arc) const {
      return blue(arc);
    }

    void firstInc(Edge& arc, bool& dir, const Node& node) const {
      if (Parent::red(node)) {
        Parent::firstFromRed(arc, node);
        dir = true;
      } else {
        Parent::firstFromBlue(arc, node);
        dir = static_cast<Edge&>(arc) == INVALID;
      }
    }
    void nextInc(Edge& arc, bool& dir) const {
      if (dir) {
        Parent::nextFromRed(arc);
      } else {
        Parent::nextFromBlue(arc);
        if (arc == INVALID) dir = true;
      }
    }

    class Arc : public Edge {
      friend class BidirBpGraphExtender;
    protected:
      bool forward;

      Arc(const Edge& arc, bool _forward)
        : Edge(arc), forward(_forward) {}

    public:
      Arc() {}
      Arc (Invalid) : Edge(INVALID), forward(true) {}
      bool operator==(const Arc& i) const {
        return Edge::operator==(i) && forward == i.forward;
      }
      bool operator!=(const Arc& i) const {
        return Edge::operator!=(i) || forward != i.forward;
      }
      bool operator<(const Arc& i) const {
        return Edge::operator<(i) ||
          (!(i.forward<forward) && Edge(*this)<Edge(i));
      }
    };

    void first(Arc& arc) const {
      Parent::first(static_cast<Edge&>(arc));
      arc.forward = true;
    }

    void next(Arc& arc) const {
      if (!arc.forward) {
        Parent::next(static_cast<Edge&>(arc));
      }
      arc.forward = !arc.forward;
    }

    void firstOut(Arc& arc, const Node& node) const {
      if (Parent::red(node)) {
        Parent::firstFromRed(arc, node);
        arc.forward = true;
      } else {
        Parent::firstFromBlue(arc, node);
        arc.forward = static_cast<Edge&>(arc) == INVALID;
      }
    }
    void nextOut(Arc& arc) const {
      if (arc.forward) {
        Parent::nextFromRed(arc);
      } else {
        Parent::nextFromBlue(arc);
        arc.forward = static_cast<Edge&>(arc) == INVALID;
      }
    }

    void firstIn(Arc& arc, const Node& node) const {
      if (Parent::blue(node)) {
        Parent::firstFromBlue(arc, node);
        arc.forward = true;
      } else {
        Parent::firstFromRed(arc, node);
        arc.forward = static_cast<Edge&>(arc) == INVALID;
      }
    }
    void nextIn(Arc& arc) const {
      if (arc.forward) {
        Parent::nextFromBlue(arc);
      } else {
        Parent::nextFromRed(arc);
        arc.forward = static_cast<Edge&>(arc) == INVALID;
      }
    }

    Node source(const Arc& arc) const {
      return arc.forward ? Parent::red(arc) : Parent::blue(arc);
    }
    Node target(const Arc& arc) const {
      return arc.forward ? Parent::blue(arc) : Parent::red(arc);
    }

    int id(const Arc& arc) const {
      return (Parent::id(static_cast<const Edge&>(arc)) << 1) +
        (arc.forward ? 0 : 1);
    }
    Arc arcFromId(int ix) const {
      return Arc(Parent::fromEdgeId(ix >> 1), (ix & 1) == 0);
    }
    int maxArcId() const {
      return (Parent::maxEdgeId() << 1) + 1;
    }

    bool direction(const Arc& arc) const {
      return arc.forward;
    }

    Arc direct(const Edge& arc, bool dir) const {
      return Arc(arc, dir);
    }

    int arcNum() const {
      return 2 * Parent::edgeNum();
    }

    int edgeNum() const {
      return Parent::edgeNum();
    }


  };
}

#endif
