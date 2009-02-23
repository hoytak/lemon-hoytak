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

#ifndef LEMON_EULER_H
#define LEMON_EULER_H

#include<lemon/core.h>
#include<lemon/adaptors.h>
#include<lemon/connectivity.h>
#include <list>

/// \ingroup graph_prop
/// \file
/// \brief Euler tour
///
///This file provides an Euler tour iterator and ways to check
///if a digraph is euler.


namespace lemon {

  ///Euler iterator for digraphs.

  /// \ingroup graph_prop
  ///This iterator converts to the \c Arc type of the digraph and using
  ///operator ++, it provides an Euler tour of a \e directed
  ///graph (if there exists).
  ///
  ///For example
  ///if the given digraph is Euler (i.e it has only one nontrivial component
  ///and the in-degree is equal to the out-degree for all nodes),
  ///the following code will put the arcs of \c g
  ///to the vector \c et according to an
  ///Euler tour of \c g.
  ///\code
  ///  std::vector<ListDigraph::Arc> et;
  ///  for(DiEulerIt<ListDigraph> e(g),e!=INVALID;++e)
  ///    et.push_back(e);
  ///\endcode
  ///If \c g is not Euler then the resulted tour will not be full or closed.
  ///\sa EulerIt
  ///\todo Test required
  template<class Digraph>
  class DiEulerIt
  {
    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::ArcIt ArcIt;
    typedef typename Digraph::OutArcIt OutArcIt;
    typedef typename Digraph::InArcIt InArcIt;

    const Digraph &g;
    typename Digraph::template NodeMap<OutArcIt> nedge;
    std::list<Arc> euler;

  public:

    ///Constructor

    ///\param _g A digraph.
    ///\param start The starting point of the tour. If it is not given
    ///       the tour will start from the first node.
    DiEulerIt(const Digraph &_g,typename Digraph::Node start=INVALID)
      : g(_g), nedge(g)
    {
      if(start==INVALID) start=NodeIt(g);
      for(NodeIt n(g);n!=INVALID;++n) nedge[n]=OutArcIt(g,n);
      while(nedge[start]!=INVALID) {
        euler.push_back(nedge[start]);
        Node next=g.target(nedge[start]);
        ++nedge[start];
        start=next;
      }
    }

    ///Arc Conversion
    operator Arc() { return euler.empty()?INVALID:euler.front(); }
    bool operator==(Invalid) { return euler.empty(); }
    bool operator!=(Invalid) { return !euler.empty(); }

    ///Next arc of the tour
    DiEulerIt &operator++() {
      Node s=g.target(euler.front());
      euler.pop_front();
      //This produces a warning.Strange.
      //std::list<Arc>::iterator next=euler.begin();
      typename std::list<Arc>::iterator next=euler.begin();
      while(nedge[s]!=INVALID) {
        euler.insert(next,nedge[s]);
        Node n=g.target(nedge[s]);
        ++nedge[s];
        s=n;
      }
      return *this;
    }
    ///Postfix incrementation

    ///\warning This incrementation
    ///returns an \c Arc, not an \ref DiEulerIt, as one may
    ///expect.
    Arc operator++(int)
    {
      Arc e=*this;
      ++(*this);
      return e;
    }
  };

  ///Euler iterator for graphs.

  /// \ingroup graph_prop
  ///This iterator converts to the \c Arc (or \c Edge)
  ///type of the digraph and using
  ///operator ++, it provides an Euler tour of an undirected
  ///digraph (if there exists).
  ///
  ///For example
  ///if the given digraph if Euler (i.e it has only one nontrivial component
  ///and the degree of each node is even),
  ///the following code will print the arc IDs according to an
  ///Euler tour of \c g.
  ///\code
  ///  for(EulerIt<ListGraph> e(g),e!=INVALID;++e) {
  ///    std::cout << g.id(Edge(e)) << std::eol;
  ///  }
  ///\endcode
  ///Although the iterator provides an Euler tour of an graph,
  ///it still returns Arcs in order to indicate the direction of the tour.
  ///(But Arc will convert to Edges, of course).
  ///
  ///If \c g is not Euler then the resulted tour will not be full or closed.
  ///\sa EulerIt
  ///\todo Test required
  template<class Digraph>
  class EulerIt
  {
    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::Edge Edge;
    typedef typename Digraph::ArcIt ArcIt;
    typedef typename Digraph::OutArcIt OutArcIt;
    typedef typename Digraph::InArcIt InArcIt;

    const Digraph &g;
    typename Digraph::template NodeMap<OutArcIt> nedge;
    typename Digraph::template EdgeMap<bool> visited;
    std::list<Arc> euler;

  public:

    ///Constructor

    ///\param _g An graph.
    ///\param start The starting point of the tour. If it is not given
    ///       the tour will start from the first node.
    EulerIt(const Digraph &_g,typename Digraph::Node start=INVALID)
      : g(_g), nedge(g), visited(g,false)
    {
      if(start==INVALID) start=NodeIt(g);
      for(NodeIt n(g);n!=INVALID;++n) nedge[n]=OutArcIt(g,n);
      while(nedge[start]!=INVALID) {
        euler.push_back(nedge[start]);
        visited[nedge[start]]=true;
        Node next=g.target(nedge[start]);
        ++nedge[start];
        start=next;
        while(nedge[start]!=INVALID && visited[nedge[start]]) ++nedge[start];
      }
    }

    ///Arc Conversion
    operator Arc() const { return euler.empty()?INVALID:euler.front(); }
    ///Arc Conversion
    operator Edge() const { return euler.empty()?INVALID:euler.front(); }
    ///\e
    bool operator==(Invalid) const { return euler.empty(); }
    ///\e
    bool operator!=(Invalid) const { return !euler.empty(); }

    ///Next arc of the tour
    EulerIt &operator++() {
      Node s=g.target(euler.front());
      euler.pop_front();
      typename std::list<Arc>::iterator next=euler.begin();

      while(nedge[s]!=INVALID) {
        while(nedge[s]!=INVALID && visited[nedge[s]]) ++nedge[s];
        if(nedge[s]==INVALID) break;
        else {
          euler.insert(next,nedge[s]);
          visited[nedge[s]]=true;
          Node n=g.target(nedge[s]);
          ++nedge[s];
          s=n;
        }
      }
      return *this;
    }

    ///Postfix incrementation

    ///\warning This incrementation
    ///returns an \c Arc, not an \ref EulerIt, as one may
    ///expect.
    Arc operator++(int)
    {
      Arc e=*this;
      ++(*this);
      return e;
    }
  };


  ///Checks if the graph is Eulerian

  /// \ingroup graph_prop
  ///Checks if the graph is Eulerian. It works for both directed and undirected
  ///graphs.
  ///\note By definition, a digraph is called \e Eulerian if
  ///and only if it is connected and the number of its incoming and outgoing
  ///arcs are the same for each node.
  ///Similarly, an undirected graph is called \e Eulerian if
  ///and only if it is connected and the number of incident arcs is even
  ///for each node. <em>Therefore, there are digraphs which are not Eulerian,
  ///but still have an Euler tour</em>.
  ///\todo Test required
  template<class Digraph>
#ifdef DOXYGEN
  bool
#else
  typename enable_if<UndirectedTagIndicator<Digraph>,bool>::type
  eulerian(const Digraph &g)
  {
    for(typename Digraph::NodeIt n(g);n!=INVALID;++n)
      if(countIncEdges(g,n)%2) return false;
    return connected(g);
  }
  template<class Digraph>
  typename disable_if<UndirectedTagIndicator<Digraph>,bool>::type
#endif
  eulerian(const Digraph &g)
  {
    for(typename Digraph::NodeIt n(g);n!=INVALID;++n)
      if(countInArcs(g,n)!=countOutArcs(g,n)) return false;
    return connected(Undirector<const Digraph>(g));
  }

}

#endif
