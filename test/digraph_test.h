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

#ifndef LEMON_TEST_GRAPH_TEST_H
#define LEMON_TEST_GRAPH_TEST_H

//#include <lemon/graph_utils.h>
#include "test_tools.h"

//! \ingroup misc
//! \file
//! \brief Some utility and test cases to test digraph classes.
namespace lemon {

  ///Structure returned by \ref addPetersen().

  ///Structure returned by \ref addPetersen().
  ///
  template<class Digraph> 
  struct PetStruct
  {
    ///Vector containing the outer nodes.
    std::vector<typename Digraph::Node> outer;
    ///Vector containing the inner nodes.
    std::vector<typename Digraph::Node> inner;
    ///Vector containing the edges of the inner circle.
    std::vector<typename Digraph::Arc> incir;
    ///Vector containing the edges of the outer circle.
    std::vector<typename Digraph::Arc> outcir;
    ///Vector containing the chord edges.
    std::vector<typename Digraph::Arc> chords;
  };



  ///Adds a Petersen graph to \c G.

  ///Adds a Petersen graph to \c G.
  ///\return The nodes and edges of the generated graph.

  template<typename Digraph>
  PetStruct<Digraph> addPetersen(Digraph &G,int num = 5)
  {
    PetStruct<Digraph> n;

    for(int i=0;i<num;i++) {
      n.outer.push_back(G.addNode());
      n.inner.push_back(G.addNode());
    }

    for(int i=0;i<num;i++) {
      n.chords.push_back(G.addArc(n.outer[i],n.inner[i]));
      n.outcir.push_back(G.addArc(n.outer[i],n.outer[(i+1) % num]));
      n.incir.push_back(G.addArc(n.inner[i],n.inner[(i+2) % num]));
    }
    return n;
  }

  /// \brief Adds to the digraph the reverse pair of all edges.
  ///
  /// Adds to the digraph the reverse pair of all edges.
  ///
  template<class Digraph> 
  void bidirDigraph(Digraph &G)
  {
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::ArcIt ArcIt;
  
    std::vector<Arc> ee;
  
    for(ArcIt e(G);e!=INVALID;++e) ee.push_back(e);

    for(typename std::vector<Arc>::iterator p=ee.begin();p!=ee.end();p++)
      G.addArc(G.target(*p),G.source(*p));
  }


  /// \brief Checks the bidirectioned Petersen graph.
  ///
  ///  Checks the bidirectioned Petersen graph.
  ///
  template<class Digraph> 
  void checkBidirPetersen(Digraph &G, int num = 5)
  {
    typedef typename Digraph::Node Node;

    typedef typename Digraph::ArcIt ArcIt;
    typedef typename Digraph::NodeIt NodeIt;

    checkDigraphNodeList(G, 2 * num);
    checkDigraphArcList(G, 6 * num);

    for(NodeIt n(G);n!=INVALID;++n) {
      checkDigraphInArcList(G, n, 3);
      checkDigraphOutArcList(G, n, 3);
    }  
  }

  template<class Digraph> void checkDigraphNodeList(Digraph &G, int nn)
  {
    typename Digraph::NodeIt n(G);
    for(int i=0;i<nn;i++) {
      check(n!=INVALID,"Wrong Node list linking.");
      ++n;
    }
    check(n==INVALID,"Wrong Node list linking.");
  }

  template<class Digraph>
  void checkDigraphArcList(Digraph &G, int nn)
  {
    typedef typename Digraph::ArcIt ArcIt;

    ArcIt e(G);
    for(int i=0;i<nn;i++) {
      check(e!=INVALID,"Wrong Arc list linking.");
      ++e;
    }
    check(e==INVALID,"Wrong Arc list linking.");
  }

  template<class Digraph> 
  void checkDigraphOutArcList(Digraph &G, typename Digraph::Node n, int nn)
  {
    typename Digraph::OutArcIt e(G,n);
    for(int i=0;i<nn;i++) {
      check(e!=INVALID,"Wrong OutArc list linking.");
      check(n==G.source(e), "Wrong OutArc list linking.");
      ++e;
    }
    check(e==INVALID,"Wrong OutArc list linking.");
  }

  template<class Digraph> void 
  checkDigraphInArcList(Digraph &G, typename Digraph::Node n, int nn)
  {
    typename Digraph::InArcIt e(G,n);
    for(int i=0;i<nn;i++) {
      check(e!=INVALID,"Wrong InArc list linking.");
      check(n==G.target(e), "Wrong InArc list linking.");
      ++e;
    }
    check(e==INVALID,"Wrong InArc list linking.");
  }

  template <class Digraph> 
  void checkDigraph() {
    const int num = 5;
    Digraph G;
    addPetersen(G, num);
    bidirDigraph(G);
    checkBidirPetersen(G, num);
  }

  template <class Digraph> 
  void checkDigraphIterators(const Digraph& digraph) {
    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::ArcIt ArcIt;
    typedef typename Digraph::InArcIt InArcIt;
    typedef typename Digraph::OutArcIt OutArcIt;
    //    typedef ConArcIt<Digraph> ConArcIt;
  }

  ///\file
  ///\todo Check target(), source() as well;

  
} //namespace lemon


#endif
