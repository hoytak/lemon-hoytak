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

#ifndef LEMON_TEST_TEST_TOOLS_H
#define LEMON_TEST_TEST_TOOLS_H

#include <iostream>
#include <vector>

#include <cstdlib>
#include <ctime>

#include <lemon/concept_check.h>
#include <lemon/concepts/digraph.h>

#include <lemon/random.h>

using namespace lemon;

//! \ingroup misc
//! \file
//! \brief Some utilities to write test programs.


///If \c rc is fail, writes an error message end exit.

///If \c rc is fail, writes an error message end exit.
///The error message contains the file name and the line number of the
///source code in a standard from, which makes it possible to go there
///using good source browsers like e.g. \c emacs.
///
///For example
///\code check(0==1,"This is obviously false.");\endcode will
///print this (and then exits).
///\verbatim digraph_test.cc:123: error: This is obviously false. \endverbatim
///
///\todo It should be in \c error.h
#define check(rc, msg) \
  if(!(rc)) { \
    std::cerr << __FILE__ ":" << __LINE__ << ": error: " << msg << std::endl; \
    abort(); \
  } else { } \

///Structure returned by \ref addPetersen().

///Structure returned by \ref addPetersen().
///
template<class Digraph> struct PetStruct
{
  ///Vector containing the outer nodes.
  std::vector<typename Digraph::Node> outer;
  ///Vector containing the inner nodes.
  std::vector<typename Digraph::Node> inner;
  ///Vector containing the arcs of the inner circle.
  std::vector<typename Digraph::Arc> incir;
  ///Vector containing the arcs of the outer circle.
  std::vector<typename Digraph::Arc> outcir;
  ///Vector containing the chord arcs.
  std::vector<typename Digraph::Arc> chords;
};



///Adds a Petersen digraph to \c G.

///Adds a Petersen digraph to \c G.
///\return The nodes and arcs of the generated digraph.

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

/// \brief Adds to the digraph the reverse pair of all arcs.
///
/// Adds to the digraph the reverse pair of all arcs.
///
template<class Digraph> void bidirDigraph(Digraph &G)
{
  typedef typename Digraph::Arc Arc;
  typedef typename Digraph::ArcIt ArcIt;
  
  std::vector<Arc> ee;
  
  for(ArcIt e(G);e!=INVALID;++e) ee.push_back(e);

  for(typename std::vector<Arc>::iterator p=ee.begin();p!=ee.end();p++)
    G.addArc(G.target(*p),G.source(*p));
}


/// \brief Checks the bidirectioned Petersen digraph.
///
///  Checks the bidirectioned Petersen digraph.
///
template<class Digraph> void checkBidirPetersen(Digraph &G, int num = 5)
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

///Structure returned by \ref addUPetersen().

///Structure returned by \ref addUPetersen().
///
template<class Digraph> struct UPetStruct
{
  ///Vector containing the outer nodes.
  std::vector<typename Digraph::Node> outer;
  ///Vector containing the inner nodes.
  std::vector<typename Digraph::Node> inner;
  ///Vector containing the arcs of the inner circle.
  std::vector<typename Digraph::Edge> incir;
  ///Vector containing the arcs of the outer circle.
  std::vector<typename Digraph::Edge> outcir;
  ///Vector containing the chord arcs.
  std::vector<typename Digraph::Edge> chords;
};

///Adds a Petersen digraph to the undirected \c G.

///Adds a Petersen digraph to the undirected \c G.
///\return The nodes and arcs of the generated digraph.

template<typename Digraph>
UPetStruct<Digraph> addUPetersen(Digraph &G,int num=5)
{
  UPetStruct<Digraph> n;

  for(int i=0;i<num;i++) {
    n.outer.push_back(G.addNode());
    n.inner.push_back(G.addNode());
  }

 for(int i=0;i<num;i++) {
   n.chords.push_back(G.addArc(n.outer[i],n.inner[i]));
   n.outcir.push_back(G.addArc(n.outer[i],n.outer[(i+1)%5]));
   n.incir.push_back(G.addArc(n.inner[i],n.inner[(i+2)%5]));
 }
 return n;
}

#endif
