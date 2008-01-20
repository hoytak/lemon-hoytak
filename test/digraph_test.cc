/* -*- C++ -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library
 *
 * Copyright (C) 2003-2007
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

#include <iostream>
#include <vector>

#include <lemon/concepts/digraph.h>
#include <lemon/list_graph.h>
//#include <lemon/smart_graph.h>
//#include <lemon/full_graph.h>
//#include <lemon/hypercube_graph.h>

#include "test_tools.h"
#include "digraph_test.h"
#include "map_test.h"


using namespace lemon;
using namespace lemon::concepts;


int main() {
  { // checking digraph components
    checkConcept<BaseDigraphComponent, BaseDigraphComponent >();

    checkConcept<IDableDigraphComponent<>, 
      IDableDigraphComponent<> >();

    checkConcept<IterableDigraphComponent<>, 
      IterableDigraphComponent<> >();

    checkConcept<MappableDigraphComponent<>, 
      MappableDigraphComponent<> >();

  }
  { // checking skeleton digraphs
    checkConcept<Digraph, Digraph>();
  }
  { // checking list digraph
    checkConcept<Digraph, ListDigraph >();
    checkConcept<AlterableDigraphComponent<>, ListDigraph>();
    checkConcept<ExtendableDigraphComponent<>, ListDigraph>();
    checkConcept<ClearableDigraphComponent<>, ListDigraph>();
    checkConcept<ErasableDigraphComponent<>, ListDigraph>();

    checkDigraph<ListDigraph>();
    checkGraphNodeMap<ListDigraph>();
    checkGraphArcMap<ListDigraph>();
  }
//   { // checking smart digraph
//     checkConcept<Digraph, SmartDigraph >();

//     checkDigraph<SmartDigraph>();
//     checkDigraphNodeMap<SmartDigraph>();
//     checkDigraphArcMap<SmartDigraph>();
//   }
//   { // checking full digraph
//     checkConcept<Digraph, FullDigraph >();
//   }
//   { // checking full digraph
//     checkConcept<Digraph, HyperCubeDigraph >();
//   }

  std::cout << __FILE__ ": All tests passed.\n";

  return 0;
}
