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

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <lemon/concept_check.h>
#include <lemon/concepts/heap.h>

#include <lemon/list_graph.h>

#include <lemon/digraph_reader.h>

#include <lemon/bin_heap.h>
#include <lemon/fib_heap.h>
#include <lemon/radix_heap.h>
#include <lemon/bucket_heap.h>

#include "test_tools.h"

#include "heap_test.h"

#include <lemon/time_measure.h>

using namespace lemon;
using namespace lemon::concepts;


int main() {

  typedef int Item;
  typedef int Prio;
  typedef IntIntMap ItemIntMap;

  typedef ListDigraph Digraph;

  typedef Digraph::Arc Arc;
  typedef Digraph::Node Node;
  typedef Digraph::ArcIt ArcIt;
  typedef Digraph::NodeIt NodeIt;
  typedef Digraph::ArcMap<int> LengthMap;

  Digraph digraph;
  LengthMap length(digraph);
  Node start;

  /// \todo create own test digraph

  std::string f_name;
  if( getenv("srcdir") )
    f_name = std::string(getenv("srcdir"));
  else f_name = ".";
  f_name += "/test/dijkstra_test.lgf";
  
  std::ifstream input(f_name.c_str());
  check(input, "Input file '" << f_name << "' not found.");
  DigraphReader<Digraph>(input, digraph).
    readArcMap("capacity", length).
    readNode("source", start).
    run();  
 
  {
    std::cerr << "Checking Bin Heap" << std::endl;

    typedef BinHeap<Prio, ItemIntMap> IntHeap;
    checkConcept<Heap<Prio, ItemIntMap>, IntHeap>();
    heapSortTest<IntHeap>(100);
    heapIncreaseTest<IntHeap>(100);
    
    typedef FibHeap<Prio, Digraph::NodeMap<int> > NodeHeap;
    checkConcept<Heap<Prio, Digraph::NodeMap<int> >, NodeHeap>();
    Timer timer;
    dijkstraHeapTest<Digraph, LengthMap, NodeHeap>(digraph, length, start);
    std::cout << timer << std::endl;
  }
  {
    std::cerr << "Checking Fib Heap" << std::endl;

    typedef FibHeap<Prio, ItemIntMap> IntHeap;
    checkConcept<Heap<Prio, ItemIntMap>, IntHeap>();
    heapSortTest<IntHeap>(100);
    heapIncreaseTest<IntHeap>(100);

    typedef FibHeap<Prio, Digraph::NodeMap<int> > NodeHeap;
    checkConcept<Heap<Prio, Digraph::NodeMap<int> >, NodeHeap>();
    Timer timer;
    dijkstraHeapTest<Digraph, LengthMap, NodeHeap>(digraph, length, start);
    std::cout << timer << std::endl;
  }
  {
    std::cerr << "Checking Radix Heap" << std::endl;

    typedef RadixHeap<ItemIntMap> IntHeap;
    checkConcept<Heap<Prio, ItemIntMap>, IntHeap>();
    heapSortTest<IntHeap>(100);
    heapIncreaseTest<IntHeap>(100);

    typedef RadixHeap<Digraph::NodeMap<int> > NodeHeap;
    checkConcept<Heap<Prio, Digraph::NodeMap<int> >, NodeHeap>();
    Timer timer;
    dijkstraHeapTest<Digraph, LengthMap, NodeHeap>(digraph, length, start);
    std::cout << timer << std::endl;
  }

  {
    std::cerr << "Checking Bucket Heap" << std::endl;

    typedef BucketHeap<ItemIntMap> IntHeap;
    checkConcept<Heap<Prio, ItemIntMap>, IntHeap>();
    heapSortTest<IntHeap>(100);
    heapIncreaseTest<IntHeap>(100);

    typedef BucketHeap<Digraph::NodeMap<int> > NodeHeap;
    checkConcept<Heap<Prio, Digraph::NodeMap<int> >, NodeHeap>();
    Timer timer;
    dijkstraHeapTest<Digraph, LengthMap, NodeHeap>(digraph, length, start);
    std::cout << timer << std::endl;
  }

  std::cout << __FILE__ ": All tests passed.\n";

  return 0;
}
