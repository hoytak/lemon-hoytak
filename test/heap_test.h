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

#include <vector>
#include <algorithm>

#include <lemon/dijkstra.h>

class IntIntMap : public std::vector<int> {
public:
  typedef std::vector<int> Parent;

  typedef int Key;
  typedef int Value;

  IntIntMap() : Parent() {}
  IntIntMap(int n) : Parent(n) {}
  IntIntMap(int n, int v) : Parent(n, v) {}

  void set(int key, int value) {
    Parent::operator[](key) = value;
  }
};


template <typename _Heap>
void heapSortTest(int n) {
  typedef _Heap Heap;
  IntIntMap map(n, -1);

  Heap heap(map);
  
  std::vector<int> v(n);

  for (int i = 0; i < n; ++i) {
    v[i] = rnd[1000];
    heap.push(i, v[i]);
  }
  std::sort(v.begin(), v.end());
  for (int i = 0; i < n; ++i) {
    check(v[i] == heap.prio() ,"Wrong order in heap sort.");
    heap.pop();
  }
}

template <typename _Heap>
void heapIncreaseTest(int n) {
  typedef _Heap Heap;
  IntIntMap map(n, -1);

  Heap heap(map);
  
  std::vector<int> v(n);

  for (int i = 0; i < n; ++i) {
    v[i] = rnd[1000];
    heap.push(i, v[i]);
  }
  for (int i = 0; i < n; ++i) {
    v[i] += rnd[1000];
    heap.increase(i, v[i]);
  }
  std::sort(v.begin(), v.end());
  for (int i = 0; i < n; ++i) {
    check(v[i] == heap.prio() ,"Wrong order in heap increase test.");
    heap.pop();
  }
}



template <typename _Digraph, typename _LengthMap, typename _Heap>
void dijkstraHeapTest(_Digraph& digraph, _LengthMap& length,
		      typename _Digraph::Node& start) {

  typedef _Heap Heap;
  typedef _Digraph Digraph;
  typedef _LengthMap LengthMap;

  typedef typename Digraph::Node Node;
  typedef typename Digraph::Arc Arc;
  typedef typename Digraph::NodeIt NodeIt;
  typedef typename Digraph::ArcIt ArcIt;

  typename Dijkstra<Digraph, LengthMap>::template DefStandardHeap<Heap>::
    Create dijkstra(digraph, length);

  dijkstra.run(start);

  for(ArcIt e(digraph); e!=INVALID; ++e) {
    Node u=digraph.source(e); 
    Node v=digraph.target(e);
    if (dijkstra.reached(u)) {
      check( dijkstra.dist(v) - dijkstra.dist(u) <= length[e],
      	     "Error in a shortest path tree arc!");
    }
  }

  for(NodeIt v(digraph); v!=INVALID; ++v) {
    if ( dijkstra.reached(v) && dijkstra.predArc(v) != INVALID ) {
      Arc e=dijkstra.predArc(v);
      Node u=digraph.source(e);
      check( dijkstra.dist(v) - dijkstra .dist(u) == length[e],
	     "Error in a shortest path tree arc!");
    }
  }

}
