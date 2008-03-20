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

#include <lemon/list_graph.h>
#include <lemon/maps.h>
#include <lemon/unionfind.h>
#include "test_tools.h"

using namespace lemon;
using namespace std;

typedef UnionFindEnum<ListGraph::NodeMap<int> > UFE;

int main() {
  ListGraph g;
  ListGraph::NodeMap<int> base(g);
  UFE U(base);
  vector<ListGraph::Node> n;
  
  for(int i=0;i<20;i++) n.push_back(g.addNode());

  U.insert(n[1]);
  U.insert(n[2]);

  check(U.join(n[1],n[2]) != -1,"Test failed.");

  U.insert(n[3]);
  U.insert(n[4]);
  U.insert(n[5]);
  U.insert(n[6]);
  U.insert(n[7]);


  check(U.join(n[1],n[4]) != -1,"Test failed.");
  check(U.join(n[2],n[4]) == -1,"Test failed.");
  check(U.join(n[3],n[5]) != -1,"Test failed.");


  U.insert(n[8],U.find(n[5]));


  check(U.size(U.find(n[4])) == 3,"Test failed.");
  check(U.size(U.find(n[5])) == 3,"Test failed.");
  check(U.size(U.find(n[6])) == 1,"Test failed.");
  check(U.size(U.find(n[2])) == 3,"Test failed.");


  U.insert(n[9]);
  U.insert(n[10],U.find(n[9]));


  check(U.join(n[8],n[10]) != -1,"Test failed.");


  check(U.size(U.find(n[4])) == 3,"Test failed.");
  check(U.size(U.find(n[9])) == 5,"Test failed.");

  check(U.size(U.find(n[8])) == 5,"Test failed.");

  U.erase(n[9]);
  U.erase(n[1]);

  check(U.size(U.find(n[10])) == 4,"Test failed.");
  check(U.size(U.find(n[2])) == 2,"Test failed.");

  U.erase(n[6]);
  U.split(U.find(n[8]));


  check(U.size(U.find(n[4])) == 2,"Test failed.");
  check(U.size(U.find(n[3])) == 1,"Test failed.");
  check(U.size(U.find(n[2])) == 2,"Test failed.");


  check(U.join(n[3],n[4]) != -1,"Test failed.");
  check(U.join(n[2],n[4]) == -1,"Test failed.");


  check(U.size(U.find(n[4])) == 3,"Test failed.");
  check(U.size(U.find(n[3])) == 3,"Test failed.");
  check(U.size(U.find(n[2])) == 3,"Test failed.");

  U.eraseClass(U.find(n[4]));
  U.eraseClass(U.find(n[7]));

  return 0;
}
