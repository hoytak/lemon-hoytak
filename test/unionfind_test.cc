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

#include <lemon/maps.h>
#include <lemon/unionfind.h>
#include "test_tools.h"

using namespace lemon;
using namespace std;

typedef UnionFindEnum<StdMap<int, int> > UFE;

void print(UFE const &ufe) {
  cout << "Print the classes of the structure:" << endl;
  int i = 1;
  for (UFE::ClassIt cit(ufe); cit != INVALID; ++cit) {
    cout << "  " << i << " (" << cit << "):" << flush;
    for (UFE::ItemIt iit(ufe, cit); iit != INVALID; ++iit) {
      cout << " " << iit << flush;
    }
    cout << endl;
    i++;
  }
  cout << "done" << endl;
}


int main() {
  StdMap<int, int> base;
  UFE U(base);

  U.insert(1);
  U.insert(2);

  check(U.join(1,2) != -1,"Test failed.");

  U.insert(3);
  U.insert(4);
  U.insert(5);
  U.insert(6);
  U.insert(7);


  check(U.join(1,4) != -1,"Test failed.");
  check(U.join(2,4) == -1,"Test failed.");
  check(U.join(3,5) != -1,"Test failed.");


  U.insert(8,U.find(5));


  check(U.size(U.find(4)) == 3,"Test failed.");
  check(U.size(U.find(5)) == 3,"Test failed.");
  check(U.size(U.find(6)) == 1,"Test failed.");
  check(U.size(U.find(2)) == 3,"Test failed.");


  U.insert(9);
  U.insert(10,U.find(9));


  check(U.join(8,10) != -1,"Test failed.");


  check(U.size(U.find(4)) == 3,"Test failed.");
  check(U.size(U.find(9)) == 5,"Test failed.");

  check(U.size(U.find(8)) == 5,"Test failed.");

  U.erase(9);
  U.erase(1);

  check(U.size(U.find(10)) == 4,"Test failed.");
  check(U.size(U.find(2)) == 2,"Test failed.");

  U.erase(6);
  U.split(U.find(8));


  check(U.size(U.find(4)) == 2,"Test failed.");
  check(U.size(U.find(3)) == 1,"Test failed.");
  check(U.size(U.find(2)) == 2,"Test failed.");


  check(U.join(3,4) != -1,"Test failed.");
  check(U.join(2,4) == -1,"Test failed.");


  check(U.size(U.find(4)) == 3,"Test failed.");
  check(U.size(U.find(3)) == 3,"Test failed.");
  check(U.size(U.find(2)) == 3,"Test failed.");

  U.eraseClass(U.find(4));
  U.eraseClass(U.find(7));

  return 0;
}
