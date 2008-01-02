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

#include <lemon/dim2.h>
#include <iostream>
#include "test_tools.h"

using namespace std;
using namespace lemon;

int main()
{
  cout << "Testing classes 'dim2::Point' and 'dim2::BoundingBox'." << endl;

  typedef dim2::Point<int> Point;

  Point p;
  check(p.size()==2, "Wrong vector initialization.");

  Point a(1,2);
  Point b(3,4);
  check(a[0]==1 && a[1]==2, "Wrong vector initialization.");

  p = a+b;
  check(p.x==4 && p.y==6, "Wrong vector addition.");

  p = a-b;
  check(p.x==-2 && p.y==-2, "Wrong vector subtraction.");

  check(a.normSquare()==5,"Wrong vector norm calculation.");
  check(a*b==11, "Wrong vector scalar product.");

  int l=2;
  p = a*l;
  check(p.x==2 && p.y==4, "Wrong vector multiplication by a scalar.");

  p = b/l;
  check(p.x==1 && p.y==2, "Wrong vector division by a scalar.");

  typedef dim2::BoundingBox<int> BB;
  BB box1;
  check(box1.empty(), "It should be empty.");

  box1.add(a);
  check(!box1.empty(), "It should not be empty.");
  box1.add(b);

  check(box1.bottomLeft().x==1 &&
        box1.bottomLeft().y==2 &&
        box1.topRight().x==3 &&
        box1.topRight().y==4,
        "Wrong addition of points to box.");

  p.x=2; p.y=3;
  check(box1.inside(p), "It should be inside.");

  p.x=1; p.y=3;
  check(box1.inside(p), "It should be inside.");

  p.x=0; p.y=3;
  check(!box1.inside(p), "It should not be inside.");

  BB box2(p);
  check(!box2.empty(),
        "It should not be empty. Constructed from 1 point.");

  box2.add(box1);
  check(box2.inside(p),
        "It should be inside. Incremented a box with another one.");

  return 0;
}
