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

#include <lemon/dim2.h>
#include <iostream>
#include "test_tools.h"

using namespace std;
using namespace lemon;

int main()
{
  typedef dim2::Point<int> Point;

  Point p;
  check(p.size()==2, "Wrong dim2::Point initialization.");

  Point a(1,2);
  Point b(3,4);
  check(a[0]==1 && a[1]==2, "Wrong dim2::Point initialization.");

  p = a+b;
  check(p.x==4 && p.y==6, "Wrong dim2::Point addition.");

  p = a-b;
  check(p.x==-2 && p.y==-2, "Wrong dim2::Point subtraction.");

  check(a.normSquare()==5,"Wrong dim2::Point norm calculation.");
  check(a*b==11, "Wrong dim2::Point scalar product.");

  int l=2;
  p = a*l;
  check(p.x==2 && p.y==4, "Wrong dim2::Point multiplication by a scalar.");

  p = b/l;
  check(p.x==1 && p.y==2, "Wrong dim2::Point division by a scalar.");

  typedef dim2::BoundingBox<int> BB;
  BB box1;
  check(box1.empty(), "Wrong empty() in dim2::BoundingBox.");

  box1.add(a);
  check(!box1.empty(), "Wrong empty() in dim2::BoundingBox.");
  box1.add(b);

  check(box1.bottomLeft().x==1 &&
        box1.bottomLeft().y==2 &&
        box1.topRight().x==3 &&
        box1.topRight().y==4,
        "Wrong addition of points to dim2::BoundingBox.");

  p.x=2; p.y=3;
  check(box1.inside(p), "Wrong inside() in dim2::BoundingBox.");

  p.x=1; p.y=3;
  check(box1.inside(p), "Wrong inside() in dim2::BoundingBox.");

  p.x=0; p.y=3;
  check(!box1.inside(p), "Wrong inside() in dim2::BoundingBox.");

  BB box2(p);
  check(!box2.empty(), "Wrong empty() in dim2::BoundingBox.");

  box2.add(box1);
  check(box2.inside(p), "Wrong inside() in dim2::BoundingBox.");

  return 0;
}
