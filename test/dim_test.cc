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

  cout << "Testing classes `dim2::Point' and `dim2::BoundingBox'." << endl;

  typedef dim2::Point<int> Point;
	
  Point seged;
  check(seged.size()==2, "Wrong vector addition");

  Point a(1,2);
  Point b(3,4);

  check(a[0]==1 && a[1]==2, "Wrong vector addition");

  seged = a+b;
  check(seged.x==4 && seged.y==6, "Wrong vector addition");

  seged = a-b;
  check(seged.x==-2 && seged.y==-2, "a-b");

  check(a.normSquare()==5,"Wrong norm calculation");
  check(a*b==11, "a*b");

  int l=2;
  seged = a*l;
  check(seged.x==2 && seged.y==4, "a*l");

  seged = b/l;
  check(seged.x==1 && seged.y==2, "b/l");

  typedef dim2::BoundingBox<int> BB;
  BB doboz1;
  check(doboz1.empty(), "It should be empty.");
	
  doboz1.add(a);
  check(!doboz1.empty(), "It should not be empty.");
  doboz1.add(b);

  check(doboz1.bottomLeft().x==1 && 
        doboz1.bottomLeft().y==2 &&
        doboz1.topRight().x==3 && 
        doboz1.topRight().y==4,  
        "added points to box");

  seged.x=2;seged.y=3;
  check(doboz1.inside(seged),"It should be inside.");

  seged.x=1;seged.y=3;
  check(doboz1.inside(seged),"It should be inside.");

  seged.x=0;seged.y=3;
  check(!doboz1.inside(seged),"It should not be inside.");

  BB doboz2(seged);
  check(!doboz2.empty(),
        "It should not be empty. Constructed from 1 point.");

  doboz2.add(doboz1);
  check(doboz2.inside(seged),
        "It should be inside. Incremented a box with another one.");
}
