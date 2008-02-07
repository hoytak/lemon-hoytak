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

#include <lemon/error.h>
#include "test_tools.h"

using namespace lemon;
using std::cout;
using std::endl;

void faulty_fn() {
  fault("This is a fault message");
}

void exception_fn() {
  throw Exception("This is a function throwing exception with some args: ")
    << 5 << ", " << 18;
}

void unfinished_fn() {
  LEMON_FIXME("unfinished_fn() is unfinished!");
}


int main() {
  try {
    faulty_fn();
    check(false, "A faulty function did not fail.");
  }
  catch(const Exception &e) {
    cout << "Exeption = \"" << e.what() << "\" (Right behaviour)" << endl;
  }

  try {
    exception_fn();
    check(false, "The function did not throw Exception.");
  }
  catch(const Exception &e) {
    cout << "Exeption = \"" << e.what() << "\" (Right behaviour)" << endl;
  }

  try {
    unfinished_fn();
    check(false, "FIXME macro does not work.");
  }
  catch(const Exception &e) {
    cout << "Exeption = \"" << e.what() << "\" (Right behaviour)" << endl;
  }

  return 0;
}
