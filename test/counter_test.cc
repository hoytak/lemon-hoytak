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

#include <lemon/counter.h>
#include <vector>

using namespace lemon;

template <typename T>
void bubbleSort(std::vector<T>& v) {
  Counter op("Bubble Sort - Operations: ");
  Counter::NoSubCounter as(op, "Assignments: ");
  Counter::NoSubCounter co(op, "Comparisons: ");
  for (int i = v.size()-1; i > 0; --i) {
    for (int j = 0; j < i; ++j) {
      if (v[j] > v[j+1]) {
        T tmp = v[j];
        v[j] = v[j+1];
        v[j+1] = tmp;
        as += 3;
      }
      ++co;
    }
  }
}

template <typename T>
void insertionSort(std::vector<T>& v) {
  Counter op("Insertion Sort - Operations: ");
  Counter::NoSubCounter as(op, "Assignments: ");
  Counter::NoSubCounter co(op, "Comparisons: ");
  for (int i = 1; i < int(v.size()); ++i) {
    T value = v[i];
    ++as;
    int j = i;
    while (j > 0 && v[j-1] > value) {
      v[j] = v[j-1];
      --j;
      ++co; ++as;
    }
    v[j] = value;
    ++as;
  }
}

template <typename MyCounter>
void counterTest() {
  MyCounter c("Main Counter: ");
  c++;
  typename MyCounter::SubCounter d(c, "SubCounter: ");
  d++;
  typename MyCounter::SubCounter::NoSubCounter e(d, "SubSubCounter: ");
  e++;
  d+=3;
  c-=4;
  e-=2;
  c.reset(2);
  c.reset();
}

void init(std::vector<int>& v) {
  v[0] = 10; v[1] = 60; v[2] = 20; v[3] = 90; v[4] = 100;
  v[5] = 80; v[6] = 40; v[7] = 30; v[8] = 50; v[9] = 70; 
}

int main()
{
  counterTest<Counter>();
  counterTest<NoCounter>();
  
  std::vector<int> x(10);
  init(x); bubbleSort(x);
  init(x); insertionSort(x);

  return 0;
}
