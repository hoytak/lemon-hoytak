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

///\file \brief Test cases for time_measure.h
///
///\todo To be extended


int fibonacci(int f) 
{
  static lemon::Counter count("Fibonacci steps: ");
  count++;
  if(f<1) return 0;
  else if(f==1) return 1;
  else return fibonacci(f-1)+fibonacci(f-2);
}

int main()
{
  fibonacci(10);
  
  {  
    typedef lemon::Counter MyCounter;
    MyCounter c("Main counter: ");
    c++;
    c++;
    MyCounter::SubCounter d(c,"Subcounter: ");
    d++;
    d++;
    MyCounter::SubCounter::SubCounter e(d,"SubSubCounter: ");
    e++;
    e++;
  }
  
  {
    typedef lemon::NoCounter MyCounter;
    MyCounter c("Main counter: ");
    c++;
    c++;
    MyCounter::SubCounter d(c,"Subcounter: ");
    d++;
    d++;
    MyCounter::SubCounter::SubCounter e(d,"SubSubCounter: ");
    e++;
    e++;
  }

  return 0;
}
