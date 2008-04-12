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

#ifndef LEMON_COUNTER_H
#define LEMON_COUNTER_H

#include <string>
#include <iostream>

///\ingroup timecount
///\file
///\brief Tools for counting steps and events

namespace lemon 
{

  template<class P> class _NoSubCounter;

  template<class P>
  class _SubCounter 
  {
    P &_parent;
    std::string _title;
    std::ostream &_os;
    int count;
  public:

    typedef _SubCounter<_SubCounter<P> > SubCounter;
    typedef _NoSubCounter<_SubCounter<P> > NoSubCounter;

    _SubCounter(P &parent)
      : _parent(parent), _title(), _os(std::cerr), count(0) {}
    _SubCounter(P &parent,std::string title,std::ostream &os=std::cerr)
      : _parent(parent), _title(title), _os(os), count(0) {}
    _SubCounter(P &parent,const char *title,std::ostream &os=std::cerr)
      : _parent(parent), _title(title), _os(os), count(0) {}
    ~_SubCounter() { 
      _os << _title << count <<std::endl;
      _parent+=count;
    }
    _SubCounter &operator++() { count++; return *this;}
    int operator++(int) { return count++; }
    _SubCounter &operator--() { count--; return *this;}
    int operator--(int) { return count--; }
    _SubCounter &operator+=(int c) { count+=c; return *this;}
    _SubCounter &operator-=(int c) { count-=c; return *this;}
    void reset(int c=0) {count=c;}
    operator int() {return count;}
  };

  template<class P>
  class _NoSubCounter 
  {
    P &_parent;
  public:
    typedef _NoSubCounter<_NoSubCounter<P> > SubCounter;
    typedef _NoSubCounter<_NoSubCounter<P> > NoSubCounter;
  
    _NoSubCounter(P &parent) :_parent(parent) {}
    _NoSubCounter(P &parent,std::string,std::ostream &) 
      :_parent(parent) {}
    _NoSubCounter(P &parent,std::string) 
      :_parent(parent) {}
    _NoSubCounter(P &parent,const char *,std::ostream &)
      :_parent(parent) {}
    _NoSubCounter(P &parent,const char *)
      :_parent(parent) {}
    ~_NoSubCounter() {}
    _NoSubCounter &operator++() { ++_parent; return *this;}
    int operator++(int) { _parent++; return 0;}
    _NoSubCounter &operator--() { --_parent; return *this;}
    int operator--(int) { _parent--; return 0;}
    _NoSubCounter &operator+=(int c) { _parent+=c; return *this;}
    _NoSubCounter &operator-=(int c) { _parent-=c; return *this;}
    void reset(int) {}
    void reset() {}
    operator int() {return 0;}
  };


  /// \addtogroup timecount
  /// @{

  ///A counter class

  ///This class makes it easier to count certain events. You can increment
  ///or decrement the counter using operator++ and operator--.
  ///A report is automatically printed on destruction.
  ///\todo More doc
  class Counter 
  {
    std::string _title;
    std::ostream &_os;
    int count;
  public:
    ///\e

    ///\todo document please.
    ///
    typedef _SubCounter<Counter> SubCounter;
    ///\e

    ///\todo document please.
    ///
    typedef _NoSubCounter<Counter> NoSubCounter;

    ///\e
    Counter() : _title(), _os(std::cerr), count(0) {}
    ///\e
    Counter(std::string title,std::ostream &os=std::cerr) 
      : _title(title), _os(os), count(0) {}
    ///\e
    Counter(const char *title,std::ostream &os=std::cerr)
      : _title(title), _os(os), count(0) {}
    ///Destructor. Prints the given title and the value of the counter.
    ~Counter() {
      _os << _title << count <<std::endl;
    }
    ///\e
    Counter &operator++() { count++; return *this;}
    ///\e
    int operator++(int) { return count++;}
    ///\e
    Counter &operator--() { count--; return *this;}
    ///\e
    int operator--(int) { return count--;}
    ///\e
    Counter &operator+=(int c) { count+=c; return *this;}
    ///\e
    Counter &operator-=(int c) { count-=c; return *this;}
    ///\e
    void reset(int c=0) {count=c;}
    ///\e
    operator int() {return count;}
  };

  ///'Do nothing' version of \ref Counter

  ///'Do nothing' version of \ref Counter.
  ///\sa Counter
  class NoCounter
  {
  public:
    typedef _NoSubCounter<NoCounter> SubCounter;
    typedef _NoSubCounter<NoCounter> NoSubCounter;

    NoCounter() {}
    NoCounter(std::string,std::ostream &) {}
    NoCounter(const char *,std::ostream &) {}
    NoCounter(std::string) {}
    NoCounter(const char *) {}
    NoCounter &operator++() { return *this; }
    int operator++(int) { return 0; }
    NoCounter &operator--() { return *this; }
    int operator--(int) { return 0; }
    NoCounter &operator+=(int) { return *this;}
    NoCounter &operator-=(int) { return *this;}
    void reset(int) {}
    void reset() {}
    operator int() {return 0;}
  };

  ///@}
}

#endif
