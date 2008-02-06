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

#include <deque>
#include <set>

#include <lemon/concept_check.h>
#include <lemon/concepts/maps.h>
#include <lemon/maps.h>

#include "test_tools.h"

using namespace lemon;
using namespace lemon::concepts;

struct A {};
inline bool operator<(A, A) { return true; }
struct B {};

class F {
public:
  typedef A argument_type;
  typedef B result_type;

  B operator()(const A &) const {return B();}
};

int func(A) {return 3;}

int binc(int, B) {return 4;}

typedef ReadMap<A,double> DoubleMap;
typedef ReadWriteMap<A, double> WriteDoubleMap;

typedef ReadMap<A,bool> BoolMap;
typedef ReadWriteMap<A, bool> BoolWriteMap;

int main()
{ // checking graph components
  
  checkConcept<ReadMap<A,B>, ReadMap<A,B> >();
  checkConcept<WriteMap<A,B>, WriteMap<A,B> >();
  checkConcept<ReadWriteMap<A,B>, ReadWriteMap<A,B> >();
  checkConcept<ReferenceMap<A,B,B&,const B&>, ReferenceMap<A,B,B&,const B&> >();

  checkConcept<ReadMap<A,double>, AddMap<DoubleMap,DoubleMap> >();
  checkConcept<ReadMap<A,double>, SubMap<DoubleMap,DoubleMap> >();
  checkConcept<ReadMap<A,double>, MulMap<DoubleMap,DoubleMap> >();
  checkConcept<ReadMap<A,double>, DivMap<DoubleMap,DoubleMap> >();
  checkConcept<ReadMap<A,double>, NegMap<DoubleMap> >();
  checkConcept<ReadWriteMap<A,double>, NegWriteMap<WriteDoubleMap> >();
  checkConcept<ReadMap<A,double>, AbsMap<DoubleMap> >();
  checkConcept<ReadMap<A,double>, ShiftMap<DoubleMap> >();
  checkConcept<ReadWriteMap<A,double>, ShiftWriteMap<WriteDoubleMap> >();
  checkConcept<ReadMap<A,double>, ScaleMap<DoubleMap> >();
  checkConcept<ReadWriteMap<A,double>, ScaleWriteMap<WriteDoubleMap> >();
  checkConcept<ReadMap<A,double>, ForkMap<DoubleMap, DoubleMap> >();
  checkConcept<ReadWriteMap<A,double>, 
    ForkWriteMap<WriteDoubleMap, WriteDoubleMap> >();
  
  checkConcept<ReadMap<B,double>, ComposeMap<DoubleMap,ReadMap<B,A> > >();

  checkConcept<ReadMap<A,B>, FunctorMap<F, A, B> >();

  checkConcept<ReadMap<A, bool>, NotMap<BoolMap> >();
  checkConcept<ReadWriteMap<A, bool>, NotWriteMap<BoolWriteMap> >();

  checkConcept<WriteMap<A, bool>, StoreBoolMap<A*> >();
  checkConcept<WriteMap<A, bool>, BackInserterBoolMap<std::deque<A> > >();
  checkConcept<WriteMap<A, bool>, FrontInserterBoolMap<std::deque<A> > >();
  checkConcept<WriteMap<A, bool>, InserterBoolMap<std::set<A> > >();
  checkConcept<WriteMap<A, bool>, FillBoolMap<WriteMap<A, B> > >();
  checkConcept<WriteMap<A, bool>, SettingOrderBoolMap<WriteMap<A, int> > >();

  int a;
  
  a=mapFunctor(constMap<A,int>(2))(A());
  check(a==2,"Something is wrong with mapFunctor");

  B b;
  b=functorMap(F())[A()];

  a=functorMap(&func)[A()];
  check(a==3,"Something is wrong with functorMap");

  a=combineMap(constMap<B, int, 1>(), identityMap<B>(), &binc)[B()];
  check(a==4,"Something is wrong with combineMap");
  

  std::cout << __FILE__ ": All tests passed.\n";
  
  return 0;
}
