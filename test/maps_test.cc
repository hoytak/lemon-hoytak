/* -*- mode: C++; indent-tabs-mode: nil; -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library.
 *
 * Copyright (C) 2003-2009
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

class C {
  int x;
public:
  C(int _x) : x(_x) {}
};

class F {
public:
  typedef A argument_type;
  typedef B result_type;

  B operator()(const A&) const { return B(); }
private:
  F& operator=(const F&);
};

int func(A) { return 3; }

int binc(int a, B) { return a+1; }

typedef ReadMap<A, double> DoubleMap;
typedef ReadWriteMap<A, double> DoubleWriteMap;
typedef ReferenceMap<A, double, double&, const double&> DoubleRefMap;

typedef ReadMap<A, bool> BoolMap;
typedef ReadWriteMap<A, bool> BoolWriteMap;
typedef ReferenceMap<A, bool, bool&, const bool&> BoolRefMap;

int main()
{
  // Map concepts
  checkConcept<ReadMap<A,B>, ReadMap<A,B> >();
  checkConcept<ReadMap<A,C>, ReadMap<A,C> >();
  checkConcept<WriteMap<A,B>, WriteMap<A,B> >();
  checkConcept<WriteMap<A,C>, WriteMap<A,C> >();
  checkConcept<ReadWriteMap<A,B>, ReadWriteMap<A,B> >();
  checkConcept<ReadWriteMap<A,C>, ReadWriteMap<A,C> >();
  checkConcept<ReferenceMap<A,B,B&,const B&>, ReferenceMap<A,B,B&,const B&> >();
  checkConcept<ReferenceMap<A,C,C&,const C&>, ReferenceMap<A,C,C&,const C&> >();

  // NullMap
  {
    checkConcept<ReadWriteMap<A,B>, NullMap<A,B> >();
    NullMap<A,B> map1;
    NullMap<A,B> map2 = map1;
    map1 = nullMap<A,B>();
  }

  // ConstMap
  {
    checkConcept<ReadWriteMap<A,B>, ConstMap<A,B> >();
    checkConcept<ReadWriteMap<A,C>, ConstMap<A,C> >();
    ConstMap<A,B> map1;
    ConstMap<A,B> map2 = B();
    ConstMap<A,B> map3 = map1;
    map1 = constMap<A>(B());
    map1 = constMap<A,B>();
    map1.setAll(B());
    ConstMap<A,C> map4(C(1));
    ConstMap<A,C> map5 = map4;
    map4 = constMap<A>(C(2));
    map4.setAll(C(3));

    checkConcept<ReadWriteMap<A,int>, ConstMap<A,int> >();
    check(constMap<A>(10)[A()] == 10, "Something is wrong with ConstMap");

    checkConcept<ReadWriteMap<A,int>, ConstMap<A,Const<int,10> > >();
    ConstMap<A,Const<int,10> > map6;
    ConstMap<A,Const<int,10> > map7 = map6;
    map6 = constMap<A,int,10>();
    map7 = constMap<A,Const<int,10> >();
    check(map6[A()] == 10 && map7[A()] == 10,
          "Something is wrong with ConstMap");
  }

  // IdentityMap
  {
    checkConcept<ReadMap<A,A>, IdentityMap<A> >();
    IdentityMap<A> map1;
    IdentityMap<A> map2 = map1;
    map1 = identityMap<A>();

    checkConcept<ReadMap<double,double>, IdentityMap<double> >();
    check(identityMap<double>()[1.0] == 1.0 &&
          identityMap<double>()[3.14] == 3.14,
          "Something is wrong with IdentityMap");
  }

  // RangeMap
  {
    checkConcept<ReferenceMap<int,B,B&,const B&>, RangeMap<B> >();
    RangeMap<B> map1;
    RangeMap<B> map2(10);
    RangeMap<B> map3(10,B());
    RangeMap<B> map4 = map1;
    RangeMap<B> map5 = rangeMap<B>();
    RangeMap<B> map6 = rangeMap<B>(10);
    RangeMap<B> map7 = rangeMap(10,B());

    checkConcept< ReferenceMap<int, double, double&, const double&>,
                  RangeMap<double> >();
    std::vector<double> v(10, 0);
    v[5] = 100;
    RangeMap<double> map8(v);
    RangeMap<double> map9 = rangeMap(v);
    check(map9.size() == 10 && map9[2] == 0 && map9[5] == 100,
          "Something is wrong with RangeMap");
  }

  // SparseMap
  {
    checkConcept<ReferenceMap<A,B,B&,const B&>, SparseMap<A,B> >();
    SparseMap<A,B> map1;
    SparseMap<A,B> map2 = B();
    SparseMap<A,B> map3 = sparseMap<A,B>();
    SparseMap<A,B> map4 = sparseMap<A>(B());

    checkConcept< ReferenceMap<double, int, int&, const int&>,
                  SparseMap<double, int> >();
    std::map<double, int> m;
    SparseMap<double, int> map5(m);
    SparseMap<double, int> map6(m,10);
    SparseMap<double, int> map7 = sparseMap(m);
    SparseMap<double, int> map8 = sparseMap(m,10);

    check(map5[1.0] == 0 && map5[3.14] == 0 &&
          map6[1.0] == 10 && map6[3.14] == 10,
          "Something is wrong with SparseMap");
    map5[1.0] = map6[3.14] = 100;
    check(map5[1.0] == 100 && map5[3.14] == 0 &&
          map6[1.0] == 10 && map6[3.14] == 100,
          "Something is wrong with SparseMap");
  }

  // ComposeMap
  {
    typedef ComposeMap<DoubleMap, ReadMap<B,A> > CompMap;
    checkConcept<ReadMap<B,double>, CompMap>();
    CompMap map1 = CompMap(DoubleMap(),ReadMap<B,A>());
    CompMap map2 = composeMap(DoubleMap(), ReadMap<B,A>());

    SparseMap<double, bool> m1(false); m1[3.14] = true;
    RangeMap<double> m2(2); m2[0] = 3.0; m2[1] = 3.14;
    check(!composeMap(m1,m2)[0] && composeMap(m1,m2)[1],
          "Something is wrong with ComposeMap")
  }

  // CombineMap
  {
    typedef CombineMap<DoubleMap, DoubleMap, std::plus<double> > CombMap;
    checkConcept<ReadMap<A,double>, CombMap>();
    CombMap map1 = CombMap(DoubleMap(), DoubleMap());
    CombMap map2 = combineMap(DoubleMap(), DoubleMap(), std::plus<double>());

    check(combineMap(constMap<B,int,2>(), identityMap<B>(), &binc)[B()] == 3,
          "Something is wrong with CombineMap");
  }

  // FunctorToMap, MapToFunctor
  {
    checkConcept<ReadMap<A,B>, FunctorToMap<F,A,B> >();
    checkConcept<ReadMap<A,B>, FunctorToMap<F> >();
    FunctorToMap<F> map1;
    FunctorToMap<F> map2 = FunctorToMap<F>(F());
    B b = functorToMap(F())[A()];

    checkConcept<ReadMap<A,B>, MapToFunctor<ReadMap<A,B> > >();
    MapToFunctor<ReadMap<A,B> > map = MapToFunctor<ReadMap<A,B> >(ReadMap<A,B>());

    check(functorToMap(&func)[A()] == 3,
          "Something is wrong with FunctorToMap");
    check(mapToFunctor(constMap<A,int>(2))(A()) == 2,
          "Something is wrong with MapToFunctor");
    check(mapToFunctor(functorToMap(&func))(A()) == 3 &&
          mapToFunctor(functorToMap(&func))[A()] == 3,
          "Something is wrong with FunctorToMap or MapToFunctor");
    check(functorToMap(mapToFunctor(constMap<A,int>(2)))[A()] == 2,
          "Something is wrong with FunctorToMap or MapToFunctor");
  }

  // ConvertMap
  {
    checkConcept<ReadMap<double,double>,
      ConvertMap<ReadMap<double, int>, double> >();
    ConvertMap<RangeMap<bool>, int> map1(rangeMap(1, true));
    ConvertMap<RangeMap<bool>, int> map2 = convertMap<int>(rangeMap(2, false));
  }

  // ForkMap
  {
    checkConcept<DoubleWriteMap, ForkMap<DoubleWriteMap, DoubleWriteMap> >();

    typedef RangeMap<double> RM;
    typedef SparseMap<int, double> SM;
    RM m1(10, -1);
    SM m2(-1);
    checkConcept<ReadWriteMap<int, double>, ForkMap<RM, SM> >();
    checkConcept<ReadWriteMap<int, double>, ForkMap<SM, RM> >();
    ForkMap<RM, SM> map1(m1,m2);
    ForkMap<SM, RM> map2 = forkMap(m2,m1);
    map2.set(5, 10);
    check(m1[1] == -1 && m1[5] == 10 && m2[1] == -1 &&
          m2[5] == 10 && map2[1] == -1 && map2[5] == 10,
          "Something is wrong with ForkMap");
  }

  // Arithmetic maps:
  // - AddMap, SubMap, MulMap, DivMap
  // - ShiftMap, ShiftWriteMap, ScaleMap, ScaleWriteMap
  // - NegMap, NegWriteMap, AbsMap
  {
    checkConcept<DoubleMap, AddMap<DoubleMap,DoubleMap> >();
    checkConcept<DoubleMap, SubMap<DoubleMap,DoubleMap> >();
    checkConcept<DoubleMap, MulMap<DoubleMap,DoubleMap> >();
    checkConcept<DoubleMap, DivMap<DoubleMap,DoubleMap> >();

    ConstMap<int, double> c1(1.0), c2(3.14);
    IdentityMap<int> im;
    ConvertMap<IdentityMap<int>, double> id(im);
    check(addMap(c1,id)[0] == 1.0  && addMap(c1,id)[10] == 11.0,
          "Something is wrong with AddMap");
    check(subMap(id,c1)[0] == -1.0 && subMap(id,c1)[10] == 9.0,
          "Something is wrong with SubMap");
    check(mulMap(id,c2)[0] == 0    && mulMap(id,c2)[2]  == 6.28,
          "Something is wrong with MulMap");
    check(divMap(c2,id)[1] == 3.14 && divMap(c2,id)[2]  == 1.57,
          "Something is wrong with DivMap");

    checkConcept<DoubleMap, ShiftMap<DoubleMap> >();
    checkConcept<DoubleWriteMap, ShiftWriteMap<DoubleWriteMap> >();
    checkConcept<DoubleMap, ScaleMap<DoubleMap> >();
    checkConcept<DoubleWriteMap, ScaleWriteMap<DoubleWriteMap> >();
    checkConcept<DoubleMap, NegMap<DoubleMap> >();
    checkConcept<DoubleWriteMap, NegWriteMap<DoubleWriteMap> >();
    checkConcept<DoubleMap, AbsMap<DoubleMap> >();

    check(shiftMap(id, 2.0)[1] == 3.0 && shiftMap(id, 2.0)[10] == 12.0,
          "Something is wrong with ShiftMap");
    check(shiftWriteMap(id, 2.0)[1] == 3.0 &&
          shiftWriteMap(id, 2.0)[10] == 12.0,
          "Something is wrong with ShiftWriteMap");
    check(scaleMap(id, 2.0)[1] == 2.0 && scaleMap(id, 2.0)[10] == 20.0,
          "Something is wrong with ScaleMap");
    check(scaleWriteMap(id, 2.0)[1] == 2.0 &&
          scaleWriteMap(id, 2.0)[10] == 20.0,
          "Something is wrong with ScaleWriteMap");
    check(negMap(id)[1] == -1.0 && negMap(id)[-10] == 10.0,
          "Something is wrong with NegMap");
    check(negWriteMap(id)[1] == -1.0 && negWriteMap(id)[-10] == 10.0,
          "Something is wrong with NegWriteMap");
    check(absMap(id)[1] == 1.0 && absMap(id)[-10] == 10.0,
          "Something is wrong with AbsMap");
  }

  // Logical maps:
  // - TrueMap, FalseMap
  // - AndMap, OrMap
  // - NotMap, NotWriteMap
  // - EqualMap, LessMap
  {
    checkConcept<BoolMap, TrueMap<A> >();
    checkConcept<BoolMap, FalseMap<A> >();
    checkConcept<BoolMap, AndMap<BoolMap,BoolMap> >();
    checkConcept<BoolMap, OrMap<BoolMap,BoolMap> >();
    checkConcept<BoolMap, NotMap<BoolMap> >();
    checkConcept<BoolWriteMap, NotWriteMap<BoolWriteMap> >();
    checkConcept<BoolMap, EqualMap<DoubleMap,DoubleMap> >();
    checkConcept<BoolMap, LessMap<DoubleMap,DoubleMap> >();

    TrueMap<int> tm;
    FalseMap<int> fm;
    RangeMap<bool> rm(2);
    rm[0] = true; rm[1] = false;
    check(andMap(tm,rm)[0] && !andMap(tm,rm)[1] &&
          !andMap(fm,rm)[0] && !andMap(fm,rm)[1],
          "Something is wrong with AndMap");
    check(orMap(tm,rm)[0] && orMap(tm,rm)[1] &&
          orMap(fm,rm)[0] && !orMap(fm,rm)[1],
          "Something is wrong with OrMap");
    check(!notMap(rm)[0] && notMap(rm)[1],
          "Something is wrong with NotMap");
    check(!notWriteMap(rm)[0] && notWriteMap(rm)[1],
          "Something is wrong with NotWriteMap");

    ConstMap<int, double> cm(2.0);
    IdentityMap<int> im;
    ConvertMap<IdentityMap<int>, double> id(im);
    check(lessMap(id,cm)[1] && !lessMap(id,cm)[2] && !lessMap(id,cm)[3],
          "Something is wrong with LessMap");
    check(!equalMap(id,cm)[1] && equalMap(id,cm)[2] && !equalMap(id,cm)[3],
          "Something is wrong with EqualMap");
  }

  // LoggerBoolMap
  {
    typedef std::vector<int> vec;
    vec v1;
    vec v2(10);
    LoggerBoolMap<std::back_insert_iterator<vec> >
      map1(std::back_inserter(v1));
    LoggerBoolMap<vec::iterator> map2(v2.begin());
    map1.set(10, false);
    map1.set(20, true);   map2.set(20, true);
    map1.set(30, false);  map2.set(40, false);
    map1.set(50, true);   map2.set(50, true);
    map1.set(60, true);   map2.set(60, true);
    check(v1.size() == 3 && v2.size() == 10 &&
          v1[0]==20 && v1[1]==50 && v1[2]==60 &&
          v2[0]==20 && v2[1]==50 && v2[2]==60,
          "Something is wrong with LoggerBoolMap");

    int i = 0;
    for ( LoggerBoolMap<vec::iterator>::Iterator it = map2.begin();
          it != map2.end(); ++it )
      check(v1[i++] == *it, "Something is wrong with LoggerBoolMap");
  }

  // Iterable bool map
  {
    typedef SmartGraph Graph;
    typedef SmartGraph::Node Item;

    typedef IterableBoolMap<SmartGraph, SmartGraph::Node> Ibm;
    checkConcept<ReadWriteMap<Item, int>, Ibm>();

    const int num = 10;
    Graph g;
    std::vector<Item> items;
    for (int i = 0; i < num; ++i) {
      items.push_back(g.addNode());
    }

    Ibm map1(g, true);
    int n = 0;
    for (Ibm::TrueIt it(map1); it != INVALID; ++it) {
      check(map1[static_cast<Item>(it)], "Wrong TrueIt");
      ++n;
    }
    check(n == num, "Wrong number");

    n = 0;
    for (Ibm::ItemIt it(map1, true); it != INVALID; ++it) {
        check(map1[static_cast<Item>(it)], "Wrong ItemIt for true");
        ++n;
    }
    check(n == num, "Wrong number");
    check(Ibm::FalseIt(map1) == INVALID, "Wrong FalseIt");
    check(Ibm::ItemIt(map1, false) == INVALID, "Wrong ItemIt for false");

    map1[items[5]] = true;

    n = 0;
    for (Ibm::ItemIt it(map1, true); it != INVALID; ++it) {
        check(map1[static_cast<Item>(it)], "Wrong ItemIt for true");
        ++n;
    }
    check(n == num, "Wrong number");

    map1[items[num / 2]] = false;
    check(map1[items[num / 2]] == false, "Wrong map value");

    n = 0;
    for (Ibm::TrueIt it(map1); it != INVALID; ++it) {
        check(map1[static_cast<Item>(it)], "Wrong TrueIt for true");
        ++n;
    }
    check(n == num - 1, "Wrong number");

    n = 0;
    for (Ibm::FalseIt it(map1); it != INVALID; ++it) {
        check(!map1[static_cast<Item>(it)], "Wrong FalseIt for true");
        ++n;
    }
    check(n == 1, "Wrong number");

    map1[items[0]] = false;
    check(map1[items[0]] == false, "Wrong map value");

    map1[items[num - 1]] = false;
    check(map1[items[num - 1]] == false, "Wrong map value");

    n = 0;
    for (Ibm::TrueIt it(map1); it != INVALID; ++it) {
        check(map1[static_cast<Item>(it)], "Wrong TrueIt for true");
        ++n;
    }
    check(n == num - 3, "Wrong number");
    check(map1.trueNum() == num - 3, "Wrong number");

    n = 0;
    for (Ibm::FalseIt it(map1); it != INVALID; ++it) {
        check(!map1[static_cast<Item>(it)], "Wrong FalseIt for true");
        ++n;
    }
    check(n == 3, "Wrong number");
    check(map1.falseNum() == 3, "Wrong number");
  }

  // Iterable int map
  {
    typedef SmartGraph Graph;
    typedef SmartGraph::Node Item;
    typedef IterableIntMap<SmartGraph, SmartGraph::Node> Iim;

    checkConcept<ReadWriteMap<Item, int>, Iim>();

    const int num = 10;
    Graph g;
    std::vector<Item> items;
    for (int i = 0; i < num; ++i) {
      items.push_back(g.addNode());
    }

    Iim map1(g);
    check(map1.size() == 0, "Wrong size");

    for (int i = 0; i < num; ++i) {
      map1[items[i]] = i;
    }
    check(map1.size() == num, "Wrong size");

    for (int i = 0; i < num; ++i) {
      Iim::ItemIt it(map1, i);
      check(static_cast<Item>(it) == items[i], "Wrong value");
      ++it;
      check(static_cast<Item>(it) == INVALID, "Wrong value");
    }

    for (int i = 0; i < num; ++i) {
      map1[items[i]] = i % 2;
    }
    check(map1.size() == 2, "Wrong size");

    int n = 0;
    for (Iim::ItemIt it(map1, 0); it != INVALID; ++it) {
      check(map1[static_cast<Item>(it)] == 0, "Wrong Value");
      ++n;
    }
    check(n == (num + 1) / 2, "Wrong number");

    for (Iim::ItemIt it(map1, 1); it != INVALID; ++it) {
      check(map1[static_cast<Item>(it)] == 1, "Wrong Value");
      ++n;
    }
    check(n == num, "Wrong number");

  }

  // Iterable value map
  {
    typedef SmartGraph Graph;
    typedef SmartGraph::Node Item;
    typedef IterableValueMap<SmartGraph, SmartGraph::Node, double> Ivm;

    checkConcept<ReadWriteMap<Item, double>, Ivm>();

    const int num = 10;
    Graph g;
    std::vector<Item> items;
    for (int i = 0; i < num; ++i) {
      items.push_back(g.addNode());
    }

    Ivm map1(g, 0.0);
    check(distance(map1.beginValue(), map1.endValue()) == 1, "Wrong size");
    check(*map1.beginValue() == 0.0, "Wrong value");

    for (int i = 0; i < num; ++i) {
      map1.set(items[i], static_cast<double>(i));
    }
    check(distance(map1.beginValue(), map1.endValue()) == num, "Wrong size");

    for (int i = 0; i < num; ++i) {
      Ivm::ItemIt it(map1, static_cast<double>(i));
      check(static_cast<Item>(it) == items[i], "Wrong value");
      ++it;
      check(static_cast<Item>(it) == INVALID, "Wrong value");
    }

    for (Ivm::ValueIterator vit = map1.beginValue();
         vit != map1.endValue(); ++vit) {
      check(map1[static_cast<Item>(Ivm::ItemIt(map1, *vit))] == *vit,
            "Wrong ValueIterator");
    }

    for (int i = 0; i < num; ++i) {
      map1.set(items[i], static_cast<double>(i % 2));
    }
    check(distance(map1.beginValue(), map1.endValue()) == 2, "Wrong size");

    int n = 0;
    for (Ivm::ItemIt it(map1, 0.0); it != INVALID; ++it) {
      check(map1[static_cast<Item>(it)] == 0.0, "Wrong Value");
      ++n;
    }
    check(n == (num + 1) / 2, "Wrong number");

    for (Ivm::ItemIt it(map1, 1.0); it != INVALID; ++it) {
      check(map1[static_cast<Item>(it)] == 1.0, "Wrong Value");
      ++n;
    }
    check(n == num, "Wrong number");

  }
  return 0;
}
