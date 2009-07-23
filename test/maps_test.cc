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
#include <lemon/list_graph.h>

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
    checkConcept<WriteMap<int, bool>, LoggerBoolMap<vec::iterator> >();
    checkConcept<WriteMap<int, bool>,
                 LoggerBoolMap<std::back_insert_iterator<vec> > >();

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
  
  // IdMap, RangeIdMap
  {
    typedef ListDigraph Graph;
    DIGRAPH_TYPEDEFS(Graph);

    checkConcept<ReadMap<Node, int>, IdMap<Graph, Node> >();
    checkConcept<ReadMap<Arc, int>, IdMap<Graph, Arc> >();
    checkConcept<ReadMap<Node, int>, RangeIdMap<Graph, Node> >();
    checkConcept<ReadMap<Arc, int>, RangeIdMap<Graph, Arc> >();
    
    Graph gr;
    IdMap<Graph, Node> nmap(gr);
    IdMap<Graph, Arc> amap(gr);
    RangeIdMap<Graph, Node> nrmap(gr);
    RangeIdMap<Graph, Arc> armap(gr);
    
    Node n0 = gr.addNode();
    Node n1 = gr.addNode();
    Node n2 = gr.addNode();
    
    Arc a0 = gr.addArc(n0, n1);
    Arc a1 = gr.addArc(n0, n2);
    Arc a2 = gr.addArc(n2, n1);
    Arc a3 = gr.addArc(n2, n0);
    
    check(nmap[n0] == gr.id(n0) && nmap(gr.id(n0)) == n0, "Wrong IdMap");
    check(nmap[n1] == gr.id(n1) && nmap(gr.id(n1)) == n1, "Wrong IdMap");
    check(nmap[n2] == gr.id(n2) && nmap(gr.id(n2)) == n2, "Wrong IdMap");

    check(amap[a0] == gr.id(a0) && amap(gr.id(a0)) == a0, "Wrong IdMap");
    check(amap[a1] == gr.id(a1) && amap(gr.id(a1)) == a1, "Wrong IdMap");
    check(amap[a2] == gr.id(a2) && amap(gr.id(a2)) == a2, "Wrong IdMap");
    check(amap[a3] == gr.id(a3) && amap(gr.id(a3)) == a3, "Wrong IdMap");

    check(nmap.inverse()[gr.id(n0)] == n0, "Wrong IdMap::InverseMap");
    check(amap.inverse()[gr.id(a0)] == a0, "Wrong IdMap::InverseMap");
    
    check(nrmap.size() == 3 && armap.size() == 4,
          "Wrong RangeIdMap::size()");

    check(nrmap[n0] == 0 && nrmap(0) == n0, "Wrong RangeIdMap");
    check(nrmap[n1] == 1 && nrmap(1) == n1, "Wrong RangeIdMap");
    check(nrmap[n2] == 2 && nrmap(2) == n2, "Wrong RangeIdMap");
    
    check(armap[a0] == 0 && armap(0) == a0, "Wrong RangeIdMap");
    check(armap[a1] == 1 && armap(1) == a1, "Wrong RangeIdMap");
    check(armap[a2] == 2 && armap(2) == a2, "Wrong RangeIdMap");
    check(armap[a3] == 3 && armap(3) == a3, "Wrong RangeIdMap");

    check(nrmap.inverse()[0] == n0, "Wrong RangeIdMap::InverseMap");
    check(armap.inverse()[0] == a0, "Wrong RangeIdMap::InverseMap");
    
    gr.erase(n1);
    
    if (nrmap[n0] == 1) nrmap.swap(n0, n2);
    nrmap.swap(n2, n0);
    if (armap[a1] == 1) armap.swap(a1, a3);
    armap.swap(a3, a1);
    
    check(nrmap.size() == 2 && armap.size() == 2,
          "Wrong RangeIdMap::size()");

    check(nrmap[n0] == 1 && nrmap(1) == n0, "Wrong RangeIdMap");
    check(nrmap[n2] == 0 && nrmap(0) == n2, "Wrong RangeIdMap");
    
    check(armap[a1] == 1 && armap(1) == a1, "Wrong RangeIdMap");
    check(armap[a3] == 0 && armap(0) == a3, "Wrong RangeIdMap");

    check(nrmap.inverse()[0] == n2, "Wrong RangeIdMap::InverseMap");
    check(armap.inverse()[0] == a3, "Wrong RangeIdMap::InverseMap");
  }
  
  // CrossRefMap
  {
    typedef ListDigraph Graph;
    DIGRAPH_TYPEDEFS(Graph);

    checkConcept<ReadWriteMap<Node, int>,
                 CrossRefMap<Graph, Node, int> >();
    checkConcept<ReadWriteMap<Node, bool>,
                 CrossRefMap<Graph, Node, bool> >();
    checkConcept<ReadWriteMap<Node, double>,
                 CrossRefMap<Graph, Node, double> >();
    
    Graph gr;
    typedef CrossRefMap<Graph, Node, char> CRMap;
    typedef CRMap::ValueIterator ValueIt;
    CRMap map(gr);
    
    Node n0 = gr.addNode();
    Node n1 = gr.addNode();
    Node n2 = gr.addNode();
    
    map.set(n0, 'A');
    map.set(n1, 'B');
    map.set(n2, 'C');
    
    check(map[n0] == 'A' && map('A') == n0 && map.inverse()['A'] == n0,
          "Wrong CrossRefMap");
    check(map[n1] == 'B' && map('B') == n1 && map.inverse()['B'] == n1,
          "Wrong CrossRefMap");
    check(map[n2] == 'C' && map('C') == n2 && map.inverse()['C'] == n2,
          "Wrong CrossRefMap");
    check(map.count('A') == 1 && map.count('B') == 1 && map.count('C') == 1,
          "Wrong CrossRefMap::count()");
    
    ValueIt it = map.beginValue();
    check(*it++ == 'A' && *it++ == 'B' && *it++ == 'C' &&
          it == map.endValue(), "Wrong value iterator");
    
    map.set(n2, 'A');

    check(map[n0] == 'A' && map[n1] == 'B' && map[n2] == 'A',
          "Wrong CrossRefMap");
    check(map('A') == n0 && map.inverse()['A'] == n0, "Wrong CrossRefMap");
    check(map('B') == n1 && map.inverse()['B'] == n1, "Wrong CrossRefMap");
    check(map('C') == INVALID && map.inverse()['C'] == INVALID,
          "Wrong CrossRefMap");
    check(map.count('A') == 2 && map.count('B') == 1 && map.count('C') == 0,
          "Wrong CrossRefMap::count()");

    it = map.beginValue();
    check(*it++ == 'A' && *it++ == 'A' && *it++ == 'B' &&
          it == map.endValue(), "Wrong value iterator");

    map.set(n0, 'C');

    check(map[n0] == 'C' && map[n1] == 'B' && map[n2] == 'A',
          "Wrong CrossRefMap");
    check(map('A') == n2 && map.inverse()['A'] == n2, "Wrong CrossRefMap");
    check(map('B') == n1 && map.inverse()['B'] == n1, "Wrong CrossRefMap");
    check(map('C') == n0 && map.inverse()['C'] == n0, "Wrong CrossRefMap");
    check(map.count('A') == 1 && map.count('B') == 1 && map.count('C') == 1,
          "Wrong CrossRefMap::count()");

    it = map.beginValue();
    check(*it++ == 'A' && *it++ == 'B' && *it++ == 'C' &&
          it == map.endValue(), "Wrong value iterator");
  }

  return 0;
}
