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

#ifndef LEMON_FIB_HEAP_H
#define LEMON_FIB_HEAP_H

///\file
///\ingroup auxdat
///\brief Fibonacci Heap implementation.

#include <vector>
#include <functional>
#include <lemon/math.h>

namespace lemon {

  /// \ingroup auxdat
  ///
  ///\brief Fibonacci Heap.
  ///
  ///This class implements the \e Fibonacci \e heap data structure. A \e heap
  ///is a data structure for storing items with specified values called \e
  ///priorities in such a way that finding the item with minimum priority is
  ///efficient. \c CMP specifies the ordering of the priorities. In a heap
  ///one can change the priority of an item, add or erase an item, etc.
  ///
  ///The methods \ref increase and \ref erase are not efficient in a Fibonacci
  ///heap. In case of many calls to these operations, it is better to use a
  ///\ref BinHeap "binary heap".
  ///
  ///\param PRIO Type of the priority of the items.
  ///\param IM A read and writable Item int map, used internally
  ///to handle the cross references.
  ///\param CMP A class for the ordering of the priorities. The
  ///default is \c std::less<PRIO>.
  ///
  ///\sa BinHeap
  ///\sa Dijkstra
#ifdef DOXYGEN
  template <typename PRIO, typename IM, typename CMP>
#else
  template <typename PRIO, typename IM, typename CMP = std::less<PRIO> >
#endif
  class FibHeap {
  public:
    ///\e
    typedef IM ItemIntMap;
    ///\e
    typedef PRIO Prio;
    ///\e
    typedef typename ItemIntMap::Key Item;
    ///\e
    typedef std::pair<Item,Prio> Pair;
    ///\e
    typedef CMP Compare;

  private:
    class Store;

    std::vector<Store> _data;
    int _minimum;
    ItemIntMap &_iim;
    Compare _comp;
    int _num;

  public:

    /// \brief Type to represent the items states.
    ///
    /// Each Item element have a state associated to it. It may be "in heap",
    /// "pre heap" or "post heap". The latter two are indifferent from the
    /// heap's point of view, but may be useful to the user.
    ///
    /// The item-int map must be initialized in such way that it assigns
    /// \c PRE_HEAP (<tt>-1</tt>) to any element to be put in the heap.
    enum State {
      IN_HEAP = 0,    ///< = 0.
      PRE_HEAP = -1,  ///< = -1.
      POST_HEAP = -2  ///< = -2.
    };

    /// \brief The constructor
    ///
    /// \c map should be given to the constructor, since it is
    ///   used internally to handle the cross references.
    explicit FibHeap(ItemIntMap &map)
      : _minimum(0), _iim(map), _num() {}

    /// \brief The constructor
    ///
    /// \c map should be given to the constructor, since it is used
    /// internally to handle the cross references. \c comp is an
    /// object for ordering of the priorities.
    FibHeap(ItemIntMap &map, const Compare &comp)
      : _minimum(0), _iim(map), _comp(comp), _num() {}

    /// \brief The number of items stored in the heap.
    ///
    /// Returns the number of items stored in the heap.
    int size() const { return _num; }

    /// \brief Checks if the heap stores no items.
    ///
    ///   Returns \c true if and only if the heap stores no items.
    bool empty() const { return _num==0; }

    /// \brief Make empty this heap.
    ///
    /// Make empty this heap. It does not change the cross reference
    /// map.  If you want to reuse a heap what is not surely empty you
    /// should first clear the heap and after that you should set the
    /// cross reference map for each item to \c PRE_HEAP.
    void clear() {
      _data.clear(); _minimum = 0; _num = 0;
    }

    /// \brief \c item gets to the heap with priority \c value independently
    /// if \c item was already there.
    ///
    /// This method calls \ref push(\c item, \c value) if \c item is not
    /// stored in the heap and it calls \ref decrease(\c item, \c value) or
    /// \ref increase(\c item, \c value) otherwise.
    void set (const Item& item, const Prio& value) {
      int i=_iim[item];
      if ( i >= 0 && _data[i].in ) {
        if ( _comp(value, _data[i].prio) ) decrease(item, value);
        if ( _comp(_data[i].prio, value) ) increase(item, value);
      } else push(item, value);
    }

    /// \brief Adds \c item to the heap with priority \c value.
    ///
    /// Adds \c item to the heap with priority \c value.
    /// \pre \c item must not be stored in the heap.
    void push (const Item& item, const Prio& value) {
      int i=_iim[item];
      if ( i < 0 ) {
        int s=_data.size();
        _iim.set( item, s );
        Store st;
        st.name=item;
        _data.push_back(st);
        i=s;
      } else {
        _data[i].parent=_data[i].child=-1;
        _data[i].degree=0;
        _data[i].in=true;
        _data[i].marked=false;
      }

      if ( _num ) {
        _data[_data[_minimum].right_neighbor].left_neighbor=i;
        _data[i].right_neighbor=_data[_minimum].right_neighbor;
        _data[_minimum].right_neighbor=i;
        _data[i].left_neighbor=_minimum;
        if ( _comp( value, _data[_minimum].prio) ) _minimum=i;
      } else {
        _data[i].right_neighbor=_data[i].left_neighbor=i;
        _minimum=i;
      }
      _data[i].prio=value;
      ++_num;
    }

    /// \brief Returns the item with minimum priority relative to \c Compare.
    ///
    /// This method returns the item with minimum priority relative to \c
    /// Compare.
    /// \pre The heap must be nonempty.
    Item top() const { return _data[_minimum].name; }

    /// \brief Returns the minimum priority relative to \c Compare.
    ///
    /// It returns the minimum priority relative to \c Compare.
    /// \pre The heap must be nonempty.
    const Prio& prio() const { return _data[_minimum].prio; }

    /// \brief Returns the priority of \c item.
    ///
    /// It returns the priority of \c item.
    /// \pre \c item must be in the heap.
    const Prio& operator[](const Item& item) const {
      return _data[_iim[item]].prio;
    }

    /// \brief Deletes the item with minimum priority relative to \c Compare.
    ///
    /// This method deletes the item with minimum priority relative to \c
    /// Compare from the heap.
    /// \pre The heap must be non-empty.
    void pop() {
      /*The first case is that there are only one root.*/
      if ( _data[_minimum].left_neighbor==_minimum ) {
        _data[_minimum].in=false;
        if ( _data[_minimum].degree!=0 ) {
          makeroot(_data[_minimum].child);
          _minimum=_data[_minimum].child;
          balance();
        }
      } else {
        int right=_data[_minimum].right_neighbor;
        unlace(_minimum);
        _data[_minimum].in=false;
        if ( _data[_minimum].degree > 0 ) {
          int left=_data[_minimum].left_neighbor;
          int child=_data[_minimum].child;
          int last_child=_data[child].left_neighbor;

          makeroot(child);

          _data[left].right_neighbor=child;
          _data[child].left_neighbor=left;
          _data[right].left_neighbor=last_child;
          _data[last_child].right_neighbor=right;
        }
        _minimum=right;
        balance();
      } // the case where there are more roots
      --_num;
    }

    /// \brief Deletes \c item from the heap.
    ///
    /// This method deletes \c item from the heap, if \c item was already
    /// stored in the heap. It is quite inefficient in Fibonacci heaps.
    void erase (const Item& item) {
      int i=_iim[item];

      if ( i >= 0 && _data[i].in ) {
        if ( _data[i].parent!=-1 ) {
          int p=_data[i].parent;
          cut(i,p);
          cascade(p);
        }
        _minimum=i;     //As if its prio would be -infinity
        pop();
      }
    }

    /// \brief Decreases the priority of \c item to \c value.
    ///
    /// This method decreases the priority of \c item to \c value.
    /// \pre \c item must be stored in the heap with priority at least \c
    ///   value relative to \c Compare.
    void decrease (Item item, const Prio& value) {
      int i=_iim[item];
      _data[i].prio=value;
      int p=_data[i].parent;

      if ( p!=-1 && _comp(value, _data[p].prio) ) {
        cut(i,p);
        cascade(p);
      }
      if ( _comp(value, _data[_minimum].prio) ) _minimum=i;
    }

    /// \brief Increases the priority of \c item to \c value.
    ///
    /// This method sets the priority of \c item to \c value. Though
    /// there is no precondition on the priority of \c item, this
    /// method should be used only if it is indeed necessary to increase
    /// (relative to \c Compare) the priority of \c item, because this
    /// method is inefficient.
    void increase (Item item, const Prio& value) {
      erase(item);
      push(item, value);
    }


    /// \brief Returns if \c item is in, has already been in, or has never
    /// been in the heap.
    ///
    /// This method returns PRE_HEAP if \c item has never been in the
    /// heap, IN_HEAP if it is in the heap at the moment, and POST_HEAP
    /// otherwise. In the latter case it is possible that \c item will
    /// get back to the heap again.
    State state(const Item &item) const {
      int i=_iim[item];
      if( i>=0 ) {
        if ( _data[i].in ) i=0;
        else i=-2;
      }
      return State(i);
    }

    /// \brief Sets the state of the \c item in the heap.
    ///
    /// Sets the state of the \c item in the heap. It can be used to
    /// manually clear the heap when it is important to achive the
    /// better time _complexity.
    /// \param i The item.
    /// \param st The state. It should not be \c IN_HEAP.
    void state(const Item& i, State st) {
      switch (st) {
      case POST_HEAP:
      case PRE_HEAP:
        if (state(i) == IN_HEAP) {
          erase(i);
        }
        _iim[i] = st;
        break;
      case IN_HEAP:
        break;
      }
    }

  private:

    void balance() {

      int maxdeg=int( std::floor( 2.08*log(double(_data.size()))))+1;

      std::vector<int> A(maxdeg,-1);

      /*
       *Recall that now minimum does not point to the minimum prio element.
       *We set minimum to this during balance().
       */
      int anchor=_data[_minimum].left_neighbor;
      int next=_minimum;
      bool end=false;

      do {
        int active=next;
        if ( anchor==active ) end=true;
        int d=_data[active].degree;
        next=_data[active].right_neighbor;

        while (A[d]!=-1) {
          if( _comp(_data[active].prio, _data[A[d]].prio) ) {
            fuse(active,A[d]);
          } else {
            fuse(A[d],active);
            active=A[d];
          }
          A[d]=-1;
          ++d;
        }
        A[d]=active;
      } while ( !end );


      while ( _data[_minimum].parent >=0 )
        _minimum=_data[_minimum].parent;
      int s=_minimum;
      int m=_minimum;
      do {
        if ( _comp(_data[s].prio, _data[_minimum].prio) ) _minimum=s;
        s=_data[s].right_neighbor;
      } while ( s != m );
    }

    void makeroot(int c) {
      int s=c;
      do {
        _data[s].parent=-1;
        s=_data[s].right_neighbor;
      } while ( s != c );
    }

    void cut(int a, int b) {
      /*
       *Replacing a from the children of b.
       */
      --_data[b].degree;

      if ( _data[b].degree !=0 ) {
        int child=_data[b].child;
        if ( child==a )
          _data[b].child=_data[child].right_neighbor;
        unlace(a);
      }


      /*Lacing a to the roots.*/
      int right=_data[_minimum].right_neighbor;
      _data[_minimum].right_neighbor=a;
      _data[a].left_neighbor=_minimum;
      _data[a].right_neighbor=right;
      _data[right].left_neighbor=a;

      _data[a].parent=-1;
      _data[a].marked=false;
    }

    void cascade(int a) {
      if ( _data[a].parent!=-1 ) {
        int p=_data[a].parent;

        if ( _data[a].marked==false ) _data[a].marked=true;
        else {
          cut(a,p);
          cascade(p);
        }
      }
    }

    void fuse(int a, int b) {
      unlace(b);

      /*Lacing b under a.*/
      _data[b].parent=a;

      if (_data[a].degree==0) {
        _data[b].left_neighbor=b;
        _data[b].right_neighbor=b;
        _data[a].child=b;
      } else {
        int child=_data[a].child;
        int last_child=_data[child].left_neighbor;
        _data[child].left_neighbor=b;
        _data[b].right_neighbor=child;
        _data[last_child].right_neighbor=b;
        _data[b].left_neighbor=last_child;
      }

      ++_data[a].degree;

      _data[b].marked=false;
    }

    /*
     *It is invoked only if a has siblings.
     */
    void unlace(int a) {
      int leftn=_data[a].left_neighbor;
      int rightn=_data[a].right_neighbor;
      _data[leftn].right_neighbor=rightn;
      _data[rightn].left_neighbor=leftn;
    }


    class Store {
      friend class FibHeap;

      Item name;
      int parent;
      int left_neighbor;
      int right_neighbor;
      int child;
      int degree;
      bool marked;
      bool in;
      Prio prio;

      Store() : parent(-1), child(-1), degree(), marked(false), in(true) {}
    };
  };

} //namespace lemon

#endif //LEMON_FIB_HEAP_H

