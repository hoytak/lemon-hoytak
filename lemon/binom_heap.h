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

#ifndef LEMON_BINOM_HEAP_H
#define LEMON_BINOM_HEAP_H

///\file
///\ingroup heaps
///\brief Binomial Heap implementation.

#include <vector>
#include <utility>
#include <functional>
#include <lemon/math.h>
#include <lemon/counter.h>

namespace lemon {

  /// \ingroup heaps
  ///
  ///\brief Binomial heap data structure.
  ///
  /// This class implements the \e binomial \e heap data structure.
  /// It fully conforms to the \ref concepts::Heap "heap concept".
  ///
  /// The methods \ref increase() and \ref erase() are not efficient
  /// in a binomial heap. In case of many calls of these operations,
  /// it is better to use other heap structure, e.g. \ref BinHeap
  /// "binary heap".
  ///
  /// \tparam PR Type of the priorities of the items.
  /// \tparam IM A read-writable item map with \c int values, used
  /// internally to handle the cross references.
  /// \tparam CMP A functor class for comparing the priorities.
  /// The default is \c std::less<PR>.
#ifdef DOXYGEN
  template <typename PR, typename IM, typename CMP>
#else
  template <typename PR, typename IM, typename CMP = std::less<PR> >
#endif
  class BinomHeap {
  public:
    /// Type of the item-int map.
    typedef IM ItemIntMap;
    /// Type of the priorities.
    typedef PR Prio;
    /// Type of the items stored in the heap.
    typedef typename ItemIntMap::Key Item;
    /// Functor type for comparing the priorities.
    typedef CMP Compare;

    /// \brief Type to represent the states of the items.
    ///
    /// Each item has a state associated to it. It can be "in heap",
    /// "pre-heap" or "post-heap". The latter two are indifferent from the
    /// heap's point of view, but may be useful to the user.
    ///
    /// The item-int map must be initialized in such way that it assigns
    /// \c PRE_HEAP (<tt>-1</tt>) to any element to be put in the heap.
    enum State {
      IN_HEAP = 0,    ///< = 0.
      PRE_HEAP = -1,  ///< = -1.
      POST_HEAP = -2  ///< = -2.
    };

  private:
    class store;

    std::vector<store> _data;
    int _min, _head;
    ItemIntMap &_iim;
    Compare _comp;
    int _num_items;

  public:
    /// \brief Constructor.
    ///
    /// Constructor.
    /// \param map A map that assigns \c int values to the items.
    /// It is used internally to handle the cross references.
    /// The assigned value must be \c PRE_HEAP (<tt>-1</tt>) for each item.
    explicit BinomHeap(ItemIntMap &map)
      : _min(0), _head(-1), _iim(map), _num_items(0) {}

    /// \brief Constructor.
    ///
    /// Constructor.
    /// \param map A map that assigns \c int values to the items.
    /// It is used internally to handle the cross references.
    /// The assigned value must be \c PRE_HEAP (<tt>-1</tt>) for each item.
    /// \param comp The function object used for comparing the priorities.
    BinomHeap(ItemIntMap &map, const Compare &comp)
      : _min(0), _head(-1), _iim(map), _comp(comp), _num_items(0) {}

    /// \brief The number of items stored in the heap.
    ///
    /// This function returns the number of items stored in the heap.
    int size() const { return _num_items; }

    /// \brief Check if the heap is empty.
    ///
    /// This function returns \c true if the heap is empty.
    bool empty() const { return _num_items==0; }

    /// \brief Make the heap empty.
    ///
    /// This functon makes the heap empty.
    /// It does not change the cross reference map. If you want to reuse
    /// a heap that is not surely empty, you should first clear it and
    /// then you should set the cross reference map to \c PRE_HEAP
    /// for each item.
    void clear() {
      _data.clear(); _min=0; _num_items=0; _head=-1;
    }

    /// \brief Set the priority of an item or insert it, if it is
    /// not stored in the heap.
    ///
    /// This method sets the priority of the given item if it is
    /// already stored in the heap. Otherwise it inserts the given
    /// item into the heap with the given priority.
    /// \param item The item.
    /// \param value The priority.
    void set (const Item& item, const Prio& value) {
      int i=_iim[item];
      if ( i >= 0 && _data[i].in ) {
        if ( _comp(value, _data[i].prio) ) decrease(item, value);
        if ( _comp(_data[i].prio, value) ) increase(item, value);
      } else push(item, value);
    }

    /// \brief Insert an item into the heap with the given priority.
    ///
    /// This function inserts the given item into the heap with the
    /// given priority.
    /// \param item The item to insert.
    /// \param value The priority of the item.
    /// \pre \e item must not be stored in the heap.
    void push (const Item& item, const Prio& value) {
      int i=_iim[item];
      if ( i<0 ) {
        int s=_data.size();
        _iim.set( item,s );
        store st;
        st.name=item;
        _data.push_back(st);
        i=s;
      }
      else {
        _data[i].parent=_data[i].right_neighbor=_data[i].child=-1;
        _data[i].degree=0;
        _data[i].in=true;
      }
      _data[i].prio=value;

      if( 0==_num_items ) { _head=i; _min=i; }
      else { merge(i); }

      _min = findMin();

      ++_num_items;
    }

    /// \brief Return the item having minimum priority.
    ///
    /// This function returns the item having minimum priority.
    /// \pre The heap must be non-empty.
    Item top() const { return _data[_min].name; }

    /// \brief The minimum priority.
    ///
    /// This function returns the minimum priority.
    /// \pre The heap must be non-empty.
    Prio prio() const { return _data[_min].prio; }

    /// \brief The priority of the given item.
    ///
    /// This function returns the priority of the given item.
    /// \param item The item.
    /// \pre \e item must be in the heap.
    const Prio& operator[](const Item& item) const {
      return _data[_iim[item]].prio;
    }

    /// \brief Remove the item having minimum priority.
    ///
    /// This function removes the item having minimum priority.
    /// \pre The heap must be non-empty.
    void pop() {
      _data[_min].in=false;

      int head_child=-1;
      if ( _data[_min].child!=-1 ) {
        int child=_data[_min].child;
        int neighb;
        int prev=-1;
        while( child!=-1 ) {
          neighb=_data[child].right_neighbor;
          _data[child].parent=-1;
          _data[child].right_neighbor=prev;
          head_child=child;
          prev=child;
          child=neighb;
        }
      }

      // The first case is that there are only one root.
      if ( -1==_data[_head].right_neighbor ) {
        _head=head_child;
      }
      // The case where there are more roots.
      else {
        if( _head!=_min )  { unlace(_min); }
        else { _head=_data[_head].right_neighbor; }

        merge(head_child);
      }
      _min=findMin();
      --_num_items;
    }

    /// \brief Remove the given item from the heap.
    ///
    /// This function removes the given item from the heap if it is
    /// already stored.
    /// \param item The item to delete.
    /// \pre \e item must be in the heap.
    void erase (const Item& item) {
      int i=_iim[item];
      if ( i >= 0 && _data[i].in ) {
        decrease( item, _data[_min].prio-1 );
        pop();
      }
    }

    /// \brief Decrease the priority of an item to the given value.
    ///
    /// This function decreases the priority of an item to the given value.
    /// \param item The item.
    /// \param value The priority.
    /// \pre \e item must be stored in the heap with priority at least \e value.
    void decrease (Item item, const Prio& value) {
      int i=_iim[item];

      if( _comp( value,_data[i].prio ) ) {
        _data[i].prio=value;

        int p_loc=_data[i].parent, loc=i;
        int parent, child, neighb;

        while( -1!=p_loc && _comp(_data[loc].prio,_data[p_loc].prio) ) {

          // parent set for other loc_child
          child=_data[loc].child;
          while( -1!=child ) {
            _data[child].parent=p_loc;
            child=_data[child].right_neighbor;
          }

          // parent set for other p_loc_child
          child=_data[p_loc].child;
          while( -1!=child ) {
            _data[child].parent=loc;
            child=_data[child].right_neighbor;
          }

          child=_data[p_loc].child;
          _data[p_loc].child=_data[loc].child;
          if( child==loc )
            child=p_loc;
          _data[loc].child=child;

          // left_neighb set for p_loc
          if( _data[loc].child!=p_loc ) {
            while( _data[child].right_neighbor!=loc )
              child=_data[child].right_neighbor;
            _data[child].right_neighbor=p_loc;
          }

          // left_neighb set for loc
          parent=_data[p_loc].parent;
          if( -1!=parent ) child=_data[parent].child;
          else child=_head;

          if( child!=p_loc ) {
            while( _data[child].right_neighbor!=p_loc )
              child=_data[child].right_neighbor;
            _data[child].right_neighbor=loc;
          }

          neighb=_data[p_loc].right_neighbor;
          _data[p_loc].right_neighbor=_data[loc].right_neighbor;
          _data[loc].right_neighbor=neighb;

          _data[p_loc].parent=loc;
          _data[loc].parent=parent;

          if( -1!=parent && _data[parent].child==p_loc )
            _data[parent].child=loc;

          /*if new parent will be the first root*/
          if( _head==p_loc )
            _head=loc;

          p_loc=_data[loc].parent;
        }
      }
      if( _comp(value,_data[_min].prio) ) {
        _min=i;
      }
    }

    /// \brief Increase the priority of an item to the given value.
    ///
    /// This function increases the priority of an item to the given value.
    /// \param item The item.
    /// \param value The priority.
    /// \pre \e item must be stored in the heap with priority at most \e value.
    void increase (Item item, const Prio& value) {
      erase(item);
      push(item, value);
    }

    /// \brief Return the state of an item.
    ///
    /// This method returns \c PRE_HEAP if the given item has never
    /// been in the heap, \c IN_HEAP if it is in the heap at the moment,
    /// and \c POST_HEAP otherwise.
    /// In the latter case it is possible that the item will get back
    /// to the heap again.
    /// \param item The item.
    State state(const Item &item) const {
      int i=_iim[item];
      if( i>=0 ) {
        if ( _data[i].in ) i=0;
        else i=-2;
      }
      return State(i);
    }

    /// \brief Set the state of an item in the heap.
    ///
    /// This function sets the state of the given item in the heap.
    /// It can be used to manually clear the heap when it is important
    /// to achive better time complexity.
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
    int findMin() {
      int min_loc=-1, min_val;
      int x=_head;
      if( x!=-1 ) {
        min_val=_data[x].prio;
        min_loc=x;
        x=_data[x].right_neighbor;

        while( x!=-1 ) {
          if( _comp( _data[x].prio,min_val ) ) {
            min_val=_data[x].prio;
            min_loc=x;
          }
          x=_data[x].right_neighbor;
        }
      }
      return min_loc;
    }

    void merge(int a) {
      interleave(a);

      int x=_head;
      if( -1!=x ) {
        int x_prev=-1, x_next=_data[x].right_neighbor;
        while( -1!=x_next ) {
          if( _data[x].degree!=_data[x_next].degree || ( -1!=_data[x_next].right_neighbor && _data[_data[x_next].right_neighbor].degree==_data[x].degree ) ) {
            x_prev=x;
            x=x_next;
          }
          else {
            if( _comp(_data[x].prio,_data[x_next].prio) ) {
              _data[x].right_neighbor=_data[x_next].right_neighbor;
              fuse(x_next,x);
            }
            else {
              if( -1==x_prev ) { _head=x_next; }
              else {
                _data[x_prev].right_neighbor=x_next;
              }
              fuse(x,x_next);
              x=x_next;
            }
          }
          x_next=_data[x].right_neighbor;
        }
      }
    }

    void interleave(int a) {
      int other=-1, head_other=-1;

      while( -1!=a || -1!=_head ) {
        if( -1==a ) {
          if( -1==head_other ) {
            head_other=_head;
          }
          else {
            _data[other].right_neighbor=_head;
          }
          _head=-1;
        }
        else if( -1==_head ) {
          if( -1==head_other ) {
            head_other=a;
          }
          else {
            _data[other].right_neighbor=a;
          }
          a=-1;
        }
        else {
          if( _data[a].degree<_data[_head].degree ) {
            if( -1==head_other ) {
              head_other=a;
            }
            else {
              _data[other].right_neighbor=a;
            }
            other=a;
            a=_data[a].right_neighbor;
          }
          else {
            if( -1==head_other ) {
              head_other=_head;
            }
            else {
              _data[other].right_neighbor=_head;
            }
            other=_head;
            _head=_data[_head].right_neighbor;
          }
        }
      }
      _head=head_other;
    }

    // Lacing a under b
    void fuse(int a, int b) {
      _data[a].parent=b;
      _data[a].right_neighbor=_data[b].child;
      _data[b].child=a;

      ++_data[b].degree;
    }

    // It is invoked only if a has siblings.
    void unlace(int a) {
      int neighb=_data[a].right_neighbor;
      int other=_head;

      while( _data[other].right_neighbor!=a )
        other=_data[other].right_neighbor;
      _data[other].right_neighbor=neighb;
    }

  private:

    class store {
      friend class BinomHeap;

      Item name;
      int parent;
      int right_neighbor;
      int child;
      int degree;
      bool in;
      Prio prio;

      store() : parent(-1), right_neighbor(-1), child(-1), degree(0), in(true) {}
    };
  };

} //namespace lemon

#endif //LEMON_BINOM_HEAP_H

