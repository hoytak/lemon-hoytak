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

#ifndef LEMON_BINOM_HEAP_H
#define LEMON_BINOM_HEAP_H

///\file
///\ingroup auxdat
///\brief Binomial Heap implementation.

#include <vector>
#include <functional>
#include <lemon/math.h>
#include <lemon/counter.h>

namespace lemon {

  /// \ingroup auxdat
  ///
  ///\brief Binomial Heap.
  ///
  ///This class implements the \e Binomial \e heap data structure. A \e heap
  ///is a data structure for storing items with specified values called \e
  ///priorities in such a way that finding the item with minimum priority is
  ///efficient. \c Compare specifies the ordering of the priorities. In a heap
  ///one can change the priority of an item, add or erase an item, etc.
  ///
  ///The methods \ref increase and \ref erase are not efficient in a Binomial
  ///heap. In case of many calls to these operations, it is better to use a
  ///\ref BinHeap "binary heap".
  ///
  ///\param _Prio Type of the priority of the items.
  ///\param _ItemIntMap A read and writable Item int map, used internally
  ///to handle the cross references.
  ///\param _Compare A class for the ordering of the priorities. The
  ///default is \c std::less<_Prio>.
  ///
  ///\sa BinHeap
  ///\sa Dijkstra
  ///\author Dorian Batha

#ifdef DOXYGEN
  template <typename _Prio,
            typename _ItemIntMap,
            typename _Compare>
#else
  template <typename _Prio,
            typename _ItemIntMap,
            typename _Compare = std::less<_Prio> >
#endif
  class BinomHeap {
  public:
    typedef _ItemIntMap ItemIntMap;
    typedef _Prio Prio;
    typedef typename ItemIntMap::Key Item;
    typedef std::pair<Item,Prio> Pair;
    typedef _Compare Compare;

  private:
    class store;

    std::vector<store> container;
    int minimum, head;
    ItemIntMap &iimap;
    Compare comp;
    int num_items;

  public:
    ///Status of the nodes
    enum State {
      ///The node is in the heap
      IN_HEAP = 0,
      ///The node has never been in the heap
      PRE_HEAP = -1,
      ///The node was in the heap but it got out of it
      POST_HEAP = -2
    };

    /// \brief The constructor
    ///
    /// \c _iimap should be given to the constructor, since it is
    ///   used internally to handle the cross references.
    explicit BinomHeap(ItemIntMap &_iimap)
      : minimum(0), head(-1), iimap(_iimap), num_items() {}

    /// \brief The constructor
    ///
    /// \c _iimap should be given to the constructor, since it is used
    /// internally to handle the cross references. \c _comp is an
    /// object for ordering of the priorities.
    BinomHeap(ItemIntMap &_iimap, const Compare &_comp)
      : minimum(0), head(-1), iimap(_iimap), comp(_comp), num_items() {}

    /// \brief The number of items stored in the heap.
    ///
    /// Returns the number of items stored in the heap.
    int size() const { return num_items; }

    /// \brief Checks if the heap stores no items.
    ///
    ///   Returns \c true if and only if the heap stores no items.
    bool empty() const { return num_items==0; }

    /// \brief Make empty this heap.
    ///
    /// Make empty this heap. It does not change the cross reference
    /// map.  If you want to reuse a heap what is not surely empty you
    /// should first clear the heap and after that you should set the
    /// cross reference map for each item to \c PRE_HEAP.
    void clear() {
      container.clear(); minimum=0; num_items=0; head=-1;
    }

    /// \brief \c item gets to the heap with priority \c value independently
    /// if \c item was already there.
    ///
    /// This method calls \ref push(\c item, \c value) if \c item is not
    /// stored in the heap and it calls \ref decrease(\c item, \c value) or
    /// \ref increase(\c item, \c value) otherwise.
    void set (const Item& item, const Prio& value) {
      int i=iimap[item];
      if ( i >= 0 && container[i].in ) {
        if ( comp(value, container[i].prio) ) decrease(item, value);
        if ( comp(container[i].prio, value) ) increase(item, value);
      } else push(item, value);
    }

    /// \brief Adds \c item to the heap with priority \c value.
    ///
    /// Adds \c item to the heap with priority \c value.
    /// \pre \c item must not be stored in the heap.
    void push (const Item& item, const Prio& value) {
      int i=iimap[item];
      if ( i<0 ) {
        int s=container.size();
        iimap.set( item,s );
        store st;
        st.name=item;
        container.push_back(st);
        i=s;
      }
      else {
        container[i].parent=container[i].right_neighbor=container[i].child=-1;
        container[i].degree=0;
        container[i].in=true;
      }
      container[i].prio=value;

      if( 0==num_items ) { head=i; minimum=i; }
      else { merge(i); }

      minimum = find_min();

      ++num_items;
    }

    /// \brief Returns the item with minimum priority relative to \c Compare.
    ///
    /// This method returns the item with minimum priority relative to \c
    /// Compare.
    /// \pre The heap must be nonempty.
    Item top() const { return container[minimum].name; }

    /// \brief Returns the minimum priority relative to \c Compare.
    ///
    /// It returns the minimum priority relative to \c Compare.
    /// \pre The heap must be nonempty.
    const Prio& prio() const { return container[minimum].prio; }

    /// \brief Returns the priority of \c item.
    ///
    /// It returns the priority of \c item.
    /// \pre \c item must be in the heap.
    const Prio& operator[](const Item& item) const {
      return container[iimap[item]].prio;
    }

    /// \brief Deletes the item with minimum priority relative to \c Compare.
    ///
    /// This method deletes the item with minimum priority relative to \c
    /// Compare from the heap.
    /// \pre The heap must be non-empty.
    void pop() {
      container[minimum].in=false;

      int head_child=-1;
      if ( container[minimum].child!=-1 ) {
        int child=container[minimum].child;
        int neighb;
        int prev=-1;
        while( child!=-1 ) {
          neighb=container[child].right_neighbor;
          container[child].parent=-1;
          container[child].right_neighbor=prev;
          head_child=child;
          prev=child;
          child=neighb;
        }
      }

      // The first case is that there are only one root.
      if ( -1==container[head].right_neighbor ) {
        head=head_child;
      }
      // The case where there are more roots.
      else {
        if( head!=minimum )  { unlace(minimum); }
        else { head=container[head].right_neighbor; }

        merge(head_child);
      }
      minimum=find_min();
      --num_items;
    }

    /// \brief Deletes \c item from the heap.
    ///
    /// This method deletes \c item from the heap, if \c item was already
    /// stored in the heap. It is quite inefficient in Binomial heaps.
    void erase (const Item& item) {
      int i=iimap[item];
      if ( i >= 0 && container[i].in ) {
        decrease( item, container[minimum].prio-1 );
        pop();
      }
    }

    /// \brief Decreases the priority of \c item to \c value.
    ///
    /// This method decreases the priority of \c item to \c value.
    /// \pre \c item must be stored in the heap with priority at least \c
    ///   value relative to \c Compare.
    void decrease (Item item, const Prio& value) {
      int i=iimap[item];

      if( comp( value,container[i].prio ) ) {
        container[i].prio=value;

        int p_loc=container[i].parent, loc=i;
        int parent, child, neighb;

        while( -1!=p_loc && comp(container[loc].prio,container[p_loc].prio) ) {

          // parent set for other loc_child
          child=container[loc].child;
          while( -1!=child ) {
            container[child].parent=p_loc;
            child=container[child].right_neighbor;
          }

          // parent set for other p_loc_child
          child=container[p_loc].child;
          while( -1!=child ) {
            container[child].parent=loc;
            child=container[child].right_neighbor;
          }

          child=container[p_loc].child;
          container[p_loc].child=container[loc].child;
          if( child==loc )
            child=p_loc;
          container[loc].child=child;

          // left_neighb set for p_loc
          if( container[loc].child!=p_loc ) {
            while( container[child].right_neighbor!=loc )
              child=container[child].right_neighbor;
            container[child].right_neighbor=p_loc;
          }

          // left_neighb set for loc
          parent=container[p_loc].parent;
          if( -1!=parent ) child=container[parent].child;
          else child=head;

          if( child!=p_loc ) {
            while( container[child].right_neighbor!=p_loc )
              child=container[child].right_neighbor;
            container[child].right_neighbor=loc;
          }

          neighb=container[p_loc].right_neighbor;
          container[p_loc].right_neighbor=container[loc].right_neighbor;
          container[loc].right_neighbor=neighb;

          container[p_loc].parent=loc;
          container[loc].parent=parent;

          if( -1!=parent && container[parent].child==p_loc )
            container[parent].child=loc;

          /*if new parent will be the first root*/
          if( head==p_loc )
            head=loc;

          p_loc=container[loc].parent;
        }
      }
      if( comp(value,container[minimum].prio) ) {
        minimum=i;
      }
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
      int i=iimap[item];
      if( i>=0 ) {
        if ( container[i].in ) i=0;
        else i=-2;
      }
      return State(i);
    }

    /// \brief Sets the state of the \c item in the heap.
    ///
    /// Sets the state of the \c item in the heap. It can be used to
    /// manually clear the heap when it is important to achive the
    /// better time complexity.
    /// \param i The item.
    /// \param st The state. It should not be \c IN_HEAP.
    void state(const Item& i, State st) {
      switch (st) {
      case POST_HEAP:
      case PRE_HEAP:
        if (state(i) == IN_HEAP) {
          erase(i);
        }
        iimap[i] = st;
        break;
      case IN_HEAP:
        break;
      }
    }

  private:
    int find_min() {
      int min_loc=-1, min_val;
      int x=head;
      if( x!=-1 ) {
        min_val=container[x].prio;
        min_loc=x;
        x=container[x].right_neighbor;

        while( x!=-1 ) {
          if( comp( container[x].prio,min_val ) ) {
            min_val=container[x].prio;
            min_loc=x;
          }
          x=container[x].right_neighbor;
        }
      }
      return min_loc;
    }

    void merge(int a) {
      interleave(a);

      int x=head;
      if( -1!=x ) {
        int x_prev=-1, x_next=container[x].right_neighbor;
        while( -1!=x_next ) {
          if( container[x].degree!=container[x_next].degree || ( -1!=container[x_next].right_neighbor && container[container[x_next].right_neighbor].degree==container[x].degree ) ) {
            x_prev=x;
            x=x_next;
          }
          else {
            if( comp(container[x].prio,container[x_next].prio) ) {
              container[x].right_neighbor=container[x_next].right_neighbor;
              fuse(x_next,x);
            }
            else {
              if( -1==x_prev ) { head=x_next; }
              else {
                container[x_prev].right_neighbor=x_next;
              }
              fuse(x,x_next);
              x=x_next;
            }
          }
          x_next=container[x].right_neighbor;
        }
      }
    }

    void interleave(int a) {
      int other=-1, head_other=-1;

      while( -1!=a || -1!=head ) {
        if( -1==a ) {
          if( -1==head_other ) {
            head_other=head;
          }
          else {
            container[other].right_neighbor=head;
          }
          head=-1;
        }
        else if( -1==head ) {
          if( -1==head_other ) {
            head_other=a;
          }
          else {
            container[other].right_neighbor=a;
          }
          a=-1;
        }
        else {
          if( container[a].degree<container[head].degree ) {
            if( -1==head_other ) {
              head_other=a;
            }
            else {
              container[other].right_neighbor=a;
            }
            other=a;
            a=container[a].right_neighbor;
          }
          else {
            if( -1==head_other ) {
              head_other=head;
            }
            else {
              container[other].right_neighbor=head;
            }
            other=head;
            head=container[head].right_neighbor;
          }
        }
      }
      head=head_other;
    }

    // Lacing a under b
    void fuse(int a, int b) {
      container[a].parent=b;
      container[a].right_neighbor=container[b].child;
      container[b].child=a;

      ++container[b].degree;
    }

    // It is invoked only if a has siblings.
    void unlace(int a) {
      int neighb=container[a].right_neighbor;
      int other=head;

      while( container[other].right_neighbor!=a )
        other=container[other].right_neighbor;
      container[other].right_neighbor=neighb;
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

