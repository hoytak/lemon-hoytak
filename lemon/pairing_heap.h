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

#ifndef LEMON_PAIRING_HEAP_H
#define LEMON_PAIRING_HEAP_H

///\file
///\ingroup auxdat
///\brief Pairing Heap implementation.

#include <vector>
#include <functional>
#include <lemon/math.h>

namespace lemon {

  /// \ingroup auxdat
  ///
  ///\brief Pairing Heap.
  ///
  ///This class implements the \e Pairing \e heap data structure. A \e heap
  ///is a data structure for storing items with specified values called \e
  ///priorities in such a way that finding the item with minimum priority is
  ///efficient. \c Compare specifies the ordering of the priorities. In a heap
  ///one can change the priority of an item, add or erase an item, etc.
  ///
  ///The methods \ref increase and \ref erase are not efficient in a Pairing
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
  class PairingHeap {
  public:
    typedef _ItemIntMap ItemIntMap;
    typedef _Prio Prio;
    typedef typename ItemIntMap::Key Item;
    typedef std::pair<Item,Prio> Pair;
    typedef _Compare Compare;

  private:
    class store;

    std::vector<store> container;
    int minimum;
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
    explicit PairingHeap(ItemIntMap &_iimap)
      : minimum(0), iimap(_iimap), num_items(0) {}

    /// \brief The constructor
    ///
    /// \c _iimap should be given to the constructor, since it is used
    /// internally to handle the cross references. \c _comp is an
    /// object for ordering of the priorities.
    PairingHeap(ItemIntMap &_iimap, const Compare &_comp)
      : minimum(0), iimap(_iimap), comp(_comp), num_items(0) {}

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
      container.clear();
      minimum = 0;
      num_items = 0;
    }

    /// \brief \c item gets to the heap with priority \c value independently
    /// if \c item was already there.
    ///
    /// This method calls \ref push(\c item, \c value) if \c item is not
    /// stored in the heap and it calls \ref decrease(\c item, \c value) or
    /// \ref increase(\c item, \c value) otherwise.
    void set (const Item& item, const Prio& value) {
      int i=iimap[item];
      if ( i>=0 && container[i].in ) {
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
      if( i<0 ) {
        int s=container.size();
        iimap.set(item, s);
        store st;
        st.name=item;
        container.push_back(st);
        i=s;
      } else {
        container[i].parent=container[i].child=-1;
        container[i].left_child=false;
        container[i].degree=0;
        container[i].in=true;
      }

      container[i].prio=value;

      if ( num_items!=0 ) {
        if ( comp( value, container[minimum].prio) ) {
          fuse(i,minimum);
          minimum=i;
        }
        else fuse(minimum,i);
      }
      else minimum=i;

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
      int TreeArray[num_items];
      int i=0, num_child=0, child_right = 0;
      container[minimum].in=false;

      if( -1!=container[minimum].child ) {
        i=container[minimum].child;
        TreeArray[num_child] = i;
        container[i].parent = -1;
        container[minimum].child = -1;

        ++num_child;
        int ch=-1;
        while( container[i].child!=-1 ) {
          ch=container[i].child;
          if( container[ch].left_child && i==container[ch].parent ) {
            i=ch;
            //break;
          } else {
            if( container[ch].left_child ) {
              child_right=container[ch].parent;
              container[ch].parent = i;
              --container[i].degree;
            }
            else {
              child_right=ch;
              container[i].child=-1;
              container[i].degree=0;
            }
            container[child_right].parent = -1;
            TreeArray[num_child] = child_right;
            i = child_right;
            ++num_child;
          }
        }

        int other;
        for( i=0; i<num_child-1; i+=2 ) {
          if ( !comp(container[TreeArray[i]].prio,
                     container[TreeArray[i+1]].prio) ) {
            other=TreeArray[i];
            TreeArray[i]=TreeArray[i+1];
            TreeArray[i+1]=other;
          }
          fuse( TreeArray[i], TreeArray[i+1] );
        }

        i = (0==(num_child % 2)) ? num_child-2 : num_child-1;
        while(i>=2) {
          if ( comp(container[TreeArray[i]].prio,
                    container[TreeArray[i-2]].prio) ) {
            other=TreeArray[i];
            TreeArray[i]=TreeArray[i-2];
            TreeArray[i-2]=other;
          }
          fuse( TreeArray[i-2], TreeArray[i] );
          i-=2;
        }
        minimum = TreeArray[0];
      }

      if ( 0==num_child ) {
        minimum = container[minimum].child;
      }

      if (minimum >= 0) container[minimum].left_child = false;

      --num_items;
    }

    /// \brief Deletes \c item from the heap.
    ///
    /// This method deletes \c item from the heap, if \c item was already
    /// stored in the heap. It is quite inefficient in Pairing heaps.
    void erase (const Item& item) {
      int i=iimap[item];
      if ( i>=0 && container[i].in ) {
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
      container[i].prio=value;
      int p=container[i].parent;

      if( container[i].left_child && i!=container[p].child ) {
        p=container[p].parent;
      }

      if ( p!=-1 && comp(value,container[p].prio) ) {
        cut(i,p);
        if ( comp(container[minimum].prio,value) ) {
          fuse(minimum,i);
        } else {
          fuse(i,minimum);
          minimum=i;
        }
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
      push(item,value);
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
        if( container[i].in ) i=0;
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
        if (state(i) == IN_HEAP) erase(i);
        iimap[i]=st;
        break;
      case IN_HEAP:
        break;
      }
    }

  private:

    void cut(int a, int b) {
      int child_a;
      switch (container[a].degree) {
        case 2:
          child_a = container[container[a].child].parent;
          if( container[a].left_child ) {
            container[child_a].left_child=true;
            container[b].child=child_a;
            container[child_a].parent=container[a].parent;
          }
          else {
            container[child_a].left_child=false;
            container[child_a].parent=b;
            if( a!=container[b].child )
              container[container[b].child].parent=child_a;
            else
              container[b].child=child_a;
          }
          --container[a].degree;
          container[container[a].child].parent=a;
          break;

        case 1:
          child_a = container[a].child;
          if( !container[child_a].left_child ) {
            --container[a].degree;
            if( container[a].left_child ) {
              container[child_a].left_child=true;
              container[child_a].parent=container[a].parent;
              container[b].child=child_a;
            }
            else {
              container[child_a].left_child=false;
              container[child_a].parent=b;
              if( a!=container[b].child )
                container[container[b].child].parent=child_a;
              else
                container[b].child=child_a;
            }
            container[a].child=-1;
          }
          else {
            --container[b].degree;
            if( container[a].left_child ) {
              container[b].child =
                (1==container[b].degree) ? container[a].parent : -1;
            } else {
              if (1==container[b].degree)
                container[container[b].child].parent=b;
              else
                container[b].child=-1;
            }
          }
          break;

        case 0:
          --container[b].degree;
          if( container[a].left_child ) {
            container[b].child =
              (0!=container[b].degree) ? container[a].parent : -1;
          } else {
            if( 0!=container[b].degree )
              container[container[b].child].parent=b;
            else
              container[b].child=-1;
          }
          break;
      }
      container[a].parent=-1;
      container[a].left_child=false;
    }

    void fuse(int a, int b) {
      int child_a = container[a].child;
      int child_b = container[b].child;
      container[a].child=b;
      container[b].parent=a;
      container[b].left_child=true;

      if( -1!=child_a ) {
        container[b].child=child_a;
        container[child_a].parent=b;
        container[child_a].left_child=false;
        ++container[b].degree;

        if( -1!=child_b ) {
           container[b].child=child_b;
           container[child_b].parent=child_a;
        }
      }
      else { ++container[a].degree; }
    }

    class store {
      friend class PairingHeap;

      Item name;
      int parent;
      int child;
      bool left_child;
      int degree;
      bool in;
      Prio prio;

      store() : parent(-1), child(-1), left_child(false), degree(0), in(true) {}
    };
  };

} //namespace lemon

#endif //LEMON_PAIRING_HEAP_H

