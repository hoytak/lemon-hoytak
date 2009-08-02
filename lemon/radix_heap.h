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

#ifndef LEMON_RADIX_HEAP_H
#define LEMON_RADIX_HEAP_H

///\ingroup auxdat
///\file
///\brief Radix Heap implementation.

#include <vector>
#include <lemon/error.h>

namespace lemon {


  /// \ingroup auxdata
  ///
  /// \brief A Radix Heap implementation.
  ///
  /// This class implements the \e radix \e heap data structure. A \e heap
  /// is a data structure for storing items with specified values called \e
  /// priorities in such a way that finding the item with minimum priority is
  /// efficient. This heap type can store only items with \e int priority.
  /// In a heap one can change the priority of an item, add or erase an
  /// item, but the priority cannot be decreased under the last removed
  /// item's priority.
  ///
  /// \param IM A read and writable Item int map, used internally
  /// to handle the cross references.
  ///
  /// \see BinHeap
  /// \see Dijkstra
  template <typename IM>
  class RadixHeap {

  public:
    typedef typename IM::Key Item;
    typedef int Prio;
    typedef IM ItemIntMap;

    /// \brief Exception thrown by RadixHeap.
    ///
    /// This Exception is thrown when a smaller priority
    /// is inserted into the \e RadixHeap then the last time erased.
    /// \see RadixHeap

    class UnderFlowPriorityError : public Exception {
    public:
      virtual const char* what() const throw() {
        return "lemon::RadixHeap::UnderFlowPriorityError";
      }
    };

    /// \brief Type to represent the items states.
    ///
    /// Each Item element have a state associated to it. It may be "in heap",
    /// "pre heap" or "post heap". The latter two are indifferent from the
    /// heap's point of view, but may be useful to the user.
    ///
    /// The ItemIntMap \e should be initialized in such way that it maps
    /// PRE_HEAP (-1) to any element to be put in the heap...
    enum State {
      IN_HEAP = 0,
      PRE_HEAP = -1,
      POST_HEAP = -2
    };

  private:

    struct RadixItem {
      int prev, next, box;
      Item item;
      int prio;
      RadixItem(Item _item, int _prio) : item(_item), prio(_prio) {}
    };

    struct RadixBox {
      int first;
      int min, size;
      RadixBox(int _min, int _size) : first(-1), min(_min), size(_size) {}
    };

    std::vector<RadixItem> data;
    std::vector<RadixBox> boxes;

    ItemIntMap &_iim;


  public:
    /// \brief The constructor.
    ///
    /// The constructor.
    ///
    /// \param map It should be given to the constructor, since it is used
    /// internally to handle the cross references. The value of the map
    /// should be PRE_HEAP (-1) for each element.
    ///
    /// \param minimal The initial minimal value of the heap.
    /// \param capacity It determines the initial capacity of the heap.
    RadixHeap(ItemIntMap &map, int minimal = 0, int capacity = 0)
      : _iim(map) {
      boxes.push_back(RadixBox(minimal, 1));
      boxes.push_back(RadixBox(minimal + 1, 1));
      while (lower(boxes.size() - 1, capacity + minimal - 1)) {
        extend();
      }
    }

    /// The number of items stored in the heap.
    ///
    /// \brief Returns the number of items stored in the heap.
    int size() const { return data.size(); }
    /// \brief Checks if the heap stores no items.
    ///
    /// Returns \c true if and only if the heap stores no items.
    bool empty() const { return data.empty(); }

    /// \brief Make empty this heap.
    ///
    /// Make empty this heap. It does not change the cross reference
    /// map.  If you want to reuse a heap what is not surely empty you
    /// should first clear the heap and after that you should set the
    /// cross reference map for each item to \c PRE_HEAP.
    void clear(int minimal = 0, int capacity = 0) {
      data.clear(); boxes.clear();
      boxes.push_back(RadixBox(minimal, 1));
      boxes.push_back(RadixBox(minimal + 1, 1));
      while (lower(boxes.size() - 1, capacity + minimal - 1)) {
        extend();
      }
    }

  private:

    bool upper(int box, Prio pr) {
      return pr < boxes[box].min;
    }

    bool lower(int box, Prio pr) {
      return pr >= boxes[box].min + boxes[box].size;
    }

    /// \brief Remove item from the box list.
    void remove(int index) {
      if (data[index].prev >= 0) {
        data[data[index].prev].next = data[index].next;
      } else {
        boxes[data[index].box].first = data[index].next;
      }
      if (data[index].next >= 0) {
        data[data[index].next].prev = data[index].prev;
      }
    }

    /// \brief Insert item into the box list.
    void insert(int box, int index) {
      if (boxes[box].first == -1) {
        boxes[box].first = index;
        data[index].next = data[index].prev = -1;
      } else {
        data[index].next = boxes[box].first;
        data[boxes[box].first].prev = index;
        data[index].prev = -1;
        boxes[box].first = index;
      }
      data[index].box = box;
    }

    /// \brief Add a new box to the box list.
    void extend() {
      int min = boxes.back().min + boxes.back().size;
      int bs = 2 * boxes.back().size;
      boxes.push_back(RadixBox(min, bs));
    }

    /// \brief Move an item up into the proper box.
    void bubble_up(int index) {
      if (!lower(data[index].box, data[index].prio)) return;
      remove(index);
      int box = findUp(data[index].box, data[index].prio);
      insert(box, index);
    }

    /// \brief Find up the proper box for the item with the given prio.
    int findUp(int start, int pr) {
      while (lower(start, pr)) {
        if (++start == int(boxes.size())) {
          extend();
        }
      }
      return start;
    }

    /// \brief Move an item down into the proper box.
    void bubble_down(int index) {
      if (!upper(data[index].box, data[index].prio)) return;
      remove(index);
      int box = findDown(data[index].box, data[index].prio);
      insert(box, index);
    }

    /// \brief Find up the proper box for the item with the given prio.
    int findDown(int start, int pr) {
      while (upper(start, pr)) {
        if (--start < 0) throw UnderFlowPriorityError();
      }
      return start;
    }

    /// \brief Find the first not empty box.
    int findFirst() {
      int first = 0;
      while (boxes[first].first == -1) ++first;
      return first;
    }

    /// \brief Gives back the minimal prio of the box.
    int minValue(int box) {
      int min = data[boxes[box].first].prio;
      for (int k = boxes[box].first; k != -1; k = data[k].next) {
        if (data[k].prio < min) min = data[k].prio;
      }
      return min;
    }

    /// \brief Rearrange the items of the heap and makes the
    /// first box not empty.
    void moveDown() {
      int box = findFirst();
      if (box == 0) return;
      int min = minValue(box);
      for (int i = 0; i <= box; ++i) {
        boxes[i].min = min;
        min += boxes[i].size;
      }
      int curr = boxes[box].first, next;
      while (curr != -1) {
        next = data[curr].next;
        bubble_down(curr);
        curr = next;
      }
    }

    void relocate_last(int index) {
      if (index != int(data.size()) - 1) {
        data[index] = data.back();
        if (data[index].prev != -1) {
          data[data[index].prev].next = index;
        } else {
          boxes[data[index].box].first = index;
        }
        if (data[index].next != -1) {
          data[data[index].next].prev = index;
        }
        _iim[data[index].item] = index;
      }
      data.pop_back();
    }

  public:

    /// \brief Insert an item into the heap with the given priority.
    ///
    /// Adds \c i to the heap with priority \c p.
    /// \param i The item to insert.
    /// \param p The priority of the item.
    void push(const Item &i, const Prio &p) {
      int n = data.size();
      _iim.set(i, n);
      data.push_back(RadixItem(i, p));
      while (lower(boxes.size() - 1, p)) {
        extend();
      }
      int box = findDown(boxes.size() - 1, p);
      insert(box, n);
    }

    /// \brief Returns the item with minimum priority.
    ///
    /// This method returns the item with minimum priority.
    /// \pre The heap must be nonempty.
    Item top() const {
      const_cast<RadixHeap<ItemIntMap>&>(*this).moveDown();
      return data[boxes[0].first].item;
    }

    /// \brief Returns the minimum priority.
    ///
    /// It returns the minimum priority.
    /// \pre The heap must be nonempty.
    Prio prio() const {
      const_cast<RadixHeap<ItemIntMap>&>(*this).moveDown();
      return data[boxes[0].first].prio;
     }

    /// \brief Deletes the item with minimum priority.
    ///
    /// This method deletes the item with minimum priority.
    /// \pre The heap must be non-empty.
    void pop() {
      moveDown();
      int index = boxes[0].first;
      _iim[data[index].item] = POST_HEAP;
      remove(index);
      relocate_last(index);
    }

    /// \brief Deletes \c i from the heap.
    ///
    /// This method deletes item \c i from the heap, if \c i was
    /// already stored in the heap.
    /// \param i The item to erase.
    void erase(const Item &i) {
      int index = _iim[i];
      _iim[i] = POST_HEAP;
      remove(index);
      relocate_last(index);
   }

    /// \brief Returns the priority of \c i.
    ///
    /// This function returns the priority of item \c i.
    /// \pre \c i must be in the heap.
    /// \param i The item.
    Prio operator[](const Item &i) const {
      int idx = _iim[i];
      return data[idx].prio;
    }

    /// \brief \c i gets to the heap with priority \c p independently
    /// if \c i was already there.
    ///
    /// This method calls \ref push(\c i, \c p) if \c i is not stored
    /// in the heap and sets the priority of \c i to \c p otherwise.
    /// It may throw an \e UnderFlowPriorityException.
    /// \param i The item.
    /// \param p The priority.
    void set(const Item &i, const Prio &p) {
      int idx = _iim[i];
      if( idx < 0 ) {
        push(i, p);
      }
      else if( p >= data[idx].prio ) {
        data[idx].prio = p;
        bubble_up(idx);
      } else {
        data[idx].prio = p;
        bubble_down(idx);
      }
    }


    /// \brief Decreases the priority of \c i to \c p.
    ///
    /// This method decreases the priority of item \c i to \c p.
    /// \pre \c i must be stored in the heap with priority at least \c p, and
    /// \c should be greater or equal to the last removed item's priority.
    /// \param i The item.
    /// \param p The priority.
    void decrease(const Item &i, const Prio &p) {
      int idx = _iim[i];
      data[idx].prio = p;
      bubble_down(idx);
    }

    /// \brief Increases the priority of \c i to \c p.
    ///
    /// This method sets the priority of item \c i to \c p.
    /// \pre \c i must be stored in the heap with priority at most \c p
    /// \param i The item.
    /// \param p The priority.
    void increase(const Item &i, const Prio &p) {
      int idx = _iim[i];
      data[idx].prio = p;
      bubble_up(idx);
    }

    /// \brief Returns if \c item is in, has already been in, or has
    /// never been in the heap.
    ///
    /// This method returns PRE_HEAP if \c item has never been in the
    /// heap, IN_HEAP if it is in the heap at the moment, and POST_HEAP
    /// otherwise. In the latter case it is possible that \c item will
    /// get back to the heap again.
    /// \param i The item.
    State state(const Item &i) const {
      int s = _iim[i];
      if( s >= 0 ) s = 0;
      return State(s);
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
        _iim[i] = st;
        break;
      case IN_HEAP:
        break;
      }
    }

  }; // class RadixHeap

} // namespace lemon

#endif // LEMON_RADIX_HEAP_H
