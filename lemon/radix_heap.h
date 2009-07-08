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
///\brief Radix heap implementation.

#include <vector>
#include <lemon/error.h>

namespace lemon {


  /// \ingroup auxdat
  ///
  /// \brief Radix heap data structure.
  ///
  /// This class implements the \e radix \e heap data structure.
  /// It practically conforms to the \ref concepts::Heap "heap concept",
  /// but it has some limitations due its special implementation.
  /// The type of the priorities must be \c int and the priority of an
  /// item cannot be decreased under the priority of the last removed item.
  ///
  /// \tparam IM A read-writable item map with \c int values, used
  /// internally to handle the cross references.
  template <typename IM>
  class RadixHeap {

  public:

    /// Type of the item-int map.
    typedef IM ItemIntMap;
    /// Type of the priorities.
    typedef int Prio;
    /// Type of the items stored in the heap.
    typedef typename ItemIntMap::Key Item;

    /// \brief Exception thrown by RadixHeap.
    ///
    /// This exception is thrown when an item is inserted into a
    /// RadixHeap with a priority smaller than the last erased one.
    /// \see RadixHeap
    class UnderFlowPriorityError : public Exception {
    public:
      virtual const char* what() const throw() {
        return "lemon::RadixHeap::UnderFlowPriorityError";
      }
    };

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

    /// \brief Constructor.
    ///
    /// Constructor.
    /// \param map A map that assigns \c int values to the items.
    /// It is used internally to handle the cross references.
    /// The assigned value must be \c PRE_HEAP (<tt>-1</tt>) for each item.
    /// \param minimum The initial minimum value of the heap.
    /// \param capacity The initial capacity of the heap.
    RadixHeap(ItemIntMap &map, int minimum = 0, int capacity = 0)
      : _iim(map)
    {
      boxes.push_back(RadixBox(minimum, 1));
      boxes.push_back(RadixBox(minimum + 1, 1));
      while (lower(boxes.size() - 1, capacity + minimum - 1)) {
        extend();
      }
    }

    /// \brief The number of items stored in the heap.
    ///
    /// This function returns the number of items stored in the heap.
    int size() const { return data.size(); }

    /// \brief Check if the heap is empty.
    ///
    /// This function returns \c true if the heap is empty.
    bool empty() const { return data.empty(); }

    /// \brief Make the heap empty.
    ///
    /// This functon makes the heap empty.
    /// It does not change the cross reference map. If you want to reuse
    /// a heap that is not surely empty, you should first clear it and
    /// then you should set the cross reference map to \c PRE_HEAP
    /// for each item.
    /// \param minimum The minimum value of the heap.
    /// \param capacity The capacity of the heap.
    void clear(int minimum = 0, int capacity = 0) {
      data.clear(); boxes.clear();
      boxes.push_back(RadixBox(minimum, 1));
      boxes.push_back(RadixBox(minimum + 1, 1));
      while (lower(boxes.size() - 1, capacity + minimum - 1)) {
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

    // Remove item from the box list
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

    // Insert item into the box list
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

    // Add a new box to the box list
    void extend() {
      int min = boxes.back().min + boxes.back().size;
      int bs = 2 * boxes.back().size;
      boxes.push_back(RadixBox(min, bs));
    }

    // Move an item up into the proper box.
    void bubble_up(int index) {
      if (!lower(data[index].box, data[index].prio)) return;
      remove(index);
      int box = findUp(data[index].box, data[index].prio);
      insert(box, index);
    }

    // Find up the proper box for the item with the given priority
    int findUp(int start, int pr) {
      while (lower(start, pr)) {
        if (++start == int(boxes.size())) {
          extend();
        }
      }
      return start;
    }

    // Move an item down into the proper box
    void bubble_down(int index) {
      if (!upper(data[index].box, data[index].prio)) return;
      remove(index);
      int box = findDown(data[index].box, data[index].prio);
      insert(box, index);
    }

    // Find down the proper box for the item with the given priority
    int findDown(int start, int pr) {
      while (upper(start, pr)) {
        if (--start < 0) throw UnderFlowPriorityError();
      }
      return start;
    }

    // Find the first non-empty box
    int findFirst() {
      int first = 0;
      while (boxes[first].first == -1) ++first;
      return first;
    }

    // Gives back the minimum priority of the given box
    int minValue(int box) {
      int min = data[boxes[box].first].prio;
      for (int k = boxes[box].first; k != -1; k = data[k].next) {
        if (data[k].prio < min) min = data[k].prio;
      }
      return min;
    }

    // Rearrange the items of the heap and make the first box non-empty
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
    /// This function inserts the given item into the heap with the
    /// given priority.
    /// \param i The item to insert.
    /// \param p The priority of the item.
    /// \pre \e i must not be stored in the heap.
    /// \warning This method may throw an \c UnderFlowPriorityException.
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

    /// \brief Return the item having minimum priority.
    ///
    /// This function returns the item having minimum priority.
    /// \pre The heap must be non-empty.
    Item top() const {
      const_cast<RadixHeap<ItemIntMap>&>(*this).moveDown();
      return data[boxes[0].first].item;
    }

    /// \brief The minimum priority.
    ///
    /// This function returns the minimum priority.
    /// \pre The heap must be non-empty.
    Prio prio() const {
      const_cast<RadixHeap<ItemIntMap>&>(*this).moveDown();
      return data[boxes[0].first].prio;
     }

    /// \brief Remove the item having minimum priority.
    ///
    /// This function removes the item having minimum priority.
    /// \pre The heap must be non-empty.
    void pop() {
      moveDown();
      int index = boxes[0].first;
      _iim[data[index].item] = POST_HEAP;
      remove(index);
      relocate_last(index);
    }

    /// \brief Remove the given item from the heap.
    ///
    /// This function removes the given item from the heap if it is
    /// already stored.
    /// \param i The item to delete.
    /// \pre \e i must be in the heap.
    void erase(const Item &i) {
      int index = _iim[i];
      _iim[i] = POST_HEAP;
      remove(index);
      relocate_last(index);
   }

    /// \brief The priority of the given item.
    ///
    /// This function returns the priority of the given item.
    /// \param i The item.
    /// \pre \e i must be in the heap.
    Prio operator[](const Item &i) const {
      int idx = _iim[i];
      return data[idx].prio;
    }

    /// \brief Set the priority of an item or insert it, if it is
    /// not stored in the heap.
    ///
    /// This method sets the priority of the given item if it is
    /// already stored in the heap. Otherwise it inserts the given
    /// item into the heap with the given priority.
    /// \param i The item.
    /// \param p The priority.
    /// \pre \e i must be in the heap.
    /// \warning This method may throw an \c UnderFlowPriorityException.
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

    /// \brief Decrease the priority of an item to the given value.
    ///
    /// This function decreases the priority of an item to the given value.
    /// \param i The item.
    /// \param p The priority.
    /// \pre \e i must be stored in the heap with priority at least \e p.
    /// \warning This method may throw an \c UnderFlowPriorityException.
    void decrease(const Item &i, const Prio &p) {
      int idx = _iim[i];
      data[idx].prio = p;
      bubble_down(idx);
    }

    /// \brief Increase the priority of an item to the given value.
    ///
    /// This function increases the priority of an item to the given value.
    /// \param i The item.
    /// \param p The priority.
    /// \pre \e i must be stored in the heap with priority at most \e p.
    void increase(const Item &i, const Prio &p) {
      int idx = _iim[i];
      data[idx].prio = p;
      bubble_up(idx);
    }

    /// \brief Return the state of an item.
    ///
    /// This method returns \c PRE_HEAP if the given item has never
    /// been in the heap, \c IN_HEAP if it is in the heap at the moment,
    /// and \c POST_HEAP otherwise.
    /// In the latter case it is possible that the item will get back
    /// to the heap again.
    /// \param i The item.
    State state(const Item &i) const {
      int s = _iim[i];
      if( s >= 0 ) s = 0;
      return State(s);
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

  }; // class RadixHeap

} // namespace lemon

#endif // LEMON_RADIX_HEAP_H
