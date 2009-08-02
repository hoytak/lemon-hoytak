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

#ifndef LEMON_BUCKET_HEAP_H
#define LEMON_BUCKET_HEAP_H

///\ingroup auxdat
///\file
///\brief Bucket Heap implementation.

#include <vector>
#include <utility>
#include <functional>

namespace lemon {

  namespace _bucket_heap_bits {

    template <bool MIN>
    struct DirectionTraits {
      static bool less(int left, int right) {
        return left < right;
      }
      static void increase(int& value) {
        ++value;
      }
    };

    template <>
    struct DirectionTraits<false> {
      static bool less(int left, int right) {
        return left > right;
      }
      static void increase(int& value) {
        --value;
      }
    };

  }

  /// \ingroup auxdat
  ///
  /// \brief A Bucket Heap implementation.
  ///
  /// This class implements the \e bucket \e heap data structure. A \e heap
  /// is a data structure for storing items with specified values called \e
  /// priorities in such a way that finding the item with minimum priority is
  /// efficient. The bucket heap is very simple implementation, it can store
  /// only integer priorities and it stores for each priority in the
  /// \f$ [0..C) \f$ range a list of items. So it should be used only when
  /// the priorities are small. It is not intended to use as dijkstra heap.
  ///
  /// \param IM A read and write Item int map, used internally
  /// to handle the cross references.
  /// \param MIN If the given parameter is false then instead of the
  /// minimum value the maximum can be retrivied with the top() and
  /// prio() member functions.
  template <typename IM, bool MIN = true>
  class BucketHeap {

  public:
    /// \e
    typedef typename IM::Key Item;
    /// \e
    typedef int Prio;
    /// \e
    typedef std::pair<Item, Prio> Pair;
    /// \e
    typedef IM ItemIntMap;

  private:

    typedef _bucket_heap_bits::DirectionTraits<MIN> Direction;

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

  public:
    /// \brief The constructor.
    ///
    /// The constructor.
    /// \param map should be given to the constructor, since it is used
    /// internally to handle the cross references. The value of the map
    /// should be PRE_HEAP (-1) for each element.
    explicit BucketHeap(ItemIntMap &map) : _iim(map), _minimum(0) {}

    /// The number of items stored in the heap.
    ///
    /// \brief Returns the number of items stored in the heap.
    int size() const { return _data.size(); }

    /// \brief Checks if the heap stores no items.
    ///
    /// Returns \c true if and only if the heap stores no items.
    bool empty() const { return _data.empty(); }

    /// \brief Make empty this heap.
    ///
    /// Make empty this heap. It does not change the cross reference
    /// map.  If you want to reuse a heap what is not surely empty you
    /// should first clear the heap and after that you should set the
    /// cross reference map for each item to \c PRE_HEAP.
    void clear() {
      _data.clear(); _first.clear(); _minimum = 0;
    }

  private:

    void relocate_last(int idx) {
      if (idx + 1 < int(_data.size())) {
        _data[idx] = _data.back();
        if (_data[idx].prev != -1) {
          _data[_data[idx].prev].next = idx;
        } else {
          _first[_data[idx].value] = idx;
        }
        if (_data[idx].next != -1) {
          _data[_data[idx].next].prev = idx;
        }
        _iim[_data[idx].item] = idx;
      }
      _data.pop_back();
    }

    void unlace(int idx) {
      if (_data[idx].prev != -1) {
        _data[_data[idx].prev].next = _data[idx].next;
      } else {
        _first[_data[idx].value] = _data[idx].next;
      }
      if (_data[idx].next != -1) {
        _data[_data[idx].next].prev = _data[idx].prev;
      }
    }

    void lace(int idx) {
      if (int(_first.size()) <= _data[idx].value) {
        _first.resize(_data[idx].value + 1, -1);
      }
      _data[idx].next = _first[_data[idx].value];
      if (_data[idx].next != -1) {
        _data[_data[idx].next].prev = idx;
      }
      _first[_data[idx].value] = idx;
      _data[idx].prev = -1;
    }

  public:
    /// \brief Insert a pair of item and priority into the heap.
    ///
    /// Adds \c p.first to the heap with priority \c p.second.
    /// \param p The pair to insert.
    void push(const Pair& p) {
      push(p.first, p.second);
    }

    /// \brief Insert an item into the heap with the given priority.
    ///
    /// Adds \c i to the heap with priority \c p.
    /// \param i The item to insert.
    /// \param p The priority of the item.
    void push(const Item &i, const Prio &p) {
      int idx = _data.size();
      _iim[i] = idx;
      _data.push_back(BucketItem(i, p));
      lace(idx);
      if (Direction::less(p, _minimum)) {
        _minimum = p;
      }
    }

    /// \brief Returns the item with minimum priority.
    ///
    /// This method returns the item with minimum priority.
    /// \pre The heap must be nonempty.
    Item top() const {
      while (_first[_minimum] == -1) {
        Direction::increase(_minimum);
      }
      return _data[_first[_minimum]].item;
    }

    /// \brief Returns the minimum priority.
    ///
    /// It returns the minimum priority.
    /// \pre The heap must be nonempty.
    Prio prio() const {
      while (_first[_minimum] == -1) {
        Direction::increase(_minimum);
      }
      return _minimum;
    }

    /// \brief Deletes the item with minimum priority.
    ///
    /// This method deletes the item with minimum priority from the heap.
    /// \pre The heap must be non-empty.
    void pop() {
      while (_first[_minimum] == -1) {
        Direction::increase(_minimum);
      }
      int idx = _first[_minimum];
      _iim[_data[idx].item] = -2;
      unlace(idx);
      relocate_last(idx);
    }

    /// \brief Deletes \c i from the heap.
    ///
    /// This method deletes item \c i from the heap, if \c i was
    /// already stored in the heap.
    /// \param i The item to erase.
    void erase(const Item &i) {
      int idx = _iim[i];
      _iim[_data[idx].item] = -2;
      unlace(idx);
      relocate_last(idx);
    }


    /// \brief Returns the priority of \c i.
    ///
    /// This function returns the priority of item \c i.
    /// \pre \c i must be in the heap.
    /// \param i The item.
    Prio operator[](const Item &i) const {
      int idx = _iim[i];
      return _data[idx].value;
    }

    /// \brief \c i gets to the heap with priority \c p independently
    /// if \c i was already there.
    ///
    /// This method calls \ref push(\c i, \c p) if \c i is not stored
    /// in the heap and sets the priority of \c i to \c p otherwise.
    /// \param i The item.
    /// \param p The priority.
    void set(const Item &i, const Prio &p) {
      int idx = _iim[i];
      if (idx < 0) {
        push(i, p);
      } else if (Direction::less(p, _data[idx].value)) {
        decrease(i, p);
      } else {
        increase(i, p);
      }
    }

    /// \brief Decreases the priority of \c i to \c p.
    ///
    /// This method decreases the priority of item \c i to \c p.
    /// \pre \c i must be stored in the heap with priority at least \c
    /// p relative to \c Compare.
    /// \param i The item.
    /// \param p The priority.
    void decrease(const Item &i, const Prio &p) {
      int idx = _iim[i];
      unlace(idx);
      _data[idx].value = p;
      if (Direction::less(p, _minimum)) {
        _minimum = p;
      }
      lace(idx);
    }

    /// \brief Increases the priority of \c i to \c p.
    ///
    /// This method sets the priority of item \c i to \c p.
    /// \pre \c i must be stored in the heap with priority at most \c
    /// p relative to \c Compare.
    /// \param i The item.
    /// \param p The priority.
    void increase(const Item &i, const Prio &p) {
      int idx = _iim[i];
      unlace(idx);
      _data[idx].value = p;
      lace(idx);
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
      int idx = _iim[i];
      if (idx >= 0) idx = 0;
      return State(idx);
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

  private:

    struct BucketItem {
      BucketItem(const Item& _item, int _value)
        : item(_item), value(_value) {}

      Item item;
      int value;

      int prev, next;
    };

    ItemIntMap& _iim;
    std::vector<int> _first;
    std::vector<BucketItem> _data;
    mutable int _minimum;

  }; // class BucketHeap

  /// \ingroup auxdat
  ///
  /// \brief A Simplified Bucket Heap implementation.
  ///
  /// This class implements a simplified \e bucket \e heap data
  /// structure.  It does not provide some functionality but it faster
  /// and simplier data structure than the BucketHeap. The main
  /// difference is that the BucketHeap stores for every key a double
  /// linked list while this class stores just simple lists. In the
  /// other way it does not support erasing each elements just the
  /// minimal and it does not supports key increasing, decreasing.
  ///
  /// \param IM A read and write Item int map, used internally
  /// to handle the cross references.
  /// \param MIN If the given parameter is false then instead of the
  /// minimum value the maximum can be retrivied with the top() and
  /// prio() member functions.
  ///
  /// \sa BucketHeap
  template <typename IM, bool MIN = true >
  class SimpleBucketHeap {

  public:
    typedef typename IM::Key Item;
    typedef int Prio;
    typedef std::pair<Item, Prio> Pair;
    typedef IM ItemIntMap;

  private:

    typedef _bucket_heap_bits::DirectionTraits<MIN> Direction;

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

  public:

    /// \brief The constructor.
    ///
    /// The constructor.
    /// \param map should be given to the constructor, since it is used
    /// internally to handle the cross references. The value of the map
    /// should be PRE_HEAP (-1) for each element.
    explicit SimpleBucketHeap(ItemIntMap &map)
      : _iim(map), _free(-1), _num(0), _minimum(0) {}

    /// \brief Returns the number of items stored in the heap.
    ///
    /// The number of items stored in the heap.
    int size() const { return _num; }

    /// \brief Checks if the heap stores no items.
    ///
    /// Returns \c true if and only if the heap stores no items.
    bool empty() const { return _num == 0; }

    /// \brief Make empty this heap.
    ///
    /// Make empty this heap. It does not change the cross reference
    /// map.  If you want to reuse a heap what is not surely empty you
    /// should first clear the heap and after that you should set the
    /// cross reference map for each item to \c PRE_HEAP.
    void clear() {
      _data.clear(); _first.clear(); _free = -1; _num = 0; _minimum = 0;
    }

    /// \brief Insert a pair of item and priority into the heap.
    ///
    /// Adds \c p.first to the heap with priority \c p.second.
    /// \param p The pair to insert.
    void push(const Pair& p) {
      push(p.first, p.second);
    }

    /// \brief Insert an item into the heap with the given priority.
    ///
    /// Adds \c i to the heap with priority \c p.
    /// \param i The item to insert.
    /// \param p The priority of the item.
    void push(const Item &i, const Prio &p) {
      int idx;
      if (_free == -1) {
        idx = _data.size();
        _data.push_back(BucketItem(i));
      } else {
        idx = _free;
        _free = _data[idx].next;
        _data[idx].item = i;
      }
      _iim[i] = idx;
      if (p >= int(_first.size())) _first.resize(p + 1, -1);
      _data[idx].next = _first[p];
      _first[p] = idx;
      if (Direction::less(p, _minimum)) {
        _minimum = p;
      }
      ++_num;
    }

    /// \brief Returns the item with minimum priority.
    ///
    /// This method returns the item with minimum priority.
    /// \pre The heap must be nonempty.
    Item top() const {
      while (_first[_minimum] == -1) {
        Direction::increase(_minimum);
      }
      return _data[_first[_minimum]].item;
    }

    /// \brief Returns the minimum priority.
    ///
    /// It returns the minimum priority.
    /// \pre The heap must be nonempty.
    Prio prio() const {
      while (_first[_minimum] == -1) {
        Direction::increase(_minimum);
      }
      return _minimum;
    }

    /// \brief Deletes the item with minimum priority.
    ///
    /// This method deletes the item with minimum priority from the heap.
    /// \pre The heap must be non-empty.
    void pop() {
      while (_first[_minimum] == -1) {
        Direction::increase(_minimum);
      }
      int idx = _first[_minimum];
      _iim[_data[idx].item] = -2;
      _first[_minimum] = _data[idx].next;
      _data[idx].next = _free;
      _free = idx;
      --_num;
    }

    /// \brief Returns the priority of \c i.
    ///
    /// This function returns the priority of item \c i.
    /// \warning This operator is not a constant time function
    /// because it scans the whole data structure to find the proper
    /// value.
    /// \pre \c i must be in the heap.
    /// \param i The item.
    Prio operator[](const Item &i) const {
      for (int k = 0; k < _first.size(); ++k) {
        int idx = _first[k];
        while (idx != -1) {
          if (_data[idx].item == i) {
            return k;
          }
          idx = _data[idx].next;
        }
      }
      return -1;
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
      int idx = _iim[i];
      if (idx >= 0) idx = 0;
      return State(idx);
    }

  private:

    struct BucketItem {
      BucketItem(const Item& _item)
        : item(_item) {}

      Item item;
      int next;
    };

    ItemIntMap& _iim;
    std::vector<int> _first;
    std::vector<BucketItem> _data;
    int _free, _num;
    mutable int _minimum;

  }; // class SimpleBucketHeap

}

#endif
