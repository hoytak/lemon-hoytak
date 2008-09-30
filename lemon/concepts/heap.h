/* -*- mode: C++; indent-tabs-mode: nil; -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library.
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

///\ingroup concept
///\file
///\brief The concept of heaps.

#ifndef LEMON_CONCEPT_HEAP_H
#define LEMON_CONCEPT_HEAP_H

#include <lemon/core.h>

namespace lemon {

  namespace concepts {

    /// \addtogroup concept
    /// @{

    /// \brief The heap concept.
    ///
    /// Concept class describing the main interface of heaps.
    template <typename Priority, typename ItemIntMap>
    class Heap {
    public:

      /// Type of the items stored in the heap.
      typedef typename ItemIntMap::Key Item;

      /// Type of the priorities.
      typedef Priority Prio;

      /// \brief Type to represent the states of the items.
      ///
      /// Each item has a state associated to it. It can be "in heap",
      /// "pre heap" or "post heap". The later two are indifferent
      /// from the point of view of the heap, but may be useful for
      /// the user.
      ///
      /// The \c ItemIntMap must be initialized in such a way, that it
      /// assigns \c PRE_HEAP (<tt>-1</tt>) to every item.
      enum State {
        IN_HEAP = 0,
        PRE_HEAP = -1,
        POST_HEAP = -2
      };

      /// \brief The constructor.
      ///
      /// The constructor.
      /// \param map A map that assigns \c int values to keys of type
      /// \c Item. It is used internally by the heap implementations to
      /// handle the cross references. The assigned value must be
      /// \c PRE_HEAP (<tt>-1</tt>) for every item.
      explicit Heap(ItemIntMap &map) {}

      /// \brief The number of items stored in the heap.
      ///
      /// Returns the number of items stored in the heap.
      int size() const { return 0; }

      /// \brief Checks if the heap is empty.
      ///
      /// Returns \c true if the heap is empty.
      bool empty() const { return false; }

      /// \brief Makes the heap empty.
      ///
      /// Makes the heap empty.
      void clear();

      /// \brief Inserts an item into the heap with the given priority.
      ///
      /// Inserts the given item into the heap with the given priority.
      /// \param i The item to insert.
      /// \param p The priority of the item.
      void push(const Item &i, const Prio &p) {}

      /// \brief Returns the item having minimum priority.
      ///
      /// Returns the item having minimum priority.
      /// \pre The heap must be non-empty.
      Item top() const {}

      /// \brief The minimum priority.
      ///
      /// Returns the minimum priority.
      /// \pre The heap must be non-empty.
      Prio prio() const {}

      /// \brief Removes the item having minimum priority.
      ///
      /// Removes the item having minimum priority.
      /// \pre The heap must be non-empty.
      void pop() {}

      /// \brief Removes an item from the heap.
      ///
      /// Removes the given item from the heap if it is already stored.
      /// \param i The item to delete.
      void erase(const Item &i) {}

      /// \brief The priority of an item.
      ///
      /// Returns the priority of the given item.
      /// \pre \c i must be in the heap.
      /// \param i The item.
      Prio operator[](const Item &i) const {}

      /// \brief Sets the priority of an item or inserts it, if it is
      /// not stored in the heap.
      ///
      /// This method sets the priority of the given item if it is
      /// already stored in the heap.
      /// Otherwise it inserts the given item with the given priority.
      ///
      /// \param i The item.
      /// \param p The priority.
      void set(const Item &i, const Prio &p) {}

      /// \brief Decreases the priority of an item to the given value.
      ///
      /// Decreases the priority of an item to the given value.
      /// \pre \c i must be stored in the heap with priority at least \c p.
      /// \param i The item.
      /// \param p The priority.
      void decrease(const Item &i, const Prio &p) {}

      /// \brief Increases the priority of an item to the given value.
      ///
      /// Increases the priority of an item to the given value.
      /// \pre \c i must be stored in the heap with priority at most \c p.
      /// \param i The item.
      /// \param p The priority.
      void increase(const Item &i, const Prio &p) {}

      /// \brief Returns if an item is in, has already been in, or has
      /// never been in the heap.
      ///
      /// This method returns \c PRE_HEAP if the given item has never
      /// been in the heap, \c IN_HEAP if it is in the heap at the moment,
      /// and \c POST_HEAP otherwise.
      /// In the latter case it is possible that the item will get back
      /// to the heap again.
      /// \param i The item.
      State state(const Item &i) const {}

      /// \brief Sets the state of an item in the heap.
      ///
      /// Sets the state of the given item in the heap. It can be used
      /// to manually clear the heap when it is important to achive the
      /// better time complexity.
      /// \param i The item.
      /// \param st The state. It should not be \c IN_HEAP.
      void state(const Item& i, State st) {}


      template <typename _Heap>
      struct Constraints {
      public:
        void constraints() {
          typedef typename _Heap::Item OwnItem;
          typedef typename _Heap::Prio OwnPrio;
          typedef typename _Heap::State OwnState;

          Item item;
          Prio prio;
          item=Item();
          prio=Prio();
          ignore_unused_variable_warning(item);
          ignore_unused_variable_warning(prio);

          OwnItem own_item;
          OwnPrio own_prio;
          OwnState own_state;
          own_item=Item();
          own_prio=Prio();
          ignore_unused_variable_warning(own_item);
          ignore_unused_variable_warning(own_prio);
          ignore_unused_variable_warning(own_state);

          _Heap heap1(map);
          _Heap heap2 = heap1;
          ignore_unused_variable_warning(heap1);
          ignore_unused_variable_warning(heap2);

          int s = heap.size();
          ignore_unused_variable_warning(s);
          bool e = heap.empty();
          ignore_unused_variable_warning(e);

          prio = heap.prio();
          item = heap.top();
          prio = heap[item];
          own_prio = heap.prio();
          own_item = heap.top();
          own_prio = heap[own_item];

          heap.push(item, prio);
          heap.push(own_item, own_prio);
          heap.pop();

          heap.set(item, prio);
          heap.decrease(item, prio);
          heap.increase(item, prio);
          heap.set(own_item, own_prio);
          heap.decrease(own_item, own_prio);
          heap.increase(own_item, own_prio);

          heap.erase(item);
          heap.erase(own_item);
          heap.clear();

          own_state = heap.state(own_item);
          heap.state(own_item, own_state);

          own_state = _Heap::PRE_HEAP;
          own_state = _Heap::IN_HEAP;
          own_state = _Heap::POST_HEAP;
        }

        _Heap& heap;
        ItemIntMap& map;
      };
    };

    /// @}
  } // namespace lemon
}
#endif // LEMON_CONCEPT_PATH_H
