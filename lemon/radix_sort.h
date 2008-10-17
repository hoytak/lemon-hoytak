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

#ifndef RADIX_SORT_H
#define RADIX_SORT_H

/// \ingroup auxalg
/// \file
/// \brief Radix sort
///
/// Linear time sorting algorithms

#include <vector>
#include <limits>
#include <iterator>
#include <algorithm>

namespace lemon {

  namespace _radix_sort_bits {

    template <typename Value>
    struct Identity {
      const Value& operator()(const Value& val) {
	return val;
      }
    };


    template <typename Value, typename Iterator, typename Functor>
    Iterator radixSortPartition(Iterator first, Iterator last, 
				Functor functor, Value mask) {
      while (first != last && !(functor(*first) & mask)) {
	++first;
      }
      if (first == last) {
	return first;
      }
      --last;
      while (first != last && (functor(*last) & mask)) {
	--last;
      }
      if (first == last) {
	return first;
      }
      std::iter_swap(first, last);
      ++first;
      if (!(first < last)) {
	return first;
      }
      while (true) {
	while (!(functor(*first) & mask)) {
	  ++first;
	}
	--last;
	while (functor(*last) & mask) {
	  --last;
	}
	if (!(first < last)) {
	  return first;
	}
	std::iter_swap(first, last);
	++first;
      }
    }

    template <typename Iterator, typename Functor>
    Iterator radixSortSignPartition(Iterator first, Iterator last, 
				    Functor functor) {
      while (first != last && functor(*first) < 0) {
	++first;
      }
      if (first == last) {
	return first;
      }
      --last;
      while (first != last && functor(*last) >= 0) {
	--last;
      }
      if (first == last) {
	return first;
      }
      std::iter_swap(first, last);
      ++first;
      if (!(first < last)) {
	return first;
      }
      while (true) {
	while (functor(*first) < 0) {
	  ++first;
	}
	--last;
	while (functor(*last) >= 0) {
	  --last;
	}
	if (!(first < last)) {
	  return first;
	}
	std::iter_swap(first, last);
	++first;
      }
    }

    template <typename Value, typename Iterator, typename Functor>
    void radixIntroSort(Iterator first, Iterator last, 
			Functor functor, Value mask) {
      while (mask != 0 && last - first > 1) {
	Iterator cut = radixSortPartition(first, last, functor, mask);
	mask >>= 1;
	radixIntroSort(first, cut, functor, mask);
	first = cut;
      }
    }

    template <typename Value, typename Iterator, typename Functor>
    void radixSignedSort(Iterator first, Iterator last, Functor functor) {

      Iterator cut = radixSortSignPartition(first, last, functor);

      Value mask;
      int max_digit;
      Iterator it;

      mask = ~0; max_digit = 0;
      for (it = first; it != cut; ++it) {
	while ((mask & functor(*it)) != mask) {
	  ++max_digit;
	  mask <<= 1;
	}
      }
      radixIntroSort(first, cut, functor, 1 << max_digit);

      mask = 0; max_digit = 0;
      for (it = cut; it != last; ++it) {
	while ((mask | functor(*it)) != mask) {
	  ++max_digit;
	  mask <<= 1; mask |= 1;
	}
      }
      radixIntroSort(cut, last, functor, 1 << max_digit);
    }

    template <typename Value, typename Iterator, typename Functor>
    void radixUnsignedSort(Iterator first, Iterator last, Functor functor) {

      Value mask = 0;
      int max_digit = 0;

      Iterator it;
      for (it = first; it != last; ++it) {
	while ((mask | functor(*it)) != mask) {
	  ++max_digit;
	  mask <<= 1; mask |= 1;
	}
      }
      radixIntroSort(first, last, functor, 1 << max_digit);
    }


    template <typename Value, 
	      bool sign = std::numeric_limits<Value>::is_signed >
    struct RadixSortSelector {
      template <typename Iterator, typename Functor>
      static void sort(Iterator first, Iterator last, Functor functor) {
	radixSignedSort<Value>(first, last, functor);
      }
    };

    template <typename Value>
    struct RadixSortSelector<Value, false> {
      template <typename Iterator, typename Functor>
      static void sort(Iterator first, Iterator last, Functor functor) {
	radixUnsignedSort<Value>(first, last, functor);
      }
    };

  }

  /// \ingroup auxalg
  ///
  /// \brief Sorts the STL compatible range into ascending order.
  ///
  /// The \c radixSort sorts the STL compatible range into ascending
  /// order.  The radix sort algorithm can sort the items which mapped
  /// to an integer with an adaptable unary function \c functor and the
  /// order will be ascending by these mapped values. As function
  /// specialization it is possible to use a normal function instead
  /// of the functor object or if the functor is not given it will use
  /// an identity function instead.
  ///
  /// This implemented radix sort is a special quick sort which pivot
  /// value is choosen to partite the items on the next
  /// bit. Therefore, let be \c c the maximal capacity and \c n the
  /// number of the items in the container, the time complexity of the
  /// algorithm is \f$ O(\log(c)n) \f$ and the additional space
  /// complexity is \f$ O(\log(c)) \f$.
  ///
  /// \param first The begin of the given range.
  /// \param last The end of the given range.
  /// \param functor An adaptible unary function or a normal function
  /// which maps the items to any integer type which can be either
  /// signed or unsigned.
  template <typename Iterator, typename Functor>
  void radixSort(Iterator first, Iterator last, Functor functor) {
    using namespace _radix_sort_bits;
    typedef typename Functor::result_type Value;
    RadixSortSelector<Value>::sort(first, last, functor);
  }

  template <typename Iterator, typename Value, typename Key>
  void radixSort(Iterator first, Iterator last, Value (*functor)(Key)) {
    using namespace _radix_sort_bits;
    RadixSortSelector<Value>::sort(first, last, functor);
  }

  template <typename Iterator, typename Value, typename Key>
  void radixSort(Iterator first, Iterator last, Value& (*functor)(Key)) {
    using namespace _radix_sort_bits;
    RadixSortSelector<Value>::sort(first, last, functor);
  }

  template <typename Iterator, typename Value, typename Key>
  void radixSort(Iterator first, Iterator last, Value (*functor)(Key&)) {
    using namespace _radix_sort_bits;
    RadixSortSelector<Value>::sort(first, last, functor);
  }

  template <typename Iterator, typename Value, typename Key>
  void radixSort(Iterator first, Iterator last, Value& (*functor)(Key&)) {
    using namespace _radix_sort_bits;
    RadixSortSelector<Value>::sort(first, last, functor);
  }

  template <typename Iterator>
  void radixSort(Iterator first, Iterator last) {
    using namespace _radix_sort_bits;
    typedef typename std::iterator_traits<Iterator>::value_type Value;
    RadixSortSelector<Value>::sort(first, last, Identity<Value>());
  }

  namespace _radix_sort_bits {

    template <typename Value>
    unsigned char valueByte(Value value, int byte) {
      return value >> (std::numeric_limits<unsigned char>::digits * byte);
    }

    template <typename Functor, typename Key>
    void counterIntroSort(Key *first, Key *last, Key *target, 
			  int byte, Functor functor) {
      const int size = 
	unsigned(std::numeric_limits<unsigned char>::max()) + 1;
      std::vector<int> counter(size);
      for (int i = 0; i < size; ++i) {
	counter[i] = 0;
      }
      Key *it = first;
      while (first != last) {
	++counter[valueByte(functor(*first), byte)]; 
	++first;
      }
      int prev, num = 0;
      for (int i = 0; i < size; ++i) {
	prev = num;
	num += counter[i];
	counter[i] = prev;
      }
      while (it != last) {
	target[counter[valueByte(functor(*it), byte)]++] = *it;
	++it;
      }
    }

    template <typename Functor, typename Key>
    void signedCounterIntroSort(Key *first, Key *last, Key *target, 
				int byte, Functor functor) {
      const int size = 
	unsigned(std::numeric_limits<unsigned char>::max()) + 1;
      std::vector<int> counter(size);
      for (int i = 0; i < size; ++i) {
	counter[i] = 0;
      }
      Key *it = first;
      while (first != last) {
	counter[valueByte(functor(*first), byte)]++;
	++first;
      }
      int prev, num = 0;
      for (int i = size / 2; i < size; ++i) {
	prev = num;
	num += counter[i];
	counter[i] = prev;
      }
      for (int i = 0; i < size / 2; ++i) {
	prev = num;
	num += counter[i];
	counter[i] = prev;
      }
      while (it != last) {
	target[counter[valueByte(functor(*it), byte)]++] = *it;
	++it;
      }
    }

  
    template <typename Value, typename Iterator, typename Functor>
    void counterSignedSort(Iterator first, Iterator last, Functor functor) {
      if (first == last) return;
      typedef typename std::iterator_traits<Iterator>::value_type Key;
      typedef std::allocator<Key> Allocator;
      Allocator allocator;

      int length = std::distance(first, last);
      Key* buffer = allocator.allocate(2 * length);
      try {
	bool dir = true;
	std::copy(first, last, buffer);
	for (int i = 0; i < int(sizeof(Value)) - 1; ++i) {
	  if (dir) {
	    counterIntroSort(buffer, buffer + length, buffer + length, 
			     i, functor);
	  } else {
	    counterIntroSort(buffer + length, buffer + 2 * length, buffer, 
			     i, functor);
	  }
	  dir = !dir;
	}
	if (dir) {
	  signedCounterIntroSort(buffer, buffer + length, buffer + length, 
				 sizeof(Value) - 1, functor);
	  std::copy(buffer + length, buffer + 2 * length, first);
	}	else {
	  signedCounterIntroSort(buffer + length, buffer + 2 * length, buffer, 
				 sizeof(Value) - 1, functor);
	  std::copy(buffer, buffer + length, first);
	}
      } catch (...) {
	allocator.deallocate(buffer, 2 * length);
	throw;
      }
      allocator.deallocate(buffer, 2 * length);
    }

    template <typename Value, typename Iterator, typename Functor>
    void counterUnsignedSort(Iterator first, Iterator last, Functor functor) {
      if (first == last) return;
      typedef typename std::iterator_traits<Iterator>::value_type Key;
      typedef std::allocator<Key> Allocator;
      Allocator allocator;

      int length = std::distance(first, last);
      Key *buffer = allocator.allocate(2 * length);
      try {
	bool dir = true;
	std::copy(first, last, buffer);
	for (int i = 0; i < int(sizeof(Value)); ++i) {
	  if (dir) {
	    counterIntroSort(buffer, buffer + length, 
			     buffer + length, i, functor);
	  } else {
	    counterIntroSort(buffer + length, buffer + 2 * length, 
			     buffer, i, functor);
	  }
	  dir = !dir;
	}
	if (dir) {
	  std::copy(buffer, buffer + length, first);
	}	else {
	  std::copy(buffer + length, buffer + 2 * length, first);
	}
      } catch (...) {
	allocator.deallocate(buffer, 2 * length);
	throw;
      }
      allocator.deallocate(buffer, 2 * length);
    }



    template <typename Value, 
	      bool sign = std::numeric_limits<Value>::is_signed >
    struct CounterSortSelector {
      template <typename Iterator, typename Functor>
      static void sort(Iterator first, Iterator last, Functor functor) {
	counterSignedSort<Value>(first, last, functor);
      }
    };

    template <typename Value>
    struct CounterSortSelector<Value, false> {
      template <typename Iterator, typename Functor>
      static void sort(Iterator first, Iterator last, Functor functor) {
	counterUnsignedSort<Value>(first, last, functor);
      }
    };

  }

  /// \ingroup auxalg
  ///
  /// \brief Sorts stable the STL compatible range into ascending order.
  ///
  /// The \c counterSort sorts the STL compatible range into ascending
  /// order.  The counter sort algorithm can sort the items which
  /// mapped to an integer with an adaptable unary function \c functor
  /// and the order will be ascending by these mapped values. As
  /// function specialization it is possible to use a normal function
  /// instead of the functor object or if the functor is not given it
  /// will use an identity function instead.
  ///
  /// The implemented counter sort use a radix forward sort on the
  /// bytes of the integer number. The algorithm sorts the items
  /// byte-by-byte, first it counts how many times occurs a byte value
  /// in the containerm, and with the occurence number the container
  /// can be copied to an other in asceding order in \c O(n) time.
  /// Let be \c c the maximal capacity of the integer type and \c n
  /// the number of the items in the container, the time complexity of
  /// the algorithm is \f$ O(\log(c)n) \f$ and the additional space
  /// complexity is \f$ O(n) \f$.
  ///
  /// The sorting algorithm is stable, i.e. the order of two equal
  /// element remains the same.
  ///
  /// \param first The begin of the given range.
  /// \param last The end of the given range.
  /// \param functor An adaptible unary function or a normal function
  /// which maps the items to any integer type which can be either
  /// signed or unsigned.
  template <typename Iterator, typename Functor>
  void counterSort(Iterator first, Iterator last, Functor functor) {
    using namespace _radix_sort_bits;
    typedef typename Functor::result_type Value;
    CounterSortSelector<Value>::sort(first, last, functor);
  }

  template <typename Iterator, typename Value, typename Key>
  void counterSort(Iterator first, Iterator last, Value (*functor)(Key)) {
    using namespace _radix_sort_bits;
    CounterSortSelector<Value>::sort(first, last, functor);
  }

  template <typename Iterator, typename Value, typename Key>
  void counterSort(Iterator first, Iterator last, Value& (*functor)(Key)) {
    using namespace _radix_sort_bits;
    CounterSortSelector<Value>::sort(first, last, functor);
  }

  template <typename Iterator, typename Value, typename Key>
  void counterSort(Iterator first, Iterator last, Value (*functor)(Key&)) {
    using namespace _radix_sort_bits;
    CounterSortSelector<Value>::sort(first, last, functor);
  }

  template <typename Iterator, typename Value, typename Key>
  void counterSort(Iterator first, Iterator last, Value& (*functor)(Key&)) {
    using namespace _radix_sort_bits;
    CounterSortSelector<Value>::sort(first, last, functor);
  }

  template <typename Iterator>
  void counterSort(Iterator first, Iterator last) {
    using namespace _radix_sort_bits;
    typedef typename std::iterator_traits<Iterator>::value_type Value;
    CounterSortSelector<Value>::sort(first, last, Identity<Value>());
  }

}

#endif
