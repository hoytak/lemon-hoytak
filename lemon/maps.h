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

#ifndef LEMON_MAPS_H
#define LEMON_MAPS_H

#include <iterator>
#include <functional>
#include <vector>

#include <lemon/bits/utility.h>
#include <lemon/bits/traits.h>

///\file
///\ingroup maps
///\brief Miscellaneous property maps

#include <map>

namespace lemon {

  /// \addtogroup maps
  /// @{

  /// Base class of maps.

  /// Base class of maps. It provides the necessary type definitions
  /// required by the map %concepts.
  template<typename K, typename V>
  class MapBase {
  public:
    /// \biref The key type of the map.
    typedef K Key;
    /// \brief The value type of the map.
    /// (The type of objects associated with the keys).
    typedef V Value;
  };


  /// Null map. (a.k.a. DoNothingMap)

  /// This map can be used if you have to provide a map only for
  /// its type definitions, or if you have to provide a writable map,
  /// but data written to it is not required (i.e. it will be sent to
  /// <tt>/dev/null</tt>).
  /// It conforms the \ref concepts::ReadWriteMap "ReadWriteMap" concept.
  ///
  /// \sa ConstMap
  template<typename K, typename V>
  class NullMap : public MapBase<K, V> {
  public:
    typedef MapBase<K, V> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Gives back a default constructed element.
    Value operator[](const Key&) const { return Value(); }
    /// Absorbs the value.
    void set(const Key&, const Value&) {}
  };

  /// Returns a \ref NullMap class

  /// This function just returns a \ref NullMap class.
  /// \relates NullMap
  template <typename K, typename V>
  NullMap<K, V> nullMap() {
    return NullMap<K, V>();
  }


  /// Constant map.

  /// This \ref concepts::ReadMap "readable map" assigns a specified
  /// value to each key.
  ///
  /// In other aspects it is equivalent to \ref NullMap.
  /// So it conforms the \ref concepts::ReadWriteMap "ReadWriteMap"
  /// concept, but it absorbs the data written to it.
  ///
  /// The simplest way of using this map is through the constMap()
  /// function.
  ///
  /// \sa NullMap
  /// \sa IdentityMap
  template<typename K, typename V>
  class ConstMap : public MapBase<K, V> {
  private:
    V _value;
  public:
    typedef MapBase<K, V> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Default constructor

    /// Default constructor.
    /// The value of the map will be default constructed.
    ConstMap() {}

    /// Constructor with specified initial value

    /// Constructor with specified initial value.
    /// \param v The initial value of the map.
    ConstMap(const Value &v) : _value(v) {}

    /// Gives back the specified value.
    Value operator[](const Key&) const { return _value; }

    /// Absorbs the value.
    void set(const Key&, const Value&) {}

    /// Sets the value that is assigned to each key.
    void setAll(const Value &v) {
      _value = v;
    }

    template<typename V1>
    ConstMap(const ConstMap<K, V1> &, const Value &v) : _value(v) {}
  };

  /// Returns a \ref ConstMap class

  /// This function just returns a \ref ConstMap class.
  /// \relates ConstMap
  template<typename K, typename V>
  inline ConstMap<K, V> constMap(const V &v) {
    return ConstMap<K, V>(v);
  }

  template<typename K, typename V>
  inline ConstMap<K, V> constMap() {
    return ConstMap<K, V>();
  }


  template<typename T, T v>
  struct Const {};

  /// Constant map with inlined constant value.

  /// This \ref concepts::ReadMap "readable map" assigns a specified
  /// value to each key.
  ///
  /// In other aspects it is equivalent to \ref NullMap.
  /// So it conforms the \ref concepts::ReadWriteMap "ReadWriteMap"
  /// concept, but it absorbs the data written to it.
  ///
  /// The simplest way of using this map is through the constMap()
  /// function.
  ///
  /// \sa NullMap
  /// \sa IdentityMap
  template<typename K, typename V, V v>
  class ConstMap<K, Const<V, v> > : public MapBase<K, V> {
  public:
    typedef MapBase<K, V> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor.
    ConstMap() {}

    /// Gives back the specified value.
    Value operator[](const Key&) const { return v; }

    /// Absorbs the value.
    void set(const Key&, const Value&) {}
  };

  /// Returns a \ref ConstMap class with inlined constant value

  /// This function just returns a \ref ConstMap class with inlined
  /// constant value.
  /// \relates ConstMap
  template<typename K, typename V, V v>
  inline ConstMap<K, Const<V, v> > constMap() {
    return ConstMap<K, Const<V, v> >();
  }


  /// Identity map.

  /// This \ref concepts::ReadMap "read-only map" gives back the given
  /// key as value without any modification.
  ///
  /// \sa ConstMap
  template <typename T>
  class IdentityMap : public MapBase<T, T> {
  public:
    typedef MapBase<T, T> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Gives back the given value without any modification.
    Value operator[](const Key &k) const {
      return k;
    }
  };

  /// Returns an \ref IdentityMap class

  /// This function just returns an \ref IdentityMap class.
  /// \relates IdentityMap
  template<typename T>
  inline IdentityMap<T> identityMap() {
    return IdentityMap<T>();
  }


  /// \brief Map for storing values for integer keys from the range
  /// <tt>[0..size-1]</tt>.
  ///
  /// This map is essentially a wrapper for \c std::vector. It assigns
  /// values to integer keys from the range <tt>[0..size-1]</tt>.
  /// It can be used with some data structures, for example
  /// \ref UnionFind, \ref BinHeap, when the used items are small
  /// integers. This map conforms the \ref concepts::ReferenceMap
  /// "ReferenceMap" concept.
  ///
  /// The simplest way of using this map is through the rangeMap()
  /// function.
  template <typename V>
  class RangeMap : public MapBase<int, V> {
    template <typename V1>
    friend class RangeMap;
  private:

    typedef std::vector<V> Vector;
    Vector _vector;

  public:

    typedef MapBase<int, V> Parent;
    /// Key type
    typedef typename Parent::Key Key;
    /// Value type
    typedef typename Parent::Value Value;
    /// Reference type
    typedef typename Vector::reference Reference;
    /// Const reference type
    typedef typename Vector::const_reference ConstReference;

    typedef True ReferenceMapTag;

  public:

    /// Constructor with specified default value.
    RangeMap(int size = 0, const Value &value = Value())
      : _vector(size, value) {}

    /// Constructs the map from an appropriate \c std::vector.
    template <typename V1>
    RangeMap(const std::vector<V1>& vector)
      : _vector(vector.begin(), vector.end()) {}

    /// Constructs the map from another \ref RangeMap.
    template <typename V1>
    RangeMap(const RangeMap<V1> &c)
      : _vector(c._vector.begin(), c._vector.end()) {}

    /// Returns the size of the map.
    int size() {
      return _vector.size();
    }

    /// Resizes the map.

    /// Resizes the underlying \c std::vector container, so changes the
    /// keyset of the map.
    /// \param size The new size of the map. The new keyset will be the
    /// range <tt>[0..size-1]</tt>.
    /// \param value The default value to assign to the new keys.
    void resize(int size, const Value &value = Value()) {
      _vector.resize(size, value);
    }

  private:

    RangeMap& operator=(const RangeMap&);

  public:

    ///\e
    Reference operator[](const Key &k) {
      return _vector[k];
    }

    ///\e
    ConstReference operator[](const Key &k) const {
      return _vector[k];
    }

    ///\e
    void set(const Key &k, const Value &v) {
      _vector[k] = v;
    }
  };

  /// Returns a \ref RangeMap class

  /// This function just returns a \ref RangeMap class.
  /// \relates RangeMap
  template<typename V>
  inline RangeMap<V> rangeMap(int size = 0, const V &value = V()) {
    return RangeMap<V>(size, value);
  }

  /// \brief Returns a \ref RangeMap class created from an appropriate
  /// \c std::vector

  /// This function just returns a \ref RangeMap class created from an
  /// appropriate \c std::vector.
  /// \relates RangeMap
  template<typename V>
  inline RangeMap<V> rangeMap(const std::vector<V> &vector) {
    return RangeMap<V>(vector);
  }


  /// Map type based on \c std::map

  /// This map is essentially a wrapper for \c std::map with addition
  /// that you can specify a default value for the keys that are not
  /// stored actually. This value can be different from the default
  /// contructed value (i.e. \c %Value()).
  /// This type conforms the \ref concepts::ReferenceMap "ReferenceMap"
  /// concept.
  ///
  /// This map is useful if a default value should be assigned to most of
  /// the keys and different values should be assigned only to a few
  /// keys (i.e. the map is "sparse").
  /// The name of this type also refers to this important usage.
  ///
  /// Apart form that this map can be used in many other cases since it
  /// is based on \c std::map, which is a general associative container.
  /// However keep in mind that it is usually not as efficient as other
  /// maps.
  ///
  /// The simplest way of using this map is through the sparseMap()
  /// function.
  template <typename K, typename V, typename Compare = std::less<K> >
  class SparseMap : public MapBase<K, V> {
    template <typename K1, typename V1, typename C1>
    friend class SparseMap;
  public:

    typedef MapBase<K, V> Parent;
    /// Key type
    typedef typename Parent::Key Key;
    /// Value type
    typedef typename Parent::Value Value;
    /// Reference type
    typedef Value& Reference;
    /// Const reference type
    typedef const Value& ConstReference;

    typedef True ReferenceMapTag;

  private:

    typedef std::map<K, V, Compare> Map;
    Map _map;
    Value _value;

  public:

    /// \brief Constructor with specified default value.
    SparseMap(const Value &value = Value()) : _value(value) {}
    /// \brief Constructs the map from an appropriate \c std::map, and
    /// explicitly specifies a default value.
    template <typename V1, typename Comp1>
    SparseMap(const std::map<Key, V1, Comp1> &map,
              const Value &value = Value())
      : _map(map.begin(), map.end()), _value(value) {}

    /// \brief Constructs the map from another \ref SparseMap.
    template<typename V1, typename Comp1>
    SparseMap(const SparseMap<Key, V1, Comp1> &c)
      : _map(c._map.begin(), c._map.end()), _value(c._value) {}

  private:

    SparseMap& operator=(const SparseMap&);

  public:

    ///\e
    Reference operator[](const Key &k) {
      typename Map::iterator it = _map.lower_bound(k);
      if (it != _map.end() && !_map.key_comp()(k, it->first))
        return it->second;
      else
        return _map.insert(it, std::make_pair(k, _value))->second;
    }

    ///\e
    ConstReference operator[](const Key &k) const {
      typename Map::const_iterator it = _map.find(k);
      if (it != _map.end())
        return it->second;
      else
        return _value;
    }

    ///\e
    void set(const Key &k, const Value &v) {
      typename Map::iterator it = _map.lower_bound(k);
      if (it != _map.end() && !_map.key_comp()(k, it->first))
        it->second = v;
      else
        _map.insert(it, std::make_pair(k, v));
    }

    ///\e
    void setAll(const Value &v) {
      _value = v;
      _map.clear();
    }
  };

  /// Returns a \ref SparseMap class

  /// This function just returns a \ref SparseMap class with specified
  /// default value.
  /// \relates SparseMap
  template<typename K, typename V, typename Compare>
  inline SparseMap<K, V, Compare> sparseMap(const V& value = V()) {
    return SparseMap<K, V, Compare>(value);
  }

  template<typename K, typename V>
  inline SparseMap<K, V, std::less<K> > sparseMap(const V& value = V()) {
    return SparseMap<K, V, std::less<K> >(value);
  }

  /// \brief Returns a \ref SparseMap class created from an appropriate
  /// \c std::map

  /// This function just returns a \ref SparseMap class created from an
  /// appropriate \c std::map.
  /// \relates SparseMap
  template<typename K, typename V, typename Compare>
  inline SparseMap<K, V, Compare>
    sparseMap(const std::map<K, V, Compare> &map, const V& value = V())
  {
    return SparseMap<K, V, Compare>(map, value);
  }

  /// @}

  /// \addtogroup map_adaptors
  /// @{

  /// Composition of two maps

  /// This \ref concepts::ReadMap "read-only map" returns the
  /// composition of two given maps. That is to say, if \c m1 is of
  /// type \c M1 and \c m2 is of \c M2, then for
  /// \code
  ///   ComposeMap<M1, M2> cm(m1,m2);
  /// \endcode
  /// <tt>cm[x]</tt> will be equal to <tt>m1[m2[x]]</tt>.
  ///
  /// The \c Key type of the map is inherited from \c M2 and the
  /// \c Value type is from \c M1.
  /// \c M2::Value must be convertible to \c M1::Key.
  ///
  /// The simplest way of using this map is through the composeMap()
  /// function.
  ///
  /// \sa CombineMap
  ///
  /// \todo Check the requirements.
  template <typename M1, typename M2>
  class ComposeMap : public MapBase<typename M2::Key, typename M1::Value> {
    const M1 &_m1;
    const M2 &_m2;
  public:
    typedef MapBase<typename M2::Key, typename M1::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    ComposeMap(const M1 &m1, const M2 &m2) : _m1(m1), _m2(m2) {}

    /// \e
    typename MapTraits<M1>::ConstReturnValue
    operator[](const Key &k) const { return _m1[_m2[k]]; }
  };

  /// Returns a \ref ComposeMap class

  /// This function just returns a \ref ComposeMap class.
  ///
  /// If \c m1 and \c m2 are maps and the \c Value type of \c m2 is
  /// convertible to the \c Key of \c m1, then <tt>composeMap(m1,m2)[x]</tt>
  /// will be equal to <tt>m1[m2[x]]</tt>.
  ///
  /// \relates ComposeMap
  template <typename M1, typename M2>
  inline ComposeMap<M1, M2> composeMap(const M1 &m1, const M2 &m2) {
    return ComposeMap<M1, M2>(m1, m2);
  }


  /// Combination of two maps using an STL (binary) functor.

  /// This \ref concepts::ReadMap "read-only map" takes two maps and a
  /// binary functor and returns the combination of the two given maps
  /// using the functor.
  /// That is to say, if \c m1 is of type \c M1 and \c m2 is of \c M2
  /// and \c f is of \c F, then for
  /// \code
  ///   CombineMap<M1,M2,F,V> cm(m1,m2,f);
  /// \endcode
  /// <tt>cm[x]</tt> will be equal to <tt>f(m1[x],m2[x])</tt>.
  ///
  /// The \c Key type of the map is inherited from \c M1 (\c M1::Key
  /// must be convertible to \c M2::Key) and the \c Value type is \c V.
  /// \c M2::Value and \c M1::Value must be convertible to the
  /// corresponding input parameter of \c F and the return type of \c F
  /// must be convertible to \c V.
  ///
  /// The simplest way of using this map is through the combineMap()
  /// function.
  ///
  /// \sa ComposeMap
  ///
  /// \todo Check the requirements.
  template<typename M1, typename M2, typename F,
           typename V = typename F::result_type>
  class CombineMap : public MapBase<typename M1::Key, V> {
    const M1 &_m1;
    const M2 &_m2;
    F _f;
  public:
    typedef MapBase<typename M1::Key, V> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    CombineMap(const M1 &m1, const M2 &m2, const F &f = F())
      : _m1(m1), _m2(m2), _f(f) {}
    /// \e
    Value operator[](const Key &k) const { return _f(_m1[k],_m2[k]); }
  };

  /// Returns a \ref CombineMap class

  /// This function just returns a \ref CombineMap class.
  ///
  /// For example, if \c m1 and \c m2 are both maps with \c double
  /// values, then
  /// \code
  ///   combineMap(m1,m2,std::plus<double>())
  /// \endcode
  /// is equivalent to
  /// \code
  ///   addMap(m1,m2)
  /// \endcode
  ///
  /// This function is specialized for adaptable binary function
  /// classes and C++ functions.
  ///
  /// \relates CombineMap
  template<typename M1, typename M2, typename F, typename V>
  inline CombineMap<M1, M2, F, V>
  combineMap(const M1 &m1, const M2 &m2, const F &f) {
    return CombineMap<M1, M2, F, V>(m1,m2,f);
  }

  template<typename M1, typename M2, typename F>
  inline CombineMap<M1, M2, F, typename F::result_type>
  combineMap(const M1 &m1, const M2 &m2, const F &f) {
    return combineMap<M1, M2, F, typename F::result_type>(m1,m2,f);
  }

  template<typename M1, typename M2, typename K1, typename K2, typename V>
  inline CombineMap<M1, M2, V (*)(K1, K2), V>
  combineMap(const M1 &m1, const M2 &m2, V (*f)(K1, K2)) {
    return combineMap<M1, M2, V (*)(K1, K2), V>(m1,m2,f);
  }


  /// Converts an STL style (unary) functor to a map

  /// This \ref concepts::ReadMap "read-only map" returns the value
  /// of a given functor. Actually, it just wraps the functor and
  /// provides the \c Key and \c Value typedefs.
  ///
  /// Template parameters \c K and \c V will become its \c Key and
  /// \c Value. In most cases they have to be given explicitly because
  /// a functor typically does not provide \c argument_type and
  /// \c result_type typedefs.
  /// Parameter \c F is the type of the used functor.
  ///
  /// The simplest way of using this map is through the functorToMap()
  /// function.
  ///
  /// \sa MapToFunctor
  template<typename F,
           typename K = typename F::argument_type,
           typename V = typename F::result_type>
  class FunctorToMap : public MapBase<K, V> {
    F _f;
  public:
    typedef MapBase<K, V> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    FunctorToMap(const F &f = F()) : _f(f) {}
    /// \e
    Value operator[](const Key &k) const { return _f(k); }
  };

  /// Returns a \ref FunctorToMap class

  /// This function just returns a \ref FunctorToMap class.
  ///
  /// This function is specialized for adaptable binary function
  /// classes and C++ functions.
  ///
  /// \relates FunctorToMap
  template<typename K, typename V, typename F>
  inline FunctorToMap<F, K, V> functorToMap(const F &f) {
    return FunctorToMap<F, K, V>(f);
  }

  template <typename F>
  inline FunctorToMap<F, typename F::argument_type, typename F::result_type>
    functorToMap(const F &f)
  {
    return FunctorToMap<F, typename F::argument_type,
      typename F::result_type>(f);
  }

  template <typename K, typename V>
  inline FunctorToMap<V (*)(K), K, V> functorToMap(V (*f)(K)) {
    return FunctorToMap<V (*)(K), K, V>(f);
  }


  /// Converts a map to an STL style (unary) functor

  /// This class converts a map to an STL style (unary) functor.
  /// That is it provides an <tt>operator()</tt> to read its values.
  ///
  /// For the sake of convenience it also works as a usual
  /// \ref concepts::ReadMap "readable map", i.e. <tt>operator[]</tt>
  /// and the \c Key and \c Value typedefs also exist.
  ///
  /// The simplest way of using this map is through the mapToFunctor()
  /// function.
  ///
  ///\sa FunctorToMap
  template <typename M>
  class MapToFunctor : public MapBase<typename M::Key, typename M::Value> {
    const M &_m;
  public:
    typedef MapBase<typename M::Key, typename M::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    typedef typename Parent::Key argument_type;
    typedef typename Parent::Value result_type;

    /// Constructor
    MapToFunctor(const M &m) : _m(m) {}
    /// \e
    Value operator()(const Key &k) const { return _m[k]; }
    /// \e
    Value operator[](const Key &k) const { return _m[k]; }
  };

  /// Returns a \ref MapToFunctor class

  /// This function just returns a \ref MapToFunctor class.
  /// \relates MapToFunctor
  template<typename M>
  inline MapToFunctor<M> mapToFunctor(const M &m) {
    return MapToFunctor<M>(m);
  }


  /// \brief Map adaptor to convert the \c Value type of a map to
  /// another type using the default conversion.

  /// Map adaptor to convert the \c Value type of a \ref concepts::ReadMap
  /// "readable map" to another type using the default conversion.
  /// The \c Key type of it is inherited from \c M and the \c Value
  /// type is \c V.
  /// This type conforms the \ref concepts::ReadMap "ReadMap" concept.
  ///
  /// The simplest way of using this map is through the convertMap()
  /// function.
  template <typename M, typename V>
  class ConvertMap : public MapBase<typename M::Key, V> {
    const M &_m;
  public:
    typedef MapBase<typename M::Key, V> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor

    /// Constructor.
    /// \param m The underlying map.
    ConvertMap(const M &m) : _m(m) {}

    /// \e
    Value operator[](const Key &k) const { return _m[k]; }
  };

  /// Returns a \ref ConvertMap class

  /// This function just returns a \ref ConvertMap class.
  /// \relates ConvertMap
  template<typename V, typename M>
  inline ConvertMap<M, V> convertMap(const M &map) {
    return ConvertMap<M, V>(map);
  }


  /// Applies all map setting operations to two maps

  /// This map has two \ref concepts::WriteMap "writable map" parameters
  /// and each write request will be passed to both of them.
  /// If \c M1 is also \ref concepts::ReadMap "readable", then the read
  /// operations will return the corresponding values of \c M1.
  ///
  /// The \c Key and \c Value types are inherited from \c M1.
  /// The \c Key and \c Value of \c M2 must be convertible from those
  /// of \c M1.
  ///
  /// The simplest way of using this map is through the forkMap()
  /// function.
  template<typename  M1, typename M2>
  class ForkMap : public MapBase<typename M1::Key, typename M1::Value> {
    M1 &_m1;
    M2 &_m2;
  public:
    typedef MapBase<typename M1::Key, typename M1::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    ForkMap(M1 &m1, M2 &m2) : _m1(m1), _m2(m2) {}
    /// Returns the value associated with the given key in the first map.
    Value operator[](const Key &k) const { return _m1[k]; }
    /// Sets the value associated with the given key in both maps.
    void set(const Key &k, const Value &v) { _m1.set(k,v); _m2.set(k,v); }
  };

  /// Returns a \ref ForkMap class

  /// This function just returns a \ref ForkMap class.
  /// \relates ForkMap
  template <typename M1, typename M2>
  inline ForkMap<M1,M2> forkMap(M1 &m1, M2 &m2) {
    return ForkMap<M1,M2>(m1,m2);
  }


  /// Sum of two maps

  /// This \ref concepts::ReadMap "read-only map" returns the sum
  /// of the values of the two given maps.
  /// Its \c Key and \c Value types are inherited from \c M1.
  /// The \c Key and \c Value of \c M2 must be convertible to those of
  /// \c M1.
  ///
  /// If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
  /// \code
  ///   AddMap<M1,M2> am(m1,m2);
  /// \endcode
  /// <tt>am[x]</tt> will be equal to <tt>m1[x]+m2[x]</tt>.
  ///
  /// The simplest way of using this map is through the addMap()
  /// function.
  ///
  /// \sa SubMap, MulMap, DivMap
  /// \sa ShiftMap, ShiftWriteMap
  template<typename M1, typename M2>
  class AddMap : public MapBase<typename M1::Key, typename M1::Value> {
    const M1 &_m1;
    const M2 &_m2;
  public:
    typedef MapBase<typename M1::Key, typename M1::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    AddMap(const M1 &m1, const M2 &m2) : _m1(m1), _m2(m2) {}
    /// \e
    Value operator[](const Key &k) const { return _m1[k]+_m2[k]; }
  };

  /// Returns an \ref AddMap class

  /// This function just returns an \ref AddMap class.
  ///
  /// For example, if \c m1 and \c m2 are both maps with \c double
  /// values, then <tt>addMap(m1,m2)[x]</tt> will be equal to
  /// <tt>m1[x]+m2[x]</tt>.
  ///
  /// \relates AddMap
  template<typename M1, typename M2>
  inline AddMap<M1, M2> addMap(const M1 &m1, const M2 &m2) {
    return AddMap<M1, M2>(m1,m2);
  }


  /// Difference of two maps

  /// This \ref concepts::ReadMap "read-only map" returns the difference
  /// of the values of the two given maps.
  /// Its \c Key and \c Value types are inherited from \c M1.
  /// The \c Key and \c Value of \c M2 must be convertible to those of
  /// \c M1.
  ///
  /// If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
  /// \code
  ///   SubMap<M1,M2> sm(m1,m2);
  /// \endcode
  /// <tt>sm[x]</tt> will be equal to <tt>m1[x]-m2[x]</tt>.
  ///
  /// The simplest way of using this map is through the subMap()
  /// function.
  ///
  /// \sa AddMap, MulMap, DivMap
  template<typename M1, typename M2>
  class SubMap : public MapBase<typename M1::Key, typename M1::Value> {
    const M1 &_m1;
    const M2 &_m2;
  public:
    typedef MapBase<typename M1::Key, typename M1::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    SubMap(const M1 &m1, const M2 &m2) : _m1(m1), _m2(m2) {}
    /// \e
    Value operator[](const Key &k) const { return _m1[k]-_m2[k]; }
  };

  /// Returns a \ref SubMap class

  /// This function just returns a \ref SubMap class.
  ///
  /// For example, if \c m1 and \c m2 are both maps with \c double
  /// values, then <tt>subMap(m1,m2)[x]</tt> will be equal to
  /// <tt>m1[x]-m2[x]</tt>.
  ///
  /// \relates SubMap
  template<typename M1, typename M2>
  inline SubMap<M1, M2> subMap(const M1 &m1, const M2 &m2) {
    return SubMap<M1, M2>(m1,m2);
  }


  /// Product of two maps

  /// This \ref concepts::ReadMap "read-only map" returns the product
  /// of the values of the two given maps.
  /// Its \c Key and \c Value types are inherited from \c M1.
  /// The \c Key and \c Value of \c M2 must be convertible to those of
  /// \c M1.
  ///
  /// If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
  /// \code
  ///   MulMap<M1,M2> mm(m1,m2);
  /// \endcode
  /// <tt>mm[x]</tt> will be equal to <tt>m1[x]*m2[x]</tt>.
  ///
  /// The simplest way of using this map is through the mulMap()
  /// function.
  ///
  /// \sa AddMap, SubMap, DivMap
  /// \sa ScaleMap, ScaleWriteMap
  template<typename M1, typename M2>
  class MulMap : public MapBase<typename M1::Key, typename M1::Value> {
    const M1 &_m1;
    const M2 &_m2;
  public:
    typedef MapBase<typename M1::Key, typename M1::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    MulMap(const M1 &m1,const M2 &m2) : _m1(m1), _m2(m2) {}
    /// \e
    Value operator[](const Key &k) const { return _m1[k]*_m2[k]; }
  };

  /// Returns a \ref MulMap class

  /// This function just returns a \ref MulMap class.
  ///
  /// For example, if \c m1 and \c m2 are both maps with \c double
  /// values, then <tt>mulMap(m1,m2)[x]</tt> will be equal to
  /// <tt>m1[x]*m2[x]</tt>.
  ///
  /// \relates MulMap
  template<typename M1, typename M2>
  inline MulMap<M1, M2> mulMap(const M1 &m1,const M2 &m2) {
    return MulMap<M1, M2>(m1,m2);
  }


  /// Quotient of two maps

  /// This \ref concepts::ReadMap "read-only map" returns the quotient
  /// of the values of the two given maps.
  /// Its \c Key and \c Value types are inherited from \c M1.
  /// The \c Key and \c Value of \c M2 must be convertible to those of
  /// \c M1.
  ///
  /// If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
  /// \code
  ///   DivMap<M1,M2> dm(m1,m2);
  /// \endcode
  /// <tt>dm[x]</tt> will be equal to <tt>m1[x]/m2[x]</tt>.
  ///
  /// The simplest way of using this map is through the divMap()
  /// function.
  ///
  /// \sa AddMap, SubMap, MulMap
  template<typename M1, typename M2>
  class DivMap : public MapBase<typename M1::Key, typename M1::Value> {
    const M1 &_m1;
    const M2 &_m2;
  public:
    typedef MapBase<typename M1::Key, typename M1::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    DivMap(const M1 &m1,const M2 &m2) : _m1(m1), _m2(m2) {}
    /// \e
    Value operator[](const Key &k) const { return _m1[k]/_m2[k]; }
  };

  /// Returns a \ref DivMap class

  /// This function just returns a \ref DivMap class.
  ///
  /// For example, if \c m1 and \c m2 are both maps with \c double
  /// values, then <tt>divMap(m1,m2)[x]</tt> will be equal to
  /// <tt>m1[x]/m2[x]</tt>.
  ///
  /// \relates DivMap
  template<typename M1, typename M2>
  inline DivMap<M1, M2> divMap(const M1 &m1,const M2 &m2) {
    return DivMap<M1, M2>(m1,m2);
  }


  /// Shifts a map with a constant.

  /// This \ref concepts::ReadMap "read-only map" returns the sum of
  /// the given map and a constant value (i.e. it shifts the map with
  /// the constant). Its \c Key and \c Value are inherited from \c M.
  ///
  /// Actually,
  /// \code
  ///   ShiftMap<M> sh(m,v);
  /// \endcode
  /// is equivalent to
  /// \code
  ///   ConstMap<M::Key, M::Value> cm(v);
  ///   AddMap<M, ConstMap<M::Key, M::Value> > sh(m,cm);
  /// \endcode
  ///
  /// The simplest way of using this map is through the shiftMap()
  /// function.
  ///
  /// \sa ShiftWriteMap
  template<typename M, typename C = typename M::Value>
  class ShiftMap : public MapBase<typename M::Key, typename M::Value> {
    const M &_m;
    C _v;
  public:
    typedef MapBase<typename M::Key, typename M::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor

    /// Constructor.
    /// \param m The undelying map.
    /// \param v The constant value.
    ShiftMap(const M &m, const C &v) : _m(m), _v(v) {}
    /// \e
    Value operator[](const Key &k) const { return _m[k]+_v; }
  };

  /// Shifts a map with a constant (read-write version).

  /// This \ref concepts::ReadWriteMap "read-write map" returns the sum
  /// of the given map and a constant value (i.e. it shifts the map with
  /// the constant). Its \c Key and \c Value are inherited from \c M.
  /// It makes also possible to write the map.
  ///
  /// The simplest way of using this map is through the shiftWriteMap()
  /// function.
  ///
  /// \sa ShiftMap
  template<typename M, typename C = typename M::Value>
  class ShiftWriteMap : public MapBase<typename M::Key, typename M::Value> {
    M &_m;
    C _v;
  public:
    typedef MapBase<typename M::Key, typename M::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor

    /// Constructor.
    /// \param m The undelying map.
    /// \param v The constant value.
    ShiftWriteMap(M &m, const C &v) : _m(m), _v(v) {}
    /// \e
    Value operator[](const Key &k) const { return _m[k]+_v; }
    /// \e
    void set(const Key &k, const Value &v) { _m.set(k, v-_v); }
  };

  /// Returns a \ref ShiftMap class

  /// This function just returns a \ref ShiftMap class.
  ///
  /// For example, if \c m is a map with \c double values and \c v is
  /// \c double, then <tt>shiftMap(m,v)[x]</tt> will be equal to
  /// <tt>m[x]+v</tt>.
  ///
  /// \relates ShiftMap
  template<typename M, typename C>
  inline ShiftMap<M, C> shiftMap(const M &m, const C &v) {
    return ShiftMap<M, C>(m,v);
  }

  /// Returns a \ref ShiftWriteMap class

  /// This function just returns a \ref ShiftWriteMap class.
  ///
  /// For example, if \c m is a map with \c double values and \c v is
  /// \c double, then <tt>shiftWriteMap(m,v)[x]</tt> will be equal to
  /// <tt>m[x]+v</tt>.
  /// Moreover it makes also possible to write the map.
  ///
  /// \relates ShiftWriteMap
  template<typename M, typename C>
  inline ShiftWriteMap<M, C> shiftWriteMap(M &m, const C &v) {
    return ShiftWriteMap<M, C>(m,v);
  }


  /// Scales a map with a constant.

  /// This \ref concepts::ReadMap "read-only map" returns the value of
  /// the given map multiplied from the left side with a constant value.
  /// Its \c Key and \c Value are inherited from \c M.
  ///
  /// Actually,
  /// \code
  ///   ScaleMap<M> sc(m,v);
  /// \endcode
  /// is equivalent to
  /// \code
  ///   ConstMap<M::Key, M::Value> cm(v);
  ///   MulMap<ConstMap<M::Key, M::Value>, M> sc(cm,m);
  /// \endcode
  ///
  /// The simplest way of using this map is through the scaleMap()
  /// function.
  ///
  /// \sa ScaleWriteMap
  template<typename M, typename C = typename M::Value>
  class ScaleMap : public MapBase<typename M::Key, typename M::Value> {
    const M &_m;
    C _v;
  public:
    typedef MapBase<typename M::Key, typename M::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor

    /// Constructor.
    /// \param m The undelying map.
    /// \param v The constant value.
    ScaleMap(const M &m, const C &v) : _m(m), _v(v) {}
    /// \e
    Value operator[](const Key &k) const { return _v*_m[k]; }
  };

  /// Scales a map with a constant (read-write version).

  /// This \ref concepts::ReadWriteMap "read-write map" returns the value of
  /// the given map multiplied from the left side with a constant value.
  /// Its \c Key and \c Value are inherited from \c M.
  /// It can also be used as write map if the \c / operator is defined
  /// between \c Value and \c C and the given multiplier is not zero.
  ///
  /// The simplest way of using this map is through the scaleWriteMap()
  /// function.
  ///
  /// \sa ScaleMap
  template<typename M, typename C = typename M::Value>
  class ScaleWriteMap : public MapBase<typename M::Key, typename M::Value> {
    M &_m;
    C _v;
  public:
    typedef MapBase<typename M::Key, typename M::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor

    /// Constructor.
    /// \param m The undelying map.
    /// \param v The constant value.
    ScaleWriteMap(M &m, const C &v) : _m(m), _v(v) {}
    /// \e
    Value operator[](const Key &k) const { return _v*_m[k]; }
    /// \e
    void set(const Key &k, const Value &v) { _m.set(k, v/_v); }
  };

  /// Returns a \ref ScaleMap class

  /// This function just returns a \ref ScaleMap class.
  ///
  /// For example, if \c m is a map with \c double values and \c v is
  /// \c double, then <tt>scaleMap(m,v)[x]</tt> will be equal to
  /// <tt>v*m[x]</tt>.
  ///
  /// \relates ScaleMap
  template<typename M, typename C>
  inline ScaleMap<M, C> scaleMap(const M &m, const C &v) {
    return ScaleMap<M, C>(m,v);
  }

  /// Returns a \ref ScaleWriteMap class

  /// This function just returns a \ref ScaleWriteMap class.
  ///
  /// For example, if \c m is a map with \c double values and \c v is
  /// \c double, then <tt>scaleWriteMap(m,v)[x]</tt> will be equal to
  /// <tt>v*m[x]</tt>.
  /// Moreover it makes also possible to write the map.
  ///
  /// \relates ScaleWriteMap
  template<typename M, typename C>
  inline ScaleWriteMap<M, C> scaleWriteMap(M &m, const C &v) {
    return ScaleWriteMap<M, C>(m,v);
  }


  /// Negative of a map

  /// This \ref concepts::ReadMap "read-only map" returns the negative
  /// of the values of the given map (using the unary \c - operator).
  /// Its \c Key and \c Value are inherited from \c M.
  ///
  /// If M::Value is \c int, \c double etc., then
  /// \code
  ///   NegMap<M> neg(m);
  /// \endcode
  /// is equivalent to
  /// \code
  ///   ScaleMap<M> neg(m,-1);
  /// \endcode
  ///
  /// The simplest way of using this map is through the negMap()
  /// function.
  ///
  /// \sa NegWriteMap
  template<typename M>
  class NegMap : public MapBase<typename M::Key, typename M::Value> {
    const M& _m;
  public:
    typedef MapBase<typename M::Key, typename M::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    NegMap(const M &m) : _m(m) {}
    /// \e
    Value operator[](const Key &k) const { return -_m[k]; }
  };

  /// Negative of a map (read-write version)

  /// This \ref concepts::ReadWriteMap "read-write map" returns the
  /// negative of the values of the given map (using the unary \c -
  /// operator).
  /// Its \c Key and \c Value are inherited from \c M.
  /// It makes also possible to write the map.
  ///
  /// If M::Value is \c int, \c double etc., then
  /// \code
  ///   NegWriteMap<M> neg(m);
  /// \endcode
  /// is equivalent to
  /// \code
  ///   ScaleWriteMap<M> neg(m,-1);
  /// \endcode
  ///
  /// The simplest way of using this map is through the negWriteMap()
  /// function.
  ///
  /// \sa NegMap
  template<typename M>
  class NegWriteMap : public MapBase<typename M::Key, typename M::Value> {
    M &_m;
  public:
    typedef MapBase<typename M::Key, typename M::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    NegWriteMap(M &m) : _m(m) {}
    /// \e
    Value operator[](const Key &k) const { return -_m[k]; }
    /// \e
    void set(const Key &k, const Value &v) { _m.set(k, -v); }
  };

  /// Returns a \ref NegMap class

  /// This function just returns a \ref NegMap class.
  ///
  /// For example, if \c m is a map with \c double values, then
  /// <tt>negMap(m)[x]</tt> will be equal to <tt>-m[x]</tt>.
  ///
  /// \relates NegMap
  template <typename M>
  inline NegMap<M> negMap(const M &m) {
    return NegMap<M>(m);
  }

  /// Returns a \ref NegWriteMap class

  /// This function just returns a \ref NegWriteMap class.
  ///
  /// For example, if \c m is a map with \c double values, then
  /// <tt>negWriteMap(m)[x]</tt> will be equal to <tt>-m[x]</tt>.
  /// Moreover it makes also possible to write the map.
  ///
  /// \relates NegWriteMap
  template <typename M>
  inline NegWriteMap<M> negWriteMap(M &m) {
    return NegWriteMap<M>(m);
  }


  /// Absolute value of a map

  /// This \ref concepts::ReadMap "read-only map" returns the absolute
  /// value of the values of the given map.
  /// Its \c Key and \c Value are inherited from \c M.
  /// \c Value must be comparable to \c 0 and the unary \c -
  /// operator must be defined for it, of course.
  ///
  /// The simplest way of using this map is through the absMap()
  /// function.
  template<typename M>
  class AbsMap : public MapBase<typename M::Key, typename M::Value> {
    const M &_m;
  public:
    typedef MapBase<typename M::Key, typename M::Value> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    AbsMap(const M &m) : _m(m) {}
    /// \e
    Value operator[](const Key &k) const {
      Value tmp = _m[k];
      return tmp >= 0 ? tmp : -tmp;
    }

  };

  /// Returns an \ref AbsMap class

  /// This function just returns an \ref AbsMap class.
  ///
  /// For example, if \c m is a map with \c double values, then
  /// <tt>absMap(m)[x]</tt> will be equal to <tt>m[x]</tt> if
  /// it is positive or zero and <tt>-m[x]</tt> if <tt>m[x]</tt> is
  /// negative.
  ///
  /// \relates AbsMap
  template<typename M>
  inline AbsMap<M> absMap(const M &m) {
    return AbsMap<M>(m);
  }

  /// @}

  // Logical maps and map adaptors:

  /// \addtogroup maps
  /// @{

  /// Constant \c true map.

  /// This \ref concepts::ReadMap "read-only map" assigns \c true to
  /// each key.
  ///
  /// Note that
  /// \code
  ///   TrueMap<K> tm;
  /// \endcode
  /// is equivalent to
  /// \code
  ///   ConstMap<K,bool> tm(true);
  /// \endcode
  ///
  /// \sa FalseMap
  /// \sa ConstMap
  template <typename K>
  class TrueMap : public MapBase<K, bool> {
  public:
    typedef MapBase<K, bool> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Gives back \c true.
    Value operator[](const Key&) const { return true; }
  };

  /// Returns a \ref TrueMap class

  /// This function just returns a \ref TrueMap class.
  /// \relates TrueMap
  template<typename K>
  inline TrueMap<K> trueMap() {
    return TrueMap<K>();
  }


  /// Constant \c false map.

  /// This \ref concepts::ReadMap "read-only map" assigns \c false to
  /// each key.
  ///
  /// Note that
  /// \code
  ///   FalseMap<K> fm;
  /// \endcode
  /// is equivalent to
  /// \code
  ///   ConstMap<K,bool> fm(false);
  /// \endcode
  ///
  /// \sa TrueMap
  /// \sa ConstMap
  template <typename K>
  class FalseMap : public MapBase<K, bool> {
  public:
    typedef MapBase<K, bool> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Gives back \c false.
    Value operator[](const Key&) const { return false; }
  };

  /// Returns a \ref FalseMap class

  /// This function just returns a \ref FalseMap class.
  /// \relates FalseMap
  template<typename K>
  inline FalseMap<K> falseMap() {
    return FalseMap<K>();
  }

  /// @}

  /// \addtogroup map_adaptors
  /// @{

  /// Logical 'and' of two maps

  /// This \ref concepts::ReadMap "read-only map" returns the logical
  /// 'and' of the values of the two given maps.
  /// Its \c Key type is inherited from \c M1 and its \c Value type is
  /// \c bool. \c M2::Key must be convertible to \c M1::Key.
  ///
  /// If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
  /// \code
  ///   AndMap<M1,M2> am(m1,m2);
  /// \endcode
  /// <tt>am[x]</tt> will be equal to <tt>m1[x]&&m2[x]</tt>.
  ///
  /// The simplest way of using this map is through the andMap()
  /// function.
  ///
  /// \sa OrMap
  /// \sa NotMap, NotWriteMap
  template<typename M1, typename M2>
  class AndMap : public MapBase<typename M1::Key, bool> {
    const M1 &_m1;
    const M2 &_m2;
  public:
    typedef MapBase<typename M1::Key, bool> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    AndMap(const M1 &m1, const M2 &m2) : _m1(m1), _m2(m2) {}
    /// \e
    Value operator[](const Key &k) const { return _m1[k]&&_m2[k]; }
  };

  /// Returns an \ref AndMap class

  /// This function just returns an \ref AndMap class.
  ///
  /// For example, if \c m1 and \c m2 are both maps with \c bool values,
  /// then <tt>andMap(m1,m2)[x]</tt> will be equal to
  /// <tt>m1[x]&&m2[x]</tt>.
  ///
  /// \relates AndMap
  template<typename M1, typename M2>
  inline AndMap<M1, M2> andMap(const M1 &m1, const M2 &m2) {
    return AndMap<M1, M2>(m1,m2);
  }


  /// Logical 'or' of two maps

  /// This \ref concepts::ReadMap "read-only map" returns the logical
  /// 'or' of the values of the two given maps.
  /// Its \c Key type is inherited from \c M1 and its \c Value type is
  /// \c bool. \c M2::Key must be convertible to \c M1::Key.
  ///
  /// If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
  /// \code
  ///   OrMap<M1,M2> om(m1,m2);
  /// \endcode
  /// <tt>om[x]</tt> will be equal to <tt>m1[x]||m2[x]</tt>.
  ///
  /// The simplest way of using this map is through the orMap()
  /// function.
  ///
  /// \sa AndMap
  /// \sa NotMap, NotWriteMap
  template<typename M1, typename M2>
  class OrMap : public MapBase<typename M1::Key, bool> {
    const M1 &_m1;
    const M2 &_m2;
  public:
    typedef MapBase<typename M1::Key, bool> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    OrMap(const M1 &m1, const M2 &m2) : _m1(m1), _m2(m2) {}
    /// \e
    Value operator[](const Key &k) const { return _m1[k]||_m2[k]; }
  };

  /// Returns an \ref OrMap class

  /// This function just returns an \ref OrMap class.
  ///
  /// For example, if \c m1 and \c m2 are both maps with \c bool values,
  /// then <tt>orMap(m1,m2)[x]</tt> will be equal to
  /// <tt>m1[x]||m2[x]</tt>.
  ///
  /// \relates OrMap
  template<typename M1, typename M2>
  inline OrMap<M1, M2> orMap(const M1 &m1, const M2 &m2) {
    return OrMap<M1, M2>(m1,m2);
  }


  /// Logical 'not' of a map

  /// This \ref concepts::ReadMap "read-only map" returns the logical
  /// negation of the values of the given map.
  /// Its \c Key is inherited from \c M and its \c Value is \c bool.
  ///
  /// The simplest way of using this map is through the notMap()
  /// function.
  ///
  /// \sa NotWriteMap
  template <typename M>
  class NotMap : public MapBase<typename M::Key, bool> {
    const M &_m;
  public:
    typedef MapBase<typename M::Key, bool> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    NotMap(const M &m) : _m(m) {}
    /// \e
    Value operator[](const Key &k) const { return !_m[k]; }
  };

  /// Logical 'not' of a map (read-write version)

  /// This \ref concepts::ReadWriteMap "read-write map" returns the
  /// logical negation of the values of the given map.
  /// Its \c Key is inherited from \c M and its \c Value is \c bool.
  /// It makes also possible to write the map. When a value is set,
  /// the opposite value is set to the original map.
  ///
  /// The simplest way of using this map is through the notWriteMap()
  /// function.
  ///
  /// \sa NotMap
  template <typename M>
  class NotWriteMap : public MapBase<typename M::Key, bool> {
    M &_m;
  public:
    typedef MapBase<typename M::Key, bool> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    NotWriteMap(M &m) : _m(m) {}
    /// \e
    Value operator[](const Key &k) const { return !_m[k]; }
    /// \e
    void set(const Key &k, bool v) { _m.set(k, !v); }
  };

  /// Returns a \ref NotMap class

  /// This function just returns a \ref NotMap class.
  ///
  /// For example, if \c m is a map with \c bool values, then
  /// <tt>notMap(m)[x]</tt> will be equal to <tt>!m[x]</tt>.
  ///
  /// \relates NotMap
  template <typename M>
  inline NotMap<M> notMap(const M &m) {
    return NotMap<M>(m);
  }

  /// Returns a \ref NotWriteMap class

  /// This function just returns a \ref NotWriteMap class.
  ///
  /// For example, if \c m is a map with \c bool values, then
  /// <tt>notWriteMap(m)[x]</tt> will be equal to <tt>!m[x]</tt>.
  /// Moreover it makes also possible to write the map.
  ///
  /// \relates NotWriteMap
  template <typename M>
  inline NotWriteMap<M> notWriteMap(M &m) {
    return NotWriteMap<M>(m);
  }


  /// Combination of two maps using the \c == operator

  /// This \ref concepts::ReadMap "read-only map" assigns \c true to
  /// the keys for which the corresponding values of the two maps are
  /// equal.
  /// Its \c Key type is inherited from \c M1 and its \c Value type is
  /// \c bool. \c M2::Key must be convertible to \c M1::Key.
  ///
  /// If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
  /// \code
  ///   EqualMap<M1,M2> em(m1,m2);
  /// \endcode
  /// <tt>em[x]</tt> will be equal to <tt>m1[x]==m2[x]</tt>.
  ///
  /// The simplest way of using this map is through the equalMap()
  /// function.
  ///
  /// \sa LessMap
  template<typename M1, typename M2>
  class EqualMap : public MapBase<typename M1::Key, bool> {
    const M1 &_m1;
    const M2 &_m2;
  public:
    typedef MapBase<typename M1::Key, bool> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    EqualMap(const M1 &m1, const M2 &m2) : _m1(m1), _m2(m2) {}
    /// \e
    Value operator[](const Key &k) const { return _m1[k]==_m2[k]; }
  };

  /// Returns an \ref EqualMap class

  /// This function just returns an \ref EqualMap class.
  ///
  /// For example, if \c m1 and \c m2 are maps with keys and values of
  /// the same type, then <tt>equalMap(m1,m2)[x]</tt> will be equal to
  /// <tt>m1[x]==m2[x]</tt>.
  ///
  /// \relates EqualMap
  template<typename M1, typename M2>
  inline EqualMap<M1, M2> equalMap(const M1 &m1, const M2 &m2) {
    return EqualMap<M1, M2>(m1,m2);
  }


  /// Combination of two maps using the \c < operator

  /// This \ref concepts::ReadMap "read-only map" assigns \c true to
  /// the keys for which the corresponding value of the first map is
  /// less then the value of the second map.
  /// Its \c Key type is inherited from \c M1 and its \c Value type is
  /// \c bool. \c M2::Key must be convertible to \c M1::Key.
  ///
  /// If \c m1 is of type \c M1 and \c m2 is of \c M2, then for
  /// \code
  ///   LessMap<M1,M2> lm(m1,m2);
  /// \endcode
  /// <tt>lm[x]</tt> will be equal to <tt>m1[x]<m2[x]</tt>.
  ///
  /// The simplest way of using this map is through the lessMap()
  /// function.
  ///
  /// \sa EqualMap
  template<typename M1, typename M2>
  class LessMap : public MapBase<typename M1::Key, bool> {
    const M1 &_m1;
    const M2 &_m2;
  public:
    typedef MapBase<typename M1::Key, bool> Parent;
    typedef typename Parent::Key Key;
    typedef typename Parent::Value Value;

    /// Constructor
    LessMap(const M1 &m1, const M2 &m2) : _m1(m1), _m2(m2) {}
    /// \e
    Value operator[](const Key &k) const { return _m1[k]<_m2[k]; }
  };

  /// Returns an \ref LessMap class

  /// This function just returns an \ref LessMap class.
  ///
  /// For example, if \c m1 and \c m2 are maps with keys and values of
  /// the same type, then <tt>lessMap(m1,m2)[x]</tt> will be equal to
  /// <tt>m1[x]<m2[x]</tt>.
  ///
  /// \relates LessMap
  template<typename M1, typename M2>
  inline LessMap<M1, M2> lessMap(const M1 &m1, const M2 &m2) {
    return LessMap<M1, M2>(m1,m2);
  }

  namespace _maps_bits {

    template <typename _Iterator, typename Enable = void>
    struct IteratorTraits {
      typedef typename std::iterator_traits<_Iterator>::value_type Value;
    };

    template <typename _Iterator>
    struct IteratorTraits<_Iterator,
      typename exists<typename _Iterator::container_type>::type>
    {
      typedef typename _Iterator::container_type::value_type Value;
    };

  }

  /// \brief Writable bool map for logging each \c true assigned element
  ///
  /// A \ref concepts::WriteMap "writable" bool map for logging
  /// each \c true assigned element, i.e it copies subsequently each
  /// keys set to \c true to the given iterator.
  /// The most important usage of it is storing certain nodes or arcs
  /// that were marked \c true by an algorithm.
  ///
  /// There are several algorithms that provide solutions through bool
  /// maps and most of them assign \c true at most once for each key.
  /// In these cases it is a natural request to store each \c true
  /// assigned elements (in order of the assignment), which can be
  /// easily done with LoggerBoolMap.
  ///
  /// The simplest way of using this map is through the loggerBoolMap()
  /// function.
  ///
  /// \tparam It The type of the iterator.
  /// \tparam Ke The key type of the map. The default value set
  /// according to the iterator type should work in most cases.
  ///
  /// \note The container of the iterator must contain enough space
  /// for the elements or the iterator should be an inserter iterator.
#ifdef DOXYGEN
  template <typename It, typename Ke>
#else
  template <typename It,
            typename Ke=typename _maps_bits::IteratorTraits<It>::Value>
#endif
  class LoggerBoolMap {
  public:
    typedef It Iterator;

    typedef Ke Key;
    typedef bool Value;

    /// Constructor
    LoggerBoolMap(Iterator it)
      : _begin(it), _end(it) {}

    /// Gives back the given iterator set for the first key
    Iterator begin() const {
      return _begin;
    }

    /// Gives back the the 'after the last' iterator
    Iterator end() const {
      return _end;
    }

    /// The set function of the map
    void set(const Key& key, Value value) {
      if (value) {
        *_end++ = key;
      }
    }

  private:
    Iterator _begin;
    Iterator _end;
  };

  /// Returns a \ref LoggerBoolMap class

  /// This function just returns a \ref LoggerBoolMap class.
  ///
  /// The most important usage of it is storing certain nodes or arcs
  /// that were marked \c true by an algorithm.
  /// For example it makes easier to store the nodes in the processing
  /// order of Dfs algorithm, as the following examples show.
  /// \code
  ///   std::vector<Node> v;
  ///   dfs(g,s).processedMap(loggerBoolMap(std::back_inserter(v))).run();
  /// \endcode
  /// \code
  ///   std::vector<Node> v(countNodes(g));
  ///   dfs(g,s).processedMap(loggerBoolMap(v.begin())).run();
  /// \endcode
  ///
  /// \note The container of the iterator must contain enough space
  /// for the elements or the iterator should be an inserter iterator.
  ///
  /// \note LoggerBoolMap is just \ref concepts::WriteMap "writable", so
  /// it cannot be used when a readable map is needed, for example as
  /// \c ReachedMap for \ref Bfs, \ref Dfs and \ref Dijkstra algorithms.
  ///
  /// \relates LoggerBoolMap
  template<typename Iterator>
  inline LoggerBoolMap<Iterator> loggerBoolMap(Iterator it) {
    return LoggerBoolMap<Iterator>(it);
  }

  /// @}
}

#endif // LEMON_MAPS_H
