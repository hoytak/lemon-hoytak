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

///\ingroup paths
///\file
///\brief Classes for representing paths in digraphs.
///

#ifndef LEMON_PATH_UTILS_H
#define LEMON_PATH_UTILS_H

#include <lemon/concepts/path.h>

namespace lemon {

  namespace _path_bits {

    template <typename Path, typename Enable = void>
    struct RevTagIndicator {
      static const bool value = false;
    };

    template <typename Digraph>
    struct RevTagIndicator<
      Digraph, 
      typename enable_if<typename Digraph::RevTag, void>::type
    > {
      static const bool value = true;
    };

    template <typename Target, typename Source,
              typename BuildEnable = void, typename RevEnable = void>
    struct PathCopySelector {
      static void copy(Target& target, const Source& source) {
        target.clear();
        for (typename Source::ArcIt it(source); it != INVALID; ++it) {
          target.addBack(it);
        }
      }
    };

    template <typename Target, typename Source, typename BuildEnable>
    struct PathCopySelector<
      Target, Source, BuildEnable, 
      typename enable_if<typename Source::RevPathTag, void>::type> {
      static void copy(Target& target, const Source& source) {
        target.clear();
        for (typename Source::RevArcIt it(source); it != INVALID; ++it) {
          target.addFront(it);
        }
      }
    };

    template <typename Target, typename Source, typename RevEnable>
    struct PathCopySelector<
      Target, Source, 
      typename enable_if<typename Target::BuildTag, void>::type, RevEnable> {
      static void copy(Target& target, const Source& source) {
        target.clear();
        target.build(source);
      }
    };

    template <typename Target, typename Source>
    struct PathCopySelector<
      Target, Source, 
      typename enable_if<typename Target::BuildTag, void>::type,
      typename enable_if<typename Source::RevPathTag, void>::type> {
      static void copy(Target& target, const Source& source) {
        target.clear();
        target.buildRev(source);
      }
    };

  }


  /// \brief Make a copy of a path.
  ///
  ///  This function makes a copy of a path.
  template <typename Target, typename Source>
  void copyPath(Target& target, const Source& source) {
    checkConcept<concepts::PathDumper<typename Source::Digraph>, Source>();
    _path_bits::PathCopySelector<Target, Source>::copy(target, source);
  }

  /// \brief Check the consistency of a path.
  ///
  /// This function checks that the target of each arc is the same
  /// as the source of the next one. 
  /// 
  template <typename Digraph, typename Path>
  bool checkPath(const Digraph& digraph, const Path& path) {
    typename Path::ArcIt it(path);
    if (it == INVALID) return true;
    typename Digraph::Node node = digraph.target(it);
    ++it;
    while (it != INVALID) {
      if (digraph.source(it) != node) return false;
      node = digraph.target(it);
      ++it;
    }
    return true;
  }

  /// \brief The source of a path
  ///
  /// This function returns the source of the given path.
  template <typename Digraph, typename Path>
  typename Digraph::Node pathSource(const Digraph& digraph, const Path& path) {
    return digraph.source(path.front());
  }

  /// \brief The target of a path
  ///
  /// This function returns the target of the given path.
  template <typename Digraph, typename Path>
  typename Digraph::Node pathTarget(const Digraph& digraph, const Path& path) {
    return digraph.target(path.back());
  }

  /// \brief Class which helps to iterate through the nodes of a path
  ///
  /// In a sense, the path can be treated as a list of arcs. The
  /// lemon path type stores only this list. As a consequence, it
  /// cannot enumerate the nodes in the path and the zero length paths
  /// cannot have a source node.
  ///
  /// This class implements the node iterator of a path structure. To
  /// provide this feature, the underlying digraph should be passed to
  /// the constructor of the iterator.
  template <typename Path>
  class PathNodeIt {
  private:
    const typename Path::Digraph *_digraph;
    typename Path::ArcIt _it;
    typename Path::Digraph::Node _nd;

  public:

    typedef typename Path::Digraph Digraph;
    typedef typename Digraph::Node Node;
    
    /// Default constructor
    PathNodeIt() {}
    /// Invalid constructor
    PathNodeIt(Invalid) 
      : _digraph(0), _it(INVALID), _nd(INVALID) {}
    /// Constructor
    PathNodeIt(const Digraph& digraph, const Path& path) 
      : _digraph(&digraph), _it(path) {
      _nd = (_it != INVALID ? _digraph->source(_it) : INVALID);
    }
    /// Constructor
    PathNodeIt(const Digraph& digraph, const Path& path, const Node& src) 
      : _digraph(&digraph), _it(path), _nd(src) {}

    ///Conversion to Digraph::Node
    operator Node() const {
      return _nd;
    }

    /// Next node
    PathNodeIt& operator++() {
      if (_it == INVALID) _nd = INVALID;
      else {
	_nd = _digraph->target(_it);
	++_it;
      }
      return *this;
    }

    /// Comparison operator
    bool operator==(const PathNodeIt& n) const { 
      return _it == n._it && _nd == n._nd; 
    }
    /// Comparison operator
    bool operator!=(const PathNodeIt& n) const { 
      return _it != n._it || _nd != n._nd; 
    }
    /// Comparison operator
    bool operator<(const PathNodeIt& n) const { 
      return (_it < n._it && _nd != INVALID);
    }
    
  };
  
}

#endif
