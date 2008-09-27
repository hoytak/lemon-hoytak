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
///\brief Classes for representing paths in digraphs.
///

#ifndef LEMON_CONCEPT_PATH_H
#define LEMON_CONCEPT_PATH_H

#include <lemon/core.h>
#include <lemon/concept_check.h>

namespace lemon {
  namespace concepts {

    /// \addtogroup concept
    /// @{

    /// \brief A skeleton structure for representing directed paths in
    /// a digraph.
    ///
    /// A skeleton structure for representing directed paths in a
    /// digraph.
    /// \tparam _Digraph The digraph type in which the path is.
    ///
    /// In a sense, the path can be treated as a list of arcs. The
    /// lemon path type stores just this list. As a consequence it
    /// cannot enumerate the nodes in the path and the zero length
    /// paths cannot store the source.
    ///
    template <typename _Digraph>
    class Path {
    public:

      /// Type of the underlying digraph.
      typedef _Digraph Digraph;
      /// Arc type of the underlying digraph.
      typedef typename Digraph::Arc Arc;

      class ArcIt;

      /// \brief Default constructor
      Path() {}

      /// \brief Template constructor
      template <typename CPath>
      Path(const CPath& cpath) {}

      /// \brief Template assigment
      template <typename CPath>
      Path& operator=(const CPath& cpath) {
        ignore_unused_variable_warning(cpath);
        return *this;
      }

      /// Length of the path ie. the number of arcs in the path.
      int length() const { return 0;}

      /// Returns whether the path is empty.
      bool empty() const { return true;}

      /// Resets the path to an empty path.
      void clear() {}

      /// \brief LEMON style iterator for path arcs
      ///
      /// This class is used to iterate on the arcs of the paths.
      class ArcIt {
      public:
        /// Default constructor
        ArcIt() {}
        /// Invalid constructor
        ArcIt(Invalid) {}
        /// Constructor for first arc
        ArcIt(const Path &) {}

        /// Conversion to Arc
        operator Arc() const { return INVALID; }

        /// Next arc
        ArcIt& operator++() {return *this;}

        /// Comparison operator
        bool operator==(const ArcIt&) const {return true;}
        /// Comparison operator
        bool operator!=(const ArcIt&) const {return true;}
        /// Comparison operator
        bool operator<(const ArcIt&) const {return false;}

      };

      template <typename _Path>
      struct Constraints {
        void constraints() {
          Path<Digraph> pc;
          _Path p, pp(pc);
          int l = p.length();
          int e = p.empty();
          p.clear();

          p = pc;

          typename _Path::ArcIt id, ii(INVALID), i(p);

          ++i;
          typename Digraph::Arc ed = i;

          e = (i == ii);
          e = (i != ii);
          e = (i < ii);

          ignore_unused_variable_warning(l);
          ignore_unused_variable_warning(pp);
          ignore_unused_variable_warning(e);
          ignore_unused_variable_warning(id);
          ignore_unused_variable_warning(ii);
          ignore_unused_variable_warning(ed);
        }
      };

    };

    namespace _path_bits {

      template <typename _Digraph, typename _Path, typename RevPathTag = void>
      struct PathDumperConstraints {
        void constraints() {
          int l = p.length();
          int e = p.empty();

          typename _Path::ArcIt id, i(p);

          ++i;
          typename _Digraph::Arc ed = i;

          e = (i == INVALID);
          e = (i != INVALID);

          ignore_unused_variable_warning(l);
          ignore_unused_variable_warning(e);
          ignore_unused_variable_warning(id);
          ignore_unused_variable_warning(ed);
        }
        _Path& p;
      };

      template <typename _Digraph, typename _Path>
      struct PathDumperConstraints<
        _Digraph, _Path,
        typename enable_if<typename _Path::RevPathTag, void>::type
      > {
        void constraints() {
          int l = p.length();
          int e = p.empty();

          typename _Path::RevArcIt id, i(p);

          ++i;
          typename _Digraph::Arc ed = i;

          e = (i == INVALID);
          e = (i != INVALID);

          ignore_unused_variable_warning(l);
          ignore_unused_variable_warning(e);
          ignore_unused_variable_warning(id);
          ignore_unused_variable_warning(ed);
        }
        _Path& p;
      };

    }


    /// \brief A skeleton structure for path dumpers.
    ///
    /// A skeleton structure for path dumpers. The path dumpers are
    /// the generalization of the paths. The path dumpers can
    /// enumerate the arcs of the path wheter in forward or in
    /// backward order.  In most time these classes are not used
    /// directly rather it used to assign a dumped class to a real
    /// path type.
    ///
    /// The main purpose of this concept is that the shortest path
    /// algorithms can enumerate easily the arcs in reverse order.
    /// If we would like to give back a real path from these
    /// algorithms then we should create a temporarly path object. In
    /// LEMON such algorithms gives back a path dumper what can
    /// assigned to a real path and the dumpers can be implemented as
    /// an adaptor class to the predecessor map.

    /// \tparam _Digraph  The digraph type in which the path is.
    ///
    /// The paths can be constructed from any path type by a
    /// template constructor or a template assignment operator.
    ///
    template <typename _Digraph>
    class PathDumper {
    public:

      /// Type of the underlying digraph.
      typedef _Digraph Digraph;
      /// Arc type of the underlying digraph.
      typedef typename Digraph::Arc Arc;

      /// Length of the path ie. the number of arcs in the path.
      int length() const { return 0;}

      /// Returns whether the path is empty.
      bool empty() const { return true;}

      /// \brief Forward or reverse dumping
      ///
      /// If the RevPathTag is defined and true then reverse dumping
      /// is provided in the path dumper. In this case instead of the
      /// ArcIt the RevArcIt iterator should be implemented in the
      /// dumper.
      typedef False RevPathTag;

      /// \brief LEMON style iterator for path arcs
      ///
      /// This class is used to iterate on the arcs of the paths.
      class ArcIt {
      public:
        /// Default constructor
        ArcIt() {}
        /// Invalid constructor
        ArcIt(Invalid) {}
        /// Constructor for first arc
        ArcIt(const PathDumper&) {}

        /// Conversion to Arc
        operator Arc() const { return INVALID; }

        /// Next arc
        ArcIt& operator++() {return *this;}

        /// Comparison operator
        bool operator==(const ArcIt&) const {return true;}
        /// Comparison operator
        bool operator!=(const ArcIt&) const {return true;}
        /// Comparison operator
        bool operator<(const ArcIt&) const {return false;}

      };

      /// \brief LEMON style iterator for path arcs
      ///
      /// This class is used to iterate on the arcs of the paths in
      /// reverse direction.
      class RevArcIt {
      public:
        /// Default constructor
        RevArcIt() {}
        /// Invalid constructor
        RevArcIt(Invalid) {}
        /// Constructor for first arc
        RevArcIt(const PathDumper &) {}

        /// Conversion to Arc
        operator Arc() const { return INVALID; }

        /// Next arc
        RevArcIt& operator++() {return *this;}

        /// Comparison operator
        bool operator==(const RevArcIt&) const {return true;}
        /// Comparison operator
        bool operator!=(const RevArcIt&) const {return true;}
        /// Comparison operator
        bool operator<(const RevArcIt&) const {return false;}

      };

      template <typename _Path>
      struct Constraints {
        void constraints() {
          function_requires<_path_bits::
            PathDumperConstraints<Digraph, _Path> >();
        }
      };

    };


    ///@}
  }

} // namespace lemon

#endif // LEMON_CONCEPT_PATH_H
