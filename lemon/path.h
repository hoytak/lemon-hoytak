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

#ifndef LEMON_PATH_H
#define LEMON_PATH_H

#include <vector>
#include <algorithm>

#include <lemon/path_utils.h>
#include <lemon/error.h>
#include <lemon/bits/invalid.h>

namespace lemon {

  /// \addtogroup paths
  /// @{


  /// \brief A structure for representing directed paths in a digraph.
  ///
  /// A structure for representing directed path in a digraph.
  /// \param Digraph The digraph type in which the path is.
  ///
  /// In a sense, the path can be treated as a list of arcs. The
  /// lemon path type stores just this list. As a consequence, it
  /// cannot enumerate the nodes of the path and the source node of
  /// a zero length path is undefined.
  ///
  /// This implementation is a back and front insertable and erasable
  /// path type. It can be indexed in O(1) time. The front and back
  /// insertion and erase is done in O(1) (amortized) time. The
  /// implementation uses two vectors for storing the front and back
  /// insertions.
  template <typename _Digraph>
  class Path {
  public:

    typedef _Digraph Digraph;
    typedef typename Digraph::Arc Arc;

    /// \brief Default constructor
    ///
    /// Default constructor
    Path() {}

    /// \brief Template copy constructor
    ///
    /// This constuctor initializes the path from any other path type.
    /// It simply makes a copy of the given path.
    template <typename CPath>
    Path(const CPath& cpath) {
      copyPath(*this, cpath);
    }

    /// \brief Template copy assignment
    ///
    /// This operator makes a copy of a path of any other type.
    template <typename CPath>
    Path& operator=(const CPath& cpath) {
      copyPath(*this, cpath);
      return *this;
    }

    /// \brief Lemon style iterator for path arcs
    ///
    /// This class is used to iterate on the arcs of the paths.
    class ArcIt {
      friend class Path;
    public:
      /// \brief Default constructor
      ArcIt() {}
      /// \brief Invalid constructor
      ArcIt(Invalid) : path(0), idx(-1) {}
      /// \brief Initializate the iterator to the first arc of path
      ArcIt(const Path &_path) 
        : path(&_path), idx(_path.empty() ? -1 : 0) {}

    private:

      ArcIt(const Path &_path, int _idx) 
        : path(&_path), idx(_idx) {}

    public:

      /// \brief Conversion to Arc
      operator const Arc&() const {
        return path->nth(idx);
      }

      /// \brief Next arc
      ArcIt& operator++() { 
        ++idx;
        if (idx >= path->length()) idx = -1; 
        return *this; 
      }

      /// \brief Comparison operator
      bool operator==(const ArcIt& e) const { return idx==e.idx; }
      /// \brief Comparison operator
      bool operator!=(const ArcIt& e) const { return idx!=e.idx; }
      /// \brief Comparison operator
      bool operator<(const ArcIt& e) const { return idx<e.idx; }

    private:
      const Path *path;
      int idx;
    };

    /// \brief Length of the path.
    int length() const { return head.size() + tail.size(); }
    /// \brief Return whether the path is empty.
    bool empty() const { return head.empty() && tail.empty(); }

    /// \brief Reset the path to an empty one.
    void clear() { head.clear(); tail.clear(); }

    /// \brief The nth arc.
    ///
    /// \pre n is in the [0..length() - 1] range
    const Arc& nth(int n) const {
      return n < int(head.size()) ? *(head.rbegin() + n) :
        *(tail.begin() + (n - head.size()));
    }

    /// \brief Initialize arc iterator to point to the nth arc
    ///
    /// \pre n is in the [0..length() - 1] range
    ArcIt nthIt(int n) const {
      return ArcIt(*this, n);
    }

    /// \brief The first arc of the path
    const Arc& front() const {
      return head.empty() ? tail.front() : head.back();
    }

    /// \brief Add a new arc before the current path
    void addFront(const Arc& arc) {
      head.push_back(arc);
    }

    /// \brief Erase the first arc of the path
    void eraseFront() {
      if (!head.empty()) {
        head.pop_back();
      } else {
        head.clear();
        int halfsize = tail.size() / 2;
        head.resize(halfsize);
        std::copy(tail.begin() + 1, tail.begin() + halfsize + 1,
                  head.rbegin());
        std::copy(tail.begin() + halfsize + 1, tail.end(), tail.begin());
        tail.resize(tail.size() - halfsize - 1);
      }
    }

    /// \brief The last arc of the path
    const Arc& back() const {
      return tail.empty() ? head.front() : tail.back();
    }

    /// \brief Add a new arc behind the current path
    void addBack(const Arc& arc) {
      tail.push_back(arc);
    }

    /// \brief Erase the last arc of the path
    void eraseBack() {
      if (!tail.empty()) {
        tail.pop_back();
      } else {
        int halfsize = head.size() / 2;
        tail.resize(halfsize);
        std::copy(head.begin() + 1, head.begin() + halfsize + 1,
                  tail.rbegin());
        std::copy(head.begin() + halfsize + 1, head.end(), head.begin());
        head.resize(head.size() - halfsize - 1);
      }
    }

    typedef True BuildTag;

    template <typename CPath>
    void build(const CPath& path) {
      int len = path.length();
      tail.reserve(len);
      for (typename CPath::ArcIt it(path); it != INVALID; ++it) {
        tail.push_back(it);
      }
    }

    template <typename CPath>
    void buildRev(const CPath& path) {
      int len = path.length();
      head.reserve(len);
      for (typename CPath::RevArcIt it(path); it != INVALID; ++it) {
        head.push_back(it);
      }
    }

  protected:
    typedef std::vector<Arc> Container;
    Container head, tail;

  };

  /// \brief A structure for representing directed paths in a digraph.
  ///
  /// A structure for representing directed path in a digraph.
  /// \param Digraph The digraph type in which the path is.
  ///
  /// In a sense, the path can be treated as a list of arcs. The
  /// lemon path type stores just this list. As a consequence it
  /// cannot enumerate the nodes in the path and the zero length paths
  /// cannot store the source.
  ///
  /// This implementation is a just back insertable and erasable path
  /// type. It can be indexed in O(1) time. The back insertion and
  /// erasure is amortized O(1) time. This implementation is faster
  /// then the \c Path type because it use just one vector for the
  /// arcs.
  template <typename _Digraph>
  class SimplePath {
  public:

    typedef _Digraph Digraph;
    typedef typename Digraph::Arc Arc;

    /// \brief Default constructor
    ///
    /// Default constructor
    SimplePath() {}

    /// \brief Template copy constructor
    ///
    /// This path can be initialized with any other path type. It just
    /// makes a copy of the given path.
    template <typename CPath>
    SimplePath(const CPath& cpath) {
      copyPath(*this, cpath);
    }

    /// \brief Template copy assignment
    ///
    /// This path can be initialized with any other path type. It just
    /// makes a copy of the given path.
    template <typename CPath>
    SimplePath& operator=(const CPath& cpath) {
      copyPath(*this, cpath);
      return *this;
    }

    /// \brief Iterator class to iterate on the arcs of the paths
    ///
    /// This class is used to iterate on the arcs of the paths
    ///
    /// Of course it converts to Digraph::Arc
    class ArcIt {
      friend class SimplePath;
    public:
      /// Default constructor
      ArcIt() {}
      /// Invalid constructor
      ArcIt(Invalid) : path(0), idx(-1) {}
      /// \brief Initializate the constructor to the first arc of path
      ArcIt(const SimplePath &_path) 
        : path(&_path), idx(_path.empty() ? -1 : 0) {}

    private:

      /// Constructor with starting point
      ArcIt(const SimplePath &_path, int _idx) 
        : idx(_idx), path(&_path) {}

    public:

      ///Conversion to Digraph::Arc
      operator const Arc&() const {
        return path->nth(idx);
      }

      /// Next arc
      ArcIt& operator++() { 
        ++idx;
        if (idx >= path->length()) idx = -1; 
        return *this; 
      }

      /// Comparison operator
      bool operator==(const ArcIt& e) const { return idx==e.idx; }
      /// Comparison operator
      bool operator!=(const ArcIt& e) const { return idx!=e.idx; }
      /// Comparison operator
      bool operator<(const ArcIt& e) const { return idx<e.idx; }

    private:
      const SimplePath *path;
      int idx;
    };

    /// \brief Length of the path.
    int length() const { return data.size(); }
    /// \brief Return true if the path is empty.
    bool empty() const { return data.empty(); }

    /// \brief Reset the path to an empty one.
    void clear() { data.clear(); }

    /// \brief The nth arc.
    ///
    /// \pre n is in the [0..length() - 1] range
    const Arc& nth(int n) const {
      return data[n];
    }

    /// \brief  Initializes arc iterator to point to the nth arc.
    ArcIt nthIt(int n) const {
      return ArcIt(*this, n);
    }

    /// \brief The first arc of the path.
    const Arc& front() const {
      return data.front();
    }

    /// \brief The last arc of the path.
    const Arc& back() const {
      return data.back();
    }

    /// \brief Add a new arc behind the current path.
    void addBack(const Arc& arc) {
      data.push_back(arc);
    }

    /// \brief Erase the last arc of the path
    void eraseBack() {
      data.pop_back();
    }

    typedef True BuildTag;

    template <typename CPath>
    void build(const CPath& path) {
      int len = path.length();
      data.resize(len);
      int index = 0;
      for (typename CPath::ArcIt it(path); it != INVALID; ++it) {
        data[index] = it;;
        ++index;
      }
    }

    template <typename CPath>
    void buildRev(const CPath& path) {
      int len = path.length();
      data.resize(len);
      int index = len;
      for (typename CPath::RevArcIt it(path); it != INVALID; ++it) {
        --index;
        data[index] = it;;
      }
    }

  protected:
    typedef std::vector<Arc> Container;
    Container data;

  };

  /// \brief A structure for representing directed paths in a digraph.
  ///
  /// A structure for representing directed path in a digraph.
  /// \param Digraph The digraph type in which the path is.
  ///
  /// In a sense, the path can be treated as a list of arcs. The
  /// lemon path type stores just this list. As a consequence it
  /// cannot enumerate the nodes in the path and the zero length paths
  /// cannot store the source.
  ///
  /// This implementation is a back and front insertable and erasable
  /// path type. It can be indexed in O(k) time, where k is the rank
  /// of the arc in the path. The length can be computed in O(n)
  /// time. The front and back insertion and erasure is O(1) time
  /// and it can be splited and spliced in O(1) time.
  template <typename _Digraph>
  class ListPath {
  public:

    typedef _Digraph Digraph;
    typedef typename Digraph::Arc Arc;

  protected:

    // the std::list<> is incompatible 
    // hard to create invalid iterator
    struct Node {
      Arc arc;
      Node *next, *prev;
    };

    Node *first, *last;

    std::allocator<Node> alloc;

  public:
 
    /// \brief Default constructor
    ///
    /// Default constructor
    ListPath() : first(0), last(0) {}

    /// \brief Template copy constructor
    ///
    /// This path can be initialized with any other path type. It just
    /// makes a copy of the given path.
    template <typename CPath>
    ListPath(const CPath& cpath) : first(0), last(0) {
      copyPath(*this, cpath);
    }

    /// \brief Destructor of the path
    ///
    /// Destructor of the path
    ~ListPath() {
      clear();
    }

    /// \brief Template copy assignment
    ///
    /// This path can be initialized with any other path type. It just
    /// makes a copy of the given path.
    template <typename CPath>
    ListPath& operator=(const CPath& cpath) {
      copyPath(*this, cpath);
      return *this;
    }

    /// \brief Iterator class to iterate on the arcs of the paths
    ///
    /// This class is used to iterate on the arcs of the paths
    ///
    /// Of course it converts to Digraph::Arc
    class ArcIt {
      friend class ListPath;
    public:
      /// Default constructor
      ArcIt() {}
      /// Invalid constructor
      ArcIt(Invalid) : path(0), node(0) {}
      /// \brief Initializate the constructor to the first arc of path
      ArcIt(const ListPath &_path) 
        : path(&_path), node(_path.first) {}

    protected:

      ArcIt(const ListPath &_path, Node *_node) 
        : path(&_path), node(_node) {}


    public:

      ///Conversion to Digraph::Arc
      operator const Arc&() const {
        return node->arc;
      }

      /// Next arc
      ArcIt& operator++() { 
        node = node->next;
        return *this; 
      }

      /// Comparison operator
      bool operator==(const ArcIt& e) const { return node==e.node; }
      /// Comparison operator
      bool operator!=(const ArcIt& e) const { return node!=e.node; }
      /// Comparison operator
      bool operator<(const ArcIt& e) const { return node<e.node; }

    private:
      const ListPath *path;
      Node *node;
    };

    /// \brief The nth arc.
    ///
    /// This function looks for the nth arc in O(n) time.
    /// \pre n is in the [0..length() - 1] range
    const Arc& nth(int n) const {
      Node *node = first;
      for (int i = 0; i < n; ++i) {
        node = node->next;
      }
      return node->arc;
    }

    /// \brief Initializes arc iterator to point to the nth arc.
    ArcIt nthIt(int n) const {
      Node *node = first;
      for (int i = 0; i < n; ++i) {
        node = node->next;
      }
      return ArcIt(*this, node);
    }

    /// \brief Length of the path.
    int length() const {
      int len = 0;
      Node *node = first;
      while (node != 0) {
        node = node->next;
        ++len;
      }
      return len;
    }

    /// \brief Return true if the path is empty.
    bool empty() const { return first == 0; }

    /// \brief Reset the path to an empty one.
    void clear() {
      while (first != 0) {
        last = first->next;
        alloc.destroy(first);
        alloc.deallocate(first, 1);
        first = last;
      }
    }

    /// \brief The first arc of the path
    const Arc& front() const {
      return first->arc;
    }

    /// \brief Add a new arc before the current path
    void addFront(const Arc& arc) {
      Node *node = alloc.allocate(1);
      alloc.construct(node, Node());
      node->prev = 0;
      node->next = first;
      node->arc = arc;
      if (first) {
        first->prev = node;
        first = node;
      } else {
        first = last = node;
      }
    }

    /// \brief Erase the first arc of the path
    void eraseFront() {
      Node *node = first;
      first = first->next;
      if (first) {
        first->prev = 0;
      } else {
        last = 0;
      }
      alloc.destroy(node);
      alloc.deallocate(node, 1);
    }

    /// \brief The last arc of the path.
    const Arc& back() const {
      return last->arc;
    }

    /// \brief Add a new arc behind the current path.
    void addBack(const Arc& arc) {
      Node *node = alloc.allocate(1);
      alloc.construct(node, Node());
      node->next = 0;
      node->prev = last;
      node->arc = arc;
      if (last) {
        last->next = node;
        last = node;
      } else {
        last = first = node;
      }
    }

    /// \brief Erase the last arc of the path
    void eraseBack() {
      Node *node = last;
      last = last->prev;
      if (last) {
        last->next = 0;
      } else {
        first = 0;
      }
      alloc.destroy(node);
      alloc.deallocate(node, 1);
    }

    /// \brief Splice a path to the back of the current path.
    ///
    /// It splices \c tpath to the back of the current path and \c
    /// tpath becomes empty. The time complexity of this function is
    /// O(1).
    void spliceBack(ListPath& tpath) {
      if (first) {
        if (tpath.first) {
          last->next = tpath.first;
          tpath.first->prev = last;
          last = tpath.last;
        }
      } else {
        first = tpath.first;
        last = tpath.last;
      }
      tpath.first = tpath.last = 0;
    }

    /// \brief Splice a path to the front of the current path.
    ///
    /// It splices \c tpath before the current path and \c tpath
    /// becomes empty. The time complexity of this function
    /// is O(1).
    void spliceFront(ListPath& tpath) {
      if (first) {
        if (tpath.first) {
          first->prev = tpath.last;
          tpath.last->next = first;
          first = tpath.first;
        }
      } else {
        first = tpath.first;
        last = tpath.last;
      }
      tpath.first = tpath.last = 0;
    }

    /// \brief Splice a path into the current path.
    ///
    /// It splices the \c tpath into the current path before the
    /// position of \c it iterator and \c tpath becomes empty. The
    /// time complexity of this function is O(1). If the \c it is
    /// \c INVALID then it will splice behind the current path.
    void splice(ArcIt it, ListPath& tpath) {
      if (it.node) {
        if (tpath.first) {
          tpath.first->prev = it.node->prev;
          if (it.node->prev) {
            it.node->prev->next = tpath.first;
          } else {
            first = tpath.first;
          }
          it.node->prev = tpath.last;
          tpath.last->next = it.node;
        }
      } else {
        if (first) {
          if (tpath.first) {
            last->next = tpath.first;
            tpath.first->prev = last;
            last = tpath.last;
          }
        } else {
          first = tpath.first;
          last = tpath.last;
        }
      }
      tpath.first = tpath.last = 0;
    }

    /// \brief Split the current path.
    ///
    /// It splits the current path into two parts. The part before
    /// the iterator \c it will remain in the current path and the part
    /// starting with
    /// \c it will put into \c tpath. If \c tpath have arcs
    /// before the operation they are removed first.  The time
    /// complexity of this function is O(1) plus the the time of emtying
    /// \c tpath. If \c it is \c INVALID then it just clears \c tpath
    void split(ArcIt it, ListPath& tpath) {
      tpath.clear();
      if (it.node) {
        tpath.first = it.node;
        tpath.last = last;
        if (it.node->prev) {
          last = it.node->prev;
          last->next = 0;
        } else {
          first = last = 0;
        }
        it.node->prev = 0;
      }
    }


    typedef True BuildTag;

    template <typename CPath>
    void build(const CPath& path) {
      for (typename CPath::ArcIt it(path); it != INVALID; ++it) {
        addBack(it);
      }
    }

    template <typename CPath>
    void buildRev(const CPath& path) {
      for (typename CPath::RevArcIt it(path); it != INVALID; ++it) {
        addFront(it);
      }
    }

  };

  /// \brief A structure for representing directed paths in a digraph.
  ///
  /// A structure for representing directed path in a digraph.
  /// \param Digraph The digraph type in which the path is.
  ///
  /// In a sense, the path can be treated as a list of arcs. The
  /// lemon path type stores just this list. As a consequence it
  /// cannot enumerate the nodes in the path and the source node of
  /// a zero length path is undefined.
  ///
  /// This implementation is completly static, i.e. it can be copy constucted
  /// or copy assigned from another path, but otherwise it cannot be
  /// modified.
  ///
  /// Being the the most memory efficient path type in LEMON,
  /// it is intented to be
  /// used when you want to store a large number of paths.
  template <typename _Digraph>
  class StaticPath {
  public:

    typedef _Digraph Digraph;
    typedef typename Digraph::Arc Arc;

    /// \brief Default constructor
    ///
    /// Default constructor
    StaticPath() : len(0), arcs(0) {}
    
    /// \brief Template copy constructor
    ///
    /// This path can be initialized from any other path type.
    template <typename CPath>
    StaticPath(const CPath& cpath) : arcs(0) {
      copyPath(*this, cpath);
    }

    /// \brief Destructor of the path
    ///
    /// Destructor of the path
    ~StaticPath() {
      if (arcs) delete[] arcs;
    }

    /// \brief Template copy assignment
    ///
    /// This path can be made equal to any other path type. It simply
    /// makes a copy of the given path.
    template <typename CPath>
    StaticPath& operator=(const CPath& cpath) {
      copyPath(*this, cpath);
      return *this;
    }

    /// \brief Iterator class to iterate on the arcs of the paths
    ///
    /// This class is used to iterate on the arcs of the paths
    ///
    /// Of course it converts to Digraph::Arc
    class ArcIt {
      friend class StaticPath;
    public:
      /// Default constructor
      ArcIt() {}
      /// Invalid constructor
      ArcIt(Invalid) : path(0), idx(-1) {}
      /// Initializate the constructor to the first arc of path
      ArcIt(const StaticPath &_path) 
        : path(&_path), idx(_path.empty() ? -1 : 0) {}

    private:

      /// Constructor with starting point
      ArcIt(const StaticPath &_path, int _idx) 
        : idx(_idx), path(&_path) {}

    public:

      ///Conversion to Digraph::Arc
      operator const Arc&() const {
        return path->nth(idx);
      }

      /// Next arc
      ArcIt& operator++() { 
        ++idx;
        if (idx >= path->length()) idx = -1; 
        return *this; 
      }

      /// Comparison operator
      bool operator==(const ArcIt& e) const { return idx==e.idx; }
      /// Comparison operator
      bool operator!=(const ArcIt& e) const { return idx!=e.idx; }
      /// Comparison operator
      bool operator<(const ArcIt& e) const { return idx<e.idx; }

    private:
      const StaticPath *path;
      int idx;
    };

    /// \brief The nth arc.
    ///
    /// \pre n is in the [0..length() - 1] range
    const Arc& nth(int n) const {
      return arcs[n];
    }

    /// \brief The arc iterator pointing to the nth arc.
    ArcIt nthIt(int n) const {
      return ArcIt(*this, n);
    }

    /// \brief The length of the path.
    int length() const { return len; }

    /// \brief Return true when the path is empty.
    int empty() const { return len == 0; }

    /// \break Erase all arcs in the digraph.
    void clear() {
      len = 0;
      if (arcs) delete[] arcs;
      arcs = 0;
    }

    /// \brief The first arc of the path.
    const Arc& front() const {
      return arcs[0];
    }

    /// \brief The last arc of the path.
    const Arc& back() const {
      return arcs[len - 1];
    }


    typedef True BuildTag;

    template <typename CPath>
    void build(const CPath& path) {
      len = path.length();
      arcs = new Arc[len];
      int index = 0;
      for (typename CPath::ArcIt it(path); it != INVALID; ++it) {
        arcs[index] = it;
        ++index;
      }
    }

    template <typename CPath>
    void buildRev(const CPath& path) {
      len = path.length();
      arcs = new Arc[len];
      int index = len;
      for (typename CPath::RevArcIt it(path); it != INVALID; ++it) {
        --index;
        arcs[index] = it;
      }
    }

  private:
    int len;
    Arc* arcs;
  };

  ///@}

} // namespace lemon

#endif // LEMON_PATH_H
