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

#ifndef LEMON_DFS_H
#define LEMON_DFS_H

///\ingroup search
///\file
///\brief DFS algorithm.

#include <lemon/list_graph.h>
#include <lemon/bits/path_dump.h>
#include <lemon/core.h>
#include <lemon/error.h>
#include <lemon/maps.h>
#include <lemon/path.h>

namespace lemon {

  ///Default traits class of Dfs class.

  ///Default traits class of Dfs class.
  ///\tparam GR Digraph type.
  template<class GR>
  struct DfsDefaultTraits
  {
    ///The type of the digraph the algorithm runs on.
    typedef GR Digraph;

    ///\brief The type of the map that stores the predecessor
    ///arcs of the %DFS paths.
    ///
    ///The type of the map that stores the predecessor
    ///arcs of the %DFS paths.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    typedef typename Digraph::template NodeMap<typename Digraph::Arc> PredMap;
    ///Instantiates a PredMap.

    ///This function instantiates a PredMap.
    ///\param g is the digraph, to which we would like to define the
    ///PredMap.
    static PredMap *createPredMap(const Digraph &g)
    {
      return new PredMap(g);
    }

    ///The type of the map that indicates which nodes are processed.

    ///The type of the map that indicates which nodes are processed.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    typedef NullMap<typename Digraph::Node,bool> ProcessedMap;
    ///Instantiates a ProcessedMap.

    ///This function instantiates a ProcessedMap.
    ///\param g is the digraph, to which
    ///we would like to define the ProcessedMap
#ifdef DOXYGEN
    static ProcessedMap *createProcessedMap(const Digraph &g)
#else
    static ProcessedMap *createProcessedMap(const Digraph &)
#endif
    {
      return new ProcessedMap();
    }

    ///The type of the map that indicates which nodes are reached.

    ///The type of the map that indicates which nodes are reached.
    ///It must meet the \ref concepts::ReadWriteMap "ReadWriteMap" concept.
    typedef typename Digraph::template NodeMap<bool> ReachedMap;
    ///Instantiates a ReachedMap.

    ///This function instantiates a ReachedMap.
    ///\param g is the digraph, to which
    ///we would like to define the ReachedMap.
    static ReachedMap *createReachedMap(const Digraph &g)
    {
      return new ReachedMap(g);
    }

    ///The type of the map that stores the distances of the nodes.

    ///The type of the map that stores the distances of the nodes.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    typedef typename Digraph::template NodeMap<int> DistMap;
    ///Instantiates a DistMap.

    ///This function instantiates a DistMap.
    ///\param g is the digraph, to which we would like to define the
    ///DistMap.
    static DistMap *createDistMap(const Digraph &g)
    {
      return new DistMap(g);
    }
  };

  ///%DFS algorithm class.

  ///\ingroup search
  ///This class provides an efficient implementation of the %DFS algorithm.
  ///
  ///There is also a \ref dfs() "function-type interface" for the DFS
  ///algorithm, which is convenient in the simplier cases and it can be
  ///used easier.
  ///
  ///\tparam GR The type of the digraph the algorithm runs on.
  ///The default value is \ref ListDigraph. The value of GR is not used
  ///directly by \ref Dfs, it is only passed to \ref DfsDefaultTraits.
  ///\tparam TR Traits class to set various data types used by the algorithm.
  ///The default traits class is
  ///\ref DfsDefaultTraits "DfsDefaultTraits<GR>".
  ///See \ref DfsDefaultTraits for the documentation of
  ///a Dfs traits class.
#ifdef DOXYGEN
  template <typename GR,
            typename TR>
#else
  template <typename GR=ListDigraph,
            typename TR=DfsDefaultTraits<GR> >
#endif
  class Dfs {
  public:

    ///The type of the digraph the algorithm runs on.
    typedef typename TR::Digraph Digraph;

    ///\brief The type of the map that stores the predecessor arcs of the
    ///DFS paths.
    typedef typename TR::PredMap PredMap;
    ///The type of the map that stores the distances of the nodes.
    typedef typename TR::DistMap DistMap;
    ///The type of the map that indicates which nodes are reached.
    typedef typename TR::ReachedMap ReachedMap;
    ///The type of the map that indicates which nodes are processed.
    typedef typename TR::ProcessedMap ProcessedMap;
    ///The type of the paths.
    typedef PredMapPath<Digraph, PredMap> Path;

    ///The traits class.
    typedef TR Traits;

  private:

    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::OutArcIt OutArcIt;

    //Pointer to the underlying digraph.
    const Digraph *G;
    //Pointer to the map of predecessor arcs.
    PredMap *_pred;
    //Indicates if _pred is locally allocated (true) or not.
    bool local_pred;
    //Pointer to the map of distances.
    DistMap *_dist;
    //Indicates if _dist is locally allocated (true) or not.
    bool local_dist;
    //Pointer to the map of reached status of the nodes.
    ReachedMap *_reached;
    //Indicates if _reached is locally allocated (true) or not.
    bool local_reached;
    //Pointer to the map of processed status of the nodes.
    ProcessedMap *_processed;
    //Indicates if _processed is locally allocated (true) or not.
    bool local_processed;

    std::vector<typename Digraph::OutArcIt> _stack;
    int _stack_head;

    //Creates the maps if necessary.
    void create_maps()
    {
      if(!_pred) {
        local_pred = true;
        _pred = Traits::createPredMap(*G);
      }
      if(!_dist) {
        local_dist = true;
        _dist = Traits::createDistMap(*G);
      }
      if(!_reached) {
        local_reached = true;
        _reached = Traits::createReachedMap(*G);
      }
      if(!_processed) {
        local_processed = true;
        _processed = Traits::createProcessedMap(*G);
      }
    }

  protected:

    Dfs() {}

  public:

    typedef Dfs Create;

    ///\name Named template parameters

    ///@{

    template <class T>
    struct SetPredMapTraits : public Traits {
      typedef T PredMap;
      static PredMap *createPredMap(const Digraph &)
      {
        LEMON_ASSERT(false, "PredMap is not initialized");
        return 0; // ignore warnings
      }
    };
    ///\brief \ref named-templ-param "Named parameter" for setting
    ///PredMap type.
    ///
    ///\ref named-templ-param "Named parameter" for setting
    ///PredMap type.
    template <class T>
    struct SetPredMap : public Dfs<Digraph, SetPredMapTraits<T> > {
      typedef Dfs<Digraph, SetPredMapTraits<T> > Create;
    };

    template <class T>
    struct SetDistMapTraits : public Traits {
      typedef T DistMap;
      static DistMap *createDistMap(const Digraph &)
      {
        LEMON_ASSERT(false, "DistMap is not initialized");
        return 0; // ignore warnings
      }
    };
    ///\brief \ref named-templ-param "Named parameter" for setting
    ///DistMap type.
    ///
    ///\ref named-templ-param "Named parameter" for setting
    ///DistMap type.
    template <class T>
    struct SetDistMap : public Dfs< Digraph, SetDistMapTraits<T> > {
      typedef Dfs<Digraph, SetDistMapTraits<T> > Create;
    };

    template <class T>
    struct SetReachedMapTraits : public Traits {
      typedef T ReachedMap;
      static ReachedMap *createReachedMap(const Digraph &)
      {
        LEMON_ASSERT(false, "ReachedMap is not initialized");
        return 0; // ignore warnings
      }
    };
    ///\brief \ref named-templ-param "Named parameter" for setting
    ///ReachedMap type.
    ///
    ///\ref named-templ-param "Named parameter" for setting
    ///ReachedMap type.
    template <class T>
    struct SetReachedMap : public Dfs< Digraph, SetReachedMapTraits<T> > {
      typedef Dfs< Digraph, SetReachedMapTraits<T> > Create;
    };

    template <class T>
    struct SetProcessedMapTraits : public Traits {
      typedef T ProcessedMap;
      static ProcessedMap *createProcessedMap(const Digraph &)
      {
        LEMON_ASSERT(false, "ProcessedMap is not initialized");
        return 0; // ignore warnings
      }
    };
    ///\brief \ref named-templ-param "Named parameter" for setting
    ///ProcessedMap type.
    ///
    ///\ref named-templ-param "Named parameter" for setting
    ///ProcessedMap type.
    template <class T>
    struct SetProcessedMap : public Dfs< Digraph, SetProcessedMapTraits<T> > {
      typedef Dfs< Digraph, SetProcessedMapTraits<T> > Create;
    };

    struct SetStandardProcessedMapTraits : public Traits {
      typedef typename Digraph::template NodeMap<bool> ProcessedMap;
      static ProcessedMap *createProcessedMap(const Digraph &g)
      {
        return new ProcessedMap(g);
      }
    };
    ///\brief \ref named-templ-param "Named parameter" for setting
    ///ProcessedMap type to be <tt>Digraph::NodeMap<bool></tt>.
    ///
    ///\ref named-templ-param "Named parameter" for setting
    ///ProcessedMap type to be <tt>Digraph::NodeMap<bool></tt>.
    ///If you don't set it explicitly, it will be automatically allocated.
    struct SetStandardProcessedMap :
      public Dfs< Digraph, SetStandardProcessedMapTraits > {
      typedef Dfs< Digraph, SetStandardProcessedMapTraits > Create;
    };

    ///@}

  public:

    ///Constructor.

    ///Constructor.
    ///\param g The digraph the algorithm runs on.
    Dfs(const Digraph &g) :
      G(&g),
      _pred(NULL), local_pred(false),
      _dist(NULL), local_dist(false),
      _reached(NULL), local_reached(false),
      _processed(NULL), local_processed(false)
    { }

    ///Destructor.
    ~Dfs()
    {
      if(local_pred) delete _pred;
      if(local_dist) delete _dist;
      if(local_reached) delete _reached;
      if(local_processed) delete _processed;
    }

    ///Sets the map that stores the predecessor arcs.

    ///Sets the map that stores the predecessor arcs.
    ///If you don't use this function before calling \ref run(),
    ///it will allocate one. The destructor deallocates this
    ///automatically allocated map, of course.
    ///\return <tt> (*this) </tt>
    Dfs &predMap(PredMap &m)
    {
      if(local_pred) {
        delete _pred;
        local_pred=false;
      }
      _pred = &m;
      return *this;
    }

    ///Sets the map that indicates which nodes are reached.

    ///Sets the map that indicates which nodes are reached.
    ///If you don't use this function before calling \ref run(),
    ///it will allocate one. The destructor deallocates this
    ///automatically allocated map, of course.
    ///\return <tt> (*this) </tt>
    Dfs &reachedMap(ReachedMap &m)
    {
      if(local_reached) {
        delete _reached;
        local_reached=false;
      }
      _reached = &m;
      return *this;
    }

    ///Sets the map that indicates which nodes are processed.

    ///Sets the map that indicates which nodes are processed.
    ///If you don't use this function before calling \ref run(),
    ///it will allocate one. The destructor deallocates this
    ///automatically allocated map, of course.
    ///\return <tt> (*this) </tt>
    Dfs &processedMap(ProcessedMap &m)
    {
      if(local_processed) {
        delete _processed;
        local_processed=false;
      }
      _processed = &m;
      return *this;
    }

    ///Sets the map that stores the distances of the nodes.

    ///Sets the map that stores the distances of the nodes calculated by
    ///the algorithm.
    ///If you don't use this function before calling \ref run(),
    ///it will allocate one. The destructor deallocates this
    ///automatically allocated map, of course.
    ///\return <tt> (*this) </tt>
    Dfs &distMap(DistMap &m)
    {
      if(local_dist) {
        delete _dist;
        local_dist=false;
      }
      _dist = &m;
      return *this;
    }

  public:

    ///\name Execution control
    ///The simplest way to execute the algorithm is to use
    ///one of the member functions called \ref lemon::Dfs::run() "run()".
    ///\n
    ///If you need more control on the execution, first you must call
    ///\ref lemon::Dfs::init() "init()", then you can add a source node
    ///with \ref lemon::Dfs::addSource() "addSource()".
    ///Finally \ref lemon::Dfs::start() "start()" will perform the
    ///actual path computation.

    ///@{

    ///Initializes the internal data structures.

    ///Initializes the internal data structures.
    ///
    void init()
    {
      create_maps();
      _stack.resize(countNodes(*G));
      _stack_head=-1;
      for ( NodeIt u(*G) ; u!=INVALID ; ++u ) {
        _pred->set(u,INVALID);
        _reached->set(u,false);
        _processed->set(u,false);
      }
    }

    ///Adds a new source node.

    ///Adds a new source node to the set of nodes to be processed.
    ///
    ///\pre The stack must be empty. (Otherwise the algorithm gives
    ///false results.)
    ///
    ///\warning Distances will be wrong (or at least strange) in case of
    ///multiple sources.
    void addSource(Node s)
    {
      LEMON_DEBUG(emptyQueue(), "The stack is not empty.");
      if(!(*_reached)[s])
        {
          _reached->set(s,true);
          _pred->set(s,INVALID);
          OutArcIt e(*G,s);
          if(e!=INVALID) {
            _stack[++_stack_head]=e;
            _dist->set(s,_stack_head);
          }
          else {
            _processed->set(s,true);
            _dist->set(s,0);
          }
        }
    }

    ///Processes the next arc.

    ///Processes the next arc.
    ///
    ///\return The processed arc.
    ///
    ///\pre The stack must not be empty.
    Arc processNextArc()
    {
      Node m;
      Arc e=_stack[_stack_head];
      if(!(*_reached)[m=G->target(e)]) {
        _pred->set(m,e);
        _reached->set(m,true);
        ++_stack_head;
        _stack[_stack_head] = OutArcIt(*G, m);
        _dist->set(m,_stack_head);
      }
      else {
        m=G->source(e);
        ++_stack[_stack_head];
      }
      while(_stack_head>=0 && _stack[_stack_head]==INVALID) {
        _processed->set(m,true);
        --_stack_head;
        if(_stack_head>=0) {
          m=G->source(_stack[_stack_head]);
          ++_stack[_stack_head];
        }
      }
      return e;
    }

    ///Next arc to be processed.

    ///Next arc to be processed.
    ///
    ///\return The next arc to be processed or \c INVALID if the stack
    ///is empty.
    OutArcIt nextArc() const
    {
      return _stack_head>=0?_stack[_stack_head]:INVALID;
    }

    ///\brief Returns \c false if there are nodes
    ///to be processed.
    ///
    ///Returns \c false if there are nodes
    ///to be processed in the queue (stack).
    bool emptyQueue() const { return _stack_head<0; }

    ///Returns the number of the nodes to be processed.

    ///Returns the number of the nodes to be processed in the queue (stack).
    int queueSize() const { return _stack_head+1; }

    ///Executes the algorithm.

    ///Executes the algorithm.
    ///
    ///This method runs the %DFS algorithm from the root node
    ///in order to compute the DFS path to each node.
    ///
    /// The algorithm computes
    ///- the %DFS tree,
    ///- the distance of each node from the root in the %DFS tree.
    ///
    ///\pre init() must be called and a root node should be
    ///added with addSource() before using this function.
    ///
    ///\note <tt>d.start()</tt> is just a shortcut of the following code.
    ///\code
    ///  while ( !d.emptyQueue() ) {
    ///    d.processNextArc();
    ///  }
    ///\endcode
    void start()
    {
      while ( !emptyQueue() ) processNextArc();
    }

    ///Executes the algorithm until the given target node is reached.

    ///Executes the algorithm until the given target node is reached.
    ///
    ///This method runs the %DFS algorithm from the root node
    ///in order to compute the DFS path to \c t.
    ///
    ///The algorithm computes
    ///- the %DFS path to \c t,
    ///- the distance of \c t from the root in the %DFS tree.
    ///
    ///\pre init() must be called and a root node should be
    ///added with addSource() before using this function.
    void start(Node t)
    {
      while ( !emptyQueue() && G->target(_stack[_stack_head])!=t )
        processNextArc();
    }

    ///Executes the algorithm until a condition is met.

    ///Executes the algorithm until a condition is met.
    ///
    ///This method runs the %DFS algorithm from the root node
    ///until an arc \c a with <tt>am[a]</tt> true is found.
    ///
    ///\param am A \c bool (or convertible) arc map. The algorithm
    ///will stop when it reaches an arc \c a with <tt>am[a]</tt> true.
    ///
    ///\return The reached arc \c a with <tt>am[a]</tt> true or
    ///\c INVALID if no such arc was found.
    ///
    ///\pre init() must be called and a root node should be
    ///added with addSource() before using this function.
    ///
    ///\warning Contrary to \ref Bfs and \ref Dijkstra, \c am is an arc map,
    ///not a node map.
    template<class ArcBoolMap>
    Arc start(const ArcBoolMap &am)
    {
      while ( !emptyQueue() && !am[_stack[_stack_head]] )
        processNextArc();
      return emptyQueue() ? INVALID : _stack[_stack_head];
    }

    ///Runs the algorithm from the given source node.

    ///This method runs the %DFS algorithm from node \c s
    ///in order to compute the DFS path to each node.
    ///
    ///The algorithm computes
    ///- the %DFS tree,
    ///- the distance of each node from the root in the %DFS tree.
    ///
    ///\note <tt>d.run(s)</tt> is just a shortcut of the following code.
    ///\code
    ///  d.init();
    ///  d.addSource(s);
    ///  d.start();
    ///\endcode
    void run(Node s) {
      init();
      addSource(s);
      start();
    }

    ///Finds the %DFS path between \c s and \c t.

    ///This method runs the %DFS algorithm from node \c s
    ///in order to compute the DFS path to node \c t
    ///(it stops searching when \c t is processed)
    ///
    ///\return \c true if \c t is reachable form \c s.
    ///
    ///\note Apart from the return value, <tt>d.run(s,t)</tt> is
    ///just a shortcut of the following code.
    ///\code
    ///  d.init();
    ///  d.addSource(s);
    ///  d.start(t);
    ///\endcode
    bool run(Node s,Node t) {
      init();
      addSource(s);
      start(t);
      return reached(t);
    }

    ///Runs the algorithm to visit all nodes in the digraph.

    ///This method runs the %DFS algorithm in order to compute the
    ///%DFS path to each node.
    ///
    ///The algorithm computes
    ///- the %DFS tree,
    ///- the distance of each node from the root in the %DFS tree.
    ///
    ///\note <tt>d.run()</tt> is just a shortcut of the following code.
    ///\code
    ///  d.init();
    ///  for (NodeIt n(digraph); n != INVALID; ++n) {
    ///    if (!d.reached(n)) {
    ///      d.addSource(n);
    ///      d.start();
    ///    }
    ///  }
    ///\endcode
    void run() {
      init();
      for (NodeIt it(*G); it != INVALID; ++it) {
        if (!reached(it)) {
          addSource(it);
          start();
        }
      }
    }

    ///@}

    ///\name Query Functions
    ///The result of the %DFS algorithm can be obtained using these
    ///functions.\n
    ///Either \ref lemon::Dfs::run() "run()" or \ref lemon::Dfs::start()
    ///"start()" must be called before using them.

    ///@{

    ///The DFS path to a node.

    ///Returns the DFS path to a node.
    ///
    ///\warning \c t should be reachable from the root.
    ///
    ///\pre Either \ref run() or \ref start() must be called before
    ///using this function.
    Path path(Node t) const { return Path(*G, *_pred, t); }

    ///The distance of a node from the root.

    ///Returns the distance of a node from the root.
    ///
    ///\warning If node \c v is not reachable from the root, then
    ///the return value of this function is undefined.
    ///
    ///\pre Either \ref run() or \ref start() must be called before
    ///using this function.
    int dist(Node v) const { return (*_dist)[v]; }

    ///Returns the 'previous arc' of the %DFS tree for a node.

    ///This function returns the 'previous arc' of the %DFS tree for the
    ///node \c v, i.e. it returns the last arc of a %DFS path from the
    ///root to \c v. It is \c INVALID
    ///if \c v is not reachable from the root(s) or if \c v is a root.
    ///
    ///The %DFS tree used here is equal to the %DFS tree used in
    ///\ref predNode().
    ///
    ///\pre Either \ref run() or \ref start() must be called before using
    ///this function.
    Arc predArc(Node v) const { return (*_pred)[v];}

    ///Returns the 'previous node' of the %DFS tree.

    ///This function returns the 'previous node' of the %DFS
    ///tree for the node \c v, i.e. it returns the last but one node
    ///from a %DFS path from the root to \c v. It is \c INVALID
    ///if \c v is not reachable from the root(s) or if \c v is a root.
    ///
    ///The %DFS tree used here is equal to the %DFS tree used in
    ///\ref predArc().
    ///
    ///\pre Either \ref run() or \ref start() must be called before
    ///using this function.
    Node predNode(Node v) const { return (*_pred)[v]==INVALID ? INVALID:
                                  G->source((*_pred)[v]); }

    ///\brief Returns a const reference to the node map that stores the
    ///distances of the nodes.
    ///
    ///Returns a const reference to the node map that stores the
    ///distances of the nodes calculated by the algorithm.
    ///
    ///\pre Either \ref run() or \ref init()
    ///must be called before using this function.
    const DistMap &distMap() const { return *_dist;}

    ///\brief Returns a const reference to the node map that stores the
    ///predecessor arcs.
    ///
    ///Returns a const reference to the node map that stores the predecessor
    ///arcs, which form the DFS tree.
    ///
    ///\pre Either \ref run() or \ref init()
    ///must be called before using this function.
    const PredMap &predMap() const { return *_pred;}

    ///Checks if a node is reachable from the root(s).

    ///Returns \c true if \c v is reachable from the root(s).
    ///\pre Either \ref run() or \ref start()
    ///must be called before using this function.
    bool reached(Node v) const { return (*_reached)[v]; }

    ///@}
  };

  ///Default traits class of dfs() function.

  ///Default traits class of dfs() function.
  ///\tparam GR Digraph type.
  template<class GR>
  struct DfsWizardDefaultTraits
  {
    ///The type of the digraph the algorithm runs on.
    typedef GR Digraph;

    ///\brief The type of the map that stores the predecessor
    ///arcs of the %DFS paths.
    ///
    ///The type of the map that stores the predecessor
    ///arcs of the %DFS paths.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    typedef typename Digraph::template NodeMap<typename Digraph::Arc> PredMap;
    ///Instantiates a PredMap.

    ///This function instantiates a PredMap.
    ///\param g is the digraph, to which we would like to define the
    ///PredMap.
    static PredMap *createPredMap(const Digraph &g)
    {
      return new PredMap(g);
    }

    ///The type of the map that indicates which nodes are processed.

    ///The type of the map that indicates which nodes are processed.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    ///By default it is a NullMap.
    typedef NullMap<typename Digraph::Node,bool> ProcessedMap;
    ///Instantiates a ProcessedMap.

    ///This function instantiates a ProcessedMap.
    ///\param g is the digraph, to which
    ///we would like to define the ProcessedMap.
#ifdef DOXYGEN
    static ProcessedMap *createProcessedMap(const Digraph &g)
#else
    static ProcessedMap *createProcessedMap(const Digraph &)
#endif
    {
      return new ProcessedMap();
    }

    ///The type of the map that indicates which nodes are reached.

    ///The type of the map that indicates which nodes are reached.
    ///It must meet the \ref concepts::ReadWriteMap "ReadWriteMap" concept.
    typedef typename Digraph::template NodeMap<bool> ReachedMap;
    ///Instantiates a ReachedMap.

    ///This function instantiates a ReachedMap.
    ///\param g is the digraph, to which
    ///we would like to define the ReachedMap.
    static ReachedMap *createReachedMap(const Digraph &g)
    {
      return new ReachedMap(g);
    }

    ///The type of the map that stores the distances of the nodes.

    ///The type of the map that stores the distances of the nodes.
    ///It must meet the \ref concepts::WriteMap "WriteMap" concept.
    typedef typename Digraph::template NodeMap<int> DistMap;
    ///Instantiates a DistMap.

    ///This function instantiates a DistMap.
    ///\param g is the digraph, to which we would like to define
    ///the DistMap
    static DistMap *createDistMap(const Digraph &g)
    {
      return new DistMap(g);
    }

    ///The type of the DFS paths.

    ///The type of the DFS paths.
    ///It must meet the \ref concepts::Path "Path" concept.
    typedef lemon::Path<Digraph> Path;
  };

  /// Default traits class used by DfsWizard

  /// To make it easier to use Dfs algorithm
  /// we have created a wizard class.
  /// This \ref DfsWizard class needs default traits,
  /// as well as the \ref Dfs class.
  /// The \ref DfsWizardBase is a class to be the default traits of the
  /// \ref DfsWizard class.
  template<class GR>
  class DfsWizardBase : public DfsWizardDefaultTraits<GR>
  {

    typedef DfsWizardDefaultTraits<GR> Base;
  protected:
    //The type of the nodes in the digraph.
    typedef typename Base::Digraph::Node Node;

    //Pointer to the digraph the algorithm runs on.
    void *_g;
    //Pointer to the map of reached nodes.
    void *_reached;
    //Pointer to the map of processed nodes.
    void *_processed;
    //Pointer to the map of predecessors arcs.
    void *_pred;
    //Pointer to the map of distances.
    void *_dist;
    //Pointer to the DFS path to the target node.
    void *_path;
    //Pointer to the distance of the target node.
    int *_di;

    public:
    /// Constructor.

    /// This constructor does not require parameters, therefore it initiates
    /// all of the attributes to \c 0.
    DfsWizardBase() : _g(0), _reached(0), _processed(0), _pred(0),
                      _dist(0), _path(0), _di(0) {}

    /// Constructor.

    /// This constructor requires one parameter,
    /// others are initiated to \c 0.
    /// \param g The digraph the algorithm runs on.
    DfsWizardBase(const GR &g) :
      _g(reinterpret_cast<void*>(const_cast<GR*>(&g))),
      _reached(0), _processed(0), _pred(0), _dist(0),  _path(0), _di(0) {}

  };

  /// Auxiliary class for the function-type interface of DFS algorithm.

  /// This auxiliary class is created to implement the
  /// \ref dfs() "function-type interface" of \ref Dfs algorithm.
  /// It does not have own \ref run() method, it uses the functions
  /// and features of the plain \ref Dfs.
  ///
  /// This class should only be used through the \ref dfs() function,
  /// which makes it easier to use the algorithm.
  template<class TR>
  class DfsWizard : public TR
  {
    typedef TR Base;

    ///The type of the digraph the algorithm runs on.
    typedef typename TR::Digraph Digraph;

    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::OutArcIt OutArcIt;

    ///\brief The type of the map that stores the predecessor
    ///arcs of the DFS paths.
    typedef typename TR::PredMap PredMap;
    ///\brief The type of the map that stores the distances of the nodes.
    typedef typename TR::DistMap DistMap;
    ///\brief The type of the map that indicates which nodes are reached.
    typedef typename TR::ReachedMap ReachedMap;
    ///\brief The type of the map that indicates which nodes are processed.
    typedef typename TR::ProcessedMap ProcessedMap;
    ///The type of the DFS paths
    typedef typename TR::Path Path;

  public:

    /// Constructor.
    DfsWizard() : TR() {}

    /// Constructor that requires parameters.

    /// Constructor that requires parameters.
    /// These parameters will be the default values for the traits class.
    /// \param g The digraph the algorithm runs on.
    DfsWizard(const Digraph &g) :
      TR(g) {}

    ///Copy constructor
    DfsWizard(const TR &b) : TR(b) {}

    ~DfsWizard() {}

    ///Runs DFS algorithm from the given source node.

    ///This method runs DFS algorithm from node \c s
    ///in order to compute the DFS path to each node.
    void run(Node s)
    {
      Dfs<Digraph,TR> alg(*reinterpret_cast<const Digraph*>(Base::_g));
      if (Base::_pred)
        alg.predMap(*reinterpret_cast<PredMap*>(Base::_pred));
      if (Base::_dist)
        alg.distMap(*reinterpret_cast<DistMap*>(Base::_dist));
      if (Base::_reached)
        alg.reachedMap(*reinterpret_cast<ReachedMap*>(Base::_reached));
      if (Base::_processed)
        alg.processedMap(*reinterpret_cast<ProcessedMap*>(Base::_processed));
      if (s!=INVALID)
        alg.run(s);
      else
        alg.run();
    }

    ///Finds the DFS path between \c s and \c t.

    ///This method runs DFS algorithm from node \c s
    ///in order to compute the DFS path to node \c t
    ///(it stops searching when \c t is processed).
    ///
    ///\return \c true if \c t is reachable form \c s.
    bool run(Node s, Node t)
    {
      Dfs<Digraph,TR> alg(*reinterpret_cast<const Digraph*>(Base::_g));
      if (Base::_pred)
        alg.predMap(*reinterpret_cast<PredMap*>(Base::_pred));
      if (Base::_dist)
        alg.distMap(*reinterpret_cast<DistMap*>(Base::_dist));
      if (Base::_reached)
        alg.reachedMap(*reinterpret_cast<ReachedMap*>(Base::_reached));
      if (Base::_processed)
        alg.processedMap(*reinterpret_cast<ProcessedMap*>(Base::_processed));
      alg.run(s,t);
      if (Base::_path)
        *reinterpret_cast<Path*>(Base::_path) = alg.path(t);
      if (Base::_di)
        *Base::_di = alg.dist(t);
      return alg.reached(t);
      }

    ///Runs DFS algorithm to visit all nodes in the digraph.

    ///This method runs DFS algorithm in order to compute
    ///the DFS path to each node.
    void run()
    {
      run(INVALID);
    }

    template<class T>
    struct SetPredMapBase : public Base {
      typedef T PredMap;
      static PredMap *createPredMap(const Digraph &) { return 0; };
      SetPredMapBase(const TR &b) : TR(b) {}
    };
    ///\brief \ref named-func-param "Named parameter"
    ///for setting PredMap object.
    ///
    ///\ref named-func-param "Named parameter"
    ///for setting PredMap object.
    template<class T>
    DfsWizard<SetPredMapBase<T> > predMap(const T &t)
    {
      Base::_pred=reinterpret_cast<void*>(const_cast<T*>(&t));
      return DfsWizard<SetPredMapBase<T> >(*this);
    }

    template<class T>
    struct SetReachedMapBase : public Base {
      typedef T ReachedMap;
      static ReachedMap *createReachedMap(const Digraph &) { return 0; };
      SetReachedMapBase(const TR &b) : TR(b) {}
    };
    ///\brief \ref named-func-param "Named parameter"
    ///for setting ReachedMap object.
    ///
    /// \ref named-func-param "Named parameter"
    ///for setting ReachedMap object.
    template<class T>
    DfsWizard<SetReachedMapBase<T> > reachedMap(const T &t)
    {
      Base::_reached=reinterpret_cast<void*>(const_cast<T*>(&t));
      return DfsWizard<SetReachedMapBase<T> >(*this);
    }

    template<class T>
    struct SetDistMapBase : public Base {
      typedef T DistMap;
      static DistMap *createDistMap(const Digraph &) { return 0; };
      SetDistMapBase(const TR &b) : TR(b) {}
    };
    ///\brief \ref named-func-param "Named parameter"
    ///for setting DistMap object.
    ///
    /// \ref named-func-param "Named parameter"
    ///for setting DistMap object.
    template<class T>
    DfsWizard<SetDistMapBase<T> > distMap(const T &t)
    {
      Base::_dist=reinterpret_cast<void*>(const_cast<T*>(&t));
      return DfsWizard<SetDistMapBase<T> >(*this);
    }

    template<class T>
    struct SetProcessedMapBase : public Base {
      typedef T ProcessedMap;
      static ProcessedMap *createProcessedMap(const Digraph &) { return 0; };
      SetProcessedMapBase(const TR &b) : TR(b) {}
    };
    ///\brief \ref named-func-param "Named parameter"
    ///for setting ProcessedMap object.
    ///
    /// \ref named-func-param "Named parameter"
    ///for setting ProcessedMap object.
    template<class T>
    DfsWizard<SetProcessedMapBase<T> > processedMap(const T &t)
    {
      Base::_processed=reinterpret_cast<void*>(const_cast<T*>(&t));
      return DfsWizard<SetProcessedMapBase<T> >(*this);
    }

    template<class T>
    struct SetPathBase : public Base {
      typedef T Path;
      SetPathBase(const TR &b) : TR(b) {}
    };
    ///\brief \ref named-func-param "Named parameter"
    ///for getting the DFS path to the target node.
    ///
    ///\ref named-func-param "Named parameter"
    ///for getting the DFS path to the target node.
    template<class T>
    DfsWizard<SetPathBase<T> > path(const T &t)
    {
      Base::_path=reinterpret_cast<void*>(const_cast<T*>(&t));
      return DfsWizard<SetPathBase<T> >(*this);
    }

    ///\brief \ref named-func-param "Named parameter"
    ///for getting the distance of the target node.
    ///
    ///\ref named-func-param "Named parameter"
    ///for getting the distance of the target node.
    DfsWizard dist(const int &d)
    {
      Base::_di=const_cast<int*>(&d);
      return *this;
    }

  };

  ///Function-type interface for DFS algorithm.

  ///\ingroup search
  ///Function-type interface for DFS algorithm.
  ///
  ///This function also has several \ref named-func-param "named parameters",
  ///they are declared as the members of class \ref DfsWizard.
  ///The following examples show how to use these parameters.
  ///\code
  ///  // Compute the DFS tree
  ///  dfs(g).predMap(preds).distMap(dists).run(s);
  ///
  ///  // Compute the DFS path from s to t
  ///  bool reached = dfs(g).path(p).dist(d).run(s,t);
  ///\endcode

  ///\warning Don't forget to put the \ref DfsWizard::run() "run()"
  ///to the end of the parameter list.
  ///\sa DfsWizard
  ///\sa Dfs
  template<class GR>
  DfsWizard<DfsWizardBase<GR> >
  dfs(const GR &digraph)
  {
    return DfsWizard<DfsWizardBase<GR> >(digraph);
  }

#ifdef DOXYGEN
  /// \brief Visitor class for DFS.
  ///
  /// This class defines the interface of the DfsVisit events, and
  /// it could be the base of a real visitor class.
  template <typename _Digraph>
  struct DfsVisitor {
    typedef _Digraph Digraph;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::Node Node;
    /// \brief Called for the source node of the DFS.
    ///
    /// This function is called for the source node of the DFS.
    void start(const Node& node) {}
    /// \brief Called when the source node is leaved.
    ///
    /// This function is called when the source node is leaved.
    void stop(const Node& node) {}
    /// \brief Called when a node is reached first time.
    ///
    /// This function is called when a node is reached first time.
    void reach(const Node& node) {}
    /// \brief Called when an arc reaches a new node.
    ///
    /// This function is called when the DFS finds an arc whose target node
    /// is not reached yet.
    void discover(const Arc& arc) {}
    /// \brief Called when an arc is examined but its target node is
    /// already discovered.
    ///
    /// This function is called when an arc is examined but its target node is
    /// already discovered.
    void examine(const Arc& arc) {}
    /// \brief Called when the DFS steps back from a node.
    ///
    /// This function is called when the DFS steps back from a node.
    void leave(const Node& node) {}
    /// \brief Called when the DFS steps back on an arc.
    ///
    /// This function is called when the DFS steps back on an arc.
    void backtrack(const Arc& arc) {}
  };
#else
  template <typename _Digraph>
  struct DfsVisitor {
    typedef _Digraph Digraph;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::Node Node;
    void start(const Node&) {}
    void stop(const Node&) {}
    void reach(const Node&) {}
    void discover(const Arc&) {}
    void examine(const Arc&) {}
    void leave(const Node&) {}
    void backtrack(const Arc&) {}

    template <typename _Visitor>
    struct Constraints {
      void constraints() {
        Arc arc;
        Node node;
        visitor.start(node);
        visitor.stop(arc);
        visitor.reach(node);
        visitor.discover(arc);
        visitor.examine(arc);
        visitor.leave(node);
        visitor.backtrack(arc);
      }
      _Visitor& visitor;
    };
  };
#endif

  /// \brief Default traits class of DfsVisit class.
  ///
  /// Default traits class of DfsVisit class.
  /// \tparam _Digraph The type of the digraph the algorithm runs on.
  template<class _Digraph>
  struct DfsVisitDefaultTraits {

    /// \brief The type of the digraph the algorithm runs on.
    typedef _Digraph Digraph;

    /// \brief The type of the map that indicates which nodes are reached.
    ///
    /// The type of the map that indicates which nodes are reached.
    /// It must meet the \ref concepts::ReadWriteMap "ReadWriteMap" concept.
    typedef typename Digraph::template NodeMap<bool> ReachedMap;

    /// \brief Instantiates a ReachedMap.
    ///
    /// This function instantiates a ReachedMap.
    /// \param digraph is the digraph, to which
    /// we would like to define the ReachedMap.
    static ReachedMap *createReachedMap(const Digraph &digraph) {
      return new ReachedMap(digraph);
    }

  };

  /// \ingroup search
  ///
  /// \brief %DFS algorithm class with visitor interface.
  ///
  /// This class provides an efficient implementation of the %DFS algorithm
  /// with visitor interface.
  ///
  /// The %DfsVisit class provides an alternative interface to the Dfs
  /// class. It works with callback mechanism, the DfsVisit object calls
  /// the member functions of the \c Visitor class on every DFS event.
  ///
  /// This interface of the DFS algorithm should be used in special cases
  /// when extra actions have to be performed in connection with certain
  /// events of the DFS algorithm. Otherwise consider to use Dfs or dfs()
  /// instead.
  ///
  /// \tparam _Digraph The type of the digraph the algorithm runs on.
  /// The default value is
  /// \ref ListDigraph. The value of _Digraph is not used directly by
  /// \ref DfsVisit, it is only passed to \ref DfsVisitDefaultTraits.
  /// \tparam _Visitor The Visitor type that is used by the algorithm.
  /// \ref DfsVisitor "DfsVisitor<_Digraph>" is an empty visitor, which
  /// does not observe the DFS events. If you want to observe the DFS
  /// events, you should implement your own visitor class.
  /// \tparam _Traits Traits class to set various data types used by the
  /// algorithm. The default traits class is
  /// \ref DfsVisitDefaultTraits "DfsVisitDefaultTraits<_Digraph>".
  /// See \ref DfsVisitDefaultTraits for the documentation of
  /// a DFS visit traits class.
#ifdef DOXYGEN
  template <typename _Digraph, typename _Visitor, typename _Traits>
#else
  template <typename _Digraph = ListDigraph,
            typename _Visitor = DfsVisitor<_Digraph>,
            typename _Traits = DfsVisitDefaultTraits<_Digraph> >
#endif
  class DfsVisit {
  public:

    ///The traits class.
    typedef _Traits Traits;

    ///The type of the digraph the algorithm runs on.
    typedef typename Traits::Digraph Digraph;

    ///The visitor type used by the algorithm.
    typedef _Visitor Visitor;

    ///The type of the map that indicates which nodes are reached.
    typedef typename Traits::ReachedMap ReachedMap;

  private:

    typedef typename Digraph::Node Node;
    typedef typename Digraph::NodeIt NodeIt;
    typedef typename Digraph::Arc Arc;
    typedef typename Digraph::OutArcIt OutArcIt;

    //Pointer to the underlying digraph.
    const Digraph *_digraph;
    //Pointer to the visitor object.
    Visitor *_visitor;
    //Pointer to the map of reached status of the nodes.
    ReachedMap *_reached;
    //Indicates if _reached is locally allocated (true) or not.
    bool local_reached;

    std::vector<typename Digraph::Arc> _stack;
    int _stack_head;

    //Creates the maps if necessary.
    void create_maps() {
      if(!_reached) {
        local_reached = true;
        _reached = Traits::createReachedMap(*_digraph);
      }
    }

  protected:

    DfsVisit() {}

  public:

    typedef DfsVisit Create;

    /// \name Named template parameters

    ///@{
    template <class T>
    struct SetReachedMapTraits : public Traits {
      typedef T ReachedMap;
      static ReachedMap *createReachedMap(const Digraph &digraph) {
        LEMON_ASSERT(false, "ReachedMap is not initialized");
        return 0; // ignore warnings
      }
    };
    /// \brief \ref named-templ-param "Named parameter" for setting
    /// ReachedMap type.
    ///
    /// \ref named-templ-param "Named parameter" for setting ReachedMap type.
    template <class T>
    struct SetReachedMap : public DfsVisit< Digraph, Visitor,
                                            SetReachedMapTraits<T> > {
      typedef DfsVisit< Digraph, Visitor, SetReachedMapTraits<T> > Create;
    };
    ///@}

  public:

    /// \brief Constructor.
    ///
    /// Constructor.
    ///
    /// \param digraph The digraph the algorithm runs on.
    /// \param visitor The visitor object of the algorithm.
    DfsVisit(const Digraph& digraph, Visitor& visitor)
      : _digraph(&digraph), _visitor(&visitor),
        _reached(0), local_reached(false) {}

    /// \brief Destructor.
    ~DfsVisit() {
      if(local_reached) delete _reached;
    }

    /// \brief Sets the map that indicates which nodes are reached.
    ///
    /// Sets the map that indicates which nodes are reached.
    /// If you don't use this function before calling \ref run(),
    /// it will allocate one. The destructor deallocates this
    /// automatically allocated map, of course.
    /// \return <tt> (*this) </tt>
    DfsVisit &reachedMap(ReachedMap &m) {
      if(local_reached) {
        delete _reached;
        local_reached=false;
      }
      _reached = &m;
      return *this;
    }

  public:

    /// \name Execution control
    /// The simplest way to execute the algorithm is to use
    /// one of the member functions called \ref lemon::DfsVisit::run()
    /// "run()".
    /// \n
    /// If you need more control on the execution, first you must call
    /// \ref lemon::DfsVisit::init() "init()", then you can add several
    /// source nodes with \ref lemon::DfsVisit::addSource() "addSource()".
    /// Finally \ref lemon::DfsVisit::start() "start()" will perform the
    /// actual path computation.

    /// @{

    /// \brief Initializes the internal data structures.
    ///
    /// Initializes the internal data structures.
    void init() {
      create_maps();
      _stack.resize(countNodes(*_digraph));
      _stack_head = -1;
      for (NodeIt u(*_digraph) ; u != INVALID ; ++u) {
        _reached->set(u, false);
      }
    }

    ///Adds a new source node.

    ///Adds a new source node to the set of nodes to be processed.
    ///
    ///\pre The stack must be empty. (Otherwise the algorithm gives
    ///false results.)
    ///
    ///\warning Distances will be wrong (or at least strange) in case of
    ///multiple sources.
    void addSource(Node s)
    {
      LEMON_DEBUG(emptyQueue(), "The stack is not empty.");
      if(!(*_reached)[s]) {
          _reached->set(s,true);
          _visitor->start(s);
          _visitor->reach(s);
          Arc e;
          _digraph->firstOut(e, s);
          if (e != INVALID) {
            _stack[++_stack_head] = e;
          } else {
            _visitor->leave(s);
          }
        }
    }

    /// \brief Processes the next arc.
    ///
    /// Processes the next arc.
    ///
    /// \return The processed arc.
    ///
    /// \pre The stack must not be empty.
    Arc processNextArc() {
      Arc e = _stack[_stack_head];
      Node m = _digraph->target(e);
      if(!(*_reached)[m]) {
        _visitor->discover(e);
        _visitor->reach(m);
        _reached->set(m, true);
        _digraph->firstOut(_stack[++_stack_head], m);
      } else {
        _visitor->examine(e);
        m = _digraph->source(e);
        _digraph->nextOut(_stack[_stack_head]);
      }
      while (_stack_head>=0 && _stack[_stack_head] == INVALID) {
        _visitor->leave(m);
        --_stack_head;
        if (_stack_head >= 0) {
          _visitor->backtrack(_stack[_stack_head]);
          m = _digraph->source(_stack[_stack_head]);
          _digraph->nextOut(_stack[_stack_head]);
        } else {
          _visitor->stop(m);
        }
      }
      return e;
    }

    /// \brief Next arc to be processed.
    ///
    /// Next arc to be processed.
    ///
    /// \return The next arc to be processed or INVALID if the stack is
    /// empty.
    Arc nextArc() const {
      return _stack_head >= 0 ? _stack[_stack_head] : INVALID;
    }

    /// \brief Returns \c false if there are nodes
    /// to be processed.
    ///
    /// Returns \c false if there are nodes
    /// to be processed in the queue (stack).
    bool emptyQueue() const { return _stack_head < 0; }

    /// \brief Returns the number of the nodes to be processed.
    ///
    /// Returns the number of the nodes to be processed in the queue (stack).
    int queueSize() const { return _stack_head + 1; }

    /// \brief Executes the algorithm.
    ///
    /// Executes the algorithm.
    ///
    /// This method runs the %DFS algorithm from the root node
    /// in order to compute the %DFS path to each node.
    ///
    /// The algorithm computes
    /// - the %DFS tree,
    /// - the distance of each node from the root in the %DFS tree.
    ///
    /// \pre init() must be called and a root node should be
    /// added with addSource() before using this function.
    ///
    /// \note <tt>d.start()</tt> is just a shortcut of the following code.
    /// \code
    ///   while ( !d.emptyQueue() ) {
    ///     d.processNextArc();
    ///   }
    /// \endcode
    void start() {
      while ( !emptyQueue() ) processNextArc();
    }

    /// \brief Executes the algorithm until the given target node is reached.
    ///
    /// Executes the algorithm until the given target node is reached.
    ///
    /// This method runs the %DFS algorithm from the root node
    /// in order to compute the DFS path to \c t.
    ///
    /// The algorithm computes
    /// - the %DFS path to \c t,
    /// - the distance of \c t from the root in the %DFS tree.
    ///
    /// \pre init() must be called and a root node should be added
    /// with addSource() before using this function.
    void start(Node t) {
      while ( !emptyQueue() && _digraph->target(_stack[_stack_head]) != t )
        processNextArc();
    }

    /// \brief Executes the algorithm until a condition is met.
    ///
    /// Executes the algorithm until a condition is met.
    ///
    /// This method runs the %DFS algorithm from the root node
    /// until an arc \c a with <tt>am[a]</tt> true is found.
    ///
    /// \param am A \c bool (or convertible) arc map. The algorithm
    /// will stop when it reaches an arc \c a with <tt>am[a]</tt> true.
    ///
    /// \return The reached arc \c a with <tt>am[a]</tt> true or
    /// \c INVALID if no such arc was found.
    ///
    /// \pre init() must be called and a root node should be added
    /// with addSource() before using this function.
    ///
    /// \warning Contrary to \ref Bfs and \ref Dijkstra, \c am is an arc map,
    /// not a node map.
    template <typename AM>
    Arc start(const AM &am) {
      while ( !emptyQueue() && !am[_stack[_stack_head]] )
        processNextArc();
      return emptyQueue() ? INVALID : _stack[_stack_head];
    }

    /// \brief Runs the algorithm from the given source node.
    ///
    /// This method runs the %DFS algorithm from node \c s.
    /// in order to compute the DFS path to each node.
    ///
    /// The algorithm computes
    /// - the %DFS tree,
    /// - the distance of each node from the root in the %DFS tree.
    ///
    /// \note <tt>d.run(s)</tt> is just a shortcut of the following code.
    ///\code
    ///   d.init();
    ///   d.addSource(s);
    ///   d.start();
    ///\endcode
    void run(Node s) {
      init();
      addSource(s);
      start();
    }

    /// \brief Finds the %DFS path between \c s and \c t.

    /// This method runs the %DFS algorithm from node \c s
    /// in order to compute the DFS path to node \c t
    /// (it stops searching when \c t is processed).
    ///
    /// \return \c true if \c t is reachable form \c s.
    ///
    /// \note Apart from the return value, <tt>d.run(s,t)</tt> is
    /// just a shortcut of the following code.
    ///\code
    ///   d.init();
    ///   d.addSource(s);
    ///   d.start(t);
    ///\endcode
    bool run(Node s,Node t) {
      init();
      addSource(s);
      start(t);
      return reached(t);
    }

    /// \brief Runs the algorithm to visit all nodes in the digraph.

    /// This method runs the %DFS algorithm in order to
    /// compute the %DFS path to each node.
    ///
    /// The algorithm computes
    /// - the %DFS tree,
    /// - the distance of each node from the root in the %DFS tree.
    ///
    /// \note <tt>d.run()</tt> is just a shortcut of the following code.
    ///\code
    ///   d.init();
    ///   for (NodeIt n(digraph); n != INVALID; ++n) {
    ///     if (!d.reached(n)) {
    ///       d.addSource(n);
    ///       d.start();
    ///     }
    ///   }
    ///\endcode
    void run() {
      init();
      for (NodeIt it(*_digraph); it != INVALID; ++it) {
        if (!reached(it)) {
          addSource(it);
          start();
        }
      }
    }

    ///@}

    /// \name Query Functions
    /// The result of the %DFS algorithm can be obtained using these
    /// functions.\n
    /// Either \ref lemon::DfsVisit::run() "run()" or
    /// \ref lemon::DfsVisit::start() "start()" must be called before
    /// using them.
    ///@{

    /// \brief Checks if a node is reachable from the root(s).
    ///
    /// Returns \c true if \c v is reachable from the root(s).
    /// \pre Either \ref run() or \ref start()
    /// must be called before using this function.
    bool reached(Node v) { return (*_reached)[v]; }

    ///@}

  };

} //END OF NAMESPACE LEMON

#endif
