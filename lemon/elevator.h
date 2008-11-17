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

#ifndef LEMON_ELEVATOR_H
#define LEMON_ELEVATOR_H

///\ingroup auxdat
///\file
///\brief Elevator class
///
///Elevator class implements an efficient data structure
///for labeling items in push-relabel type algorithms.
///

#include <test/test_tools.h>
namespace lemon {

  ///Class for handling "labels" in push-relabel type algorithms.

  ///A class for handling "labels" in push-relabel type algorithms.
  ///
  ///\ingroup auxdat
  ///Using this class you can assign "labels" (nonnegative integer numbers)
  ///to the edges or nodes of a graph, manipulate and query them through
  ///operations typically arising in "push-relabel" type algorithms.
  ///
  ///Each item is either \em active or not, and you can also choose a
  ///highest level active item.
  ///
  ///\sa LinkedElevator
  ///
  ///\param Graph the underlying graph type
  ///\param Item Type of the items the data is assigned to (Graph::Node,
  ///Graph::Edge, Graph::UEdge)
  template<class Graph, class Item>
  class Elevator
  {
  public:

    typedef Item Key;
    typedef int Value;

  private:

    typedef typename std::vector<Item>::iterator Vit;
    typedef typename ItemSetTraits<Graph,Item>::template Map<Vit>::Type VitMap;
    typedef typename ItemSetTraits<Graph,Item>::template Map<int>::Type IntMap;

    const Graph &_g;
    int _max_level;
    int _item_num;
    VitMap _where;
    IntMap _level;
    std::vector<Item> _items;
    std::vector<Vit> _first;
    std::vector<Vit> _last_active;

    int _highest_active;

    void copy(Item i, Vit p)
    {
      _where[*p=i]=p;
    }
    void copy(Vit s, Vit p)
    {
      if(s!=p)
        {
          Item i=*s;
          *p=i;
          _where[i]=p;
        }
    }
    void swap(Vit i, Vit j)
    {
      Item ti=*i;
      Vit ct = _where[ti];
      _where[ti]=_where[*i=*j];
      _where[*j]=ct;
      *j=ti;
    }

  public:

    ///Constructor with given maximum level.

    ///Constructor with given maximum level.
    ///
    ///\param g The underlying graph
    ///\param max_level Set the range of the possible labels to
    ///[0...\c max_level]
    Elevator(const Graph &g,int max_level) :
      _g(g),
      _max_level(max_level),
      _item_num(_max_level),
      _where(g),
      _level(g,0),
      _items(_max_level),
      _first(_max_level+2),
      _last_active(_max_level+2),
      _highest_active(-1) {}
    ///Constructor.

    ///Constructor.
    ///
    ///\param g The underlying graph
    ///The range of the possible labels is [0...\c max_level],
    ///where \c max_level is equal to the number of labeled items in the graph.
    Elevator(const Graph &g) :
      _g(g),
      _max_level(countItems<Graph, Item>(g)),
      _item_num(_max_level),
      _where(g),
      _level(g,0),
      _items(_max_level),
      _first(_max_level+2),
      _last_active(_max_level+2),
      _highest_active(-1)
    {
    }

    ///Activate item \c i.

    ///Activate item \c i.
    ///\pre Item \c i shouldn't be active before.
    void activate(Item i)
    {
      const int l=_level[i];
      swap(_where[i],++_last_active[l]);
      if(l>_highest_active) _highest_active=l;
    }

    ///Deactivate item \c i.

    ///Deactivate item \c i.
    ///\pre Item \c i must be active before.
    void deactivate(Item i)
    {
      swap(_where[i],_last_active[_level[i]]--);
      while(_highest_active>=0 &&
            _last_active[_highest_active]<_first[_highest_active])
        _highest_active--;
    }

    ///Query whether item \c i is active
    bool active(Item i) const { return _where[i]<=_last_active[_level[i]]; }

    ///Return the level of item \c i.
    int operator[](Item i) const { return _level[i]; }

    ///Return the number of items on level \c l.
    int onLevel(int l) const
    {
      return _first[l+1]-_first[l];
    }
    ///Return true if the level is empty.
    bool emptyLevel(int l) const
    {
      return _first[l+1]-_first[l]==0;
    }
    ///Return the number of items above level \c l.
    int aboveLevel(int l) const
    {
      return _first[_max_level+1]-_first[l+1];
    }
    ///Return the number of active items on level \c l.
    int activesOnLevel(int l) const
    {
      return _last_active[l]-_first[l]+1;
    }
    ///Return true if there is not active item on level \c l.
    bool activeFree(int l) const
    {
      return _last_active[l]<_first[l];
    }
    ///Return the maximum allowed level.
    int maxLevel() const
    {
      return _max_level;
    }

    ///\name Highest Active Item
    ///Functions for working with the highest level
    ///active item.

    ///@{

    ///Return a highest level active item.

    ///Return a highest level active item.
    ///
    ///\return the highest level active item or INVALID if there is no active
    ///item.
    Item highestActive() const
    {
      return _highest_active>=0?*_last_active[_highest_active]:INVALID;
    }

    ///Return a highest active level.

    ///Return a highest active level.
    ///
    ///\return the level of the highest active item or -1 if there is no active
    ///item.
    int highestActiveLevel() const
    {
      return _highest_active;
    }

    ///Lift the highest active item by one.

    ///Lift the item returned by highestActive() by one.
    ///
    void liftHighestActive()
    {
      ++_level[*_last_active[_highest_active]];
      swap(_last_active[_highest_active]--,_last_active[_highest_active+1]);
      --_first[++_highest_active];
    }

    ///Lift the highest active item.

    ///Lift the item returned by highestActive() to level \c new_level.
    ///
    ///\warning \c new_level must be strictly higher
    ///than the current level.
    ///
    void liftHighestActive(int new_level)
    {
      const Item li = *_last_active[_highest_active];

      copy(--_first[_highest_active+1],_last_active[_highest_active]--);
      for(int l=_highest_active+1;l<new_level;l++)
        {
          copy(--_first[l+1],_first[l]);
          --_last_active[l];
        }
      copy(li,_first[new_level]);
      _level[li]=new_level;
      _highest_active=new_level;
    }

    ///Lift the highest active item.

    ///Lift the item returned by highestActive() to the top level and
    ///deactivates it.
    ///
    ///\warning \c new_level must be strictly higher
    ///than the current level.
    ///
    void liftHighestActiveToTop()
    {
      const Item li = *_last_active[_highest_active];

      copy(--_first[_highest_active+1],_last_active[_highest_active]--);
      for(int l=_highest_active+1;l<_max_level;l++)
        {
          copy(--_first[l+1],_first[l]);
          --_last_active[l];
        }
      copy(li,_first[_max_level]);
      --_last_active[_max_level];
      _level[li]=_max_level;

      while(_highest_active>=0 &&
            _last_active[_highest_active]<_first[_highest_active])
        _highest_active--;
    }

    ///@}

    ///\name Active Item on Certain Level
    ///Functions for working with the active items.

    ///@{

    ///Returns an active item on level \c l.

    ///Returns an active item on level \c l.
    ///
    ///Returns an active item on level \c l or \ref INVALID if there is no such
    ///an item. (\c l must be from the range [0...\c max_level].
    Item activeOn(int l) const
    {
      return _last_active[l]>=_first[l]?*_last_active[l]:INVALID;
    }

    ///Lifts the active item returned by \c activeOn() member function.

    ///Lifts the active item returned by \c activeOn() member function
    ///by one.
    Item liftActiveOn(int level)
    {
      ++_level[*_last_active[level]];
      swap(_last_active[level]--, --_first[level+1]);
      if (level+1>_highest_active) ++_highest_active;
    }

    ///Lifts the active item returned by \c activeOn() member function.

    ///Lifts the active item returned by \c activeOn() member function
    ///to the given level.
    void liftActiveOn(int level, int new_level)
    {
      const Item ai = *_last_active[level];

      copy(--_first[level+1], _last_active[level]--);
      for(int l=level+1;l<new_level;l++)
        {
          copy(_last_active[l],_first[l]);
          copy(--_first[l+1], _last_active[l]--);
        }
      copy(ai,_first[new_level]);
      _level[ai]=new_level;
      if (new_level>_highest_active) _highest_active=new_level;
    }

    ///Lifts the active item returned by \c activeOn() member function.

    ///Lifts the active item returned by \c activeOn() member function
    ///to the top level.
    void liftActiveToTop(int level)
    {
      const Item ai = *_last_active[level];

      copy(--_first[level+1],_last_active[level]--);
      for(int l=level+1;l<_max_level;l++)
        {
          copy(_last_active[l],_first[l]);
          copy(--_first[l+1], _last_active[l]--);
        }
      copy(ai,_first[_max_level]);
      --_last_active[_max_level];
      _level[ai]=_max_level;

      if (_highest_active==level) {
        while(_highest_active>=0 &&
              _last_active[_highest_active]<_first[_highest_active])
          _highest_active--;
      }
    }

    ///@}

    ///Lift an active item to a higher level.

    ///Lift an active item to a higher level.
    ///\param i The item to be lifted. It must be active.
    ///\param new_level The new level of \c i. It must be strictly higher
    ///than the current level.
    ///
    void lift(Item i, int new_level)
    {
      const int lo = _level[i];
      const Vit w = _where[i];

      copy(_last_active[lo],w);
      copy(--_first[lo+1],_last_active[lo]--);
      for(int l=lo+1;l<new_level;l++)
        {
          copy(_last_active[l],_first[l]);
          copy(--_first[l+1],_last_active[l]--);
        }
      copy(i,_first[new_level]);
      _level[i]=new_level;
      if(new_level>_highest_active) _highest_active=new_level;
    }

    ///Mark the node as it did not reach the max level

    ///Mark the node as it did not reach the max level. It sets the
    ///level to the under the max level value. The node will be never
    ///more activated because the push operation from the maximum
    ///level is forbidden in the push-relabel algorithms. The node
    ///should be lifted previously to the top level.
    void markToBottom(Item i) {
      _level[i] = _max_level - 1;
    }

    ///Lift all nodes on and above a level to the top (and deactivate them).

    ///This function lifts all nodes on and above level \c l to \c
    ///maxLevel(), and also deactivates them.
    void liftToTop(int l)
    {
      const Vit f=_first[l];
      const Vit tl=_first[_max_level];
      for(Vit i=f;i!=tl;++i)
        _level[*i]=_max_level;
      for(int i=l;i<=_max_level;i++)
        {
          _first[i]=f;
          _last_active[i]=f-1;
        }
      for(_highest_active=l-1;
          _highest_active>=0 &&
            _last_active[_highest_active]<_first[_highest_active];
          _highest_active--) ;
    }

  private:
    int _init_lev;
    Vit _init_num;

  public:

    ///\name Initialization
    ///Using this function you can initialize the levels of the item.
    ///\n
    ///This initializatios is started with calling \c initStart().
    ///Then the
    ///items should be listed levels by levels statring with the lowest one
    ///(with level 0). This is done by using \c initAddItem()
    ///and \c initNewLevel(). Finally \c initFinish() must be called.
    ///The items not listed will be put on the highest level.
    ///@{

    ///Start the initialization process.

    void initStart()
    {
      _init_lev=0;
      _init_num=_items.begin();
      _first[0]=_items.begin();
      _last_active[0]=_items.begin()-1;
      Vit n=_items.begin();
      for(typename ItemSetTraits<Graph,Item>::ItemIt i(_g);i!=INVALID;++i)
        {
          *n=i;
          _where[i]=n;
          _level[i]=_max_level;
          ++n;
        }
    }

    ///Add an item to the current level.

    void initAddItem(Item i)
    {
     swap(_where[i],_init_num);
      _level[i]=_init_lev;
      ++_init_num;
    }

    ///Start a new level.

    ///Start a new level.
    ///It shouldn't be used before the items on level 0 are listed.
    void initNewLevel()
    {
      _init_lev++;
      _first[_init_lev]=_init_num;
      _last_active[_init_lev]=_init_num-1;
    }

    ///Finalize the initialization process.

    void initFinish()
    {
      for(_init_lev++;_init_lev<=_max_level;_init_lev++)
        {
          _first[_init_lev]=_init_num;
          _last_active[_init_lev]=_init_num-1;
        }
      _first[_max_level+1]=_items.begin()+_item_num;
      _last_active[_max_level+1]=_items.begin()+_item_num-1;
      _highest_active = -1;
    }

    ///@}

  };

  ///Class for handling "labels" in push-relabel type algorithms.

  ///A class for handling "labels" in push-relabel type algorithms.
  ///
  ///\ingroup auxdat
  ///Using this class you can assign "labels" (nonnegative integer numbers)
  ///to the edges or nodes of a graph, manipulate and query them through
  ///operations typically arising in "push-relabel" type algorithms.
  ///
  ///Each item is either \em active or not, and you can also choose a
  ///highest level active item.
  ///
  ///\sa Elevator
  ///
  ///\param Graph the underlying graph type
  ///\param Item Type of the items the data is assigned to (Graph::Node,
  ///Graph::Edge, Graph::UEdge)
  template <class Graph, class Item>
  class LinkedElevator {
  public:

    typedef Item Key;
    typedef int Value;

  private:

    typedef typename ItemSetTraits<Graph,Item>::
    template Map<Item>::Type ItemMap;
    typedef typename ItemSetTraits<Graph,Item>::
    template Map<int>::Type IntMap;
    typedef typename ItemSetTraits<Graph,Item>::
    template Map<bool>::Type BoolMap;

    const Graph &_graph;
    int _max_level;
    int _item_num;
    std::vector<Item> _first, _last;
    ItemMap _prev, _next;
    int _highest_active;
    IntMap _level;
    BoolMap _active;

  public:
    ///Constructor with given maximum level.

    ///Constructor with given maximum level.
    ///
    ///\param g The underlying graph
    ///\param max_level Set the range of the possible labels to
    ///[0...\c max_level]
    LinkedElevator(const Graph& graph, int max_level)
      : _graph(graph), _max_level(max_level), _item_num(_max_level),
        _first(_max_level + 1), _last(_max_level + 1),
        _prev(graph), _next(graph),
        _highest_active(-1), _level(graph), _active(graph) {}

    ///Constructor.

    ///Constructor.
    ///
    ///\param g The underlying graph
    ///The range of the possible labels is [0...\c max_level],
    ///where \c max_level is equal to the number of labeled items in the graph.
    LinkedElevator(const Graph& graph)
      : _graph(graph), _max_level(countItems<Graph, Item>(graph)),
        _item_num(_max_level),
        _first(_max_level + 1), _last(_max_level + 1),
        _prev(graph, INVALID), _next(graph, INVALID),
        _highest_active(-1), _level(graph), _active(graph) {}


    ///Activate item \c i.

    ///Activate item \c i.
    ///\pre Item \c i shouldn't be active before.
    void activate(Item i) {
      _active.set(i, true);

      int level = _level[i];
      if (level > _highest_active) {
        _highest_active = level;
      }

      if (_prev[i] == INVALID || _active[_prev[i]]) return;
      //unlace
      _next.set(_prev[i], _next[i]);
      if (_next[i] != INVALID) {
        _prev.set(_next[i], _prev[i]);
      } else {
        _last[level] = _prev[i];
      }
      //lace
      _next.set(i, _first[level]);
      _prev.set(_first[level], i);
      _prev.set(i, INVALID);
      _first[level] = i;

    }

    ///Deactivate item \c i.

    ///Deactivate item \c i.
    ///\pre Item \c i must be active before.
    void deactivate(Item i) {
      _active.set(i, false);
      int level = _level[i];

      if (_next[i] == INVALID || !_active[_next[i]])
        goto find_highest_level;

      //unlace
      _prev.set(_next[i], _prev[i]);
      if (_prev[i] != INVALID) {
        _next.set(_prev[i], _next[i]);
      } else {
        _first[_level[i]] = _next[i];
      }
      //lace
      _prev.set(i, _last[level]);
      _next.set(_last[level], i);
      _next.set(i, INVALID);
      _last[level] = i;

    find_highest_level:
      if (level == _highest_active) {
        while (_highest_active >= 0 && activeFree(_highest_active))
          --_highest_active;
      }
    }

    ///Query whether item \c i is active
    bool active(Item i) const { return _active[i]; }

    ///Return the level of item \c i.
    int operator[](Item i) const { return _level[i]; }

    ///Return the number of items on level \c l.
    int onLevel(int l) const {
      int num = 0;
      Item n = _first[l];
      while (n != INVALID) {
        ++num;
        n = _next[n];
      }
      return num;
    }

    ///Return true if the level is empty.
    bool emptyLevel(int l) const {
      return _first[l] == INVALID;
    }

    ///Return the number of items above level \c l.
    int aboveLevel(int l) const {
      int num = 0;
      for (int level = l + 1; level < _max_level; ++level)
        num += onLevel(level);
      return num;
    }

    ///Return the number of active items on level \c l.
    int activesOnLevel(int l) const {
      int num = 0;
      Item n = _first[l];
      while (n != INVALID && _active[n]) {
        ++num;
        n = _next[n];
      }
      return num;
    }

    ///Return true if there is not active item on level \c l.
    bool activeFree(int l) const {
      return _first[l] == INVALID || !_active[_first[l]];
    }

    ///Return the maximum allowed level.
    int maxLevel() const {
      return _max_level;
    }

    ///\name Highest Active Item
    ///Functions for working with the highest level
    ///active item.

    ///@{

    ///Return a highest level active item.

    ///Return a highest level active item.
    ///
    ///\return the highest level active item or INVALID if there is no
    ///active item.
    Item highestActive() const {
      return _highest_active >= 0 ? _first[_highest_active] : INVALID;
    }

    ///Return a highest active level.

    ///Return a highest active level.
    ///
    ///\return the level of the highest active item or -1 if there is
    ///no active item.
    int highestActiveLevel() const {
      return _highest_active;
    }

    ///Lift the highest active item by one.

    ///Lift the item returned by highestActive() by one.
    ///
    void liftHighestActive() {
      Item i = _first[_highest_active];
      if (_next[i] != INVALID) {
        _prev.set(_next[i], INVALID);
        _first[_highest_active] = _next[i];
      } else {
        _first[_highest_active] = INVALID;
        _last[_highest_active] = INVALID;
      }
      _level.set(i, ++_highest_active);
      if (_first[_highest_active] == INVALID) {
        _first[_highest_active] = i;
        _last[_highest_active] = i;
        _prev.set(i, INVALID);
        _next.set(i, INVALID);
      } else {
        _prev.set(_first[_highest_active], i);
        _next.set(i, _first[_highest_active]);
        _first[_highest_active] = i;
      }
    }

    ///Lift the highest active item.

    ///Lift the item returned by highestActive() to level \c new_level.
    ///
    ///\warning \c new_level must be strictly higher
    ///than the current level.
    ///
    void liftHighestActive(int new_level) {
      Item i = _first[_highest_active];
      if (_next[i] != INVALID) {
        _prev.set(_next[i], INVALID);
        _first[_highest_active] = _next[i];
      } else {
        _first[_highest_active] = INVALID;
        _last[_highest_active] = INVALID;
      }
      _level.set(i, _highest_active = new_level);
      if (_first[_highest_active] == INVALID) {
        _first[_highest_active] = _last[_highest_active] = i;
        _prev.set(i, INVALID);
        _next.set(i, INVALID);
      } else {
        _prev.set(_first[_highest_active], i);
        _next.set(i, _first[_highest_active]);
        _first[_highest_active] = i;
      }
    }

    ///Lift the highest active to top.

    ///Lift the item returned by highestActive() to the top level and
    ///deactivates the node.
    ///
    void liftHighestActiveToTop() {
      Item i = _first[_highest_active];
      _level.set(i, _max_level);
      if (_next[i] != INVALID) {
        _prev.set(_next[i], INVALID);
        _first[_highest_active] = _next[i];
      } else {
        _first[_highest_active] = INVALID;
        _last[_highest_active] = INVALID;
      }
      while (_highest_active >= 0 && activeFree(_highest_active))
        --_highest_active;
    }

    ///@}

    ///\name Active Item on Certain Level
    ///Functions for working with the active items.

    ///@{

    ///Returns an active item on level \c l.

    ///Returns an active item on level \c l.
    ///
    ///Returns an active item on level \c l or \ref INVALID if there is no such
    ///an item. (\c l must be from the range [0...\c max_level].
    Item activeOn(int l) const
    {
      return _active[_first[l]] ? _first[l] : INVALID;
    }

    ///Lifts the active item returned by \c activeOn() member function.

    ///Lifts the active item returned by \c activeOn() member function
    ///by one.
    Item liftActiveOn(int l)
    {
      Item i = _first[l];
      if (_next[i] != INVALID) {
        _prev.set(_next[i], INVALID);
        _first[l] = _next[i];
      } else {
        _first[l] = INVALID;
        _last[l] = INVALID;
      }
      _level.set(i, ++l);
      if (_first[l] == INVALID) {
        _first[l] = _last[l] = i;
        _prev.set(i, INVALID);
        _next.set(i, INVALID);
      } else {
        _prev.set(_first[l], i);
        _next.set(i, _first[l]);
        _first[l] = i;
      }
      if (_highest_active < l) {
        _highest_active = l;
      }
    }

    /// \brief Lifts the active item returned by \c activeOn() member function.
    ///
    /// Lifts the active item returned by \c activeOn() member function
    /// to the given level.
    void liftActiveOn(int l, int new_level)
    {
      Item i = _first[l];
      if (_next[i] != INVALID) {
        _prev.set(_next[i], INVALID);
        _first[l] = _next[i];
      } else {
        _first[l] = INVALID;
        _last[l] = INVALID;
      }
      _level.set(i, l = new_level);
      if (_first[l] == INVALID) {
        _first[l] = _last[l] = i;
        _prev.set(i, INVALID);
        _next.set(i, INVALID);
      } else {
        _prev.set(_first[l], i);
        _next.set(i, _first[l]);
        _first[l] = i;
      }
      if (_highest_active < l) {
        _highest_active = l;
      }
    }

    ///Lifts the active item returned by \c activeOn() member function.

    ///Lifts the active item returned by \c activeOn() member function
    ///to the top level.
    void liftActiveToTop(int l)
    {
      Item i = _first[l];
      if (_next[i] != INVALID) {
        _prev.set(_next[i], INVALID);
        _first[l] = _next[i];
      } else {
        _first[l] = INVALID;
        _last[l] = INVALID;
      }
      _level.set(i, _max_level);
      if (l == _highest_active) {
        while (_highest_active >= 0 && activeFree(_highest_active))
          --_highest_active;
      }
    }

    ///@}

    /// \brief Lift an active item to a higher level.
    ///
    /// Lift an active item to a higher level.
    /// \param i The item to be lifted. It must be active.
    /// \param new_level The new level of \c i. It must be strictly higher
    /// than the current level.
    ///
    void lift(Item i, int new_level) {
      if (_next[i] != INVALID) {
        _prev.set(_next[i], _prev[i]);
      } else {
        _last[new_level] = _prev[i];
      }
      if (_prev[i] != INVALID) {
        _next.set(_prev[i], _next[i]);
      } else {
        _first[new_level] = _next[i];
      }
      _level.set(i, new_level);
      if (_first[new_level] == INVALID) {
        _first[new_level] = _last[new_level] = i;
        _prev.set(i, INVALID);
        _next.set(i, INVALID);
      } else {
        _prev.set(_first[new_level], i);
        _next.set(i, _first[new_level]);
        _first[new_level] = i;
      }
      if (_highest_active < new_level) {
        _highest_active = new_level;
      }
    }

    ///Mark the node as it did not reach the max level

    ///Mark the node as it did not reach the max level. It sets the
    ///level to the under the max level value. The node will be never
    ///more activated because the push operation from the maximum
    ///level is forbidden in the push-relabel algorithms. The node
    ///should be lifted previously to the top level.
    void markToBottom(Item i) {
      _level.set(i, _max_level - 1);
    }

    ///Lift all nodes on and above a level to the top (and deactivate them).

    ///This function lifts all nodes on and above level \c l to \c
    ///maxLevel(), and also deactivates them.
    void liftToTop(int l)  {
      for (int i = l + 1; _first[i] != INVALID; ++i) {
        Item n = _first[i];
        while (n != INVALID) {
          _level.set(n, _max_level);
          n = _next[n];
        }
        _first[i] = INVALID;
        _last[i] = INVALID;
      }
      if (_highest_active > l - 1) {
        _highest_active = l - 1;
        while (_highest_active >= 0 && activeFree(_highest_active))
          --_highest_active;
      }
    }

  private:

    int _init_level;

  public:

    ///\name Initialization
    ///Using this function you can initialize the levels of the item.
    ///\n
    ///This initializatios is started with calling \c initStart().
    ///Then the
    ///items should be listed levels by levels statring with the lowest one
    ///(with level 0). This is done by using \c initAddItem()
    ///and \c initNewLevel(). Finally \c initFinish() must be called.
    ///The items not listed will be put on the highest level.
    ///@{

    ///Start the initialization process.

    void initStart() {

      for (int i = 0; i <= _max_level; ++i) {
        _first[i] = _last[i] = INVALID;
      }
      _init_level = 0;
      for(typename ItemSetTraits<Graph,Item>::ItemIt i(_graph);
          i != INVALID; ++i) {
        _level.set(i, _max_level);
        _active.set(i, false);
      }
    }

    ///Add an item to the current level.

    void initAddItem(Item i) {
      _level.set(i, _init_level);
      if (_last[_init_level] == INVALID) {
        _first[_init_level] = i;
        _last[_init_level] = i;
        _prev.set(i, INVALID);
        _next.set(i, INVALID);
      } else {
        _prev.set(i, _last[_init_level]);
        _next.set(i, INVALID);
        _next.set(_last[_init_level], i);
        _last[_init_level] = i;
      }
    }

    ///Start a new level.

    ///Start a new level.
    ///It shouldn't be used before the items on level 0 are listed.
    void initNewLevel() {
      ++_init_level;
    }

    ///Finalize the initialization process.

    void initFinish() {
      _highest_active = -1;
    }

    ///@}

  };


} //END OF NAMESPACE LEMON

#endif

