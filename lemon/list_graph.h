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

#ifndef LEMON_LIST_GRAPH_H
#define LEMON_LIST_GRAPH_H

///\ingroup graphs
///\file
///\brief ListDigraph, ListGraph classes.

#include <lemon/core.h>
#include <lemon/error.h>
#include <lemon/bits/graph_extender.h>

#include <vector>
#include <list>

namespace lemon {

  class ListDigraphBase {

  protected:
    struct NodeT {
      int first_in, first_out;
      int prev, next;
    };

    struct ArcT {
      int target, source;
      int prev_in, prev_out;
      int next_in, next_out;
    };

    std::vector<NodeT> nodes;

    int first_node;

    int first_free_node;

    std::vector<ArcT> arcs;

    int first_free_arc;

  public:

    typedef ListDigraphBase Digraph;

    class Node {
      friend class ListDigraphBase;
    protected:

      int id;
      explicit Node(int pid) { id = pid;}

    public:
      Node() {}
      Node (Invalid) { id = -1; }
      bool operator==(const Node& node) const {return id == node.id;}
      bool operator!=(const Node& node) const {return id != node.id;}
      bool operator<(const Node& node) const {return id < node.id;}
    };

    class Arc {
      friend class ListDigraphBase;
    protected:

      int id;
      explicit Arc(int pid) { id = pid;}

    public:
      Arc() {}
      Arc (Invalid) { id = -1; }
      bool operator==(const Arc& arc) const {return id == arc.id;}
      bool operator!=(const Arc& arc) const {return id != arc.id;}
      bool operator<(const Arc& arc) const {return id < arc.id;}
    };



    ListDigraphBase()
      : nodes(), first_node(-1),
        first_free_node(-1), arcs(), first_free_arc(-1) {}


    int maxNodeId() const { return nodes.size()-1; }
    int maxArcId() const { return arcs.size()-1; }

    Node source(Arc e) const { return Node(arcs[e.id].source); }
    Node target(Arc e) const { return Node(arcs[e.id].target); }


    void first(Node& node) const {
      node.id = first_node;
    }

    void next(Node& node) const {
      node.id = nodes[node.id].next;
    }


    void first(Arc& arc) const {
      int n;
      for(n = first_node;
          n!=-1 && nodes[n].first_in == -1;
          n = nodes[n].next) {}
      arc.id = (n == -1) ? -1 : nodes[n].first_in;
    }

    void next(Arc& arc) const {
      if (arcs[arc.id].next_in != -1) {
        arc.id = arcs[arc.id].next_in;
      } else {
        int n;
        for(n = nodes[arcs[arc.id].target].next;
            n!=-1 && nodes[n].first_in == -1;
            n = nodes[n].next) {}
        arc.id = (n == -1) ? -1 : nodes[n].first_in;
      }
    }

    void firstOut(Arc &e, const Node& v) const {
      e.id = nodes[v.id].first_out;
    }
    void nextOut(Arc &e) const {
      e.id=arcs[e.id].next_out;
    }

    void firstIn(Arc &e, const Node& v) const {
      e.id = nodes[v.id].first_in;
    }
    void nextIn(Arc &e) const {
      e.id=arcs[e.id].next_in;
    }


    static int id(Node v) { return v.id; }
    static int id(Arc e) { return e.id; }

    static Node nodeFromId(int id) { return Node(id);}
    static Arc arcFromId(int id) { return Arc(id);}

    bool valid(Node n) const {
      return n.id >= 0 && n.id < static_cast<int>(nodes.size()) &&
        nodes[n.id].prev != -2;
    }

    bool valid(Arc a) const {
      return a.id >= 0 && a.id < static_cast<int>(arcs.size()) &&
        arcs[a.id].prev_in != -2;
    }

    Node addNode() {
      int n;

      if(first_free_node==-1) {
        n = nodes.size();
        nodes.push_back(NodeT());
      } else {
        n = first_free_node;
        first_free_node = nodes[n].next;
      }

      nodes[n].next = first_node;
      if(first_node != -1) nodes[first_node].prev = n;
      first_node = n;
      nodes[n].prev = -1;

      nodes[n].first_in = nodes[n].first_out = -1;

      return Node(n);
    }

    Arc addArc(Node u, Node v) {
      int n;

      if (first_free_arc == -1) {
        n = arcs.size();
        arcs.push_back(ArcT());
      } else {
        n = first_free_arc;
        first_free_arc = arcs[n].next_in;
      }

      arcs[n].source = u.id;
      arcs[n].target = v.id;

      arcs[n].next_out = nodes[u.id].first_out;
      if(nodes[u.id].first_out != -1) {
        arcs[nodes[u.id].first_out].prev_out = n;
      }

      arcs[n].next_in = nodes[v.id].first_in;
      if(nodes[v.id].first_in != -1) {
        arcs[nodes[v.id].first_in].prev_in = n;
      }

      arcs[n].prev_in = arcs[n].prev_out = -1;

      nodes[u.id].first_out = nodes[v.id].first_in = n;

      return Arc(n);
    }

    void erase(const Node& node) {
      int n = node.id;

      if(nodes[n].next != -1) {
        nodes[nodes[n].next].prev = nodes[n].prev;
      }

      if(nodes[n].prev != -1) {
        nodes[nodes[n].prev].next = nodes[n].next;
      } else {
        first_node = nodes[n].next;
      }

      nodes[n].next = first_free_node;
      first_free_node = n;
      nodes[n].prev = -2;

    }

    void erase(const Arc& arc) {
      int n = arc.id;

      if(arcs[n].next_in!=-1) {
        arcs[arcs[n].next_in].prev_in = arcs[n].prev_in;
      }

      if(arcs[n].prev_in!=-1) {
        arcs[arcs[n].prev_in].next_in = arcs[n].next_in;
      } else {
        nodes[arcs[n].target].first_in = arcs[n].next_in;
      }


      if(arcs[n].next_out!=-1) {
        arcs[arcs[n].next_out].prev_out = arcs[n].prev_out;
      }

      if(arcs[n].prev_out!=-1) {
        arcs[arcs[n].prev_out].next_out = arcs[n].next_out;
      } else {
        nodes[arcs[n].source].first_out = arcs[n].next_out;
      }

      arcs[n].next_in = first_free_arc;
      first_free_arc = n;
      arcs[n].prev_in = -2;
    }

    void clear() {
      arcs.clear();
      nodes.clear();
      first_node = first_free_node = first_free_arc = -1;
    }

  protected:
    void changeTarget(Arc e, Node n)
    {
      if(arcs[e.id].next_in != -1)
        arcs[arcs[e.id].next_in].prev_in = arcs[e.id].prev_in;
      if(arcs[e.id].prev_in != -1)
        arcs[arcs[e.id].prev_in].next_in = arcs[e.id].next_in;
      else nodes[arcs[e.id].target].first_in = arcs[e.id].next_in;
      if (nodes[n.id].first_in != -1) {
        arcs[nodes[n.id].first_in].prev_in = e.id;
      }
      arcs[e.id].target = n.id;
      arcs[e.id].prev_in = -1;
      arcs[e.id].next_in = nodes[n.id].first_in;
      nodes[n.id].first_in = e.id;
    }
    void changeSource(Arc e, Node n)
    {
      if(arcs[e.id].next_out != -1)
        arcs[arcs[e.id].next_out].prev_out = arcs[e.id].prev_out;
      if(arcs[e.id].prev_out != -1)
        arcs[arcs[e.id].prev_out].next_out = arcs[e.id].next_out;
      else nodes[arcs[e.id].source].first_out = arcs[e.id].next_out;
      if (nodes[n.id].first_out != -1) {
        arcs[nodes[n.id].first_out].prev_out = e.id;
      }
      arcs[e.id].source = n.id;
      arcs[e.id].prev_out = -1;
      arcs[e.id].next_out = nodes[n.id].first_out;
      nodes[n.id].first_out = e.id;
    }

  };

  typedef DigraphExtender<ListDigraphBase> ExtendedListDigraphBase;

  /// \addtogroup graphs
  /// @{

  ///A general directed graph structure.

  ///\ref ListDigraph is a simple and fast <em>directed graph</em>
  ///implementation based on static linked lists that are stored in
  ///\c std::vector structures.
  ///
  ///It conforms to the \ref concepts::Digraph "Digraph concept" and it
  ///also provides several useful additional functionalities.
  ///Most of the member functions and nested classes are documented
  ///only in the concept class.
  ///
  ///An important extra feature of this digraph implementation is that
  ///its maps are real \ref concepts::ReferenceMap "reference map"s.
  ///
  ///\sa concepts::Digraph

  class ListDigraph : public ExtendedListDigraphBase {
  private:
    ///ListDigraph is \e not copy constructible. Use copyDigraph() instead.

    ///ListDigraph is \e not copy constructible. Use copyDigraph() instead.
    ///
    ListDigraph(const ListDigraph &) :ExtendedListDigraphBase() {};
    ///\brief Assignment of ListDigraph to another one is \e not allowed.
    ///Use copyDigraph() instead.

    ///Assignment of ListDigraph to another one is \e not allowed.
    ///Use copyDigraph() instead.
    void operator=(const ListDigraph &) {}
  public:

    typedef ExtendedListDigraphBase Parent;

    /// Constructor

    /// Constructor.
    ///
    ListDigraph() {}

    ///Add a new node to the digraph.

    ///Add a new node to the digraph.
    ///\return the new node.
    Node addNode() { return Parent::addNode(); }

    ///Add a new arc to the digraph.

    ///Add a new arc to the digraph with source node \c s
    ///and target node \c t.
    ///\return the new arc.
    Arc addArc(const Node& s, const Node& t) {
      return Parent::addArc(s, t);
    }

    ///\brief Erase a node from the digraph.
    ///
    ///Erase a node from the digraph.
    ///
    void erase(const Node& n) { Parent::erase(n); }

    ///\brief Erase an arc from the digraph.
    ///
    ///Erase an arc from the digraph.
    ///
    void erase(const Arc& a) { Parent::erase(a); }

    /// Node validity check

    /// This function gives back true if the given node is valid,
    /// ie. it is a real node of the graph.
    ///
    /// \warning A Node pointing to a removed item
    /// could become valid again later if new nodes are
    /// added to the graph.
    bool valid(Node n) const { return Parent::valid(n); }

    /// Arc validity check

    /// This function gives back true if the given arc is valid,
    /// ie. it is a real arc of the graph.
    ///
    /// \warning An Arc pointing to a removed item
    /// could become valid again later if new nodes are
    /// added to the graph.
    bool valid(Arc a) const { return Parent::valid(a); }

    /// Change the target of \c a to \c n

    /// Change the target of \c a to \c n
    ///
    ///\note The <tt>ArcIt</tt>s and <tt>OutArcIt</tt>s referencing
    ///the changed arc remain valid. However <tt>InArcIt</tt>s are
    ///invalidated.
    ///
    ///\warning This functionality cannot be used together with the Snapshot
    ///feature.
    void changeTarget(Arc a, Node n) {
      Parent::changeTarget(a,n);
    }
    /// Change the source of \c a to \c n

    /// Change the source of \c a to \c n
    ///
    ///\note The <tt>InArcIt</tt>s referencing the changed arc remain
    ///valid. However the <tt>ArcIt</tt>s and <tt>OutArcIt</tt>s are
    ///invalidated.
    ///
    ///\warning This functionality cannot be used together with the Snapshot
    ///feature.
    void changeSource(Arc a, Node n) {
      Parent::changeSource(a,n);
    }

    /// Invert the direction of an arc.

    ///\note The <tt>ArcIt</tt>s referencing the changed arc remain
    ///valid. However <tt>OutArcIt</tt>s and <tt>InArcIt</tt>s are
    ///invalidated.
    ///
    ///\warning This functionality cannot be used together with the Snapshot
    ///feature.
    void reverseArc(Arc e) {
      Node t=target(e);
      changeTarget(e,source(e));
      changeSource(e,t);
    }

    /// Reserve memory for nodes.

    /// Using this function it is possible to avoid the superfluous memory
    /// allocation: if you know that the digraph you want to build will
    /// be very large (e.g. it will contain millions of nodes and/or arcs)
    /// then it is worth reserving space for this amount before starting
    /// to build the digraph.
    /// \sa reserveArc
    void reserveNode(int n) { nodes.reserve(n); };

    /// Reserve memory for arcs.

    /// Using this function it is possible to avoid the superfluous memory
    /// allocation: if you know that the digraph you want to build will
    /// be very large (e.g. it will contain millions of nodes and/or arcs)
    /// then it is worth reserving space for this amount before starting
    /// to build the digraph.
    /// \sa reserveNode
    void reserveArc(int m) { arcs.reserve(m); };

    ///Contract two nodes.

    ///This function contracts two nodes.
    ///Node \p b will be removed but instead of deleting
    ///incident arcs, they will be joined to \p a.
    ///The last parameter \p r controls whether to remove loops. \c true
    ///means that loops will be removed.
    ///
    ///\note The <tt>ArcIt</tt>s referencing a moved arc remain
    ///valid. However <tt>InArcIt</tt>s and <tt>OutArcIt</tt>s
    ///may be invalidated.
    ///
    ///\warning This functionality cannot be used together with the Snapshot
    ///feature.
    void contract(Node a, Node b, bool r = true)
    {
      for(OutArcIt e(*this,b);e!=INVALID;) {
        OutArcIt f=e;
        ++f;
        if(r && target(e)==a) erase(e);
        else changeSource(e,a);
        e=f;
      }
      for(InArcIt e(*this,b);e!=INVALID;) {
        InArcIt f=e;
        ++f;
        if(r && source(e)==a) erase(e);
        else changeTarget(e,a);
        e=f;
      }
      erase(b);
    }

    ///Split a node.

    ///This function splits a node. First a new node is added to the digraph,
    ///then the source of each outgoing arc of \c n is moved to this new node.
    ///If \c connect is \c true (this is the default value), then a new arc
    ///from \c n to the newly created node is also added.
    ///\return The newly created node.
    ///
    ///\note The <tt>ArcIt</tt>s referencing a moved arc remain
    ///valid. However <tt>InArcIt</tt>s and <tt>OutArcIt</tt>s may
    ///be invalidated.
    ///
    ///\warning This functionality cannot be used in conjunction with the
    ///Snapshot feature.
    Node split(Node n, bool connect = true) {
      Node b = addNode();
      for(OutArcIt e(*this,n);e!=INVALID;) {
        OutArcIt f=e;
        ++f;
        changeSource(e,b);
        e=f;
      }
      if (connect) addArc(n,b);
      return b;
    }

    ///Split an arc.

    ///This function splits an arc. First a new node \c b is added to
    ///the digraph, then the original arc is re-targeted to \c
    ///b. Finally an arc from \c b to the original target is added.
    ///
    ///\return The newly created node.
    ///
    ///\warning This functionality cannot be used together with the
    ///Snapshot feature.
    Node split(Arc e) {
      Node b = addNode();
      addArc(b,target(e));
      changeTarget(e,b);
      return b;
    }

    /// \brief Class to make a snapshot of the digraph and restore
    /// it later.
    ///
    /// Class to make a snapshot of the digraph and restore it later.
    ///
    /// The newly added nodes and arcs can be removed using the
    /// restore() function.
    ///
    /// \warning Arc and node deletions and other modifications (e.g.
    /// contracting, splitting, reversing arcs or nodes) cannot be
    /// restored. These events invalidate the snapshot.
    class Snapshot {
    protected:

      typedef Parent::NodeNotifier NodeNotifier;

      class NodeObserverProxy : public NodeNotifier::ObserverBase {
      public:

        NodeObserverProxy(Snapshot& _snapshot)
          : snapshot(_snapshot) {}

        using NodeNotifier::ObserverBase::attach;
        using NodeNotifier::ObserverBase::detach;
        using NodeNotifier::ObserverBase::attached;

      protected:

        virtual void add(const Node& node) {
          snapshot.addNode(node);
        }
        virtual void add(const std::vector<Node>& nodes) {
          for (int i = nodes.size() - 1; i >= 0; ++i) {
            snapshot.addNode(nodes[i]);
          }
        }
        virtual void erase(const Node& node) {
          snapshot.eraseNode(node);
        }
        virtual void erase(const std::vector<Node>& nodes) {
          for (int i = 0; i < int(nodes.size()); ++i) {
            snapshot.eraseNode(nodes[i]);
          }
        }
        virtual void build() {
          Node node;
          std::vector<Node> nodes;
          for (notifier()->first(node); node != INVALID;
               notifier()->next(node)) {
            nodes.push_back(node);
          }
          for (int i = nodes.size() - 1; i >= 0; --i) {
            snapshot.addNode(nodes[i]);
          }
        }
        virtual void clear() {
          Node node;
          for (notifier()->first(node); node != INVALID;
               notifier()->next(node)) {
            snapshot.eraseNode(node);
          }
        }

        Snapshot& snapshot;
      };

      class ArcObserverProxy : public ArcNotifier::ObserverBase {
      public:

        ArcObserverProxy(Snapshot& _snapshot)
          : snapshot(_snapshot) {}

        using ArcNotifier::ObserverBase::attach;
        using ArcNotifier::ObserverBase::detach;
        using ArcNotifier::ObserverBase::attached;

      protected:

        virtual void add(const Arc& arc) {
          snapshot.addArc(arc);
        }
        virtual void add(const std::vector<Arc>& arcs) {
          for (int i = arcs.size() - 1; i >= 0; ++i) {
            snapshot.addArc(arcs[i]);
          }
        }
        virtual void erase(const Arc& arc) {
          snapshot.eraseArc(arc);
        }
        virtual void erase(const std::vector<Arc>& arcs) {
          for (int i = 0; i < int(arcs.size()); ++i) {
            snapshot.eraseArc(arcs[i]);
          }
        }
        virtual void build() {
          Arc arc;
          std::vector<Arc> arcs;
          for (notifier()->first(arc); arc != INVALID;
               notifier()->next(arc)) {
            arcs.push_back(arc);
          }
          for (int i = arcs.size() - 1; i >= 0; --i) {
            snapshot.addArc(arcs[i]);
          }
        }
        virtual void clear() {
          Arc arc;
          for (notifier()->first(arc); arc != INVALID;
               notifier()->next(arc)) {
            snapshot.eraseArc(arc);
          }
        }

        Snapshot& snapshot;
      };

      ListDigraph *digraph;

      NodeObserverProxy node_observer_proxy;
      ArcObserverProxy arc_observer_proxy;

      std::list<Node> added_nodes;
      std::list<Arc> added_arcs;


      void addNode(const Node& node) {
        added_nodes.push_front(node);
      }
      void eraseNode(const Node& node) {
        std::list<Node>::iterator it =
          std::find(added_nodes.begin(), added_nodes.end(), node);
        if (it == added_nodes.end()) {
          clear();
          arc_observer_proxy.detach();
          throw NodeNotifier::ImmediateDetach();
        } else {
          added_nodes.erase(it);
        }
      }

      void addArc(const Arc& arc) {
        added_arcs.push_front(arc);
      }
      void eraseArc(const Arc& arc) {
        std::list<Arc>::iterator it =
          std::find(added_arcs.begin(), added_arcs.end(), arc);
        if (it == added_arcs.end()) {
          clear();
          node_observer_proxy.detach();
          throw ArcNotifier::ImmediateDetach();
        } else {
          added_arcs.erase(it);
        }
      }

      void attach(ListDigraph &_digraph) {
        digraph = &_digraph;
        node_observer_proxy.attach(digraph->notifier(Node()));
        arc_observer_proxy.attach(digraph->notifier(Arc()));
      }

      void detach() {
        node_observer_proxy.detach();
        arc_observer_proxy.detach();
      }

      bool attached() const {
        return node_observer_proxy.attached();
      }

      void clear() {
        added_nodes.clear();
        added_arcs.clear();
      }

    public:

      /// \brief Default constructor.
      ///
      /// Default constructor.
      /// To actually make a snapshot you must call save().
      Snapshot()
        : digraph(0), node_observer_proxy(*this),
          arc_observer_proxy(*this) {}

      /// \brief Constructor that immediately makes a snapshot.
      ///
      /// This constructor immediately makes a snapshot of the digraph.
      /// \param _digraph The digraph we make a snapshot of.
      Snapshot(ListDigraph &_digraph)
        : node_observer_proxy(*this),
          arc_observer_proxy(*this) {
        attach(_digraph);
      }

      /// \brief Make a snapshot.
      ///
      /// Make a snapshot of the digraph.
      ///
      /// This function can be called more than once. In case of a repeated
      /// call, the previous snapshot gets lost.
      /// \param _digraph The digraph we make the snapshot of.
      void save(ListDigraph &_digraph) {
        if (attached()) {
          detach();
          clear();
        }
        attach(_digraph);
      }

      /// \brief Undo the changes until the last snapshot.
      //
      /// Undo the changes until the last snapshot created by save().
      void restore() {
        detach();
        for(std::list<Arc>::iterator it = added_arcs.begin();
            it != added_arcs.end(); ++it) {
          digraph->erase(*it);
        }
        for(std::list<Node>::iterator it = added_nodes.begin();
            it != added_nodes.end(); ++it) {
          digraph->erase(*it);
        }
        clear();
      }

      /// \brief Gives back true when the snapshot is valid.
      ///
      /// Gives back true when the snapshot is valid.
      bool valid() const {
        return attached();
      }
    };

  };

  ///@}

  class ListGraphBase {

  protected:

    struct NodeT {
      int first_out;
      int prev, next;
    };

    struct ArcT {
      int target;
      int prev_out, next_out;
    };

    std::vector<NodeT> nodes;

    int first_node;

    int first_free_node;

    std::vector<ArcT> arcs;

    int first_free_arc;

  public:

    typedef ListGraphBase Digraph;

    class Node;
    class Arc;
    class Edge;

    class Node {
      friend class ListGraphBase;
    protected:

      int id;
      explicit Node(int pid) { id = pid;}

    public:
      Node() {}
      Node (Invalid) { id = -1; }
      bool operator==(const Node& node) const {return id == node.id;}
      bool operator!=(const Node& node) const {return id != node.id;}
      bool operator<(const Node& node) const {return id < node.id;}
    };

    class Edge {
      friend class ListGraphBase;
    protected:

      int id;
      explicit Edge(int pid) { id = pid;}

    public:
      Edge() {}
      Edge (Invalid) { id = -1; }
      bool operator==(const Edge& edge) const {return id == edge.id;}
      bool operator!=(const Edge& edge) const {return id != edge.id;}
      bool operator<(const Edge& edge) const {return id < edge.id;}
    };

    class Arc {
      friend class ListGraphBase;
    protected:

      int id;
      explicit Arc(int pid) { id = pid;}

    public:
      operator Edge() const { 
        return id != -1 ? edgeFromId(id / 2) : INVALID; 
      }

      Arc() {}
      Arc (Invalid) { id = -1; }
      bool operator==(const Arc& arc) const {return id == arc.id;}
      bool operator!=(const Arc& arc) const {return id != arc.id;}
      bool operator<(const Arc& arc) const {return id < arc.id;}
    };



    ListGraphBase()
      : nodes(), first_node(-1),
        first_free_node(-1), arcs(), first_free_arc(-1) {}


    int maxNodeId() const { return nodes.size()-1; }
    int maxEdgeId() const { return arcs.size() / 2 - 1; }
    int maxArcId() const { return arcs.size()-1; }

    Node source(Arc e) const { return Node(arcs[e.id ^ 1].target); }
    Node target(Arc e) const { return Node(arcs[e.id].target); }

    Node u(Edge e) const { return Node(arcs[2 * e.id].target); }
    Node v(Edge e) const { return Node(arcs[2 * e.id + 1].target); }

    static bool direction(Arc e) {
      return (e.id & 1) == 1;
    }

    static Arc direct(Edge e, bool d) {
      return Arc(e.id * 2 + (d ? 1 : 0));
    }

    void first(Node& node) const {
      node.id = first_node;
    }

    void next(Node& node) const {
      node.id = nodes[node.id].next;
    }

    void first(Arc& e) const {
      int n = first_node;
      while (n != -1 && nodes[n].first_out == -1) {
        n = nodes[n].next;
      }
      e.id = (n == -1) ? -1 : nodes[n].first_out;
    }

    void next(Arc& e) const {
      if (arcs[e.id].next_out != -1) {
        e.id = arcs[e.id].next_out;
      } else {
        int n = nodes[arcs[e.id ^ 1].target].next;
        while(n != -1 && nodes[n].first_out == -1) {
          n = nodes[n].next;
        }
        e.id = (n == -1) ? -1 : nodes[n].first_out;
      }
    }

    void first(Edge& e) const {
      int n = first_node;
      while (n != -1) {
        e.id = nodes[n].first_out;
        while ((e.id & 1) != 1) {
          e.id = arcs[e.id].next_out;
        }
        if (e.id != -1) {
          e.id /= 2;
          return;
        }
        n = nodes[n].next;
      }
      e.id = -1;
    }

    void next(Edge& e) const {
      int n = arcs[e.id * 2].target;
      e.id = arcs[(e.id * 2) | 1].next_out;
      while ((e.id & 1) != 1) {
        e.id = arcs[e.id].next_out;
      }
      if (e.id != -1) {
        e.id /= 2;
        return;
      }
      n = nodes[n].next;
      while (n != -1) {
        e.id = nodes[n].first_out;
        while ((e.id & 1) != 1) {
          e.id = arcs[e.id].next_out;
        }
        if (e.id != -1) {
          e.id /= 2;
          return;
        }
        n = nodes[n].next;
      }
      e.id = -1;
    }

    void firstOut(Arc &e, const Node& v) const {
      e.id = nodes[v.id].first_out;
    }
    void nextOut(Arc &e) const {
      e.id = arcs[e.id].next_out;
    }

    void firstIn(Arc &e, const Node& v) const {
      e.id = ((nodes[v.id].first_out) ^ 1);
      if (e.id == -2) e.id = -1;
    }
    void nextIn(Arc &e) const {
      e.id = ((arcs[e.id ^ 1].next_out) ^ 1);
      if (e.id == -2) e.id = -1;
    }

    void firstInc(Edge &e, bool& d, const Node& v) const {
      int a = nodes[v.id].first_out;
      if (a != -1 ) {
        e.id = a / 2;
        d = ((a & 1) == 1);
      } else {
        e.id = -1;
        d = true;
      }
    }
    void nextInc(Edge &e, bool& d) const {
      int a = (arcs[(e.id * 2) | (d ? 1 : 0)].next_out);
      if (a != -1 ) {
        e.id = a / 2;
        d = ((a & 1) == 1);
      } else {
        e.id = -1;
        d = true;
      }
    }

    static int id(Node v) { return v.id; }
    static int id(Arc e) { return e.id; }
    static int id(Edge e) { return e.id; }

    static Node nodeFromId(int id) { return Node(id);}
    static Arc arcFromId(int id) { return Arc(id);}
    static Edge edgeFromId(int id) { return Edge(id);}

    bool valid(Node n) const {
      return n.id >= 0 && n.id < static_cast<int>(nodes.size()) &&
        nodes[n.id].prev != -2;
    }

    bool valid(Arc a) const {
      return a.id >= 0 && a.id < static_cast<int>(arcs.size()) &&
        arcs[a.id].prev_out != -2;
    }

    bool valid(Edge e) const {
      return e.id >= 0 && 2 * e.id < static_cast<int>(arcs.size()) &&
        arcs[2 * e.id].prev_out != -2;
    }

    Node addNode() {
      int n;

      if(first_free_node==-1) {
        n = nodes.size();
        nodes.push_back(NodeT());
      } else {
        n = first_free_node;
        first_free_node = nodes[n].next;
      }

      nodes[n].next = first_node;
      if (first_node != -1) nodes[first_node].prev = n;
      first_node = n;
      nodes[n].prev = -1;

      nodes[n].first_out = -1;

      return Node(n);
    }

    Edge addEdge(Node u, Node v) {
      int n;

      if (first_free_arc == -1) {
        n = arcs.size();
        arcs.push_back(ArcT());
        arcs.push_back(ArcT());
      } else {
        n = first_free_arc;
        first_free_arc = arcs[n].next_out;
      }

      arcs[n].target = u.id;
      arcs[n | 1].target = v.id;

      arcs[n].next_out = nodes[v.id].first_out;
      if (nodes[v.id].first_out != -1) {
        arcs[nodes[v.id].first_out].prev_out = n;
      }
      arcs[n].prev_out = -1;
      nodes[v.id].first_out = n;

      arcs[n | 1].next_out = nodes[u.id].first_out;
      if (nodes[u.id].first_out != -1) {
        arcs[nodes[u.id].first_out].prev_out = (n | 1);
      }
      arcs[n | 1].prev_out = -1;
      nodes[u.id].first_out = (n | 1);

      return Edge(n / 2);
    }

    void erase(const Node& node) {
      int n = node.id;

      if(nodes[n].next != -1) {
        nodes[nodes[n].next].prev = nodes[n].prev;
      }

      if(nodes[n].prev != -1) {
        nodes[nodes[n].prev].next = nodes[n].next;
      } else {
        first_node = nodes[n].next;
      }

      nodes[n].next = first_free_node;
      first_free_node = n;
      nodes[n].prev = -2;
    }

    void erase(const Edge& edge) {
      int n = edge.id * 2;

      if (arcs[n].next_out != -1) {
        arcs[arcs[n].next_out].prev_out = arcs[n].prev_out;
      }

      if (arcs[n].prev_out != -1) {
        arcs[arcs[n].prev_out].next_out = arcs[n].next_out;
      } else {
        nodes[arcs[n | 1].target].first_out = arcs[n].next_out;
      }

      if (arcs[n | 1].next_out != -1) {
        arcs[arcs[n | 1].next_out].prev_out = arcs[n | 1].prev_out;
      }

      if (arcs[n | 1].prev_out != -1) {
        arcs[arcs[n | 1].prev_out].next_out = arcs[n | 1].next_out;
      } else {
        nodes[arcs[n].target].first_out = arcs[n | 1].next_out;
      }

      arcs[n].next_out = first_free_arc;
      first_free_arc = n;
      arcs[n].prev_out = -2;
      arcs[n | 1].prev_out = -2;

    }

    void clear() {
      arcs.clear();
      nodes.clear();
      first_node = first_free_node = first_free_arc = -1;
    }

  protected:

    void changeV(Edge e, Node n) {
      if(arcs[2 * e.id].next_out != -1) {
        arcs[arcs[2 * e.id].next_out].prev_out = arcs[2 * e.id].prev_out;
      }
      if(arcs[2 * e.id].prev_out != -1) {
        arcs[arcs[2 * e.id].prev_out].next_out =
          arcs[2 * e.id].next_out;
      } else {
        nodes[arcs[(2 * e.id) | 1].target].first_out =
          arcs[2 * e.id].next_out;
      }

      if (nodes[n.id].first_out != -1) {
        arcs[nodes[n.id].first_out].prev_out = 2 * e.id;
      }
      arcs[(2 * e.id) | 1].target = n.id;
      arcs[2 * e.id].prev_out = -1;
      arcs[2 * e.id].next_out = nodes[n.id].first_out;
      nodes[n.id].first_out = 2 * e.id;
    }

    void changeU(Edge e, Node n) {
      if(arcs[(2 * e.id) | 1].next_out != -1) {
        arcs[arcs[(2 * e.id) | 1].next_out].prev_out =
          arcs[(2 * e.id) | 1].prev_out;
      }
      if(arcs[(2 * e.id) | 1].prev_out != -1) {
        arcs[arcs[(2 * e.id) | 1].prev_out].next_out =
          arcs[(2 * e.id) | 1].next_out;
      } else {
        nodes[arcs[2 * e.id].target].first_out =
          arcs[(2 * e.id) | 1].next_out;
      }

      if (nodes[n.id].first_out != -1) {
        arcs[nodes[n.id].first_out].prev_out = ((2 * e.id) | 1);
      }
      arcs[2 * e.id].target = n.id;
      arcs[(2 * e.id) | 1].prev_out = -1;
      arcs[(2 * e.id) | 1].next_out = nodes[n.id].first_out;
      nodes[n.id].first_out = ((2 * e.id) | 1);
    }

  };

  typedef GraphExtender<ListGraphBase> ExtendedListGraphBase;


  /// \addtogroup graphs
  /// @{

  ///A general undirected graph structure.

  ///\ref ListGraph is a simple and fast <em>undirected graph</em>
  ///implementation based on static linked lists that are stored in
  ///\c std::vector structures.
  ///
  ///It conforms to the \ref concepts::Graph "Graph concept" and it
  ///also provides several useful additional functionalities.
  ///Most of the member functions and nested classes are documented
  ///only in the concept class.
  ///
  ///An important extra feature of this graph implementation is that
  ///its maps are real \ref concepts::ReferenceMap "reference map"s.
  ///
  ///\sa concepts::Graph

  class ListGraph : public ExtendedListGraphBase {
  private:
    ///ListGraph is \e not copy constructible. Use copyGraph() instead.

    ///ListGraph is \e not copy constructible. Use copyGraph() instead.
    ///
    ListGraph(const ListGraph &) :ExtendedListGraphBase()  {};
    ///\brief Assignment of ListGraph to another one is \e not allowed.
    ///Use copyGraph() instead.

    ///Assignment of ListGraph to another one is \e not allowed.
    ///Use copyGraph() instead.
    void operator=(const ListGraph &) {}
  public:
    /// Constructor

    /// Constructor.
    ///
    ListGraph() {}

    typedef ExtendedListGraphBase Parent;

    typedef Parent::OutArcIt IncEdgeIt;

    /// \brief Add a new node to the graph.
    ///
    /// Add a new node to the graph.
    /// \return the new node.
    Node addNode() { return Parent::addNode(); }

    /// \brief Add a new edge to the graph.
    ///
    /// Add a new edge to the graph with source node \c s
    /// and target node \c t.
    /// \return the new edge.
    Edge addEdge(const Node& s, const Node& t) {
      return Parent::addEdge(s, t);
    }

    /// \brief Erase a node from the graph.
    ///
    /// Erase a node from the graph.
    ///
    void erase(const Node& n) { Parent::erase(n); }

    /// \brief Erase an edge from the graph.
    ///
    /// Erase an edge from the graph.
    ///
    void erase(const Edge& e) { Parent::erase(e); }
    /// Node validity check

    /// This function gives back true if the given node is valid,
    /// ie. it is a real node of the graph.
    ///
    /// \warning A Node pointing to a removed item
    /// could become valid again later if new nodes are
    /// added to the graph.
    bool valid(Node n) const { return Parent::valid(n); }
    /// Arc validity check

    /// This function gives back true if the given arc is valid,
    /// ie. it is a real arc of the graph.
    ///
    /// \warning An Arc pointing to a removed item
    /// could become valid again later if new edges are
    /// added to the graph.
    bool valid(Arc a) const { return Parent::valid(a); }
    /// Edge validity check

    /// This function gives back true if the given edge is valid,
    /// ie. it is a real arc of the graph.
    ///
    /// \warning A Edge pointing to a removed item
    /// could become valid again later if new edges are
    /// added to the graph.
    bool valid(Edge e) const { return Parent::valid(e); }
    /// \brief Change the end \c u of \c e to \c n
    ///
    /// This function changes the end \c u of \c e to node \c n.
    ///
    ///\note The <tt>EdgeIt</tt>s and <tt>ArcIt</tt>s referencing the
    ///changed edge are invalidated and if the changed node is the
    ///base node of an iterator then this iterator is also
    ///invalidated.
    ///
    ///\warning This functionality cannot be used together with the
    ///Snapshot feature.
    void changeU(Edge e, Node n) {
      Parent::changeU(e,n);
    }
    /// \brief Change the end \c v of \c e to \c n
    ///
    /// This function changes the end \c v of \c e to \c n.
    ///
    ///\note The <tt>EdgeIt</tt>s referencing the changed edge remain
    ///valid, however <tt>ArcIt</tt>s and if the changed node is the
    ///base node of an iterator then this iterator is invalidated.
    ///
    ///\warning This functionality cannot be used together with the
    ///Snapshot feature.
    void changeV(Edge e, Node n) {
      Parent::changeV(e,n);
    }
    /// \brief Contract two nodes.
    ///
    /// This function contracts two nodes.
    /// Node \p b will be removed but instead of deleting
    /// its neighboring arcs, they will be joined to \p a.
    /// The last parameter \p r controls whether to remove loops. \c true
    /// means that loops will be removed.
    ///
    /// \note The <tt>ArcIt</tt>s referencing a moved arc remain
    /// valid.
    ///
    ///\warning This functionality cannot be used together with the
    ///Snapshot feature.
    void contract(Node a, Node b, bool r = true) {
      for(IncEdgeIt e(*this, b); e!=INVALID;) {
        IncEdgeIt f = e; ++f;
        if (r && runningNode(e) == a) {
          erase(e);
        } else if (u(e) == b) {
          changeU(e, a);
        } else {
          changeV(e, a);
        }
        e = f;
      }
      erase(b);
    }


    /// \brief Class to make a snapshot of the graph and restore
    /// it later.
    ///
    /// Class to make a snapshot of the graph and restore it later.
    ///
    /// The newly added nodes and edges can be removed
    /// using the restore() function.
    ///
    /// \warning Edge and node deletions and other modifications
    /// (e.g. changing nodes of edges, contracting nodes) cannot be
    /// restored. These events invalidate the snapshot.
    class Snapshot {
    protected:

      typedef Parent::NodeNotifier NodeNotifier;

      class NodeObserverProxy : public NodeNotifier::ObserverBase {
      public:

        NodeObserverProxy(Snapshot& _snapshot)
          : snapshot(_snapshot) {}

        using NodeNotifier::ObserverBase::attach;
        using NodeNotifier::ObserverBase::detach;
        using NodeNotifier::ObserverBase::attached;

      protected:

        virtual void add(const Node& node) {
          snapshot.addNode(node);
        }
        virtual void add(const std::vector<Node>& nodes) {
          for (int i = nodes.size() - 1; i >= 0; ++i) {
            snapshot.addNode(nodes[i]);
          }
        }
        virtual void erase(const Node& node) {
          snapshot.eraseNode(node);
        }
        virtual void erase(const std::vector<Node>& nodes) {
          for (int i = 0; i < int(nodes.size()); ++i) {
            snapshot.eraseNode(nodes[i]);
          }
        }
        virtual void build() {
          Node node;
          std::vector<Node> nodes;
          for (notifier()->first(node); node != INVALID;
               notifier()->next(node)) {
            nodes.push_back(node);
          }
          for (int i = nodes.size() - 1; i >= 0; --i) {
            snapshot.addNode(nodes[i]);
          }
        }
        virtual void clear() {
          Node node;
          for (notifier()->first(node); node != INVALID;
               notifier()->next(node)) {
            snapshot.eraseNode(node);
          }
        }

        Snapshot& snapshot;
      };

      class EdgeObserverProxy : public EdgeNotifier::ObserverBase {
      public:

        EdgeObserverProxy(Snapshot& _snapshot)
          : snapshot(_snapshot) {}

        using EdgeNotifier::ObserverBase::attach;
        using EdgeNotifier::ObserverBase::detach;
        using EdgeNotifier::ObserverBase::attached;

      protected:

        virtual void add(const Edge& edge) {
          snapshot.addEdge(edge);
        }
        virtual void add(const std::vector<Edge>& edges) {
          for (int i = edges.size() - 1; i >= 0; ++i) {
            snapshot.addEdge(edges[i]);
          }
        }
        virtual void erase(const Edge& edge) {
          snapshot.eraseEdge(edge);
        }
        virtual void erase(const std::vector<Edge>& edges) {
          for (int i = 0; i < int(edges.size()); ++i) {
            snapshot.eraseEdge(edges[i]);
          }
        }
        virtual void build() {
          Edge edge;
          std::vector<Edge> edges;
          for (notifier()->first(edge); edge != INVALID;
               notifier()->next(edge)) {
            edges.push_back(edge);
          }
          for (int i = edges.size() - 1; i >= 0; --i) {
            snapshot.addEdge(edges[i]);
          }
        }
        virtual void clear() {
          Edge edge;
          for (notifier()->first(edge); edge != INVALID;
               notifier()->next(edge)) {
            snapshot.eraseEdge(edge);
          }
        }

        Snapshot& snapshot;
      };

      ListGraph *graph;

      NodeObserverProxy node_observer_proxy;
      EdgeObserverProxy edge_observer_proxy;

      std::list<Node> added_nodes;
      std::list<Edge> added_edges;


      void addNode(const Node& node) {
        added_nodes.push_front(node);
      }
      void eraseNode(const Node& node) {
        std::list<Node>::iterator it =
          std::find(added_nodes.begin(), added_nodes.end(), node);
        if (it == added_nodes.end()) {
          clear();
          edge_observer_proxy.detach();
          throw NodeNotifier::ImmediateDetach();
        } else {
          added_nodes.erase(it);
        }
      }

      void addEdge(const Edge& edge) {
        added_edges.push_front(edge);
      }
      void eraseEdge(const Edge& edge) {
        std::list<Edge>::iterator it =
          std::find(added_edges.begin(), added_edges.end(), edge);
        if (it == added_edges.end()) {
          clear();
          node_observer_proxy.detach();
          throw EdgeNotifier::ImmediateDetach();
        } else {
          added_edges.erase(it);
        }
      }

      void attach(ListGraph &_graph) {
        graph = &_graph;
        node_observer_proxy.attach(graph->notifier(Node()));
        edge_observer_proxy.attach(graph->notifier(Edge()));
      }

      void detach() {
        node_observer_proxy.detach();
        edge_observer_proxy.detach();
      }

      bool attached() const {
        return node_observer_proxy.attached();
      }

      void clear() {
        added_nodes.clear();
        added_edges.clear();
      }

    public:

      /// \brief Default constructor.
      ///
      /// Default constructor.
      /// To actually make a snapshot you must call save().
      Snapshot()
        : graph(0), node_observer_proxy(*this),
          edge_observer_proxy(*this) {}

      /// \brief Constructor that immediately makes a snapshot.
      ///
      /// This constructor immediately makes a snapshot of the graph.
      /// \param _graph The graph we make a snapshot of.
      Snapshot(ListGraph &_graph)
        : node_observer_proxy(*this),
          edge_observer_proxy(*this) {
        attach(_graph);
      }

      /// \brief Make a snapshot.
      ///
      /// Make a snapshot of the graph.
      ///
      /// This function can be called more than once. In case of a repeated
      /// call, the previous snapshot gets lost.
      /// \param _graph The graph we make the snapshot of.
      void save(ListGraph &_graph) {
        if (attached()) {
          detach();
          clear();
        }
        attach(_graph);
      }

      /// \brief Undo the changes until the last snapshot.
      //
      /// Undo the changes until the last snapshot created by save().
      void restore() {
        detach();
        for(std::list<Edge>::iterator it = added_edges.begin();
            it != added_edges.end(); ++it) {
          graph->erase(*it);
        }
        for(std::list<Node>::iterator it = added_nodes.begin();
            it != added_nodes.end(); ++it) {
          graph->erase(*it);
        }
        clear();
      }

      /// \brief Gives back true when the snapshot is valid.
      ///
      /// Gives back true when the snapshot is valid.
      bool valid() const {
        return attached();
      }
    };
  };

  /// @}
} //namespace lemon


#endif
