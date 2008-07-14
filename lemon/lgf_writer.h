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

///\ingroup lemon_io
///\file
///\brief \ref lgf-format "Lemon Graph Format" writer.


#ifndef LEMON_LGF_WRITER_H
#define LEMON_LGF_WRITER_H

#include <iostream>
#include <fstream>
#include <sstream>

#include <algorithm>

#include <vector>
#include <functional>

#include <lemon/assert.h>
#include <lemon/graph_utils.h>

namespace lemon {

  namespace _writer_bits {

    template <typename Value>
    struct DefaultConverter {
      std::string operator()(const Value& value) {
        std::ostringstream os;
        os << value;
        return os.str();
      }
    };

    template <typename T>
    bool operator<(const T&, const T&) {
      throw DataFormatError("Label map is not comparable");
    }

    template <typename _Map>
    class MapLess {
    public:
      typedef _Map Map;
      typedef typename Map::Key Item;

    private:
      const Map& _map;

    public:
      MapLess(const Map& map) : _map(map) {}

      bool operator()(const Item& left, const Item& right) {
        return _map[left] < _map[right];
      }
    };

    template <typename _Graph, bool _dir, typename _Map>
    class GraphArcMapLess {
    public:
      typedef _Map Map;
      typedef _Graph Graph;
      typedef typename Graph::Edge Item;

    private:
      const Graph& _graph;
      const Map& _map;

    public:
      GraphArcMapLess(const Graph& graph, const Map& map)
        : _graph(graph), _map(map) {}

      bool operator()(const Item& left, const Item& right) {
        return _map[_graph.direct(left, _dir)] <
          _map[_graph.direct(right, _dir)];
      }
    };

    template <typename _Item>
    class MapStorageBase {
    public:
      typedef _Item Item;

    public:
      MapStorageBase() {}
      virtual ~MapStorageBase() {}

      virtual std::string get(const Item& item) = 0;
      virtual void sort(std::vector<Item>&) = 0;
    };

    template <typename _Item, typename _Map,
              typename _Converter = DefaultConverter<typename _Map::Value> >
    class MapStorage : public MapStorageBase<_Item> {
    public:
      typedef _Map Map;
      typedef _Converter Converter;
      typedef _Item Item;

    private:
      const Map& _map;
      Converter _converter;

    public:
      MapStorage(const Map& map, const Converter& converter = Converter())
        : _map(map), _converter(converter) {}
      virtual ~MapStorage() {}

      virtual std::string get(const Item& item) {
        return _converter(_map[item]);
      }
      virtual void sort(std::vector<Item>& items) {
        MapLess<Map> less(_map);
        std::sort(items.begin(), items.end(), less);
      }
    };

    template <typename _Graph, bool _dir, typename _Map,
              typename _Converter = DefaultConverter<typename _Map::Value> >
    class GraphArcMapStorage : public MapStorageBase<typename _Graph::Edge> {
    public:
      typedef _Map Map;
      typedef _Converter Converter;
      typedef _Graph Graph;
      typedef typename Graph::Edge Item;
      static const bool dir = _dir;

    private:
      const Graph& _graph;
      const Map& _map;
      Converter _converter;

    public:
      GraphArcMapStorage(const Graph& graph, const Map& map,
                         const Converter& converter = Converter())
        : _graph(graph), _map(map), _converter(converter) {}
      virtual ~GraphArcMapStorage() {}

      virtual std::string get(const Item& item) {
        return _converter(_map[_graph.direct(item, dir)]);
      }
      virtual void sort(std::vector<Item>& items) {
        GraphArcMapLess<Graph, dir, Map> less(_graph, _map);
        std::sort(items.begin(), items.end(), less);
      }
    };

    class ValueStorageBase {
    public:
      ValueStorageBase() {}
      virtual ~ValueStorageBase() {}

      virtual std::string get() = 0;
    };

    template <typename _Value, typename _Converter = DefaultConverter<_Value> >
    class ValueStorage : public ValueStorageBase {
    public:
      typedef _Value Value;
      typedef _Converter Converter;

    private:
      const Value& _value;
      Converter _converter;

    public:
      ValueStorage(const Value& value, const Converter& converter = Converter())
        : _value(value), _converter(converter) {}

      virtual std::string get() {
        return _converter(_value);
      }
    };

    template <typename Value>
    struct MapLookUpConverter {
      const std::map<Value, std::string>& _map;

      MapLookUpConverter(const std::map<Value, std::string>& map)
        : _map(map) {}

      std::string operator()(const Value& str) {
        typename std::map<Value, std::string>::const_iterator it =
          _map.find(str);
        if (it == _map.end()) {
          throw DataFormatError("Item not found");
        }
        return it->second;
      }
    };

    template <typename Graph>
    struct GraphArcLookUpConverter {
      const Graph& _graph;
      const std::map<typename Graph::Edge, std::string>& _map;

      GraphArcLookUpConverter(const Graph& graph,
                              const std::map<typename Graph::Edge,
                                             std::string>& map)
        : _graph(graph), _map(map) {}

      std::string operator()(const typename Graph::Arc& val) {
        typename std::map<typename Graph::Edge, std::string>
          ::const_iterator it = _map.find(val);
        if (it == _map.end()) {
          throw DataFormatError("Item not found");
        }
        return (_graph.direction(val) ? '+' : '-') + it->second;
      }
    };

    inline bool isWhiteSpace(char c) {
      return c == ' ' || c == '\t' || c == '\v' ||
        c == '\n' || c == '\r' || c == '\f';
    }

    inline bool isEscaped(char c) {
      return c == '\\' || c == '\"' || c == '\'' ||
        c == '\a' || c == '\b';
    }

    inline static void writeEscape(std::ostream& os, char c) {
      switch (c) {
      case '\\':
        os << "\\\\";
        return;
      case '\"':
        os << "\\\"";
        return;
      case '\a':
        os << "\\a";
        return;
      case '\b':
        os << "\\b";
        return;
      case '\f':
        os << "\\f";
        return;
      case '\r':
        os << "\\r";
        return;
      case '\n':
        os << "\\n";
        return;
      case '\t':
        os << "\\t";
        return;
      case '\v':
        os << "\\v";
        return;
      default:
        if (c < 0x20) {
          std::ios::fmtflags flags = os.flags();
          os << '\\' << std::oct << static_cast<int>(c);
          os.flags(flags);
        } else {
          os << c;
        }
        return;
      }
    }

    inline bool requireEscape(const std::string& str) {
      if (str.empty() || str[0] == '@') return true;
      std::istringstream is(str);
      char c;
      while (is.get(c)) {
        if (isWhiteSpace(c) || isEscaped(c)) {
          return true;
        }
      }
      return false;
    }

    inline std::ostream& writeToken(std::ostream& os, const std::string& str) {

      if (requireEscape(str)) {
        os << '\"';
        for (std::string::const_iterator it = str.begin();
             it != str.end(); ++it) {
          writeEscape(os, *it);
        }
        os << '\"';
      } else {
        os << str;
      }
      return os;
    }

  }

  template <typename Digraph>
  class DigraphWriter;

  template <typename Digraph>
  DigraphWriter<Digraph> digraphWriter(std::ostream& os,
                                       const Digraph& digraph);

  template <typename Digraph>
  DigraphWriter<Digraph> digraphWriter(const std::string& fn,
                                       const Digraph& digraph);

  template <typename Digraph>
  DigraphWriter<Digraph> digraphWriter(const char *fn,
                                       const Digraph& digraph);

  /// \ingroup lemon_io
  ///
  /// \brief \ref lgf-format "LGF" writer for directed graphs
  ///
  /// This utility writes an \ref lgf-format "LGF" file.
  ///
  /// The writing method does a batch processing. The user creates a
  /// writer object, then various writing rules can be added to the
  /// writer, and eventually the writing is executed with the \c run()
  /// member function. A map writing rule can be added to the writer
  /// with the \c nodeMap() or \c arcMap() members. An optional
  /// converter parameter can also be added as a standard functor
  /// converting from the value type of the map to \c std::string. If it
  /// is set, it will determine how the value type of the map is written to
  /// the output stream. If the functor is not set, then a default
  /// conversion will be used. The \c attribute(), \c node() and \c
  /// arc() functions are used to add attribute writing rules.
  ///
  ///\code
  /// DigraphWriter<Digraph>(std::cout, digraph).
  ///   nodeMap("coordinates", coord_map).
  ///   nodeMap("size", size).
  ///   nodeMap("title", title).
  ///   arcMap("capacity", cap_map).
  ///   node("source", src).
  ///   node("target", trg).
  ///   attribute("caption", caption).
  ///   run();
  ///\endcode
  ///
  ///
  /// By default, the writer does not write additional captions to the
  /// sections, but they can be give as an optional parameter of
  /// the \c nodes(), \c arcs() or \c
  /// attributes() functions.
  ///
  /// The \c skipNodes() and \c skipArcs() functions forbid the
  /// writing of the sections. If two arc sections should be written
  /// to the output, it can be done in two passes, the first pass
  /// writes the node section and the first arc section, then the
  /// second pass skips the node section and writes just the arc
  /// section to the stream. The output stream can be retrieved with
  /// the \c ostream() function, hence the second pass can append its
  /// output to the output of the first pass.
  template <typename _Digraph>
  class DigraphWriter {
  public:

    typedef _Digraph Digraph;
    TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);

  private:


    std::ostream* _os;
    bool local_os;

    const Digraph& _digraph;

    std::string _nodes_caption;
    std::string _arcs_caption;
    std::string _attributes_caption;

    typedef std::map<Node, std::string> NodeIndex;
    NodeIndex _node_index;
    typedef std::map<Arc, std::string> ArcIndex;
    ArcIndex _arc_index;

    typedef std::vector<std::pair<std::string,
      _writer_bits::MapStorageBase<Node>* > > NodeMaps;
    NodeMaps _node_maps;

    typedef std::vector<std::pair<std::string,
      _writer_bits::MapStorageBase<Arc>* > >ArcMaps;
    ArcMaps _arc_maps;

    typedef std::vector<std::pair<std::string,
      _writer_bits::ValueStorageBase*> > Attributes;
    Attributes _attributes;

    bool _skip_nodes;
    bool _skip_arcs;

  public:

    /// \brief Constructor
    ///
    /// Construct a directed graph writer, which writes to the given
    /// output stream.
    DigraphWriter(std::ostream& is, const Digraph& digraph)
      : _os(&is), local_os(false), _digraph(digraph),
        _skip_nodes(false), _skip_arcs(false) {}

    /// \brief Constructor
    ///
    /// Construct a directed graph writer, which writes to the given
    /// output file.
    DigraphWriter(const std::string& fn, const Digraph& digraph)
      : _os(new std::ofstream(fn.c_str())), local_os(true), _digraph(digraph),
        _skip_nodes(false), _skip_arcs(false) {}

    /// \brief Constructor
    ///
    /// Construct a directed graph writer, which writes to the given
    /// output file.
    DigraphWriter(const char* fn, const Digraph& digraph)
      : _os(new std::ofstream(fn)), local_os(true), _digraph(digraph),
        _skip_nodes(false), _skip_arcs(false) {}

    /// \brief Destructor
    ~DigraphWriter() {
      for (typename NodeMaps::iterator it = _node_maps.begin();
           it != _node_maps.end(); ++it) {
        delete it->second;
      }

      for (typename ArcMaps::iterator it = _arc_maps.begin();
           it != _arc_maps.end(); ++it) {
        delete it->second;
      }

      for (typename Attributes::iterator it = _attributes.begin();
           it != _attributes.end(); ++it) {
        delete it->second;
      }

      if (local_os) {
        delete _os;
      }
    }

  private:

    friend DigraphWriter<Digraph> digraphWriter<>(std::ostream& os,
                                                  const Digraph& digraph);
    friend DigraphWriter<Digraph> digraphWriter<>(const std::string& fn,
                                                  const Digraph& digraph);
    friend DigraphWriter<Digraph> digraphWriter<>(const char *fn,
                                                  const Digraph& digraph);

    DigraphWriter(DigraphWriter& other)
      : _os(other._os), local_os(other.local_os), _digraph(other._digraph),
        _skip_nodes(other._skip_nodes), _skip_arcs(other._skip_arcs) {

      other._os = 0;
      other.local_os = false;

      _node_index.swap(other._node_index);
      _arc_index.swap(other._arc_index);

      _node_maps.swap(other._node_maps);
      _arc_maps.swap(other._arc_maps);
      _attributes.swap(other._attributes);

      _nodes_caption = other._nodes_caption;
      _arcs_caption = other._arcs_caption;
      _attributes_caption = other._attributes_caption;
    }

    DigraphWriter& operator=(const DigraphWriter&);

  public:

    /// \name Writing rules
    /// @{

    /// \brief Node map writing rule
    ///
    /// Add a node map writing rule to the writer.
    template <typename Map>
    DigraphWriter& nodeMap(const std::string& caption, const Map& map) {
      checkConcept<concepts::ReadMap<Node, typename Map::Value>, Map>();
      _writer_bits::MapStorageBase<Node>* storage =
        new _writer_bits::MapStorage<Node, Map>(map);
      _node_maps.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Node map writing rule
    ///
    /// Add a node map writing rule with specialized converter to the
    /// writer.
    template <typename Map, typename Converter>
    DigraphWriter& nodeMap(const std::string& caption, const Map& map,
                           const Converter& converter = Converter()) {
      checkConcept<concepts::ReadMap<Node, typename Map::Value>, Map>();
      _writer_bits::MapStorageBase<Node>* storage =
        new _writer_bits::MapStorage<Node, Map, Converter>(map, converter);
      _node_maps.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Arc map writing rule
    ///
    /// Add an arc map writing rule to the writer.
    template <typename Map>
    DigraphWriter& arcMap(const std::string& caption, const Map& map) {
      checkConcept<concepts::ReadMap<Arc, typename Map::Value>, Map>();
      _writer_bits::MapStorageBase<Arc>* storage =
        new _writer_bits::MapStorage<Arc, Map>(map);
      _arc_maps.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Arc map writing rule
    ///
    /// Add an arc map writing rule with specialized converter to the
    /// writer.
    template <typename Map, typename Converter>
    DigraphWriter& arcMap(const std::string& caption, const Map& map,
                          const Converter& converter = Converter()) {
      checkConcept<concepts::ReadMap<Arc, typename Map::Value>, Map>();
      _writer_bits::MapStorageBase<Arc>* storage =
        new _writer_bits::MapStorage<Arc, Map, Converter>(map, converter);
      _arc_maps.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Attribute writing rule
    ///
    /// Add an attribute writing rule to the writer.
    template <typename Value>
    DigraphWriter& attribute(const std::string& caption, const Value& value) {
      _writer_bits::ValueStorageBase* storage =
        new _writer_bits::ValueStorage<Value>(value);
      _attributes.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Attribute writing rule
    ///
    /// Add an attribute writing rule with specialized converter to the
    /// writer.
    template <typename Value, typename Converter>
    DigraphWriter& attribute(const std::string& caption, const Value& value,
                             const Converter& converter = Converter()) {
      _writer_bits::ValueStorageBase* storage =
        new _writer_bits::ValueStorage<Value, Converter>(value, converter);
      _attributes.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Node writing rule
    ///
    /// Add a node writing rule to the writer.
    DigraphWriter& node(const std::string& caption, const Node& node) {
      typedef _writer_bits::MapLookUpConverter<Node> Converter;
      Converter converter(_node_index);
      _writer_bits::ValueStorageBase* storage =
        new _writer_bits::ValueStorage<Node, Converter>(node, converter);
      _attributes.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Arc writing rule
    ///
    /// Add an arc writing rule to writer.
    DigraphWriter& arc(const std::string& caption, const Arc& arc) {
      typedef _writer_bits::MapLookUpConverter<Arc> Converter;
      Converter converter(_arc_index);
      _writer_bits::ValueStorageBase* storage =
        new _writer_bits::ValueStorage<Arc, Converter>(arc, converter);
      _attributes.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \name Section captions
    /// @{

    /// \brief Add an additional caption to the \c \@nodes section
    ///
    /// Add an additional caption to the \c \@nodes section.
    DigraphWriter& nodes(const std::string& caption) {
      _nodes_caption = caption;
      return *this;
    }

    /// \brief Add an additional caption to the \c \@arcs section
    ///
    /// Add an additional caption to the \c \@arcs section.
    DigraphWriter& arcs(const std::string& caption) {
      _arcs_caption = caption;
      return *this;
    }

    /// \brief Add an additional caption to the \c \@attributes section
    ///
    /// Add an additional caption to the \c \@attributes section.
    DigraphWriter& attributes(const std::string& caption) {
      _attributes_caption = caption;
      return *this;
    }

    /// \name Skipping section
    /// @{

    /// \brief Skip writing the node set
    ///
    /// The \c \@nodes section will not be written to the stream.
    DigraphWriter& skipNodes() {
      LEMON_ASSERT(!_skip_nodes, "Multiple usage of skipNodes() member");
      _skip_nodes = true;
      return *this;
    }

    /// \brief Skip writing arc set
    ///
    /// The \c \@arcs section will not be written to the stream.
    DigraphWriter& skipArcs() {
      LEMON_ASSERT(!_skip_arcs, "Multiple usage of skipArcs() member");
      _skip_arcs = true;
      return *this;
    }

    /// @}

  private:

    void writeNodes() {
      _writer_bits::MapStorageBase<Node>* label = 0;
      for (typename NodeMaps::iterator it = _node_maps.begin();
           it != _node_maps.end(); ++it) {
        if (it->first == "label") {
          label = it->second;
          break;
        }
      }

      *_os << "@nodes";
      if (!_nodes_caption.empty()) {
        _writer_bits::writeToken(*_os << ' ', _nodes_caption);
      }
      *_os << std::endl;

      if (label == 0) {
        *_os << "label" << '\t';
      }
      for (typename NodeMaps::iterator it = _node_maps.begin();
           it != _node_maps.end(); ++it) {
        _writer_bits::writeToken(*_os, it->first) << '\t';
      }
      *_os << std::endl;

      std::vector<Node> nodes;
      for (NodeIt n(_digraph); n != INVALID; ++n) {
        nodes.push_back(n);
      }

      if (label == 0) {
        IdMap<Digraph, Node> id_map(_digraph);
        _writer_bits::MapLess<IdMap<Digraph, Node> > id_less(id_map);
        std::sort(nodes.begin(), nodes.end(), id_less);
      } else {
        label->sort(nodes);
      }

      for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        Node n = nodes[i];
        if (label == 0) {
          std::ostringstream os;
          os << _digraph.id(n);
          _writer_bits::writeToken(*_os, os.str());
          *_os << '\t';
          _node_index.insert(std::make_pair(n, os.str()));
        }
        for (typename NodeMaps::iterator it = _node_maps.begin();
             it != _node_maps.end(); ++it) {
          std::string value = it->second->get(n);
          _writer_bits::writeToken(*_os, value);
          if (it->first == "label") {
            _node_index.insert(std::make_pair(n, value));
          }
          *_os << '\t';
        }
        *_os << std::endl;
      }
    }

    void createNodeIndex() {
      _writer_bits::MapStorageBase<Node>* label = 0;
      for (typename NodeMaps::iterator it = _node_maps.begin();
           it != _node_maps.end(); ++it) {
        if (it->first == "label") {
          label = it->second;
          break;
        }
      }

      if (label == 0) {
        for (NodeIt n(_digraph); n != INVALID; ++n) {
          std::ostringstream os;
          os << _digraph.id(n);
          _node_index.insert(std::make_pair(n, os.str()));
        }
      } else {
        for (NodeIt n(_digraph); n != INVALID; ++n) {
          std::string value = label->get(n);
          _node_index.insert(std::make_pair(n, value));
        }
      }
    }

    void writeArcs() {
      _writer_bits::MapStorageBase<Arc>* label = 0;
      for (typename ArcMaps::iterator it = _arc_maps.begin();
           it != _arc_maps.end(); ++it) {
        if (it->first == "label") {
          label = it->second;
          break;
        }
      }

      *_os << "@arcs";
      if (!_arcs_caption.empty()) {
        _writer_bits::writeToken(*_os << ' ', _arcs_caption);
      }
      *_os << std::endl;

      *_os << '\t' << '\t';
      if (label == 0) {
        *_os << "label" << '\t';
      }
      for (typename ArcMaps::iterator it = _arc_maps.begin();
           it != _arc_maps.end(); ++it) {
        _writer_bits::writeToken(*_os, it->first) << '\t';
      }
      *_os << std::endl;

      std::vector<Arc> arcs;
      for (ArcIt n(_digraph); n != INVALID; ++n) {
        arcs.push_back(n);
      }

      if (label == 0) {
        IdMap<Digraph, Arc> id_map(_digraph);
        _writer_bits::MapLess<IdMap<Digraph, Arc> > id_less(id_map);
        std::sort(arcs.begin(), arcs.end(), id_less);
      } else {
        label->sort(arcs);
      }

      for (int i = 0; i < static_cast<int>(arcs.size()); ++i) {
        Arc a = arcs[i];
        _writer_bits::writeToken(*_os, _node_index.
                                 find(_digraph.source(a))->second);
        *_os << '\t';
        _writer_bits::writeToken(*_os, _node_index.
                                 find(_digraph.target(a))->second);
        *_os << '\t';
        if (label == 0) {
          std::ostringstream os;
          os << _digraph.id(a);
          _writer_bits::writeToken(*_os, os.str());
          *_os << '\t';
          _arc_index.insert(std::make_pair(a, os.str()));
        }
        for (typename ArcMaps::iterator it = _arc_maps.begin();
             it != _arc_maps.end(); ++it) {
          std::string value = it->second->get(a);
          _writer_bits::writeToken(*_os, value);
          if (it->first == "label") {
            _arc_index.insert(std::make_pair(a, value));
          }
          *_os << '\t';
        }
        *_os << std::endl;
      }
    }

    void createArcIndex() {
      _writer_bits::MapStorageBase<Arc>* label = 0;
      for (typename ArcMaps::iterator it = _arc_maps.begin();
           it != _arc_maps.end(); ++it) {
        if (it->first == "label") {
          label = it->second;
          break;
        }
      }

      if (label == 0) {
        for (ArcIt a(_digraph); a != INVALID; ++a) {
          std::ostringstream os;
          os << _digraph.id(a);
          _arc_index.insert(std::make_pair(a, os.str()));
        }
      } else {
        for (ArcIt a(_digraph); a != INVALID; ++a) {
          std::string value = label->get(a);
          _arc_index.insert(std::make_pair(a, value));
        }
      }
    }

    void writeAttributes() {
      if (_attributes.empty()) return;
      *_os << "@attributes";
      if (!_attributes_caption.empty()) {
        _writer_bits::writeToken(*_os << ' ', _attributes_caption);
      }
      *_os << std::endl;
      for (typename Attributes::iterator it = _attributes.begin();
           it != _attributes.end(); ++it) {
        _writer_bits::writeToken(*_os, it->first) << ' ';
        _writer_bits::writeToken(*_os, it->second->get());
        *_os << std::endl;
      }
    }

  public:

    /// \name Execution of the writer
    /// @{

    /// \brief Start the batch processing
    ///
    /// This function starts the batch processing.
    void run() {
      if (!_skip_nodes) {
        writeNodes();
      } else {
        createNodeIndex();
      }
      if (!_skip_arcs) {
        writeArcs();
      } else {
        createArcIndex();
      }
      writeAttributes();
    }

    /// \brief Give back the stream of the writer
    ///
    /// Give back the stream of the writer.
    std::ostream& ostream() {
      return *_os;
    }

    /// @}
  };

  /// \brief Return a \ref DigraphWriter class
  ///
  /// This function just returns a \ref DigraphWriter class.
  /// \relates DigraphWriter
  template <typename Digraph>
  DigraphWriter<Digraph> digraphWriter(std::ostream& os,
                                       const Digraph& digraph) {
    DigraphWriter<Digraph> tmp(os, digraph);
    return tmp;
  }

  /// \brief Return a \ref DigraphWriter class
  ///
  /// This function just returns a \ref DigraphWriter class.
  /// \relates DigraphWriter
  template <typename Digraph>
  DigraphWriter<Digraph> digraphWriter(const std::string& fn,
                                       const Digraph& digraph) {
    DigraphWriter<Digraph> tmp(fn, digraph);
    return tmp;
  }

  /// \brief Return a \ref DigraphWriter class
  ///
  /// This function just returns a \ref DigraphWriter class.
  /// \relates DigraphWriter
  template <typename Digraph>
  DigraphWriter<Digraph> digraphWriter(const char* fn,
                                       const Digraph& digraph) {
    DigraphWriter<Digraph> tmp(fn, digraph);
    return tmp;
  }

  template <typename Graph>
  class GraphWriter;

  template <typename Graph>
  GraphWriter<Graph> graphWriter(std::ostream& os, const Graph& graph);

  template <typename Graph>
  GraphWriter<Graph> graphWriter(const std::string& fn, const Graph& graph);

  template <typename Graph>
  GraphWriter<Graph> graphWriter(const char *fn, const Graph& graph);

  /// \ingroup lemon_io
  ///
  /// \brief \ref lgf-format "LGF" writer for directed graphs
  ///
  /// This utility writes an \ref lgf-format "LGF" file.
  ///
  /// It can be used almost the same way as \c DigraphWriter.
  /// The only difference is that this class can handle edges and
  /// edge maps as well as arcs and arc maps.
  ///
  /// The arc maps are written into the file as two columns, the
  /// caption of the columns are the name of the map prefixed with \c
  /// '+' and \c '-'. The arcs are written into the \c \@attributes
  /// section as a \c '+' or a \c '-' prefix (depends on the direction
  /// of the arc) and the label of corresponding edge.
  template <typename _Graph>
  class GraphWriter {
  public:

    typedef _Graph Graph;
    TEMPLATE_GRAPH_TYPEDEFS(Graph);

  private:


    std::ostream* _os;
    bool local_os;

    Graph& _graph;

    std::string _nodes_caption;
    std::string _edges_caption;
    std::string _attributes_caption;

    typedef std::map<Node, std::string> NodeIndex;
    NodeIndex _node_index;
    typedef std::map<Edge, std::string> EdgeIndex;
    EdgeIndex _edge_index;

    typedef std::vector<std::pair<std::string,
      _writer_bits::MapStorageBase<Node>* > > NodeMaps;
    NodeMaps _node_maps;

    typedef std::vector<std::pair<std::string,
      _writer_bits::MapStorageBase<Edge>* > >EdgeMaps;
    EdgeMaps _edge_maps;

    typedef std::vector<std::pair<std::string,
      _writer_bits::ValueStorageBase*> > Attributes;
    Attributes _attributes;

    bool _skip_nodes;
    bool _skip_edges;

  public:

    /// \brief Constructor
    ///
    /// Construct a directed graph writer, which writes to the given
    /// output stream.
    GraphWriter(std::ostream& is, const Graph& graph)
      : _os(&is), local_os(false), _graph(graph),
        _skip_nodes(false), _skip_edges(false) {}

    /// \brief Constructor
    ///
    /// Construct a directed graph writer, which writes to the given
    /// output file.
    GraphWriter(const std::string& fn, const Graph& graph)
      : _os(new std::ofstream(fn.c_str())), local_os(true), _graph(graph),
        _skip_nodes(false), _skip_edges(false) {}

    /// \brief Constructor
    ///
    /// Construct a directed graph writer, which writes to the given
    /// output file.
    GraphWriter(const char* fn, const Graph& graph)
      : _os(new std::ofstream(fn)), local_os(true), _graph(graph),
        _skip_nodes(false), _skip_edges(false) {}

    /// \brief Destructor
    ~GraphWriter() {
      for (typename NodeMaps::iterator it = _node_maps.begin();
           it != _node_maps.end(); ++it) {
        delete it->second;
      }

      for (typename EdgeMaps::iterator it = _edge_maps.begin();
           it != _edge_maps.end(); ++it) {
        delete it->second;
      }

      for (typename Attributes::iterator it = _attributes.begin();
           it != _attributes.end(); ++it) {
        delete it->second;
      }

      if (local_os) {
        delete _os;
      }
    }

  private:

    friend GraphWriter<Graph> graphWriter<>(std::ostream& os,
                                            const Graph& graph);
    friend GraphWriter<Graph> graphWriter<>(const std::string& fn,
                                            const Graph& graph);
    friend GraphWriter<Graph> graphWriter<>(const char *fn,
                                            const Graph& graph);

    GraphWriter(GraphWriter& other)
      : _os(other._os), local_os(other.local_os), _graph(other._graph),
        _skip_nodes(other._skip_nodes), _skip_edges(other._skip_edges) {

      other._os = 0;
      other.local_os = false;

      _node_index.swap(other._node_index);
      _edge_index.swap(other._edge_index);

      _node_maps.swap(other._node_maps);
      _edge_maps.swap(other._edge_maps);
      _attributes.swap(other._attributes);

      _nodes_caption = other._nodes_caption;
      _edges_caption = other._edges_caption;
      _attributes_caption = other._attributes_caption;
    }

    GraphWriter& operator=(const GraphWriter&);

  public:

    /// \name Writing rules
    /// @{

    /// \brief Node map writing rule
    ///
    /// Add a node map writing rule to the writer.
    template <typename Map>
    GraphWriter& nodeMap(const std::string& caption, const Map& map) {
      checkConcept<concepts::ReadMap<Node, typename Map::Value>, Map>();
      _writer_bits::MapStorageBase<Node>* storage =
        new _writer_bits::MapStorage<Node, Map>(map);
      _node_maps.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Node map writing rule
    ///
    /// Add a node map writing rule with specialized converter to the
    /// writer.
    template <typename Map, typename Converter>
    GraphWriter& nodeMap(const std::string& caption, const Map& map,
                           const Converter& converter = Converter()) {
      checkConcept<concepts::ReadMap<Node, typename Map::Value>, Map>();
      _writer_bits::MapStorageBase<Node>* storage =
        new _writer_bits::MapStorage<Node, Map, Converter>(map, converter);
      _node_maps.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Edge map writing rule
    ///
    /// Add an edge map writing rule to the writer.
    template <typename Map>
    GraphWriter& edgeMap(const std::string& caption, const Map& map) {
      checkConcept<concepts::ReadMap<Edge, typename Map::Value>, Map>();
      _writer_bits::MapStorageBase<Edge>* storage =
        new _writer_bits::MapStorage<Edge, Map>(map);
      _edge_maps.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Edge map writing rule
    ///
    /// Add an edge map writing rule with specialized converter to the
    /// writer.
    template <typename Map, typename Converter>
    GraphWriter& edgeMap(const std::string& caption, const Map& map,
                          const Converter& converter = Converter()) {
      checkConcept<concepts::ReadMap<Edge, typename Map::Value>, Map>();
      _writer_bits::MapStorageBase<Edge>* storage =
        new _writer_bits::MapStorage<Edge, Map, Converter>(map, converter);
      _edge_maps.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Arc map writing rule
    ///
    /// Add an arc map writing rule to the writer.
    template <typename Map>
    GraphWriter& arcMap(const std::string& caption, const Map& map) {
      checkConcept<concepts::ReadMap<Arc, typename Map::Value>, Map>();
      _writer_bits::MapStorageBase<Edge>* forward_storage =
        new _writer_bits::GraphArcMapStorage<Graph, true, Map>(_graph, map);
      _edge_maps.push_back(std::make_pair('+' + caption, forward_storage));
      _writer_bits::MapStorageBase<Edge>* backward_storage =
        new _writer_bits::GraphArcMapStorage<Graph, false, Map>(_graph, map);
      _edge_maps.push_back(std::make_pair('-' + caption, backward_storage));
      return *this;
    }

    /// \brief Arc map writing rule
    ///
    /// Add an arc map writing rule with specialized converter to the
    /// writer.
    template <typename Map, typename Converter>
    GraphWriter& arcMap(const std::string& caption, const Map& map,
                          const Converter& converter = Converter()) {
      checkConcept<concepts::ReadMap<Arc, typename Map::Value>, Map>();
      _writer_bits::MapStorageBase<Edge>* forward_storage =
        new _writer_bits::GraphArcMapStorage<Graph, true, Map, Converter>
        (_graph, map, converter);
      _edge_maps.push_back(std::make_pair('+' + caption, forward_storage));
      _writer_bits::MapStorageBase<Edge>* backward_storage =
        new _writer_bits::GraphArcMapStorage<Graph, false, Map, Converter>
        (_graph, map, converter);
      _edge_maps.push_back(std::make_pair('-' + caption, backward_storage));
      return *this;
    }

    /// \brief Attribute writing rule
    ///
    /// Add an attribute writing rule to the writer.
    template <typename Value>
    GraphWriter& attribute(const std::string& caption, const Value& value) {
      _writer_bits::ValueStorageBase* storage =
        new _writer_bits::ValueStorage<Value>(value);
      _attributes.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Attribute writing rule
    ///
    /// Add an attribute writing rule with specialized converter to the
    /// writer.
    template <typename Value, typename Converter>
    GraphWriter& attribute(const std::string& caption, const Value& value,
                             const Converter& converter = Converter()) {
      _writer_bits::ValueStorageBase* storage =
        new _writer_bits::ValueStorage<Value, Converter>(value, converter);
      _attributes.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Node writing rule
    ///
    /// Add a node writing rule to the writer.
    GraphWriter& node(const std::string& caption, const Node& node) {
      typedef _writer_bits::MapLookUpConverter<Node> Converter;
      Converter converter(_node_index);
      _writer_bits::ValueStorageBase* storage =
        new _writer_bits::ValueStorage<Node, Converter>(node, converter);
      _attributes.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Edge writing rule
    ///
    /// Add an edge writing rule to writer.
    GraphWriter& edge(const std::string& caption, const Edge& edge) {
      typedef _writer_bits::MapLookUpConverter<Edge> Converter;
      Converter converter(_edge_index);
      _writer_bits::ValueStorageBase* storage =
        new _writer_bits::ValueStorage<Edge, Converter>(edge, converter);
      _attributes.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Arc writing rule
    ///
    /// Add an arc writing rule to writer.
    GraphWriter& arc(const std::string& caption, const Arc& arc) {
      typedef _writer_bits::GraphArcLookUpConverter<Graph> Converter;
      Converter converter(_graph, _edge_index);
      _writer_bits::ValueStorageBase* storage =
        new _writer_bits::ValueStorage<Arc, Converter>(arc, converter);
      _attributes.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \name Section captions
    /// @{

    /// \brief Add an additional caption to the \c \@nodes section
    ///
    /// Add an additional caption to the \c \@nodes section.
    GraphWriter& nodes(const std::string& caption) {
      _nodes_caption = caption;
      return *this;
    }

    /// \brief Add an additional caption to the \c \@arcs section
    ///
    /// Add an additional caption to the \c \@arcs section.
    GraphWriter& edges(const std::string& caption) {
      _edges_caption = caption;
      return *this;
    }

    /// \brief Add an additional caption to the \c \@attributes section
    ///
    /// Add an additional caption to the \c \@attributes section.
    GraphWriter& attributes(const std::string& caption) {
      _attributes_caption = caption;
      return *this;
    }

    /// \name Skipping section
    /// @{

    /// \brief Skip writing the node set
    ///
    /// The \c \@nodes section will not be written to the stream.
    GraphWriter& skipNodes() {
      LEMON_ASSERT(!_skip_nodes, "Multiple usage of skipNodes() member");
      _skip_nodes = true;
      return *this;
    }

    /// \brief Skip writing edge set
    ///
    /// The \c \@edges section will not be written to the stream.
    GraphWriter& skipEdges() {
      LEMON_ASSERT(!_skip_edges, "Multiple usage of skipEdges() member");
      _skip_edges = true;
      return *this;
    }

    /// @}

  private:

    void writeNodes() {
      _writer_bits::MapStorageBase<Node>* label = 0;
      for (typename NodeMaps::iterator it = _node_maps.begin();
           it != _node_maps.end(); ++it) {
        if (it->first == "label") {
          label = it->second;
          break;
        }
      }

      *_os << "@nodes";
      if (!_nodes_caption.empty()) {
        _writer_bits::writeToken(*_os << ' ', _nodes_caption);
      }
      *_os << std::endl;

      if (label == 0) {
        *_os << "label" << '\t';
      }
      for (typename NodeMaps::iterator it = _node_maps.begin();
           it != _node_maps.end(); ++it) {
        _writer_bits::writeToken(*_os, it->first) << '\t';
      }
      *_os << std::endl;

      std::vector<Node> nodes;
      for (NodeIt n(_graph); n != INVALID; ++n) {
        nodes.push_back(n);
      }

      if (label == 0) {
        IdMap<Graph, Node> id_map(_graph);
        _writer_bits::MapLess<IdMap<Graph, Node> > id_less(id_map);
        std::sort(nodes.begin(), nodes.end(), id_less);
      } else {
        label->sort(nodes);
      }

      for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        Node n = nodes[i];
        if (label == 0) {
          std::ostringstream os;
          os << _graph.id(n);
          _writer_bits::writeToken(*_os, os.str());
          *_os << '\t';
          _node_index.insert(std::make_pair(n, os.str()));
        }
        for (typename NodeMaps::iterator it = _node_maps.begin();
             it != _node_maps.end(); ++it) {
          std::string value = it->second->get(n);
          _writer_bits::writeToken(*_os, value);
          if (it->first == "label") {
            _node_index.insert(std::make_pair(n, value));
          }
          *_os << '\t';
        }
        *_os << std::endl;
      }
    }

    void createNodeIndex() {
      _writer_bits::MapStorageBase<Node>* label = 0;
      for (typename NodeMaps::iterator it = _node_maps.begin();
           it != _node_maps.end(); ++it) {
        if (it->first == "label") {
          label = it->second;
          break;
        }
      }

      if (label == 0) {
        for (NodeIt n(_graph); n != INVALID; ++n) {
          std::ostringstream os;
          os << _graph.id(n);
          _node_index.insert(std::make_pair(n, os.str()));
        }
      } else {
        for (NodeIt n(_graph); n != INVALID; ++n) {
          std::string value = label->get(n);
          _node_index.insert(std::make_pair(n, value));
        }
      }
    }

    void writeEdges() {
      _writer_bits::MapStorageBase<Edge>* label = 0;
      for (typename EdgeMaps::iterator it = _edge_maps.begin();
           it != _edge_maps.end(); ++it) {
        if (it->first == "label") {
          label = it->second;
          break;
        }
      }

      *_os << "@edges";
      if (!_edges_caption.empty()) {
        _writer_bits::writeToken(*_os << ' ', _edges_caption);
      }
      *_os << std::endl;

      *_os << '\t' << '\t';
      if (label == 0) {
        *_os << "label" << '\t';
      }
      for (typename EdgeMaps::iterator it = _edge_maps.begin();
           it != _edge_maps.end(); ++it) {
        _writer_bits::writeToken(*_os, it->first) << '\t';
      }
      *_os << std::endl;

      std::vector<Edge> edges;
      for (EdgeIt n(_graph); n != INVALID; ++n) {
        edges.push_back(n);
      }

      if (label == 0) {
        IdMap<Graph, Edge> id_map(_graph);
        _writer_bits::MapLess<IdMap<Graph, Edge> > id_less(id_map);
        std::sort(edges.begin(), edges.end(), id_less);
      } else {
        label->sort(edges);
      }

      for (int i = 0; i < static_cast<int>(edges.size()); ++i) {
        Edge e = edges[i];
        _writer_bits::writeToken(*_os, _node_index.
                                 find(_graph.u(e))->second);
        *_os << '\t';
        _writer_bits::writeToken(*_os, _node_index.
                                 find(_graph.v(e))->second);
        *_os << '\t';
        if (label == 0) {
          std::ostringstream os;
          os << _graph.id(e);
          _writer_bits::writeToken(*_os, os.str());
          *_os << '\t';
          _edge_index.insert(std::make_pair(e, os.str()));
        }
        for (typename EdgeMaps::iterator it = _edge_maps.begin();
             it != _edge_maps.end(); ++it) {
          std::string value = it->second->get(e);
          _writer_bits::writeToken(*_os, value);
          if (it->first == "label") {
            _edge_index.insert(std::make_pair(e, value));
          }
          *_os << '\t';
        }
        *_os << std::endl;
      }
    }

    void createEdgeIndex() {
      _writer_bits::MapStorageBase<Edge>* label = 0;
      for (typename EdgeMaps::iterator it = _edge_maps.begin();
           it != _edge_maps.end(); ++it) {
        if (it->first == "label") {
          label = it->second;
          break;
        }
      }

      if (label == 0) {
        for (EdgeIt e(_graph); e != INVALID; ++e) {
          std::ostringstream os;
          os << _graph.id(e);
          _edge_index.insert(std::make_pair(e, os.str()));
        }
      } else {
        for (EdgeIt e(_graph); e != INVALID; ++e) {
          std::string value = label->get(e);
          _edge_index.insert(std::make_pair(e, value));
        }
      }
    }

    void writeAttributes() {
      if (_attributes.empty()) return;
      *_os << "@attributes";
      if (!_attributes_caption.empty()) {
        _writer_bits::writeToken(*_os << ' ', _attributes_caption);
      }
      *_os << std::endl;
      for (typename Attributes::iterator it = _attributes.begin();
           it != _attributes.end(); ++it) {
        _writer_bits::writeToken(*_os, it->first) << ' ';
        _writer_bits::writeToken(*_os, it->second->get());
        *_os << std::endl;
      }
    }

  public:

    /// \name Execution of the writer
    /// @{

    /// \brief Start the batch processing
    ///
    /// This function starts the batch processing.
    void run() {
      if (!_skip_nodes) {
        writeNodes();
      } else {
        createNodeIndex();
      }
      if (!_skip_edges) {
        writeEdges();
      } else {
        createEdgeIndex();
      }
      writeAttributes();
    }

    /// \brief Give back the stream of the writer
    ///
    /// Give back the stream of the writer
    std::ostream& ostream() {
      return *_os;
    }

    /// @}
  };

  /// \brief Return a \ref GraphWriter class
  ///
  /// This function just returns a \ref GraphWriter class.
  /// \relates GraphWriter
  template <typename Graph>
  GraphWriter<Graph> graphWriter(std::ostream& os, const Graph& graph) {
    GraphWriter<Graph> tmp(os, graph);
    return tmp;
  }

  /// \brief Return a \ref GraphWriter class
  ///
  /// This function just returns a \ref GraphWriter class.
  /// \relates GraphWriter
  template <typename Graph>
  GraphWriter<Graph> graphWriter(const std::string& fn, const Graph& graph) {
    GraphWriter<Graph> tmp(fn, graph);
    return tmp;
  }

  /// \brief Return a \ref GraphWriter class
  ///
  /// This function just returns a \ref GraphWriter class.
  /// \relates GraphWriter
  template <typename Graph>
  GraphWriter<Graph> graphWriter(const char* fn, const Graph& graph) {
    GraphWriter<Graph> tmp(fn, graph);
    return tmp;
  }
}

#endif
