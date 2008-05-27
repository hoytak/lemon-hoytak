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

///\ingroup lemon_io
///\file
///\brief Lemon Graph Format reader.


#ifndef LEMON_LGF_READER_H
#define LEMON_LGF_READER_H

#include <iostream>
#include <fstream>
#include <sstream>

#include <set>
#include <map>

#include <lemon/assert.h>
#include <lemon/graph_utils.h>

#include <lemon/lgf_writer.h>

#include <lemon/concept_check.h>
#include <lemon/concepts/maps.h>

namespace lemon {

  namespace _reader_bits {

    template <typename Value>
    struct DefaultConverter {
      Value operator()(const std::string& str) {
	std::istringstream is(str);
	Value value;
	is >> value;

	char c;
	if (is >> std::ws >> c) {
	  throw DataFormatError("Remaining characters in token");
	}
	return value;
      }
    };

    template <>
    struct DefaultConverter<std::string> {
      std::string operator()(const std::string& str) {
	return str;
      }
    };

    template <typename _Item>    
    class MapStorageBase {
    public:
      typedef _Item Item;

    public:
      MapStorageBase() {}
      virtual ~MapStorageBase() {}

      virtual void set(const Item& item, const std::string& value) = 0;

    };

    template <typename _Item, typename _Map, 
	      typename _Converter = DefaultConverter<typename _Map::Value> >
    class MapStorage : public MapStorageBase<_Item> {
    public:
      typedef _Map Map;
      typedef _Converter Converter;
      typedef _Item Item;
      
    private:
      Map& _map;
      Converter _converter;

    public:
      MapStorage(Map& map, const Converter& converter = Converter()) 
	: _map(map), _converter(converter) {}
      virtual ~MapStorage() {}

      virtual void set(const Item& item ,const std::string& value) {
	_map.set(item, _converter(value));
      }
    };

    class ValueStorageBase {
    public:
      ValueStorageBase() {}
      virtual ~ValueStorageBase() {}

      virtual void set(const std::string&) = 0;
    };

    template <typename _Value, typename _Converter = DefaultConverter<_Value> >
    class ValueStorage : public ValueStorageBase {
    public:
      typedef _Value Value;
      typedef _Converter Converter;

    private:
      Value& _value;
      Converter _converter;

    public:
      ValueStorage(Value& value, const Converter& converter = Converter())
 	: _value(value), _converter(converter) {}

      virtual void set(const std::string& value) {
	_value = _converter(value);
      }
    };

    template <typename Value>
    struct MapLookUpConverter {
      const std::map<std::string, Value>& _map;

      MapLookUpConverter(const std::map<std::string, Value>& map)
        : _map(map) {}

      Value operator()(const std::string& str) {
        typename std::map<std::string, Value>::const_iterator it =
          _map.find(str);
        if (it == _map.end()) {
          std::ostringstream msg;
          msg << "Item not found: " << str;
          throw DataFormatError(msg.str().c_str());
        }
        return it->second;
      }
    };

    bool isWhiteSpace(char c) {
      return c == ' ' || c == '\t' || c == '\v' || 
        c == '\n' || c == '\r' || c == '\f'; 
    }
    
    bool isOct(char c) {
      return '0' <= c && c <='7'; 
    }
    
    int valueOct(char c) {
      LEMON_ASSERT(isOct(c), "The character is not octal.");
      return c - '0';
    }

    bool isHex(char c) {
      return ('0' <= c && c <= '9') || 
	('a' <= c && c <= 'z') || 
	('A' <= c && c <= 'Z'); 
    }
    
    int valueHex(char c) {
      LEMON_ASSERT(isHex(c), "The character is not hexadecimal.");
      if ('0' <= c && c <= '9') return c - '0';
      if ('a' <= c && c <= 'z') return c - 'a' + 10;
      return c - 'A' + 10;
    }

    bool isIdentifierFirstChar(char c) {
      return ('a' <= c && c <= 'z') ||
	('A' <= c && c <= 'Z') || c == '_';
    }

    bool isIdentifierChar(char c) {
      return isIdentifierFirstChar(c) ||
	('0' <= c && c <= '9');
    }

    char readEscape(std::istream& is) {
      char c;
      if (!is.get(c))
	throw DataFormatError("Escape format error");

      switch (c) {
      case '\\':
	return '\\';
      case '\"':
	return '\"';
      case '\'':
	return '\'';
      case '\?':
	return '\?';
      case 'a':
	return '\a';
      case 'b':
	return '\b';
      case 'f':
	return '\f';
      case 'n':
	return '\n';
      case 'r':
	return '\r';
      case 't':
	return '\t';
      case 'v':
	return '\v';
      case 'x':
	{
	  int code;
	  if (!is.get(c) || !isHex(c)) 
	    throw DataFormatError("Escape format error");
	  else if (code = valueHex(c), !is.get(c) || !isHex(c)) is.putback(c);
	  else code = code * 16 + valueHex(c);
	  return code;
	}
      default:
	{
	  int code;
	  if (!isOct(c)) 
	    throw DataFormatError("Escape format error");
	  else if (code = valueOct(c), !is.get(c) || !isOct(c)) 
	    is.putback(c);
	  else if (code = code * 8 + valueOct(c), !is.get(c) || !isOct(c)) 
	    is.putback(c);
	  else code = code * 8 + valueOct(c);
	  return code;
	}	      
      } 
    }
    
    std::istream& readToken(std::istream& is, std::string& str) {
      std::ostringstream os;

      char c;
      is >> std::ws;
      
      if (!is.get(c)) 
	return is;

      if (c == '\"') {
	while (is.get(c) && c != '\"') {
	  if (c == '\\') 
	    c = readEscape(is);
	  os << c;
	}
	if (!is) 
	  throw DataFormatError("Quoted format error");
      } else {
	is.putback(c);
	while (is.get(c) && !isWhiteSpace(c)) {
	  if (c == '\\') 
	    c = readEscape(is);
	  os << c;
	}
	if (!is) {
	  is.clear();
	} else {
	  is.putback(c);
	}
      }
      str = os.str();
      return is;
    }

    class Section {
    public:
      virtual ~Section() {}
      virtual void process(std::istream& is, int& line_num) = 0;
    };

    template <typename Functor>
    class LineSection : public Section {
    private:

      Functor _functor;

    public:
      
      LineSection(const Functor& functor) : _functor(functor) {}
      virtual ~LineSection() {}

      virtual void process(std::istream& is, int& line_num) {
	char c;
	std::string line;
	while (is.get(c) && c != '@') {
	  if (c == '\n') {
	    ++line_num;
	  } else if (c == '#') {
	    getline(is, line);
	    ++line_num;
	  } else if (!isWhiteSpace(c)) {
	    is.putback(c);
	    getline(is, line);
	    _functor(line);
	    ++line_num;
	  }
	}
	if (is) is.putback(c);
	else if (is.eof()) is.clear();
      }
    };

    template <typename Functor>
    class StreamSection : public Section {
    private:

      Functor _functor;

    public:
      
      StreamSection(const Functor& functor) : _functor(functor) {}
      virtual ~StreamSection() {} 

      virtual void process(std::istream& is, int& line_num) {
	_functor(is, line_num);
	char c;
	std::string line;
	while (is.get(c) && c != '@') {
	  if (c == '\n') {
	    ++line_num;
	  } else if (!isWhiteSpace(c)) {
	    getline(is, line);
	    ++line_num;
	  }
	}
	if (is) is.putback(c);
	else if (is.eof()) is.clear();	
      }
    };
    
  }

  /// \ingroup lemon_io
  ///  
  /// \brief LGF reader for directed graphs
  ///
  /// This utility reads an \ref lgf-format "LGF" file.
  ///
  /// The reading method does a batch processing. The user creates a
  /// reader object, then various reading rules can be added to the
  /// reader, and eventually the reading is executed with the \c run()
  /// member function. A map reading rule can be added to the reader
  /// with the \c nodeMap() or \c arcMap() members. An optional
  /// converter parameter can also be added as a standard functor
  /// converting from std::string to the value type of the map. If it
  /// is set, it will determine how the tokens in the file should be
  /// is converted to the map's value type. If the functor is not set,
  /// then a default conversion will be used. One map can be read into
  /// multiple map objects at the same time. The \c attribute(), \c
  /// node() and \c arc() functions are used to add attribute reading
  /// rules.
  ///
  ///\code
  ///     DigraphReader<Digraph>(std::cin, digraph).
  ///       nodeMap("coordinates", coord_map).
  ///       arcMap("capacity", cap_map).
  ///       node("source", src).
  ///       node("target", trg).
  ///       attribute("caption", caption).
  ///       run();
  ///\endcode
  ///
  /// By default the reader uses the first section in the file of the
  /// proper type. If a section has an optional name, then it can be
  /// selected for reading by giving an optional name parameter to the
  /// \c nodes(), \c arcs() or \c attributes() functions. The readers
  /// also can load extra sections with the \c sectionLines() and
  /// sectionStream() functions.
  ///
  /// The \c useNodes() and \c useArcs() functions are used to tell the reader
  /// that the nodes or arcs should not be constructed (added to the
  /// graph) during the reading, but instead the label map of the items
  /// are given as a parameter of these functions. An
  /// application of these function is multipass reading, which is
  /// important if two \e \@arcs sections must be read from the
  /// file. In this example the first phase would read the node set and one
  /// of the arc sets, while the second phase would read the second arc
  /// set into an \e ArcSet class (\c SmartArcSet or \c ListArcSet).
  /// The previously read label node map should be passed to the \c
  /// useNodes() functions. Another application of multipass reading when
  /// paths are given as a node map or an arc map. It is impossible read this in
  /// a single pass, because the arcs are not constructed when the node
  /// maps are read.
  template <typename _Digraph>
  class DigraphReader {
  public:

    typedef _Digraph Digraph;
    TEMPLATE_DIGRAPH_TYPEDEFS(Digraph);
    
  private:


    std::istream* _is;
    bool local_is;

    Digraph& _digraph;

    std::string _nodes_caption;
    std::string _arcs_caption;
    std::string _attributes_caption;

    typedef std::map<std::string, Node> NodeIndex;
    NodeIndex _node_index;
    typedef std::map<std::string, Arc> ArcIndex;
    ArcIndex _arc_index;
    
    typedef std::vector<std::pair<std::string, 
      _reader_bits::MapStorageBase<Node>*> > NodeMaps;    
    NodeMaps _node_maps; 

    typedef std::vector<std::pair<std::string,
      _reader_bits::MapStorageBase<Arc>*> >ArcMaps;
    ArcMaps _arc_maps;

    typedef std::multimap<std::string, _reader_bits::ValueStorageBase*> 
      Attributes;
    Attributes _attributes;

    typedef std::map<std::string, _reader_bits::Section*> Sections;
    Sections _sections;

    bool _use_nodes;
    bool _use_arcs;

    int line_num;
    std::istringstream line;

  public:

    /// \brief Constructor
    ///
    /// Construct a directed graph reader, which reads from the given
    /// input stream.
    DigraphReader(std::istream& is, Digraph& digraph) 
      : _is(&is), local_is(false), _digraph(digraph),
	_use_nodes(false), _use_arcs(false) {}

    /// \brief Constructor
    ///
    /// Construct a directed graph reader, which reads from the given
    /// file.
    DigraphReader(const std::string& fn, Digraph& digraph) 
      : _is(new std::ifstream(fn.c_str())), local_is(true), _digraph(digraph),
    	_use_nodes(false), _use_arcs(false) {}
    
    /// \brief Constructor
    ///
    /// Construct a directed graph reader, which reads from the given
    /// file.
    DigraphReader(const char* fn, Digraph& digraph) 
      : _is(new std::ifstream(fn)), local_is(true), _digraph(digraph),
    	_use_nodes(false), _use_arcs(false) {}

    /// \brief Copy constructor
    ///
    /// The copy constructor transfers all data from the other reader,
    /// therefore the copied reader will not be usable more. 
    DigraphReader(DigraphReader& other) 
      : _is(other._is), local_is(other.local_is), _digraph(other._digraph),
	_use_nodes(other._use_nodes), _use_arcs(other._use_arcs) {

      other.is = 0;
      other.local_is = false;
      
      _node_index.swap(other._node_index);
      _arc_index.swap(other._arc_index);

      _node_maps.swap(other._node_maps);
      _arc_maps.swap(other._arc_maps);
      _attributes.swap(other._attributes);

      _nodes_caption = other._nodes_caption;
      _arcs_caption = other._arcs_caption;
      _attributes_caption = other._attributes_caption;

      _sections.swap(other._sections);
    }

    /// \brief Destructor
    ~DigraphReader() {
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

      for (typename Sections::iterator it = _sections.begin(); 
	   it != _sections.end(); ++it) {
	delete it->second;
      }

      if (local_is) {
	delete _is;
      }

    }

  private:
    
    DigraphReader& operator=(const DigraphReader&);

  public:

    /// \name Reading rules
    /// @{
    
    /// \brief Node map reading rule
    ///
    /// Add a node map reading rule to the reader.
    template <typename Map>
    DigraphReader& nodeMap(const std::string& caption, Map& map) {
      checkConcept<concepts::WriteMap<Node, typename Map::Value>, Map>();
      _reader_bits::MapStorageBase<Node>* storage = 
	new _reader_bits::MapStorage<Node, Map>(map);
      _node_maps.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Node map reading rule
    ///
    /// Add a node map reading rule with specialized converter to the
    /// reader.
    template <typename Map, typename Converter>
    DigraphReader& nodeMap(const std::string& caption, Map& map, 
			   const Converter& converter = Converter()) {
      checkConcept<concepts::WriteMap<Node, typename Map::Value>, Map>();
      _reader_bits::MapStorageBase<Node>* storage = 
	new _reader_bits::MapStorage<Node, Map, Converter>(map, converter);
      _node_maps.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Arc map reading rule
    ///
    /// Add an arc map reading rule to the reader.
    template <typename Map>
    DigraphReader& arcMap(const std::string& caption, Map& map) {
      checkConcept<concepts::WriteMap<Arc, typename Map::Value>, Map>();
      _reader_bits::MapStorageBase<Arc>* storage = 
	new _reader_bits::MapStorage<Arc, Map>(map);
      _arc_maps.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Arc map reading rule
    ///
    /// Add an arc map reading rule with specialized converter to the
    /// reader.
    template <typename Map, typename Converter>
    DigraphReader& arcMap(const std::string& caption, Map& map, 
			  const Converter& converter = Converter()) {
      checkConcept<concepts::WriteMap<Arc, typename Map::Value>, Map>();
      _reader_bits::MapStorageBase<Arc>* storage = 
	new _reader_bits::MapStorage<Arc, Map, Converter>(map, converter);
      _arc_maps.push_back(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Attribute reading rule
    ///
    /// Add an attribute reading rule to the reader.
    template <typename Value>
    DigraphReader& attribute(const std::string& caption, Value& value) {
      _reader_bits::ValueStorageBase* storage = 
	new _reader_bits::ValueStorage<Value>(value);
      _attributes.insert(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Attribute reading rule
    ///
    /// Add an attribute reading rule with specialized converter to the
    /// reader.
    template <typename Value, typename Converter>
    DigraphReader& attribute(const std::string& caption, Value& value, 
			     const Converter& converter = Converter()) {
      _reader_bits::ValueStorageBase* storage = 
	new _reader_bits::ValueStorage<Value, Converter>(value, converter);
      _attributes.insert(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Node reading rule
    ///
    /// Add a node reading rule to reader.
    DigraphReader& node(const std::string& caption, Node& node) {
      typedef _reader_bits::MapLookUpConverter<Node> Converter;
      Converter converter(_node_index);
      _reader_bits::ValueStorageBase* storage = 
	new _reader_bits::ValueStorage<Node, Converter>(node, converter);
      _attributes.insert(std::make_pair(caption, storage));
      return *this;
    }

    /// \brief Arc reading rule
    ///
    /// Add an arc reading rule to reader.
    DigraphReader& arc(const std::string& caption, Arc& arc) {
      typedef _reader_bits::MapLookUpConverter<Arc> Converter;
      Converter converter(_arc_index);
      _reader_bits::ValueStorageBase* storage = 
	new _reader_bits::ValueStorage<Arc, Converter>(arc, converter);
      _attributes.insert(std::make_pair(caption, storage));
      return *this;
    }

    /// @}

    /// \name Select section by name
    /// @{

    /// \brief Set \c \@nodes section to be read
    ///
    /// Set \c \@nodes section to be read
    DigraphReader& nodes(const std::string& caption) {
      _nodes_caption = caption;
      return *this;
    }

    /// \brief Set \c \@arcs section to be read
    ///
    /// Set \c \@arcs section to be read
    DigraphReader& arcs(const std::string& caption) {
      _arcs_caption = caption;
      return *this;
    }

    /// \brief Set \c \@attributes section to be read
    ///
    /// Set \c \@attributes section to be read
    DigraphReader& attributes(const std::string& caption) {
      _attributes_caption = caption;
      return *this;
    }

    /// @}

    /// \name Section readers
    /// @{

    /// \brief Add a section processor with line oriented reading
    ///
    /// In the \e LGF file extra sections can be placed, which contain
    /// any data in arbitrary format. These sections can be read with
    /// this function line by line. The first parameter is the type
    /// descriptor of the section, the second is a functor, which
    /// takes just one \c std::string parameter. At the reading
    /// process, each line of the section will be given to the functor
    /// object. However, the empty lines and the comment lines are
    /// filtered out, and the leading whitespaces are stipped from
    /// each processed string.
    ///
    /// For example let's see a section, which contain several
    /// integers, which should be inserted into a vector.
    ///\code
    ///  @numbers
    ///  12 45 23
    ///  4
    ///  23 6
    ///\endcode
    ///
    /// The functor is implemented as an struct:
    ///\code
    ///  struct NumberSection {
    ///    std::vector<int>& _data;
    ///    NumberSection(std::vector<int>& data) : _data(data) {}
    ///    void operator()(const std::string& line) {
    ///      std::istringstream ls(line);
    ///      int value;
    ///      while (ls >> value) _data.push_back(value);
    ///    }
    ///  };
    ///
    ///  // ...
    ///
    ///  reader.sectionLines("numbers", NumberSection(vec));  
    ///\endcode
    template <typename Functor>
    DigraphReader& sectionLines(const std::string& type, Functor functor) {
      LEMON_ASSERT(!type.empty(), "Type is not empty.");
      LEMON_ASSERT(_sections.find(type) == _sections.end(), 
		   "Multiple reading of section.");
      LEMON_ASSERT(type != "nodes" && type != "arcs" && type != "edges" &&
		   type != "attributes", "Multiple reading of section.");
      _sections.insert(std::make_pair(type, 
        new _reader_bits::LineSection<Functor>(functor)));
      return *this;
    }


    /// \brief Add a section processor with stream oriented reading
    ///
    /// In the \e LGF file extra sections can be placed, which contain
    /// any data in arbitrary format. These sections can be read
    /// directly with this function. The first parameter is the type
    /// of the section, the second is a functor, which takes an \c
    /// std::istream& and an int& parameter, the latter regard to the
    /// line number of stream. The functor can read the input while
    /// the section go on, and the line number should be modified
    /// accordingly.
    template <typename Functor>
    DigraphReader& sectionStream(const std::string& type, Functor functor) {
      LEMON_ASSERT(!type.empty(), "Type is not empty.");
      LEMON_ASSERT(_sections.find(type) == _sections.end(), 
		   "Multiple reading of section.");
      LEMON_ASSERT(type != "nodes" && type != "arcs" && type != "edges" &&
		   type != "attributes", "Multiple reading of section.");
      _sections.insert(std::make_pair(type, 
	 new _reader_bits::StreamSection<Functor>(functor)));
      return *this;
    }    
    
    /// @}

    /// \name Using previously constructed node or arc set
    /// @{

    /// \brief Use previously constructed node set
    ///
    /// Use previously constructed node set, and specify the node
    /// label map.
    template <typename Map>
    DigraphReader& useNodes(const Map& map) {
      checkConcept<concepts::ReadMap<Node, typename Map::Value>, Map>();
      LEMON_ASSERT(!_use_nodes, "Multiple usage of useNodes() member"); 
      _use_nodes = true;
      _writer_bits::DefaultConverter<typename Map::Value> converter;
      for (NodeIt n(_digraph); n != INVALID; ++n) {
	_node_index.insert(std::make_pair(converter(map[n]), n));
      }
      return *this;
    }

    /// \brief Use previously constructed node set
    ///
    /// Use previously constructed node set, and specify the node
    /// label map and a functor which converts the label map values to
    /// std::string.
    template <typename Map, typename Converter>
    DigraphReader& useNodes(const Map& map, 
			    const Converter& converter = Converter()) {
      checkConcept<concepts::ReadMap<Node, typename Map::Value>, Map>();
      LEMON_ASSERT(!_use_nodes, "Multiple usage of useNodes() member"); 
      _use_nodes = true;
      for (NodeIt n(_digraph); n != INVALID; ++n) {
	_node_index.insert(std::make_pair(converter(map[n]), n));
      }
      return *this;
    }

    /// \brief Use previously constructed arc set
    ///
    /// Use previously constructed arc set, and specify the arc
    /// label map.
    template <typename Map>
    DigraphReader& useArcs(const Map& map) {
      checkConcept<concepts::ReadMap<Arc, typename Map::Value>, Map>();
      LEMON_ASSERT(!_use_arcs, "Multiple usage of useArcs() member");
      _use_arcs = true;
      _writer_bits::DefaultConverter<typename Map::Value> converter;
      for (ArcIt a(_digraph); a != INVALID; ++a) {
	_arc_index.insert(std::make_pair(converter(map[a]), a));
      }
      return *this;
    }

    /// \brief Use previously constructed arc set
    ///
    /// Use previously constructed arc set, and specify the arc
    /// label map and a functor which converts the label map values to
    /// std::string.
    template <typename Map, typename Converter>
    DigraphReader& useArcs(const Map& map, 
			    const Converter& converter = Converter()) {
      checkConcept<concepts::ReadMap<Arc, typename Map::Value>, Map>();
      LEMON_ASSERT(!_use_arcs, "Multiple usage of useArcs() member"); 
      _use_arcs = true;
      for (ArcIt a(_digraph); a != INVALID; ++a) {
	_arc_index.insert(std::make_pair(converter(map[a]), a));
      }
      return *this;
    }

    /// @}

  private:

    bool readLine() {
      std::string str;
      while(++line_num, std::getline(*_is, str)) {
	line.clear(); line.str(str);
	char c;
	if (line >> std::ws >> c && c != '#') {
	  line.putback(c);
	  return true;
	}
      }
      return false;
    }

    bool readSuccess() {
      return static_cast<bool>(*_is);
    }
    
    void skipSection() {
      char c;
      while (readSuccess() && line >> c && c != '@') {
	readLine();
      }
      line.putback(c);
    }

    void readNodes() {

      std::vector<int> map_index(_node_maps.size());
      int map_num, label_index;

      if (!readLine()) 
	throw DataFormatError("Cannot find map captions");
      
      {
	std::map<std::string, int> maps;
	
	std::string map;
	int index = 0;
	while (_reader_bits::readToken(line, map)) {
	  if (maps.find(map) != maps.end()) {
	    std::ostringstream msg;
	    msg << "Multiple occurence of node map: " << map;
	    throw DataFormatError(msg.str().c_str());
	  }
	  maps.insert(std::make_pair(map, index));
	  ++index;
	}
	
	for (int i = 0; i < static_cast<int>(_node_maps.size()); ++i) {
	  std::map<std::string, int>::iterator jt = 
	    maps.find(_node_maps[i].first);
	  if (jt == maps.end()) {
	    std::ostringstream msg;
	    msg << "Map not found in file: " << _node_maps[i].first;
	    throw DataFormatError(msg.str().c_str());
	  }
	  map_index[i] = jt->second;
	}

	{
	  std::map<std::string, int>::iterator jt = maps.find("label");
	  if (jt == maps.end())
	    throw DataFormatError("Label map not found in file");
	  label_index = jt->second;
	}
	map_num = maps.size();
      }

      char c;
      while (readLine() && line >> c && c != '@') {
	line.putback(c);

	std::vector<std::string> tokens(map_num);
	for (int i = 0; i < map_num; ++i) {
	  if (!_reader_bits::readToken(line, tokens[i])) {
	    std::ostringstream msg;
	    msg << "Column not found (" << i + 1 << ")";
	    throw DataFormatError(msg.str().c_str());
	  }
	}
	if (line >> std::ws >> c)
	  throw DataFormatError("Extra character on the end of line");
	
	Node n;
	if (!_use_nodes) {
	  n = _digraph.addNode();
	  _node_index.insert(std::make_pair(tokens[label_index], n));
	} else {
	  typename std::map<std::string, Node>::iterator it =
	    _node_index.find(tokens[label_index]);
	  if (it == _node_index.end()) {
	    std::ostringstream msg;
	    msg << "Node with label not found: " << tokens[label_index];
	    throw DataFormatError(msg.str().c_str());	    
	  }
	  n = it->second;
	}

	for (int i = 0; i < static_cast<int>(_node_maps.size()); ++i) {
	  _node_maps[i].second->set(n, tokens[map_index[i]]);
	}

      }
      if (readSuccess()) {
	line.putback(c);
      }
    }

    void readArcs() {

      std::vector<int> map_index(_arc_maps.size());
      int map_num, label_index;

      if (!readLine()) 
	throw DataFormatError("Cannot find map captions");
      
      {
	std::map<std::string, int> maps;
	
	std::string map;
	int index = 0;
	while (_reader_bits::readToken(line, map)) {
	  if (maps.find(map) != maps.end()) {
	    std::ostringstream msg;
	    msg << "Multiple occurence of arc map: " << map;
	    throw DataFormatError(msg.str().c_str());
	  }
	  maps.insert(std::make_pair(map, index));
	  ++index;
	}
	
	for (int i = 0; i < static_cast<int>(_arc_maps.size()); ++i) {
	  std::map<std::string, int>::iterator jt = 
	    maps.find(_arc_maps[i].first);
	  if (jt == maps.end()) {
	    std::ostringstream msg;
	    msg << "Map not found in file: " << _arc_maps[i].first;
	    throw DataFormatError(msg.str().c_str());
	  }
	  map_index[i] = jt->second;
	}

	{
	  std::map<std::string, int>::iterator jt = maps.find("label");
	  if (jt == maps.end())
	    throw DataFormatError("Label map not found in file");
	  label_index = jt->second;
	}
	map_num = maps.size();
      }

      char c;
      while (readLine() && line >> c && c != '@') {
	line.putback(c);

	std::string source_token;
	std::string target_token;

	if (!_reader_bits::readToken(line, source_token))
	  throw DataFormatError("Source not found");

	if (!_reader_bits::readToken(line, target_token))
	  throw DataFormatError("Source not found");
	
	std::vector<std::string> tokens(map_num);
	for (int i = 0; i < map_num; ++i) {
	  if (!_reader_bits::readToken(line, tokens[i])) {
	    std::ostringstream msg;
	    msg << "Column not found (" << i + 1 << ")";
	    throw DataFormatError(msg.str().c_str());
	  }
	}
	if (line >> std::ws >> c)
	  throw DataFormatError("Extra character on the end of line");
	
	Arc a;
	if (!_use_arcs) {

          typename NodeIndex::iterator it;
 
          it = _node_index.find(source_token);
          if (it == _node_index.end()) {
            std::ostringstream msg;
            msg << "Item not found: " << source_token;
            throw DataFormatError(msg.str().c_str());
          }
          Node source = it->second;

          it = _node_index.find(target_token);
          if (it == _node_index.end()) {       
            std::ostringstream msg;            
            msg << "Item not found: " << target_token;
            throw DataFormatError(msg.str().c_str());
          }                                          
          Node target = it->second;                            

	  a = _digraph.addArc(source, target);
	  _arc_index.insert(std::make_pair(tokens[label_index], a));
	} else {
	  typename std::map<std::string, Arc>::iterator it =
	    _arc_index.find(tokens[label_index]);
	  if (it == _arc_index.end()) {
	    std::ostringstream msg;
	    msg << "Arc with label not found: " << tokens[label_index];
	    throw DataFormatError(msg.str().c_str());	    
	  }
	  a = it->second;
	}

	for (int i = 0; i < static_cast<int>(_arc_maps.size()); ++i) {
	  _arc_maps[i].second->set(a, tokens[map_index[i]]);
	}

      }
      if (readSuccess()) {
	line.putback(c);
      }
    }

    void readAttributes() {

      std::set<std::string> read_attr;

      char c;
      while (readLine() && line >> c && c != '@') {
	line.putback(c);
	
	std::string attr, token;
	if (!_reader_bits::readToken(line, attr))
	  throw DataFormatError("Attribute name not found");
	if (!_reader_bits::readToken(line, token))
	  throw DataFormatError("Attribute value not found");
	if (line >> c)
	  throw DataFormatError("Extra character on the end of line");	  

	{
	  std::set<std::string>::iterator it = read_attr.find(attr);
	  if (it != read_attr.end()) {
	    std::ostringstream msg;
	    msg << "Multiple occurence of attribute " << attr;
	    throw DataFormatError(msg.str().c_str());
	  }
	  read_attr.insert(attr);
	}
	
	{
	  typename Attributes::iterator it = _attributes.lower_bound(attr);
	  while (it != _attributes.end() && it->first == attr) {
	    it->second->set(token);
	    ++it;
	  }
	}

      }
      if (readSuccess()) {
	line.putback(c);
      }
      for (typename Attributes::iterator it = _attributes.begin();
	   it != _attributes.end(); ++it) {
	if (read_attr.find(it->first) == read_attr.end()) {
	  std::ostringstream msg;
	  msg << "Attribute not found in file: " << it->first;
	  throw DataFormatError(msg.str().c_str());
	}	
      }
    }

  public:

    /// \name Execution of the reader    
    /// @{

    /// \brief Start the batch processing
    ///
    /// This function starts the batch processing
    void run() {
      
      LEMON_ASSERT(_is != 0, "This reader assigned to an other reader");
      
      bool nodes_done = false;
      bool arcs_done = false;
      bool attributes_done = false;
      std::set<std::string> extra_sections;

      line_num = 0;      
      readLine();

      while (readSuccess()) {
	skipSection();
	try {
	  char c;
	  std::string section, caption;
	  line >> c;
	  _reader_bits::readToken(line, section);
	  _reader_bits::readToken(line, caption);

	  if (line >> c) 
	    throw DataFormatError("Extra character on the end of line");

	  if (section == "nodes" && !nodes_done) {
	    if (_nodes_caption.empty() || _nodes_caption == caption) {
	      readNodes();
	      nodes_done = true;
	    }
	  } else if ((section == "arcs" || section == "edges") && 
		     !arcs_done) {
	    if (_arcs_caption.empty() || _arcs_caption == caption) {
	      readArcs();
	      arcs_done = true;
	    }
	  } else if (section == "attributes" && !attributes_done) {
	    if (_attributes_caption.empty() || _attributes_caption == caption) {
	      readAttributes();
	      attributes_done = true;
	    }
	  } else {
	    if (extra_sections.find(section) != extra_sections.end()) {
	      std::ostringstream msg;
	      msg << "Multiple occurence of section " << section;
	      throw DataFormatError(msg.str().c_str());
	    }
	    Sections::iterator it = _sections.find(section);
	    if (it != _sections.end()) {
	      extra_sections.insert(section);
	      it->second->process(*_is, line_num);
	      readLine();
	    } else {
	      readLine();
	      skipSection();
	    }
	  }
	} catch (DataFormatError& error) {
	  error.line(line_num);
	  throw;
	}	
      }

      if (!nodes_done) {
	throw DataFormatError("Section @nodes not found");
      }

      if (!arcs_done) {
	throw DataFormatError("Section @arcs not found");
      }

      if (!attributes_done && !_attributes.empty()) {
	throw DataFormatError("Section @attributes not found");
      }

    }

    /// @}
    
  };

  /// \relates DigraphReader
  template <typename Digraph>
  DigraphReader<Digraph> digraphReader(std::istream& is, Digraph& digraph) {
    return DigraphReader<Digraph>(is, digraph);
  }

  /// \relates DigraphReader
  template <typename Digraph>
  DigraphReader<Digraph> digraphReader(const std::string& fn, 
				       Digraph& digraph) {
    return DigraphReader<Digraph>(fn, digraph);
  }

  /// \relates DigraphReader
  template <typename Digraph>
  DigraphReader<Digraph> digraphReader(const char* fn, Digraph& digraph) {
    return DigraphReader<Digraph>(fn, digraph);
  }
}

#endif
