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

#ifndef LEMON_ERROR_H
#define LEMON_ERROR_H

/// \ingroup exceptions
/// \file
/// \brief Basic exception classes and error handling.

#include <exception>
#include <string>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <memory>

namespace lemon {

  /// \addtogroup exceptions
  /// @{

  /// \brief Exception safe wrapper class.
  ///
  /// Exception safe wrapper class to implement the members of exceptions.
  template <typename _Type>
  class ExceptionMember {
  public:
    typedef _Type Type;

    ExceptionMember() throw() {
      try {
        ptr.reset(new Type());
      } catch (...) {}
    }

    ExceptionMember(const Type& type) throw() {
      try {
        ptr.reset(new Type());
        if (ptr.get() == 0) return;
        *ptr = type;
      } catch (...) {}
    }

    ExceptionMember(const ExceptionMember& copy) throw() {
      try {
        if (!copy.valid()) return;
        ptr.reset(new Type());
        if (ptr.get() == 0) return;
        *ptr = copy.get();
      } catch (...) {}
    }

    ExceptionMember& operator=(const ExceptionMember& copy) throw() {
      if (ptr.get() == 0) return;
      try {
        if (!copy.valid()) return;
         *ptr = copy.get();
      } catch (...) {}
    }

    void set(const Type& type) throw() {
      if (ptr.get() == 0) return;
      try {
        *ptr = type;
      } catch (...) {}
    }

    const Type& get() const {
      return *ptr;
    }

    bool valid() const throw() {
      return ptr.get() != 0;
    }

  private:
    std::auto_ptr<_Type> ptr;
  };

  /// Exception-safe convenient error message builder class.

  /// Helper class which provides a convenient ostream-like (operator <<
  /// based) interface to create a string message. Mostly useful in
  /// exception classes (therefore the name).
  class ErrorMessage {
  protected:
    ///\e

    ///\todo The good solution is boost::shared_ptr...
    ///
    mutable std::auto_ptr<std::ostringstream> buf;

    ///\e
    bool init() throw() {
      try {
        buf.reset(new std::ostringstream);
      }
      catch(...) {
        buf.reset();
      }
      return buf.get();
    }

  public:

    ///\e
    ErrorMessage() throw() { init(); }

    ErrorMessage(const ErrorMessage& em) throw() : buf(em.buf) { }

    ///\e
    ErrorMessage(const char *msg) throw() {
      init();
      *this << msg;
    }

    ///\e
    ErrorMessage(const std::string &msg) throw() {
      init();
      *this << msg;
    }

    ///\e
    template <typename T>
    ErrorMessage& operator<<(const T &t) throw() {
      if( ! buf.get() ) return *this;

      try {
        *buf << t;
      }
      catch(...) {
        buf.reset();
      }
      return *this;
    }

    ///\e
    const char* message() throw() {
      if( ! buf.get() ) return 0;

      const char* mes = 0;
      try {
        mes = buf->str().c_str();
      }
      catch(...) {}
      return mes;
    }

  };

  /// Generic exception class.

  /// Base class for exceptions used in LEMON.
  ///
  class Exception : public std::exception {
  public:
    ///\e
    Exception() {}
    ///\e
    virtual ~Exception() throw() {}
    ///\e
    virtual const char* what() const throw() {
      return "lemon::Exception";
    }
  };

  /// One of the two main subclasses of \ref Exception.

  /// Logic errors represent problems in the internal logic of a program;
  /// in theory, these are preventable, and even detectable before the
  /// program runs (e.g. violations of class invariants).
  ///
  /// A typical example for this is \ref UninitializedParameter.
  class LogicError : public Exception {
  public:
    virtual const char* what() const throw() {
      return "lemon::LogicError";
    }
  };

  /// \ref Exception for uninitialized parameters.

  /// This error represents problems in the initialization
  /// of the parameters of the algorithms.
  class UninitializedParameter : public LogicError {
  public:
    virtual const char* what() const throw() {
      return "lemon::UninitializedParameter";
    }
  };


  /// One of the two main subclasses of \ref Exception.

  /// Runtime errors represent problems outside the scope of a program;
  /// they cannot be easily predicted and can generally only be caught
  /// as the program executes.
  class RuntimeError : public Exception {
  public:
    virtual const char* what() const throw() {
      return "lemon::RuntimeError";
    }
  };

  ///\e
  class RangeError : public RuntimeError {
  public:
    virtual const char* what() const throw() {
      return "lemon::RangeError";
    }
  };

  ///\e
  class IoError : public RuntimeError {
  public:
    virtual const char* what() const throw() {
      return "lemon::IoError";
    }
  };

  ///\e
  class DataFormatError : public IoError {
  protected:
    ExceptionMember<std::string> _message;
    ExceptionMember<std::string> _file;
    int _line;

    mutable ExceptionMember<std::string> _message_holder;
  public:

    DataFormatError(const DataFormatError &dfe) :
      IoError(dfe), _message(dfe._message), _file(dfe._file),
      _line(dfe._line) {}

    ///\e
    explicit DataFormatError(const char *the_message)
      : _message(the_message), _line(0) {}

    ///\e
    DataFormatError(const std::string &file_name, int line_num,
                    const char *the_message)
      : _message(the_message), _line(line_num) { file(file_name); }

    ///\e
    void line(int ln) { _line = ln; }
    ///\e
    void message(const std::string& msg) { _message.set(msg); }
    ///\e
    void file(const std::string &fl) { _file.set(fl); }

    ///\e
    int line() const { return _line; }
    ///\e
    const char* message() const {
      if (_message.valid() && !_message.get().empty()) {
        return _message.get().c_str();
      } else {
        return 0;
      }
    }

    /// \brief Returns the filename.
    ///
    /// Returns \e null if the filename was not specified.
    const char* file() const {
      if (_file.valid() && !_file.get().empty()) {
        return _file.get().c_str();
      } else {
        return 0;
      }
    }

    ///\e
    virtual const char* what() const throw() {
      try {
        std::ostringstream ostr;
        ostr << "lemon:DataFormatError" << ": ";
        if (message()) ostr << message();
        if( file() || line() != 0 ) {
          ostr << " (";
          if( file() ) ostr << "in file '" << file() << "'";
          if( file() && line() != 0 ) ostr << " ";
          if( line() != 0 ) ostr << "at line " << line();
          ostr << ")";
        }
        _message_holder.set(ostr.str());
      }
      catch (...) {}
      if( _message_holder.valid()) return _message_holder.get().c_str();
      return "lemon:DataFormatError";
    }

    virtual ~DataFormatError() throw() {}
  };

  ///\e
  class FileOpenError : public IoError {
  protected:
    ExceptionMember<std::string> _file;

    mutable ExceptionMember<std::string> _message_holder;
  public:

    FileOpenError(const FileOpenError &foe) :
      IoError(foe), _file(foe._file) {}

    ///\e
    explicit FileOpenError(const std::string& fl)
      : _file(fl) {}


    ///\e
    void file(const std::string &fl) { _file.set(fl); }

    /// \brief Returns the filename.
    ///
    /// Returns \e null if the filename was not specified.
    const char* file() const {
      if (_file.valid() && !_file.get().empty()) {
        return _file.get().c_str();
      } else {
        return 0;
      }
    }

    ///\e
    virtual const char* what() const throw() {
      try {
        std::ostringstream ostr;
        ostr << "lemon::FileOpenError" << ": ";
        ostr << "Cannot open file - " << file();
        _message_holder.set(ostr.str());
      }
      catch (...) {}
      if( _message_holder.valid()) return _message_holder.get().c_str();
      return "lemon::FileOpenError";
    }
    virtual ~FileOpenError() throw() {}
  };

  class IoParameterError : public IoError {
  protected:
    ExceptionMember<std::string> _message;
    ExceptionMember<std::string> _file;

    mutable ExceptionMember<std::string> _message_holder;
  public:

    IoParameterError(const IoParameterError &ile) :
      IoError(ile), _message(ile._message), _file(ile._file) {}

    ///\e
    explicit IoParameterError(const char *the_message)
      : _message(the_message) {}

    ///\e
    IoParameterError(const char *file_name, const char *the_message)
      : _message(the_message), _file(file_name) {}

     ///\e
    void message(const std::string& msg) { _message.set(msg); }
    ///\e
    void file(const std::string &fl) { _file.set(fl); }

     ///\e
    const char* message() const {
      if (_message.valid()) {
        return _message.get().c_str();
      } else {
        return 0;
      }
    }

    /// \brief Returns the filename.
    ///
    /// Returns \c 0 if the filename was not specified.
    const char* file() const {
      if (_file.valid()) {
        return _file.get().c_str();
      } else {
        return 0;
      }
    }

    ///\e
    virtual const char* what() const throw() {
      try {
        std::ostringstream ostr;
        if (message()) ostr << message();
        if (file()) ostr << "(when reading file '" << file() << "')";
        _message_holder.set(ostr.str());
      }
      catch (...) {}
      if( _message_holder.valid() ) return _message_holder.get().c_str();
      return "lemon:IoParameterError";
    }
    virtual ~IoParameterError() throw() {}
  };

  /// @}

}

#endif // LEMON_ERROR_H
