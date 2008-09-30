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

  /// \brief Generic exception class.
  ///
  /// Base class for exceptions used in LEMON.
  ///
  class Exception : public std::exception {
  public:
    ///\e Constructor
    Exception() {}
    ///\e Virtual destructor
    virtual ~Exception() throw() {}
    ///\e A short description of the exception
    virtual const char* what() const throw() {
      return "lemon::Exception";
    }
  };

  /// \brief Input-Output error
  ///
  /// This exception is thrown when a file operation cannot be
  /// succeeded.
  class IoError : public Exception {
  protected:
    std::string _message;
    std::string _file;

    mutable std::string _what;
  public:

    /// Copy constructor
    IoError(const IoError &error) {
      message(error._message);
      file(error._file);
    }

    /// Constructor
    explicit IoError(const char *message) {
      IoError::message(message);
    }

    /// Constructor
    explicit IoError(const std::string &message) {
      IoError::message(message);
    }

    /// Constructor
    IoError(const std::string &file, const char *message) {
      IoError::message(message);
      IoError::file(file);
    }

    /// Constructor
    IoError(const std::string &file, const std::string &message) {
      IoError::message(message);
      IoError::file(file);
    }

    /// Virtual destructor
    virtual ~IoError() throw() {}

    /// Set the error message
    void message(const char *message) {
      try {
        _message = message;
      } catch (...) {}
    }

    /// Set the error message
    void message(const std::string& message) {
      try {
        _message = message;
      } catch (...) {}
    }

    /// Set the file name
    void file(const std::string &file) {
      try {
        _file = file;
      } catch (...) {}
    }

    /// Returns the error message
    const std::string& message() const {
      return _message;
    }

    /// \brief Returns the filename
    ///
    /// Returns the filename or empty string if the filename was not
    /// specified.
    const std::string& file() const {
      return _file;
    }

    /// \brief Returns a short error message
    ///
    /// Returns a short error message which contains the message, the
    /// file name and the line number.
    virtual const char* what() const throw() {
      try {
        _what.clear();
        std::ostringstream oss;
        oss << "lemon:IoError" << ": ";
        oss << message();
        if (!file().empty()) {
          oss << " (";
          if (!file().empty()) oss << "with file '" << file() << "'";
          oss << ")";
        }
        _what = oss.str();
      }
      catch (...) {}
      if (!_what.empty()) return _what.c_str();
      else return "lemon:IoError";
    }

  };

  /// \brief Format error
  ///
  /// This class is used to indicate if an input file has wrong
  /// formatting, or a data representation is not legal.
  class FormatError : public Exception {
  protected:
    std::string _message;
    std::string _file;
    int _line;

    mutable std::string _what;
  public:

    /// Copy constructor
    FormatError(const FormatError &error) {
      message(error._message);
      file(error._file);
      line(error._line);
    }

    /// Constructor
    explicit FormatError(const char *message) {
      FormatError::message(message);
      _line = 0;
    }

    /// Constructor
    explicit FormatError(const std::string &message) {
      FormatError::message(message);
      _line = 0;
    }

    /// Constructor
    FormatError(const std::string &file, int line, const char *message) {
      FormatError::message(message);
      FormatError::file(file);
      FormatError::line(line);
    }

    /// Constructor
    FormatError(const std::string &file, int line, const std::string &message) {
      FormatError::message(message);
      FormatError::file(file);
      FormatError::line(line);
    }

    /// Virtual destructor
    virtual ~FormatError() throw() {}

    /// Set the line number
    void line(int line) { _line = line; }

    /// Set the error message
    void message(const char *message) {
      try {
        _message = message;
      } catch (...) {}
    }

    /// Set the error message
    void message(const std::string& message) {
      try {
        _message = message;
      } catch (...) {}
    }

    /// Set the file name
    void file(const std::string &file) {
      try {
        _file = file;
      } catch (...) {}
    }

    /// \brief Returns the line number
    ///
    /// Returns the line number or zero if it was not specified.
    int line() const { return _line; }

    /// Returns the error message
    const std::string& message() const {
      return _message;
    }

    /// \brief Returns the filename
    ///
    /// Returns the filename or empty string if the filename was not
    /// specified.
    const std::string& file() const {
      return _file;
    }

    /// \brief Returns a short error message
    ///
    /// Returns a short error message which contains the message, the
    /// file name and the line number.
    virtual const char* what() const throw() {
      try {
        _what.clear();
        std::ostringstream oss;
        oss << "lemon:FormatError" << ": ";
        oss << message();
        if (!file().empty() || line() != 0) {
          oss << " (";
          if (!file().empty()) oss << "in file '" << file() << "'";
          if (!file().empty() && line() != 0) oss << " ";
          if (line() != 0) oss << "at line " << line();
          oss << ")";
        }
        _what = oss.str();
      }
      catch (...) {}
      if (!_what.empty()) return _what.c_str();
      else return "lemon:FormatError";
    }

  };

  /// @}

}

#endif // LEMON_ERROR_H
