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

#ifndef LEMON_ASSERT_H
#define LEMON_ASSERT_H

/// \ingroup exceptions
/// \file
/// \brief Extended assertion handling

#include <lemon/error.h>

namespace lemon {

  /// @{

  ///\e
  class AssertionFailedError : public LogicError {
  protected:
    const char *_assertion;
    const char *_file;
    int _line;
    const char *_function;
    const char *_message;

    mutable ExceptionMember<std::string> _message_holder;
  public:
    ///\e
    AssertionFailedError(const char *file, int line, const char *function,
			 const char *msg, const char *assertion = 0) :
      _assertion(assertion), _file(file), _line(line), 
      _function(function), _message(msg) {}

    ///\e
    const char* assertion() const { return _assertion; }
    ///\e
    const char* message() const { return _message; }
    ///\e
    const char* file() const { return _file; }
    ///\e
    const char* function() const { return _function; }
    ///\e
    int line() const { return _line; }


    virtual const char* what() const throw() {
      try {
	std::ostringstream ostr;
	ostr << _file << ":" << _line << ": ";
	if (_function)
	  ostr << _function << ": ";
	ostr << _message;
	if (_assertion)
	   ostr << " (assertion '" << _assertion << "' failed)";
	_message_holder.set(ostr.str());
	return ostr.str().c_str();
      }
      catch(...) {}
      if( _message_holder.valid() ) return _message_holder.get().c_str();
      return "lemon::AssertionFailedError";
    }
    virtual ~AssertionFailedError() throw() {}
  };


  inline void assert_fail_log(const char *file, int line,
			      const char *function,
			      const std::exception& exception, 
			      const char *assertion)
  {
    std::cerr << file << ":" << line << ": ";
    if (function)
      std::cerr << function << ": ";
    std::cerr << exception.what();
    if (assertion)
      std::cerr << " (assertion '" << assertion << "' failed)";
    std::cerr << std::endl;
  }

  inline void assert_fail_log(const char *file, int line, const char *function,
			      const char *message, const char *assertion)
  {
    std::cerr << file << ":" << line << ": ";
    if (function)
      std::cerr << function << ": ";
    std::cerr << message;
    if (assertion)
      std::cerr << " (assertion '" << assertion << "' failed)";
    std::cerr << std::endl;
  }

  inline void assert_fail_log(const char *file, int line, const char *function, 
			      const std::string& message, const char *assertion)
  {
    assert_fail_log(file, line, function, message.c_str(), assertion);
  }

  inline void assert_fail_abort(const char *file, int line, 
				const char *function,
				const std::exception& exception,
				const char *assertion)
  {
    assert_fail_log(file, line, function, exception, assertion);
    std::abort();
  }

  inline void assert_fail_abort(const char *file, int line,
				const char *function, const char* message,
				const char *assertion)
  {
    assert_fail_log(file, line, function, message, assertion);
    std::abort();
  }

  inline void assert_fail_abort(const char *file, int line, 
				const char *function, 
				const std::string& message,
				const char *assertion)
  {
    assert_fail_log(file, line, function, message.c_str(), assertion);
    std::abort();
  }

  inline void assert_fail_error(const char *file, int line, 
				  const char *function,
				  const std::exception& exception,
				  const char *assertion)
  {
    throw AssertionFailedError(file, line, function, 
			       exception.what(), assertion);
  }

  inline void assert_fail_error(const char *file, int line,
				  const char *function, const char *message,
				  const char *assertion)
  {
    throw AssertionFailedError(file, line, function, message, assertion);
  }

  inline void assert_fail_error(const char *file, int line,
				  const char *function, 
				  const std::string& message,
				  const char *assertion)
  {
    throw AssertionFailedError(file, line, function, message.c_str(), assertion);
  }

  template <typename Exception>
  inline void assert_fail_exception(const char *, int, const char *,
				    const Exception& exception,
				    const char *, const std::exception* = 
				    static_cast<const Exception*>(0))
  {
    throw exception;
  }

  inline void assert_fail_exception(const char *file, int line,
				    const char *function, const char *message,
				    const char *assertion)
  {
    throw AssertionFailedError(file, line, function, message, assertion);
  }

  inline void assert_fail_exception(const char *file, int line, 
				    const char *function, 
				    const std::string& message,
				    const char *assertion)
  {
    throw AssertionFailedError(file, line, function, message.c_str(), assertion);
  }

/// @}

}
#endif // LEMON_ASSERT_H

#undef LEMON_ASSERT
#undef LEMON_FIXME

#if (defined(LEMON_ASSERT_LOG) ? 1 : 0) +		\
  (defined(LEMON_ASSERT_ABORT) ? 1 : 0) +		\
  (defined(LEMON_ASSERT_ERROR) ? 1 : 0) +		\
  (defined(LEMON_ASSERT_EXCEPTION) ? 1 : 0) +		\
  (defined(LEMON_ASSERT_CUSTOM) ? 1 : 0) > 1
#error "LEMON assertion system is not set properly"
#endif

#if ((defined(LEMON_ASSERT_LOG) ? 1 : 0) +		\
     (defined(LEMON_ASSERT_ABORT) ? 1 : 0) +		\
     (defined(LEMON_ASSERT_ERROR) ? 1 : 0) +		\
     (defined(LEMON_ASSERT_EXCEPTION) ? 1 : 0) +	\
     (defined(LEMON_ASSERT_CUSTOM) ? 1 : 0) == 1 ||	\
     defined(LEMON_ENABLE_ASSERTS)) &&			\
  defined(LEMON_DISABLE_ASSERTS)
#error "LEMON assertion system is not set properly"
#endif


#if defined LEMON_ASSERT_LOG
#  undef LEMON_ASSERT_HANDLER
#  define LEMON_ASSERT_HANDLER ::lemon::assert_fail_log
#elif defined LEMON_ASSERT_ABORT
#  undef LEMON_ASSERT_HANDLER
#  define LEMON_ASSERT_HANDLER ::lemon::assert_fail_abort
#elif defined LEMON_ASSERT_ERROR
#  undef LEMON_ASSERT_HANDLER
#  define LEMON_ASSERT_HANDLER ::lemon::assert_fail_error
#elif defined LEMON_ASSERT_EXCEPTION
#  undef LEMON_ASSERT_HANDLER
#  define LEMON_ASSERT_HANDLER ::lemon::assert_fail_exception
#elif defined LEMON_ASSERT_CUSTOM
#  undef LEMON_ASSERT_HANDLER
#  ifndef LEMON_CUSTOM_ASSERT_HANDLER
#    error "LEMON_CUSTOM_ASSERT_HANDLER is not set"
#  endif
#  define LEMON_ASSERT_HANDLER LEMON_CUSTOM_ASSERT_HANDLER
#elif defined LEMON_ENABLE_ASSERTS
#  undef LEMON_ASSERT_HANDLER
#  define LEMON_ASSERT_HANDLER ::lemon::assert_fail_abort
#else
#  undef LEMON_ASSERT_HANDLER
#endif


#ifndef LEMON_FUNCTION_NAME
#  define LEMON_FUNCTION_NAME (__PRETTY_FUNCTION__)
#endif

#ifdef DOXYGEN

/// \ingroup exceptions
///
/// \brief Macro for assertion with customizable message
///
/// Macro for assertion with customizable message.  
/// \param exp An expression that must be convertible to \c bool.
/// If it is \c false, then an assertion is raised. The concrete
/// behaviour depends on the settings of the assertion system.
/// \param msg A <tt>const char*</tt>, a <tt>const std::string&</tt> or
/// a <tt>const std::exception&</tt> parameter, which can be used to
/// provide information about the circumstances of the failed assertion.
///
/// The assertions are disabled in the default behaviour.
/// You can enable them with the following code:
/// \code
/// #define LEMON_ENABLE_ASSERTS
/// \endcode
/// or with compilation parameters:
/// \code
/// g++ -DLEMON_ENABLE_ASSERTS
/// make CXXFLAGS='-DLEMON_ENABLE_ASSERTS'
/// \endcode
/// 
/// The LEMON assertion system has a wide range of customization
/// properties. As a default behaviour the failed assertion prints a
/// short log message to the standard error and aborts the execution.
///
/// The following modes can be used in the assertion system: 
///
/// - \c LEMON_ASSERT_LOG The failed assertion prints a short log
///   message to the standard error and continues the execution.
/// - \c LEMON_ASSERT_ABORT This mode is similar to the
///   \c LEMON_ASSERT_LOG, but it aborts the program. It is the default
///   behaviour mode when the assertions are enabled with
///   \c LEMON_ENABLE_ASSERTS.
/// - \c LEMON_ASSERT_ERROR The assertion throws an
///   \ref lemon::AssertionFailedError "AssertionFailedError".
///   If the \c msg parameter is an exception, then the result of the
///   \ref lemon::Exception::what() "what()" member function is passed
///   as error message.
/// - \c LEMON_ASSERT_EXCEPTION If the specified \c msg is an
///   exception, then it raised directly (solving that the exception
///   can not be thrown polymorphically), otherwise an \ref
///   lemon::AssertionFailedError "AssertionFailedError" is thrown with
///   the given parameters.
/// - \c LEMON_ASSERT_CUSTOM The user can define own assertion handler
///   functions. Three overloaded functions should be defined with the
///   following parameter lists:
///   \code
///     void custom_assert_handler(const char* file, int line, const char* function,
///                                const char* message, const char* assertion);
///     void custom_assert_handler(const char* file, int line, const char* function,
///                                const std::string& message, const char* assertion);
///     void custom_assert_handler(const char* file, int line, const char* function,
///                                const std::exception& message, const char* assertion);
///   \endcode
///   The name of the functions should be defined as the \c
///   LEMON_CUSTOM_ASSERT_HANDLER macro name. 
///   \code
///     #define LEMON_CUSTOM_ASSERT_HANDLER custom_assert_handler
///   \endcode
///   Whenever an assertion is occured, one of the custom assertion
///   handlers is called with appropiate parameters.
///
/// The assertion mode can also be changed within one compilation unit.
/// If the macros are redefined with other settings and the
/// \ref lemon/assert.h "assert.h" file is reincluded, then the
/// behaviour is changed appropiately to the new settings.
#  define LEMON_ASSERT(exp, msg)					\
  (static_cast<void> (!!(exp) ? 0 : (					\
    LEMON_ASSERT_HANDLER(__FILE__, __LINE__,				\
			 LEMON_FUNCTION_NAME,				\
			 msg, #exp), 0)))


/// \ingroup exceptions
///
/// \brief Macro for mark not yet implemented features.
///
/// Macro for mark not yet implemented features and outstanding bugs.
/// It is close to be the shortcut of the following code:
/// \code
///   LEMON_ASSERT(false, msg);
/// \endcode
#  define LEMON_FIXME(msg)						\
       (LEMON_ASSERT_HANDLER(__FILE__, __LINE__, LEMON_FUNCTION_NAME,	\
			     "FIXME: " msg, static_cast<const char*>(0)))

#else

#  ifndef LEMON_ASSERT_HANDLER
#    define LEMON_ASSERT(exp, msg)  (static_cast<void>(0))
#    define LEMON_FIXME(msg) (static_cast<void>(0))
#  else
#    define LEMON_ASSERT(exp, msg)                 \
       (static_cast<void> (!!(exp) ? 0 : (         \
         LEMON_ASSERT_HANDLER(__FILE__, __LINE__,  \
                              LEMON_FUNCTION_NAME, \
                              msg, #exp), 0)))
#    define LEMON_FIXME(msg) \
       (LEMON_ASSERT_HANDLER(__FILE__, __LINE__, LEMON_FUNCTION_NAME,	\
			     "FIXME: " msg,  static_cast<const char*>(0)))
#  endif

#endif


