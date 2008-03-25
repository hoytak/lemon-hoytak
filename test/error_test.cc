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

#include <iostream>

#include <lemon/error.h>
#include "test_tools.h"

using namespace lemon;

#ifdef LEMON_ENABLE_ASSERTS
#undef LEMON_ENABLE_ASSERTS
#endif

#ifdef LEMON_DISABLE_ASSERTS
#undef LEMON_DISABLE_ASSERTS
#endif

//checking disabled asserts
#define LEMON_DISABLE_ASSERTS
#include <lemon/assert.h>

void no_assertion_text_disable() {
  LEMON_ASSERT(true, "This is a fault message");
}

void no_assertion_exception_disable() {
  LEMON_ASSERT(true, Exception());
}

void assertion_text_disable() {
  LEMON_ASSERT(false, "This is a fault message");
}

void assertion_exception_disable() {
  LEMON_ASSERT(false, Exception());
}

void fixme_disable() {
  LEMON_FIXME("fixme_disable() is fixme!");
}

void check_assertion_disable() {
  no_assertion_text_disable();
  no_assertion_exception_disable();
  assertion_exception_disable();
  assertion_text_disable();
  fixme_disable();
}
#undef LEMON_DISABLE_ASSERTS


#define LEMON_ASSERT_ERROR
#include <lemon/assert.h>

void no_assertion_text_error() {
  LEMON_ASSERT(true, "This is a fault message");
}

void no_assertion_exception_error() {
  LEMON_ASSERT(true, Exception());
}

void assertion_text_error() {
  LEMON_ASSERT(false, "This is a fault message");
}

void assertion_exception_error() {
  LEMON_ASSERT(false, Exception());
}

void fixme_error() {
  LEMON_FIXME("fixme_error() is fixme!");
}

void check_assertion_error() {
  no_assertion_text_error();
  no_assertion_exception_error();
  try {
    assertion_exception_error();
    check(false, "Assertion error");
  } catch (const AssertionFailedError& e) {
  }

  try {
    assertion_text_error();
    check(false, "Assertion error");
  } catch (const AssertionFailedError& e) {
  }

  try {
    fixme_error();
    check(false, "Assertion error");
  } catch (const AssertionFailedError& e) {
  }
}
#undef LEMON_ASSERT_ERROR

#define LEMON_ASSERT_EXCEPTION
#include <lemon/assert.h>

void no_assertion_text_exception() {
  LEMON_ASSERT(true, "This is a fault message");
}

void no_assertion_exception_exception() {
  LEMON_ASSERT(true, Exception());
}

void assertion_text_exception() {
  LEMON_ASSERT(false, "This is a fault message");
}

void assertion_exception_exception() {
  LEMON_ASSERT(false, Exception());
}

void fixme_exception() {
  LEMON_FIXME("fixme_exception() is fixme!");
}

void check_assertion_exception() {
  no_assertion_text_exception();
  no_assertion_exception_exception();
  try {
    assertion_exception_exception();
    check(false, "Assertion error");
  } catch (const Exception& e) {
  }

  try {
    assertion_text_exception();
    check(false, "Assertion error");
  } catch (const AssertionFailedError& e) {
  }

  try {
    assertion_text_exception();
    check(false, "Assertion error");
  } catch (const AssertionFailedError& e) {
  }

  try {
    fixme_exception();
    check(false, "Assertion error");
  } catch (const AssertionFailedError& e) {
  }
}
#undef LEMON_ASSERT_EXCEPTION

#define LEMON_ASSERT_LOG

#include <lemon/assert.h>

void no_assertion_text_log() {
  LEMON_ASSERT(true, "This is a fault message");
}

void no_assertion_exception_log() {
  LEMON_ASSERT(true, Exception());
}

void assertion_text_log() {
  LEMON_ASSERT(false, "This is a fault message");
}

void assertion_exception_log() {
  LEMON_ASSERT(false, Exception());
}

void fixme_log() {
  LEMON_FIXME("fixme_log() is fixme!");
}

void check_assertion_log() {
  no_assertion_text_log();
  no_assertion_exception_log();
  std::cerr << "The next 3 failure messages are expected: " << std::endl;
  assertion_exception_log();
  assertion_text_log();
  fixme_log();
  std::cerr << "End of expected error messages" << std::endl;
}
#undef LEMON_ASSERT_LOG

#define LEMON_ASSERT_CUSTOM

static int cnt = 0;
void my_assert_handler(const char*, int, const char*, 
		       const char*, const char*) {
  ++cnt;
}

void my_assert_handler(const char*, int, const char*, 
		       const std::exception&, const char*) {
  ++cnt;
}

void my_assert_handler(const char*, int, const char*, 
		       const std::string&, const char*) {
  ++cnt;
}


#define LEMON_CUSTOM_ASSERT_HANDLER my_assert_handler
#include <lemon/assert.h>

void no_assertion_text_custom() {
  LEMON_ASSERT(true, "This is a fault message");
}

void no_assertion_exception_custom() {
  LEMON_ASSERT(true, Exception());
}

void assertion_text_custom() {
  LEMON_ASSERT(false, "This is a fault message");
}

void assertion_exception_custom() {
  LEMON_ASSERT(false, Exception());
}

void fixme_custom() {
  LEMON_FIXME("fixme_custom() is fixme!");
}

void check_assertion_custom() {
  no_assertion_text_custom();
  no_assertion_exception_custom();
  assertion_exception_custom();
  assertion_text_custom();
  fixme_custom();
  check(cnt == 3, "The custom assert handler does not work");
}

#undef LEMON_ASSERT_CUSTOM


int main() {
  check_assertion_disable();
  check_assertion_error();
  check_assertion_exception();
  check_assertion_log();
  check_assertion_custom();

  return 0;
}
