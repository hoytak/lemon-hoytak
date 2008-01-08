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

// This file contains a modified version of the concept checking
// utility from BOOST.
// See the appropriate copyright notice below.

// (C) Copyright Jeremy Siek 2000.
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// Revision History:
//   05 May   2001: Workarounds for HP aCC from Thomas Matelich. (Jeremy Siek)
//   02 April 2001: Removed limits header altogether. (Jeremy Siek)
//   01 April 2001: Modified to use new <boost/limits.hpp> header. (JMaddock)
//

// See http://www.boost.org/libs/concept_check for documentation.

///\file
///\brief Basic utilities for concept checking.
///
///\todo Are we still using BOOST concept checking utility? 
///Is the BOOST copyright notice necessary?

#ifndef LEMON_CONCEPT_CHECK_H
#define LEMON_CONCEPT_CHECK_H

namespace lemon {

  /*
    "inline" is used for ignore_unused_variable_warning()
    and function_requires() to make sure there is no
    overtarget with g++.
  */

  template <class T> inline void ignore_unused_variable_warning(const T&) { }

  ///\e
  template <class Concept>
  inline void function_requires()
  {
#if !defined(NDEBUG)
    void (Concept::*x)() = & Concept::constraints;
    ignore_unused_variable_warning(x);
#endif
  }

  ///\e
  template <typename Concept, typename Type>
  inline void checkConcept() {
#if !defined(NDEBUG)
    typedef typename Concept::template Constraints<Type> ConceptCheck;
    void (ConceptCheck::*x)() = & ConceptCheck::constraints;
    ignore_unused_variable_warning(x);
#endif
  }

} // namespace lemon

#endif // LEMON_CONCEPT_CHECK_H
