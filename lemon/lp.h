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

#ifndef LEMON_LP_H
#define LEMON_LP_H

#include<lemon/config.h>


#ifdef HAVE_GLPK
#include <lemon/lp_glpk.h>
#include <lemon/mip_glpk.h>
#elif HAVE_CPLEX
#include <lemon/lp_cplex.h>
#include <lemon/mip_cplex.h>
#elif HAVE_SOPLEX
#include <lemon/lp_soplex.h>
#endif

///\file
///\brief Defines a default LP solver
///\ingroup lp_group
namespace lemon {

#ifdef DOXYGEN
  ///The default LP solver identifier

  ///The default LP solver identifier.
  ///\ingroup lp_group
  ///
  ///Currently, the possible values are \c GLPK or \c CPLEX
#define DEFAULT_LP SOLVER
  ///The default LP solver

  ///The default LP solver.
  ///\ingroup lp_group
  ///
  ///Currently, it is either \c LpGlpk or \c LpCplex
  typedef LpGlpk Lp;
  ///The default LP solver identifier string

  ///The default LP solver identifier string.
  ///\ingroup lp_group
  ///
  ///Currently, the possible values are "GLPK" or "CPLEX"
  const char default_solver_name[]="SOLVER";

  ///The default ILP solver.

  ///The default ILP solver.
  ///\ingroup lp_group
  ///
  ///Currently, it is either \c LpGlpk or \c LpCplex
  typedef MipGlpk Mip;
#else
#ifdef HAVE_GLPK
#define DEFAULT_LP GLPK
  typedef LpGlpk Lp;
  typedef MipGlpk Mip;
  const char default_solver_name[]="GLPK";
#elif HAVE_CPLEX
#define DEFAULT_LP CPLEX
  typedef LpCplex Lp;
  typedef MipCplex Mip;
  const char default_solver_name[]="CPLEX";
#elif HAVE_SOPLEX
#define DEFAULT_LP SOPLEX
  typedef LpSoplex Lp;
  const char default_solver_name[]="SOPLEX";
#endif
#endif

} //namespace lemon

#endif //LEMON_LP_H
