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
#elif HAVE_CPLEX
#include <lemon/lp_cplex.h>
#elif HAVE_SOPLEX
#include <lemon/lp_soplex.h>
#elif HAVE_CLP
#include <lemon/lp_clp.h>
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
  ///Currently, the possible values are \c LP_GLPK, \c LP_CPLEX, \c
  ///LP_SOPLEX or \c LP_CLP
#define LEMON_DEFAULT_LP SOLVER
  ///The default LP solver

  ///The default LP solver.
  ///\ingroup lp_group
  ///
  ///Currently, it is either \c LpGlpk, \c LpCplex, \c LpSoplex or \c LpClp
  typedef LpGlpk Lp;

  ///The default MIP solver identifier

  ///The default MIP solver identifier.
  ///\ingroup lp_group
  ///
  ///Currently, the possible values are \c MIP_GLPK or \c MIP_CPLEX
#define LEMON_DEFAULT_MIP SOLVER
  ///The default MIP solver.

  ///The default MIP solver.
  ///\ingroup lp_group
  ///
  ///Currently, it is either \c MipGlpk or \c MipCplex
  typedef MipGlpk Mip;
#else
#ifdef HAVE_GLPK
# define LEMON_DEFAULT_LP LP_GLPK
  typedef LpGlpk Lp;
# define LEMON_DEFAULT_MIP MIP_GLPK
  typedef MipGlpk Mip;
#elif HAVE_CPLEX
# define LEMON_DEFAULT_LP LP_CPLEX
  typedef LpCplex Lp;
# define LEMON_DEFAULT_MIP MIP_CPLEX
  typedef MipCplex Mip;
#elif HAVE_SOPLEX
# define DEFAULT_LP LP_SOPLEX
  typedef LpSoplex Lp;
#elif HAVE_CLP
# define DEFAULT_LP LP_CLP
  typedef LpClp Lp;  
#endif
#endif

} //namespace lemon

#endif //LEMON_LP_H
