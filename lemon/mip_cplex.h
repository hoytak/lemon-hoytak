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

#ifndef LEMON_MIP_CPLEX_H
#define LEMON_MIP_CPLEX_H

///\file
///\brief Header of the LEMON-CPLEX mip solver interface.
///\ingroup lp_group


#include <lemon/lp_cplex.h>

namespace lemon {

  /// \brief Interface for the CPLEX MIP solver
  ///
  /// This class implements an interface for the CPLEX MIP solver.
  ///\ingroup lp_group
  class MipCplex : public MipSolverBase, public LpCplex{

  public:

    typedef MipSolverBase ParentMip;
    typedef LpCplex ParentLp;

    MipCplex();
    //~MipCplex();




  protected:

    virtual ColTypes _colType(int col) const;
    virtual void _colType(int col, ColTypes col_type);

    virtual LpCplex::SolveExitStatus _solve();
    virtual LpCplex::SolutionStatus _getMipStatus() const;
    virtual ParentLp::Value _getPrimal(int i) const;
    virtual ParentLp::Value _getPrimalValue() const;
  };

} //END OF NAMESPACE LEMON

#endif // END OF LEMON_MIP_CPLEX_H
