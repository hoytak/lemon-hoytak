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

#ifndef LEMON_MIP_GLPK_H
#define LEMON_MIP_GLPK_H

///\file
///\brief Header of the LEMON-GLPK mip solver interface.
///\ingroup lp_group


#include <lemon/lp_glpk.h>

namespace lemon {
  /// \brief Interface for the GLPK MIP solver
  ///
  /// This class implements an interface for the GLPK MIP solver.
  ///\ingroup lp_group
  class MipGlpk : public MipSolverBase, public LpGlpk{

  public:

    typedef MipSolverBase ParentMip;
    typedef LpGlpk ParentLp;

    MipGlpk();
    //~MipGlpk();



  protected:

    virtual ColTypes _colType(int col) const;
    virtual void _colType(int col, ColTypes col_type);

    virtual LpGlpk::SolveExitStatus _solve();
    virtual LpGlpk::SolutionStatus _getMipStatus() const;
    virtual ParentLp::Value _getPrimal(int i) const;
    virtual ParentLp::Value _getPrimalValue() const;
  };

} //END OF NAMESPACE LEMON

#endif // END OF LEMON_MIP_GLPK_H
