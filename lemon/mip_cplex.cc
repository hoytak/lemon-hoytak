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

///\file
///\brief Implementation of the LEMON-CPLEX mip solver interface.

#include <lemon/mip_cplex.h>

extern "C" {
#include <ilcplex/cplex.h>
}

namespace lemon {

  MipCplex::MipCplex() {
    //This is unnecessary: setting integrality constraints on
    //variables will set this, too

    ///\todo The constant CPXPROB_MIP is
    ///called CPXPROB_MILP in later versions
#if CPX_VERSION < 800
    CPXchgprobtype( env,  lp, CPXPROB_MIP);
#else
    CPXchgprobtype( env,  lp, CPXPROB_MILP);
#endif

  }

  void MipCplex::_colType(int i, MipCplex::ColTypes col_type){

    // Note If a variable is to be changed to binary, a call to CPXchgbds
    // should also be made to change the bounds to 0 and 1.

    int indices[1];
    indices[0]=i;
    char ctype[1];
    switch (col_type){
      case INT:
        ctype[0]=CPX_INTEGER;//'I'
        break;
      case REAL:
        ctype[0]=CPX_CONTINUOUS        ;//'C'
        break;
    default:;
        //FIXME problem
    }
    CPXchgctype (env, lp, 1, indices, ctype);
  }

  MipCplex::ColTypes MipCplex::_colType(int i) const {

    char ctype[1];
    CPXgetctype (env, lp, ctype, i, i);
    switch (ctype[0]){

    case CPX_INTEGER:
      return INT;
    case CPX_CONTINUOUS:
      return REAL;
    default:
      return REAL;//Error!
    }

  }

  LpCplex::SolveExitStatus MipCplex::_solve(){

    status = CPXmipopt (env, lp);
    if (status==0)
      return SOLVED;
    else
      return UNSOLVED;

  }


  LpCplex::SolutionStatus MipCplex::_getMipStatus() const {

    int stat = CPXgetstat(env, lp);

    //Fortunately, MIP statuses did not change for cplex 8.0
    switch (stat)
    {
      case CPXMIP_OPTIMAL:
        // Optimal integer solution has been found.
      case CPXMIP_OPTIMAL_TOL:
        // Optimal soluton with the tolerance defined by epgap or epagap has
        // been found.
        return OPTIMAL;
        //This also exists in later issues
        //    case CPXMIP_UNBOUNDED:
        //return INFINITE;
      case CPXMIP_INFEASIBLE:
        return INFEASIBLE;
      default:
        return UNDEFINED;
    }
    //Unboundedness not treated well: the following is from cplex 9.0 doc
    // About Unboundedness

    // The treatment of models that are unbounded involves a few
    // subtleties. Specifically, a declaration of unboundedness means that
    // ILOG CPLEX has determined that the model has an unbounded
    // ray. Given any feasible solution x with objective z, a multiple of
    // the unbounded ray can be added to x to give a feasible solution
    // with objective z-1 (or z+1 for maximization models). Thus, if a
    // feasible solution exists, then the optimal objective is
    // unbounded. Note that ILOG CPLEX has not necessarily concluded that
    // a feasible solution exists. Users can call the routine CPXsolninfo
    // to determine whether ILOG CPLEX has also concluded that the model
    // has a feasible solution.

  }

  MipCplex::Value MipCplex::_getPrimal(int i) const {
    Value x;
    CPXgetmipx(env, lp, &x, i, i);
    return x;
  }

  MipCplex::Value MipCplex::_getPrimalValue() const {
    Value objval;
    CPXgetmipobjval(env, lp, &objval);
    return objval;
  }
} //END OF NAMESPACE LEMON
