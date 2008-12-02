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
///\brief Implementation of the LEMON-GLPK mip solver interface.

#include <lemon/mip_glpk.h>

extern "C" {
#include <glpk.h>
}

#if GLP_MAJOR_VERSION > 4 || (GLP_MAJOR_VERSION == 4 && GLP_MINOR_VERSION > 15)
#define LEMON_glp(func) (glp_##func)
#define LEMON_lpx(func) (lpx_##func)

#define LEMON_GLP(def) (GLP_##def)
#define LEMON_LPX(def) (LPX_##def)

#else

#define LEMON_glp(func) (lpx_##func)
#define LEMON_lpx(func) (lpx_##func)

#define LEMON_GLP(def) (LPX_##def)
#define LEMON_LPX(def) (LPX_##def)

#endif

namespace lemon {

  MipGlpk::MipGlpk() {
#if !(GLP_MAJOR_VERSION > 4 || \
      (GLP_MAJOR_VERSION == 4 && GLP_MINOR_VERSION > 15))
    LEMON_lpx(set_class)(lp,LEMON_GLP(MIP));
#endif
  }

  void MipGlpk::_colType(int i, MipGlpk::ColTypes col_type){
    switch (col_type){
      case INT:
        LEMON_glp(set_col_kind)(lp,i,LEMON_GLP(IV));
        break;
      case REAL:
        LEMON_glp(set_col_kind)(lp,i,LEMON_GLP(CV));
        break;
    default:;
        //FIXME problem
    }
  }

  MipGlpk::ColTypes MipGlpk::_colType(int i) const {
    switch (LEMON_glp(get_col_kind)(lp,i)){
    case LEMON_GLP(IV):
      return INT;//Or binary
    case LEMON_GLP(CV):
      return REAL;
    default:
      return REAL;//Error!
    }

  }

  LpGlpk::SolveExitStatus MipGlpk::_solve() {
    int result = LEMON_lpx(simplex)(lp);

    // hack: mip does not contain integer variable
#if GLP_MAJOR_VERSION == 4 && GLP_MINOR_VERSION == 16
    int tmp = -1;
    if (LEMON_glp(get_num_int(lp)) == 0) {
      tmp = LEMON_lpx(add_cols)(lp, 1);
      LEMON_glp(set_col_bnds)(lp, tmp, LEMON_GLP(FX), 0.0, 0.0);
      LEMON_glp(set_col_kind)(lp, tmp, LEMON_GLP(IV));
    }
#endif

    if (LEMON_lpx(get_status)(lp)==LEMON_LPX(OPT)) {
      //Maybe we could try the routine lpx_intopt(lp), a revised
      //version of lpx_integer

      result = LEMON_lpx(integer)(lp);
      switch (result){
      case LEMON_LPX(E_OK):
        solved = true;
        break;
      default:
        solved = false;
      }
    } else {
      solved = false;
    }
#if GLP_MAJOR_VERSION == 4 && GLP_MINOR_VERSION == 16
    if (tmp != -1) {
      int tmpa[2];
      tmpa[1] = tmp;
      LEMON_lpx(del_cols)(lp, 1, tmpa);
    }
#endif
    return solved ? SOLVED : UNSOLVED;
  }


  LpGlpk::SolutionStatus MipGlpk::_getMipStatus() const {

    if (LEMON_lpx(get_status)(lp)==LEMON_LPX(OPT)){
      //Meg kell nezni: ha az LP is infinite, akkor ez is, ha az is
      //infeasible, akkor ez is, de ez lehet maskepp is infeasible.
      int stat= LEMON_lpx(mip_status)(lp);

      switch (stat) {
      case LEMON_LPX(I_UNDEF)://Undefined (no solve has been run yet)
        return UNDEFINED;
      case LEMON_LPX(I_NOFEAS)://There is no feasible integral solution
        return INFEASIBLE;
        //     case LEMON_LPX(UNBND)://Unbounded
        //       return INFINITE;
      case LEMON_LPX(I_FEAS)://Feasible
        return FEASIBLE;
      case LEMON_LPX(I_OPT)://Feasible
        return OPTIMAL;
      default:
        return UNDEFINED; //to avoid gcc warning
      //FIXME error
      }
    }
    else
      return UNDEFINED; //Maybe we could refine this: what does the LP
                        //relaxation look like

  }

  MipGlpk::Value MipGlpk::_getPrimal(int i) const {
    return LEMON_glp(mip_col_val)(lp,i);
  }

  MipGlpk::Value MipGlpk::_getPrimalValue() const {
    return LEMON_glp(mip_obj_val)(lp);
  }
} //END OF NAMESPACE LEMON
