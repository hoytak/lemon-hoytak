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
///\brief Implementation of the LEMON-GLPK lp solver interface.

#include <lemon/lp_glpk.h>
//#include <iostream>

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

  LpGlpk::LpGlpk() : Parent() {
    solved = false;
    rows = _lp_bits::LpId(1);
    cols = _lp_bits::LpId(1);
    lp = LEMON_glp(create_prob)();
    LEMON_glp(create_index)(lp);
    messageLevel(0);
  }

  LpGlpk::LpGlpk(const LpGlpk &glp) : Parent() {
    solved = false;
    rows = _lp_bits::LpId(1);
    cols = _lp_bits::LpId(1);
    lp = LEMON_glp(create_prob)();
    LEMON_glp(create_index)(lp);
    messageLevel(0);
    //Coefficient matrix, row bounds
    LEMON_glp(add_rows)(lp, LEMON_glp(get_num_rows)(glp.lp));
    LEMON_glp(add_cols)(lp, LEMON_glp(get_num_cols)(glp.lp));
    int len;
    std::vector<int> ind(1+LEMON_glp(get_num_cols)(glp.lp));
    std::vector<Value> val(1+LEMON_glp(get_num_cols)(glp.lp));
    for (int i=1;i<=LEMON_glp(get_num_rows)(glp.lp);++i)
      {
        len=LEMON_glp(get_mat_row)(glp.lp,i,&*ind.begin(),&*val.begin());
        LEMON_glp(set_mat_row)(lp, i,len,&*ind.begin(),&*val.begin());
        LEMON_glp(set_row_bnds)(lp,i,
                                LEMON_glp(get_row_type)(glp.lp,i),
                                LEMON_glp(get_row_lb)(glp.lp,i),
                                LEMON_glp(get_row_ub)(glp.lp,i));
      }

    //Objective function, coloumn bounds
    LEMON_glp(set_obj_dir)(lp, LEMON_glp(get_obj_dir)(glp.lp));
    //Objectif function's constant term treated separately
    LEMON_glp(set_obj_coef)(lp,0,LEMON_glp(get_obj_coef)(glp.lp,0));
    for (int i=1;i<=LEMON_glp(get_num_cols)(glp.lp);++i)
      {
        LEMON_glp(set_obj_coef)(lp,i,
                                LEMON_glp(get_obj_coef)(glp.lp,i));
        LEMON_glp(set_col_bnds)(lp,i,
                                LEMON_glp(get_col_type)(glp.lp,i),
                                LEMON_glp(get_col_lb)(glp.lp,i),
                                LEMON_glp(get_col_ub)(glp.lp,i));
      }
    rows = glp.rows;
    cols = glp.cols;
  }

  LpGlpk::~LpGlpk() {
    LEMON_glp(delete_prob)(lp);
  }

  int LpGlpk::_addCol() {
    int i=LEMON_glp(add_cols)(lp, 1);
    LEMON_glp(set_col_bnds)(lp, i, LEMON_GLP(FR), 0.0, 0.0);
    solved = false;
    return i;
  }

  ///\e


  LpSolverBase* LpGlpk::_newLp()
  {
    LpGlpk* newlp = new LpGlpk;
    return newlp;
  }

  ///\e

  LpSolverBase* LpGlpk::_copyLp()
  {
    LpGlpk *newlp = new LpGlpk(*this);
    return newlp;
  }

  int LpGlpk::_addRow() {
    int i=LEMON_glp(add_rows)(lp, 1);
    solved = false;
    return i;
  }


  void LpGlpk::_eraseCol(int i) {
    int ca[2];
    ca[1]=i;
    LEMON_glp(del_cols)(lp, 1, ca);
    solved = false;
  }

  void LpGlpk::_eraseRow(int i) {
    int ra[2];
    ra[1]=i;
    LEMON_glp(del_rows)(lp, 1, ra);
    solved = false;
  }

  void LpGlpk::_getColName(int c, std::string & name) const
  {

    const char *n = LEMON_glp(get_col_name)(lp,c);
    name = n?n:"";
  }


  void LpGlpk::_setColName(int c, const std::string & name)
  {
    LEMON_glp(set_col_name)(lp,c,const_cast<char*>(name.c_str()));

  }

  int LpGlpk::_colByName(const std::string& name) const
  {
    int k = LEMON_glp(find_col)(lp, const_cast<char*>(name.c_str()));
    return k > 0 ? k : -1;
  }


  void LpGlpk::_setRowCoeffs(int i, ConstRowIterator b, ConstRowIterator e)
  {
    std::vector<int> indices;
    std::vector<Value> values;

    indices.push_back(0);
    values.push_back(0);

    for(ConstRowIterator it=b; it!=e; ++it) {
      indices.push_back(it->first);
      values.push_back(it->second);
    }

    LEMON_glp(set_mat_row)(lp, i, values.size() - 1,
                                &indices[0], &values[0]);

    solved = false;
  }

  void LpGlpk::_getRowCoeffs(int ix, RowIterator b) const
  {
    int length = LEMON_glp(get_mat_row)(lp, ix, 0, 0);

    std::vector<int> indices(length + 1);
    std::vector<Value> values(length + 1);

    LEMON_glp(get_mat_row)(lp, ix, &indices[0], &values[0]);

    for (int i = 1; i <= length; ++i) {
      *b = std::make_pair(indices[i], values[i]);
      ++b;
    }
  }

  void LpGlpk::_setColCoeffs(int ix, ConstColIterator b, ConstColIterator e) {

    std::vector<int> indices;
    std::vector<Value> values;

    indices.push_back(0);
    values.push_back(0);

    for(ConstColIterator it=b; it!=e; ++it) {
      indices.push_back(it->first);
      values.push_back(it->second);
    }

    LEMON_glp(set_mat_col)(lp, ix, values.size() - 1,
                                &indices[0], &values[0]);

    solved = false;
  }

  void LpGlpk::_getColCoeffs(int ix, ColIterator b) const
  {
    int length = LEMON_glp(get_mat_col)(lp, ix, 0, 0);

    std::vector<int> indices(length + 1);
    std::vector<Value> values(length + 1);

    LEMON_glp(get_mat_col)(lp, ix, &indices[0], &values[0]);

    for (int i = 1; i <= length; ++i) {
      *b = std::make_pair(indices[i], values[i]);
      ++b;
    }
  }

  void LpGlpk::_setCoeff(int ix, int jx, Value value)
  {

    if (LEMON_glp(get_num_cols)(lp) < LEMON_glp(get_num_rows)(lp)) {

      int length=LEMON_glp(get_mat_row)(lp, ix, 0, 0);

      std::vector<int> indices(length + 2);
      std::vector<Value> values(length + 2);

      LEMON_glp(get_mat_row)(lp, ix, &indices[0], &values[0]);

      //The following code does not suppose that the elements of the
      //array indices are sorted
      bool found=false;
      for (int i = 1; i <= length; ++i) {
        if (indices[i]==jx){
          found=true;
          values[i]=value;
          break;
        }
      }
      if (!found){
        ++length;
        indices[length]=jx;
        values[length]=value;
      }

      LEMON_glp(set_mat_row)(lp, ix, length, &indices[0], &values[0]);

    } else {

      int length=LEMON_glp(get_mat_col)(lp, jx, 0, 0);

      std::vector<int> indices(length + 2);
      std::vector<Value> values(length + 2);

      LEMON_glp(get_mat_col)(lp, jx, &indices[0], &values[0]);

      //The following code does not suppose that the elements of the
      //array indices are sorted
      bool found=false;
      for (int i = 1; i <= length; ++i) {
        if (indices[i]==ix){
          found=true;
          values[i]=value;
          break;
        }
      }
      if (!found){
        ++length;
        indices[length]=ix;
        values[length]=value;
      }

      LEMON_glp(set_mat_col)(lp, jx, length, &indices[0], &values[0]);
    }

    solved = false;
  }

  LpGlpk::Value LpGlpk::_getCoeff(int ix, int jx) const
  {

    int length=LEMON_glp(get_mat_row)(lp, ix, 0, 0);

    std::vector<int> indices(length + 1);
    std::vector<Value> values(length + 1);

    LEMON_glp(get_mat_row)(lp, ix, &indices[0], &values[0]);

    //The following code does not suppose that the elements of the
    //array indices are sorted
    for (int i = 1; i <= length; ++i) {
      if (indices[i]==jx){
        return values[i];
      }
    }
    return 0;

  }


  void LpGlpk::_setColLowerBound(int i, Value lo)
  {
    if (lo==INF) {
      //FIXME error
    }
    int b=LEMON_glp(get_col_type)(lp, i);
    double up=LEMON_glp(get_col_ub)(lp, i);
    if (lo==-INF) {
      switch (b) {
      case LEMON_GLP(FR):
      case LEMON_GLP(LO):
        LEMON_glp(set_col_bnds)(lp, i, LEMON_GLP(FR), lo, up);
        break;
      case LEMON_GLP(UP):
        break;
      case LEMON_GLP(DB):
      case LEMON_GLP(FX):
        LEMON_glp(set_col_bnds)(lp, i, LEMON_GLP(UP), lo, up);
        break;
      default: ;
        //FIXME error
      }
    } else {
      switch (b) {
      case LEMON_GLP(FR):
      case LEMON_GLP(LO):
        LEMON_glp(set_col_bnds)(lp, i, LEMON_GLP(LO), lo, up);
        break;
      case LEMON_GLP(UP):
      case LEMON_GLP(DB):
      case LEMON_GLP(FX):
        if (lo==up)
          LEMON_glp(set_col_bnds)(lp, i, LEMON_GLP(FX), lo, up);
        else
          LEMON_glp(set_col_bnds)(lp, i, LEMON_GLP(DB), lo, up);
        break;
      default: ;
        //FIXME error
      }
    }

    solved = false;
  }

  LpGlpk::Value LpGlpk::_getColLowerBound(int i) const
  {
    int b=LEMON_glp(get_col_type)(lp, i);
      switch (b) {
      case LEMON_GLP(LO):
      case LEMON_GLP(DB):
      case LEMON_GLP(FX):
        return LEMON_glp(get_col_lb)(lp, i);
      default: ;
        return -INF;
      }
  }

  void LpGlpk::_setColUpperBound(int i, Value up)
  {
    if (up==-INF) {
      //FIXME error
    }
    int b=LEMON_glp(get_col_type)(lp, i);
    double lo=LEMON_glp(get_col_lb)(lp, i);
    if (up==INF) {
      switch (b) {
      case LEMON_GLP(FR):
      case LEMON_GLP(LO):
        break;
      case LEMON_GLP(UP):
        LEMON_glp(set_col_bnds)(lp, i, LEMON_GLP(FR), lo, up);
        break;
      case LEMON_GLP(DB):
      case LEMON_GLP(FX):
        LEMON_glp(set_col_bnds)(lp, i, LEMON_GLP(LO), lo, up);
        break;
      default: ;
        //FIXME error
      }
    } else {
      switch (b) {
      case LEMON_GLP(FR):
        LEMON_glp(set_col_bnds)(lp, i, LEMON_GLP(UP), lo, up);
        break;
      case LEMON_GLP(UP):
        LEMON_glp(set_col_bnds)(lp, i, LEMON_GLP(UP), lo, up);
        break;
      case LEMON_GLP(LO):
      case LEMON_GLP(DB):
      case LEMON_GLP(FX):
        if (lo==up)
          LEMON_glp(set_col_bnds)(lp, i, LEMON_GLP(FX), lo, up);
        else
          LEMON_glp(set_col_bnds)(lp, i, LEMON_GLP(DB), lo, up);
        break;
      default: ;
        //FIXME error
      }
    }

    solved = false;
  }

  LpGlpk::Value LpGlpk::_getColUpperBound(int i) const
  {
    int b=LEMON_glp(get_col_type)(lp, i);
      switch (b) {
      case LEMON_GLP(UP):
      case LEMON_GLP(DB):
      case LEMON_GLP(FX):
        return LEMON_glp(get_col_ub)(lp, i);
      default: ;
        return INF;
      }
  }

  void LpGlpk::_setRowBounds(int i, Value lb, Value ub)
  {
    //Bad parameter
    if (lb==INF || ub==-INF) {
      //FIXME error
    }

    if (lb == -INF){
      if (ub == INF){
        LEMON_glp(set_row_bnds)(lp, i, LEMON_GLP(FR), lb, ub);
      }
      else{
        LEMON_glp(set_row_bnds)(lp, i, LEMON_GLP(UP), lb, ub);
      }
    }
    else{
      if (ub==INF){
        LEMON_glp(set_row_bnds)(lp, i, LEMON_GLP(LO), lb, ub);

      }
      else{
        if (lb == ub){
          LEMON_glp(set_row_bnds)(lp, i, LEMON_GLP(FX), lb, ub);
        }
        else{
          LEMON_glp(set_row_bnds)(lp, i, LEMON_GLP(DB), lb, ub);
        }
      }
    }

    solved = false;
  }

  void LpGlpk::_getRowBounds(int i, Value &lb, Value &ub) const
  {

    int b=LEMON_glp(get_row_type)(lp, i);
    switch (b) {
    case LEMON_GLP(FR):
    case LEMON_GLP(UP):
      lb = -INF;
        break;
    default:
      lb=LEMON_glp(get_row_lb)(lp, i);
    }

    switch (b) {
    case LEMON_GLP(FR):
    case LEMON_GLP(LO):
      ub = INF;
        break;
    default:
      ub=LEMON_glp(get_row_ub)(lp, i);
    }

  }

  void LpGlpk::_setObjCoeff(int i, Value obj_coef)
  {
    //i=0 means the constant term (shift)
    LEMON_glp(set_obj_coef)(lp, i, obj_coef);

    solved = false;
  }

  LpGlpk::Value LpGlpk::_getObjCoeff(int i) const {
    //i=0 means the constant term (shift)
    return LEMON_glp(get_obj_coef)(lp, i);
  }

  void LpGlpk::_clearObj()
  {
    for (int i=0;i<=LEMON_glp(get_num_cols)(lp);++i){
      LEMON_glp(set_obj_coef)(lp, i, 0);
    }

    solved = false;
  }

  LpGlpk::SolveExitStatus LpGlpk::_solve()
  {
    // A way to check the problem to be solved
    //LEMON_glp(write_cpxlp(lp,"naittvan.cpx");

    LEMON_lpx(std_basis)(lp);
    int i =  LEMON_lpx(simplex)(lp);

    switch (i) {
    case LEMON_LPX(E_OK):
      solved = true;
      return SOLVED;
    default:
      return UNSOLVED;
    }
  }

  LpGlpk::Value LpGlpk::_getPrimal(int i) const
  {
    return LEMON_glp(get_col_prim)(lp,i);
  }

  LpGlpk::Value LpGlpk::_getDual(int i) const
  {
    return LEMON_glp(get_row_dual)(lp,i);
  }

  LpGlpk::Value LpGlpk::_getPrimalValue() const
  {
    return LEMON_glp(get_obj_val)(lp);
  }
  bool LpGlpk::_isBasicCol(int i) const
  {
    return (LEMON_glp(get_col_stat)(lp, i)==LEMON_GLP(BS));
  }


  LpGlpk::SolutionStatus LpGlpk::_getPrimalStatus() const
  {
    if (!solved) return UNDEFINED;
    int stat=  LEMON_lpx(get_status)(lp);
    switch (stat) {
    case LEMON_LPX(UNDEF)://Undefined (no solve has been run yet)
      return UNDEFINED;
    case LEMON_LPX(NOFEAS)://There is no feasible solution (primal, I guess)
    case LEMON_LPX(INFEAS)://Infeasible
      return INFEASIBLE;
    case LEMON_LPX(UNBND)://Unbounded
      return INFINITE;
    case LEMON_LPX(FEAS)://Feasible
      return FEASIBLE;
    case LEMON_LPX(OPT)://Feasible
      return OPTIMAL;
    default:
      return UNDEFINED; //to avoid gcc warning
      //FIXME error
    }
  }

  LpGlpk::SolutionStatus LpGlpk::_getDualStatus() const
  {
    if (!solved) return UNDEFINED;
    switch (LEMON_lpx(get_dual_stat)(lp)) {
    case LEMON_LPX(D_UNDEF)://Undefined (no solve has been run yet)
      return UNDEFINED;
    case LEMON_LPX(D_NOFEAS)://There is no dual feasible solution
//    case LEMON_LPX(D_INFEAS://Infeasible
      return INFEASIBLE;
    case LEMON_LPX(D_FEAS)://Feasible
      switch (LEMON_lpx(get_status)(lp)) {
      case LEMON_LPX(NOFEAS):
        return INFINITE;
      case LEMON_LPX(OPT):
        return OPTIMAL;
      default:
        return FEASIBLE;
      }
    default:
      return UNDEFINED; //to avoid gcc warning
      //FIXME error
    }
  }

  LpGlpk::ProblemTypes LpGlpk::_getProblemType() const
  {
    if (!solved) return UNKNOWN;
      //int stat=  LEMON_glp(get_status(lp);
    int statp=  LEMON_lpx(get_prim_stat)(lp);
    int statd=  LEMON_lpx(get_dual_stat)(lp);
    if (statp==LEMON_LPX(P_FEAS) && statd==LEMON_LPX(D_FEAS))
        return PRIMAL_DUAL_FEASIBLE;
    if (statp==LEMON_LPX(P_FEAS) && statd==LEMON_LPX(D_NOFEAS))
        return PRIMAL_FEASIBLE_DUAL_INFEASIBLE;
    if (statp==LEMON_LPX(P_NOFEAS) && statd==LEMON_LPX(D_FEAS))
        return PRIMAL_INFEASIBLE_DUAL_FEASIBLE;
    if (statp==LEMON_LPX(P_NOFEAS) && statd==LEMON_LPX(D_NOFEAS))
        return PRIMAL_DUAL_INFEASIBLE;
    //In all other cases
    return UNKNOWN;
  }

  void LpGlpk::_setMax()
  {
    solved = false;
    LEMON_glp(set_obj_dir)(lp, LEMON_GLP(MAX));
  }

  void LpGlpk::_setMin()
  {
    solved = false;
    LEMON_glp(set_obj_dir)(lp, LEMON_GLP(MIN));
  }

  bool LpGlpk::_isMax() const
  {
    return (LEMON_glp(get_obj_dir)(lp)==LEMON_GLP(MAX));
  }



  void LpGlpk::messageLevel(int m)
  {
    LEMON_lpx(set_int_parm)(lp, LEMON_LPX(K_MSGLEV), m);
  }

  void LpGlpk::presolver(bool b)
  {
    LEMON_lpx(set_int_parm)(lp, LEMON_LPX(K_PRESOL), b);
  }


} //END OF NAMESPACE LEMON
