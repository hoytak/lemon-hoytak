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

#include <iostream>
#include <vector>
#include <lemon/lp_cplex.h>

extern "C" {
#include <ilcplex/cplex.h>
}


///\file
///\brief Implementation of the LEMON-CPLEX lp solver interface.
namespace lemon {

  LpCplex::LpCplex() {
    //    env = CPXopenCPLEXdevelop(&status);
    env = CPXopenCPLEX(&status);
    lp = CPXcreateprob(env, &status, "LP problem");
  }

  LpCplex::LpCplex(const LpCplex& cplex) : LpSolverBase() {
    env = CPXopenCPLEX(&status);
    lp = CPXcloneprob(env, cplex.lp, &status);
    rows = cplex.rows;
    cols = cplex.cols;
  }

  LpCplex::~LpCplex() {
    CPXfreeprob(env,&lp);
    CPXcloseCPLEX(&env);
  }

  LpSolverBase* LpCplex::_newLp()
  {
    //The first approach opens a new environment
    return new LpCplex();
  }

  LpSolverBase* LpCplex::_copyLp() {
    return new LpCplex(*this);
  }

  int LpCplex::_addCol()
  {
    int i = CPXgetnumcols(env, lp);
    Value lb[1],ub[1];
    lb[0]=-INF;
    ub[0]=INF;
    status = CPXnewcols(env, lp, 1, NULL, lb, ub, NULL, NULL);
    return i;
  }


  int LpCplex::_addRow()
  {
    //We want a row that is not constrained
    char sense[1];
    sense[0]='L';//<= constraint
    Value rhs[1];
    rhs[0]=INF;
    int i = CPXgetnumrows(env, lp);
    status = CPXnewrows(env, lp, 1, rhs, sense, NULL, NULL);
    return i;
  }


  void LpCplex::_eraseCol(int i) {
    CPXdelcols(env, lp, i, i);
  }

  void LpCplex::_eraseRow(int i) {
    CPXdelrows(env, lp, i, i);
  }

  void LpCplex::_getColName(int col, std::string &name) const
  {
    ///\bug Untested
    int storespace;
    CPXgetcolname(env, lp, 0, 0, 0, &storespace, col, col);
    if (storespace == 0) {
      name.clear();
      return;
    }

    storespace *= -1;
    std::vector<char> buf(storespace);
    char *names[1];
    int dontcare;
    ///\bug return code unchecked for error
    CPXgetcolname(env, lp, names, &*buf.begin(), storespace,
                  &dontcare, col, col);
    name = names[0];
  }

  void LpCplex::_setColName(int col, const std::string &name)
  {
    ///\bug Untested
    char *names[1];
    names[0] = const_cast<char*>(name.c_str());
    ///\bug return code unchecked for error
    CPXchgcolname(env, lp, 1, &col, names);
  }

  int LpCplex::_colByName(const std::string& name) const
  {
    int index;
    if (CPXgetcolindex(env, lp,
                       const_cast<char*>(name.c_str()), &index) == 0) {
      return index;
    }
    return -1;
  }

  ///\warning Data at index 0 is ignored in the arrays.
  void LpCplex::_setRowCoeffs(int i, ConstRowIterator b, ConstRowIterator e)
  {
    std::vector<int> indices;
    std::vector<int> rowlist;
    std::vector<Value> values;

    for(ConstRowIterator it=b; it!=e; ++it) {
      indices.push_back(it->first);
      values.push_back(it->second);
      rowlist.push_back(i);
    }

    status = CPXchgcoeflist(env, lp, values.size(),
                            &rowlist[0], &indices[0], &values[0]);
  }

  void LpCplex::_getRowCoeffs(int i, RowIterator b) const {
    int tmp1, tmp2, tmp3, length;
    CPXgetrows(env, lp, &tmp1, &tmp2, 0, 0, 0, &length, i, i);

    length = -length;
    std::vector<int> indices(length);
    std::vector<double> values(length);

    CPXgetrows(env, lp, &tmp1, &tmp2, &indices[0], &values[0],
               length, &tmp3, i, i);

    for (int i = 0; i < length; ++i) {
      *b = std::make_pair(indices[i], values[i]);
      ++b;
    }

    /// \todo implement
  }

  void LpCplex::_setColCoeffs(int i, ConstColIterator b, ConstColIterator e)
  {
    std::vector<int> indices;
    std::vector<int> collist;
    std::vector<Value> values;

    for(ConstColIterator it=b; it!=e; ++it) {
      indices.push_back(it->first);
      values.push_back(it->second);
      collist.push_back(i);
    }

    status = CPXchgcoeflist(env, lp, values.size(),
                            &indices[0], &collist[0], &values[0]);
  }

  void LpCplex::_getColCoeffs(int i, ColIterator b) const {

    int tmp1, tmp2, tmp3, length;
    CPXgetcols(env, lp, &tmp1, &tmp2, 0, 0, 0, &length, i, i);

    length = -length;
    std::vector<int> indices(length);
    std::vector<double> values(length);

    CPXgetcols(env, lp, &tmp1, &tmp2, &indices[0], &values[0],
               length, &tmp3, i, i);

    for (int i = 0; i < length; ++i) {
      *b = std::make_pair(indices[i], values[i]);
      ++b;
    }

  }

  void LpCplex::_setCoeff(int row, int col, Value value)
  {
    CPXchgcoef(env, lp, row, col, value);
  }

  LpCplex::Value LpCplex::_getCoeff(int row, int col) const
  {
    LpCplex::Value value;
    CPXgetcoef(env, lp, row, col, &value);
    return value;
  }

  void LpCplex::_setColLowerBound(int i, Value value)
  {
    int indices[1];
    indices[0]=i;
    char lu[1];
    lu[0]='L';
    Value bd[1];
    bd[0]=value;
    status = CPXchgbds(env, lp, 1, indices, lu, bd);

  }

  LpCplex::Value LpCplex::_getColLowerBound(int i) const
  {
    LpCplex::Value x;
    CPXgetlb (env, lp, &x, i, i);
    if (x <= -CPX_INFBOUND) x = -INF;
    return x;
  }

  void LpCplex::_setColUpperBound(int i, Value value)
  {
    int indices[1];
    indices[0]=i;
    char lu[1];
    lu[0]='U';
    Value bd[1];
    bd[0]=value;
    status = CPXchgbds(env, lp, 1, indices, lu, bd);
  }

  LpCplex::Value LpCplex::_getColUpperBound(int i) const
  {
    LpCplex::Value x;
    CPXgetub (env, lp, &x, i, i);
    if (x >= CPX_INFBOUND) x = INF;
    return x;
  }

  //This will be easier to implement
  void LpCplex::_setRowBounds(int i, Value lb, Value ub)
  {
    //Bad parameter
    if (lb==INF || ub==-INF) {
      //FIXME error
    }

    int cnt=1;
    int indices[1];
    indices[0]=i;
    char sense[1];

    if (lb==-INF){
      sense[0]='L';
      CPXchgsense(env, lp, cnt, indices, sense);
      CPXchgcoef(env, lp, i, -1, ub);

    }
    else{
      if (ub==INF){
        sense[0]='G';
        CPXchgsense(env, lp, cnt, indices, sense);
        CPXchgcoef(env, lp, i, -1, lb);
      }
      else{
        if (lb == ub){
          sense[0]='E';
          CPXchgsense(env, lp, cnt, indices, sense);
          CPXchgcoef(env, lp, i, -1, lb);
        }
        else{
          sense[0]='R';
          CPXchgsense(env, lp, cnt, indices, sense);
          CPXchgcoef(env, lp, i, -1, lb);
          CPXchgcoef(env, lp, i, -2, ub-lb);
        }
      }
    }
  }

//   void LpCplex::_setRowLowerBound(int i, Value value)
//   {
//     //Not implemented, obsolete
//   }

//   void LpCplex::_setRowUpperBound(int i, Value value)
//   {
//     //Not implemented, obsolete
// //     //TODO Ezt kell meg megirni
// //     //type of the problem
// //     char sense[1];
// //     status = CPXgetsense(env, lp, sense, i, i);
// //     Value rhs[1];
// //     status = CPXgetrhs(env, lp, rhs, i, i);

// //     switch (sense[0]) {
// //     case 'L'://<= constraint
// //       break;
// //     case 'E'://= constraint
// //       break;
// //     case 'G'://>= constraint
// //       break;
// //     case 'R'://ranged constraint
// //       break;
// //     default: ;
// //       //FIXME error
// //     }

// //     status = CPXchgcoef(env, lp, i, -2, value_rng);
//   }

  void LpCplex::_getRowBounds(int i, Value &lb, Value &ub) const
  {
    char sense;
    CPXgetsense(env, lp, &sense,i,i);
    lb=-INF;
    ub=INF;
    switch (sense)
      {
      case 'L':
        CPXgetcoef(env, lp, i, -1, &ub);
        break;
      case 'G':
        CPXgetcoef(env, lp, i, -1, &lb);
        break;
      case 'E':
        CPXgetcoef(env, lp, i, -1, &lb);
        ub=lb;
        break;
      case 'R':
        CPXgetcoef(env, lp, i, -1, &lb);
        Value x;
        CPXgetcoef(env, lp, i, -2, &x);
        ub=lb+x;
        break;
      }
  }

  void LpCplex::_setObjCoeff(int i, Value obj_coef)
  {
    CPXchgcoef(env, lp, -1, i, obj_coef);
  }

  LpCplex::Value LpCplex::_getObjCoeff(int i) const
  {
    Value x;
    CPXgetcoef(env, lp, -1, i, &x);
    return x;
  }

  void LpCplex::_clearObj()
  {
    for (int i=0;i< CPXgetnumcols(env, lp);++i){
      CPXchgcoef(env, lp, -1, i, 0);
    }

  }
  // The routine returns zero unless an error occurred during the
  // optimization. Examples of errors include exhausting available
  // memory (CPXERR_NO_MEMORY) or encountering invalid data in the
  // CPLEX problem object (CPXERR_NO_PROBLEM). Exceeding a
  // user-specified CPLEX limit, or proving the model infeasible or
  // unbounded, are not considered errors. Note that a zero return
  // value does not necessarily mean that a solution exists. Use query
  // routines CPXsolninfo, CPXgetstat, and CPXsolution to obtain
  // further information about the status of the optimization.
  LpCplex::SolveExitStatus LpCplex::_solve()
  {
    //CPX_PARAM_LPMETHOD
    status = CPXlpopt(env, lp);
    //status = CPXprimopt(env, lp);
#if CPX_VERSION >= 800
    if (status)
    {
      return UNSOLVED;
    }
    else
    {
      switch (CPXgetstat(env, lp))
      {
        case CPX_STAT_OPTIMAL:
        case CPX_STAT_INFEASIBLE:
        case CPX_STAT_UNBOUNDED:
          return SOLVED;
        default:
          return UNSOLVED;
      }
    }
#else
    if (status == 0){
      //We want to exclude some cases
      switch (CPXgetstat(env, lp)){
      case CPX_OBJ_LIM:
      case CPX_IT_LIM_FEAS:
      case CPX_IT_LIM_INFEAS:
      case CPX_TIME_LIM_FEAS:
      case CPX_TIME_LIM_INFEAS:
        return UNSOLVED;
      default:
        return SOLVED;
      }
    }
    else{
      return UNSOLVED;
    }
#endif
  }

  LpCplex::Value LpCplex::_getPrimal(int i) const
  {
    Value x;
    CPXgetx(env, lp, &x, i, i);
    return x;
  }

  LpCplex::Value LpCplex::_getDual(int i) const
  {
    Value y;
    CPXgetpi(env, lp, &y, i, i);
    return y;
  }

  LpCplex::Value LpCplex::_getPrimalValue() const
  {
    Value objval;
    //method = CPXgetmethod (env, lp);
    //printf("CPXgetprobtype %d \n",CPXgetprobtype(env,lp));
    CPXgetobjval(env, lp, &objval);
    //printf("Objective value: %g \n",objval);
    return objval;
  }
  bool LpCplex::_isBasicCol(int i) const
  {
    std::vector<int> cstat(CPXgetnumcols(env, lp));
    CPXgetbase(env, lp, &*cstat.begin(), NULL);
    return (cstat[i]==CPX_BASIC);
  }

//7.5-os cplex statusai (Vigyazat: a 9.0-asei masok!)
// This table lists the statuses, returned by the CPXgetstat()
// routine, for solutions to LP problems or mixed integer problems. If
// no solution exists, the return value is zero.

// For Simplex, Barrier
// 1          CPX_OPTIMAL
//          Optimal solution found
// 2          CPX_INFEASIBLE
//          Problem infeasible
// 3    CPX_UNBOUNDED
//          Problem unbounded
// 4          CPX_OBJ_LIM
//          Objective limit exceeded in Phase II
// 5          CPX_IT_LIM_FEAS
//          Iteration limit exceeded in Phase II
// 6          CPX_IT_LIM_INFEAS
//          Iteration limit exceeded in Phase I
// 7          CPX_TIME_LIM_FEAS
//          Time limit exceeded in Phase II
// 8          CPX_TIME_LIM_INFEAS
//          Time limit exceeded in Phase I
// 9          CPX_NUM_BEST_FEAS
//          Problem non-optimal, singularities in Phase II
// 10         CPX_NUM_BEST_INFEAS
//          Problem non-optimal, singularities in Phase I
// 11         CPX_OPTIMAL_INFEAS
//          Optimal solution found, unscaled infeasibilities
// 12         CPX_ABORT_FEAS
//          Aborted in Phase II
// 13         CPX_ABORT_INFEAS
//          Aborted in Phase I
// 14          CPX_ABORT_DUAL_INFEAS
//          Aborted in barrier, dual infeasible
// 15          CPX_ABORT_PRIM_INFEAS
//          Aborted in barrier, primal infeasible
// 16          CPX_ABORT_PRIM_DUAL_INFEAS
//          Aborted in barrier, primal and dual infeasible
// 17          CPX_ABORT_PRIM_DUAL_FEAS
//          Aborted in barrier, primal and dual feasible
// 18          CPX_ABORT_CROSSOVER
//          Aborted in crossover
// 19          CPX_INForUNBD
//          Infeasible or unbounded
// 20   CPX_PIVOT
//       User pivot used
//
//     Ezeket hova tegyem:
// ??case CPX_ABORT_DUAL_INFEAS
// ??case CPX_ABORT_CROSSOVER
// ??case CPX_INForUNBD
// ??case CPX_PIVOT

//Some more interesting stuff:

// CPX_PARAM_LPMETHOD  1062  int  LPMETHOD
// 0 Automatic
// 1 Primal Simplex
// 2 Dual Simplex
// 3 Network Simplex
// 4 Standard Barrier
// Default: 0
// Description: Method for linear optimization.
// Determines which algorithm is used when CPXlpopt() (or "optimize"
// in the Interactive Optimizer) is called. Currently the behavior of
// the "Automatic" setting is that CPLEX simply invokes the dual
// simplex method, but this capability may be expanded in the future
// so that CPLEX chooses the method based on problem characteristics
#if CPX_VERSION < 900
  void statusSwitch(CPXENVptr env,int& stat){
    int lpmethod;
    CPXgetintparam (env,CPX_PARAM_LPMETHOD,&lpmethod);
    if (lpmethod==2){
      if (stat==CPX_UNBOUNDED){
        stat=CPX_INFEASIBLE;
      }
      else{
        if (stat==CPX_INFEASIBLE)
          stat=CPX_UNBOUNDED;
      }
    }
  }
#else
  void statusSwitch(CPXENVptr,int&){}
#endif

  LpCplex::SolutionStatus LpCplex::_getPrimalStatus() const
  {
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

    int stat = CPXgetstat(env, lp);
#if CPX_VERSION >= 800
    switch (stat)
    {
      case CPX_STAT_OPTIMAL:
        return OPTIMAL;
      case CPX_STAT_UNBOUNDED:
        return INFINITE;
      case CPX_STAT_INFEASIBLE:
        return INFEASIBLE;
      default:
        return UNDEFINED;
    }
#else
    statusSwitch(env,stat);
    //CPXgetstat(env, lp);
    //printf("A primal status: %d, CPX_OPTIMAL=%d \n",stat,CPX_OPTIMAL);
    switch (stat) {
    case 0:
      return UNDEFINED; //Undefined
    case CPX_OPTIMAL://Optimal
      return OPTIMAL;
    case CPX_UNBOUNDED://Unbounded
      return INFEASIBLE;//In case of dual simplex
      //return INFINITE;
    case CPX_INFEASIBLE://Infeasible
 //    case CPX_IT_LIM_INFEAS:
//     case CPX_TIME_LIM_INFEAS:
//     case CPX_NUM_BEST_INFEAS:
//     case CPX_OPTIMAL_INFEAS:
//     case CPX_ABORT_INFEAS:
//     case CPX_ABORT_PRIM_INFEAS:
//     case CPX_ABORT_PRIM_DUAL_INFEAS:
      return INFINITE;//In case of dual simplex
      //return INFEASIBLE;
//     case CPX_OBJ_LIM:
//     case CPX_IT_LIM_FEAS:
//     case CPX_TIME_LIM_FEAS:
//     case CPX_NUM_BEST_FEAS:
//     case CPX_ABORT_FEAS:
//     case CPX_ABORT_PRIM_DUAL_FEAS:
//       return FEASIBLE;
    default:
      return UNDEFINED; //Everything else comes here
      //FIXME error
    }
#endif
  }

//9.0-as cplex verzio statusai
// CPX_STAT_ABORT_DUAL_OBJ_LIM
// CPX_STAT_ABORT_IT_LIM
// CPX_STAT_ABORT_OBJ_LIM
// CPX_STAT_ABORT_PRIM_OBJ_LIM
// CPX_STAT_ABORT_TIME_LIM
// CPX_STAT_ABORT_USER
// CPX_STAT_FEASIBLE_RELAXED
// CPX_STAT_INFEASIBLE
// CPX_STAT_INForUNBD
// CPX_STAT_NUM_BEST
// CPX_STAT_OPTIMAL
// CPX_STAT_OPTIMAL_FACE_UNBOUNDED
// CPX_STAT_OPTIMAL_INFEAS
// CPX_STAT_OPTIMAL_RELAXED
// CPX_STAT_UNBOUNDED

  LpCplex::SolutionStatus LpCplex::_getDualStatus() const
  {
    int stat = CPXgetstat(env, lp);
#if CPX_VERSION >= 800
    switch (stat)
    {
      case CPX_STAT_OPTIMAL:
        return OPTIMAL;
      case CPX_STAT_UNBOUNDED:
        return INFEASIBLE;
      default:
        return UNDEFINED;
    }
#else
    statusSwitch(env,stat);
    switch (stat) {
    case 0:
      return UNDEFINED; //Undefined
    case CPX_OPTIMAL://Optimal
      return OPTIMAL;
    case CPX_UNBOUNDED:
     return INFEASIBLE;
    default:
      return UNDEFINED; //Everything else comes here
      //FIXME error
    }
#endif
  }

  LpCplex::ProblemTypes LpCplex::_getProblemType() const
  {
    int stat = CPXgetstat(env, lp);
#if CPX_VERSION >= 800
    switch (stat)
    {
      case CPX_STAT_OPTIMAL:
        return PRIMAL_DUAL_FEASIBLE;
      case CPX_STAT_UNBOUNDED:
         return PRIMAL_FEASIBLE_DUAL_INFEASIBLE;
      default:
        return UNKNOWN;
    }
#else
    switch (stat) {
    case CPX_OPTIMAL://Optimal
        return PRIMAL_DUAL_FEASIBLE;
    case CPX_UNBOUNDED:
         return PRIMAL_FEASIBLE_DUAL_INFEASIBLE;
//         return PRIMAL_INFEASIBLE_DUAL_FEASIBLE;
//         return PRIMAL_DUAL_INFEASIBLE;

//Seems to be that this is all we can say for sure
    default:
        //In all other cases
        return UNKNOWN;
      //FIXME error
    }
#endif
  }

  void LpCplex::_setMax()
  {
    CPXchgobjsen(env, lp, CPX_MAX);
   }
  void LpCplex::_setMin()
  {
    CPXchgobjsen(env, lp, CPX_MIN);
   }

  bool LpCplex::_isMax() const
  {
    if (CPXgetobjsen(env, lp)==CPX_MAX)
      return true;
    else
      return false;
  }

} //namespace lemon

