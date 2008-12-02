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

#include <lemon/lp_skeleton.h>

///\file
///\brief A skeleton file to implement LP solver interfaces
namespace lemon {

  LpSolverBase* LpSkeleton::_newLp()
  {
    LpSolverBase *tmp=0;
    return tmp;
  }

  LpSolverBase* LpSkeleton::_copyLp()
  {
    LpSolverBase *tmp=0;
    return tmp;
  }

  int LpSkeleton::_addCol()
  {
    return ++col_num;
  }

  int LpSkeleton::_addRow()
  {
    return ++row_num;
  }

  void LpSkeleton::_eraseCol(int ) {
  }

  void LpSkeleton::_eraseRow(int) {
  }

  void LpSkeleton::_getColName(int, std::string &) const {
  }


  void LpSkeleton::_setColName(int, const std::string &) {
  }

  int LpSkeleton::_colByName(const std::string&) const { return -1; }


  void LpSkeleton::_setRowCoeffs(int, ConstRowIterator, ConstRowIterator) {
  }

  void LpSkeleton::_getRowCoeffs(int, RowIterator) const {
  }

  void LpSkeleton::_setColCoeffs(int, ConstColIterator, ConstColIterator) {
  }

  void LpSkeleton::_getColCoeffs(int, ColIterator) const {
  }

  void LpSkeleton::_setCoeff(int, int, Value )
  {
  }

  LpSkeleton::Value LpSkeleton::_getCoeff(int, int) const
  {
    return 0;
  }


  void LpSkeleton::_setColLowerBound(int, Value)
  {
  }

  LpSkeleton::Value LpSkeleton::_getColLowerBound(int) const
  {
    return 0;
  }

  void LpSkeleton::_setColUpperBound(int, Value)
  {
  }

  LpSkeleton::Value LpSkeleton::_getColUpperBound(int) const
  {
    return 0;
  }

//   void LpSkeleton::_setRowLowerBound(int, Value)
//   {
//   }

//   void LpSkeleton::_setRowUpperBound(int, Value)
//   {
//   }

  void LpSkeleton::_setRowBounds(int, Value, Value)
  {
  }

  void LpSkeleton::_getRowBounds(int, Value&, Value&) const
  {
  }

  void LpSkeleton::_setObjCoeff(int, Value)
  {
  }

  LpSkeleton::Value LpSkeleton::_getObjCoeff(int) const
  {
    return 0;
  }

  void LpSkeleton::_setMax()
  {
  }

  void LpSkeleton::_setMin()
  {
  }

  bool LpSkeleton::_isMax() const
  {
    return true;
  }


  void LpSkeleton::_clearObj()
  {
  }

  LpSkeleton::SolveExitStatus LpSkeleton::_solve()
  {
    return SOLVED;
  }

  LpSkeleton::Value LpSkeleton::_getPrimal(int) const
  {
    return 0;
  }

  LpSkeleton::Value LpSkeleton::_getDual(int) const
  {
    return 0;
  }

  LpSkeleton::Value LpSkeleton::_getPrimalValue() const
  {
    return 0;
  }

  LpSkeleton::SolutionStatus LpSkeleton::_getPrimalStatus() const
  {
    return UNDEFINED;
  }

  LpSkeleton::SolutionStatus LpSkeleton::_getDualStatus() const
  {
    return UNDEFINED;
  }

  LpSkeleton::ProblemTypes LpSkeleton::_getProblemType() const
  {
    return UNKNOWN;
  }

  bool LpSkeleton::_isBasicCol(int) const
  {
    return true;
  }

} //namespace lemon

