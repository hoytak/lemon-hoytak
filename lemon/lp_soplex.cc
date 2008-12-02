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

#include<iostream>
#include<lemon/lp_soplex.h>

#include <soplex/soplex.h>


///\file
///\brief Implementation of the LEMON-SOPLEX lp solver interface.
namespace lemon {

  LpSoplex::LpSoplex() : LpSolverBase() {
    rows.setIdHandler(relocateIdHandler);
    cols.setIdHandler(relocateIdHandler);
    soplex = new soplex::SoPlex;
    solved = false;
  }

  LpSoplex::~LpSoplex() {
    delete soplex;
  }

  LpSoplex::LpSoplex(const LpSoplex& lp) : LpSolverBase() {
    rows = lp.rows;
    rows.setIdHandler(relocateIdHandler);

    cols = lp.cols;
    cols.setIdHandler(relocateIdHandler);

    soplex = new soplex::SoPlex;
    (*static_cast<soplex::SPxLP*>(soplex)) = *(lp.soplex);

    colNames = lp.colNames;
    invColNames = lp.invColNames;

    primal_value = lp.primal_value;
    dual_value = lp.dual_value;

  }

  LpSolverBase* LpSoplex::_newLp() {
    LpSoplex* newlp = new LpSoplex();
    return newlp;
  }

  LpSolverBase* LpSoplex::_copyLp() {
    LpSoplex* newlp = new LpSoplex(*this);
    return newlp;
  }

  int LpSoplex::_addCol() {
    soplex::LPCol c;
    c.setLower(-soplex::infinity);
    c.setUpper(soplex::infinity);
    soplex->addCol(c);

    colNames.push_back(std::string());
    primal_value.push_back(0.0);
    solved = false;

    return soplex->nCols() - 1;
  }

  int LpSoplex::_addRow() {
    soplex::LPRow r;
    r.setLhs(-soplex::infinity);
    r.setRhs(soplex::infinity);
    soplex->addRow(r);

    dual_value.push_back(0.0);
    solved = false;

    return soplex->nRows() - 1;
  }


  void LpSoplex::_eraseCol(int i) {
    soplex->removeCol(i);
    invColNames.erase(colNames[i]);
    colNames[i] = colNames.back();
    invColNames[colNames.back()] = i;
    colNames.pop_back();
    primal_value[i] = primal_value.back();
    primal_value.pop_back();
    solved = false;
  }

  void LpSoplex::_eraseRow(int i) {
    soplex->removeRow(i);
    dual_value[i] = dual_value.back();
    dual_value.pop_back();
    solved = false;
  }

  void LpSoplex::_getColName(int c, std::string &name) const {
    name = colNames[c];
  }

  void LpSoplex::_setColName(int c, const std::string &name) {
    invColNames.erase(colNames[c]);
    colNames[c] = name;
    if (!name.empty()) {
      invColNames.insert(std::make_pair(name, c));
    }
  }

  int LpSoplex::_colByName(const std::string& name) const {
    std::map<std::string, int>::const_iterator it =
      invColNames.find(name);
    if (it != invColNames.end()) {
      return it->second;
    } else {
      return -1;
    }
  }


  void LpSoplex::_setRowCoeffs(int i, ConstRowIterator b, ConstRowIterator e) {
    for (int j = 0; j < soplex->nCols(); ++j) {
      soplex->changeElement(i, j, 0.0);
    }
    for(ConstRowIterator it = b; it != e; ++it) {
      soplex->changeElement(i, it->first, it->second);
    }
    solved = false;
  }

  void LpSoplex::_getRowCoeffs(int i, RowIterator b) const {
    const soplex::SVector& vec = soplex->rowVector(i);
    for (int k = 0; k < vec.size(); ++k) {
      *b = std::make_pair(vec.index(k), vec.value(k));
      ++b;
    }
  }

  void LpSoplex::_setColCoeffs(int j, ConstColIterator b, ConstColIterator e) {
    for (int i = 0; i < soplex->nRows(); ++i) {
      soplex->changeElement(i, j, 0.0);
    }
    for(ConstColIterator it = b; it != e; ++it) {
      soplex->changeElement(it->first, j, it->second);
    }
    solved = false;
  }

  void LpSoplex::_getColCoeffs(int i, ColIterator b) const {
    const soplex::SVector& vec = soplex->colVector(i);
    for (int k = 0; k < vec.size(); ++k) {
      *b = std::make_pair(vec.index(k), vec.value(k));
      ++b;
    }
  }

  void LpSoplex::_setCoeff(int i, int j, Value value) {
    soplex->changeElement(i, j, value);
    solved = false;
  }

  LpSoplex::Value LpSoplex::_getCoeff(int i, int j) const {
    return soplex->rowVector(i)[j];
  }

  void LpSoplex::_setColLowerBound(int i, Value value) {
    soplex->changeLower(i, value != -INF ? value : -soplex::infinity);
    solved = false;
  }

  LpSoplex::Value LpSoplex::_getColLowerBound(int i) const {
    double value = soplex->lower(i);
    return value != -soplex::infinity ? value : -INF;
  }

  void LpSoplex::_setColUpperBound(int i, Value value) {
    soplex->changeUpper(i, value != INF ? value : soplex::infinity);
    solved = false;
  }

  LpSoplex::Value LpSoplex::_getColUpperBound(int i) const {
    double value = soplex->upper(i);
    return value != soplex::infinity ? value : INF;
  }

  void LpSoplex::_setRowBounds(int i, Value lb, Value ub) {
    soplex->changeRange(i, lb != -INF ? lb : -soplex::infinity,
                        ub != INF ? ub : soplex::infinity);
    solved = false;
  }
  void LpSoplex::_getRowBounds(int i, Value &lower, Value &upper) const {
    lower = soplex->lhs(i);
    if (lower == -soplex::infinity) lower = -INF;
    upper = soplex->rhs(i);
    if (upper == -soplex::infinity) upper = INF;
  }

  void LpSoplex::_setObjCoeff(int i, Value obj_coef) {
    soplex->changeObj(i, obj_coef);
    solved = false;
  }

  LpSoplex::Value LpSoplex::_getObjCoeff(int i) const {
    return soplex->obj(i);
  }

  void LpSoplex::_clearObj() {
    for (int i = 0; i < soplex->nCols(); ++i) {
      soplex->changeObj(i, 0.0);
    }
    solved = false;
  }

  LpSoplex::SolveExitStatus LpSoplex::_solve() {
    soplex::SPxSolver::Status status = soplex->solve();

    soplex::Vector pv(primal_value.size(), &primal_value[0]);
    soplex->getPrimal(pv);

    soplex::Vector dv(dual_value.size(), &dual_value[0]);
    soplex->getDual(dv);

    switch (status) {
    case soplex::SPxSolver::OPTIMAL:
    case soplex::SPxSolver::INFEASIBLE:
    case soplex::SPxSolver::UNBOUNDED:
      solved = true;
      return SOLVED;
    default:
      return UNSOLVED;
    }
  }

  LpSoplex::Value LpSoplex::_getPrimal(int i) const {
    return primal_value[i];
  }

  LpSoplex::Value LpSoplex::_getDual(int i) const {
    return dual_value[i];
  }

  LpSoplex::Value LpSoplex::_getPrimalValue() const {
    return soplex->objValue();
  }

  bool LpSoplex::_isBasicCol(int i) const {
    return soplex->getBasisColStatus(i) == soplex::SPxSolver::BASIC;
  }

  LpSoplex::SolutionStatus LpSoplex::_getPrimalStatus() const {
    if (!solved) return UNDEFINED;
    switch (soplex->status()) {
    case soplex::SPxSolver::OPTIMAL:
      return OPTIMAL;
    case soplex::SPxSolver::UNBOUNDED:
      return INFINITE;
    case soplex::SPxSolver::INFEASIBLE:
      return INFEASIBLE;
    default:
      return UNDEFINED;
    }
  }

  LpSoplex::SolutionStatus LpSoplex::_getDualStatus() const {
    if (!solved) return UNDEFINED;
    switch (soplex->status()) {
    case soplex::SPxSolver::OPTIMAL:
      return OPTIMAL;
    case soplex::SPxSolver::UNBOUNDED:
      return INFEASIBLE;
    default:
      return UNDEFINED;
    }
  }

  LpSoplex::ProblemTypes LpSoplex::_getProblemType() const {
    if (!solved) return UNKNOWN;
    switch (soplex->status()) {
    case soplex::SPxSolver::OPTIMAL:
      return PRIMAL_DUAL_FEASIBLE;
    case soplex::SPxSolver::UNBOUNDED:
      return PRIMAL_FEASIBLE_DUAL_INFEASIBLE;
    default:
      return UNKNOWN;
    }
  }

  void LpSoplex::_setMax() {
    soplex->changeSense(soplex::SPxSolver::MAXIMIZE);
    solved = false;
  }
  void LpSoplex::_setMin() {
    soplex->changeSense(soplex::SPxSolver::MINIMIZE);
    solved = false;
  }
  bool LpSoplex::_isMax() const {
    return soplex->spxSense() == soplex::SPxSolver::MAXIMIZE;
  }


} //namespace lemon

