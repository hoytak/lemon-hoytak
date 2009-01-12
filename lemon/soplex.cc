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
#include <lemon/soplex.h>

#include <soplex/soplex.h>


///\file
///\brief Implementation of the LEMON-SOPLEX lp solver interface.
namespace lemon {

  LpSoplex::LpSoplex() {
    soplex = new soplex::SoPlex;
  }

  LpSoplex::~LpSoplex() {
    delete soplex;
  }

  LpSoplex::LpSoplex(const LpSoplex& lp) {
    rows = lp.rows;
    cols = lp.cols;

    soplex = new soplex::SoPlex;
    (*static_cast<soplex::SPxLP*>(soplex)) = *(lp.soplex);

    _col_names = lp._col_names;
    _col_names_ref = lp._col_names_ref;

    _row_names = lp._row_names;
    _row_names_ref = lp._row_names_ref;

  }

  void LpSoplex::_clear_temporals() {
    _primal_values.clear();
    _dual_values.clear();
  }

  LpSoplex* LpSoplex::_newSolver() const {
    LpSoplex* newlp = new LpSoplex();
    return newlp;
  }

  LpSoplex* LpSoplex::_cloneSolver() const {
    LpSoplex* newlp = new LpSoplex(*this);
    return newlp;
  }

  const char* LpSoplex::_solverName() const { return "LpSoplex"; }

  int LpSoplex::_addCol() {
    soplex::LPCol c;
    c.setLower(-soplex::infinity);
    c.setUpper(soplex::infinity);
    soplex->addCol(c);

    _col_names.push_back(std::string());

    return soplex->nCols() - 1;
  }

  int LpSoplex::_addRow() {
    soplex::LPRow r;
    r.setLhs(-soplex::infinity);
    r.setRhs(soplex::infinity);
    soplex->addRow(r);

    _row_names.push_back(std::string());

    return soplex->nRows() - 1;
  }


  void LpSoplex::_eraseCol(int i) {
    soplex->removeCol(i);
    _col_names_ref.erase(_col_names[i]);
    _col_names[i] = _col_names.back();
    _col_names_ref[_col_names.back()] = i;
    _col_names.pop_back();
  }

  void LpSoplex::_eraseRow(int i) {
    soplex->removeRow(i);
    _row_names_ref.erase(_row_names[i]);
    _row_names[i] = _row_names.back();
    _row_names_ref[_row_names.back()] = i;
    _row_names.pop_back();
  }

  void LpSoplex::_eraseColId(int i) {
    cols.eraseIndex(i);
    cols.relocateIndex(i, cols.maxIndex());
  }
  void LpSoplex::_eraseRowId(int i) {
    rows.eraseIndex(i);
    rows.relocateIndex(i, rows.maxIndex());
  }

  void LpSoplex::_getColName(int c, std::string &name) const {
    name = _col_names[c];
  }

  void LpSoplex::_setColName(int c, const std::string &name) {
    _col_names_ref.erase(_col_names[c]);
    _col_names[c] = name;
    if (!name.empty()) {
      _col_names_ref.insert(std::make_pair(name, c));
    }
  }

  int LpSoplex::_colByName(const std::string& name) const {
    std::map<std::string, int>::const_iterator it =
      _col_names_ref.find(name);
    if (it != _col_names_ref.end()) {
      return it->second;
    } else {
      return -1;
    }
  }

  void LpSoplex::_getRowName(int r, std::string &name) const {
    name = _row_names[r];
  }

  void LpSoplex::_setRowName(int r, const std::string &name) {
    _row_names_ref.erase(_row_names[r]);
    _row_names[r] = name;
    if (!name.empty()) {
      _row_names_ref.insert(std::make_pair(name, r));
    }
  }

  int LpSoplex::_rowByName(const std::string& name) const {
    std::map<std::string, int>::const_iterator it =
      _row_names_ref.find(name);
    if (it != _row_names_ref.end()) {
      return it->second;
    } else {
      return -1;
    }
  }


  void LpSoplex::_setRowCoeffs(int i, ExprIterator b, ExprIterator e) {
    for (int j = 0; j < soplex->nCols(); ++j) {
      soplex->changeElement(i, j, 0.0);
    }
    for(ExprIterator it = b; it != e; ++it) {
      soplex->changeElement(i, it->first, it->second);
    }
  }

  void LpSoplex::_getRowCoeffs(int i, InsertIterator b) const {
    const soplex::SVector& vec = soplex->rowVector(i);
    for (int k = 0; k < vec.size(); ++k) {
      *b = std::make_pair(vec.index(k), vec.value(k));
      ++b;
    }
  }

  void LpSoplex::_setColCoeffs(int j, ExprIterator b, ExprIterator e) {
    for (int i = 0; i < soplex->nRows(); ++i) {
      soplex->changeElement(i, j, 0.0);
    }
    for(ExprIterator it = b; it != e; ++it) {
      soplex->changeElement(it->first, j, it->second);
    }
  }

  void LpSoplex::_getColCoeffs(int i, InsertIterator b) const {
    const soplex::SVector& vec = soplex->colVector(i);
    for (int k = 0; k < vec.size(); ++k) {
      *b = std::make_pair(vec.index(k), vec.value(k));
      ++b;
    }
  }

  void LpSoplex::_setCoeff(int i, int j, Value value) {
    soplex->changeElement(i, j, value);
  }

  LpSoplex::Value LpSoplex::_getCoeff(int i, int j) const {
    return soplex->rowVector(i)[j];
  }

  void LpSoplex::_setColLowerBound(int i, Value value) {
    LEMON_ASSERT(value != INF, "Invalid bound");
    soplex->changeLower(i, value != -INF ? value : -soplex::infinity);
  }

  LpSoplex::Value LpSoplex::_getColLowerBound(int i) const {
    double value = soplex->lower(i);
    return value != -soplex::infinity ? value : -INF;
  }

  void LpSoplex::_setColUpperBound(int i, Value value) {
    LEMON_ASSERT(value != -INF, "Invalid bound");
    soplex->changeUpper(i, value != INF ? value : soplex::infinity);
  }

  LpSoplex::Value LpSoplex::_getColUpperBound(int i) const {
    double value = soplex->upper(i);
    return value != soplex::infinity ? value : INF;
  }

  void LpSoplex::_setRowLowerBound(int i, Value lb) {
    LEMON_ASSERT(lb != INF, "Invalid bound");
    soplex->changeRange(i, lb != -INF ? lb : -soplex::infinity, soplex->rhs(i));
  }

  LpSoplex::Value LpSoplex::_getRowLowerBound(int i) const {
    double res = soplex->lhs(i);
    return res == -soplex::infinity ? -INF : res;
  }

  void LpSoplex::_setRowUpperBound(int i, Value ub) {
    LEMON_ASSERT(ub != -INF, "Invalid bound");
    soplex->changeRange(i, soplex->lhs(i), ub != INF ? ub : soplex::infinity);
  }

  LpSoplex::Value LpSoplex::_getRowUpperBound(int i) const {
    double res = soplex->rhs(i);
    return res == soplex::infinity ? INF : res;
  }

  void LpSoplex::_setObjCoeffs(ExprIterator b, ExprIterator e) {
    for (int j = 0; j < soplex->nCols(); ++j) {
      soplex->changeObj(j, 0.0);
    }
    for (ExprIterator it = b; it != e; ++it) {
      soplex->changeObj(it->first, it->second);
    }
  }

  void LpSoplex::_getObjCoeffs(InsertIterator b) const {
    for (int j = 0; j < soplex->nCols(); ++j) {
      Value coef = soplex->obj(j);
      if (coef != 0.0) {
        *b = std::make_pair(j, coef);
        ++b;
      }
    }
  }

  void LpSoplex::_setObjCoeff(int i, Value obj_coef) {
    soplex->changeObj(i, obj_coef);
  }

  LpSoplex::Value LpSoplex::_getObjCoeff(int i) const {
    return soplex->obj(i);
  }

  LpSoplex::SolveExitStatus LpSoplex::_solve() {

    _clear_temporals();

    soplex::SPxSolver::Status status = soplex->solve();

    switch (status) {
    case soplex::SPxSolver::OPTIMAL:
    case soplex::SPxSolver::INFEASIBLE:
    case soplex::SPxSolver::UNBOUNDED:
      return SOLVED;
    default:
      return UNSOLVED;
    }
  }

  LpSoplex::Value LpSoplex::_getPrimal(int i) const {
    if (_primal_values.empty()) {
      _primal_values.resize(soplex->nCols());
      soplex::Vector pv(_primal_values.size(), &_primal_values.front());
      soplex->getPrimal(pv);
    }
    return _primal_values[i];
  }

  LpSoplex::Value LpSoplex::_getDual(int i) const {
    if (_dual_values.empty()) {
      _dual_values.resize(soplex->nRows());
      soplex::Vector dv(_dual_values.size(), &_dual_values.front());
      soplex->getDual(dv);
    }
    return _dual_values[i];
  }

  LpSoplex::Value LpSoplex::_getPrimalValue() const {
    return soplex->objValue();
  }

  LpSoplex::VarStatus LpSoplex::_getColStatus(int i) const {
    switch (soplex->getBasisColStatus(i)) {
    case soplex::SPxSolver::BASIC:
      return BASIC;
    case soplex::SPxSolver::ON_UPPER:
      return UPPER;
    case soplex::SPxSolver::ON_LOWER:
      return LOWER;
    case soplex::SPxSolver::FIXED:
      return FIXED;
    case soplex::SPxSolver::ZERO:
      return FREE;
    default:
      LEMON_ASSERT(false, "Wrong column status");
      return VarStatus();
    }
  }

  LpSoplex::VarStatus LpSoplex::_getRowStatus(int i) const {
    switch (soplex->getBasisRowStatus(i)) {
    case soplex::SPxSolver::BASIC:
      return BASIC;
    case soplex::SPxSolver::ON_UPPER:
      return UPPER;
    case soplex::SPxSolver::ON_LOWER:
      return LOWER;
    case soplex::SPxSolver::FIXED:
      return FIXED;
    case soplex::SPxSolver::ZERO:
      return FREE;
    default:
      LEMON_ASSERT(false, "Wrong row status");
      return VarStatus();
    }
  }

  LpSoplex::Value LpSoplex::_getPrimalRay(int i) const {
    if (_primal_ray.empty()) {
      _primal_ray.resize(soplex->nCols());
      soplex::Vector pv(_primal_ray.size(), &_primal_ray.front());
      soplex->getDualfarkas(pv);
    }
    return _primal_ray[i];
  }

  LpSoplex::Value LpSoplex::_getDualRay(int i) const {
    if (_dual_ray.empty()) {
      _dual_ray.resize(soplex->nRows());
      soplex::Vector dv(_dual_ray.size(), &_dual_ray.front());
      soplex->getDualfarkas(dv);
    }
    return _dual_ray[i];
  }

  LpSoplex::ProblemType LpSoplex::_getPrimalType() const {
    switch (soplex->status()) {
    case soplex::SPxSolver::OPTIMAL:
      return OPTIMAL;
    case soplex::SPxSolver::UNBOUNDED:
      return UNBOUNDED;
    case soplex::SPxSolver::INFEASIBLE:
      return INFEASIBLE;
    default:
      return UNDEFINED;
    }
  }

  LpSoplex::ProblemType LpSoplex::_getDualType() const {
    switch (soplex->status()) {
    case soplex::SPxSolver::OPTIMAL:
      return OPTIMAL;
    case soplex::SPxSolver::UNBOUNDED:
      return UNBOUNDED;
    case soplex::SPxSolver::INFEASIBLE:
      return INFEASIBLE;
    default:
      return UNDEFINED;
    }
  }

  void LpSoplex::_setSense(Sense sense) {
    switch (sense) {
    case MIN:
      soplex->changeSense(soplex::SPxSolver::MINIMIZE);
      break;
    case MAX:
      soplex->changeSense(soplex::SPxSolver::MAXIMIZE);
    }
  }

  LpSoplex::Sense LpSoplex::_getSense() const {
    switch (soplex->spxSense()) {
    case soplex::SPxSolver::MAXIMIZE:
      return MAX;
    case soplex::SPxSolver::MINIMIZE:
      return MIN;
    default:
      LEMON_ASSERT(false, "Wrong sense.");
      return LpSoplex::Sense();
    }
  }

  void LpSoplex::_clear() {
    soplex->clear();
    _col_names.clear();
    _col_names_ref.clear();
    _row_names.clear();
    _row_names_ref.clear();
    cols.clear();
    rows.clear();
    _clear_temporals();
  }

} //namespace lemon

