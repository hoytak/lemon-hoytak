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

#include <lemon/lp_clp.h>
#include <coin/ClpSimplex.hpp>

namespace lemon {

  LpClp::LpClp() {
    _prob = new ClpSimplex();
    _init_temporals();
    messageLevel(MESSAGE_NO_OUTPUT);
  }

  LpClp::LpClp(const LpClp& other) {
    _prob = new ClpSimplex(*other._prob);
    rows = other.rows;
    cols = other.cols;
    _init_temporals();
    messageLevel(MESSAGE_NO_OUTPUT);
  }

  LpClp::~LpClp() {
    delete _prob;
    _clear_temporals();
  }

  void LpClp::_init_temporals() {
    _primal_ray = 0;
    _dual_ray = 0;
  }

  void LpClp::_clear_temporals() {
    if (_primal_ray) {
      delete[] _primal_ray;
      _primal_ray = 0;
    }
    if (_dual_ray) {
      delete[] _dual_ray;
      _dual_ray = 0;
    }
  }

  LpClp* LpClp::_newSolver() const {
    LpClp* newlp = new LpClp;
    return newlp;
  }

  LpClp* LpClp::_cloneSolver() const {
    LpClp* copylp = new LpClp(*this);
    return copylp;
  }

  const char* LpClp::_solverName() const { return "LpClp"; }

  int LpClp::_addCol() {
    _prob->addColumn(0, 0, 0, -COIN_DBL_MAX, COIN_DBL_MAX, 0.0);
    return _prob->numberColumns() - 1;
  }

  int LpClp::_addRow() {
    _prob->addRow(0, 0, 0, -COIN_DBL_MAX, COIN_DBL_MAX);
    return _prob->numberRows() - 1;
  }


  void LpClp::_eraseCol(int c) {
    _col_names_ref.erase(_prob->getColumnName(c));
    _prob->deleteColumns(1, &c);
  }

  void LpClp::_eraseRow(int r) {
    _row_names_ref.erase(_prob->getRowName(r));
    _prob->deleteRows(1, &r);
  }

  void LpClp::_eraseColId(int i) {
    cols.eraseIndex(i);
    cols.shiftIndices(i);
  }

  void LpClp::_eraseRowId(int i) {
    rows.eraseIndex(i);
    rows.shiftIndices(i);
  }

  void LpClp::_getColName(int c, std::string& name) const {
    name = _prob->getColumnName(c);
  }

  void LpClp::_setColName(int c, const std::string& name) {
    _prob->setColumnName(c, const_cast<std::string&>(name));
    _col_names_ref[name] = c;
  }

  int LpClp::_colByName(const std::string& name) const {
    std::map<std::string, int>::const_iterator it = _col_names_ref.find(name);
    return it != _col_names_ref.end() ? it->second : -1;
  }

  void LpClp::_getRowName(int r, std::string& name) const {
    name = _prob->getRowName(r);
  }

  void LpClp::_setRowName(int r, const std::string& name) {
    _prob->setRowName(r, const_cast<std::string&>(name));
    _row_names_ref[name] = r;
  }

  int LpClp::_rowByName(const std::string& name) const {
    std::map<std::string, int>::const_iterator it = _row_names_ref.find(name);
    return it != _row_names_ref.end() ? it->second : -1;
  }


  void LpClp::_setRowCoeffs(int ix, ExprIterator b, ExprIterator e) {
    std::map<int, Value> coeffs;

    int n = _prob->clpMatrix()->getNumCols();

    const int* indices = _prob->clpMatrix()->getIndices();
    const double* elements = _prob->clpMatrix()->getElements();

    for (int i = 0; i < n; ++i) {
      CoinBigIndex begin = _prob->clpMatrix()->getVectorStarts()[i];
      CoinBigIndex end = begin + _prob->clpMatrix()->getVectorLengths()[i];

      const int* it = std::lower_bound(indices + begin, indices + end, ix);
      if (it != indices + end && *it == ix && elements[it - indices] != 0.0) {
        coeffs[i] = 0.0;
      }
    }

    for (ExprIterator it = b; it != e; ++it) {
      coeffs[it->first] = it->second;
    }

    for (std::map<int, Value>::iterator it = coeffs.begin();
         it != coeffs.end(); ++it) {
      _prob->modifyCoefficient(ix, it->first, it->second);
    }
  }

  void LpClp::_getRowCoeffs(int ix, InsertIterator b) const {
    int n = _prob->clpMatrix()->getNumCols();

    const int* indices = _prob->clpMatrix()->getIndices();
    const double* elements = _prob->clpMatrix()->getElements();

    for (int i = 0; i < n; ++i) {
      CoinBigIndex begin = _prob->clpMatrix()->getVectorStarts()[i];
      CoinBigIndex end = begin + _prob->clpMatrix()->getVectorLengths()[i];

      const int* it = std::lower_bound(indices + begin, indices + end, ix);
      if (it != indices + end && *it == ix) {
        *b = std::make_pair(i, elements[it - indices]);
      }
    }
  }

  void LpClp::_setColCoeffs(int ix, ExprIterator b, ExprIterator e) {
    std::map<int, Value> coeffs;

    CoinBigIndex begin = _prob->clpMatrix()->getVectorStarts()[ix];
    CoinBigIndex end = begin + _prob->clpMatrix()->getVectorLengths()[ix];

    const int* indices = _prob->clpMatrix()->getIndices();
    const double* elements = _prob->clpMatrix()->getElements();

    for (CoinBigIndex i = begin; i != end; ++i) {
      if (elements[i] != 0.0) {
        coeffs[indices[i]] = 0.0;
      }
    }
    for (ExprIterator it = b; it != e; ++it) {
      coeffs[it->first] = it->second;
    }
    for (std::map<int, Value>::iterator it = coeffs.begin();
         it != coeffs.end(); ++it) {
      _prob->modifyCoefficient(it->first, ix, it->second);
    }
  }

  void LpClp::_getColCoeffs(int ix, InsertIterator b) const {
    CoinBigIndex begin = _prob->clpMatrix()->getVectorStarts()[ix];
    CoinBigIndex end = begin + _prob->clpMatrix()->getVectorLengths()[ix];

    const int* indices = _prob->clpMatrix()->getIndices();
    const double* elements = _prob->clpMatrix()->getElements();

    for (CoinBigIndex i = begin; i != end; ++i) {
      *b = std::make_pair(indices[i], elements[i]);
      ++b;
    }
  }

  void LpClp::_setCoeff(int ix, int jx, Value value) {
    _prob->modifyCoefficient(ix, jx, value);
  }

  LpClp::Value LpClp::_getCoeff(int ix, int jx) const {
    CoinBigIndex begin = _prob->clpMatrix()->getVectorStarts()[ix];
    CoinBigIndex end = begin + _prob->clpMatrix()->getVectorLengths()[ix];

    const int* indices = _prob->clpMatrix()->getIndices();
    const double* elements = _prob->clpMatrix()->getElements();

    const int* it = std::lower_bound(indices + begin, indices + end, jx);
    if (it != indices + end && *it == jx) {
      return elements[it - indices];
    } else {
      return 0.0;
    }
  }

  void LpClp::_setColLowerBound(int i, Value lo) {
    _prob->setColumnLower(i, lo == - INF ? - COIN_DBL_MAX : lo);
  }

  LpClp::Value LpClp::_getColLowerBound(int i) const {
    double val = _prob->getColLower()[i];
    return val == - COIN_DBL_MAX ? - INF : val;
  }

  void LpClp::_setColUpperBound(int i, Value up) {
    _prob->setColumnUpper(i, up == INF ? COIN_DBL_MAX : up);
  }

  LpClp::Value LpClp::_getColUpperBound(int i) const {
    double val = _prob->getColUpper()[i];
    return val == COIN_DBL_MAX ? INF : val;
  }

  void LpClp::_setRowLowerBound(int i, Value lo) {
    _prob->setRowLower(i, lo == - INF ? - COIN_DBL_MAX : lo);
  }

  LpClp::Value LpClp::_getRowLowerBound(int i) const {
    double val = _prob->getRowLower()[i];
    return val == - COIN_DBL_MAX ? - INF : val;
  }

  void LpClp::_setRowUpperBound(int i, Value up) {
    _prob->setRowUpper(i, up == INF ? COIN_DBL_MAX : up);
  }

  LpClp::Value LpClp::_getRowUpperBound(int i) const {
    double val = _prob->getRowUpper()[i];
    return val == COIN_DBL_MAX ? INF : val;
  }

  void LpClp::_setObjCoeffs(ExprIterator b, ExprIterator e) {
    int num = _prob->clpMatrix()->getNumCols();
    for (int i = 0; i < num; ++i) {
      _prob->setObjectiveCoefficient(i, 0.0);
    }
    for (ExprIterator it = b; it != e; ++it) {
      _prob->setObjectiveCoefficient(it->first, it->second);
    }
  }

  void LpClp::_getObjCoeffs(InsertIterator b) const {
    int num = _prob->clpMatrix()->getNumCols();
    for (int i = 0; i < num; ++i) {
      Value coef = _prob->getObjCoefficients()[i];
      if (coef != 0.0) {
        *b = std::make_pair(i, coef);
        ++b;
      }
    }
  }

  void LpClp::_setObjCoeff(int i, Value obj_coef) {
    _prob->setObjectiveCoefficient(i, obj_coef);
  }

  LpClp::Value LpClp::_getObjCoeff(int i) const {
    return _prob->getObjCoefficients()[i];
  }

  LpClp::SolveExitStatus LpClp::_solve() {
    return _prob->primal() >= 0 ? SOLVED : UNSOLVED;
  }

  LpClp::SolveExitStatus LpClp::solvePrimal() {
    return _prob->primal() >= 0 ? SOLVED : UNSOLVED;
  }

  LpClp::SolveExitStatus LpClp::solveDual() {
    return _prob->dual() >= 0 ? SOLVED : UNSOLVED;
  }

  LpClp::SolveExitStatus LpClp::solveBarrier() {
    return _prob->barrier() >= 0 ? SOLVED : UNSOLVED;
  }

  LpClp::Value LpClp::_getPrimal(int i) const {
    return _prob->primalColumnSolution()[i];
  }
  LpClp::Value LpClp::_getPrimalValue() const {
    return _prob->objectiveValue();
  }

  LpClp::Value LpClp::_getDual(int i) const {
    return _prob->dualRowSolution()[i];
  }

  LpClp::Value LpClp::_getPrimalRay(int i) const {
    if (!_primal_ray) {
      _primal_ray = _prob->unboundedRay();
      LEMON_ASSERT(_primal_ray != 0, "Primal ray is not provided");
    }
    return _primal_ray[i];
  }

  LpClp::Value LpClp::_getDualRay(int i) const {
    if (!_dual_ray) {
      _dual_ray = _prob->infeasibilityRay();
      LEMON_ASSERT(_dual_ray != 0, "Dual ray is not provided");
    }
    return _dual_ray[i];
  }

  LpClp::VarStatus LpClp::_getColStatus(int i) const {
    switch (_prob->getColumnStatus(i)) {
    case ClpSimplex::basic:
      return BASIC;
    case ClpSimplex::isFree:
      return FREE;
    case ClpSimplex::atUpperBound:
      return UPPER;
    case ClpSimplex::atLowerBound:
      return LOWER;
    case ClpSimplex::isFixed:
      return FIXED;
    case ClpSimplex::superBasic:
      return FREE;
    default:
      LEMON_ASSERT(false, "Wrong column status");
      return VarStatus();
    }
  }

  LpClp::VarStatus LpClp::_getRowStatus(int i) const {
    switch (_prob->getColumnStatus(i)) {
    case ClpSimplex::basic:
      return BASIC;
    case ClpSimplex::isFree:
      return FREE;
    case ClpSimplex::atUpperBound:
      return UPPER;
    case ClpSimplex::atLowerBound:
      return LOWER;
    case ClpSimplex::isFixed:
      return FIXED;
    case ClpSimplex::superBasic:
      return FREE;
    default:
      LEMON_ASSERT(false, "Wrong row status");
      return VarStatus();
    }
  }


  LpClp::ProblemType LpClp::_getPrimalType() const {
    if (_prob->isProvenOptimal()) {
      return OPTIMAL;
    } else if (_prob->isProvenPrimalInfeasible()) {
      return INFEASIBLE;
    } else if (_prob->isProvenDualInfeasible()) {
      return UNBOUNDED;
    } else {
      return UNDEFINED;
    }
  }

  LpClp::ProblemType LpClp::_getDualType() const {
    if (_prob->isProvenOptimal()) {
      return OPTIMAL;
    } else if (_prob->isProvenDualInfeasible()) {
      return INFEASIBLE;
    } else if (_prob->isProvenPrimalInfeasible()) {
      return INFEASIBLE;
    } else {
      return UNDEFINED;
    }
  }

  void LpClp::_setSense(LpClp::Sense sense) {
    switch (sense) {
    case MIN:
      _prob->setOptimizationDirection(1);
      break;
    case MAX:
      _prob->setOptimizationDirection(-1);
      break;
    }
  }

  LpClp::Sense LpClp::_getSense() const {
    double dir = _prob->optimizationDirection();
    if (dir > 0.0) {
      return MIN;
    } else {
      return MAX;
    }
  }

  void LpClp::_clear() {
    delete _prob;
    _prob = new ClpSimplex();
    rows.clear();
    cols.clear();
    _col_names_ref.clear();
    _clear_temporals();
  }

  void LpClp::messageLevel(MessageLevel m) {
    _prob->setLogLevel(static_cast<int>(m));
  }

} //END OF NAMESPACE LEMON
