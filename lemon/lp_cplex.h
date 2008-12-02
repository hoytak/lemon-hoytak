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

#ifndef LEMON_LP_CPLEX_H
#define LEMON_LP_CPLEX_H

///\file
///\brief Header of the LEMON-CPLEX lp solver interface.

#include <lemon/lp_base.h>

struct cpxenv;
struct cpxlp;

namespace lemon {


  /// \brief Interface for the CPLEX solver
  ///
  /// This class implements an interface for the CPLEX LP solver.
  class LpCplex :virtual public LpSolverBase {

  public:

    typedef LpSolverBase Parent;

    /// \e
    int status;
    cpxenv* env;
    cpxlp* lp;


    /// \e
    LpCplex();
    /// \e
    LpCplex(const LpCplex&);
    /// \e
    ~LpCplex();

  protected:
    virtual LpSolverBase* _newLp();
    virtual LpSolverBase* _copyLp();


    virtual int _addCol();
    virtual int _addRow();
    virtual void _eraseCol(int i);
    virtual void _eraseRow(int i);
    virtual void _getColName(int col, std::string & name) const;
    virtual void _setColName(int col, const std::string & name);
    virtual int _colByName(const std::string& name) const;
    virtual void _setRowCoeffs(int i, ConstRowIterator b, ConstRowIterator e);
    virtual void _getRowCoeffs(int i, RowIterator b) const;
    virtual void _setColCoeffs(int i, ConstColIterator b, ConstColIterator e);
    virtual void _getColCoeffs(int i, ColIterator b) const;
    virtual void _setCoeff(int row, int col, Value value);
    virtual Value _getCoeff(int row, int col) const;

    virtual void _setColLowerBound(int i, Value value);
    virtual Value _getColLowerBound(int i) const;
    virtual void _setColUpperBound(int i, Value value);
    virtual Value _getColUpperBound(int i) const;

//     virtual void _setRowLowerBound(int i, Value value);
//     virtual void _setRowUpperBound(int i, Value value);
    virtual void _setRowBounds(int i, Value lower, Value upper);
    virtual void _getRowBounds(int i, Value &lb, Value &ub) const;
    virtual void _setObjCoeff(int i, Value obj_coef);
    virtual Value _getObjCoeff(int i) const;
    virtual void _clearObj();


    virtual SolveExitStatus _solve();
    virtual Value _getPrimal(int i) const;
    virtual Value _getDual(int i) const;
    virtual Value _getPrimalValue() const;
    virtual bool _isBasicCol(int i) const;

    virtual SolutionStatus _getPrimalStatus() const;
    virtual SolutionStatus _getDualStatus() const;
    virtual ProblemTypes _getProblemType() const;


    virtual void _setMax();
    virtual void _setMin();

    virtual bool _isMax() const;

  public:

    cpxenv* cplexEnv() { return env; }
    cpxlp* cplexLp() { return lp; }

  };
} //END OF NAMESPACE LEMON

#endif //LEMON_LP_CPLEX_H

