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

#ifndef LEMON_LP_SKELETON
#define LEMON_LP_SKELETON

#include <lemon/lp_base.h>

///\file
///\brief A skeleton file to implement LP solver interfaces
namespace lemon {

  ///A skeleton class to implement LP solver interfaces
  class LpSkeleton :public LpSolverBase {
    int col_num,row_num;

  protected:

    ///\e
    virtual LpSolverBase* _newLp();
    ///\e
    virtual LpSolverBase* _copyLp();
    /// \e
    virtual int _addCol();
    /// \e
    virtual int _addRow();
    /// \e
    virtual void _eraseCol(int i);
    /// \e
    virtual void _eraseRow(int i);
    /// \e
    virtual void _getColName(int col, std::string & name) const;
    /// \e
    virtual void _setColName(int col, const std::string & name);
    /// \e
    virtual int _colByName(const std::string& name) const;

    /// \e
    virtual void _setRowCoeffs(int i, ConstRowIterator b, ConstRowIterator e);
    /// \e
    virtual void _getRowCoeffs(int i, RowIterator b) const;
    /// \e
    virtual void _setColCoeffs(int i, ConstColIterator b, ConstColIterator e);
    /// \e
    virtual void _getColCoeffs(int i, ColIterator b) const;

    /// Set one element of the coefficient matrix
    virtual void _setCoeff(int row, int col, Value value);

    /// Get one element of the coefficient matrix
    virtual Value _getCoeff(int row, int col) const;

    /// The lower bound of a variable (column) have to be given by an
    /// extended number of type Value, i.e. a finite number of type
    /// Value or -\ref INF.
    virtual void _setColLowerBound(int i, Value value);
    /// \e

    /// The lower bound of a variable (column) is an
    /// extended number of type Value, i.e. a finite number of type
    /// Value or -\ref INF.
    virtual Value _getColLowerBound(int i) const;

    /// The upper bound of a variable (column) have to be given by an
    /// extended number of type Value, i.e. a finite number of type
    /// Value or \ref INF.
    virtual void _setColUpperBound(int i, Value value);
    /// \e

    /// The upper bound of a variable (column) is an
    /// extended number of type Value, i.e. a finite number of type
    /// Value or \ref INF.
    virtual Value _getColUpperBound(int i) const;

//     /// The lower bound of a linear expression (row) have to be given by an
//     /// extended number of type Value, i.e. a finite number of type
//     /// Value or -\ref INF.
//     virtual void _setRowLowerBound(int i, Value value);
//     /// \e

//     /// The upper bound of a linear expression (row) have to be given by an
//     /// extended number of type Value, i.e. a finite number of type
//     /// Value or \ref INF.
//     virtual void _setRowUpperBound(int i, Value value);

    /// The lower and upper bound of a linear expression (row) have to be
    /// given by an
    /// extended number of type Value, i.e. a finite number of type
    /// Value or +/-\ref INF.
    virtual void _setRowBounds(int i, Value lb, Value ub);
    /// \e


    /// The lower and the upper bound of
    /// a constraint (row) are
    /// extended numbers of type Value, i.e.  finite numbers of type
    /// Value, -\ref INF or \ref INF.
    virtual void _getRowBounds(int i, Value &lb, Value &ub) const;
    /// \e


    /// \e
    virtual void _clearObj();
    /// \e
    virtual void _setObjCoeff(int i, Value obj_coef);

    /// \e
    virtual Value _getObjCoeff(int i) const;

    ///\e

    ///\bug Wrong interface
    ///
    virtual SolveExitStatus _solve();

    ///\e

    ///\bug Wrong interface
    ///
    virtual Value _getPrimal(int i) const;

    ///\e

    ///\bug Wrong interface
    ///
    virtual Value _getDual(int i) const;

    ///\e

    ///\bug Wrong interface
    ///
    virtual Value _getPrimalValue() const;

    ///\e

    ///\bug Wrong interface
    ///
    virtual SolutionStatus _getPrimalStatus() const;

    ////e
    virtual SolutionStatus _getDualStatus() const;


    ///\e
    virtual ProblemTypes _getProblemType() const;

    ///\e
    virtual void _setMax();
    ///\e
    virtual void _setMin();

    ///\e
    virtual bool _isMax() const;



    ///\e
    virtual bool _isBasicCol(int i) const;



  public:
    LpSkeleton() : LpSolverBase(), col_num(0), row_num(0) {}
  };

} //namespace lemon

#endif // LEMON_LP_SKELETON
