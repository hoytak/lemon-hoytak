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

#ifndef LEMON_LP_SOPLEX_H
#define LEMON_LP_SOPLEX_H

///\file
///\brief Header of the LEMON-SOPLEX lp solver interface.

#include <vector>
#include <string>

#include <lemon/lp_base.h>

// Forward declaration
namespace soplex {
  class SoPlex;
}

namespace lemon {

  /// \ingroup lp_group
  ///
  /// \brief Interface for the SOPLEX solver
  ///
  /// This class implements an interface for the SoPlex LP solver.
  /// The SoPlex library is an object oriented lp solver library
  /// developed at the Konrad-Zuse-Zentrum für Informationstechnik
  /// Berlin (ZIB). You can find detailed information about it at the
  /// <tt>http://soplex.zib.de</tt> address.
  class LpSoplex :virtual public LpSolverBase {
  protected:

    _lp_bits::RelocateIdHandler relocateIdHandler;

    soplex::SoPlex* soplex;
    bool solved;

    std::vector<std::string> colNames;
    std::map<std::string, int> invColNames;

    std::vector<Value> primal_value;
    std::vector<Value> dual_value;


  public:

    typedef LpSolverBase Parent;


    /// \e
    LpSoplex();
    /// \e
    LpSoplex(const LpSoplex&);
    /// \e
    ~LpSoplex();

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
    virtual void _setRowBounds(int i, Value lower, Value upper);
    virtual void _getRowBounds(int i, Value &lower, Value &upper) const;
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

  };
} //END OF NAMESPACE LEMON

#endif //LEMON_LP_SOPLEX_H

