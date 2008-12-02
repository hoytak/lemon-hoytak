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

#ifndef LEMON_LP_GLPK_H
#define LEMON_LP_GLPK_H

///\file
///\brief Header of the LEMON-GLPK lp solver interface.
///\ingroup lp_group

#include <lemon/lp_base.h>

// forward declaration
#ifndef _GLP_PROB
#define _GLP_PROB
typedef struct { double _prob; } glp_prob;
/* LP/MIP problem object */
#endif

namespace lemon {


  /// \brief Interface for the GLPK LP solver
  ///
  /// This class implements an interface for the GLPK LP solver.
  ///\ingroup lp_group
  class LpGlpk : virtual public LpSolverBase {
  protected:

    typedef glp_prob LPX;
    glp_prob* lp;
    bool solved;

  public:

    typedef LpSolverBase Parent;

    LpGlpk();
    LpGlpk(const LpGlpk &);
    ~LpGlpk();

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
    virtual void _getRowBounds(int i, Value &lb, Value &ub) const;
    virtual void _setObjCoeff(int i, Value obj_coef);
    virtual Value _getObjCoeff(int i) const;
    virtual void _clearObj();

    ///\e

    ///\todo It should be clarified
    ///
    virtual SolveExitStatus _solve();
    virtual Value _getPrimal(int i) const;
    virtual Value _getDual(int i) const;
    virtual Value _getPrimalValue() const;
    virtual bool _isBasicCol(int i) const;
    ///\e

    ///\todo It should be clarified
    ///
    virtual SolutionStatus _getPrimalStatus() const;
    virtual SolutionStatus _getDualStatus() const;
    virtual ProblemTypes _getProblemType() const;

    virtual void _setMax();
    virtual void _setMin();

    virtual bool _isMax() const;

  public:
    ///Set the verbosity of the messages

    ///Set the verbosity of the messages
    ///
    ///\param m is the level of the messages output by the solver routines.
    ///The possible values are:
    ///- 0 --- no output (default value)
    ///- 1 --- error messages only
    ///- 2 --- normal output
    ///- 3 --- full output (includes informational messages)
    void messageLevel(int m);
    ///Turns on or off the presolver

    ///Turns on (\c b is \c true) or off (\c b is \c false) the presolver
    ///
    ///The presolver is off by default.
    void presolver(bool b);

    ///Pointer to the underlying GLPK data structure.
    LPX *lpx() {return lp;}

    ///Returns the constraint identifier understood by GLPK.
    int lpxRow(Row r) { return _lpId(r); }

    ///Returns the variable identifier understood by GLPK.
    int lpxCol(Col c) { return _lpId(c); }
  };
} //END OF NAMESPACE LEMON

#endif //LEMON_LP_GLPK_H

