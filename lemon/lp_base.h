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

#ifndef LEMON_LP_BASE_H
#define LEMON_LP_BASE_H

#include<iostream>
#include<vector>
#include<map>
#include<limits>
#include<lemon/math.h>

#include<lemon/core.h>
#include<lemon/bits/lp_id.h>

///\file
///\brief The interface of the LP solver interface.
///\ingroup lp_group
namespace lemon {

  /// Function to decide whether a floating point value is finite or not.

  /// Retruns true if the argument is not infinity, minus infinity or NaN.
  /// It does the same as the isfinite() function defined by C99.
  template <typename T>
  bool isFinite(T value)
  {
    typedef std::numeric_limits<T> Lim;
    if ((Lim::has_infinity && (value == Lim::infinity() || value ==
                               -Lim::infinity())) ||
        ((Lim::has_quiet_NaN || Lim::has_signaling_NaN) && value != value))
    {
      return false;
    }
    return true;
  }

  ///Common base class for LP solvers

  ///\todo Much more docs
  ///\ingroup lp_group
  class LpSolverBase {

  protected:

    _lp_bits::LpId rows;
    _lp_bits::LpId cols;

  public:

    ///Possible outcomes of an LP solving procedure
    enum SolveExitStatus {
      ///This means that the problem has been successfully solved: either
      ///an optimal solution has been found or infeasibility/unboundedness
      ///has been proved.
      SOLVED = 0,
      ///Any other case (including the case when some user specified
      ///limit has been exceeded)
      UNSOLVED = 1
    };

      ///\e
    enum SolutionStatus {
      ///Feasible solution hasn't been found (but may exist).

      ///\todo NOTFOUND might be a better name.
      ///
      UNDEFINED = 0,
      ///The problem has no feasible solution
      INFEASIBLE = 1,
      ///Feasible solution found
      FEASIBLE = 2,
      ///Optimal solution exists and found
      OPTIMAL = 3,
      ///The cost function is unbounded

      ///\todo Give a feasible solution and an infinite ray (and the
      ///corresponding bases)
      INFINITE = 4
    };

    ///\e The type of the investigated LP problem
    enum ProblemTypes {
      ///Primal-dual feasible
      PRIMAL_DUAL_FEASIBLE = 0,
      ///Primal feasible dual infeasible
      PRIMAL_FEASIBLE_DUAL_INFEASIBLE = 1,
      ///Primal infeasible dual feasible
      PRIMAL_INFEASIBLE_DUAL_FEASIBLE = 2,
      ///Primal-dual infeasible
      PRIMAL_DUAL_INFEASIBLE = 3,
      ///Could not determine so far
      UNKNOWN = 4
    };

    ///The floating point type used by the solver
    typedef double Value;
    ///The infinity constant
    static const Value INF;
    ///The not a number constant
    static const Value NaN;

    static inline bool isNaN(const Value& v) { return v!=v; }

    friend class Col;
    friend class ColIt;
    friend class Row;

    ///Refer to a column of the LP.

    ///This type is used to refer to a column of the LP.
    ///
    ///Its value remains valid and correct even after the addition or erase of
    ///other columns.
    ///
    ///\todo Document what can one do with a Col (INVALID, comparing,
    ///it is similar to Node/Edge)
    class Col {
    protected:
      int id;
      friend class LpSolverBase;
      friend class MipSolverBase;
      explicit Col(int _id) : id(_id) {}
    public:
      typedef Value ExprValue;
      typedef True LpSolverCol;
      Col() {}
      Col(const Invalid&) : id(-1) {}
      bool operator< (Col c) const  {return id< c.id;}
      bool operator> (Col c) const  {return id> c.id;}
      bool operator==(Col c) const  {return id==c.id;}
      bool operator!=(Col c) const  {return id!=c.id;}
    };

    class ColIt : public Col {
      const LpSolverBase *_lp;
    public:
      ColIt() {}
      ColIt(const LpSolverBase &lp) : _lp(&lp)
      {
        _lp->cols.firstFix(id);
      }
      ColIt(const Invalid&) : Col(INVALID) {}
      ColIt &operator++()
      {
        _lp->cols.nextFix(id);
        return *this;
      }
    };

    static int id(const Col& col) { return col.id; }


    ///Refer to a row of the LP.

    ///This type is used to refer to a row of the LP.
    ///
    ///Its value remains valid and correct even after the addition or erase of
    ///other rows.
    ///
    ///\todo Document what can one do with a Row (INVALID, comparing,
    ///it is similar to Node/Edge)
    class Row {
    protected:
      int id;
      friend class LpSolverBase;
      explicit Row(int _id) : id(_id) {}
    public:
      typedef Value ExprValue;
      typedef True LpSolverRow;
      Row() {}
      Row(const Invalid&) : id(-1) {}

      bool operator< (Row c) const  {return id< c.id;}
      bool operator> (Row c) const  {return id> c.id;}
      bool operator==(Row c) const  {return id==c.id;}
      bool operator!=(Row c) const  {return id!=c.id;}
    };

    class RowIt : public Row {
      const LpSolverBase *_lp;
    public:
      RowIt() {}
      RowIt(const LpSolverBase &lp) : _lp(&lp)
      {
        _lp->rows.firstFix(id);
      }
      RowIt(const Invalid&) : Row(INVALID) {}
      RowIt &operator++()
      {
        _lp->rows.nextFix(id);
        return *this;
      }
    };

    static int id(const Row& row) { return row.id; }

  protected:

    int _lpId(const Col& c) const {
      return cols.floatingId(id(c));
    }

    int _lpId(const Row& r) const {
      return rows.floatingId(id(r));
    }

    Col _item(int i, Col) const {
      return Col(cols.fixId(i));
    }

    Row _item(int i, Row) const {
      return Row(rows.fixId(i));
    }


  public:

    ///Linear expression of variables and a constant component

    ///This data structure stores a linear expression of the variables
    ///(\ref Col "Col"s) and also has a constant component.
    ///
    ///There are several ways to access and modify the contents of this
    ///container.
    ///- Its it fully compatible with \c std::map<Col,double>, so for expamle
    ///if \c e is an Expr and \c v and \c w are of type \ref Col, then you can
    ///read and modify the coefficients like
    ///these.
    ///\code
    ///e[v]=5;
    ///e[v]+=12;
    ///e.erase(v);
    ///\endcode
    ///or you can also iterate through its elements.
    ///\code
    ///double s=0;
    ///for(LpSolverBase::Expr::iterator i=e.begin();i!=e.end();++i)
    ///  s+=i->second;
    ///\endcode
    ///(This code computes the sum of all coefficients).
    ///- Numbers (<tt>double</tt>'s)
    ///and variables (\ref Col "Col"s) directly convert to an
    ///\ref Expr and the usual linear operations are defined, so
    ///\code
    ///v+w
    ///2*v-3.12*(v-w/2)+2
    ///v*2.1+(3*v+(v*12+w+6)*3)/2
    ///\endcode
    ///are valid \ref Expr "Expr"essions.
    ///The usual assignment operations are also defined.
    ///\code
    ///e=v+w;
    ///e+=2*v-3.12*(v-w/2)+2;
    ///e*=3.4;
    ///e/=5;
    ///\endcode
    ///- The constant member can be set and read by \ref constComp()
    ///\code
    ///e.constComp()=12;
    ///double c=e.constComp();
    ///\endcode
    ///
    ///\note \ref clear() not only sets all coefficients to 0 but also
    ///clears the constant components.
    ///
    ///\sa Constr
    ///
    class Expr : public std::map<Col,Value>
    {
    public:
      typedef LpSolverBase::Col Key;
      typedef LpSolverBase::Value Value;

    protected:
      typedef std::map<Col,Value> Base;

      Value const_comp;
    public:
      typedef True IsLinExpression;
      ///\e
      Expr() : Base(), const_comp(0) { }
      ///\e
      Expr(const Key &v) : const_comp(0) {
        Base::insert(std::make_pair(v, 1));
      }
      ///\e
      Expr(const Value &v) : const_comp(v) {}
      ///\e
      void set(const Key &v,const Value &c) {
        Base::insert(std::make_pair(v, c));
      }
      ///\e
      Value &constComp() { return const_comp; }
      ///\e
      const Value &constComp() const { return const_comp; }

      ///Removes the components with zero coefficient.
      void simplify() {
        for (Base::iterator i=Base::begin(); i!=Base::end();) {
          Base::iterator j=i;
          ++j;
          if ((*i).second==0) Base::erase(i);
          i=j;
        }
      }

      void simplify() const {
        const_cast<Expr*>(this)->simplify();
      }

      ///Removes the coefficients closer to zero than \c tolerance.
      void simplify(double &tolerance) {
        for (Base::iterator i=Base::begin(); i!=Base::end();) {
          Base::iterator j=i;
          ++j;
          if (std::fabs((*i).second)<tolerance) Base::erase(i);
          i=j;
        }
      }

      ///Sets all coefficients and the constant component to 0.
      void clear() {
        Base::clear();
        const_comp=0;
      }

      ///\e
      Expr &operator+=(const Expr &e) {
        for (Base::const_iterator j=e.begin(); j!=e.end(); ++j)
          (*this)[j->first]+=j->second;
        const_comp+=e.const_comp;
        return *this;
      }
      ///\e
      Expr &operator-=(const Expr &e) {
        for (Base::const_iterator j=e.begin(); j!=e.end(); ++j)
          (*this)[j->first]-=j->second;
        const_comp-=e.const_comp;
        return *this;
      }
      ///\e
      Expr &operator*=(const Value &c) {
        for (Base::iterator j=Base::begin(); j!=Base::end(); ++j)
          j->second*=c;
        const_comp*=c;
        return *this;
      }
      ///\e
      Expr &operator/=(const Value &c) {
        for (Base::iterator j=Base::begin(); j!=Base::end(); ++j)
          j->second/=c;
        const_comp/=c;
        return *this;
      }

    };

    ///Linear constraint

    ///This data stucture represents a linear constraint in the LP.
    ///Basically it is a linear expression with a lower or an upper bound
    ///(or both). These parts of the constraint can be obtained by the member
    ///functions \ref expr(), \ref lowerBound() and \ref upperBound(),
    ///respectively.
    ///There are two ways to construct a constraint.
    ///- You can set the linear expression and the bounds directly
    ///  by the functions above.
    ///- The operators <tt>\<=</tt>, <tt>==</tt> and  <tt>\>=</tt>
    ///  are defined between expressions, or even between constraints whenever
    ///  it makes sense. Therefore if \c e and \c f are linear expressions and
    ///  \c s and \c t are numbers, then the followings are valid expressions
    ///  and thus they can be used directly e.g. in \ref addRow() whenever
    ///  it makes sense.
    ///\code
    ///  e<=s
    ///  e<=f
    ///  e==f
    ///  s<=e<=t
    ///  e>=t
    ///\endcode
    ///\warning The validity of a constraint is checked only at run time, so
    ///e.g. \ref addRow(<tt>x[1]\<=x[2]<=5</tt>) will compile, but will throw 
    ///an assertion.
    class Constr
    {
    public:
      typedef LpSolverBase::Expr Expr;
      typedef Expr::Key Key;
      typedef Expr::Value Value;

    protected:
      Expr _expr;
      Value _lb,_ub;
    public:
      ///\e
      Constr() : _expr(), _lb(NaN), _ub(NaN) {}
      ///\e
      Constr(Value lb,const Expr &e,Value ub) :
        _expr(e), _lb(lb), _ub(ub) {}
      ///\e
      Constr(const Expr &e,Value ub) :
        _expr(e), _lb(NaN), _ub(ub) {}
      ///\e
      Constr(Value lb,const Expr &e) :
        _expr(e), _lb(lb), _ub(NaN) {}
      ///\e
      Constr(const Expr &e) :
        _expr(e), _lb(NaN), _ub(NaN) {}
      ///\e
      void clear()
      {
        _expr.clear();
        _lb=_ub=NaN;
      }

      ///Reference to the linear expression
      Expr &expr() { return _expr; }
      ///Cont reference to the linear expression
      const Expr &expr() const { return _expr; }
      ///Reference to the lower bound.

      ///\return
      ///- \ref INF "INF": the constraint is lower unbounded.
      ///- \ref NaN "NaN": lower bound has not been set.
      ///- finite number: the lower bound
      Value &lowerBound() { return _lb; }
      ///The const version of \ref lowerBound()
      const Value &lowerBound() const { return _lb; }
      ///Reference to the upper bound.

      ///\return
      ///- \ref INF "INF": the constraint is upper unbounded.
      ///- \ref NaN "NaN": upper bound has not been set.
      ///- finite number: the upper bound
      Value &upperBound() { return _ub; }
      ///The const version of \ref upperBound()
      const Value &upperBound() const { return _ub; }
      ///Is the constraint lower bounded?
      bool lowerBounded() const {
        return isFinite(_lb);
      }
      ///Is the constraint upper bounded?
      bool upperBounded() const {
        return isFinite(_ub);
      }

    };

    ///Linear expression of rows

    ///This data structure represents a column of the matrix,
    ///thas is it strores a linear expression of the dual variables
    ///(\ref Row "Row"s).
    ///
    ///There are several ways to access and modify the contents of this
    ///container.
    ///- Its it fully compatible with \c std::map<Row,double>, so for expamle
    ///if \c e is an DualExpr and \c v
    ///and \c w are of type \ref Row, then you can
    ///read and modify the coefficients like
    ///these.
    ///\code
    ///e[v]=5;
    ///e[v]+=12;
    ///e.erase(v);
    ///\endcode
    ///or you can also iterate through its elements.
    ///\code
    ///double s=0;
    ///for(LpSolverBase::DualExpr::iterator i=e.begin();i!=e.end();++i)
    ///  s+=i->second;
    ///\endcode
    ///(This code computes the sum of all coefficients).
    ///- Numbers (<tt>double</tt>'s)
    ///and variables (\ref Row "Row"s) directly convert to an
    ///\ref DualExpr and the usual linear operations are defined, so
    ///\code
    ///v+w
    ///2*v-3.12*(v-w/2)
    ///v*2.1+(3*v+(v*12+w)*3)/2
    ///\endcode
    ///are valid \ref DualExpr "DualExpr"essions.
    ///The usual assignment operations are also defined.
    ///\code
    ///e=v+w;
    ///e+=2*v-3.12*(v-w/2);
    ///e*=3.4;
    ///e/=5;
    ///\endcode
    ///
    ///\sa Expr
    ///
    class DualExpr : public std::map<Row,Value>
    {
    public:
      typedef LpSolverBase::Row Key;
      typedef LpSolverBase::Value Value;

    protected:
      typedef std::map<Row,Value> Base;

    public:
      typedef True IsLinExpression;
      ///\e
      DualExpr() : Base() { }
      ///\e
      DualExpr(const Key &v) {
        Base::insert(std::make_pair(v, 1));
      }
      ///\e
      void set(const Key &v,const Value &c) {
        Base::insert(std::make_pair(v, c));
      }

      ///Removes the components with zero coefficient.
      void simplify() {
        for (Base::iterator i=Base::begin(); i!=Base::end();) {
          Base::iterator j=i;
          ++j;
          if ((*i).second==0) Base::erase(i);
          i=j;
        }
      }

      void simplify() const {
        const_cast<DualExpr*>(this)->simplify();
      }

      ///Removes the coefficients closer to zero than \c tolerance.
      void simplify(double &tolerance) {
        for (Base::iterator i=Base::begin(); i!=Base::end();) {
          Base::iterator j=i;
          ++j;
          if (std::fabs((*i).second)<tolerance) Base::erase(i);
          i=j;
        }
      }

      ///Sets all coefficients to 0.
      void clear() {
        Base::clear();
      }

      ///\e
      DualExpr &operator+=(const DualExpr &e) {
        for (Base::const_iterator j=e.begin(); j!=e.end(); ++j)
          (*this)[j->first]+=j->second;
        return *this;
      }
      ///\e
      DualExpr &operator-=(const DualExpr &e) {
        for (Base::const_iterator j=e.begin(); j!=e.end(); ++j)
          (*this)[j->first]-=j->second;
        return *this;
      }
      ///\e
      DualExpr &operator*=(const Value &c) {
        for (Base::iterator j=Base::begin(); j!=Base::end(); ++j)
          j->second*=c;
        return *this;
      }
      ///\e
      DualExpr &operator/=(const Value &c) {
        for (Base::iterator j=Base::begin(); j!=Base::end(); ++j)
          j->second/=c;
        return *this;
      }
    };


  private:

    template <typename _Expr>
    class MappedOutputIterator {
    public:

      typedef std::insert_iterator<_Expr> Base;

      typedef std::output_iterator_tag iterator_category;
      typedef void difference_type;
      typedef void value_type;
      typedef void reference;
      typedef void pointer;

      MappedOutputIterator(const Base& _base, const LpSolverBase& _lp)
        : base(_base), lp(_lp) {}

      MappedOutputIterator& operator*() {
        return *this;
      }

      MappedOutputIterator& operator=(const std::pair<int, Value>& value) {
        *base = std::make_pair(lp._item(value.first, typename _Expr::Key()),
                               value.second);
        return *this;
      }

      MappedOutputIterator& operator++() {
        ++base;
        return *this;
      }

      MappedOutputIterator operator++(int) {
        MappedOutputIterator tmp(*this);
        ++base;
        return tmp;
      }

      bool operator==(const MappedOutputIterator& it) const {
        return base == it.base;
      }

      bool operator!=(const MappedOutputIterator& it) const {
        return base != it.base;
      }

    private:
      Base base;
      const LpSolverBase& lp;
    };

    template <typename Expr>
    class MappedInputIterator {
    public:

      typedef typename Expr::const_iterator Base;

      typedef typename Base::iterator_category iterator_category;
      typedef typename Base::difference_type difference_type;
      typedef const std::pair<int, Value> value_type;
      typedef value_type reference;
      class pointer {
      public:
        pointer(value_type& _value) : value(_value) {}
        value_type* operator->() { return &value; }
      private:
        value_type value;
      };

      MappedInputIterator(const Base& _base, const LpSolverBase& _lp)
        : base(_base), lp(_lp) {}

      reference operator*() {
        return std::make_pair(lp._lpId(base->first), base->second);
      }

      pointer operator->() {
        return pointer(operator*());
      }

      MappedInputIterator& operator++() {
        ++base;
        return *this;
      }

      MappedInputIterator operator++(int) {
        MappedInputIterator tmp(*this);
        ++base;
        return tmp;
      }

      bool operator==(const MappedInputIterator& it) const {
        return base == it.base;
      }

      bool operator!=(const MappedInputIterator& it) const {
        return base != it.base;
      }

    private:
      Base base;
      const LpSolverBase& lp;
    };

  protected:

    /// STL compatible iterator for lp col
    typedef MappedInputIterator<Expr> ConstRowIterator;
    /// STL compatible iterator for lp row
    typedef MappedInputIterator<DualExpr> ConstColIterator;

    /// STL compatible iterator for lp col
    typedef MappedOutputIterator<Expr> RowIterator;
    /// STL compatible iterator for lp row
    typedef MappedOutputIterator<DualExpr> ColIterator;

    //Abstract virtual functions
    virtual LpSolverBase* _newLp() = 0;
    virtual LpSolverBase* _copyLp(){
      LpSolverBase* newlp = _newLp();

      std::map<Col, Col> ref;
      for (LpSolverBase::ColIt it(*this); it != INVALID; ++it) {
        Col ccol = newlp->addCol();
        ref[it] = ccol;
        newlp->colName(ccol, colName(it));
        newlp->colLowerBound(ccol, colLowerBound(it));
        newlp->colUpperBound(ccol, colUpperBound(it));
      }

      for (LpSolverBase::RowIt it(*this); it != INVALID; ++it) {
        Expr e = row(it), ce;
        for (Expr::iterator jt = e.begin(); jt != e.end(); ++jt) {
          ce[ref[jt->first]] = jt->second;
        }
        ce += e.constComp();
        Row r = newlp->addRow(ce);

        double lower, upper;
        getRowBounds(it, lower, upper);
        newlp->rowBounds(r, lower, upper);
      }

      return newlp;
    };

    virtual int _addCol() = 0;
    virtual int _addRow() = 0;

    virtual void _eraseCol(int col) = 0;
    virtual void _eraseRow(int row) = 0;

    virtual void _getColName(int col, std::string & name) const = 0;
    virtual void _setColName(int col, const std::string & name) = 0;
    virtual int _colByName(const std::string& name) const = 0;

    virtual void _setRowCoeffs(int i, ConstRowIterator b,
                               ConstRowIterator e) = 0;
    virtual void _getRowCoeffs(int i, RowIterator b) const = 0;
    virtual void _setColCoeffs(int i, ConstColIterator b,
                               ConstColIterator e) = 0;
    virtual void _getColCoeffs(int i, ColIterator b) const = 0;
    virtual void _setCoeff(int row, int col, Value value) = 0;
    virtual Value _getCoeff(int row, int col) const = 0;
    virtual void _setColLowerBound(int i, Value value) = 0;
    virtual Value _getColLowerBound(int i) const = 0;
    virtual void _setColUpperBound(int i, Value value) = 0;
    virtual Value _getColUpperBound(int i) const = 0;
    virtual void _setRowBounds(int i, Value lower, Value upper) = 0;
    virtual void _getRowBounds(int i, Value &lower, Value &upper) const = 0;

    virtual void _setObjCoeff(int i, Value obj_coef) = 0;
    virtual Value _getObjCoeff(int i) const = 0;
    virtual void _clearObj()=0;

    virtual SolveExitStatus _solve() = 0;
    virtual Value _getPrimal(int i) const = 0;
    virtual Value _getDual(int i) const = 0;
    virtual Value _getPrimalValue() const = 0;
    virtual bool _isBasicCol(int i) const = 0;
    virtual SolutionStatus _getPrimalStatus() const = 0;
    virtual SolutionStatus _getDualStatus() const = 0;
    virtual ProblemTypes _getProblemType() const = 0;

    virtual void _setMax() = 0;
    virtual void _setMin() = 0;


    virtual bool _isMax() const = 0;

    //Own protected stuff

    //Constant component of the objective function
    Value obj_const_comp;

  public:

    ///\e
    LpSolverBase() : obj_const_comp(0) {}

    ///\e
    virtual ~LpSolverBase() {}

    ///Creates a new LP problem
    LpSolverBase* newLp() {return _newLp();}
    ///Makes a copy of the LP problem
    LpSolverBase* copyLp() {return _copyLp();}

    ///\name Build up and modify the LP

    ///@{

    ///Add a new empty column (i.e a new variable) to the LP
    Col addCol() { Col c; _addCol(); c.id = cols.addId(); return c;}

    ///\brief Adds several new columns
    ///(i.e a variables) at once
    ///
    ///This magic function takes a container as its argument
    ///and fills its elements
    ///with new columns (i.e. variables)
    ///\param t can be
    ///- a standard STL compatible iterable container with
    ///\ref Col as its \c values_type
    ///like
    ///\code
    ///std::vector<LpSolverBase::Col>
    ///std::list<LpSolverBase::Col>
    ///\endcode
    ///- a standard STL compatible iterable container with
    ///\ref Col as its \c mapped_type
    ///like
    ///\code
    ///std::map<AnyType,LpSolverBase::Col>
    ///\endcode
    ///- an iterable lemon \ref concepts::WriteMap "write map" like
    ///\code
    ///ListGraph::NodeMap<LpSolverBase::Col>
    ///ListGraph::EdgeMap<LpSolverBase::Col>
    ///\endcode
    ///\return The number of the created column.
#ifdef DOXYGEN
    template<class T>
    int addColSet(T &t) { return 0;}
#else
    template<class T>
    typename enable_if<typename T::value_type::LpSolverCol,int>::type
    addColSet(T &t,dummy<0> = 0) {
      int s=0;
      for(typename T::iterator i=t.begin();i!=t.end();++i) {*i=addCol();s++;}
      return s;
    }
    template<class T>
    typename enable_if<typename T::value_type::second_type::LpSolverCol,
                       int>::type
    addColSet(T &t,dummy<1> = 1) {
      int s=0;
      for(typename T::iterator i=t.begin();i!=t.end();++i) {
        i->second=addCol();
        s++;
      }
      return s;
    }
    template<class T>
    typename enable_if<typename T::MapIt::Value::LpSolverCol,
                       int>::type
    addColSet(T &t,dummy<2> = 2) {
      int s=0;
      for(typename T::MapIt i(t); i!=INVALID; ++i)
        {
          i.set(addCol());
          s++;
        }
      return s;
    }
#endif

    ///Set a column (i.e a dual constraint) of the LP

    ///\param c is the column to be modified
    ///\param e is a dual linear expression (see \ref DualExpr)
    ///a better one.
    void col(Col c,const DualExpr &e) {
      e.simplify();
      _setColCoeffs(_lpId(c), ConstColIterator(e.begin(), *this),
                    ConstColIterator(e.end(), *this));
    }

    ///Get a column (i.e a dual constraint) of the LP

    ///\param r is the column to get
    ///\return the dual expression associated to the column
    DualExpr col(Col c) const {
      DualExpr e;
      _getColCoeffs(_lpId(c), ColIterator(std::inserter(e, e.end()), *this));
      return e;
    }

    ///Add a new column to the LP

    ///\param e is a dual linear expression (see \ref DualExpr)
    ///\param obj is the corresponding component of the objective
    ///function. It is 0 by default.
    ///\return The created column.
    Col addCol(const DualExpr &e, Value o = 0) {
      Col c=addCol();
      col(c,e);
      objCoeff(c,o);
      return c;
    }

    ///Add a new empty row (i.e a new constraint) to the LP

    ///This function adds a new empty row (i.e a new constraint) to the LP.
    ///\return The created row
    Row addRow() { Row r; _addRow(); r.id = rows.addId(); return r;}

    ///\brief Add several new rows
    ///(i.e a constraints) at once
    ///
    ///This magic function takes a container as its argument
    ///and fills its elements
    ///with new row (i.e. variables)
    ///\param t can be
    ///- a standard STL compatible iterable container with
    ///\ref Row as its \c values_type
    ///like
    ///\code
    ///std::vector<LpSolverBase::Row>
    ///std::list<LpSolverBase::Row>
    ///\endcode
    ///- a standard STL compatible iterable container with
    ///\ref Row as its \c mapped_type
    ///like
    ///\code
    ///std::map<AnyType,LpSolverBase::Row>
    ///\endcode
    ///- an iterable lemon \ref concepts::WriteMap "write map" like
    ///\code
    ///ListGraph::NodeMap<LpSolverBase::Row>
    ///ListGraph::EdgeMap<LpSolverBase::Row>
    ///\endcode
    ///\return The number of rows created.
#ifdef DOXYGEN
    template<class T>
    int addRowSet(T &t) { return 0;}
#else
    template<class T>
    typename enable_if<typename T::value_type::LpSolverRow,int>::type
    addRowSet(T &t,dummy<0> = 0) {
      int s=0;
      for(typename T::iterator i=t.begin();i!=t.end();++i) {*i=addRow();s++;}
      return s;
    }
    template<class T>
    typename enable_if<typename T::value_type::second_type::LpSolverRow,
                       int>::type
    addRowSet(T &t,dummy<1> = 1) {
      int s=0;
      for(typename T::iterator i=t.begin();i!=t.end();++i) {
        i->second=addRow();
        s++;
      }
      return s;
    }
    template<class T>
    typename enable_if<typename T::MapIt::Value::LpSolverRow,
                       int>::type
    addRowSet(T &t,dummy<2> = 2) {
      int s=0;
      for(typename T::MapIt i(t); i!=INVALID; ++i)
        {
          i.set(addRow());
          s++;
        }
      return s;
    }
#endif

    ///Set a row (i.e a constraint) of the LP

    ///\param r is the row to be modified
    ///\param l is lower bound (-\ref INF means no bound)
    ///\param e is a linear expression (see \ref Expr)
    ///\param u is the upper bound (\ref INF means no bound)
    ///\bug This is a temporary function. The interface will change to
    ///a better one.
    ///\todo Option to control whether a constraint with a single variable is
    ///added or not.
    void row(Row r, Value l, const Expr &e, Value u) {
      e.simplify();
      _setRowCoeffs(_lpId(r), ConstRowIterator(e.begin(), *this),
                    ConstRowIterator(e.end(), *this));
      _setRowBounds(_lpId(r),l-e.constComp(),u-e.constComp());
    }

    ///Set a row (i.e a constraint) of the LP

    ///\param r is the row to be modified
    ///\param c is a linear expression (see \ref Constr)
    void row(Row r, const Constr &c) {
      row(r, c.lowerBounded()?c.lowerBound():-INF,
          c.expr(), c.upperBounded()?c.upperBound():INF);
    }


    ///Get a row (i.e a constraint) of the LP

    ///\param r is the row to get
    ///\return the expression associated to the row
    Expr row(Row r) const {
      Expr e;
      _getRowCoeffs(_lpId(r), RowIterator(std::inserter(e, e.end()), *this));
      return e;
    }

    ///Add a new row (i.e a new constraint) to the LP

    ///\param l is the lower bound (-\ref INF means no bound)
    ///\param e is a linear expression (see \ref Expr)
    ///\param u is the upper bound (\ref INF means no bound)
    ///\return The created row.
    ///\bug This is a temporary function. The interface will change to
    ///a better one.
    Row addRow(Value l,const Expr &e, Value u) {
      Row r=addRow();
      row(r,l,e,u);
      return r;
    }

    ///Add a new row (i.e a new constraint) to the LP

    ///\param c is a linear expression (see \ref Constr)
    ///\return The created row.
    Row addRow(const Constr &c) {
      Row r=addRow();
      row(r,c);
      return r;
    }
    ///Erase a coloumn (i.e a variable) from the LP

    ///\param c is the coloumn to be deleted
    ///\todo Please check this
    void eraseCol(Col c) {
      _eraseCol(_lpId(c));
      cols.eraseId(c.id);
    }
    ///Erase a  row (i.e a constraint) from the LP

    ///\param r is the row to be deleted
    ///\todo Please check this
    void eraseRow(Row r) {
      _eraseRow(_lpId(r));
      rows.eraseId(r.id);
    }

    /// Get the name of a column

    ///\param c is the coresponding coloumn
    ///\return The name of the colunm
    std::string colName(Col c) const {
      std::string name;
      _getColName(_lpId(c), name);
      return name;
    }

    /// Set the name of a column

    ///\param c is the coresponding coloumn
    ///\param name The name to be given
    void colName(Col c, const std::string& name) {
      _setColName(_lpId(c), name);
    }

    /// Get the column by its name

    ///\param name The name of the column
    ///\return the proper column or \c INVALID
    Col colByName(const std::string& name) const {
      int k = _colByName(name);
      return k != -1 ? Col(cols.fixId(k)) : Col(INVALID);
    }

    /// Set an element of the coefficient matrix of the LP

    ///\param r is the row of the element to be modified
    ///\param c is the coloumn of the element to be modified
    ///\param val is the new value of the coefficient

    void coeff(Row r, Col c, Value val) {
      _setCoeff(_lpId(r),_lpId(c), val);
    }

    /// Get an element of the coefficient matrix of the LP

    ///\param r is the row of the element in question
    ///\param c is the coloumn of the element in question
    ///\return the corresponding coefficient

    Value coeff(Row r, Col c) const {
      return _getCoeff(_lpId(r),_lpId(c));
    }

    /// Set the lower bound of a column (i.e a variable)

    /// The lower bound of a variable (column) has to be given by an
    /// extended number of type Value, i.e. a finite number of type
    /// Value or -\ref INF.
    void colLowerBound(Col c, Value value) {
      _setColLowerBound(_lpId(c),value);
    }

    /// Get the lower bound of a column (i.e a variable)

    /// This function returns the lower bound for column (variable) \t c
    /// (this might be -\ref INF as well).
    ///\return The lower bound for coloumn \t c
    Value colLowerBound(Col c) const {
      return _getColLowerBound(_lpId(c));
    }

    ///\brief Set the lower bound of  several columns
    ///(i.e a variables) at once
    ///
    ///This magic function takes a container as its argument
    ///and applies the function on all of its elements.
    /// The lower bound of a variable (column) has to be given by an
    /// extended number of type Value, i.e. a finite number of type
    /// Value or -\ref INF.
#ifdef DOXYGEN
    template<class T>
    void colLowerBound(T &t, Value value) { return 0;}
#else
    template<class T>
    typename enable_if<typename T::value_type::LpSolverCol,void>::type
    colLowerBound(T &t, Value value,dummy<0> = 0) {
      for(typename T::iterator i=t.begin();i!=t.end();++i) {
        colLowerBound(*i, value);
      }
    }
    template<class T>
    typename enable_if<typename T::value_type::second_type::LpSolverCol,
                       void>::type
    colLowerBound(T &t, Value value,dummy<1> = 1) {
      for(typename T::iterator i=t.begin();i!=t.end();++i) {
        colLowerBound(i->second, value);
      }
    }
    template<class T>
    typename enable_if<typename T::MapIt::Value::LpSolverCol,
                       void>::type
    colLowerBound(T &t, Value value,dummy<2> = 2) {
      for(typename T::MapIt i(t); i!=INVALID; ++i){
        colLowerBound(*i, value);
      }
    }
#endif

    /// Set the upper bound of a column (i.e a variable)

    /// The upper bound of a variable (column) has to be given by an
    /// extended number of type Value, i.e. a finite number of type
    /// Value or \ref INF.
    void colUpperBound(Col c, Value value) {
      _setColUpperBound(_lpId(c),value);
    };

    /// Get the upper bound of a column (i.e a variable)

    /// This function returns the upper bound for column (variable) \t c
    /// (this might be \ref INF as well).
    ///\return The upper bound for coloumn \t c
    Value colUpperBound(Col c) const {
      return _getColUpperBound(_lpId(c));
    }

    ///\brief Set the upper bound of  several columns
    ///(i.e a variables) at once
    ///
    ///This magic function takes a container as its argument
    ///and applies the function on all of its elements.
    /// The upper bound of a variable (column) has to be given by an
    /// extended number of type Value, i.e. a finite number of type
    /// Value or \ref INF.
#ifdef DOXYGEN
    template<class T>
    void colUpperBound(T &t, Value value) { return 0;}
#else
    template<class T>
    typename enable_if<typename T::value_type::LpSolverCol,void>::type
    colUpperBound(T &t, Value value,dummy<0> = 0) {
      for(typename T::iterator i=t.begin();i!=t.end();++i) {
        colUpperBound(*i, value);
      }
    }
    template<class T>
    typename enable_if<typename T::value_type::second_type::LpSolverCol,
                       void>::type
    colUpperBound(T &t, Value value,dummy<1> = 1) {
      for(typename T::iterator i=t.begin();i!=t.end();++i) {
        colUpperBound(i->second, value);
      }
    }
    template<class T>
    typename enable_if<typename T::MapIt::Value::LpSolverCol,
                       void>::type
    colUpperBound(T &t, Value value,dummy<2> = 2) {
      for(typename T::MapIt i(t); i!=INVALID; ++i){
        colUpperBound(*i, value);
      }
    }
#endif

    /// Set the lower and the upper bounds of a column (i.e a variable)

    /// The lower and the upper bounds of
    /// a variable (column) have to be given by an
    /// extended number of type Value, i.e. a finite number of type
    /// Value, -\ref INF or \ref INF.
    void colBounds(Col c, Value lower, Value upper) {
      _setColLowerBound(_lpId(c),lower);
      _setColUpperBound(_lpId(c),upper);
    }

    ///\brief Set the lower and the upper bound of several columns
    ///(i.e a variables) at once
    ///
    ///This magic function takes a container as its argument
    ///and applies the function on all of its elements.
    /// The lower and the upper bounds of
    /// a variable (column) have to be given by an
    /// extended number of type Value, i.e. a finite number of type
    /// Value, -\ref INF or \ref INF.
#ifdef DOXYGEN
    template<class T>
    void colBounds(T &t, Value lower, Value upper) { return 0;}
#else
    template<class T>
    typename enable_if<typename T::value_type::LpSolverCol,void>::type
    colBounds(T &t, Value lower, Value upper,dummy<0> = 0) {
      for(typename T::iterator i=t.begin();i!=t.end();++i) {
        colBounds(*i, lower, upper);
      }
    }
    template<class T>
    typename enable_if<typename T::value_type::second_type::LpSolverCol,
                       void>::type
    colBounds(T &t, Value lower, Value upper,dummy<1> = 1) {
      for(typename T::iterator i=t.begin();i!=t.end();++i) {
        colBounds(i->second, lower, upper);
      }
    }
    template<class T>
    typename enable_if<typename T::MapIt::Value::LpSolverCol,
                       void>::type
    colBounds(T &t, Value lower, Value upper,dummy<2> = 2) {
      for(typename T::MapIt i(t); i!=INVALID; ++i){
        colBounds(*i, lower, upper);
      }
    }
#endif


    /// Set the lower and the upper bounds of a row (i.e a constraint)

    /// The lower and the upper bound of a constraint (row) have to be
    /// given by an extended number of type Value, i.e. a finite
    /// number of type Value, -\ref INF or \ref INF. There is no
    /// separate function for the lower and the upper bound because
    /// that would have been hard to implement for CPLEX.
    void rowBounds(Row c, Value lower, Value upper) {
      _setRowBounds(_lpId(c),lower, upper);
    }

    /// Get the lower and the upper bounds of a row (i.e a constraint)

    /// The lower and the upper bound of
    /// a constraint (row) are
    /// extended numbers of type Value, i.e.  finite numbers of type
    /// Value, -\ref INF or \ref INF.
    /// \todo There is no separate function for the
    /// lower and the upper bound because we had problems with the
    /// implementation of the setting functions for CPLEX:
    /// check out whether this can be done for these functions.
    void getRowBounds(Row c, Value &lower, Value &upper) const {
      _getRowBounds(_lpId(c),lower, upper);
    }

    ///Set an element of the objective function
    void objCoeff(Col c, Value v) {_setObjCoeff(_lpId(c),v); };

    ///Get an element of the objective function
    Value objCoeff(Col c) const { return _getObjCoeff(_lpId(c)); };

    ///Set the objective function

    ///\param e is a linear expression of type \ref Expr.
    void obj(Expr e) {
      _clearObj();
      for (Expr::iterator i=e.begin(); i!=e.end(); ++i)
        objCoeff((*i).first,(*i).second);
      obj_const_comp=e.constComp();
    }

    ///Get the objective function

    ///\return the objective function as a linear expression of type \ref Expr.
    Expr obj() const {
      Expr e;
      for (ColIt it(*this); it != INVALID; ++it) {
        double c = objCoeff(it);
        if (c != 0.0) {
          e.insert(std::make_pair(it, c));
        }
      }
      return e;
    }


    ///Maximize
    void max() { _setMax(); }
    ///Minimize
    void min() { _setMin(); }

    ///Query function: is this a maximization problem?
    bool isMax() const {return _isMax(); }

    ///Query function: is this a minimization problem?
    bool isMin() const {return !isMax(); }

    ///@}


    ///\name Solve the LP

    ///@{

    ///\e Solve the LP problem at hand
    ///
    ///\return The result of the optimization procedure. Possible
    ///values and their meanings can be found in the documentation of
    ///\ref SolveExitStatus.
    ///
    ///\todo Which method is used to solve the problem
    SolveExitStatus solve() { return _solve(); }

    ///@}

    ///\name Obtain the solution

    ///@{

    /// The status of the primal problem (the original LP problem)
    SolutionStatus primalStatus() const {
      return _getPrimalStatus();
    }

    /// The status of the dual (of the original LP) problem
    SolutionStatus dualStatus() const {
      return _getDualStatus();
    }

    ///The type of the original LP problem
    ProblemTypes problemType() const {
      return _getProblemType();
    }

    ///\e
    Value primal(Col c) const { return _getPrimal(_lpId(c)); }
    ///\e
    Value primal(const Expr& e) const {
      double res = e.constComp();
      for (std::map<Col, double>::const_iterator it = e.begin();
           it != e.end(); ++it) {
        res += _getPrimal(_lpId(it->first)) * it->second;
      }
      return res;
    }

    ///\e
    Value dual(Row r) const { return _getDual(_lpId(r)); }
    ///\e
    Value dual(const DualExpr& e) const {
      double res = 0.0;
      for (std::map<Row, double>::const_iterator it = e.begin();
           it != e.end(); ++it) {
        res += _getPrimal(_lpId(it->first)) * it->second;
      }
      return res;
    }

    ///\e
    bool isBasicCol(Col c) const { return _isBasicCol(_lpId(c)); }

    ///\e

    ///\return
    ///- \ref INF or -\ref INF means either infeasibility or unboundedness
    /// of the primal problem, depending on whether we minimize or maximize.
    ///- \ref NaN if no primal solution is found.
    ///- The (finite) objective value if an optimal solution is found.
    Value primalValue() const { return _getPrimalValue()+obj_const_comp;}
    ///@}

  };


  /// \ingroup lp_group
  ///
  /// \brief Common base class for MIP solvers
  /// \todo Much more docs
  class MipSolverBase : virtual public LpSolverBase{
  public:

    ///Possible variable (coloumn) types (e.g. real, integer, binary etc.)
    enum ColTypes {
      ///Continuous variable
      REAL = 0,
      ///Integer variable

      ///Unfortunately, cplex 7.5 somewhere writes something like
      ///#define INTEGER 'I'
      INT = 1
      ///\todo No support for other types yet.
    };

    ///Sets the type of the given coloumn to the given type
    ///
    ///Sets the type of the given coloumn to the given type.
    void colType(Col c, ColTypes col_type) {
      _colType(_lpId(c),col_type);
    }

    ///Gives back the type of the column.
    ///
    ///Gives back the type of the column.
    ColTypes colType(Col c) const {
      return _colType(_lpId(c));
    }

    ///Sets the type of the given Col to integer or remove that property.
    ///
    ///Sets the type of the given Col to integer or remove that property.
    void integer(Col c, bool enable) {
      if (enable)
        colType(c,INT);
      else
        colType(c,REAL);
    }

    ///Gives back whether the type of the column is integer or not.
    ///
    ///Gives back the type of the column.
    ///\return true if the column has integer type and false if not.
    bool integer(Col c) const {
      return (colType(c)==INT);
    }

    /// The status of the MIP problem
    SolutionStatus mipStatus() const {
      return _getMipStatus();
    }

  protected:

    virtual ColTypes _colType(int col) const = 0;
    virtual void _colType(int col, ColTypes col_type) = 0;
    virtual SolutionStatus _getMipStatus() const = 0;

  };

  ///\relates LpSolverBase::Expr
  ///
  inline LpSolverBase::Expr operator+(const LpSolverBase::Expr &a,
                                      const LpSolverBase::Expr &b)
  {
    LpSolverBase::Expr tmp(a);
    tmp+=b;
    return tmp;
  }
  ///\e

  ///\relates LpSolverBase::Expr
  ///
  inline LpSolverBase::Expr operator-(const LpSolverBase::Expr &a,
                                      const LpSolverBase::Expr &b)
  {
    LpSolverBase::Expr tmp(a);
    tmp-=b;
    return tmp;
  }
  ///\e

  ///\relates LpSolverBase::Expr
  ///
  inline LpSolverBase::Expr operator*(const LpSolverBase::Expr &a,
                                      const LpSolverBase::Value &b)
  {
    LpSolverBase::Expr tmp(a);
    tmp*=b;
    return tmp;
  }

  ///\e

  ///\relates LpSolverBase::Expr
  ///
  inline LpSolverBase::Expr operator*(const LpSolverBase::Value &a,
                                      const LpSolverBase::Expr &b)
  {
    LpSolverBase::Expr tmp(b);
    tmp*=a;
    return tmp;
  }
  ///\e

  ///\relates LpSolverBase::Expr
  ///
  inline LpSolverBase::Expr operator/(const LpSolverBase::Expr &a,
                                      const LpSolverBase::Value &b)
  {
    LpSolverBase::Expr tmp(a);
    tmp/=b;
    return tmp;
  }

  ///\e

  ///\relates LpSolverBase::Constr
  ///
  inline LpSolverBase::Constr operator<=(const LpSolverBase::Expr &e,
                                         const LpSolverBase::Expr &f)
  {
    return LpSolverBase::Constr(-LpSolverBase::INF,e-f,0);
  }

  ///\e

  ///\relates LpSolverBase::Constr
  ///
  inline LpSolverBase::Constr operator<=(const LpSolverBase::Value &e,
                                         const LpSolverBase::Expr &f)
  {
    return LpSolverBase::Constr(e,f);
  }

  ///\e

  ///\relates LpSolverBase::Constr
  ///
  inline LpSolverBase::Constr operator<=(const LpSolverBase::Expr &e,
                                         const LpSolverBase::Value &f)
  {
    return LpSolverBase::Constr(-LpSolverBase::INF,e,f);
  }

  ///\e

  ///\relates LpSolverBase::Constr
  ///
  inline LpSolverBase::Constr operator>=(const LpSolverBase::Expr &e,
                                         const LpSolverBase::Expr &f)
  {
    return LpSolverBase::Constr(-LpSolverBase::INF,f-e,0);
  }


  ///\e

  ///\relates LpSolverBase::Constr
  ///
  inline LpSolverBase::Constr operator>=(const LpSolverBase::Value &e,
                                         const LpSolverBase::Expr &f)
  {
    return LpSolverBase::Constr(f,e);
  }


  ///\e

  ///\relates LpSolverBase::Constr
  ///
  inline LpSolverBase::Constr operator>=(const LpSolverBase::Expr &e,
                                         const LpSolverBase::Value &f)
  {
    return LpSolverBase::Constr(f,e,LpSolverBase::INF);
  }

  ///\e

  ///\relates LpSolverBase::Constr
  ///
  inline LpSolverBase::Constr operator==(const LpSolverBase::Expr &e,
                                         const LpSolverBase::Value &f)
  {
    return LpSolverBase::Constr(f,e,f);
  }

  ///\e

  ///\relates LpSolverBase::Constr
  ///
  inline LpSolverBase::Constr operator==(const LpSolverBase::Expr &e,
                                         const LpSolverBase::Expr &f)
  {
    return LpSolverBase::Constr(0,e-f,0);
  }

  ///\e

  ///\relates LpSolverBase::Constr
  ///
  inline LpSolverBase::Constr operator<=(const LpSolverBase::Value &n,
                                         const LpSolverBase::Constr&c)
  {
    LpSolverBase::Constr tmp(c);
    LEMON_ASSERT(LpSolverBase::isNaN(tmp.lowerBound()), "Wrong LP constraint");
    tmp.lowerBound()=n;
    return tmp;
  }
  ///\e

  ///\relates LpSolverBase::Constr
  ///
  inline LpSolverBase::Constr operator<=(const LpSolverBase::Constr& c,
                                         const LpSolverBase::Value &n)
  {
    LpSolverBase::Constr tmp(c);
    LEMON_ASSERT(LpSolverBase::isNaN(tmp.upperBound()), "Wrong LP constraint");
    tmp.upperBound()=n;
    return tmp;
  }

  ///\e

  ///\relates LpSolverBase::Constr
  ///
  inline LpSolverBase::Constr operator>=(const LpSolverBase::Value &n,
                                         const LpSolverBase::Constr&c)
  {
    LpSolverBase::Constr tmp(c);
    LEMON_ASSERT(LpSolverBase::isNaN(tmp.upperBound()), "Wrong LP constraint");
    tmp.upperBound()=n;
    return tmp;
  }
  ///\e

  ///\relates LpSolverBase::Constr
  ///
  inline LpSolverBase::Constr operator>=(const LpSolverBase::Constr& c,
                                         const LpSolverBase::Value &n)
  {
    LpSolverBase::Constr tmp(c);
    LEMON_ASSERT(LpSolverBase::isNaN(tmp.lowerBound()), "Wrong LP constraint");
    tmp.lowerBound()=n;
    return tmp;
  }

  ///\e

  ///\relates LpSolverBase::DualExpr
  ///
  inline LpSolverBase::DualExpr operator+(const LpSolverBase::DualExpr &a,
                                          const LpSolverBase::DualExpr &b)
  {
    LpSolverBase::DualExpr tmp(a);
    tmp+=b;
    return tmp;
  }
  ///\e

  ///\relates LpSolverBase::DualExpr
  ///
  inline LpSolverBase::DualExpr operator-(const LpSolverBase::DualExpr &a,
                                          const LpSolverBase::DualExpr &b)
  {
    LpSolverBase::DualExpr tmp(a);
    tmp-=b;
    return tmp;
  }
  ///\e

  ///\relates LpSolverBase::DualExpr
  ///
  inline LpSolverBase::DualExpr operator*(const LpSolverBase::DualExpr &a,
                                          const LpSolverBase::Value &b)
  {
    LpSolverBase::DualExpr tmp(a);
    tmp*=b;
    return tmp;
  }

  ///\e

  ///\relates LpSolverBase::DualExpr
  ///
  inline LpSolverBase::DualExpr operator*(const LpSolverBase::Value &a,
                                          const LpSolverBase::DualExpr &b)
  {
    LpSolverBase::DualExpr tmp(b);
    tmp*=a;
    return tmp;
  }
  ///\e

  ///\relates LpSolverBase::DualExpr
  ///
  inline LpSolverBase::DualExpr operator/(const LpSolverBase::DualExpr &a,
                                          const LpSolverBase::Value &b)
  {
    LpSolverBase::DualExpr tmp(a);
    tmp/=b;
    return tmp;
  }


} //namespace lemon

#endif //LEMON_LP_BASE_H
