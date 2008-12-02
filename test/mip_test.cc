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

#include "test_tools.h"


#ifdef HAVE_CONFIG_H
#include <lemon/config.h>
#endif

#ifdef HAVE_CPLEX
#include <lemon/mip_cplex.h>
#endif

#ifdef HAVE_GLPK
#include <lemon/mip_glpk.h>
#endif


using namespace lemon;

void solveAndCheck(MipSolverBase& lp, MipSolverBase::SolutionStatus stat,
                   double exp_opt) {
  using std::string;

  lp.solve();
  //int decimal,sign;
  std::ostringstream buf;
  buf << "Primalstatus should be: " << int(stat)
      <<" and it is "<<int(lp.mipStatus());


  //  itoa(stat,buf1, 10);
  check(lp.mipStatus()==stat, buf.str());

  if (stat ==  MipSolverBase::OPTIMAL) {
    std::ostringstream sbuf;
    buf << "Wrong optimal value: the right optimum is " << exp_opt;
    check(std::abs(lp.primalValue()-exp_opt) < 1e-3, sbuf.str());
    //+ecvt(exp_opt,2)
  }
}

void aTest(MipSolverBase& mip)
{
 //The following example is very simple


  typedef MipSolverBase::Row Row;
  typedef MipSolverBase::Col Col;



  Col x1 = mip.addCol();
  Col x2 = mip.addCol();


  //Objective function
  mip.obj(x1);

  mip.max();


  //Unconstrained optimization
  mip.solve();
  //Check it out!

  //Constraints
  mip.addRow(2*x1+x2 <=2);
  mip.addRow(x1-2*x2 <=0);

  //Nonnegativity of the variable x1
  mip.colLowerBound(x1, 0);

  //Maximization of x1
  //over the triangle with vertices (0,0),(4/5,2/5),(0,2)
  double expected_opt=4.0/5.0;
  solveAndCheck(mip, MipSolverBase::OPTIMAL, expected_opt);

  //Restrict x2 to integer
  mip.colType(x2,MipSolverBase::INT);
  expected_opt=1.0/2.0;
  solveAndCheck(mip, MipSolverBase::OPTIMAL, expected_opt);


  //Restrict both to integer
  mip.colType(x1,MipSolverBase::INT);
  expected_opt=0;
  solveAndCheck(mip, MipSolverBase::OPTIMAL, expected_opt);



}


int main()
{

#ifdef HAVE_GLPK
  MipGlpk mip1;
  aTest(mip1);
#endif

#ifdef HAVE_CPLEX
  MipCplex mip2;
  aTest(mip2);
#endif

  return 0;

}
