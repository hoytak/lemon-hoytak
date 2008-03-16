/* -*- C++ -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library
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

///\ingroup demos
///\file
///\brief Argument parser demo
///
/// This example shows how the argument parser can be used.
///
/// \include arg_parser_demo.cc

#include <lemon/arg_parser.h>

using namespace lemon;
int main(int argc, const char **argv)
{
  ArgParser ap(argc,argv);
  int i;
  std::string s;
  double d;
  bool b,sil;
  bool g1,g2,g3;
  ap.refOption("n", "An integer input.", i, true)
    .refOption("val", "A double input.", d)
    .synonym("vals","val")
    .refOption("name", "A string input.", s)
    .refOption("f", "A switch.", b)
    .refOption("nohelp", "", sil)
    .refOption("gra","Choice A",g1)
    .refOption("grb","Choice B",g2)
    .refOption("grc","Choice C",g3)
    .optionGroup("gr","gra")
    .optionGroup("gr","grb")
    .optionGroup("gr","grc")
    .mandatoryGroup("gr")
    .onlyOneGroup("gr")
    .other("infile","The input file.")
    .other("...");
  
  ap.parse();

  std::cout << "Parameters of '" << ap.commandName() << "':\n";

  if(ap.given("n")) std::cout << "  Value of -n: " << i << std::endl;
  if(ap.given("val")) std::cout << "  Value of -val: " << d << std::endl;
  if(ap.given("name")) std::cout << "  Value of -name: " << s << std::endl;
  if(ap.given("f")) std::cout << "  -f is given\n";
  if(ap.given("nohelp")) std::cout << "  Value of -nohelp: " << sil << std::endl;
  if(ap.given("gra")) std::cout << "  -gra is given\n";
  if(ap.given("grb")) std::cout << "  -grb is given\n";
  if(ap.given("grc")) std::cout << "  -grc is given\n";
                                     
  switch(ap.files().size()) {
  case 0:
    std::cout << "  No file argument was given.\n";
    break;
  case 1:
    std::cout << "  1 file argument was given. It is:\n";
    break;
  default:
    std::cout << "  "
	      << ap.files().size() << " file arguments were given. They are:\n";
  }
  for(unsigned int i=0;i<ap.files().size();++i)
    std::cout << "    '" << ap.files()[i] << "'\n";
  
}
