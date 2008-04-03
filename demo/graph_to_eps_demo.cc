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

/// \ingroup demos
/// \file
/// \brief Demo of the graph grawing function \ref graphToEps()
///
/// This demo program shows examples how to  use the function \ref
/// graphToEps(). It takes no input but simply creates  six
/// <tt>.eps</tt> files demonstrating the capability of \ref
/// graphToEps(), and showing how to draw directed/graphs,
/// how to handle parallel egdes, how to change the properties (like
/// color, shape, size, title etc.) of nodes and arcs individually
/// using appropriate \ref maps-page "graph maps".
///
/// \include graph_to_eps_demo.cc

#include <lemon/math.h>

#include<lemon/graph_to_eps.h>
#include<lemon/list_graph.h>
#include<lemon/graph_utils.h>

using namespace std;
using namespace lemon;

int main()
{
  Palette palette;
  Palette paletteW(-1,true);

  ListDigraph g;
  typedef ListDigraph::Node Node;
  typedef ListDigraph::NodeIt NodeIt;
  typedef ListDigraph::Arc Arc;
  typedef dim2::Point<int> Point;
  
  Node n1=g.addNode();
  Node n2=g.addNode();
  Node n3=g.addNode();
  Node n4=g.addNode();
  Node n5=g.addNode();

  ListDigraph::NodeMap<Point> coords(g);
  ListDigraph::NodeMap<double> sizes(g);
  ListDigraph::NodeMap<int> colors(g);
  ListDigraph::NodeMap<int> shapes(g);
  ListDigraph::ArcMap<int> ecolors(g);
  ListDigraph::ArcMap<int> widths(g);
  
  coords[n1]=Point(50,50);  sizes[n1]=1; colors[n1]=1; shapes[n1]=0;
  coords[n2]=Point(50,70);  sizes[n2]=2; colors[n2]=2; shapes[n2]=2;
  coords[n3]=Point(70,70);  sizes[n3]=1; colors[n3]=3; shapes[n3]=0;
  coords[n4]=Point(70,50);  sizes[n4]=2; colors[n4]=4; shapes[n4]=1;
  coords[n5]=Point(85,60);  sizes[n5]=3; colors[n5]=5; shapes[n5]=2;
  
  Arc e;

  e=g.addArc(n1,n2); ecolors[e]=0; widths[e]=1;
  e=g.addArc(n2,n3); ecolors[e]=0; widths[e]=1;
  e=g.addArc(n3,n5); ecolors[e]=0; widths[e]=3;
  e=g.addArc(n5,n4); ecolors[e]=0; widths[e]=1;
  e=g.addArc(n4,n1); ecolors[e]=0; widths[e]=1;
  e=g.addArc(n2,n4); ecolors[e]=1; widths[e]=2;
  e=g.addArc(n3,n4); ecolors[e]=2; widths[e]=1;
  
  IdMap<ListDigraph,Node> id(g);

  cout << "Create 'graph_to_eps_demo_out_pure.eps'" << endl;
  graphToEps(g,"graph_to_eps_demo_out_pure.eps").
    //scale(10).
    coords(coords).
    title("Sample .eps figure").
    copyright("(C) 2003-2007 LEMON Project").
    run();

  cout << "Create 'graph_to_eps_demo_out.eps'" << endl;
  graphToEps(g,"graph_to_eps_demo_out.eps").
    //scale(10).
    coords(coords).
    title("Sample .eps figure").
    copyright("(C) 2003-2007 LEMON Project").
    absoluteNodeSizes().absoluteArcWidths().
    nodeScale(2).nodeSizes(sizes).
    nodeShapes(shapes).
    nodeColors(composeMap(palette,colors)).
    arcColors(composeMap(palette,ecolors)).
    arcWidthScale(.4).arcWidths(widths).
    nodeTexts(id).nodeTextSize(3).
    run();


  cout << "Create 'graph_to_eps_demo_out_arr.eps'" << endl;
  graphToEps(g,"graph_to_eps_demo_out_arr.eps").
    //scale(10).
    title("Sample .eps figure (with arrowheads)").
    copyright("(C) 2003-2007 LEMON Project").
    absoluteNodeSizes().absoluteArcWidths().
    nodeColors(composeMap(palette,colors)).
    coords(coords).
    nodeScale(2).nodeSizes(sizes).
    nodeShapes(shapes).
    arcColors(composeMap(palette,ecolors)).
    arcWidthScale(.4).arcWidths(widths).
    nodeTexts(id).nodeTextSize(3).
    drawArrows().arrowWidth(1).arrowLength(1).
    run();

  e=g.addArc(n1,n4); ecolors[e]=2; widths[e]=1;
  e=g.addArc(n4,n1); ecolors[e]=1; widths[e]=2;

  e=g.addArc(n1,n2); ecolors[e]=1; widths[e]=1;
  e=g.addArc(n1,n2); ecolors[e]=2; widths[e]=1;
  e=g.addArc(n1,n2); ecolors[e]=3; widths[e]=1;
  e=g.addArc(n1,n2); ecolors[e]=4; widths[e]=1;
  e=g.addArc(n1,n2); ecolors[e]=5; widths[e]=1;
  e=g.addArc(n1,n2); ecolors[e]=6; widths[e]=1;
  e=g.addArc(n1,n2); ecolors[e]=7; widths[e]=1;

  cout << "Create 'graph_to_eps_demo_out_par.eps'" << endl;
  graphToEps(g,"graph_to_eps_demo_out_par.eps").
    //scale(10).
    title("Sample .eps figure (parallel arcs)").
    copyright("(C) 2003-2007 LEMON Project").
    absoluteNodeSizes().absoluteArcWidths().
    nodeShapes(shapes).
    coords(coords).
    nodeScale(2).nodeSizes(sizes).
    nodeColors(composeMap(palette,colors)).
    arcColors(composeMap(palette,ecolors)).
    arcWidthScale(.4).arcWidths(widths).
    nodeTexts(id).nodeTextSize(3).
    enableParallel().parArcDist(1.5).
    run();
  
  cout << "Create 'graph_to_eps_demo_out_par_arr.eps'" << endl;
  graphToEps(g,"graph_to_eps_demo_out_par_arr.eps").
    //scale(10).
    title("Sample .eps figure (parallel arcs and arrowheads)").
    copyright("(C) 2003-2007 LEMON Project").
    absoluteNodeSizes().absoluteArcWidths().
    nodeScale(2).nodeSizes(sizes).
    coords(coords).
    nodeShapes(shapes).
    nodeColors(composeMap(palette,colors)).
    arcColors(composeMap(palette,ecolors)).
    arcWidthScale(.3).arcWidths(widths).
    nodeTexts(id).nodeTextSize(3).
    enableParallel().parArcDist(1).
    drawArrows().arrowWidth(1).arrowLength(1).
    run();

  cout << "Create 'graph_to_eps_demo_out_a4.eps'" << endl;
  graphToEps(g,"graph_to_eps_demo_out_a4.eps").scaleToA4().
    title("Sample .eps figure (fits to A4)").
    copyright("(C) 2003-2007 LEMON Project").
    absoluteNodeSizes().absoluteArcWidths().
    nodeScale(2).nodeSizes(sizes).
    coords(coords).
    nodeShapes(shapes).
    nodeColors(composeMap(palette,colors)).
    arcColors(composeMap(palette,ecolors)).
    arcWidthScale(.3).arcWidths(widths).
    nodeTexts(id).nodeTextSize(3).
    enableParallel().parArcDist(1).
    drawArrows().arrowWidth(1).arrowLength(1).
    run();

  ListDigraph h;
  ListDigraph::NodeMap<int> hcolors(h);
  ListDigraph::NodeMap<Point> hcoords(h);
  
  int cols=int(sqrt(double(palette.size())));
  for(int i=0;i<int(paletteW.size());i++) {
    Node n=h.addNode();
    hcoords[n]=Point(i%cols,i/cols);
    hcolors[n]=i;
  }
  
  cout << "Create 'graph_to_eps_demo_out_colors.eps'" << endl;
  graphToEps(h,"graph_to_eps_demo_out_colors.eps").
    //scale(60).
    title("Sample .eps figure (Palette demo)").
    copyright("(C) 2003-2007 LEMON Project").
    coords(hcoords).
    absoluteNodeSizes().absoluteArcWidths().
    nodeScale(45).
    distantColorNodeTexts().
    //    distantBWNodeTexts().
    nodeTexts(hcolors).nodeTextSize(.6).
    nodeColors(composeMap(paletteW,hcolors)).
    run();
}
