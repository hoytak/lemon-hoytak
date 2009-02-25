#include <iostream>

#include "test_tools.h"
#include <lemon/smart_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/gomory_hu.h>
#include <cstdlib>

using namespace std;
using namespace lemon;

typedef SmartGraph Graph;

char test_lgf[] =
  "@nodes\n"
  "label\n"
  "0\n"
  "1\n"
  "2\n"
  "3\n"
  "4\n"
  "@arcs\n"
  "     label capacity\n"
  "0 1  0     1\n"
  "1 2  1     1\n"
  "2 3  2     1\n"
  "0 3  4     5\n"
  "0 3  5     10\n"
  "0 3  6     7\n"
  "4 2  7     1\n"
  "@attributes\n"
  "source 0\n"
  "target 3\n";
  
GRAPH_TYPEDEFS(Graph);
typedef Graph::EdgeMap<int> IntEdgeMap;
typedef Graph::NodeMap<bool> BoolNodeMap;

int cutValue(const Graph& graph, const BoolNodeMap& cut,
	     const IntEdgeMap& capacity) {

  int sum = 0;
  for (EdgeIt e(graph); e != INVALID; ++e) {
    Node s = graph.u(e);
    Node t = graph.v(e);

    if (cut[s] != cut[t]) {
      sum += capacity[e];
    }
  }
  return sum;
}


int main() {
  Graph graph;
  IntEdgeMap capacity(graph);

  std::istringstream input(test_lgf);
  GraphReader<Graph>(graph, input).
    edgeMap("capacity", capacity).run();

  GomoryHu<Graph> ght(graph, capacity);
  ght.init();
  ght.run();

  for (NodeIt u(graph); u != INVALID; ++u) {
    for (NodeIt v(graph); v != u; ++v) {
      Preflow<Graph, IntEdgeMap> pf(graph, capacity, u, v);
      pf.runMinCut();
      BoolNodeMap cm(graph);
      ght.minCutMap(u, v, cm);
      check(pf.flowValue() == ght.minCutValue(u, v), "Wrong cut 1");
      check(cm[u] != cm[v], "Wrong cut 3");
      check(pf.flowValue() == cutValue(graph, cm, capacity), "Wrong cut 2");

      int sum=0;
      for(GomoryHu<Graph>::MinCutEdgeIt a(ght, u, v);a!=INVALID;++a)
        sum+=capacity[a]; 
      check(sum == ght.minCutValue(u, v), "Problem with MinCutEdgeIt");

      sum=0;
      for(GomoryHu<Graph>::MinCutNodeIt n(ght, u, v,true);n!=INVALID;++n)
        sum++;
      for(GomoryHu<Graph>::MinCutNodeIt n(ght, u, v,false);n!=INVALID;++n)
        sum++;
      check(sum == countNodes(graph), "Problem with MinCutNodeIt");
      
    }
  }
  
  return 0;
}
