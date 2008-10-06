#!/bin/bash

set -e

if [ $# -eq 0 -o x$1 = "x-h" -o x$1 = "x-help" -o x$1 = "x--help" ]; then
	echo "Usage:"
	echo "  $0 source-file"
	exit
fi

TMP=`mktemp`

sed -e "s/bipartite undirected graph/bipartite graph/g"\
	-e "s/undirected graph/_gr_aph_label_/g"\
	-e "s/undirected edge/_ed_ge_label_/g"\
	-e "s/graph_/_gr_aph_label__/g"\
	-e "s/_graph/__gr_aph_label_/g"\
	-e "s/UGraph/_Gr_aph_label_/g"\
	-e "s/uGraph/_gr_aph_label_/g"\
	-e "s/ugraph/_gr_aph_label_/g"\
	-e "s/Graph/_Digr_aph_label_/g"\
	-e "s/graph/_digr_aph_label_/g"\
	-e "s/UEdge/_Ed_ge_label_/g"\
	-e "s/uEdge/_ed_ge_label_/g"\
	-e "s/uedge/_ed_ge_label_/g"\
	-e "s/IncEdgeIt/_In_cEd_geIt_label_/g"\
	-e "s/Edge/_Ar_c_label_/g"\
	-e "s/edge/_ar_c_label_/g"\
	-e "s/ANode/_Re_d_label_/g"\
	-e "s/BNode/_Blu_e_label_/g"\
	-e "s/A-Node/_Re_d_label_/g"\
	-e "s/B-Node/_Blu_e_label_/g"\
	-e "s/anode/_re_d_label_/g"\
	-e "s/bnode/_blu_e_label_/g"\
	-e "s/aNode/_re_d_label_/g"\
	-e "s/bNode/_blu_e_label_/g"\
	-e "s/_Digr_aph_label_/Digraph/g"\
	-e "s/_digr_aph_label_/digraph/g"\
	-e "s/_Gr_aph_label_/Graph/g"\
	-e "s/_gr_aph_label_/graph/g"\
	-e "s/_Ar_c_label_/Arc/g"\
	-e "s/_ar_c_label_/arc/g"\
	-e "s/_Ed_ge_label_/Edge/g"\
	-e "s/_ed_ge_label_/edge/g"\
	-e "s/_In_cEd_geIt_label_/IncEdgeIt/g"\
	-e "s/_Re_d_label_/Red/g"\
	-e "s/_Blu_e_label_/Blue/g"\
	-e "s/_re_d_label_/red/g"\
	-e "s/_blu_e_label_/blue/g"\
<$1 > $TMP

mv $TMP $1