#!/bin/bash

set -e

if [ $# -eq 0 -o x$1 = "x-h" -o x$1 = "x-help" -o x$1 = "x--help" ]; then
	echo "Usage:"
	echo "  $0 source-file"
	exit
fi

TMP=`mktemp`

sed	-e "s/undirected graph/_gr_aph_label_/g"\
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
	-e "s/\(\W\)DefPredMap\(\W\)/\1SetPredMap\2/g"\
	-e "s/\(\W\)DefPredMap$/\1SetPredMap/g"\
	-e "s/^DefPredMap\(\W\)/SetPredMap\1/g"\
	-e "s/^DefPredMap$/SetPredMap/g"\
	-e "s/\(\W\)DefDistMap\(\W\)/\1SetDistMap\2/g"\
	-e "s/\(\W\)DefDistMap$/\1SetDistMap/g"\
	-e "s/^DefDistMap\(\W\)/SetDistMap\1/g"\
	-e "s/^DefDistMap$/SetDistMap/g"\
	-e "s/\(\W\)DefReachedMap\(\W\)/\1SetReachedMap\2/g"\
	-e "s/\(\W\)DefReachedMap$/\1SetReachedMap/g"\
	-e "s/^DefReachedMap\(\W\)/SetReachedMap\1/g"\
	-e "s/^DefReachedMap$/SetReachedMap/g"\
	-e "s/\(\W\)DefProcessedMap\(\W\)/\1SetProcessedMap\2/g"\
	-e "s/\(\W\)DefProcessedMap$/\1SetProcessedMap/g"\
	-e "s/^DefProcessedMap\(\W\)/SetProcessedMap\1/g"\
	-e "s/^DefProcessedMap$/SetProcessedMap/g"\
	-e "s/\(\W\)DefHeap\(\W\)/\1SetHeap\2/g"\
	-e "s/\(\W\)DefHeap$/\1SetHeap/g"\
	-e "s/^DefHeap\(\W\)/SetHeap\1/g"\
	-e "s/^DefHeap$/SetHeap/g"\
	-e "s/\(\W\)DefStandardHeap\(\W\)/\1SetStandradHeap\2/g"\
	-e "s/\(\W\)DefStandardHeap$/\1SetStandradHeap/g"\
	-e "s/^DefStandardHeap\(\W\)/SetStandradHeap\1/g"\
	-e "s/^DefStandardHeap$/SetStandradHeap/g"\
	-e "s/\(\W\)DefOperationTraits\(\W\)/\1SetOperationTraits\2/g"\
	-e "s/\(\W\)DefOperationTraits$/\1SetOperationTraits/g"\
	-e "s/^DefOperationTraits\(\W\)/SetOperationTraits\1/g"\
	-e "s/^DefOperationTraits$/SetOperationTraits/g"\
	-e "s/\(\W\)DefProcessedMapToBeDefaultMap\(\W\)/\1SetStandardProcessedMap\2/g"\
	-e "s/\(\W\)DefProcessedMapToBeDefaultMap$/\1SetStandardProcessedMap/g"\
	-e "s/^DefProcessedMapToBeDefaultMap\(\W\)/SetStandardProcessedMap\1/g"\
	-e "s/^DefProcessedMapToBeDefaultMap$/SetStandardProcessedMap/g"\
	-e "s/\(\W\)IntegerMap\(\W\)/\1RangeMap\2/g"\
	-e "s/\(\W\)IntegerMap$/\1RangeMap/g"\
	-e "s/^IntegerMap\(\W\)/RangeMap\1/g"\
	-e "s/^IntegerMap$/RangeMap/g"\
	-e "s/\(\W\)integerMap\(\W\)/\1rangeMap\2/g"\
	-e "s/\(\W\)integerMap$/\1rangeMap/g"\
	-e "s/^integerMap\(\W\)/rangeMap\1/g"\
	-e "s/^integerMap$/rangeMap/g"\
	-e "s/\(\W\)copyGraph\(\W\)/\1graphCopy\2/g"\
	-e "s/\(\W\)copyGraph$/\1graphCopy/g"\
	-e "s/^copyGraph\(\W\)/graphCopy\1/g"\
	-e "s/^copyGraph$/graphCopy/g"\
	-e "s/\(\W\)copyDigraph\(\W\)/\1digraphCopy\2/g"\
	-e "s/\(\W\)copyDigraph$/\1digraphCopy/g"\
	-e "s/^copyDigraph\(\W\)/digraphCopy\1/g"\
	-e "s/^copyDigraph$/digraphCopy/g"\
	-e "s/\(\W\)\([sS]\)tdMap\(\W\)/\1\2parseMap\3/g"\
	-e "s/\(\W\)\([sS]\)tdMap$/\1\2parseMap/g"\
	-e "s/^\([sS]\)tdMap\(\W\)/\1parseMap\2/g"\
	-e "s/^\([sS]\)tdMap$/\1parseMap/g"\
	-e "s/\(\W\)\([Ff]\)unctorMap\(\W\)/\1\2unctorToMap\3/g"\
	-e "s/\(\W\)\([Ff]\)unctorMap$/\1\2unctorToMap/g"\
	-e "s/^\([Ff]\)unctorMap\(\W\)/\1unctorToMap\2/g"\
	-e "s/^\([Ff]\)unctorMap$/\1unctorToMap/g"\
	-e "s/\(\W\)\([Mm]\)apFunctor\(\W\)/\1\2apToFunctor\3/g"\
	-e "s/\(\W\)\([Mm]\)apFunctor$/\1\2apToFunctor/g"\
	-e "s/^\([Mm]\)apFunctor\(\W\)/\1apToFunctor\2/g"\
	-e "s/^\([Mm]\)apFunctor$/\1apToFunctor/g"\
	-e "s/\(\W\)\([Ff]\)orkWriteMap\(\W\)/\1\2orkMap\3/g"\
	-e "s/\(\W\)\([Ff]\)orkWriteMap$/\1\2orkMap/g"\
	-e "s/^\([Ff]\)orkWriteMap\(\W\)/\1orkMap\2/g"\
	-e "s/^\([Ff]\)orkWriteMap$/\1orkMap/g"\
	-e "s/\(\W\)StoreBoolMap\(\W\)/\1LoggerBoolMap\2/g"\
	-e "s/\(\W\)StoreBoolMap$/\1LoggerBoolMap/g"\
	-e "s/^StoreBoolMap\(\W\)/LoggerBoolMap\1/g"\
	-e "s/^StoreBoolMap$/LoggerBoolMap/g"\
	-e "s/\(\W\)storeBoolMap\(\W\)/\1loggerBoolMap\2/g"\
	-e "s/\(\W\)storeBoolMap$/\1loggerBoolMap/g"\
	-e "s/^storeBoolMap\(\W\)/loggerBoolMap\1/g"\
	-e "s/^storeBoolMap$/loggerBoolMap/g"\
	-e "s/\(\W\)BoundingBox\(\W\)/\1Box\2/g"\
	-e "s/\(\W\)BoundingBox$/\1Box/g"\
	-e "s/^BoundingBox\(\W\)/Box\1/g"\
	-e "s/^BoundingBox$/Box/g"\
<$1 > $TMP

mv $TMP $1