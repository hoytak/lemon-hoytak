#!/bin/bash

set -e

if [ $# -eq 0 -o x$1 = "x-h" -o x$1 = "x-help" -o x$1 = "x--help" ]; then
    echo "Usage:"
    echo "  $0 source-file(s)"
    exit
fi

for i in $@
do
    echo Update $i...
    TMP=`mktemp`
    sed -e "s/undirected graph/_gr_aph_label_/g"\
        -e "s/undirected edge/_ed_ge_label_/g"\
        -e "s/UGraph/_Gr_aph_label_/g"\
        -e "s/u[Gg]raph/_gr_aph_label_/g"\
        -e "s/\<Graph\>/_Digr_aph_label_/g"\
        -e "s/\<graph\>/_digr_aph_label_/g"\
        -e "s/\<Graphs\>/_Digr_aph_label_s/g"\
        -e "s/\<graphs\>/_digr_aph_label_s/g"\
        -e "s/_Graph/__Gr_aph_label_/g"\
        -e "s/\([Gg]\)raph\([a-z_]\)/_\1r_aph_label_\2/g"\
        -e "s/\([a-z_]\)graph/\1_gr_aph_label_/g"\
        -e "s/Graph/_Digr_aph_label_/g"\
        -e "s/graph/_digr_aph_label_/g"\
        -e "s/UEdge/_Ed_ge_label_/g"\
        -e "s/u[Ee]dge/_ed_ge_label_/g"\
        -e "s/IncEdgeIt/_In_cEd_geIt_label_/g"\
        -e "s/\<Edge\>/_Ar_c_label_/g"\
        -e "s/\<edge\>/_ar_c_label_/g"\
        -e "s/\<Edges\>/_Ar_c_label_s/g"\
        -e "s/\<edges\>/_ar_c_label_s/g"\
        -e "s/_Edge/__Ed_ge_label_/g"\
        -e "s/Edge\([a-z_]\)/_Ed_ge_label_\1/g"\
        -e "s/edge\([a-z_]\)/_ed_ge_label_\1/g"\
        -e "s/\([a-z_]\)edge/\1_ed_ge_label_/g"\
        -e "s/Edge/_Ar_c_label_/g"\
        -e "s/edge/_ar_c_label_/g"\
        -e "s/A[Nn]ode/_Re_d_label_/g"\
        -e "s/B[Nn]ode/_Blu_e_label_/g"\
        -e "s/A-[Nn]ode/_Re_d_label_/g"\
        -e "s/B-[Nn]ode/_Blu_e_label_/g"\
        -e "s/a[Nn]ode/_re_d_label_/g"\
        -e "s/b[Nn]ode/_blu_e_label_/g"\
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
        -e "s/DigraphToEps/GraphToEps/g"\
        -e "s/digraphToEps/graphToEps/g"\
        -e "s/\<DefPredMap\>/SetPredMap/g"\
        -e "s/\<DefDistMap\>/SetDistMap/g"\
        -e "s/\<DefReachedMap\>/SetReachedMap/g"\
        -e "s/\<DefProcessedMap\>/SetProcessedMap/g"\
        -e "s/\<DefHeap\>/SetHeap/g"\
        -e "s/\<DefStandardHeap\>/SetStandradHeap/g"\
        -e "s/\<DefOperationTraits\>/SetOperationTraits/g"\
        -e "s/\<DefProcessedMapToBeDefaultMap\>/SetStandardProcessedMap/g"\
        -e "s/\<copyGraph\>/graphCopy/g"\
        -e "s/\<copyDigraph\>/digraphCopy/g"\
        -e "s/\<IntegerMap\>/RangeMap/g"\
        -e "s/\<integerMap\>/rangeMap/g"\
        -e "s/\<\([sS]\)tdMap\>/\1parseMap/g"\
        -e "s/\<\([Ff]\)unctorMap\>/\1unctorToMap/g"\
        -e "s/\<\([Mm]\)apFunctor\>/\1apToFunctor/g"\
        -e "s/\<\([Ff]\)orkWriteMap\>/\1orkMap/g"\
        -e "s/\<StoreBoolMap\>/LoggerBoolMap/g"\
        -e "s/\<storeBoolMap\>/loggerBoolMap/g"\
        -e "s/\<BoundingBox\>/Box/g"\
    <$i > $TMP
    mv $TMP $i
done
