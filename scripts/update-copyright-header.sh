#!/bin/bash

YEAR=`date +2003-%Y`
HGROOT=`hg root`

function update_file() {
    TMP_FILE=`mktemp`
    FILE_NAME=$1

    (echo "/* -*- C++ -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library
 *
 * Copyright (C) "$YEAR"
 * Egervary Jeno Kombinatorikus Optimalizalasi Kutatocsoport
 * (Egervary Research Group on Combinatorial Optimization, EGRES).
 *
 * Permission to use, modify and distribute this software is granted
 * provided that this copyright notice appears in all copies. For
 * precise terms see the accompanying LICENSE file.
 *
 * This software is provided \"AS IS\" with no warranty of any kind,
 * express or implied, and with no claim as to its suitability for any
 * purpose.
 *
 */
"
	awk 'BEGIN { pm=0; }
     pm==3 { print }
     /\/\* / && pm==0 { pm=1;}
     /[^:blank:]/ && (pm==0 || pm==2) { pm=3; print;}
     /\*\// && pm==1 { pm=2;}
    ' $1
	) >$TMP_FILE
    rm $FILE_NAME
    mv $TMP_FILE $FILE_NAME
}


if [ $# == 0 ]; then
    echo -n Update all source files...
    for i in `hg manifest|grep -E  '\.(cc|h|dox)$'`
    do
	update_file $HGROOT/$i
    done
    echo ' done.'
else
    for i in $*
    do
	update_file $i
    done
fi
