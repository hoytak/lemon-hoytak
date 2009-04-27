#!/bin/bash

YEAR=`date +%Y`
HGROOT=`hg root`

function hg_year() {
    if [ -n "$(hg st $1)" ]; then
        echo $YEAR
    else
        hg log -l 1 --template='{date|isodate}\n' $1 |
        cut -d '-' -f 1
    fi
}

function update_header() {
    TMP_FILE=`mktemp`
    FILE_NAME=$1

    (echo "/* -*- mode: C++; indent-tabs-mode: nil; -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library.
 *
 * Copyright (C) 2003-"$(hg_year $1)"
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

    HEADER_CH=`diff -q $TMP_FILE $FILE_NAME >/dev/null&&echo NO||echo YES`

    rm $FILE_NAME
    mv $TMP_FILE $FILE_NAME
}

function update_tabs() {
    TMP_FILE=`mktemp`
    FILE_NAME=$1

    cat $1 |
    sed -e 's/\t/        /g' >$TMP_FILE

    TABS_CH=`diff -q $TMP_FILE $FILE_NAME >/dev/null&&echo NO||echo YES`

    rm $FILE_NAME
    mv $TMP_FILE $FILE_NAME
}

function remove_trailing_space() {
    TMP_FILE=`mktemp`
    FILE_NAME=$1

    cat $1 |
    sed -e 's/ \+$//g' >$TMP_FILE

    SPACES_CH=`diff -q $TMP_FILE $FILE_NAME >/dev/null&&echo NO||echo YES`

    rm $FILE_NAME
    mv $TMP_FILE $FILE_NAME
}

function long_line_test() {
    cat $1 |grep -q -E '.{81,}'
}

function update_file() {
    echo -n '    update' $i ...

    update_header $1
    update_tabs $1
    remove_trailing_space $1

    CHANGED=NO;
    if [[ $HEADER_CH = YES ]];
    then
	echo -n '  [header updated]'
	CHANGED=YES;
    fi
    if [[ $TABS_CH = YES ]];
    then
	echo -n ' [tabs removed]'
	CHANGED=YES;
    fi
    if [[ $SPACES_CH = YES ]];
    then
	echo -n ' [trailing spaces removed]'
	CHANGED=YES;
    fi
    if long_line_test $1 ;
    then
	echo -n ' [LONG LINES]'
	((LONG_LINE_FILES++))
    fi
    echo
    if [[ $CHANGED = YES ]];
    then
	((CHANGED_FILES++))
    fi
}

CHANGED_FILES=0
TOTAL_FILES=0
LONG_LINE_FILES=0
if [ $# == 0 ]; then
    echo Update all source files...
    for i in `hg manifest|grep -E  '\.(cc|h|dox)$'`
    do
	update_file $HGROOT/$i
	((TOTAL_FILES++))
    done
    echo '  done.'
else
    for i in $*
    do
	update_file $i
	((TOTAL_FILES++))
    done
fi
echo $CHANGED_FILES out of $TOTAL_FILES files has been changed.
if [[ $LONG_LINE_FILES -gt 1 ]]; then
    echo
    echo WARNING: $LONG_LINE_FILES files contains long lines!    
    echo
elif [[ $LONG_LINE_FILES -gt 0 ]]; then
    echo
    echo WARNING: a file contains long lines!
    echo
fi
