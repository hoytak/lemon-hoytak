#! /usr/bin/env python

import sys
import os

if len(sys.argv)>1 and sys.argv[1] in ["-h","--help"]:
    print """
This utility just prints the length of the longest path
in the revision graph from revison 0 to the current one.
"""
    exit(0)
plist = os.popen("HGRCPATH='' hg parents --template='{rev}\n'").readlines()
if len(plist)>1:
    print "You are in the process of merging"
    exit(1)
PAR = int(plist[0])

f = os.popen("HGRCPATH='' hg log -r 0:tip --template='{rev} {parents}\n'").\
    readlines()
REV = -1
lengths=[]
for l in f:
    REV+=1
    s = l.split()
    rev = int(s[0])
    if REV != rev:
        print "Something is seriously wrong"
        exit(1)
    if len(s) == 1:
        par1 = par2 = rev - 1
    elif len(s) == 2:
        par1 = par2 = int(s[1].split(":")[0])
    else:
        par1 = int(s[1].split(":")[0])
        par2 = int(s[2].split(":")[0])
    if rev == 0:
        lengths.append(0)
    else:
        lengths.append(max(lengths[par1],lengths[par2])+1)
print lengths[PAR]
