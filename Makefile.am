ACLOCAL_AMFLAGS = -I m4

AM_CPPFLAGS = -I$(top_srcdir) -I$(top_builddir)
LDADD = $(top_builddir)/lemon/libemon.la

EXTRA_DIST = \
	LICENSE \
	m4/lx_check_cplex.m4 \
	m4/lx_check_glpk.m4 \
	m4/lx_check_soplex.m4 \
	CMakeLists.txt

pkgconfigdir = $(libdir)/pkgconfig
lemondir = $(pkgincludedir)
bitsdir = $(lemondir)/bits
conceptdir = $(lemondir)/concepts
pkgconfig_DATA =
lib_LTLIBRARIES =
lemon_HEADERS =
bits_HEADERS =
concept_HEADERS =
noinst_HEADERS =
noinst_PROGRAMS =
bin_PROGRAMS =
check_PROGRAMS =
TESTS =
XFAIL_TESTS =

include lemon/Makefile.am
include test/Makefile.am
include doc/Makefile.am
include demo/Makefile.am
include benchmark/Makefile.am
include tools/Makefile.am

MRPROPERFILES = \
	aclocal.m4 \
	config.h.in \
	config.h.in~ \
	configure \
	Makefile.in \
	build-aux/config.guess \
	build-aux/config.sub \
	build-aux/depcomp \
	build-aux/install-sh \
	build-aux/ltmain.sh \
	build-aux/missing \
	doc/doxygen.log

mrproper:
	$(MAKE) $(AM_MAKEFLAGS) maintainer-clean
	-rm -f $(MRPROPERFILES)

dist-bz2: dist
	zcat $(PACKAGE)-$(VERSION).tar.gz | \
	bzip2 --best -c > $(PACKAGE)-$(VERSION).tar.bz2

distcheck-bz2: distcheck
	zcat $(PACKAGE)-$(VERSION).tar.gz | \
	bzip2 --best -c > $(PACKAGE)-$(VERSION).tar.bz2

.PHONY: mrproper dist-bz2 distcheck-bz2
