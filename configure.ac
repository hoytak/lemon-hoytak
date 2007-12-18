dnl Process this file with autoconf to produce a configure script.

dnl Version information.
m4_define([lemon_version_major], [0])
m4_define([lemon_version_minor], [6])
m4_define([lemon_version_micro], [90])
m4_define([lemon_version_nano], [])
m4_define([lemon_version_tag], [hg])
m4_define([lemon_hg_revision], [m4_normalize(esyscmd([hg tip |grep ^changeset |cut -d ':' -f 3]))])
m4_define([lemon_version], [lemon_version_major().lemon_version_minor()ifelse(lemon_version_micro(), [], [], [.lemon_version_micro()])ifelse(lemon_version_nano(), [], [], [.lemon_version_nano()])ifelse(lemon_version_tag(), [], [], lemon_version_tag(), [hg], [[_]lemon_version_tag()[]lemon_hg_revision()], [[_]lemon_version_tag()])])

AC_PREREQ([2.59])
AC_INIT([LEMON], [lemon_version()], [etik-ol@cs.elte.hu], [lemon])
AC_CONFIG_AUX_DIR([build-aux])
AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE([-Wall -Werror foreign subdir-objects])
AC_CONFIG_SRCDIR([lemon/list_graph.h])
AC_CONFIG_HEADERS([config.h lemon/config.h])

lx_cmdline_cxxflags_set=${CXXFLAGS+set}

dnl Checks for programs.
AC_PROG_CXX
AC_PROG_CXXCPP
AC_PROG_INSTALL
AC_DISABLE_SHARED
AC_PROG_LIBTOOL

if test x"$lx_cmdline_cxxflags_set" != x"set" -a "$GXX" = yes; then
  CXXFLAGS="$CXXFLAGS -Wall -W -Wall -W -Wunused -Wformat=2 -Wctor-dtor-privacy -Wnon-virtual-dtor -Wno-char-subscripts -Wwrite-strings -Wno-char-subscripts -Wreturn-type -Wcast-qual -Wcast-align -Wsign-promo -Woverloaded-virtual -Woverloaded-virtual -ansi -fno-strict-aliasing -Wold-style-cast -Wno-unknown-pragmas"
fi

AC_CHECK_PROG([doxygen_found],[doxygen],[yes],[no])

dnl Checks for libraries.
LX_CHECK_GLPK
LX_CHECK_CPLEX
LX_CHECK_SOPLEX

dnl Enable/disable installing the documentation
AC_ARG_ENABLE([doc],
AS_HELP_STRING([--enable-doc@<:@=yes|no|full@:>@], [build the documentation (full enables internal documentation too) @<:@default=yes@:>@])
AS_HELP_STRING([--disable-doc], [do not build the documentation]),
              [], [enable_doc=yes])

AC_MSG_CHECKING([whether to build the documention])
case "$enable_doc" in
  yes)
    DOXYGEN_INTERNAL_DOCS=NO
    AC_MSG_RESULT([yes])
    ;;
  full)
    DOXYGEN_INTERNAL_DOCS=YES
    AC_MSG_RESULT([full])
    ;;
  no)
    DOXYGEN_INTERNAL_DOCS=NO
    AC_MSG_RESULT([no])
    ;;
  *)
    AC_MSG_ERROR([bad value $enable_doc for option --enable-doc])
    ;;
esac
AC_SUBST(DOXYGEN_INTERNAL_DOCS)
AM_CONDITIONAL([WANT_DOC], [test x"$enable_doc" != x"no"])

dnl Disable/enable building the demo programs
AC_ARG_ENABLE([demo],
AS_HELP_STRING([--enable-demo], [build the demo programs])
AS_HELP_STRING([--disable-demo], [do not build the demo programs @<:@default@:>@]),
              [], [enable_demo=no])
AC_MSG_CHECKING([whether to build the demo programs])
if test x"$enable_demo" != x"no"; then
  AC_MSG_RESULT([yes])
else
  AC_MSG_RESULT([no])
fi
AM_CONDITIONAL([WANT_DEMO], [test x"$enable_demo" != x"no"])

dnl Disable/enable building the binary tools
AC_ARG_ENABLE([tools],
AS_HELP_STRING([--enable-tools], [build additional tools @<:@default@:>@])
AS_HELP_STRING([--disable-tools], [do not build additional tools]),
              [], [enable_tools=yes])
AC_MSG_CHECKING([whether to build the additional tools])
if test x"$enable_tools" != x"no"; then
  AC_MSG_RESULT([yes])
else
  AC_MSG_RESULT([no])
fi
AM_CONDITIONAL([WANT_TOOLS], [test x"$enable_tools" != x"no"])

dnl Disable/enable building the benchmarks
AC_ARG_ENABLE([benchmark],
AS_HELP_STRING([--enable-benchmark], [build the benchmarks])
AS_HELP_STRING([--disable-benchmark], [do not build the benchmarks @<:@default@:>@]),
              [], [enable_benchmark=no])
AC_MSG_CHECKING([whether to build the benchmarks])
if test x"$enable_benchmark" != x"no"; then
  AC_MSG_RESULT([yes])
else
  AC_MSG_RESULT([no])
fi
AM_CONDITIONAL([WANT_BENCHMARK], [test x"$enable_benchmark" != x"no"])

dnl Checks for header files.
AC_CHECK_HEADERS(limits.h sys/time.h sys/times.h unistd.h)

dnl Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_C_INLINE
AC_TYPE_SIZE_T
AC_HEADER_TIME
AC_STRUCT_TM

dnl Checks for library functions.
AC_HEADER_STDC
AC_CHECK_FUNCS(gettimeofday times ctime_r)

AC_CONFIG_FILES([
Makefile
doc/Doxyfile
lemon/lemon.pc
])

AC_OUTPUT

echo
echo '****************************** SUMMARY ******************************'
echo
echo Package version............... : $PACKAGE-$VERSION
echo
echo C++ compiler.................. : $CXX
echo C++ compiles flags............ : $CXXFLAGS
echo
echo GLPK support.................. : $lx_glpk_found
echo CPLEX support................. : $lx_cplex_found
echo SOPLEX support................ : $lx_soplex_found
echo
echo build benchmarks.............. : $enable_benchmark
echo build demo programs........... : $enable_demo
echo build additional tools........ : $enable_tools
echo
echo The packace will be installed in
echo -n '  '
echo $prefix.
echo
echo The documentation will be installed in
echo -n '  '
eval echo ${datadir}/doc/$PACKAGE.
echo
echo '*********************************************************************'

echo
echo configure complete, now type \'make\' and then \'make install\'.
echo
