AC_DEFUN([LX_CHECK_CLP],
[
  AC_ARG_WITH([clp],
AS_HELP_STRING([--with-clp@<:@=PREFIX@:>@], [search for CLP under PREFIX or under the default search paths if PREFIX is not given @<:@default@:>@])
AS_HELP_STRING([--without-clp], [disable checking for CLP]),
              [], [with_clp=yes])

  AC_ARG_WITH([clp-includedir],
AS_HELP_STRING([--with-clp-includedir=DIR], [search for CLP headers in DIR]),
              [], [with_clp_includedir=no])

  AC_ARG_WITH([clp-libdir],
AS_HELP_STRING([--with-clp-libdir=DIR], [search for CLP libraries in DIR]),
              [], [with_clp_libdir=no])

  lx_clp_found=no
  if test x"$with_clp" != x"no"; then
    AC_MSG_CHECKING([for CLP])

    if test x"$with_clp_includedir" != x"no"; then
      CLP_CXXFLAGS="-I$with_clp_includedir"
    elif test x"$with_clp" != x"yes"; then
      CLP_CXXFLAGS="-I$with_clp/include"
    fi

    if test x"$with_clp_libdir" != x"no"; then
      CLP_LDFLAGS="-L$with_clp_libdir"
    elif test x"$with_clp" != x"yes"; then
      CLP_LDFLAGS="-L$with_clp/lib"
    fi
    CLP_LIBS="-lClp -lCoinUtils -lm"

    lx_save_cxxflags="$CXXFLAGS"
    lx_save_ldflags="$LDFLAGS"
    lx_save_libs="$LIBS"
    CXXFLAGS="$CLP_CXXFLAGS"
    LDFLAGS="$CLP_LDFLAGS"
    LIBS="$CLP_LIBS"

    lx_clp_test_prog='
      #include <coin/ClpModel.hpp>

      int main(int argc, char** argv)
      {
        ClpModel clp;
        return 0;
      }'

    AC_LANG_PUSH(C++)
    AC_LINK_IFELSE([$lx_clp_test_prog], [lx_clp_found=yes], [lx_clp_found=no])
    AC_LANG_POP(C++)

    CXXFLAGS="$lx_save_cxxflags"
    LDFLAGS="$lx_save_ldflags"
    LIBS="$lx_save_libs"

    if test x"$lx_clp_found" = x"yes"; then
      AC_DEFINE([HAVE_CLP], [1], [Define to 1 if you have CLP.])
      lx_lp_found=yes
      AC_DEFINE([HAVE_LP], [1], [Define to 1 if you have any LP solver.])
      AC_MSG_RESULT([yes])
    else
      CLP_CXXFLAGS=""
      CLP_LDFLAGS=""
      CLP_LIBS=""
      AC_MSG_RESULT([no])
    fi
  fi
  CLP_LIBS="$CLP_LDFLAGS $CLP_LIBS"
  AC_SUBST(CLP_CXXFLAGS)
  AC_SUBST(CLP_LIBS)
  AM_CONDITIONAL([HAVE_CLP], [test x"$lx_clp_found" = x"yes"])
])
