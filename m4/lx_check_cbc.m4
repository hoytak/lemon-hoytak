AC_DEFUN([LX_CHECK_CBC],
[
  AC_ARG_WITH([cbc],
AS_HELP_STRING([--with-cbc@<:@=PREFIX@:>@], [search for CBC under PREFIX or under the default search paths if PREFIX is not given @<:@default@:>@])
AS_HELP_STRING([--without-cbc], [disable checking for CBC]),
              [], [with_cbc=yes])

  AC_ARG_WITH([cbc-includedir],
AS_HELP_STRING([--with-cbc-includedir=DIR], [search for CBC headers in DIR]),
              [], [with_cbc_includedir=no])

  AC_ARG_WITH([cbc-libdir],
AS_HELP_STRING([--with-cbc-libdir=DIR], [search for CBC libraries in DIR]),
              [], [with_cbc_libdir=no])

  lx_cbc_found=no
  if test x"$with_cbc" != x"no"; then
    AC_MSG_CHECKING([for CBC])

    if test x"$with_cbc_includedir" != x"no"; then
      CBC_CXXFLAGS="-I$with_cbc_includedir"
    elif test x"$with_cbc" != x"yes"; then
      CBC_CXXFLAGS="-I$with_cbc/include"
    fi

    if test x"$with_cbc_libdir" != x"no"; then
      CBC_LDFLAGS="-L$with_cbc_libdir"
    elif test x"$with_cbc" != x"yes"; then
      CBC_LDFLAGS="-L$with_cbc/lib"
    fi
    CBC_LIBS="-lOsi -lCbc -lOsiCbc -lCbcSolver -lClp -lOsiClp -lCoinUtils -lVol -lOsiVol -lCgl -lm -llapack -lblas"

    lx_save_cxxflags="$CXXFLAGS"
    lx_save_ldflags="$LDFLAGS"
    lx_save_libs="$LIBS"
    CXXFLAGS="$CBC_CXXFLAGS"
    LDFLAGS="$CBC_LDFLAGS"
    LIBS="$CBC_LIBS"

    lx_cbc_test_prog='
      #include <coin/CbcModel.hpp>

      int main(int argc, char** argv)
      {
        CbcModel cbc;
        return 0;
      }'

    AC_LANG_PUSH(C++)
    AC_LINK_IFELSE([$lx_cbc_test_prog], [lx_cbc_found=yes], [lx_cbc_found=no])
    AC_LANG_POP(C++)

    CXXFLAGS="$lx_save_cxxflags"
    LDFLAGS="$lx_save_ldflags"
    LIBS="$lx_save_libs"

    if test x"$lx_cbc_found" = x"yes"; then
      AC_DEFINE([HAVE_CBC], [1], [Define to 1 if you have CBC.])
      lx_lp_found=yes
      AC_DEFINE([HAVE_LP], [1], [Define to 1 if you have any LP solver.])
      AC_MSG_RESULT([yes])
    else
      CBC_CXXFLAGS=""
      CBC_LDFLAGS=""
      CBC_LIBS=""
      AC_MSG_RESULT([no])
    fi
  fi
  CBC_LIBS="$CBC_LDFLAGS $CBC_LIBS"
  AC_SUBST(CBC_CXXFLAGS)
  AC_SUBST(CBC_LIBS)
  AM_CONDITIONAL([HAVE_CBC], [test x"$lx_cbc_found" = x"yes"])
])
