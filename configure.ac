AC_PREREQ([2.61])
AC_INIT([libfirmware], [0.1], [mkschreder.uk@gmail.com])
AC_CONFIG_AUX_DIR(aux-dist)
AM_INIT_AUTOMAKE

AC_PROG_MAKE_SET
AC_PROG_CC
AC_PROG_CXX
AC_LANG(C)

AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_MACRO_DIR([ultim84])

AX_CHECK_STDC
AX_SUPERWARN

AC_COMPILE_CHECK(typeof, [int x; typeof(x) y;], [AC_DEFINE([HAVE_TYPEOF], [1])])

AM_CONDITIONAL([XC16_GCC], [test "x$host" = "xxc16"])
AM_COND_IF([XC16_GCC],
	[AC_PREFIX_DEFAULT($(dirname $(dirname `which xc16-gcc`)))],
	[
		AC_ARG_ENABLE(coverage, Enable coverage reporting, [AX_COVERAGE], [])
		AC_CONFIG_FILES([tests/Makefile])
	])

LT_PREREQ([2.2])
LT_INIT([dlopen shared])
PKG_PROG_PKG_CONFIG([0.25])

AC_ARG_WITH([target],
    [AC_HELP_STRING([--with-target],
        [specify which board we are compiling for])],
    [],
    [AC_MSG_WARN("You should specify target for libfirmware! Use --with-target=<target> to set target!")])

AM_CONDITIONAL([WITH_TARGET_LINUX], [test x$with_target = xlinux])
AM_COND_IF([WITH_TARGET_LINUX], (AC_DEFINE([WITH_TARGET_LINUX], [1], [Build for linux system])))

AM_CONDITIONAL([WITH_TARGET_STM32F429_439XX], [test x$with_target = xstm32f429_439xx])
AM_COND_IF([WITH_TARGET_STM32F429_439XX], (AC_DEFINE([WITH_TARGET_STM32F429_439XX], [1], [Build for stm32f429_439xx processor])))

AM_CONDITIONAL([WITH_TARGET_STM32F429_439XX], [test x$with_target = xstm32f429zet6])
AM_COND_IF([WITH_TARGET_STM32F429_439XX], (AC_DEFINE([WITH_TARGET_STM32F429_439XX], [1], [Build for stm32f429_439xx processor])))

AM_CONDITIONAL([WITH_TARGET_STM32F427], [test x$with_target = xstm32f427])
AM_COND_IF([WITH_TARGET_STM32F427], (AC_DEFINE([WITH_TARGET_STM32F427], [1], [Build for stm32f427 processor])))

AM_CONDITIONAL([WITH_TARGET_STM32F40_41XXX], [test x$with_target = xstm32f40_41xxx])
AM_COND_IF([WITH_TARGET_STM32F40_41XXX], (AC_DEFINE([WITH_TARGET_STM32F40_41XXX], [1], [Build for stm32f40_41xxx processor])))

AM_CONDITIONAL([WITH_TARGET_STM32F10X], [test x$with_target = xstm32f10x])
AM_COND_IF([WITH_TARGET_STM32F10X], (AC_DEFINE([WITH_TARGET_STM32F10X], [1], [Build for stm32f10x processor])))

AC_CONFIG_FILES([Makefile include/Makefile src/Makefile libfirmware.pc])

AC_OUTPUT

AX_PRINT_INFO
