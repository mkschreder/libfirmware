AC_PREREQ([2.61])
AC_INIT([libfirmware], [0.1], [mkschreder.uk@gmail.com])
AC_CONFIG_AUX_DIR(aux-dist)
AM_INIT_AUTOMAKE

AC_PROG_MAKE_SET
AC_LANG(C)

AC_CONFIG_MACRO_DIR([m4])
PKG_PROG_PKG_CONFIG([0.25])

LT_PREREQ([2.2])
LT_INIT([dlopen shared])

AC_ARG_WITH([target],
    [AC_HELP_STRING([--with-target],
        [specify which board we are compiling for])],
    [],
    [AC_MSG_ERROR("\nYou must specify target for libfirmware!\n    Use --with-target=<target> to set target!\n")])

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
