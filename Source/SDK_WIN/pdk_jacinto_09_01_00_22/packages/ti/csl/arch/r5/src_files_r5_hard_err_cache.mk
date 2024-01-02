
ifeq ($(SOC),$(filter $(SOC), am65xx j7200 j721e j721s2 j784s4 am77x tpr12 awr294x am263x am64x am62x am62a))
  PACKAGE_SRCS_COMMON += arch/r5/src/csl_arm_r5_hard_err_cache.asm
  PACKAGE_SRCS_COMMON += arch/r5/csl_arm_r5_hard_err_cache.h
  PACKAGE_SRCS_COMMON += arch/r5/src_files_r5_hard_err_cache.mk
  ifeq ($(ARCH),armv7r)
    SRCDIR += ./arch/r5/src
    INCDIR += ./arch/r5
    SRCS_ASM_COMMON += csl_arm_r5_hard_err_cache.asm
  endif

endif
