
ifeq ($(SOC),$(filter $(SOC), am65xx j7200 j721e j721s2 j784s4 am64x am62x am62a am62px))
PACKAGE_SRCS_COMMON += cslr_fss.h csl_fss.h src/ip/fss/src_files_fss.mk src/ip/fss/V0/csl_fss.h src/ip/fss/V0/priv
SRCDIR += src/ip/fss/V0/priv
SRCS_COMMON += csl_fss.c

ifeq ($(SOC),$(filter $(SOC), am65xx j7200 j721e j721s2 j784s4))
PACKAGE_SRCS_COMMON += src/ip/fss/V0/V0_1
INCDIR += src/ip/fss/V0/V0_1
endif

ifeq ($(SOC),$(filter $(SOC), am64x am62x am62a am62px))
PACKAGE_SRCS_COMMON += src/ip/fss/V0/V0_2
INCDIR += src/ip/fss/V0/V0_2
endif

endif
