
ifeq ($(SOC),$(filter $(SOC), am65xx j721e))
PACKAGE_SRCS_COMMON += cslr_dru.h csl_dru.h src/ip/dru/src_files_dru.mk src/ip/dru/V0
SRCDIR += src/ip/dru/V0/priv
INCDIR += src/ip/dru/V0
SRCS_COMMON += csl_dru.c
endif

ifeq ($(SOC),$(filter $(SOC), j721s2 j784s4))
PACKAGE_SRCS_COMMON += cslr_dru.h csl_dru.h src/ip/dru/src_files_dru.mk src/ip/dru/V0/priv src/ip/dru/V1
SRCDIR += src/ip/dru/V0/priv
INCDIR += src/ip/dru/V1
SRCS_COMMON += csl_dru.c
endif

ifeq ($(SOC),$(filter $(SOC), am62a am62px))
PACKAGE_SRCS_COMMON += cslr_dru.h csl_dru.h src/ip/dru/src_files_dru.mk src/ip/dru/V0/priv src/ip/dru/V2 src/ip/dru/V0
SRCDIR += src/ip/dru/V0/priv
INCDIR += src/ip/dru/V2
SRCS_COMMON += csl_dru.c
endif
