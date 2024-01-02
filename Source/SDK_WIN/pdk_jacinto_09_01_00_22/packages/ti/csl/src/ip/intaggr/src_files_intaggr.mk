
ifeq ($(SOC),$(filter $(SOC), am65xx j7200 j721e))
PACKAGE_SRCS_COMMON += cslr_intaggr.h csl_intaggr.h src/ip/intaggr/src_files_intaggr.mk src/ip/intaggr/V0
SRCDIR += src/ip/intaggr/V0/priv
INCDIR += src/ip/intaggr/V0/V0_1
SRCS_COMMON += csl_intaggr.c
endif

ifeq ($(SOC),$(filter $(SOC), am64x am62x am62a am62px j721s2 j784s4))
PACKAGE_SRCS_COMMON += cslr_intaggr.h csl_intaggr.h src/ip/intaggr/src_files_intaggr.mk src/ip/intaggr/V0
SRCDIR += src/ip/intaggr/V0/priv
INCDIR += src/ip/intaggr/V0/V0_2
SRCS_COMMON += csl_intaggr.c
endif
