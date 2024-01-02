#
# This file is common makefile for building VHWA DOF test app for both TI-RTOS/baremetal
#
SRCDIR = .
SRCDIR += ../common
INCDIR =

# List all the external components/interfaces, whose interface header files
#  need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk

# List all the components required by the application
COMP_LIST_COMMON = fvid2 vhwa
SRCS_COMMON += vhwa_examples_common.c vhwa_dof_api.c vhwa_common_crc.c

ifeq ($(BUILD_OS_TYPE), baremetal)
  COMP_LIST_COMMON += $(PDK_COMMON_BAREMETAL_COMP)
  SRCS_COMMON += main_baremetal.c
  EXTERNAL_LNKCMD_FILE_LOCAL = ../common/$(SOC)/vhwa_r5.cmd
endif

ifeq ($(BUILD_OS_TYPE),freertos)
  CFLAGS_OS_DEFINES = -DFREERTOS
  INCLUDE_EXTERNAL_INTERFACES += freertos
  COMP_LIST_COMMON += $(PDK_COMMON_FREERTOS_COMP)
  SRCS_COMMON += main_rtos.c

  EXTERNAL_LNKCMD_FILE_LOCAL = ../common/$(SOC)/vhwa_freertos_r5.cmd
endif

ifeq ($(BUILD_OS_TYPE),safertos)
  CFLAGS_OS_DEFINES = -DSAFERTOS
  INCLUDE_EXTERNAL_INTERFACES += safertos
  COMP_LIST_COMMON += $(PDK_COMMON_SAFERTOS_COMP)
  SRCS_COMMON += main_rtos.c

  EXTERNAL_LNKCMD_FILE_LOCAL = ../common/vhwa_safertos_r5.cmd
endif

# Common source files and CFLAGS across all platforms and cores
PACKAGE_SRCS_COMMON = . ../include ../common

LNKFLAGS_LOCAL_mpu1_0 += --entry Entry
CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(CFLAGS_GLOBAL_$(BOARD)) $(CFLAGS_OS_DEFINES)

# Core/SoC/platform specific source files and CFLAGS
# Example:
#   SRCS_<core/SoC/platform-name> =
#   CFLAGS_LOCAL_<core/SoC/platform-name> =

# Include common make files
ifeq ($(MAKERULEDIR), )
#Makerule path not defined, define this and assume relative path from ROOTDIR
  MAKERULEDIR := $(ROOTDIR)/ti/build/makerules
  export MAKERULEDIR
endif
include $(MAKERULEDIR)/common.mk

LIB_LIST_VHWA = $(vhwa_LIBPATH)/$(SOC)/$(CORE)/$(BUILD_PROFILE)/vhwa.aer5f
LNK_LIBS += $(LIB_LIST_VHWA)
# OBJs and libraries are built by using rule defined in rules_<target>.mk
#     and need not be explicitly specified here

# Nothing beyond this point
