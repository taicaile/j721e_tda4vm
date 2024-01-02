#
# This file is the makefile for building sbl library.
#
include $(PDK_INSTALL_PATH)/ti/build/Rules.make

ifeq ($(SBL_USE_DMA),no)
  ifneq ($(BOOTMODE), cust)
    ifeq ($(HLOS_BOOT),yes)
      BASE_NAME = sbl_lib_$(BOOTMODE)_nondma_hlos
    else
      BASE_NAME = sbl_lib_$(BOOTMODE)_nondma
    endif
  else
    BASE_NAME = sbl_lib_$(BOOTMODE)
  endif
else
  ifeq ($(HLOS_BOOT),yes)
    BASE_NAME = sbl_lib_$(BOOTMODE)_hlos
  else
    BASE_NAME = sbl_lib_$(BOOTMODE)
  endif
endif

HS_SUFFIX=
ifeq ($(BUILD_HS),yes)
HS_SUFFIX=_hs
endif

MODULE_NAME = $(BASE_NAME)$(HS_SUFFIX)

INCDIR	+= $(PDK_INSTALL_PATH)
INCDIR	+= $(PDK_INSTALL_PATH)/ti/drv/uart/soc/$(SOC)

INCDIR	+= $(PDK_SBL_COMP_PATH)
INCDIR	+= $(PDK_SBL_COMP_PATH)/soc
INCDIR	+= $(PDK_SBL_COMP_PATH)/board/src
INCDIR	+= $(PDK_SBL_COMP_PATH)/src/rprc
INCDIR	+= $(PDK_SBL_COMP_PATH)/src/ospi
INCDIR	+= $(PDK_SBL_COMP_PATH)/src/hyperflash
INCDIR	+= $(PDK_SBL_COMP_PATH)/src/mmcsd
INCDIR	+= $(PDK_SBL_COMP_PATH)/src/emmc
INCDIR	+= $(PDK_SBL_COMP_PATH)/src/uart
INCDIR	+= $(PDK_SBL_COMP_PATH)/soc/k3

SRCDIR	+= $(PDK_SBL_COMP_PATH)/board/src
SRCDIR	+= $(PDK_SBL_COMP_PATH)/src/rprc
SRCDIR	+=$(PDK_SBL_COMP_PATH)/src/ospi
SRCDIR	+=$(PDK_SBL_COMP_PATH)/src/hyperflash
SRCDIR	+=$(PDK_SBL_COMP_PATH)/src/mmcsd
SRCDIR	+=$(PDK_SBL_COMP_PATH)/src/emmc
SRCDIR	+=$(PDK_SBL_COMP_PATH)/src/uart
SRCDIR	+= $(PDK_SBL_COMP_PATH)/soc/k3
SRCDIR	+= $(PDK_INSTALL_PATH)/ti/drv/uart/soc/$(SOC)

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES =
INCLUDE_INTERNAL_INTERFACES =

CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(SBL_CFLAGS)
ifeq ($(SBL_USE_DMA),no)
CFLAGS_LOCAL_COMMON += -DSBL_USE_DMA=0
else
CFLAGS_LOCAL_COMMON += -DSBL_USE_DMA=1
endif

PACKAGE_SRCS_COMMON  = ./build ./src ./tools ./board/src ./board/sbl_common_osal.c
PACKAGE_SRCS_COMMON += ./soc/sbl_soc.h
PACKAGE_SRCS_COMMON += ./soc/k3 ./board/k3

PACKAGE_SRCS_COMMON += ./.gitignore ./sbl_component.mk ./makefile ./sbl_ver.h

# Common source files and CFLAGS across all platforms and cores
SRCS_COMMON = sbl_soc.c
SRCS_COMMON += sbl_rprc.c
SRCS_COMMON += sbl_slave_core_boot.c
SRCS_COMMON += UART_soc.c
SRCS_COMMON += sbl_sci_client.c
SRCS_COMMON += sbl_vid_map.c
ifeq ($(SBL_USE_DMA), yes)
SRCS_COMMON += sbl_dma.c
endif

SRCS_ASM_COMMON += sbl_misc.asm
SRCS_ASM_COMMON += sbl_init.asm

# scratch memory given to the SBL
# for image load and parsing
SBL_CFLAGS += -DSBL_SCRATCH_MEM_START=0xC2000000
SBL_CFLAGS += -DSBL_SCRATCH_MEM_SIZE=0x4000000

# Check for custom flags
ifeq ($(BOOTMODE), cust)
  SBL_CFLAGS = $(CUST_SBL_FLAGS)
endif # ifeq ($(BOOTMODE), cust)

# HLOS Boot flags
ifeq ($(HLOS_BOOT),yes)
  SBL_CFLAGS += -DSBL_ENABLE_HLOS_BOOT -DSBL_HLOS_OWNS_FLASH
endif

# HS Boot flags
ifeq ($(BUILD_HS), yes)
  SBL_CFLAGS += -DSBL_BUILD_HS -DBUILD_HS
endif

# BOOTMODE specific CFLAGS
ifeq ($(BOOTMODE), mmcsd)
  SBL_CFLAGS += -DBOOT_MMCSD
endif # ifeq ($(BOOTMODE), mmcsd)

ifeq ($(BOOTMODE), emmc)
  SBL_CFLAGS += -DBOOT_EMMC
endif # ifeq ($(BOOTMODE), emmc)

ifeq ($(BOOTMODE), ospi)
  SBL_CFLAGS += -DBOOT_OSPI
endif # ifeq ($(BOOTMODE), ospi)

ifeq ($(BOOTMODE), hyperflash)
  SBL_CFLAGS += -DBOOT_HYPERFLASH
endif # ifeq ($(BOOTMODE), hyperflash)

ifeq ($(BOOTMODE), uart)
  SBL_CFLAGS += -DBOOT_UART
endif # ifeq ($(BOOTMODE), uart)

ifeq ($(BOOTMODE), qspi)
  SBL_CFLAGS += -DBOOT_QSPI
endif # ifeq ($(BOOTMODE), qspi)

ifeq ($(filter $(SBL_CFLAGS), -DBOOT_MMCSD), -DBOOT_MMCSD)
  SRCS_COMMON += sbl_mmcsd.c
endif # ifeq ($(filter $(SBL_CFLAGS), -DBOOT_MMCSD), -DBOOT_MMCSD)

ifeq ($(filter $(SBL_CFLAGS), -DBOOT_EMMC), -DBOOT_EMMC)
  SRCS_COMMON += sbl_emmc.c
endif # ifeq ($(filter $(SBL_CFLAGS), -DBOOT_EMMC), -DBOOT_EMMC)

ifeq ($(filter $(SBL_CFLAGS), -DBOOT_OSPI), -DBOOT_OSPI)
  SRCS_COMMON += sbl_ospi.c
endif # ifeq ($(filter $(SBL_CFLAGS), -DBOOT_OSPI), -DBOOT_OSPI)

ifeq ($(filter $(SBL_CFLAGS), -DBOOT_HYPERFLASH), -DBOOT_HYPERFLASH)
  SRCS_COMMON += sbl_hyperflash.c
endif # ifeq ($(filter $(SBL_CFLAGS), -DBOOT_HYPERFLASH), -DBOOT_HYPERFLASH)

ifeq ($(filter $(SBL_CFLAGS), -DBOOT_UART), -DBOOT_UART)
  SRCS_COMMON += sbl_uart.c sbl_xmodem.c
endif # ifeq ($(filter $(SBL_CFLAGS), -DBOOT_UART), -DBOOT_UART)

ifeq ($(filter $(SBL_CFLAGS), -DBOOT_QSPI), -DBOOT_QSPI)
  SRCS_COMMON += sbl_qspi_boardflash.c
endif # ifeq ($(filter $(SBL_CFLAGS), -DBOOT_QSPI), -DBOOT_QSPI)

# Core/SoC/platform specific source files and CFLAGS
# Example:
#   SRCS_<core/SoC/platform-name> =
#   CFLAGS_LOCAL_<core/SoC/platform-name> =
SRCS_j721e += j721e_qos_data.c
SRCS_j721s2 += j721s2_qos_data.c
SRCS_j784s4 += j784s4_qos_data.c

# User needs to change below macro to boot app with size more than 500 KB.
# E.g. If app is x KB then give MAX_APP_SIZE_EMMC as (x * 1024) in hexadecimal
MAX_APP_SIZE_EMMC ?= 0x7D000
SBL_CFLAGS += -DMAX_APP_SIZE_EMMC=$(MAX_APP_SIZE_EMMC)

# Include common make files
ifeq ($(MAKERULEDIR), )
#Makerule path not defined, define this and assume relative path from ROOTDIR
  MAKERULEDIR := $(ROOTDIR)/ti/build/makerules
  export MAKERULEDIR
endif
include $(MAKERULEDIR)/common.mk

# OBJs and libraries are built by using rule defined in rules_<target>.mk
#     and need not be explicitly specified here

# Nothing beyond this point
