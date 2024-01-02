#
# This file is the makefile for building IPC example app
#
SRCDIR += . $(IPC_COMMON_PATH)/src
INCDIR +=

# List all the external components/interfaces, whose interface header files
#  need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk

# Common source files and CFLAGS across all platforms and cores
PACKAGE_SRCS_COMMON += $(IPC_COMMON_PATH)/src $(IPC_COMMON_PATH)/$(SOC) $(IPC_COMMON_PATH)/makefile.mk

# List all the components required by the application
ifeq ($(BUILD_OS_TYPE), baremetal)
  COMP_LIST_COMMON =  $(PDK_COMMON_BAREMETAL_COMP)
  SRCS_COMMON += main_baremetal.c ipc_testsetup_baremetal.c
  SRCS_COMMON += ipc_trace.c
  COMP_LIST_COMMON += ipc_baremetal
  CFLAGS_LOCAL_COMMON += -DIPC_EXCLUDE_CTRL_TASKS -DBAREMETAL
  ifeq ($(ISA),$(filter $(ISA), a53 a72))
    LNKFLAGS_LOCAL_$(CORE) += --entry Entry
  endif
  ifeq ($(ISA), r5f)
	  SRCS_COMMON += r5f_mpu_$(SOC)_default.c
  endif
  ifeq ($(SOC),$(filter $(SOC), j721e j7200 j721s2 j784s4))
    EXTERNAL_LNKCMD_FILE_LOCAL = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/$(SOC)/linker_$(ISA)_$(CORE).lds
    ifeq ($(ECHO_TEST_BTCM), 1)
      ifeq ($(ISA), r5f)
        ifeq ($(CORE),mcu1_0)
	        EXTERNAL_LNKCMD_FILE_LOCAL = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/$(SOC)/linker_$(ISA)_$(CORE)_btcm.lds
        endif
      endif
    endif
  endif
endif

ifeq ($(BUILD_OS_TYPE), freertos)
  COMP_LIST_COMMON =  $(PDK_COMMON_FREERTOS_COMP)
  COMP_LIST_COMMON += ipc
  ifeq ($(SOC),$(filter $(SOC), j721e j7200 j721s2 j784s4))
    ifeq ($(CORE),mcu1_0)
      COMP_LIST_COMMON += sciserver_tirtos
    endif
  endif
  SRCS_COMMON += main_rtos.c ipc_testsetup.c
  SRCS_COMMON += ipc_trace.c
  CFLAGS_LOCAL_COMMON += -DFREERTOS
  INCLUDE_EXTERNAL_INTERFACES += freertos
  ifeq ($(ISA), r5f)
	  SRCS_COMMON += r5f_mpu_$(SOC)_default.c
  endif
  ifeq ($(ISA), c66)
    INCDIR += $(IPC_COMMON_PATH)/$(SOC)/$(BUILD_OS_TYPE)/
    SRCS_COMMON += c66_cache_mar.c
  endif
  ifeq ($(ISA), c7x)
    INCDIR += $(IPC_COMMON_PATH)/$(SOC)/$(BUILD_OS_TYPE)/
    SRCS_COMMON += c7x_mmu.c
  endif
  EXTERNAL_LNKCMD_FILE_LOCAL = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/$(SOC)/$(BUILD_OS_TYPE)/linker_$(ISA)_$(CORE)_$(BUILD_OS_TYPE).lds
  APPEND_LNKCMD_FILE += $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/$(SOC)/$(BUILD_OS_TYPE)/memory_map_ddr.cmd
  ifeq ($(ECHO_TEST_BTCM), 1)
    ifeq ($(ISA), r5f)
      EXTERNAL_LNKCMD_FILE_LOCAL = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/$(SOC)/$(BUILD_OS_TYPE)/linker_$(ISA)_$(CORE)_btcm_$(BUILD_OS_TYPE).lds
    endif
  endif
endif

ifeq ($(BUILD_OS_TYPE), safertos)
  COMP_LIST_COMMON =  $(PDK_COMMON_SAFERTOS_COMP)
  COMP_LIST_COMMON += ipc
  ifeq ($(SOC),$(filter $(SOC), j721e j7200 j721s2 j784s4))
    ifeq ($(CORE),mcu1_0)
      COMP_LIST_COMMON += sciserver_tirtos
    endif
  endif
  SRCS_COMMON += main_rtos.c ipc_testsetup.c
  SRCS_COMMON += ipc_trace.c
  CFLAGS_LOCAL_COMMON += -DSAFERTOS
  INCLUDE_EXTERNAL_INTERFACES += safertos
  ifeq ($(ISA), r5f)
	  SRCS_COMMON += r5f_mpu_$(SOC)_safertos.c
  endif
  ifeq ($(ISA), c66)
    INCDIR += $(IPC_COMMON_PATH)/$(SOC)/$(BUILD_OS_TYPE)/
    SRCS_COMMON += c66_cache_mar.c
  endif
  ifeq ($(ISA), c7x)
    INCDIR += $(IPC_COMMON_PATH)/$(SOC)/$(BUILD_OS_TYPE)/
    SRCS_COMMON += c7x_mmu.c
  endif
  EXTERNAL_LNKCMD_FILE_LOCAL = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/$(SOC)/$(BUILD_OS_TYPE)/linker_$(ISA)_$(CORE)_$(BUILD_OS_TYPE).lds
  APPEND_LNKCMD_FILE += $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/$(SOC)/$(BUILD_OS_TYPE)/memory_map_ddr.cmd
  ifeq ($(ECHO_TEST_BTCM), 1)
    ifeq ($(ISA), r5f)
      EXTERNAL_LNKCMD_FILE_LOCAL = $(PDK_INSTALL_PATH)/ti/drv/ipc/examples/common/$(SOC)/$(BUILD_OS_TYPE)/linker_$(ISA)_$(CORE)_btcm_$(BUILD_OS_TYPE).lds
    endif
  endif
endif

CFLAGS_LOCAL_COMMON += $(PDK_CFLAGS)

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

# OBJs and libraries are built by using rule defined in rules_<target>.mk
#     and need not be explicitly specified here

# Nothing beyond this point
