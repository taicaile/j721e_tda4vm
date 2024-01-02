# File: dss_component.mk
#       This file is component include make file of DSS driver library.
# List of variables set in this file and their purpose:
# <mod>_RELPATH        - This is the relative path of the module, typically from
#                        top-level directory of the package
# <mod>_PATH           - This is the absolute path of the module. It derives from
#                        absolute path of the top-level directory (set in env.mk)
#                        and relative path set above
# <mod>_INCLUDE        - This is the path that has interface header files of the
#                        module. This can be multiple directories (space separated)
# <mod>_PKG_LIST       - Names of the modules (and sub-modules) that are a part
#                        part of this module, including itself.
# <mod>_BOARD_DEPENDENCY - "yes": means the code for this module depends on
#                             platform and the compiled obj/lib has to be kept
#                             under <platform> directory
#                             "no" or "" or if this variable is not defined: means
#                             this module has no platform dependent code and hence
#                             the obj/libs are not kept under <platform> dir.
# <mod>_CORE_DEPENDENCY     - "yes": means the code for this module depends on
#                             core and the compiled obj/lib has to be kept
#                             under <core> directory
#                             "no" or "" or if this variable is not defined: means
#                             this module has no core dependent code and hence
#                             the obj/libs are not kept under <core> dir.
# <mod>_APP_STAGE_FILES     - List of source files that belongs to the module
#                             <mod>, but that needs to be compiled at application
#                             build stage (in the context of the app). This is
#                             primarily for link time configurations or if the
#                             source file is dependent on options/defines that are
#                             application dependent. This can be left blank or
#                             not defined at all, in which case, it means there
#                             no source files in the module <mod> that are required
#                             to be compiled in the application build stage.
#
ifeq ($(dss_component_make_include), )

drvdss_SOCLIST         = am65xx j721e j721s2 j784s4
drvdss_BOARDLIST       = am65xx_evm j721e_evm j721s2_evm j784s4_evm
drvdss_am65xx_CORELIST = mpu1_0
drvdss_j721e_CORELIST  = mcu2_0
drvdss_j721s2_CORELIST  = mcu2_0
drvdss_j784s4_CORELIST  = mcu2_0
drvdssm2m_SOCLIST         = j721e j721s2 j784s4
drvdssm2m_BOARDLIST       = j721e_evm j721s2_evm j784s4_evm
drvdssm2m_j721e_CORELIST  = mcu2_0
drvdssm2m_j721s2_CORELIST = mcu2_0
drvdssm2m_j784s4_CORELIST = mcu2_0
drvdss_RTOS_LIST       = $(DEFAULT_RTOS_LIST)

define DRV_DSS_RTOS_BOARDLIST_RULE

drvdss_$(1)_BOARDLIST = $(filter $(DEFAULT_BOARDLIST_$(1)), $(drvdss_BOARDLIST))

endef

DRV_DSS_RTOS_BOARDLIST_MACRO_LIST := $(foreach curos, $(drvdss_RTOS_LIST), $(call DRV_DSS_RTOS_BOARDLIST_RULE,$(curos)))

$(eval ${DRV_DSS_RTOS_BOARDLIST_MACRO_LIST})

############################
# DSS package
# List of components included under DSS lib
# The components included here are built and will be part of DSS lib
############################
dss_LIB_LIST =

############################
# DSS app lib package
# List of components included under DSS app lib
# The components included here are built and will be part of DSS app lib
############################
dss_APP_LIB_LIST =

############################
# DSS examples
# List of examples under DSS (+= is used at each example definition)
# All the tests mentioned in list are built when test target is called
# List below all examples for allowed values
############################
dss_EXAMPLE_LIST =

#
# DSS Modules
#

# DSS library
dss_COMP_LIST = dss
dss_RELPATH = ti/drv/dss
dss_PATH = $(PDK_DSS_COMP_PATH)
dss_LIBNAME = dss
dss_LIBPATH = $(PDK_DSS_COMP_PATH)/lib
dss_MAKEFILE = -fsrc/makefile
export dss_MAKEFILE
export dss_LIBNAME
export dss_LIBPATH
dss_BOARD_DEPENDENCY = no
dss_CORE_DEPENDENCY = yes
export dss_COMP_LIST
export dss_BOARD_DEPENDENCY
export dss_CORE_DEPENDENCY
dss_PKG_LIST = dss
dss_INCLUDE = $(dss_PATH)
dss_SOCLIST = $(drvdss_SOCLIST)
export dss_SOCLIST
dss_$(SOC)_CORELIST = $(drvdss_$(SOC)_CORELIST)
export dss_$(SOC)_CORELIST
dss_LIB_LIST += dss

dss_app_utils_COMP_LIST = dss_app_utils
dss_app_utils_RELPATH = ti/drv/dss/examples/utils
dss_app_utils_PATH = $(PDK_DSS_COMP_PATH)/examples/utils
dss_app_utils_LIBNAME = dss_app_utils
dss_app_utils_LIBPATH = $(PDK_DSS_COMP_PATH)/lib
dss_app_utils_OBJPATH = $(dss_app_utils_RELPATH)/app_utils_nonos
dss_app_utils_MAKEFILE = -fmakefile
export dss_app_utils_LIBNAME
export dss_app_utils_LIBPATH
export dss_app_utils_MAKEFILE
export dss_app_utils_OBJPATH
dss_app_utils_BOARD_DEPENDENCY = yes
dss_app_utils_CORE_DEPENDENCY = yes
export dss_app_utils_COMP_LIST
export dss_app_utils_BOARD_DEPENDENCY
export dss_app_utils_CORE_DEPENDENCY
dss_app_utils_PKG_LIST = dss_app_utils
dss_app_utils_INCLUDE = $(dss_app_utils_PATH)
dss_app_utils_SOCLIST = $(drvdss_SOCLIST)
export dss_app_utils_SOCLIST
dss_app_utils_$(SOC)_CORELIST = $(drvdss_$(SOC)_CORELIST)
export dss_app_utils_$(SOC)_CORELIST

dss_APP_LIB_LIST += dss_app_utils

#
# DSS Examples
#

# DSS colorbar test app
define DSS_COLORBAR_TESTAPP_RULE

export dss_colorbar_testapp_$(1)_COMP_LIST = dss_colorbar_testapp_$(1)
dss_colorbar_testapp_$(1)_RELPATH = ti/drv/dss/examples/dss_colorbar_test
dss_colorbar_testapp_$(1)_PATH = $(PDK_DSS_COMP_PATH)/examples/dss_colorbar_test
export dss_colorbar_testapp_$(1)_BOARD_DEPENDENCY = yes
export dss_colorbar_testapp_$(1)_CORE_DEPENDENCY = yes
export dss_colorbar_testapp_$(1)_XDC_CONFIGURO = $(if $(findstring tirtos, $(1)), yes, no)
export dss_colorbar_testapp_$(1)_MAKEFILE = -f makefile BUILD_OS_TYPE=$(1)
dss_colorbar_testapp_$(1)_PKG_LIST = dss_colorbar_testapp_$(1)
dss_colorbar_testapp_$(1)_INCLUDE = $(dss_colorbar_testapp_$(1)_PATH)
export dss_colorbar_testapp_$(1)_BOARDLIST = $(filter $(DEFAULT_BOARDLIST_$(1)), $(drvdss_BOARDLIST) )
export dss_colorbar_testapp_$(1)_$(SOC)_CORELIST = $(filter $(DEFAULT_$(SOC)_CORELIST_$(1)), $(drvdss_$(SOC)_CORELIST))
export dss_colorbar_testapp_$(1)_SBL_APPIMAGEGEN = yes
ifneq ($(1),$(filter $(1), safertos))
dss_EXAMPLE_LIST += dss_colorbar_testapp_$(1)
else
ifneq ($(wildcard $(SAFERTOS_KERNEL_INSTALL_PATH)),)
dss_EXAMPLE_LIST += dss_colorbar_testapp_$(1)
endif
endif
endef

DSS_COLORBAR_TESTAPP_MACRO_LIST := $(foreach curos, $(drvdss_RTOS_LIST), $(call DSS_COLORBAR_TESTAPP_RULE,$(curos)))

$(eval ${DSS_COLORBAR_TESTAPP_MACRO_LIST})

# DSS display test app
define DSS_DISPLAY_TESTAPP_RULE

export dss_display_testapp_$(1)_COMP_LIST = dss_display_testapp_$(1)
dss_display_testapp_$(1)_RELPATH = ti/drv/dss/examples/dss_display_test
dss_display_testapp_$(1)_PATH = $(PDK_DSS_COMP_PATH)/examples/dss_display_test
export dss_display_testapp_$(1)_BOARD_DEPENDENCY = yes
export dss_display_testapp_$(1)_CORE_DEPENDENCY = yes
export dss_display_testapp_$(1)_XDC_CONFIGURO = $(if $(findstring tirtos, $(1)), yes, no)
export dss_display_testapp_$(1)_MAKEFILE = -f makefile BUILD_OS_TYPE=$(1)
dss_display_testapp_$(1)_PKG_LIST = dss_display_testapp_$(1)
dss_display_testapp_$(1)_INCLUDE = $(dss_display_testapp_$(1)_PATH)
export dss_display_testapp_$(1)_BOARDLIST = $(filter $(DEFAULT_BOARDLIST_$(1)), $(drvdss_BOARDLIST) )
export dss_display_testapp_$(1)_$(SOC)_CORELIST = $(filter $(DEFAULT_$(SOC)_CORELIST_$(1)), $(drvdss_$(SOC)_CORELIST))
export dss_display_testapp_$(1)_SBL_APPIMAGEGEN = yes
ifneq ($(1),$(filter $(1), safertos))
dss_EXAMPLE_LIST += dss_display_testapp_$(1)
else
ifneq ($(wildcard $(SAFERTOS_KERNEL_INSTALL_PATH)),)
dss_EXAMPLE_LIST += dss_display_testapp_$(1)
endif
endif

endef

DSS_DISPLAY_TESTAPP_MACRO_LIST := $(foreach curos, $(drvdss_RTOS_LIST), $(call DSS_DISPLAY_TESTAPP_RULE,$(curos)))

$(eval ${DSS_DISPLAY_TESTAPP_MACRO_LIST})

# DSS display baremetal test app
dss_baremetal_display_testapp_COMP_LIST = dss_baremetal_display_testapp
dss_baremetal_display_testapp_RELPATH = ti/drv/dss/examples/dss_display_test
dss_baremetal_display_testapp_PATH = $(PDK_DSS_COMP_PATH)/examples/dss_display_test
dss_baremetal_display_testapp_MAKEFILE = -fmakefile BUILD_OS_TYPE=baremetal
export dss_baremetal_display_testapp_MAKEFILE
dss_baremetal_display_testapp_BOARD_DEPENDENCY = yes
dss_baremetal_display_testapp_CORE_DEPENDENCY = yes
export dss_baremetal_display_testapp_COMP_LIST
export dss_baremetal_display_testapp_BOARD_DEPENDENCY
export dss_baremetal_display_testapp_CORE_DEPENDENCY
dss_baremetal_display_testapp_PKG_LIST = dss_baremetal_display_testapp
dss_baremetal_display_testapp_INCLUDE = $(dss_baremetal_display_testapp_PATH)
dss_baremetal_display_testapp_BOARDLIST = $(drvdss_BOARDLIST)
export dss_baremetal_display_testapp_BOARDLIST
dss_baremetal_display_testapp_$(SOC)_CORELIST = $(drvdss_$(SOC)_CORELIST)
export dss_baremetal_display_testapp_$(SOC)_CORELIST
dss_EXAMPLE_LIST += dss_baremetal_display_testapp
ifeq ($(SOC),$(filter $(SOC), am65xx j721e j721s2 j784s4))
dss_baremetal_display_testapp_SBL_APPIMAGEGEN = yes
export dss_baremetal_display_testapp_SBL_APPIMAGEGEN
endif

# DSS display M2M test app
define DSS_M2M_TESTAPP_RULE

export dss_m2m_testapp_$(1)_COMP_LIST = dss_m2m_testapp_$(1)
dss_m2m_testapp_$(1)_RELPATH = ti/drv/dss/examples/dss_m2m_test
dss_m2m_testapp_$(1)_PATH = $(PDK_DSS_COMP_PATH)/examples/dss_m2m_test
export dss_m2m_testapp_$(1)_BOARD_DEPENDENCY = yes
export dss_m2m_testapp_$(1)_CORE_DEPENDENCY = yes
export dss_m2m_testapp_$(1)_XDC_CONFIGURO = $(if $(findstring tirtos, $(1)), yes, no)
export dss_m2m_testapp_$(1)_MAKEFILE = -f makefile BUILD_OS_TYPE=$(1)
dss_m2m_testapp_$(1)_PKG_LIST = dss_m2m_testapp_$(1)
dss_m2m_testapp_$(1)_INCLUDE = $(dss_m2m_testapp_$(1)_PATH)
export dss_m2m_testapp_$(1)_BOARDLIST = $(filter $(DEFAULT_BOARDLIST_$(1)), $(drvdssm2m_BOARDLIST) )
export dss_m2m_testapp_$(1)_$(SOC)_CORELIST = $(filter $(DEFAULT_$(SOC)_CORELIST_$(1)), $(drvdssm2m_$(SOC)_CORELIST))
export dss_m2m_testapp_$(1)_SBL_APPIMAGEGEN = yes
ifneq ($(1),$(filter $(1), safertos))
dss_EXAMPLE_LIST += dss_m2m_testapp_$(1)
else
ifneq ($(wildcard $(SAFERTOS_KERNEL_INSTALL_PATH)),)
dss_EXAMPLE_LIST += dss_m2m_testapp_$(1)
endif
endif

endef

DSS_M2M_TESTAPP_MACRO_LIST := $(foreach curos, $(drvdss_RTOS_LIST), $(call DSS_M2M_TESTAPP_RULE,$(curos)))

$(eval ${DSS_M2M_TESTAPP_MACRO_LIST})

export dss_LIB_LIST
export dss_APP_LIB_LIST
export dss_EXAMPLE_LIST
export drvdss_LIB_LIST = $(dss_LIB_LIST)
export drvdss_APP_LIB_LIST = $(dss_APP_LIB_LIST)
export drvdss_EXAMPLE_LIST = $(dss_EXAMPLE_LIST)

DSS_CFLAGS = $(FVID2_CFLAGS)

export DSS_CFLAGS

dss_component_make_include := 1
endif
