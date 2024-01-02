# File: sciclient_component.mk
#       This file is component include make file.
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
ifeq ($(sciclient_component_make_include), )

drvsciclient_RTOS_LIST           = $(DEFAULT_RTOS_LIST)

############################
# sciclient package
# List of components included under sciclient
# The components included here are built and will be part of sciclient lib
############################
sciclient_LIB_LIST = sciclient
ifeq ($(SOC),$(filter $(SOC), am65xx j721e j7200 j721s2 j784s4))
sciclient_LIB_LIST += sciclient_hs
endif

ifeq ($(SOC),$(filter $(SOC), j721e j7200 j721s2 j784s4))
sciclient_LIB_LIST += rm_pm_hal
sciclient_LIB_LIST += sciserver_tirtos
sciclient_LIB_LIST += sciserver_baremetal
sciclient_LIB_LIST += sciclient_direct
sciclient_LIB_LIST += sciclient_direct_hs
endif

drvsciclient_BOARDLIST = am65xx_evm am65xx_idk j721e_sim j721e_evm j7200_evm j721s2_evm j784s4_evm am64x_evm
drvsciclient_SOCLIST = am65xx j721e j7200 j721s2 j784s4 am64x
drvsciclient_am65xx_CORELIST = mcu1_0 mcu1_1 mpu1_0
drvsciclient_j721e_CORELIST = mpu1_0 mcu1_0 mcu1_1 mcu2_0 mcu2_1 mcu3_0 mcu3_1 c66xdsp_1 c66xdsp_2 c7x_1 c7x-hostemu
drvsciclient_j7200_CORELIST = mpu1_0 mcu1_0 mcu1_1 mcu2_0 mcu2_1
drvsciclient_j721s2_CORELIST = mpu1_0 mcu1_0 mcu1_1 mcu2_0 mcu2_1 mcu3_0 mcu3_1 c7x_1 c7x_2 c7x-hostemu
drvsciclient_j784s4_CORELIST = mpu1_0 mcu1_0 mcu1_1 mcu2_0 mcu2_1 mcu3_0 mcu3_1 mcu4_0 mcu4_1 c7x_1 c7x_2 c7x_3 c7x_4 c7x-hostemu
drvsciclient_am64x_CORELIST = mpu1_0 mcu1_0 mcu1_1 mcu2_0 mcu2_1 m4f_0
drvsciclient_DISABLE_PARALLEL_MAKE = yes

define DRV_SCICLIENT_RTOS_BOARDLIST_RULE

drvsciclient_$(1)_BOARDLIST = $(filter $(DEFAULT_BOARDLIST_$(1)), $(drvsciclient_BOARDLIST))

endef

DRV_SCICLIENT_RTOS_BOARDLIST_MACRO_LIST := $(foreach curos, $(drvsciclient_RTOS_LIST), $(call DRV_SCICLIENT_RTOS_BOARDLIST_RULE,$(curos)))

$(eval ${DRV_SCICLIENT_RTOS_BOARDLIST_MACRO_LIST})

export sciclient_COMP_LIST = sciclient
export sciclient_RELPATH = ti/drv/sciclient
export sciclient_OBJPATH = ti/drv/sciclient
export sciclient_LIBNAME = sciclient
export sciclient_PATH = $(PDK_SCICLIENT_COMP_PATH)
export sciclient_LIBPATH = $(PDK_SCICLIENT_COMP_PATH)/lib
export sciclient_MAKEFILE = -fsrc/sciclient_indirect_makefile BUILD_HS=no
# Simulator versus Silicon has a different Firmware Image.
export sciclient_BOARD_DEPENDENCY = no
ifeq ($(BOARD),$(filter $(BOARD), j721e_ccqt j721e_loki j721e_hostemu j721s2_hostemu j784s4_hostemu))
export sciclient_BOARD_DEPENDENCY = yes
endif
export sciclient_CORE_DEPENDENCY = yes
export sciclient_PKG_LIST = sciclient
export sciclient_INCLUDE = $(sciclient_PATH)
export sciclient_SOCLIST = $(drvsciclient_SOCLIST)
export sciclient_BOARDLIST = $(drvsciclient_BOARDLIST)
export sciclient_$(SOC)_CORELIST = $(drvsciclient_$(SOC)_CORELIST)

export sciclient_hs_COMP_LIST = sciclient_hs
export sciclient_hs_RELPATH = ti/drv/sciclient
export sciclient_hs_OBJPATH = ti/drv/sciclient_hs
export sciclient_hs_PATH = $(PDK_SCICLIENT_COMP_PATH)
export sciclient_hs_LIBNAME = sciclient_hs
export sciclient_hs_LIBPATH = $(PDK_SCICLIENT_COMP_PATH)/lib
export sciclient_hs_MAKEFILE = -fsrc/sciclient_indirect_makefile BUILD_HS=yes
# Simulator versus Silicon has a different Firmware Image.
export sciclient_hs_BOARD_DEPENDENCY = no
ifeq ($(BOARD),$(filter $(BOARD), j721e_ccqt j721e_loki j721e_hostemu j721s2_hostemu j784s4_hostemu))
export sciclient_hs_BOARD_DEPENDENCY = yes
endif
export sciclient_hs_BOARD_DEPENDENCY
export sciclient_hs_CORE_DEPENDENCY = yes
export sciclient_hs_PKG_LIST = sciclient_hs
export sciclient_hs_INCLUDE = $(sciclient_hs_PATH)
export sciclient_hs_SOCLIST = $(drvsciclient_SOCLIST)
export sciclient_hs_BOARDLIST = $(drvsciclient_BOARDLIST)
export sciclient_hs_$(SOC)_CORELIST = $(drvsciclient_$(SOC)_CORELIST)

export sciclient_direct_COMP_LIST = sciclient_direct
export sciclient_direct_RELPATH = ti/drv/sciclient_direct
export sciclient_direct_OBJPATH = ti/drv/sciclient_direct
export sciclient_direct_LIBNAME = sciclient_direct
export sciclient_direct_PATH = $(PDK_SCICLIENT_COMP_PATH)
export sciclient_direct_LIBPATH = $(PDK_SCICLIENT_COMP_PATH)/lib
export sciclient_direct_MAKEFILE = -fsrc/sciclient_direct_makefile BUILD_HS=no
export sciclient_direct_BOARD_DEPENDENCY = no
export sciclient_direct_CORE_DEPENDENCY = yes
export sciclient_direct_PKG_LIST = sciclient_direct
export sciclient_direct_INCLUDE = $(sciclient_direct_PATH)
export sciclient_direct_SOCLIST = j721e j7200 j721s2 j784s4
export sciclient_direct_BOARDLIST = j721e_evm j7200_evm j721s2_evm j784s4_evm
export sciclient_direct_$(SOC)_CORELIST = mcu1_0

export sciclient_direct_hs_COMP_LIST = sciclient_direct_hs
export sciclient_direct_hs_RELPATH = ti/drv/sciclient_direct_hs
export sciclient_direct_hs_OBJPATH = ti/drv/sciclient_direct_hs
export sciclient_direct_hs_LIBNAME = sciclient_direct_hs
export sciclient_direct_hs_PATH = $(PDK_SCICLIENT_COMP_PATH)
export sciclient_direct_hs_LIBPATH = $(PDK_SCICLIENT_COMP_PATH)/lib
export sciclient_direct_hs_MAKEFILE = -fsrc/sciclient_direct_makefile BUILD_HS=yes
export sciclient_direct_hs_BOARD_DEPENDENCY = no
export sciclient_direct_hs_CORE_DEPENDENCY = yes
export sciclient_direct_hs_PKG_LIST = sciclient_direct_hs
export sciclient_direct_hs_INCLUDE = $(sciclient_hs_direct_PATH)
export sciclient_direct_hs_SOCLIST = j721e j7200 j721s2 j784s4
export sciclient_direct_hs_BOARDLIST = j721e_evm j7200_evm j721s2_evm j784s4_evm
export sciclient_direct_hs_$(SOC)_CORELIST = mcu1_0

export sciserver_tirtos_COMP_LIST = sciserver_tirtos
export sciserver_tirtos_RELPATH = ti/drv/sciserver_tirtos
export sciserver_tirtos_OBJPATH = ti/drv/sciserver_tirtos
export sciserver_tirtos_LIBNAME = sciserver_tirtos
export sciserver_tirtos_PATH = $(PDK_SCICLIENT_COMP_PATH)
export sciserver_tirtos_LIBPATH = $(PDK_SCICLIENT_COMP_PATH)/lib
export sciserver_tirtos_MAKEFILE = -fsrc/sciserver_makefile
export sciserver_tirtos_BOARD_DEPENDENCY = no
export sciserver_tirtos_CORE_DEPENDENCY = yes
export sciserver_tirtos_PKG_LIST = sciserver_tirtos
export sciserver_tirtos_INCLUDE = $(sciserver_PATH)
export sciserver_tirtos_SOCLIST = j721e j7200 j721s2 j784s4
export sciserver_tirtos_BOARDLIST = j721e_evm j7200_evm j721s2_evm j784s4_evm
export sciserver_tirtos_$(SOC)_CORELIST = mcu1_0

export sciserver_baremetal_COMP_LIST = sciserver_baremetal
export sciserver_baremetal_RELPATH = ti/drv/sciserver_baremetal
export sciserver_baremetal_OBJPATH = ti/drv/sciserver_baremetal
export sciserver_baremetal_LIBNAME = sciserver_baremetal
export sciserver_baremetal_PATH = $(PDK_SCICLIENT_COMP_PATH)
export sciserver_baremetal_LIBPATH = $(PDK_SCICLIENT_COMP_PATH)/lib
export sciserver_baremetal_MAKEFILE = -fsrc/sciserver_makefile IS_BAREMETAL=yes
export sciserver_baremetal_BOARD_DEPENDENCY = no
export sciserver_baremetal_CORE_DEPENDENCY = yes
export sciserver_baremetal_PKG_LIST = sciserver_baremetal
export sciserver_baremetal_INCLUDE = $(sciserver_PATH)
export sciserver_baremetal_SOCLIST = j721e j7200 j721s2 j784s4
export sciserver_baremetal_BOARDLIST = j721e_evm j7200_evm j721s2_evm j784s4_evm
export sciserver_baremetal_$(SOC)_CORELIST = mcu1_0


export sciclient_UTILS_LIST = sciclient_boardcfg
export sciclient_boardcfg_COMP_LIST = sciclient_boardcfg
export sciclient_boardcfg_RELPATH = ti/drv/sciclient
export sciclient_boardcfg_PATH = $(PDK_SCICLIENT_COMP_PATH)
export sciclient_boardcfg_MAKEFILE = -fsrc/boardcfg_makefile
export sciclient_boardcfg_BOARD_DEPENDENCY = yes
export sciclient_boardcfg_CORE_DEPENDENCY = yes
export sciclient_boardcfg_BOARDLIST = $(sciclient_BOARDLIST)
export sciclient_boardcfg_$(SOC)_CORELIST = mcu1_0
export sciclient_boardcfg_LIBNAME = sciclient_boardcfg

export sciclient_UTILS_LIST += sciclient_boardcfg_combined
export sciclient_boardcfg_combined_COMP_LIST = sciclient_boardcfg_combined
export sciclient_boardcfg_combined_RELPATH = ti/drv/sciclient
export sciclient_boardcfg_combined_PATH = $(PDK_SCICLIENT_COMP_PATH)
export sciclient_boardcfg_combined_MAKEFILE = -fsrc/boardcfg_makefile SBL_IMAGE_TYPE=combined
export sciclient_boardcfg_combined_BOARD_DEPENDENCY = yes
export sciclient_boardcfg_combined_CORE_DEPENDENCY = yes
export sciclient_boardcfg_combined_BOARDLIST = j7200_evm j721s2_evm j784s4_evm
export sciclient_boardcfg_combined_$(SOC)_CORELIST = mcu1_0
export sciclient_boardcfg_combined_LIBNAME = sciclient_boardcfg_combined

export sciclient_UTILS_LIST += sciclient_boardcfg_hs
export sciclient_boardcfg_hs_COMP_LIST = sciclient_boardcfg_hs
export sciclient_boardcfg_hs_RELPATH = ti/drv/sciclient
export sciclient_boardcfg_hs_PATH = $(PDK_SCICLIENT_COMP_PATH)
export sciclient_boardcfg_hs_MAKEFILE = -fsrc/boardcfg_makefile BUILD_HS=yes
export sciclient_boardcfg_hs_BOARD_DEPENDENCY = yes
export sciclient_boardcfg_hs_CORE_DEPENDENCY = yes
export sciclient_boardcfg_hs_BOARDLIST = $(sciclient_BOARDLIST)
export sciclient_boardcfg_hs_$(SOC)_CORELIST = mcu1_0
export sciclient_boardcfg_hs_LIBNAME = sciclient_boardcfg_hs

export rm_pm_hal_COMP_LIST = rm_pm_hal
export rm_pm_hal_RELPATH = ti/drv/sciclient/src/rm_pm_hal
export rm_pm_hal_OBJPATH = ti/drv/sciclient/src/rm_pm_hal
export rm_pm_hal_PATH = $(PDK_SCICLIENT_COMP_PATH)
export rm_pm_hal_LIBNAME = rm_pm_hal
export rm_pm_hal_LIBPATH = $(PDK_SCICLIENT_COMP_PATH)/lib
export rm_pm_hal_MAKEFILE = -fsrc/rm_pm_hal_makefile BUILD_HS=no
# Simulator versus Silicon has a different Firmware Image.
export rm_pm_hal_BOARD_DEPENDENCY = no
export rm_pm_hal_CORE_DEPENDENCY = yes
export rm_pm_hal_PKG_LIST = rm_pm_hal
export rm_pm_hal_INCLUDE = $(rm_pm_hal_PATH)
export rm_pm_hal_SOCLIST = j721e j7200 j721s2 j784s4
export rm_pm_hal_BOARDLIST = j721e_evm j7200_evm j721s2_evm j784s4_evm
export rm_pm_hal_$(SOC)_CORELIST = mcu1_0

############################
# sciclient examples
# List of examples under sciclient (+= is used at each example definition)
# All the tests mentioned in list are built when test target is called
# List below all examples for allowed values
############################
sciclient_EXAMPLE_LIST =

# SCICLIENT Firmware Boot Application
export sciclient_firmware_boot_TestApp_COMP_LIST = sciclient_firmware_boot_TestApp
export sciclient_firmware_boot_TestApp_RELPATH = ti/drv/sciclient/examples/sciclient_firmware_boot_TestApp
export sciclient_firmware_boot_TestApp_PATH = $(PDK_SCICLIENT_COMP_PATH)/examples/sciclient_firmware_boot_TestApp
export sciclient_firmware_boot_TestApp_SBL_CERT_KEY=$(SBL_CERT_KEY)
export sciclient_firmware_boot_TestApp_BOARD_DEPENDENCY = no
export sciclient_firmware_boot_TestApp_CORE_DEPENDENCY = yes
export sciclient_firmware_boot_TestApp_PKG_LIST = sciclient_firmware_boot_TestApp
export sciclient_firmware_boot_TestApp_INCLUDE = $(sciclient_firmware_boot_TestApp_PATH)
export sciclient_firmware_boot_TestApp_BOARDLIST = am65xx_evm am64x_evm j7200_evm j721e_evm j721s2_evm j784s4_evm
export sciclient_firmware_boot_TestApp_$(SOC)_CORELIST = mcu1_0
export sciclient_firmware_boot_TestApp_SBL_APPIMAGEGEN = no
ifeq ($(CORE),mcu1_0)
export sciclient_firmware_boot_TestApp_SBL_IMAGEGEN = yes
else
export sciclient_firmware_boot_TestApp_SBL_IMAGEGEN = no
endif
sciclient_EXAMPLE_LIST += sciclient_firmware_boot_TestApp

# SCICLIENT CCS Init Application
export sciclient_ccs_init_COMP_LIST = sciclient_ccs_init
export sciclient_ccs_init_RELPATH = ti/drv/sciclient/examples/sciclient_ccs_init
export sciclient_ccs_init_PATH = $(PDK_SCICLIENT_COMP_PATH)/examples/sciclient_ccs_init
export sciclient_ccs_init_SBL_CERT_KEY=$(SBL_CERT_KEY)
export sciclient_ccs_init_BOARD_DEPENDENCY = no
export sciclient_ccs_init_CORE_DEPENDENCY = yes
export sciclient_ccs_init_PKG_LIST = sciclient_ccs_init
export sciclient_ccs_init_INCLUDE = $(sciclient_ccs_init_PATH)
export sciclient_ccs_init_BOARDLIST = am65xx_evm j721e_sim j721e_evm am64x_evm j7200_evm j721s2_evm j784s4_evm
# This application is only for mcu1_0
export sciclient_ccs_init_$(SOC)_CORELIST = mcu1_0
export sciclient_ccs_init_SBL_APPIMAGEGEN = no
export sciclient_ccs_init_SBL_IMAGEGEN = yes
sciclient_EXAMPLE_LIST += sciclient_ccs_init

# SCICLIENT Tools: Clear CLEC Secure Claim C7x Application
export sciclient_clear_clec_secure_claim_COMP_LIST = sciclient_clear_clec_secure_claim
export sciclient_clear_clec_secure_claim_RELPATH = ti/drv/sciclient/tools/clearClecSecureClaim
export sciclient_clear_clec_secure_claim_PATH = $(PDK_SCICLIENT_COMP_PATH)/tools/clearClecSecureClaim
export sciclient_clear_clec_secure_claim_BOARD_DEPENDENCY = no
export sciclient_clear_clec_secure_claim_CORE_DEPENDENCY = yes
export sciclient_clear_clec_secure_claim_PKG_LIST = sciclient_clear_clec_secure_claim
export sciclient_clear_clec_secure_claim_INCLUDE = $(sciclient_clear_clec_secure_claim_PATH)
export sciclient_clear_clec_secure_claim_BOARDLIST = j721e_evm j721s2_evm j784s4_evm
# This application is only for C7x
export sciclient_clear_clec_secure_claim_$(SOC)_CORELIST = c7x_1
export sciclient_clear_clec_secure_claim_SBL_APPIMAGEGEN = yes
# sciclient_EXAMPLE_LIST += sciclient_clear_clec_secure_claim

# SCICLIENT RTOS Applications
define SCICLIENT_RTOS_APP_RULE

export sciclient_rtos_app_$(1)_COMP_LIST = sciclient_rtos_app_$(1)
export sciclient_rtos_app_$(1)_RELPATH = ti/drv/sciclient/examples/sciclient_rtos_app
export sciclient_rtos_app_$(1)_PATH = $(PDK_SCICLIENT_COMP_PATH)/examples/sciclient_rtos_app
export sciclient_rtos_app_$(1)_BOARD_DEPENDENCY = no
export sciclient_rtos_app_$(1)_CORE_DEPENDENCY = yes
export sciclient_rtos_app_$(1)_PKG_LIST = sciclient_rtos_app_$(1)
export sciclient_rtos_app_$(1)_INCLUDE = $(sciclient_rtos_app_$(1)_PATH)
export sciclient_rtos_app_$(1)_BOARDLIST = $(filter $(DEFAULT_BOARDLIST_$(1)), am65xx_evm j721e_sim j721e_evm am64x_evm j7200_evm j721s2_evm j784s4_evm)

ifeq ($(SOC), am64x)
# No M4F for the RTOS App
export sciclient_rtos_app_$(1)_$(SOC)_CORELIST = $(filter $(DEFAULT_$(SOC)_CORELIST_$(1)), mpu1_0 mcu1_0 mcu1_1 mcu2_0 mcu2_1)
else
export sciclient_rtos_app_$(1)_$(SOC)_CORELIST = $(filter $(DEFAULT_$(SOC)_CORELIST_$(1)), $(drvsciclient_$(SOC)_CORELIST))
endif
export sciclient_rtos_app_$(1)_SBL_APPIMAGEGEN = yes
export sciclient_rtos_app_$(1)_SBL_IMAGEGEN = no
export sciclient_rtos_app_$(1)_MAKEFILE = -f makefile BUILD_OS_TYPE=$(1)
export sciclient_rtos_app_$(1)_XDC_CONFIGURO = $(if $(findstring tirtos, $(1)), yes, no)
ifneq ($(1),$(filter $(1), safertos))
sciclient_EXAMPLE_LIST += sciclient_rtos_app_$(1)
else
ifneq ($(wildcard $(SAFERTOS_KERNEL_INSTALL_PATH)),)
sciclient_EXAMPLE_LIST += sciclient_rtos_app_$(1)
endif
endif

endef

SCICLIENT_RTOS_APP_MACRO_LIST := $(foreach curos, $(drvsciclient_RTOS_LIST) safertos, $(call SCICLIENT_RTOS_APP_RULE,$(curos)))

$(eval ${SCICLIENT_RTOS_APP_MACRO_LIST})


# SCICLIENT RTOS UTs
define SCICLIENT_UNIT_TESTAPP_RULE

export sciclient_unit_testapp_$(1)_COMP_LIST = sciclient_unit_testapp_$(1)
export sciclient_unit_testapp_$(1)_RELPATH = ti/drv/sciclient/examples/sciclient_unit_testapp
export sciclient_unit_testapp_$(1)_PATH = $(PDK_SCICLIENT_COMP_PATH)/examples/sciclient_unit_testapp
export sciclient_unit_testapp_$(1)_BOARD_DEPENDENCY = yes
export sciclient_unit_testapp_$(1)_CORE_DEPENDENCY = yes
export sciclient_unit_testapp_$(1)_PKG_LIST = sciclient_unit_testapp_$(1)
export sciclient_unit_testapp_$(1)_INCLUDE = $(sciclient_unit_testapp_$(1)_PATH)
export sciclient_unit_testapp_$(1)_BOARDLIST = $(filter $(DEFAULT_BOARDLIST_$(1)), am65xx_evm j721e_sim j721e_evm am64x_evm j7200_evm j721s2_evm j784s4_evm)

ifeq ($(SOC),am64x)
# No M4F for the RTOS App
export sciclient_unit_testapp_$(1)_$(SOC)_CORELIST = $(filter $(DEFAULT_$(SOC)_CORELIST_$(1)), mpu1_0 mcu1_0 mcu1_1 mcu2_0 mcu2_1)
else
export sciclient_unit_testapp_$(1)_$(SOC)_CORELIST = $(filter $(DEFAULT_$(SOC)_CORELIST_$(1)), $(drvsciclient_$(SOC)_CORELIST))
endif
export sciclient_unit_testapp_$(1)_SBL_APPIMAGEGEN = no
ifeq ($(SOC),$(filter $(SOC), j721e am65xx j7200 j721s2 j784s4 am64x))
export sciclient_unit_testapp_$(1)_SBL_APPIMAGEGEN = yes
endif
export sciclient_unit_testapp_$(1)_SBL_IMAGEGEN = no
export sciclient_unit_testapp_$(1)_MAKEFILE = -f makefile BUILD_OS_TYPE=$(1)
export sciclient_unit_testapp_$(1)_XDC_CONFIGURO = $(if $(findstring tirtos, $(1)), yes, no)
ifneq ($(1),$(filter $(1), safertos))
sciclient_EXAMPLE_LIST += sciclient_unit_testapp_$(1)
else
ifneq ($(wildcard $(SAFERTOS_KERNEL_INSTALL_PATH)),)
sciclient_EXAMPLE_LIST += sciclient_unit_testapp_$(1)
endif
endif

endef

SCICLIENT_UNIT_TESTAPP_MACRO_LIST := $(foreach curos, $(drvsciclient_RTOS_LIST) safertos, $(call SCICLIENT_UNIT_TESTAPP_RULE,$(curos)))

$(eval ${SCICLIENT_UNIT_TESTAPP_MACRO_LIST})


# SCISERVER RTOS Apps
define SCISERVER_TESTAPP_RULE

export sciserver_testapp_$(1)_COMP_LIST = sciserver_testapp_$(1)
export sciserver_testapp_$(1)_RELPATH = ti/drv/sciclient/examples/sciserver_testapp
export sciserver_testapp_$(1)_PATH = $(PDK_SCICLIENT_COMP_PATH)/examples/sciserver_testapp
export sciserver_testapp_$(1)_BOARD_DEPENDENCY = no
export sciserver_testapp_$(1)_CORE_DEPENDENCY = yes
export sciserver_testapp_$(1)_PKG_LIST = sciserver_testapp_$(1)
export sciserver_testapp_$(1)_INCLUDE = $(sciserver_testapp_$(1)_PATH)
export sciserver_testapp_$(1)_BOARDLIST = $(filter $(DEFAULT_BOARDLIST_$(1)), j721e_evm j7200_evm j721s2_evm j784s4_evm)
export sciserver_testapp_$(1)_$(SOC)_CORELIST = $(filter $(DEFAULT_$(SOC)_CORELIST_$(1)), mcu1_0)
export sciserver_testapp_$(1)_SBL_APPIMAGEGEN = yes
export sciserver_testapp_$(1)_SBL_IMAGEGEN = no
export sciserver_testapp_$(1)_MAKEFILE = -f makefile BUILD_OS_TYPE=$(1)
export sciserver_testapp_$(1)_XDC_CONFIGURO = $(if $(findstring tirtos, $(1)), yes, no)
ifneq ($(1),$(filter $(1), safertos))
sciclient_EXAMPLE_LIST += sciserver_testapp_$(1)
else
ifneq ($(wildcard $(SAFERTOS_KERNEL_INSTALL_PATH)),)
ifeq ($(BUILD_PROFILE), release)
# Only supporting release build for SafeRTOS, since debug binaries won't fit in available memory.
sciclient_EXAMPLE_LIST += sciserver_testapp_$(1)
endif
endif
endif
endef

SCISERVER_TESTAPP_MACRO_LIST := $(foreach curos, $(drvsciclient_RTOS_LIST) safertos, $(call SCISERVER_TESTAPP_RULE,$(curos)))

$(eval ${SCISERVER_TESTAPP_MACRO_LIST})


# SCICLIENT RTOS Firewall Unit tests
define SCICLIENT_FW_TESTAPP_RULE

export sciclient_fw_testapp_$(1)_COMP_LIST = sciclient_fw_testapp_$(1)
export sciclient_fw_testapp_$(1)_RELPATH = ti/drv/sciclient/examples/sciclient_fw_testapp
export sciclient_fw_testapp_$(1)_PATH = $(PDK_SCICLIENT_COMP_PATH)/examples/sciclient_fw_testapp
export sciclient_fw_testapp_$(1)_BOARD_DEPENDENCY = no
export sciclient_fw_testapp_$(1)_CORE_DEPENDENCY = yes
export sciclient_fw_testapp_$(1)_PKG_LIST = sciclient_fw_testapp_$(1)
#Disable J7AHP till FW CSLR is fixed
export sciclient_fw_testapp_$(1)_BOARDLIST = $(filter $(DEFAULT_BOARDLIST_$(1)), am65xx_evm j721e_evm j7200_evm j721s2_evm j784s4_evm)
export sciclient_fw_testapp_$(1)_$(SOC)_CORELIST = $(filter $(DEFAULT_$(SOC)_CORELIST_$(1)), mcu1_0)
export sciclient_fw_testapp_$(1)_SBL_APPIMAGEGEN = yes
export sciclient_fw_testapp_$(1)_SBL_IMAGEGEN = no
export sciclient_fw_testapp_$(1)_MAKEFILE = -f makefile BUILD_OS_TYPE=$(1)
export sciclient_fw_testapp_$(1)_XDC_CONFIGURO = $(if $(findstring tirtos, $(1)), yes, no)
ifneq ($(1),$(filter $(1), safertos))
sciclient_EXAMPLE_LIST += sciclient_fw_testapp_$(1)
else
ifneq ($(wildcard $(SAFERTOS_KERNEL_INSTALL_PATH)),)
sciclient_EXAMPLE_LIST += sciclient_fw_testapp_$(1)
endif
endif

endef

SCICLIENT_FW_TESTAPP_MACRO_LIST := $(foreach curos, $(drvsciclient_RTOS_LIST) safertos, $(call SCICLIENT_FW_TESTAPP_RULE,$(curos)))

$(eval ${SCICLIENT_FW_TESTAPP_MACRO_LIST})


# SCICLIENT PMIC shutdown test
define SCICLIENT_PMIC_SHUTDOWN_TESTAPP_RULE

export sciclient_pmic_shutdown_testapp_$(1)_COMP_LIST = sciclient_pmic_shutdown_testapp_$(1)
export sciclient_pmic_shutdown_testapp_$(1)_RELPATH = ti/drv/sciclient/examples/sciclient_pmic_shutdown_testapp
export sciclient_pmic_shutdown_testapp_$(1)_PATH = $(PDK_SCICLIENT_COMP_PATH)/examples/sciclient_pmic_shutdown_testapp
export sciclient_pmic_shutdown_testapp_$(1)_BOARD_DEPENDENCY = no
export sciclient_pmic_shutdown_testapp_$(1)_CORE_DEPENDENCY = yes
export sciclient_pmic_shutdown_testapp_$(1)_PKG_LIST = sciclient_pmic_shutdown_testapp_$(1)
export sciclient_pmic_shutdown_testapp_$(1)_BOARDLIST = $(filter $(DEFAULT_BOARDLIST_$(1)),j721e_evm j7200_evm j721s2_evm j784s4_evm)
export sciclient_pmic_shutdown_testapp_$(1)_$(SOC)_CORELIST = $(filter $(DEFAULT_$(SOC)_CORELIST_$(1)), mcu1_0)
export sciclient_pmic_shutdown_testapp_$(1)_SBL_APPIMAGEGEN = yes
export sciclient_pmic_shutdown_testapp_$(1)_SBL_IMAGEGEN = no
export sciclient_pmic_shutdown_testapp_$(1)_MAKEFILE = -f makefile BUILD_OS_TYPE=$(1)
export sciclient_pmic_shutdown_testapp_$(1)_XDC_CONFIGURO = $(if $(findstring tirtos, $(1)), yes, no)
ifneq ($(1),$(filter $(1), safertos))
sciclient_EXAMPLE_LIST += sciclient_pmic_shutdown_testapp_$(1)
else
ifneq ($(wildcard $(SAFERTOS_KERNEL_INSTALL_PATH)),)
sciclient_EXAMPLE_LIST += sciclient_pmic_shutdown_testapp_$(1)
endif
endif

endef

SCICLIENT_PMIC_SHUTDOWN_TESTAPP_MACRO_LIST := $(foreach curos, $(drvsciclient_RTOS_LIST) safertos, $(call SCICLIENT_PMIC_SHUTDOWN_TESTAPP_RULE,$(curos)))

$(eval ${SCICLIENT_PMIC_SHUTDOWN_TESTAPP_MACRO_LIST})

export sciclient_LIB_LIST
export sciclient_EXAMPLE_LIST
export drvsciclient_LIB_LIST = $(sciclient_LIB_LIST)
export drvsciclient_EXAMPLE_LIST  = $(sciclient_EXAMPLE_LIST)

sciclient_component_make_include := 1
endif
