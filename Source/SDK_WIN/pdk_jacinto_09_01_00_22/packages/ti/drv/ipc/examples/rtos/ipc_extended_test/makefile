#
# This file is the makefile for building IPC multi-core echo test app for RTOS
#
ifeq ($(RULES_MAKE), )
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(RULES_MAKE)
endif

APP_NAME = ipc_extended_test_$(BUILD_OS_TYPE)

SRCS_COMMON += ipc_extended_test.c ipc_extended_testsetup.c
PACKAGE_SRCS_COMMON = .
CFLAGS_LOCAL_COMMON += -DIPC_EXTENDED_TEST

#Path for including common files
IPC_COMMON_PATH = ../../common

include $(IPC_COMMON_PATH)/makefile.mk
