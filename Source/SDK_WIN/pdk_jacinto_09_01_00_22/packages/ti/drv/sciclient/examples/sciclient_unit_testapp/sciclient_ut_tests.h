/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 *  \file sciclient_ut_tests.h
 *
 *  \brief This file defines the test cases for SCICLIENT UT.
 */

#ifndef SCICLIENT_TEST_CASES_H_
#define SCICLIENT_TEST_CASES_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <ti/csl/soc.h>
#include <ti/drv/sciclient/examples/common/sciclient_appCommon.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define SCICLIENT_NUM_TESTCASES   (sizeof (gSciclientTestcaseParams) / \
                                   sizeof (App_sciclientTestParams_t))

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Defines the various SCICLIENT test cases. */
App_sciclientTestParams_t gSciclientTestcaseParams[] =
{
    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        1U,

        /** *reqId **/
        "PDK-2142::PDK-2140::PDK-2139::PDK-2141",

        /** *testCaseName **/
        "SCICLIENT DMSC Get Firmware Version (Polled)",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient loads firmware and gets ACK after sending get revision\
         message to firmware without any error ",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_SANITY)
    },

    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        2U,

        /** *reqId **/
        "PDK-2142::PDK-2140::PDK-2139::PDK-2141",

        /** *testCaseName **/
        "SCICLIENT DMSC Get Firmware Version (Interrupt)",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient loads firmware and gets ACK after sending get revision\
         message to firmware without any error ",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_SANITY)
    },
    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        3U,

        /** *reqId **/
        "PDK-2490::PDK-2915::PDK-2907::PDK-2906",

        /** *testCaseName **/
        "SCICLIENT Check for Invalid Req Prm",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient_service should fail when \
         called with invalid parameters ",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_NEGATIVE)
    },

    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        4U,

        /** *reqId **/
        "PDK-2490::PDK-2915::PDK-2907::PDK-2906",

        /** *testCaseName **/
        "SCICLIENT Check Timeout",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient_service should fail with timeout when \
         called with invalid parameters ",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_NEGATIVE)
    },
#if defined(SOC_AM65XX) || defined(SOC_J721E) || defined (SOC_J7200) || defined (SOC_J721S2) || defined (SOC_J784S4)
    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        5U,

        /** *reqId **/
        "PDK:4277",

        /** *testCaseName **/
        "SCICLIENT MSMC Query Test",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient_msmcQuery should return the correct \
         start and end address of msmc region ",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_SANITY)
    },
#endif
#if defined(SOC_AM65XX)
    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        6U,

        /** *reqId **/
        "PDK:2145",

        /** *testCaseName **/
        "SCICLIENT RM Get Configuruation Test",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient_Get coniguration should return correctly for PG1.0 and PG2.0 ",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_SANITY)
    }
#elif defined(ENABLE_MSG_FWD)
    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        6U,

        /** *reqId **/
        "PDK:10418",

        /** *testCaseName **/
        "SCICLIENT TIFS2DM Message Forwarding Test",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient gets ACK and the DEV ID 0 state info, after sending get device\
         state message on a secure queue (to TIFS and then DM) without any error.",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_SANITY)
    },
#endif
#if defined(ENABLE_FW_NOTIFICATION)
    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        7U,

        /** *reqId **/
        "PDK-12208::PDK-12209",

        /** *testCaseName **/
        "SCICLIENT Firewall Exception Notification Testapp",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient should receive Firewall exception interrupts and be able to read\
        the firewall exception logging register value.",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_SANITY)
    },
#endif
#if ((defined (SOC_J721S2) || defined (SOC_J784S4)) && defined(BUILD_MCU2_0))
    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        8U,

        /** *reqId **/
        "PDK-13207",

        /** *testCaseName **/
        "PVU to Main R5 Interrupt Route Testing",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient programs the interrupt route between PVU and R5F core in the Main domain\
        and also tests if the R5F core is executing the registered ISR on generating the\
        interrupt from PVU. It also deletes the set interrupt route between PVU and Main R5F\
        core and tests if the R5F core is still executing the registered ISR on generating\
        the interrupt from PVU.",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_SANITY)
    },
#endif
#if ((defined (SOC_J721S2) || defined (SOC_J784S4)) && defined(BUILD_MCU2_0))
    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        9U,

        /** *reqId **/
        "PDK-13207",

        /** *testCaseName **/
        "PVU to GIC Interrupt Route Testing",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient programs the interrupt route between PVU and GIC in the Main domain.",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_SANITY)
    },
#endif
#if ((defined (SOC_J721E) || defined (SOC_J7200) || defined (SOC_J721S2) || defined (SOC_J784S4)) && defined (BUILD_MCU1_0))
    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        10U,

        /** *reqId **/
        "PDK-13154",

        /** *testCaseName **/
        "SCICLIENT UART Interrupt Route Testing",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient programs the interrupt route between Main UART and MCU R5F core\
        and tests the interaction between Main UART and MCU R5F core using the UART_write function.",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_SANITY)
    }
#endif
};

#endif /* #ifndef SCICLIENT_TEST_CASES_H_ */
