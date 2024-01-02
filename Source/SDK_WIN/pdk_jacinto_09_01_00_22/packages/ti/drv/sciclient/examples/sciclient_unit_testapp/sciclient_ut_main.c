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
 *  \file sciclient_ut_main.c
 *
 *  \brief Implementation of Sciclient Unit Test
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/osal/osal.h>
#include <ti/osal/TimerP.h>
#include <ti/osal/TaskP.h>

#include <stdint.h>
#include <string.h>
#include <ti/csl/tistdtypes.h>
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/csl/hw_types.h>
#if ((defined (SOC_J721S2) || defined (SOC_J784S4)) && defined(BUILD_MCU2_0))
#include <ti/csl/cslr_pvu.h>
#endif
#include <ti/drv/sciclient/examples/common/sciclient_appCommon.h>
#include <ti/board/board.h>
#include <ti/drv/sciclient/examples/sciclient_unit_testapp/sciclient_ut_tests.h>
#if ((defined (SOC_J721E) || defined (SOC_J7200) || defined (SOC_J721S2) || defined (SOC_J784S4)) && defined (BUILD_MCU1_0))
#include <ti/drv/sciclient/examples/sciclient_unit_testapp/uart_utils.h>
#endif
#if defined(ENABLE_FW_NOTIFICATION)
#include <ti/drv/sciclient/examples/sciclient_unit_testapp/sciclient_fw_notify.h>
#endif
#if defined (BUILD_C7X)
#include <ti/csl/csl_clec.h>
#endif

/* This is only needed as this test case is running back to back Sciclient
 * Init and de-inits.
 */
#include <ti/drv/sciclient/src/sciclient/sciclient_priv.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

void mainTask(void* arg0, void* arg1);
static int32_t App_getRevisionTestPol(void);
static int32_t App_getRevisionTestIntr(void);
static int32_t App_timeoutTest(void);
static int32_t App_invalidReqPrmTest(void);
#if defined (SOC_AM65XX) || defined (SOC_J721E) || defined (SOC_J7200) || defined (SOC_J721S2) || defined (SOC_J784S4)
static int32_t App_msmcQueryTest(void);
#endif
#if defined(SOC_AM65XX)
static int32_t App_rmGetResourceRange(void);
#elif defined(ENABLE_MSG_FWD)
static int32_t App_tifs2dmMsgForwardingTest(void);
#endif
#if defined(ENABLE_FW_NOTIFICATION)
static int32_t App_fwExcpNotificationTest(void);
#endif
#if ((defined (SOC_J721S2) || defined (SOC_J784S4)) && defined(BUILD_MCU2_0))
static int32_t App_pvu2R5IntrTest(void);
static void App_pvu2R5IntrTestIsr(void);
static int32_t App_pvu2GICIntrTest(void);
#endif
#if ((defined (SOC_J721E) || defined (SOC_J7200) || defined (SOC_J721S2) || defined (SOC_J784S4) || defined (j784s4_evm)) && defined (BUILD_MCU1_0))
static int32_t App_mainUart2MCUR5IntrTest(void);
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* For SafeRTOS on R5F with FFI Support, task stack should be aligned to the stack size */
#if defined(SAFERTOS) && defined (BUILD_MCU)
static uint8_t  gAppTskStackMain[32*1024] __attribute__((aligned(32*1024))) = { 0 };
#else
static uint8_t  gAppTskStackMain[32*1024] __attribute__((aligned(8192)));
#endif
/* IMPORTANT NOTE: For C7x,
 * - stack size and stack ptr MUST be 8KB aligned
 * - AND min stack size MUST be 32KB
 * - AND stack assigned for task context is "size - 8KB"
 *       - 8KB chunk for the stack area is used for interrupt handling in this task context
 */
#if ((defined (SOC_J721S2) || defined (SOC_J784S4)) && defined(BUILD_MCU2_0))
static volatile int32_t gAppIsrExecNum=0;
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int main(void)
{
    TaskP_Handle task;
    TaskP_Params taskParams;

    uint32_t retVal = CSL_PASS;

    /*  This should be called before any other OS calls (like Task creation, OS_start, etc..) */
    OS_init();

    memset( gAppTskStackMain, 0xFF, sizeof( gAppTskStackMain ) );
    TaskP_Params_init(&taskParams);
    taskParams.priority =2;
    taskParams.stack        = gAppTskStackMain;
    taskParams.stacksize    = sizeof (gAppTskStackMain);
    task = TaskP_create(&mainTask, &taskParams);
    if(NULL==task)
    {
        OS_stop();
    }

    OS_start();

    return retVal;
}

void mainTask(void* arg0, void* arg1)
{
    /*To suppress unused variable warning*/
    (void)arg0;
    (void)arg1;

    volatile uint32_t loopForever = 1U;

    App_sciclientParser();

    while(loopForever);
}
uint32_t App_sciclientGetNumTests(void)
{
    return SCICLIENT_NUM_TESTCASES;
}

int32_t App_sciclientTestMain(App_sciclientTestParams_t *testParams)
{
    switch (testParams->testcaseId)
    {
        case 1:
            testParams->testResult = App_getRevisionTestPol();
            break;
        case 2:
            testParams->testResult = App_getRevisionTestIntr();
            break;
        case 3:
            testParams->testResult = App_invalidReqPrmTest();
            break;
        case 4:
            testParams->testResult = App_timeoutTest();
            break;
#if defined (SOC_AM65XX) || defined (SOC_J721E) || defined (SOC_J7200) || defined (SOC_J721S2) || defined (SOC_J784S4)
        case 5:
            testParams->testResult = App_msmcQueryTest();
            break;
#endif
#if defined (SOC_AM65XX)
        case 6:
            testParams->testResult = App_rmGetResourceRange();
            break;
#elif defined(ENABLE_MSG_FWD)
        case 6:
            testParams->testResult = App_tifs2dmMsgForwardingTest();
            break;
#endif
#if defined(ENABLE_FW_NOTIFICATION)
        case 7:
            testParams->testResult = App_fwExcpNotificationTest();
            break;
#endif
#if ((defined (SOC_J721S2) || defined (SOC_J784S4)) && defined(BUILD_MCU2_0))
        case 8:
            testParams->testResult = App_pvu2R5IntrTest();
            break;
#endif
#if ((defined (SOC_J721S2) || defined (SOC_J784S4)) && defined(BUILD_MCU2_0))
        case 9:
            testParams->testResult = App_pvu2GICIntrTest();
            break;
#endif
#if ((defined (SOC_J721E) || defined (SOC_J7200) || defined (SOC_J721S2) || defined (SOC_J784S4)) && defined (BUILD_MCU1_0))
        case 10:
            testParams->testResult = App_mainUart2MCUR5IntrTest();
            break;
#endif
        default:
            break;
    }
    return 0;
}

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

/* This is only needed as this test case is running back to back Sciclient
 * Init and de-inits.
 */
extern Sciclient_ServiceHandle_t gSciclientHandle;

int32_t App_getRevisionTestPol(void)
{
    int32_t status = CSL_EFAIL;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL,
        0 /* isSecure = 0 un secured for all cores */
    };
    struct tisci_msg_version_req request;
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t *) &request,
        sizeof(request),
        SCICLIENT_SERVICE_WAIT_FOREVER
    };

    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };
    /* This is only needed as this test case is running back to back Sciclient
     * Init and de-inits.
     */
    while (gSciclientHandle.initCount != 0)
    {
        status = Sciclient_deinit();
    }

    status = Sciclient_init(&config);

    if (status == CSL_PASS)
    {
        status = Sciclient_service(&reqPrm, &respPrm);

        if (CSL_PASS == status)
        {
            if (respPrm.flags == TISCI_MSG_FLAG_ACK)
            {
                status = CSL_PASS;
                App_sciclientPrintf(
                                  " DMSC Firmware Version %s\n",
                                  (char *) response.str);
                App_sciclientPrintf(
                                  " Firmware revision 0x%x\n", response.version);
                App_sciclientPrintf(
                                  " ABI revision %d.%d\n", response.abi_major,
                                  response.abi_minor);
            }
            else
            {
                App_sciclientPrintf(
                                  " DMSC Firmware Get Version failed \n");
            }
        }
        else
        {
            App_sciclientPrintf(
                              " DMSC Firmware Get Version failed \n");
        }
    }
    if (status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
    return status;
}

int32_t App_getRevisionTestIntr(void)
{
    int32_t status = CSL_EFAIL;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_INTERRUPT,
        NULL,
        0 /* isSecure = 0 un secured for all cores */
    };

    struct tisci_msg_version_req request;
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t *) &request,
        sizeof(request),
        SCICLIENT_SERVICE_WAIT_FOREVER
    };
    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };
    /* This is only needed as this test case is running back to back Sciclient
     * Init and de-inits.
     */
    while (gSciclientHandle.initCount != 0)
    {
        status = Sciclient_deinit();
    }

    status = Sciclient_init(&config);
    if (status == CSL_PASS)
    {
        status = Sciclient_service(&reqPrm, &respPrm);
        if (CSL_PASS == status)
        {
            if (respPrm.flags == TISCI_MSG_FLAG_ACK)
            {
                status = CSL_PASS;
                App_sciclientPrintf(
                                  " DMSC Firmware Version %s\n",
                                  (char *) response.str);
                App_sciclientPrintf(
                                  " Firmware revision 0x%x\n", response.version);
                App_sciclientPrintf(
                                  " ABI revision %d.%d\n", response.abi_major,
                                  response.abi_minor);
            }
            else
            {
                App_sciclientPrintf(
                                  " DMSC Firmware Get Version failed \n");
            }
        }
        else
        {
            App_sciclientPrintf(
                              " DMSC Firmware Get Version failed \n");
        }
    }
    if (status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
    return status;
}

static int32_t App_invalidReqPrmTest(void)
{
    int32_t status = CSL_EFAIL;

    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL,
        0 /* isSecure = 0 un secured for all cores */
    };
    struct tisci_msg_version_req request;
    const Sciclient_ReqPrm_t      reqPrm_badTxSize =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t*) &request,
        100,
        SCICLIENT_SERVICE_WAIT_FOREVER
    };

    const Sciclient_ReqPrm_t      reqPrm_good =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t*)&request,
        0,
        SCICLIENT_SERVICE_WAIT_FOREVER
    };

    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm_badRxsize =
    {
        0,
        (uint8_t *) &response,
        100
    };

    Sciclient_RespPrm_t           respPrm_good =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };
    /* This is only needed as this test case is running back to back Sciclient
     * Init and de-inits.
     */
    while (gSciclientHandle.initCount != 0)
    {
        status = Sciclient_deinit();
    }
    status = Sciclient_init(&config);
    if (status == CSL_PASS)
    {
        status = Sciclient_service(NULL, &respPrm_good);
        if (CSL_EBADARGS == status)
        {
            status = CSL_PASS;
            App_sciclientPrintf(" NULL Arg Test PASSED \n");
        }
        else
        {
            App_sciclientPrintf(" NULL Arg Test FAILED \n");
        }
        status = Sciclient_service(&reqPrm_badTxSize, &respPrm_good);
        if (CSL_EBADARGS == status)
        {
            status = CSL_PASS;
            App_sciclientPrintf(" Tx Payload Check PASSED \n");
        }
        else
        {
            App_sciclientPrintf(" Tx Payload Check FAILED \n");
        }
        status = Sciclient_service(&reqPrm_good, &respPrm_badRxsize);
        if (CSL_EBADARGS == status)
        {
            status = CSL_PASS;
            App_sciclientPrintf(" Rx Payload Check PASSED \n");
        }
        else
        {
            App_sciclientPrintf(" Rx Payload Check FAILED \n");
        }
    }
    if (status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
    return status;
}

static int32_t App_timeoutTest(void)
{
    int32_t status = CSL_EFAIL;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL,
        0 /* isSecure = 0 un secured for all cores */
    };
    struct tisci_msg_version_req request;
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t *)&request,
        sizeof(request),
        (uint32_t)0x0A
    };
    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };
    /* This is only needed as this test case is running back to back Sciclient
     * Init and de-inits.
     */
    while (gSciclientHandle.initCount != 0)
    {
        status = Sciclient_deinit();
    }
    status = Sciclient_init(&config);
    if (status == CSL_PASS)
    {
        status = Sciclient_service(&reqPrm, &respPrm);
        if (CSL_ETIMEOUT == status)
        {
            status = CSL_PASS;
            App_sciclientPrintf(
                             " Timeout test PASSED \n");
        }
        else
        {
            App_sciclientPrintf(
                             " Timeout Test FAILED \n");
        }
    }
    if (status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
    return status;
}

#if defined (SOC_AM65XX) || defined (SOC_J721E) || defined (SOC_J7200) || defined (SOC_J721S2) || defined (SOC_J784S4)
static int32_t App_msmcQueryTest(void)
{
    int32_t status = CSL_EFAIL;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL,
        0 /* isSecure = 0 un secured for all cores */
    };

    const struct tisci_query_msmc_req req = {};
    struct tisci_query_msmc_resp resp;
    /* This is only needed as this test case is running back to back Sciclient
     * Init and de-inits.
     */
    while (gSciclientHandle.initCount != 0)
    {
        status = Sciclient_deinit();
    }
    status = Sciclient_init(&config);
    if (status == CSL_PASS)
    {
        status = Sciclient_msmcQuery(&req, &resp, SCICLIENT_SERVICE_WAIT_FOREVER);
    }
    if (status == CSL_PASS)
    {
        App_sciclientPrintf("MSMC Start Address = 0x%X%X\n", resp.msmc_start_high, resp.msmc_start_low);
        App_sciclientPrintf("MSMC End Address = 0x%X%X\n", resp.msmc_end_high, resp.msmc_end_low);
    }
    else
    {
        App_sciclientPrintf("MSMC Query FAILED !!");
    }
    if (status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
    return status;
}
#endif

#if defined (SOC_AM65XX)
struct appSciclientRmGetRangeTest
{
    uint16_t           type;
    uint8_t            subtype;
    int32_t            expectedResult;
};

struct appSciclientRmGetRangeTest gAppSciclientTstParams [] =
{
    {TISCI_DEV_NAVSS0_MODSS_INTA0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT, CSL_PASS},
    {TISCI_DEV_NAVSS0_MODSS_INTA0, TISCI_RESASG_SUBTYPE_IA_VINT, CSL_PASS},
    {TISCI_DEV_NAVSS0_MODSS_INTA1, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT, CSL_PASS},
    {TISCI_DEV_NAVSS0_MODSS_INTA1, TISCI_RESASG_SUBTYPE_IA_VINT, CSL_PASS},
    {TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT, CSL_PASS},
    {TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_IA_VINT, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_IA_VINT, CSL_PASS},
    {TISCI_DEV_NAVSS0_PROXY0, TISCI_RESASG_SUBTYPE_PROXY_PROXIES, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_PROXY0, TISCI_RESASG_SUBTYPE_PROXY_PROXIES, CSL_PASS},
    {TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H, CSL_PASS},
    {TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX, CSL_PASS},
    {TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_EXT, CSL_PASS},
    {TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H, CSL_PASS},
    {TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX, CSL_PASS},
    {TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_GP, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_GP, CSL_PASS},
    {TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_ERROR_OES, CSL_PASS},
    {TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_VIRTID, CSL_PASS},
    {TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_MONITORS, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_ERROR_OES, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_VIRTID, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_MONITORS, CSL_PASS},
    {TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN, CSL_PASS},
    {TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN, CSL_PASS},
    {TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_ECHAN, CSL_PASS},
    {TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN, CSL_PASS},
    {TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN, CSL_PASS},
    {TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_TRIGGER, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_TRIGGER, CSL_PASS},
    {TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON, CSL_PASS},
    {TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_INVALID_FLOW_OES, CSL_PASS},
    {TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_GLOBAL_CONFIG, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_INVALID_FLOW_OES, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_GLOBAL_CONFIG, CSL_PASS},
    {TISCI_DEV_CMPEVENT_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_MAIN2MCU_LVL_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_MAIN2MCU_LVL_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_MAIN2MCU_PLS_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_GPIOMUX_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_TIMESYNC_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_WKUP_GPIOMUX_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS},
    {TISCI_DEV_MCU_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT, CSL_PASS}
};

static int32_t App_rmGetResourceRange(void)
{
    int32_t status = CSL_EFAIL;
    uint32_t i = 0U;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL,
        0 /* isSecure = 0 un secured for all cores */
    };
    /* This is only needed as this test case is running back to back Sciclient
     * Init and de-inits.
     */
    while (gSciclientHandle.initCount != 0)
    {
        status = Sciclient_deinit();
    }
    status = Sciclient_init(&config);

    for (i = 0U; i < sizeof (gAppSciclientTstParams)/sizeof (struct appSciclientRmGetRangeTest); i++)
    {
        struct tisci_msg_rm_get_resource_range_resp res;
        struct tisci_msg_rm_get_resource_range_req  req;
        if (status == CSL_PASS)
        {
            req.type           = gAppSciclientTstParams[i].type;
            req.subtype        = gAppSciclientTstParams[i].subtype;
            req.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

            /* Get interrupt number range */
            status =  Sciclient_rmGetResourceRange(
                        &req,
                        &res,
                        0xFFFFFFFFU);
        }
        if (status == CSL_PASS)
        {
            App_sciclientPrintf("\n\rType: %d, Subtype : %d, Start: %d, Num: %d",
                                gAppSciclientTstParams[i].type,
                                gAppSciclientTstParams[i].subtype,
                                res.range_start,
                                res.range_num);
        }
        else
        {
            if (status == gAppSciclientTstParams[i].expectedResult)
            {
                App_sciclientPrintf("\n\rType: %d, Subtype : %d, Expected Failure",
                                gAppSciclientTstParams[i].type,
                                gAppSciclientTstParams[i].subtype);
                status = CSL_PASS;
            }
            else
            {
                App_sciclientPrintf("\n\rType: %d, Subtype : %d, Unexpected Failure!!!!!!!",
                                gAppSciclientTstParams[i].type,
                                gAppSciclientTstParams[i].subtype);
            }
        }
    }

    if (status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
    return status;
}
#elif defined(ENABLE_MSG_FWD)
static int32_t App_tifs2dmMsgForwardingTest(void)
{
    int32_t status = CSL_EFAIL;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_INTERRUPT,
        NULL,
        1 /* isSecure = 1 to test passing message on secure channel */
    };

    App_sciclientPrintf(" Starting TIFS2DM msg forwarding test (from secure queue)\n");

    struct tisci_msg_get_device_req request;
    request.id = 0x1; /* Use DEV ID of 1 as a device to check state (any DEV ID will do) */
    const Sciclient_ReqPrm_t        reqPrm =
    {
        TISCI_MSG_GET_DEVICE,
        TISCI_MSG_FLAG_AOP,
        (uint8_t *) &request,
        sizeof(request),
        SCICLIENT_SERVICE_WAIT_FOREVER
    };

    struct tisci_msg_get_device_resp response;
    Sciclient_RespPrm_t              respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof(response)
    };
    /* This is only needed as this test case is running back to back Sciclient
     * Init and de-inits.
     */
    while (gSciclientHandle.initCount != 0)
    {
        status = Sciclient_deinit();
    }

    status = Sciclient_init(&config);
    if (status == CSL_PASS)
    {
        status = Sciclient_service(&reqPrm, &respPrm);
        if (CSL_PASS == status)
        {
            if ((respPrm.flags & TISCI_MSG_FLAG_ACK) == TISCI_MSG_FLAG_ACK)
            {
                status = CSL_PASS;
                App_sciclientPrintf(
                                  " Device ID 0 - Context Loss Count: %d\n",
                                  response.context_loss_count);
                App_sciclientPrintf(
                                  "             - Resets: %d\n",
                                  response.resets);
                App_sciclientPrintf(
                                  "             - Programmed state: %d\n",
                                  response.programmed_state);
                App_sciclientPrintf(
                                  "             - Current state: %d\n",
                                  response.current_state);
            }
            else
            {
                App_sciclientPrintf(
                                  " Device ID 0 - Get device state failed \n");
            }
        }
        else
        {
            App_sciclientPrintf(
                              " Device ID 0 - Get device state failed \n");
        }
    }
    if (status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
    return status;
}
#endif

#if defined(ENABLE_FW_NOTIFICATION)
extern uint32_t gInterruptRecieved;
static int32_t App_fwExcpNotificationTest(void)
{
    int32_t status = CSL_PASS;
    volatile uint32_t* excpRegCmbn;
    volatile uint32_t* excpRegDmsc;
    OsalRegisterIntrParams_t    intrPrmsDmscIntr;
    OsalRegisterIntrParams_t    intrPrmsCmbnIntr;
    CSL_R5ExptnHandlers sciclientR5ExptnHandlers;
    HwiP_Handle hwiPHandleDmscIntr;
    HwiP_Handle hwiPHandleCmbnIntr;

    App_sciclientPrintf(" Starting firewall notification handling (from secure queue)\n");
    
    excpRegDmsc = (volatile uint32_t*)DMSC_FW_EXCP_REG;
    excpRegCmbn = (volatile uint32_t*)CMBN_FW_EXCP_REG;
    
    /* Register an abort exception handler */
    Intc_InitExptnHandlers(&sciclientR5ExptnHandlers);
    sciclientR5ExptnHandlers.dabtExptnHandler = (void*)App_fwAbortHandlerIsr;
    Intc_RegisterExptnHandlers(&sciclientR5ExptnHandlers);

    /* Interrupt registration for dmsc interrupt notification routing */
    Osal_RegisterInterrupt_initParams(&intrPrmsCmbnIntr);  
    intrPrmsCmbnIntr.corepacConfig.isrRoutine       =
                                (void*)App_fwNotiIsrCmbn;
    intrPrmsCmbnIntr.corepacConfig.corepacEventNum  = 0;
    intrPrmsCmbnIntr.corepacConfig.intVecNum        = CMBN_FW_EXCEPTION_INTR;
    status =Osal_RegisterInterrupt(&intrPrmsCmbnIntr, &hwiPHandleCmbnIntr);

    /* Interrupt registration for dmsc interrupt notification routing */
    Osal_RegisterInterrupt_initParams(&intrPrmsDmscIntr);                                                              
    intrPrmsDmscIntr.corepacConfig.isrRoutine       =
                                (void*)App_fwNotiIsrDmsc;
    intrPrmsDmscIntr.corepacConfig.corepacEventNum  = 0;
    intrPrmsDmscIntr.corepacConfig.intVecNum        = DMSC_FW_EXCEPTION_INTR;
    status =Osal_RegisterInterrupt(&intrPrmsDmscIntr, &hwiPHandleDmscIntr);
    
    /* Invoking a firewall exception notification for the dmsc exception handler by writing to dmsc address */
    *excpRegDmsc = 0x01;

    Osal_delay(1U); /*Delay of 1ms*/

    /* Invoking a firewall exception notification for the cmbn exception handler by writing to dmsc address */
    *excpRegCmbn = 0x01;

    if (gInterruptRecieved!=EXCEPTION_INTERRUPT_CNT)
    {
        status = CSL_EFAIL;
    }
    return status;

}
#endif

#if ((defined (SOC_J721S2) || defined (SOC_J784S4)) && defined(BUILD_MCU2_0))
static void App_pvu2R5IntrTestIsr(void)
{
    gAppIsrExecNum++;

    /* Clear the PVU interrupt by writing 1 to pend clear register */
    CSL_REG32_WR(CSL_NAVSS0_IO_PVU0_CFG_MMRS_BASE+CSL_PVU_EXCEPTION_PEND_CLEAR, 1);

    /* Disable the PVU interrupt by writing 1 to enable clear register */
    CSL_REG32_WR(CSL_NAVSS0_IO_PVU0_CFG_MMRS_BASE+CSL_PVU_EXCEPTION_ENABLE_CLEAR, 1);
}

static int32_t App_pvu2R5IntrTest(void)
{
    int32_t status = CSL_PASS;
    int32_t sciclient_init_status = CSL_PASS;
    int32_t pvu2R5_route_status = CSL_PASS;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL,
        0 /* isSecure = 0 un secured for all cores */
    };
    uint16_t intNum=0;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;
    struct tisci_msg_rm_get_resource_range_req  req;
    struct tisci_msg_rm_get_resource_range_resp res;
    struct tisci_msg_rm_irq_release_req rmIrqReqRel;
    memset(&req, 0, sizeof(req));
    memset(&res, 0, sizeof(res));
    memset(&rmIrqReqRel,0,sizeof(rmIrqReqRel));

    /* This is only needed as this test case is running back to back Sciclient
     * Init and de-inits.
     */
    while (gSciclientHandle.initCount != 0)
    {
        status = Sciclient_deinit();
    }
    status = Sciclient_init(&config);
    sciclient_init_status = status;

    if(sciclient_init_status == CSL_PASS)
    {
        App_sciclientPrintf("PVU to Main R5 Interrupt Test\n");

        /* Enable the PVU interrupt by writing 1 to enable set register */
        CSL_REG32_WR(CSL_NAVSS0_IO_PVU0_CFG_MMRS_BASE+CSL_PVU_EXCEPTION_ENABLE_SET, 1);

        req.type = TISCI_DEV_NAVSS0_INTR_0;
        req.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        status  = Sciclient_rmGetResourceRange(&req,
                                               &res,
                                               SCICLIENT_SERVICE_WAIT_FOREVER);
        
        if(status == CSL_PASS)
        {
            App_sciclientPrintf("Sciclient_rmGetResourceRange() execution is successful\n");
            status = Sciclient_rmIrqTranslateIrOutput(req.type,
                                                      res.range_start,
                                                      TISCI_DEV_R5FSS0_CORE0,
                                                      &intNum);
            if(status == CSL_PASS)
            {
                App_sciclientPrintf("Sciclient_rmIrqTranslateIrOutput() execution is successful and host interrupt number is %d\n", intNum);

                memset(&rmIrqReq, 0, sizeof(rmIrqReq));
                rmIrqReq.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID;
                rmIrqReq.valid_params  |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
                rmIrqReq.src_id         = TISCI_DEV_NAVSS0_PVU_0;
                rmIrqReq.src_index      = 0;
                rmIrqReq.dst_id         = TISCI_DEV_R5FSS0_CORE0;
                rmIrqReq.dst_host_irq   = intNum;
                rmIrqReq.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

                status                    = Sciclient_rmIrqSet(&rmIrqReq,
                                                            &rmIrqResp,
                                                            SCICLIENT_SERVICE_WAIT_FOREVER);
                pvu2R5_route_status       = status;
                if(status == CSL_PASS)
                {
                    App_sciclientPrintf("Sciclient_rmIrqSet() execution is successful\n");
                }
                else
                {
                    App_sciclientPrintf("Sciclient_rmIrqSet() has failed\n");
                }
            }
            else
            {
                App_sciclientPrintf("Sciclient_rmIrqTranslateIrOutput() has failed\n");
            }
        }
        else
        {
            App_sciclientPrintf("Sciclient_rmGetResourceRange() has failed\n");
        }

        /* Generates an interrupt from PVU and checks if the registered ISR is hit.
        * This helps in checking the correctness of PVU to R5 interrupt routing. */
        if(status == CSL_PASS)
        {
            OsalRegisterIntrParams_t intrPrmsCmbnIntr;
            HwiP_Handle hwiPHandleCmbnIntr;

            Osal_RegisterInterrupt_initParams(&intrPrmsCmbnIntr);
            intrPrmsCmbnIntr.corepacConfig.isrRoutine       = (void (*)(uintptr_t))App_pvu2R5IntrTestIsr;
            intrPrmsCmbnIntr.corepacConfig.corepacEventNum  = 0;
            intrPrmsCmbnIntr.corepacConfig.intVecNum        = rmIrqReq.dst_host_irq;
            status = Osal_RegisterInterrupt(&intrPrmsCmbnIntr, &hwiPHandleCmbnIntr);
            if(status == CSL_PASS)
            {
                App_sciclientPrintf("Interrupt registration is successful\n");
                /* Generate the PVU interrupt by writing 1 to pend set register */
                CSL_REG32_WR(CSL_NAVSS0_IO_PVU0_CFG_MMRS_BASE+CSL_PVU_EXCEPTION_PEND_SET, 1);
                if(gAppIsrExecNum == 1)
                {
                    App_sciclientPrintf("ISR is hit, hence PVU to Main R5 interrupt route is properly set\n");
                    status = CSL_PASS;
                }
                else
                {
                    App_sciclientPrintf("ISR is not hit, hence PVU to Main R5 interrupt route is not properly set\n");
                    /* Clear the PVU interrupt by writing 1 to pend clear register */
                    CSL_REG32_WR(CSL_NAVSS0_IO_PVU0_CFG_MMRS_BASE+CSL_PVU_EXCEPTION_PEND_CLEAR, 1);

                    /* Disable the PVU interrupt by writing 1 to enable clear register */
                    CSL_REG32_WR(CSL_NAVSS0_IO_PVU0_CFG_MMRS_BASE+CSL_PVU_EXCEPTION_ENABLE_CLEAR, 1);
                    status = CSL_EFAIL;
                }
            }
            else
            {
                App_sciclientPrintf("Interrupt registration has failed\n");
            }
        }

        if(pvu2R5_route_status == CSL_PASS)
        {
            App_sciclientPrintf("\nDeleting PVU to Main R5 Interrupt Route\n");
            
            rmIrqReqRel.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID;
            rmIrqReqRel.valid_params  |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
            rmIrqReqRel.src_id         = TISCI_DEV_NAVSS0_PVU_0;
            rmIrqReqRel.src_index      = 0;
            rmIrqReqRel.dst_id         = TISCI_DEV_R5FSS0_CORE0;
            rmIrqReqRel.dst_host_irq   = intNum;
            rmIrqReqRel.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
            gAppIsrExecNum = 0;

            status = Sciclient_rmIrqRelease(&rmIrqReqRel, SCICLIENT_SERVICE_WAIT_FOREVER);
            if(status == CSL_PASS)
            {
                App_sciclientPrintf("Sciclient_rmIrqRelease() execution is successful\n");
            }
            else
            {
                App_sciclientPrintf("Sciclient_rmIrqRelease() has failed\n");
            }
        }
    }
    else
    {
        App_sciclientPrintf("Sciclient_init() has failed\n");
    }

    if (sciclient_init_status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
        
    return status;
}
#endif

#if ((defined (SOC_J721S2) || defined (SOC_J784S4)) && defined(BUILD_MCU2_0))
static int32_t App_pvu2GICIntrTest(void)
{
    int32_t status = CSL_PASS;
    int32_t sciclient_init_status = CSL_PASS;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL,
        0 /* isSecure = 0 un secured for all cores */
    };
    uint16_t intNum=0;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;
    struct tisci_msg_rm_get_resource_range_req  req;
    struct tisci_msg_rm_get_resource_range_resp res;
    memset(&req, 0, sizeof(req));
    memset(&res, 0, sizeof(res));

    /* This is only needed as this test case is running back to back Sciclient
     * Init and de-inits.
     */
    while (gSciclientHandle.initCount != 0)
    {
        status = Sciclient_deinit();
    }
    status = Sciclient_init(&config);
    sciclient_init_status = status;

    if(sciclient_init_status == CSL_PASS)
    {
        App_sciclientPrintf("PVU to GIC Interrupt Test\n");

        req.type = TISCI_DEV_NAVSS0_INTR_0;
        req.secondary_host = TISCI_HOST_ID_A72_2;

        status  = Sciclient_rmGetResourceRange(&req,
                                               &res,
                                               SCICLIENT_SERVICE_WAIT_FOREVER);        
        if(status == CSL_PASS)
        {
            App_sciclientPrintf("Sciclient_rmGetResourceRange() execution is successful\n");
            status = Sciclient_rmIrqTranslateIrOutput(req.type,
                                                      res.range_start,
                                                      TISCI_DEV_COMPUTE_CLUSTER0_GIC500SS,
                                                      &intNum);
            if(status == CSL_PASS)
            {
                App_sciclientPrintf("Sciclient_rmIrqTranslateIrOutput() execution is successful and host interrupt number is %d\n", intNum);

                memset(&rmIrqReq, 0, sizeof(rmIrqReq));
                rmIrqReq.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID;
                rmIrqReq.valid_params  |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
                rmIrqReq.valid_params  |= TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID;
                rmIrqReq.src_id         = TISCI_DEV_NAVSS0_PVU_0;
                rmIrqReq.src_index      = 0;
                rmIrqReq.dst_id         = TISCI_DEV_COMPUTE_CLUSTER0_GIC500SS;
                rmIrqReq.dst_host_irq   = intNum;
                rmIrqReq.secondary_host = TISCI_HOST_ID_A72_2;

                status                    = Sciclient_rmIrqSet(&rmIrqReq,
                                                            &rmIrqResp,
                                                            SCICLIENT_SERVICE_WAIT_FOREVER);
                if(status == CSL_PASS)
                {
                    App_sciclientPrintf("Sciclient_rmIrqSet() execution is successful\n");
                }
                else
                {
                    App_sciclientPrintf("Sciclient_rmIrqSet() has failed\n");
                }
            }
            else
            {
                App_sciclientPrintf("Sciclient_rmIrqTranslateIrOutput() has failed\n");
            }
        }
        else
        {
            App_sciclientPrintf("Sciclient_rmGetResourceRange() has failed\n");
        }
    }
    else
    {
        App_sciclientPrintf("Sciclient_init() has failed\n");
    }

    if (sciclient_init_status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
        
    return status;
}
#endif

#if ((defined (SOC_J721E) || defined (SOC_J7200) || defined (SOC_J721S2) || defined (SOC_J784S4)) && defined (BUILD_MCU1_0))
static int32_t App_mainUart2MCUR5IntrTest(void)
{
    int32_t status = CSL_PASS;
    int32_t sciclientInitStatus = CSL_PASS;
    int32_t uartWriteStatus = CSL_PASS;
    int32_t mainUart2MCUR5RouteStatus = CSL_PASS;
    uint32_t mainUartTestInstance;
    const UART_Params userParams = {
        UART_MODE_BLOCKING,     /* readMode */
        UART_MODE_BLOCKING,     /* writeMode */
        SemaphoreP_WAIT_FOREVER,/* readTimeout */
        SemaphoreP_WAIT_FOREVER,/* writeTimeout */
        NULL,                  /* readCallback */
        NULL,                 /* writeCallback */
        UART_RETURN_NEWLINE,  /* readReturnMode */
        UART_DATA_TEXT,       /* readDataMode */
        UART_DATA_TEXT,       /* writeDataMode */
        UART_ECHO_ON,         /* readEcho */
        115200,               /* baudRate */
        UART_LEN_8,           /* dataLength */
        UART_STOP_ONE,        /* stopBits */
        UART_PAR_NONE         /* parityType */
    };
    struct UART_HWAttrs uartHwAttrs;
    UART_Handle uartHandle = NULL;
    char echoPrompt[] = "Testing Main UART to MCU R5F routing\n";

    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;
    struct tisci_msg_rm_get_resource_range_req  req;
    struct tisci_msg_rm_get_resource_range_resp res;
    struct tisci_msg_rm_irq_release_req rmIrqReqRel;

    uint16_t intNum = 0;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL,
        0 /* isSecure = 0 un secured for all cores */
    };
    /* This is only needed as this test case is running back to back Sciclient
     * Init and de-inits.
     */
    while (gSciclientHandle.initCount != 0)
    {
        status = Sciclient_deinit();
    }
    status = Sciclient_init(&config);
    sciclientInitStatus = status;

    memset(&req, 0, sizeof(req));
    memset(&res, 0, sizeof(res));
    memset(&rmIrqReqRel,0,sizeof(rmIrqReqRel));
    mainUartTestInstance = BOARD_UART_INSTANCE;
    if(BOARD_UART_INSTANCE == 0)
    {
        mainUartTestInstance = App_getMainUartTestInstance();
    }
    /* Sets the Main UART configuration */
    UART_socGetInitCfg(BOARD_UART_INSTANCE, &uartHwAttrs);
    App_setMainUartHwAttrs(BOARD_UART_INSTANCE, &uartHwAttrs);
    
    (uartHwAttrs).configSocIntrPath = NULL;
    /* Programs the interrupt route between Main domain UART and MCU R5F core */
    req.type = TISCI_DEV_MAIN2MCU_LVL_INTRTR0;
    req.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
    status  = Sciclient_rmGetResourceRange(&req,
                                        &res,
                                        SCICLIENT_SERVICE_WAIT_FOREVER);    
    if(status == CSL_PASS)
    {
        status = Sciclient_rmIrqTranslateIrOutput(req.type,
                                                res.range_start,
                                                TISCI_DEV_MCU_R5FSS0_CORE0,
                                                &intNum);
        if(status == CSL_PASS)
        {
            memset(&rmIrqReq, 0, sizeof(rmIrqReq));
            rmIrqReq.valid_params       = TISCI_MSG_VALUE_RM_DST_ID_VALID;
            rmIrqReq.valid_params      |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
            rmIrqReq.src_id             = App_getUartSrcID(uartHwAttrs.baseAddr);
            rmIrqReq.src_index          = 0;
            rmIrqReq.dst_id             = TISCI_DEV_MCU_R5FSS0_CORE0;
            rmIrqReq.dst_host_irq       = intNum;
            rmIrqReq.secondary_host     = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
            status                      = Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SCICLIENT_SERVICE_WAIT_FOREVER);
            mainUart2MCUR5RouteStatus = status;
        }
    }
    (uartHwAttrs).intNum = intNum;
    (uartHwAttrs).enableInterrupt = 1;

    UART_socSetInitCfg(mainUartTestInstance, &uartHwAttrs);

    uartHandle = UART_open(mainUartTestInstance, (UART_Params*) &userParams);

    uartWriteStatus = UART_write(uartHandle, echoPrompt, sizeof(echoPrompt));

    if(mainUart2MCUR5RouteStatus == CSL_PASS)
    {
        /* Release the Route between Main UART and MCU R5F */
        rmIrqReqRel.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID;
        rmIrqReqRel.valid_params  |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
        rmIrqReqRel.src_id         = App_getUartSrcID(uartHwAttrs.baseAddr);
        rmIrqReqRel.src_index      = 0;
        rmIrqReqRel.dst_id         = TISCI_DEV_MCU_R5FSS0_CORE0;
        rmIrqReqRel.dst_host_irq   = intNum;
        rmIrqReqRel.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        status = Sciclient_rmIrqRelease(&rmIrqReqRel, SCICLIENT_SERVICE_WAIT_FOREVER);
    }
    UART_close(uartHandle);

    if (sciclientInitStatus == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
    if(uartWriteStatus >= 0)
    {
        uartWriteStatus = CSL_PASS;
    }
    else
    {
        uartWriteStatus =  CSL_EFAIL;
    }
    return uartWriteStatus;
}
#endif

#if defined(BUILD_MPU) || defined (BUILD_C7X)
extern void Osal_initMmuDefault(void);
void InitMmu(void)
{
    Osal_initMmuDefault();
}
#endif
