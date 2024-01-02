/*
 *  Copyright (C) 2017-2018 Texas Instruments Incorporated
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
 *  \file sciclient_firmware_boot_main.c
 *
 *  \brief Implementation of System firmware boot test
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <ti/csl/tistdtypes.h>
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/csl/hw_types.h>
#include <ti/osal/CacheP.h>
#include <ti/drv/sciclient/examples/common/sciclient_appCommon.h>
#include <ti/osal/TimerP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PRINT_UART

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile static uint64_t time_usecs[5] = { 0 };

volatile uint32_t loop = 1U;

uint32_t gSciclient_firmware[(SCICLIENT_FIRMWARE_SIZE_IN_BYTES + 3)/4] __attribute__((section(".firmware"))) = SCICLIENT_FIRMWARE;
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int32_t App_loadFirmwareTest(void);
void App_printPerfStats(void);
static int32_t App_boardCfgTest(void);
static int32_t App_getRevisionTestPol(void);
void _resetvectors (void);


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/**
 *  main
 *  Application main function.
 */
#if defined (SOC_J7200) || defined (SOC_J721S2) || defined (SOC_J784S4)
#define CSL_MCU_ARMSS_ATCM_BASE (0x0U)
#endif
int32_t main(void)
{
    int32_t status;
#if defined (SOC_J721E) || defined (SOC_J7200) || defined (SOC_J721S2) || defined (SOC_J784S4)
    /* Relocate CSL Vectors to ATCM*/
    memcpy((void *)CSL_MCU_ARMSS_ATCM_BASE, (void *)_resetvectors, 0x100);
#elif defined (SOC_AM65XX)
    /* Relocate CSL Vectors to ATCM*/
    memcpy((void *)CSL_MCU_ATCM_BASE, (void *)_resetvectors, 0x100);
#else
    /* Relocate CSL Vectors to ATCM*/
    memcpy((void *)CSL_R5FSS0_ATCM_BASE, (void *)_resetvectors, 0x100);
#endif
    status = App_loadFirmwareTest();
#ifdef PRINT_UART
    App_sciclientConsoleInit();
#endif
    if (status == CSL_PASS)
    {
        status = App_boardCfgTest();
    }
    if (status == CSL_PASS)
    {
        status = App_getRevisionTestPol();
    }
    #if defined(SOC_AM65XX)
    if (status == CSL_PASS)
    {
        App_printPerfStats();
    }
    #endif
#if defined (SOC_J721E) || defined (SOC_J7200) || defined (SOC_J721S2) || defined (SOC_J784S4)
    if (status == CSL_PASS)
    {
        App_sciclientPrintf("Trying a Domain Reset...\n");
        status = Sciclient_pmDomainReset(DOMGRP_01, SCICLIENT_SERVICE_WAIT_FOREVER); 
    }
#endif

    if (status == CSL_PASS)
    {
        App_sciclientPrintf("All tests have passed.\n");
    }
    else
    {
        App_sciclientPrintf("Some Test have failed\n");
    }
    
    while (loop) {;}
    
    return 0;
}

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

int32_t App_loadFirmwareTest(void)
{
    uint64_t start_ticks=0, stop_ticks=0;

    int32_t status = CSL_EFAIL;
    void *sysfw_ptr = gSciclient_firmware;

    sysfw_ptr = (void *)&gSciclient_firmware;

    /*Do a cache writeback*/
    CacheP_wbInv(sysfw_ptr, SCICLIENT_FIRMWARE_SIZE_IN_BYTES);

    start_ticks = TimerP_getTimeInUsecs();
    status = Sciclient_loadFirmware(sysfw_ptr);
    stop_ticks = TimerP_getTimeInUsecs();

    time_usecs[0U] = (stop_ticks-start_ticks);

    start_ticks=0U;
    stop_ticks=0U;

    return status;
}

int32_t App_getRevisionTestPol(void)
{
    uint64_t start_ticks=0, stop_ticks=0;

    int32_t status = CSL_EFAIL;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL
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
 

    start_ticks = TimerP_getTimeInUsecs();
    status = Sciclient_init(&config);
    stop_ticks = TimerP_getTimeInUsecs();

    time_usecs[1] = (stop_ticks-start_ticks);

    start_ticks=0U;
    stop_ticks=0U;

    if (status == CSL_PASS)
    {
        start_ticks = TimerP_getTimeInUsecs();
        status = Sciclient_service(&reqPrm, &respPrm);
        stop_ticks = TimerP_getTimeInUsecs();

        time_usecs[2] = (stop_ticks-start_ticks);

        start_ticks=0U;
        stop_ticks=0U;

        if (CSL_PASS == status)
        {
            if (respPrm.flags == TISCI_MSG_FLAG_ACK)
            {
                status = CSL_PASS;
#ifdef PRINT_UART
                App_sciclientPrintf(
                                  " DMSC Firmware Version %s\n",
                                  (char *) response.str);
                App_sciclientPrintf(
                                  " Firmware revision 0x%x\n", response.version);
                App_sciclientPrintf(
                                  " ABI revision %d.%d\n", response.abi_major,
                                  response.abi_minor);
#endif
            }
            else
            {
#ifdef PRINT_UART
                App_sciclientPrintf(
                                  " DMSC Firmware Get Version failed \n");
#endif
                status = CSL_EFAIL;
            }
        }
        else
        {
#ifdef PRINT_UART
            App_sciclientPrintf(
                              " DMSC Firmware Get Version failed \n");
#endif
        }
    }
    if (status == CSL_PASS)
    {
        start_ticks = TimerP_getTimeInUsecs();
        status = Sciclient_deinit();
        stop_ticks = TimerP_getTimeInUsecs();

        time_usecs[3] = (stop_ticks-start_ticks);

        start_ticks=0U;
        stop_ticks=0U;
    }
    return status;
}

int32_t App_boardCfgTest(void)
{
    int32_t status = CSL_PASS;
    int32_t temp_status = CSL_EFAIL;

    uint64_t start_ticks=0U, stop_ticks=0U;

    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL
    };

    if (Sciclient_init(&config) == CSL_PASS)
    {
        start_ticks = TimerP_getTimeInUsecs();
        temp_status = Sciclient_boardCfg(NULL);
        stop_ticks = TimerP_getTimeInUsecs();
        time_usecs[4] = (stop_ticks-start_ticks);

        start_ticks=0U;
        stop_ticks=0U;

        if (temp_status != CSL_PASS)
        {
            status = CSL_EFAIL;
        }
        if ((status == CSL_PASS) && (Sciclient_boardCfgPm(NULL) == CSL_PASS))
        {
#ifdef PRINT_UART
            App_sciclientPrintf(
                              " Board configuration for PM test...PASSED \n");
#endif
        }
        else
        {
#ifdef PRINT_UART
            App_sciclientPrintf(
                              " Board configuration for PM test...FAILED \n");
#endif
            status = CSL_EFAIL;
        }

        if ((status == CSL_PASS) && (Sciclient_boardCfgRm(NULL) == CSL_PASS))
        {
#ifdef PRINT_UART
            App_sciclientPrintf(
                              " Board configuration for RM test...PASSED \n");
#endif
        }
        else
        {
#ifdef PRINT_UART
            App_sciclientPrintf(
                              " Board configuration for RM test...FAILED \n");
#endif
            status = CSL_EFAIL;
        }
        if ((status == CSL_PASS) && (Sciclient_boardCfgSec(NULL) == CSL_PASS))
        {
#ifdef PRINT_UART
            App_sciclientPrintf(
                              " Board configuration for SECURITY test......PASSED \n");
#endif
        }
        else
        {
#ifdef PRINT_UART
            App_sciclientPrintf(
                              " Board configuration for SECURITY test...FAILED \n");
#endif
            status = CSL_EFAIL;
        }
    }
    else
    {
#ifdef PRINT_UART
        App_sciclientPrintf(
                          "Sciclient init...FAILED \n");
#endif
        status = CSL_EFAIL;
    }
    Sciclient_deinit();
    return status;
}

void App_printPerfStats()
{
#ifdef PRINT_UART
    App_sciclientPrintf("\n======================================\n");
    App_sciclientPrintf("           PERFORMANCE OF APIS          \n");
    App_sciclientPrintf("======================================\n");

    App_sciclientPrintf("Sciclient_loadFirmware |   %d us    \n", (uint32_t) (time_usecs[0] & (0xFFFFFFFFU)));
    App_sciclientPrintf("Sciclient_init         |   %d us    \n", (uint32_t) (time_usecs[1] & (0xFFFFFFFFU)));
    App_sciclientPrintf("Sciclient_service      |   %d us    \n", (uint32_t) (time_usecs[2] & (0xFFFFFFFFU)));
    App_sciclientPrintf("Sciclient_deinit       |   %d us    \n", (uint32_t) (time_usecs[3] & (0xFFFFFFFFU)));
    App_sciclientPrintf("Sciclient_boardCfg     |   %d us    \n", (uint32_t) (time_usecs[4] & (0xFFFFFFFFU)));
    App_sciclientPrintf("======================================\n\n Note: Here Sciclient_service is done for getRevision(Polling).\n");
#endif
}
