/**
 *   Copyright (c) Texas Instruments Incorporated 2019
 *   All rights reserved.
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
 */

/**
 *  \file main_baremetal.c
 *
 *  \brief Main file for baremetal build
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdint.h>
#include <ti/board/board.h>

#include <ti/drv/vhwa/include/vhwa_m2mSde.h>
#include <ti/drv/vhwa/examples/include/vhwa_sde_api.h>
#include "vhwa_sde_cfg.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Input Offset between two test cases, used mainly for zebu/qt,
 * so that buffers can be loaded in one shot */
#define VHWA_SDE_TEST_IN_BUF_OFFSET     (8*1024*1024)

/* Output Offset between two test cases, used mainly for zebu/qt,
 * so that buffers can be saved in one shot */
#define VHWA_SDE_TEST_OUT_BUF_OFFSET    (16*1024*1024)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t App_init(void);
static void App_deInit(void);
static void App_udmaPrint(const char *str);
static void AppSde_Test(AppSde_TestParams *tObj);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*
* Application Buffers
*/
/* Buffers to store input and out data */
static uint64_t gSdeBaseSrcBuf  = (uint64_t)(0x90000000U);
static uint64_t gSdeRefSrcBuf   =
                        (uint64_t)(0xA0000000U);
static uint64_t gSdeOutBuf      =
                        (uint64_t)(0xB0000000U);
static uint32_t gSdeBaseSrcBufFreeIdx = 0u;
static uint32_t gSdeRefSrcBufFreeIdx = 0u;
static uint32_t gSdeOutBufFreeIdx = 0u;

uint32_t gConfScoreMap[][DMPAC_SDE_NUM_SCORE_MAP] =
{
    /* CS MAP 1  */
    {0x10, 0x3e, 0x45, 0x49, 0x4b, 0x5a, 0x5e, 0x7F/* unused set to maximum */},
};

static AppSde_TestCfg gAppSdeTestCfg[] =
{
    #include <ti/drv/vhwa/examples/include/vhwa_sde_test_cfg.h>
};

static AppSde_TestParams gAppSdeObj[] = VHWA_SDE_TIRTOS_CFG;

static struct Udma_DrvObj gSdeAppUdmaDrvObj;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int main(void)
{
    int32_t                 status;
    uint32_t                testCnt;
    AppSde_TestParams      *tObj = NULL;


    status = App_init();

    App_startTimer();

    if (FVID2_SOK == status)
    {
        for (testCnt = 0u; testCnt <
                (sizeof(gAppSdeObj) / sizeof(AppSde_TestParams)); testCnt ++)
        {
            tObj = &gAppSdeObj[testCnt];

            gSdeBaseSrcBufFreeIdx = 0u;
            gSdeRefSrcBufFreeIdx = 0u;
            gSdeOutBufFreeIdx = 0u;

            /* Only single Testcases are supported */
            if ((TRUE == tObj->isEnableTest) && (1U == tObj->numHandles))
            {
                App_print (" Starting Test %s\n", tObj->testName);
                AppSde_Test(tObj);
            }
        }
    }

    App_print (" Exiting \n");
    App_deInit();

    return(0);
}

static void AppSde_Test(AppSde_TestParams *tObj)
{
    int32_t            status;
    uint32_t           rCnt;
    uint32_t           refBuffSize = 0;
    uint32_t           baseBuffSize = 0;
    uint32_t           outBuffSize = 0;

    status = AppSde_Create(tObj, 0U);
    if (FVID2_SOK != status)
    {
        App_print(" SDE_TEST_APP: Create Failed\n");
        status = FVID2_EFAIL;
    }

    status = AppSde_SetParams(tObj, 0U);
    if (FVID2_SOK != status)
    {
        App_print(" SDE_TEST_APP: SetParams Failed \n");
    }

    if (FVID2_SOK == status)
    {
        AppSde_AllocBuffers(tObj, 0U,
                            gSdeBaseSrcBuf+gSdeBaseSrcBufFreeIdx, &baseBuffSize,
                            gSdeRefSrcBuf+gSdeRefSrcBufFreeIdx, &refBuffSize,
                            gSdeOutBuf+gSdeOutBufFreeIdx, &outBuffSize);
        gSdeBaseSrcBufFreeIdx += baseBuffSize;
        gSdeRefSrcBufFreeIdx += refBuffSize;
        gSdeOutBufFreeIdx += outBuffSize;
    }

    for (rCnt = 0u; (rCnt < tObj->repeatCnt) && (FVID2_SOK == status); rCnt ++)
    {
        AppSde_PrepareRequest(tObj, 0U);

        /* Submit Request*/
        status = AppSde_SubmitRequest(tObj, 0U);

        if (FVID2_SOK == status)
        {
            /* Wait for Request Completion */
            status = AppSde_WaitForComplRequest(tObj, 0U);
            if (FVID2_SOK != status)
            {
                App_print (" Failed to wait for request completion \n");
            }
        }
        else
        {
            App_print (" Failed to Submit Request \n");
        }

        if (FVID2_SOK != status)
        {
            break;
        }
        else
        {
            App_print (" Completed RepeatCnt %d\n\n", rCnt);
        }
    }

    AppSde_Delete(tObj, 0U);
}

static int32_t App_init(void)
{
    int32_t                 status;
    uint32_t                instId;
    Udma_DrvHandle          drvHandle = &gSdeAppUdmaDrvObj;
    Udma_InitPrms           udmaInitPrms;
    Board_initCfg           boardCfg;
    Fvid2_InitPrms initPrmsFvid2;

    boardCfg = BOARD_INIT_PINMUX_CONFIG | BOARD_INIT_UART_STDIO;
    Board_init(boardCfg);

    Fvid2InitPrms_init(&initPrmsFvid2);
    initPrmsFvid2.printFxn = App_print;
    status = Fvid2_init(&initPrmsFvid2);

    if (FVID2_SOK != status)
    {
        App_print(" main: FVID2_Init Failed !!!\r\n");
    }

    if (FVID2_SOK == status)
    {
        /* Initialize UDMA and get the handle, it will be used in both CRC layer,
           as well as in the driver */
        /* UDMA driver init */
        instId = UDMA_INST_ID_MAIN_0;
        UdmaInitPrms_init(instId, &udmaInitPrms);
        udmaInitPrms.printFxn = &App_udmaPrint;
        status = Udma_init(drvHandle, &udmaInitPrms);
        if(UDMA_SOK != status)
        {
            App_print("[Error] UDMA init failed!!\n");
            status = FVID2_EFAIL;
        }
    }

    status = AppSde_Init(drvHandle);
    if (FVID2_SOK == status)
    {
        /* Initialize the UDMA channel for CRC */
        status = AppSde_CrcInit(drvHandle);
    }

    return (status);
}

static void App_deInit(void)
{
    int32_t         status;
    Udma_DrvHandle  drvHandle = &gSdeAppUdmaDrvObj;

    Vhwa_m2mSdeDeInit();

    AppSde_CrcDeinit(drvHandle);

    status = Udma_deinit(drvHandle);
    if(UDMA_SOK != status)
    {
        App_print("[Error] UDMA deinit failed!!\n");
    }

    Fvid2_deInit(NULL);
}

static void App_udmaPrint(const char *str)
{
    App_print(str);

    return;
}
