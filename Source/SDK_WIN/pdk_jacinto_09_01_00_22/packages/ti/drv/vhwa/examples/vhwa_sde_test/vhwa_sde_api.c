/*
*  Copyright (c) Texas Instruments Incorporated 2019
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
*  \file vhwa_sde_api.c
*
*  \brief VHWA SDE APIs
*/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdint.h>
#include <ti/drv/vhwa/include/vhwa_m2mSde.h>
#include <ti/drv/vhwa/examples/include/vhwa_sde_api.h>
#include <ti/drv/sciclient/include/sciclient_pm.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int32_t AppSde_CompareCrc(AppSde_TestParams *tObj, uint32_t hidx);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

AppSde_TestObj gAppSdeTestObj[APP_SDE_MAX_HANDLES];

AppCrc_hdlPrms gSdeCrcChHandle;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t AppSde_Init(Udma_DrvHandle udmaDrvHndl)
{
    int32_t                 status;
    Vhwa_M2mSdeSl2AllocPrms sl2AllocPrms;
    Vhwa_M2mSdeInitParams   initPrms;

    /* Powerup SDE module */
    Sciclient_pmSetModuleState (TISCI_DEV_DMPAC0_SDE_0,
            TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
            TISCI_MSG_FLAG_AOP,
            0xFFFFFFFFU);

    /* Initialize SDE Init parameters */
    Vhwa_M2mSdeInitParamsInit(&initPrms);

    /* Set UDMA driver handle */
    initPrms.udmaDrvHndl = udmaDrvHndl;

    status = Vhwa_m2mSdeInit(&initPrms);
    if (FVID2_SOK != status)
    {
        App_print(" SDE_TEST_APP: SDE Init Failed\n");
    }
    else
    {
        /* Initilize SL2 parameters */
        Vhwa_M2mSdeSl2AllocPrmsInit(&sl2AllocPrms);

        sl2AllocPrms.maxImgWidth  = APP_SDE_DEFAULT_IMG_WIDTH;
        sl2AllocPrms.inCcsf       = APP_SDE_DEFAULT_CCSF;
        sl2AllocPrms.searchRange  = APP_SDE_DEFAULT_SR;

        status = Vhwa_m2mSdeAllocSl2(&sl2AllocPrms);
        if (FVID2_SOK != status)
        {
            App_print(" SDE_TEST_APP: SL2 Alloc Failed !!!\n");
        }
    }

    return (status);
}

int32_t AppSdeFrameComplCb(Fvid2_Handle handle, void *appData)
{
    AppSde_TestObj *tObj = (AppSde_TestObj *)appData;

    if (NULL != tObj)
    {
        SemaphoreP_post(tObj->waitForProcessCmpl);
    }

    return FVID2_SOK;
}

void AppSdeErrorCb(Fvid2_Handle handle, uint32_t errEvents, void *appData)
{
    AppSde_TestObj *tObj = (AppSde_TestObj *)appData;

    if (NULL != tObj)
    {
        if(errEvents & VHWA_SDE_RD_ERR)
        {
            /* SL2 RD Error */
            errEvents = (errEvents & (~VHWA_SDE_RD_ERR));
        }
        else if(errEvents & VHWA_SDE_WR_ERR)
        {
            /* SL2 WR Error */
            errEvents = (errEvents & (~VHWA_SDE_WR_ERR));
        }
        else if(errEvents & VHWA_SDE_FOCO0_SL2_WR_ERR)
        {
            /* SL2 WR Error */
            errEvents = (errEvents & (~VHWA_SDE_FOCO0_SL2_WR_ERR));
        }
        else if(errEvents & VHWA_SDE_FOCO0_VBUSM_RD_ERR)
        {
            /* SL2 WR Error */
            errEvents = (errEvents & (~VHWA_SDE_FOCO0_VBUSM_RD_ERR));
        }
    }
}

int32_t AppSde_Create(AppSde_TestParams *tObj, uint32_t hidx)
{
    int32_t             status;
    SemaphoreP_Params   params;
    AppSde_TestObj      *appObj = &gAppSdeTestObj[hidx];

    Fvid2Utils_memset(appObj, 0x0, sizeof(AppSde_TestObj));

    SemaphoreP_Params_init(&params);
    params.mode = SemaphoreP_Mode_BINARY;
    appObj->waitForProcessCmpl = SemaphoreP_create(0U, &params);
    if (NULL != appObj->waitForProcessCmpl)
    {
        if (TRUE == tObj->testCfg[hidx]->enablePsa)
        {
            appObj->createArgs.enablePsa = TRUE;
        }
        else
        {
            appObj->createArgs.enablePsa = FALSE;
        }

        if(tObj->isPerformanceTest)
        {
            appObj->createArgs.getTimeStamp = App_getTimerTicks;
        }

        appObj->cbPrms.cbFxn   = AppSdeFrameComplCb;
        appObj->cbPrms.appData = appObj;

        appObj->handle = Fvid2_create(FVID2_VHWA_M2M_SDE_DRV_ID,
            VHWA_M2M_SDE_DRV_INST_ID, (void *)&appObj->createArgs,
            NULL, &appObj->cbPrms);
    }
    else
    {
        App_print (" Could not Create Semaphore \n");
    }

    if (NULL != appObj->handle)
    {
        status = FVID2_SOK;
    }
    else
    {
        status = FVID2_EFAIL;
    }

    return (status);
}


void AppSde_Delete(AppSde_TestParams *tObj, uint32_t hidx)
{
    AppSde_TestObj      *appObj = &gAppSdeTestObj[hidx];

    if (NULL != appObj->handle)
    {
        Fvid2_delete(appObj->handle, NULL);
    }

    if (NULL != appObj->waitForProcessCmpl)
    {
        SemaphoreP_delete(appObj->waitForProcessCmpl);
    }
}

int32_t AppSde_SetParams(AppSde_TestParams *tObj, uint32_t hidx)
{
    int32_t status;
    uint32_t idx;
    AppSde_TestCfg *tCfg;
    Vhwa_M2mSdePrms *sdePrms;
    AppSde_TestObj  *appObj = &gAppSdeTestObj[hidx];
    Sde_ErrEventParams  errPrms;

    tCfg = tObj->testCfg[hidx];
    sdePrms = &appObj->sdePrms;

    sdePrms->sdeCfg.enableSDE = tCfg->enableSDE;
    sdePrms->sdeCfg.medianFilter = tCfg->medianFilter;
    sdePrms->sdeCfg.width = tCfg->imgWidth;
    sdePrms->sdeCfg.height = tCfg->imgHeight;
    sdePrms->sdeCfg.minDisparity = tCfg->minDisparity;
    sdePrms->sdeCfg.searchRange = tCfg->searchRange;
    sdePrms->sdeCfg.lrThreshold = tCfg->lrThreshold;
    sdePrms->sdeCfg.enableTextureFilter = tCfg->enableTextureFilter;
    sdePrms->sdeCfg.textureFilterThreshold = tCfg->textureFilterThreshold;
    sdePrms->sdeCfg.penaltyP1 = tCfg->penaltyP1;
    sdePrms->sdeCfg.penaltyP2 = tCfg->penaltyP2;

    for(idx = 0; idx < DMPAC_SDE_NUM_SCORE_MAP; idx++)
    {
        sdePrms->sdeCfg.confScoreMap[idx] = tObj->sdeConfScore[hidx][idx];
    }

    sdePrms->inOutImgFmt[SDE_INPUT_REFERENCE_IMG].pitch[0U] = tCfg->refPitch;
    sdePrms->inOutImgFmt[SDE_INPUT_BASE_IMG].pitch[0U] = tCfg->basePitch;
    sdePrms->inOutImgFmt[SDE_OUTPUT].pitch[0U] = tCfg->outPitch;

    sdePrms->inOutImgFmt[SDE_INPUT_REFERENCE_IMG].ccsFormat = tCfg->imgStrFmt;
    sdePrms->inOutImgFmt[SDE_INPUT_BASE_IMG].ccsFormat = tCfg->imgStrFmt;

    if(tCfg->imgStrFmt == FVID2_CCSF_BITS12_UNPACKED16 ||
       tCfg->imgStrFmt == FVID2_CCSF_BITS8_PACKED)
    {
        sdePrms->focoPrms.shiftM1 = tCfg->focoCfg.shiftM1;
        sdePrms->focoPrms.dir = tCfg->focoCfg.dir;
        sdePrms->focoPrms.round = tCfg->focoCfg.round;
    }

    status = Fvid2_control(appObj->handle, VHWA_M2M_IOCTL_SDE_SET_PARAMS,
        sdePrms, NULL);

    if (FVID2_SOK == status)
    {
        errPrms.errEvents =
            VHWA_SDE_RD_ERR | VHWA_SDE_WR_ERR |
            VHWA_SDE_FOCO0_SL2_WR_ERR | VHWA_SDE_FOCO0_VBUSM_RD_ERR;

        errPrms.cbFxn = AppSdeErrorCb;

        errPrms.appData = appObj;

        status = Fvid2_control(appObj->handle,
            VHWA_M2M_IOCTL_SDE_REGISTER_ERR_CB, &errPrms, NULL);
    }

    return (status);
}


int32_t AppSde_AllocBuffers(AppSde_TestParams *tObj, uint32_t hidx,
                         uint64_t sdeBaseSrcBuf, uint32_t *baseSrcBuffSize,
                         uint64_t sdeRefSrcBuf, uint32_t *refSrcBuffSize,
                         uint64_t sdeDesBuf, uint32_t *desBuffSize)
{
    AppSde_TestCfg     *tCfg   = tObj->testCfg[hidx];
    AppSde_TestObj  *appObj = &gAppSdeTestObj[hidx];

    appObj->inFrm[SDE_INPUT_REFERENCE_IMG].addr[0] = sdeRefSrcBuf;
    appObj->inFrm[SDE_INPUT_BASE_IMG].addr[0] = sdeBaseSrcBuf;
    appObj->outFrm.addr[0] = sdeDesBuf;

    App_print("SDE Buffers\n");
    App_print("Ref Buff 0x%x\nBase Buff 0x%x\nDes Buff 0x%x\n",
            (uint32_t)appObj->inFrm[SDE_INPUT_REFERENCE_IMG].addr[0],
            (uint32_t)appObj->inFrm[SDE_INPUT_BASE_IMG].addr[0],
            (uint32_t)appObj->outFrm.addr[0]);

    *refSrcBuffSize  = tCfg->imgHeight * tCfg->refPitch;
    *baseSrcBuffSize = tCfg->imgHeight * tCfg->basePitch;
    *desBuffSize     = tCfg->imgHeight * tCfg->outPitch;

    return (FVID2_SOK);
}

int32_t AppSde_CrcInit(Udma_DrvHandle udmaDrvHndl)
{
    int32_t status = FVID2_SOK;

    gSdeCrcChHandle.crcChannel = APP_SDE_CRC_CHANNEL;
    gSdeCrcChHandle.udmaCrcChHandle = App_crcCreate(udmaDrvHndl,
                                                 &gSdeCrcChHandle.crcChannel);
    if(gSdeCrcChHandle.udmaCrcChHandle == NULL)
    {
        App_print(" App_crcCreate Error\n");
        status = FVID2_EFAIL;
    }

    return status;
}

int32_t AppSde_CrcDeinit(Udma_DrvHandle udmaDrvHndl)
{
    int32_t status = FVID2_SOK;

    App_crcDelete(udmaDrvHndl, &gSdeCrcChHandle);

    return status;
}

static int32_t AppSde_CompareCrc(AppSde_TestParams *tObj, uint32_t hidx)
{
    int32_t status = FVID2_SOK;
    uint64_t crcSignVal = 0;
    AppSde_TestCfg *tCfg;
    AppCrc_inPrms srcPrms;
    AppSde_TestObj  *appObj = &gAppSdeTestObj[hidx];


    tCfg = tObj->testCfg[hidx];

    if (TRUE == tCfg->isCrcAvail)
    {
        srcPrms.width = tCfg->imgWidth * 2;
        srcPrms.pitch = tCfg->outPitch;
        srcPrms.height = tCfg->imgHeight-4;

        status = App_getCrc(&gSdeCrcChHandle,
                            appObj->outFrm.addr[0],
                            &srcPrms, &crcSignVal);
        if (FVID2_SOK == status)
        {
            if (crcSignVal != tCfg->crcSign)
            {
                status = FVID2_EFAIL;
            }
        }
    }

    return (status);
}

void AppSde_PrepareRequest(AppSde_TestParams *tObj, uint32_t hidx)
{
    Fvid2_FrameList *inFrmList;
    Fvid2_FrameList *outFrmList;
    AppSde_TestObj *appObj = &gAppSdeTestObj[hidx];

    inFrmList = &appObj->inFrmList;
    outFrmList = &appObj->outFrmList;

    /* Initialize SDE Input Frame List */
    inFrmList->frames[SDE_INPUT_REFERENCE_IMG] =
                            &appObj->inFrm[SDE_INPUT_REFERENCE_IMG];
    inFrmList->frames[SDE_INPUT_BASE_IMG] =
                            &appObj->inFrm[SDE_INPUT_BASE_IMG];
    inFrmList->numFrames = 2U;

    /* Initialize SDE Output Frame List */
    outFrmList->frames[0U] = &appObj->outFrm;
    outFrmList->numFrames = 1U;
}

int32_t AppSde_SubmitRequest(AppSde_TestParams *tObj, uint32_t hidx)
{
    int32_t status;
    AppSde_TestObj *appObj = &gAppSdeTestObj[hidx];

    /* Submit Request*/
    status = Fvid2_processRequest(appObj->handle,
                                  &appObj->inFrmList,
                                  &appObj->outFrmList,
                                  FVID2_TIMEOUT_FOREVER);

    if (FVID2_SOK != status)
    {
        App_print
            (" SDE: Failed to submit new Frame returned %x for SDE Handle # %d\n",
            status, hidx);
    }

    return (status);
}

int32_t AppSde_WaitForComplRequest(AppSde_TestParams *tObj, uint32_t hidx)
{
    int32_t          status;
    Fvid2_FrameList *inFrmList;
    Fvid2_FrameList *outFrmList;
    AppSde_TestObj  *appObj = &gAppSdeTestObj[hidx];

    SemaphoreP_pend(appObj->waitForProcessCmpl, SemaphoreP_WAIT_FOREVER);

    inFrmList = &appObj->inFrmList;
    outFrmList = &appObj->outFrmList;

    status = Fvid2_getProcessedRequest(appObj->handle,
                                inFrmList, outFrmList, 0);
    if (FVID2_SOK == status)
    {
        status = AppSde_CompareCrc(tObj, hidx);
        if (FVID2_SOK != status)
        {
            App_print (" SDE: CRC Check Failed Handle Cnt %d\n", hidx);
        }
    }

    if (FVID2_SOK == status)
    {
        if(tObj->testCfg[hidx]->enableHistogram)
        {
            status = Fvid2_control(appObj->handle, VHWA_M2M_IOCTL_SDE_GET_HISTOGRAM,
                        appObj->csHistogram, NULL);
            if(FVID2_SOK != status)
            {
                App_print (" SDE: getHistogram failed Handle Cnt %d, status = %x\n",
                            hidx, status);
            }
        }
    }
    else
    {
        App_print (" SDE: Failed SDE Handle Cnt %d, status = %x\n",
            hidx, status);
    }

    return (status);
}

void AppSde_SyncStart(AppSde_TestParams *tObj, uint32_t hidx)
{
    uint32_t status;
    AppSde_TestObj  *appObj = &gAppSdeTestObj[hidx];

    status = Fvid2_control(appObj->handle,
                            VHWA_M2M_IOCTL_SDE_SYNC_START,
                            NULL, NULL);
    if(FVID2_SOK != status)
    {
        App_print(" SDE Sync Start Failed \n");
    }
}
