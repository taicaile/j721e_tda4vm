/**
 *   Copyright (c) Texas Instruments Incorporated 2018
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
*  \file vhwa_msc_api.c
*
*  \brief VHWA MSC sample application
*/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdint.h>
#include <ti/drv/vhwa/vhwa_msc.h>
#include <ti/drv/vhwa/examples/include/vhwa_msc_api.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int32_t AppMsc_CompareCrc(App_MscTestParams *tObj, uint32_t hIdx);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
App_MscTestObj gAppMscTestObj[APP_MSC_MAX_INSTANCES][APP_MSC_MAX_HANDLES];
AppCrc_hdlPrms gMscCrcChHandle;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t AppMsc_Init(Udma_DrvHandle drvHandle)
{
    int32_t status = FVID2_EBADARGS;
    uint32_t cnt, idx;
    Vhwa_M2mMscInitParams initPrms;
    Vhwa_M2mMscSl2AllocPrms sl2Prms;

    Vhwa_m2mMscInitParamsInit(&initPrms);

    initPrms.drvHandle = drvHandle;

    status = Vhwa_m2mMscInit(&initPrms);

    if (FVID2_SOK == status)
    {
        for(cnt = 0; cnt < VHWA_M2M_MSC_MAX_INST; cnt++)
        {
            for(idx = 0; idx < VHWA_M2M_MSC_MAX_IN_CHANNEL; idx++)
            {
                sl2Prms.maxInWidth[cnt][idx]    = APP_MAX_IN_IMG_WIDTH;
                sl2Prms.inCcsf[cnt][idx]        = APP_IN_IMG_CCSF;
                sl2Prms.inBuffDepth[cnt][idx]   = APP_MAX_IN_IMG_BUFF_DEFPTH;
            }
        }

        for(cnt = 0; cnt < MSC_MAX_OUTPUT; cnt++)
        {
            sl2Prms.maxOutWidth[cnt]   = APP_MAX_OUT_IMG_WIDTH;
            sl2Prms.outCcsf[cnt]       = APP_OUT_IMG_CCSF;
            sl2Prms.outBuffDepth[cnt]  = APP_MAX_OUT_IMG_BUFF_DEFPTH;
        }
        status = Vhwa_m2mMscAllocSl2(&sl2Prms);
    }

    if(FVID2_SOK != status)
    {
        App_print(" MSC_TEST_APP: MSC Init Failed\n");
    }

    return (status);
}

int32_t AppMsc_DeInit(void)
{
    int32_t retVal;

    retVal = Vhwa_m2mMscDeInit();

    return (retVal);
}

int32_t AppMscFrameComplCb0(Fvid2_Handle handle, void *appData)
{
    App_MscTestObj *tObj = (App_MscTestObj *)appData;

    if (NULL != tObj)
    {
        SemaphoreP_post(tObj->waitForProcessCmpl);
    }

    return FVID2_SOK;
}

int32_t AppMscFrameComplCb1(Fvid2_Handle handle, void *appData)
{
    App_MscTestObj *tObj = (App_MscTestObj *)appData;

    if (NULL != tObj)
    {
        SemaphoreP_post(tObj->waitForProcessCmpl);
    }

    return FVID2_SOK;
}

void AppMscErrorCb(Fvid2_Handle handle, uint32_t errEvents, void *appData)
{
    App_MscTestObj *tObj = (App_MscTestObj *)appData;

    if (NULL != tObj)
    {
        if(errEvents & VHWA_MSC_VBUSM_RD_ERR)
        {
            /* SL2 RD Error */
            errEvents = (errEvents & (~VHWA_MSC_VBUSM_RD_ERR));
        }
        else if(errEvents & VHWA_MSC_SL2_WR_ERR)
        {
            /* SL2 WR Error */
            errEvents = (errEvents & (~VHWA_MSC_SL2_WR_ERR));
        }
    }
}

int32_t AppMsc_Create(App_MscTestParams *tObj, uint32_t hndlIdx)
{
    int32_t             status;
    SemaphoreP_Params   params;
    App_MscTestObj *appObj = &gAppMscTestObj[tObj->testCfg[hndlIdx]->mscThreadId][hndlIdx];

    SemaphoreP_Params_init(&params);
    params.mode = SemaphoreP_Mode_BINARY;
    appObj->waitForProcessCmpl = SemaphoreP_create(0U, &params);


    if (NULL != appObj->waitForProcessCmpl)
    {
        if (TRUE == tObj->testCfg[hndlIdx]->isLsePsaAvail)
        {
            appObj->createArgs.enablePsa = TRUE;
        }
        else
        {
            appObj->createArgs.enablePsa = FALSE;
        }

        if(tObj->testCfg[hndlIdx]->mscThreadId == (uint32_t)VPAC_MSC_TEST_INST_ID_0)
        {
            appObj->cbPrms.cbFxn   = AppMscFrameComplCb0;
            appObj->cbPrms.appData = appObj;
        }
        else
        {
            appObj->cbPrms.cbFxn   = AppMscFrameComplCb1;
            appObj->cbPrms.appData = appObj;
        }

        if(tObj->isPerformanceTest)
        {
            appObj->createArgs.getTimeStamp = App_getTimerTicks;
        }

        appObj->handle = Fvid2_create(FVID2_VHWA_M2M_MSC_DRV_ID,
                            tObj->testCfg[hndlIdx]->mscThreadId,
                            &appObj->createArgs,
                            NULL,
                            &appObj->cbPrms);
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

void AppMsc_Delete(App_MscTestParams *tObj, uint32_t hndlIdx)
{
    App_MscTestObj *appObj = &gAppMscTestObj[tObj->testCfg[hndlIdx]->mscThreadId][hndlIdx];

    if (NULL != appObj->handle)
    {
        Fvid2_delete(appObj->handle, NULL);
    }

    if (NULL != appObj->waitForProcessCmpl)
    {
        SemaphoreP_delete(appObj->waitForProcessCmpl);
    }
}

int32_t AppMsc_SetParams(App_MscTestParams *tObj, uint32_t hndlIdx)
{
    int32_t status, cnt;
    App_MscTestCfg *tCfg = tObj->testCfg[hndlIdx];
    App_MscTestObj *appObj = &gAppMscTestObj[tObj->testCfg[hndlIdx]->mscThreadId][hndlIdx];
    Vhwa_M2mMscParams *mscPrms;
    Msc_ScConfig *scCfg;
    Msc_ErrEventParams  errPrms;

    mscPrms = &appObj->mscPrms[hndlIdx];

    mscPrms->loopBack = tCfg->loopBack;

    mscPrms->enableLineSkip = tCfg->inFrm.skipInputLine;
    mscPrms->inFmt.width = tCfg->inFrm.inWidth;
    mscPrms->inFmt.height = tCfg->inFrm.inHeight;
    mscPrms->inFmt.dataFormat = tCfg->inFrm.inDataFmt;
    mscPrms->inFmt.ccsFormat = tCfg->inFrm.inCcsf;
    mscPrms->inFmt.pitch[0U] = tCfg->inFrm.inPitch;
    mscPrms->mscCfg.tapSel = tCfg->inFrm.tapSel;

#if defined(SOC_J721S2) || defined (SOC_J784S4)
    mscPrms->secChPrms.enable = tCfg->inSecFrm.enable;
    mscPrms->secChPrms.pitch = tCfg->inSecFrm.inPitch;
    mscPrms->secChPrms.ccsf = tCfg->inSecFrm.inCcsf;
#endif

    for(cnt = 0; cnt < MSC_MAX_OUTPUT;cnt++)
    {
        scCfg = &mscPrms->mscCfg.scCfg[cnt];
        if(tCfg->mscCfgPrms[cnt].enable == TRUE)
        {
            scCfg->enable = TRUE;

            mscPrms->outFmt[cnt].width = tCfg->outFrm[cnt].outWidth;
            mscPrms->outFmt[cnt].height = tCfg->outFrm[cnt].outHeight;
            mscPrms->outFmt[cnt].dataFormat = tCfg->outFrm[cnt].outDataFmt;
            mscPrms->outFmt[cnt].ccsFormat = tCfg->outFrm[cnt].outCcsf;
            mscPrms->outFmt[cnt].pitch[0U] = tCfg->outFrm[cnt].outPitch;

            scCfg->inRoi.cropStartX = tCfg->mscCfgPrms[cnt].inPos.startX;
            scCfg->inRoi.cropStartY = tCfg->mscCfgPrms[cnt].inPos.startY;
            scCfg->inRoi.cropWidth = tCfg->outFrm[cnt].roiWidth;
            scCfg->inRoi.cropHeight = tCfg->outFrm[cnt].roiHeight;
            scCfg->outWidth = tCfg->outFrm[cnt].outWidth;
            scCfg->outHeight = tCfg->outFrm[cnt].outHeight;
            scCfg->horzAccInit = tCfg->mscCfgPrms[cnt].horzAccInit;
            scCfg->vertAccInit = tCfg->mscCfgPrms[cnt].vertAccInit;
            scCfg->filtMode = tCfg->mscCfgPrms[cnt].filtMode;
            scCfg->phaseMode = tCfg->mscCfgPrms[cnt].phaseMode;
            scCfg->hsSpCoeffSel = tCfg->mscCfgPrms[cnt].hsSpCoeffSel;
            scCfg->vsSpCoeffSel = tCfg->mscCfgPrms[cnt].vsSpCoeffSel;
            scCfg->hsMpCoeffSel = tCfg->mscCfgPrms[cnt].hsMpCoeffSel;
            scCfg->vsMpCoeffSel = tCfg->mscCfgPrms[cnt].vsMpCoeffSel;
            scCfg->coeffShift = tCfg->mscCfgPrms[cnt].coeffShift;
            scCfg->isSignedData = tCfg->mscCfgPrms[cnt].isSignedData;
            scCfg->isEnableFiltSatMode = tCfg->mscCfgPrms[cnt].isEnableFiltSatMode;
        }
        else
        {
            scCfg->enable = FALSE;
        }
    }

    mscPrms->loopBack = tCfg->loopBack;

    status = Fvid2_control(appObj->handle, VHWA_M2M_IOCTL_MSC_SET_PARAMS,
                            mscPrms, NULL);
    if (FVID2_SOK == status)
    {
        errPrms.errEvents =
            VHWA_MSC_VBUSM_RD_ERR | VHWA_MSC_SL2_WR_ERR;

        errPrms.cbFxn = AppMscErrorCb;

        errPrms.appData = appObj;

        status = Fvid2_control(appObj->handle,
            VHWA_M2M_IOCTL_MSC_REGISTER_ERR_CB, &errPrms, NULL);
    }

    return (status);
}

int32_t AppMsc_AllocBuffers(App_MscTestParams *tObj, uint32_t hndlIdx,
    uint64_t srcBuf, uint32_t *srcBufSize, uint32_t srcOffset,
    uint64_t dstBuf, uint32_t *dstBufSize, uint32_t dstOffset)
{
    uint32_t  cnt;
    App_MscTestCfg *tCfg = tObj->testCfg[hndlIdx];
    App_MscTestObj *appObj = &gAppMscTestObj[tObj->testCfg[hndlIdx]->mscThreadId][hndlIdx];

    /* Change to allocation based on image size after bring up */
    *srcBufSize = 0u;

    appObj->inFrm.addr[0U] = srcBuf;
    appObj->inFrm.addr[1U] = (srcBuf +
                        (tCfg->inFrm.inPitch * tCfg->inFrm.inHeight));

    srcBuf += srcOffset;
    *srcBufSize += srcOffset;

    App_print(" MSC Input BufferAddress 0x%x, 0x%x\n",
            (uint32_t)appObj->inFrm.addr[0U],
            (uint32_t)appObj->inFrm.addr[1U]);

    *dstBufSize = 0u;
    for(cnt = 0; cnt < MSC_MAX_OUTPUT; cnt++)
    {
        if(tCfg->mscCfgPrms[cnt].enable == TRUE)
        {
            GT_assert(VhwaMscTrace, (((tCfg->outFrm[cnt].outPitch *
                        tCfg->outFrm[cnt].outHeight * 3) / 2) <= dstOffset));

            appObj->outFrm[cnt].addr[0U] = dstBuf;

            if(FVID2_DF_YUV420SP_UV == tCfg->outFrm[cnt].outDataFmt ||
               FVID2_DF_YUV420SP_VU == tCfg->outFrm[cnt].outDataFmt)
            {
                appObj->outFrm[cnt].addr[1U] = (dstBuf +
                        (tCfg->outFrm[cnt].outPitch * tCfg->outFrm[cnt].outHeight));

            }
            *dstBufSize += dstOffset;
            dstBuf += dstOffset;

            App_print(" MSC Output BufferAddress 0x%x, 0x%x\n",
                    (uint32_t)appObj->outFrm[cnt].addr[0U],
                    (uint32_t)appObj->outFrm[cnt].addr[1U]);
        }
    }

    return (FVID2_SOK);
}

int32_t AppMsc_SetCoeff(App_MscTestParams *tObj, uint32_t hndlIdx,
                        Msc_Coeff * coefTbl)
{
    int32_t status;
    App_MscTestObj *appObj = &gAppMscTestObj[tObj->testCfg[hndlIdx]->mscThreadId][hndlIdx];

    status = Fvid2_control(appObj->handle, VHWA_M2M_IOCTL_MSC_SET_COEFF,
        coefTbl, NULL);

    return (status);
}

int32_t AppMsc_CrcInit(Udma_DrvHandle udmaDrvHndl)
{
    int32_t status = FVID2_SOK;

    gMscCrcChHandle.crcChannel = APP_MSC_CRC_CHANNEL;
    gMscCrcChHandle.udmaCrcChHandle = App_crcCreate(udmaDrvHndl,
                                                 &gMscCrcChHandle.crcChannel);
    if(gMscCrcChHandle.udmaCrcChHandle == NULL)
    {
        App_print(" App_crcCreate Error\n");
        status = FVID2_EFAIL;
    }

    return status;
}

int32_t AppMsc_CrcDeinit(Udma_DrvHandle udmaDrvHndl)
{
    int32_t status = FVID2_SOK;

    App_crcDelete(udmaDrvHndl, &gMscCrcChHandle);

    return status;
}

static int32_t AppMsc_CompareCrc(App_MscTestParams *tObj, uint32_t hIdx)
{
    int32_t status = FVID2_SOK;
    uint64_t crcSignVal = 0;
    uint32_t outCnt;
    App_MscTestCfg *tCfg;
    AppCrc_inPrms srcPrms;
    App_MscTestObj *appObj = &gAppMscTestObj[tObj->testCfg[hIdx]->mscThreadId][hIdx];

    tCfg = tObj->testCfg[hIdx];

    if (TRUE == tCfg->isCrcAvail)
    {
        for (outCnt = 0u; outCnt < 10u && status == FVID2_SOK; outCnt ++)
        {
            if(tCfg->mscCfgPrms[outCnt].enable == TRUE)
            {
                srcPrms.width = Vhwa_calcHorzSizeInBytes(tCfg->outFrm[outCnt].outWidth,
                                                    tCfg->outFrm[outCnt].outCcsf);
                srcPrms.pitch = tCfg->outFrm[outCnt].outPitch;

                if(tCfg->outFrm[outCnt].outDataFmt == FVID2_DF_YUV420SP_UV ||
                    tCfg->outFrm[outCnt].outDataFmt == FVID2_DF_YUV420SP_VU)
                {
                    srcPrms.height = tCfg->outFrm[outCnt].outHeight*3/2;
                }
                else if(tCfg->outFrm[outCnt].outDataFmt == FVID2_DF_YUV422I_UYVY ||
                    tCfg->outFrm[outCnt].outDataFmt == FVID2_DF_YUV422I_YUYV ||
                    tCfg->outFrm[outCnt].outDataFmt == FVID2_DF_YUV422SP_UV ||
                    tCfg->outFrm[outCnt].outDataFmt == FVID2_DF_YUV422SP_VU)
                {
                    srcPrms.height = tCfg->outFrm[outCnt].outHeight*2;
                }
                else
                {
                    srcPrms.height = tCfg->outFrm[outCnt].outHeight;
                }

                status = App_getCrc(&gMscCrcChHandle,
                                    (uint64_t)appObj->outFrm[outCnt].addr[0U],
                                    &srcPrms, &crcSignVal);
                if (FVID2_SOK == status)
                {
                    if (crcSignVal != tCfg->crcSign[outCnt])
                    {
                        status = FVID2_EFAIL;
                    }
                }
            }
        }
    }

    return (status);
}


void AppMsc_PrepareRequest(App_MscTestParams *tObj, uint32_t hndlIdx)
{
    uint32_t            mscCnt;
    App_MscTestCfg     *mCfg;
    Fvid2_FrameList    *inFrmList;
    Fvid2_FrameList    *outFrmList;
    App_MscTestObj *appObj = &gAppMscTestObj[tObj->testCfg[hndlIdx]->mscThreadId][hndlIdx];

    inFrmList = &appObj->inFrmList;
    outFrmList = &appObj->outFrmList;

    mCfg = tObj->testCfg[hndlIdx];

    App_print(" MSC%d Handle %d, Width = %d",
        mCfg->mscThreadId, hndlIdx,
        mCfg->inFrm.inWidth);
    App_print(", Height = %d\n",
        mCfg->inFrm.inHeight);

    inFrmList->frames[0] = &appObj->inFrm;

    outFrmList->numFrames = 0u;
    for(mscCnt = 0; mscCnt < 10; mscCnt ++)
    {
        outFrmList->frames[mscCnt] = &appObj->outFrm[mscCnt];
    }

    inFrmList->numFrames = 1U;
    outFrmList->numFrames = 10u;
}

int32_t AppMsc_SubmitRequest(App_MscTestParams *tObj, uint32_t hndlIdx)
{
    int32_t status;
    Fvid2_FrameList    *inFrmList;
    Fvid2_FrameList    *outFrmList;
    App_MscTestObj *appObj = &gAppMscTestObj[tObj->testCfg[hndlIdx]->mscThreadId][hndlIdx];

    inFrmList  = &appObj->inFrmList;
    outFrmList = &appObj->outFrmList;

    /* Submit MSC Request*/
    status = Fvid2_processRequest(appObj->handle, inFrmList, outFrmList,
                                  FVID2_TIMEOUT_FOREVER);
    if (FVID2_SOK != status)
    {
        App_print
            (" MSC: Fvid2_processRequest returned %x for MSC Handle # %d\n",
            status, hndlIdx);
    }

    return (status);
}

int32_t AppMsc_WaitForComplRequest(App_MscTestParams *tObj,
                                    uint32_t hndlIdx)
{
    int32_t status;
    Fvid2_FrameList    *inFrmList;
    Fvid2_FrameList    *outFrmList;
    App_MscTestObj *appObj = &gAppMscTestObj[tObj->testCfg[hndlIdx]->mscThreadId][hndlIdx];

    SemaphoreP_pend(appObj->waitForProcessCmpl, SemaphoreP_WAIT_FOREVER);

    inFrmList = &appObj->inFrmList;
    outFrmList = &appObj->outFrmList;

    status = Fvid2_getProcessedRequest(appObj->handle,
        inFrmList, outFrmList, 0);
    if (FVID2_SOK != status)
    {
        App_print
            (" MSC: Failed MSC Handle Cnt %d; status = %x\n", hndlIdx, status);
    }
    else
    {
        status = AppMsc_CompareCrc(tObj, hndlIdx);
        if (FVID2_SOK != status)
        {
            App_print (" MSC: CRC Check Failed Handle Cnt %d\n", hndlIdx);
        }
    }

    return (status);
}

void AppMsc_SyncStart(App_MscTestParams *tObj, uint32_t hidx)
{
    uint32_t status;
    App_MscTestObj  *appObj = &gAppMscTestObj[tObj->testCfg[hidx]->mscThreadId][hidx];

    status = Fvid2_control(appObj->handle,
                            VHWA_M2M_IOCTL_MSC_SYNC_START,
                            NULL, NULL);
    if(FVID2_SOK != status)
    {
        App_print(" MSC Sync Start Failed \n");
    }
}
