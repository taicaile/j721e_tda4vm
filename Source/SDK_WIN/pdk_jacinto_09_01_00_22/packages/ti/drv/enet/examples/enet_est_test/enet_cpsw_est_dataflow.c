/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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

/*!
 * \file  enet_cpsw_est_dataflow.c
 *
 * \brief This file contains the implementation of the APIs for data flow
 *        for the CPSW EST example app.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enet_cpsw_est_common.h"
#include "enet_cpsw_est_cfg.h"
#include "enet_cpsw_est_dataflow.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENETAPP_VLAN_TPID                            (0x8100U)
#define ENETAPP_VLAN_PCP_OFFSET                      (13U)
#define ENETAPP_VLAN_PCP_MASK                        (0x7U)
#define ENETAPP_VLAN_DEI_OFFSET                      (12U)
#define ENETAPP_VLAN_DEI_MASK                        (0x1U)
#define ENETAPP_VLAN_VID_MASK                        (0xFFFU)
#define ENETAPP_VLAN_TCI(pcp, dei, vid)              ((((pcp) & ENETAPP_VLAN_PCP_MASK) << ENETAPP_VLAN_PCP_OFFSET) | \
                                                      (((dei) & ENETAPP_VLAN_DEI_MASK) << ENETAPP_VLAN_DEI_OFFSET) | \
                                                      (((vid) & ENETAPP_VLAN_VID_MASK)))

/* Experimental EtherType used in TX test packets */
#define ENETAPP_TEST_TX_ETHERTYPE                    (0x88B5U)

/* TX test packet length (total length including L2 header is 64) */
#define ENETAPP_TEST_TX_PKT_PAYLOAD_LEN              (46U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetApp_rxIsr(void *appData);

static void EnetApp_txIsr(void *appData);

static void EnetApp_rxTask(void *args1, void *args2);

static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);

static void EnetApp_initTxFreePktQ(void);

static uint32_t EnetApp_retrieveFreeTxPkts(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void EnetApp_rxIsr(void *appData)
{
    EnetApp_Obj *enetApp = (EnetApp_Obj *)appData;

    SemaphoreP_post(enetApp->hRxSem);
}

static void EnetApp_txIsr(void *appData)
{
    /* Does nothing,
     * implemented this to enable pTxCh->evtInitFlag
     * by giving a dummy address to notifyCb
     */
}

int32_t EnetApp_openDma(void)
{
    int32_t status = ENET_SOK;
    EnetUdma_OpenRxFlowPrms rxChCfg;
    EnetUdma_OpenTxChPrms   txChCfg;

    /* Open the CPSW TX channel  */
    if (status == ENET_SOK)
    {
        EnetDma_initTxChParams(&txChCfg);

        txChCfg.hUdmaDrv = gEnetApp.hMainUdmaDrv;
        txChCfg.cbArg    = &gEnetApp;
        txChCfg.notifyCb = EnetApp_txIsr;
        txChCfg.useGlobalEvt = true;

        EnetAppUtils_setCommonTxChPrms(&txChCfg);

        EnetAppUtils_openTxCh(gEnetApp.hEnet,
                              gEnetApp.coreKey,
                              gEnetApp.coreId,
                              &gEnetApp.txChNum,
                              &gEnetApp.hTxCh,
                              &txChCfg);

        if (gEnetApp.hTxCh == NULL)
        {
#if FIX_RM
            /* Free the channel number if open Tx channel failed */
            EnetAppUtils_freeTxCh(gEnetApp.hEnet,
                                  gEnetApp.coreKey,
                                  gEnetApp.coreId,
                                  gEnetApp.txChNum);
#endif
            EnetAppUtils_print("Failed to open TX channel\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(gEnetApp.hTxCh != NULL);
        }

        EnetAppUtils_assert(txChCfg.useGlobalEvt == true);
    }

    /* Allocate TX packets and keep them locally enqueued */
    if (status == ENET_SOK)
    {
        EnetApp_initTxFreePktQ();
    }

    if (status == ENET_SOK)
    {
        status = EnetDma_enableTxEvent(gEnetApp.hTxCh);
    }

    /* Open the RX flow for Regular frames */
    if (status == ENET_SOK)
    {
        EnetDma_initRxChParams(&rxChCfg);

        rxChCfg.hUdmaDrv = gEnetApp.hMainUdmaDrv;
        rxChCfg.notifyCb = EnetApp_rxIsr;
        rxChCfg.cbArg   = &gEnetApp;
        rxChCfg.useGlobalEvt = true;

        EnetAppUtils_setCommonRxFlowPrms(&rxChCfg);
        EnetAppUtils_openRxFlow(gEnetApp.enetType,
                                gEnetApp.hEnet,
                                gEnetApp.coreKey,
                                gEnetApp.coreId,
                                true,
                                &gEnetApp.rxStartFlowIdx,
                                &gEnetApp.rxFlowIdx,
                                &gEnetApp.macAddr[0U],
                                &gEnetApp.hRxCh,
                                &rxChCfg);

        EnetAppUtils_assert(rxChCfg.useGlobalEvt == true);
        EnetAppUtils_assert(rxChCfg.flowPrms.sizeThreshEn == 0U);

        if (gEnetApp.hRxCh == NULL)
        {
            EnetAppUtils_print("EnetApp_openRxCh() failed to open RX flow\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(gEnetApp.hRxCh != NULL);
        }
    }

    /* Allocate RX buffers and submit them to DMA */
    if (status == ENET_SOK)
    {
        EnetApp_initRxReadyPktQ(gEnetApp.hRxCh);
    }

     return status;
}

void EnetApp_closeDma(void)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    /* Close RX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Close Regular RX channel */
    EnetAppUtils_closeRxFlow(gEnetApp.enetType,
                             gEnetApp.hEnet,
                             gEnetApp.coreKey,
                             gEnetApp.coreId,
                             true,
                             &fqPktInfoQ,
                             &cqPktInfoQ,
                             gEnetApp.rxStartFlowIdx,
                             gEnetApp.rxFlowIdx,
                             gEnetApp.macAddr,
                             gEnetApp.hRxCh);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Retrieve any pending TX packets from driver */
    EnetApp_retrieveFreeTxPkts();

    EnetAppUtils_closeTxCh(gEnetApp.hEnet,
                           gEnetApp.coreKey,
                           gEnetApp.coreId,
                           &fqPktInfoQ,
                           &cqPktInfoQ,
                           gEnetApp.hTxCh,
                           gEnetApp.txChNum);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&gEnetApp.txFreePktInfoQ);
}

static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    int32_t status;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    EnetQueue_initQ(&rxFreeQ);
    EnetQueue_initQ(&rxReadyQ);

    /* Allocate packets from pool */
    for (i = 0U; i < ENET_MEM_NUM_RX_PKTS; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetApp,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);

        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any packets which are ready */
    status = EnetDma_retrieveRxPktQ(hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);

    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    /* Prime RX channel with free packets */
    EnetDma_submitRxPktQ(hRxCh, &rxFreeQ);

    /* Assert here, as during init, the number of DMA descriptors should be equal to
     * the number of free Ethernet buffers available with app */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxFreeQ) == 0U);
}

static void EnetApp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[1];

    EnetQueue_initQ(&gEnetApp.txFreePktInfoQ);

    scatterSegments[0] = ENET_MEM_LARGE_POOL_PKT_SIZE;

    /* Allocate TX packets from pool, save them in local queue (txFreePktInfoQ) */
    for (i = 0U; i < ENET_MEM_NUM_TX_PKTS; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetApp,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gEnetApp.txFreePktInfoQ, &pPktInfo->node);
    }

    EnetAppUtils_print("initQs() txFreePktInfoQ initialized with %d pkts\n",
                           EnetQueue_getQCount(&gEnetApp.txFreePktInfoQ));
}

static uint32_t EnetApp_retrieveFreeTxPkts(void)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    uint32_t txFreeQCnt = 0U;
    int32_t status;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gEnetApp.hTxCh, &txFreeQ);
    if (status == ENET_SOK)
    {
        txFreeQCnt = EnetQueue_getQCount(&txFreeQ);

        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        while (NULL != pktInfo)
        {
            EnetDma_checkPktState(&pktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_DRIVER,
                                    ENET_PKTSTATE_APP_WITH_FREEQ);

            EnetQueue_enq(&gEnetApp.txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print("Failed to retrieve TX pkts: %d\r\n", status);
    }

    return txFreeQCnt;
}

void EnetApp_createRxTask(void)
{
    TaskP_Params taskParams;
    SemaphoreP_Params params;

    /* Initialize timer semaphore params */
    SemaphoreP_Params_init(&params);
    params.mode = SemaphoreP_Mode_BINARY;

    gEnetApp.hRxSem = SemaphoreP_create(0, &params);
    EnetAppUtils_assert(gEnetApp.hRxSem != NULL);

    SemaphoreP_Params_init(&params);
    params.mode = SemaphoreP_Mode_COUNTING;
    gEnetApp.hRxDoneSem = SemaphoreP_create(0, &params);
    EnetAppUtils_assert(gEnetApp.hRxDoneSem != NULL);

    TaskP_Params_init(&taskParams);
    taskParams.priority   = 5U;
    taskParams.stack      = gEnetAppTaskStackRx;
    taskParams.stacksize  = sizeof(gEnetAppTaskStackRx);
    taskParams.name       = (const char *)"RX Task";

    gEnetApp.hAppRxTask = TaskP_create(&EnetApp_rxTask, &taskParams);
    EnetAppUtils_assert(gEnetApp.hAppRxTask != NULL);
}

void EnetApp_destroyRxTask(void)
{
    SemaphoreP_delete(gEnetApp.hRxSem);
    SemaphoreP_delete(gEnetApp.hRxDoneSem);
    TaskP_delete(&gEnetApp.hAppRxTask);
}

uint64_t EnetApp_getCurrentTime(void)
{
    Enet_IoctlPrms prms;
    int32_t status;
    uint64_t tsVal = 0ULL;

    /* Software push event */
    ENET_IOCTL_SET_OUT_ARGS(&prms, &tsVal);
    status = Enet_ioctl(gEnetApp.hEnet, gEnetApp.coreId, ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP, &prms);
    if (status != ENET_SOK)
    {
        tsVal = 0ULL;
    }

    return tsVal;
}

static void EnetApp_rxTask(void *args1, void *args2)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *rxPktInfo;
    int32_t status = ENET_SOK;

    while ((ENET_SOK == status) && gEnetApp.run)
    {
        /* Initialize local queues */
        EnetQueue_initQ(&rxReadyQ);
        EnetQueue_initQ(&rxFreeQ);

        /* Wait for packet reception */
		SemaphoreP_pend(gEnetApp.hRxSem, SemaphoreP_WAIT_FOREVER);

        /* Get the packets received so far */
        status = EnetDma_retrieveRxPktQ(gEnetApp.hRxCh, &rxReadyQ);
        if (status != ENET_SOK)
        {
            /* Should we bail out here? */
            EnetAppUtils_print("Failed to retrieve RX pkt queue: %d\r\n", status);
            continue;
        }

        /* Consume (just drop) the received packets and send them back */
        rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        while (rxPktInfo != NULL)
        {
            EnetDma_checkPktState(&rxPktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_DRIVER,
                                    ENET_PKTSTATE_APP_WITH_READYQ);

            EnetDma_checkPktState(&rxPktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_READYQ,
                                    ENET_PKTSTATE_APP_WITH_FREEQ);

            /* Release the received packet */
            EnetQueue_enq(&rxFreeQ, &rxPktInfo->node);
            rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        }

        EnetAppUtils_validatePacketState(&rxFreeQ,
                                         ENET_PKTSTATE_APP_WITH_FREEQ,
                                         ENET_PKTSTATE_APP_WITH_DRIVER);

        /* Submit now processed buffers */
        EnetDma_submitRxPktQ(gEnetApp.hRxCh, &rxFreeQ);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to submit RX pkt queue: %d\r\n", status);
        }
    }

    SemaphoreP_post(gEnetApp.hRxDoneSem);
}

void EnetApp_txTest(void)
{
    EnetDma_PktQ dmaPktQ;
    EnetDma_Pkt *dmaPkt;
    EthVlanFrame *txFrame;
    uint32_t i, j;
    int32_t status;
    uint8_t bcastMac[ENET_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    EnetQueue_initQ(&dmaPktQ);

    /* Retrieve TX packets from driver and recycle them */
    EnetApp_retrieveFreeTxPkts();

    for (i = 0U; i < ENETAPP_TEST_TX_PKT_CNT; i++)
    {
        /* Dequeue one free TX Eth packet */
        dmaPkt = (EnetDma_Pkt *)EnetQueue_deq(&gEnetApp.txFreePktInfoQ);
        if (dmaPkt != NULL)
        {
            /* Fill the TX frame with test content */
            txFrame = (EthVlanFrame *)dmaPkt->sgList.list[0].bufPtr;
            memcpy(txFrame->hdr.dstMac, &bcastMac[0U], ENET_MAC_ADDR_LEN);
            memcpy(txFrame->hdr.srcMac, &gEnetApp.macAddr[0U], ENET_MAC_ADDR_LEN);

            txFrame->hdr.tpid = Enet_htons(ENETAPP_VLAN_TPID);
            txFrame->hdr.tci  = Enet_htons(ENETAPP_VLAN_TCI(i%8, 0, 100));
            txFrame->hdr.etherType = Enet_htons(ENETAPP_TEST_TX_ETHERTYPE);

            memset(&txFrame->payload[0U], i, ENETAPP_TEST_TX_PKT_PAYLOAD_LEN);

            dmaPkt->sgList.list[0].segmentFilledLen = sizeof(EthVlanFrameHeader) + ENETAPP_TEST_TX_PKT_PAYLOAD_LEN;
            dmaPkt->sgList.numScatterSegments = 1;
            EnetAppUtils_assert(dmaPkt->sgList.list[0].segmentAllocLen >= dmaPkt->sgList.list[0].segmentFilledLen);
            for (j = 1U; j < ENET_UDMA_CPSW_MAX_SG_LIST; j++)
            {
                dmaPkt->sgList.list[j].segmentFilledLen = 0U;
                EnetAppUtils_assert(dmaPkt->sgList.list[j].segmentFilledLen == 0U);
            }
            dmaPkt->appPriv = &gEnetApp;

            dmaPkt->chkSumInfo = 0U;
            dmaPkt->txPortNum  = ENET_MAC_PORT_INV;
            dmaPkt->tsInfo.enableHostTxTs = false;
            dmaPkt->txPktTc = 0U;
            dmaPkt->txTsId = 0U;
            EnetDma_checkPktState(&dmaPkt->pktState,
                                  ENET_PKTSTATE_MODULE_APP,
                                  ENET_PKTSTATE_APP_WITH_FREEQ,
                                  ENET_PKTSTATE_APP_WITH_DRIVER);

            /* Enqueue the packet for later transmission */
            EnetQueue_enq(&dmaPktQ, &dmaPkt->node);
        }
        else
        {
            EnetAppUtils_print("Drop due to TX pkt not available\r\n");
        }
    }

    /* Submit TX packet queue to driver for transmission */
    status = EnetDma_submitTxPktQ(gEnetApp.hTxCh, &dmaPktQ);

    if (EnetQueue_getQCount(&dmaPktQ)!=0)
    {
        EnetAppUtils_assert(false);
    }

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to submit TX pkt queue: %d\r\n", status);
    }
}
