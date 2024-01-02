/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/**
 * Copyright (c) 2018 Texas Instruments Incorporated
 *
 * This file is dervied from the ``ethernetif.c'' skeleton Ethernet network
 * interface driver for lwIP.
 *
 */

/* Standard language headers */
#include <stdio.h>
#include <assert.h>
#include <string.h>

/**
 * lwIP specific header files
 */
#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"

/**
 * PDK header files
 */
#include <ti/drv/enet/enet.h>
#include <ti/drv/enet/include/core/enet_utils.h>
#include <ti/drv/enet/intercore/intercore.h>
#include <ti/drv/enet/lwipific/inc/lwip2enet_ic.h>
#include <ti/drv/enet/lwipific/inc/bufpool.h>
#include <ti/drv/enet/enet_cfg.h>
#include <ti/drv/uart/UART_stdio.h>


/* ========================================================================== */
/*                          Extern variables                                  */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                             Local Macros                                   */
/* ========================================================================== */

#define OS_TASKPRIHIGH              8

#define LWIPIF_IPC_INIT_TASK_PRI       (OS_TASKPRIHIGH)

/* Number of packets sent before a notification is sent to the receiver */
#define IC_PKT_NOTIFY_THRESHOLD     1

#if defined (SAFERTOS)
#define  IPC_TASK_STACK_ALIGN       (IPC_TASK_STACKSIZE)
#else
#define  IPC_TASK_STACK_ALIGN       0x2000U
#endif

/* ========================================================================== */
/*                    Local Function Declarations                             */
/* ========================================================================== */
#if !(IC_ETH_RX_POLLING_MODE)
static int32_t Lwip2EnetIc_ipcInit(Lwip2EnetIc_Handle hLwip2EnetIc);
static void Lwip2EnetIc_ipcRxTaskFxn(void *arg0,
                                     void *arg1);
#endif
/* ========================================================================== */
/*                            Local Variables                                 */
/* ========================================================================== */

Lwip2EnetIc_Object gLwip2EnetIcObj[IC_ETH_MAX_VIRTUAL_IF];

Lwip2EnetIc_Params gIcEnetParams[IC_ETH_MAX_VIRTUAL_IF]=
{
    {
        .instId             = IC_ETH_IF_MCU2_0_MCU2_1,
        .ownerId            = IPC_MCU2_0,
        .txQId              = ICQ_MCU2_0_TO_MCU2_1,
        .rxQId              = ICQ_MCU2_1_TO_MCU2_0,
        .bufPoolId          = BUFPOOL_MCU2_0_R5,
        .reqEndPtId         = ICETH_IPC_ENDPT_MCU2_0_R5,
        .remoteCoreId       = IPC_MCU2_1,
        .endPtName          = "ENDPT_ICETH_MCU2_0_R5",
        .remoteEndPtName    = "ENDPT_ICETH_MCU2_1",
        .macAddr            = {0x00,0x01,0x02,0x03,0x04,0x05},
    },
    {
        .instId             = IC_ETH_IF_MCU2_1_MCU2_0,
        .ownerId            = IPC_MCU2_0,
        .txQId              = ICQ_MCU2_1_TO_MCU2_0,
        .rxQId              = ICQ_MCU2_0_TO_MCU2_1,
        .bufPoolId          = BUFPOOL_MCU2_1,
        .reqEndPtId         = ICETH_IPC_ENDPT_MCU2_1,
        .remoteCoreId       = IPC_MCU2_0,
        .endPtName          = "ENDPT_ICETH_MCU2_1",
        .remoteEndPtName    = "ENDPT_ICETH_MCU2_0_R5",
        .macAddr            = {0x00,0x01,0x02,0x04,0x05,0x06},
    },
    {
        .instId             = IC_ETH_IF_MCU2_0_A72,
        .ownerId            = IPC_MCU2_0,
        .txQId              = ICQ_MCU2_0_TO_A72,
        .rxQId              = ICQ_A72_TO_MCU2_0,
        .bufPoolId          = BUFPOOL_MCU2_0_A72,
        .reqEndPtId         = ICETH_IPC_ENDPT_MCU2_0_A72,
        .remoteCoreId       = IPC_MPU1_0,
        .endPtName          = "ENDPT_ICETH_MCU2_0_A72",
        .remoteEndPtName    = "ENDPT_ICETH_A72",
        .macAddr            = {0x00,0x01,0x02,0x05,0x06,0x07},
    },
};

#if !(IC_ETH_RX_POLLING_MODE)
static uint8_t gLwip2EnetIc_ipcRxTaskStack[IC_ETH_MAX_VIRTUAL_IF][IPC_TASK_STACKSIZE]
__attribute__ ((section(".bss:taskStackSection")))
__attribute__ ((aligned(IPC_TASK_STACK_ALIGN)));
#endif

uint8_t  gLwip2EnetIc_rpMsgRecvBuf[IC_ETH_MAX_VIRTUAL_IF][ICETH_RPMSG_DATA_SIZE]
__attribute__ ((section (".ipc_data_buffer"), aligned (8)));

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                    API/Public Function Definitions                         */
/* ========================================================================== */

/**
 * Initializes intercore transport
 */
Lwip2EnetIc_Handle Lwip2EnetIc_open(uint32_t instId)
{
    int32_t status;
    Lwip2EnetIc_Handle hLwip2EnetIc;
    Ic_Object_Handle hIcObj;    

    SemaphoreP_Params semaphoreParams;

    hLwip2EnetIc    = &gLwip2EnetIcObj[instId];
    hIcObj          = &(hLwip2EnetIc->icObj);

    /* Initialize the allocated memory block. */
    memset(hLwip2EnetIc, 0, sizeof(Lwip2EnetIc_Object));

    hLwip2EnetIc->instId = instId;
    /* Create semaphore objects, init shutDownFlag status */
    hLwip2EnetIc->shutDownFlag = false;

    hLwip2EnetIc->hShutDownSem = SemaphoreP_create(0, NULL);
    Lwip2EnetIc_assert(NULL != hLwip2EnetIc->hShutDownSem);

    hIcObj->selfCoreId = EnetSoc_getCoreId();
    
    /* Only MCU2_0 and MCU2_1 supported for now */
    Lwip2EnetIc_assert( (hIcObj->selfCoreId == IPC_MCU2_0) ||
                (hIcObj->selfCoreId == IPC_MCU2_1) );

    hIcObj->remoteCoreId = gIcEnetParams[instId].remoteCoreId;
    hIcObj->txQId        = gIcEnetParams[instId].txQId;
    hIcObj->rxQId        = gIcEnetParams[instId].rxQId;
    hIcObj->myReqEndPtId = gIcEnetParams[instId].reqEndPtId;

    memset(hIcObj->myEndPtName, '\0', sizeof(hIcObj->myEndPtName));
    memset(hIcObj->remoteEndPtName, '\0', sizeof(hIcObj->remoteEndPtName));
    strcpy(hIcObj->myEndPtName, gIcEnetParams[instId].endPtName);
    strcpy(hIcObj->remoteEndPtName, gIcEnetParams[instId].remoteEndPtName);

    /* Initialize shared bufpool */
    hLwip2EnetIc->hBufPool =
        (BufPool_Handle)&(BufPoolTable_Handle[gIcEnetParams[instId].bufPoolId]);
    
    status = BufPool_init(hLwip2EnetIc->hBufPool,
                          gIcEnetParams[instId].bufPoolId,
                          BUFPOOL_BUF_MAX);

    Lwip2EnetIc_assert(status == LWIP2ENETIC_OK);

    memcpy((void*)&(hLwip2EnetIc->macAddr), (void*)&gIcEnetParams[instId].macAddr, sizeof(hLwip2EnetIc->macAddr));

    hIcObj->txQ_Handle = (IcQ_Handle)&(IcQ_globalQTable_Handle[hIcObj->txQId]);
    hIcObj->rxQ_Handle = (IcQ_Handle)&(IcQ_globalQTable_Handle[hIcObj->rxQId]);
    Lwip2EnetIc_assert(hIcObj->txQ_Handle != NULL);
    Lwip2EnetIc_assert(hIcObj->rxQ_Handle != NULL);

    /* TODO: Allow application to provide custom print function */
    hLwip2EnetIc->print = (Enet_Print) &EnetUtils_printf;

#if !(IC_ETH_RX_POLLING_MODE)
    /* Initialize IPC */
    status = Lwip2EnetIc_ipcInit(hLwip2EnetIc);
    Lwip2EnetIc_assert(status == LWIP2ENETIC_OK);
    hIcObj->pktNotifyThresh = IC_PKT_NOTIFY_THRESHOLD;
#endif

    /* If we are the owner of this interface then initialize shared memory
     * transport otherwise wait here until it is initialized by the owner
     */
    if(gIcEnetParams[instId].ownerId == hIcObj->selfCoreId)
    {
        /* Initialize our TX i.e. peer's RX queue */
        status = IcQueue_initQ(hIcObj->txQ_Handle, hIcObj->txQId, ICQ_MAX_QUEUE_SIZE);
        Lwip2EnetIc_assert(status == LWIP2ENETIC_OK);

        /* Initialize our RX i.e. peer's TX queue */
        status = IcQueue_initQ(hIcObj->rxQ_Handle, hIcObj->rxQId, ICQ_MAX_QUEUE_SIZE);
        Lwip2EnetIc_assert(status == LWIP2ENETIC_OK);
    }
    else
    {
        /* Wait for the shared memory transport to be initialized
         * NOTE: This is called by LWIPIF_LWIP_IC_init which is called
         * by netif_add so netif_add will be blocked untill transport
         * initialization is complete
         */
        while(!IcQueue_isQValid(hIcObj->txQ_Handle) ||
                !IcQueue_isQValid(hIcObj->rxQ_Handle) )
        {
            TaskP_sleep(100);
        }
    }

    SemaphoreP_Params_init(&semaphoreParams);
    semaphoreParams.mode     = SemaphoreP_Mode_BINARY;
    hLwip2EnetIc->hRxPacketSem = SemaphoreP_create(0, NULL);
    Lwip2EnetIc_assert(NULL != hLwip2EnetIc->hRxPacketSem);

    hLwip2EnetIc->initDone = TRUE;
    hLwip2EnetIc->linkIsUp = TRUE;

    return hLwip2EnetIc;
}

/*!
 *  @b Lwip2Enet_close
 *  @n
 *      Closes Ethernet peripheral and disables interrupts.
 *
 *  \param[in]  hLwip2EnetIc
 *      Lwip2Enet_object structure pointer.
 *
 *  \retval
 *      void
 */
void Lwip2EnetIc_close(Lwip2EnetIc_Handle hLwip2EnetIc)
{

    Lwip2EnetIc_assert(NULL != hLwip2EnetIc);
    Lwip2EnetIc_assert(NULL != hLwip2EnetIc->hRxPacketSem);
    Lwip2EnetIc_assert(NULL != hLwip2EnetIc->hShutDownSem);
    /* Set the translation layer shutdown flag */
    hLwip2EnetIc->shutDownFlag = true;

    /* Pend on hShutDownSem (twice for two sub-tasks) */
    SemaphoreP_pend(hLwip2EnetIc->hShutDownSem, SemaphoreP_WAIT_FOREVER);
    SemaphoreP_pend(hLwip2EnetIc->hShutDownSem, SemaphoreP_WAIT_FOREVER);
    
    /* Delete the semaphore objects */
    SemaphoreP_delete(&hLwip2EnetIc->hRxPacketSem);
    SemaphoreP_delete(&hLwip2EnetIc->hShutDownSem);

    /* Clear the allocated translation */
    memset(hLwip2EnetIc, 0, sizeof(Lwip2EnetIc_Object));
}

/* ========================================================================== */
/*                    Local Function Definitions                              */
/* ========================================================================== */
#if !(IC_ETH_RX_POLLING_MODE)
static int32_t Lwip2EnetIc_ipcInit(Lwip2EnetIc_Handle hLwip2EnetIc)
{
    TaskP_Params taskParams;

    RPMessage_Params    rpMsgParams;
    int32_t             status;

    Ic_Object_Handle    hIcObj;
    hIcObj              = &(hLwip2EnetIc->icObj);

    /* Create RP message endpoint */
    RPMessageParams_init(&rpMsgParams);
    rpMsgParams.requestedEndpt = hIcObj->myReqEndPtId;
    rpMsgParams.buf = gLwip2EnetIc_rpMsgRecvBuf[hLwip2EnetIc->instId];
    rpMsgParams.bufSize = ICETH_RPMSG_DATA_SIZE;

    hIcObj->localEndPt_Handle = RPMessage_create(&rpMsgParams, &hIcObj->myEndPtId);
    if(!hIcObj->localEndPt_Handle)
    {
        hLwip2EnetIc->print("IPC init (core : %s): Failed to create endpoint\r\n", Ipc_mpGetSelfName());
    }
    else
    {
        status = RPMessage_announce(RPMESSAGE_ALL, hIcObj->myEndPtId, hIcObj->myEndPtName);
    }

    if(status != IPC_SOK) 
    {
        hLwip2EnetIc->print("IPC init (core : %s): RPMessage_announce() for %s failed\n",
                hIcObj->myEndPtName);
    }
    else
    {
        /* RPmsg RX task for messages coming in to our ic eth endpoint */
        TaskP_Params_init(&taskParams);
        taskParams.name         = (const char *)"Lwip2EnetIc_ipcRxTask";
        taskParams.priority     = LWIPIF_IPC_INIT_TASK_PRI;
        taskParams.stack        = gLwip2EnetIc_ipcRxTaskStack[hLwip2EnetIc->instId];
        taskParams.stacksize    = IPC_TASK_STACKSIZE;
        taskParams.arg0         = hLwip2EnetIc;

        hLwip2EnetIc->hIpcRxTask = TaskP_create(&Lwip2EnetIc_ipcRxTaskFxn, &taskParams);
        Lwip2EnetIc_assert(NULL != hLwip2EnetIc->hIpcRxTask);
    }

    return status;
}

static void Lwip2EnetIc_ipcRxTaskFxn(void *arg0,
                                     void *arg1)
{
    uint32_t		remoteEndPt;
    uint32_t		remoteProcId;
    uint16_t		len;
    int32_t		status = 0;
    IcEth_MsgObj        rxMsg;
    
    Lwip2EnetIc_Handle  hLwip2EnetIc; 
    Ic_Object_Handle    hIcObj;

    hLwip2EnetIc = (Lwip2EnetIc_Handle)arg0;
    hIcObj       = &(hLwip2EnetIc->icObj);


    while(!hLwip2EnetIc->shutDownFlag)
    {
        status = RPMessage_recv(hIcObj->localEndPt_Handle,
                                (void*)&rxMsg, &len,
                                &remoteEndPt,
                                &remoteProcId,
                                IPC_RPMESSAGE_TIMEOUT_FOREVER);
        if(status != IPC_SOK) 
        {
            hLwip2EnetIc->print("\nRecvTask: failed with code %d\n", status);
        }
        else
        {
            Lwip2EnetIc_assert(rxMsg.dstCore == hIcObj->selfCoreId);

            hIcObj->numRxPktsPending += rxMsg.numPktsQd;

            SemaphoreP_post(hLwip2EnetIc->hRxPacketSem); 
        }
    }

    /* We are shutting down, notify that we are done */
    SemaphoreP_post(hLwip2EnetIc->hShutDownSem);

}


int32_t Lwip2EnetIc_remoteCorePktNotify(Lwip2EnetIc_Handle hLwip2EnetIc)
{
    uint32_t            remoteEndPt;
    uint32_t            remoteProcId;
    int32_t             status = 0;
    IcEth_MsgObj        txMsg;

    Ic_Object_Handle    hIcObj;
    hIcObj              = &(hLwip2EnetIc->icObj);

    /* Prepare the notification */
    memset((void*)&txMsg, 0, sizeof(txMsg));

    txMsg.msgType   = ICETH_PKTS_AVAILABLE;
    txMsg.srcCore   = hIcObj->selfCoreId;
    txMsg.dstCore   = hIcObj->remoteCoreId;
    txMsg.numPktsQd = hIcObj->pktNotifyThresh;

    /* Get remote endpoint Id */
    status = RPMessage_getRemoteEndPt(hIcObj->remoteCoreId,
                                      hIcObj->remoteEndPtName,
                                      &remoteProcId,
                                      &remoteEndPt,
                                      SemaphoreP_WAIT_FOREVER);

    if(hIcObj->remoteCoreId != remoteProcId) 
    {
        hLwip2EnetIc->print("\nSendTask-%d: RPMessage_getRemoteEndPt() failed, status %d\n",
                hIcObj->remoteCoreId, status);
    }
    else
    {
        status = RPMessage_send(hIcObj->localEndPt_Handle,
                                hIcObj->remoteCoreId,
                                remoteEndPt,
                                hIcObj->myEndPtId,
                                (Ptr)&txMsg,
                                (uint16_t)ICETH_IPC_MSG_SIZE);
    }

    if (status != IPC_SOK) 
    {
        hLwip2EnetIc->print("\nSendTask-%d: Failed from %s to %s...\n", 
                            hIcObj->remoteCoreId,
                            Ipc_mpGetSelfName(),
                            Ipc_mpGetName(hIcObj->remoteCoreId));
    }

    return status;
}
#endif
