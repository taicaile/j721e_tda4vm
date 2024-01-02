/*
 *  Copyright (c) Texas Instruments Incorporated 2018
 *  All rights reserved.
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
 *  \file ipc_mailbox.c
 *
 *  \brief Implementation of ipc Mailbox interaction
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/ipc/ipc.h>
#include <ti/drv/ipc/soc/ipc_soc.h>
#include <ti/drv/ipc/src/ipc_mailbox.h>
#include <ti/csl/src/ip/mailbox/V0/mailbox.h>

#ifndef IPC_EXCLUDE_POLLED_RX
#include <ti/osal/TaskP.h>
#endif

#include <string.h>

#include "ipc_osal.h"
#include "ipc_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Fix Me, need not be 256, can be max remote proc / max proc, i.e. 1 as
    structure which uses this is instantiated max proc times. */
#define  IPC_MBOX_MAXFIFO_CNT    16U
/* Fix Me, need not be 256, can be max remote proc / max proc */
#define  IPC_MBOX_MAXDATA        8U

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct Ipc_MailboxFifo_s
{
    int32_t                 cfgNdx;
    Mailbox_hwiCallback     func;
    uint32_t                arg;
    uint32_t                queueId;
} Ipc_MailboxFifo;

/* mboxData */
typedef struct Ipc_MailboxData_s
{
    uintptr_t         baseAddr;
    uint32_t          fifoCnt;
    Ipc_MailboxFifo   fifoTable[IPC_MBOX_MAXFIFO_CNT];
    uint32_t          noMsgCnt;
    uint32_t          intCnt;
    uint32_t 	      userId;

} Ipc_MailboxData;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Mailbox Internal Callback
 */
static void Ipc_mailboxInternalCallback(uintptr_t arg);

/**
 *  \brief Enable remote processor interrupt
 */
static void Ipc_mailboxEnable(uintptr_t baseAddr, uint32_t userId, uint32_t queueId);

/**
 *  \brief Disables remote processor interrupt
 */
static void Ipc_mailboxDisable(uintptr_t baseAddr, uint32_t userId, uint32_t queueId);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint32_t               g_ipc_mBoxCnt = 0U;
Ipc_MailboxData        g_ipc_mBoxData[IPC_MBOX_MAXDATA];
/**
 * \brief Maps mBoxData allocated to a given Remote Processor
 */
uintptr_t       gIpcRProcIdToMBoxDataMap[IPC_MAX_PROCS];

/* ========================================================================== */
/*                             Local Functions                                */
/* ========================================================================== */

#ifndef IPC_EXCLUDE_POLLED_RX
#define IPC_POLL_TIMER  100

uint32_t     g_pollTaskExit = FALSE;
TaskP_Handle g_pollTask     = NULL;

void Mailbox_Poll_Task(void* arg0, void* arg1)
{
    uint32_t                n    = 0U;
    uint32_t                cnt  = 0U;
    uint32_t                msg[4];
    Ipc_MailboxData        *mbox = NULL;
    Ipc_MailboxFifo        *fifo = NULL;

    while(FALSE == g_pollTaskExit)
    {
        for(n = 0; n < g_ipc_mBoxCnt; n++)
        {
             mbox = &g_ipc_mBoxData[n];
             for(cnt = 0; cnt < mbox->fifoCnt; cnt++)
             {
                 fifo = &mbox->fifoTable[cnt];
                 if(MailboxGetMessageCount(mbox->baseAddr, fifo->queueId) > 0)
                 {
                    /* Get the message from Mailbox fifo */
                    MailboxGetMessage(mbox->baseAddr, fifo->queueId, msg);

                    /* Clear new message status of Mailbox */
                    MailboxClrNewMsgStatus(mbox->baseAddr, mbox->userId, fifo->queueId);

#ifdef  DEBUG_PRINT
                    SystemP_printf("Mailbox polling : BaseAddr 0x%x queue %d value %d arg(procId) %d\n",
                        mbox->baseAddr,  fifo->queueId, msg[0], fifo->arg);
#endif
                    /* Call the function with arg */
                    (mbox->fifoTable[cnt].func)(msg, fifo->arg);
                 }
             }
        }
        /* Temporarily we use Task_yield() */
#ifdef QNX_OS
        TaskP_sleep(IPC_POLL_TIMER);
#else
        TaskP_yield();
#endif
    }
}

void Mailbox_Poll_TaskExit()
{
    g_pollTaskExit = TRUE;
}

#if defined(BUILD_C7X) && !defined(MAILBOX_INTERRUPT_MODE)
static uint8_t g_pollTaskStack[IPC_TASK_STACKSIZE]
__attribute__ ((section(".bss:taskStackSection")))
__attribute__ ((aligned(8192)))
    ;
#endif

void StartmailboxPollingTask()
{
    TaskP_Params param;

    TaskP_Params_init(&param);
    param.priority = 2;
#if defined(BUILD_C7X) && !defined(MAILBOX_INTERRUPT_MODE)
    param.stack = g_pollTaskStack;
#endif
    param.stacksize = IPC_TASK_STACKSIZE;
    g_pollTask = TaskP_create(&(void *) Mailbox_Poll_Task, &param);
}

#endif /* IPC_EXCLUDE_POLLED_RX */

/* ========================================================================== */
/*                             Functions                                      */
/* ========================================================================== */

int32_t Ipc_mailboxModuleStartup (void)
{
    int32_t  retVal = IPC_SOK;

    memset(gIpcRProcIdToMBoxDataMap, 0, sizeof(gIpcRProcIdToMBoxDataMap));

    return retVal;
}

/**
 *  \brief Enable remote processor interrupt
 */
static void Ipc_mailboxEnable(uintptr_t baseAddr, uint32_t userId, uint32_t queueId)
{
    MailboxEnableNewMsgInt(baseAddr, userId, queueId);
}

/**
 *  \brief Disables remote processor interrupt
 */
static void Ipc_mailboxDisable(uintptr_t baseAddr, uint32_t userId, uint32_t queueId)
{
    MailboxDisableNewMsgInt(baseAddr, userId, queueId);
}

/**
 *  \brief Clear interrupt and return the message.
 */
uint32_t Ipc_mailboxClear(uintptr_t baseAddr, uint32_t queueId)
{
    uint32_t retVal = 0;
    uint32_t msg[4];

    retVal = MailboxGetMessage(baseAddr, queueId, msg);

    return retVal;
}

/**
 *  \brief Send interrupt to the remote processor
 */
int32_t Ipc_mailboxSend(uint32_t selfId, uint32_t remoteProcId, uint32_t val,
                        uint32_t timeoutCnt)
{
    int32_t   status = 0;
    uint32_t  clusterId;
    uint32_t  userId;
    uint32_t  queueId;
    uintptr_t baseAddr;
    uint32_t  cnt = timeoutCnt;

#ifndef IPC_ONE_CONTEXT_FOR_HISRGATE_HWIGATE
    Ipc_Object      *pObj  = NULL;
    Ipc_OsalPrms    *pOsal = NULL;
    uintptr_t        key   = 0U;

    pObj = getIpcObjInst(0U);
    pOsal = &pObj->initPrms.osalPrms;
#endif /* IPC_ONE_CONTEXT_FOR_HISRGATE_HWIGATE */

    Ipc_getMailboxInfoTx(selfId, remoteProcId,
        &clusterId, &userId, &queueId);

    if( (clusterId != MAILBOX_CLUSTER_INVALID) && (queueId < 16U) )
    {
        uint32_t    retVal;

        baseAddr = Ipc_getMailboxBaseAddr(clusterId);

        /* In case of baremetal, the HISR protection and Interrupt disable would
            same. This function is invoked by Virtio_kick() to let other cores
            know that there is new message available.
            Virtio_kick is called under HISR protection
            which means that we should not invoke protection again. As recursive
            interrupt disable is not recommended */
#ifndef IPC_ONE_CONTEXT_FOR_HISRGATE_HWIGATE
        if (NULL != pOsal->disableAllIntr)
        {
            key = pOsal->disableAllIntr();
        }
#endif /* IPC_ONE_CONTEXT_FOR_HISRGATE_HWIGATE */

        do
        {
            retVal = MailboxSendMessage(baseAddr, queueId, val);
            cnt--;
        } while( (cnt != 0U) && (retVal == MESSAGE_INVALID));

        if(MESSAGE_INVALID == retVal)
        {
#ifdef DEBUG_PRINT
            SystemP_printf("Ipc_mailboxSend : BaseAddr 0x%x queue %d value %d failed\n",
                baseAddr,  queueId, val);
#endif
            status = IPC_EFAIL;
        }

#if defined DEBUG_PRINT
        SystemP_printf("Ipc_mailboxSend : BaseAddr 0x%x queue %d value %d\n",
            baseAddr,  queueId, val);
#endif

#ifndef IPC_ONE_CONTEXT_FOR_HISRGATE_HWIGATE
        if (NULL != pOsal->restoreAllIntr)
        {
            pOsal->restoreAllIntr(key);
        }
#endif /* IPC_ONE_CONTEXT_FOR_HISRGATE_HWIGATE */

    }

    return status;
}

int32_t Ipc_mailboxRegister(uint16_t selfId, uint16_t remoteProcId,
           Mailbox_hwiCallback func, uint32_t arg, uint32_t timeoutCnt)
{
    int32_t               retVal = IPC_SOK;
    uint32_t              clusterId;
    uint32_t              userId;
    uint32_t              queueId;
    uintptr_t             baseAddr;
    uint32_t              n;
    Ipc_MailboxData      *mbox = NULL;
    Ipc_Object           *pObj  = NULL;
    Ipc_OsalPrms         *pOsal = NULL;

    pObj = getIpcObjInst(0U);
    pOsal = &pObj->initPrms.osalPrms;

    Ipc_getMailboxInfoRx(selfId, remoteProcId,
        &clusterId, &userId, &queueId);

    if( (clusterId != MAILBOX_CLUSTER_INVALID) && (IPC_MAX_PROCS > remoteProcId))
    {
        baseAddr = Ipc_getMailboxBaseAddr(clusterId);
    }
    else
    {
        SystemP_printf("Ipc_Mailbox_register : failed Invalid cluster..\n");
        retVal = IPC_EFAIL;
    }

    if(retVal == IPC_SOK)
    {
        for(n = 0; n < g_ipc_mBoxCnt; n++)
        {
            if((baseAddr == g_ipc_mBoxData[n].baseAddr) && (userId == g_ipc_mBoxData[n].userId))
            {
              break;
            }
        }

        /* Get the MailBox Data */
        mbox = &g_ipc_mBoxData[n];

        if(n == g_ipc_mBoxCnt)
        {
            /* Could not find one, this is new entry */
            mbox->baseAddr = baseAddr;
            mbox->fifoCnt  = 0;
            mbox->userId   = userId;

            if (NULL != pOsal->registerIntr)
            {
                /* Do not clear the mailbox, other cores could have already sent messages */

#ifdef MAILBOX_INTERRUPT_MODE
                {
                    Ipc_MbConfig cfg;

                    MailboxClrNewMsgStatus(baseAddr, userId, queueId);

#ifdef IPC_SUPPORT_SCICLIENT
                    /* Get the Interrupt Configuration */
                    Ipc_getMailboxIntrRouterCfg(selfId, clusterId, userId, &cfg, g_ipc_mBoxCnt);

                    {
                        /* Release the resource first */
                        Ipc_sciclientIrqRelease(selfId, clusterId, userId, cfg.eventId);

                        uint32_t timeout_cnt = 10;
                        do
                        {
                            retVal = Ipc_sciclientIrqSet(selfId, clusterId, userId, cfg.eventId);
                            if(retVal != 0)
                            {
                                SystemP_printf("Failed to register irq through sciclient...%x\n", retVal);
                            }
                            timeout_cnt--;
                        }while((retVal != 0) && (timeout_cnt > 0U));

                        if(timeout_cnt == 0U)
                        {
                            retVal = IPC_EFAIL;
                        }
                    }
#endif
                    /* Register Mailbox interrupt now... */
                    if (retVal == IPC_SOK)
                    {
                        /* disable the mailbox interrupt (from previous runs) */
                        Ipc_mailboxDisable(baseAddr, userId, queueId);
                        pObj->interruptHandle = pOsal->registerIntr(
                                &cfg,Ipc_mailboxInternalCallback,
                                (uintptr_t)mbox);
                    }

                }
#endif /* MAILBOX_INTERRUPT_MODE */
	    }

#ifndef IPC_EXCLUDE_POLLED_RX
#ifndef MAILBOX_INTERRUPT_MODE
            {
                /* Mailbox interrupt is not supported currently */
                if(g_pollTask == NULL)
                {
                    StartmailboxPollingTask();
                }
            }
#endif /* MAILBOX_INTERRUPT_MODE */
#endif /* IPC_EXCLUDE_POLLED_RX */

            g_ipc_mBoxCnt++;
        }

        /* Add the fifo data for the remoteProcId. */
        mbox->fifoTable[mbox->fifoCnt].cfgNdx  = (int32_t)n;
        mbox->fifoTable[mbox->fifoCnt].func    = func;
        mbox->fifoTable[mbox->fifoCnt].arg     = arg;
        mbox->fifoTable[mbox->fifoCnt].queueId = queueId;
        gIpcRProcIdToMBoxDataMap[remoteProcId] = (uintptr_t)mbox;

        mbox->fifoCnt++;

#ifdef DEBUG_PRINT
        SystemP_printf("Ipc_MB(%d): Self %d Remote %d (c%d,u%d,q%d) arg %d,total %d\n",
                mbox->fifoCnt, selfId, remoteProcId, clusterId, userId, queueId, arg,
                g_ipc_mBoxCnt);
#endif
    }

    return retVal;
}

/** \brief Low Level Mailbox ISR for a given remote processor */
void Ipc_mailboxIsr(uint32_t remoteProcId)
{
    uintptr_t mBoxData = 0U;

    if (IPC_MAX_PROCS > remoteProcId)
    {
        mBoxData = gIpcRProcIdToMBoxDataMap[remoteProcId];
        if (0U != mBoxData)
        {
            Ipc_mailboxInternalCallback(mBoxData);
        }
    }

    return;
}

/*!
 *  ======== Ipc_mailboxInternalCallback ========
 */
static void Ipc_mailboxInternalCallback(uintptr_t arg)
{
    uint32_t              n;
    Ipc_MailboxData      *mbox;
    uint32_t              msg[4];
    Ipc_MailboxFifo      *fifo;

    mbox = (Ipc_MailboxData *)arg;
    if(mbox != NULL)
    {
        mbox->intCnt++;

        for(n = 0; n < mbox->fifoCnt; n++)
        {
            fifo = &mbox->fifoTable[n];

            if(0U != MailboxGetRawNewMsgStatus(mbox->baseAddr, mbox->userId, fifo->queueId))
            {
                if( MailboxGetMessageCount(mbox->baseAddr, fifo->queueId) > 0U)
                {
                    /* Get the message from Mailbox fifo */
                    MailboxGetMessage(mbox->baseAddr, fifo->queueId, msg);

                    /* Clear new message status of Mailbox */
                    MailboxClrNewMsgStatus(mbox->baseAddr, mbox->userId,
                            fifo->queueId);

                    /* Call the function with arg */
                    (mbox->fifoTable[n].func)(msg, fifo->arg);
                }
                else
                {
                    /* Clear new message status of Mailbox */
                    MailboxClrNewMsgStatus(mbox->baseAddr, mbox->userId, fifo->queueId);
                }
            }
        }
        if(n == mbox->fifoCnt)
        {
            mbox->noMsgCnt++;
        }
    }
}

void Ipc_mailboxEnableNewMsgInt(uint16_t selfId, uint16_t remoteProcId)
{
    uint32_t  clusterId;
    uint32_t  userId;
    uint32_t  queueId;
    uintptr_t baseAddr;

    if ((IPC_MAX_PROCS > selfId) && (IPC_MAX_PROCS > remoteProcId))
    {
        Ipc_getMailboxInfoRx(selfId, remoteProcId, &clusterId, &userId,
                                &queueId);

        baseAddr = Ipc_getMailboxBaseAddr(clusterId);

        Ipc_mailboxEnable(baseAddr, userId, queueId);
    }
}

void Ipc_mailboxDisableNewMsgInt(uint16_t selfId, uint16_t remoteProcId)
{
    uint32_t  clusterId;
    uint32_t  userId;
    uint32_t  queueId;
    uintptr_t baseAddr;

    if ((IPC_MAX_PROCS > selfId) && (IPC_MAX_PROCS > remoteProcId))
    {
        Ipc_getMailboxInfoRx(selfId, remoteProcId, &clusterId, &userId,
                                &queueId);

        baseAddr = Ipc_getMailboxBaseAddr(clusterId);

        Ipc_mailboxDisable(baseAddr, userId, queueId);
    }
}

void *Mailbox_plugInterrupt(Ipc_MbConfig *cfg, Ipc_OsalIsrFxn func, uintptr_t arg)
{
    OsalRegisterIntrParams_t    intrPrms;
    OsalInterruptRetCode_e      osalRetVal;
    HwiP_Handle                 hwiHandle = NULL;
    uint32_t                    coreIntrNum = 0U;
#ifndef IPC_SUPPORT_SCICLIENT
    CSL_IntrRouterCfg           irRegs;
#endif

#ifdef DEBUG_PRINT
    SystemP_printf("Navss Rtr: input %d, output %d%d\n",
        cfg->inputIntrNum, cfg->outputIntrNum);
#endif
#ifdef QNX_OS
#ifdef DEBUG_PRINT
    SystemP_printf("Mailbox_plugInterrupt: Navss Rtr input %d, output %d\n",
        cfg->inputIntrNum, cfg->outputIntrNum);
#endif
#endif
#ifndef IPC_SUPPORT_SCICLIENT
    /* Configure Main NavSS512 interrupt router */
    #ifdef QNX_OS
    if(g_navssIntRtrBaseVirtAddr == 0)
    {
        g_navssIntRtrBaseVirtAddr = IpcUtils_getMemoryAddress(IPC_MCU_NAVSS0_INTR0_CFG_BASE,
                NVSS_INTRTR_SIZE );
    }
    irRegs.pIntrRouterRegs = (CSL_intr_router_cfgRegs *)g_navssIntRtrBaseVirtAddr;
    irRegs.pIntdRegs       = (CSL_intr_router_intd_cfgRegs *) NULL;
    irRegs.numInputIntrs   = MAIN_NAVSS_MAILBOX_INPUTINTR_MAX;
    irRegs.numOutputIntrs  = MAIN_NAVSS_MAILBOX_OUTPUTINTR_MAX;
    CSL_intrRouterCfgMux(&irRegs, cfg->inputIntrNum, cfg->outputIntrNum);
    #else
    irRegs.pIntrRouterRegs = (CSL_intr_router_cfgRegs *)IPC_MCU_NAVSS0_INTR0_CFG_BASE;
    irRegs.pIntdRegs       = (CSL_intr_router_intd_cfgRegs *) NULL;
    irRegs.numInputIntrs   = MAIN_NAVSS_MAILBOX_INPUTINTR_MAX;
    irRegs.numOutputIntrs  = MAIN_NAVSS_MAILBOX_OUTPUTINTR_MAX;
    CSL_intrRouterCfgMux(&irRegs, cfg->inputIntrNum, cfg->outputIntrNum);
    #endif

#if defined (SOC_AM65XX)
#if defined(BUILD_MCU1_0) || defined(BUILD_MCU1_1)
    Ipc_main2mcu_intRouter(cfg);
#endif
#endif

    /* Configure C66x Interrupt Router now */
#if defined(BUILD_C66X)
    Ipc_configC66xIntrRouter(cfg->eventId );
#endif

#endif  /* IPC_SUPPORT_SCICLIENT */

    /*
     * CLEC needs to be configured for all modes - CSL and Sciclient
     **/
#if defined(BUILD_C7X)
    /* Pass the corePackEvent and the base (which was derived from the NAVSS IR o/p
     * range returned from BoardCfg) to route the corressponding CLEC i/p Event to
     * a C7x IRQ. The returned IRQ num is used to register Interrupt with OSAL. */
    coreIntrNum = Ipc_configClecRouter(cfg->eventId, cfg->eventIdBase);
#else
    coreIntrNum = cfg->eventId;
#endif

#ifdef QNX_OS
#if 1 //def DEBUG_PRINT
    SystemP_printf("Mailbox_plugInterrupt: interrupt Number %d, arg 0x%X\n",
        cfg->eventId, (uint32_t)arg);
#endif
#endif

    /* Register interrupts */
    Osal_RegisterInterrupt_initParams(&intrPrms);
    intrPrms.corepacConfig.arg              = arg;
    intrPrms.corepacConfig.isrRoutine       = func;
    intrPrms.corepacConfig.priority         = cfg->priority;

#if defined(BUILD_C66X)
    intrPrms.corepacConfig.corepacEventNum  = coreIntrNum;
    intrPrms.corepacConfig.intVecNum        = OSAL_REGINT_INTVEC_EVENT_COMBINER;
#else
    intrPrms.corepacConfig.intVecNum        = coreIntrNum;
    intrPrms.corepacConfig.corepacEventNum  = 0;
#endif
#ifdef QNX_OS
    intrPrms.corepacConfig.intAutoEnable  = 1;
#endif
    osalRetVal = Osal_RegisterInterrupt(&intrPrms, &hwiHandle);
    if(OSAL_INT_SUCCESS != osalRetVal)
    {
        SystemP_printf("Mailbox_plugInterrupt : Failed to register ISR...\n");
    }

    return (hwiHandle);
}
