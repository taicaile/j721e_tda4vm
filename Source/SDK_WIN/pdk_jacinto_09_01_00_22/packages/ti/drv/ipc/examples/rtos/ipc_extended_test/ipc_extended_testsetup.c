/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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
 *  \file ipc_extended_testsetup.c
 *
 *  \brief Multi-core IPC extended test application performing basic echo
 *         communication using the IPC driver.
 *         This sample application demonstrates following tests:
 *              - Test Invalid configurations for array of processors,current processors
 *                and remote processors Ids.
 *              - Test Invalid configurations for IPC Driver intialization,deinitialization
 *                and mailbox registration APIs.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <ti/drv/ipc/examples/common/src/ipc_setup.h>
#include <ti/drv/ipc/ipc.h>
#include <ti/drv/ipc/examples/rtos/ipc_extended_test/ipc_extended_setup.h>
#include <ti/drv/ipc/src/ipc_mailbox.h>
#include <ti/drv/ipc/src/ipc_virtioPrivate.h>

#include <ti/osal/osal.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define RP_MSG_TEST_BUF_SIZE     (512 + 2)
#define IPC_APP_MSGSIZE  256U
#define IPC_APP_SERVICE  "ti.ipc4.ping-pong"
#define IPC_APP_FAKE_SERVICE "negative-test"
#define IPC_APP_ENDPT1   13U
#define IPC_APP_ENDPT2   14U
#define IPC_APP_INVALID_ID 50U
#define IPC_APP_NUMMSGS  10000 /* number of message sent per task */
#define IPC_APP_MP_INVALID_ID            (0xFFFFFFFFU)
uint32_t rpmsgNegDataSize = RPMSG_DATA_SIZE;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef enum IpcApp_ExtTestId {
      TEST_VIRTIO_INIT_PARAM_NULL,
      TEST_RPMSG_INIT_PARAM_NULL,
      TEST_MP_SET_CONFG_NULL,
      TEST_MP_SET_CONFG_INVALID_SELFPROC,
      TEST_MP_SET_CONFG_INVALID_SELFPROC_AND_NUMPROC,
      TEST_MP_SET_CONFG_INVALID_NUMPROC,
      TEST_END
} IpcApp_ExtTestId;

typedef struct IpcApp_ExtTestParams_s
{
    uint32_t  testId;
    char     *testName;
} IpcApp_TestParams;

IpcApp_TestParams ExtTestCases[] =
{
      { TEST_VIRTIO_INIT_PARAM_NULL, "test_virtio_init_param_null" },
      { TEST_RPMSG_INIT_PARAM_NULL, "test_rpmsg_init_param_null" },
      { TEST_MP_SET_CONFG_NULL, "test_mp_set_confg_null"},
      { TEST_MP_SET_CONFG_INVALID_SELFPROC, "test_mp_set_confg_invalid_selfproc"},
      { TEST_MP_SET_CONFG_INVALID_SELFPROC_AND_NUMPROC, "test_mp_set_confg_invalid_selfproc_numproc"},
      { TEST_MP_SET_CONFG_INVALID_NUMPROC, "test_mp_set_confg_invalid_numproc"},
      { TEST_END, NULL },
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern uint8_t  *pCntrlBuf;
extern uint8_t  *pTaskBuf;
extern uint8_t  *pSendTaskBuf;
extern uint8_t  *pRecvTaskBuf;
extern uint8_t  *pTimeoutBuf;
extern uint8_t  *pSysVqBuf;
extern uint32_t  selfProcId;
extern uint32_t *pRemoteProcArray;
extern uint32_t  gNumRemoteProc;

static uint32_t  gTotalTests = 0;
static uint32_t  gTotalTestsPassed = 0;
static uint32_t  gTotalTestsFailed = 0;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void IpcAPP_ReportResult(uint32_t id, int32_t result)
{
      gTotalTests++;
      if (result == IPC_SOK)
      {
          App_printf("[%s] [%s] : [TEST_PASS]\n", Ipc_mpGetSelfName(), ExtTestCases[id].testName);
          UART_printf("[%s] [%s] : [TEST_PASS]\n", Ipc_mpGetSelfName(), ExtTestCases[id].testName);
          gTotalTestsPassed++;
      }
      else
      {
          App_printf("[%s] [%s] : [TEST_FAIL]\n", Ipc_mpGetSelfName(), ExtTestCases[id].testName);
          UART_printf("[%s] [%s] : [TEST_FAIL]\n", Ipc_mpGetSelfName(), ExtTestCases[id].testName);
          gTotalTestsFailed++;
      }
}

/*
 * This "Task" waits for a "ping" message from any processor
 * then replies with a "pong" message.
 */
int32_t Rpmsg_Extended_ResponderFxn(uint32_t testId)
{
      RPMessage_Params params;
      uint32_t         remoteProcId = 0;
      void             *buf;

      buf = pRecvTaskBuf;
      if (buf == NULL)
      {
          App_printf("RecvTask: buffer allocation failed\n");
          return IPC_EFAIL;
      }

      RPMessageParams_init(&params);

      Ipc_newMessageIsr(selfProcId);

      /* Testing invalid remoteprocId for mailbox ISR */
      Ipc_mailboxIsr(remoteProcId);

      Ipc_mailboxIsr(IPC_APP_INVALID_ID);

      Ipc_mailboxIsr(IPC_APP_ENDPT1);

      return IPC_SOK;
}

int32_t IpcApp_Extended_test(uint32_t testId)
{
       uint32_t         numProc = gNumRemoteProc;
       Ipc_VirtIoParams vqParam;
       uint32_t         index = 0;
       int32_t          status;
       char*            name = "mcu2_0";

       /* Step 1: Sets configuration parameters for
       *  current processor, number of processors,
       *  and array of processors
       */

       status = Ipc_mpSetConfig(selfProcId, numProc, pRemoteProcArray);
       if(status == IPC_SOK)
       {
           IpcAPP_ReportResult(TEST_MP_SET_CONFG_NULL, IPC_SOK);
       }
       else
       {
           IpcAPP_ReportResult(TEST_MP_SET_CONFG_NULL, IPC_EFAIL);
           return IPC_EFAIL;
       }

       status = Ipc_mpSetConfig(IPC_APP_INVALID_ID, numProc, pRemoteProcArray);
       if(status != IPC_SOK)
       {
           IpcAPP_ReportResult(TEST_MP_SET_CONFG_INVALID_SELFPROC, IPC_SOK);
       }
       else
       {
           IpcAPP_ReportResult(TEST_MP_SET_CONFG_INVALID_SELFPROC, IPC_EFAIL);
           return IPC_EFAIL;
       }

       status = Ipc_mpSetConfig(IPC_APP_INVALID_ID, IPC_APP_INVALID_ID, pRemoteProcArray);
       if(status != IPC_SOK)
       {
           IpcAPP_ReportResult(TEST_MP_SET_CONFG_INVALID_SELFPROC_AND_NUMPROC, IPC_SOK);
       }
       else
       {
           IpcAPP_ReportResult(TEST_MP_SET_CONFG_INVALID_SELFPROC_AND_NUMPROC, IPC_EFAIL);
           return IPC_EFAIL;
       }

       status = Ipc_mpSetConfig(IPC_APP_INVALID_ID, IPC_APP_INVALID_ID, pRemoteProcArray);
       if(status != IPC_SOK)
       {
           IpcAPP_ReportResult(TEST_MP_SET_CONFG_INVALID_NUMPROC, IPC_SOK);
       }
       else
       {
           IpcAPP_ReportResult(TEST_MP_SET_CONFG_INVALID_NUMPROC, IPC_EFAIL);
           return IPC_EFAIL;
       }

       Ipc_mpSetConfig(selfProcId, 5U, pRemoteProcArray);

       /* Get MultiProc id for the processor
       * with corresponding MultiProc name
       */

       Ipc_mpGetId(name);

       Ipc_mpGetId((char*)(NULL));

       Ipc_mpGetId((char*)(selfProcId));

       Ipc_mpGetId(0U);

       Ipc_mpGetId((char*)(IPC_APP_MP_INVALID_ID));

       Ipc_mpGetName((uint32_t)(NULL));

       Ipc_mpGetName(selfProcId);

       Ipc_mpGetName(17U);

       Ipc_mpGetRemoteProcId(IPC_APP_ENDPT1);

       App_printf("IPC_echo_test (core : %s) .....\r\n", Ipc_mpGetSelfName());

       /* Initialize IPC module */
       Ipc_init(NULL);

       /* Step2 : Initialize Virtio */
       vqParam.vqObjBaseAddr = (void*)pSysVqBuf;
       vqParam.vqBufSize     = numProc * Ipc_getVqObjMemoryRequiredPerCore();
       vqParam.vringBaseAddr = (void*)VRING_BASE_ADDRESS;
       vqParam.vringBufSize  = IPC_VRING_BUFFER_SIZE;
       vqParam.timeoutCnt    = 100;  /* Wait for counts */

       /* Test Ipc_initVirtIO with NULL parameters */

       status = Ipc_initVirtIO( NULL );
       if(status != IPC_SOK)
       {
           IpcAPP_ReportResult(TEST_VIRTIO_INIT_PARAM_NULL, IPC_SOK);
       }
       else
       {
           IpcAPP_ReportResult(TEST_VIRTIO_INIT_PARAM_NULL, IPC_EFAIL);
           return IPC_EFAIL;
       }

       App_printf("Ipc_initVirtIO\n");
       vqParam.vringBufSize  = IPC_VRING_BUFFER_SIZE;
       status = Ipc_initVirtIO(&vqParam);
       if (status != IPC_SOK)
       {
          App_printf("[%s] Ipc_initVirtIO failed\n");
          return IPC_EFAIL;
       }

       /* Step 3: Initialize RPMessage */
       RPMessage_Params cntrlParam;
       uint32_t         p = 0U;

       /* Initialize an RPMessage_Params structure */
       RPMessageParams_init(&cntrlParam);

       /* Set HeapMemory for control task */
       cntrlParam.buf         = pCntrlBuf;
       cntrlParam.bufSize     = rpmsgNegDataSize;
       cntrlParam.stackBuffer = &pTaskBuf[index * IPC_TASK_STACKSIZE];
       cntrlParam.stackSize   = IPC_TASK_STACKSIZE;

       /* Test RPMessage_init with NULL parameters */

       status = RPMessage_init(NULL);
       if(status != IPC_SOK)
       {
          IpcAPP_ReportResult(TEST_RPMSG_INIT_PARAM_NULL, IPC_SOK);
       }
       else
       {
         IpcAPP_ReportResult(TEST_RPMSG_INIT_PARAM_NULL, IPC_EFAIL);
         return IPC_EFAIL;
       }

       App_printf("RPMessage_init\n");
       cntrlParam.stackSize   = IPC_TASK_STACKSIZE;
       status = RPMessage_init(&cntrlParam);
       if (status != IPC_SOK)
       {
          App_printf("[%s] RPMessage_init failed\n", Ipc_mpGetSelfName());
          return IPC_EFAIL;
       }
       Virtio_setCallback(p,NULL,NULL);

      return Rpmsg_Extended_ResponderFxn(testId);
}

int32_t IpcApp_echoExtTest(void)
{
      uint32_t  i = 0;
      uintptr_t baseAddr = 0;
      uint32_t  queueId = 0;
      uint32_t  remoteProcId=0;

      IpcApp_Extended_test(i);

      Ipc_mailboxClear(baseAddr,queueId);

      /* Test mailbox to disable new message Initialization */
      Ipc_mailboxDisableNewMsgInt(selfProcId,remoteProcId);

      Ipc_mailboxDisableNewMsgInt(IPC_APP_ENDPT1,IPC_APP_ENDPT1);

      Ipc_mailboxDisableNewMsgInt(IPC_APP_ENDPT1,15U);

      Ipc_mailboxDisableNewMsgInt(15U,IPC_APP_ENDPT1);

      /* Test rpmessage deinitialization */
      RPMessage_deInit();

      UART_printf("Tests finished\n");
      UART_printf("Total tests: %d Passed: %d Failed %d\n", gTotalTests, gTotalTestsPassed, gTotalTestsFailed);
      if (gTotalTests == gTotalTestsPassed)
      {
          UART_printf("All tests have passed.\n\n\n");
      }
      return 0;
}

