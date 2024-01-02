/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file sciclient_pmic_shutdown_testapp.c
 *
 * \brief Calls the PMIC shutdown sequence
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ti/csl/csl_types.h>
#include <ti/osal/osal.h>
#include <ti/osal/TaskP.h>
#include <ti/board/board.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/sciclient/examples/common/sciclient_appCommon.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Test application stack size */
#define APP_TSK_STACK_MAIN              (16U * 1024U)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void taskFxn(void* a0, void* a1);
static void Pmic_powerOff();

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Test application stack */
/* For SafeRTOS on R5F with FFI Support, task stack should be aligned to the stack size */
#if defined(SAFERTOS) && defined (BUILD_MCU)
static uint8_t  gAppTskStackMain[APP_TSK_STACK_MAIN] __attribute__((aligned(APP_TSK_STACK_MAIN))) = { 0 };
#else
static uint8_t  gAppTskStackMain[APP_TSK_STACK_MAIN] __attribute__((aligned(64)));
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int main(void)
{
    TaskP_Handle task;
    TaskP_Params taskParams;

    /*  This should be called before any other OS calls (like Task creation, OS_start, etc..) */
    OS_init();

    /* Initialize the task params */
    TaskP_Params_init(&taskParams);
    /* Set the task priority higher than the default priority (1) */
    taskParams.priority     = 2;
    taskParams.stack        = gAppTskStackMain;
    taskParams.stacksize    = sizeof (gAppTskStackMain);

    task = TaskP_create(&taskFxn, &taskParams);
    if(NULL == task)
    {
        OS_stop();
    }
    OS_start();    /* does not return */

    return(0);
}

static void taskFxn(void* a0, void* a1)
{
    Board_initCfg           boardCfg;

    boardCfg = BOARD_INIT_PINMUX_CONFIG |
               BOARD_INIT_UART_STDIO;
    Board_init(boardCfg);

    Pmic_powerOff();

    return;
}

static void Pmic_powerOff()
{
    App_sciclientPrintf("\n#### Start of PMIC Poweroff Test ####\n");
    int32_t ret=CSL_PASS;

    /* Poweroff */
    ret = Sciclient_pmSetModuleState(TISCI_DEV_BOARD0,
                               TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
                               TISCI_MSG_FLAG_AOP,
                               SCICLIENT_SERVICE_WAIT_FOREVER);
    /* Incase of Sucess, following code should not be excuted */
    App_sciclientPrintf("ERROR!!: Device is still powered ON\n");

    if(ret != CSL_PASS)
    {
        App_sciclientPrintf("Sciclient function call to shutdown PMIC is failed\n");
    }
    else
    {
        App_sciclientPrintf(" Sciclient function call is passed but failed to poweroff the PMIC\n");
    }

    App_sciclientPrintf("Testcase is Failed!!!");

}