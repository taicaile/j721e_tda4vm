/**
 *  \file   main_osal_test.c
 *
 *  \brief  OSAL driver test application main file.
 *
 */

/*
 * Copyright (C) 2016-2022 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#if defined(USE_BIOS)
/* XDCtools Header files */
#include <xdc/std.h>
#if defined (SOC_J721E) || defined(SOC_J7200) || defined(SOC_J721S2) || defined(SOC_J784S4)
// workaround for A72 does not supported in SYS/BIOS yet
#if defined (BUILD_C7X)
#ifndef BARE_METAL
#include <ti/sysbios/family/c7x/Hwi.h>
#endif //BARE_METAL
#endif //BUILD_C7X
#else
#include <xdc/cfg/global.h>
#endif
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/osal/SwiP.h>
#endif

#if !defined(BARE_METAL)
/* include for both safertos and freertos. */
#include <ti/osal/TaskP.h>
#include <ti/osal/EventP.h>
#endif

#if defined (FREERTOS)
#include "FreeRTOS.h"
#include <ti/osal/MemoryP.h>
#include <ti/osal/LoadP.h>
#endif

#if defined (SAFERTOS)
#include "SafeRTOS.h"
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>

/* TI-RTOS Header files */
#include <ti/osal/osal.h>
#include <ti/osal/soc/osal_soc.h>
#include "OSAL_log.h"

#include "OSAL_board.h"

#if defined (SOC_J721E) || defined(SOC_J7200) || defined(SOC_J721S2) || defined(SOC_J784S4)
#include <ti/drv/sciclient/sciclient.h>
#endif
/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/
#define TWO_TIMER_INTERRUPT_TEST 0
#if defined(SOC_TPR12) || defined (SOC_AWR294X)
#define ENABLE_DEBUG_LOG_TEST	1
#endif

#include <ti/csl/soc.h>

#if defined (BUILD_C7X)
#include <ti/csl/csl_clec.h>
#include <ti/csl/arch/csl_arch.h>
#endif
#include <ti/csl/tistdtypes.h>
#if defined (BARE_METAL)
#if !defined(SOC_TPR12) && !defined (SOC_AWR294X)
#include <ti/csl/csl_timer.h>
#endif
#include <ti/csl/arch/csl_arch.h>

#if   defined (SOC_AM571x) || defined (SOC_AM572x) || defined (SOC_AM574x)
#undef  TWO_TIMER_INTERRUPT_TEST
#define TWO_TIMER_INTERRUPT_TEST 1
#endif
#elif defined (USE_BIOS)
void ErrorHandler(Error_Block *eb)
{
    OSAL_log("ErrorHandler: ");
    Error_print(eb);
}
#endif

#if defined(BUILD_C66X)

/* To set C66 timer interrupts on J7ES */
void C66xTimerInterruptInit(void);

    #if defined(SOC_J721E)
    /* To set C66 timer interrupts on J7ES */
        #if defined (BUILD_C66X_1)
            #define OSAL_TEST_CORE_TISCI_ID         (TISCI_DEV_C66SS0_CORE0)
            #define OSAL_TEST_OS_TIMER_TISCI_ID     (TISCI_DEV_TIMER0)
        #endif
        #if defined (BUILD_C66X_2)
            #define OSAL_TEST_CORE_TISCI_ID         (TISCI_DEV_C66SS1_CORE0)
            #define OSAL_TEST_OS_TIMER_TISCI_ID     (TISCI_DEV_TIMER1)
        #endif
        /* DMTimers used by OS */ 
        #if defined(USE_BIOS)
            /* On C66x builds we define OS timer tick in the configuration file to
            * trigger event #21 for C66x_1 and #20 for C66x_2. 
            * Map DMTimer 0 interrupt to these events.
            */
            #define OSAL_TEST_OS_TIMER_INT_NUM          (14U)
            #if defined (BUILD_C66X_1)
                #define OSAL_TEST_OS_TIMER_EVENT_NUM    (21U)
            #endif
            #if defined (BUILD_C66X_2)
                #define OSAL_TEST_OS_TIMER_EVENT_NUM    (20U)
            #endif
        #endif
        #if defined(FREERTOS)
            #define OSAL_TEST_OS_TIMER_INT_NUM          ((uint32_t)configTIMER_INT_NUM) /* 14 for C66x_1; 15 for C66x_2 */
            #define OSAL_TEST_OS_TIMER_EVENT_NUM        ((uint32_t)configTIMER_EVENT_ID) /* 21 for C66x_1; 20 for C66x_2 */
        #endif
        #if defined(SAFERTOS)
            #define OSAL_TEST_OS_TIMER_INT_NUM          ((uint32_t)configTIMER_INT_NUM) /* 14 for C66x_1; 15 for C66x_2 */
            #define OSAL_TEST_OS_TIMER_EVENT_NUM        ((uint32_t)configTIMER_EVENT_ID) /* 21 for C66x_1; 20 for C66x_2 */
        #endif
        /* DMTimers used for OSAL Timer Test */ 
        /* The Event 20/21 is used for DMTimer0/1 by SysBIOS/FreeRTOS by default, 
        * so we need to use a different one here */
        #define OSAL_TEST_TIMER_TISCI_ID                (TISCI_DEV_TIMER2)
        #define OSAL_TEST_TIMER_EVENT_NUM               (22U)
        #define OSAL_TEST_TIMER_INT_NUM                 (OSAL_TEST_OS_TIMER_INT_NUM + 1U)
    #endif
    
#endif

#ifdef BUILD_C7X

/* To set C7x interrupts of DMTimers used for OSAL Timer Test on J721E/J721S2/J784S4 */
void C7x_ConfigureTimerOutput(void);

    /* DMTimers used for OSAL Timer Test */ 
    #if defined(SOC_J721E)
        #define OSAL_TEST_CLEC_BASE_ADDRESS             (CSL_COMPUTE_CLUSTER0_CLEC_REGS_BASE)
        /* DMTimer 2 and Interrupt num 16 is used by SysBIOS/FreeRTOS by default, 
        * so we need to use a different one here */
        #define OSAL_TEST_TIMER_INT_NUM                 (14U)
        #define OSAL_TEST_TIMER_EVENT_NUM               (CSLR_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_TIMER0_INTR_PEND_0 + 992)
    #endif
    #if defined(SOC_J721S2)
        #if defined(FREERTOS)
            #define OSAL_TEST_CLEC_BASE_ADDRESS             (CSL_COMPUTE_CLUSTER0_CLEC_REGS_BASE)
            /* DMTimer 0/1 and Interrupt num 14(configTIMER_INT_NUM) 
            * is used by FreeRTOS by default for C7x_1/C7x_2 respectively, 
            * so we need to use a different one here */
            /* 15 for C7x_1 - DMTimer 1; 15 for C7x_2 - DMTimer 2 */
            #define OSAL_TEST_TIMER_INT_NUM                 (configTIMER_INT_NUM + 1U) 
            #define OSAL_TEST_TIMER_EVENT_NUM               (CSLR_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_TIMER0_INTR_PEND_0 + 992U + configTIMER_ID + 1U)
        #elif defined(SAFERTOS)
            #define OSAL_TEST_CLEC_BASE_ADDRESS             (CSL_COMPUTE_CLUSTER0_CLEC_REGS_BASE)
            /* DMTimer 0/1 and Interrupt num 14 is used by SafeRTOS by default for C7x_1/C7x_2 respectively, 
             * So we need to use a different one here */
            #if defined (BUILD_C7X_1)
                #define OSAL_TEST_TIMER_INT_NUM                 (OSAL_SAFERTOS_OS_TIMER_INT_NUM_C7X_1+1U)
                #define OSAL_TEST_TIMER_EVENT_NUM               (CSLR_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_TIMER0_INTR_PEND_0 + 992U + OSAL_SAFERTOS_OS_TIMER_ID_C7X_1 + 1U)
            #elif defined (BUILD_C7X_2)
                #define OSAL_TEST_TIMER_INT_NUM                 (OSAL_SAFERTOS_OS_TIMER_INT_NUM_C7X_2+1U)
                #define OSAL_TEST_TIMER_EVENT_NUM               (CSLR_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_TIMER0_INTR_PEND_0 + 992U + OSAL_SAFERTOS_OS_TIMER_ID_C7X_2 + 1U)
            #endif
        #endif
    #endif
    #if defined(SOC_J784S4)
        #if defined(FREERTOS)
            #define OSAL_TEST_CLEC_BASE_ADDRESS             (CSL_COMPUTE_CLUSTER0_CLEC_REGS_BASE)
            /* DMTimer 0/1/2/3 and Interrupt num 14(configTIMER_INT_NUM) 
            * is used by FreeRTOS by default for C7x_1/C7x_2/C7x_3/C7x_4 respectively, 
            * so we need to use a different one here */
            /* 15 for C7x_1 - DMTimer 1; 15 for C7x_2 - DMTimer 2 */
            /* 15 for C7x_3 - DMTimer 3; 15 for C7x_4 - DMTimer 4 */
            #define OSAL_TEST_TIMER_INT_NUM                 (configTIMER_INT_NUM + 1U) 
            #define OSAL_TEST_TIMER_EVENT_NUM               (CSLR_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_TIMER0_INTR_PEND_0 + 992U + configTIMER_ID + 1U)
        #elif defined(SAFERTOS)
            #define OSAL_TEST_CLEC_BASE_ADDRESS             (CSL_COMPUTE_CLUSTER0_CLEC_REGS_BASE)
            /* DMTimer 0/1/2/3 and Interrupt num 14 is used by SafeRTOS 
             * by default for C7x_1/C7x_2/C7x_3/C7x_4 respectively, 
             * So we need to use a different one here */
            /* 15 for C7x_1 - DMTimer 1; 15 for C7x_2 - DMTimer 2 */
            /* 15 for C7x_3 - DMTimer 3; 15 for C7x_4 - DMTimer 4 */
            #if defined (BUILD_C7X_1)
                #define OSAL_TEST_TIMER_INT_NUM                 (OSAL_SAFERTOS_OS_TIMER_INT_NUM_C7X_1+1U)
                #define OSAL_TEST_TIMER_EVENT_NUM               (CSLR_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_TIMER0_INTR_PEND_0 + 992U + OSAL_SAFERTOS_OS_TIMER_ID_C7X_1 + 1U)
            #elif defined (BUILD_C7X_2)
                #define OSAL_TEST_TIMER_INT_NUM                 (OSAL_SAFERTOS_OS_TIMER_INT_NUM_C7X_2+1U)
                #define OSAL_TEST_TIMER_EVENT_NUM               (CSLR_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_TIMER0_INTR_PEND_0 + 992U + OSAL_SAFERTOS_OS_TIMER_ID_C7X_2 + 1U)
            #elif defined (BUILD_C7X_3)
                #define OSAL_TEST_TIMER_INT_NUM                 (OSAL_SAFERTOS_OS_TIMER_INT_NUM_C7X_3+1U)
                #define OSAL_TEST_TIMER_EVENT_NUM               (CSLR_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_TIMER0_INTR_PEND_0 + 992U + OSAL_SAFERTOS_OS_TIMER_ID_C7X_3 + 1U)
            #elif defined (BUILD_C7X_4)
                #define OSAL_TEST_TIMER_INT_NUM                 (OSAL_SAFERTOS_OS_TIMER_INT_NUM_C7X_4+1U)
                #define OSAL_TEST_TIMER_EVENT_NUM               (CSLR_COMPUTE_CLUSTER0_CLEC_SOC_EVENTS_IN_TIMER0_INTR_PEND_0 + 992U + OSAL_SAFERTOS_OS_TIMER_ID_C7X_4 + 1U)            
            #endif
        #endif
    #endif

#endif

#undef  ENABLE_GET_TIME_TEST
#if defined(SOC_AM65XX) || defined(SOC_J721E) || defined(SOC_AM572x) || defined(SOC_K2G) || defined(SOC_AM335x) || defined(SOC_AM437x) || defined(SOC_J7200)|| defined(SOC_TPR12) || defined (SOC_AWR294X) || defined (SOC_J721S2) || defined(SOC_J784S4)
#define ENABLE_GET_TIME_TEST     1
#endif

#ifndef NULL_PTR
#define NULL_PTR ((void *)0x0)
#endif

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
#if ( __ARM_ARCH == 5)
TimerP_Handle handle;
#endif

/* Test application stack size */
#if defined (BUILD_C7X)
/* Temp workaround to avoid assertion failure: A_stackSizeTooSmall : Task stack size must be >= 16KB.
  * until the Bug PDK-7605 is resolved */
#define APP_TSK_STACK_MAIN              (64U * 1024U)
#else
#define APP_TSK_STACK_MAIN              (16U * 1024U)
#endif

#ifdef BARE_METAL
/* No task support for Bare metal */
#else
/* Test application stack */
/* For SafeRTOS on R5F with FFI Support, task stack should be aligned to the stack size */
#if defined(SAFERTOS) && defined (BUILD_MCU)
static uint8_t  gAppTskStackMain[APP_TSK_STACK_MAIN] __attribute__(( aligned( APP_TSK_STACK_MAIN ))) = { 0 };
#else
static uint8_t  gAppTskStackMain[APP_TSK_STACK_MAIN] __attribute__(( aligned( 0x2000 )));
#endif
#endif


/* Can not have NOINIT in --ram-model of linking on C7x or C66x cores.
 * Hence, can not test extended_system_pre_init for C7x or C66x cores.
 */
#if defined(BUILD_MCU)
__attribute__((noinit)) uint32_t gStartupHookTestProbeA;
#endif
uint32_t gStartupHookTestProbeB = 0XABABABABU;
uint32_t gStartupHookTestResult = 2U;
void extended_system_pre_init(void)
{
#if defined(BUILD_MCU)
    gStartupHookTestProbeA = 0xBAD0BAD0U;
#endif
}
void extended_system_post_cinit(void)
{
#if defined(BUILD_MCU)
    if ((0xBAD0BAD0U != gStartupHookTestProbeA) || (0XABABABABU != gStartupHookTestProbeB))
    {
        gStartupHookTestResult = 1U;
    }
    else
    {
        gStartupHookTestResult = 0U;
    }
#elif defined (BUILD_C7X) || defined (_TMS320C6X)
    if (0XABABABABU != gStartupHookTestProbeB)
    {
        gStartupHookTestResult = 1U;
    }
    else
    {
        gStartupHookTestResult = 0U;
    }
#else
    gStartupHookTestResult = 0U;
#endif
}

/*
 *  ======== Board_initOSAL ========
 */
void Board_initOSAL(void)
{
    Board_initCfg boardCfg;
    Board_STATUS  status;
#if defined(evmK2E) || defined(evmC6678)
    boardCfg = BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#else
    boardCfg = BOARD_INIT_PINMUX_CONFIG;
    /* For TPR12 AWR294X we dont do module clock init from app. This is done by
     * GEL file or SBL. Doing Module Clock init causes OSAL RTI interrupts
     * to stop if another core also runs OSAL test
     */
    #if !defined(SOC_TPR12) && !defined (SOC_AWR294X)
    #if !defined(_TMS320C6X)
        boardCfg |= BOARD_INIT_MODULE_CLOCK;
    #endif
    #endif /* !defined(SOC_TPR12) */

    #if defined (UART_CONSOLE)
        boardCfg |= BOARD_INIT_UART_STDIO;
    #endif
#endif
    status = Board_init(boardCfg);

    if (status != BOARD_SOK)
    {
        OSAL_log(" Board_init() is not successful...unexpected results may happen \n");
    }

    return;
}

bool OSAL_startup_hook_test()
{
    bool retval = true;
    if( 0U != gStartupHookTestResult )
    {
        retval = false;
    }
    return retval;
}

#if !defined (BARE_METAL)
/*
 *  ======== Floating point test function ========
 */
#define FLOAT_CALC_ITER 100
#if defined (BUILD_MCU) && defined (SAFERTOS)
uint8_t gFloatingNoiseTaskStack[4*1024] __attribute__((aligned(4*1024)));
#else
uint8_t gFloatingNoiseTaskStack[32*1024] __attribute__(( aligned( 0x2000 )));
#endif
SemaphoreP_Handle noiseSem, floatSem;

/* Aim of this task is to do some floating point operations in between operations carried by OSAL_floating_point_test */
void floatingNoiseTaskFxn( void *arg1, void *arg2 )
{
    /* Below variable values are randomly picked. No special significance. */
    volatile double floating_noise = 1.09, val_2p0 = 2.0;
    uint8_t count = FLOAT_CALC_ITER;
    /* FreeRTOS task needs to call portTASK_USES_FLOATING_POINT if configFLOATING_POINT_CONTEXT is 0.
     * SafeRTOS enables FPU by default.
     */
#if defined (FREERTOS) && (configFLOATING_POINT_CONTEXT==0)
    portTASK_USES_FLOATING_POINT();
#endif

    while(count--)
    {
        SemaphoreP_pend(noiseSem, SemaphoreP_WAIT_FOREVER);
        floating_noise = floating_noise + 0.1;
        floating_noise = floating_noise * val_2p0;
        floating_noise = floating_noise / val_2p0;
        SemaphoreP_post(floatSem);
    }
}

bool OSAL_floating_point_test()
{
    /* FreeRTOS task needs to call portTASK_USES_FLOATING_POINT if configFLOATING_POINT_CONTEXT is 0.
     * SafeRTOS enables FPU by default.
     */
    #if defined (FREERTOS) && (configFLOATING_POINT_CONTEXT==0)
    portTASK_USES_FLOATING_POINT();
    #endif
    /* Below variable values are randomly picked. No special significance. */
    volatile double expected_float = 1.07, calculated_float = 1.07, val_1p02 = 1.02;
    uint8_t count = FLOAT_CALC_ITER;
    TaskP_Params taskParams;
    SemaphoreP_Params semParams;
    TaskP_Handle taskHandle;
    bool retval = false;
 
    /* Initialize and create Noise task and task faculties. */
    TaskP_Params_init(&taskParams);
    taskParams.priority = 2;
    taskParams.stack = gFloatingNoiseTaskStack;
    taskParams.stacksize = sizeof(gFloatingNoiseTaskStack);
    SemaphoreP_Params_init(&semParams);
    noiseSem = SemaphoreP_create(1U, &semParams);
    SemaphoreP_Params_init(&semParams);
    floatSem = SemaphoreP_create(0U, &semParams);
 
    taskHandle = TaskP_create(&floatingNoiseTaskFxn, &taskParams);
    if (taskHandle == NULL)
    {
        OSAL_log("\n Floating Noise Task could not be created!!\n");
    }
   
    /* Floating point calculations under test here, while exchanging CPU time with the noisy floating point task. */
    while (count--)
    {
        SemaphoreP_pend(floatSem, SemaphoreP_WAIT_FOREVER);
        calculated_float = pow(calculated_float, val_1p02);
        SemaphoreP_post(noiseSem);
    }
    /* Calculate the expected value. */
    count = FLOAT_CALC_ITER;
    while( count-- )
    {
        expected_float = pow(expected_float, 1.02);
    }
   
    OSAL_log("\n Calculated float is %u.%u, and expected float is %u.%u ...\n", (uint32_t)calculated_float, (uint32_t)((calculated_float-(uint32_t)calculated_float)*1000000), (uint32_t)expected_float, (uint32_t)((expected_float-(uint32_t)expected_float)*1000000));
 
    /* Match the expected and the calculated value. if same return true, else return false. */
    if (calculated_float == expected_float)
    {
        retval = true;
    }
 
    return retval;
}
#endif

/*
 *  ======== HWI test function ========
 */
volatile   uint64_t gTestlocalTimeout = 0x300000U;

#if (defined (SOC_AM65XX) || defined (SOC_AM64X) || defined(SOC_J721E) || defined(SOC_J7200) || defined (SOC_TPR12) || defined (SOC_AWR294X) || defined (SOC_J721S2) || defined(SOC_J784S4)) && (!defined(BUILD_C66X))&&(!defined(BUILD_C7X))
#define INT_NUM_IRQ 32U
#define LOOP_CNT    100
volatile uint64_t gFlagIRQ = 0;

void myIsrIRQ(uintptr_t arg)
{
    gFlagIRQ = 1;
    gTestlocalTimeout = 0x300000;
}
bool  OSAL_core_hwi_test()
{

    HwiP_Params hwiParams;
    HwiP_Handle handle;
    volatile int intCount = 0;
    int32_t ret;
    bool test_pass=true;

    HwiP_Params_init(&hwiParams);

    OSAL_log("Creating Hwi myIsr \n");
    handle = HwiP_create(INT_NUM_IRQ, myIsrIRQ, &hwiParams);
    if (handle == NULL_PTR) {
      OSAL_log("Failed to create the HwiP handle \n");
      test_pass = false;
    }

    if (test_pass == true)
    {
        while (intCount != LOOP_CNT)
        {
            ret=HwiP_post(INT_NUM_IRQ);

            if(ret==osal_UNSUPPORTED)
            { /* In case of unsupported SOC/error */
              OSAL_log("HwiP_post unsupported/failed!\n");
              test_pass=false;
              break;
            }

            /* Wait for software timeout, ISR should hit
             * otherwise return the test as failed */
            while (gTestlocalTimeout != 0U)
            {
              gTestlocalTimeout--;
              if (gFlagIRQ)
              {
                gFlagIRQ = 0;
                intCount++;
                break;
              }
            }
            /* Wait is over - did not get any interrupts posted/received
             * declare the test as fail
             */
            if (gTestlocalTimeout == 0)
            {
              OSAL_log("Failed to get interrupts \n");
              test_pass = false;
              break;
            }
        }

        OSAL_log("%d IRQs received. Test over !\n",intCount);
        ret = HwiP_delete(handle);
        if (ret != HwiP_OK)
        {
          OSAL_log(" Failed to delete HwiP handle \n");
          test_pass= false;
        }
    }

    return test_pass;
}
#endif

bool OSAL_hwi_test()
{
  bool pass = true;
#if (defined (SOC_AM65XX) || defined (SOC_AM64X) || defined(SOC_J721E) || defined(SOC_J7200) || defined (SOC_TPR12) || defined (SOC_AWR294X) || defined (SOC_J721S2) || defined(SOC_J784S4)) && (!defined(BUILD_C66X))&&(!defined(BUILD_C7X))
  pass = OSAL_core_hwi_test();
#endif
  return pass;
}

typedef enum UT_Timer_Type_s {
    UT_Timer_DMTIMER = 0,
    UT_Timer_TIMER64 = 1,
    UT_Timer_RTITIMER = 2
} UT_Timer_Type_t;


#if defined(SOC_AM335x)
#define CM_PER_TIMER3_CLKCTRL             (0x44e00084)
#define CM_DPLL_CLKSEL_TIMER3_CLK         (0x44e0050c)
UT_Timer_Type_t  timer_type    =          UT_Timer_DMTIMER;
#define OSAL_TEST_TIMER_ID                (1U)
#define OSAL_TEST_TIMER_PERIOD            (5000U)
#elif defined(SOC_AM437x)
#define CM_PER_TIMER3_CLKCTRL             (0x44DF8800 + 0x538)
#define CM_DPLL_CLKSEL_TIMER3_CLK         (0x44DF4200 + 0x8)
UT_Timer_Type_t  timer_type  =            UT_Timer_DMTIMER;
#define OSAL_TEST_TIMER_ID                (3U)
#define OSAL_TEST_TIMER_PERIOD            (5000U)

#elif defined(SOC_K2E) || defined(SOC_K2L)
UT_Timer_Type_t  timer_type =             UT_Timer_TIMER64;
#define OSAL_TEST_TIMER_ID                (TimerP_ANY)
#define OSAL_TEST_TIMER_PERIOD            (5000U)
#elif defined(SOC_K2H) || defined(SOC_K2K)
UT_Timer_Type_t  timer_type =             UT_Timer_TIMER64;
#define OSAL_TEST_TIMER_ID                (8U)
#define OSAL_TEST_TIMER_PERIOD            (5000U)
#elif defined(SOC_K2G)
UT_Timer_Type_t  timer_type =             UT_Timer_TIMER64;
#define OSAL_TEST_TIMER_ID                (1U)
#define OSAL_TEST_TIMER_ID2               (2U)
#define OSAL_TEST_TIMER_PERIOD            (5000U)
#elif defined(SOC_AM572x) || defined (SOC_AM574x)
UT_Timer_Type_t  timer_type   =           UT_Timer_DMTIMER;
  #if defined (__ARM_ARCH_7A__)
    #define OSAL_TEST_TIMER_ID                (1U)
    #define OSAL_TEST_TIMER_ID2               (9U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif   defined (_TMS320C6X)
    #define OSAL_TEST_TIMER_ID                (4U)
    #define OSAL_TEST_TIMER_ID2               (5U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #else
    #define OSAL_TEST_TIMER_ID                (8U)
    #define OSAL_TEST_TIMER_ID2               (10U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #endif
#elif defined(SOC_AM571x)
UT_Timer_Type_t  timer_type   =           UT_Timer_DMTIMER;
  #if defined (__ARM_ARCH_7A__)
    #define OSAL_TEST_TIMER_ID                (1U)
    #define OSAL_TEST_TIMER_ID2               (9U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif   defined (_TMS320C6X)
    #define OSAL_TEST_TIMER_ID                (4U)
    #define OSAL_TEST_TIMER_ID2               (5U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #else
    #define OSAL_TEST_TIMER_ID                (8U)
    #define OSAL_TEST_TIMER_ID2               (10U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #endif
#elif defined(SOC_OMAPL137) || defined (SOC_OMAPL138)
UT_Timer_Type_t  timer_type =             UT_Timer_TIMER64;
#define OSAL_TEST_TIMER_ID                (1U)
#define OSAL_TEST_TIMER_PERIOD            (5000U)

#elif defined(SOC_AM65XX)
  UT_Timer_Type_t  timer_type    =          UT_Timer_DMTIMER;
  #if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R') 
    #define OSAL_TEST_TIMER_ID                (1U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #else
    #define OSAL_TEST_TIMER_ID                (4U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #endif
#elif defined(SOC_J721E)
  UT_Timer_Type_t  timer_type    =          UT_Timer_DMTIMER;
  #if defined (BUILD_MCU1_0)
    #define OSAL_TEST_TIMER_ID                (2U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R') 
    #define OSAL_TEST_TIMER_ID                (1U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif defined (BUILD_C66X_1)
    #define OSAL_TEST_TIMER_ID                (2U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif defined (BUILD_C66X_2)
    #define OSAL_TEST_TIMER_ID                (2U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif defined (BUILD_C7X_1)
    #define OSAL_TEST_TIMER_ID                (0U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #else
    #define OSAL_TEST_TIMER_ID                (2U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #endif
#elif defined(SOC_J721S2)
  UT_Timer_Type_t  timer_type    =          UT_Timer_DMTIMER;
  #if defined (BUILD_MCU1_0)
    #define OSAL_TEST_TIMER_ID                (2U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R') 
    #define OSAL_TEST_TIMER_ID                (1U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif defined (BUILD_C7X_1)
    #define OSAL_TEST_TIMER_ID                (1U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif defined (BUILD_C7X_2)
    #define OSAL_TEST_TIMER_ID                (2U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #else
    #define OSAL_TEST_TIMER_ID                (6U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #endif
#elif defined(SOC_J784S4)
  UT_Timer_Type_t  timer_type    =          UT_Timer_DMTIMER;
  #if defined (BUILD_MCU1_0)
    #define OSAL_TEST_TIMER_ID                (2U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R') 
    #define OSAL_TEST_TIMER_ID                (1U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif defined (BUILD_C7X_1)
    #define OSAL_TEST_TIMER_ID                (1U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif defined (BUILD_C7X_2)
    #define OSAL_TEST_TIMER_ID                (2U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif defined (BUILD_C7X_3)
    #define OSAL_TEST_TIMER_ID                (3U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif defined (BUILD_C7X_4)
    #define OSAL_TEST_TIMER_ID                (4U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #else
    #define OSAL_TEST_TIMER_ID                (6U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #endif
#elif defined(SOC_J7200)
  UT_Timer_Type_t  timer_type    =          UT_Timer_DMTIMER;
  #if defined (BUILD_MCU1_0)
    #define OSAL_TEST_TIMER_ID                (2U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R') 
    #define OSAL_TEST_TIMER_ID                (1U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #else
    #define OSAL_TEST_TIMER_ID                (2U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #endif
#elif defined(SOC_AM64X)
  UT_Timer_Type_t  timer_type    =          UT_Timer_DMTIMER;
  #if defined(BUILD_MCU)
    #define OSAL_TEST_TIMER_ID                (1U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #endif
  #if defined(BUILD_MPU)
    #define OSAL_TEST_TIMER_ID                (4U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #endif
  #if defined(BUILD_M4F)
    #define OSAL_TEST_TIMER_ID                (1U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #endif
#elif (defined(SOC_TPR12) || defined (SOC_AWR294X))
    UT_Timer_Type_t  timer_type =           UT_Timer_RTITIMER;
#define OSAL_TEST_TIMER_ID                    (TimerP_ANY)
#if defined(SIM_BUILD)
#define OSAL_TEST_TIMER_PERIOD                (500U)
#else
#define OSAL_TEST_TIMER_PERIOD                (5000U)
#endif
#else
UT_Timer_Type_t  timer_type   =           UT_Timer_DMTIMER;
  #if defined (__ARM_ARCH_7A__)
    #define OSAL_TEST_TIMER_ID                (1U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #elif   defined (_TMS320C6X)
    #define OSAL_TEST_TIMER_ID                (4U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #else
    #define OSAL_TEST_TIMER_ID                (8U)
    #define OSAL_TEST_TIMER_PERIOD            (5000U)
  #endif
#endif

#if defined(SIM_BUILD)
#define      OSAL_GET_TIME_MAX_SAMPLES  (10U)
#else
#define      OSAL_GET_TIME_MAX_SAMPLES  (20U)
#endif
volatile uint32_t timerIsrCount = 0;
volatile uint32_t timerIsr2Count = 0;

#if defined(SIM_BUILD)
#define OSAL_TIMER_TEST_MAX_INTERRUPTS      (10U)
#else
#define OSAL_TIMER_TEST_MAX_INTERRUPTS      (100U)
#endif


#if defined (ENABLE_GET_TIME_TEST)
uint64_t     gTestTimeRd[OSAL_GET_TIME_MAX_SAMPLES];
#endif

void timerIsr(void *arg)
{
#if (defined(BARE_METAL) && ( __ARM_ARCH == 5))
    TimerP_ClearInterrupt(handle);
#endif

#if defined (ENABLE_GET_TIME_TEST)
    if (timerIsrCount < OSAL_GET_TIME_MAX_SAMPLES)
    {
        gTestTimeRd[timerIsrCount] = TimerP_getTimeInUsecs();
    }
#endif
    timerIsrCount++;
}


#if    TWO_TIMER_INTERRUPT_TEST
void timerIsr2(void *arg)
{
    timerIsr2Count++;
}
#endif

/*
 * ========= Osal Delay Test function =========
 */
#if defined(SIM_BUILD)
#define OSAL_DELAY_TIME         1
#else
#define OSAL_DELAY_TIME         10
#endif
bool Osal_delay_test(void)
{
#if !defined(SOC_AM64X)
   int32_t i;
  /* This test assumes that Board init has been already
   * called outside this function
   */
   /* Notice a '.' on terminal for every 1 second */

   for (i=0; i<10;i++) {
#endif
      Osal_delay(OSAL_DELAY_TIME);
      OSAL_log(".");
#if !defined(SOC_AM64X)
   }
#endif
   OSAL_log("\n");

  return(true);
}

#if defined (ENABLE_GET_TIME_TEST)
/*
 * ========= Osal getTime Test function =========
 */
bool Osal_getTime_test(void)
{
   int32_t      i;
   bool         retVal   = true;

   OSAL_log(" \n     time read in micro seconds is \n ");
   for (i = 0; i < OSAL_GET_TIME_MAX_SAMPLES; i++)
   {
       OSAL_log(" %d \n ", (uint32_t) gTestTimeRd[i]);
   }

   return(retVal);
}
#endif

/*
 *  ======== Timer test function ========
 */
bool OSAL_timer_test()
{
    TimerP_Params timerParams;
#if ( __ARM_ARCH != 5)
    /* Timer handle is defined as global for OMAPL13x ARM9 core since
       it is required for clearing the interrupt from ISR */
    TimerP_Handle handle;
#endif

#if    TWO_TIMER_INTERRUPT_TEST
    TimerP_Handle handle2;
#endif
    TimerP_Status timerStatus;
    Osal_HwAttrs  hwAttrs;

#if defined(BARE_METAL)
    int32_t       id    = OSAL_TEST_TIMER_ID;
#else
    /* Pass TimerP_ANY for FreeRTOS, since FreeRTOS kernel running on each core
     * will use one timer per core. So if there is a conflict in the id passed
     * TimerP_create will fail. See FreeRTOS_config_<core>.h for details about
     * the timer instance used by each core */
    int32_t       id    = TimerP_ANY;
#endif

#if defined(SOC_AM437x)
    /* We can't use Timer_ANY for Sys BIOS as it allocates Timer0 which is
     * firewalled on HS device and hence changing the Test Timer ID = 3 for
     * both HS and GP parts
     */
    id                  = OSAL_TEST_TIMER_ID;
#endif

#if defined(SOC_J721E) || defined(SOC_J7200) || defined(SOC_J721S2) || defined(SOC_J784S4)
#if !(defined(BARE_METAL) || defined(FREERTOS))
#if defined(BUILD_C66X) || defined(BUILD_C7X) || defined(BUILD_MCU1_0)
    id                  = OSAL_TEST_TIMER_ID;
#endif
#endif
#endif
#if defined(SOC_J721E)
#if defined(FREERTOS)
#if defined(BUILD_C66X)
    id                  = OSAL_TEST_TIMER_ID;
#endif
#endif
#endif
#if defined(SOC_J721E) || defined(SOC_J721S2) || defined(SOC_J784S4)
#if defined(BUILD_C7X)
    id                  = OSAL_TEST_TIMER_ID;
#endif
#endif

    volatile      int32_t i;
    uint32_t      prevCount, ctrlBitmap = OSAL_HWATTR_SET_OSALDELAY_TIMER_BASE ;
    bool          ret = true;
    int32_t       osal_ret;

#if (defined (SOC_AM437x) || defined (SOC_AM335x))
    *(unsigned int*)CM_DPLL_CLKSEL_TIMER3_CLK = 0x1; /* high frequency input clock */

    // enable the TIMER
    *(unsigned int*)CM_PER_TIMER3_CLKCTRL = 0x2; /* Module is explicitly enabled */
#endif

    /* Set the timer base for the osal delay */
    Osal_getHwAttrs(&hwAttrs);

#if defined (SOC_AM437x)
    hwAttrs.osalDelayTimerBaseAddr = (uintptr_t) 0x48040000U; /* DMTIMER2 */
#endif

#if defined (OSAL_TEST_CPU_FREQ_KHZ)
    hwAttrs.cpuFreqKHz = OSAL_TEST_CPU_FREQ_KHZ;
    ctrlBitmap        |= OSAL_HWATTR_SET_CPU_FREQ;
#endif

    /* This API should return osal_OK for AM3/AM4 and return unsupported for other Socs */
    osal_ret = Osal_setHwAttrs(ctrlBitmap, &hwAttrs);

#if defined (SOC_AM437x) || defined (SOC_AM335x)
    if (osal_ret == osal_OK) {
      ret = true;
    }
    else {
      ret = false;
    }
#else
    if ((osal_ret == osal_UNSUPPORTED) ||
       (osal_ret == osal_OK)) {
      ret = true;
    }
    else {
      ret = false;
    }
#endif /* (SOC_AM437x) || defined (SOC_AM335x) */

    TimerP_Params_init(&timerParams);
    timerParams.runMode    = TimerP_RunMode_CONTINUOUS;
    timerParams.startMode  = TimerP_StartMode_USER;
    timerParams.periodType = TimerP_PeriodType_MICROSECS;
    timerParams.period     = OSAL_TEST_TIMER_PERIOD;

#if defined(SOC_J721E) || defined(SOC_J721S2) || defined(SOC_J784S4)
#if defined(_TMS320C6X)
	timerParams.eventId    = OSAL_TEST_TIMER_EVENT_NUM;
    timerParams.intNum     = OSAL_TEST_TIMER_INT_NUM;
    OSAL_log("\n set intNum=%d, eventId=%d, id=%d,  \n", timerParams.intNum, timerParams.eventId, id);
#endif
#endif

#if defined(SOC_J721E) || defined(SOC_J721S2) || defined(SOC_J784S4)
#if defined(BUILD_C7X)
    timerParams.intNum     = OSAL_TEST_TIMER_INT_NUM;
    OSAL_log("\n set intNum=%d, id=%d,  \n", timerParams.intNum, id);
#endif
#endif

#if defined(_TMS320C6X)
#if defined(SOC_TPR12) || defined(SOC_AWR294X)
    timerParams.intNum     = 16;
    OSAL_log("\n set intNum=%d, id=%d,  \n", timerParams.intNum, id);
#endif
#endif

    if (timer_type == UT_Timer_TIMER64)
    {
      timerParams.timerMode = TimerP_Timer64Mode_UNCHAINED;
      timerParams.timerHalf = TimerP_Timer64Half_LOWER;
    }
    handle = TimerP_create(id, (TimerP_Fxn)&timerIsr, &timerParams);

    /* don't expect the handle to be null */
    if (handle == NULL_PTR)
    {
      OSAL_log("\n Timer Create error \n");
      ret = false;
    }

#if defined(ENABLE_GET_TIME_TEST)
     TimerP_getTimeInUsecs();
#endif

#if    TWO_TIMER_INTERRUPT_TEST

#if (defined(_TMS320C6X) && (!defined(SOC_J721E)))
    timerParams.intNum     = 14;
#endif

    if (ret == true)  {
      handle2 = TimerP_create(OSAL_TEST_TIMER_ID2, (TimerP_Fxn)&timerIsr2, &timerParams);
      /* don't expect the handle to be null */
      if (handle2 == NULL_PTR)
      {
        OSAL_log("\n Timer Create error for handle2 \n");
        ret = false;
      }
    }
#endif

    if (ret == true)
    {
      timerIsrCount = 0U;
      timerStatus = TimerP_start(handle);

      if (timerStatus != TimerP_OK) {
        OSAL_log("Err: Coult not start the timer %d \n", id);
        ret = false;
      }

#if    TWO_TIMER_INTERRUPT_TEST
      timerIsr2Count = 0U;
      timerStatus = TimerP_start(handle2);

      if (timerStatus != TimerP_OK) {
        OSAL_log("Err: Coult not start the timer %d \n", id + 1);
        ret = false;
      }

      while (1)
      {
        if (timerIsr2Count >= OSAL_TIMER_TEST_MAX_INTERRUPTS) {
          timerStatus = TimerP_stop(handle2);
          if (timerStatus != TimerP_OK) {
            OSAL_log("Err: Coult not stop the timer %d \n", id);
            ret = false;
          }
          break;
        }
      }
#endif

      while (1)
      {
        if (timerIsrCount >= OSAL_TIMER_TEST_MAX_INTERRUPTS) {
          timerStatus = TimerP_stop(handle);
          if (timerStatus != TimerP_OK) {
            OSAL_log("Err: Coult not stop the timer %d \n", id);
            ret = false;
          }

          break;
        }
      }

      /* Latch the ISR count after the timer stops */
      prevCount = (timerIsrCount);

      /* Give some time for all pending interrupts to be processed */
      for(i = 0; i < 1000; i++)
      {
        asm (" nop ");
      }

      /* Now check if timer ISR gets kicked in
       * timer ISR should not be kicked in */
      if (ret == true)
      {
         for (i = 0; i < 50000; i++);

         if (prevCount != timerIsrCount)
         {
           OSAL_log(" Error: Timer appears to be not stopped via OSAL calls, val1 = %u and val2 = %u \n ", prevCount, timerIsrCount);
           ret = false;
         }
         else
         {
           OSAL_log("  TimerTestLog: got %u ticks from timer id = %d \n", timerIsrCount, id);
#if    TWO_TIMER_INTERRUPT_TEST
           OSAL_log("  TimerTestLog: got %u ticks from timer id = %d \n", timerIsr2Count, id+1);
#endif
         }
       }
    }

    if (handle != NULL_PTR)
    {
      if (TimerP_delete(handle) != TimerP_OK)
      {
        ret = false;
      }
    }

    for (i = 0; i < (log2(TIMERP_AVAILABLE_MASK+1U)+1U); i++)
    {
        TimerP_Params_init(&timerParams);
        timerParams.runMode    = TimerP_RunMode_CONTINUOUS;
        timerParams.startMode  = TimerP_StartMode_USER;
        timerParams.periodType = TimerP_PeriodType_MICROSECS;
        timerParams.period     = OSAL_TEST_TIMER_PERIOD;
        handle = TimerP_create(TimerP_ANY, NULL, &timerParams);
        /* don't expect the handle to be null */
        if (handle == NULL_PTR)
        {
            OSAL_log("\n Error: Timer Create failed for %d th time \n", i);
            ret = false;
        }
        if (TimerP_delete(handle) != TimerP_OK)
        {
            OSAL_log("\n Error: Timer Delete failed for %d th time \n", i);
            ret = false;
        }
    }    
    return (ret);
}

/*
 *  ======== Clock test function ========
 */
bool OSAL_clock_test()
{
    return true;
}

#ifdef MANUAL_CACHE_TEST

#if defined (__ARM_ARCH_7A__)
#include <ti/csl/csl_a15.h>
#include <ti/csl/csl_armGicAux.h>
#elif defined (_TMS320C6X)
#include <ti/csl/csl_cacheAux.h>
#endif

#if defined (SOC_AM437x) || defined(SOC_AM335x)
#include <ti/starterware/include/chipdb.h>
#include <ti/starterware/include/armv7a/pl310.h>
#include <ti/starterware/include/armv7a/pl310_config.h>
#endif

#define BUF_LEN 256
#define LOCAL_CPU_WRITE_PATTERN_1 0xA1A1A1A1
#define LOCAL_CPU_WRITE_PATTERN_2 0xA2A2A2A2
#define LOCAL_CPU_WRITE_PATTERN_3 0xA3A3A3A3

#define ALTERNATE_CPU_WRITE_PATTERN_1 0xB1B1B1B1
#define ALTERNATE_CPU_WRITE_PATTERN_2 0xB2B2B2B2
#define ALTERNATE_CPU_WRITE_PATTERN_3 0xB3B3B3B3

#if defined (__ARM_ARCH_7A__)
uint32_t ddr_buf[BUF_LEN] __attribute__((aligned(256))); // GCC way of aligning
#elif defined (_TMS320C6X)
#pragma DATA_SECTION(ddr_buf,".cachetest_ddrbuf")
#pragma DATA_ALIGN(ddr_buf,256)
uint32_t ddr_buf[BUF_LEN];
#elif defined(__TI_ARM_V7M4__)
uint32_t ddr_buf[BUF_LEN];
#endif
void OSAL_cache_test_fill_buf(uint32_t *buf, uint32_t len,uint32_t pattern) {
  uint32_t i;
  for(i=0;i<len;i++) {
     buf[i]=pattern;
  }
}
/* Returns FALSE if all the buffer is not 'pattern'. This will read the contents though */
bool OSAL_cache_test_check_buf(uint32_t *buf, uint32_t len,uint32_t pattern) {
  uint32_t i;
  bool match=TRUE;
  for(i=0;i<len;i++) {
     if(buf[i]!=pattern) {
       match=FALSE;
     }
  }
  return(match);
}

#if defined (SOC_AM572x) || defined (SOC_AM574x)
bool OSAL_cache_test()
{

  bool match,res=TRUE;
#if defined (__ARM_ARCH_7A__)
  CSL_a15EnableCache();
#elif defined(_TMS320C6X)
    /* Enable for most of the DDR3 memory , especially the first two MAR regions where ddr_buf will reside */
    CACHE_enableCaching(128);
    CACHE_enableCaching(129);
    CACHE_setL1DSize(CACHE_L1_4KCACHE);
    CACHE_setL2Size(CACHE_1024KCACHE);
#endif
  OSAL_log("This manual cache test is a CCS based test. Please do the following\n");
  OSAL_log("1) On the local CPU open two memory windows for the address 0x%x\n: One CPU view and One Physical memory view\n",&ddr_buf[0]);
  OSAL_log("2) Connect an alternate CPU (for example if local cpu is A15, open a c66/m4 as the alternate CPU\n");
  OSAL_log("3) On the alternate CPU ( preferably a different window through (CCS->New Window)) Open the address 0x%x\n with a physical memory view\n",&ddr_buf[0]);

  OSAL_log("\n**** Forcing a read to get the buffer cached() ******\n");
  OSAL_cache_test_check_buf(ddr_buf,BUF_LEN,0); /* Checking for dummy pattern, just to force the cache read */

  OSAL_log("\n**** Testing Cache_Invalidate() ******\n");

  OSAL_log("\nWriting  0x%x local ddr memory\n",LOCAL_CPU_WRITE_PATTERN_1);
    /************************ TEST for CacheP_Inv() *********************/
    /* A15- Write LOCAL_CPU_WRITE_PATTERN_1 in to ddr_buf[0] */
    OSAL_cache_test_fill_buf(ddr_buf,BUF_LEN,LOCAL_CPU_WRITE_PATTERN_1);

    /* Check in the alternate cpu window (m4/c66x/a15) if it can see it (Physical memory). It shouldnt if cache is enabled. */
    OSAL_log("\nCheck in the alternate cpu window (m4/c66x/a15) if it can see it (Physical memory). It shouldn't appear as it is still in cache\n");

    OSAL_log("\nInvalidating cache.\n");
    /* Now OSAL_CacheInv() - Invalidates and writes back cache() */
    CacheP_Inv(ddr_buf,BUF_LEN*sizeof(uint32_t));

    OSAL_log("\nCheck in alternate cpu window (m4/c66x/a15) if it can see it (Physical memory). For A15 it should (Invalidate also writes back for A15), for others it should not \n");

    OSAL_log("\nNow write(%d 32-bit words) of pattern 0x%x on the alternate CPU physical memory window (a15/m4/c66x) \n",BUF_LEN,ALTERNATE_CPU_WRITE_PATTERN_1);
    OSAL_log("\nCheck if this pattern 0x%x has made it to the physical memory view of this cpu.It should. \n",ALTERNATE_CPU_WRITE_PATTERN_1);

#if defined(__ARM_ARCH_7A__)
    /* The below invalidate test is to account for ARM prefetching. On A15 Cache Inv writes back too. To see any changes AFTER the DDR was
    externally modified, a Cache Invalidate is required */
    CacheP_Inv(ddr_buf,BUF_LEN*sizeof(uint32_t));
#endif
    OSAL_log("\nChecking if local CPU read fetches the 0x%x value updated by the alternate cpu core\n",ALTERNATE_CPU_WRITE_PATTERN_1);
    /* Now read the value on A15 now */
    match=OSAL_cache_test_check_buf(ddr_buf,BUF_LEN,ALTERNATE_CPU_WRITE_PATTERN_1);
    if(match==TRUE) {
        OSAL_log("\nPattern matched. OSAL_CacheInv() Test passed\n");
    } else {
        OSAL_log("\n OSAL_CacheInv() Test failed\n");
    }

    /************************ TEST for CacheP_wb() *********************/
    OSAL_log("\n**** Testing Cache_WriteBack() ******\n");

    OSAL_log("\nWriting  0x%x local ddr memory\n",LOCAL_CPU_WRITE_PATTERN_2);
    /* local cpu- Write LOCAL_CPU_WRITE_PATTERN_2 in to ddr_buf[0] */
    OSAL_cache_test_fill_buf(ddr_buf,BUF_LEN,LOCAL_CPU_WRITE_PATTERN_2);

    OSAL_log("\nCheck in the alternate cpu window (m4/c66x/a15) if the pattern 0x%x can be seen it (Physical memory). It shouldn't appear as it is still in cache\n",LOCAL_CPU_WRITE_PATTERN_2);

    OSAL_log("\nWriting back cache.\n");
    /* Now OSAL_CacheWb() - Writes back cache() */
    CacheP_wb(ddr_buf,BUF_LEN*sizeof(uint32_t));

    /* Check in DSP window if it can see it (Physical memory). It should now. */
    OSAL_log("\nCheck in alternate cpu window (m4/c66x/a15) and verify that the 0x%x is seen in Physical memory\n",LOCAL_CPU_WRITE_PATTERN_2);

    /* If so, write back portion is tested. */
    OSAL_log("\nNow write the pattern 0x%x on the alternate CPU physical memory window (a15/m4/c66x) \n",ALTERNATE_CPU_WRITE_PATTERN_2);
    /* Since, on this local cpu, we just wrote back cache() only, and not invalidated, the ALTERNATE_CPU_WRITE_PATTERN_2 wont be seen in the local cpu's cached memory
       Read Variable on A15 to ensure CPU Read(). The value should be the old LOCAL_CPU_WRITE_PATTERN_2.. and NOT the ALTERNATE_CPU_WRITE_PATTERN_2.. which the alternate CPU wrote. */
    match=OSAL_cache_test_check_buf(ddr_buf,BUF_LEN,LOCAL_CPU_WRITE_PATTERN_2);
    if(match==TRUE) {
        OSAL_log("\nPattern matched. OSAL_Cache_Writeback() Test passed\n");
     } else {
        OSAL_log("\n OSAL_Cache_Writeback() Test failed\n");
     }

    /************************ TEST for CacheP_wbInv() *********************/
     OSAL_log("\n**** Testing Cache_WriteBack_Invalidate() ******\n");

     OSAL_log("\nWriting  0x%x local ddr memory\n",LOCAL_CPU_WRITE_PATTERN_3);

     /* local cpu- Write LOCAL_CPU_WRITE_PATTERN_3 in to ddr_buf[0] */
     OSAL_cache_test_fill_buf(ddr_buf,BUF_LEN,LOCAL_CPU_WRITE_PATTERN_3);

     OSAL_log("\nCheck in the alternate cpu window (m4/c66x/a15) if the pattern 0x%x can be seen it (Physical memory). It shouldn't appear as it is still in cache\n",LOCAL_CPU_WRITE_PATTERN_3);

     OSAL_log("\nWriting back cache and invalidating it.\n");

     /* Now OSAL_CacheWbInv() - Writes back cache and invalidate() */
      CacheP_wbInv(ddr_buf,BUF_LEN*sizeof(uint32_t));

      /* Check in DSP window if it can see it (Physical memory). It should now. */
      OSAL_log("\nCheck in alternate cpu window (m4/c66x/a15) and verify that the 0x%x is seen in Physical memory. If not it is a failure \n",LOCAL_CPU_WRITE_PATTERN_3);


      OSAL_log("\nNow write the pattern 0x%x on the alternate CPU physical memory window (a15/m4/c66x) \n",ALTERNATE_CPU_WRITE_PATTERN_3);

#if defined(__ARM_ARCH_7A__)
    /* This is due to ARM prefetching */
     CacheP_Inv(ddr_buf,BUF_LEN*sizeof(uint32_t));
#endif

    /* Since, on local cpu, we not only wrote back cache but also invalidated it,the local cpu should be able the alternate pattern now */
    match=OSAL_cache_test_check_buf(ddr_buf,BUF_LEN,ALTERNATE_CPU_WRITE_PATTERN_3);
    if(match==TRUE) {
        OSAL_log("\nPattern matched. OSAL_Cache_WritebackInvalidate() Test passed\n");
     } else {
        OSAL_log("\nOSAL_Cache_WritebackInvalidate() Test failed\n");
     }

     return(res);
}
#endif /* SOC_AM572x || SOC_AM574x */
#endif /* MANUAL_CACHE_TEST */

/*
 *  ======== Extended memory block test function ========
 * This test aims at testing the create and delete functions that
 * are enhanced to support extended memory blocks for SemaphoreP and HwiP
 */
#define OSAL_TEST_NUM_EXT_SEMAPHORES    (1U)
#define OSAL_TEST_NUM_EXT_HWIPS         (1U)

#if defined (BARE_METAL)
  #define SEMP_BLOCK_SIZE (OSAL_TEST_NUM_EXT_SEMAPHORES * OSAL_NONOS_SEMAPHOREP_SIZE_BYTES)
  #define HWIP_BLOCK_SIZE (OSAL_TEST_NUM_EXT_HWIPS * OSAL_NONOS_HWIP_SIZE_BYTES)
  uint8_t semPMemBlock[SEMP_BLOCK_SIZE];
  uint8_t hwiPMemBlock[HWIP_BLOCK_SIZE];
#elif defined (FREERTOS)
  #define SEMP_BLOCK_SIZE (OSAL_TEST_NUM_EXT_SEMAPHORES * OSAL_FREERTOS_SEMAPHOREP_SIZE_BYTES)
  #if defined (BUILD_C7X)
    #define HWIP_BLOCK_SIZE (OSAL_TEST_NUM_EXT_HWIPS * OSAL_FREERTOS_HWIP_C7X_SIZE_BYTES)
  #else
    #define HWIP_BLOCK_SIZE (OSAL_TEST_NUM_EXT_HWIPS * OSAL_NONOS_HWIP_SIZE_BYTES)
  #endif
  uint8_t semPMemBlock[SEMP_BLOCK_SIZE]
  __attribute__ ((aligned(64)));
  uint8_t hwiPMemBlock[HWIP_BLOCK_SIZE]
  __attribute__ ((aligned(64)));
#elif defined (SAFERTOS)
  #define SEMP_BLOCK_SIZE (OSAL_TEST_NUM_EXT_SEMAPHORES * OSAL_SAFERTOS_SEMAPHOREP_SIZE_BYTES)
  #if defined (BUILD_C7X)
    #define HWIP_BLOCK_SIZE (OSAL_TEST_NUM_EXT_HWIPS * OSAL_SAFERTOS_HWIP_C7X_SIZE_BYTES)
  #else
    #define HWIP_BLOCK_SIZE (OSAL_TEST_NUM_EXT_HWIPS * OSAL_NONOS_HWIP_SIZE_BYTES)
  #endif
  uint8_t semPMemBlock[SEMP_BLOCK_SIZE]
  __attribute__ ((aligned(64)));
  uint8_t hwiPMemBlock[HWIP_BLOCK_SIZE]
  __attribute__ ((aligned(64)));
#endif


static void myIsr(void)
{
}

bool OSAL_ExtBlock_test(void)
{
    SemaphoreP_Params semParams;
    HwiP_Params       hwiParams;
    Osal_HwAttrs      hwAttrs;
    int32_t           osal_ret;
    uint32_t          ctrlBitMap = ( OSAL_HWATTR_SET_SEMP_EXT_BASE |
                                     OSAL_HWATTR_SET_HWIP_EXT_BASE);
    SemaphoreP_Handle semHandle;
    HwiP_Handle       hwiHandle;

    /* Set the timer base for the osal delay */
    osal_ret = Osal_getHwAttrs(&hwAttrs);
    if (osal_ret != osal_OK)
    {
        return (false);
    }

    /* This API should return osal_OK for AM3/AM4 and return unsupported for other Socs */
    hwAttrs.extSemaphorePBlock.base = (uintptr_t) &semPMemBlock[0];
    hwAttrs.extSemaphorePBlock.size = SEMP_BLOCK_SIZE;
    hwAttrs.extHwiPBlock.size       = HWIP_BLOCK_SIZE;
    hwAttrs.extHwiPBlock.base       = (uintptr_t) &hwiPMemBlock[0];
    osal_ret = Osal_setHwAttrs(ctrlBitMap, &hwAttrs);
    if (osal_ret != osal_OK)
    {
        return (false);
    }

    /* Default parameter initialization */
    SemaphoreP_Params_init(&semParams);

    /* create the semaphore block */
    semHandle = SemaphoreP_create(0, &semParams);
    if (semHandle == (SemaphoreP_Handle) NULL_PTR)
    {
        return (false);
    }

    /* Verify the block created is in the extended memory block range */
    if (semHandle != (SemaphoreP_Handle) &semPMemBlock[0])
    {
        return(false);
    }

    /* delete the semaphore block */
    osal_ret = SemaphoreP_delete(semHandle);
    if (osal_ret != (SemaphoreP_Status) SemaphoreP_OK)
    {
        return (false);
    }


    /* Default parameter initialization */
    HwiP_Params_init(&hwiParams);

    /* create the hwi block */
    hwiHandle = HwiP_create(8U, (HwiP_Fxn)myIsr, &hwiParams);
    if (hwiHandle == (HwiP_Handle) NULL_PTR)
    {
        return (false);
    }

    /* Verify the block created is in the extended memory block range */
    if (hwiHandle != (HwiP_Handle) &hwiPMemBlock[0])
    {
        return(false);
    }

    /* delete the hwi block */
    osal_ret = HwiP_delete(hwiHandle);
    if (osal_ret != (HwiP_Status) HwiP_OK)
    {
        return (false);
    }

    /* Clear the extended block base for next tests */
    /* This API should return osal_OK for AM3/AM4 and return unsupported for other Socs */
    hwAttrs.extSemaphorePBlock.base = (uintptr_t) NULL_PTR;
    hwAttrs.extSemaphorePBlock.size = 0;
    hwAttrs.extHwiPBlock.size       = 0;
    hwAttrs.extHwiPBlock.base       = (uintptr_t) NULL_PTR;
    osal_ret = Osal_setHwAttrs(ctrlBitMap, &hwAttrs);
    if (osal_ret != osal_OK)
    {
        return (false);
    }
    return(true);

}


/*
 *  ======== Semaphore test function ========
 */
bool OSAL_semaphore_test()
{
    SemaphoreP_Params semParams;
    SemaphoreP_Handle handle1, handle2;

    SemaphoreP_Params_init(&semParams);

    /* Test 1: counting mode, no timeout */
    semParams.mode = SemaphoreP_Mode_COUNTING;
    handle1 = SemaphoreP_create(0, &semParams);
    if (handle1 == NULL_PTR)
    {
        return false;
    }
    if (SemaphoreP_post(handle1) != SemaphoreP_OK)
    {
        return false;
    }
    if (SemaphoreP_pend(handle1, 0) != SemaphoreP_OK)
    {
        return false;
    }

    /* Test 2: binary mode, with timeout */
    semParams.mode = SemaphoreP_Mode_BINARY;
    handle2 = SemaphoreP_create(1, &semParams);
    if (handle2 == NULL_PTR)
    {
        return false;
    }
    if (SemaphoreP_pend(handle2, 0) != SemaphoreP_OK)
    {
        return false;
    }
    if (SemaphoreP_pend(handle2, 10) != SemaphoreP_TIMEOUT)
    {
        return false;
    }

    /* SemaphoreP_delete is not available in Safertos */
    if (SemaphoreP_delete(handle1) != SemaphoreP_OK)
    {
        return false;
    }

    if (SemaphoreP_delete(handle2) != SemaphoreP_OK)
    {
        return false;
    }

    return true;
}

#if !defined(BARE_METAL)

/*
 *  ======== Queue test function ========
 */
typedef struct Test_Queue_Buf_s
{
    QueueP_Elem lnk;
    uint32_t    index;
    uint8_t*    pkt;

} Test_Queue_Buf;

bool OSAL_queue_test()
{
    QueueP_Params   params;
    QueueP_Handle   handle;
    Test_Queue_Buf buf[10], *pBuf;
    QueueP_Status   status;
    uint32_t i;

    QueueP_Params_init(&params);

    handle = QueueP_create(&params);

    if (handle == NULL_PTR)
    {
        OSAL_log("Failed to create queue \n");
        return false;
    }

    for (i = 0; i < 10; i++)
    {
        buf[i].index = i;
        buf[i].pkt = NULL;
    }

    /* Test 1: queue push/pop test */
    for (i = 0; i < 10; i++)
    {
        status = QueueP_put(handle, (QueueP_Elem *)&buf[i]);
        if(QueueP_OK != status)
        {
            OSAL_log("Failed to push to queue \n");
            return false;
        }
    }

    for (i = 0; i < 10; i++)
    {
        pBuf = (Test_Queue_Buf *)QueueP_get(handle);

        if (pBuf == NULL_PTR)
        {
            OSAL_log("Failed to pop queue element %d \n", i);
            return false;
        }

        if (pBuf->index != i)
        {
            OSAL_log("Pop element %d, but expect %d \n", pBuf->index, i);
            return false;

        }
    }

    /* Test 2: queue empty test */
    /* After poping all the pushed elements, queue should be empty */ 
    if (QueueP_EMPTY != QueueP_isEmpty(handle))
    {
        OSAL_log("Empty queue check failed\n");
        return false;
    }

    /* When called with an empty queue, QueueP_get should return a pointer to the queue itself */
    pBuf = QueueP_get(handle);

    if (pBuf != NULL)
    {
        OSAL_log("Queue is still not empty with element %p, handle %p \n", pBuf, handle);
        return false;
    }

    status = QueueP_delete(handle);
    if(QueueP_OK != status)
    {
        OSAL_log("Failed to delete queue \n");
        return false;
    }

    return true;
}

/*
 *  ======== Event test function ========
 */
bool OSAL_event_test()
{
    EventP_Params   params;
    EventP_Handle   handle;
    EventP_Status   status;
    uint32_t        eventMask;
    uint32_t        retEventMask;

    EventP_Params_init(&params);

    handle = EventP_create(&params);

    if (handle == NULL_PTR)
    {
        OSAL_log("Failed to create event \n");
        return false;
    }

    /* Test 1: event post test */
    eventMask = EventP_ID_00 + EventP_ID_02 + EventP_ID_04 + EventP_ID_05;
    status = EventP_post(handle, eventMask);
    if(EventP_OK != status)
    {
        OSAL_log("Failed to post an event \n");
        return false;
    }

    /* Test 2: event wait with AND logic test */
    eventMask = EventP_ID_00 + EventP_ID_02;
    retEventMask = EventP_wait(handle, eventMask, EventP_WaitMode_ALL, EventP_WAIT_FOREVER);
    if((retEventMask & eventMask) != eventMask)
    {
        OSAL_log("EventP_wait returned %d, but expect %d \n", retEventMask, eventMask);
        return false;
    }

    /* Test 3: EventP_wait Timeout test */
    /* Event 0 should be cleared since its already consumed  */
    eventMask = EventP_ID_00 + EventP_ID_04;
    retEventMask = EventP_wait(handle, eventMask, EventP_WaitMode_ALL, EventP_NO_WAIT);
    if((retEventMask & eventMask) == eventMask)
    {
        OSAL_log("EventP_wait Timeout test failed \n");
        OSAL_log("EventP_wait returned %d, but expect %d \n", retEventMask, 0);
        return false;
    }
    
    /* Test 4: event wait with OR logic test */
    eventMask = EventP_ID_00 + EventP_ID_04;
    retEventMask = EventP_wait(handle, eventMask, EventP_WaitMode_ANY, EventP_WAIT_FOREVER);
    if((retEventMask & eventMask) != EventP_ID_04)
    {
        OSAL_log("EventP_wait returned %d, but expect %d \n", retEventMask, EventP_ID_04);
        return false;
    }

    /* Test 5: event get posted events test */
    retEventMask = EventP_getPostedEvents(handle);
    if((retEventMask & EventP_ID_05) != EventP_ID_05)
    {
        OSAL_log("EventP_getPostedEvents returned %d, but expect %d \n", retEventMask, EventP_ID_05);
        return false;
    }

    status = EventP_delete(&handle);
    if(EventP_OK != status)
    {
        OSAL_log("Failed to delete event \n");
        return false;
    }

    return true;
}

/*
 *  ======== Mutex test function ========
 */
#define OSAL_MUTEX_TEST_ITERATIONS  (10U)
#define OSAL_MUTEX_TEST_TASK_PRIO   (1U)
#if defined(SOC_TPR12) || defined (SOC_AWR294X)
#define OSAL_MUTEX_TEST_NUM_TASKS   (2U)
#else
#define OSAL_MUTEX_TEST_NUM_TASKS   (5U)
#endif

#if defined(SAFERTOS)
static uint8_t  gAppTskStackMutexTask[OSAL_MUTEX_TEST_NUM_TASKS][APP_TSK_STACK_MAIN] __attribute__((aligned(APP_TSK_STACK_MAIN)));
#else
static uint8_t  gAppTskStackMutexTask[OSAL_MUTEX_TEST_NUM_TASKS][APP_TSK_STACK_MAIN] __attribute__((aligned(32)));
#endif

uint32_t gCnt = 0U;
bool gMutexTestStatus = TRUE;

void mutexTestFxn(MutexP_Handle mutexHandle, SemaphoreP_Handle hDoneSem)
{
    uint32_t        idx;
    uint32_t        temp;
    MutexP_Status   status = MutexP_OK;

    for(idx = 0U; idx < OSAL_MUTEX_TEST_ITERATIONS; idx++)
    {
        status = MutexP_lock(mutexHandle, MutexP_WAIT_FOREVER);
        if(MutexP_OK != status)
        {
            OSAL_log("Failed to lock mutex \n");
            gMutexTestStatus = FALSE;
        }

        temp = ++gCnt;
        {
            /* Force a task switch to verify mutex lock effect */
            TaskP_yield();
        }
        gCnt = temp;

        status = MutexP_unlock(mutexHandle);
        if(MutexP_OK != status)
        {
            OSAL_log("Failed to unlock mutex \n");
            gMutexTestStatus = FALSE;
        }
    }
    
    SemaphoreP_post(hDoneSem);
}

bool OSAL_mutex_test()
{
    TaskP_Params        taskParams;
    TaskP_Handle        hMutexTestTask[OSAL_MUTEX_TEST_ITERATIONS];
    SemaphoreP_Params   semPrms;
    SemaphoreP_Handle   hDoneSem;
    MutexP_Status       status = MutexP_OK;
    uint32_t            i; 
    
    MutexP_Object mutexObj;
    MutexP_Handle mutexHandle;

    mutexHandle = MutexP_create(&mutexObj);
    if (mutexHandle == NULL_PTR)
    {
        OSAL_log("Failed to create mutex \n");
        return false;
    }

    /* Create semaphore to signal completion of mutex tasks */
    SemaphoreP_Params_init(&semPrms);
    semPrms.mode = SemaphoreP_Mode_COUNTING;
    hDoneSem = SemaphoreP_create(0U, &semPrms);

    /* Create tasks */
    for(i = 0U; i < OSAL_MUTEX_TEST_NUM_TASKS; i++)
    {
        TaskP_Params_init(&taskParams);
        memset( &gAppTskStackMutexTask[i], 0xFF, sizeof( gAppTskStackMutexTask[i] ) );
        taskParams.priority     = OSAL_MUTEX_TEST_TASK_PRIO; 
        taskParams.stack        = &gAppTskStackMutexTask[i];
        taskParams.stacksize    = APP_TSK_STACK_MAIN;
        taskParams.arg0         = mutexHandle;
        taskParams.arg1         = hDoneSem;
        hMutexTestTask[i] = TaskP_create(&mutexTestFxn, &taskParams);
    }

    /* Wait for tasks completion */
    for(i = 0U; i < OSAL_MUTEX_TEST_NUM_TASKS; i++)
    {
        SemaphoreP_pend(hDoneSem, SemaphoreP_WAIT_FOREVER);
    }

    if(gCnt != (OSAL_MUTEX_TEST_NUM_TASKS * OSAL_MUTEX_TEST_ITERATIONS))
    {
        OSAL_log("Mutex Lock Test Failed.\n Expected Count = %d. Actual Count = %d. \n",
                    (uint32_t)(OSAL_MUTEX_TEST_NUM_TASKS * OSAL_MUTEX_TEST_ITERATIONS),
                    (uint32_t)gCnt);
        gMutexTestStatus = FALSE;
    }
    
    Osal_delay(OSAL_DELAY_TIME);
    /* Delete tasks */
    for(i = 0U; i < OSAL_MUTEX_TEST_NUM_TASKS; i++)
    {
        TaskP_delete(&hMutexTestTask[i]);
    }

    /* Delete Semaphore */
    SemaphoreP_delete(hDoneSem);
    hDoneSem = NULL;

    status = MutexP_delete(mutexHandle);
    if(MutexP_OK != status)
    {
        OSAL_log("Failed to delete mutex \n");
        gMutexTestStatus = FALSE;
    }

    return gMutexTestStatus;
}

#endif /* #if !defined(BARE_METAL) && !defined (SAFERTOS) */

#if defined(FREERTOS)
/*
 *  ======== Load test function ========
 */
#define OSAL_LOAD_TEST_TASK_PRIO      (1U)
#define OSAL_LOAD_TEST_NUM_TASKS      (3U)

static uint8_t  gAppTskStackLoadTask[OSAL_LOAD_TEST_NUM_TASKS+1][APP_TSK_STACK_MAIN] __attribute__((aligned(32)));

TaskP_Handle hLoadTestTask[OSAL_LOAD_TEST_NUM_TASKS+1];

static bool loadPrint()
{
    uint32_t            i; 
    LoadP_Status        status = LoadP_OK;
    LoadP_Stats         loadStatsTask[OSAL_LOAD_TEST_NUM_TASKS+1];
    uint32_t            cpuLoad;

    /* Query Load stats for each task and print % load */  
    for(i = 0U; i <= OSAL_LOAD_TEST_NUM_TASKS; i++)
    {
        status += LoadP_getTaskLoad(hLoadTestTask[i], &loadStatsTask[i]);

        if(loadStatsTask[i].percentLoad > 0U)
        {
            OSAL_log("    %s - Load = %d%% \n", loadStatsTask[i].name, loadStatsTask[i].percentLoad);
        }
        else
        {   
            /* Load less than 1%, Try to get fractional part */
            OSAL_log("    %s - Load = 0.000%d%% \n", loadStatsTask[i].name, loadStatsTask[i].threadTime/(loadStatsTask[i].totalTime/(100*1000)));
        }
    }
    if(LoadP_OK != status)
    {
        OSAL_log("Failed to get tasks load measurement \n");
        return false;
    }

    /* Query CPU Load */
    cpuLoad = LoadP_getCPULoad();
    OSAL_log("\n    CPU Load = %d%% \n\n", cpuLoad);

    return true;
}

static void loadPrintFxn(SemaphoreP_Handle hDoneSem, void *arg1)
{
    uint32_t    printIntervalInMs = 500;

    while(1)
    {
        TaskP_sleepInMsecs(printIntervalInMs);
        
        /* Check for signal to exit */
        if(SemaphoreP_OK == SemaphoreP_pend(hDoneSem, SemaphoreP_NO_WAIT))
        {
            break;
        }
        
        OSAL_log("** Intermediate Load Stats ** \n");
        loadPrint();
    }
}

void loadTestFxn(SemaphoreP_Handle hSignalSem, void *arg1)
{
    /* Wait for signal to start */
    SemaphoreP_pend(hSignalSem, SemaphoreP_WAIT_FOREVER);

    while(1)
    {
        /* Check for signal to exit */
        if(SemaphoreP_OK == SemaphoreP_pend(hSignalSem, SemaphoreP_NO_WAIT))
        {
            break;
        }
    };

}

bool OSAL_load_test()
{
    TaskP_Params        taskParams;
    SemaphoreP_Params   semPrms;
    SemaphoreP_Handle   hDoneSem;
    SemaphoreP_Handle   hTaskSignalSem[OSAL_LOAD_TEST_NUM_TASKS];
    TaskP_Handle        hPrintTask;
    uint32_t            i; 
    char *taskNameStr[] = { "Task A", "Task B", "Task C"};
    bool status         = true;

    /* Create semaphore to signal completion of load tasks */
    SemaphoreP_Params_init(&semPrms);
    semPrms.mode = SemaphoreP_Mode_COUNTING;
    hDoneSem = SemaphoreP_create(0U, &semPrms);

    /* Reset Load measurement */
    LoadP_reset();

    /* Create tasks and signal semaphores*/
    for(i = 0U; i < OSAL_LOAD_TEST_NUM_TASKS; i++)
    {
        SemaphoreP_Params_init(&semPrms);
        semPrms.mode = SemaphoreP_Mode_COUNTING;
        hTaskSignalSem[i] = SemaphoreP_create(0U, &semPrms);
        
        TaskP_Params_init(&taskParams);
        taskParams.priority     = OSAL_LOAD_TEST_TASK_PRIO;   
        taskParams.name         = (const char *)taskNameStr[i];
        taskParams.stack        = &gAppTskStackLoadTask[i];
        taskParams.stacksize    = APP_TSK_STACK_MAIN;
        taskParams.arg0         = hTaskSignalSem[i];
        hLoadTestTask[i] = TaskP_create(&loadTestFxn, &taskParams);
    }
    hLoadTestTask[OSAL_LOAD_TEST_NUM_TASKS] = TaskP_self();

    /* Create print task, to print the load in regular intervals */
    TaskP_Params_init(&taskParams);
    taskParams.priority     = OSAL_LOAD_TEST_TASK_PRIO + 2U;   
    taskParams.stack        = &gAppTskStackLoadTask[i];
    taskParams.stacksize    = APP_TSK_STACK_MAIN;
    taskParams.arg0         = hDoneSem;
    hPrintTask = TaskP_create(&loadPrintFxn, &taskParams); 

    /* ==============================================================================
     * Stay in while loop of Task A for ~2s, (Expected Task Load:......2s/7s = ~28%)
     *           followed by Task B for ~1s, (Expected Task Load:......1s/7s = ~14%)
     *           followed by Task C for ~0.5s(Expected Task Load:.....0.5/7s =  ~7%)
     *  And finally go to idle mode for ~3.5s(Expected CPU Load :(7-3.5)s/7s = ~50%) 
     * ------------------------------------------------------------------------------
     * Total Time: ~(2s+1s+0.5+3.5s) =  ~7s |
     * =============================================================================*/
    {
        uint32_t waitTimeInMsec[OSAL_LOAD_TEST_NUM_TASKS] = {2000, 1000, 500};
        uint32_t idleTimeInMsec = 3500;
        
        for(i = 0U; i < OSAL_LOAD_TEST_NUM_TASKS; i++)
        {
            /* Signal start */
            SemaphoreP_post(hTaskSignalSem[i]);
            /* Spin in Task 'i' */
            TaskP_sleepInMsecs(waitTimeInMsec[i]);
            /* Signal exit */
            SemaphoreP_post(hTaskSignalSem[i]);
        }

        TaskP_sleepInMsecs(idleTimeInMsec);
    }

    SemaphoreP_post(hDoneSem);

    OSAL_log("*************** FINAL LOAD STATS ************* \n");
    status = loadPrint();

    /* Delete tasks and signal semaphores */
    for(i = 0U; i < OSAL_LOAD_TEST_NUM_TASKS; i++)
    {
        TaskP_delete(&hLoadTestTask[i]);
        SemaphoreP_delete(hTaskSignalSem[i]);
    }
    /* Delete Print Task */
    TaskP_delete(&hPrintTask);

    /* Delete Semaphore */
    SemaphoreP_delete(hDoneSem);
    hDoneSem = NULL;

    return status;
}

#endif /* #if defined(FREERTOS) */


#if defined(USE_BIOS)

/*
 *  ======== SWI test function ========
 */
volatile bool gFlagSwi;
void mySwiFxn(uintptr_t arg0, uintptr_t arg1)
{
    gFlagSwi = 1U;
}

bool OSAL_swi_test()
{
    SwiP_Handle handle;
    SwiP_Params swiParams;
    SwiP_Status status;
    bool        retVal = true;

    SwiP_Params_init(&swiParams);
    handle = SwiP_create((SwiP_Fxn)&mySwiFxn, &swiParams);
    if (handle == NULL_PTR)
    {
        OSAL_log("Failed to create software interrupt \n");
        retVal = false;
    }

    if (retVal == true)
    {
        gFlagSwi = 0;
        status = SwiP_post(handle);
        if (status != SwiP_OK)
        {
            OSAL_log("Failed to post software interrupt \n");
            retVal = false;
        }
    }

    if (retVal == true)
    {
        gTestlocalTimeout = 0x300000U;
        while (gTestlocalTimeout != 0U)
        {
            gTestlocalTimeout--;
            if (gFlagSwi)
            {
                gFlagSwi = 0;
                retVal = true;
                break;
            }
        }

        if (gTestlocalTimeout == 0)
        {
            OSAL_log("Failed to get software interrupt \n");
            retVal = false;
        }
    }

    if (retVal == true)
    {
        status = SwiP_delete(&handle);
        if (status != SwiP_OK)
        {
            OSAL_log("Failed to delete software interrupt \n");
            retVal = false;
        }
    }

    return (retVal);
}
#endif

#if ENABLE_DEBUG_LOG_TEST
bool OSAL_log_test()
{
    bool        retVal = true;

    OSAL_log("BegugP Log test starts!\n");

    DebugP_log0 ("debugP_log0 run successfully\n");
    DebugP_log1 ("debugP_log1 run with %u argument uccessfully\n", 1U);
    DebugP_log2 ("debugP_log2 run with %u arguments (2, %d) successfully\n", 2U, 2);
    DebugP_log3 ("debugP_log3 run with %u arguments (3, %d, %d) successfully\n", 3U, 3, 3);
    DebugP_log4 ("debugP_log4 run with %u arguments (4, %d, %d, 0x%x) successfully\n", 4U, 4, 4, 64);

    return (retVal);
}
#endif

#ifndef BARE_METAL
#if defined(USE_BIOS)
#include <ti/sysbios/knl/Clock.h>
#endif
#ifndef SIM_BUILD
#define   OSAL_TASKP_TEST_ITERATION    (10U)
#define   OSAL_TASKP_TEST_1MS          (1000U)
#define   OSAL_TASKP_TEST_TICKS        (1000U)
#else
#define   OSAL_TASKP_TEST_ITERATION    (2U)
#define   OSAL_TASKP_TEST_1MS          (10U)
#define   OSAL_TASKP_TEST_TICKS        (10U)
#endif

uint64_t OSAL_get_ticks()
{
    uint64_t ticks;
#if  defined(FREERTOS)
    ticks = (uint64_t)portGET_RUN_TIME_COUNTER_VALUE();
#elif defined(SAFERTOS)
    ticks = (uint64_t)TimerP_getTimeInUsecs();
#else
    ticks = (uint64_t)Clock_getTicks();
#endif
    return ticks;
}

bool OSAL_task_sleep_test(void)
{
    int32_t i;
    uint64_t    start_time_nticks, end_time_nticks;
    uint32_t    diff_nticks, diff_tout;

    /* nTicks in OSAL is 1000 ticks per milli seconds
     * hence the task sleep = 1000 milliseconds should
     * provide the same sleep time as in TaskP_sleep(nTicks = 1000)
     */
    start_time_nticks = OSAL_get_ticks();

    for (i=0; i < OSAL_TASKP_TEST_ITERATION; i++)
    {
       TaskP_sleep(OSAL_TASKP_TEST_TICKS);
    }
    end_time_nticks = OSAL_get_ticks();
    diff_nticks     = (uint32_t)(end_time_nticks - start_time_nticks);

    start_time_nticks = OSAL_get_ticks();

    for (i=0; i < OSAL_TASKP_TEST_ITERATION; i++)
    {
       TaskP_sleepInMsecs(OSAL_TASKP_TEST_1MS);
    }
    end_time_nticks = OSAL_get_ticks();
    diff_tout       = (uint32_t)(end_time_nticks - start_time_nticks);

    OSAL_log(" \n \
               diff_nticks = %d \n \
               diff_tout   = %d \n ", diff_nticks, diff_tout);
#if defined(USE_BIOS)
    OSAL_log(" Clock_tickPeriod = %d \n \
               Clock_tickSource = %d ", Clock_tickPeriod, Clock_tickSource);
#endif

    return (true);

}
#endif

#if  defined(FREERTOS)
bool OSAL_mempry_test()
{
    void *memPtr1, *memPtr2, *memPtr[5];
    uint32_t align;
    bool retVal = true;
    uint32_t i;

    /* Test1: Allocate 16 bytes aligned memory. */
    align = 16;
    memPtr1 = MemoryP_ctrlAlloc(100, align);
    if (memPtr1 != NULL)
    {
        /* Check if teh allocated mempry is 16 bytes aligned. */
        if (((uintptr_t)memPtr1 & (align - 1)) != 0)
        {
            retVal = false;
        }
    }
    else
    {
        retVal = false;
    }

    /* Test2: Allocate 64 bytes aligned memory. */
    align = 64;
    memPtr2 = MemoryP_ctrlAlloc(200, align);
    if (memPtr2 != NULL)
    {
        /* Check if teh allocated mempry is 16 bytes aligned. */
        if (((uintptr_t)memPtr2 & (align - 1)) != 0)
        {
            retVal = false;
        }
    }
    else
    {
        retVal = false;
    }

    if (memPtr1 != NULL)
    {
        MemoryP_ctrlFree(memPtr1, 100);
    }
    if (memPtr2 != NULL)
    {
        MemoryP_ctrlFree(memPtr2, 200);
    }

    /* Test2: check memory leak
     * multiple iteration of alloc and free to give same mem pointer.
     */
    for (i=0; i<5; i++)
    {
        align = 64;
        memPtr[i] = MemoryP_ctrlAlloc(200, align);
        MemoryP_ctrlFree(memPtr[i], 200);
    }
    for (i=1; i<5; i++)
    {
        if (memPtr[i] != memPtr[i-1])
        {
            retVal = false;
            break;
        }
    }
    return retVal;
}
#endif

bool OSAL_TimerP_ANY_test( void )
{
	uint32_t looper, numRestricted = 0;
	uint32_t result=true, availMask=TIMERP_AVAILABLE_MASK, availableTimers = 0;
	TimerP_Params params;
	TimerP_Handle handle[TimerP_numTimerDevices];
	TimerP_Status timerStatus = TimerP_FAILURE;

	/* Figure out the number of timers available to use with the TIMERP_AVAILABLE_MASK. */
	while(availMask > 0U)
	{
		if(1U == (availMask & 1U))
		{
			availableTimers++;
		}
		availMask = availMask >> 1U;
	}

/* Figure out the number of restricted timers per core. */
#if defined (BUILD_MCU)
	uint32_t coreId;
    extern Osal_timerReserved Osal_mcuRestrictedTimerID[TimerP_ANY_MAX_RESTRICTIONS], Osal_mainMcuRestrictedTimerID[TimerP_ANY_MAX_RESTRICTIONS];
	coreId = Osal_getCoreId();
	if ((OSAL_MCU1_0 == coreId) || (OSAL_MCU1_1 == coreId))
	{
		for (looper = 0; looper < TimerP_ANY_MAX_RESTRICTIONS; looper++)
        {
            if(OSAL_INVALID_CORE_ID != Osal_mcuRestrictedTimerID[looper].coreId)
            {
                numRestricted++;
            }
        }
	}
	else
	{
		for (looper = 0; looper < TimerP_ANY_MAX_RESTRICTIONS; looper++)
        {
            if(OSAL_INVALID_CORE_ID != Osal_mainMcuRestrictedTimerID[looper].coreId)
            {
                numRestricted++;
            }
        }
	}
#elif defined (BUILD_C7X)
    extern Osal_timerReserved Osal_c7xRestrictedTimerID[TimerP_ANY_MAX_RESTRICTIONS];
    for (looper = 0; looper < TimerP_ANY_MAX_RESTRICTIONS; looper++)
    {
        if(OSAL_INVALID_CORE_ID != Osal_c7xRestrictedTimerID[looper].coreId)
        {
            numRestricted++;
        }
    }
#elif defined (BUILD_C66X)
	extern Osal_timerReserved Osal_c66RestrictedTimerID[TimerP_ANY_MAX_RESTRICTIONS];
    for (looper = 0; looper < TimerP_ANY_MAX_RESTRICTIONS; looper++)
    {
        if(OSAL_INVALID_CORE_ID != Osal_c66RestrictedTimerID[looper].coreId)
        {
            numRestricted++;
        }
    }
#elif defined (BUILD_MPU)
	extern Osal_timerReserved Osal_mpuRestrictedTimerID[TimerP_ANY_MAX_RESTRICTIONS];
    for (looper = 0; looper < TimerP_ANY_MAX_RESTRICTIONS; looper++)
    {
        if(OSAL_INVALID_CORE_ID != Osal_mpuRestrictedTimerID[looper].coreId)
        {
            numRestricted++;
        }
    }
#endif
	OSAL_log(" Timers available for TimerP_ANY: %d, Restricted Timers: %d\n", availableTimers, numRestricted);
	/* Create as many timers, as are expected to get created succesfully. */
	for(looper = 0; looper < availableTimers - numRestricted; looper++)
	{
		TimerP_Params_init(&params);
		params.startMode = TimerP_StartMode_USER;
		handle[looper] = TimerP_create(TimerP_ANY, NULL, &params);
		if (NULL == handle[looper])
		{
		    result = false;
		    break;
		}
	}
	OSAL_log(" Created %d Timers...\n", looper);

	/* This TimerP_create should fail, as not more timers are available to get created. */
	if ( true == result )
	{
		handle[looper] = TimerP_create(TimerP_ANY, NULL, NULL);
		if (NULL != handle[looper])
		{
		    result = false;
		}
	}
	/* Delete all create timers. */
	if ( true == result )
	{
		for(looper = 0; looper < availableTimers - numRestricted; looper++)
		{
		    timerStatus = TimerP_delete(handle[looper]);
		    if (TimerP_FAILURE == timerStatus)
		    {
                OSAL_log("[ERROR]: TimerP_create was supposed to fail, but it went through!!!");
		        result = false;
		        break;
		    }
		}
		OSAL_log(" Deleted %d Timers...\n", looper);
	}
    return result;
}

/*
 *  ======== main test function ========
 */
#if defined(BARE_METAL)
void osal_test()
#else
void osal_test(void *arg0, void *arg1)
#endif
{
    bool testFail = false;
    Osal_StaticMemStatus pMemStats;

    Board_initOSAL();

    if (true == OSAL_startup_hook_test())
    {
        OSAL_log("\n StartuphookP test has passed. \n");
    }
    else
    {
        OSAL_log("\n StartuphookP test has failed. \n");
    }

#if defined (BUILD_C7X)
    C7x_ConfigureTimerOutput();
#endif

#ifdef BUILD_M4F    
    OSAL_log("\n M4 test \n");
#endif

#if defined(BUILD_C66X)
/* To set C66 timer interrupts on J7ES */
    C66xTimerInterruptInit();
#endif

    OSAL_log(" OSAL Test Starting...\n Takes about 30 seconds ...\n"); 
    if(OSAL_TimerP_ANY_test() == true)
    {
        OSAL_log("\n TimerP_ANY test has passed. \n");
    }
    else
    {
        OSAL_log("\n TimerP_ANY test has failed. \n");
        testFail = true;
    }
#if defined(BARE_METAL)
    /* No TASKP test for BAREmetal */
#else
    /* TASK Sleep APIs test for RTOS */
    if (OSAL_task_sleep_test() == true)
    {
        OSAL_log("\n TaskP tests have passed. \n");
    }
    else
    {
        OSAL_log("\n TaskP tests have failed. \n");
    }
#endif

    if(OSAL_hwi_test() == true)
    {
        OSAL_log("\n HWI tests have passed. \n");
    }
    else
    {
        OSAL_log("\n HWI tests have failed. \n");
        testFail = true;
    }

#ifdef ENABLE_TIMER_TEST
    if(OSAL_timer_test() == true)
    {
        OSAL_log("\n Timer tests have passed. \n");
    }
    else
    {
        OSAL_log("\n Timer tests have failed. \n");
        testFail = true;
    }
#endif

/* The test does not support below SoCs */
#if !defined(SOC_TDA2XX) && !defined(SOC_TDA2PX) && !defined(SOC_TDA2EX) && !defined(SOC_TDA3XX)
  OSAL_log("\n Running Osal_Delay test:");

  if(Osal_delay_test() == true)
  {
      OSAL_log("\n Osal_Delay  tests have passed. \n");
  }
  else
  {
      OSAL_log("\n Osal_Delay  tests have failed. \n");
      testFail = true;
  }
#endif

#ifdef ENABLE_GET_TIME_TEST
    OSAL_log("\n Running Osal_getTime test:");

    if(Osal_getTime_test() == true)
    {
        OSAL_log("\n Osal_getTime  tests have passed. \n");
    }
    else
    {
        OSAL_log("\n Osal_getTime  tests have failed. \n");
        testFail = true;
    }
#endif

    if(OSAL_clock_test() == true)
    {
        OSAL_log("\n Clock tests have passed. \n");
    }
    else
    {
        OSAL_log("\n Clock tests have failed. \n");
        testFail = true;
    }

    if(OSAL_semaphore_test() == true)
    {
        OSAL_log("\n Semaphore tests have passed. \n");
    }
    else
    {
        OSAL_log("\n Semaphore tests have failed. \n");
        testFail = true;
    }

#ifdef ENABLE_EXT_BLOCK_TEST

    if(OSAL_ExtBlock_test() == true)
    {
        OSAL_log("\n Extended Memory Block tests for HwiP/SwiP have passed. \n");
    }
    else
    {
        OSAL_log("\n Extended Memory Block tests for HwiP/SwiP have failed. \n");
        testFail = true;
    }
#endif

    #ifdef MANUAL_CACHE_TEST
    /* This test is valid only for MANUAL testing */
    OSAL_cache_test();
    #endif

    /* Now print the static memory statistics */
    if(Osal_getStaticMemStatus(&pMemStats) == osal_OK)
    {
       OSAL_log("\n Semaphore Statistics:     \
                 \n  PeakSemObjs    = %u,     \
                 \n  numMaxSemObjs  = %u,     \
                 \n  numFreeSemObjs = %u \n", \
                 pMemStats.peakSemObjs,       \
                 pMemStats.numMaxSemObjs,     \
                 pMemStats.numFreeSemObjs);

       OSAL_log("\n Hwi Statistics:           \
                 \n  PeakHwiObjs    = %u,     \
                 \n  numMaxHwiObjs  = %u,     \
                 \n  numFreeHwiObjs = %u \n", \
                 pMemStats.peakHwiObjs,       \
                 pMemStats.numMaxHwiObjs,     \
                 pMemStats.numFreeHwiObjs);

       OSAL_log("\n Timer Statistics:           \
                 \n  PeakTimerObjs    = %u,     \
                 \n  numMaxTimerObjs  = %u,     \
                 \n  numFreeTimerObjs = %u \n", \
                 pMemStats.peakTimerObjs,       \
                 pMemStats.numMaxTimerObjs,     \
                 pMemStats.numFreeTimerObjs);

    }
    else
    {
      testFail = true;
      OSAL_log("\n Memory Statistics query failed \n");
    }

#if defined(USE_BIOS)
    if(OSAL_swi_test() == true)
    {
        OSAL_log("\n SWI tests have passed. \n");
    }
    else
    {
        OSAL_log("\n SWI tests have failed. \n");
        testFail = true;
    }
#endif

#if !defined(BARE_METAL)
    /* No QueueP,EventP,MutexP tests for Baremetal */
    if(OSAL_queue_test() == true)
    {
        OSAL_log("\n Queue tests have passed. \n");
    }
    else
    {
        OSAL_log("\n Queue tests have failed. \n");
        testFail = true;
    }

    if(OSAL_event_test() == true)
    {
        OSAL_log("\n Event tests have passed. \n");
    }
    else
    {
        OSAL_log("\n Event tests have failed. \n");
        testFail = true;
    }

    if(OSAL_mutex_test() == true)
    {
        OSAL_log("\n Mutex tests have passed. \n");
    }
    else
    {
        OSAL_log("\n Mutex tests have failed. \n");
        testFail = true;
    }
#endif /* #if !defined(BARE_METAL) && !defined (SAFERTOS) */

#if defined(FREERTOS)    
    OSAL_log(" \n OSAL Load Test Starting...\n Takes about 10 seconds ...\n\n"); 
    if(OSAL_load_test() == true)
    {
        OSAL_log("\n Load tests have passed. \n");
    }
    else
    {
        OSAL_log("\n Load tests have failed. \n");
        testFail = true;
    }
#endif /* #if defined(FREERTOS) */

#if ENABLE_DEBUG_LOG_TEST
    if(OSAL_log_test() == true)
    {
        OSAL_log("\n DebugP Log tests have passed. \n");
    }
    else
    {
        OSAL_log("\n DebugP Log tests have failed. \n");
        testFail = true;
    }
#endif

#if defined(FREERTOS)
    if(OSAL_mempry_test() == true)
    {
        OSAL_log("\n MemoryP Log tests have passed. \n");
    }
    else
    {
        OSAL_log("\n MemoryP Log tests have failed. \n");
        testFail = true;
    }
#endif

#if !defined (BARE_METAL)
    if (OSAL_floating_point_test() == true)
    {
        OSAL_log("\n Floating point tests have passed. \n");
    }
    else
    {
        testFail = true;
    }
#endif

    if(testFail == true)
    {
        OSAL_log("\n Some tests have failed. \n");
    }
    else
    {
        OSAL_log("\n All tests have passed. \n");
    }

#ifdef BARE_METAL
    while (1)
    {
    }
#endif

}

#ifdef BUILD_C7X
void sysIdleLoop(void)
{
   __asm(" IDLE");
}

#include <ti/csl/csl_clec.h>

void C7x_ConfigureTimerOutput()
{
    CSL_ClecEventConfig   cfgClec;
    CSL_CLEC_EVTRegs     *clecBaseAddr = (CSL_CLEC_EVTRegs*)OSAL_TEST_CLEC_BASE_ADDRESS;

    uint32_t input         = OSAL_TEST_TIMER_EVENT_NUM; /* Used for Timer Interrupt */
    uint32_t corepackEvent = OSAL_TEST_TIMER_INT_NUM;

    /* Configure CLEC */
    cfgClec.secureClaimEnable = FALSE;
    cfgClec.evtSendEnable     = TRUE;
    cfgClec.rtMap             = CSL_CLEC_RTMAP_CPU_ALL;
    cfgClec.extEvtNum         = 0;
    cfgClec.c7xEvtNum         = corepackEvent;
    CSL_clecClearEvent(clecBaseAddr, input);
    CSL_clecConfigEventLevel(clecBaseAddr, input, 0); /* configure interrupt as pulse */
    CSL_clecConfigEvent(clecBaseAddr, input, &cfgClec);

}

#endif

#if defined(BUILD_C66X)

/* To set C66 timer interrupts on J7ES */
void C66xTimerInterruptInit(void)
{
    /* Map DMTimer interrupt to C6xx events through DMSC RM API. */
#if defined (_TMS320C6X) && !defined(SOC_TPR12) && !defined (SOC_AWR294X)
    /* DMTimers used by OS */ 
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;
#if !defined(SAFERTOS) 
 /*  SAFERTOS already configured IR for Timer Interrupts as a part of OS_init*/
    rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_DST_ID_VALID |
                                      TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.src_id                 = OSAL_TEST_OS_TIMER_TISCI_ID;
    rmIrqReq.src_index              = 0U;
    rmIrqReq.dst_id                 = OSAL_TEST_CORE_TISCI_ID;
    rmIrqReq.dst_host_irq           = OSAL_TEST_OS_TIMER_EVENT_NUM;
    /* Unused params */
    rmIrqReq.global_event           = 0U;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SCICLIENT_SERVICE_WAIT_FOREVER);
#endif
    /* DMTimers used for OSAL Timer Test  */ 
    rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_DST_ID_VALID |
                                      TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.src_id                 = OSAL_TEST_TIMER_TISCI_ID;
    rmIrqReq.src_index              = 0U;
    rmIrqReq.dst_id                 = OSAL_TEST_CORE_TISCI_ID;
    rmIrqReq.dst_host_irq           = OSAL_TEST_TIMER_EVENT_NUM;
    /* Unused params */
    rmIrqReq.global_event           = 0U;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SCICLIENT_SERVICE_WAIT_FOREVER);

#endif
}

void sysIdleLoop(void)
{
   __asm(" IDLE");
}

#endif

/*
 *  ======== main ========
 */
int main(void)
{

#ifdef BARE_METAL
    osal_test();
#else
    /* All other platforms have the task created under RTSC cfg file
     * hence not needed to be created again as below
     * For AM65XX TPR12 and J7 the common RTSC cfg file is used and hence there is
     * no test application specific task is created in teh RTSC cfg file
     */
#if defined (SOC_AM65XX) || defined (SOC_J721E) || defined(SOC_J7200) || defined(SOC_TPR12) || defined (SOC_AWR294X)|| defined(SOC_AM64X) || defined(SOC_J721S2) || defined(SOC_J784S4)
    TaskP_Params taskParams;

#if defined(USE_BIOS)
    Error_Block  eb;
#endif
    OS_init();
    memset( gAppTskStackMain, 0xFF, sizeof( gAppTskStackMain ) );
    TaskP_Params_init(&taskParams);
    taskParams.priority =2;
    taskParams.stack        = gAppTskStackMain;
    taskParams.stacksize    = sizeof (gAppTskStackMain);
#if defined(USE_BIOS)
    taskParams.pErrBlk      = &eb;
#endif
    TaskP_create(&osal_test, &taskParams);
#endif
    OS_start();
#endif
    return (0);
}

#if defined(BUILD_MPU) || defined (BUILD_C7X)
extern void Osal_initMmuDefault(void);
void InitMmu(void)
{
    Osal_initMmuDefault();
}
#endif

