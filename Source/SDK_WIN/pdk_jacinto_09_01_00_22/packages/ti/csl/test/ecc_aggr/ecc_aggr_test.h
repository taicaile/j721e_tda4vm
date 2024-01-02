/* Copyright (c) Texas Instruments Incorporated 2019-2020
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
 *  \file     ecc_aggr_test.h
 *
 *  \brief    This file contains ECC Aggregator test code defines.
 *
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/soc.h>
#include <ti/csl/csl_ecc_aggr.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/csl/hw_types.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/board/board.h>

#if !defined(CSL_ECC_AGGR_TEST_H)
#define CSL_ECC_AGGR_TEST_H

/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/* Define the test interface */
typedef struct cslEccAggrTest_s
{
    int32_t  (*testFunction)(void);   /* The code that runs the test */
    char      *name;                  /* The test name */
    int32_t    testStatus;            /* Test Status */
} cslEccAggrTest_t;

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define CSL_APP_TEST_NOT_RUN        (-(int32_t) (2))
#define CSL_APP_TEST_FAILED         (-(int32_t) (1))
#define CSL_APP_TEST_PASS           ( (int32_t) (0))

#ifdef SOC_AM65XX
#define CSL_TEST_ECC_AGGR_BASE CSL_MCU_ARMSS0_CORE0_ECC_AGGR_BASE
#elif defined(SOC_J721E) || defined(SOC_J7200)
#define CSL_TEST_ECC_AGGR_BASE CSL_MCU_R5FSS0_CORE0_ECC_AGGR_BASE
#elif defined(SOC_AM64X)
#define CSL_TEST_ECC_AGGR_BASE CSL_R5FSS0_CORE0_ECC_AGGR_BASE
#endif

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
static int32_t cslApp_initBoard(void);
static void cslApp_print(const char * str);
static void cslApp_printArg(uint32_t value);

/*===========================================================================*/
/*                         External function declarations                    */
/*===========================================================================*/
extern int32_t cslEccAggr_apiTest(void);
extern int32_t cslEccAggr_negTest(void);

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/

static void cslApp_print(const char * str)
{
    UART_printf(str);
}

static void cslApp_printArg(uint32_t value)
{
   UART_printf("0x%x \n", value);
}
#endif /* CSL_ECC_AGGR_TEST_H */
/* Nothing past this point */
