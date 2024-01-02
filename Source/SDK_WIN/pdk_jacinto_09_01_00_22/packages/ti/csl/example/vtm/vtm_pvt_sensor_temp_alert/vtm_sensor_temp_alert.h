/*
 *  Copyright (c) Texas Instruments Incorporated 2020-2022
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

#ifndef VTM_SENSOR_TEMP_ALERT_H_
#define VTM_SENSOR_TEMP_ALERT_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/soc.h>
#include <ti/csl/hw_types.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/csl/csl_vtm.h>
#include <ti/csl/example/utils/uart_console/inc/uartConfig.h>
#include <ti/osal/osal.h>

#define TEST_THR_AUTO_TEST_FOR_RTOS_NIGHTLY    (1)
#define TEST_MAX_SENSOR_FOR_VTM_ALERT          (5)

#if defined (CSL_VTM_SS_UNIT)
#define CSL_VTM_VAL_EFUSE_0                    (0x31D77D73)
#define CSL_VTM_VAL_EFUSE_1                    (0xF0783C1E)
#define CSL_VTM_VAL_EFUSE_2                    (0x7B5B41E0)
#define CSL_VTM_VAL_EFUSE_3                    (0x001B1B5B)
#endif

#if defined (CSL_VTM_TT_UNIT)
#define CSL_VTM_VAL_EFUSE_0                    (0x32D75C32)
#define CSL_VTM_VAL_EFUSE_1                    (0xE8783C1E)
#define CSL_VTM_VAL_EFUSE_2                    (0xFDDE01E0)
#define CSL_VTM_VAL_EFUSE_3                    (0x001DDDFD)
#endif

#if defined (CSL_VTM_FF_UNIT)
#define CSL_VTM_VAL_EFUSE_0                    (0x31D77CF3)
#define CSL_VTM_VAL_EFUSE_1                    (0xE8743A1D) 
#define CSL_VTM_VAL_EFUSE_2                    (0xFDBE21D0)
#define CSL_VTM_VAL_EFUSE_3                    (0x001DFDFD)
#endif

#define TEST_CSL_VTM_LT_THR0_VAL               (95000)
#define TEST_CSL_VTM_GT_THR1_VAL               (105000)
#define TEST_CSL_VTM_GT_THR2_VAL               (115000)

#include <ti/board/board.h>
#include <ti/osal/osal.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/csl/csl_vtm.h>
#if defined (BUILD_MCU)
#include <ti/csl/csl_intr_router.h>
#endif

#ifdef UNITY_INCLUDE_CONFIG_H
#include <ti/build/unit-test/Unity/src/unity.h>
#include <ti/build/unit-test/config/unity_config.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

#define TEST_CSL_VTM_LT_THR0_SET    (1)
#define TEST_CSL_VTM_GT_THR1_SET    (2)
#define TEST_CSL_VTM_GT_THR2_SET    (3)

/* Threshold interrupts */
#if defined(SOC_J721E) || defined(SOC_J7200) || defined (SOC_J721S2) || defined (SOC_J784S4)
#define TEST_CSL_VTM_LT_THR0_INTR_NUM   (CSLR_MCU_R5FSS0_CORE0_INTR_WKUP_VTM0_THERM_LVL_LT_TH0_INTR_0)
#define TEST_CSL_VTM_GT_THR1_INTR_NUM   (CSLR_MCU_R5FSS0_CORE0_INTR_WKUP_VTM0_THERM_LVL_GT_TH1_INTR_0)
#define TEST_CSL_VTM_GT_THR2_INTR_NUM   (CSLR_MCU_R5FSS0_CORE0_INTR_WKUP_VTM0_THERM_LVL_GT_TH2_INTR_0)
#endif

#if defined(SOC_AM64X)
#define TEST_CSL_VTM_LT_THR0_INTR_NUM   (CSLR_R5FSS0_CORE0_INTR_VTM0_THERM_LVL_LT_TH0_INTR_0)
#define TEST_CSL_VTM_GT_THR1_INTR_NUM   (CSLR_R5FSS0_CORE0_INTR_VTM0_THERM_LVL_GT_TH1_INTR_0)
#define TEST_CSL_VTM_GT_THR2_INTR_NUM   (CSLR_R5FSS0_CORE0_INTR_VTM0_THERM_LVL_GT_TH2_INTR_0)
#endif
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef VTM_SENSOR_TEMP_ALERT_H_ */
