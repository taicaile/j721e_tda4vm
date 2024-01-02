/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 *  \file cpsw_test_sanity.h
 *
 *  \brief cpsw sanity test private header file.
 */

#ifndef ENET_TEST_SANITY_H_
#define ENET_TEST_SANITY_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "../utils/test_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENET_TEST_SANITY_NUM_ITERATIONS              (SANITY_APP_NUM_ITERATION)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
int32_t EnetTestSanity_Run(EnetTestTaskObj *taskObj);

int32_t EnetTestMacSpeed_Run(EnetTestTaskObj *taskObj);

void EnetTestSanity_updatePortLinkCfgAutoNeg(EnetPer_PortLinkCfg *pLinkArgs,
                                             Enet_MacPort portNum);

void EnetTestSanity_updatePortLinkCfg100Mbps(EnetPer_PortLinkCfg *pLinkArgs,
                                             Enet_MacPort portNum);

int32_t EnetTest_xmitTest(EnetTestTaskObj *taskObj,
                           uint32_t txCfgIndex,
                           uint32_t num,
                           const uint8_t *dstAddr,
                           const uint8_t *srcAddr,
                           uint16_t etherType,
                           uint16_t length);

int32_t EnetTest_transmitPkts(EnetTestTaskObj *taskObj,
                               EnetDma_PktQ *pTxSubmitQ,
                               uint32_t txCfgIndex);

void EnetTest_sendCmd(EnetTestTaskObj *taskObj,
                       uint8_t cmd,
                       uint32_t txCfgIndex);

void EnetTest_test_0001(EnetTestTaskObj *taskObj,
                         uint32_t txCfgIndex,
                         uint32_t rxCfgIndex);

void EnetTest_test_0002(EnetTestTaskObj *taskObj,
                         uint32_t txCfgIndex,
                         uint32_t rxCfgIndex);

void EnetTest_test_0100(EnetTestTaskObj *taskObj,
                         uint32_t txCfgIndex,
                         uint32_t rxCfgIndex);

void EnetTest_discardRxPkts(EnetTestTaskObj *taskObj,
                             uint32_t rxCfgIndex);

void EnetTest_freeRxBuffer(EnetTestTaskObj *taskObj,
                            uint32_t rxCfgIndex);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ENET_TEST_SANITY_H_ */
