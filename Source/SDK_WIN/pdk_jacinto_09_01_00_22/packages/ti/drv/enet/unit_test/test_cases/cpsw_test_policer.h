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
 *  \file cpsw_test_policer.h
 *
 *  \brief enet_policer test private header file.
 */

#ifndef ENET_TEST_POLICE_H_
#define ENET_TEST_POLICE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define ENETTESTPOLICER_DEFAULT_RXFLOWCFGID                               (0)
#define ENETTESTPOLICER_TYPE1_RXFLOWCFGID                                 (1)
#define ENETTESTPOLICER_TYPE2_RXFLOWCFGID                                 (2)
#define ENETTESTPOLICER_TYPE3_RXFLOWCFGID                                 (3)
#define ENETTESTPOLICER_DEFAULT_TXCHCFGID                               (0)
#define ENETTESTPOLICER_TYPE1_TXCHCFGID                                 (1)
#define ENETTESTPOLICER_TYPE2_TXCHCFGID                                 (2)

#define ENET_TEST_POLICER_PKT_RECVCOUNT                                   (30000U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
int32_t EnetTestPolicer_Run(EnetTestTaskObj *taskObj);

int32_t EnetTestPolicer_processRxPkt(EnetTestTaskObj *taskObj,
                                     uint32_t rxFlowId,
                                     uint32_t pktNum,
                                     EnetDma_Pkt *pktInfo,
                                     bool *testComplete);

void EnetTestPolicer_setOpenPrms(EnetTestTaskObj *taskObj,
                                 Cpsw_Cfg *pCpswCfg,
                                 EnetOsal_Cfg *pOsalPrms,
                                 EnetUtils_Cfg *pUtilsPrms);

void EnetTestPolicer_updatePortLinkCfg(EnetPer_PortLinkCfg *pLinkArgs,
                                       Enet_MacPort portNum);

int32_t EnetTestPolicer_setTxOpenParams(EnetUdma_OpenTxChPrms *pTxChPrms,
                                        EnetTestTaskObj *taskObj,
                                        uint32_t txPSILThreadId,
                                        uint32_t txChCfgIdx);

int32_t EnetTestPolicer_setRxOpenParams(EnetUdma_OpenRxFlowPrms *pRxFlowPrms,
                                        EnetTestTaskObj *taskObj,
                                        uint32_t rxFlowCfgIdx);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ENET_TEST_POLICE_H_ */
