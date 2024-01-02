/*
 *  Copyright (C) 2023-2023 Texas Instruments Incorporated
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
 *  \file sciclient_fw_notify.c
 *
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <ti/drv/sciclient/examples/common/sciclient_appCommon.h>
#include <ti/drv/sciclient/examples/sciclient_unit_testapp/sciclient_fw_notify.h>

uint32_t gInterruptRecieved;
uint32_t gDmscGlbRegs = DMSC_FW_EXCP_LOG_REG;
uint32_t gFwExcpExtInputRegAddr[FW_MAX_ISR_INPUTS] = 
{
    [0] = CMBN_FW_EXCP_LOG_REG_0,
    [1] = CMBN_FW_EXCP_LOG_REG_1,
    [2] = CMBN_FW_EXCP_LOG_REG_3,
    [3] = CMBN_FW_EXCP_LOG_REG_4,  
    [4] = CMBN_FW_EXCP_LOG_REG_5,
    [5] = CMBN_FW_EXCP_LOG_REG_6,
    [6] = CMBN_FW_EXCP_LOG_REG_7,
    [7] = CMBN_FW_EXCP_LOG_REG_8,
    [8] = CMBN_FW_EXCP_LOG_REG_9,
    [9] = CMBN_FW_EXCP_LOG_REG_10,
    [10] = CMBN_FW_EXCP_LOG_REG_11,
    [11] = CMBN_FW_EXCP_LOG_REG_12,
    [12] = CMBN_FW_EXCP_LOG_REG_13,
    [13] = CMBN_FW_EXCP_LOG_REG_14,
    [14] = CMBN_FW_EXCP_LOG_REG_15,
    [15] = CMBN_FW_EXCP_LOG_REG_16,
    [16] = CMBN_FW_EXCP_LOG_REG_17,
    [17] = CMBN_FW_EXCP_LOG_REG_18,
    [18] = CMBN_FW_EXCP_LOG_REG_19, 
    [19] = CMBN_FW_EXCP_LOG_REG_20,
    [20] = CMBN_FW_EXCP_LOG_REG_21, 
    [21] = CMBN_FW_EXCP_LOG_REG_24,
    [22] = CMBN_FW_EXCP_LOG_REG_25,
    [23] = CMBN_FW_EXCP_LOG_REG_26,
    [24] = CMBN_FW_EXCP_LOG_REG_27,
    [25] = CMBN_FW_EXCP_LOG_REG_28,
    [26] = CMBN_FW_EXCP_LOG_REG_29,
    [27] = CMBN_FW_EXCP_LOG_REG_30,
    [28] = CMBN_FW_EXCP_LOG_REG_31,
    [29] = CMBN_FW_EXCP_LOG_REG_32,
    [30] = CMBN_FW_EXCP_LOG_REG_36,
    [31] = CMBN_FW_EXCP_LOG_REG_37,
    [32] = CMBN_FW_EXCP_LOG_REG_38,
    [33] = CMBN_FW_EXCP_LOG_REG_39, 
    [34] = CMBN_FW_EXCP_LOG_REG_40,
    [35] = CMBN_FW_EXCP_LOG_REG_41,
    [36] = CMBN_FW_EXCP_LOG_REG_42,
    [37] = CMBN_FW_EXCP_LOG_REG_43,
    [38] = CMBN_FW_EXCP_LOG_REG_44,
    [39] = CMBN_FW_EXCP_LOG_REG_45,
    [40] = CMBN_FW_EXCP_LOG_REG_46,
    [41] = CMBN_FW_EXCP_LOG_REG_47, 
    [42] = CMBN_FW_EXCP_LOG_REG_48,
    [43] = CMBN_FW_EXCP_LOG_REG_50,
    [44] = CMBN_FW_EXCP_LOG_REG_51,
    [45] = CMBN_FW_EXCP_LOG_REG_52,
    [46] = CMBN_FW_EXCP_LOG_REG_53,
    [47] = CMBN_FW_EXCP_LOG_REG_54,
    [48] = CMBN_FW_EXCP_LOG_REG_55,
    [49] = CMBN_FW_EXCP_LOG_REG_56,
    [50] = CMBN_FW_EXCP_LOG_REG_57, 
};  

void App_fwAbortHandlerIsr(void)
{
     App_sciclientPrintf("Exception due to abort handler\n");
}

void App_fwExcepSendTrace(App_fwExceptionData_t *ptr)
{
    uint32_t i = 0;
    App_sciclientPrintf("Firewall exception Logging register\n");
    App_sciclientPrintf("FW Exception %x\n", ptr->hdr[0]);
    App_sciclientPrintf("%x\n", ptr->hdr[1]);
    for (i = 0; i < SIZE_EXCEPTION_LOG_REG; i++) {
        App_sciclientPrintf("%x\n", ptr->data[i]);
    }
    App_sciclientPrintf("\n");

    return;
}

void App_fwNotiIsrDmsc(void)
{  
    App_fwExceptionData_t *excepPtr;
    excepPtr = (App_fwExceptionData_t *) gDmscGlbRegs;
    gInterruptRecieved++; 
    /* Do trace */
    App_fwExcepSendTrace(excepPtr);
}

void App_fwNotiIsrCmbn(void)
{
    uint32_t i = 0;
    App_fwExceptionData_t *excepPtr;
    gInterruptRecieved++; 
    for (i = 0; i < FW_MAX_ISR_INPUTS; i++) 
    {
        excepPtr =
            (App_fwExceptionData_t *)
            gFwExcpExtInputRegAddr
            [i];
        if (excepPtr != NULL && excepPtr->hdr[0]!=0) 
        {
            App_fwExcepSendTrace(excepPtr);
        }
        
    }
}




