/********************************************************************
 * Copyright (C) 2017 Texas Instruments Incorporated.
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
 *  Name        : cslr_viss_fcp.h
*/
#ifndef CSLR_VISS_FCP_H_
#define CSLR_VISS_FCP_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <ti/csl/cslr.h>
#include <stdint.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_VISS_FCP_FLEXCC_C8G8_REGS_BASE                                     (0x00002800U)
#define CSL_VISS_FCP_FLEXCC_CFG_REGS_BASE                                      (0x00000000U)
#define CSL_VISS_FCP_FLEXCC_CONTRASTC1_REGS_BASE                               (0x00000800U)
#define CSL_VISS_FCP_FLEXCC_CONTRASTC2_REGS_BASE                               (0x00001000U)
#define CSL_VISS_FCP_FLEXCC_CONTRASTC3_REGS_BASE                               (0x00001800U)
#define CSL_VISS_FCP_FLEXCC_HIST_REGS_BASE                                     (0x00003800U)
#define CSL_VISS_FCP_FLEXCC_LINE_REGS_BASE                                     (0x00008000U)
#define CSL_VISS_FCP_FLEXCC_S8B8_REGS_BASE                                     (0x00003000U)
#define CSL_VISS_FCP_FLEXCC_Y8R8_REGS_BASE                                     (0x00002000U)
#define CSL_VISS_FCP_FLEXCFA_REGS_BASE                                         (0x00000000U)
#define CSL_VISS_FCP_FLEXEE_REGS_BASE                                          (0x00000000U)


#ifdef __cplusplus
}
#endif
#endif
