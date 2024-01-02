/*
 * Copyright (c) 2019, Texas Instruments Incorporated
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
/*
 *  ======== Core_utils.c ========
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ti/osal/osal.h>
#include <ti/osal/soc/osal_soc.h>

#if defined(BUILD_MCU)
#include <ti/csl/arch/r5/csl_arm_r5.h>
#elif defined(BUILD_C7X)
#include <ti/csl/csl_clec.h>
#elif defined (_TMS320C6X)
#include <ti/csl/csl_chipAux.h>
#endif

uint32_t Osal_getCoreId(void)
{
    uint32_t coreId=OSAL_INVALID_CORE_ID;
#if defined (BUILD_MCU)
	CSL_ArmR5CPUInfo info = {0};

    CSL_armR5GetCpuID(&info);
    if (info.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)
    {
        /* MAIN SS Pulsar R5 SS0 */
        coreId = (info.cpuID == CSL_ARM_R5_CPU_ID_0)?
                                    OSAL_MCU1_0:
                                        OSAL_MCU1_1;
    }
    else if (info.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1)
    {
        /* MAIN SS Pulsar R5 SS0 */
        coreId = (info.cpuID == CSL_ARM_R5_CPU_ID_0)?
                                    OSAL_MCU2_0:
                                        OSAL_MCU2_1;
    }
#if defined (SOC_J721E) || defined(SOC_J721S2) || defined (SOC_J784S4)
    else if (info.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_2)
    {
        /* MAIN SS Pulsar R5 SS1 */
        coreId = (info.cpuID == CSL_ARM_R5_CPU_ID_0)?
                                    OSAL_MCU3_0:
                                        OSAL_MCU3_1;
    }
#if defined (SOC_J784S4)
	else if (info.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_3)
    {
        /* MAIN SS Pulsar R5 SS1 */
        coreId = (info.cpuID == CSL_ARM_R5_CPU_ID_0)?
                                    OSAL_MCU4_0:
                                        OSAL_MCU4_1;
    }
#endif /* #if defined (SOC_J721E) || defined(SOC_J721S2) || defined (SOC_J784S4) */
#endif /* #if defined (SOC_J784S4) */
	else
	{
		coreId = OSAL_INVALID_CORE_ID;
	}
#elif defined (BUILD_C7X)
	uint32_t rtmapCpuId;
	rtmapCpuId = CSL_clecGetC7xRtmapCpuId();

	if(CSL_CLEC_RTMAP_CPU_4 == rtmapCpuId)
	{
		coreId=OSAL_C7X_1;
	}
#if defined (SOC_J721S2) || defined (SOC_J784S4)
	else if(CSL_CLEC_RTMAP_CPU_5 == rtmapCpuId)
	{
		coreId=OSAL_C7X_2;
	}
#if defined (SOC_J784S4)
	else if (CSL_CLEC_RTMAP_CPU_6 == rtmapCpuId)
	{
		coreId=OSAL_C7X_3;
	}
	else if (CSL_CLEC_RTMAP_CPU_7 == rtmapCpuId)
	{
		coreId=OSAL_C7X_4;
	}
#endif /* #if defined (SOC_J784S4) */
#endif /* #if defined (SOC_J721S2) || defined (SOC_J784S4) */
	else
	{
		coreId = OSAL_INVALID_CORE_ID;
	}
#elif defined (BUILD_C66X)
	uint32_t dspNum;
	dspNum = CSL_chipReadDNUM();
	if(0U == dspNum)
	{
		coreId = OSAL_C66_1;
	}
	else if (1U == dspNum)
	{
		coreId = OSAL_C66_2;
	}
	else
	{
		coreId = OSAL_INVALID_CORE_ID;
	}
#endif
	return coreId;
}

void CacheP_fenceCpu2Dma(uintptr_t addr, uint32_t size, Osal_CacheP_isCoherent isCoherent)
{
    /* CPU to DMA would be to do Wb call if it is not coherent */
    if (OSAL_CACHEP_NOT_COHERENT == isCoherent)
    {
        CacheP_wb( (const void *)addr, size);
    }
}

void CacheP_fenceDma2Cpu(uintptr_t addr, uint32_t size, Osal_CacheP_isCoherent isCoherent)
{
    /* DMA to CPU would be to do Cache inv call if it is not coherent */
    if (OSAL_CACHEP_NOT_COHERENT == isCoherent)
    {
        CacheP_Inv( (const void *)addr, size);
    }
}
/* Nothing past this point */
