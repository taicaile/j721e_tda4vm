/**
 *   Copyright (c) Texas Instruments Incorporated 2018
 *   All rights reserved.
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
 *  \file vhwa_soc.h
 *
 *  \brief VHWA Driver J721S2/J784S4 SOC file containing private APIs used by VHWA
 *         controller driver.
 */

#ifndef VHWA_SOC_H_
#define VHWA_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/csl/cslr_vpac3.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** Number of spare socket for VISS */
#define VHWA_VISS_SPARE_SOCKET          (1u)
/** Number of spare socket for MSC */
#define VHWA_MSC_SPARE_SOCKET           (1u)
/**
 *  \anchor VHWA_irqNum
 *  \name   Interrupt numbers for VHWA
 *  \brief  This is default IRQ numbers for the VHWA IPs for the Main R5F core
 *          This can be changed by user, after initializing init parameters.
 *
 *  @{
 */
#if defined VHWA_M2M_VPAC_INSTANCE
#if (VHWA_M2M_VPAC_INSTANCE == 0)
    #define VHWA_M2M_VISS_IRQ_NUM                               (34u)
    #define VHWA_M2M_MSC0_IRQ_NUM                               (35u)
    #define VHWA_M2M_MSC1_IRQ_NUM                               (36u)
    #define VHWA_M2M_LDC_IRQ_NUM                                (37u)
    #define VHWA_M2M_NF_IRQ_NUM                                 (38u)
#endif
#endif
#if defined VHWA_M2M_VPAC_INSTANCE
#if (VHWA_M2M_VPAC_INSTANCE == 1)
    #define VHWA_M2M_VISS_IRQ_NUM                               (330u)
    #define VHWA_M2M_MSC0_IRQ_NUM                               (331u)
    #define VHWA_M2M_MSC1_IRQ_NUM                               (332u)
    #define VHWA_M2M_LDC_IRQ_NUM                                (333u)
    #define VHWA_M2M_NF_IRQ_NUM                                 (334u)
#endif
#endif

#define VHWA_M2M_SDE_IRQ_NUM                                (32u)
#define VHWA_M2M_DOF_IRQ_NUM                                (33u)
/** @} */

/** \brief DMPAC SL2 start Address */
#define DMPAC_SL2_START_ADDRESS         \
    (CSL_DMPAC0_DMPAC_TOP_DOF_INFRA_DMPAC_BASE_MEM_SLV_CBASS_STRIPE_MSRAM_SLV_BASE)

/** \brief VPAC SL2 start Address */
#define VPAC_SL2_START_ADDRESS          (0x3f00000000ULL)

/** \brief DMPAC/VPAC SL2 Size */
#define VPAC_DMPAC_SL2_SIZE             \
    (CSL_VPAC0_VPAC_TOP_PAC_BASE_MEM_SLV_CBASS_STRIPE_MSRAM_SLV_SIZE)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    uint32_t           irqNum;
    /**< Core IRQ Number to be used for LDC,
     *   Used internally for isr registration */
    uint32_t           vhwaIrqNum;
    /**< VHWA IRQ Number,
     *   Refer to #VHWA_IrqNum for valid number.
     *   Used to enable the irq in INTD number */
} Vhwa_IrqInfo;

/* ========================================================================== */
/*                  Internal/Private Function Declarations                    */
/* ========================================================================== */
/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief API to get Soc Specific information, like base address,
 *         irq number ete., for MSC Driver.
 *
 *  \param  None
 *
 *  \return Pointer to Vhwa_IrqInfo object
 */
void Msc_getIrqInfo(Vhwa_IrqInfo irqInfo[]);

/**
 *  \brief API to get Soc Specific information, like base address,
 *         irq number ete., for LDC Driver.
 *
 *  \param  None
 *
 *  \return Pointer to Vhwa_IrqInfo object
 */
void Ldc_getIrqInfo(Vhwa_IrqInfo *irqInfo);

/**
 *  \brief API to get interrupt number for DOF,
 *
 *  \param  None
 *
 *  \return Pointer to Vhwa_IrqInfo object
 */
void Dof_getIrqInfo(Vhwa_IrqInfo *irqInfo);

/**
 *  \brief API to get Soc Specific information, like base address,
 *         irq number ete., for VISS Driver.
 *
 *  \param  None
 *
 *  \return Pointer to Vhwa_IrqInfo object
 */
void Viss_getIrqInfo(Vhwa_IrqInfo *irqInfo);

/**
 *  \brief API to get Soc Specific information, like base address,
 *         irq number ete., for NF Driver.
 *
 *  \param  None
 *
 *  \return Pointer to Vhwa_IrqInfo object
 */
void Nf_getIrqInfo(Vhwa_IrqInfo *irqInfo);

/**
 *  \brief API to get interrupt number for SDE
 *
 *  \param  None
 *
 *  \return Pointer to Vhwa_IrqInfo object
 */
void Sde_getIrqInfo(Vhwa_IrqInfo *irqInfo);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef VHWA_SOC_H_ */
