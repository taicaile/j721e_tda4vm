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
 *  \file vhwa_soc_priv.h
 *
 *  \brief VHWA Driver J721E SOC file containing private APIs used by VHWA
 *         controller driver.
 */

#ifndef VHWA_SOC_PRIV_H_
#define VHWA_SOC_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/csl/cslr_vpac.h>
#include <ti/csl/cslr_dmpac.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Structure containing information about MSC register space.
 */
typedef struct
{
    CSL_vpac_cntlRegs       *vpacCntlRegs;
    /**< VPAC Top Register Base address */
    CSL_vpac_intd_cfgRegs   *vpacIntdRegs;
    /**< VPAC intd Register Base address */
    CSL_mscRegs             *mscRegs;
    /**< MSC Register Base Address */
    CSL_lseRegs             *lseRegs;
    /**< LSE Register Base Address */
    CSL_htsRegs             *htsRegs;
    /**< UTC Base Address */
} Msc_SocInfo;

typedef struct
{
    CSL_vpac_cntlRegs       *vpacCntlRegs;
    /**< VPAC Top Register Base address */
    CSL_vpac_intd_cfgRegs   *vpacIntdRegs;
    /**< VPAC intd Register Base address */
    CSL_ldc_coreRegs        *ldcRegs;
    /**< MSC Register Base Address */
    CSL_lseRegs             *lseRegs;
    /**< LSE Register Base Address */
    CSL_htsRegs             *htsRegs;
    /**< UTC Base Address */
    uint32_t                *lumaLutBaseAddr;
    /**< Luma Lut Base Address */
    uint32_t                *chromaLutBaseAddr;
    /**< Luma Lut Base Address */
} Ldc_SocInfo;

typedef struct
{
    CSL_dmpacRegs            *dmpacCntlRegs;
    /**< DMPAC Top Register Base address */
    CSL_dmpac_intd_cfgRegs   *dmpacIntdRegs;
    /**< DMPAC intd Register Base address */
    CSL_dmpac_foco_coreRegs  *dmpacFocoRegs;
    /**< DMPAC FOCO Register Base address */
    CSL_dmpac_dofRegs        *dofRegs;
    /**< DOF Register Base Address */
    CSL_lseRegs              *lseRegs;
    /**< LSE Register Base Address */
    CSL_htsRegs              *htsRegs;
    /**< UTC Base Address */
} Dof_SocInfo;

typedef struct
{
    CSL_dmpacRegs            *dmpacCntlRegs;
    /**< DMPAC Top Register Base address */
    CSL_dmpac_intd_cfgRegs   *dmpacIntdRegs;
    /**< DMPAC intd Register Base address */
    CSL_dmpac_foco_coreRegs  *dmpacFocoRegs;
    /**< DMPAC FOCO Register Base address */
    CSL_dmpac_sdeRegs        *sdeRegs;
    /**< SDE Register Base Address */
    CSL_lseRegs              *lseRegs;
    /**< LSE Register Base Address */
    CSL_htsRegs              *htsRegs;
    /**< UTC Base Address */
} Sde_SocInfo;

typedef struct
{
    CSL_vpac_cntlRegs       *vpacCntlRegs;
    /**< VPAC Top Register Base address */
    CSL_vpac_intd_cfgRegs   *vpacIntdRegs;
    /**< VPAC intd Register Base address */
    uint32_t                 vissBaseAddr;
    /**< VISS Base Register Address,
     *   From this base address, all other sub-module's addresses
     *   are derived using cslr_vpac_viss.h file */
    CSL_htsRegs             *htsRegs;
    /**< HTS Register Base Address */
} Viss_SocInfo;

/**
 *  \brief Structure containing information about NF register space.
 */
typedef struct
{
    CSL_vpac_cntlRegs       *vpacCntlRegs;
    /**< VPAC Top Register Base address */
    CSL_vpac_intd_cfgRegs   *vpacIntdRegs;
    /**< VPAC intd Register Base address */
    CSL_vpac_nfRegs         *nfRegs;
    /**< NF Register Base Address */
    CSL_lseRegs             *lseRegs;
    /**< LSE Register Base Address */
    CSL_htsRegs             *htsRegs;
    /**< UTC Base Address */
} Nf_SocInfo;

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
 *  \return Pointer to Msc_SocInfo object
 */
void Msc_getSocInfo(Msc_SocInfo *socInfo);

/**
 *  \brief API to get Soc Specific information, like base address,
 *         irq number ete., for LDC Driver.
 *
 *  \param  None
 *
 *  \return Pointer to Ldc_SocInfo object
 */
void Ldc_getSocInfo(Ldc_SocInfo *socInfo);

/**
 *  \brief API to get Soc Specific information, like base address,
 *         irq number ete., for DOF Driver.
 *
 *  \param  None
 *
 *  \return Pointer to Dof_SocInfo object
 */
void Dof_getSocInfo(Dof_SocInfo *socInfo);

/**
 *  \brief API to get Soc Specific information, like base address,
 *         irq number ete., for VISS Driver.
 *
 *  \param  None
 *
 *  \return Pointer to Msc_SocInfo object
 */
void Viss_getSocInfo(Viss_SocInfo *socInfo);

/**
 *  \brief API to get Soc Specific information, like base address,
 *         irq number ete., for NF Driver.
 *
 *  \param  None
 *
 *  \return Pointer to Nf_SocInfo object
 */
void Nf_getSocInfo(Nf_SocInfo *socInfo);

/**
 *  \brief API to get Soc Specific information, like base address,
 *         irq number ete., for SDE Driver.
 *
 *  \param  None
 *
 *  \return Pointer to Sde_SocInfo object
 */
void Sde_getSocInfo(Sde_SocInfo *socInfo);

void Fc_getSocHtsInfo(CSL_htsRegs **htsRegs);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef VHWA_SOC_PRIV_H_ */
