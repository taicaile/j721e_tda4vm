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

/*!
 * \file  cpsw_ale_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        CPSW ALE module which are meant for internal use in Enet Per drivers.
 */

#ifndef CPSW_ALE_PRIV_H_
#define CPSW_ALE_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/csl/csl_cpswitch.h>
#include <ti/drv/enet/include/core/enet_mod_fdb.h>
#include <ti/drv/enet/include/mod/cpsw_ale.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Flow priority feature mask. */
#define CPSW_ALE_FEATURE_FLOW_PRIORITY        (ENET_BIT(0U))

/*! \brief IP header white-list feature mask. */
#define CPSW_ALE_FEATURE_IP_HDR_WHITELIST     (ENET_BIT(1U))

/*! \brief Multihost mode feature mask. */
#define CPSW_ALE_FEATURE_MULTIHOST            (ENET_BIT(2U))

/*! \brief Number of words in an ALE Table Entry. */
#define CPSW_ALE_ENTRY_WORDS                  (3U)

/*! \brief Number of words in an ALE Policer Table Entry. */
#define CPSW_ALE_POL_ENTRY_WORDS              (8U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Address type: source or destination.
 */
typedef enum CpswAle_AddrType_e
{
    /*! Source address type */
    CPSW_ALE_ADDR_TYPE_SRC,

    /*! Destination address type */
    CPSW_ALE_ADDR_TYPE_DST
} CpswAle_AddrType;

/*!
* \brief Structure to save context of ALE
*/
typedef struct CpswAle_Ctxt_s
{
    /*! ALE_CONTROL register value */
    uint32_t control;

    /*! ALE_CONTROL 2 register value */
    uint32_t control2;

    /*! ALE_PRESCALE register value */
    uint32_t prescale;

    /*! ALE_AGING_CTRL register value */
    uint32_t agingCtrl;

    /*! ALE Port Control register value */
    uint32_t portCtl[CPSW_ALE_NUM_PORTS];

    /*! ALE_UVLAN_MEMBER - ALE Unknown VLAN Member Mask Register register value */
    uint32_t uvlanMember;

    /*! ALE_UVLAN_URCAST - ALE Unknown VLAN Unregistered Multicast Flood Mask Register register value */
    uint32_t uvlanUrcast;

    /*! ALE_UVLAN_RMCAST - ALE Unknown VLAN Registered Multicast Flood Mask Register register value */
    uint32_t uvlanRmcast;

    /*! ALE_UVLAN_UNTAG - ALE Unknown VLAN force Untagged Egress Mask Register register value */
    uint32_t uvlanUntag;

    /*! POLICECONTROL register value */
    uint32_t policeControl;

    /*! THREADMAPDEF register value */
    uint32_t threadMapDef;

    /*! THREADMAPCTL register value */
    uint32_t threadMapCtl;

    /*! THREADMAPVAL register value */
    uint32_t threadMapVal;

    /*! ALE Table Entries  saved for  MAX_ALE_ENTRY_SAVED entries */
    uint32_t aleTable[CPSW_ALE_MAX_TABLE_ENTRY_SAVED][CPSW_ALE_ENTRY_WORDS];

    /*! ALE Policer Table Entries  saved for  MAX_POLICER_ENTRY_SAVED entries */
    uint32_t alePolTable[CPSW_ALE_MAX_POLICER_ENTRY_SAVED][CPSW_ALE_POL_ENTRY_WORDS];
} CpswAle_Ctxt;

 /*!
 * \brief Structure to define start and end index of a partition in policer
 */
typedef struct CpswAle_PolicerPartInfo_s
{
    /*! Partition start index */
    uint16_t startIdx;

    /*! Partition end index */
    uint16_t endIdx;
} CpswAle_PolicerPartInfo;

/*!
 * \brief CPSW ALE object.
 */
typedef struct CpswAle_Obj_s
{
    /*! EnetMod must be the first member */
    EnetMod_Obj enetMod;

    /*! Total number of ports */
    uint32_t numPorts;

    /*! Type of ALE table */
    CSL_CPSW_ALETABLE_TYPE tableType;

    /*! Ethernet peripheral type. Required to query SoC parameters (clock freq) */
    Enet_Type enetType;

    /*! Peripheral instance number. Required to query SoC parameters (clock freq) */
    uint32_t instId;

    /*! CPSW in VLAN aware mode (i.e port VLAN id enabled) */
    bool pvidEn;

    /*! Active receive filter settings */
    CpswAle_RxFilter rxFilter;

    /*! Whether ALE timer is active or not */
    bool softTimerActive;

    /*! Current timer count (increments when ALE timer is marked as active) */
    uint32_t softTickCnt;

    /*! ALE timeout count */
    uint32_t tickTimeoutCnt;

    /*! ALE functional clock frequency in Hz */
    uint32_t aleFreqHz;

    /*! Default Port VLAN ID */
    CpswAle_PortVlanCfg pvid[CPSW_ALE_NUM_PORTS];

    /*! ALE policer partition table */
    CpswAle_PolicerPartInfo policerTablePartInfo[CPSW_ALE_POLICER_TABLE_PART_MAX];

    /*! ALE context */
    CpswAle_Ctxt ctxt;
} CpswAle_Obj;

/*!
 * \brief CPSW ALE module handle.
 */
typedef CpswAle_Obj *CpswAle_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Open and initialize CPSW ALE.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param cfg       Configuration parameters
 * \param cfgSize   Size of the configuration parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswAle_open(EnetMod_Handle hMod,
                     Enet_Type enetType,
                     uint32_t instId,
                     const void *cfg,
                     uint32_t cfgSize);

/*!
 * \brief Rejoin a running CPSW ALE module.
 *
 * \param hMod      Enet Module handle
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswAle_rejoin(EnetMod_Handle hMod,
                       Enet_Type enetType,
                       uint32_t instId);

/*!
 * \brief Run an IOCTL operation on CPSW ALE.
 *
 * \param hMod         Enet Module handle
 * \param cmd          IOCTL command Id
 * \param prms         IOCTL parameters
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswAle_ioctl(EnetMod_Handle hMod,
                      uint32_t cmd,
                      Enet_IoctlPrms *prms);

/*!
 * \brief Close CPSW ALE.
 *
 * \param hMod         Enet Module handle
 */
void CpswAle_close(EnetMod_Handle hMod);

/*!
 * \brief Save the context for CPSW ALE.
 *
 * \param hMod         Enet Module handle
 */
void CpswAle_saveCtxt(EnetMod_Handle hMod);

/*!
 * \brief Restore the context for CPSW ALE.
 *
 * \param hMod         Enet Module handle
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t CpswAle_restoreCtxt(EnetMod_Handle hMod);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* CPSW_ALE_PRIV_H_ */
