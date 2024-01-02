/*
 *  Copyright (c) Texas Instruments Incorporated 2023
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *	Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 *
 *	Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in the
 *	documentation and/or other materials provided with the
 *	distribution.
 *
 *	Neither the name of Texas Instruments Incorporated nor the names of
 *	its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/enet/examples/utils/include/enet_apputils.h>
#include <ti/drv/enet/examples/utils/include/enet_mcm.h>
#include <ti/drv/enet/examples/utils/include/enet_board.h>
#include <ti/osal/LoadP.h>
#include <tsn_gptp/gptp_config.h>
#include <tsn_combase/combase.h>
#include <ti/drv/enet/enet.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Stack size and alignment of the task in charge of gPTP stack initialization
 * and CPU load */
#define TSN_TSK_STACK_SIZE              (16U * 1024U)
#define TSN_TSK_STACK_ALIGN             TSN_TSK_STACK_SIZE

/* Maximum number of Ethernet ports (CPSW9G in J721E/J784S4) */
#define MAX_NUM_MAC_PORTS               (8U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* Example application parameters: CPSW instance and ports to be tested */
typedef struct EnetApp_Cfg_s
{
    /* Peripheral instance type */
    Enet_Type enetType;

    /* Peripheral instance id */
    uint32_t instId;

    /* Ethernet ports to be enabled */
    EnetBoard_EthPort ethPorts[MAX_NUM_MAC_PORTS];

    /* Number of Ethernet ports to be enabled */
    uint32_t numEthPorts;
} EnetApp_Cfg;

/* Container structure of all example app variables */
typedef struct EnetApp_Obj_s
{
    /* UDMA driver handle */
    Udma_DrvHandle hUdmaDrv;

    /* Handle to task which prints CPU load periodically */
    TaskP_Handle hTsnTask;
} EnetApp_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

extern int32_t EnetApp_initTsn(void);
extern int32_t EnetApp_deinitTsn(void);
extern int32_t EnetApp_gPtpStart(char *netdevs[]);

static int32_t EnetApp_init(void);
static void EnetApp_deinit(void);
static void EnetApp_appInitTask(void *a0, void *a1);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static uint8_t gAvbAppTskStack[TSN_TSK_STACK_SIZE]
__attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#if defined(SOC_J721E)
#define CFG_NUM_MAC_PORTS (2U)
static EnetApp_Cfg gEnetAppCfg =
{
    .enetType = ENET_CPSW_9G,
    .instId   = 0U,
    .ethPorts =
    {
        {    /* 'PRG1_RGMII1_B' in GESI expansion board */
            .macPort = ENET_MAC_PORT_1,
            .mii     = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
            .boardId = ENETBOARD_GESI_ID,
            .expPort = ENETBOARD_EXP_PORT_GESI,
        },
        {   /* 'PRG0_RGMII1_B' in GESI expansion board */
            .macPort = ENET_MAC_PORT_3,
            .mii     = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
            .boardId = ENETBOARD_GESI_ID,
            .expPort = ENETBOARD_EXP_PORT_GESI,
        }
    },
    .numEthPorts = CFG_NUM_MAC_PORTS,
};
#elif defined(SOC_J7200)
#define CFG_NUM_MAC_PORTS (1U)
static EnetApp_Cfg gEnetAppCfg =
{
    .enetType = ENET_CPSW_5G,
    .instId   = 0U,
    .ethPorts =
    {
        {   /* 'P0' port in QpENet board (QSGMII expansion board) */
            .macPort = ENET_MAC_PORT_1,
            .mii     = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_QUAD_SERIAL_MAIN },
            .boardId = ENETBOARD_QPENET_ID,
            .expPort = ENETBOARD_EXP_PORT_ENET,
        },
    },
    .numEthPorts = CFG_NUM_MAC_PORTS,
};
#elif defined(SOC_J721S2)
#define CFG_NUM_MAC_PORTS (1U)
static EnetApp_Cfg gEnetAppCfg =
{
    .enetType = ENET_CPSW_2G,
    .instId   = 1U,
    .ethPorts =
    {
        {    /* 'PRG0_RGMII1_B' port in GESI expansion board */
            .macPort = ENET_MAC_PORT_1,
            .mii     = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_REDUCED },
            .boardId = ENETBOARD_GESI_ID,
            .expPort = ENETBOARD_EXP_PORT_GESI,
        },
    },
    .numEthPorts = CFG_NUM_MAC_PORTS,
};
#elif defined(SOC_J784S4)
#define CFG_NUM_MAC_PORTS (1U)
static EnetApp_Cfg gEnetAppCfg =
{
    .enetType = ENET_CPSW_9G,
    .instId   = 0U,
    .ethPorts =
    {
        {   /* 'P0' port in QpENet board (QSGMII expansion board) */
            .macPort = ENET_MAC_PORT_1,
            .mii     = { ENET_MAC_LAYER_GMII, ENET_MAC_SUBLAYER_QUAD_SERIAL_MAIN },
            .boardId = ENETBOARD_QPENET_ID,
            .expPort = ENETBOARD_EXP_PORT_ENET1,
        },
    },
    .numEthPorts = CFG_NUM_MAC_PORTS,
};
#else
#error "SOC is not supported"
#endif

/* Container object of all TSN example app related parameters/variables */
static EnetApp_Obj gEnetAppObj =
{
    .hTsnTask  = NULL,
};

/* These 2 following vars are required to be in global memory */
static char gEnetApp_netDevs[CFG_NUM_MAC_PORTS][IFNAMSIZ] = {0};
static char *gEnetApp_gPtpNetDevs[CFG_NUM_MAC_PORTS+1] = {0};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int main(void)
{
    TaskP_Params params;

    OS_init();

    TaskP_Params_init(&params);
    params.name	     = (char *)"tsnapp_task";
    params.priority  = 15U;
    params.stack     = gAvbAppTskStack;
    params.stacksize = sizeof(gAvbAppTskStack);

    gEnetAppObj.hTsnTask = TaskP_create(EnetApp_appInitTask, &params);
    if (gEnetAppObj.hTsnTask == NULL)
    {
        EnetAppUtils_print("Failed to create CPU load task\n");
        EnetAppUtils_assert(false);
    }

    OS_start();	/* does not return */

    return(0);
}

static void EnetApp_appInitTask(void *a0, void *a1)
{
    int useHwPhase = 1;
    int singleClk = 1;
    lld_ethdev_t ethdevs[MAX_NUMBER_ENET_DEVS] = {0};
    int32_t i;
    int32_t status;

    /* Board initialization which will configure pinmux, UART through PDK board library */
    EnetBoard_init();

    EnetAppUtils_print("==========================\n");
    EnetAppUtils_print("     TSN Example App      \n");
    EnetAppUtils_print("==========================\n");

    /* Configure CPSW clocks: CPPI_CLK, RGMII_MHZ_[5,50,250]_CLK and
     * CPTS_REF_CLK to 500MHz */
    EnetAppUtils_enableClocks(gEnetAppCfg.enetType, gEnetAppCfg.instId);

    /* Setup requested Ethernet ports in the board, including SERDES,
     * clocking, PHY config params, etc. */
    status = EnetBoard_setupPorts(gEnetAppCfg.enetType,
                                  gEnetAppCfg.instId,
                                  &gEnetAppCfg.ethPorts[0U],
                                  gEnetAppCfg.numEthPorts);
    EnetAppUtils_assert(status == ENET_SOK);
    EnetAppUtils_print("%s: EnetBoard_setupPorts() done\n", __func__);

    /* Open Enet LLD and UDMA LLD */
    status = EnetApp_init();
    EnetAppUtils_assert(status == ENET_SOK);
    EnetAppUtils_print("%s: EnetApp_init() done\n", __func__);

    /* Initialize TSN stack */
    if (EnetApp_initTsn() < 0)
    {
        EnetAppUtils_print("Failed to tsn_init!\n");
        EnetAppUtils_assert(false);
    }

    /* Populate netdevs from MAC ports enabled in the test */
    for (i = 0; i < gEnetAppCfg.numEthPorts; i++)
    {
        snprintf(&gEnetApp_netDevs[i][0], IFNAMSIZ, "tilld%d", i);
        /* gEnetApp_gPtpNetDevs[i] must hold a global address, not a stack address */
        gEnetApp_gPtpNetDevs[i] = &gEnetApp_netDevs[i][0];
        /* ethdevs[i].netdev must hold a global address, not a stack address */
        ethdevs[i].netdev = gEnetApp_netDevs[i];
        ethdevs[i].macport = gEnetAppCfg.ethPorts[i].macPort;
    }

    if (cb_lld_init_devs_table(ethdevs, i, gEnetAppCfg.enetType,
                               gEnetAppCfg.instId) < 0)
    {
        EnetAppUtils_print("Failed to int devs table!\n");
        EnetAppUtils_assert(false);
    }

    /* Apply phase adjustment directly to the HW */
    gptpconf_set_item(CONF_USE_HW_PHASE_ADJUSTMENT, &useHwPhase);

    /* CPSW has a single clock for all the ports */
    gptpconf_set_item(CONF_SINGLE_CLOCK_MODE, &singleClk);

    if (EnetApp_gPtpStart(gEnetApp_gPtpNetDevs) < 0)
    {
        EnetAppUtils_print("Failed to start gptp!\n");
        EnetAppUtils_assert(false);
    }

    EnetAppUtils_print("%s: gptp start done!\n", __func__);

    while (TRUE)
    {
#if defined(FREERTOS)
        EnetAppUtils_print("CPU Load: %u%%\n", LoadP_getCPULoad());
        LoadP_reset();
#endif
        TaskP_sleep(10000U);
    }
}

/*
 * Set ALE config parameters
 */
void EnetApp_initAleConfig(CpswAle_Cfg *aleCfg)
{
    aleCfg->modeFlags = CPSW_ALE_CFG_MODULE_EN;

    aleCfg->agingCfg.autoAgingEn     = true;
    aleCfg->agingCfg.agingPeriodInMs = 1000;

    aleCfg->nwSecCfg.vid0ModeEn = true;

    aleCfg->vlanCfg.aleVlanAwareMode  = FALSE;
    aleCfg->vlanCfg.cpswVlanAwareMode = FALSE;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask   = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask  = CPSW_ALE_ALL_PORTS_MASK;
}

/*
 * Callback called by Enet MCM to request app to populate the link parameters (PHY config,
 * MAC port config, speed/duplex) to be used to open a given MAC port.
 * MCM will call this callback for each port in enetMcmCfg.macPortList.
 */
void EnetApp_initLinkArgs(EnetPer_PortLinkCfg *linkArgs,
                          Enet_MacPort macPort)
{
    CpswMacPort_Cfg *macCfg = (CpswMacPort_Cfg *)linkArgs->macCfg;
    EnetPhy_Cfg *phyCfg = &linkArgs->phyCfg;
    EnetMacPort_LinkCfg *linkCfg = &linkArgs->linkCfg;
    EnetMacPort_Interface *mii = &linkArgs->mii;
    const EnetBoard_EthPort *ethPort = NULL;
    const EnetBoard_PortCfg *portCfg;
    uint32_t i;

    /* Setup board for requested Ethernet port */
    for (i = 0U; i < gEnetAppCfg.numEthPorts; i++)
    {
        if (gEnetAppCfg.ethPorts[i].macPort == macPort)
        {
            ethPort = &gEnetAppCfg.ethPorts[i];
        }
    }

    /* Check that we have config params for the requested MAC port */
    if (ethPort == NULL)
    {
        EnetAppUtils_print("MAC port %u is not enabled in the test\n", ENET_MACPORT_ID(macPort));
        EnetAppUtils_assert(false);
    }

    /* Initialize MAC configuration with driver's default values */
    CpswMacPort_initCfg(macCfg);

    /* Get Ethernet port configuration params from Enet board library, including MAC port
     * params, PHY config params, MII type, speed/duplexity */
    portCfg = EnetBoard_getPortCfg(gEnetAppCfg.enetType, gEnetAppCfg.instId, ethPort);
    if (portCfg != NULL)
    {
        EnetPhy_initCfg(phyCfg);
        phyCfg->phyAddr = portCfg->phyCfg.phyAddr;
        phyCfg->isStrapped = portCfg->phyCfg.isStrapped;
        phyCfg->loopbackEn = false;
        phyCfg->skipExtendedCfg = portCfg->phyCfg.skipExtendedCfg;
        phyCfg->extendedCfgSize = portCfg->phyCfg.extendedCfgSize;
        memcpy(phyCfg->extendedCfg, portCfg->phyCfg.extendedCfg, phyCfg->extendedCfgSize);

        *linkCfg = portCfg->linkCfg;

        mii->layerType    = ethPort->mii.layerType;
        mii->sublayerType = ethPort->mii.sublayerType;
        mii->variantType  = ENET_MAC_VARIANT_FORCED;
        macCfg->sgmiiMode = portCfg->sgmiiMode;
    }
    else
    {
        EnetAppUtils_print("No port configuration found for MAC port %u\n", ENET_MACPORT_ID(macPort));
        EnetAppUtils_assert(false);
    }
}

/*
 * Callback called by Enet LLD when there is link-up or link-down events reported by PHY driver
 */
static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\n", ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}

/*
 * Callback called by Enet LLD when MDIO reports a link status change, if MDIO is in normal mode
 */
static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg)
{
}

/*
 * Set link status change callback functions
 */
static void EnetApp_initEnetLinkCbPrms(Cpsw_Cfg *cpswCfg)
{
    cpswCfg->mdioLinkStateChangeCb = EnetApp_mdioLinkStatusChange;
    cpswCfg->mdioLinkStateChangeCbArg = &gEnetAppObj;

    cpswCfg->portLinkStatusChangeCb	= &EnetApp_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = &gEnetAppObj;
}

/*
 * Open UDMA LLD and Enet LLD (via Enet MCM)
 */
static int32_t EnetApp_init(void)
{
    int32_t status = ENET_SOK;
    EnetMcm_InitConfig enetMcmCfg;
    Cpsw_Cfg cpswCfg;
    EnetRm_ResCfg *resCfg;
    EnetUdma_Cfg dmaCfg;
    uint32_t i;

    EnetAppUtils_assert(gEnetAppCfg.numEthPorts <=
                        Enet_getMacPortMax(gEnetAppCfg.enetType, gEnetAppCfg.instId));

    /* Open UDMA */
    gEnetAppObj.hUdmaDrv = EnetAppUtils_udmaOpen(gEnetAppCfg.enetType, NULL);
    EnetAppUtils_assert(NULL != gEnetAppObj.hUdmaDrv);

    dmaCfg.rxChInitPrms.dmaPriority = UDMA_DEFAULT_RX_CH_DMA_PRIORITY;
    dmaCfg.hUdmaDrv = gEnetAppObj.hUdmaDrv;

    resCfg = &cpswCfg.resCfg;
    cpswCfg.dmaCfg = (void *)&dmaCfg;

    Enet_initCfg(gEnetAppCfg.enetType, gEnetAppCfg.instId, &cpswCfg, sizeof(cpswCfg));
    cpswCfg.vlanCfg.vlanAware = false;
    cpswCfg.hostPortCfg.removeCrc = true;
    cpswCfg.hostPortCfg.padShortPacket = true;
    cpswCfg.hostPortCfg.passCrcErrors = true;

    /* CPTS_RFT_CLK is sourced from MAIN_SYSCLK0 (500MHz) */
    cpswCfg.cptsCfg.cptsRftClkFreq = CPSW_CPTS_RFTCLK_FREQ_500MHZ;

    EnetApp_initEnetLinkCbPrms(&cpswCfg);
    resCfg = &cpswCfg.resCfg;

    EnetApp_initAleConfig(&cpswCfg.aleCfg);

    /* Configure MCM (Multi-Client Manager) parameters: peripheral type, port ids, etc */
    enetMcmCfg.perCfg = &cpswCfg;
    EnetAppUtils_assert(NULL != enetMcmCfg.perCfg);

    EnetAppUtils_initResourceConfig(gEnetAppCfg.enetType, gEnetAppCfg.instId,
                                    EnetSoc_getCoreId(), resCfg);

    enetMcmCfg.enetType           = gEnetAppCfg.enetType;
    enetMcmCfg.instId             = gEnetAppCfg.instId;
    enetMcmCfg.setPortLinkCfg     = EnetApp_initLinkArgs;
    enetMcmCfg.numMacPorts        = gEnetAppCfg.numEthPorts;
    enetMcmCfg.periodicTaskPeriod = ENETPHY_FSM_TICK_PERIOD_MS;
    enetMcmCfg.print              = EnetAppUtils_print;
    enetMcmCfg.traceTsFunc        = NULL;
    enetMcmCfg.extTraceFunc       = NULL;

    for (i = 0U; i < gEnetAppCfg.numEthPorts; i++)
    {
        enetMcmCfg.macPortList[i] = gEnetAppCfg.ethPorts[i].macPort;
    }

    /* Initialize MCM which internally opens Enet LLD */
    status = EnetMcm_init(&enetMcmCfg);

    return status;
}

/*
 * Close UDMA LLD and Enet LLD
 */
static void EnetApp_deinit(void)
{
    EnetAppUtils_udmaclose(gEnetAppObj.hUdmaDrv);
    EnetMcm_deInit(gEnetAppCfg.enetType);
    memset(&gEnetAppObj, 0U, sizeof(gEnetAppObj));
}
