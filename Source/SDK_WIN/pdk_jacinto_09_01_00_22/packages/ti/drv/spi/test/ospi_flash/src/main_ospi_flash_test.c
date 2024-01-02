/*
 * Copyright (C) 2017 - 2023 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 *  \file   main_ospi_flash_test.c
 *
 *  \brief  Test application main file. This application will write and read
 *          the data to/from OSPI NOR flash through Board flash interface.
 *
 */
#include <stdint.h>
#include <stdio.h>

#if defined(OSPI_TESTAPP_RTOS)
#include "ti/osal/osal.h"
#include "ti/osal/TaskP.h"
#include "ti/osal/LoadP.h"
#if defined(SOC_AM65XX) || defined(SOC_J721E) || defined(SOC_J7200) || defined(SOC_AM64X) || defined(SOC_J721S2) || defined(SOC_J784S4)
#if defined (__aarch64__)
#include <ti/sysbios/family/arm/v8a/Mmu.h>
#endif
#endif
#endif /* #if defined(OSPI_TESTAPP_RTOS) */

/* TI-RTOS Header files */
#include <ti/drv/spi/SPI.h>
#include <ti/drv/spi/src/SPI_osal.h>
#include <ti/drv/spi/soc/SPI_soc.h>
#include <ti/drv/spi/test/src/SPI_log.h>

#include <ti/board/board.h>
#include <ti/board/board_cfg.h>
#include <ti/board/src/flash/include/board_flash.h>
#include <ti/board/src/flash/nor/ospi/nor_spi_patterns.h>
#if defined(SOC_J7200) || defined(SOC_AM64X)
#include <ti/board/src/flash/nor/ospi/nor_xspi.h>
#elif defined(SOC_J721S2) || defined(SOC_J784S4)
#include <ti/board/src/flash/nor/ospi/nor_xspi.h>
#include <ti/board/src/flash/nand/ospi/nand_ospi.h>
#else
#include <ti/board/src/flash/nor/ospi/nor_ospi.h>
#endif
#if defined(SOC_J721S2) || defined(SOC_J784S4)
#include <ti/csl/csl_gpio.h>
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>
#endif

#ifdef SPI_DMA_ENABLE
#include <ti/osal/CacheP.h>
#endif

#if defined(SOC_AM65XX) || defined(SOC_J721E) || defined(SOC_J7200) || defined(SOC_AM64X) || defined(SOC_J721S2) || defined(SOC_J784S4)
#include <ti/csl/cslr_fss.h>
#include <ti/csl/csl_rat.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/drv/sciclient/sciclient.h>
#ifdef SPI_DMA_ENABLE
#include <ti/drv/udma/udma.h>
#endif
#if defined(SOC_AM64X) || defined(SOC_J7200)
#include <ti/board/src/flash/nor/device/s28hs512t.h>
#elif defined(SOC_J721S2) || defined(SOC_J784S4)
#include <ti/board/src/flash/nor/device/s28hs512t.h>
#include <ti/board/src/flash/nand/device/w35n01jwtbag.h>
#else
#include <ti/board/src/flash/nor/device/m35xu512.h>
#endif

#if defined(SOC_AM65XX)
#include <ti/csl/soc/am65xx/src/cslr_soc_baseaddress.h>
#include <ti/csl/soc/am65xx/src/cslr_mcu_ctrl_mmr.h>
#include <ti/csl/soc/am65xx/src/cslr_mcu_pll_mmr.h>
#include <ti/csl/soc/am65xx/src/cslr_wkup_ctrl_mmr.h>
#endif

#if defined(SOC_J721E)
#include <ti/csl/soc/j721e/src/cslr_soc_baseaddress.h>
#include <ti/csl/soc/j721e/src/cslr_mcu_ctrl_mmr.h>
#include <ti/csl/soc/j721e/src/cslr_mcu_pll_mmr.h>
#include <ti/csl/soc/j721e/src/cslr_wkup_ctrl_mmr.h>
#endif

#if defined(SOC_J7200)
#include <ti/csl/soc/j7200/src/cslr_soc_baseaddress.h>
#include <ti/csl/soc/j7200/src/cslr_mcu_ctrl_mmr.h>
#include <ti/csl/soc/j7200/src/cslr_mcu_pll_mmr.h>
#include <ti/csl/soc/j7200/src/cslr_wkup_ctrl_mmr.h>
#endif

#if defined(SOC_J721S2)
#include <ti/csl/soc/j721s2/src/cslr_soc_baseaddress.h>
#include <ti/csl/soc/j721s2/src/cslr_mcu_ctrl_mmr.h>
#include <ti/csl/soc/j721s2/src/cslr_mcu_pll_mmr.h>
#include <ti/csl/soc/j721s2/src/cslr_wkup_ctrl_mmr.h>
#endif

#if defined(SOC_J784S4)
#include <ti/csl/soc/j784s4/src/cslr_soc_baseaddress.h>
#include <ti/csl/soc/j784s4/src/cslr_mcu_ctrl_mmr.h>
#include <ti/csl/soc/j784s4/src/cslr_mcu_pll_mmr.h>
#include <ti/csl/soc/j784s4/src/cslr_wkup_ctrl_mmr.h>
#endif

#endif


/* Define the OSPI test interface */
typedef struct OSPI_Tests_s
{
    bool     (*testFunc)(void *);
    int32_t  testId;
    bool     dacMode;
    bool     dmaMode;
    bool     norFlash;
    uint32_t phyOpMode;
    uint32_t clk;
    char     testDesc[80];

} OSPI_Tests;

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/
#define OSPI_PROFILE        /* Enable profiling */
#define OSPI_WRITE          /* Enable write */
#define OSPI_WRITE_TUNING   /* Enable write tuning data pattern to the falsh */
#undef  OSPI_QSPI_FLASH     /* Enable QSPI flash test using OSPI controller */

/* OSPI test ID definitions */
#define OSPI_TEST_ID_DAC_133M     0   /* OSPI flash test in Direct Acess Controller mode at 133MHz RCLK */
#define OSPI_TEST_ID_INDAC_133M   1   /* OSPI flash test in Indirect Acess Controller mode at 133MHz RCLK */
#define OSPI_TEST_ID_DAC_DMA_133M 2   /* OSPI flash test in Direct Acess Controller DMA mode at 133MHz RCLK */
#define OSPI_TEST_ID_DAC_166M     3   /* OSPI flash test in Direct Acess Controller mode at 166MHz RCLK */
#define OSPI_TEST_ID_INDAC_166M   4   /* OSPI flash test in Indirect Acess Controller mode at 166MHz RCLK */
#define OSPI_TEST_ID_DAC_DMA_166M 5   /* OSPI flash test in Direct Acess Controller DMA mode at 166MHz RCLK */
#define OSPI_TEST_ID_DAC_133M_SPI 6   /* OSPI flash test in Direct Acess Controller legacy SPI mode at 133MHz RCLK */
#define OSPI_TEST_ID_WR_TUNING    7   /* OSPI flash test in Direct Acess Controller legacy SPI mode to write tuning data */
#define OSPI_NAND_TEST_ID_DAC_133M     8   /* OSPI flash test in Direct Acess Controller mode at 133MHz RCLK */
#define OSPI_NAND_TEST_ID_INDAC_133M   9   /* OSPI flash test in Indirect Acess Controller mode at 133MHz RCLK */
#define OSPI_NAND_TEST_ID_DAC_DMA_133M 10   /* OSPI flash test in Direct Acess Controller DMA mode at 133MHz RCLK */
#define OSPI_NAND_TEST_ID_DAC_166M     11   /* OSPI flash test in Direct Acess Controller mode at 166MHz RCLK */
#define OSPI_NAND_TEST_ID_INDAC_166M   12  /* OSPI flash test in Indirect Acess Controller mode at 166MHz RCLK */
#define OSPI_NAND_TEST_ID_DAC_DMA_166M 13  /* OSPI flash test in Direct Acess Controller DMA mode at 166MHz RCLK */
#define OSPI_NAND_TEST_ID_DAC_133M_SPI 14  /* OSPI flash test in Direct Acess Controller legacy SPI mode at 133MHz RCLK */
#define OSPI_TEST_ID_PHY_CFG_MASTER    15   /* OSPI Phy Config Master mode test */
#define OSPI_TEST_ID_PHY_CFG_BYPASS    16   /* OSPI Phy Config Bypass mode test */
#define OSPI_NAND_TEST_ID_DAC_OSDR_50M     17   /* OSPI flash test in Direct Acess Controller legacy SPI mode at 50MHz RCLK */
#define OSPI_NAND_TEST_ID_DAC_OSDR_166M    18   /* OSPI flash test in Direct Acess Controller legacy SPI mode at 166MHz RCLK */
#define OSPI_NAND_TEST_ID_WR_TUNING        19   /* OSPI flash test in Direct Acess Controller legacy SPI mode to write tuning data */

/* OSPI NOR flash offset address for read/write test */
#define TEST_ADDR_OFFSET   (0U)

/* Test read/write buffer length in bytes */
#define TEST_BUF_LEN       (0x100000U)

/* Total read/write length in bytes */
#define TEST_DATA_LEN      (TEST_BUF_LEN)

/* Read/write sengment length in bytes */
#define TEST_XFER_SEG_LEN  (TEST_BUF_LEN)

/* Flash offset where the PHY tuning data pattern stored */
#define TEST_TUNE_PATTERN_OFFSET  (NOR_TUNING_DATA_OFFSET)

/* Flash selection mux */
#if defined(SOC_J721S2) || defined(SOC_J784S4)
#define OSPI_FLASH_SEL_PIN      (6U)
#define OSPI_FLASH_SEL_NOR      (0U)
#define OSPI_FLASH_SEL_NAND     (1U)
#endif

/* GTC Timer Definitions */
#define OSPI_FLASH_GTC_MOD_ID           (TISCI_DEV_GTC0)
#define OSPI_FLASH_GTC_CLK_ID           (TISCI_DEV_GTC0_GTC_CLK)
#define OSPI_FLASH_GTC_CLK_FREQ         (200000000U)

#define OSPI_FLASH_DEVICE_ID            (TISCI_DEV_MCU_FSS0_OSPI_0)
#define OSPI_FLASH_RCLK_CLK             (TISCI_DEV_MCU_FSS0_OSPI_0_OSPI_RCLK_CLK)
#define OSPI_FLASH_PARENT_CLK_166M      (TISCI_DEV_MCU_FSS0_OSPI_0_OSPI_RCLK_CLK_PARENT_HSDIV4_16FFT_MCU_2_HSDIVOUT4_CLK)
#define OSPI_FLASH_PARENT_CLK_133M      (TISCI_DEV_MCU_FSS0_OSPI_0_OSPI_RCLK_CLK_PARENT_HSDIV4_16FFT_MCU_1_HSDIVOUT4_CLK)

/* Get GTC Timer Ticks */
#define Ospi_getGTCTimerTicks()         (*((uint64_t *)(CSL_GTC0_GTC_CFG1_BASE + 0x8U)))
/* GTC ticks to Usecs */
#define OSPI_GTC_TICKS_TO_USEC(X)       ((X * 1000000)/OSPI_FLASH_GTC_CLK_FREQ)

/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/* Function to generate known data */
static void GeneratePattern(uint8_t *txBuf, uint8_t *rxBuf, uint32_t length);

/* Data compare function */
bool VerifyData(uint8_t *expData,
                uint8_t *rxData,
                uint32_t length);

#ifdef OSPI_PROFILE
static int32_t App_setGTCClk(uint32_t moduleId,
                            uint32_t clkId,
                            uint64_t clkRateHz);

int32_t OspiFlash_ClkRateSet(uint32_t modId,
                             uint32_t clkId,
                             uint64_t clkRate);
#endif

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
#if defined(OSPI_TESTAPP_RTOS)
/* Test application stack */
#define APP_TSK_STACK_MAIN              (16U * 1024U)
#if defined(SAFERTOS)
static uint8_t  gAppTskStackMain[APP_TSK_STACK_MAIN] __attribute__((aligned(APP_TSK_STACK_MAIN)));
#else
static uint8_t  gAppTskStackMain[APP_TSK_STACK_MAIN] __attribute__((aligned(32)));
#endif
#endif

/* Buffer containing the known data that needs to be written to flash */
#if defined(SOC_AM65XX) || defined(SOC_AM64X)
#ifdef SPI_DMA_ENABLE
uint8_t txBuf[TEST_BUF_LEN]  __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT))) __attribute__((section(".benchmark_buffer")));
#else
uint8_t txBuf[TEST_BUF_LEN]  __attribute__((aligned(128))) __attribute__((section(".benchmark_buffer")));
#endif
#endif

#if defined(SOC_J721E) || defined(SOC_J7200) || defined(SOC_J721S2) || defined(SOC_J784S4)
#ifdef SPI_DMA_ENABLE
uint8_t txBuf[TEST_BUF_LEN]  __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT))) __attribute__((section(".benchmark_buffer")));
#else
uint8_t txBuf[TEST_BUF_LEN]  __attribute__((aligned(128))) __attribute__((section(".benchmark_buffer")));
#endif
#endif

/* Buffer containing the received data */
#if defined(SOC_AM65XX) || defined(SOC_AM64X)
#ifdef SPI_DMA_ENABLE
uint8_t rxBuf[TEST_BUF_LEN]  __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT))) __attribute__((section(".benchmark_buffer")));
#else
uint8_t rxBuf[TEST_BUF_LEN]  __attribute__((aligned(128))) __attribute__((section(".benchmark_buffer")));
#endif
#endif

#if defined(SOC_J721E) || defined(SOC_J7200) || defined(SOC_J721S2) || defined(SOC_J784S4)
#ifdef SPI_DMA_ENABLE
uint8_t rxBuf[TEST_BUF_LEN]  __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT))) __attribute__((section(".benchmark_buffer")));
#else
uint8_t rxBuf[TEST_BUF_LEN]  __attribute__((aligned(128))) __attribute__((section(".benchmark_buffer")));
#endif
#endif

#if defined(SOC_AM65XX) || defined(SOC_J721E) || defined(SOC_J7200) || defined(SOC_AM64X) || defined(SOC_J721S2) || defined(SOC_J784S4)

#ifdef SPI_DMA_ENABLE
/*
 * Ring parameters
 */
/** \brief Number of ring entries - we can prime this much memcpy operations */
#define UDMA_TEST_APP_RING_ENTRIES      (1U)
/** \brief Size (in bytes) of each ring entry (Size of pointer - 64-bit) */
#define UDMA_TEST_APP_RING_ENTRY_SIZE   (sizeof(uint64_t))
/** \brief Total ring memory */
#define UDMA_TEST_APP_RING_MEM_SIZE     (UDMA_TEST_APP_RING_ENTRIES * \
                                         UDMA_TEST_APP_RING_ENTRY_SIZE)
/** \brief This ensures every channel memory is aligned */
#define UDMA_TEST_APP_RING_MEM_SIZE_ALIGN ((UDMA_TEST_APP_RING_MEM_SIZE + UDMA_CACHELINE_ALIGNMENT) & ~(UDMA_CACHELINE_ALIGNMENT - 1U))
/**
 *  \brief UDMA TR packet descriptor memory.
 *  This contains the CSL_UdmapCppi5TRPD + Padding to sizeof(CSL_UdmapTR15) +
 *  one Type_15 TR (CSL_UdmapTR15) + one TR response of 4 bytes.
 *  Since CSL_UdmapCppi5TRPD is less than CSL_UdmapTR15, size is just two times
 *  CSL_UdmapTR15 for alignment.
 */
#define UDMA_TEST_APP_TRPD_SIZE         ((sizeof(CSL_UdmapTR15) * 2U) + 4U)

/** \brief This ensures every channel memory is aligned */
#define UDMA_TEST_APP_TRPD_SIZE_ALIGN   ((UDMA_TEST_APP_TRPD_SIZE + UDMA_CACHELINE_ALIGNMENT) & ~(UDMA_CACHELINE_ALIGNMENT - 1U))

/*
 * UDMA driver objects
 */
struct Udma_DrvObj      gUdmaDrvObj;
struct Udma_ChObj       gUdmaChObj;
struct Udma_EventObj    gUdmaCqEventObj;

Udma_DrvHandle          gDrvHandle = NULL;
/*
 * UDMA Memories
 */
static uint8_t gTxRingMem[UDMA_TEST_APP_RING_MEM_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gTxCompRingMem[UDMA_TEST_APP_RING_MEM_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gTxTdCompRingMem[UDMA_TEST_APP_RING_MEM_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gUdmaTprdMem[UDMA_TEST_APP_TRPD_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static OSPI_dmaInfo gUdmaInfo;

static void App_print(const char *str)
{
    UART_printf("%s", str);
    return;
}

int32_t Ospi_udma_init(OSPI_v0_HwAttrs *cfg)
{
    int32_t         retVal = UDMA_SOK;
    Udma_InitPrms   initPrms;
    uint32_t        instId;

    if (gDrvHandle == NULL)
    {
        /* UDMA driver init */
#if defined (SOC_AM64X)
        /* Use Block Copy DMA instance for AM64x */
        instId = UDMA_INST_ID_BCDMA_0;
#else
        /* Use MCU NAVSS for MCU domain cores. Rest cores all uses Main NAVSS */
#if defined (BUILD_MCU1_0) || defined (BUILD_MCU1_1)
        instId = UDMA_INST_ID_MCU_0;
#else
        instId = UDMA_INST_ID_MAIN_0;
#endif
#endif

        UdmaInitPrms_init(instId, &initPrms);
        initPrms.printFxn = &App_print;
        retVal = Udma_init(&gUdmaDrvObj, &initPrms);
        if(UDMA_SOK == retVal)
        {
            gDrvHandle = &gUdmaDrvObj;
        }
    }

    if(gDrvHandle != NULL)
    {
        gUdmaInfo.drvHandle      = (void *)gDrvHandle;
        gUdmaInfo.chHandle       = (void *)&gUdmaChObj;
        gUdmaInfo.ringMem        = (void *)&gTxRingMem[0];
        gUdmaInfo.cqRingMem      = (void *)&gTxCompRingMem[0];
        gUdmaInfo.tdCqRingMem    = (void *)&gTxTdCompRingMem[0];
        gUdmaInfo.tprdMem        = (void *)&gUdmaTprdMem[0];
        gUdmaInfo.eventHandle    = (void *)&gUdmaCqEventObj;
        cfg->dmaInfo             = &gUdmaInfo;
    }

    return (retVal);
}

int32_t Ospi_udma_deinit(void)
{
    int32_t         retVal = UDMA_SOK;

    if (gDrvHandle != NULL)
    {
        retVal = Udma_deinit(gDrvHandle);
        if(UDMA_SOK == retVal)
        {
            gDrvHandle = NULL;
        }
    }

    return (retVal);
}
#endif

#if defined(OSPI_TESTAPP_RTOS)

#if defined(BUILD_MPU) || defined (BUILD_C7X)
extern void Osal_initMmuDefault(void);
void InitMmu(void)
{
    Osal_initMmuDefault();
}
#endif

#endif
#endif

#if defined(SOC_AM65XX)
/* define the unlock and lock values */
#define KICK0_UNLOCK_VAL 0x68EF3490
#define KICK1_UNLOCK_VAL 0xD172BC5A
#define KICK_LOCK_VAL    0x00000000

//
#define MAIN_MMR_BASE_ADDRESS   CSL_CTRL_MMR0_CFG0_BASE
#define MCU_MMR_BASE_ADDRESS    CSL_MCU_CTRL_MMR0_CFG0_BASE
#define WKUP_MMR_BASE_ADDRESS   CSL_WKUP_CTRL_MMR0_CFG0_BASE

#define CSL2PTR (uint32_t *)(uintptr_t)

uint32_t MMR_unlock_one(uint32_t * kick0, uint32_t * kick1)
{
    // initialize the status variable
    uint32_t status = 1;

    // if either of the kick lock registers are locked
    if (!(*kick0 & 0x1) | !(*kick1 & 0x1))
    {
        // unlock the partition by writing the unlock values to the kick lock registers
        *kick0 = KICK0_UNLOCK_VAL;
        *kick1 = KICK1_UNLOCK_VAL;
    }

    // check to see if either of the kick registers are unlocked.
    if (!(*kick0 & 0x1))
    {
        status = 0;
    }

    // return the status to the calling program
    return status;

}

uint32_t MMR_lock_one(uint32_t * kick0, uint32_t * kick1)
{
    // create status return variable
    uint32_t status = 1;

    // check to see if either of the kick registers are unlocked.
    if ((*kick0 & 0x1))
    {
        // write the kick lock value to the kick lock registers to lock the partition
        *kick0 = KICK_LOCK_VAL;
        *kick1 = KICK_LOCK_VAL;
    }

    // check to see if either of the kick registers are still unlocked.
    if ((*kick0 & 0x1))
    {
        status = 0;
    }
    // return success or failure
    return status;
}

uint32_t MCU_CTRL_MMR_unlock_all()
{

    // initialize the status variable
    uint32_t status = 1;

    // Unlock the 0th partition
    status &= MMR_unlock_one(
            CSL2PTR ( MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0),
            CSL2PTR ( MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1));

    // Unlock the 1st partition
    status &= MMR_unlock_one(
            CSL2PTR ( MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0),
            CSL2PTR ( MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1));

    // Unlock the 2nd partition
    status &= MMR_unlock_one(
            CSL2PTR ( MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0),
            CSL2PTR ( MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1));

    // Unlock the 3rd partition
    status &= MMR_unlock_one(
            CSL2PTR ( MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0),
            CSL2PTR ( MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1));

    // Unlock the 4th partition
    status &= MMR_unlock_one(
            CSL2PTR ( MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0),
            CSL2PTR ( MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1));
    return status;
}

uint32_t MCU_CTRL_MMR_lock_all()
{

    // initialize the status variable
    uint32_t status = 1;

    // lock the 0th partition
    status &= MMR_lock_one(
            CSL2PTR (MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK0),
            CSL2PTR (MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK0_KICK1));

    // lock the 1st partition
    status &= MMR_lock_one(
            CSL2PTR (MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK0),
            CSL2PTR (MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK1_KICK1));
    // lock the 2nd partition
    status &= MMR_lock_one(
            CSL2PTR (MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK0),
            CSL2PTR (MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK2_KICK1));
    // lock the 3rd partition
    status &= MMR_lock_one(
            CSL2PTR (MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK0),
            CSL2PTR (MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK3_KICK1));
    // lock the 4th partition
    status &= MMR_lock_one(
            CSL2PTR (MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK0),
            CSL2PTR (MCU_MMR_BASE_ADDRESS
                    + CSL_MCU_CTRL_MMR_CFG0_LOCK4_KICK1));
    return status;
}

void OSPI_configClk(uint32_t freq, bool usePHY)
{
	uint32_t divider = 0x12;

    MCU_CTRL_MMR_unlock_all();


    if(freq == OSPI_MODULE_CLK_166M)
    {
        /* Select CPSWHSDIV4 */
        *(uint32_t *)(CSL_MCU_CTRL_MMR0_CFG0_BASE + CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL) = 1;

        if (usePHY)
            divider = 0xC;   /* 2000/12 = 166 */
        else
            divider = 0x6;

        *(uint32_t*)(CSL_MCU_PLL0_CFG_BASE+CSL_MCU_PLL_MMR_CFG_PLL1_HSDIV_CLKDIV) = \
                (*(uint32_t*)(CSL_MCU_PLL0_CFG_BASE+CSL_MCU_PLL_MMR_CFG_PLL1_HSDIV_CLKDIV) & 0x00FFFFFF) | ((divider-1) << 24);

        /* Load new values on 0->1 transition of bit 31 of tenabldiv */
        *(uint32_t*)(CSL_MCU_PLL0_CFG_BASE+CSL_MCU_PLL_MMR_CFG_PLL1_HSDIV_CTRL) &= 0x7FFFFFFF;
        Osal_delay(1);
        *(uint32_t*)(CSL_MCU_PLL0_CFG_BASE+CSL_MCU_PLL_MMR_CFG_PLL1_HSDIV_CTRL) |= 0x80000000;
    }
    else
    {
        /* Select MCUPLL0HSDIV4 */
        *(uint32_t *)(CSL_MCU_CTRL_MMR0_CFG0_BASE + CSL_MCU_CTRL_MMR_CFG0_MCU_OSPI0_CLKSEL) = 0;

        if(freq == OSPI_MODULE_CLK_133M)
            divider = 18;    /* 2400/18 = 133 */
        if(freq == OSPI_MODULE_CLK_160M)
            divider = 15;    /* 2400/15 = 160 */
        if(freq == OSPI_MODULE_CLK_200M)
            divider = 12;    /* 2400/12 = 200 */

        *(uint32_t*)(CSL_MCU_PLL0_CFG_BASE+CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV) = \
                (*(uint32_t*)(CSL_MCU_PLL0_CFG_BASE+CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV) & 0x00FFFFFF) | ((divider-1) << 24);

        /* Load new values on 0->1 transition of bit 31 of tenabldiv */
        *(uint32_t*)(CSL_MCU_PLL0_CFG_BASE+CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL) &= 0x7FFFFFFF;
        Osal_delay(1);
        *(uint32_t*)(CSL_MCU_PLL0_CFG_BASE+CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL) |= 0x80000000;
    }
    MCU_CTRL_MMR_lock_all();

    SPI_log("\n OSPI RCLK running at %d MHz. \n", freq);
    return;

}
#endif

/**
 * \brief  OSPI mux selection function.
 *
 * This function selects the flash by enabling the GPIO pin which is used
 * for selecting NOR/NAND flash interfaces by muxer.
 *
 */
#if defined(SOC_J721S2) || defined(SOC_J784S4)

/* GPIO Driver board specific pin configuration structure */
GPIO_PinConfig gpioPinConfigs[] = {
	OSPI_FLASH_SEL_PIN  | GPIO_CFG_OUTPUT,
};

/* GPIO Driver call back functions */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,
};

/* GPIO Driver configuration structure */
GPIO_v0_Config GPIO_v0_config = {
    gpioPinConfigs,
    gpioCallbackFunctions,
    sizeof(gpioPinConfigs) / sizeof(GPIO_PinConfig),
    sizeof(gpioCallbackFunctions) / sizeof(GPIO_CallbackFxn),
    0,
};

static void OSPI_flashMux(uint32_t flashType)
{
    GPIO_v0_HwAttrs gpioCfg;
    GPIO_socGetInitCfg(0, &gpioCfg);
    gpioCfg.baseAddr = CSL_WKUP_GPIO0_BASE;
    GPIO_socSetInitCfg(0, &gpioCfg);

    GPIO_init();

    if(flashType == OSPI_FLASH_SEL_NOR)
    {
        GPIO_write(0, GPIO_PIN_LOW);
    }
    else
    {
        GPIO_write(0, GPIO_PIN_HIGH);
    }
}
#endif

#if defined(SOC_J721E) || defined(SOC_J7200) || defined(SOC_AM64X) || defined(SOC_J721S2) || defined(SOC_J784S4)
void OSPI_configClk(uint32_t freq, bool usePHY)
{
    OSPI_v0_HwAttrs ospi_cfg;
	int32_t retVal;
    uint64_t ospi_rclk_freq;
    uint32_t parClk;
#if defined (SOC_AM64X)
    uint32_t clkID[] = {
                           TISCI_DEV_FSS0_OSPI_0_OSPI_RCLK_CLK,
    };
    uint32_t devID[] = {
                           TISCI_DEV_FSS0_OSPI_0,
    };
#else
    uint32_t clkID[] = {
                           TISCI_DEV_MCU_FSS0_OSPI_0_OSPI_RCLK_CLK,
                           TISCI_DEV_MCU_FSS0_OSPI_1_OSPI_RCLK_CLK
    };
    uint32_t devID[] = {
                           TISCI_DEV_MCU_FSS0_OSPI_0,
	                       TISCI_DEV_MCU_FSS0_OSPI_1
    };
#endif

    /* Get the default SPI init configurations */
    OSPI_socGetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);

    retVal = Sciclient_pmModuleClkRequest(devID[BOARD_OSPI_NOR_INSTANCE],
                                          clkID[BOARD_OSPI_NOR_INSTANCE],
                                          TISCI_MSG_VALUE_CLOCK_SW_STATE_REQ,
                                          TISCI_MSG_FLAG_AOP,
                                          SCICLIENT_SERVICE_WAIT_FOREVER);
    if (retVal != CSL_PASS)
    {
        SPI_log("\n Sciclient_pmModuleClkRequest failed");
	    goto clk_cfg_exit;
    }

    /* Max clocks */
    if (freq == OSPI_MODULE_CLK_166M)
    {
#if defined (SOC_AM64X)
        parClk = TISCI_DEV_FSS0_OSPI_0_OSPI_RCLK_CLK_PARENT_HSDIV4_16FFT_MAIN_0_HSDIVOUT1_CLK;
#else
        parClk = TISCI_DEV_MCU_FSS0_OSPI_0_OSPI_RCLK_CLK_PARENT_HSDIV4_16FFT_MCU_2_HSDIVOUT4_CLK;
#endif

        retVal = Sciclient_pmSetModuleClkParent(devID[BOARD_OSPI_NOR_INSTANCE],
                                                clkID[BOARD_OSPI_NOR_INSTANCE],
                                                parClk,
                                                SCICLIENT_SERVICE_WAIT_FOREVER);
    }
    else
    {
#if defined (SOC_AM64X)
        parClk = TISCI_DEV_FSS0_OSPI_0_OSPI_RCLK_CLK_PARENT_HSDIV4_16FFT_MAIN_0_HSDIVOUT1_CLK;
#else
        parClk = TISCI_DEV_MCU_FSS0_OSPI_0_OSPI_RCLK_CLK_PARENT_HSDIV4_16FFT_MCU_1_HSDIVOUT4_CLK;
#endif
        retVal = Sciclient_pmSetModuleClkParent(devID[BOARD_OSPI_NOR_INSTANCE],
                                                clkID[BOARD_OSPI_NOR_INSTANCE],
                                                parClk,
                                                SCICLIENT_SERVICE_WAIT_FOREVER);
    }

    if (retVal != CSL_PASS)
    {
        SPI_log("\n Sciclient_pmSetModuleClkParent failed");
        goto clk_cfg_exit;
    }

	ospi_cfg.funcClk = freq;
    OSPI_socSetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);

    ospi_rclk_freq = (uint64_t)freq;
    retVal = Sciclient_pmSetModuleClkFreq(devID[BOARD_OSPI_NOR_INSTANCE],
                                          clkID[BOARD_OSPI_NOR_INSTANCE],
                                          ospi_rclk_freq,
                                          TISCI_MSG_FLAG_AOP,
                                          SCICLIENT_SERVICE_WAIT_FOREVER);

    if (retVal != CSL_PASS)
    {
        SPI_log("\n Sciclient_pmSetModuleClkFreq failed");
	    goto clk_cfg_exit;
    }

	ospi_rclk_freq = 0;
    retVal = Sciclient_pmGetModuleClkFreq(devID[BOARD_OSPI_NOR_INSTANCE],
                                          clkID[BOARD_OSPI_NOR_INSTANCE],
                                          &ospi_rclk_freq,
                                          SCICLIENT_SERVICE_WAIT_FOREVER);
    if (retVal != CSL_PASS)
    {
        SPI_log("\n Sciclient_pmGetModuleClkFreq failed");
	    goto clk_cfg_exit;
    }

    SPI_log("\n OSPI RCLK running at %d MHz. \n", (uint32_t)ospi_rclk_freq);

clk_cfg_exit:
      return;
}
#endif

void OSPI_initConfig(OSPI_Tests *test)
{
    OSPI_v0_HwAttrs ospi_cfg;

#if defined (BUILD_MCU)
    /* Change interrupt number based on core */
    OSPI_socInit();
#endif

    /* Get the default OSPI init configurations */
    OSPI_socGetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);

#ifdef OSPI_QSPI_FLASH
    ospi_cfg.xferLines = OSPI_XFER_LINES_QUAD;
#endif

#ifdef SPI_CACHE_ENABLE
    ospi_cfg.cacheEnable = true;
#endif

    /* Modify the default OSPI configurations */
    ospi_cfg.dacEnable = test->dacMode;
    if (test->dacMode)
    {
        ospi_cfg.dmaEnable = test->dmaMode;
        /* Enable PHY in DAC mode */
        if (test->norFlash)
        {
            ospi_cfg.phyEnable = true;
        }
        else
        {
            ospi_cfg.phyEnable = false;
        }
        ospi_cfg.intrEnable = false;
#ifdef SPI_DMA_ENABLE
        if (ospi_cfg.dmaEnable)
        {
            Ospi_udma_init(&ospi_cfg);
        }
#endif
    }
    else
    {
        /* Enable interrupt in INDAC mode */
        ospi_cfg.intrEnable = true;
        /* Disable PHY in INDAC mode */
        ospi_cfg.phyEnable = false;
        ospi_cfg.dmaEnable = false;
    }

    if ((test->testId == OSPI_TEST_ID_DAC_133M_SPI) || (test->testId == OSPI_NAND_TEST_ID_DAC_133M_SPI))
    {
        /* Disable PHY in legacy SPI mode (1-1-1) */
        ospi_cfg.phyEnable = false;
        ospi_cfg.dtrEnable = false;
        ospi_cfg.xferLines = OSPI_XFER_LINES_SINGLE;
    }
    else
    {
        ospi_cfg.dtrEnable = true;
        ospi_cfg.xferLines = OSPI_XFER_LINES_OCTAL;
    }

    if (test->testId == OSPI_TEST_ID_WR_TUNING)
    {
        ospi_cfg.phyEnable = false;
        ospi_cfg.cacheEnable = false;
    }

    if (test->testId == OSPI_NAND_TEST_ID_WR_TUNING)
    {
        ospi_cfg.phyEnable = false;
    }

    ospi_cfg.funcClk = test->clk;

    if(test->norFlash)
    {
        if (ospi_cfg.funcClk == OSPI_MODULE_CLK_133M)
        {
            ospi_cfg.devDelays[3] = OSPI_DEV_DELAY_CSDA;
        }
        else
        {
            ospi_cfg.devDelays[3] = OSPI_DEV_DELAY_CSDA_3;
        }
    }
    else
    {
        ospi_cfg.devDelays[0] = OSPI_DEV_DELAY_CSDA_A;
        ospi_cfg.devDelays[1] = OSPI_DEV_DELAY_CSDA_A;
        ospi_cfg.devDelays[2] = OSPI_DEV_DELAY_CSDA_A;
        ospi_cfg.devDelays[3] = OSPI_DEV_DELAY_CSDA_A;
        ospi_cfg.blkSize = 16;
        ospi_cfg.baudRateDiv = 8;
        ospi_cfg.rdDataCapDelay = 0U;
    }
    if (test->testId == OSPI_NAND_TEST_ID_DAC_OSDR_50M)
    {
        ospi_cfg.phyEnable = false;
        ospi_cfg.baudRateDiv = 4U;
        ospi_cfg.dtrEnable = false;
        ospi_cfg.xferLines = OSPI_XFER_LINES_OCTAL;
    }

    if (test->testId == OSPI_NAND_TEST_ID_DAC_OSDR_166M)
    {
        ospi_cfg.phyEnable = true;
        ospi_cfg.baudRateDiv = 0U;
        ospi_cfg.dtrEnable = false;
        ospi_cfg.xferLines = OSPI_XFER_LINES_OCTAL;
        ospi_cfg.devDelays[0] = 0x01U;
        ospi_cfg.devDelays[1] = 0x00U;
        ospi_cfg.devDelays[2] = 0x00U;
        ospi_cfg.devDelays[3] = 0x00U;
    }

    if(ospi_cfg.intrEnable)
    {
        /* Set interrupt path */
        if(OSPI_configSocIntrPath(&ospi_cfg, TRUE) != CSL_PASS)
        {
            SPI_log("\n Set interrupt path failed!\n");
        }
        else
        {
            SPI_log("\n The interrupt path has been set with interrupt number %d", ospi_cfg.intrNum);
        }
    }

    /* Set the default OSPI init configurations */
    OSPI_socSetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);
}

void OSPI_deInitConfig(OSPI_Tests *test)
{
    OSPI_v0_HwAttrs ospi_cfg;

    /* Get the default OSPI init configurations */
    OSPI_socGetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);

    /* Release interrupt path */
    OSPI_configSocIntrPath(&ospi_cfg, FALSE);
}

/*
 *  ======== OSPI unit test function ========
 */

static bool OSPI_phyConfigTest(void *arg)
{
    OSPI_Params     spiParams;  /* SPI params structure */
    OSPI_Handle     hwHandle;  /* SPI handle */
    OSPI_v0_HwAttrs ospiCfg;
    bool            retVal = true;
    int32_t         status = SPI_STATUS_SUCCESS;
    OSPI_Tests      *test = (OSPI_Tests *)arg;
    uint32_t        data[3];
    uint32_t        phyOpMode;

#if defined (BUILD_MCU)
    /* Change interrupt number based on core */
    status = OSPI_socInit();
    if(status != SPI_STATUS_SUCCESS)
    {
        SPI_log("\nOSPI_socInit failed!!\n");
        retVal = false;
    }
    else
#endif
    {
        /* Get the OSPI SoC configurations */
        status = OSPI_socGetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_NOR_INSTANCE, &ospiCfg);
        if(status != SPI_STATUS_SUCCESS)
        {
            SPI_log("\nOSPI_socGetInitCfg failed!!\n");
            retVal = false;
        }
        else
        {
            ospiCfg.phyOpMode = test->phyOpMode;

            status = OSPI_socSetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_NOR_INSTANCE, &ospiCfg);
            if(status != SPI_STATUS_SUCCESS)
            {
                SPI_log("\nOSPI_socGetInitCfg failed!!\n");
                retVal = false;
            }
            else
            {
                /* Use default SPI config params if no params provided */
                OSPI_Params_init(&spiParams);
                hwHandle = (OSPI_Handle)OSPI_open(BOARD_OSPI_DOMAIN, BOARD_OSPI_NOR_INSTANCE, &spiParams);

                if(hwHandle == NULL)
                {
                    SPI_log("\nOSPI_open failed!!\n");
                    retVal = false;
                }
                else
                {
                    OSPI_v0_HwAttrs const        *hwAttrs= (OSPI_v0_HwAttrs const *)hwHandle->hwAttrs;
                    const CSL_ospi_flash_cfgRegs *pRegs = (const CSL_ospi_flash_cfgRegs *)(hwAttrs->baseAddr);

                    data[0] = TRUE;
                    data[1] = 0U;
                    data[2] = 0U;

                    status = OSPI_control(hwHandle, OSPI_V0_CMD_CFG_PHY, (void *)data);

                    if(status != SPI_STATUS_SUCCESS)
                    {
                        SPI_log("\nOSPI_control failed!!\n");
                        retVal = false;
                    }
                    else
                    {
                        phyOpMode = CSL_REG32_FEXT(&pRegs->PHY_MASTER_CONTROL_REG,
                                        OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_BYPASS_MODE_FLD);

                        if(test->phyOpMode == CSL_OSPI_CFG_PHY_OP_MODE_MASTER)
                        {
                            if(phyOpMode != 0U)
                            {
                                SPI_log("\nFAIL: PHY not configured in master mode\n");
                                retVal = false;
                            }
                            else
                            {
                                SPI_log("\nPHY configured in master mode\n");
                                retVal = true;
                            }
                        }
                        if(test->phyOpMode == CSL_OSPI_CFG_PHY_OP_MODE_BYPASS)
                        {
                            if(phyOpMode != 1U)
                            {
                                SPI_log("\nFAIL: PHY not configured in bypass mode\n");
                                retVal = false;
                            }
                            else
                            {
                                SPI_log("\nPHY configured in bypass mode\n");
                                retVal = true;
                            }
                        }
                    }
                    OSPI_close(hwHandle);
                }
            }
        }
    }
    return retVal;
}
static bool OSPI_flash_test(void *arg)
{
    Board_flashHandle boardHandle;
    Board_FlashInfo  *flashInfo;
#ifdef OSPI_WRITE
    uint32_t          blockNum;     /* flash block number */
    uint32_t          pageNum;      /* flash page number */
#endif
    bool              testPassed = true;  /* return value */
    uint32_t          writeMode = OSPI_FLASH_OCTAL_PAGE_PROG;
    uint32_t          readMode = OSPI_FLASH_OCTAL_READ;
    uint32_t          deviceId;     /* flash device ID */
    OSPI_Tests       *test = (OSPI_Tests *)arg;
    uint32_t          i;
    uint32_t          offset;
    uint32_t          xferLen;
    uint32_t          testLen = TEST_DATA_LEN;
    uint32_t          testSegLen = TEST_XFER_SEG_LEN;
#ifdef OSPI_PROFILE
    uint64_t          startTick; /* start GTC tick */
    uint64_t          numTicks; /* numbet of GTC ticks */
    float             xferRate;
    uint32_t          xferRateInt;
#if defined(FREERTOS)
    uint32_t          cpuLoad;
#endif
#endif
    uint32_t          startOffset;
    uint8_t          *pBuf;
    uint32_t          tuneEnable;
    uint32_t          blockSize;

    if ((test->testId == OSPI_TEST_ID_WR_TUNING) || (test->testId == OSPI_NAND_TEST_ID_WR_TUNING))
    {
        testLen = NOR_ATTACK_VECTOR_SIZE;
#if defined(SOC_J721S2) || defined(SOC_J784S4)
        if(test->norFlash == true)
        {
            startOffset = NOR_TUNING_DATA_OFFSET;
        }
        else
        {
            startOffset = NAND_TUNING_DATA_OFFSET;
        }
#else
        startOffset = NOR_TUNING_DATA_OFFSET;
#endif
    }
    else
    {
        testLen = TEST_DATA_LEN;
        startOffset = TEST_ADDR_OFFSET;
    }

    if(test->dacMode)
    {
        tuneEnable = TRUE;
    }
    else
    {
        tuneEnable = FALSE;
    }

    OSPI_initConfig(test);
    OSPI_v0_HwAttrs ospi_cfg;

    OSPI_socGetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);

#if defined(SOC_J7200) || defined(SOC_AM64X)
    deviceId = BOARD_FLASH_ID_S28HS512T;
#elif defined(SOC_J721S2) || defined(SOC_J784S4)
    if(test->norFlash)
    {
        deviceId = BOARD_FLASH_ID_S28HS512T;
        OSPI_flashMux(OSPI_FLASH_SEL_NOR);
    }
    else
    {
        deviceId = BOARD_FLASH_ID_W35N01JWTBAG;
        OSPI_flashMux(OSPI_FLASH_SEL_NAND);
    }
#else
    deviceId = BOARD_FLASH_ID_MT35XU512ABA1G12;
#endif

#if defined(SOC_J721S2) || defined(SOC_J784S4)
    if(test->norFlash)
    {
        blockSize = NOR_BLOCK_SIZE;
    }
    else
    {
        blockSize = NAND_BLOCK_SIZE;
    }
#else
    blockSize = NOR_BLOCK_SIZE;
#endif

#ifdef OSPI_PROFILE
    /* Get start time stamp for the Board_flashOpen measurement */
    startTick = Ospi_getGTCTimerTicks();
#endif
    /* Open the Board OSPI NOR device with OSPI port 0
       and use default OSPI configurations */
    boardHandle = Board_flashOpen(deviceId,
                                  BOARD_OSPI_NOR_INSTANCE, (void *)(&tuneEnable));

#ifdef OSPI_PROFILE
    /* Get elapsed time for the Board_flashOpen measurement */
    numTicks = Ospi_getGTCTimerTicks() - startTick;
    SPI_log("\n Board_flashOpen elpased time %d us \n", (uint32_t)OSPI_GTC_TICKS_TO_USEC(numTicks));
#endif
    if (!boardHandle)
    {
        SPI_log("\n Board_flashOpen failed. \n");
        testPassed = false;
        goto err;
    }
    else
    {
        flashInfo = (Board_FlashInfo *)boardHandle;
        SPI_log("\n OSPI NOR device ID: 0x%x, manufacturer ID: 0x%x \n",
                flashInfo->device_id, flashInfo->manufacturer_id);
    }

    /* Generate the data */
    GeneratePattern(txBuf, rxBuf, testLen);
    if ((test->testId == OSPI_TEST_ID_WR_TUNING) || (test->testId == OSPI_NAND_TEST_ID_WR_TUNING))
    {
        pBuf = nor_attack_vector;
    }
    else
    {
        pBuf = txBuf;
    }
#ifdef SPI_DMA_ENABLE
    if (test->dmaMode)
    {
        CacheP_wbInv((void *)pBuf, (int32_t)testLen);
        CacheP_wbInv((void *)rxBuf, (int32_t)testLen);
    }
#endif

#ifdef OSPI_WRITE
    if((test->testId != OSPI_NAND_TEST_ID_DAC_OSDR_50M) && (test->testId != OSPI_NAND_TEST_ID_DAC_OSDR_166M))
    {
    #if defined(SOC_J7200) || defined(SOC_AM64X) || defined(SOC_J721S2) || defined(SOC_J784S4)
        if (test->norFlash && test->dacMode)                          /* DAC writes are not supported on Cypress xSPI Flash - Switch to INDAC mode for write as WA to PDK-7115 */
        {
            if(test->norFlash)
            {
                SPI_log("\n The Cypress xSPI Flash does not support DAC writes, switching to INDAC mode. \n");
            }
            else
            {
                SPI_log("\n The Winbond NAND Flash does not support DAC writes, switching to INDAC mode. \n");
            }
            Board_flashClose(boardHandle);
        #if SPI_DMA_ENABLE
            if (test->dmaMode)
            {
                Ospi_udma_deinit();
            }
        #endif
            OSPI_socGetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);
            ospi_cfg.dacEnable = false;
            ospi_cfg.phyEnable = false;
            ospi_cfg.dmaEnable = false;
            ospi_cfg.intrEnable = true;
            OSPI_socSetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);
            boardHandle = Board_flashOpen(deviceId, BOARD_OSPI_NOR_INSTANCE, (void *)(&tuneEnable));
        }
    #endif
        for (i = 0; i < testLen; i += blockSize)
        {
            offset = startOffset + i;
            if (Board_flashOffsetToBlkPage(boardHandle, offset,
                                        &blockNum, &pageNum))
            {
                SPI_log("\n Board_flashOffsetToBlkPage failed. \n");
                testPassed = false;
                goto err;
            }

            /* Erase block, to which data has to be written */
            if (Board_flashEraseBlk(boardHandle, blockNum))
            {
                SPI_log("\n Board_flashEraseBlk failed. \n");
                testPassed = false;
                goto err;
            }
        }
    #ifdef OSPI_PROFILE
    #if defined(FREERTOS)
        LoadP_reset( );
    #endif
        /* Get start time stamp for the write performance measurement */
        startTick = Ospi_getGTCTimerTicks();
    #endif
        for (i = 0; i < testLen; i += testSegLen)
        {
            offset = startOffset + i;
            if ((testLen - i) < testSegLen)
            {
                xferLen = testLen - i;
            }
            else
            {
                xferLen = testSegLen;
            }
            /* Write buffer to flash */
            if (Board_flashWrite(boardHandle, offset, &pBuf[i],
                                xferLen, (void *)(&writeMode)))
            {
                SPI_log("\n Board_flashWrite failed. \n");
                testPassed = false;
                goto err;
            }
        }

    #ifdef OSPI_PROFILE
        numTicks = Ospi_getGTCTimerTicks() - startTick;
    #if defined(FREERTOS)
        cpuLoad = LoadP_getCPULoad();
    #endif
        /* calculate the write transfer rate in KBps */
        xferRate = (float) (((float) (testLen)) / OSPI_GTC_TICKS_TO_USEC(numTicks)) * 1000;
        xferRateInt = (uint32_t)xferRate;
        SPI_log("\n Board_flashWrite %d bytes at transfer rate %u KBps \n", testLen, xferRateInt);
    #if defined(FREERTOS)
        SPI_log("\n Board_flashWrite CPU load %d%% \n", cpuLoad);
    #endif
    #endif
    }
#endif /* OSPI_WRITE */

#if defined(SOC_J7200) || defined(SOC_AM64X) || defined(SOC_J721S2) || defined(SOC_J784S4)
    if (test->norFlash && test->dacMode)                          /* DAC reads are supported - switching back to DAC mode */
    {
        if(test->norFlash)
        {
            SPI_log("\n The Cypress xSPI Flash supports DAC reads, switching back to DAC mode. \n");
        }
        else
        {
            SPI_log("\n The Winbond NAND Flash supports DAC reads, switching back to DAC mode. \n");
        }
        OSPI_v0_HwAttrs ospi_cfg;
        Board_flashClose(boardHandle);
        OSPI_socGetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);
        if (test->testId == OSPI_TEST_ID_DAC_133M_SPI)
        {
            /* Disable PHY in legacy SPI mode (1-1-1) */
            ospi_cfg.phyEnable = false;
            ospi_cfg.dtrEnable = false;
            ospi_cfg.xferLines = OSPI_XFER_LINES_SINGLE;
        }
        else if (test->testId == OSPI_NAND_TEST_ID_DAC_OSDR_50M)
        {
            /* Disable PHY in legacy SPI mode (1-1-8) */
            ospi_cfg.phyEnable = false;
            ospi_cfg.baudRateDiv = 4U;
            ospi_cfg.dtrEnable = false;
            ospi_cfg.xferLines = OSPI_XFER_LINES_OCTAL;
        }
        else if (test->testId == OSPI_NAND_TEST_ID_DAC_OSDR_166M)
        {
            ospi_cfg.phyEnable = true;
            ospi_cfg.baudRateDiv = 4U;
            ospi_cfg.dtrEnable = false;
            ospi_cfg.xferLines = OSPI_XFER_LINES_OCTAL;
        }
        else
        {
            /* Enable PHY in octal SPI mode (8-8-8) */
            ospi_cfg.phyEnable = true;
            ospi_cfg.dtrEnable = true;
            ospi_cfg.xferLines = OSPI_XFER_LINES_OCTAL;
        }

        ospi_cfg.dacEnable = true;
        ospi_cfg.dmaEnable = test->dmaMode;
        ospi_cfg.intrEnable = false;
    #ifdef SPI_DMA_ENABLE
        if (ospi_cfg.dmaEnable)
        {
            Ospi_udma_init(&ospi_cfg);
        }
    #endif
        OSPI_socSetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);
        boardHandle = Board_flashOpen(deviceId, BOARD_OSPI_NOR_INSTANCE, (void *)(&tuneEnable));
    }
#endif
#ifdef OSPI_PROFILE
    if((test->dacMode) && (test->testId!=OSPI_NAND_TEST_ID_DAC_OSDR_50M) &&\
       (test->testId!=OSPI_TEST_ID_DAC_133M_SPI) && (test->testId!=OSPI_TEST_ID_WR_TUNING) &&\
       (test->testId!=OSPI_NAND_TEST_ID_DAC_133M_SPI) && (test->testId!=OSPI_NAND_TEST_ID_WR_TUNING))
    {
        uint64_t    startPhyTuningTick;
        uint64_t    elapsedPhyTuningTicks;
        uint8_t     tmpRxBuf[4U];

        /* Dummy flash read to get just the PHY tuning time */
        startPhyTuningTick = Ospi_getGTCTimerTicks();
        if (Board_flashRead(boardHandle, 0U, &tmpRxBuf[0], 4U, (void *)(&readMode)))
        {
            SPI_log("\n Dummy Board_flashRead for PHY tuning measurement failed. \n");
            testPassed = false;
            goto err;
        }
        elapsedPhyTuningTicks = Ospi_getGTCTimerTicks() - startPhyTuningTick;
        SPI_log("\n PHY tuning + 4 byte read took %d us \n", (uint32_t)OSPI_GTC_TICKS_TO_USEC(elapsedPhyTuningTicks));
    }
#if defined(FREERTOS)
    LoadP_reset( );
#endif
    /* Get start time stamp for the read performance measurement */
    startTick = Ospi_getGTCTimerTicks();
#endif
    for (i = 0; i < testLen; i += testSegLen)
    {
        offset = startOffset + i;
        if ((testLen - i) < testSegLen)
        {
            xferLen = testLen - i;
        }
        else
        {
            xferLen = testSegLen;
        }
        if (Board_flashRead(boardHandle, offset, &rxBuf[i],
                            xferLen, (void *)(&readMode)))
        {
            SPI_log("\n Board_flashRead failed. \n");
            testPassed = false;
            goto err;
        }
    }

#ifdef OSPI_PROFILE
    numTicks = Ospi_getGTCTimerTicks() - startTick;
#if defined(FREERTOS)
    cpuLoad = LoadP_getCPULoad();
#endif
    /* calculate the write transfer rate in KBps */
    xferRate = (float) (((float) (testLen)) / OSPI_GTC_TICKS_TO_USEC(numTicks)) * 1000;
    xferRateInt = (uint32_t)xferRate;
    SPI_log("\n Board_flashRead %d bytes in %d Usecs at transfer rate %u KBps \n", testLen, (int)OSPI_GTC_TICKS_TO_USEC(numTicks), xferRateInt);
#if defined(FREERTOS)
    SPI_log("\n Board_flashRead CPU load %d%% \n", cpuLoad);
#endif
#endif

#ifdef OSPI_WRITE
    /* Verify Data */
    if (VerifyData(pBuf, rxBuf, testLen) == false)
    {
        SPI_log("\n Data mismatch. \n");
        testPassed = false;
        goto err;
    }
#endif

err:
    OSPI_deInitConfig(test);

    if (boardHandle != 0U)
    {
        Board_flashClose(boardHandle);
    }

#if SPI_DMA_ENABLE
    if (test->dmaMode)
    {
        Ospi_udma_deinit();
    }
#endif

    return (testPassed);
}

void OSPI_test_print_test_desc(OSPI_Tests *test)
{
    char        testId[16] = {0, };

    /* Print unit test ID */
    sprintf(testId, "%d", test->testId);
    SPI_log("\r\n OSPI UT %s\r\n", testId);

    /* Print test description */
    SPI_log("\r\n %s\r\n", test->testDesc);
}

OSPI_Tests Ospi_tests[] =
{
#ifdef OSPI_WRITE_TUNING
    {OSPI_flash_test,       OSPI_TEST_ID_WR_TUNING,           true,   false,  true,       CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_133M, "\r\n OSPI flash test slave to write tuning data to flash"},
#endif
    {OSPI_flash_test,       OSPI_TEST_ID_DAC_133M,            true,   false,  true,       CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_133M, "\r\n OSPI flash test slave in DAC mode at 133MHz RCLK"},
    {OSPI_flash_test,       OSPI_TEST_ID_INDAC_133M,          false,  false,  true,       CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_133M, "\r\n OSPI flash test slave in INDAC mode at 133MHz RCLK"},
#ifdef SPI_DMA_ENABLE
    {OSPI_flash_test,       OSPI_TEST_ID_DAC_DMA_133M,        true,   true,   true,       CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_133M, "\r\n OSPI flash test slave in DAC DMA mode at 133MHz RCLK"},
#endif
    {OSPI_flash_test,       OSPI_TEST_ID_DAC_166M,            true,   false,  true,       CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_166M, "\r\n OSPI flash test slave in DAC mode at 166MHz RCLK"},
    {OSPI_flash_test,       OSPI_TEST_ID_INDAC_166M,          false,  false,  true,       CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_166M, "\r\n OSPI flash test slave in INDAC mode at 166MHz RCLK"},
#ifdef SPI_DMA_ENABLE
    {OSPI_flash_test,       OSPI_TEST_ID_DAC_DMA_166M,        true,   true,   true,       CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_166M, "\r\n OSPI flash test slave in DAC DMA mode at 166MHz RCLK"},
#endif
    {OSPI_flash_test,       OSPI_TEST_ID_DAC_133M_SPI,        true,   false,  true,       CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_133M, "\r\n OSPI flash test slave in DAC Legacy SPI mode at 133MHz RCLK"},
#if defined(SOC_J721S2) || defined(SOC_J784S4)
    {OSPI_flash_test,       OSPI_NAND_TEST_ID_WR_TUNING,      true,  false,  false,      CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_133M, "\r\n OSPI flash test slave to write tuning data to nand flash"},
    {OSPI_flash_test,       OSPI_NAND_TEST_ID_INDAC_166M,     false,  false,  false,      CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_166M, "\r\n OSPI NAND flash test slave in INDAC mode at 166MHz RCLK"},
    {OSPI_flash_test,       OSPI_NAND_TEST_ID_DAC_133M_SPI,   true,   false,  false,      CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_133M, "\r\n OSPI NAND flash test slave in DAC Legacy SPI mode at 133MHz RCLK"},
    {OSPI_flash_test,       OSPI_NAND_TEST_ID_DAC_OSDR_50M,  true,   false,  false,       CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_200M, "\r\n OSPI flash test slave in DAC SDR OSPI mode at 50MHz RCLK"},
#ifdef SPI_DMA_ENABLE
    {OSPI_flash_test,       OSPI_NAND_TEST_ID_DAC_OSDR_166M,  true,   true,  false,       CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_166M, "\r\n OSPI flash test slave in DAC SDR OSPI mode at 166MHz RCLK"},
#endif
    {OSPI_flash_test,       OSPI_NAND_TEST_ID_INDAC_133M,     false,  false,  false,      CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_133M, "\r\n OSPI NAND flash test slave in INDAC mode at 133MHz RCLK"},
    {OSPI_flash_test,       OSPI_NAND_TEST_ID_DAC_133M,       true,   false,  false,      CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_133M, "\r\n OSPI NAND flash test slave in DAC mode at 133MHz RCLK"},
#ifdef SPI_DMA_ENABLE
    {OSPI_flash_test,       OSPI_NAND_TEST_ID_DAC_DMA_133M,   true,   true,   false,      CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_133M, "\r\n OSPI NAND flash test slave in DAC DMA mode at 133MHz RCLK"},
#endif
    {OSPI_flash_test,       OSPI_NAND_TEST_ID_DAC_166M,       true,   false,  false,      CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_166M, "\r\n OSPI NAND flash test slave in DAC mode at 166MHz RCLK"},
#ifdef SPI_DMA_ENABLE
    {OSPI_flash_test,       OSPI_NAND_TEST_ID_DAC_DMA_166M,   true,   true,   false,      CSL_OSPI_CFG_PHY_OP_MODE_DEFAULT,   OSPI_MODULE_CLK_166M, "\r\n OSPI NAND flash test slave in DAC DMA mode at 166MHz RCLK"},
#endif
#endif
    {OSPI_phyConfigTest,    OSPI_TEST_ID_PHY_CFG_MASTER,      true,   false,  true,       CSL_OSPI_CFG_PHY_OP_MODE_MASTER,   OSPI_MODULE_CLK_133M, "\r\n OSPI Phy Config Master mode test"},
    {OSPI_phyConfigTest,    OSPI_TEST_ID_PHY_CFG_BYPASS,      true,   false,  true,       CSL_OSPI_CFG_PHY_OP_MODE_BYPASS,   OSPI_MODULE_CLK_133M, "\r\n OSPI Phy Config bypass mode test"},
    {NULL, }
};

/*
 *  ======== test function ========
 */
#if defined(OSPI_TESTAPP_RTOS)
void spi_test(void* arg0, void* arg1)
#else
void spi_test()
#endif
{
    uint32_t    i;
    bool        testFail = false;
    OSPI_Tests *test;

#if defined(SAFERTOS)
    /* Call board init functions */
    Board_initCfg boardCfg;
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
        BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;

    Board_init(boardCfg);
#endif

    /* Init OSPI driver */
    OSPI_init();

    for (i = 0; ; i++)
    {
        test = &Ospi_tests[i];
        if (test->testFunc == NULL)
        {
            break;
        }

        OSPI_configClk(test->clk, true);

        OSPI_test_print_test_desc(test);

        if (test->testFunc((void *)test) == true)
        {
            SPI_log("\r\n %s have passed\r\n", test->testDesc);
        }
        else
        {
            SPI_log("\r\n %s have failed\r\n", test->testDesc);
            testFail = true;
        }
    }

    if(testFail == true)
    {
        SPI_log("\n Some tests have failed. \n");
    }
    else
    {
        SPI_log("\n All tests have passed. \n");
    }

    SPI_log("Done\n");

    while (true)
    {
    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
#if !defined(SAFERTOS)
    Board_initCfg boardCfg;
#endif
#if defined(OSPI_TESTAPP_RTOS)
    TaskP_Handle task;
    TaskP_Params taskParams;
#endif
#if !defined(SAFERTOS)
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
        BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#if defined (SOC_J7200)
    /* Need to do PLL config through board init for proper clock input on J7200 */
    boardCfg |= BOARD_INIT_PLL;
#endif
    Board_init(boardCfg);
#endif

#ifdef OSPI_PROFILE
    /* Configure GTC Timer for profiling */
    App_setGTCClk(OSPI_FLASH_GTC_MOD_ID,
                           OSPI_FLASH_GTC_CLK_ID,
                           OSPI_FLASH_GTC_CLK_FREQ);
#endif

#if defined(OSPI_TESTAPP_RTOS)

	OS_init();
    /* Initialize the task params */
    TaskP_Params_init(&taskParams);
    /* Set the task priority higher than the default priority (1) */
    taskParams.priority     = 2;
    taskParams.stack        = gAppTskStackMain;
    taskParams.stacksize    = sizeof (gAppTskStackMain);

    /* Start BIOS */
	task = TaskP_create(&spi_test, &taskParams);
    if (task == NULL) {
        UART_printf("TaskP_create() failed!\n");
        OS_stop();
    }
    OS_start();
#else
    spi_test();
#endif
    return (0);
}

/*
 *  ======== CompareData ========
 */
bool VerifyData(uint8_t *expData,
                uint8_t *rxData,
                uint32_t length)
{
    uint32_t idx = 0;
    uint32_t match = 1;
    bool retVal = false;

    for(idx = 0; ((idx < length) && (match != 0)); idx++)
    {
        if(*expData != *rxData)
        {
            match = 0;
            SPI_log("Data mismatch at idx %d\n", idx);
            SPI_log("exp data:%x, rxData:%x\n", *expData, *rxData);
        }
        expData++;
        rxData++;
    }

    if(match == 1)
    {
        retVal = true;
    }

    return retVal;
}

/*
 *  ======== GeneratePattern ========
 */
static void GeneratePattern(uint8_t *txBuf, uint8_t *rxBuf, uint32_t length)
{
    volatile uint32_t idx;
    volatile uint8_t *txPtr = txBuf;
    volatile uint8_t *rxPtr = rxBuf;

    for(idx = 0; idx < length; idx++)
    {
        if (idx < (length/2))
        {
            *txPtr++ = (uint8_t)idx;
        }
        else if (idx < (length/4*3))
        {
            *txPtr++ = 0xaa;
        }
        else
        {
            *txPtr++ = 0x55;
        }
    	*rxPtr++ = (uint8_t)0U;
    }
}

#ifdef OSPI_PROFILE
static int32_t App_setGTCClk(uint32_t moduleId,
                      uint32_t clkId,
                      uint64_t clkRateHz)
{
    int32_t retVal;
    uint64_t currClkFreqHz;

    retVal = Sciclient_pmGetModuleClkFreq(moduleId,
                                          clkId,
                                          &currClkFreqHz,
                                          SCICLIENT_SERVICE_WAIT_FOREVER);
    if ((retVal == CSL_PASS) &&
        (currClkFreqHz != clkRateHz))
    {
        retVal = OspiFlash_ClkRateSet(moduleId, clkId, clkRateHz);
    }

    /* Enable GTC */
    HW_WR_REG32(CSL_GTC0_GTC_CFG1_BASE + 0x0U, 0x1);

    return (retVal);
}

int32_t OspiFlash_ClkRateSet(uint32_t modId,
                             uint32_t clkId,
                             uint64_t clkRate)
{
    uint32_t i                          = 0U;
    int32_t status                      = SPI_STATUS_SUCCESS;
    int32_t finalStatus                 = SPI_STATUS_SUCCESS;
    uint64_t respClkRate                = 0;
    uint32_t numParents                 = 0U;
    uint32_t moduleClockParentChanged   = 0U;
    uint32_t clockStatus                = 0U;
    uint32_t origParent                 = 0U;
    uint32_t foundParent                = 0U;

    /* Check if the clock is enabled or not */
    status = Sciclient_pmModuleGetClkStatus(modId,
                                            clkId,
                                            &clockStatus,
                                            SCICLIENT_SERVICE_WAIT_FOREVER);
    if (status == CSL_PASS)
    {
        /* Get the number of parents for the clock */
        status = Sciclient_pmGetModuleClkNumParent(modId,
                                                   clkId,
                                                   &numParents,
                                                   SCICLIENT_SERVICE_WAIT_FOREVER);
        if ((status == CSL_PASS) && (numParents > 1U))
        {
            status = Sciclient_pmGetModuleClkParent(modId,
                                                    clkId,
                                                    &origParent,
                                                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }
    }
    if (status == CSL_PASS)
    {
        /* Disabling the clock */
        status = Sciclient_pmModuleClkRequest(modId,
                                              clkId,
                                              TISCI_MSG_VALUE_CLOCK_SW_STATE_UNREQ,
                                              0U,
                                              SCICLIENT_SERVICE_WAIT_FOREVER);
    }
    if (status == CSL_PASS)
    {
        foundParent = 0U;
        /* Try to loop and change parents of the clock */
        for(i=0U;i<numParents;i++)
        {
            if (numParents > 1U)
            {
                /* Setting the new parent */
                status = Sciclient_pmSetModuleClkParent(modId,
                                                        clkId,
                                                        clkId+i+1U,
                                                        SCICLIENT_SERVICE_WAIT_FOREVER);
                /* Check if the clock can be set to desirable freq. */
                if (status == CSL_PASS)
                {
                    moduleClockParentChanged = 1U;
                }
            }
            if (status == CSL_PASS)
            {
                status = Sciclient_pmQueryModuleClkFreq(modId,
                                                        clkId,
                                                        clkRate,
                                                        &respClkRate,
                                                        SCICLIENT_SERVICE_WAIT_FOREVER);
            }
            if ((status == CSL_PASS) && (respClkRate == clkRate))
            {
                foundParent = 1U;
                break;
            }
        }
    }
    if ((status == CSL_PASS) && (numParents == 0U))
    {
        status = Sciclient_pmQueryModuleClkFreq(modId,
                                                clkId,
                                                clkRate,
                                                &respClkRate,
                                                SCICLIENT_SERVICE_WAIT_FOREVER);
        if ((status == CSL_PASS) && (respClkRate == clkRate))
        {
            foundParent = 1U;
        }
    }
    if (foundParent == 1U)
    {
        /* Set the clock at the desirable frequency*/
        status = Sciclient_pmSetModuleClkFreq(modId,
                                              clkId,
                                              clkRate,
                                              TISCI_MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE,
                                              SCICLIENT_SERVICE_WAIT_FOREVER);
    }
    else
    {
        status = CSL_EFAIL;
    }
    if ((status == CSL_PASS) &&
        (clockStatus == (uint32_t) TISCI_MSG_VALUE_CLOCK_SW_STATE_UNREQ))
    {
        /* Restore the clock again to original state */
        status = Sciclient_pmModuleClkRequest(modId,
                                              clkId,
                                              clockStatus,
                                              0U,
                                              SCICLIENT_SERVICE_WAIT_FOREVER);
    }
    finalStatus = status;
    if ((status != CSL_PASS) && (moduleClockParentChanged == 1U))
    {
        /* Setting the original parent if failure */
        (void) Sciclient_pmSetModuleClkParent(modId,
                                              clkId,
                                              origParent,
                                              SCICLIENT_SERVICE_WAIT_FOREVER);
    }

    return (finalStatus);
}
#endif
