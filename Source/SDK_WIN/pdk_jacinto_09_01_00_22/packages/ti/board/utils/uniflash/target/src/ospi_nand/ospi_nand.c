/*
 * Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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
 *  \file    ospi_nand.c
 *
 *  \brief   ospi NAND flash writer API implementation.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "ospi_nand.h"

#ifdef SPI_DMA_ENABLE
#include <ti/osal/CacheP.h>
#endif

/* ========================================================================== */
/*                      Internal Function Declarations                        */
/* ========================================================================== */

static int8_t UFP_ospiNandInit(void);

static int8_t UFP_ospiNandFlashErase(uint32_t offset, uint32_t length);

static int8_t UFP_ospiNandFlashImage(uint8_t *flashAddr, uint8_t *checkAddr,
                                     uint32_t offset, uint32_t size);

static int8_t UFP_ospiNandClose(void);

#ifdef SPI_DMA_ENABLE
/*
 * UDMA Memories
 */
static uint8_t gTxRingMem[UDMA_TEST_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gTxCompRingMem[UDMA_TEST_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gTxTdCompRingMem[UDMA_TEST_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gUdmaTprdMem[UDMA_TEST_APP_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

Board_flashHandle gOspiHandle;

/* Flash programmer ospi function table */
const UFP_fxnTable UFP_ospiNandFxnTable = {
    &UFP_ospiNandInit,
    &UFP_ospiNandFlashErase,
    &UFP_ospiNandFlashImage,
    &UFP_ospiNandClose
};

#ifdef SPI_DMA_ENABLE
/*
 * UDMA driver objects
 */
static struct Udma_DrvObj      gUdmaDrvObj;
static struct Udma_ChObj       gUdmaChObj;
static struct Udma_EventObj    gUdmaCqEventObj;

static Udma_DrvHandle          gDrvHandle = NULL;

static OSPI_dmaInfo gUdmaInfo;

#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#ifdef SPI_DMA_ENABLE
/**
 *  \brief		This function initializes the UDMA module .
 *
 *  \param		cfg		[IN]	OSPI HW Attributes.
 *
 *  \return		int32_t
 *				UDMA_SOK in case of success or appropriate error code
 *
 */
static int32_t ospiUdmaInit(OSPI_v0_HwAttrs *cfg)
{
    int32_t         retVal = UDMA_SOK;
    Udma_InitPrms   initPrms;
    uint32_t        instId;

    if (gDrvHandle == NULL)
    {
        /* UDMA driver init */
#if defined (__aarch64__)
        instId = UDMA_INST_ID_MAIN_0;
#else
        instId = UDMA_INST_ID_MCU_0;
#endif
        UdmaInitPrms_init(instId, &initPrms);
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

/**
 *  \brief		This function De initializes the UDMA module.
 *
 *  \param		void
 *
 *  \return		int32_t
 *				UDMA_SOK in case of success or appropriate error code
 *
 */
static int32_t ospiUdmaDeinit(void)
{
    int32_t   retVal = UDMA_SOK;

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

/**
 *  \brief		This function closes the ospi Handle.
 *
 *  \return		int8_t
 *					0		- in case of success
 *
 */
static int8_t UFP_ospiNandClose(void)
{
    Board_flashClose(gOspiHandle);
#ifdef SPI_DMA_ENABLE
    ospiUdmaDeinit();
#endif
    return 0;
}

/**
 *  \brief		This function reads the image from specified location of ospi.
 *
 *	\param		dst			[OUT]  	Pointer to store the read image
 *	\param		offset		[IN]   	offset of the image on Flash
 *	\param		length		[IN]	Size of image.
 *
 *  \return		int8_t
 *					0		- in case of success
 *	               -1		- in case of failure.
 *
 */
static int8_t UFP_ospiNandFlashRead(uint8_t *dst, uint32_t offset, uint32_t length)
{
    uint32_t readMode;
    readMode = OSPI_FLASH_OCTAL_READ;

    if (Board_flashRead(gOspiHandle, offset, dst, length, &readMode))
    {
        return -1;
    }

    return 0;
}

/**
 *  \brief		This function writes the image to ospi on specified location.
 *
 *	\param		src			[IN]   	Pointer to the image to be flashed
 *	\param		offset		[IN]   	Offset to flash the image
 *	\param		length		[IN]	size of the image to be flashed.
 *
 *  \return		int8_t
 *					0		- in case of success
 *	               -1		- in case of failure.
 *
 */
static int8_t UFP_ospiNandFlashWrite(uint8_t *src, uint32_t offset, uint32_t length)
{
    uint32_t startBlockNum, endBlockNum, pageNum, i;
    uint32_t writeMode;

    writeMode = OSPI_FLASH_OCTAL_PAGE_PROG;

    if (!(offset % NAND_BLOCK_SIZE))
    {
        /* Get starting block number */
        if (Board_flashOffsetToBlkPage(gOspiHandle, offset, &startBlockNum, &pageNum))
        {
            return -1;
        }

        if(length < NAND_BLOCK_SIZE)
        {
            endBlockNum = startBlockNum;
        }
        else
        {
            /* Get ending block number */
            if (Board_flashOffsetToBlkPage(gOspiHandle, offset+length, &endBlockNum, &pageNum))
            {
                return -1;
            }
        }

        /* Erase blocks, to which data has to be written */
        for(i = startBlockNum; i <= endBlockNum; i++)
        {
            if (Board_flashEraseBlk(gOspiHandle, i))
            {
                return -1;
            }
        }
    }

    /* Write buffer to flash */
    for(i = 0; i < length; i+=OSPI_NAND_WR_LEN)
    {
        if (Board_flashWrite(gOspiHandle, offset, src, OSPI_NAND_WR_LEN, &writeMode))
        {
            return -1;
        }
        offset = offset + OSPI_NAND_WR_LEN;
        src = src + OSPI_NAND_WR_LEN;
	}

    return 0;
}

/**
 *  \brief		This function writes the image to ospi and reads back
 *				the image on specified address.
 *
 *  \param      flashAddr   [IN]    Pointer to the image to be flashed
 *	\param		checkAddr   [OUT]   Pointer to store the read image
 *  \param      offset      [IN]    Offset to flash the image.
 *  \param      size        [IN]    size of the image to be flashed.
 *
 *  \return		int8_t
 *					0		- in case of success
 *	               -1		- in case of failure.
 *
 */
static int8_t UFP_ospiNandFlashImage(uint8_t *flashAddr, uint8_t *checkAddr,
                                 uint32_t offset, uint32_t size)
{
#if defined(SPI_DMA_ENABLE)
    CacheP_wbInv((void *)flashAddr, (int32_t)size);
    CacheP_wbInv((void *)checkAddr, (int32_t)size);
#endif

    int8_t ret;
    ret = UFP_ospiNandFlashWrite(flashAddr, offset, size);
    if (ret != 0)
    {
        return -1;
    }

    ret = UFP_ospiNandFlashRead(checkAddr, offset, size);
    if (ret != 0)
    {
        return -1;
    }

#if defined(SPI_DMA_ENABLE)
    CacheP_wbInv((void *)checkAddr, (int32_t)size);
#endif

	return 0;
}

/**
 *  \brief		This function erases the ospi flash upto specified length
 *              from the specified address.
 *
 *	\param		offset		[IN]    Offset to erase the Flash.
 *	\param		length		[IN]    Erase length.
 *
 *  \return		int8_t
 *					0		- in case of success
 *	               -1		- in case of failure.
 */
static int8_t UFP_ospiNandFlashErase(uint32_t offset, uint32_t length)
{
    uint32_t startBlockNum, endBlockNum, pageNum, i;

    /* Get starting block number */
    if (Board_flashOffsetToBlkPage(gOspiHandle, offset,
                                   &startBlockNum, &pageNum))
    {
        return -1;
    }

    /* Get ending block number */
    endBlockNum = (offset + length)/(NAND_BLOCK_SIZE);

    /* Erase blocks, to which data has to be written */
    for(i = startBlockNum; i <= endBlockNum; i++)
    {
        if (Board_flashEraseBlk(gOspiHandle, i))
        {
            return -1;
        }
    }
	return 0;
}

/**
 *  \brief		This function initializes ospi flash.
 *				
 *  \return		int8_t
 *					0		- in case of success
 *	               -1		- in case of failure.
 *
 */
static int8_t UFP_ospiNandInit(void)
{
    OSPI_v0_HwAttrs ospi_cfg;
    uint32_t tuneEnable = FALSE;

    /* Get the default ospi init configurations */
    OSPI_socGetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_INSTANCE, &ospi_cfg);

    /* Modify the default ospi configurations if necessary */
    /* Turning off interrupts for baremetal mode. May be re-enabled by app */
    ospi_cfg.intrEnable = false;
#ifdef SPI_DMA_ENABLE
    ospi_cfg.dmaEnable  = true;
    ospiUdmaInit(&ospi_cfg);
#endif
    ospi_cfg.phyEnable  = false;

    /* Set the default ospi init configurations */
    OSPI_socSetInitCfg(BOARD_OSPI_DOMAIN, BOARD_OSPI_INSTANCE, &ospi_cfg);

    /* Open the Board ospi NAND device with ospi port 0
       and use default ospi configurations */
    gOspiHandle = Board_flashOpen(OSPI_NAND_FLASH_ID, BOARD_OSPI_INSTANCE, (void *)(&tuneEnable));

    if (!gOspiHandle)
    {
        return -1;
    }
    return 0;
}
