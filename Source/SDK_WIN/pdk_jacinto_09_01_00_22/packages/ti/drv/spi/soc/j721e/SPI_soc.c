/**
 *  \file   SPI_soc.c
 *
 *  \brief  J721E device specific hardware attributes.
 *
 */

/*
 * Copyright (C) 2019 - present Texas Instruments Incorporated - http://www.ti.com/
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

#include <ti/csl/soc/j721e/src/cslr_soc.h>
#include <ti/csl/soc/j721e/src/csl_psilcfg_thread_map.h>
#include <ti/drv/spi/soc/SPI_soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/spi/test/src/SPI_log.h>

/* SPI configuration structure */
SPI_v1_HWAttrs spiInitCfg[CSL_MCSPI_DOMAIN_CNT][CSL_MCSPI_PER_CNT] =
{
    /* MCU Domain MCSPI Instances */
    {
        {
            (uint32_t)CSL_MCU_MCSPI0_CFG_BASE,
            CSLR_MCU_R5FSS0_CORE0_INTR_MCU_MCSPI0_INTR_SPI_0,
            0,                                  /* eventId */
            (uint32_t)SPI_PINMODE_4_PIN,        /* pinMode */
            MCSPI_CHANNEL_0,                    /* chNum */
            MCSPI_SINGLE_CH,                    /* chMode */
            (bool)true,                         /* enableIntr */
            50000000,                           /* inputClkFreq */
            MCSPI_INITDLY_0,                    /* initDelay */
            MCSPI_RX_TX_FIFO_SIZE,              /* rxTrigLvl */
            MCSPI_RX_TX_FIFO_SIZE,              /* txTrigLvl */
            {
                {
                    MCSPI_CS_POL_LOW,                  /* csPolarity */
                    MCSPI_DATA_LINE_COMM_MODE_7,       /* dataLineCommMode */
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,   /* tcs */
                    MCSPI_TX_RX_MODE,                  /* trMode */
                    NULL,                              /* dmaInfo */
                    TRUE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
            },
            CSL_PDMA_CH_MCU_MCSPI0_CH0_RX,      /* rxDmaEventNumber */
            CSL_PDMA_CH_MCU_MCSPI0_CH0_TX,      /* txDmaEventNumber */
            0,                                  /* edmaTxTCC */
            0,                                  /* edmaRxTCC */
            0,                                  /* edmaTxTC */
            0,                                  /* edmaRxTC */
            NULL,                               /* edmaHandle */
            (bool)false,                        /* dmaMode */
        },
        {
            (uint32_t)CSL_MCU_MCSPI1_CFG_BASE,
            CSLR_MCU_R5FSS0_CORE0_INTR_MCU_MCSPI1_INTR_SPI_0,
            0,
            (uint32_t)SPI_PINMODE_4_PIN,
            MCSPI_CHANNEL_0,
            MCSPI_SINGLE_CH,
            (bool)true,
            50000000,
            MCSPI_INITDLY_0,
            MCSPI_RX_TX_FIFO_SIZE,
            MCSPI_RX_TX_FIFO_SIZE,
            {
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    TRUE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
            },
            CSL_PDMA_CH_MCU_MCSPI1_CH0_RX,
            CSL_PDMA_CH_MCU_MCSPI1_CH0_TX,
            0,
            0,
            0,
            0,
            NULL,
            (bool)false
        },
        {
            (uint32_t)CSL_MCU_MCSPI2_CFG_BASE,
            CSLR_MCU_R5FSS0_CORE0_INTR_MCU_MCSPI2_INTR_SPI_0,
            0,
            (uint32_t)SPI_PINMODE_4_PIN,
            MCSPI_CHANNEL_0,
            MCSPI_SINGLE_CH,
            (bool)true,
            50000000,
            MCSPI_INITDLY_0,
            MCSPI_RX_TX_FIFO_SIZE,
            MCSPI_RX_TX_FIFO_SIZE,
            {
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    TRUE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
            },
            CSL_PDMA_CH_MCU_MCSPI2_CH0_RX,
            CSL_PDMA_CH_MCU_MCSPI2_CH0_TX,
            0,
            0,
            0,
            0,
            NULL,
            (bool)false
        },
    },
    /* MAIN Domain MCSPI Instances */
    {
        {
            (uint32_t)CSL_MCSPI0_CFG_BASE,                         /* baseAddr */
            CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_MCSPI0_INTR_SPI_0,  /* intNum */
            0,                                  /* eventId */
            (uint32_t)SPI_PINMODE_4_PIN,        /* pinMode */
            MCSPI_CHANNEL_0,                    /* chNum */
            MCSPI_SINGLE_CH,                    /* chMode */
            (bool)true,                         /* enableIntr */
            50000000,                           /* inputClkFreq */
            MCSPI_INITDLY_0,                    /* initDelay */
            MCSPI_RX_TX_FIFO_SIZE,              /* rxTrigLvl */
            MCSPI_RX_TX_FIFO_SIZE,              /* txTrigLvl */
            {
                {
                    MCSPI_CS_POL_LOW,                  /* csPolarity */
                    MCSPI_DATA_LINE_COMM_MODE_7,       /* dataLineCommMode */
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,   /* tcs */
                    MCSPI_TX_RX_MODE,                  /* trMode */
                    NULL,                               /* edmaHandle */
                    TRUE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
            },
            CSL_PDMA_CH_MAIN_MCSPI0_CH0_RX,     /* rxDmaEventNumber */
            CSL_PDMA_CH_MAIN_MCSPI0_CH0_TX,     /* txDmaEventNumber */
            0,                                  /* edmaTxTCC */
            0,                                  /* edmaRxTCC */
            0,                                  /* edmaTxTC */
            0,                                  /* edmaRxTC */
            NULL,                               /* edmaHandle */
            (bool)false
        },
        {
            (uint32_t)CSL_MCSPI1_CFG_BASE,
            CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_MCSPI1_INTR_SPI_0,
            0,
            (uint32_t)SPI_PINMODE_4_PIN,
            MCSPI_CHANNEL_0,
            MCSPI_SINGLE_CH,
            (bool)true,
            50000000,
            MCSPI_INITDLY_0,
            MCSPI_RX_TX_FIFO_SIZE,
            MCSPI_RX_TX_FIFO_SIZE,
            {
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    TRUE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
            },
            CSL_PDMA_CH_MAIN_MCSPI1_CH0_RX,
            CSL_PDMA_CH_MAIN_MCSPI1_CH0_TX,
            0,
            0,
            0,
            0,
            NULL,
            (bool)false
        },
        {
            (uint32_t)CSL_MCSPI2_CFG_BASE,
            CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_MCSPI2_INTR_SPI_0,
            0,
            (uint32_t)SPI_PINMODE_4_PIN,
            MCSPI_CHANNEL_0,
            MCSPI_SINGLE_CH,
            (bool)true,
            50000000,
            MCSPI_INITDLY_0,
            MCSPI_RX_TX_FIFO_SIZE,
            MCSPI_RX_TX_FIFO_SIZE,
            {
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    TRUE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
            },
            CSL_PDMA_CH_MAIN_MCSPI2_CH0_RX,
            CSL_PDMA_CH_MAIN_MCSPI2_CH0_TX,
            0,
            0,
            0,
            0,
            NULL,
            (bool)false
        },
        {
            (uint32_t)CSL_MCSPI3_CFG_BASE,
            CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_MCSPI3_INTR_SPI_0,
            0,
            (uint32_t)SPI_PINMODE_4_PIN,
            MCSPI_CHANNEL_0,
            MCSPI_SINGLE_CH,
            (bool)true,
            50000000,
            MCSPI_INITDLY_0,
            MCSPI_RX_TX_FIFO_SIZE,
            MCSPI_RX_TX_FIFO_SIZE,
            {
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    TRUE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
            },
            CSL_PDMA_CH_MAIN_MCSPI3_CH0_RX,
            CSL_PDMA_CH_MAIN_MCSPI3_CH0_TX,
            0,
            0,
            0,
            0,
            NULL,
            (bool)false
        },
        {
            (uint32_t)CSL_MCSPI4_CFG_BASE,
            CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_MCSPI4_INTR_SPI_0,
            0,
            (uint32_t)SPI_PINMODE_4_PIN,
            MCSPI_CHANNEL_0,
            MCSPI_SINGLE_CH,
            (bool)true,
            50000000,
            MCSPI_INITDLY_0,
            MCSPI_RX_TX_FIFO_SIZE,
            MCSPI_RX_TX_FIFO_SIZE,
            {
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    TRUE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
            },
            CSL_PDMA_CH_MAIN_MCSPI4_CH0_RX,
            CSL_PDMA_CH_MAIN_MCSPI4_CH0_TX,
            0,
            0,
            0,
            0,
            NULL,
            (bool)false
        },
        {
            (uint32_t)CSL_MCSPI5_CFG_BASE,
            CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_MCSPI5_INTR_SPI_0,
            0,
            (uint32_t)SPI_PINMODE_4_PIN,
            MCSPI_CHANNEL_0,
            MCSPI_SINGLE_CH,
            (bool)true,
            50000000,
            MCSPI_INITDLY_0,
            MCSPI_RX_TX_FIFO_SIZE,
            MCSPI_RX_TX_FIFO_SIZE,
            {
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    TRUE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
            },
            CSL_PDMA_CH_MAIN_MCSPI5_CH0_RX,
            CSL_PDMA_CH_MAIN_MCSPI5_CH0_TX,
            0,
            0,
            0,
            0,
            NULL,
            (bool)false
        },
        {
            (uint32_t)CSL_MCSPI6_CFG_BASE,
            CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_MCSPI6_INTR_SPI_0,
            0,
            (uint32_t)SPI_PINMODE_4_PIN,
            MCSPI_CHANNEL_0,
            MCSPI_SINGLE_CH,
            (bool)true,
            50000000,
            MCSPI_INITDLY_0,
            MCSPI_RX_TX_FIFO_SIZE,
            MCSPI_RX_TX_FIFO_SIZE,
            {
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    TRUE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
            },
            CSL_PDMA_CH_MAIN_MCSPI6_CH0_RX,
            CSL_PDMA_CH_MAIN_MCSPI6_CH0_TX,
            0,
            0,
            0,
            0,
            NULL,
            (bool)false
        },
        {
            (uint32_t)CSL_MCSPI7_CFG_BASE,
            CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_MCSPI7_INTR_SPI_0,
            0,
            (uint32_t)SPI_PINMODE_4_PIN,
            MCSPI_CHANNEL_0,
            MCSPI_SINGLE_CH,
            (bool)true,
            50000000,
            MCSPI_INITDLY_0,
            MCSPI_RX_TX_FIFO_SIZE,
            MCSPI_RX_TX_FIFO_SIZE,
            {
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    TRUE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
                {
                    MCSPI_CS_POL_LOW,
                    MCSPI_DATA_LINE_COMM_MODE_7,
                    MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY,
                    MCSPI_TX_RX_MODE,
                    NULL,                              /* dmaInfo */
                    FALSE
                },
            },
            CSL_PDMA_CH_MAIN_MCSPI7_CH0_RX,
            CSL_PDMA_CH_MAIN_MCSPI7_CH0_TX,
            0,
            0,
            0,
            0,
            NULL,
            (bool)false
        },
    },
};


/* SPI objects */
SPI_v1_Object SpiObjects[CSL_MCSPI_DOMAIN_CNT][CSL_MCSPI_PER_CNT];

/* OSPI configuration structure */
OSPI_v0_HwAttrs ospiInitCfg[SPI_OSPI_DOMAIN_CNT][SPI_OSPI_PER_CNT + 1U] =
{
    {
        {
            0,                                  /* Instance Number */
            (uintptr_t)CSL_MCU_FSS0_OSPI0_CTRL_BASE,      /* flash config register baseAddr */
    #if defined (__aarch64__)
            (uintptr_t)CSL_MCU_FSS0_DAT_REG0_BASE,        /* OSPI data base address */
    #else
            (uintptr_t)CSL_MCU_FSS0_DAT_REG1_BASE,
    #endif
            CSL_OSPI_CFG_PHY_OP_MODE_MASTER,   /* PHY operating mode */ 
            CSL_OSPI_CFG_PHY_DLL_MODE_DEFAULT,   /* PHY DLL force half cycle lock */
            OSPI_MODULE_CLOCK,                 /* Input frequency */
    #if defined(BUILD_MPU)
        CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_MCU_FSS0_OSPI_0_OSPI_LVL_INTR_0,
    #else
        CSLR_MCU_R5FSS0_CORE0_INTR_MCU_FSS0_OSPI_0_OSPI_LVL_INTR_0, /* OSPI int number for ARM GIC INTC */
    #endif
            0,                                 /* Event ID not used for ARM INTC */
            OSPI_OPER_MODE_CFG,                /* operMode */
            CSL_OSPI_CS0,                      /* chipSelect */
            CSL_OSPI_CLK_MODE_0,               /* frmFmt */
            {
                0,                             /* default Chip Select Start of Transfer Delay */
                0,                             /* default Chip Select End of Transfer Delay */
                0,                             /* default Chip Select De-Assert Different Slaves Delay */
                OSPI_DEV_DELAY_CSDA            /* default Chip Select De-Assert Delay */
            },
            256,                               /* device page size is 256 bytes  */
            17,                                /* device block size is 2 ^ 17 = 128K bytes */
            OSPI_XFER_LINES_OCTAL,             /* xferLines */
            (bool)false,                       /* Interrupt mode */
            (bool)true,                        /* Direct Access Controller mode */
            (bool)false,                       /* DMA mode */
            NULL,                              /* dmaInfo */
            (bool)true,                        /* enable PHY */
            0,                                 /* rdDataCapDelay */
            (bool)true,                        /* enable DDR */
            (bool)false,                       /* enable XIP */
            10U,                               /* Chip Select Start Of Transfer delay */
            0,                                 /* Baudrate divider. Set to a non-zero value (2 to 32)
                                                to override default divider settings */
            (bool)false,                       /* enable Cache */
        },
        {
            1,                                  /* Instance Number */
            (uintptr_t)CSL_MCU_FSS0_OSPI1_CTRL_BASE,
    #if defined (__aarch64__)
            (uintptr_t)CSL_MCU_FSS0_OSPI1_R0_BASE,
    #else
            (uintptr_t)(CSL_MCU_FSS0_DAT_REG1_BASE + 0x08000000U),
    #endif
            CSL_OSPI_CFG_PHY_OP_MODE_MASTER,  /* PHY operating mode */
            CSL_OSPI_CFG_PHY_DLL_MODE_DEFAULT,   /* PHY DLL force half cycle lock */
            OSPI_MODULE_CLOCK,
    #if defined(BUILD_MPU)
            CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_MCU_FSS0_OSPI_1_OSPI_LVL_INTR_0,
    #else
            CSLR_MCU_R5FSS0_CORE0_INTR_MCU_FSS0_OSPI_1_OSPI_LVL_INTR_0,
    #endif
            0,
            OSPI_OPER_MODE_CFG,
            CSL_OSPI_CS0,
            CSL_OSPI_CLK_MODE_0,
            {
                0,
                0,
                0,
                OSPI_DEV_DELAY_CSDA
            },
            256,
            17,
            OSPI_XFER_LINES_OCTAL,
            (bool)false,
            (bool)true,
            (bool)false,
            NULL,
            (bool)true,
            0,
            (bool)true,
            (bool)false,
            10U,
            0,
            (bool)false,
        },
        {
            (uintptr_t)(0U),
        },
    },
    {
        {
            (uintptr_t)(0U),
        },
        {
            (uintptr_t)(0U),
        },
        {
            (uintptr_t)(0U),
        },
    }
};

/* OSPI objects */
OSPI_v0_Object OspiObjects[SPI_OSPI_DOMAIN_CNT][SPI_OSPI_PER_CNT + 1U];

/* SPI configuration structure */
CSL_PUBLIC_CONST SPI_config_list SPI_config = {
    {
        {
            &SPI_FxnTable_v1,
            &SpiObjects[0][0],
            &spiInitCfg[0][0]
        },
        {
            &SPI_FxnTable_v1,
            &SpiObjects[0][1],
            &spiInitCfg[0][1]
        },
        {
            &SPI_FxnTable_v1,
            &SpiObjects[0][2],
            &spiInitCfg[0][2]
        },
    },
    {
        {
            &SPI_FxnTable_v1,
            &SpiObjects[1][0],
            &spiInitCfg[1][0]
        },
        {
            &SPI_FxnTable_v1,
            &SpiObjects[1][1],
            &spiInitCfg[1][1]
        },
        {
            &SPI_FxnTable_v1,
            &SpiObjects[1][2],
            &spiInitCfg[1][2]
        },
        {
            &SPI_FxnTable_v1,
            &SpiObjects[1][3],
            &spiInitCfg[1][3]
        },
        {
            &SPI_FxnTable_v1,
            &SpiObjects[1][4],
            &spiInitCfg[1][4]
        },
        {
            &SPI_FxnTable_v1,
            &SpiObjects[1][5],
            &spiInitCfg[1][5]
        },
        {
            &SPI_FxnTable_v1,
            &SpiObjects[1][6],
            &spiInitCfg[1][6]
        },
        {
            &SPI_FxnTable_v1,
            &SpiObjects[1][7],
            &spiInitCfg[1][7]
        },
    },
};

/* OSPI configuration structure */
CSL_PUBLIC_CONST OSPI_config_list OSPI_config = {
    {
        {
            &OSPI_FxnTable_v0,
            &OspiObjects[0][0],
            &ospiInitCfg[0][0]
        },
        {
            &OSPI_FxnTable_v0,
            &OspiObjects[0][1],
            &ospiInitCfg[0][1]
        },
    },
    {
        {
            &OSPI_FxnTable_v0,
            &OspiObjects[1][0],
            &ospiInitCfg[1][0]
        },
        {
            &OSPI_FxnTable_v0,
            &OspiObjects[1][1],
            &ospiInitCfg[1][1]
        },
    },
};

/* MCSPI configuration structure */
MCSPI_config_list MCSPI_config = {
    /* McSPI MCU domain config list */
    {
        {
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[0][0])),
                0
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[0][0])),
                1
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[0][0])),
                2
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[0][0])),
                3
            },
        },
        {
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[0][1])),
                0
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[0][1])),
                1
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[0][1])),
                2
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[0][1])),
                3
            },
        },
        {
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[0][2])),
                0
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[0][2])),
                1
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[0][2])),
                2
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[0][2])),
                3
            },
        },
    },
    /* McSPI MAIN domain config list */
    {
        {
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][0])),
                0
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][0])),
                1
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][0])),
                2
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][0])),
                3
            },
        },
        {
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][1])),
                0
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][1])),
                1
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][1])),
                2
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][1])),
                3
            },
        },
        {
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][2])),
                0
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][2])),
                1
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][2])),
                2
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][2])),
                3
            },
        },
        {
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][3])),
                0
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][3])),
                1
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][3])),
                2
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][3])),
                3
            },
        },
        {
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][4])),
                0
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][4])),
                1
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][4])),
                2
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][4])),
                3
            },
        },
        {
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][5])),
                0
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][5])),
                1
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][5])),
                2
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][5])),
                3
            },
        },
        {
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][6])),
                0
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][6])),
                1
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][6])),
                2
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][6])),
                3
            },
        },
        {
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][7])),
                0
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][7])),
                1
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][7])),
                2
            },
            {
                &MCSPI_FxnTable_v1,
                (SPI_Handle)(&(SPI_config[1][7])),
                3
            },
        }
    }
};

/**
 * \brief  This API gets the SoC level of SPI intial configuration
 *
 * \param  domain    SPI instance domain.
 * \param  idx       SPI instance index.
 * \param  cfg       Pointer to SPI SOC initial config.
 *
 * \return 0 success: -1: error
 *
 */
int32_t SPI_socGetInitCfg(uint32_t domain, uint32_t idx, SPI_v1_HWAttrs *cfg)
{
    int32_t ret = 0;

    if (idx < CSL_MCSPI_PER_CNT)
    {
        *cfg = spiInitCfg[domain][idx];
    }
    else
    {
        ret = (int32_t)(-1);
    }
    return ret;
}

/**
 * \brief  This API sets the SoC level of SPI intial configuration
 *
 * \param  domain    SPI instance domain.
 * \param  idx       SPI instance index.
 * \param  cfg       Pointer to SPI SOC initial config.
 *
 * \return           0 success: -1: error
 *
 */
int32_t SPI_socSetInitCfg(uint32_t domain, uint32_t idx, const SPI_v1_HWAttrs *cfg)
{
    int32_t ret = 0;

    if (idx < CSL_MCSPI_PER_CNT)
    {
        spiInitCfg[domain][idx] = *cfg;
    }
    else
    {
        ret = (int32_t)(-1);
    }
    return ret;
}

/**
 * \brief  This API gets the SoC level of OSPI intial configuration
 *
 * \param  idx       OSPI instance index.
 * \param  cfg       Pointer to OSPI SOC initial config.
 *
 * \return 0 success: -1: error
 *
 */
int32_t OSPI_socGetInitCfg(uint32_t domain, uint32_t idx, OSPI_v0_HwAttrs *cfg)
{
    int32_t ret = 0;

    if (idx < SPI_OSPI_PER_CNT)
    {
        *cfg = ospiInitCfg[domain][idx];
    }
    else
    {
        ret = -1;
    }

    return ret;
}

/**
 * \brief  This API sets the SoC level of OSPI intial configuration
 *
 * \param  idx       OSPI instance index.
 * \param  cfg       Pointer to OSPI SOC initial config.
 *
 * \return           0 success: -1: error
 *
 */
int32_t OSPI_socSetInitCfg(uint32_t domain, uint32_t idx, const OSPI_v0_HwAttrs *cfg)
{
    int32_t ret = 0;

    if (idx < SPI_OSPI_PER_CNT)
    {
        ospiInitCfg[domain][idx] = *cfg;
    }
    else
    {
        ret = -1;
    }

    return ret;
}

/**
 * \brief  This API update the default SoC level of configurations
 *         based on the core and domain
 *
 *         ospiInitCfg table configures MCU domain's OSPI instances by
 *         default for R5, OSPI_socInit() is called to
 *         overwrite the defaut configurations with the configurations
 *         of Main domain's OSPI instances if R5 is on the Main domain
 *
 * \param  none
 *
 * \return           0 success: -1: error
 *
 */
int32_t OSPI_socInit(void)
{
    int32_t ret = 0;
#if defined (BUILD_MCU)
    CSL_ArmR5CPUInfo r5CpuInfo = {0};

    CSL_armR5GetCpuID(&r5CpuInfo);

    if (r5CpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)
    {
        /* Configure the MCU SS OSPI instances for MCU SS Pulsar R5 . R5FSS'n'_CORE'n' */
        /*  ********* OSPI-0 . intrNum = R5FSS'n'_CORE'n' **************** */
        if(r5CpuInfo.cpuID == 0U)
        {
            /*  ********* OSPI-0 . intrNum = R5FSS'n'_CORE'n' **************** */
            ospiInitCfg[0][0].intrNum = CSLR_MCU_R5FSS0_CORE0_INTR_MCU_FSS0_OSPI_0_OSPI_LVL_INTR_0;
            /* ********* OSPI-1 . intrNum = R5FSS'n'_INTRTR0 **************** */
            ospiInitCfg[0][1].intrNum = CSLR_MCU_R5FSS0_CORE0_INTR_MCU_FSS0_OSPI_1_OSPI_LVL_INTR_0;
        }
        else
        {
            /*  ********* OSPI-0 . intrNum = R5FSS'n'_CORE'n' **************** */
            ospiInitCfg[0][0].intrNum = CSLR_MCU_R5FSS0_CORE1_INTR_MCU_FSS0_OSPI_0_OSPI_LVL_INTR_0;
            /* ********* OSPI-1 . intrNum = R5FSS'n'_INTRTR0 **************** */
            ospiInitCfg[0][1].intrNum = CSLR_MCU_R5FSS0_CORE1_INTR_MCU_FSS0_OSPI_1_OSPI_LVL_INTR_0;
        }
    }
    else if (r5CpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1) /* Main R5 SS0*/
    {
        /* Configure the Main SS OSPI instances for Main SS Pulsar R5 . R5FSS'n'_CORE'n' */
        /*  ********* OSPI-0 . intrNum = R5FSS'n'_CORE'n' **************** */
        if(r5CpuInfo.cpuID == 0U)
        {
            /*  ********* OSPI-0 . intrNum = R5FSS'n'_CORE'n' **************** */
            ospiInitCfg[0][0].intrNum = CSLR_R5FSS0_CORE0_INTR_R5FSS0_INTROUTER0_OUTL_0;
            /* ********* OSPI-1 . intrNum = R5FSS'n'_INTRTR0 **************** */
            ospiInitCfg[0][1].intrNum = CSLR_R5FSS0_CORE0_INTR_R5FSS0_INTROUTER0_OUTL_1;
        }
        else
        {
            /*  ********* OSPI-0 . intrNum = R5FSS'n'_CORE'n' **************** */
            ospiInitCfg[0][0].intrNum = CSLR_R5FSS0_CORE1_INTR_R5FSS0_INTROUTER0_OUTL_128;
            /* ********* OSPI-1 . intrNum = R5FSS'n'_INTRTR0 **************** */
            ospiInitCfg[0][1].intrNum = CSLR_R5FSS0_CORE1_INTR_R5FSS0_INTROUTER0_OUTL_129;
        }
    }
    else if (r5CpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_2) /* Main R5 SS1*/
    {
        /* Configure the Main SS OSPI instances for Main SS Pulsar R5 . R5FSS'n'_CORE'n' */
        /*  ********* OSPI-0 . intrNum = R5FSS'n'_CORE'n' **************** */
        if(r5CpuInfo.cpuID == 0U)
        {
            /*  ********* OSPI-0 . intrNum = R5FSS'n'_CORE'n' **************** */
            ospiInitCfg[0][0].intrNum = CSLR_R5FSS1_CORE0_INTR_R5FSS1_INTROUTER0_OUTL_0;
            /* ********* OSPI-1 . intrNum = R5FSS'n'_INTRTR0 **************** */
            ospiInitCfg[0][1].intrNum = CSLR_R5FSS1_CORE0_INTR_R5FSS1_INTROUTER0_OUTL_1;
        }
        else
        {
            /*  ********* OSPI-0 . intrNum = R5FSS'n'_CORE'n' **************** */
            ospiInitCfg[0][0].intrNum = CSLR_R5FSS1_CORE1_INTR_R5FSS1_INTROUTER0_OUTL_128;
            /* ********* OSPI-1 . intrNum = R5FSS'n'_INTRTR0 **************** */
            ospiInitCfg[0][1].intrNum = CSLR_R5FSS1_CORE1_INTR_R5FSS1_INTROUTER0_OUTL_129;
        }
    }
    else
    {
        ret = -1;
    }
#endif
    return (ret);
}

/**
 * \brief  This function will configure the interrupt path to the destination CPU
 *         using DMSC firmware via sciclient. if setIntrPath is set to TRUE,
 *         a path is set, else the interrupt path is released
 *
 * \param  hwAttrs_ptr Pointer to hardware attributes
 * \param  setIntrPath Set or release interrupt
 *
 * \return           0 success: -1: error
 *
 */
int32_t OSPI_configSocIntrPath(void *hwAttrs_ptr, bool setIntrPath)
{
    int32_t retVal = 0;
#if defined(BUILD_MCU)
    CSL_ArmR5CPUInfo r5CpuInfo = {0};
    OSPI_v0_HwAttrs *hwAttrs = (OSPI_v0_HwAttrs*)(hwAttrs_ptr);
    struct tisci_msg_rm_irq_set_req  rmIrqReq;
    struct tisci_msg_rm_irq_set_resp rmIrqResp;
    struct tisci_msg_rm_irq_release_req rmIrqRelease;
    struct tisci_msg_rm_get_resource_range_resp res;
    struct tisci_msg_rm_get_resource_range_req  req;
    uint16_t src_id,src_index,dst_id = 0U,dst_host_irq;
    uint16_t intNum, intRangeNum;

    memset(&rmIrqReq, 0, sizeof(rmIrqReq));
    memset(&rmIrqResp, 0, sizeof(rmIrqResp));
    memset(&rmIrqRelease, 0, sizeof(rmIrqRelease));
    memset(&res, 0, sizeof(res));
    memset(&req, 0, sizeof(req));

    CSL_armR5GetCpuID(&r5CpuInfo);

    if (r5CpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)             /* MCU R5 */
    {
        /* No routing required */
    }
    else
    {
        if(hwAttrs->instNum==0)
        {
            src_id = TISCI_DEV_MCU_FSS0_OSPI_0;
        }
        else
        {
            src_id = TISCI_DEV_MCU_FSS0_OSPI_1;
        }

        if (r5CpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1)         /* MAIN R5 SS0 */
        {
            req.type = TISCI_DEV_R5FSS0_INTROUTER0;
            if(r5CpuInfo.cpuID == 0U)
            {
                dst_id = TISCI_DEV_R5FSS0_CORE0;    /* Main R5 -SS0 - CPU0 */
            }
            else
            {
                dst_id = TISCI_DEV_R5FSS0_CORE1;    /* Main R5 -SS0 - CPU1*/
            }
        }
        else if (r5CpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_2)    /* MAIN R5 SS1 */
        {
            req.type = TISCI_DEV_R5FSS1_INTROUTER0;
            if(r5CpuInfo.cpuID == 0U)
            {
                dst_id = TISCI_DEV_R5FSS1_CORE0;    /* Main R5 -SS1 - CPU0 */
            }
            else
            {
                dst_id = TISCI_DEV_R5FSS1_CORE1;    /* Main R5 -SS1 - CPU1*/
            }
        }

        if(setIntrPath)
        {
            req.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

            /* Get interrupt number range */
            retVal = Sciclient_rmGetResourceRange(&req,
                                                &res,
                                                SCICLIENT_SERVICE_WAIT_FOREVER);

            if (CSL_PASS != retVal || res.range_num == 0)
            {
                /* Try with HOST_ID_ALL */
                req.secondary_host = TISCI_HOST_ID_ALL;
                retVal = Sciclient_rmGetResourceRange(&req,
                                                    &res,
                                                    SCICLIENT_SERVICE_WAIT_FOREVER);
            }
            if (CSL_PASS == retVal)
            {
                intRangeNum = res.range_num;
                if (intRangeNum == 0)
                {
                    retVal = -1;
                }
            }
            if (CSL_PASS == retVal)
            {
                /* Translation must happen after this offset */
                retVal = Sciclient_rmIrqTranslateIrOutput(req.type,
                                                        res.range_start,
                                                        dst_id,
                                                        &intNum);
            }

            if (CSL_PASS == retVal)
            {
                hwAttrs->intrNum = intNum;
                src_index = hwAttrs->eventId;               /* Index coming from the peripheral */
                dst_host_irq = hwAttrs->intrNum;            /* By default it is set for MCU R5 */

                rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_DST_ID_VALID;
                rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
                rmIrqReq.src_id                 = src_id;
                rmIrqReq.global_event           = 0U;
                rmIrqReq.src_index              = src_index;
                rmIrqReq.dst_id                 = dst_id;
                rmIrqReq.dst_host_irq           = dst_host_irq;
                rmIrqReq.ia_id                  = 0U;
                rmIrqReq.vint                   = 0U;
                rmIrqReq.vint_status_bit_index  = 0U;
                rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
                retVal                          = Sciclient_rmIrqSet(&rmIrqReq,
                                                                     &rmIrqResp,
                                                                     SCICLIENT_SERVICE_WAIT_FOREVER);
            }
        }
        else
        {
            src_index = hwAttrs->eventId;               /* Index coming from the peripheral */
            dst_host_irq = hwAttrs->intrNum;            /* By default it is set for MCU R5 */

            rmIrqRelease.valid_params           = TISCI_MSG_VALUE_RM_DST_ID_VALID;
            rmIrqRelease.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
            rmIrqRelease.src_id                 = src_id;
            rmIrqRelease.global_event           = 0U;
            rmIrqRelease.src_index              = src_index;
            rmIrqRelease.dst_id                 = dst_id;
            rmIrqRelease.dst_host_irq           = dst_host_irq;
            rmIrqRelease.ia_id                  = 0U;
            rmIrqRelease.vint                   = 0U;
            rmIrqRelease.vint_status_bit_index  = 0U;
            rmIrqRelease.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
            retVal                              = Sciclient_rmIrqRelease(&rmIrqRelease,
                                                                         SCICLIENT_SERVICE_WAIT_FOREVER);
        }
    }
#endif
    return(retVal);
}

/**
 * \brief  This API update the default SoC level of configurations
 *         based on the core and domain
 *
 *         spiInitCfg table configures MCSPI instances by
 *         default for MCU R5 0, MCSPI_socInit() is called to
 *         overwrite the defaut configurations
 *
 * \param  none
 *
 * \return           0 success: -1: error
 *
 */
int32_t MCSPI_socInit(void)
{
    int32_t ret = 0;
#if defined(BUILD_MCU)
    CSL_ArmR5CPUInfo r5CpuInfo = {0};

    CSL_armR5GetCpuID(&r5CpuInfo);

    if (r5CpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)             /* MCU R5 */
    {
        /* No change required */
    }
    else if (r5CpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1)         /* MAIN R5 SS0 */
    {
        spiInitCfg[0][0].intNum = CSLR_R5FSS0_INTROUTER0_IN_MCU_MCSPI0_INTR_SPI_0;
        spiInitCfg[0][1].intNum = CSLR_R5FSS0_INTROUTER0_IN_MCU_MCSPI1_INTR_SPI_0;
        spiInitCfg[0][2].intNum = CSLR_R5FSS0_INTROUTER0_IN_MCU_MCSPI2_INTR_SPI_0;
        spiInitCfg[1][0].intNum = CSLR_R5FSS0_INTROUTER0_IN_MCU_MCSPI0_INTR_SPI_0;
        spiInitCfg[1][1].intNum = CSLR_R5FSS0_INTROUTER0_IN_MCU_MCSPI1_INTR_SPI_0;
        spiInitCfg[1][2].intNum = CSLR_R5FSS0_INTROUTER0_IN_MCU_MCSPI2_INTR_SPI_0;
        spiInitCfg[1][3].intNum = CSLR_R5FSS0_INTROUTER0_IN_MCSPI3_INTR_SPI_0;
        spiInitCfg[1][4].intNum = CSLR_R5FSS0_INTROUTER0_IN_MCSPI4_INTR_SPI_0;
        spiInitCfg[1][5].intNum = CSLR_R5FSS0_INTROUTER0_IN_MCSPI5_INTR_SPI_0;
        spiInitCfg[1][6].intNum = CSLR_R5FSS0_INTROUTER0_IN_MCSPI6_INTR_SPI_0;
        spiInitCfg[1][7].intNum = CSLR_R5FSS0_INTROUTER0_IN_MCSPI7_INTR_SPI_0;
    }
    else if (r5CpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_2)         /* MAIN R5 SS1 */
    {
        spiInitCfg[0][0].intNum = CSLR_R5FSS1_INTROUTER0_IN_MCU_MCSPI0_INTR_SPI_0;
        spiInitCfg[0][1].intNum = CSLR_R5FSS1_INTROUTER0_IN_MCU_MCSPI1_INTR_SPI_0;
        spiInitCfg[0][2].intNum = CSLR_R5FSS1_INTROUTER0_IN_MCU_MCSPI2_INTR_SPI_0;
        spiInitCfg[1][0].intNum = CSLR_R5FSS1_INTROUTER0_IN_MCU_MCSPI0_INTR_SPI_0;
        spiInitCfg[1][1].intNum = CSLR_R5FSS1_INTROUTER0_IN_MCU_MCSPI1_INTR_SPI_0;
        spiInitCfg[1][2].intNum = CSLR_R5FSS1_INTROUTER0_IN_MCU_MCSPI2_INTR_SPI_0;
        spiInitCfg[1][3].intNum = CSLR_R5FSS1_INTROUTER0_IN_MCSPI3_INTR_SPI_0;
        spiInitCfg[1][4].intNum = CSLR_R5FSS1_INTROUTER0_IN_MCSPI4_INTR_SPI_0;
        spiInitCfg[1][5].intNum = CSLR_R5FSS1_INTROUTER0_IN_MCSPI5_INTR_SPI_0;
        spiInitCfg[1][6].intNum = CSLR_R5FSS1_INTROUTER0_IN_MCSPI6_INTR_SPI_0;
        spiInitCfg[1][7].intNum = CSLR_R5FSS1_INTROUTER0_IN_MCSPI7_INTR_SPI_0;
    }
    else
    {
        ret = -1;
    }
#endif
    return (ret);
}

/**
 * \brief  This function will configure the interrupt path to the destination CPU
 *         using DMSC firmware via sciclient. if setIntrPath is set to TRUE,
 *         a path is set, else the interrupt path is released
 *
 * \param  instance    MCSPI Instance
 * \param  hwAttrs_ptr Pointer to hardware attributes
 * \param  setIntrPath Set or release interrupt
 *
 * \return           0 success: -1: error
 *
 */
int32_t MCSPI_configSocIntrPath(uint32_t instance, void *hwAttrs_ptr, bool setIntrPath)
{
    int32_t retVal = 0;
#if defined(BUILD_MCU)
    CSL_ArmR5CPUInfo r5CpuInfo = {0};
    SPI_v1_HWAttrs *hwAttrs = (SPI_v1_HWAttrs*)(hwAttrs_ptr);
    struct tisci_msg_rm_irq_set_req  rmIrqReq;
    struct tisci_msg_rm_irq_set_resp rmIrqResp;
    struct tisci_msg_rm_irq_release_req rmIrqRelease;
    struct tisci_msg_rm_get_resource_range_resp res;
    struct tisci_msg_rm_get_resource_range_req  req;
    uint16_t src_id,src_index,dst_id = 0U,dst_host_irq;
    uint16_t intNum, intRangeNum;

    memset(&rmIrqReq, 0, sizeof(rmIrqReq));
    memset(&rmIrqResp, 0, sizeof(rmIrqResp));
    memset(&rmIrqRelease, 0, sizeof(rmIrqRelease));
    memset(&res, 0, sizeof(res));
    memset(&req, 0, sizeof(req));

    CSL_armR5GetCpuID(&r5CpuInfo);

    if (r5CpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)             /* MCU R5 */
    {
        /* No routing required */
    }
    else
    {
        if(instance==0)
        {
            src_id = TISCI_DEV_MCU_MCSPI0;
        }
        else if(instance==1)
        {
            src_id = TISCI_DEV_MCU_MCSPI1;
        }
        else if(instance==2)
        {
            src_id = TISCI_DEV_MCU_MCSPI2;
        }
        else if(instance==3)
        {
            src_id = TISCI_DEV_MCSPI3;
        }
        else if(instance==4)
        {
            src_id = TISCI_DEV_MCSPI4;
        }
        else if(instance==5)
        {
            src_id = TISCI_DEV_MCSPI5;
        }
        else if(instance==6)
        {
            src_id = TISCI_DEV_MCSPI6;
        }
        else
        {
            src_id = TISCI_DEV_MCSPI7;
        }

        if (r5CpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1)         /* MAIN R5 SS0 */
        {
            req.type = TISCI_DEV_R5FSS0_INTROUTER0;
            req.subtype = TISCI_RESASG_SUBTYPE_IR_OUTPUT;
            if(r5CpuInfo.cpuID == 0U)
            {
                dst_id = TISCI_DEV_R5FSS0_CORE0;    /* Main R5 -SS0 - CPU0 */
            }
            else
            {
                dst_id = TISCI_DEV_R5FSS0_CORE1;    /* Main R5 -SS0 - CPU1*/
            }
        }
        else if (r5CpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_2)    /* MAIN R5 SS1 */
        {
            req.type = TISCI_DEV_R5FSS1_INTROUTER0;
            req.subtype = TISCI_RESASG_SUBTYPE_IR_OUTPUT;
            if(r5CpuInfo.cpuID == 0U)
            {
                dst_id = TISCI_DEV_R5FSS1_CORE0;    /* Main R5 -SS1 - CPU0 */
            }
            else
            {
                dst_id = TISCI_DEV_R5FSS1_CORE1;    /* Main R5 -SS1 - CPU1*/
            }
        }

        if(setIntrPath)
        {
            req.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

            /* Get interrupt number range */
            retVal = Sciclient_rmGetResourceRange(&req,
                                                &res,
                                                SCICLIENT_SERVICE_WAIT_FOREVER);

            if (CSL_PASS != retVal || res.range_num == 0)
            {
                /* Try with HOST_ID_ALL */
                req.secondary_host = TISCI_HOST_ID_ALL;
                retVal = Sciclient_rmGetResourceRange(&req,
                                                    &res,
                                                    SCICLIENT_SERVICE_WAIT_FOREVER);
            }
            if (CSL_PASS == retVal)
            {
                intRangeNum = res.range_num;
                if (intRangeNum == 0)
                {
                    retVal = -1;
                }
            }
            if (CSL_PASS == retVal)
            {
                /* Translation must happen after this offset */
                retVal = Sciclient_rmIrqTranslateIrOutput(req.type,
                                                        res.range_start,
                                                        dst_id,
                                                        &intNum);
            }

            if (CSL_PASS == retVal)
            {
                hwAttrs->intNum = intNum;
                src_index = hwAttrs->eventId;               /* Index coming from the peripheral */
                dst_host_irq = hwAttrs->intNum;            /* By default it is set for MCU R5 */

                rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_DST_ID_VALID;
                rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
                rmIrqReq.src_id                 = src_id;
                rmIrqReq.global_event           = 0U;
                rmIrqReq.src_index              = src_index;
                rmIrqReq.dst_id                 = dst_id;
                rmIrqReq.dst_host_irq           = dst_host_irq;
                rmIrqReq.ia_id                  = 0U;
                rmIrqReq.vint                   = 0U;
                rmIrqReq.vint_status_bit_index  = 0U;
                rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
                retVal                          = Sciclient_rmIrqSet(&rmIrqReq,
                                                                     &rmIrqResp,
                                                                     SCICLIENT_SERVICE_WAIT_FOREVER);
            }
        }
        else
        {
            src_index = hwAttrs->eventId;               /* Index coming from the peripheral */
            dst_host_irq = hwAttrs->intNum;            /* By default it is set for MCU R5 */

            rmIrqRelease.valid_params           = TISCI_MSG_VALUE_RM_DST_ID_VALID;
            rmIrqRelease.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
            rmIrqRelease.src_id                 = src_id;
            rmIrqRelease.global_event           = 0U;
            rmIrqRelease.src_index              = src_index;
            rmIrqRelease.dst_id                 = dst_id;
            rmIrqRelease.dst_host_irq           = dst_host_irq;
            rmIrqRelease.ia_id                  = 0U;
            rmIrqRelease.vint                   = 0U;
            rmIrqRelease.vint_status_bit_index  = 0U;
            rmIrqRelease.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
            retVal                              = Sciclient_rmIrqRelease(&rmIrqRelease,
                                                                         SCICLIENT_SERVICE_WAIT_FOREVER);
        }
    }
#endif
    return(retVal);
}
