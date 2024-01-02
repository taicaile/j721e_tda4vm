/*
 *  Copyright (c) Texas Instruments Incorporated 2022
 *  All rights reserved.
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

/**
 *  \file app_utils_j784s4.c
 *
 *  \brief DSS example utility APIs for J7
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/board/src/j784s4_evm/include/board_control.h>
#include <ti/csl/soc/cslr_soc_ctrl_mmr.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/pm/pmlib.h>
#include "app_utils.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void App_configureSoC(void)
{
    /* Set drive strength */
    CSL_REG32_WR(CSL_WKUP_CTRL_MMR0_CFG0_BASE +
                 CSL_WKUP_CTRL_MMR_CFG0_H_IO_DRVSTRNGTH0_PROXY, 0xFU);

    CSL_REG32_WR(CSL_WKUP_CTRL_MMR0_CFG0_BASE +
                 CSL_WKUP_CTRL_MMR_CFG0_V_IO_DRVSTRNGTH0_PROXY, 0xFU);
}

void App_configureLCD(App_utilsLcdCfgParams cfgParams)
{
    int32_t status = PM_SUCCESS;
    uint32_t clockStatus;
    uint64_t minRate, respClkRate;
    if (APP_OUTPUT_DSI == cfgParams.outType)
    {
        /* Power on DSI and DPHY. */
        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmSetModuleState(
                TISCI_DEV_DSS_DSI0, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
        }
        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmSetModuleState(
                TISCI_DEV_DPHY_TX0, TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        /* Power off the DSS module for configuring clocks. */
        status = Sciclient_pmSetModuleState(TISCI_DEV_DSS0,
                TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
                TISCI_MSG_FLAG_AOP,
                SCICLIENT_SERVICE_WAIT_FOREVER);
        if(status == PM_SUCCESS)
        {
            printf("\n TISCI_DEV_DSS0 device shutdown successful !\r\n");
        }
        else
        {
            printf("\n TISCI_DEV_DSS0 device shutdown NOT successful !!!\r\n");
        }

        /* Check if the required clock can be supported by system firmware. */
        if(status == PM_SUCCESS)
        {
            minRate = cfgParams.pixelClk;
            status = Sciclient_pmQueryModuleClkFreq(TISCI_DEV_DSS0,
                                                TISCI_DEV_DSS0_DSS_INST0_DPI_2_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK,
                                                minRate,
                                                &respClkRate,
                                                SCICLIENT_SERVICE_WAIT_FOREVER);
        }
        
        if(status == PM_SUCCESS)
        {
            printf("\n TISCI_DEV_DSS0_DSS_INST0_DPI_2_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK possible rate = %lld Hz\r\n", respClkRate);
        }
        else
        {
            printf("\n TISCI_DEV_DSS0_DSS_INST0_DPI_2_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK requested rate range NOT possible !!\r\n");
        }

        if(status == PM_SUCCESS)
        {
            /* Check if the clock is enabled or not */
            status = Sciclient_pmModuleGetClkStatus(TISCI_DEV_DSS0,
                                                    TISCI_DEV_DSS0_DSS_INST0_DPI_2_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK,
                                                    &clockStatus,
                                                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        if ((status == PM_SUCCESS) && (respClkRate >= minRate))
        {
            /* Set the required DPI clock frequency. */
             status = Sciclient_pmSetModuleClkFreq(
                                      TISCI_DEV_DSS0,
                                      TISCI_DEV_DSS0_DSS_INST0_DPI_2_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK,
                                      respClkRate,
                                      TISCI_MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE,
                                      SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status == PM_SUCCESS && clockStatus == TISCI_MSG_VALUE_CLOCK_SW_STATE_UNREQ)
            {
                /* Enable the clock */
                status = Sciclient_pmModuleClkRequest(
                                                    TISCI_DEV_DSS0,
                                                    TISCI_DEV_DSS0_DSS_INST0_DPI_2_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK,
                                                    TISCI_MSG_VALUE_CLOCK_SW_STATE_REQ,
                                                    0U,
                                                    SCICLIENT_SERVICE_WAIT_FOREVER);
            }
        }

        /* Read back the set clock frequency. */
        uint64_t clkFreq = 0U;
        if (PM_SUCCESS == status)
        {
            PMLIBClkRateGet(TISCI_DEV_DSS0,
                TISCI_DEV_DSS0_DSS_INST0_DPI_2_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK,
                &clkFreq);
            printf("\n TISCI_DEV_DSS0_DSS_INST0_DPI_2_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK = %lld Hz\r\n", clkFreq);
        }

        /* Set the parent clock for TISCI_DEV_DSS0_DSS_INST0_DPI_2_IN_2X_CLK. */
        if (PM_SUCCESS == status)
        {
            status = Sciclient_pmSetModuleClkParent(TISCI_DEV_DSS0,
                TISCI_DEV_DSS0_DSS_INST0_DPI_2_IN_2X_CLK,
                TISCI_DEV_DSS0_DSS_INST0_DPI_2_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK,
                SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        /* Read back the clock of the child that required the clock configuration. */
        if(PM_SUCCESS == status)
        {
            clkFreq = 0;
            PMLIBClkRateGet(TISCI_DEV_DSS0,
                TISCI_DEV_DSS0_DSS_INST0_DPI_2_IN_2X_CLK,
                &clkFreq);
            printf("\n TISCI_DEV_DSS0_DSS_INST0_DPI_2_IN_2X_CLK: Expected %lld and getting %lld Hz\r\n", cfgParams.pixelClk, clkFreq);
        }    
    }
    else if (APP_OUTPUT_EDP == cfgParams.outType)
    {
        if(PM_SUCCESS == status)
        {
            /* Power on SerDes module. */
            status = Sciclient_pmSetModuleState(TISCI_DEV_SERDES_10G0,
                TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                TISCI_MSG_FLAG_AOP,
                SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        if(PM_SUCCESS == status)
        {
            /* Power on EDP module. */
            status = Sciclient_pmSetModuleState(TISCI_DEV_DSS_EDP0,
                TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                TISCI_MSG_FLAG_AOP,
                SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        /* Power off DSS before configuring the clocks. */
        status = Sciclient_pmSetModuleState(TISCI_DEV_DSS0,
                TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
                TISCI_MSG_FLAG_AOP,
                SCICLIENT_SERVICE_WAIT_FOREVER);
        if(status == PM_SUCCESS)
        {
            printf("\n TISCI_DEV_DSS0 device shutdown successful !\r\n");
        } else {
            printf("\n TISCI_DEV_DSS0 device shutdown NOT successful !!!\r\n");
        }

        /* Checking if the required pixel clock can be set by the system firmware. */
        minRate = cfgParams.pixelClk;
        status = Sciclient_pmQueryModuleClkFreq(TISCI_DEV_DSS0,
                                            TISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK,
                                            minRate,
                                            &respClkRate,
                                            SCICLIENT_SERVICE_WAIT_FOREVER);
        if(status == PM_SUCCESS)
        {
            printf("\n TISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK possible rate = %lld Hz\r\n", respClkRate);
        } else {
            printf("\n TISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK requested rate range NOT possible !!\r\n");
        }
        if(status == PM_SUCCESS)
        {
            /* Check if the clock is enabled or not. */
            status = Sciclient_pmModuleGetClkStatus(TISCI_DEV_DSS0,
                                                    TISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK,
                                                    &clockStatus,
                                                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        if ((status == PM_SUCCESS) && (respClkRate >= minRate))
        {
            /* Set the pixel clock. */
            status = Sciclient_pmSetModuleClkFreq(
                                      TISCI_DEV_DSS0,
                                      TISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK,
                                      respClkRate,
                                      TISCI_MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE,
                                      SCICLIENT_SERVICE_WAIT_FOREVER);
            if (status == PM_SUCCESS)
            {
                if (clockStatus == TISCI_MSG_VALUE_CLOCK_SW_STATE_UNREQ)
                {
                    /* Enable the required clock. */
                    status = Sciclient_pmModuleClkRequest(
                                                        TISCI_DEV_DSS0,
                                                        TISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK,
                                                        TISCI_MSG_VALUE_CLOCK_SW_STATE_REQ,
                                                        0U,
                                                        SCICLIENT_SERVICE_WAIT_FOREVER);
                }
            }
        }

        if(PM_SUCCESS == status)
        {
            uint64_t clkFreq = 0;
            status = Sciclient_pmSetModuleClkParent(TISCI_DEV_DSS0,
                TISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_2X_CLK,
                TISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK,
                SCICLIENT_SERVICE_WAIT_FOREVER);

            PMLIBClkRateGet(TISCI_DEV_DSS0,
                TISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK,
                &clkFreq);
            printf("\n TISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_2X_CLK_PARENT_HSDIV1_16FFT_MAIN_16_HSDIVOUT0_CLK Expected %lld and getting %lld Hz\r\n", cfgParams.pixelClk, clkFreq);

            clkFreq = 0;
            PMLIBClkRateGet(TISCI_DEV_DSS0,
                TISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_2X_CLK,
                &clkFreq);
            printf("\n TISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_2X_CLK Expected %lld and getting %lld Hz\r\n", cfgParams.pixelClk, clkFreq);

            printf("\nTISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_CLK is now ENABLED !\r\n");
            /* Enable the DSS device as clock configuration has been successful. */
            status = Sciclient_pmSetModuleState(TISCI_DEV_DSS0,
                TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                TISCI_MSG_FLAG_AOP,
                SCICLIENT_SERVICE_WAIT_FOREVER);
        } else {
            printf("\n TISCI_DEV_DSS0_DSS_INST0_DPI_0_IN_CLK is STILL DISABLED !!!\r\n");
        }
    }

}
