/**
* Note: This file was auto-generated by TI PinMux on 5/10/2019 at 3:40:37 PM.
*
* \file  J721E_pinmux_data.c
*
* \brief  This file contains the pin mux configurations for the boards.
*         These are prepared based on how the peripherals are extended on
*         the boards.
*
* \copyright Copyright (CU) 2019 Texas Instruments Incorporated -
*             http://www.ti.com/
*/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "J721E_pinmux.h"

/** Peripheral Pin Configurations */


static pinmuxPerCfg_t gDebugss0PinCfg[] =
{
    /* MyDEBUG1 -> TDI -> V1 */
    {
        PIN_TDI, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyDEBUG1 -> TDO -> V3 */
    {
        PIN_TDO, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyDEBUG1 -> TMS -> V2 */
    {
        PIN_TMS, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gDebugssPinCfg[] =
{
    {0, TRUE, gDebugss0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gDp0PinCfg[] =
{
    /* MyDP0 -> DP0_HPD -> Y4 */
    {
        PIN_SPI0_CS1, PIN_MODE(5) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gDpPinCfg[] =
{
    {0, TRUE, gDp0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gGpio0PinCfg[] =
{
    /* MySYSTEM1 -> GPIO0_0 -> AC18 */
    {
        PIN_EXTINTN, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO0 -> GPIO0_97 -> Y28 */
    {
        PIN_RGMII6_TX_CTL, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO0 -> GPIO0_98 -> V23 */
    {
        PIN_RGMII6_RX_CTL, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO0 -> GPIO0_117 -> W4 */
    {
        PIN_SPI1_CS1, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO0 -> GPIO0_127 -> AC4 */
    {
        PIN_UART1_CTSN, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gGpio1PinCfg[] =
{
    /* MyGPIO1 -> GPIO1_0 -> AD5 */
    {
        PIN_UART1_RTSN, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO1 -> GPIO1_3 -> W3 */
    {
        PIN_MCAN1_RX, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO1 -> GPIO1_5 -> W2 */
    {
        PIN_I3C0_SCL, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO1 -> GPIO1_6 -> W1 */
    {
        PIN_I3C0_SDA, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO1 -> GPIO1_11 -> U2 */
    {
        PIN_ECAP0_IN_APWM_OUT, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO1 -> GPIO1_12 -> U3 */
    {
        PIN_EXT_REFCLK1, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO1 -> GPIO1_22 -> R28 */
    {
        PIN_MMC1_SDWP, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO1 -> GPIO1_23 -> T28 */
    {
        PIN_MMC2_DAT3, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO1 -> GPIO1_24 -> T29 */
    {
        PIN_MMC2_DAT2, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO1 -> GPIO1_25 -> T27 */
    {
        PIN_MMC2_DAT1, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyGPIO1 -> GPIO1_26 -> T24 */
    {
        PIN_MMC2_DAT0, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gGpioPinCfg[] =
{
    {0, TRUE, gGpio0PinCfg},
    {1, TRUE, gGpio1PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gI2c2PinCfg[] =
{
    /* MyI2C2 -> I2C2_SCL -> AA1 */
    {
        PIN_SPI0_CLK, PIN_MODE(2) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    /* MyI2C2 -> I2C2_SDA -> AB5 */
    {
        PIN_SPI0_D0, PIN_MODE(2) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gI2c6PinCfg[] =
{
    /* MyI2C6 -> I2C6_SCL -> AA3 */
    {
        PIN_SPI0_D1, PIN_MODE(2) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    /* MyI2C6 -> I2C6_SDA -> Y2 */
    {
        PIN_SPI1_D1, PIN_MODE(2) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gI2c0PinCfg[] =
{
    /* MyI2C0 -> I2C0_SCL -> AC5 */
    {
        PIN_I2C0_SCL, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    /* MyI2C0 -> I2C0_SDA -> AA5 */
    {
        PIN_I2C0_SDA, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gI2c1PinCfg[] =
{
    /* MyI2C1 -> I2C1_SCL -> Y6 */
    {
        PIN_I2C1_SCL, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    /* MyI2C1 -> I2C1_SDA -> AA6 */
    {
        PIN_I2C1_SDA, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gI2c3PinCfg[] =
{
    /* MyI2C3 -> I2C3_SCL -> T26 */
    {
        PIN_MMC2_CLK, PIN_MODE(4) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    /* MyI2C3 -> I2C3_SDA -> T25 */
    {
        PIN_MMC2_CMD, PIN_MODE(4) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gI2cPinCfg[] =
{
    {2, TRUE, gI2c2PinCfg},
    {6, TRUE, gI2c6PinCfg},
    {0, TRUE, gI2c0PinCfg},
    {1, TRUE, gI2c1PinCfg},
    {3, TRUE, gI2c3PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gMcan2PinCfg[] =
{
    /* MyMCAN2 -> MCAN2_RX -> AC2 */
    {
        PIN_UART0_CTSN, PIN_MODE(3) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCAN2 -> MCAN2_TX -> AB1 */
    {
        PIN_UART0_RTSN, PIN_MODE(3) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gMcan0PinCfg[] =
{
    /* MyMCAN0 -> MCAN0_RX -> W5 */
    {
        PIN_MCAN0_RX, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCAN0 -> MCAN0_TX -> W6 */
    {
        PIN_MCAN0_TX, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMcanPinCfg[] =
{
    {2, TRUE, gMcan2PinCfg},
    {0, TRUE, gMcan0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gMcu_i2c0PinCfg[] =
{
    /* MyMCU_I2C0 -> MCU_I2C0_SCL -> J26 */
    {
        PIN_MCU_I2C0_SCL, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_I2C0 -> MCU_I2C0_SDA -> H25 */
    {
        PIN_MCU_I2C0_SDA, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMcu_i2cPinCfg[] =
{
    {0, TRUE, gMcu_i2c0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gMcu_i3c0PinCfg[] =
{
    /* MyMCU_I3C0 -> MCU_I3C0_SCL -> D26 */
    {
        PIN_MCU_I3C0_SCL, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_I3C0 -> MCU_I3C0_SDA -> D25 */
    {
        PIN_MCU_I3C0_SDA, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_I3C0 -> MCU_I3C0_SDAPULLEN -> E26 */
    {
        PIN_PMIC_POWER_EN0, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMcu_i3cPinCfg[] =
{
    {0, TRUE, gMcu_i3c0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gMcu_mcan0PinCfg[] =
{
    /* MyMCU_MCAN0 -> MCU_MCAN0_RX -> C29 */
    {
        PIN_MCU_MCAN0_RX, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_MCAN0 -> MCU_MCAN0_TX -> D29 */
    {
        PIN_MCU_MCAN0_TX, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gMcu_mcan1PinCfg[] =
{
    /* MyMCU_MCAN1 -> MCU_MCAN1_RX -> G24 */
    {
        PIN_WKUP_GPIO0_5, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_MCAN1 -> MCU_MCAN1_TX -> G25 */
    {
        PIN_WKUP_GPIO0_4, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMcu_mcanPinCfg[] =
{
    {0, TRUE, gMcu_mcan0PinCfg},
    {1, TRUE, gMcu_mcan1PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gMcu_mdio0PinCfg[] =
{
    /* MyMCU_MDIO1 -> MCU_MDIO0_MDC -> F23 */
    {
        PIN_MCU_MDIO0_MDC, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMCU_MDIO1 -> MCU_MDIO0_MDIO -> E23 */
    {
        PIN_MCU_MDIO0_MDIO, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMcu_mdioPinCfg[] =
{
    {0, TRUE, gMcu_mdio0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gMcu_fss0_ospi0PinCfg[] =
{
    /* MyMCU_OSPI0 -> MCU_OSPI0_CLK -> E20 */
    {
        PIN_MCU_OSPI0_CLK, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMCU_OSPI0 -> MCU_OSPI0_CSn0 -> F19 */
    {
        PIN_MCU_OSPI0_CSN0, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMCU_OSPI0 -> MCU_OSPI0_D0 -> D20 */
    {
        PIN_MCU_OSPI0_D0, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_OSPI0 -> MCU_OSPI0_D1 -> G19 */
    {
        PIN_MCU_OSPI0_D1, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_OSPI0 -> MCU_OSPI0_D2 -> G20 */
    {
        PIN_MCU_OSPI0_D2, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_OSPI0 -> MCU_OSPI0_D3 -> F20 */
    {
        PIN_MCU_OSPI0_D3, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_OSPI0 -> MCU_OSPI0_D4 -> F21 */
    {
        PIN_MCU_OSPI0_D4, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_OSPI0 -> MCU_OSPI0_D5 -> E21 */
    {
        PIN_MCU_OSPI0_D5, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_OSPI0 -> MCU_OSPI0_D6 -> B22 */
    {
        PIN_MCU_OSPI0_D6, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_OSPI0 -> MCU_OSPI0_D7 -> G21 */
    {
        PIN_MCU_OSPI0_D7, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_OSPI0 -> MCU_OSPI0_DQS -> D21 */
    {
        PIN_MCU_OSPI0_DQS, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gMcu_fss0_ospi1PinCfg[] =
{
    /* MyMCU_OSPI1 -> MCU_OSPI1_CLK -> F22 */
    {
        PIN_MCU_OSPI1_CLK, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMCU_OSPI1 -> MCU_OSPI1_CSn0 -> C22 */
    {
        PIN_MCU_OSPI1_CSN0, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMCU_OSPI1 -> MCU_OSPI1_D0 -> D22 */
    {
        PIN_MCU_OSPI1_D0, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_OSPI1 -> MCU_OSPI1_D1 -> G22 */
    {
        PIN_MCU_OSPI1_D1, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_OSPI1 -> MCU_OSPI1_D2 -> D23 */
    {
        PIN_MCU_OSPI1_D2, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_OSPI1 -> MCU_OSPI1_D3 -> C23 */
    {
        PIN_MCU_OSPI1_D3, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_OSPI1 -> MCU_OSPI1_DQS -> B23 */
    {
        PIN_MCU_OSPI1_DQS, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_OSPI1 -> MCU_OSPI1_LBCLKO -> A23 */
    {
        PIN_MCU_OSPI1_LBCLKO, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMcu_fss0_ospiPinCfg[] =
{
    {0, TRUE, gMcu_fss0_ospi0PinCfg},
    {1, TRUE, gMcu_fss0_ospi1PinCfg},
    {PINMUX_END}
};

static pinmuxPerCfg_t gMcu_fss0_hpb0PinCfg[] =
{
    /* MyMCU_FSS0_HPB1 -> MCU_HYPERBUS0_CK -> E20 */
    {
        PIN_MCU_OSPI0_CLK, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMCU_FSS0_HPB1 -> MCU_HYPERBUS0_CSn0 -> F19 */
    {
        PIN_MCU_OSPI0_CSN0, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMCU_FSS0_HPB1 -> MCU_HYPERBUS0_DQ0 -> D20 */
    {
        PIN_MCU_OSPI0_D0, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_FSS0_HPB1 -> MCU_HYPERBUS0_DQ1 -> G19 */
    {
        PIN_MCU_OSPI0_D1, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_FSS0_HPB1 -> MCU_HYPERBUS0_DQ2 -> G20 */
    {
        PIN_MCU_OSPI0_D2, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_FSS0_HPB1 -> MCU_HYPERBUS0_DQ3 -> F20 */
    {
        PIN_MCU_OSPI0_D3, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_FSS0_HPB1 -> MCU_HYPERBUS0_DQ4 -> F21 */
    {
        PIN_MCU_OSPI0_D4, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_FSS0_HPB1 -> MCU_HYPERBUS0_DQ5 -> E21 */
    {
        PIN_MCU_OSPI0_D5, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_FSS0_HPB1 -> MCU_HYPERBUS0_DQ6 -> B22 */
    {
        PIN_MCU_OSPI0_D6, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_FSS0_HPB1 -> MCU_HYPERBUS0_DQ7 -> G21 */
    {
        PIN_MCU_OSPI0_D7, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_FSS0_HPB1 -> MCU_OSPI0_DQS -> D21 */
    {
        PIN_MCU_OSPI0_DQS, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_FSS0_HPB1 -> MCU_OSPI0_LBCLKO -> C21 */
    {
        PIN_MCU_OSPI0_LBCLKO, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMcu_fss0_hpbPinCfg[] =
{
    {0, TRUE, gMcu_fss0_hpb0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gMcu_rgmii1PinCfg[] =
{
    /* MyMCU_RGMII1 -> MCU_RGMII1_RD0 -> B24 */
    {
        PIN_MCU_RGMII1_RD0, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_RGMII1 -> MCU_RGMII1_RD1 -> A24 */
    {
        PIN_MCU_RGMII1_RD1, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_RGMII1 -> MCU_RGMII1_RD2 -> D24 */
    {
        PIN_MCU_RGMII1_RD2, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_RGMII1 -> MCU_RGMII1_RD3 -> A25 */
    {
        PIN_MCU_RGMII1_RD3, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_RGMII1 -> MCU_RGMII1_RXC -> C24 */
    {
        PIN_MCU_RGMII1_RXC, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_RGMII1 -> MCU_RGMII1_RX_CTL -> C25 */
    {
        PIN_MCU_RGMII1_RX_CTL, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_RGMII1 -> MCU_RGMII1_TD0 -> B25 */
    {
        PIN_MCU_RGMII1_TD0, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMCU_RGMII1 -> MCU_RGMII1_TD1 -> A26 */
    {
        PIN_MCU_RGMII1_TD1, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMCU_RGMII1 -> MCU_RGMII1_TD2 -> A27 */
    {
        PIN_MCU_RGMII1_TD2, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMCU_RGMII1 -> MCU_RGMII1_TD3 -> A28 */
    {
        PIN_MCU_RGMII1_TD3, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMCU_RGMII1 -> MCU_RGMII1_TXC -> B26 */
    {
        PIN_MCU_RGMII1_TXC, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMCU_RGMII1 -> MCU_RGMII1_TX_CTL -> B27 */
    {
        PIN_MCU_RGMII1_TX_CTL, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMcu_rgmiiPinCfg[] =
{
    {1, TRUE, gMcu_rgmii1PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gMcu_uart0PinCfg[] =
{
    /* MyMCU_UART0 -> MCU_UART0_CTSn -> H29 */
    {
        PIN_WKUP_GPIO0_14, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_UART0 -> MCU_UART0_RTSn -> J27 */
    {
        PIN_WKUP_GPIO0_15, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMCU_UART0 -> MCU_UART0_RXD -> H28 */
    {
        PIN_WKUP_GPIO0_13, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_UART0 -> MCU_UART0_TXD -> G29 */
    {
        PIN_WKUP_GPIO0_12, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMcu_uartPinCfg[] =
{
    {0, TRUE, gMcu_uart0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gMdio0PinCfg[] =
{
    /* MyMDIO1 -> MDIO0_MDC -> V24 */
    {
        PIN_MDIO0_MDC, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMDIO1 -> MDIO0_MDIO -> V26 */
    {
        PIN_MDIO0_MDIO, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMdioPinCfg[] =
{
    {0, TRUE, gMdio0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gMlb0PinCfg[] =
{
    /* MyMLB0 -> MLB0_MLBCN -> AE2 */
    {
        PIN_MLB0_MLBCN, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMLB0 -> MLB0_MLBCP -> AD2 */
    {
        PIN_MLB0_MLBCP, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMLB0 -> MLB0_MLBDN -> AD3 */
    {
        PIN_MLB0_MLBDN, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMLB0 -> MLB0_MLBDP -> AC3 */
    {
        PIN_MLB0_MLBDP, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMLB0 -> MLB0_MLBSN -> AC1 */
    {
        PIN_MLB0_MLBSN, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyMLB0 -> MLB0_MLBSP -> AD1 */
    {
        PIN_MLB0_MLBSP, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMlbPinCfg[] =
{
    {0, TRUE, gMlb0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gMmcsd1PinCfg[] =
{
    /* MyMMC1 -> MMC1_CLK -> P25 */
    {
        PIN_MMC1_CLK, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    /* MyMMC1 -> MMC1_CMD -> R29 */
    {
        PIN_MMC1_CMD, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    /* MyMMC1 -> MMC1_DAT0 -> R24 */
    {
        PIN_MMC1_DAT0, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    /* MyMMC1 -> MMC1_DAT1 -> P24 */
    {
        PIN_MMC1_DAT1, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    /* MyMMC1 -> MMC1_DAT2 -> R25 */
    {
        PIN_MMC1_DAT2, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    /* MyMMC1 -> MMC1_DAT3 -> R26 */
    {
        PIN_MMC1_DAT3, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    /* MyMMC1 -> MMC1_SDCD -> P23 */
    {
        PIN_MMC1_SDCD, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    /* MyMMC1 -> MMC1_CLKLB */
    {
        PIN_MMC1_CLKLB, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMmcsdPinCfg[] =
{
    {1, TRUE, gMmcsd1PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gSystem0PinCfg[] =
{
    /* MySYSTEM1 -> AUDIO_EXT_REFCLK2 -> W26 */
    {
        PIN_RGMII6_RXC, PIN_MODE(3) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MySYSTEM1 -> OBSCLK0 -> V5 */
    {
        PIN_TIMER_IO1, PIN_MODE(2) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MySYSTEM1 -> PORz_OUT -> U1 */
    {
        PIN_PORZ_OUT, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MySYSTEM1 -> RESETSTATz -> T6 */
    {
        PIN_RESETSTATZ, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MySYSTEM1 -> SOC_SAFETY_ERRORn -> U4 */
    {
        PIN_SOC_SAFETY_ERRORN, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MySYSTEM1 -> SYSCLKOUT0 -> V6 */
    {
        PIN_TIMER_IO0, PIN_MODE(2) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gSystemPinCfg[] =
{
    {0, TRUE, gSystem0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gUart4PinCfg[] =
{
    /* MyUART4 -> UART4_RXD -> W23 */
    {
        PIN_RGMII6_TD3, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyUART4 -> UART4_TXD -> W28 */
    {
        PIN_RGMII6_TD2, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gUart0PinCfg[] =
{
    /* MyUART0 -> UART0_CTSn -> Y3 */
    {
        PIN_SPI1_CS0, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyUART0 -> UART0_RTSn -> AA2 */
    {
        PIN_SPI0_CS0, PIN_MODE(1) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyUART0 -> UART0_RXD -> AB2 */
    {
        PIN_UART0_RXD, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyUART0 -> UART0_TXD -> AB3 */
    {
        PIN_UART0_TXD, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gUart2PinCfg[] =
{
    /* MyUART2 -> UART2_RXD -> Y1 */
    {
        PIN_SPI1_CLK, PIN_MODE(3) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyUART2 -> UART2_TXD -> Y5 */
    {
        PIN_SPI1_D0, PIN_MODE(3) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gUart1PinCfg[] =
{
    /* MyUART1 -> UART1_RXD -> AA4 */
    {
        PIN_UART1_RXD, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyUART1 -> UART1_TXD -> AB4 */
    {
        PIN_UART1_TXD, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gUartPinCfg[] =
{
    {4, TRUE, gUart4PinCfg},
    {0, TRUE, gUart0PinCfg},
    {2, TRUE, gUart2PinCfg},
    {1, TRUE, gUart1PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gUsb1PinCfg[] =
{
    /* MyUSB1 -> USB1_DRVVBUS -> V4 */
    {
        PIN_MCAN1_TX, PIN_MODE(4) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gUsb0PinCfg[] =
{
    /* MyUSB0 -> USB0_DRVVBUS -> U6 */
    {
        PIN_USB0_DRVVBUS, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gUsbPinCfg[] =
{
    {1, TRUE, gUsb1PinCfg},
    {0, TRUE, gUsb0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gWkup_debugss0PinCfg[] =
{
    /* MyWKUP_DEBUG -> EMU0 -> C26 */
    {
        PIN_EMU0, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_DEBUG -> EMU1 -> B29 */
    {
        PIN_EMU1, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_DEBUG -> TCK -> E29 */
    {
        PIN_TCK, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_DEBUG -> TRSTn -> F24 */
    {
        PIN_TRSTN, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gWkup_debugssPinCfg[] =
{
    {0, TRUE, gWkup_debugss0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gWkup_gpio0PinCfg[] =
{
    /* MyWKUP_GPIO0 -> WKUP_GPIO0_0 -> F26 */
    {
        PIN_WKUP_GPIO0_0, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_GPIO0 -> WKUP_GPIO0_1 -> F25 */
    {
        PIN_WKUP_GPIO0_1, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_GPIO0 -> WKUP_GPIO0_2 -> F28 */
    {
        PIN_WKUP_GPIO0_2, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_GPIO0 -> WKUP_GPIO0_3 -> F27 */
    {
        PIN_WKUP_GPIO0_3, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_GPIO0 -> WKUP_GPIO0_6 -> F29 */
    {
        PIN_WKUP_GPIO0_6, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_GPIO0 -> WKUP_GPIO0_7 -> G28 */
    {
        PIN_WKUP_GPIO0_7, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_GPIO0 -> WKUP_GPIO0_8 -> G27 */
    {
        PIN_WKUP_GPIO0_8, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_GPIO0 -> WKUP_GPIO0_9 -> G26 */
    {
        PIN_WKUP_GPIO0_9, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_GPIO0 -> WKUP_GPIO0_17 -> C21 */
    {
        PIN_MCU_OSPI0_LBCLKO, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_GPIO0 -> WKUP_GPIO0_53 -> E24 */
    {
        PIN_MCU_SPI0_D0, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_GPIO0 -> WKUP_GPIO0_54 -> E28 */
    {
        PIN_MCU_SPI0_D1, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_GPIO0 -> WKUP_GPIO0_55 -> E25 */
    {
        PIN_MCU_SPI0_CS0, PIN_MODE(7) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gWkup_gpioPinCfg[] =
{
    {0, TRUE, gWkup_gpio0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gWkup_i2c0PinCfg[] =
{
    /* MyWKUP_I2C0 -> WKUP_I2C0_SCL -> J25 */
    {
        PIN_WKUP_I2C0_SCL, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    /* MyWKUP_I2C0 -> WKUP_I2C0_SDA -> H24 */
    {
        PIN_WKUP_I2C0_SDA, PIN_MODE(0) | \
        ((PIN_PULL_DIRECTION | PIN_INPUT_ENABLE) & (~PIN_PULL_DISABLE))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gWkup_i2cPinCfg[] =
{
    {0, TRUE, gWkup_i2c0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gWkup_system0PinCfg[] =
{
    /* MyWKUP_SYSTEM -> MCU_PORz_OUT -> B28 */
    {
        PIN_MCU_PORZ_OUT, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyWKUP_SYSTEM -> MCU_RESETSTATz -> C27 */
    {
        PIN_MCU_RESETSTATZ, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyWKUP_SYSTEM -> MCU_RESETz -> D28 */
    {
        PIN_MCU_RESETZ, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_SYSTEM -> MCU_SAFETY_ERRORn -> D27 */
    {
        PIN_MCU_SAFETY_ERRORN, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_SYSTEM -> PMIC_POWER_EN1 -> G23 */
    {
        PIN_PMIC_POWER_EN1, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    /* MyWKUP_SYSTEM -> PORz -> J24 */
    {
        PIN_PORZ, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_SYSTEM -> RESET_REQz -> C28 */
    {
        PIN_RESET_REQZ, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gWkup_systemPinCfg[] =
{
    {0, TRUE, gWkup_system0PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gWkup_uart0PinCfg[] =
{
    /* MyWKUP_UART0 -> WKUP_UART0_RXD -> J29 */
    {
        PIN_WKUP_UART0_RXD, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyWKUP_UART0 -> WKUP_UART0_TXD -> J28 */
    {
        PIN_WKUP_UART0_TXD, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE) & (~PIN_PULL_DIRECTION & ~PIN_INPUT_ENABLE))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gWkup_uartPinCfg[] =
{
    {0, TRUE, gWkup_uart0PinCfg},
    {PINMUX_END}
};

static pinmuxPerCfg_t gMcasp2PinCfg[] =
{
    /* MyMCASP2 -> MCASP2_ACLKX -> AA29 */
    {
        PIN_PRG0_PRU1_GPO19, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP2 -> MCASP2_AFSX -> AA26 */
    {
        PIN_PRG0_PRU1_GPO18, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP2 -> MCASP2_AXR3 -> Y25 */
    {
        PIN_PRG0_PRU1_GPO17, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gMcasp0PinCfg[] =
{
    /* MyMCASP10 -> MCASP10_ACLKX -> U23 */
    {
        PIN_RGMII5_TX_CTL, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP10 -> MCASP10_AFSX -> U26 */
    {
        PIN_RGMII5_RX_CTL, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP10 -> MCASP10_AXR0 -> V28 */
    {
        PIN_RGMII5_TD3, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP10 -> MCASP10_AXR1 -> V29 */
    {
        PIN_RGMII5_TD2, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP10 -> MCASP10_AXR2 -> U29 */
    {
        PIN_RGMII5_TXC, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP10 -> MCASP10_AXR3 -> U25 */
    {
        PIN_RGMII5_RXC, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP10 -> MCASP10_AXR4 -> V25 */
    {
        PIN_RGMII6_TD1, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP10 -> MCASP10_AXR5 -> W27 */
    {
        PIN_RGMII6_TD0, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP10 -> MCASP10_AXR6 -> W29 */
    {
        PIN_RGMII6_TXC, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* AUDIO_EXT_REFCLK2 (to PCM3168a) */
    {
        PIN_RGMII6_RXC, PIN_MODE(3) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxPerCfg_t gMcasp1PinCfg[] =
{
    /* MyMCASP11 -> MCASP11_ACLKX -> V27 */
    {
        PIN_RGMII5_TD1, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP11 -> MCASP11_AFSX -> U28 */
    {
        PIN_RGMII5_TD0, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP11 -> MCASP11_AXR0 -> U27 */
    {
        PIN_RGMII5_RD3, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP11 -> MCASP11_AXR1 -> U24 */
    {
        PIN_RGMII5_RD2, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP11 -> MCASP11_AXR2 -> R23 */
    {
        PIN_RGMII5_RD1, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP11 -> MCASP11_AXR3 -> T23 */
    {
        PIN_RGMII5_RD0, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP11 -> MCASP11_AXR4 -> Y29 */
    {
        PIN_RGMII6_RD3, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP11 -> MCASP11_AXR5 -> Y27 */
    {
        PIN_RGMII6_RD2, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP11 -> MCASP11_AXR6 -> W24 */
    {
        PIN_RGMII6_RD1, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCASP11 -> MCASP11_AXR7 -> W25 */
    {
        PIN_RGMII6_RD0, PIN_MODE(12) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMcaspPinCfg[] =
{
    {2, TRUE, gMcasp2PinCfg},
    {0, TRUE, gMcasp0PinCfg},
    {1, TRUE, gMcasp1PinCfg},
    {PINMUX_END}
};


static pinmuxPerCfg_t gMcu_adc0PinCfg[] =
{
    /* MyMCU_ADC0 -> MCU_ADC0_AIN0 -> K25 */
    {
        PIN_MCU_ADC0_AIN0, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_ADC0 -> MCU_ADC0_AIN1 -> K26 */
    {
        PIN_MCU_ADC0_AIN1, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_ADC0 -> MCU_ADC0_AIN2 -> K28 */
    {
        PIN_MCU_ADC0_AIN2, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_ADC0 -> MCU_ADC0_AIN3 -> L28 */
    {
        PIN_MCU_ADC0_AIN3, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_ADC0 -> MCU_ADC0_AIN4 -> K24 */
    {
        PIN_MCU_ADC0_AIN4, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_ADC0 -> MCU_ADC0_AIN5 -> K27 */
    {
        PIN_MCU_ADC0_AIN5, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_ADC0 -> MCU_ADC0_AIN6 -> K29 */
    {
        PIN_MCU_ADC0_AIN6, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    /* MyMCU_ADC0 -> MCU_ADC0_AIN7 -> L29 */
    {
        PIN_MCU_ADC0_AIN7, PIN_MODE(0) | \
        ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION))
    },
    {PINMUX_END}
};

static pinmuxModuleCfg_t gMcu_adcPinCfg[] =
{
    {0, TRUE, gMcu_adc0PinCfg},
    {PINMUX_END}
};


pinmuxBoardCfg_t gJ721E_MainPinmuxData[] =
{
    {0, gDebugssPinCfg},
    {1, gDpPinCfg},
    {2, gGpioPinCfg},
    {3, gI2cPinCfg},
    {4, gMcanPinCfg},
    {5, gMdioPinCfg},
    {6, gMlbPinCfg},
    {7, gMmcsdPinCfg},
    {8, gSystemPinCfg},
    {9, gUartPinCfg},
    {10, gUsbPinCfg},
    {11, gMcaspPinCfg},
    {PINMUX_END}
};

pinmuxBoardCfg_t gJ721E_WkupPinmuxData[] =
{
    {0, gMcu_i2cPinCfg},
    {1, gMcu_i3cPinCfg},
    {2, gMcu_mcanPinCfg},
    {3, gMcu_mdioPinCfg},
    {4, gMcu_fss0_ospiPinCfg},
    {5, gMcu_rgmiiPinCfg},
    {6, gMcu_uartPinCfg},
    {7, gWkup_debugssPinCfg},
    {8, gWkup_gpioPinCfg},
    {9, gWkup_i2cPinCfg},
    {10, gWkup_systemPinCfg},
    {11, gWkup_uartPinCfg},
    {12, gMcu_adcPinCfg},
    {PINMUX_END}
};

pinmuxBoardCfg_t gJ721E_WkupPinmuxDataHpb[] =
{
    {0, gMcu_fss0_hpbPinCfg},
    {PINMUX_END}
};
