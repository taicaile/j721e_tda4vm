/**
 * Note: This file was auto-generated by TI PinMux on 5/10/2019.
 *
 * \file   J721E_pinmux.h
 *
 * \brief  This file contains pad configure register offsets and bit-field
 *         value macros for different configurations,
 *
 *           BIT[21]        TXDISABLE       disable the pin's output driver
 *           BIT[18]        RXACTIVE        enable the pin's input buffer (typically kept enabled)
 *           BIT[17]        PULLTYPESEL     set the iternal resistor pull direction high or low (if enabled)
 *           BIT[16]        PULLUDEN        internal resistor disable (0 = enabled / 1 = disabled)
 *           BIT[3:0]       MUXMODE         select the desired function on the given pin
 *
 *  \copyright Copyright (CU) 2019 Texas Instruments Incorporated -
 *             http://www.ti.com/
 */

#ifndef _J721E_PIN_MUX_H_
#define _J721E_PIN_MUX_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/board/src/j721e_evm/include/pinmux.h>
#include <ti/csl/csl_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define PIN_MODE(mode)                  (mode)
#define PINMUX_END                      (-1)

/** \brief Active mode configurations */
/** \brief Resistor enable */
#define PIN_PULL_DISABLE                (0x1U << 16U)
/** \brief Pull direction */
#define PIN_PULL_DIRECTION              (0x1U << 17U)
/** \brief Receiver enable */
#define PIN_INPUT_ENABLE                (0x1U << 18U)
/** \brief Driver disable */
#define PIN_OUTPUT_DISABLE              (0x1U << 21U)
/** \brief Wakeup enable */
#define PIN_WAKEUP_ENABLE               (0x1U << 29U)

/** \brief Pad config register offset in control module */
enum pinMainOffsets
{
    PIN_EXTINTN                = 0x0U,
    PIN_PRG1_PRU0_GPO0         = 0x4U,
    PIN_PRG1_PRU0_GPO1         = 0x8U,
    PIN_PRG1_PRU0_GPO2         = 0xCU,
    PIN_PRG1_PRU0_GPO3         = 0x10U,
    PIN_PRG1_PRU0_GPO4         = 0x14U,
    PIN_PRG1_PRU0_GPO5         = 0x18U,
    PIN_PRG1_PRU0_GPO6         = 0x1CU,
    PIN_PRG1_PRU0_GPO7         = 0x20U,
    PIN_PRG1_PRU0_GPO8         = 0x24U,
    PIN_PRG1_PRU0_GPO9         = 0x28U,
    PIN_PRG1_PRU0_GPO10        = 0x2CU,
    PIN_PRG1_PRU0_GPO11        = 0x30U,
    PIN_PRG1_PRU0_GPO12        = 0x34U,
    PIN_PRG1_PRU0_GPO13        = 0x38U,
    PIN_PRG1_PRU0_GPO14        = 0x3CU,
    PIN_PRG1_PRU0_GPO15        = 0x40U,
    PIN_PRG1_PRU0_GPO16        = 0x44U,
    PIN_PRG1_PRU0_GPO17        = 0x4CU,
    PIN_PRG1_PRU0_GPO18        = 0x50U,
    PIN_PRG1_PRU0_GPO19        = 0x54U,
    PIN_PRG1_PRU1_GPO0         = 0x58U,
    PIN_PRG1_PRU1_GPO1         = 0x5CU,
    PIN_PRG1_PRU1_GPO2         = 0x60U,
    PIN_PRG1_PRU1_GPO3         = 0x64U,
    PIN_PRG1_PRU1_GPO4         = 0x68U,
    PIN_PRG1_PRU1_GPO5         = 0x6CU,
    PIN_PRG1_PRU1_GPO6         = 0x70U,
    PIN_PRG1_PRU1_GPO7         = 0x74U,
    PIN_PRG1_PRU1_GPO8         = 0x78U,
    PIN_PRG1_PRU1_GPO9         = 0x7CU,
    PIN_PRG1_PRU1_GPO10        = 0x80U,
    PIN_PRG1_PRU1_GPO11        = 0x84U,
    PIN_PRG1_PRU1_GPO12        = 0x88U,
    PIN_PRG1_PRU1_GPO13        = 0x8CU,
    PIN_PRG1_PRU1_GPO14        = 0x90U,
    PIN_PRG1_PRU1_GPO15        = 0x94U,
    PIN_PRG1_PRU1_GPO16        = 0x98U,
    PIN_PRG1_PRU1_GPO17        = 0x9CU,
    PIN_PRG1_PRU1_GPO18        = 0xA0U,
    PIN_PRG1_PRU1_GPO19        = 0xA4U,
    PIN_PRG1_MDIO0_MDIO        = 0xA8U,
    PIN_PRG1_MDIO0_MDC         = 0xACU,
    PIN_PRG0_PRU0_GPO0         = 0xB0U,
    PIN_PRG0_PRU0_GPO1         = 0xB4U,
    PIN_PRG0_PRU0_GPO2         = 0xB8U,
    PIN_PRG0_PRU0_GPO3         = 0xBCU,
    PIN_PRG0_PRU0_GPO4         = 0xC0U,
    PIN_PRG0_PRU0_GPO5         = 0xC4U,
    PIN_PRG0_PRU0_GPO6         = 0xC8U,
    PIN_PRG0_PRU0_GPO7         = 0xCCU,
    PIN_PRG0_PRU0_GPO8         = 0xD0U,
    PIN_PRG0_PRU0_GPO9         = 0xD4U,
    PIN_PRG0_PRU0_GPO10        = 0xD8U,
    PIN_PRG0_PRU0_GPO11        = 0xDCU,
    PIN_PRG0_PRU0_GPO12        = 0xE0U,
    PIN_PRG0_PRU0_GPO13        = 0xE4U,
    PIN_PRG0_PRU0_GPO14        = 0xE8U,
    PIN_PRG0_PRU0_GPO15        = 0xECU,
    PIN_PRG0_PRU0_GPO16        = 0xF0U,
    PIN_PRG0_PRU0_GPO17        = 0xF4U,
    PIN_PRG0_PRU0_GPO18        = 0xF8U,
    PIN_PRG0_PRU0_GPO19        = 0xFCU,
    PIN_PRG0_PRU1_GPO0         = 0x100U,
    PIN_PRG0_PRU1_GPO1         = 0x104U,
    PIN_PRG0_PRU1_GPO2         = 0x108U,
    PIN_PRG0_PRU1_GPO3         = 0x10CU,
    PIN_PRG0_PRU1_GPO4         = 0x110U,
    PIN_PRG0_PRU1_GPO5         = 0x114U,
    PIN_PRG0_PRU1_GPO6         = 0x118U,
    PIN_PRG0_PRU1_GPO7         = 0x11CU,
    PIN_PRG0_PRU1_GPO8         = 0x120U,
    PIN_PRG0_PRU1_GPO9         = 0x124U,
    PIN_PRG0_PRU1_GPO10        = 0x128U,
    PIN_PRG0_PRU1_GPO11        = 0x12CU,
    PIN_PRG0_PRU1_GPO12        = 0x130U,
    PIN_PRG0_PRU1_GPO13        = 0x134U,
    PIN_PRG0_PRU1_GPO14        = 0x138U,
    PIN_PRG0_PRU1_GPO15        = 0x13CU,
    PIN_PRG0_PRU1_GPO16        = 0x140U,
    PIN_PRG0_PRU1_GPO17        = 0x144U,
    PIN_PRG0_PRU1_GPO18        = 0x148U,
    PIN_PRG0_PRU1_GPO19        = 0x14CU,
    PIN_PRG0_MDIO0_MDIO        = 0x150U,
    PIN_PRG0_MDIO0_MDC         = 0x154U,
    PIN_RGMII5_TX_CTL          = 0x158U,
    PIN_RGMII5_RX_CTL          = 0x15CU,
    PIN_RGMII5_TD3             = 0x160U,
    PIN_RGMII5_TD2             = 0x164U,
    PIN_RGMII5_TD1             = 0x168U,
    PIN_RGMII5_TD0             = 0x16CU,
    PIN_RGMII5_TXC             = 0x170U,
    PIN_RGMII5_RXC             = 0x174U,
    PIN_RGMII5_RD3             = 0x178U,
    PIN_RGMII5_RD2             = 0x17CU,
    PIN_RGMII5_RD1             = 0x180U,
    PIN_RGMII5_RD0             = 0x184U,
    PIN_RGMII6_TX_CTL          = 0x188U,
    PIN_RGMII6_RX_CTL          = 0x18CU,
    PIN_RGMII6_TD3             = 0x190U,
    PIN_RGMII6_TD2             = 0x194U,
    PIN_RGMII6_TD1             = 0x198U,
    PIN_RGMII6_TD0             = 0x19CU,
    PIN_RGMII6_TXC             = 0x1A0U,
    PIN_RGMII6_RXC             = 0x1A4U,
    PIN_RGMII6_RD3             = 0x1A8U,
    PIN_RGMII6_RD2             = 0x1ACU,
    PIN_RGMII6_RD1             = 0x1B0U,
    PIN_RGMII6_RD0             = 0x1B4U,
    PIN_MDIO0_MDIO             = 0x1B8U,
    PIN_MDIO0_MDC              = 0x1BCU,
    PIN_SPI0_CS0               = 0x1C0U,
    PIN_SPI0_CS1               = 0x1C4U,
    PIN_SPI0_CLK               = 0x1C8U,
    PIN_SPI0_D0                = 0x1CCU,
    PIN_SPI0_D1                = 0x1D0U,
    PIN_SPI1_CS0               = 0x1D4U,
    PIN_SPI1_CS1               = 0x1D8U,
    PIN_SPI1_CLK               = 0x1DCU,
    PIN_SPI1_D0                = 0x1E0U,
    PIN_SPI1_D1                = 0x1E4U,
    PIN_UART0_RXD              = 0x1E8U,
    PIN_UART0_TXD              = 0x1ECU,
    PIN_UART0_CTSN             = 0x1F0U,
    PIN_UART0_RTSN             = 0x1F4U,
    PIN_UART1_RXD              = 0x1F8U,
    PIN_UART1_TXD              = 0x1FCU,
    PIN_UART1_CTSN             = 0x200U,
    PIN_UART1_RTSN             = 0x204U,
    PIN_MCAN0_RX               = 0x208U,
    PIN_MCAN0_TX               = 0x20CU,
    PIN_MCAN1_RX               = 0x210U,
    PIN_MCAN1_TX               = 0x214U,
    PIN_I3C0_SCL               = 0x218U,
    PIN_I3C0_SDA               = 0x21CU,
    PIN_I2C0_SCL               = 0x220U,
    PIN_I2C0_SDA               = 0x224U,
    PIN_I2C1_SCL               = 0x228U,
    PIN_I2C1_SDA               = 0x22CU,
    PIN_ECAP0_IN_APWM_OUT      = 0x230U,
    PIN_EXT_REFCLK1            = 0x234U,
    PIN_TIMER_IO0              = 0x238U,
    PIN_TIMER_IO1              = 0x23CU,
    PIN_MMC1_DAT3              = 0x240U,
    PIN_MMC1_DAT2              = 0x244U,
    PIN_MMC1_DAT1              = 0x248U,
    PIN_MMC1_DAT0              = 0x24CU,
    PIN_MMC1_CLK               = 0x250U,
    PIN_MMC1_CMD               = 0x254U,
    PIN_MMC1_SDCD              = 0x258U,
    PIN_MMC1_SDWP              = 0x25CU,
    PIN_MMC2_DAT3              = 0x260U,
    PIN_MMC2_DAT2              = 0x264U,
    PIN_MMC2_DAT1              = 0x268U,
    PIN_MMC2_DAT0              = 0x26CU,
    PIN_MMC2_CLK               = 0x270U,
    PIN_MMC2_CMD               = 0x274U,
    PIN_RESETSTATZ             = 0x278U,
    PIN_PORZ_OUT               = 0x27CU,
    PIN_SOC_SAFETY_ERRORN      = 0x280U,
    PIN_TDI                    = 0x284U,
    PIN_TDO                    = 0x288U,
    PIN_TMS                    = 0x28CU,
    PIN_USB0_DRVVBUS           = 0x290U,
    PIN_MLB0_MLBSP             = 0x294U,
    PIN_MLB0_MLBSN             = 0x298U,
    PIN_MLB0_MLBDP             = 0x29CU,
    PIN_MLB0_MLBDN             = 0x2A0U,
    PIN_MLB0_MLBCP             = 0x2A4U,
    PIN_MLB0_MLBCN             = 0x2A8U,
    PIN_MMC1_CLKLB             = 0x2ACU,
    PIN_MMC2_CLKLB             = 0x2B0U,
};

enum pinWkupOffsets
{
    PIN_MCU_OSPI0_CLK          = 0x0U,
    PIN_MCU_OSPI0_LBCLKO       = 0x4U,
    PIN_MCU_OSPI0_DQS          = 0x8U,
    PIN_MCU_OSPI0_D0           = 0xCU,
    PIN_MCU_OSPI0_D1           = 0x10U,
    PIN_MCU_OSPI0_D2           = 0x14U,
    PIN_MCU_OSPI0_D3           = 0x18U,
    PIN_MCU_OSPI0_D4           = 0x1CU,
    PIN_MCU_OSPI0_D5           = 0x20U,
    PIN_MCU_OSPI0_D6           = 0x24U,
    PIN_MCU_OSPI0_D7           = 0x28U,
    PIN_MCU_OSPI0_CSN0         = 0x2CU,
    PIN_MCU_OSPI0_CSN1         = 0x30U,
    PIN_MCU_OSPI1_CLK          = 0x34U,
    PIN_MCU_OSPI1_LBCLKO       = 0x38U,
    PIN_MCU_OSPI1_DQS          = 0x3CU,
    PIN_MCU_OSPI1_D0           = 0x40U,
    PIN_MCU_OSPI1_D1           = 0x44U,
    PIN_MCU_OSPI1_D2           = 0x48U,
    PIN_MCU_OSPI1_D3           = 0x4CU,
    PIN_MCU_OSPI1_CSN0         = 0x50U,
    PIN_MCU_OSPI1_CSN1         = 0x54U,
    PIN_MCU_RGMII1_TX_CTL      = 0x58U,
    PIN_MCU_RGMII1_RX_CTL      = 0x5CU,
    PIN_MCU_RGMII1_TD3         = 0x60U,
    PIN_MCU_RGMII1_TD2         = 0x64U,
    PIN_MCU_RGMII1_TD1         = 0x68U,
    PIN_MCU_RGMII1_TD0         = 0x6CU,
    PIN_MCU_RGMII1_TXC         = 0x70U,
    PIN_MCU_RGMII1_RXC         = 0x74U,
    PIN_MCU_RGMII1_RD3         = 0x78U,
    PIN_MCU_RGMII1_RD2         = 0x7CU,
    PIN_MCU_RGMII1_RD1         = 0x80U,
    PIN_MCU_RGMII1_RD0         = 0x84U,
    PIN_MCU_MDIO0_MDIO         = 0x88U,
    PIN_MCU_MDIO0_MDC          = 0x8CU,
    PIN_MCU_SPI0_CLK           = 0x90U,
    PIN_MCU_SPI0_D0            = 0x94U,
    PIN_MCU_SPI0_D1            = 0x98U,
    PIN_MCU_SPI0_CS0           = 0x9CU,
    PIN_WKUP_UART0_RXD         = 0xA0U,
    PIN_WKUP_UART0_TXD         = 0xA4U,
    PIN_MCU_MCAN0_TX           = 0xA8U,
    PIN_MCU_MCAN0_RX           = 0xACU,
    PIN_WKUP_GPIO0_0           = 0xB0U,
    PIN_WKUP_GPIO0_1           = 0xB4U,
    PIN_WKUP_GPIO0_2           = 0xB8U,
    PIN_WKUP_GPIO0_3           = 0xBCU,
    PIN_WKUP_GPIO0_4           = 0xC0U,
    PIN_WKUP_GPIO0_5           = 0xC4U,
    PIN_WKUP_GPIO0_6           = 0xC8U,
    PIN_WKUP_GPIO0_7           = 0xCCU,
    PIN_WKUP_GPIO0_8           = 0xD0U,
    PIN_WKUP_GPIO0_9           = 0xD4U,
    PIN_WKUP_GPIO0_10          = 0xD8U,
    PIN_WKUP_GPIO0_11          = 0xDCU,
    PIN_WKUP_GPIO0_12          = 0xE0U,
    PIN_WKUP_GPIO0_13          = 0xE4U,
    PIN_WKUP_GPIO0_14          = 0xE8U,
    PIN_WKUP_GPIO0_15          = 0xECU,
    PIN_MCU_I3C0_SCL           = 0xF0U,
    PIN_MCU_I3C0_SDA           = 0xF4U,
    PIN_WKUP_I2C0_SCL          = 0xF8U,
    PIN_WKUP_I2C0_SDA          = 0xFCU,
    PIN_MCU_I2C0_SCL           = 0x100U,
    PIN_MCU_I2C0_SDA           = 0x104U,
    PIN_PMIC_POWER_EN0         = 0x108U,
    PIN_PMIC_POWER_EN1         = 0x10CU,
    PIN_MCU_SAFETY_ERRORN      = 0x110U,
    PIN_MCU_RESETZ             = 0x114U,
    PIN_MCU_RESETSTATZ         = 0x118U,
    PIN_MCU_PORZ_OUT           = 0x11CU,
    PIN_TCK                    = 0x120U,
    PIN_TRSTN                  = 0x124U,
    PIN_EMU0                   = 0x128U,
    PIN_EMU1                   = 0x12CU,
    PIN_MCU_ADC0_AIN0          = 0x130U,
    PIN_MCU_ADC0_AIN1          = 0x134U,
    PIN_MCU_ADC0_AIN2          = 0x138U,
    PIN_MCU_ADC0_AIN3          = 0x13CU,
    PIN_MCU_ADC0_AIN4          = 0x140U,
    PIN_MCU_ADC0_AIN5          = 0x144U,
    PIN_MCU_ADC0_AIN6          = 0x148U,
    PIN_MCU_ADC0_AIN7          = 0x14CU,
    PIN_MCU_ADC1_AIN0          = 0x150U,
    PIN_MCU_ADC1_AIN1          = 0x154U,
    PIN_MCU_ADC1_AIN2          = 0x158U,
    PIN_MCU_ADC1_AIN3          = 0x15CU,
    PIN_MCU_ADC1_AIN4          = 0x160U,
    PIN_MCU_ADC1_AIN5          = 0x164U,
    PIN_MCU_ADC1_AIN6          = 0x168U,
    PIN_MCU_ADC1_AIN7          = 0x16CU,
    PIN_RESET_REQZ             = 0x170U,
    PIN_PORZ                   = 0x174U,
};

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Pinmux configuration data for the board. Auto-generated from
           Pinmux tool. */
extern pinmuxBoardCfg_t gJ721E_MainPinmuxData[];
extern pinmuxBoardCfg_t gJ721E_WkupPinmuxData[];
extern pinmuxBoardCfg_t gJ721E_MainPinmuxDataGesiIcssg[];
extern pinmuxBoardCfg_t gJ721E_WkupPinmuxDataGesiIcssg[];
extern pinmuxBoardCfg_t gJ721E_MainPinmuxDataInfo[];
extern pinmuxBoardCfg_t gJ721E_WkupPinmuxDataInfo[];
extern pinmuxBoardCfg_t gJ721E_MainPinmuxDataGesiCpsw9g[];
extern pinmuxBoardCfg_t gJ721E_WkupPinmuxDataGesiCpsw9g[];
extern pinmuxBoardCfg_t gJ721E_MainPinmuxDataGesiCpsw9gSgmii[];
extern pinmuxBoardCfg_t gJ721E_MainPinmuxDataGesiCpsw9gQsgmii[];
extern pinmuxBoardCfg_t gJ721E_WkupPinmuxDataHpb[];


#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _J721E_PIN_MUX_H_ */
