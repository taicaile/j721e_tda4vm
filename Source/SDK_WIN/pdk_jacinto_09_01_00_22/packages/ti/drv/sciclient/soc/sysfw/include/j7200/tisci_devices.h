/*
 *  Copyright (C) 2017-2023 Texas Instruments Incorporated
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
 * \ingroup TISCI
 * \defgroup tisci_devices tisci_devices
 *
 * DMSC controls the power management, security and resource management
 * of the device.
 *
 *
 * @{
 */
/**
 *
 *  \brief  This file contains:
 *
 *          WARNING!!: Autogenerated file from SYSFW. DO NOT MODIFY!!
 * Data version: 211118_090221
 *
 */
#ifndef SOC_TISCI_DEVICES_H
#define SOC_TISCI_DEVICES_H

#define TISCI_DEV_MCU_ADC0 0U
#define TISCI_DEV_MCU_ADC1 1U
#define TISCI_DEV_ATL0 2U
#define TISCI_DEV_COMPUTE_CLUSTER0 3U
#define TISCI_DEV_A72SS0_CORE0 4U
#define TISCI_DEV_COMPUTE_CLUSTER0_CFG_WRAP 5U
#define TISCI_DEV_COMPUTE_CLUSTER0_CLEC 6U
#define TISCI_DEV_COMPUTE_CLUSTER0_CORE_CORE 7U
#define TISCI_DEV_DDR0 8U
#define TISCI_DEV_COMPUTE_CLUSTER0_DEBUG_WRAP 9U
#define TISCI_DEV_COMPUTE_CLUSTER0_DIVH2_DIVH0 10U
#define TISCI_DEV_COMPUTE_CLUSTER0_DIVP_TFT0 11U
#define TISCI_DEV_COMPUTE_CLUSTER0_DMSC_WRAP 12U
#define TISCI_DEV_COMPUTE_CLUSTER0_EN_MSMC_DOMAIN 13U
#define TISCI_DEV_COMPUTE_CLUSTER0_GIC500SS 14U
#define TISCI_DEV_COMPUTE_CLUSTER0_PBIST_WRAP 17U
#define TISCI_DEV_MCU_CPSW0 18U
#define TISCI_DEV_CPSW0 19U
#define TISCI_DEV_CPT2_AGGR0 20U
#define TISCI_DEV_CPT2_AGGR1 21U
#define TISCI_DEV_WKUP_DMSC0 22U
#define TISCI_DEV_CPT2_AGGR2 23U
#define TISCI_DEV_MCU_CPT2_AGGR0 24U
#define TISCI_DEV_CPT2_AGGR3 25U
#define TISCI_DEV_CPSW_TX_RGMII0 26U
#define TISCI_DEV_STM0 29U
#define TISCI_DEV_DCC0 30U
#define TISCI_DEV_DCC1 31U
#define TISCI_DEV_DCC2 32U
#define TISCI_DEV_DCC3 33U
#define TISCI_DEV_DCC4 34U
#define TISCI_DEV_MCU_TIMER0 35U
#define TISCI_DEV_DCC5 36U
#define TISCI_DEV_DCC6 37U
#define TISCI_DEV_MAIN0 39U
#define TISCI_DEV_WKUP_WAKEUP0 40U
#define TISCI_DEV_MCU_DCC0 44U
#define TISCI_DEV_MCU_DCC1 45U
#define TISCI_DEV_MCU_DCC2 46U
#define TISCI_DEV_TIMER0 49U
#define TISCI_DEV_TIMER1 50U
#define TISCI_DEV_TIMER2 51U
#define TISCI_DEV_TIMER3 52U
#define TISCI_DEV_TIMER4 53U
#define TISCI_DEV_TIMER5 54U
#define TISCI_DEV_TIMER6 55U
#define TISCI_DEV_TIMER7 57U
#define TISCI_DEV_TIMER8 58U
#define TISCI_DEV_TIMER9 59U
#define TISCI_DEV_TIMER10 60U
#define TISCI_DEV_GTC0 61U
#define TISCI_DEV_TIMER11 62U
#define TISCI_DEV_TIMER12 63U
#define TISCI_DEV_TIMER13 64U
#define TISCI_DEV_TIMER14 65U
#define TISCI_DEV_TIMER15 66U
#define TISCI_DEV_TIMER16 67U
#define TISCI_DEV_TIMER17 68U
#define TISCI_DEV_TIMER18 69U
#define TISCI_DEV_TIMER19 70U
#define TISCI_DEV_MCU_TIMER1 71U
#define TISCI_DEV_MCU_TIMER2 72U
#define TISCI_DEV_MCU_TIMER3 73U
#define TISCI_DEV_MCU_TIMER4 74U
#define TISCI_DEV_MCU_TIMER5 75U
#define TISCI_DEV_MCU_TIMER6 76U
#define TISCI_DEV_MCU_TIMER7 77U
#define TISCI_DEV_MCU_TIMER8 78U
#define TISCI_DEV_MCU_TIMER9 79U
#define TISCI_DEV_ECAP0 80U
#define TISCI_DEV_ECAP1 81U
#define TISCI_DEV_ECAP2 82U
#define TISCI_DEV_EHRPWM0 83U
#define TISCI_DEV_EHRPWM1 84U
#define TISCI_DEV_EHRPWM2 85U
#define TISCI_DEV_EHRPWM3 86U
#define TISCI_DEV_EHRPWM4 87U
#define TISCI_DEV_EHRPWM5 88U
#define TISCI_DEV_ELM0 89U
#define TISCI_DEV_EMIF_DATA_0_VD 90U
#define TISCI_DEV_MMCSD0 91U
#define TISCI_DEV_MMCSD1 92U
#define TISCI_DEV_EQEP0 94U
#define TISCI_DEV_EQEP1 95U
#define TISCI_DEV_EQEP2 96U
#define TISCI_DEV_ESM0 97U
#define TISCI_DEV_MCU_ESM0 98U
#define TISCI_DEV_WKUP_ESM0 99U
#define TISCI_DEV_MCU_FSS0 100U
#define TISCI_DEV_MCU_FSS0_FSAS_0 101U
#define TISCI_DEV_MCU_FSS0_HYPERBUS1P0_0 102U
#define TISCI_DEV_MCU_FSS0_OSPI_0 103U
#define TISCI_DEV_MCU_FSS0_OSPI_1 104U
#define TISCI_DEV_GPIO0 105U
#define TISCI_DEV_GPIO2 107U
#define TISCI_DEV_GPIO4 109U
#define TISCI_DEV_GPIO6 111U
#define TISCI_DEV_WKUP_GPIO0 113U
#define TISCI_DEV_WKUP_GPIO1 114U
#define TISCI_DEV_GPMC0 115U
#define TISCI_DEV_I3C0 116U
#define TISCI_DEV_MCU_I3C0 117U
#define TISCI_DEV_MCU_I3C1 118U
#define TISCI_DEV_CMPEVENT_INTRTR0 123U
#define TISCI_DEV_LED0 127U
#define TISCI_DEV_MAIN2MCU_LVL_INTRTR0 128U
#define TISCI_DEV_MAIN2MCU_PLS_INTRTR0 130U
#define TISCI_DEV_GPIOMUX_INTRTR0 131U
#define TISCI_DEV_WKUP_PORZ_SYNC0 132U
#define TISCI_DEV_PSC0 133U
#define TISCI_DEV_TIMESYNC_INTRTR0 136U
#define TISCI_DEV_WKUP_GPIOMUX_INTRTR0 137U
#define TISCI_DEV_WKUP_PSC0 138U
#define TISCI_DEV_PBIST0 139U
#define TISCI_DEV_PBIST1 140U
#define TISCI_DEV_PBIST2 141U
#define TISCI_DEV_MCU_PBIST0 142U
#define TISCI_DEV_MCU_PBIST1 143U
#define TISCI_DEV_MCU_PBIST2 144U
#define TISCI_DEV_WKUP_DDPA0 145U
#define TISCI_DEV_UART0 146U
#define TISCI_DEV_MCU_UART0 149U
#define TISCI_DEV_MCAN14 150U
#define TISCI_DEV_MCAN15 151U
#define TISCI_DEV_MCAN16 152U
#define TISCI_DEV_MCAN17 153U
#define TISCI_DEV_WKUP_VTM0 154U
#define TISCI_DEV_MAIN2WKUPMCU_VD 155U
#define TISCI_DEV_MCAN0 156U
#define TISCI_DEV_BOARD0 157U
#define TISCI_DEV_MCAN1 158U
#define TISCI_DEV_MCAN2 160U
#define TISCI_DEV_MCAN3 161U
#define TISCI_DEV_MCAN4 162U
#define TISCI_DEV_MCAN5 163U
#define TISCI_DEV_MCAN6 164U
#define TISCI_DEV_MCAN7 165U
#define TISCI_DEV_MCAN8 166U
#define TISCI_DEV_MCAN9 167U
#define TISCI_DEV_MCAN10 168U
#define TISCI_DEV_MCAN11 169U
#define TISCI_DEV_MCAN12 170U
#define TISCI_DEV_MCAN13 171U
#define TISCI_DEV_MCU_MCAN0 172U
#define TISCI_DEV_MCU_MCAN1 173U
#define TISCI_DEV_MCASP0 174U
#define TISCI_DEV_MCASP1 175U
#define TISCI_DEV_MCASP2 176U
#define TISCI_DEV_I2C0 187U
#define TISCI_DEV_I2C1 188U
#define TISCI_DEV_I2C2 189U
#define TISCI_DEV_I2C3 190U
#define TISCI_DEV_I2C4 191U
#define TISCI_DEV_I2C5 192U
#define TISCI_DEV_I2C6 193U
#define TISCI_DEV_MCU_I2C0 194U
#define TISCI_DEV_MCU_I2C1 195U
#define TISCI_DEV_WKUP_I2C0 197U
#define TISCI_DEV_NAVSS0 199U
#define TISCI_DEV_NAVSS0_CPTS_0 201U
#define TISCI_DEV_A72SS0_CORE0_0 202U
#define TISCI_DEV_A72SS0_CORE0_1 203U
#define TISCI_DEV_NAVSS0_DTI_0 206U
#define TISCI_DEV_NAVSS0_MODSS_INTA_0 207U
#define TISCI_DEV_NAVSS0_MODSS_INTA_1 208U
#define TISCI_DEV_NAVSS0_UDMASS_INTA_0 209U
#define TISCI_DEV_NAVSS0_PROXY_0 210U
#define TISCI_DEV_NAVSS0_RINGACC_0 211U
#define TISCI_DEV_NAVSS0_UDMAP_0 212U
#define TISCI_DEV_NAVSS0_INTR_ROUTER_0 213U
#define TISCI_DEV_NAVSS0_MAILBOX_0 214U
#define TISCI_DEV_NAVSS0_MAILBOX_1 215U
#define TISCI_DEV_NAVSS0_MAILBOX_2 216U
#define TISCI_DEV_NAVSS0_MAILBOX_3 217U
#define TISCI_DEV_NAVSS0_MAILBOX_4 218U
#define TISCI_DEV_NAVSS0_MAILBOX_5 219U
#define TISCI_DEV_NAVSS0_MAILBOX_6 220U
#define TISCI_DEV_NAVSS0_MAILBOX_7 221U
#define TISCI_DEV_NAVSS0_MAILBOX_8 222U
#define TISCI_DEV_NAVSS0_MAILBOX_9 223U
#define TISCI_DEV_NAVSS0_MAILBOX_10 224U
#define TISCI_DEV_NAVSS0_MAILBOX_11 225U
#define TISCI_DEV_NAVSS0_SPINLOCK_0 226U
#define TISCI_DEV_NAVSS0_MCRC_0 227U
#define TISCI_DEV_NAVSS0_TBU_0 228U
#define TISCI_DEV_NAVSS0_TIMERMGR_0 230U
#define TISCI_DEV_NAVSS0_TIMERMGR_1 231U
#define TISCI_DEV_MCU_NAVSS0 232U
#define TISCI_DEV_MCU_NAVSS0_UDMASS_INTA_0 233U
#define TISCI_DEV_MCU_NAVSS0_PROXY0 234U
#define TISCI_DEV_MCU_NAVSS0_RINGACC0 235U
#define TISCI_DEV_MCU_NAVSS0_UDMAP_0 236U
#define TISCI_DEV_MCU_NAVSS0_INTR_0 237U
#define TISCI_DEV_MCU_NAVSS0_MCRC_0 238U
#define TISCI_DEV_PCIE1 240U
#define TISCI_DEV_R5FSS0 243U
#define TISCI_DEV_R5FSS0_CORE0 245U
#define TISCI_DEV_R5FSS0_CORE1 246U
#define TISCI_DEV_MCU_R5FSS0 249U
#define TISCI_DEV_MCU_R5FSS0_CORE0 250U
#define TISCI_DEV_MCU_R5FSS0_CORE1 251U
#define TISCI_DEV_RTI0 252U
#define TISCI_DEV_RTI1 253U
#define TISCI_DEV_RTI28 258U
#define TISCI_DEV_RTI29 259U
#define TISCI_DEV_MCU_RTI0 262U
#define TISCI_DEV_MCU_RTI1 263U
#define TISCI_DEV_MCU_SA2_UL0 265U
#define TISCI_DEV_MCSPI0 266U
#define TISCI_DEV_MCSPI1 267U
#define TISCI_DEV_MCSPI2 268U
#define TISCI_DEV_MCSPI3 269U
#define TISCI_DEV_MCSPI4 270U
#define TISCI_DEV_MCSPI5 271U
#define TISCI_DEV_MCSPI6 272U
#define TISCI_DEV_MCSPI7 273U
#define TISCI_DEV_MCU_MCSPI0 274U
#define TISCI_DEV_MCU_MCSPI1 275U
#define TISCI_DEV_MCU_MCSPI2 276U
#define TISCI_DEV_UART1 278U
#define TISCI_DEV_UART2 279U
#define TISCI_DEV_UART3 280U
#define TISCI_DEV_UART4 281U
#define TISCI_DEV_UART5 282U
#define TISCI_DEV_UART6 283U
#define TISCI_DEV_UART7 284U
#define TISCI_DEV_UART8 285U
#define TISCI_DEV_UART9 286U
#define TISCI_DEV_WKUP_UART0 287U
#define TISCI_DEV_USB0 288U
#define TISCI_DEV_SERDES_10G1 292U
#define TISCI_DEV_WKUPMCU2MAIN_VD 298U
#define TISCI_DEV_NAVSS0_MODSS 299U
#define TISCI_DEV_NAVSS0_UDMASS 300U
#define TISCI_DEV_NAVSS0_VIRTSS 301U
#define TISCI_DEV_MCU_NAVSS0_MODSS 302U
#define TISCI_DEV_MCU_NAVSS0_UDMASS 303U
#define TISCI_DEV_DEBUGSS_WRAP0 304U
#define TISCI_DEV_FFI_MAIN_INFRA_CBASS_VD 305U
#define TISCI_DEV_FFI_MAIN_IP_CBASS_VD 306U
#define TISCI_DEV_FFI_MAIN_RC_CBASS_VD 307U
#define TISCI_DEV_MCU_TIMER1_CLKSEL_VD 308U
#define TISCI_DEV_MCU_TIMER3_CLKSEL_VD 309U
#define TISCI_DEV_MCU_TIMER5_CLKSEL_VD 310U
#define TISCI_DEV_MCU_TIMER7_CLKSEL_VD 311U
#define TISCI_DEV_MCU_TIMER9_CLKSEL_VD 312U
#define TISCI_DEV_TIMER1_CLKSEL_VD 313U
#define TISCI_DEV_TIMER3_CLKSEL_VD 314U
#define TISCI_DEV_TIMER5_CLKSEL_VD 315U
#define TISCI_DEV_TIMER7_CLKSEL_VD 316U
#define TISCI_DEV_TIMER9_CLKSEL_VD 317U
#define TISCI_DEV_TIMER11_CLKSEL_VD 318U
#define TISCI_DEV_TIMER13_CLKSEL_VD 319U
#define TISCI_DEV_TIMER15_CLKSEL_VD 320U
#define TISCI_DEV_TIMER17_CLKSEL_VD 321U
#define TISCI_DEV_TIMER19_CLKSEL_VD 322U
#define TISCI_DEV_MAIN_PLL8_SEL_EXTWAVE_VD 323U

#endif /* SOC_TISCI_DEVICES_H */

/* @} */
