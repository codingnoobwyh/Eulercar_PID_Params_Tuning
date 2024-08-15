/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file      iomap.h
  * @author    MCU Driver Team
  * @brief     Defines chip pin map and function mode.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_IOMAP_H
#define McuMagicTag_IOMAP_H

/* get offset value of member in type struct */
#define OFFSET_OF(type, member)  (unsigned int)(&(((type *)0)->member))

#define IOCMG_PIN_MUX(regx, funcNum, regValueDefault) \
        (unsigned int)(((OFFSET_OF(IOConfig_RegStruct, regx) & 0x00000FFF) << 16) | \
                      (((regValueDefault) & 0xFFFFFFF0) | (funcNum)))
/* pin function mode info ---------------------------------------------------- */
#define GPIO0_7_AS_GPIO0_7               IOCMG_PIN_MUX(IOCFG_GPIO0_7, FUNC_MODE_0, 0x02b1)
#define GPIO0_7_AS_JTAG_TRSTN            IOCMG_PIN_MUX(IOCFG_GPIO0_7, FUNC_MODE_1, 0x02b1)
#define GPIO0_7_AS_SPI0_CSN0             IOCMG_PIN_MUX(IOCFG_GPIO0_7, FUNC_MODE_2, 0x02b1)
#define GPIO0_7_AS_UART1_CTSN            IOCMG_PIN_MUX(IOCFG_GPIO0_7, FUNC_MODE_3, 0x02b1)
#define GPIO0_7_AS_CAPM1_IN              IOCMG_PIN_MUX(IOCFG_GPIO0_7, FUNC_MODE_4, 0x02b1)
#define GPIO0_7_AS_POE0                  IOCMG_PIN_MUX(IOCFG_GPIO0_7, FUNC_MODE_5, 0x02b1)
#define GPIO0_7_AS_ACMP0_OUT             IOCMG_PIN_MUX(IOCFG_GPIO0_7, FUNC_MODE_6, 0x02b1)
#define GPIO0_7_AS_ADC_AIN4              IOCMG_PIN_MUX(IOCFG_GPIO0_7, FUNC_MODE_12, 0x02b1)

#define GPIO5_1_AS_GPIO5_1               IOCMG_PIN_MUX(IOCFG_GPIO5_1, FUNC_MODE_0, 0x0230)
#define GPIO5_1_AS_ADC0_STATUS           IOCMG_PIN_MUX(IOCFG_GPIO5_1, FUNC_MODE_5, 0x0230)
#define GPIO5_1_AS_ADC_EXT_TRIG3         IOCMG_PIN_MUX(IOCFG_GPIO5_1, FUNC_MODE_6, 0x0230)
#define GPIO5_1_AS_ADC_AIN5              IOCMG_PIN_MUX(IOCFG_GPIO5_1, FUNC_MODE_12, 0x0230)

#define GPIO2_7_AS_GPIO2_7               IOCMG_PIN_MUX(IOCFG_GPIO2_7, FUNC_MODE_0, 0x0230)
#define GPIO2_7_AS_SPI0_CLK              IOCMG_PIN_MUX(IOCFG_GPIO2_7, FUNC_MODE_2, 0x0230)
#define GPIO2_7_AS_UART1_RTSN            IOCMG_PIN_MUX(IOCFG_GPIO2_7, FUNC_MODE_3, 0x0230)
#define GPIO2_7_AS_PGA0_OUT              IOCMG_PIN_MUX(IOCFG_GPIO2_7, FUNC_MODE_13, 0x0230)

#define GPIO2_6_AS_GPIO2_6               IOCMG_PIN_MUX(IOCFG_GPIO2_6, FUNC_MODE_0, 0x0230)
#define GPIO2_6_AS_SPI0_RXD              IOCMG_PIN_MUX(IOCFG_GPIO2_6, FUNC_MODE_2, 0x0230)
#define GPIO2_6_AS_APT_EVTMP5            IOCMG_PIN_MUX(IOCFG_GPIO2_6, FUNC_MODE_6, 0x0230)
#define GPIO2_6_AS_ADC_AIN6              IOCMG_PIN_MUX(IOCFG_GPIO2_6, FUNC_MODE_12, 0x0230)
#define GPIO2_6_AS_PGA0_N0               IOCMG_PIN_MUX(IOCFG_GPIO2_6, FUNC_MODE_13, 0x0230)
#define GPIO2_6_AS_ACMP_N3               IOCMG_PIN_MUX(IOCFG_GPIO2_6, FUNC_MODE_14, 0x0230)

#define GPIO2_5_AS_GPIO2_5               IOCMG_PIN_MUX(IOCFG_GPIO2_5, FUNC_MODE_0, 0x0230)
#define GPIO2_5_AS_SPI0_TXD              IOCMG_PIN_MUX(IOCFG_GPIO2_5, FUNC_MODE_2, 0x0230)
#define GPIO2_5_AS_APT_EVTIO5            IOCMG_PIN_MUX(IOCFG_GPIO2_5, FUNC_MODE_6, 0x0230)
#define GPIO2_5_AS_ADC_AIN7              IOCMG_PIN_MUX(IOCFG_GPIO2_5, FUNC_MODE_12, 0x0230)
#define GPIO2_5_AS_PGA0_P0               IOCMG_PIN_MUX(IOCFG_GPIO2_5, FUNC_MODE_13, 0x0230)
#define GPIO2_5_AS_ACMP_P3               IOCMG_PIN_MUX(IOCFG_GPIO2_5, FUNC_MODE_14, 0x0230)

#define GPIO5_2_AS_GPIO5_2               IOCMG_PIN_MUX(IOCFG_GPIO5_2, FUNC_MODE_0, 0x0230)
#define GPIO5_2_AS_GPT2_PWM              IOCMG_PIN_MUX(IOCFG_GPIO5_2, FUNC_MODE_1, 0x0230)
#define GPIO5_2_AS_ADC0_STATUS           IOCMG_PIN_MUX(IOCFG_GPIO5_2, FUNC_MODE_5, 0x0230)
#define GPIO5_2_AS_ADC_EXT_TRIG2         IOCMG_PIN_MUX(IOCFG_GPIO5_2, FUNC_MODE_6, 0x0230)
#define GPIO5_2_AS_ADC_AIN8              IOCMG_PIN_MUX(IOCFG_GPIO5_2, FUNC_MODE_12, 0x0230)

#define GPIO3_7_AS_GPIO3_7               IOCMG_PIN_MUX(IOCFG_GPIO3_7, FUNC_MODE_0, 0x0230)
#define GPIO3_7_AS_GPT3_PWM              IOCMG_PIN_MUX(IOCFG_GPIO3_7, FUNC_MODE_1, 0x0230)
#define GPIO3_7_AS_SPI0_CSN1             IOCMG_PIN_MUX(IOCFG_GPIO3_7, FUNC_MODE_2, 0x0230)
#define GPIO3_7_AS_ACMP0_OUT             IOCMG_PIN_MUX(IOCFG_GPIO3_7, FUNC_MODE_6, 0x0230)
#define GPIO3_7_AS_ADC_AIN9              IOCMG_PIN_MUX(IOCFG_GPIO3_7, FUNC_MODE_12, 0x0230)

#define GPIO3_6_AS_GPIO3_6               IOCMG_PIN_MUX(IOCFG_GPIO3_6, FUNC_MODE_0, 0x0230)
#define GPIO3_6_AS_CAN_RX                IOCMG_PIN_MUX(IOCFG_GPIO3_6, FUNC_MODE_1, 0x0230)
#define GPIO3_6_AS_ADC_AIN10             IOCMG_PIN_MUX(IOCFG_GPIO3_6, FUNC_MODE_12, 0x0230)
#define GPIO3_6_AS_ACMP_N4               IOCMG_PIN_MUX(IOCFG_GPIO3_6, FUNC_MODE_13, 0x0230)
#define GPIO3_6_AS_PLL_TEST              IOCMG_PIN_MUX(IOCFG_GPIO3_6, FUNC_MODE_14, 0x0230)
#define GPIO3_6_AS_HOSC_TEST             IOCMG_PIN_MUX(IOCFG_GPIO3_6, FUNC_MODE_15, 0x0230)

#define GPIO3_5_AS_GPIO3_5               IOCMG_PIN_MUX(IOCFG_GPIO3_5, FUNC_MODE_0, 0x0230)
#define GPIO3_5_AS_CAN_TX                IOCMG_PIN_MUX(IOCFG_GPIO3_5, FUNC_MODE_1, 0x0230)
#define GPIO3_5_AS_ADC_EXT_TRIG0         IOCMG_PIN_MUX(IOCFG_GPIO3_5, FUNC_MODE_4, 0x0230)
#define GPIO3_5_AS_ADC_AIN11             IOCMG_PIN_MUX(IOCFG_GPIO3_5, FUNC_MODE_12, 0x0230)
#define GPIO3_5_AS_ACMP_P4               IOCMG_PIN_MUX(IOCFG_GPIO3_5, FUNC_MODE_13, 0x0230)

#define GPIO5_3_AS_GPIO5_3               IOCMG_PIN_MUX(IOCFG_GPIO5_3, FUNC_MODE_0, 0x0230)
#define GPIO5_3_AS_ADC0_STATUS           IOCMG_PIN_MUX(IOCFG_GPIO5_3, FUNC_MODE_5, 0x0230)
#define GPIO5_3_AS_ADC_EXT_TRIG1         IOCMG_PIN_MUX(IOCFG_GPIO5_3, FUNC_MODE_6, 0x0230)
#define GPIO5_3_AS_ADC_AIN12             IOCMG_PIN_MUX(IOCFG_GPIO5_3, FUNC_MODE_12, 0x0230)

#define GPIO1_5_AS_GPIO1_5               IOCMG_PIN_MUX(IOCFG_GPIO1_5, FUNC_MODE_0, 0x0230)
#define GPIO1_5_AS_SMB1_ALERTN           IOCMG_PIN_MUX(IOCFG_GPIO1_5, FUNC_MODE_2, 0x0230)
#define GPIO1_5_AS_UART2_TXD             IOCMG_PIN_MUX(IOCFG_GPIO1_5, FUNC_MODE_3, 0x0230)
#define GPIO1_5_AS_CAPM1_IN              IOCMG_PIN_MUX(IOCFG_GPIO1_5, FUNC_MODE_4, 0x0230)
#define GPIO1_5_AS_ADC_AIN13             IOCMG_PIN_MUX(IOCFG_GPIO1_5, FUNC_MODE_12, 0x0230)
#define GPIO1_5_AS_PGA1_P0               IOCMG_PIN_MUX(IOCFG_GPIO1_5, FUNC_MODE_13, 0x0230)

#define GPIO1_6_AS_GPIO1_6               IOCMG_PIN_MUX(IOCFG_GPIO1_6, FUNC_MODE_0, 0x0230)
#define GPIO1_6_AS_SMB1_SUSN             IOCMG_PIN_MUX(IOCFG_GPIO1_6, FUNC_MODE_2, 0x0230)
#define GPIO1_6_AS_UART2_RXD             IOCMG_PIN_MUX(IOCFG_GPIO1_6, FUNC_MODE_3, 0x0230)
#define GPIO1_6_AS_CAPM2_IN              IOCMG_PIN_MUX(IOCFG_GPIO1_6, FUNC_MODE_4, 0x0230)
#define GPIO1_6_AS_ADC_AIN14             IOCMG_PIN_MUX(IOCFG_GPIO1_6, FUNC_MODE_12, 0x0230)
#define GPIO1_6_AS_PGA1_N0               IOCMG_PIN_MUX(IOCFG_GPIO1_6, FUNC_MODE_13, 0x0230)

#define GPIO1_7_AS_GPIO1_7               IOCMG_PIN_MUX(IOCFG_GPIO1_7, FUNC_MODE_0, 0x0230)
#define GPIO1_7_AS_I2C1_SCL              IOCMG_PIN_MUX(IOCFG_GPIO1_7, FUNC_MODE_2, 0x0230)
#define GPIO1_7_AS_UART2_CTSN            IOCMG_PIN_MUX(IOCFG_GPIO1_7, FUNC_MODE_3, 0x0230)
#define GPIO1_7_AS_CAPM0_IN              IOCMG_PIN_MUX(IOCFG_GPIO1_7, FUNC_MODE_4, 0x0230)
#define GPIO1_7_AS_APT_EVTMP6            IOCMG_PIN_MUX(IOCFG_GPIO1_7, FUNC_MODE_6, 0x0230)
#define GPIO1_7_AS_EF_BIST_SCE           IOCMG_PIN_MUX(IOCFG_GPIO1_7, FUNC_MODE_9, 0x0230)
#define GPIO1_7_AS_PGA1_OUT              IOCMG_PIN_MUX(IOCFG_GPIO1_7, FUNC_MODE_13, 0x0230)

#define GPIO4_7_AS_GPIO4_7               IOCMG_PIN_MUX(IOCFG_GPIO4_7, FUNC_MODE_0, 0x0230)
#define GPIO4_7_AS_POE1                  IOCMG_PIN_MUX(IOCFG_GPIO4_7, FUNC_MODE_1, 0x0230)
#define GPIO4_7_AS_I2C1_SDA              IOCMG_PIN_MUX(IOCFG_GPIO4_7, FUNC_MODE_2, 0x0230)
#define GPIO4_7_AS_UART2_RTSN            IOCMG_PIN_MUX(IOCFG_GPIO4_7, FUNC_MODE_3, 0x0230)
#define GPIO4_7_AS_ADC0_STATUS           IOCMG_PIN_MUX(IOCFG_GPIO4_7, FUNC_MODE_4, 0x0230)
#define GPIO4_7_AS_EF_BIST_SDI           IOCMG_PIN_MUX(IOCFG_GPIO4_7, FUNC_MODE_9, 0x0230)
#define GPIO4_7_AS_ADC_AIN15             IOCMG_PIN_MUX(IOCFG_GPIO4_7, FUNC_MODE_12, 0x0230)
#define GPIO4_7_AS_DAC_OUT               IOCMG_PIN_MUX(IOCFG_GPIO4_7, FUNC_MODE_13, 0x0230)

#define GPIO4_5_AS_GPIO4_5               IOCMG_PIN_MUX(IOCFG_GPIO4_5, FUNC_MODE_0, 0x0230)
#define GPIO4_5_AS_I2C0_SCL              IOCMG_PIN_MUX(IOCFG_GPIO4_5, FUNC_MODE_2, 0x0230)
#define GPIO4_5_AS_UART3_CTSN            IOCMG_PIN_MUX(IOCFG_GPIO4_5, FUNC_MODE_3, 0x0230)
#define GPIO4_5_AS_SPI1_CSN0             IOCMG_PIN_MUX(IOCFG_GPIO4_5, FUNC_MODE_4, 0x0230)
#define GPIO4_5_AS_QDM0_A                IOCMG_PIN_MUX(IOCFG_GPIO4_5, FUNC_MODE_5, 0x0230)

#define GPIO4_6_AS_GPIO4_6               IOCMG_PIN_MUX(IOCFG_GPIO4_6, FUNC_MODE_0, 0x0220)
#define GPIO4_6_AS_I2C0_SDA              IOCMG_PIN_MUX(IOCFG_GPIO4_6, FUNC_MODE_2, 0x0220)
#define GPIO4_6_AS_UART3_RTSN            IOCMG_PIN_MUX(IOCFG_GPIO4_6, FUNC_MODE_3, 0x0220)
#define GPIO4_6_AS_SPI1_CLK              IOCMG_PIN_MUX(IOCFG_GPIO4_6, FUNC_MODE_4, 0x0220)
#define GPIO4_6_AS_QDM0_B                IOCMG_PIN_MUX(IOCFG_GPIO4_6, FUNC_MODE_5, 0x0220)
#define GPIO4_6_AS_EF_BIST_SDO           IOCMG_PIN_MUX(IOCFG_GPIO4_6, FUNC_MODE_9, 0x0220)

#define GPIO1_3_AS_GPIO1_3               IOCMG_PIN_MUX(IOCFG_GPIO1_3, FUNC_MODE_0, 0x0230)
#define GPIO1_3_AS_CAN_RX                IOCMG_PIN_MUX(IOCFG_GPIO1_3, FUNC_MODE_1, 0x0230)
#define GPIO1_3_AS_SMB0_ALERTN           IOCMG_PIN_MUX(IOCFG_GPIO1_3, FUNC_MODE_2, 0x0230)
#define GPIO1_3_AS_UART3_TXD             IOCMG_PIN_MUX(IOCFG_GPIO1_3, FUNC_MODE_3, 0x0230)
#define GPIO1_3_AS_SPI1_TXD              IOCMG_PIN_MUX(IOCFG_GPIO1_3, FUNC_MODE_4, 0x0230)
#define GPIO1_3_AS_QDM0_INDEX            IOCMG_PIN_MUX(IOCFG_GPIO1_3, FUNC_MODE_5, 0x0230)
#define GPIO1_3_AS_QDM0_SYNC             IOCMG_PIN_MUX(IOCFG_GPIO1_3, FUNC_MODE_6, 0x0230)

#define GPIO1_4_AS_GPIO1_4               IOCMG_PIN_MUX(IOCFG_GPIO1_4, FUNC_MODE_0, 0x0230)
#define GPIO1_4_AS_CAN_TX                IOCMG_PIN_MUX(IOCFG_GPIO1_4, FUNC_MODE_1, 0x0230)
#define GPIO1_4_AS_SMB0_SUSN             IOCMG_PIN_MUX(IOCFG_GPIO1_4, FUNC_MODE_2, 0x0230)
#define GPIO1_4_AS_UART3_RXD             IOCMG_PIN_MUX(IOCFG_GPIO1_4, FUNC_MODE_3, 0x0230)
#define GPIO1_4_AS_SPI1_RXD              IOCMG_PIN_MUX(IOCFG_GPIO1_4, FUNC_MODE_4, 0x0230)

#define GPIO3_0_AS_GPIO3_0               IOCMG_PIN_MUX(IOCFG_GPIO3_0, FUNC_MODE_0, 0x0230)
#define GPIO3_0_AS_APT0_PWMA             IOCMG_PIN_MUX(IOCFG_GPIO3_0, FUNC_MODE_1, 0x0230)
#define GPIO3_0_AS_SPI1_CSN1             IOCMG_PIN_MUX(IOCFG_GPIO3_0, FUNC_MODE_4, 0x0230)
#define GPIO3_0_AS_EF_PSW_EN             IOCMG_PIN_MUX(IOCFG_GPIO3_0, FUNC_MODE_8, 0x0230)
#define GPIO3_0_AS_EF_BIST_SCK           IOCMG_PIN_MUX(IOCFG_GPIO3_0, FUNC_MODE_9, 0x0230)
#define GPIO3_0_AS_ANA_CTRL_GPIO0        IOCMG_PIN_MUX(IOCFG_GPIO3_0, FUNC_MODE_10, 0x0230)
#define GPIO3_0_AS_ADC0_GPIO0            IOCMG_PIN_MUX(IOCFG_GPIO3_0, FUNC_MODE_11, 0x0230)

#define GPIO3_1_AS_GPIO3_1               IOCMG_PIN_MUX(IOCFG_GPIO3_1, FUNC_MODE_0, 0x0230)
#define GPIO3_1_AS_APT1_PWMA             IOCMG_PIN_MUX(IOCFG_GPIO3_1, FUNC_MODE_1, 0x0230)
#define GPIO3_1_AS_I2C1_SCL              IOCMG_PIN_MUX(IOCFG_GPIO3_1, FUNC_MODE_2, 0x0230)
#define GPIO3_1_AS_EF_PWR_DROP           IOCMG_PIN_MUX(IOCFG_GPIO3_1, FUNC_MODE_8, 0x0230)
#define GPIO3_1_AS_EF_BIST_SDO           IOCMG_PIN_MUX(IOCFG_GPIO3_1, FUNC_MODE_9, 0x0230)
#define GPIO3_1_AS_ANA_CTRL_GPIO1        IOCMG_PIN_MUX(IOCFG_GPIO3_1, FUNC_MODE_10, 0x0230)
#define GPIO3_1_AS_ADC0_GPIO1            IOCMG_PIN_MUX(IOCFG_GPIO3_1, FUNC_MODE_11, 0x0230)

#define GPIO3_2_AS_GPIO3_2               IOCMG_PIN_MUX(IOCFG_GPIO3_2, FUNC_MODE_0, 0x0230)
#define GPIO3_2_AS_APT2_PWMA             IOCMG_PIN_MUX(IOCFG_GPIO3_2, FUNC_MODE_1, 0x0230)
#define GPIO3_2_AS_I2C1_SDA              IOCMG_PIN_MUX(IOCFG_GPIO3_2, FUNC_MODE_2, 0x0230)
#define GPIO3_2_AS_SPI1_CSN0             IOCMG_PIN_MUX(IOCFG_GPIO3_2, FUNC_MODE_4, 0x0230)
#define GPIO3_2_AS_EF_PORB               IOCMG_PIN_MUX(IOCFG_GPIO3_2, FUNC_MODE_8, 0x0230)
#define GPIO3_2_AS_ADC0_GPIO2            IOCMG_PIN_MUX(IOCFG_GPIO3_2, FUNC_MODE_11, 0x0230)

#define GPIO3_3_AS_GPIO3_3               IOCMG_PIN_MUX(IOCFG_GPIO3_3, FUNC_MODE_0, 0x0230)
#define GPIO3_3_AS_APT3_PWMA             IOCMG_PIN_MUX(IOCFG_GPIO3_3, FUNC_MODE_1, 0x0230)
#define GPIO3_3_AS_POE2                  IOCMG_PIN_MUX(IOCFG_GPIO3_3, FUNC_MODE_2, 0x0230)
#define GPIO3_3_AS_WAKEUP2               IOCMG_PIN_MUX(IOCFG_GPIO3_3, FUNC_MODE_6, 0x0230)
#define GPIO3_3_AS_PMC2HOSC_PD           IOCMG_PIN_MUX(IOCFG_GPIO3_3, FUNC_MODE_11, 0x0230)

#define GPIO4_0_AS_GPIO4_0               IOCMG_PIN_MUX(IOCFG_GPIO4_0, FUNC_MODE_0, 0x0230)
#define GPIO4_0_AS_APT0_PWMB             IOCMG_PIN_MUX(IOCFG_GPIO4_0, FUNC_MODE_1, 0x0230)
#define GPIO4_0_AS_UART3_TXD             IOCMG_PIN_MUX(IOCFG_GPIO4_0, FUNC_MODE_3, 0x0230)
#define GPIO4_0_AS_SPI1_CLK              IOCMG_PIN_MUX(IOCFG_GPIO4_0, FUNC_MODE_4, 0x0230)
#define GPIO4_0_AS_ADC0_GPIO3            IOCMG_PIN_MUX(IOCFG_GPIO4_0, FUNC_MODE_11, 0x0230)

#define GPIO4_1_AS_GPIO4_1               IOCMG_PIN_MUX(IOCFG_GPIO4_1, FUNC_MODE_0, 0x0230)
#define GPIO4_1_AS_APT1_PWMB             IOCMG_PIN_MUX(IOCFG_GPIO4_1, FUNC_MODE_1, 0x0230)
#define GPIO4_1_AS_UART3_RXD             IOCMG_PIN_MUX(IOCFG_GPIO4_1, FUNC_MODE_3, 0x0230)
#define GPIO4_1_AS_SPI1_RXD              IOCMG_PIN_MUX(IOCFG_GPIO4_1, FUNC_MODE_4, 0x0230)
#define GPIO4_1_AS_ADC0_GPIO4            IOCMG_PIN_MUX(IOCFG_GPIO4_1, FUNC_MODE_11, 0x0230)

#define GPIO4_2_AS_GPIO4_2               IOCMG_PIN_MUX(IOCFG_GPIO4_2, FUNC_MODE_0, 0x0230)
#define GPIO4_2_AS_APT2_PWMB             IOCMG_PIN_MUX(IOCFG_GPIO4_2, FUNC_MODE_1, 0x0230)
#define GPIO4_2_AS_I2C0_SCL              IOCMG_PIN_MUX(IOCFG_GPIO4_2, FUNC_MODE_2, 0x0230)
#define GPIO4_2_AS_SPI1_TXD              IOCMG_PIN_MUX(IOCFG_GPIO4_2, FUNC_MODE_4, 0x0230)

#define GPIO4_3_AS_GPIO4_3               IOCMG_PIN_MUX(IOCFG_GPIO4_3, FUNC_MODE_0, 0x0230)
#define GPIO4_3_AS_APT3_PWMB             IOCMG_PIN_MUX(IOCFG_GPIO4_3, FUNC_MODE_1, 0x0230)
#define GPIO4_3_AS_I2C0_SDA              IOCMG_PIN_MUX(IOCFG_GPIO4_3, FUNC_MODE_2, 0x0230)
#define GPIO4_3_AS_SPI1_CSN1             IOCMG_PIN_MUX(IOCFG_GPIO4_3, FUNC_MODE_4, 0x0230)
#define GPIO4_3_AS_SPI0_CSN0             IOCMG_PIN_MUX(IOCFG_GPIO4_3, FUNC_MODE_5, 0x0230)

#define GPIO1_0_AS_GPIO1_0               IOCMG_PIN_MUX(IOCFG_GPIO1_0, FUNC_MODE_0, 0x0230)
#define GPIO1_0_AS_APT0_PWMA             IOCMG_PIN_MUX(IOCFG_GPIO1_0, FUNC_MODE_1, 0x0230)
#define GPIO1_0_AS_UART1_TXD             IOCMG_PIN_MUX(IOCFG_GPIO1_0, FUNC_MODE_3, 0x0230)
#define GPIO1_0_AS_SPI0_CLK              IOCMG_PIN_MUX(IOCFG_GPIO1_0, FUNC_MODE_5, 0x0230)

#define GPIO1_1_AS_GPIO1_1               IOCMG_PIN_MUX(IOCFG_GPIO1_1, FUNC_MODE_0, 0x0230)
#define GPIO1_1_AS_APT1_PWMA             IOCMG_PIN_MUX(IOCFG_GPIO1_1, FUNC_MODE_1, 0x0230)
#define GPIO1_1_AS_UART1_RXD             IOCMG_PIN_MUX(IOCFG_GPIO1_1, FUNC_MODE_3, 0x0230)
#define GPIO1_1_AS_SPI0_RXD              IOCMG_PIN_MUX(IOCFG_GPIO1_1, FUNC_MODE_5, 0x0230)

#define GPIO3_4_AS_GPIO3_4               IOCMG_PIN_MUX(IOCFG_GPIO3_4, FUNC_MODE_0, 0x0230)
#define GPIO3_4_AS_APT2_PWMA             IOCMG_PIN_MUX(IOCFG_GPIO3_4, FUNC_MODE_1, 0x0230)
#define GPIO3_4_AS_I2C1_SCL              IOCMG_PIN_MUX(IOCFG_GPIO3_4, FUNC_MODE_2, 0x0230)
#define GPIO3_4_AS_UART1_CTSN            IOCMG_PIN_MUX(IOCFG_GPIO3_4, FUNC_MODE_3, 0x0230)
#define GPIO3_4_AS_SPI0_TXD              IOCMG_PIN_MUX(IOCFG_GPIO3_4, FUNC_MODE_5, 0x0230)

#define GPIO4_4_AS_GPIO4_4               IOCMG_PIN_MUX(IOCFG_GPIO4_4, FUNC_MODE_0, 0x0230)
#define GPIO4_4_AS_APT3_PWMA             IOCMG_PIN_MUX(IOCFG_GPIO4_4, FUNC_MODE_1, 0x0230)
#define GPIO4_4_AS_I2C1_SDA              IOCMG_PIN_MUX(IOCFG_GPIO4_4, FUNC_MODE_2, 0x0230)
#define GPIO4_4_AS_UART1_RTSN            IOCMG_PIN_MUX(IOCFG_GPIO4_4, FUNC_MODE_3, 0x0230)
#define GPIO4_4_AS_SPI0_CSN1             IOCMG_PIN_MUX(IOCFG_GPIO4_4, FUNC_MODE_5, 0x0230)

#define GPIO2_0_AS_GPIO2_0               IOCMG_PIN_MUX(IOCFG_GPIO2_0, FUNC_MODE_0, 0x0230)
#define GPIO2_0_AS_I2C0_SCL              IOCMG_PIN_MUX(IOCFG_GPIO2_0, FUNC_MODE_1, 0x0230)
#define GPIO2_0_AS_SMB1_ALERTN           IOCMG_PIN_MUX(IOCFG_GPIO2_0, FUNC_MODE_2, 0x0230)
#define GPIO2_0_AS_UART3_TXD             IOCMG_PIN_MUX(IOCFG_GPIO2_0, FUNC_MODE_3, 0x0230)
#define GPIO2_0_AS_CAPM2_IN              IOCMG_PIN_MUX(IOCFG_GPIO2_0, FUNC_MODE_4, 0x0230)
#define GPIO2_0_AS_QDM1_A                IOCMG_PIN_MUX(IOCFG_GPIO2_0, FUNC_MODE_5, 0x0230)
#define GPIO2_0_AS_APT_EVTMP4            IOCMG_PIN_MUX(IOCFG_GPIO2_0, FUNC_MODE_6, 0x0230)

#define GPIO2_1_AS_GPIO2_1               IOCMG_PIN_MUX(IOCFG_GPIO2_1, FUNC_MODE_0, 0x0230)
#define GPIO2_1_AS_I2C0_SDA              IOCMG_PIN_MUX(IOCFG_GPIO2_1, FUNC_MODE_1, 0x0230)
#define GPIO2_1_AS_SMB1_SUSN             IOCMG_PIN_MUX(IOCFG_GPIO2_1, FUNC_MODE_2, 0x0230)
#define GPIO2_1_AS_UART3_RXD             IOCMG_PIN_MUX(IOCFG_GPIO2_1, FUNC_MODE_3, 0x0230)
#define GPIO2_1_AS_CAPM1_IN              IOCMG_PIN_MUX(IOCFG_GPIO2_1, FUNC_MODE_4, 0x0230)
#define GPIO2_1_AS_QDM1_B                IOCMG_PIN_MUX(IOCFG_GPIO2_1, FUNC_MODE_5, 0x0230)
#define GPIO2_1_AS_APT_EVTIO4            IOCMG_PIN_MUX(IOCFG_GPIO2_1, FUNC_MODE_6, 0x0230)

#define GPIO5_0_AS_GPIO5_0               IOCMG_PIN_MUX(IOCFG_GPIO5_0, FUNC_MODE_0, 0x0230)
#define GPIO5_0_AS_GPT2_PWM              IOCMG_PIN_MUX(IOCFG_GPIO5_0, FUNC_MODE_1, 0x0230)
#define GPIO5_0_AS_QDM1_SYNC             IOCMG_PIN_MUX(IOCFG_GPIO5_0, FUNC_MODE_3, 0x0230)
#define GPIO5_0_AS_CAPM0_IN              IOCMG_PIN_MUX(IOCFG_GPIO5_0, FUNC_MODE_4, 0x0230)
#define GPIO5_0_AS_QDM1_INDEX            IOCMG_PIN_MUX(IOCFG_GPIO5_0, FUNC_MODE_5, 0x0230)
#define GPIO5_0_AS_WAKEUP0               IOCMG_PIN_MUX(IOCFG_GPIO5_0, FUNC_MODE_6, 0x0230)
#define GPIO5_0_AS_PMU_CLDO_EN           IOCMG_PIN_MUX(IOCFG_GPIO5_0, FUNC_MODE_11, 0x0230)

#define GPIO2_2_AS_GPIO2_2               IOCMG_PIN_MUX(IOCFG_GPIO2_2, FUNC_MODE_0, 0x0230)
#define GPIO2_2_AS_CAN_RX                IOCMG_PIN_MUX(IOCFG_GPIO2_2, FUNC_MODE_1, 0x0230)
#define GPIO2_2_AS_UART0_TXD             IOCMG_PIN_MUX(IOCFG_GPIO2_2, FUNC_MODE_2, 0x0230)
#define GPIO2_2_AS_UART2_TXD             IOCMG_PIN_MUX(IOCFG_GPIO2_2, FUNC_MODE_3, 0x0230)
#define GPIO2_2_AS_CAPM2_IN              IOCMG_PIN_MUX(IOCFG_GPIO2_2, FUNC_MODE_4, 0x0230)

#define GPIO2_3_AS_GPIO2_3               IOCMG_PIN_MUX(IOCFG_GPIO2_3, FUNC_MODE_0, 0x0230)
#define GPIO2_3_AS_CAN_TX                IOCMG_PIN_MUX(IOCFG_GPIO2_3, FUNC_MODE_1, 0x0230)
#define GPIO2_3_AS_UART0_RXD             IOCMG_PIN_MUX(IOCFG_GPIO2_3, FUNC_MODE_2, 0x0230)
#define GPIO2_3_AS_UART2_RXD             IOCMG_PIN_MUX(IOCFG_GPIO2_3, FUNC_MODE_3, 0x0230)
#define GPIO2_3_AS_CAPM1_IN              IOCMG_PIN_MUX(IOCFG_GPIO2_3, FUNC_MODE_4, 0x0230)

#define GPIO0_0_AS_GPIO0_0               IOCMG_PIN_MUX(IOCFG_GPIO0_0, FUNC_MODE_0, 0x02b1)
#define GPIO0_0_AS_JTAG_TCK              IOCMG_PIN_MUX(IOCFG_GPIO0_0, FUNC_MODE_1, 0x02b1)
#define GPIO0_0_AS_UART0_CTSN            IOCMG_PIN_MUX(IOCFG_GPIO0_0, FUNC_MODE_3, 0x02b1)
#define GPIO0_0_AS_UART2_CTSN            IOCMG_PIN_MUX(IOCFG_GPIO0_0, FUNC_MODE_4, 0x02b1)

#define GPIO0_1_AS_GPIO0_1               IOCMG_PIN_MUX(IOCFG_GPIO0_1, FUNC_MODE_0, 0x0311)
#define GPIO0_1_AS_JTAG_TMS              IOCMG_PIN_MUX(IOCFG_GPIO0_1, FUNC_MODE_1, 0x0311)
#define GPIO0_1_AS_UART0_RTSN            IOCMG_PIN_MUX(IOCFG_GPIO0_1, FUNC_MODE_3, 0x0311)
#define GPIO0_1_AS_UART2_RTSN            IOCMG_PIN_MUX(IOCFG_GPIO0_1, FUNC_MODE_4, 0x0311)

#define GPIO0_2_AS_GPIO0_2               IOCMG_PIN_MUX(IOCFG_GPIO0_2, FUNC_MODE_0, 0x0731)
#define GPIO0_2_AS_RESETN                IOCMG_PIN_MUX(IOCFG_GPIO0_2, FUNC_MODE_1, 0x0731)
#define GPIO0_2_AS_SYS_RSTN_OUT          IOCMG_PIN_MUX(IOCFG_GPIO0_2, FUNC_MODE_2, 0x0731)
#define GPIO0_2_AS_PMC2CORE_POR_N        IOCMG_PIN_MUX(IOCFG_GPIO0_2, FUNC_MODE_8, 0x0731)
#define GPIO0_2_AS_PMC2EF_PWR_DROP       IOCMG_PIN_MUX(IOCFG_GPIO0_2, FUNC_MODE_9, 0x0731)
#define GPIO0_2_AS_PMC2HOSC_PD           IOCMG_PIN_MUX(IOCFG_GPIO0_2, FUNC_MODE_10, 0x0731)
#define GPIO0_2_AS_PMU_CLDO_OK           IOCMG_PIN_MUX(IOCFG_GPIO0_2, FUNC_MODE_11, 0x0731)

#define GPIO0_3_AS_GPIO0_3               IOCMG_PIN_MUX(IOCFG_GPIO0_3, FUNC_MODE_0, 0x0230)
#define GPIO0_3_AS_UART0_TXD             IOCMG_PIN_MUX(IOCFG_GPIO0_3, FUNC_MODE_3, 0x0230)
#define GPIO0_3_AS_XTAL_OUT              IOCMG_PIN_MUX(IOCFG_GPIO0_3, FUNC_MODE_12, 0x0230)

#define GPIO0_4_AS_GPIO0_4               IOCMG_PIN_MUX(IOCFG_GPIO0_4, FUNC_MODE_0, 0x0230)
#define GPIO0_4_AS_GPT0_PWM              IOCMG_PIN_MUX(IOCFG_GPIO0_4, FUNC_MODE_2, 0x0230)
#define GPIO0_4_AS_UART0_RXD             IOCMG_PIN_MUX(IOCFG_GPIO0_4, FUNC_MODE_3, 0x0230)
#define GPIO0_4_AS_XTAL_IN               IOCMG_PIN_MUX(IOCFG_GPIO0_4, FUNC_MODE_12, 0x0230)

#define GPIO2_4_AS_GPIO2_4               IOCMG_PIN_MUX(IOCFG_GPIO2_4, FUNC_MODE_0, 0x0230)
#define GPIO2_4_AS_GPT0_PWM              IOCMG_PIN_MUX(IOCFG_GPIO2_4, FUNC_MODE_2, 0x0230)
#define GPIO2_4_AS_CAPM2_IN              IOCMG_PIN_MUX(IOCFG_GPIO2_4, FUNC_MODE_4, 0x0230)
#define GPIO2_4_AS_WAKEUP3               IOCMG_PIN_MUX(IOCFG_GPIO2_4, FUNC_MODE_6, 0x0230)
#define GPIO2_4_AS_PMC2CORE_POR_N        IOCMG_PIN_MUX(IOCFG_GPIO2_4, FUNC_MODE_11, 0x0230)

#define GPIO1_2_AS_GPIO1_2               IOCMG_PIN_MUX(IOCFG_GPIO1_2, FUNC_MODE_0, 0x06b0)
#define GPIO1_2_AS_UPDATE_MODE           IOCMG_PIN_MUX(IOCFG_GPIO1_2, FUNC_MODE_1, 0x06b0)
#define GPIO1_2_AS_UART2_TXD             IOCMG_PIN_MUX(IOCFG_GPIO1_2, FUNC_MODE_3, 0x06b0)
#define GPIO1_2_AS_TEST_CLK              IOCMG_PIN_MUX(IOCFG_GPIO1_2, FUNC_MODE_5, 0x06b0)
#define GPIO1_2_AS_PMU_TEST              IOCMG_PIN_MUX(IOCFG_GPIO1_2, FUNC_MODE_12, 0x06b0)

#define GPIO5_4_AS_GPIO5_4               IOCMG_PIN_MUX(IOCFG_GPIO5_4, FUNC_MODE_0, 0x0230)
#define GPIO5_4_AS_GPT1_PWM              IOCMG_PIN_MUX(IOCFG_GPIO5_4, FUNC_MODE_1, 0x0230)
#define GPIO5_4_AS_UART2_RXD             IOCMG_PIN_MUX(IOCFG_GPIO5_4, FUNC_MODE_3, 0x0230)
#define GPIO5_4_AS_CAPM0_IN              IOCMG_PIN_MUX(IOCFG_GPIO5_4, FUNC_MODE_4, 0x0230)

#define GPIO5_5_AS_GPIO5_5               IOCMG_PIN_MUX(IOCFG_GPIO5_5, FUNC_MODE_0, 0x0230)
#define GPIO5_5_AS_GPT3_PWM              IOCMG_PIN_MUX(IOCFG_GPIO5_5, FUNC_MODE_1, 0x0230)
#define GPIO5_5_AS_CAPM1_IN              IOCMG_PIN_MUX(IOCFG_GPIO5_5, FUNC_MODE_4, 0x0230)
#define GPIO5_5_AS_WAKEUP1               IOCMG_PIN_MUX(IOCFG_GPIO5_5, FUNC_MODE_6, 0x0230)
#define GPIO5_5_AS_PMC2EF_PWR_DROP       IOCMG_PIN_MUX(IOCFG_GPIO5_5, FUNC_MODE_11, 0x0230)

#define GPIO0_5_AS_GPIO0_5               IOCMG_PIN_MUX(IOCFG_GPIO0_5, FUNC_MODE_0, 0x0221)
#define GPIO0_5_AS_JTAG_TDO              IOCMG_PIN_MUX(IOCFG_GPIO0_5, FUNC_MODE_1, 0x0221)
#define GPIO0_5_AS_GPT1_PWM              IOCMG_PIN_MUX(IOCFG_GPIO0_5, FUNC_MODE_2, 0x0221)
#define GPIO0_5_AS_UART1_RXD             IOCMG_PIN_MUX(IOCFG_GPIO0_5, FUNC_MODE_3, 0x0221)
#define GPIO0_5_AS_CAPM2_IN              IOCMG_PIN_MUX(IOCFG_GPIO0_5, FUNC_MODE_4, 0x0221)
#define GPIO0_5_AS_ADC_AIN2              IOCMG_PIN_MUX(IOCFG_GPIO0_5, FUNC_MODE_12, 0x0221)
#define GPIO0_5_AS_ACMP_P2               IOCMG_PIN_MUX(IOCFG_GPIO0_5, FUNC_MODE_13, 0x0221)

#define GPIO0_6_AS_GPIO0_6               IOCMG_PIN_MUX(IOCFG_GPIO0_6, FUNC_MODE_0, 0x0331)
#define GPIO0_6_AS_JTAG_TDI              IOCMG_PIN_MUX(IOCFG_GPIO0_6, FUNC_MODE_1, 0x0331)
#define GPIO0_6_AS_UART1_TXD             IOCMG_PIN_MUX(IOCFG_GPIO0_6, FUNC_MODE_3, 0x0331)
#define GPIO0_6_AS_CAPM0_IN              IOCMG_PIN_MUX(IOCFG_GPIO0_6, FUNC_MODE_4, 0x0331)
#define GPIO0_6_AS_ADC_AIN3              IOCMG_PIN_MUX(IOCFG_GPIO0_6, FUNC_MODE_12, 0x0331)
#define GPIO0_6_AS_ACMP_N2               IOCMG_PIN_MUX(IOCFG_GPIO0_6, FUNC_MODE_13, 0x0331)
#define GPIO0_6_AS_TSENSOR_OUT           IOCMG_PIN_MUX(IOCFG_GPIO0_6, FUNC_MODE_14, 0x0331)

#endif /* McuMagicTag_IOMAP_H */