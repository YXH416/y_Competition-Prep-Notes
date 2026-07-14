/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2500
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for DCC_100_PWM2 */
#define DCC_100_PWM2_INST                                                  TIMG0
#define DCC_100_PWM2_INST_IRQHandler                            TIMG0_IRQHandler
#define DCC_100_PWM2_INST_INT_IRQN                              (TIMG0_INT_IRQn)
#define DCC_100_PWM2_INST_CLK_FREQ                                       5000000
/* GPIO defines for channel 0 */
#define GPIO_DCC_100_PWM2_C0_PORT                                          GPIOA
#define GPIO_DCC_100_PWM2_C0_PIN                                  DL_GPIO_PIN_12
#define GPIO_DCC_100_PWM2_C0_IOMUX                               (IOMUX_PINCM34)
#define GPIO_DCC_100_PWM2_C0_IOMUX_FUNC              IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_DCC_100_PWM2_C0_IDX                             DL_TIMER_CC_0_INDEX

/* Defines for DCC_100_PWM1 */
#define DCC_100_PWM1_INST                                                  TIMA1
#define DCC_100_PWM1_INST_IRQHandler                            TIMA1_IRQHandler
#define DCC_100_PWM1_INST_INT_IRQN                              (TIMA1_INT_IRQn)
#define DCC_100_PWM1_INST_CLK_FREQ                                      10000000
/* GPIO defines for channel 0 */
#define GPIO_DCC_100_PWM1_C0_PORT                                          GPIOA
#define GPIO_DCC_100_PWM1_C0_PIN                                  DL_GPIO_PIN_28
#define GPIO_DCC_100_PWM1_C0_IOMUX                                (IOMUX_PINCM3)
#define GPIO_DCC_100_PWM1_C0_IOMUX_FUNC               IOMUX_PINCM3_PF_TIMA1_CCP0
#define GPIO_DCC_100_PWM1_C0_IDX                             DL_TIMER_CC_0_INDEX




/* Defines for OLED */
#define OLED_INST                                                           I2C1
#define OLED_INST_IRQHandler                                     I2C1_IRQHandler
#define OLED_INST_INT_IRQN                                         I2C1_INT_IRQn
#define OLED_BUS_SPEED_HZ                                                 100000
#define GPIO_OLED_SDA_PORT                                                 GPIOB
#define GPIO_OLED_SDA_PIN                                          DL_GPIO_PIN_3
#define GPIO_OLED_IOMUX_SDA                                      (IOMUX_PINCM16)
#define GPIO_OLED_IOMUX_SDA_FUNC                       IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_OLED_SCL_PORT                                                 GPIOB
#define GPIO_OLED_SCL_PIN                                          DL_GPIO_PIN_2
#define GPIO_OLED_IOMUX_SCL                                      (IOMUX_PINCM15)
#define GPIO_OLED_IOMUX_SCL_FUNC                       IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for PRINT */
#define PRINT_INST                                                         UART0
#define PRINT_INST_FREQUENCY                                            40000000
#define PRINT_INST_IRQHandler                                   UART0_IRQHandler
#define PRINT_INST_INT_IRQN                                       UART0_INT_IRQn
#define GPIO_PRINT_RX_PORT                                                 GPIOA
#define GPIO_PRINT_TX_PORT                                                 GPIOA
#define GPIO_PRINT_RX_PIN                                         DL_GPIO_PIN_11
#define GPIO_PRINT_TX_PIN                                         DL_GPIO_PIN_10
#define GPIO_PRINT_IOMUX_RX                                      (IOMUX_PINCM22)
#define GPIO_PRINT_IOMUX_TX                                      (IOMUX_PINCM21)
#define GPIO_PRINT_IOMUX_RX_FUNC                       IOMUX_PINCM22_PF_UART0_RX
#define GPIO_PRINT_IOMUX_TX_FUNC                       IOMUX_PINCM21_PF_UART0_TX
#define PRINT_BAUD_RATE                                                 (115200)
#define PRINT_IBRD_40_MHZ_115200_BAUD                                       (21)
#define PRINT_FBRD_40_MHZ_115200_BAUD                                       (45)
/* Defines for K230_UART */
#define K230_UART_INST                                                     UART2
#define K230_UART_INST_FREQUENCY                                        40000000
#define K230_UART_INST_IRQHandler                               UART2_IRQHandler
#define K230_UART_INST_INT_IRQN                                   UART2_INT_IRQn
#define GPIO_K230_UART_RX_PORT                                             GPIOA
#define GPIO_K230_UART_TX_PORT                                             GPIOA
#define GPIO_K230_UART_RX_PIN                                     DL_GPIO_PIN_22
#define GPIO_K230_UART_TX_PIN                                     DL_GPIO_PIN_21
#define GPIO_K230_UART_IOMUX_RX                                  (IOMUX_PINCM47)
#define GPIO_K230_UART_IOMUX_TX                                  (IOMUX_PINCM46)
#define GPIO_K230_UART_IOMUX_RX_FUNC                   IOMUX_PINCM47_PF_UART2_RX
#define GPIO_K230_UART_IOMUX_TX_FUNC                   IOMUX_PINCM46_PF_UART2_TX
#define K230_UART_BAUD_RATE                                             (115200)
#define K230_UART_IBRD_40_MHZ_115200_BAUD                                   (21)
#define K230_UART_FBRD_40_MHZ_115200_BAUD                                   (45)





/* Port definition for Pin Group STEP_MOTOR */
#define STEP_MOTOR_PORT                                                  (GPIOA)

/* Defines for DIR2: GPIOA.13 with pinCMx 35 on package pin 6 */
#define STEP_MOTOR_DIR2_PIN                                     (DL_GPIO_PIN_13)
#define STEP_MOTOR_DIR2_IOMUX                                    (IOMUX_PINCM35)
/* Defines for DCY2: GPIOA.14 with pinCMx 36 on package pin 7 */
#define STEP_MOTOR_DCY2_PIN                                     (DL_GPIO_PIN_14)
#define STEP_MOTOR_DCY2_IOMUX                                    (IOMUX_PINCM36)
/* Defines for SLP2: GPIOA.15 with pinCMx 37 on package pin 8 */
#define STEP_MOTOR_SLP2_PIN                                     (DL_GPIO_PIN_15)
#define STEP_MOTOR_SLP2_IOMUX                                    (IOMUX_PINCM37)
/* Defines for RST2: GPIOA.16 with pinCMx 38 on package pin 9 */
#define STEP_MOTOR_RST2_PIN                                     (DL_GPIO_PIN_16)
#define STEP_MOTOR_RST2_IOMUX                                    (IOMUX_PINCM38)
/* Port definition for Pin Group STEP_MOTOR1_A */
#define STEP_MOTOR1_A_PORT                                               (GPIOA)

/* Defines for DIR1: GPIOA.31 with pinCMx 6 on package pin 39 */
#define STEP_MOTOR1_A_DIR1_PIN                                  (DL_GPIO_PIN_31)
#define STEP_MOTOR1_A_DIR1_IOMUX                                  (IOMUX_PINCM6)
/* Defines for RST1: GPIOA.7 with pinCMx 14 on package pin 49 */
#define STEP_MOTOR1_A_RST1_PIN                                   (DL_GPIO_PIN_7)
#define STEP_MOTOR1_A_RST1_IOMUX                                 (IOMUX_PINCM14)
/* Port definition for Pin Group STEP_MOTOR1_B */
#define STEP_MOTOR1_B_PORT                                               (GPIOB)

/* Defines for SLP1: GPIOB.24 with pinCMx 52 on package pin 23 */
#define STEP_MOTOR1_B_SLP1_PIN                                  (DL_GPIO_PIN_24)
#define STEP_MOTOR1_B_SLP1_IOMUX                                 (IOMUX_PINCM52)
/* Defines for DCY1: GPIOB.20 with pinCMx 48 on package pin 19 */
#define STEP_MOTOR1_B_DCY1_PIN                                  (DL_GPIO_PIN_20)
#define STEP_MOTOR1_B_DCY1_IOMUX                                 (IOMUX_PINCM48)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_DCC_100_PWM2_init(void);
void SYSCFG_DL_DCC_100_PWM1_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_PRINT_init(void);
void SYSCFG_DL_K230_UART_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
