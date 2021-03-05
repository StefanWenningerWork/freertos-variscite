/*
 * Copyright 2017-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

#include "board.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/* M40_I2C0_SCL (number AM44), FTDI_M40_UART0_RX */
/* Routed pin properties */
#define BOARD_INITPINS_FTDI_M40_UART0_RX_PERIPHERAL                   M40__UART0   /*!< Peripheral name */
#define BOARD_INITPINS_FTDI_M40_UART0_RX_SIGNAL                          uart_rx   /*!< Signal name */
#define BOARD_INITPINS_FTDI_M40_UART0_RX_PIN_NAME                   M40_I2C0_SCL   /*!< Routed pin name */
#define BOARD_INITPINS_FTDI_M40_UART0_RX_PIN_FUNCTION_ID       SC_P_M40_I2C0_SCL   /*!< Pin function id */
#define BOARD_INITPINS_FTDI_M40_UART0_RX_LABEL               "FTDI_M40_UART0_RX"   /*!< Label */
#define BOARD_INITPINS_FTDI_M40_UART0_RX_NAME                "FTDI_M40_UART0_RX"   /*!< Identifier */

/* M40_I2C0_SDA (number AU51), FTDI_M40_UART0_TX */
/* Routed pin properties */
#define BOARD_INITPINS_FTDI_M40_UART0_TX_PERIPHERAL                   M40__UART0   /*!< Peripheral name */
#define BOARD_INITPINS_FTDI_M40_UART0_TX_SIGNAL                          uart_tx   /*!< Signal name */
#define BOARD_INITPINS_FTDI_M40_UART0_TX_PIN_NAME                   M40_I2C0_SDA   /*!< Routed pin name */
#define BOARD_INITPINS_FTDI_M40_UART0_TX_PIN_FUNCTION_ID       SC_P_M40_I2C0_SDA   /*!< Pin function id */
#define BOARD_INITPINS_FTDI_M40_UART0_TX_LABEL               "FTDI_M40_UART0_TX"   /*!< Label */
#define BOARD_INITPINS_FTDI_M40_UART0_TX_NAME                "FTDI_M40_UART0_TX"   /*!< Identifier */

/* SAI1_TXC (number AU5), SAI1_TXC */
/* Routed pin properties */
#define BOARD_INITPINS_SAI1_TXC_PERIPHERAL                             AUD__SAI1   /*!< Peripheral name */
#define BOARD_INITPINS_SAI1_TXC_SIGNAL                                   sai_txc   /*!< Signal name */
#define BOARD_INITPINS_SAI1_TXC_PIN_NAME                                SAI1_TXC   /*!< Routed pin name */
#define BOARD_INITPINS_SAI1_TXC_PIN_FUNCTION_ID                    SC_P_SAI1_TXC   /*!< Pin function id */
#define BOARD_INITPINS_SAI1_TXC_LABEL                                 "SAI1_TXC"   /*!< Label */
#define BOARD_INITPINS_SAI1_TXC_NAME                                  "SAI1_TXC"   /*!< Identifier */

/* SAI1_TXD (number AU1), SAI1_TXD */
/* Routed pin properties */
#define BOARD_INITPINS_SAI1_TXD_PERIPHERAL                             AUD__SAI1   /*!< Peripheral name */
#define BOARD_INITPINS_SAI1_TXD_SIGNAL                                   sai_txd   /*!< Signal name */
#define BOARD_INITPINS_SAI1_TXD_PIN_NAME                                SAI1_TXD   /*!< Routed pin name */
#define BOARD_INITPINS_SAI1_TXD_PIN_FUNCTION_ID                    SC_P_SAI1_TXD   /*!< Pin function id */
#define BOARD_INITPINS_SAI1_TXD_LABEL                                 "SAI1_TXD"   /*!< Label */
#define BOARD_INITPINS_SAI1_TXD_NAME                                  "SAI1_TXD"   /*!< Identifier */

/* SAI1_TXFS (number AV2), SAI1_TXFS */
/* Routed pin properties */
#define BOARD_INITPINS_SAI1_TXFS_PERIPHERAL                            AUD__SAI1   /*!< Peripheral name */
#define BOARD_INITPINS_SAI1_TXFS_SIGNAL                                 sai_txfs   /*!< Signal name */
#define BOARD_INITPINS_SAI1_TXFS_PIN_NAME                              SAI1_TXFS   /*!< Routed pin name */
#define BOARD_INITPINS_SAI1_TXFS_PIN_FUNCTION_ID                  SC_P_SAI1_TXFS   /*!< Pin function id */
#define BOARD_INITPINS_SAI1_TXFS_LABEL                               "SAI1_TXFS"   /*!< Label */
#define BOARD_INITPINS_SAI1_TXFS_NAME                                "SAI1_TXFS"   /*!< Identifier */

/* MCLK_OUT0 (number BD4), MCLK_OUT0 */
/* Routed pin properties */
#define BOARD_INITPINS_MCLK_OUT0_PERIPHERAL                             AUD__ACM   /*!< Peripheral name */
#define BOARD_INITPINS_MCLK_OUT0_SIGNAL                             acm_mclk_out   /*!< Signal name */
#define BOARD_INITPINS_MCLK_OUT0_CHANNEL                                       0   /*!< Signal channel */
#define BOARD_INITPINS_MCLK_OUT0_PIN_NAME                              MCLK_OUT0   /*!< Routed pin name */
#define BOARD_INITPINS_MCLK_OUT0_PIN_FUNCTION_ID                  SC_P_MCLK_OUT0   /*!< Pin function id */
#define BOARD_INITPINS_MCLK_OUT0_LABEL                               "MCLK_OUT0"   /*!< Label */
#define BOARD_INITPINS_MCLK_OUT0_NAME                                "MCLK_OUT0"   /*!< Identifier */

/* GPT0_CLK (number AY52), I2C1_1V8_SCL */
/* Routed pin properties */
#define BOARD_I2C_CONFIGUREPINS_I2C1_1V8_SCL_PERIPHERAL                DMA__I2C1   /*!< Peripheral name */
#define BOARD_I2C_CONFIGUREPINS_I2C1_1V8_SCL_SIGNAL                      i2c_scl   /*!< Signal name */
#define BOARD_I2C_CONFIGUREPINS_I2C1_1V8_SCL_PIN_NAME                   GPT0_CLK   /*!< Routed pin name */
#define BOARD_I2C_CONFIGUREPINS_I2C1_1V8_SCL_PIN_FUNCTION_ID       SC_P_GPT0_CLK   /*!< Pin function id */
#define BOARD_I2C_CONFIGUREPINS_I2C1_1V8_SCL_LABEL                "I2C1_1V8_SCL"   /*!< Label */
#define BOARD_I2C_CONFIGUREPINS_I2C1_1V8_SCL_NAME                 "I2C1_1V8_SCL"   /*!< Identifier */

/* GPT0_CAPTURE (number AV52), I2C1_1V8_SDA */
/* Routed pin properties */
#define BOARD_I2C_CONFIGUREPINS_I2C1_1V8_SDA_PERIPHERAL                DMA__I2C1   /*!< Peripheral name */
#define BOARD_I2C_CONFIGUREPINS_I2C1_1V8_SDA_SIGNAL                      i2c_sda   /*!< Signal name */
#define BOARD_I2C_CONFIGUREPINS_I2C1_1V8_SDA_PIN_NAME               GPT0_CAPTURE   /*!< Routed pin name */
#define BOARD_I2C_CONFIGUREPINS_I2C1_1V8_SDA_PIN_FUNCTION_ID   SC_P_GPT0_CAPTURE   /*!< Pin function id */
#define BOARD_I2C_CONFIGUREPINS_I2C1_1V8_SDA_LABEL                "I2C1_1V8_SDA"   /*!< Label */
#define BOARD_I2C_CONFIGUREPINS_I2C1_1V8_SDA_NAME                 "I2C1_1V8_SDA"   /*!< Identifier */

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif


/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 * @param ipc scfw ipchandle.
 *
 */
void BOARD_InitPins(sc_ipc_t ipc);                         /*!< Function assigned for the core: Cortex-M4F[cm4_core0] */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 * @param ipc scfw ipchandle.
 *
 */
void BOARD_I2C_ConfigurePins(sc_ipc_t ipc);                /*!< Function assigned for the core: Cortex-M4F[cm4_core0] */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
