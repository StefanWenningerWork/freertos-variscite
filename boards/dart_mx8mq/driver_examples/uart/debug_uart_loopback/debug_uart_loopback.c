/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_uart.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define M4_OUT_UART          UART3
#define M4_OUT_UART_CLK_FREQ BOARD_DEBUG_UART_CLK_FREQ
#define M4_OUT_UART_BAUDRATE 115200U

#define A53_LOG_UART    UART1

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t txbuff[]   = "Uart polling example\r\nBoard will send back received characters\r\n";
uint8_t rxbuff[20] = {0};
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    uint8_t ch;
    status_t status;
    uart_config_t config;

    /* Board specific RDC settings */
    BOARD_RdcInit();

    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitMemory();

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUART_ParityDisabled;
     * config.dataBitsCount = kUART_EightDataBits;
     * config.stopBitCount = kUART_OneStopBit;
     * config.txFifoWatermark = 2;
     * config.rxFifoWatermark = 1;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = M4_OUT_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;

    status = UART_Init(M4_OUT_UART, &config, M4_OUT_UART_CLK_FREQ);
    if (kStatus_Success != status)
    {
        return kStatus_Fail;
    }

    UART_WriteBlocking(M4_OUT_UART, txbuff, sizeof(txbuff) - 1);

    while (1)
    {
        UART_ReadBlocking(A53_LOG_UART, &ch, 1);
        UART_WriteBlocking(M4_OUT_UART, &ch, 1);
    }
}
