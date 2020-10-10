/*
 * Copyright 2017-2018 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __RTE_DEVICE_H
#define __RTE_DEVICE_H

/* USART instance mapping */
#define LPUART0 CM4__LPUART
#define LPUART1 ADMA__LPUART0
#define LPUART2 ADMA__LPUART1
#define LPUART3 ADMA__LPUART2
#define LPUART4 ADMA__LPUART3

/* Driver name mapping. */
#define RTE_USART0        0
#define RTE_USART0_DMA_EN 0
#define RTE_USART1        0
#define RTE_USART1_DMA_EN 0
#define RTE_USART2        1
#define RTE_USART2_DMA_EN 1
#define RTE_USART3        0
#define RTE_USART3_DMA_EN 0
#define RTE_USART4        0
#define RTE_USART4_DMA_EN 0

/* UART configuration. */
#define USART_RX_BUFFER_LEN 64

#define RTE_USART1_DMA_TX_CH       9
#define RTE_USART1_DMA_TX_PERI_SEL 9
#define RTE_USART1_DMA_TX_DMA_BASE ADMA__EDMA2
#define RTE_USART1_DMA_RX_CH       8
#define RTE_USART1_DMA_RX_PERI_SEL 8
#define RTE_USART1_DMA_RX_DMA_BASE ADMA__EDMA2

#define RTE_USART2_DMA_TX_CH       11
#define RTE_USART2_DMA_TX_PERI_SEL 11
#define RTE_USART2_DMA_TX_DMA_BASE ADMA__EDMA2
#define RTE_USART2_DMA_RX_CH       10
#define RTE_USART2_DMA_RX_PERI_SEL 10
#define RTE_USART2_DMA_RX_DMA_BASE ADMA__EDMA2

#define RTE_USART3_DMA_TX_CH       13
#define RTE_USART3_DMA_TX_PERI_SEL 13
#define RTE_USART3_DMA_TX_DMA_BASE ADMA__EDMA2
#define RTE_USART3_DMA_RX_CH       12
#define RTE_USART3_DMA_RX_PERI_SEL 12
#define RTE_USART3_DMA_RX_DMA_BASE ADMA__EDMA2

#define RTE_USART4_DMA_TX_CH       15
#define RTE_USART4_DMA_TX_PERI_SEL 15
#define RTE_USART4_DMA_TX_DMA_BASE ADMA__EDMA2
#define RTE_USART4_DMA_RX_CH       14
#define RTE_USART4_DMA_RX_PERI_SEL 14
#define RTE_USART4_DMA_RX_DMA_BASE ADMA__EDMA2

#endif /* __RTE_DEVICE_H */
