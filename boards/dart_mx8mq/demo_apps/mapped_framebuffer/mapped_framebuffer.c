/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
uint32_t volatile * const mem = (uint32_t *)0xB0000000;
size_t len = 800 * 480;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/* colour pattern: BBGGRR */
void draw_frame(uint8_t b, uint8_t g, uint8_t r) {
    PRINTF("start writing to mem region\r\n");
    size_t i;
    for (i = 0; i < len; ++i) {
        /*
         * Somehow this is being written in little endian.
         * unused -> blue
         * red -> green
         * green -> red
         * blue -> unused
         */
        mem[i] = (0x00 << 24) | (r << 16) | (g << 8) | b;
    }
    PRINTF("finished writing mem region\r\n");
    PRINTF("Wrote %u pixels @ %d bytes per pixel\r\n", i, sizeof(uint32_t));
}


/*!
 * @brief Main function
 */
int main(void)
{
    char ch;

    /* Init board hardware. */
    /* Board specific RDC settings */
    BOARD_RdcInit();

    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    PRINTF("mapped framebuffer experiments.\r\n");

    PRINTF("Wait for kernel to have started...\r\n");
    for (int y = 0; y < 10; ++y) {
        SDK_DelayAtLeastUs(1000000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    }

    while (1)
    {
        PRINTF("\r\nEnter colour to be written into mem region (r/g/b/k): ");
        ch = GETCHAR();
        PUTCHAR(ch);
        PRINTF("\r\n");
        switch (ch) {
            case 'r':
                draw_frame(0x00, 0x00, 0xff);
                break;
            case 'g':
                draw_frame(0x00, 0xff, 0x00);
                break;
            case 'b':
                draw_frame(0xff, 0x00, 0x00);
                break;
            case 'k':
                draw_frame(0x00, 0x00, 0x00);
                break;
            default:
                PRINTF("\r\nInvalid option.\r\n");
        }
    }
}
