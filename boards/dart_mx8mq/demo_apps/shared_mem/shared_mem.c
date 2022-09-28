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

//#include <sys/mman.h>
//#include <fcntl.h>
//#include <unistd.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */
    /* Board specific RDC settings */
    BOARD_RdcInit();

    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    PRINTF("shared mem experiments.\r\n");

    PRINTF("Wait for kernel to have started...\r\n");
    for (int y = 0; y < 10; ++y) {
        SDK_DelayAtLeastUs(1000000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    }

    unsigned char volatile * const mem = (unsigned char *)0xB0000000;
    size_t len = 0x177000;

    PRINTF("start writing shared memory\r\n");
    size_t i;
    for (i = 0; i < len; ++i) {
        mem[i] = 0xAA;
    }
    PRINTF("finished writing shared memory\r\n");
    PRINTF("Wrote %u bytes\r\n", i);

    /*
    void* addr = NULL;
    size_t len = 0x177000;
    off_t offset = 0x40500000;

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("Can't open /dev/mem");
        return -1;
    }

    unsigned char* mem = mmap(addr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
    if (mem == MAP_FAILED) {
        perror("Can't map memory");
        return -1;
    }

    size_t i;
    for (i = 0; i < 255; ++i) {
        mem[i] = i;
    }

    close(fd);
    */

    while (1)
    {
        ;;
    }
}
