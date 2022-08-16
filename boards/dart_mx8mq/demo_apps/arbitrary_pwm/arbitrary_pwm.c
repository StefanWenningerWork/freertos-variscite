/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_pwm.h"

#include "fsl_common.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ARBITRARY_PWM_BASEADDR   PWM2
#define DEMO_ARBITRARY_PWM_IRQn       PWM2_IRQn
#define DEMO_ARBITRARY_PWM_IRQHandler PWM2_IRQHandler
/*! @brief PWM period value. PWMO (Hz) = PCLK(Hz) / (period +2) */
#define PWM_PERIOD_VALUE 30

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void DEMO_ARBITRARY_PWM_IRQHandler(void)
{
    if (PWM_GetStatusFlags(DEMO_ARBITRARY_PWM_BASEADDR) & kPWM_CompareFlag)
    {
        PRINTF("PWM compare occurred\r\n");
        PWM_clearStatusFlags(DEMO_ARBITRARY_PWM_BASEADDR, kPWM_CompareFlag);
    }
    else if (PWM_GetStatusFlags(DEMO_ARBITRARY_PWM_BASEADDR) & kPWM_RolloverFlag)
    {
        PRINTF("PWM rollover occurred\r\n");
        PWM_clearStatusFlags(DEMO_ARBITRARY_PWM_BASEADDR, kPWM_RolloverFlag);
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Main function
 */
int main(void)
{
    pwm_config_t pwmConfig;

    /* Board pin, clock, debug console init */
    /* Board specific RDC settings */
    BOARD_RdcInit();

    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    PRINTF("\r\nArbitrary PWM application example.\r\n");

    /***** 1. Configure desired settings for PWM Control Register *****/
    /*!
     * config->enableStopMode = false;
     * config->enableDozeMode = false;
     * config->enableWaitMode = false;
     * config->enableDebugMode = false;
     * config->clockSource = kPWM_LowFrequencyClock;
     * config->prescale = 0U;
     * config->outputConfig = kPWM_SetAtRolloverAndClearAtcomparison;
     * config->fifoWater = kPWM_FIFOWaterMark_2;
     * config->sampleRepeat = kPWM_EachSampleOnce;
     * config->byteSwap = kPWM_ByteNoSwap;
     * config->halfWordSwap = kPWM_HalfWordNoSwap;
     */
    PWM_GetDefaultConfig(&pwmConfig);
    pwmConfig.outputConfig = kPWM_NoConfigure;
    // pwmConfig.clockSource = kPWM_PeripheralClock; // TODO: prob use highest freq available here
    // pwmConfig.prescale = 0U;

    PWM_Init(DEMO_ARBITRARY_PWM_BASEADDR, &pwmConfig);

    /***** 2. Enable desired interrupts in PWM Interrupt Register *****/
    PWM_EnableInterrupts(DEMO_ARBITRARY_PWM_BASEADDR, kPWM_CompareInterruptEnable | kPWM_RolloverInterruptEnable);

    /***** 3. Load initial samples into PWM Sample Register *****/
    // NOTE: Sample Register is FIFO buffered.
    //       The PWM module will run at the last configured duty-cycle setting if the FIFO is empty.
    // TODO: Setting the FIFO water level to 1 should have the desired effect for setting duty-cycle lenghts once only.
    PWM_SetSampleValue(DEMO_ARBITRARY_PWM_BASEADDR, 10);

    /***** 4. Check (and reset) status bits *****/
    if (PWM_GetStatusFlags(DEMO_ARBITRARY_PWM_BASEADDR))
    {
        PWM_clearStatusFlags(DEMO_ARBITRARY_PWM_BASEADDR,
                             kPWM_FIFOEmptyFlag | kPWM_RolloverFlag | kPWM_CompareFlag | kPWM_FIFOWriteErrorFlag);
    }

    /***** 5. Set desired period in PWM Period Register *****/
    // NOTE: PWM module is 16Bit
    // NOTE: PWMO (HZ) = PCLK(Hz) / period+2
    //       PWMO 1kHZ = 32kHz / 30+2
    PWM_SetPeriodValue(DEMO_ARBITRARY_PWM_BASEADDR, PWM_PERIOD_VALUE);

    /***** 6. Enable PWM *****/
    /* Enable PWM interrupt request */
    EnableIRQ(DEMO_ARBITRARY_PWM_IRQn);

    PWM_StartTimer(DEMO_ARBITRARY_PWM_BASEADDR);

    while (1)
    {
    }
}
