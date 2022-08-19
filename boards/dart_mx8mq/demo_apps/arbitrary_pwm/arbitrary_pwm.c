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
#include "fsl_gpio.h"
#include "fsl_clock.h"

#include "fsl_common.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ARBITRARY_PWM_BASEADDR   PWM2
#define DEMO_ARBITRARY_PWM_IRQn       PWM2_IRQn
#define DEMO_ARBITRARY_PWM_IRQHandler PWM2_IRQHandler
/*! @brief PWM period value. PWMO (Hz) = PCLK(Hz) / (period +2) */
#define PWM_PERIOD_VALUE 30

#define EXAMPLE_LED_GPIO     GPIO3
#define EXAMPLE_LED_GPIO_PIN 16U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
pwm_config_t pwmConfig;

/*******************************************************************************
 * Code
 ******************************************************************************/

void DEMO_ARBITRARY_PWM_IRQHandler(void)
{
    if (PWM_GetStatusFlags(DEMO_ARBITRARY_PWM_BASEADDR) & kPWM_CompareFlag)
    {
        GPIO_PinWrite(EXAMPLE_LED_GPIO, EXAMPLE_LED_GPIO_PIN, 1U);
        PWM_clearStatusFlags(DEMO_ARBITRARY_PWM_BASEADDR, kPWM_CompareFlag);
    }
    else if (PWM_GetStatusFlags(DEMO_ARBITRARY_PWM_BASEADDR) & kPWM_RolloverFlag)
    {
        GPIO_PinWrite(EXAMPLE_LED_GPIO, EXAMPLE_LED_GPIO_PIN, 0U);
        PWM_clearStatusFlags(DEMO_ARBITRARY_PWM_BASEADDR, kPWM_RolloverFlag);
    }
    SDK_ISR_EXIT_BARRIER;
}

void pulse(uint64_t length_ns) {
    /* 1. Stop PWM timer */
    // NOTE: no two pulses can happen at the same time
    //       maybe the kernel char device can block during the pulse and not allow another thread to access char device at the same time
    //       PWM does not block, maybe have another rpmsg call "finished?" that checks a flag set by rollover ISR. Kernel driver blocks until that is set.
    PWM_StopTimer(DEMO_ARBITRARY_PWM_BASEADDR);

    /* 2. Setup Clock and PWM */
    if (length_ns >= 999000000) {   //999 - 1s
        pwmConfig.clockSource = kPWM_LowFrequencyClock;
        pwmConfig.prescale = 4000U;
        PWM_Init(DEMO_ARBITRARY_PWM_BASEADDR, &pwmConfig);

        uint64_t counter_steps = length_ns / 1000000000;
        // NOTE: PWM module is 16Bit
        // NOTE: PWMO (HZ) = PCLK(Hz) / period+2
        //       PWMO 1kHZ = 32kHz / 30+2
        // NOTE: PWM runs two extra steps. These steps are put on top of the desired steps.
        // NOTE: original idea was to have the two extra steps infront of the set high ISR.
        //       this is will cause the pulse to be delayed by up to 2s in the seconds range
        PWM_SetPeriodValue(DEMO_ARBITRARY_PWM_BASEADDR, counter_steps);
    }
    else if (length_ns >= 999000) { //999 - 1ms
        pwmConfig.clockSource = kPWM_LowFrequencyClock;
        pwmConfig.prescale = 32U;
        PWM_Init(DEMO_ARBITRARY_PWM_BASEADDR, &pwmConfig);
    }
    else if (length_ns >= 999) {    //999 - 1us
        pwmConfig.clockSource = kPWM_PeripheralClock;
        pwmConfig.prescale = 1U;
        PWM_Init(DEMO_ARBITRARY_PWM_BASEADDR, &pwmConfig);

        CLOCK_UpdateRoot(kClock_RootPwm2, kCLOCK_PwmRootmuxOsc25m, 1U, 25U);
    }
    else {                          //999 - 1ns
        pwmConfig.clockSource = kPWM_PeripheralClock;
        pwmConfig.prescale = 1U;
        PWM_Init(DEMO_ARBITRARY_PWM_BASEADDR, &pwmConfig);

        CLOCK_UpdateRoot(kClock_RootPwm2, kCLOCK_PwmRootmuxSystemPll3, 1U, 1U);
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    /* Board pin, clock, debug console init */
    /* Board specific RDC settings */
    BOARD_RdcInit();

    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    PRINTF("\r\nArbitrary PWM application example.\r\n");

    /* Init output LED GPIO. */
    GPIO_PinInit(EXAMPLE_LED_GPIO, EXAMPLE_LED_GPIO_PIN, &led_config);

    /***** 1. Configure desired settings for PWM Control Register *****/
    /* default config:
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
    pwmConfig.fifoWater = kPWM_FIFOWaterMark_1;

    PWM_Init(DEMO_ARBITRARY_PWM_BASEADDR, &pwmConfig);

    /***** 2. Enable desired interrupts in PWM Interrupt Register *****/
    PWM_EnableInterrupts(DEMO_ARBITRARY_PWM_BASEADDR, kPWM_CompareInterruptEnable | kPWM_RolloverInterruptEnable);

    /***** 3. Load initial samples into PWM Sample Register *****/
    // NOTE: Sample Register is FIFO buffered.
    //       The PWM module will run at the last configured duty-cycle setting if the FIFO is empty.
    PWM_SetSampleValue(DEMO_ARBITRARY_PWM_BASEADDR, 2);

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
