/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"

#include "fsl_pwm.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_uart.h"
#include "rsc_table.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RPMSG_LITE_LINK_ID            (RL_PLATFORM_IMX8MQ_M4_USER_LINK_ID)
#define RPMSG_LITE_SHMEM_BASE         (VDEV0_VRING_BASE)
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-openamp-demo-channel"
#define RPMSG_LITE_MASTER_IS_LINUX

#define APP_DEBUG_UART_BAUDRATE (115200U) /* Debug console baud rate. */
#define APP_TASK_STACK_SIZE (256U)
#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30U)
#endif
#define APP_RPMSG_READY_EVENT_DATA (1U)

#ifdef RPMSG_LITE_MASTER_IS_LINUX   // TODO: is this necessary?
static char helloMsg[13];
#endif /* RPMSG_LITE_MASTER_IS_LINUX */

#define DEMO_ARBITRARY_PWM_BASEADDR   PWM2
#define DEMO_ARBITRARY_PWM_IRQn       PWM2_IRQn
#define DEMO_ARBITRARY_PWM_IRQHandler PWM2_IRQHandler

#define EXAMPLE_LED_GPIO     GPIO3  // TODO: this should not be necessary anymore soon
#define EXAMPLE_LED_GPIO_PIN 16U

typedef enum {
    APWM_OPERATION_SETUP            = (uint32_t)0,
    APWM_OPERATION_FIRE             = (uint32_t)1,
    APWM_OPERATION_MAKE_ENUM_32BIT  = (uint32_t)0xffffffff  // hack to have operation be the same size in A53 and M4
} APWM_OPERATION;

typedef struct {
    //GPIO_Type *base;    // TODO: this needs to be converted from input tp GPIO_Type
    uint32_t base;
    uint32_t pin;
} GPIO_PIN;

typedef struct {
    GPIO_PIN gpio;                  //4B+4B
    uint32_t pulse_length_ns_HIGH;  //4B
    uint32_t pulse_length_ns_LOW;   //4B
    APWM_OPERATION operation;       //4B
} APWM_INSTRUCTION;

static volatile APWM_INSTRUCTION instruction = {
    .pulse_length_ns_HIGH = 0,
    .pulse_length_ns_LOW = 3333,
    .gpio = {.base = 4, .pin = 23},
    .operation = APWM_OPERATION_SETUP
};

GPIO_Type *bases[] = {GPIO1, GPIO2, GPIO3, GPIO4, GPIO5};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static TaskHandle_t app_task_handle = NULL;
pwm_config_t pwmConfig;
gpio_pin_config_t default_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
GPIO_Type* curr_base;
uint32_t curr_pin;

/*******************************************************************************
 * Code
 ******************************************************************************/
void DEMO_ARBITRARY_PWM_IRQHandler(void) {
    if (PWM_GetStatusFlags(DEMO_ARBITRARY_PWM_BASEADDR) & kPWM_CompareFlag)
    {
        PRINTF("Set pin low\r\n");
        GPIO_PinWrite(curr_base, curr_pin, 0U);
        PWM_clearStatusFlags(DEMO_ARBITRARY_PWM_BASEADDR, kPWM_CompareFlag);
    }

    PRINTF("Stop timer\r\n");
    PWM_StopTimer(DEMO_ARBITRARY_PWM_BASEADDR);
    SDK_ISR_EXIT_BARRIER;
}

static void app_nameservice_isr_cb(uint32_t new_ept, const char *new_ept_name, uint32_t flags, void *user_data)
{
}

#ifdef MCMGR_USED
/*!
 * @brief Application-specific implementation of the SystemInitHook() weak function.
 */
void SystemInitHook(void)
{
    /* Initialize MCMGR - low level multicore management library. Call this
       function as close to the reset entry as possible to allow CoreUp event
       triggering. The SystemInitHook() weak function overloading is used in this
       application. */
    (void)MCMGR_EarlyInit();
}
#endif /* MCMGR_USED */

void pulse(GPIO_Type* base, uint32_t pin, uint64_t length_ns) {
    PRINTF("Starting pulse config...\r\n");
    uint64_t counter_steps = 0;
    /* 1. Stop PWM timer */
    // NOTE: no two pulses can happen at the same time
    //       maybe the kernel char device can block during the pulse and not allow another thread to access char device at the same time
    //       PWM does not block, maybe have another rpmsg call "finished?" that checks a flag set by rollover ISR. Kernel driver blocks until that is set.
    PWM_StopTimer(DEMO_ARBITRARY_PWM_BASEADDR);

    /* 2. Setup Clock and PWM */
    if (length_ns > 999000000) {   //999 - 1s
        pwmConfig.clockSource = kPWM_LowFrequencyClock;
        pwmConfig.prescale = 4000U;
        PWM_Init(DEMO_ARBITRARY_PWM_BASEADDR, &pwmConfig);

        counter_steps = (length_ns / 1000000000) * 8;
    }
    else if (length_ns > 999000) { //999 - 1ms
        pwmConfig.clockSource = kPWM_LowFrequencyClock;
        pwmConfig.prescale = 32U;
        PWM_Init(DEMO_ARBITRARY_PWM_BASEADDR, &pwmConfig);

        counter_steps = (length_ns / 1000000);
    }
    else if (length_ns > 999) {    //999 - 1us
        pwmConfig.clockSource = kPWM_PeripheralClock;
        pwmConfig.prescale = 1U;
        PWM_Init(DEMO_ARBITRARY_PWM_BASEADDR, &pwmConfig);

        CLOCK_UpdateRoot(kCLOCK_RootPwm2, kCLOCK_PwmRootmuxOsc25m, 1U, 25U);

        counter_steps = (length_ns / 1000);
    }
    else {                          //999 - 1ns
        pwmConfig.clockSource = kPWM_PeripheralClock;
        pwmConfig.prescale = 1U;
        PWM_Init(DEMO_ARBITRARY_PWM_BASEADDR, &pwmConfig);

        CLOCK_UpdateRoot(kCLOCK_RootPwm2, kCLOCK_PwmRootmuxSystemPll3, 1U, 1U);

        counter_steps = (length_ns);
    }

    // NOTE: PWM module is 16Bit
    // NOTE: PWMO (HZ) = PCLK(Hz) / period+2    // TODO: is this really accurate?
    //       PWMO 1kHZ = 32kHz / 30+2
    PWM_SetPeriodValue(DEMO_ARBITRARY_PWM_BASEADDR, counter_steps); // PWM_PR[counter_steps] + 1
    PWM_SetSampleValue(DEMO_ARBITRARY_PWM_BASEADDR, counter_steps);
    // TODO: see if first pulse is with this sample value or the default one
    PRINTF("Set Sample to %u\r\n", counter_steps);

    /* 3. Set pin high/low */
    // TODO: pin and high/low needs to be passed into function
    PRINTF("Set pin high\r\n");
    GPIO_PinWrite(base, pin, 1U);

    /* 4. Start PWM timer */
    PRINTF("start timer\r\n");
    PWM_StartTimer(DEMO_ARBITRARY_PWM_BASEADDR);
}

void init_pwm(void) {
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
    PWM_EnableInterrupts(DEMO_ARBITRARY_PWM_BASEADDR, kPWM_CompareInterruptEnable);

    /***** 3. Load initial samples into PWM Sample Register *****/
    // NOTE: Sample Register is FIFO buffered.
    //       The PWM module will run at the last configured duty-cycle setting if the FIFO is empty.
    PWM_SetSampleValue(DEMO_ARBITRARY_PWM_BASEADDR, 1);

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
    PWM_SetPeriodValue(DEMO_ARBITRARY_PWM_BASEADDR, 1);

    /***** 6. Enable PWM *****/
    /* Enable PWM interrupt request */
    EnableIRQ(DEMO_ARBITRARY_PWM_IRQn);
}

static void app_task(void *param)
{
    volatile uint32_t remote_addr;
    struct rpmsg_lite_endpoint *volatile my_ept;
    volatile rpmsg_queue_handle my_queue;
    struct rpmsg_lite_instance *volatile my_rpmsg;
    volatile rpmsg_ns_handle ns_handle;

    volatile uint64_t pulse_length_ns;

    char length_str[30] = {0};

    /* Print the initial banner */
    (void)PRINTF("\r\nRPMSG Arbitrary PWM FreeRTOS Demo...\r\n");

#ifdef MCMGR_USED
    uint32_t startupData;
    mcmgr_status_t status;

    /* Get the startup data */
    do
    {
        status = MCMGR_GetStartupData(&startupData);
    } while (status != kStatus_MCMGR_Success);

    my_rpmsg = rpmsg_lite_remote_init((void *)(char *)startupData, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);

    /* Signal the other core we are ready by triggering the event and passing the APP_RPMSG_READY_EVENT_DATA */
    (void)MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, APP_RPMSG_READY_EVENT_DATA);
#else
    (void)PRINTF("RPMSG Share Base Addr is 0x%x\r\n", RPMSG_LITE_SHMEM_BASE);
    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
#endif /* MCMGR_USED */
    while (0 == rpmsg_lite_is_link_up(my_rpmsg))
    {
    }
    (void)PRINTF("Link is up!\r\n");

    my_queue  = rpmsg_queue_create(my_rpmsg);
    my_ept    = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    ns_handle = rpmsg_ns_bind(my_rpmsg, app_nameservice_isr_cb, ((void *)0));
    /* Introduce some delay to avoid NS announce message not being captured by the master side.
       This could happen when the remote side execution is too fast and the NS announce message is triggered
       before the nameservice_isr_cb is registered on the master side. */
    SDK_DelayAtLeastUs(1000000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    (void)rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, (uint32_t)RL_NS_CREATE);
    (void)PRINTF("Nameservice announce sent.\r\n");

#ifdef RPMSG_LITE_MASTER_IS_LINUX
    /* Wait Hello handshake message from Remote Core. */
    (void)rpmsg_queue_recv(my_rpmsg, my_queue, (uint32_t *)&remote_addr, helloMsg, sizeof(helloMsg), ((void *)0), RL_BLOCK);
    (void)PRINTF("recv Handshake: %s\r\n", helloMsg);
#endif /* RPMSG_LITE_MASTER_IS_LINUX */

    for (; ;) {
        PRINTF("Waiting for instruction...\r\n");
        rpmsg_queue_recv(my_rpmsg, my_queue, (uint32_t *)&remote_addr, (char *)&instruction, sizeof(APWM_INSTRUCTION), ((void *)0), RL_BLOCK);  // TODO: check if queue is best comm mechanism for this usecase
        pulse_length_ns = ((uint64_t)instruction.pulse_length_ns_HIGH << 32) | instruction.pulse_length_ns_LOW;

        if (instruction.operation == APWM_OPERATION_SETUP) {
            sprintf(length_str, "%u", pulse_length_ns);
            PRINTF("calling pin init with base %d, pin %d (length %s)\r\n", bases[instruction.gpio.base], instruction.gpio.pin, length_str);
            //GPIO_PinInit(bases[instruction.gpio.base], instruction.gpio.pin, &default_config);
        }
        else {
            curr_base = bases[instruction.gpio.base];
            curr_pin = instruction.gpio.pin;
            sprintf(length_str, "%u", pulse_length_ns);
            PRINTF("calling pulse with base %d, pin %d, length %s\r\n", curr_base, curr_pin, length_str);
            //pulse(curr_base, curr_pin, pulse_length_ns);
        }
    }

    (void)PRINTF("instruction test done, deinitializing...\r\n");

    (void)rpmsg_lite_destroy_ept(my_rpmsg, my_ept);
    my_ept = ((void *)0);
    (void)rpmsg_queue_destroy(my_rpmsg, my_queue);
    my_queue = ((void *)0);
    (void)rpmsg_ns_unbind(my_rpmsg, ns_handle);
    (void)rpmsg_lite_deinit(my_rpmsg);

    (void)PRINTF("Looping forever...\r\n");

    /* End of the example */
    for (;;)
    {
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
     /* Initialize standard SDK demo application pins */
    /* Board specific RDC settings */
    BOARD_RdcInit();

    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    copyResourceTable();

    /* Init PWM */
    init_pwm();

#ifdef MCMGR_USED
    /* Initialize MCMGR before calling its API */
    (void)MCMGR_Init();
#endif /* MCMGR_USED */

    if (xTaskCreate(app_task, "APP_TASK", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1U, &app_task_handle) != pdPASS)
    {
        (void)PRINTF("\r\nFailed to create application task\r\n");
        for (;;)
        {
        }
    }

    vTaskStartScheduler();

    (void)PRINTF("Failed to start FreeRTOS on core0.\r\n");
    for (;;)
    {
    }
}
