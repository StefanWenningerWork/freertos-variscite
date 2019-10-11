/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "assert.h"
#include "fsl_device_registers.h"
#include "main/ipc.h"
#include "main/rpc.h"
#ifdef FSL_RTOS_FREE_RTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

/*----------------------------------------------------------------------*/
/* RPC command/response                                                 */
/*----------------------------------------------------------------------*/
/* Please note: As the xTaskResumeAll can't be used in interrupt context in FreeRTOS.
 * The sc_call_rpc can't be called from interrupt context in FreeRTOS enviroment. 
*/
void sc_call_rpc(sc_ipc_t ipc, sc_rpc_msg_t *msg, sc_bool_t no_resp)
{
#ifdef FSL_RTOS_FREE_RTOS
    /* Suspends the scheduler to make sure there's only one rpc call ongoing at a time. */
    vTaskSuspendAll();
#endif
    sc_ipc_write(ipc, msg);
    if (!no_resp)
    {
        sc_ipc_read(ipc, msg);
    }
#ifdef FSL_RTOS_FREE_RTOS
    xTaskResumeAll();
#endif
}

/*--------------------------------------------------------------------------*/
/* Open an IPC channel                                                      */
/*--------------------------------------------------------------------------*/
sc_err_t sc_ipc_open(sc_ipc_t *ipc, sc_ipc_id_t id)
{
    MU_Type *base = (MU_Type *)id;

    /* get mu base associated with ipc channel */
    if ((ipc == NULL) || (base == NULL))
    {
        return SC_ERR_IPC;
    }

    /* Clear GIEn, RIEn, TIEn, GIRn and ABFn. */
    base->CR &= ~(MU_CR_GIEn_MASK | MU_CR_RIEn_MASK | MU_CR_TIEn_MASK | MU_CR_GIRn_MASK | MU_CR_Fn_MASK);

    /* Return MU address as handle */
    *ipc = (sc_ipc_t)id;

    return SC_ERR_NONE;
}

/*--------------------------------------------------------------------------*/
/* Close an IPC channel                                                     */
/*--------------------------------------------------------------------------*/
void sc_ipc_close(sc_ipc_t ipc)
{
}

/*--------------------------------------------------------------------------*/
/* Read message from an IPC channel                                         */
/*--------------------------------------------------------------------------*/
void sc_ipc_read(sc_ipc_t ipc, void *data)
{
    MU_Type *base = (MU_Type *)ipc;
    sc_rpc_msg_t *msg = (sc_rpc_msg_t *)data;
    uint8_t count = 0;

    /* Check parms */
    if ((base == NULL) || (msg == NULL))
    {
        return;
    }

    /* Read first word */
    /* Wait RX register to be full. */
    while (!(base->SR & (1U << (MU_SR_RFn_SHIFT + 3U))))
    {
    }
    *((uint32_t *)msg) = base->RR[0];
    count++;

    /* Check size */
    if (msg->size > SC_RPC_MAX_MSG)
    {
        *((uint32_t *)msg) = 0;
        return;
    }

    /* Read remaining words */
    while (count < msg->size)
    {
        /* Wait RX register to be full. */
        while (!(base->SR & (1U << (MU_SR_RFn_SHIFT + 3U - count % MU_RR_COUNT))))
        {
        }
        msg->DATA.i32[count - 1] = base->RR[count % MU_RR_COUNT];
        count++;
    }
}

/*--------------------------------------------------------------------------*/
/* Write a message to an IPC channel                                        */
/*--------------------------------------------------------------------------*/
void sc_ipc_write(sc_ipc_t ipc, void const* data)
{
    MU_Type *base = (MU_Type *)ipc;
    sc_rpc_msg_t *msg = (sc_rpc_msg_t *)data;
    uint8_t count = 0;

    /* Check parms */
    if ((base == NULL) || (msg == NULL))
    {
        return;
    }

    /* Check size */
    if (msg->size > SC_RPC_MAX_MSG)
    {
        return;
    }

    /* Write first word */
    while (!(base->SR & (1U << (MU_SR_TEn_SHIFT + 3))))
    {
    }
    base->TR[0] = *((uint32_t *)msg);
    count++;

    /* Write remaining words */
    while (count < msg->size)
    {
        /* Wait Tx register to be empty and send Tx Data. */
        while (!(base->SR & (1U << (MU_SR_TEn_SHIFT + 3 - count % MU_TR_COUNT))))
        {
        }
        base->TR[count % MU_TR_COUNT] = msg->DATA.i32[count - 1];
        count++;
    }
}
