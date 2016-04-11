/*----------------------------------------------------------------------------
 *      CMSIS-RTOS  -  RTX
 *----------------------------------------------------------------------------
 *      Name:    HAL_CM.C
 *      Purpose: Hardware Abstraction Layer for Cortex-M
 *      Rev.:    V4.79
 *----------------------------------------------------------------------------
 *
 * Copyright (c) 1999-2009 KEIL, 2009-2015 ARM Germany GmbH
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ARM  nor the names of its contributors may be used 
 *    to endorse or promote products derived from this software without 
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/

#include "rt_TypeDef.h"
#include "RTX_Config.h"
#include "rt_HAL_CM.h"

#ifdef DBG_MSG
BIT dbg_msg;
#endif

// Prepare TCB and saved context for a first time start of a task
void rt_init_stack(P_TCB p_TCB, FUNCP task_body)
{
    uint32_t *stk, i, size;

    // Prepare a complete interrupt frame for first task start
    size = p_TCB->priv_stack >> 2;
    if (!size)
    {
        size = (uint16_t)os_stackinfo >> 2;
    }

    // Write to the top of stack
    stk = &p_TCB->stack[size];

    // Auto correct to 8-byte ARM stack alignment
    if ((uint32_t)stk & 0x04U)
    {
        stk--;
    }

    stk -= 16;

    // Default xPSR and initial PC
    stk[15] = INITIAL_xPSR;
    stk[14] = (uint32_t)task_body;

    // Clear R4-R11, R0-R3, R12, LR registers
    for (i = 0U; i < 14U; i++)
    {
        stk[i] = 0U;
    }

    // Assign a void pointer to R0
    stk[8] = (uint32_t)p_TCB->msg;

    // Initial Task stack pointer
    p_TCB->tsk_stack = (uint32_t)stk;

    // Task entry point
    p_TCB->ptask = task_body;

    // Initialize stack with magic pattern
    if (os_stackinfo & 0x10000000U)
    {
        if (size > (16U + 1U))
        {
            for (i = ((size - 16U) / 2U) - 1U; i; i--)
            {
                stk -= 2U;
                stk[1] = MAGIC_PATTERN;
                stk[0] = MAGIC_PATTERN;
            }

            if (--stk > p_TCB->stack)
            {
                *stk = MAGIC_PATTERN;
            }
        }
    }

    // Set a magic word for checking of stack overflow
    p_TCB->stack[0] = MAGIC_WORD;
}

// Get pointer to task return value registers (R0..R3) in Stack
static __inline uint32_t* rt_ret_regs(P_TCB p_TCB)
{
#ifdef __TARGET_FPU_VFP
    if (p_TCB->stack_frame)
    {
        // Extended Stack Frame: R4-R11, S16-S31, R0-R3, R12, LR, PC, xPSR, S0-S15, FPSCR
        return (uint32_t*)(p_TCB->tsk_stack + (8U * 4U) + (16U * 4U));
    }
    else
    {
        // Basic Stack Frame: R4-R11, R0-R3, R12, LR, PC, xPSR
        return (uint32_t*)(p_TCB->tsk_stack + (8U * 4U));
    }
#else // __TARGET_FPU_VFP
    // Stack Frame: R4-R11, R0-R3, R12, LR, PC, xPSR
    return (uint32_t*)(p_TCB->tsk_stack + (8U * 4U));
#endif // __TARGET_FPU_VFP
}

void rt_ret_val(P_TCB p_TCB, uint32_t v0)
{
    uint32_t* ret;

    ret = rt_ret_regs(p_TCB);
    ret[0] = v0;
}

void rt_ret_val2(P_TCB p_TCB, uint32_t v0, uint32_t v1)
{
    uint32_t* ret;

    ret = rt_ret_regs(p_TCB);
    ret[0] = v0;
    ret[1] = v1;
}

#ifdef DBG_MSG
void dbg_init(void)
{
    if (((DEMCR & DEMCR_TRCENA) != 0U)
        && ((ITM_CONTROL & ITM_ITMENA) != 0U)
        && ((ITM_ENABLE & (1UL << 31)) != 0U))
    {
        dbg_msg = __TRUE;
    }
}

void dbg_task_notify(P_TCB p_tcb, bool create)
{
    while (!ITM_PORT31_U32);
    ITM_PORT31_U32 = (uint32_t)p_tcb->ptask;

    while (!ITM_PORT31_U32);
    ITM_PORT31_U16 = (uint16_t)((create << 8) | p_tcb->task_id);
}

void dbg_task_switch(uint32_t task_id)
{
    while (!ITM_PORT31_U32);
    ITM_PORT31_U8 = (uint8_t)task_id;
}
#endif // DBG_MSG

