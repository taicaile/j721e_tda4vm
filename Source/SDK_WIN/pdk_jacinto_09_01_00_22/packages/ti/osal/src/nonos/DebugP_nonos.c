/*
 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  ======== DebugP_nonos.c ========
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>

#include <ti/osal/DebugP.h>

#if DebugP_ASSERT_ENABLED
extern volatile bool Osal_DebugP_Assert_Val;

/*
 *  ======== _DebugP_assert ========
 */
void Osal_DebugP_assert_fcn(bool expression, const char *file, int32_t line)
{
    /* Workaround to resolve the Misra-C 2012 Rule 2.7 issue */
    (void)file;
    (void)line;

    if (!expression) {
        while ((bool)true == Osal_DebugP_Assert_Val) {};
    }
}
#endif

#if DebugP_LOG_ENABLED
DebugP_exptnLogFxn Osal_exptnLogFxn;

/*
 *  ======== DebugP_registerExcptnLogFxn ========
 */
int32_t DebugP_registerExcptnLogFxn(DebugP_exptnLogFxn fxn)
{
    int32_t retval;
    if (NULL==Osal_exptnLogFxn)
    {
        Osal_exptnLogFxn=fxn;
        retval = DEBUGP_LOGFXN_REGISTER_SUCCESS;
    }
    else
    {
        retval = DEBUGP_LOGFXN_ALREADY_REGISTERD;
    }
    return retval;
}

/*
 *  ======== DebugP_exceptionLog ========
 */
void DebugP_exceptionLog(const char * format, uint32_t arg1, uint32_t arg2)
{
    if (NULL!=Osal_exptnLogFxn)
    {
        Osal_exptnLogFxn(format, arg1, arg2);
    }
}

/*
 *  ======== DebugP_deRegisterExcptnLogFxn ========
 */
void DebugP_deRegisterExcptnLogFxn(void)
{
    Osal_exptnLogFxn=NULL;
}

/*
 *  ======== DebugP_log0 ========
 */
void DebugP_log0(const char *format)
{
    /* Workaround to resolve the Misra-C 2012 Rule 2.7 issue */
    (void)format;
}

/*
 *  ======== DebugP_log1 ========
 */
void DebugP_log1(const char *format, uintptr_t p1)
{
    /* Workaround to resolve the Misra-C 2012 Rule 2.7 issue */
    (void)format;
    (void)p1;
}

/*
 *  ======== DebugP_log2 ========
 */
void DebugP_log2(const char *format, uintptr_t p1, uintptr_t p2)
{
    /* Workaround to resolve the Misra-C 2012 Rule 2.7 issue */
    (void)format;
    (void)p1;
    (void)p2;
}

/*
 *  ======== DebugP_log3 ========
 */
void DebugP_log3(const char *format, uintptr_t p1, uintptr_t p2, uintptr_t p3)
{
    /* Workaround to resolve the Misra-C 2012 Rule 2.7 issue */
    (void)format;
    (void)p1;
    (void)p2;
    (void)p3;
}

/*
 *  ======== DebugP_log4 ========
 */
void DebugP_log4(const char *format, uintptr_t p1, uintptr_t p2, uintptr_t p3, uintptr_t p4)
{
    /* Workaround to resolve the Misra-C 2012 Rule 2.7 issue */
    (void)format;
    (void)p1;
    (void)p2;
    (void)p3;
    (void)p4;
}
#endif
