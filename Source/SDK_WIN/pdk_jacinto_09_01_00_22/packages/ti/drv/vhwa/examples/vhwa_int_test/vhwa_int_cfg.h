/*
*  Copyright (c) Texas Instruments Incorporated 2019
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
*  \file vhwa_int_tirtos_cfg.h
*
*  \brief Configuration for TI RTOS
*/
#ifndef VHWA_INT_TIRTOS_CFG_
#define VHWA_INT_TIRTOS_CFG_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define VHWA_INT_TIRTOS_CFG                                                    \
{                                                                              \
    {                                                                          \
        "TC_INT_TC0",      /* Test Name */                                     \
        TRUE,              /* Enable */                                        \
        1u,                /* Num Handles */                                   \
        10u,                /* Repeat Cnt */                                    \
        TRUE,              /* LDC Enable */                                    \
        {                                                                      \
            "TC_LDC_FUNC_001",      /* Test Name */                            \
            1u,                     /* Num Handles */                          \
            1u,                     /* Repeat Cnt */                           \
            TRUE,                  /* Is Performance Test */                  \
            /* Test Configuration */                                           \
            {                                                                  \
                &gAppLdcTestCfg[3U]                                            \
            },                                                                 \
            TRUE                                                               \
        },                                                                     \
        TRUE,              /* NF Enable */                                     \
        {                                                                      \
            "TC_NF_FUNC_001",      /* Test Name */                             \
            TRUE,                                                              \
            1u,                     /* Num Handles */                          \
            1u,                     /* Repeat Cnt */                           \
            TRUE,                  /* Is Performance Test */                  \
            /* Test Configuration */                                           \
            {                                                                  \
                &gAppNfTestCfg[15U]                                            \
            },                                                                 \
        },                                                                     \
        TRUE,              /* MSC0 Enable */                                   \
        {                                                                      \
            "TC_MSC0_FUNC_TC01",                                               \
            1u,                     /* NumHandles */                           \
            1u,                     /* Repeat Cnt */                           \
            TRUE,                  /* Is Performance Test */                  \
            {&gAppMscTestCfg[18U] },                                            \
            TRUE,                                                              \
        },                                                                     \
        TRUE,              /* MSC1 Enable */                                  \
        {                                                                      \
            "TC_MSC1_FUNC_TC01",                                               \
            1u,                     /* NumHandles */                           \
            1u,                     /* Repeat Cnt */                           \
            TRUE,                  /* Is Performance Test */                  \
            {&gAppMscTestCfg[19U] },                                           \
            TRUE,                                                              \
        },                                                                     \
        FALSE,              /* SDE Enable */                                    \
        {                                                                      \
            "TC_SDE_FUNC_001",      /* Test Name */                            \
            TRUE,                                                              \
            1u,                     /* Num Handles */                          \
            1u,                     /* Repeat Cnt */                           \
            TRUE,                  /* Is Performance Test */                  \
            /* Test Configuration */                                           \
            {                                                                  \
                &gAppSdeTestCfg[5U]                                            \
            },                                                                 \
            {                                                                  \
                gSdeConfScoreMap[0U]                                           \
            },                                                                 \
        },                                                                     \
        FALSE,              /* DOF Enable */                                    \
        {                                                                      \
            "TC_DOF_FUNC_001",      /* Test Name */                            \
            TRUE,                                                              \
            1u,                     /* Num Handles */                          \
            1u,                     /* Repeat Cnt */                           \
            TRUE,                  /* Is Performance Test */                  \
            /* Test Configuration */                                           \
            {                                                                  \
                &gAppDofTestCfg[6U]                                            \
            },                                                                 \
        },                                                                     \
        TRUE,              /* VISS Enable */                                   \
        {                                                                      \
            "TC_VISS_FUNC_001",                   /* Test Name */              \
            1,                          /* Num Handles */                      \
            1,                          /* Repeate Count */                    \
            TRUE,                       /* Is Performance */                   \
            {&gAppVissTestConfig[1]},     /* Test Config */                    \
            TRUE,                                                              \
            FALSE,                                                             \
            FALSE                                                              \
        },                                                                     \
                                                                               \
    },                                                                         \
}


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None*/

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

#endif /*VHWA_INT_TIRTOS_CFG_*/




;
