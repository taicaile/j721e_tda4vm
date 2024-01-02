/*
 *  Copyright (c) Texas Instruments Incorporated 2023
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *	Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 *
 *	Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in the
 *	documentation and/or other materials provided with the
 *	distribution.
 *
 *	Neither the name of Texas Instruments Incorporated nor the names of
 *	its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <tsn_combase/combase.h>
#include <tsn_unibase/unibase_binding.h>
#include <tsn_gptp/gptpman.h>
#include <tsn_gptp/gptp_config.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/drv/enet/enet.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TSN_TASK_STACK_SIZE              (16U * 1024U)
#define TSN_TASK_STACK_ALIGN             TSN_TASK_STACK_SIZE

#define TSN_LOG_TASK_STACK_SIZE          (1U * 1024U)
#define TSN_LOG_TASK_STACK_ALIGN         (32U)

/* Logging task is very low priority so it doesn't interfere with gPTP stack */
#define TSN_LOG_TASK_PRIORITY            (1U)

/* Use UART instead of EnetAppUtils_printf because the latter has smaller buffer */
#define DPRINT                           UART_printf

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* Container structure of all TSN related parameters */
typedef struct EnetApp_TsnObj_s
{
    /* Whether TSN stack has been initialized or not */
    bool tsnInit;

    /* Whether gPTP has been started or not */
    bool gPtpStarted;

    /* gPTP task handle */
    CB_THREAD_T hPtpTask;

    /* gPTP task stack buffer */
    uint8_t gPtpStackBuf[TSN_TASK_STACK_SIZE] __attribute__ ((aligned(TSN_TASK_STACK_ALIGN)));

    /* Mutex used for TSN stack logging */
    CB_THREAD_MUTEX_T hLogMutex;

    /* TSN stack logging task handle */
    CB_THREAD_T hLogTask;

    /* TSN logger task stack buffer */
    uint8_t logTaskStackBuf[TSN_LOG_TASK_STACK_SIZE] __attribute__ ((aligned(TSN_LOG_TASK_STACK_ALIGN)));

    /* Buffer used to accumulate the log messages given by stack until 'log_task'
     * writes them to UART */
    uint8_t logBuf[4096];

    /* Buffer used to store string to be printed via UART */
    uint8_t printBuf[4096];
} EnetApp_TsnObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Container object of all TSN related variables */
static EnetApp_TsnObj gEnetAppTsnObj;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void *EnetApp_logTask(void *arg)
{
    int len;

    DPRINT("%s: started\n", __func__);

    while (true)
    {
        CB_THREAD_MUTEX_LOCK(&gEnetAppTsnObj.hLogMutex);
        len = strlen((char *)gEnetAppTsnObj.logBuf);
        if (len > 0)
        {
            memcpy(gEnetAppTsnObj.printBuf, gEnetAppTsnObj.logBuf, len);
            gEnetAppTsnObj.logBuf[0] = 0;
            gEnetAppTsnObj.printBuf[len] = 0;
        }
        CB_THREAD_MUTEX_UNLOCK(&gEnetAppTsnObj.hLogMutex);

        if (len > 0)
        {
            /* The print function will take a long time, we should not
             * call it inside the mutex lock. */
            DPRINT("%s", gEnetAppTsnObj.printBuf);
        }

        CB_USLEEP(10000);
    }

    return NULL;
}

static int EnetApp_logToBuffer(bool flush, const char *str)
{
    int used_len;
    int remain_bufsize;
    int loglen = strlen(str);

    CB_THREAD_MUTEX_LOCK(&gEnetAppTsnObj.hLogMutex);
    used_len = strlen((char *)gEnetAppTsnObj.logBuf);
    remain_bufsize = sizeof(gEnetAppTsnObj.logBuf)-used_len;
    if (remain_bufsize > loglen)
    {
        snprintf((char *)&gEnetAppTsnObj.logBuf[used_len], remain_bufsize, "%s", str);
    }
    else
    {
        snprintf((char *)&gEnetAppTsnObj.logBuf[0], sizeof(gEnetAppTsnObj.logBuf), "log ovflow!\n");
    }
    CB_THREAD_MUTEX_UNLOCK(&gEnetAppTsnObj.hLogMutex);

    return 0;
}

static int32_t EnetApp_startLogTask(void)
{
    cb_tsn_thread_attr_t attr;
    int32_t status = ENET_SOK;

    if (CB_THREAD_MUTEX_INIT(&gEnetAppTsnObj.hLogMutex, NULL) < 0)
    {
        DPRINT("Failed to int mutex!\n");
        status = ENET_EFAIL;
    }

    if (status == ENET_SOK)
    {
        cb_tsn_thread_attr_init(&attr, TSN_LOG_TASK_PRIORITY, sizeof(gEnetAppTsnObj.logTaskStackBuf), "log_task");
        cb_tsn_thread_attr_set_stackaddr(&attr, &gEnetAppTsnObj.logTaskStackBuf[0]);
        if (CB_THREAD_CREATE(&gEnetAppTsnObj.hLogTask, &attr, EnetApp_logTask, NULL) < 0)
        {
            DPRINT("Failed to create log task!\n");
            status = ENET_EFAIL;
        }
    }

    if (status != ENET_SOK)
    {
        if (gEnetAppTsnObj.hLogMutex != NULL)
        {
            CB_THREAD_MUTEX_DESTROY(&gEnetAppTsnObj.hLogMutex);
            gEnetAppTsnObj.hLogMutex = NULL;
        }
        if (gEnetAppTsnObj.hLogTask != NULL)
        {
            CB_THREAD_JOIN(gEnetAppTsnObj.hLogTask, NULL);
            gEnetAppTsnObj.hLogTask = NULL;
        }
    }

    return status;
}

static void *EnetApp_gPtpTask(void *a)
{
    char **netdevs = (char **)a;
    int32_t i;

    DPRINT("%s: started\n", __func__);

    /* Count the number of netdevs */
    for (i = 0; netdevs[i]; i++)
    {
        /* Do nothing */
    }

    /* This function has a true loop inside */
    if (gptpman_run(netdevs, i, 1, NULL) < 0)
    {
        DPRINT("%s: gptpman_run() error\n", __func__);
    }

    return NULL;
}

/*
 * Initializes the TSN application
 */
int32_t EnetApp_initTsn(void)
{
    unibase_init_para_t params;
    int32_t status = ENET_SOK;

    if (gEnetAppTsnObj.tsnInit == false)
    {
        ubb_default_initpara(&params);
        params.ub_log_initstr    = "4,ubase:45,cbase:45,gptp:56";
        params.cbset.gettime64   = cb_lld_gettime64;
        params.cbset.console_out = EnetApp_logToBuffer;

        if (EnetApp_startLogTask() < 0)
        {
            status = ENET_EFAIL;
        }

        if (status == ENET_SOK)
        {
            unibase_init(&params);
            ubb_memory_out_init(NULL, 0);

            gEnetAppTsnObj.tsnInit = true;
        }
    }

    return status;
}

/*
 * Deinitializes the TSN application
 */
int32_t EnetApp_deinitTsn(void)
{
    unibase_close();

    if (gEnetAppTsnObj.hLogTask != NULL)
    {
        CB_THREAD_JOIN(gEnetAppTsnObj.hLogTask, NULL);
        gEnetAppTsnObj.hLogTask = NULL;
    }

    if (gEnetAppTsnObj.hPtpTask != NULL)
    {
        CB_THREAD_JOIN(gEnetAppTsnObj.hPtpTask, NULL);
        gEnetAppTsnObj.hPtpTask = NULL;
    }

    if (gEnetAppTsnObj.hLogMutex != NULL)
    {
        CB_THREAD_MUTEX_DESTROY(&gEnetAppTsnObj.hLogMutex);
        gEnetAppTsnObj.hLogMutex = NULL;
    }

    unibase_close();

    gEnetAppTsnObj.tsnInit = false;
    gEnetAppTsnObj.gPtpStarted = false;

    return ENET_SOK;
}

/*
 * Starts the gptp process task. The array of net devices passed as argument
 * must be terminated with a NULL pointer.
 * For example, char *netdevs[2] = {"tilld0", NULL};
 */
int32_t EnetApp_gPtpStart(char *netdevs[])
{
    cb_tsn_thread_attr_t attr;
    int32_t status = ENET_SOK;

    if (gEnetAppTsnObj.gPtpStarted == false)
    {
        cb_tsn_thread_attr_init(&attr, 1, sizeof(gEnetAppTsnObj.gPtpStackBuf), "gptp_task");
        cb_tsn_thread_attr_set_stackaddr(&attr, &gEnetAppTsnObj.gPtpStackBuf[0]);
        if (CB_THREAD_CREATE(&gEnetAppTsnObj.hPtpTask, &attr, EnetApp_gPtpTask, netdevs) < 0)
        {
            DPRINT("Failed to create gptp task!\n");
            status = ENET_EFAIL;
        }
        else
        {
            gEnetAppTsnObj.gPtpStarted = true;
        }
    }

    return status;
}
