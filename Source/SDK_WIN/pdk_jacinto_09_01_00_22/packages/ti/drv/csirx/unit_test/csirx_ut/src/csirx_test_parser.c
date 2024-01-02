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
 *  \file csirx_test_parser.c
 *
 *  \brief CSIRX test application parser file.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <csirx_test.h>
#include <csirx_testcases.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Test application stack size */
#define APP_TSK_STACK_MAIN              (20U * 1024U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t csirxTestInit(CsirxTestParams *testParams);

static int32_t csirxTestDeinit(CsirxTestParams *testParams);

static int32_t csirxTestRunTc(CsirxTestParams *testParams);
static int32_t csirxTestCreateTestTasks(CsirxTestParams *testParams);
static int32_t csirxTestDeleteTestTasks(CsirxTestParams *testParams);
static void csirxTestTask(void *arg0, void *arg1);
static void csirxTestInitTestObj(CsirxTestParams *testParams);
static void csirxTestFillTestObj(CsirxTestParams *testParams);

static int32_t csirxTestGetTcIdx(uint32_t tcId, bool isSearchForward);
static bool csirxTestCheckIfTestToBeSkipped(CsirxTestParams *testParams,
                                            uint32_t        tcType);

static int32_t csirxTestDisplayTestInfo(CsirxTestParams *testParams);
static int32_t csirxTestGenerateTestReports(CsirxTestParams *testParams);

static void csirxTestMenuSettings(CsirxTestParams *testParams);
static void csirxTestMenuMainShow(CsirxTestParams *testParams);
static void csirxTestMenuSettingsShow(CsirxTestParams *testParams);
#if !defined (QT_BUILD)
static void csirxTestMenuCurrentSettingsShow(CsirxTestParams *testParams);
#endif
static void csirxTestSetDefaultCfg(CsirxTestParams *testParams);
static uint32_t csirxTestGetTestId(CsirxTestParams *testParams,
                                   uint32_t tcType);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static uint8_t gFrmDropBuf[CSIRX_TEST_MAX_LINE_SIZE_BYTES] __attribute__(( aligned(128), section(".image_data")));

/* Task stack */
static uint8_t  gCsirxParserTskStack[CSIRX_TEST_MAX_TASKS][APP_TSK_STACK_MAIN] __attribute__((aligned(32)));;

#if !defined (QT_BUILD)
/* Main menu 0 string */
static const char gCsirxTestMenuMain0[] =
{
    "\r\n "
    "\r\n ==============="
    "\r\n CSIRX Test Select"
    "\r\n ==============="
    "\r\n "
};

/* Main menu string */
/* Only manual testing is supported */
static const char gCsirxTestMenuMain1[] =
{
    "\r\n 1: Manual testing (select specific test case to run)"
/*    "\r\n 2: Sanity (BFT) testing"
    "\r\n 3: Regression testing"
    "\r\n 4: Full testing"
    "\r\n 5: Performance testing"
    "\r\n d: Display test cases"
    "\r\n g: Generate test report"*/
    "\r\n "
    "\r\n s: System Settings"
    "\r\n "
    "\r\n q: Quit "
    "\r\n "
    "\r\n Enter Choice: "
    "\r\n"
};

static const char gCsirxTestMenuSettings0[] = {
    "\r\n ==============="
    "\r\n System Settings"
    "\r\n ==============="
    "\r\n "
};

static const char gCsirxTestMenuSettings1[] = {
    "\r\n "
    "\r\n l: Loop Count"
    "\r\n d: Queue Depth (How many outstanding transfer per channel)"
    "\r\n r: Runtime Print Enable"
    "\r\n "
    "\r\n q: Quit "
    "\r\n"
};
#endif

uint32_t            gSkipCount;
/**< Number of test cases skipped because of platform/user settings. */
uint32_t            gDisableCount;
/**< Number of test cases disabled because of any bug etc. */

/** \brief Log enable for CSIRX Unit Test  application */
extern uint32_t gAppTrace;

extern struct Udma_DrvObj gUdmaDrvObj;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  csirxTestParser
 */
int32_t csirxTestParser(void)
{
    char            option;
    bool            done;
    int32_t         retVal = FVID2_SOK, getCharRetVal = FVID2_SOK;
    int32_t         tcIdx, startIdx;
    uint32_t        testCnt, tcType, tcId;
    uint32_t        startTime, elapsedTime, startTime1, elapsedTime1;
    uint32_t        hrs, mins, secs, durationInSecs;
    CsirxTestParams *testParams;
    uint32_t        isValidInput = FALSE;

    testParams = &gCsirxTestCases[0U];
    csirxTestSetDefaultCfg(testParams);
    csirxTestResetTestResult();

    retVal = csirxTestInit(testParams);
    if (retVal != FVID2_SOK)
    {
        GT_0trace(gAppTrace, GT_ERR, " CSIRX Test Init failed!!\n");
    }

    if(FVID2_SOK != retVal)
    {
        return retVal;
    }

    startTime             = AppUtils_getCurTimeInMsec();
    done                  = FALSE;
    gSkipCount    = 0U;
    gDisableCount = 0U;
    while(!done)
    {
        csirxTestMenuMainShow(testParams);
        if(isValidInput != TRUE)
        {
            getCharRetVal = AppUtils_getCharTimeout(&option, CSIRX_TEST_UART_TIMEOUT_MSEC);
            if(FVID2_SOK != getCharRetVal)
            {
#if !defined (QT_BUILD)
                /* Timeout */
                GT_0trace(
                    gAppTrace, GT_INFO,
                    " UART read timeout (10 sec)!!\r\n");
                /* Default to full test */
                GT_0trace(
                    gAppTrace, GT_INFO, " Automating FULL test!!\r\n");
#endif
                option = '4';
                /* Set done flag to end test */
                done   = TRUE;
            }
            else
            {
                isValidInput = TRUE;
            }
        }
        else
        {
            option = AppUtils_getChar();
        }
        GT_0trace(gAppTrace, GT_INFO, " \r\n");

        tcType = CSIRX_TCT_SANITY;
        switch(option)
        {
            case 's':
            case 'S':
                csirxTestMenuSettings(testParams);
                break;

            case 'q':
            case 'Q':
                done = TRUE;
                break;

            case '1':
                GT_0trace(gAppTrace, GT_INFO, " Manual testing \r\n");
                tcId = csirxTestGetTestId(testParams, CSIRX_TCT_ALL);
                /* Find the test case to run */
                tcIdx = csirxTestGetTcIdx(tcId, FALSE);
                if(tcIdx < 0)
                {
                    GT_0trace(gAppTrace, GT_INFO,
                              " Test case ID not found\r\n");
                }
                else
                {
                    testParams = &gCsirxTestCases[tcIdx];
                    csirxTestRunTc(testParams);
                }
                break;

            case '4':
                tcType |= CSIRX_TCT_FULL;
            case '3':
                tcType |= CSIRX_TCT_REGRESSION;
            case '2':
                if(TRUE == done)
                {
                    /* Auto run, start from 1st test case */
                    tcId = 0U;
                }
                else
                {
                    GT_0trace(
                        gAppTrace, GT_INFO,
                        " Enter Start Test Case Id "
                        "(Enter 0 to start from first): \r\n");
                    tcId = csirxTestGetTestId(testParams, tcType);
                }
                startIdx = csirxTestGetTcIdx(tcId, TRUE);
                if(startIdx < 0)
                {
                    GT_0trace(gAppTrace, GT_INFO,
                              " Test case ID not found\r\n");
                    continue;
                }

                /* Run all test cases one after the other depending on selected
                 * flag */
                startTime1         = AppUtils_getCurTimeInMsec();
                gSkipCount = gDisableCount = 0U;

                for(testCnt = startIdx;
                     testCnt < CSIRX_TEST_NUM_TESTCASES;
                     testCnt++)
                {
                    testParams = &gCsirxTestCases[testCnt];

                    /* Check whether to execute test or not */
                    if(csirxTestCheckIfTestToBeSkipped(testParams,
                                                       tcType))
                    {
                        continue;       /* Skip test */
                    }

                    csirxTestRunTc(testParams);
                }

                /* Print test results */
                csirxTestPrintTestResult(
                    testParams,
                    gSkipCount,
                    gDisableCount);

                elapsedTime1   = AppUtils_getElapsedTimeInMsec(startTime1);
                durationInSecs = ((elapsedTime1) / 1000U);
                hrs  = durationInSecs / (60U * 60U);
                mins = (durationInSecs / 60U) - (hrs * 60U);
                secs = durationInSecs - (hrs * 60U * 60U) - (mins * 60U);
                GT_3trace(gAppTrace, GT_INFO,
                          " |TOTAL TEST DURATION|: %d hrs %d mins %d secs\r\n",
                          hrs, mins, secs);
                break;

            case '5':
                tcType = CSIRX_TCT_PERFORMANCE;
                startIdx = csirxTestGetTcIdx(0U, TRUE);
                if(startIdx < 0)
                {
                    GT_0trace(gAppTrace, GT_INFO,
                              " Test case ID not found\r\n");
                    continue;
                }

                /* Run all test cases one after the other depending on selected
                 * flag */
                startTime1         = AppUtils_getCurTimeInMsec();
                gSkipCount = gDisableCount = 0U;
                for(testCnt = startIdx;
                    testCnt < CSIRX_TEST_NUM_TESTCASES;
                    testCnt++)
                {
                    testParams = &gCsirxTestCases[testCnt];

                    /* Check whether to execute test or not */
                    if(csirxTestCheckIfTestToBeSkipped(testParams, tcType))
                    {
                        continue;       /* Skip test */
                    }

                    csirxTestRunTc(testParams);
                }

                /* Print test results */
                csirxTestPrintTestResult(
                    testParams, gSkipCount, gDisableCount);

                elapsedTime1   = AppUtils_getElapsedTimeInMsec(startTime1);
                durationInSecs = ((elapsedTime1) / 1000U);
                hrs  = durationInSecs / (60U * 60U);
                mins = (durationInSecs / 60U) - (hrs * 60U);
                secs = durationInSecs - (hrs * 60U * 60U) - (mins * 60U);
                GT_3trace(gAppTrace, GT_INFO,
                          " |TOTAL TEST DURATION|: %d hrs %d mins %d secs\r\n",
                          hrs, mins, secs);
                break;

            case 'd':
            case 'D':
                /* Display test info */
                csirxTestDisplayTestInfo(testParams);
                break;

            case 'g':
            case 'G':
                /* Generate test report */
                csirxTestGenerateTestReports(testParams);
                break;

            default:
                GT_0trace(gAppTrace, GT_INFO,
                          " Invalid option try again!!\r\n");
                break;
        }
    }

#if !defined (QT_BUILD)
    csirxTestDisplayTestInfo(testParams);
    csirxTestGenerateTestReports(testParams);
#endif

    elapsedTime    = AppUtils_getElapsedTimeInMsec(startTime);
    durationInSecs = ((elapsedTime) / 1000U);
    hrs  = durationInSecs / (60U * 60U);
    mins = (durationInSecs / 60U) - (hrs * 60U);
    secs = durationInSecs - (hrs * 60U * 60U) - (mins * 60U);
    GT_3trace(gAppTrace, GT_INFO,
              " |TOTAL UT DURATION|: %d hrs %d mins %d secs\r\n",
              hrs, mins, secs);

    /* Check if all testcases passed and accordingly return pass/fail */
    for(testCnt = 0U; testCnt < CSIRX_TEST_NUM_TESTCASES; testCnt++)
    {
        testParams = &gCsirxTestCases[testCnt];
        if(TRUE == testParams->isRun)
        {
            if(FVID2_SOK != testParams->testResult)
            {
                retVal = testParams->testResult;
            }
        }
    }

    retVal = csirxTestDeinit(testParams);
    if(FVID2_SOK != retVal)
    {
        GT_0trace(gAppTrace, GT_ERR, " CSIRX Test deinit failed!!\n");
    }

    /* Print test results */
    csirxTestPrintTestResult(testParams, gSkipCount, gDisableCount);

    return retVal;
}

/**
 *  csirxTestRunTc
 */
static int32_t csirxTestRunTc(CsirxTestParams *testParams)
{
    static char     tempString[CSIRX_TEST_PRINT_BUFSIZE];
    int32_t         retVal = FVID2_SOK;
    uint32_t        startTime, hrs, mins, secs, msecs, loopCnt;
    uint32_t        durationInMsecs = 0U;
    uint32_t        durationInSecs= 0U;
    static char    *enableDisableName[] = {"Disabled", "Enabled"};

    /* NULL pointer check */
    GT_assert(gAppTrace, (NULL != testParams));
    GT_assert(gAppTrace, (NULL != testParams->tcName));

    GT_0trace(gAppTrace, GT_INFO, " \r\n");
    GT_1trace(gAppTrace, GT_INFO,
              " |TEST START|:: %d ::\r\n", testParams->tcId);
    GT_1trace(gAppTrace, GT_INFO,
              " |TEST NAME| :: %s ::\r\n", testParams->tcName);
    GT_1trace(gAppTrace, GT_INFO,
              " |TEST INFO| :: Num Tasks         : %d ::\r\n",
              testParams->numTasks);
    GT_1trace(gAppTrace, GT_INFO,
              " |TEST INFO| :: Data Check        : %s ::\r\n",
              enableDisableName[testParams->dcEnable]);
    GT_1trace(gAppTrace, GT_INFO,
              " |TEST INFO| :: Profiling         : %s ::\r\n",
              enableDisableName[testParams->profilingEnable]);
    GT_1trace(gAppTrace, GT_INFO,
              " |TEST INFO| :: Print             : %s ::\r\n",
              enableDisableName[testParams->printEnable]);

    csirxTestInitTestObj(testParams);
    /* Start the load calculation */
    // Utils_prfLoadCalcStart();

    testParams->isRun = TRUE;
    /* CSIRX FVID2 DRV init */
    /* System init */
    retVal = Csirx_init(&testParams->initPrms);
    if (retVal != FVID2_SOK)
    {
        GT_0trace(gAppTrace, GT_ERR,
                  APP_NAME ": System Init Failed!!!\r\n");
    }
    for (loopCnt  = 0U ; loopCnt < testParams->iterationCnt ; loopCnt++)
    {
        GT_1trace(gAppTrace, GT_INFO,
              " |TEST ITEARATION NUMBER| ::      : %d ::\r\n",
              (loopCnt + 1U));

        startTime = AppUtils_getCurTimeInMsec();
        retVal = csirxTestCreateTestTasks(testParams);
        durationInMsecs = AppUtils_getElapsedTimeInMsec(startTime);
        if (retVal != FVID2_SOK)
        {
            GT_0trace(gAppTrace, GT_ERR,
                          APP_NAME ": Failed in task creation!!!\r\n");
            break;
        }

        // Utils_prfLoadCalcStop();
        // Utils_prfLoadPrintAll(TRUE, gAppTrace);
        // Utils_prfLoadCalcReset();
        retVal = csirxTestDeleteTestTasks(testParams);
        if (retVal != FVID2_SOK)
        {
            GT_0trace(gAppTrace, GT_ERR,
                          APP_NAME ": Failed in task deletion!!!\r\n");
            break;
        }

    }
    /* CSIRX FVID2 DRV de-init */
    /* System init */
    retVal = Csirx_deInit();
    if (retVal != FVID2_SOK)
    {
        GT_0trace(gAppTrace, GT_ERR,
                  APP_NAME ": System De-init Failed!!!\r\n");
    }


    durationInSecs  = (durationInMsecs / 1000U);
    hrs   = durationInSecs / (60U * 60U);
    mins  = (durationInSecs / 60U) - (hrs * 60U);
    secs  = durationInSecs - (hrs * 60U * 60U) - (mins * 60U);
    msecs = durationInMsecs - ((hrs * 60U * 60U) + (mins * 60U) + secs) * 1000U;
    GT_4trace(gAppTrace, GT_INFO,
              " |TEST DURATION|:: %d:%0.2d:%0.2d:%0.3d ::\r\n",
              hrs, mins, secs, msecs);

    if((TRUE == testParams->profilingEnable) && (testParams->numTasks > 1U))
    {
        /* Total performance applicable only for multi thread.
         * Incase of multi thread, individual performance doesn't makes sense
         * Incase of single thread, the thread itself will print the accurate
         * performance. Total is not very accurate. Hence disable from here */
        // csirxTestCalcTotalPerformance(testParams, durationInMsecs);
    }

    GT_assert(gAppTrace, (NULL != testParams->tcName));
    snprintf(tempString, sizeof (tempString), "%s", testParams->tcName);
    csirxTestLogTestResult(testParams, retVal, testParams->tcId, tempString);

    if(FVID2_SOK == retVal)
    {
        GT_1trace(gAppTrace, GT_INFO,
            " |TEST RESULT|:: CSIRX Test Case %d Successful!! ::\r\n", testParams->tcId);
    }
    else
    {
        GT_1trace(gAppTrace, GT_INFO,
            " |TEST RESULT|:: CSIRX Test Case %d Failed!! ::\r\n", testParams->tcId);
    }

    /* Store the test result */
    testParams->testResult = retVal;

    return retVal;
}

int32_t csirxTestCreateTestTasks(CsirxTestParams *testParams)
{
    int32_t             retVal = FVID2_SOK;
    uint32_t            taskCnt, createdTsk = 0U;
    TaskP_Params        taskPrms;
    CsirxTestTaskObj    *taskObj;
    SemaphoreP_Params   semPrms;

    SemaphoreP_Params_init(&semPrms);
    semPrms.mode = SemaphoreP_Mode_COUNTING;
    testParams->taskCompleteSem = SemaphoreP_create(0U, &semPrms);
    if(NULL == testParams->taskCompleteSem)
    {
        GT_0trace(gAppTrace, GT_ERR, " Sem create failed\r\n");
        retVal = FVID2_EALLOC;
    }

    if(FVID2_SOK == retVal)
    {
        for(taskCnt = 0U; taskCnt < testParams->numTasks; taskCnt++)
        {
            taskObj = &testParams->taskObj[taskCnt];

            TaskP_Params_init(&taskPrms);
            taskPrms.arg0 = taskObj;
            taskPrms.stack = &gCsirxParserTskStack[0U];
            taskPrms.stacksize = APP_TSK_STACK_MAIN;
            taskObj->taskHandle = TaskP_create(&csirxTestTask,
                                               &taskPrms);
            if(NULL == taskObj->taskHandle)
            {
                retVal = FVID2_EALLOC;
                break;
            }
            createdTsk++;

            /* Register the task to the load module for calculating the load */
            // snprintf(taskObj->prfTsName, sizeof (taskObj->prfTsName), "Task%d", taskCnt);
            // Utils_prfLoadRegister(taskObj->taskHandle, taskObj->prfTsName);
        }
    }

    if(FVID2_SOK == retVal)
    {
        for(taskCnt = 0U; taskCnt < createdTsk; taskCnt++)
        {
            /* Wait for tasks to complete */
            SemaphoreP_pend(testParams->taskCompleteSem, SemaphoreP_WAIT_FOREVER);
        }
    }

    if(FVID2_SOK == retVal)
    {
        for(taskCnt = 0u; taskCnt < createdTsk; taskCnt++)
        {
            retVal += testParams->taskObj[taskCnt].testResult;
        }
    }

    if(NULL != testParams->taskCompleteSem)
    {
        SemaphoreP_delete(testParams->taskCompleteSem);
        testParams->taskCompleteSem = NULL;
    }

    return retVal;
}

static int32_t csirxTestDeleteTestTasks(CsirxTestParams *testParams)
{
    int32_t             retVal = FVID2_SOK;
    uint32_t            taskCnt;
    CsirxTestTaskObj    *taskObj;

    /* Delete all created tasks and semephores */
    for(taskCnt = 0U; taskCnt < testParams->numTasks ; taskCnt++)
    {
        taskObj = &testParams->taskObj[taskCnt];
        if(NULL != taskObj->taskHandle)
        {
            // Utils_prfLoadUnRegister(taskObj->taskHandle);
            TaskP_delete(&taskObj->taskHandle);
            taskObj->taskHandle = NULL;
        }
    }

    return retVal;
}

static void csirxTestTask(void *arg0, void *arg1)
{
    int32_t                retVal = FVID2_SOK;
    CsirxTestTaskObj       *taskObj;

    /* Run the test */
    taskObj = (CsirxTestTaskObj *) arg0;

    /* Run the test */
    GT_assert(gAppTrace, (taskObj->taskCfg->testFxnPtr != NULL));
    retVal = taskObj->taskCfg->testFxnPtr(taskObj);

    //taskObj->testResult = retVal;
    if (retVal != FVID2_SOK)
    {
        GT_0trace(gAppTrace, GT_ERR, ": Task Function returned Error...\r\n");
    }

    /* Test complete. Signal it */
    SemaphoreP_post(taskObj->testParams->taskCompleteSem);

    return;
}

static int32_t csirxTestInit(CsirxTestParams *testParams)
{
    int32_t retVal = FVID2_SOK;

    /* setup memories */
    retVal = Utils_memInit();

    return retVal;
}
static int32_t csirxTestDeinit(CsirxTestParams *testParams)
{
    int32_t         retVal = FVID2_SOK;

    retVal = Utils_memDeInit();

    return retVal;
}

static void csirxTestInitTestObj(CsirxTestParams *testParams)
{
    uint32_t         taskCnt, chIdx;
    CsirxInstCfgInfo *instInfo;
    CsirxInstObj     *instObj;

    gAppTrace = (GT_INFO1 | GT_TraceState_Enable);
    testParams->initPrms.drvHandle = &gUdmaDrvObj;
    if(FALSE == testParams->printEnable)
    {
        /* Restrict the number of prints when print is disabled */
        gAppTrace = (GT_INFO | GT_TraceState_Enable);
    }
#if defined (QT_BUILD)
    /* Enable only error prints in QT */
    gAppTrace = (GT_ERR | GT_TraceState_Enable);
#endif

    GT_assert(gAppTrace, (testParams->numTasks <= CSIRX_TEST_MAX_TASKS));
    testParams->taskCompleteSem = NULL;

    /* Prepare task object here,
      return values if ignored as it is always PASS */
    csirxTestFillTestObj(testParams);
    for(taskCnt = 0U; taskCnt < testParams->numTasks; taskCnt++)
    {
        GT_assert(gAppTrace,
                (testParams->taskObj[taskCnt].taskCfg->testFxnPtr != NULL));

        /* Initialize synch semaphore to NULL */
        testParams->taskObj[taskCnt].syncSem = NULL;
        /* reference to test parameters */
        testParams->taskObj[taskCnt].testParams = testParams;

        testParams->taskObj[taskCnt].taskHandle = NULL;
        /* instance configurations */
            instInfo = testParams->taskObj[taskCnt].instObj.instCfgInfo;
            instObj = &testParams->taskObj[taskCnt].instObj;
            GT_assert(gAppTrace, (instInfo->csiDrvInst <= CSIRX_TEST_MAX_INST));
            GT_assert(gAppTrace, (instInfo->instCfg != NULL));
            GT_assert(gAppTrace, (instInfo->frameDropBuf != (uint64_t)NULL));
            GT_assert(gAppTrace, (instInfo->frameDropBufLen != (uint32_t)NULL));
            GT_assert(gAppTrace, (instInfo->numCh != 0U));
            GT_assert(gAppTrace, (instInfo->numCh <= CSIRX_TEST_MAX_CH));
            /* channel configurations */
            for (chIdx = 0U ; chIdx < instInfo->numCh ; chIdx++)
            {
                /* Initialize number of frames captured to '0' */
                instObj->chObj[chIdx].captFrames    = 0U;
                instObj->chObj[chIdx].captErrFrames = 0U;
                instObj->chObj[chIdx].firstTs       = 0U;
                instObj->chObj[chIdx].fps           = 0U;
            }
    }

    return;
}

/**
 *  csirxTestGetTcIdx
 */
static int32_t csirxTestGetTcIdx(uint32_t tcId, bool isSearchForward)
{
    int32_t         tcIdx = -1;
    uint32_t        testCnt;
    CsirxTestParams *testParams;

    testParams = &gCsirxTestCases[0U];
    for(testCnt = 0U; testCnt < CSIRX_TEST_NUM_TESTCASES; testCnt++)
    {
        if(TRUE == isSearchForward)
        {
            if(testParams->tcId >= tcId)
            {
                tcIdx = testCnt;
                break;
            }
        }
        else
        {
            if(testParams->tcId == tcId)
            {
                tcIdx = testCnt;
                break;
            }
        }
        testParams++;
    }

    return (tcIdx);
}

/**
 *  csirxTestCheckIfTestToBeSkipped
 */
static bool csirxTestCheckIfTestToBeSkipped(CsirxTestParams *testParams,
                                            uint32_t        tcType)
{
    bool skipTest = FALSE;

    /* Check whether test case is disabled */
    if(FALSE == testParams->enableTest)
    {
        GT_assert(gAppTrace, (NULL != testParams->tcName));
        GT_0trace(gAppTrace, GT_INFO, " \r\n");
        GT_1trace(gAppTrace, GT_INFO,
                  " |TEST DISABLED|:: %d ::\r\n", testParams->tcId);
        GT_1trace(gAppTrace, GT_INFO,
                  " |TEST PARAM|:: %s ::\r\n", testParams->tcName);
        if(NULL != testParams->disableInfo)
        {
            GT_1trace(gAppTrace, GT_INFO,
                      " |TEST DISABLE REASON|:: %s ::\r\n",
                      testParams->disableInfo);
        }
        else
        {
            GT_0trace(gAppTrace, GT_INFO,
                      " |TEST DISABLE REASON|:: Not Provided!! ::\r\n");
        }

        gDisableCount++;
        skipTest = TRUE;        /* Skip test */
    }

    /* Ignore test case depending on test flag and selected option */
    if((FALSE == skipTest) && (CSIRX_TCT_MISC != testParams->tcType))
    {
        if(!(tcType & testParams->tcType))
        {
            gSkipCount++;
            skipTest = TRUE;    /* Skip test */
        }
    }

    /* Ignore test case depending on run flag */
    if(FALSE == skipTest)
    {
    }

    return (skipTest);
}

/**
 *  csirxTestDisplayTestInfo
 */
static int32_t csirxTestDisplayTestInfo(CsirxTestParams *testParams)
{
    uint32_t        sCnt, testCnt;
    char           *runStatus;
    CsirxTestParams *testPrms;
    static char    *enableDisableName[] = {"Disabled", "Enabled"};
    static char     printBuf[CSIRX_TEST_PRINT_BUFSIZE];

    /* Display test info */
    sCnt = 1;
    GT_0trace(gAppTrace, GT_INFO, " \r\n");
    GT_0trace(
        gAppTrace, GT_INFO,
        " S.No        ID         Description                                   "
        "                              Status    Auto Run\r\n");
    GT_0trace(
        gAppTrace, GT_INFO,
        " ---------------------------------------------------------------------"
        "------------------------------------------------\r\n");
    for(testCnt = 0U; testCnt < CSIRX_TEST_NUM_TESTCASES; testCnt++)
    {
        testPrms = &gCsirxTestCases[testCnt];

        runStatus = "NRY";
        if(FALSE == testPrms->isRun)
        {
            if(FALSE == testPrms->enableTest)
            {
                runStatus = "NRQ";
            }
        }
        else
        {
            if(FVID2_SOK == testPrms->testResult)
            {
                runStatus = "PASS";
            }
            else
            {
                runStatus = "FAIL";
            }
        }

        GT_assert(gAppTrace, (NULL != testPrms->tcName));
        snprintf(printBuf, sizeof (printBuf),
                 "  %3d  PDK-%-4d  %-75.75s  %-8.8s  %-8.8s",
                 sCnt,
                 testPrms->tcId,
                 testPrms->tcName,
                 runStatus,
                 enableDisableName[testPrms->enableTest]);
        GT_1trace(gAppTrace, GT_INFO, "%s\r\n", printBuf);

        sCnt++;
    }
    GT_0trace(gAppTrace, GT_INFO, " \r\n");

    return (FVID2_SOK);
}

/**
 *  csirxTestGenerateTestReports
 */
static int32_t csirxTestGenerateTestReports(CsirxTestParams *testParams)
{
    uint32_t        sCnt, testCnt;
    char           *runStatus, *category, *adequacy;
    CsirxTestParams *testPrms;
    static char     printBuf[CSIRX_TEST_PRINT_BUFSIZE];

    sCnt = 1;
    GT_0trace(gAppTrace, GT_INFO, " \r\n");
    GT_0trace(
        gAppTrace, GT_INFO,
        "S.No;ID;Requirement Mapping;Description;Pass Fail Criteria;"
        "IR;Category;Test Adequacy;Test Result;\r\n");
    for(testCnt = 0U; testCnt < CSIRX_TEST_NUM_TESTCASES; testCnt++)
    {
        testPrms = &gCsirxTestCases[testCnt];

        runStatus = "NRY";
        if(FALSE == testPrms->isRun)
        {
            if(FALSE == testPrms->enableTest)
            {
                runStatus = "NRQ";
            }
        }
        else
        {
            if(FVID2_SOK == testPrms->testResult)
            {
                runStatus = "PASS";
            }
            else
            {
                runStatus = "FAIL";
            }
        }

        if(testPrms->tcType & CSIRX_TCT_FULL)
        {
            category = "Full";
        }
        else if(testPrms->tcType & CSIRX_TCT_REGRESSION)
        {
            category = "Regression";
        }
        else
        {
            category = "Sanity";
        }

        if(testPrms->tcType & CSIRX_TCT_STRESS)
        {
            adequacy = "Stress";
        }
        else if(testPrms->tcType & CSIRX_TCT_NEGATIVE)
        {
            adequacy = "Negative";
        }
        else if(testPrms->tcType & CSIRX_TCT_PERFORMANCE)
        {
            adequacy = "Performance";
        }
        else if(testPrms->tcType & CSIRX_TCT_MISC)
        {
            adequacy = "Misc";
        }
        else if(testPrms->tcType & CSIRX_TCT_API)
        {
            adequacy = "Api";
        }
        else
        {
            adequacy = "Functional";
        }

        GT_assert(gAppTrace, (NULL != testPrms->tcName));
        snprintf(printBuf, sizeof (printBuf),
                 "%d;PDK-%d;%s;;%s;%s;%s",
                 sCnt,
                 testPrms->tcId,
                 testPrms->tcName,
                 category,
                 adequacy,
                 runStatus);
        GT_1trace(gAppTrace, GT_INFO, "%s\r\n", printBuf);
        sCnt++;
    }
    GT_0trace(gAppTrace, GT_INFO, " \r\n");

    return (FVID2_SOK);
}

/**
 *  csirxTestMenuSettings
 */
static void csirxTestMenuSettings(CsirxTestParams *testParams)
{
    /*char                option;
    bool                done = FALSE;
    int32_t             value;
    CsirxTestSystemCtrl *sysCtrl = &testParams->sysCtrl;*/

    csirxTestMenuSettingsShow(testParams);

    /*while(!done)
    {
        GT_0trace(gAppTrace, GT_INFO, " Enter Choice: \r\n");
        option = AppUtils_getChar();

        switch(option)
        {
            case 'd':
            case 'D':
                GT_0trace(gAppTrace, GT_INFO,
                          " Queue count: \r\n");
                value = AppUtils_getNum();

                if(value != USE_DEF_QDEPTH)
                {
                    sysCtrl->qdepth = value;
                }
                else
                {
                    GT_0trace(
                        gAppTrace, GT_INFO,
                        " This matches with default flag, give another value\r\n");
                }
                break;

            case 'l':
            case 'L':
                GT_0trace(gAppTrace, GT_INFO,
                          " Loop count: \r\n");
                value = AppUtils_getNum();

                if(value != CSIRX_TEST_DEF_ITERATION_CNT)
                {
                    sysCtrl->loopCnt = value;
                }
                else
                {
                    GT_0trace(
                        gAppTrace, GT_INFO,
                        " This matches with default flag, give another value\r\n");
                }
                break;

            case 'r':
            case 'R':
                GT_0trace(gAppTrace, GT_INFO,
                          " Runtime Print Enable [0: Disable, 1: Enable]: \r\n");
                value = AppUtils_getNum();

                sysCtrl->rtPrintEnable = FALSE;
                if(1 == value)
                {
                    sysCtrl->rtPrintEnable = TRUE;
                }
                break;

            case 'q':
            case 'Q':
                done = TRUE;
                break;
        }
        fflush(stdin);
    }*/

    return;
}

/**
 *  csirxTestMenuMainShow
 */
static void csirxTestMenuMainShow(CsirxTestParams *testParams)
{
#if !defined (QT_BUILD)
    GT_0trace(gAppTrace, GT_INFO, gCsirxTestMenuMain0);
    csirxTestMenuCurrentSettingsShow(testParams);
    GT_0trace(gAppTrace, GT_INFO, gCsirxTestMenuMain1);
#endif

    return;
}

/**
 *  csirxTestMenuSettingsShow
 */
static void csirxTestMenuSettingsShow(CsirxTestParams *testParams)
{
#if !defined (QT_BUILD)
    GT_0trace(gAppTrace, GT_INFO, gCsirxTestMenuSettings0);
    csirxTestMenuCurrentSettingsShow(testParams);
    GT_0trace(gAppTrace, GT_INFO, gCsirxTestMenuSettings1);
#endif

    return;
}

/**
 *  csirxTestMenuCurrentSettingsShow
 */
#if !defined (QT_BUILD)
static void csirxTestMenuCurrentSettingsShow(CsirxTestParams *testParams)
{
    // static char       *enableDisableName[] = {"OFF", "ON"};

    GT_0trace(gAppTrace, GT_INFO, "\r\n Current/Default System Settings:");
    GT_0trace(gAppTrace, GT_INFO, "\r\n ------------------------");

    GT_1trace(gAppTrace, GT_INFO,
              "\r\n Uart timemout          : %d msec",
              CSIRX_TEST_UART_TIMEOUT_MSEC);
    GT_1trace(gAppTrace, GT_INFO,
              "\r\n Default Iteration count            : %d",
              CSIRX_TEST_DEF_ITERATION_CNT);
    GT_0trace(gAppTrace, GT_INFO, "\r\n ");

    return;
}
#endif

/**
 *  csirxTestSetDefaultCfg
 */
static void csirxTestSetDefaultCfg(CsirxTestParams *testParams)
{
    uint32_t        testCnt;

    /* Mark all test cases as not run and set result to PASS */
    for(testCnt = 0U; testCnt < CSIRX_TEST_NUM_TESTCASES; testCnt++)
    {
        testParams             = &gCsirxTestCases[testCnt];
        testParams->isRun      = FALSE;
        testParams->testResult = FVID2_SOK;
    }

    return;
}

/**
 *  \brief Return the test ID to run.
 */
static uint32_t csirxTestGetTestId(CsirxTestParams *testParams, uint32_t tcType)
{
    uint32_t        testCnt;
    static int32_t  testId = 0U;
#if !defined (QT_BUILD)
    CsirxTestParams *testPrms;

    GT_0trace(gAppTrace, GT_INFO, "\r\n");
    GT_0trace(gAppTrace, GT_INFO,
              " --------------------------------------\n");
    GT_0trace(gAppTrace, GT_INFO,
              " Select test to run as per below table:\n");
    GT_0trace(gAppTrace, GT_INFO,
              " --------------------------------------\n");
    GT_0trace(gAppTrace, GT_INFO, " \n");
    for(testCnt = 0U; testCnt < CSIRX_TEST_NUM_TESTCASES; testCnt++)
    {
        testPrms = &gCsirxTestCases[testCnt];
        if(FALSE != testPrms->enableTest && (tcType & testPrms->tcType))
        {
            GT_assert(gAppTrace, (NULL != testPrms->tcName));
            GT_2trace(gAppTrace, GT_INFO,
                      "%5d: %s\n", gCsirxTestCases[testCnt].tcId,
                      gCsirxTestCases[testCnt].tcName);
        }
    }
    GT_0trace(gAppTrace, GT_INFO, " \n");
    GT_0trace(gAppTrace, GT_INFO,
              " Enter Test to Run (Use UART1 console for all cores or MCU_UART1 console for MCU) \n");
#endif

    while(1U)
    {
        testId = AppUtils_getNum();
        GT_1trace(gAppTrace, GT_INFO, "%d\n", testId);
        for(testCnt = 0U; testCnt < CSIRX_TEST_NUM_TESTCASES; testCnt++)
        {
            if((tcType & CSIRX_TCT_FULL) && (tcType != CSIRX_TCT_ALL))
            {
                if(0U == testId)
                {
                    break;
                }
            }
            if(testId == gCsirxTestCases[testCnt].tcId)
            {
                break;
            }
        }

        if(testCnt == CSIRX_TEST_NUM_TESTCASES)
        {
            GT_0trace(gAppTrace, GT_INFO,
                      "Invalid Test ID. Enter Again!!\n");
        }
        else
        {
            break;
        }
    }

    return (testId);
}

static void csirxTestFillTestObj(CsirxTestParams *testParams)
{
    uint32_t taskIdx, chIdx;
    CsirxTestTaskObj *taskObj;
    CsirxInstObj *instObj;

    for (taskIdx = 0U ; taskIdx < testParams->numTasks ; taskIdx++)
    {
        taskObj = &testParams->taskObj[taskIdx];
        taskObj->testParams = testParams;
        taskObj->taskCfg =
                        Test_getTaskCfgParams(testParams->taskCfgId[taskIdx]);
        GT_assert(gAppTrace, (NULL != taskObj->taskCfg));
        taskObj->instObj.instCfgInfo =
                        Test_getInstCfgParams(taskObj->taskCfg->instCfgId);
        /* Assign framedrop buffers here: this cannot be done in 
           'csirx_testconfig.h' due to C standard */
        taskObj->instObj.instCfgInfo->frameDropBuf = (uint64_t)&gFrmDropBuf[0U];
        GT_assert(gAppTrace, (NULL != taskObj->instObj.instCfgInfo));
        instObj = &taskObj->instObj;
        for (chIdx = 0U ; chIdx < instObj->instCfgInfo->numCh ; chIdx++)
        {
            instObj->chObj[chIdx].chCfgInfo =
                    Test_getChCfgParams(instObj->instCfgInfo->chCfgId[chIdx]);
            GT_assert(gAppTrace, (NULL != instObj->chObj[chIdx].chCfgInfo));
        }
    }
}
