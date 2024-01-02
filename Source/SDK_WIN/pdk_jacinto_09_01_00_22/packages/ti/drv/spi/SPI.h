/*
 * Copyright (c) 2014-2023, Texas Instruments Incorporated
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

/**
 *  \defgroup DRV_SPI_MODULE SPI Driver
 *
 *  @{
 */
/* @} */

/**
 *  \ingroup DRV_SPI_MODULE
 *  \defgroup DRV_SPI_API_MODULE SPI Driver API
 *            SPI driver interface
 *
 *  @{
 */

/** ============================================================================
 *  @file       SPI.h
 *
 *  @brief      SPI driver interface
 *
 *  The SPI header file should be included in an application as follows:
 *  @code
 *  #include <ti/drv/spi/SPI.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef SPI_H
#define SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <ti/csl/soc.h>

/*!
 * Common SPI_control command code reservation offset.
 * SPI driver implementations should offset command codes with SPI_CMD_RESERVED
 * growing positively
 *
 * Example implementation specific command codes:
 * @code
 * #define SPIXYZ_COMMAND0         SPI_CMD_RESERVED + 0
 * #define SPIXYZ_COMMAND1         SPI_CMD_RESERVED + 1
 * @endcode
 */
#define SPI_CMD_RESERVED            (32U)
#define OSPI_CMD_RESERVED           (32U)

/*!
 * Common SPI_control status code reservation offset.
 * SPI driver implementations should offset status codes with
 * SPI_STATUS_RESERVED growing negatively.
 *
 * Example implementation specific status codes:
 * @code
 * #define SPIXYZ_STATUS_ERROR0    SPI_STATUS_RESERVED - 0
 * #define SPIXYZ_STATUS_ERROR1    SPI_STATUS_RESERVED - 1
 * #define SPIXYZ_STATUS_ERROR2    SPI_STATUS_RESERVED - 2
 * @endcode
 */
#define SPI_STATUS_RESERVED        (-((int32_t)32))

/*!
 * \brief   Successful status code returned by SPI_control().
 *
 * SPI_control() returns SPI_STATUS_SUCCESS if the control code was executed
 * successfully.
 */
#define SPI_STATUS_SUCCESS         (int32_t)(0)

/*!
 * \brief   Generic error status code returned by SPI_control().
 *
 * SPI_control() returns SPI_STATUS_ERROR if the control code was not executed
 * successfully.
 */
#define SPI_STATUS_ERROR          (-((int32_t)1))

/*!
 * \brief   An error status code returned by SPI_control() for undefined
 * command codes.
 *
 * SPI_control() returns SPI_STATUS_UNDEFINEDCMD if the control code is not
 * recognized by the driver implementation.
 */
#define SPI_STATUS_UNDEFINEDCMD   (-((int32_t)2))

/*!
 *  @brief    Wait forever define
 */
#define SPI_WAIT_FOREVER (~(0U))

/*!
 *  @brief      A handle that is returned from a SPI_open() call.
 */
typedef struct SPI_Config_s      *SPI_Handle;

/*!
 *  @brief      Status codes that are set by the SPI driver.
 */
typedef enum SPI_Status_s {
    SPI_TRANSFER_COMPLETED = 0,
    SPI_TRANSFER_STARTED,
    SPI_TRANSFER_CANCELED,
    SPI_TRANSFER_FAILED,
    SPI_TRANSFER_CSN_DEASSERT,
    SPI_TRANSFER_TIMEOUT
} SPI_Status;

/*!
 *  @brief
 *  A ::SPI_Transaction data structure is used with SPI_transfer(). It indicates
 *  how many ::SPI_FrameFormat frames are sent and received from the buffers
 *  pointed to txBuf and rxBuf.
 *  The arg variable is an user-definable argument which gets passed to the
 *  ::SPI_CallbackFxn when the SPI driver is in ::SPI_MODE_CALLBACK.
 */
typedef struct SPI_Transaction_s {
    /* User input (write-only) fields */
    size_t     count;      /*!< Number of frames for this transaction */
    uint64_t   txDmaBuf;      /*!< Pointer to a buffer with data to be transmitted */
    uint64_t   rxDmaBuf;      /*!< Pointer to a buffer to receive data */
    void      *txBuf;      /*!< Pointer to a buffer with data to be transmitted */
    void      *rxBuf;      /*!< Pointer to a buffer to receive data */
    void      *arg;        /*!< Argument to be passed to the callback function */

    /* User output (read-only) fields */
    SPI_Status status;     /*!< Status code set by SPI_transfer */

    /* Driver-use only fields */
} SPI_Transaction;

/*!
 *  @brief      The definition of a callback function used by the SPI driver
 *              when used in ::SPI_MODE_CALLBACK
 *
 *  @param      SPI_Handle          SPI_Handle
 *  @param      SPI_Transaction*    SPI_Transaction*
 */
typedef void        (*SPI_CallbackFxn) (SPI_Handle handle,
                                        SPI_Transaction * transaction);
/*!
 *  @brief
 *  Definitions for various SPI modes of operation.
 */
typedef enum SPI_Mode_s {
    SPI_MASTER      = 0,    /*!< SPI in master mode */
    SPI_SLAVE       = 1     /*!< SPI in slave mode */
} SPI_Mode;

/*!
 *  @brief
 *  Definitions for various SPI data frame formats.
 */
typedef enum SPI_FrameFormat_s {
    SPI_POL0_PHA0   = 0,    /*!< SPI mode Polarity 0 Phase 0 */
    SPI_POL0_PHA1   = 1,    /*!< SPI mode Polarity 0 Phase 1 */
    SPI_POL1_PHA0   = 2,    /*!< SPI mode Polarity 1 Phase 0 */
    SPI_POL1_PHA1   = 3,    /*!< SPI mode Polarity 1 Phase 1 */
    SPI_TI          = 4,    /*!< TI mode */
    SPI_MW          = 5     /*!< Micro-wire mode */
} SPI_FrameFormat;

/*!
 *  @brief
 *
 *  SPI transfer mode determines the whether the SPI controller operates
 *  synchronously or asynchronously. In ::SPI_MODE_BLOCKING mode SPI_transfer()
 *  blocks code execution until the SPI transaction has completed. In
 *  ::SPI_MODE_CALLBACK SPI_transfer() does not block code execution and instead
 *  calls a ::SPI_CallbackFxn callback function when the transaction has
 *  completed.
 */
typedef enum SPI_TransferMode_s {
    /*!
     * SPI_transfer() blocks execution. This mode can only be used when called
     * within a Task context
     */
    SPI_MODE_BLOCKING,
    /*!
     * SPI_transfer() does not block code execution and will call a
     * ::SPI_CallbackFxn. This mode can be used in a Task, Swi, or Hwi context.
     */
    SPI_MODE_CALLBACK
} SPI_TransferMode;

/*!
 *  @brief SPI Parameters
 *
 *  SPI Parameters are used to with the SPI_open() call. Default values for
 *  these parameters are set using SPI_Params_init().
 *
 *  @sa         SPI_Params_init()
 */
typedef struct SPI_Params_s {
    SPI_TransferMode    transferMode;       /*!< Blocking or Callback mode */
    uint32_t            transferTimeout;    /*!< Transfer timeout in system
                                                 ticks (Not supported with all
                                                 implementations */
    SPI_CallbackFxn     transferCallbackFxn;/*!< Callback function pointer */
    SPI_Mode            mode;               /*!< Master or Slave mode */
    uint32_t            bitRate;            /*!< SPI bit rate in Hz */
    uint32_t            dataSize;           /*!< SPI data frame size in bits */
    SPI_FrameFormat     frameFormat;        /*!< SPI frame format */
    void               *custom;             /*!< Custom argument used by driver
                                                 implementation */
} SPI_Params;

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_close().
 */
typedef void        (*SPI_CloseFxn)          (SPI_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_control().
 */
typedef int32_t         (*SPI_ControlFxn)        (SPI_Handle handle,
                                              uint32_t cmd,
                                              const void *arg);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_init().
 */
typedef void        (*SPI_InitFxn)           (SPI_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_open().
 */
typedef SPI_Handle  (*SPI_OpenFxn)           (SPI_Handle handle,
                                              const SPI_Params *params);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_serviceISR().
 */
typedef void        (*SPI_ServiceISRFxn)     (SPI_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_transfer().
 */
typedef bool        (*SPI_TransferFxn)       (SPI_Handle handle,
                                              SPI_Transaction *transaction);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_transferCancel().
 */
typedef void        (*SPI_TransferCancelFxn) (SPI_Handle handle);

/*!
 *  @brief      The definition of a SPI function table that contains the
 *              required set of functions to control a specific SPI driver
 *              implementation.
 */
typedef struct SPI_FxnTable_s {
    /*! Function to close the specified peripheral */
    SPI_CloseFxn            closeFxn;

    /*! Function to implementation specific control function */
    SPI_ControlFxn          controlFxn;

    /*! Function to initialize the given data object */
    SPI_InitFxn             spiInitFxn;

    /*! Function to open the specified peripheral */
    SPI_OpenFxn             openFxn;

    /*! Function to initiate a SPI data transfer */
    SPI_TransferFxn         transferFxn;

    /*! Function to cancel SPI data transfer */
    SPI_TransferCancelFxn   transferCancelFxn;

    /*! Function to service the SPI instance */
    SPI_ServiceISRFxn       serviceISRFxn;
} SPI_FxnTable;

/*!
 *  @brief SPI Global configuration
 *
 *  The SPI_Config structure contains a set of pointers used to characterize
 *  the SPI driver implementation.
 *
 *  This structure needs to be defined before calling SPI_init() and it must
 *  not be changed thereafter.
 *
 *  @sa     SPI_init()
 */
typedef struct SPI_Config_s {
    /*! Pointer to a table of driver-specific implementations of SPI APIs */
    SPI_FxnTable const *fxnTablePtr;

    /*! Pointer to a driver specific data object */
    void               *object;

    /*! Pointer to a driver specific hardware attributes structure */
    void         const *hwAttrs;
} SPI_Config;

#define SPI_MAX_CONFIG_CNT (CSL_MCSPI_PER_CNT)
#define SPI_MAX_DOMAIN_CNT (CSL_MCSPI_DOMAIN_CNT)
typedef SPI_Config SPI_config_list[SPI_MAX_DOMAIN_CNT][SPI_MAX_CONFIG_CNT];

/*!
 *  @brief      Status codes that are set by the SPI driver.
 */
typedef enum OSPI_Status_s {
    OSPI_TRANSFER_COMPLETED = 0,
    OSPI_TRANSFER_STARTED,
    OSPI_TRANSFER_CANCELED,
    OSPI_TRANSFER_FAILED,
    OSPI_TRANSFER_CSN_DEASSERT,
    OSPI_TRANSFER_TIMEOUT
} OSPI_Status;

/*!
 *  @brief
 *  A ::SPI_Transaction data structure is used with OSPI_transfer(). It indicates
 *  how many ::SPI_FrameFormat frames are sent and received from the buffers
 *  pointed to txBuf and rxBuf.
 *  The arg variable is an user-definable argument which gets passed to the
 *  ::SPI_CallbackFxn when the SPI driver is in ::SPI_MODE_CALLBACK.
 */
typedef struct OSPI_Transaction_s {
    /* User input (write-only) fields */
    size_t     count;      /*!< Number of frames for this transaction */
    void      *txBuf;      /*!< void * to a buffer with data to be transmitted */
    void      *rxBuf;      /*!< void * to a buffer to receive data */
    void      *arg;        /*!< Argument to be passed to the callback function */

    /* User output (read-only) fields */
    OSPI_Status status;     /*!< Status code set by SPI_transfer */

    /* Driver-use only fields */
} OSPI_Transaction;

/*!
 *  @brief      A handle that is returned from a SPI_open() call.
 */
typedef struct OSPI_Config_s      *OSPI_Handle;

/*!
 *  @brief      The definition of a callback function used by the SPI driver
 *              when used in ::SPI_MODE_CALLBACK
 *
 *  @param      OSPI_Handle          OSPI_Handle
 *  @param      OSPI_Transaction*    OSPI_Transaction*
 */
typedef void       (*OSPI_CallbackFxn) (OSPI_Handle handle,
                                        OSPI_Transaction * transaction);
/*!
 *  @brief
 *  Definitions for various SPI modes of operation.
 */
typedef enum OSPI_Mode_s {
    OSPI_MASTER      = 0,    /*!< SPI in master mode */
    OSPI_SLAVE       = 1     /*!< SPI in slave mode */
} OSPI_Mode;

/*!
 *  @brief
 *  Definitions for various SPI data frame formats.
 */
typedef enum OSPI_FrameFormat_s {
    OSPI_POL0_PHA0   = 0,    /*!< SPI mode Polarity 0 Phase 0 */
    OSPI_POL0_PHA1   = 1,    /*!< SPI mode Polarity 0 Phase 1 */
    OSPI_POL1_PHA0   = 2,    /*!< SPI mode Polarity 1 Phase 0 */
    OSPI_POL1_PHA1   = 3,    /*!< SPI mode Polarity 1 Phase 1 */
    OSPI_TI          = 4,    /*!< TI mode */
    OSPI_MW          = 5     /*!< Micro-wire mode */
} OSPI_FrameFormat;

/*!
 *  @brief
 *
 *  SPI transfer mode determines the whether the SPI controller operates
 *  synchronously or asynchronously. In ::SPI_MODE_BLOCKING mode SPI_transfer()
 *  blocks code execution until the SPI transaction has completed. In
 *  ::SPI_MODE_CALLBACK OSPI_transfer() does not block code execution and instead
 *  calls a ::SPI_CallbackFxn callback function when the transaction has
 *  completed.
 */
typedef enum OSPI_TransferMode_s {
    /*!
     * SPI_transfer() blocks execution. This mode can only be used when called
     * within a Task context
     */
    OSPI_MODE_BLOCKING,
    /*!
     * SPI_transfer() does not block code execution and will call a
     * ::SPI_CallbackFxn. This mode can be used in a Task, Swi, or Hwi context.
     */
    OSPI_MODE_CALLBACK
} OSPI_TransferMode;

/*!
 *  @brief OSPI Parameters
 *
 *  OSPI Parameters are used to with the OSPI_open() call. Default values for
 *  these parameters are set using OSPI_Params_init().
 *
 *  @sa         OSPI_Params_init()
 */
typedef struct OSPI_Params_s {
    OSPI_TransferMode   transferMode;       /*!< Blocking or Callback mode */
    uint32_t            transferTimeout;    /*!< Transfer timeout in system
                                                 ticks (Not supported with all
                                                 implementations */
    OSPI_CallbackFxn    transferCallbackFxn;/*!< Callback function pointer */
    OSPI_Mode           mode;               /*!< Master or Slave mode */
    uint32_t            bitRate;            /*!< SPI bit rate in Hz */
    uint32_t            dataSize;           /*!< SPI data frame size in bits */
    OSPI_FrameFormat    frameFormat;        /*!< SPI frame format */
    void               *custom;             /*!< Custom argument used by driver
                                                 implementation */
} OSPI_Params;

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              OSPI_close().
 */
typedef void        (*OSPI_CloseFxn)          (OSPI_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              OSPI_control().
 */
typedef int32_t         (*OSPI_ControlFxn)        (OSPI_Handle handle,
                                              uint32_t cmd,
                                              const void *arg);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              OSPI_init().
 */
typedef void        (*OSPI_InitFxn)           (OSPI_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_open().
 */
typedef OSPI_Handle  (*OSPI_OpenFxn)           (OSPI_Handle handle,
                                              const OSPI_Params *params);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_serviceISR().
 */
typedef void        (*OSPI_ServiceISRFxn)     (OSPI_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              OSPI_transfer().
 */
typedef bool        (*OSPI_TransferFxn)       (OSPI_Handle handle,
                                              OSPI_Transaction *transaction);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              OSPI_transferCancel().
 */
typedef void        (*OSPI_TransferCancelFxn) (OSPI_Handle handle);


/*!
 *  @brief      The definition of a SPI function table that contains the
 *              required set of functions to control a specific SPI driver
 *              implementation.
 */
typedef struct OSPI_FxnTable_s {
    /*! Function to close the specified peripheral */
    OSPI_CloseFxn            closeFxn;

    /*! Function to implementation specific control function */
    OSPI_ControlFxn          controlFxn;

    /*! Function to initialize the given data object */
    OSPI_InitFxn             spiInitFxn;

    /*! Function to open the specified peripheral */
    OSPI_OpenFxn             openFxn;

    /*! Function to initiate a SPI data transfer */
    OSPI_TransferFxn         transferFxn;

    /*! Function to cancel SPI data transfer */
    OSPI_TransferCancelFxn   transferCancelFxn;

    /*! Function to service the SPI instance */
    OSPI_ServiceISRFxn       serviceISRFxn;
} OSPI_FxnTable;

/*!
 *  @brief OSPI Global configuration
 *
 *  The OSPI_Config structure contains a set of pointers used to characterize
 *  the OSPI driver implementation.
 *
 *  This structure needs to be defined before calling OSPI_init() and it must
 *  not be changed thereafter.
 *
 *  @sa     OSPI_init()
 */
typedef struct OSPI_Config_s {
    /*! Pointer to a table of driver-specific implementations of SPI APIs */
    OSPI_FxnTable const *fxnTablePtr;

    /*! Pointer to a driver specific data object */
    void               *object;

    /*! Pointer to a driver specific hardware attributes structure */
    void         const *hwAttrs;
} OSPI_Config;

#define OSPI_MAX_CONFIG_CNT (CSL_OSPI_PER_CNT)
#define OSPI_MAX_DOMAIN_CNT (CSL_OSPI_DOMAIN_CNT)
typedef OSPI_Config OSPI_config_list[OSPI_MAX_DOMAIN_CNT][OSPI_MAX_CONFIG_CNT];

/*!
 *  @brief  Function to close a SPI peripheral specified by the SPI handle
 *
 *  @pre    SPI_open() has to be called first.
 *
 *  @param  handle A SPI handle returned from SPI_open()
 *
 *  @sa     SPI_open()
 */
extern void SPI_close(SPI_Handle handle);

/*!
 *  @brief  Function performs implementation specific features on a given
 *          SPI_Handle.
 *
 *  @pre    SPI_open() has to be called first.
 *
 *  @param  handle      A SPI handle returned from SPI_open()
 *
 *  @param  cmd         A command value defined by the driver specific
 *                      implementation
 *
 *  @param  arg         An optional R/W (read/write) argument that is
 *                      accompanied with cmd
 *
 *  @return Implementation specific return codes. Negative values indicate
 *          unsuccessful operations.
 *
 *  @sa     SPI_open()
 */
extern int32_t SPI_control(SPI_Handle handle, uint32_t cmd, void *arg);

/*!
 *  @brief  This function initializes the SPI module.
 *
 *  @pre    The SPI_config structure must exist and be persistent before this
 *          function can be called. This function must also be called before
 *          any other SPI driver APIs. This function call does not modify any
 *          peripheral registers.
 */
extern void SPI_init(void);

/*!
 *  @brief  This function opens a given SPI peripheral.
 *
 *  @pre    SPI controller has been initialized using SPI_init()
 *
 *  @param  domain        Logical domain number of domain where SPI IP is
 *                        present
 *
 *  @param  idx           Logical peripheral number for the SPI indexed into
 *                        the SPI_config table
 *
 *  @param  params        Pointer to an parameter block, if NULL it will use
 *                        default values. All the fields in this structure are
 *                        RO (read-only).
 *
 *  @return A SPI_Handle on success or a NULL on an error or if it has been
 *          opened already.
 *
 *  @sa     SPI_init()
 *  @sa     SPI_close()
 */
extern SPI_Handle SPI_open(uint32_t domain, uint32_t idx, SPI_Params *params);

/*!
 *  @brief  Function to close a OSPI peripheral specified by the OSPI handle
 *
 *  @pre    OSPI_open() has to be called first.
 *
 *  @param  handle A OSPI handle returned from OSPI_open()
 *
 *  @sa     OSPI_open()
 */
extern void OSPI_close(OSPI_Handle handle);

/*!
 *  @brief  Function performs implementation specific features on a given
 *          OSPI_Handle.
 *
 *  @pre    OSPI_open() has to be called first.
 *
 *  @param  handle      A OSPI handle returned from OSPI_open()
 *
 *  @param  cmd         A command value defined by the driver specific
 *                      implementation
 *
 *  @param  arg         An optional R/W (read/write) argument that is
 *                      accompanied with cmd
 *
 *  @return Implementation specific return codes. Negative values indicate
 *          unsuccessful operations.
 *
 *  @sa     OSPI_open()
 */
extern int32_t OSPI_control(OSPI_Handle handle, uint32_t cmd, void *arg);

/*!
 *  @brief  This function initializes the SPI module.
 *
 *  @pre    The OSPI_config structure must exist and be persistent before this
 *          function can be called. This function must also be called before
 *          any other OSPI driver APIs. This function call does not modify any
 *          peripheral registers.
 */
extern void OSPI_init(void);

/*!
 *  @brief  This function opens a given OSPI peripheral.
 *
 *  @pre    OSPI controller has been initialized using OSPI_init()
 *
 *  @param  domain        domain where OSPI IP is located
 * 
 *  @param  idx           Logical peripheral number for the SPI indexed into
 *                        the SPI_config table
 *
 *  @param  params        Pointer to an parameter block, if NULL it will use
 *                        default values. All the fields in this structure are
 *                        RO (read-only).
 *
 *  @return A SPI_Handle on success or a NULL on an error or if it has been
 *          opened already.
 *
 *  @sa     OSPI_init()
 *  @sa     OSPI_close()
 */
extern OSPI_Handle OSPI_open(uint32_t domain, uint32_t idx, OSPI_Params *params);

/*!
 *  @brief  Function to initialize the OSPI_Params struct to its defaults
 *
 *  @param  params      An pointer to OSPI_Params structure for
 *                      initialization
 *
 *  Defaults values are:
 *      transferMode        = OSPI_MODE_BLOCKING
 *      transferTimeout     = OSPI_WAIT_FOREVER
 *      transferCallbackFxn = NULL
 *      mode                = OSPI_MASTER
 *      bitRate             = 1000000 (Hz)
 *      dataSize            = 8 (bits)
 *      frameFormat         = OSPI_POL0_PHA0
 */
extern void OSPI_Params_init(OSPI_Params *params);

/*!
 *  @brief  Function to perform SPI transactions
 *
 *  If the SPI is in ::SPI_MASTER mode, it will immediately start the
 *  transaction. If the SPI is in ::SPI_SLAVE mode, it prepares itself for a
 *  transaction with a SPI master.
 *
 *  In ::OSPI_MODE_BLOCKING, OSPI_transfer will block task execution until the
 *  transaction has completed.
 *
 *  In ::OSPI_MODE_CALLBACK, OSPI_transfer() does not block task execution and
 *  calls a ::SPI_CallbackFxn. This makes the OSPI_tranfer() safe to be used
 *  within a Task, Swi, or Hwi context. The ::SPI_Transaction structure must
 *  stay persistent until the OSPI_transfer function has completed!
 *
 *  @param  handle      A OSPI_Handle
 *
 *  @param  spiTrans    A pointer to a SPI_Transaction. All of the fields within
 *                      transaction except SPI_Transaction.count and
 *                      SPI_Transaction.status are WO (write-only) unless
 *                      otherwise noted in the driver implementations. If a
 *                      transaction timeout has occured, SPI_Transaction.count
 *                      will contain the number of frames that were transferred.
 *
 *  @return true if started successfully; else false
 *
 *  @sa     OSPI_open
 *  @sa     OSPI_transferCancel
 */
extern bool OSPI_transfer(OSPI_Handle handle, OSPI_Transaction *spiTrans);

/*!
 *  @brief  Function to initialize the SPI_Params struct to its defaults
 *
 *  @param  params      An pointer to SPI_Params structure for
 *                      initialization
 *
 *  Defaults values are:
 *      transferMode        = SPI_MODE_BLOCKING
 *      transferTimeout     = SPI_WAIT_FOREVER
 *      transferCallbackFxn = NULL
 *      mode                = SPI_MASTER
 *      bitRate             = 1000000 (Hz)
 *      dataSize            = 8 (bits)
 *      frameFormat         = SPI_POL0_PHA0
 */
extern void SPI_Params_init(SPI_Params *params);

/*!
 *  @brief  Function to service the SPI module's interrupt service routine
 *
 *  @param  handle      A SPI_Handle
 */
extern void SPI_serviceISR(SPI_Handle handle);

/*!
 *  @brief  Function to perform SPI transactions
 *
 *  If the SPI is in ::SPI_MASTER mode, it will immediately start the
 *  transaction. If the SPI is in ::SPI_SLAVE mode, it prepares itself for a
 *  transaction with a SPI master.
 *
 *  In ::SPI_MODE_BLOCKING, SPI_transfer will block task execution until the
 *  transaction has completed.
 *
 *  In ::SPI_MODE_CALLBACK, SPI_transfer() does not block task execution and
 *  calls a ::SPI_CallbackFxn. This makes the SPI_tranfer() safe to be used
 *  within a Task, Swi, or Hwi context. The ::SPI_Transaction structure must
 *  stay persistent until the SPI_transfer function has completed!
 *
 *  @param  handle      A SPI_Handle
 *
 *  @param  spiTrans    A pointer to a SPI_Transaction. All of the fields within
 *                      transaction except SPI_Transaction.count and
 *                      SPI_Transaction.status are WO (write-only) unless
 *                      otherwise noted in the driver implementations. If a
 *                      transaction timeout has occured, SPI_Transaction.count
 *                      will contain the number of frames that were transferred.
 *
 *  @return true if started successfully; else false
 *
 *  @sa     SPI_open
 *  @sa     SPI_transferCancel
 */
extern bool SPI_transfer(SPI_Handle handle, SPI_Transaction *spiTrans);

/*!
 *  @brief  Function to cancel SPI transactions
 *
 *  In ::SPI_MODE_BLOCKING, SPI_transferCancel has no effect.
 *
 *  In ::SPI_MODE_CALLBACK, SPI_transferCancel() will stop an SPI transfer if
 *  if one is in progress.
 *  If a transaction was in progress, its callback function will be called
 *  in context from which this API is called from. The ::SPI_CallbackFxn
 *  function can determine if the transaction was successful or not by reading
 *  the ::SPI_Status status value in the ::SPI_Transaction structure.
 *
 *  @param  handle      A SPI_Handle
 *
 *  @sa     SPI_open
 *  @sa     SPI_transfer
 */
extern void SPI_transferCancel(SPI_Handle handle);

#ifdef __cplusplus
}
#endif

#endif /* _SPI_H */

/* @} */
