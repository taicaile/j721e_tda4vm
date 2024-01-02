//###########################################################################
//
// FILE:   lwip2lwipif_ic.c
//
// TITLE:  lwIP Interface port file.
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright: $
//###########################################################################

/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/**
 *  Copyright (c) Texas Instruments Incorporated 2021
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
 *
 * This file is dervied from the ``ethernetif.c'' skeleton Ethernet network
 * interface driver for lwIP.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* Standard language headers */
#include <stdio.h>

#include <ti/osal/osal.h>
#include <ti/osal/TaskP.h>
#include <ti/osal/SemaphoreP.h>
#include <ti/osal/ClockP.h>
#include <ti/osal/DebugP.h>


/* lwIP specific header files */
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include <lwip/netifapi.h>
#include "netif/etharp.h"
#include "netif/ppp/pppoe.h"

/* This module's header */
#include <ti/drv/enet/intercore/intercore.h>
#include <ti/drv/enet/lwipific/inc/bufpool.h>
#include <ti/drv/enet/lwipific/inc/lwip2enet_ic.h>
#include <ti/drv/enet/lwipific/inc/lwip2lwipif_ic.h>
#include "lwipopts.h"

/* ========================================================================== */
/*                             Local Macros                                   */
/* ========================================================================== */

/* Define those to better describe your network interface. */
#define IFNAME0 'i'
#define IFNAME1 'c'

#define OS_TASKPRIHIGH               (7)

#define LWIP_RX_PACKET_TASK_PRI      (TCPIP_THREAD_PRIO + 1)

#if defined (SAFERTOS)
#define LWIP_RX_PACKET_TASK_STACK_SIZE     (16U * 1024)
#define LWIP_RX_PACKET_TASK_STACK_ALIGN    LWIP_RX_PACKET_TASK_STACK_SIZE
#else
#define LWIP_RX_PACKET_TASK_STACK_SIZE     (4096)
#define LWIP_RX_PACKET_TASK_STACK_ALIGN    (32)
#endif

/* Maximum Ethernet Payload Size. */
#define ETH_MAX_PAYLOAD             (1514)

#define VLAN_TAG_SIZE               (4U)
#define ETH_FRAME_SIZE              (ETH_MAX_PAYLOAD + VLAN_TAG_SIZE)

/* Ethernet Header */
#define ETHHDR_SIZE                 (14)

/* ========================================================================== */
/*                            Local Variables                                 */
/* ========================================================================== */

static uint8_t gLwip2LwipIf_rxTaskStack[IC_ETH_MAX_VIRTUAL_IF][LWIP_RX_PACKET_TASK_STACK_SIZE]
__attribute__ ((aligned(LWIP_RX_PACKET_TASK_STACK_ALIGN)));

/* ========================================================================== */
/*                          Extern variables                                  */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                    Local Function Declarations                             */
/* ========================================================================== */

/*!
 *  @b LWIPIF_LWIP_IC_send
 *  @n
 *  This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 *  \param[in]  netif
 *      The lwip network interface structure for this ethernetif
 *  \param[in]  p
 *      the MAC packet to send (e.g. IP packet including MAC addresses and type)
 *
 *  \retval
 *      ERR_OK if the packet could be sent
 *  \retval
 *      an err_t value if the packet couldn't be sent
 */
static err_t LWIPIF_LWIP_IC_send(struct netif *netif,
                                 struct pbuf *pStackBuf)
{
    int32_t status = ERR_OK;

    IcQ_Node nodeEnq;
    Lwip2EnetIc_Handle hLwip2EnetIc;
    Ic_Object_Handle   hIcObj;
    BufPool_Buf *pDriverBuf;

    /* Get the pointer to the private data */
    hLwip2EnetIc = (Lwip2EnetIc_Handle)netif->state;
    hIcObj       = &(hLwip2EnetIc->icObj);

    /* Return if initialization if not complete */
    if(!hLwip2EnetIc->initDone)
    {
        Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.txStats.pktDropInitErr);
        status = ERR_IF;
        goto error_handling;
    }

    /* Enqueue the packet */
    if(pStackBuf == NULL)
    {
        Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.txStats.pbufNullErr);
        status = ERR_BUF;
        goto error_handling;
    }

    /* Get the pbuf payload in one contigous buffer */
    pDriverBuf = BufPool_getBuf(hLwip2EnetIc->hBufPool);
    if(pDriverBuf == NULL)
    {
        Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.driverBufNullErr);
        /*@TODO: Should we free the pbuf here ?*/
        status = ERR_BUF;
        goto error_handling;
    }
    Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.driverBufGet);

    /* If the pbuf chain contains only one buffer, then we need to manually
     * copy the payload into our buffer. This is because in this case
     * pbuf_get_contiguous returns a pointer to the payload instead of
     * copying the payload into the user supplied buffer
     */

    if((pStackBuf->len == pStackBuf->tot_len) || (pStackBuf->next == NULL))
    {
        memcpy((void*)&(pDriverBuf->payload[0]), (void*)(pStackBuf->payload),
                pStackBuf->tot_len);
        pDriverBuf->payloadLen = pStackBuf->tot_len;
    }
    else
    {
        pbuf_get_contiguous(pStackBuf, (void*)(pDriverBuf->payload),
                ETH_FRAME_SIZE, pStackBuf->tot_len, 0);
        pDriverBuf->payloadLen = pStackBuf->tot_len;
    }

    EnetOsal_cacheWb((const void *)pDriverBuf, sizeof(BufPool_Buf));

    /* Populate the queue node */
    nodeEnq.pDataBuffer = (void*)(pDriverBuf);
    nodeEnq.dataBufferLen = pStackBuf->tot_len;
    nodeEnq.packetId = hIcObj->numTxPktsSent;

    /* Push it on the queue */
    status = IcQueue_enq(hIcObj->txQ_Handle, &nodeEnq);

    if(ICQ_RETURN_SUCCESS == status)
    {
        Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.txStats.pktEnq);
        hIcObj->numTxPktsSent++;
#if !(IC_ETH_RX_POLLING_MODE)
        if(0 == (hIcObj->numTxPktsSent % hIcObj->pktNotifyThresh))
        {
            /* Prepare and send a notification to the receiver */
            retVal = Lwip2EnetIc_remoteCorePktNotify(hLwip2EnetIc);
            Lwip2EnetIc_assert(retVal == LWIP2ENETIC_OK); 
        }
#endif
    }
    else
    {
        Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.txStats.txQFullErr);
        /* Free the driver buffer */
        BufPool_freeBuf(pDriverBuf);
        Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.driverBufFree);
        status = ERR_BUF;
        goto error_handling;
    }

error_handling:
    return status;
}


/*!
 *  @b LWIPIF_LWIP_IC_recv
 *  @n
 *  This is the RX task which consumes the packets retrieved from
 *  the driver and passes them to the LwIP stack via netif->input().
 *
 *  \param[in]  netif
 *      NETIF_DEVICE structure pointer.
 *
 *  \retval
 *      void
 */
static void LWIPIF_LWIP_IC_recv(void *arg0,
                                void *arg1)
{
    struct netif *netif = (struct netif*)arg0;
    Lwip2EnetIc_Handle hLwip2EnetIc;
    struct pbuf *pStackBuf;
    BufPool_Buf *pDriverBuf;

    Ic_Object_Handle   hIcObj;
    IcQ_Node *pNodeDeq = NULL;

    /* Get the pointer to the private data */
    hLwip2EnetIc = (Lwip2EnetIc_Handle)netif->state;
    hIcObj       = &(hLwip2EnetIc->icObj);

    /* Wait for initialization to complete */
    while(!hLwip2EnetIc->initDone)
    {
        TaskP_sleep(100);
    }

    while(!hLwip2EnetIc->shutDownFlag)
    {
#if !(IC_ETH_RX_POLLING_MODE)
        SemaphoreP_pend(hLwip2EnetIc->hRxPacketSem, SemaphoreP_WAIT_FOREVER);
#else
        TaskP_sleep(IC_ETH_RX_POLLING_INTERVAL);
#endif
        pNodeDeq = IcQueue_deq(hIcObj->rxQ_Handle);

        if(pNodeDeq)
        {
            hIcObj->numRxPktsPending--;
            Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.rxStats.pktDeq);

            /* The dequeued node contains pointer to the driver buffer */
            pDriverBuf = (BufPool_Buf*)(pNodeDeq->pDataBuffer);

            EnetOsal_cacheInv((const void*)pDriverBuf, sizeof(BufPool_Buf));

            if(pDriverBuf->payloadLen > ETH_FRAME_SIZE)
            {
                Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.rxStats.largePktErr);
                /* Free the driver buffer */
                BufPool_freeBuf(pDriverBuf);
                Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.driverBufFree);
                continue;
            }

            /* Allocate a pbuf chain from the pool. */
            pStackBuf = pbuf_alloc(PBUF_RAW, pDriverBuf->payloadLen, PBUF_POOL);
            if(pStackBuf == NULL)
            {
                Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.rxStats.pbufNullErr);
                /* Free the driver buffer */
                BufPool_freeBuf(pDriverBuf);
                Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.driverBufFree);
                continue;
            }

            /* Copy data into pbuf */
            if(ERR_OK != pbuf_take(pStackBuf,
                        (void *)&(pDriverBuf->payload[0]),
                        pDriverBuf->payloadLen))
            {
                Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.rxStats.pbufTakeErr);
                pbuf_free(pStackBuf);
                pStackBuf = NULL;
                /* Free the driver buffer */
                BufPool_freeBuf(pDriverBuf);
                Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.driverBufFree);
                continue;
            }

            /* Pass pBuf to the stack for processing */
            if (netif->input(pStackBuf, netif) != ERR_OK)
            {
                Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.rxStats.netifInputErr);
                pbuf_free(pStackBuf);
                pStackBuf = NULL;
                /* Free the driver buffer */
                BufPool_freeBuf(pDriverBuf);
                Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.driverBufFree);
                continue;
            }

            Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.rxStats.pktSentToStk);
            /* We can free the driver buffer now */
            BufPool_freeBuf(pDriverBuf);
            Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.driverBufFree);

        }
        else
        {
            Lwip2EnetIcStats_addOne(&hLwip2EnetIc->stats.rxStats.rxQEmptyWarn);
        }


    } /* !hLwip2EnetIc->shutDownFlag */

    /* We are shutting down, notify that we are done */
    SemaphoreP_post(hLwip2EnetIc->hShutDownSem);
}

/*!
 *  @b LWIPIF_LWIP_IC_start
 *  @n
 *  The function is used to initialize and start the intercore transport.
 *
 *  \param[in]  pNETIFDevice
 *      NETIF_DEVICE structure pointer.
 *
 *  \retval
 *      Success -   0
 *  \retval
 *      Error   -   <0
 */
static int LWIPIF_LWIP_IC_start(struct netif *netif)
{
    int retVal = -1;
    Lwip2EnetIc_Handle hLwip2EnetIc;
    TaskP_Params params;
    uint32_t instId = 0;

    /* Get the virtual driver instance Id for which the driver object is being created */
    instId = *(uint32_t*)(netif->state);

    /* Open the translation layer, which opens the intercore driver */
    hLwip2EnetIc = Lwip2EnetIc_open(instId);

    if (NULL != hLwip2EnetIc)
    {
        /* Save off a pointer to the translation layer */
        netif->state = (void *)hLwip2EnetIc;

        /* Create the LWIP_LWIPIF_input (i.e. packet RX) task */
        TaskP_Params_init(&params);
        params.name = (const char *)"Lwipif_Lwip_ic_recv";
        params.priority       = LWIP_RX_PACKET_TASK_PRI;
        params.stack          = gLwip2LwipIf_rxTaskStack[hLwip2EnetIc->instId];
        params.stacksize      = sizeof(gLwip2LwipIf_rxTaskStack[hLwip2EnetIc->instId]);
        params.arg0           = netif;

        hLwip2EnetIc->hLWIPIF2LWIPinput = TaskP_create(&LWIPIF_LWIP_IC_recv, &params);
        Lwip2EnetIc_assert(NULL != hLwip2EnetIc->hLWIPIF2LWIPinput);

        /* Copy the MAC Address into the network interface object here. */
        memcpy(netif->hwaddr, &hLwip2EnetIc->macAddr, (uint32_t)6U);
        netif->hwaddr_len = 6U;

        netif_set_link_up(netif);

        /* Inform the world that we are operational. */
        hLwip2EnetIc->print("[LWIPIF_LWIP_IC] Interface started successfully\n");

        retVal = 0;
    }
    else
    {
        /* Note - Use System_printf here as we are not sure if Lwip2Enet print
         * is set and not null. */
    	EnetUtils_printf("[LWIPIF_LWIP] Failed to start EnetIc\n");
    }

    return retVal;
}


/*!
 *  @b LWIPIF_LWIP_IC_stop
 *  @n
 *  The function is used to de-initialize and stop the Enet
 *  controller and device.
 *
 *  \param[in] netif
 *      NETIF structure pointer.
 */
static void LWIPIF_LWIP_IC_stop(struct netif *netif)
{
    Lwip2EnetIc_Handle hLwip2EnetIc;

    /* Get the pointer to the private data */
    hLwip2EnetIc = (Lwip2EnetIc_Handle)netif->state;

    /* Call low-level close function */
    Lwip2EnetIc_close(hLwip2EnetIc);

}

/* ========================================================================== */
/*                    API/Public Function Definitions                         */
/* ========================================================================== */

/*!
 *  @b LWIPIF_LWIP_IC_Init
 *  @n
 *  The function is used to initialize and register the peripheral
 *  with the stack.
 *
 *  \param[in]  *netif
 *      NETIF structure pointer
 *
 *  \retval
 *      Success -   ERR_OK
 */
err_t LWIPIF_LWIP_IC_init(struct netif *netif)
{

    /* Populate the Network Interface Object */
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;

    /*
     * MTU is the total size of the (IP) packet that can fit into an Ethernet.
     * For Ethernet it is 1500bytes
     */
    netif->mtu = ETH_FRAME_SIZE - ETHHDR_SIZE - VLAN_TAG_SIZE;

    /* Populate the Driver Interface Functions. */
    netif->remove_callback      = LWIPIF_LWIP_IC_stop;
    netif->output               = etharp_output;
    netif->linkoutput           = LWIPIF_LWIP_IC_send;
    netif->flags               |= NETIF_FLAG_ETHARP;
    netif->flags               |= NETIF_FLAG_ETHERNET;
    netif->flags               |= NETIF_FLAG_IGMP;

    LWIPIF_LWIP_IC_start(netif);

    EnetUtils_printf("[LWIPIF_LWIP_IC] NETIF INIT SUCCESS\n");

    return ERR_OK;
}
