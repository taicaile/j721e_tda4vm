#ifndef _PKAFW_H
#define _PKAFW_H

/******************************************************************************
 * FILE PURPOSE: PKA Firmware Image Definitions for the SA LLD 
 ******************************************************************************
 * FILE NAME:   pkafw.h
 *
 * DESCRIPTION: Define SA LLD PKA firmware image related constants and data 
 *  structures.  
 *
 * REVISION HISTORY:
 *
 *  TEXAS INSTRUMENTS TEXT FILE LICENSE
 * 
 *   Copyright (c) 2016-2018 Texas Instruments Incorporated
 * 
 *  All rights reserved not granted herein.
 *  
 *  Limited License.  
 * 
 *  Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive 
 *  license under copyrights and patents it now or hereafter owns or controls to 
 *  make, have made, use, import, offer to sell and sell ("Utilize") this software 
 *  subject to the terms herein.  With respect to the foregoing patent license, 
 *  such license is granted  solely to the extent that any such patent is necessary 
 *  to Utilize the software alone.  The patent license shall not apply to any 
 *  combinations which include this software, other than combinations with devices 
 *  manufactured by or for TI (�TI Devices�).  No hardware patent is licensed hereunder.
 * 
 *  Redistributions must preserve existing copyright notices and reproduce this license 
 *  (including the above copyright notice and the disclaimer and (if applicable) source 
 *  code license limitations below) in the documentation and/or other materials provided 
 *  with the distribution.
 *  
 *  Redistribution and use in binary form, without modification, are permitted provided 
 *  that the following conditions are met:
 * 	No reverse engineering, decompilation, or disassembly of this software is 
 *   permitted with respect to any software provided in binary form.
 * 	Any redistribution and use are licensed by TI for use only with TI Devices.
 * 	Nothing shall obligate TI to provide you with source code for the software 
 *   licensed and provided to you in object code.
 *  
 *  If software source code is provided to you, modification and redistribution of the 
 *  source code are permitted provided that the following conditions are met:
 * 	Any redistribution and use of the source code, including any resulting derivative 
 *   works, are licensed by TI for use only with TI Devices.
 * 	Any redistribution and use of any object code compiled from the source code
 *   and any resulting derivative works, are licensed by TI for use only with TI Devices.
 * 
 *  Neither the name of Texas Instruments Incorporated nor the names of its suppliers 
 *  may be used to endorse or promote products derived from this software without 
 *  specific prior written permission.
 * 
 *  DISCLAIMER.
 * 
 *  THIS SOFTWARE IS PROVIDED BY TI AND TI�S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED 
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
 *  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI�S 
 *  LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 *  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 * Note: This file may be modified for use of c99 types for variables for integration into 
 *       TI software releases
 */


/* firmware_eip29t2.h
 *
 * Downloadable Firmware for EIP29T2
 */

/*****************************************************************************
* Copyright (c) 2007-2014 INSIDE Secure B.V. All Rights Reserved.
*
* This confidential and proprietary software may be used only as authorized
* by a licensing agreement from INSIDE Secure.
*
* The entire notice above must be reproduced on all authorized copies that
* may only be made to the extent permitted by a licensing agreement from
* INSIDE Secure.
*****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
typedef struct Firmware_EIP29T2_s
{
    uint16_t  Version_MaMiPa;   /* Firmware version number */
    uint16_t  WordCount;        /* Firmware image size in 32-bit words */
    const uint32_t* Image_p;    /* Pointer to the firmware image */
      

} Firmware_EIP29T2_t;

/*----------------------------------------------------------------------------
 * Firmware_EIP29T2_GetReferences
 *
 * This function returns references to the firmware images required by
 * EIP29T2 Driver Library.
 */
void
Firmware_EIP29T2_GetReferences(
        Firmware_EIP29T2_t * const FW_p);

#ifdef __cplusplus
}
#endif
  

#endif  /* _PAFW_V0_H */
