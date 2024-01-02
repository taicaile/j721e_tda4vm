/**
 * @file csl/csl_ecc_aggr.h
 *
 * @brief
 *  Header file for functional layer of CSL ECC_AGGR.
 *
 *  It contains the various enumerations, structure definitions and function
 *  declarations
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2016-2019, Texas Instruments, Inc.
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
*/

#ifndef CSL_ECC_AGGR_TOP_H
#define CSL_ECC_AGGR_TOP_H

#include <ti/csl/soc.h>
#include <ti/csl/csl.h>
#include <ti/csl/cslr_ecc_aggr.h>

#if defined (SOC_AM65XX) || defined (SOC_J721E) || defined (SOC_J721S2) || defined (SOC_J784S4) || defined (SOC_J7200) || defined (SOC_AM64X) || defined (SOC_AM62X) || defined (SOC_AM62A) || defined (SOC_AM62PX)
#include <ti/csl/src/ip/ecc_aggr/V1/csl_ecc_aggr.h>
#endif

#endif /* CSL_ECC_AGGR_TOP_H */