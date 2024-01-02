/*
* hw_pwmss_submodule_offsets.h
*
* Copyright (C) 2017-2019 Texas Instruments Incorporated - http://www.ti.com/
*
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

#ifndef HW_PWMSS_SUBMODULE_OFFSETS_TOP_H_
#define HW_PWMSS_SUBMODULE_OFFSETS_TOP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#if defined (SOC_AM574x) || defined (SOC_TDA2XX) || defined (SOC_TDA2PX) || defined (SOC_TDA2EX) || defined (SOC_TDA3XX) || defined (SOC_DRA72x) || defined (SOC_DRA75x) || defined (SOC_DRA78x) || defined (SOC_AM437x) || defined (SOC_AM572x) || defined (SOC_AM571x) || defined(SOC_AM335x)
#include <ti/csl/src/ip/epwm/V0/hw_pwmss_submodule_offsets.h>
#elif defined (SOC_K2G) || defined (SOC_OMAPL137) || defined (SOC_OMAPL138) || defined (SOC_AM65XX) || defined (SOC_J721E) || defined (SOC_J7200) || defined (SOC_AM64X) || defined (SOC_J721S2) || defined (SOC_J784S4) || defined (SOC_AM62X) || defined (SOC_AM62A)
#include <ti/csl/src/ip/epwm/V0_1/hw_pwmss_submodule_offsets.h>
#endif

#ifdef __cplusplus
}
#endif
#endif /* HW_PWMSS_SUBMODULE_OFFSETS_TOP_H_ */
