/**************************************************************************************
 * Copyright (c) 2016-2017, ARM Limited or its affiliates. All rights reserved         *
 *                                                                                     *
 * This file and the related binary are licensed under the following license:          *
 *                                                                                     *
 * ARM Object Code and Header Files License, v1.0 Redistribution.                      *
 *                                                                                     *
 * Redistribution and use of object code, header files, and documentation, without     *
 * modification, are permitted provided that the following conditions are met:         *
 *                                                                                     *
 * 1) Redistributions must reproduce the above copyright notice and the                *
 *    following disclaimer in the documentation and/or other materials                 *
 *    provided with the distribution.                                                  *
 *                                                                                     *
 * 2) Unless to the extent explicitly permitted by law, no reverse                     *
 *    engineering, decompilation, or disassembly of is permitted.                      *
 *                                                                                     *
 * 3) Redistribution and use is permitted solely for the purpose of                    *
 *    developing or executing applications that are targeted for use                   *
 *    on an ARM-based product.                                                         *
 *                                                                                     *
 * DISCLAIMER. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND                  *
 * CONTRIBUTORS "AS IS." ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT             *
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT,        *
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE          *
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED            *
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR              *
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF              *
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING                *
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                  *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                        *
 **************************************************************************************/

#ifndef _SSI_PAL_DMA_PLAT_H
#define _SSI_PAL_DMA_PLAT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   stub function, the function should initialize the DMA mapping of the platform (if needed)
 *
 * @param[in] buffSize - buffer size in Bytes
 * @param[in] physBuffAddr - physical start address of the memory to map
 *
 * @return Virtual start address of contiguous memory
 */
extern uint32_t SaSi_PalDmaInit(uint32_t buffSize,
                                uint32_t physBuffAddr);

/**
 * @brief   free system resources created in PD_PAL_DmaInit()
 *
 *
 * @return void
 */
extern void SaSi_PalDmaTerminate(void);
#ifdef __cplusplus
}
#endif

#endif
