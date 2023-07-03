/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_ltdc.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_LTDC_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_LTDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_LTDC_NCLUT            256    /* Number of entries in the CLUTs */

/* LCDC Register Offsets ****************************************************/

#define STM32_LTDC_SSCR_OFFSET      0x0008 /* LTDC Synchronization Size Config Register */
#define STM32_LTDC_BPCR_OFFSET      0x000c /* LTDC Back Porch Configuration Register */
#define STM32_LTDC_AWCR_OFFSET      0x0010 /* LTDC Active Width Configuration Register */
#define STM32_LTDC_TWCR_OFFSET      0x0014 /* LTDC Total Width Configuration Register */
#define STM32_LTDC_GCR_OFFSET       0x0018 /* LTDC Global Control Register */
                                           /* 0x0020 Reserved */
#define STM32_LTDC_SRCR_OFFSET      0x0024 /* LTDC Shadow Reload Configuration Register */
                                           /* 0x0028 Reserved */
#define STM32_LTDC_BCCR_OFFSET      0x002c /* LTDC Background Color Configuration Register */
                                           /* 0x0030 Reserved */
#define STM32_LTDC_IER_OFFSET       0x0034 /* LTDC Interrupt Enable Register */
#define STM32_LTDC_ISR_OFFSET       0x0038 /* LTDC Interrupt Status Register */
#define STM32_LTDC_ICR_OFFSET       0x003c /* LTDC Interrupt Clear Register */
#define STM32_LTDC_LIPCR_OFFSET     0x0040 /* LTDC Line Interrupt Position Config Register */
#define STM32_LTDC_CPSR_OFFSET      0x0044 /* LTDC Current Position Status Register */
#define STM32_LTDC_CDSR_OFFSET      0x0048 /* LTDC Current Display Status Register */
                                           /* 0x004c-0x0080 Reserved */

#define STM32_LTDC_L1CR_OFFSET      0x0084 /* LTDC Layer 1 Control Register */
#define STM32_LTDC_L1WHPCR_OFFSET   0x0088 /* LTDC Layer 1 Window Horiz Pos Config Register */
#define STM32_LTDC_L1WVPCR_OFFSET   0x008c /* LTDC Layer 1 Window Vert Pos Config Register */
#define STM32_LTDC_L1CKCR_OFFSET    0x0090 /* LTDC Layer 1 Color Keying Config Register */
#define STM32_LTDC_L1PFCR_OFFSET    0x0094 /* LTDC Layer 1 Pixel Format Configuration Register */
#define STM32_LTDC_L1CACR_OFFSET    0x0098 /* LTDC Layer 1 Constant Alpha Config Register */
#define STM32_LTDC_L1DCCR_OFFSET    0x009c /* LTDC Layer 1 Default Color Config Register */
#define STM32_LTDC_L1BFCR_OFFSET    0x00a0 /* LTDC Layer 1 Blending Factors Config Register */
                                           /* 0x00A4-0x00A8 Reserved */
#define STM32_LTDC_L1CFBAR_OFFSET   0x00ac /* LTDC Layer 1 Color Frame Buffer Address Register */
#define STM32_LTDC_L1CFBLR_OFFSET   0x00b0 /* LTDC Layer 1 Color Frame Buffer Length Register */
#define STM32_LTDC_L1CFBLNR_OFFSET  0x00b4 /* LTDC Layer 1 Color Frame Buffer Line Number Register */
                                           /* 0x00B8-0x00C0 Reserved */
#define STM32_LTDC_L1CLUTWR_OFFSET  0x00c4 /* LTDC Layer 1 CLUT Write Register */
                                           /* 0x00C8-0x0100 Reserved */
#define STM32_LTDC_L2CR_OFFSET      0x0104 /* LTDC Layer 2 Control Register */
#define STM32_LTDC_L2WHPCR_OFFSET   0x0108 /* LTDC Layer 2 Window Horiz Pos Config Register */
#define STM32_LTDC_L2WVPCR_OFFSET   0x010c /* LTDC Layer 2 Window Vert Pos Config Register */
#define STM32_LTDC_L2CKCR_OFFSET    0x0110 /* LTDC Layer 2 Color Keying Config Register */
#define STM32_LTDC_L2PFCR_OFFSET    0x0114 /* LTDC Layer 2 Pixel Format Configuration Register */
#define STM32_LTDC_L2CACR_OFFSET    0x0118 /* LTDC Layer 2 Constant Alpha Config Register */
#define STM32_LTDC_L2DCCR_OFFSET    0x011c /* LTDC Layer 2 Default Color Config Register */
#define STM32_LTDC_L2BFCR_OFFSET    0x0120 /* LTDC Layer 2 Blending Factors Config Register */
                                           /* 0x0124-0x0128 Reserved */
#define STM32_LTDC_L2CFBAR_OFFSET   0x012c /* LTDC Layer 2 Color Frame Buffer Address Register */
#define STM32_LTDC_L2CFBLR_OFFSET   0x0130 /* LTDC Layer 2 Color Frame Buffer Length Register */
#define STM32_LTDC_L2CFBLNR_OFFSET  0x0134 /* LTDC Layer 2 Color Frame Buffer Line Number Register */
                                           /* 0x0138-0x0130 Reserved */
#define STM32_LTDC_L2CLUTWR_OFFSET  0x0144 /* LTDC Layer 2 CLUT Write Register */
                                           /* 0x0148-0x03ff Reserved */

/* LTDC Register Addresses **************************************************/

#define STM32_LTDC_SSCR             (STM32_LTDC_BASE + STM32_LTDC_SSCR_OFFSET)
#define STM32_LTDC_BPCR             (STM32_LTDC_BASE + STM32_LTDC_BPCR_OFFSET)
#define STM32_LTDC_AWCR             (STM32_LTDC_BASE + STM32_LTDC_AWCR_OFFSET)
#define STM32_LTDC_TWCR             (STM32_LTDC_BASE + STM32_LTDC_TWCR_OFFSET)
#define STM32_LTDC_GCR              (STM32_LTDC_BASE + STM32_LTDC_GCR_OFFSET)
#define STM32_LTDC_SRCR             (STM32_LTDC_BASE + STM32_LTDC_SRCR_OFFSET)
#define STM32_LTDC_BCCR             (STM32_LTDC_BASE + STM32_LTDC_BCCR_OFFSET)
#define STM32_LTDC_IER              (STM32_LTDC_BASE + STM32_LTDC_IER_OFFSET)
#define STM32_LTDC_ISR              (STM32_LTDC_BASE + STM32_LTDC_ISR_OFFSET)
#define STM32_LTDC_ICR              (STM32_LTDC_BASE + STM32_LTDC_ICR_OFFSET)
#define STM32_LTDC_LIPCR            (STM32_LTDC_BASE + STM32_LTDC_LIPCR_OFFSET)
#define STM32_LTDC_CPSR             (STM32_LTDC_BASE + STM32_LTDC_CPSR_OFFSET)
#define STM32_LTDC_CDSR             (STM32_LTDC_BASE + STM32_LTDC_CDSR_OFFSET)

#define STM32_LTDC_L1CR             (STM32_LTDC_BASE + STM32_LTDC_L1CR_OFFSET)
#define STM32_LTDC_L1WHPCR          (STM32_LTDC_BASE + STM32_LTDC_L1WHPCR_OFFSET)
#define STM32_LTDC_L1WVPCR          (STM32_LTDC_BASE + STM32_LTDC_L1WVPCR_OFFSET)
#define STM32_LTDC_L1CKCR           (STM32_LTDC_BASE + STM32_LTDC_L1CKCR_OFFSET)
#define STM32_LTDC_L1PFCR           (STM32_LTDC_BASE + STM32_LTDC_L1PFCR_OFFSET)
#define STM32_LTDC_L1CACR           (STM32_LTDC_BASE + STM32_LTDC_L1CACR_OFFSET)
#define STM32_LTDC_L1DCCR           (STM32_LTDC_BASE + STM32_LTDC_L1DCCR_OFFSET)
#define STM32_LTDC_L1BFCR           (STM32_LTDC_BASE + STM32_LTDC_L1BFCR_OFFSET)
#define STM32_LTDC_L1CFBAR          (STM32_LTDC_BASE + STM32_LTDC_L1CFBAR_OFFSET)
#define STM32_LTDC_L1CFBLR          (STM32_LTDC_BASE + STM32_LTDC_L1CFBLR_OFFSET)
#define STM32_LTDC_L1CFBLNR         (STM32_LTDC_BASE + STM32_LTDC_L1CFBLNR_OFFSET)
#define STM32_LTDC_L1CLUTWR         (STM32_LTDC_BASE + STM32_LTDC_L1CLUTWR_OFFSET)

#define STM32_LTDC_L2CR             (STM32_LTDC_BASE + STM32_LTDC_L2CR_OFFSET)
#define STM32_LTDC_L2WHPCR          (STM32_LTDC_BASE + STM32_LTDC_L2WHPCR_OFFSET)
#define STM32_LTDC_L2WVPCR          (STM32_LTDC_BASE + STM32_LTDC_L2WVPCR_OFFSET)
#define STM32_LTDC_L2CKCR           (STM32_LTDC_BASE + STM32_LTDC_L2CKCR_OFFSET)
#define STM32_LTDC_L2PFCR           (STM32_LTDC_BASE + STM32_LTDC_L2PFCR_OFFSET)
#define STM32_LTDC_L2CACR           (STM32_LTDC_BASE + STM32_LTDC_L2CACR_OFFSET)
#define STM32_LTDC_L2DCCR           (STM32_LTDC_BASE + STM32_LTDC_L2DCCR_OFFSET)
#define STM32_LTDC_L2BFCR           (STM32_LTDC_BASE + STM32_LTDC_L2BFCR_OFFSET)
#define STM32_LTDC_L2CFBAR          (STM32_LTDC_BASE + STM32_LTDC_L2CFBAR_OFFSET)
#define STM32_LTDC_L2CFBLR          (STM32_LTDC_BASE + STM32_LTDC_L2CFBLR_OFFSET)
#define STM32_LTDC_L2CFBLNR         (STM32_LTDC_BASE + STM32_LTDC_L2CFBLNR_OFFSET)
#define STM32_LTDC_L2CLUTWR         (STM32_LTDC_BASE + STM32_LTDC_L2CLUTWR_OFFSET)

/* LTDC Register Bit Definitions ********************************************/

/******************************************************************************/
/*                                                                            */
/*                      LCD-TFT Display Controller (LTDC)                     */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for LTDC_SSCR register  *****************/

#define LTDC_SSCR_VSH_SHIFT            (0U)
#define LTDC_SSCR_VSH_MASK            (0x7FFUL << LTDC_SSCR_VSH_SHIFT)            /*!< 0x000007FF */
#define LTDC_SSCR_VSH                LTDC_SSCR_VSH_MASK                         /*!< Vertical Synchronization Height  */
#define LTDC_SSCR_HSW_SHIFT            (16U)
#define LTDC_SSCR_HSW_MASK            (0xFFFUL << LTDC_SSCR_HSW_SHIFT)            /*!< 0x0FFF0000 */
#define LTDC_SSCR_HSW                LTDC_SSCR_HSW_MASK                         /*!< Horizontal Synchronization Width */

/********************  Bit definition for LTDC_BPCR register  *****************/

#define LTDC_BPCR_AVBP_SHIFT           (0U)
#define LTDC_BPCR_AVBP_MASK           (0x7FFUL << LTDC_BPCR_AVBP_SHIFT)           /*!< 0x000007FF */
#define LTDC_BPCR_AVBP               LTDC_BPCR_AVBP_MASK                        /*!< Accumulated Vertical Back Porch   */
#define LTDC_BPCR_AHBP_SHIFT           (16U)
#define LTDC_BPCR_AHBP_MASK           (0xFFFUL << LTDC_BPCR_AHBP_SHIFT)           /*!< 0x0FFF0000 */
#define LTDC_BPCR_AHBP               LTDC_BPCR_AHBP_MASK                        /*!< Accumulated Horizontal Back Porch */

/********************  Bit definition for LTDC_AWCR register  *****************/

#define LTDC_AWCR_AAH_SHIFT            (0U)
#define LTDC_AWCR_AAH_MASK            (0x7FFUL << LTDC_AWCR_AAH_SHIFT)            /*!< 0x000007FF */
#define LTDC_AWCR_AAH                LTDC_AWCR_AAH_MASK                         /*!< Accumulated Active height */
#define LTDC_AWCR_AAW_SHIFT            (16U)
#define LTDC_AWCR_AAW_MASK            (0xFFFUL << LTDC_AWCR_AAW_SHIFT)            /*!< 0x0FFF0000 */
#define LTDC_AWCR_AAW                LTDC_AWCR_AAW_MASK                         /*!< Accumulated Active Width */

/********************  Bit definition for LTDC_TWCR register  *****************/

#define LTDC_TWCR_TOTALH_SHIFT         (0U)
#define LTDC_TWCR_TOTALH_MASK         (0x7FFUL << LTDC_TWCR_TOTALH_SHIFT)         /*!< 0x000007FF */
#define LTDC_TWCR_TOTALH             LTDC_TWCR_TOTALH_MASK                      /*!< Total height */
#define LTDC_TWCR_TOTALW_SHIFT         (16U)
#define LTDC_TWCR_TOTALW_MASK         (0xFFFUL << LTDC_TWCR_TOTALW_SHIFT)         /*!< 0x0FFF0000 */
#define LTDC_TWCR_TOTALW(n)           ((uint32_t)(n) << LTDC_TWCR_TOTALW_SHIFT)   /*!< Total Width */

/********************  Bit definition for LTDC_GCR register  ******************/

#define LTDC_GCR_LTDCEN_SHIFT          (0U)
#define LTDC_GCR_LTDCEN_MASK          (0x1UL << LTDC_GCR_LTDCEN_SHIFT)            /*!< 0x00000001 */
#define LTDC_GCR_LTDCEN              LTDC_GCR_LTDCEN_MASK                       /*!< LCD-TFT controller enable bit       */
#define LTDC_GCR_DBW_SHIFT             (4U)
#define LTDC_GCR_DBW_MASK             (0x7UL << LTDC_GCR_DBW_SHIFT)               /*!< 0x00000070 */
#define LTDC_GCR_DBW                 LTDC_GCR_DBW_MASK                          /*!< Dither Blue Width                   */
#define LTDC_GCR_DGW_SHIFT             (8U)
#define LTDC_GCR_DGW_MASK             (0x7UL << LTDC_GCR_DGW_SHIFT)               /*!< 0x00000700 */
#define LTDC_GCR_DGW                 LTDC_GCR_DGW_MASK                          /*!< Dither Green Width                  */
#define LTDC_GCR_DRW_SHIFT             (12U)
#define LTDC_GCR_DRW_MASK             (0x7UL << LTDC_GCR_DRW_SHIFT)               /*!< 0x00007000 */
#define LTDC_GCR_DRW                 LTDC_GCR_DRW_MASK                          /*!< Dither Red Width                    */
#define LTDC_GCR_DEN_SHIFT             (16U)
#define LTDC_GCR_DEN_MASK             (0x1UL << LTDC_GCR_DEN_SHIFT)               /*!< 0x00010000 */
#define LTDC_GCR_DEN                 LTDC_GCR_DEN_MASK                          /*!< Dither Enable                       */
#define LTDC_GCR_PCPOL_SHIFT           (28U)
#define LTDC_GCR_PCPOL_MASK           (0x1UL << LTDC_GCR_PCPOL_SHIFT)             /*!< 0x10000000 */
#define LTDC_GCR_PCPOL               LTDC_GCR_PCPOL_MASK                        /*!< Pixel Clock Polarity                */
#define LTDC_GCR_DEPOL_SHIFT           (29U)
#define LTDC_GCR_DEPOL_MASK           (0x1UL << LTDC_GCR_DEPOL_SHIFT)             /*!< 0x20000000 */
#define LTDC_GCR_DEPOL               LTDC_GCR_DEPOL_MASK                        /*!< Data Enable Polarity                */
#define LTDC_GCR_VSPOL_SHIFT           (30U)
#define LTDC_GCR_VSPOL_MASK           (0x1UL << LTDC_GCR_VSPOL_SHIFT)             /*!< 0x40000000 */
#define LTDC_GCR_VSPOL               LTDC_GCR_VSPOL_MASK                        /*!< Vertical Synchronization Polarity   */
#define LTDC_GCR_HSPOL_SHIFT           (31U)
#define LTDC_GCR_HSPOL_MASK           (0x1UL << LTDC_GCR_HSPOL_SHIFT)             /*!< 0x80000000 */
#define LTDC_GCR_HSPOL               LTDC_GCR_HSPOL_MASK                        /*!< Horizontal Synchronization Polarity */


/********************  Bit definition for LTDC_SRCR register  *****************/

#define LTDC_SRCR_IMR_SHIFT            (0U)
#define LTDC_SRCR_IMR_MASK            (0x1UL << LTDC_SRCR_IMR_SHIFT)              /*!< 0x00000001 */
#define LTDC_SRCR_IMR                LTDC_SRCR_IMR_MASK                         /*!< Immediate Reload         */
#define LTDC_SRCR_VBR_SHIFT            (1U)
#define LTDC_SRCR_VBR_MASK            (0x1UL << LTDC_SRCR_VBR_SHIFT)              /*!< 0x00000002 */
#define LTDC_SRCR_VBR                LTDC_SRCR_VBR_MASK                         /*!< Vertical Blanking Reload */

/********************  Bit definition for LTDC_BCCR register  *****************/

#define LTDC_BCCR_BCBLUE_SHIFT         (0U)
#define LTDC_BCCR_BCBLUE_MASK         (0xFFUL << LTDC_BCCR_BCBLUE_SHIFT)          /*!< 0x000000FF */
#define LTDC_BCCR_BCBLUE             LTDC_BCCR_BCBLUE_MASK                      /*!< Background Blue value  */
#define LTDC_BCCR_BCGREEN_SHIFT        (8U)
#define LTDC_BCCR_BCGREEN_MASK        (0xFFUL << LTDC_BCCR_BCGREEN_SHIFT)         /*!< 0x0000FF00 */
#define LTDC_BCCR_BCGREEN            LTDC_BCCR_BCGREEN_MASK                     /*!< Background Green value */
#define LTDC_BCCR_BCRED_SHIFT          (16U)
#define LTDC_BCCR_BCRED_MASK          (0xFFUL << LTDC_BCCR_BCRED_SHIFT)           /*!< 0x00FF0000 */
#define LTDC_BCCR_BCRED              LTDC_BCCR_BCRED_MASK                       /*!< Background Red value   */

/********************  Bit definition for LTDC_IER register  ******************/

#define LTDC_IER_LIE_SHIFT             (0U)
#define LTDC_IER_LIE_MASK             (0x1UL << LTDC_IER_LIE_SHIFT)               /*!< 0x00000001 */
#define LTDC_IER_LIE                 LTDC_IER_LIE_MASK                          /*!< Line Interrupt Enable            */
#define LTDC_IER_FUIE_SHIFT            (1U)
#define LTDC_IER_FUIE_MASK            (0x1UL << LTDC_IER_FUIE_SHIFT)              /*!< 0x00000002 */
#define LTDC_IER_FUIE                LTDC_IER_FUIE_MASK                         /*!< FIFO Underrun Interrupt Enable   */
#define LTDC_IER_TERRIE_SHIFT          (2U)
#define LTDC_IER_TERRIE_MASK          (0x1UL << LTDC_IER_TERRIE_SHIFT)            /*!< 0x00000004 */
#define LTDC_IER_TERRIE              LTDC_IER_TERRIE_MASK                       /*!< Transfer Error Interrupt Enable  */
#define LTDC_IER_RRIE_SHIFT            (3U)
#define LTDC_IER_RRIE_MASK            (0x1UL << LTDC_IER_RRIE_SHIFT)              /*!< 0x00000008 */
#define LTDC_IER_RRIE                LTDC_IER_RRIE_MASK                         /*!< Register Reload interrupt enable */

/********************  Bit definition for LTDC_ISR register  ******************/

#define LTDC_ISR_LIF_SHIFT             (0U)
#define LTDC_ISR_LIF_MASK             (0x1UL << LTDC_ISR_LIF_SHIFT)               /*!< 0x00000001 */
#define LTDC_ISR_LIF                 LTDC_ISR_LIF_MASK                          /*!< Line Interrupt Flag */
#define LTDC_ISR_FUIF_SHIFT            (1U)
#define LTDC_ISR_FUIF_MASK            (0x1UL << LTDC_ISR_FUIF_SHIFT)              /*!< 0x00000002 */
#define LTDC_ISR_FUIF                LTDC_ISR_FUIF_MASK                         /*!< FIFO Underrun Interrupt Flag */
#define LTDC_ISR_TERRIF_SHIFT          (2U)
#define LTDC_ISR_TERRIF_MASK          (0x1UL << LTDC_ISR_TERRIF_SHIFT)            /*!< 0x00000004 */
#define LTDC_ISR_TERRIF              LTDC_ISR_TERRIF_MASK                       /*!< Transfer Error Interrupt Flag */
#define LTDC_ISR_RRIF_SHIFT            (3U)
#define LTDC_ISR_RRIF_MASK            (0x1UL << LTDC_ISR_RRIF_SHIFT)              /*!< 0x00000008 */
#define LTDC_ISR_RRIF                LTDC_ISR_RRIF_MASK                         /*!< Register Reload interrupt Flag */

/********************  Bit definition for LTDC_ICR register  ******************/

#define LTDC_ICR_CLIF_SHIFT            (0U)
#define LTDC_ICR_CLIF_MASK            (0x1UL << LTDC_ICR_CLIF_SHIFT)              /*!< 0x00000001 */
#define LTDC_ICR_CLIF                LTDC_ICR_CLIF_MASK                         /*!< Clears the Line Interrupt Flag */
#define LTDC_ICR_CFUIF_SHIFT           (1U)
#define LTDC_ICR_CFUIF_MASK           (0x1UL << LTDC_ICR_CFUIF_SHIFT)             /*!< 0x00000002 */
#define LTDC_ICR_CFUIF               LTDC_ICR_CFUIF_MASK                        /*!< Clears the FIFO Underrun Interrupt Flag */
#define LTDC_ICR_CTERRIF_SHIFT         (2U)
#define LTDC_ICR_CTERRIF_MASK         (0x1UL << LTDC_ICR_CTERRIF_SHIFT)           /*!< 0x00000004 */
#define LTDC_ICR_CTERRIF             LTDC_ICR_CTERRIF_MASK                      /*!< Clears the Transfer Error Interrupt Flag */
#define LTDC_ICR_CRRIF_SHIFT           (3U)
#define LTDC_ICR_CRRIF_MASK           (0x1UL << LTDC_ICR_CRRIF_SHIFT)             /*!< 0x00000008 */
#define LTDC_ICR_CRRIF               LTDC_ICR_CRRIF_MASK                        /*!< Clears Register Reload interrupt Flag */

/********************  Bit definition for LTDC_LIPCR register  ****************/

#define LTDC_LIPCR_LIPOS_SHIFT         (0U)
#define LTDC_LIPCR_LIPOS_MASK         (0x7FFUL << LTDC_LIPCR_LIPOS_SHIFT)         /*!< 0x000007FF */
#define LTDC_LIPCR_LIPOS(n)           ((uint32_t)(n) << LTDC_LIPCR_LIPOS_SHIFT)   /*!< Line Interrupt Position */

/********************  Bit definition for LTDC_CPSR register  *****************/

#define LTDC_CPSR_CYPOS_SHIFT          (0U)
#define LTDC_CPSR_CYPOS_MASK          (0xFFFFUL << LTDC_CPSR_CYPOS_SHIFT)         /*!< 0x0000FFFF */
#define LTDC_CPSR_CYPOS              LTDC_CPSR_CYPOS_MASK                       /*!< Current Y Position */
#define LTDC_CPSR_CXPOS_SHIFT          (16U)
#define LTDC_CPSR_CXPOS_MASK          (0xFFFFUL << LTDC_CPSR_CXPOS_SHIFT)         /*!< 0xFFFF0000 */
#define LTDC_CPSR_CXPOS              LTDC_CPSR_CXPOS_MASK                       /*!< Current X Position */

/********************  Bit definition for LTDC_CDSR register  *****************/

#define LTDC_CDSR_VDES_SHIFT           (0U)
#define LTDC_CDSR_VDES_MASK           (0x1UL << LTDC_CDSR_VDES_SHIFT)             /*!< 0x00000001 */
#define LTDC_CDSR_VDES               LTDC_CDSR_VDES_MASK                        /*!< Vertical Data Enable Status       */
#define LTDC_CDSR_HDES_SHIFT           (1U)
#define LTDC_CDSR_HDES_MASK           (0x1UL << LTDC_CDSR_HDES_SHIFT)             /*!< 0x00000002 */
#define LTDC_CDSR_HDES               LTDC_CDSR_HDES_MASK                        /*!< Horizontal Data Enable Status     */
#define LTDC_CDSR_VSYNCS_SHIFT         (2U)
#define LTDC_CDSR_VSYNCS_MASK         (0x1UL << LTDC_CDSR_VSYNCS_SHIFT)           /*!< 0x00000004 */
#define LTDC_CDSR_VSYNCS             LTDC_CDSR_VSYNCS_MASK                      /*!< Vertical Synchronization Status   */
#define LTDC_CDSR_HSYNCS_SHIFT         (3U)
#define LTDC_CDSR_HSYNCS_MASK         (0x1UL << LTDC_CDSR_HSYNCS_SHIFT)           /*!< 0x00000008 */
#define LTDC_CDSR_HSYNCS             LTDC_CDSR_HSYNCS_MASK                      /*!< Horizontal Synchronization Status */

/********************  Bit definition for LTDC_LxCR register  *****************/

#define LTDC_LxCR_LEN_SHIFT            (0U)
#define LTDC_LxCR_LEN_MASK            (0x1UL << LTDC_LxCR_LEN_SHIFT)              /*!< 0x00000001 */
#define LTDC_LxCR_LEN                LTDC_LxCR_LEN_MASK                         /*!< Layer Enable              */
#define LTDC_LxCR_COLKEN_SHIFT         (1U)
#define LTDC_LxCR_COLKEN_MASK         (0x1UL << LTDC_LxCR_COLKEN_SHIFT)           /*!< 0x00000002 */
#define LTDC_LxCR_COLKEN             LTDC_LxCR_COLKEN_MASK                      /*!< Color Keying Enable       */
#define LTDC_LxCR_CLUTEN_SHIFT         (4U)
#define LTDC_LxCR_CLUTEN_MASK         (0x1UL << LTDC_LxCR_CLUTEN_SHIFT)           /*!< 0x00000010 */
#define LTDC_LxCR_CLUTEN             LTDC_LxCR_CLUTEN_MASK                      /*!< Color Lockup Table Enable */

/********************  Bit definition for LTDC_LxWHPCR register  **************/

#define LTDC_LxWHPCR_WHSTPOS_SHIFT     (0U)
#define LTDC_LxWHPCR_WHSTPOS_MASK     (0xFFFUL << LTDC_LxWHPCR_WHSTPOS_SHIFT)     /*!< 0x00000FFF */
#define LTDC_LxWHPCR_WHSTPOS         LTDC_LxWHPCR_WHSTPOS_MASK                  /*!< Window Horizontal Start Position */
#define LTDC_LxWHPCR_WHSPPOS_SHIFT     (16U)
#define LTDC_LxWHPCR_WHSPPOS_MASK     (0xFFFFUL << LTDC_LxWHPCR_WHSPPOS_SHIFT)    /*!< 0xFFFF0000 */
#define LTDC_LxWHPCR_WHSPPOS         LTDC_LxWHPCR_WHSPPOS_MASK                  /*!< Window Horizontal Stop Position  */

/********************  Bit definition for LTDC_LxWVPCR register  **************/

#define LTDC_LxWVPCR_WVSTPOS_SHIFT     (0U)
#define LTDC_LxWVPCR_WVSTPOS_MASK     (0xFFFUL << LTDC_LxWVPCR_WVSTPOS_SHIFT)     /*!< 0x00000FFF */
#define LTDC_LxWVPCR_WVSTPOS         LTDC_LxWVPCR_WVSTPOS_MASK                  /*!< Window Vertical Start Position */
#define LTDC_LxWVPCR_WVSPPOS_SHIFT     (16U)
#define LTDC_LxWVPCR_WVSPPOS_MASK     (0xFFFFUL << LTDC_LxWVPCR_WVSPPOS_SHIFT)    /*!< 0xFFFF0000 */
#define LTDC_LxWVPCR_WVSPPOS         LTDC_LxWVPCR_WVSPPOS_MASK                  /*!< Window Vertical Stop Position  */

/********************  Bit definition for LTDC_LxCKCR register  ***************/

#define LTDC_LxCKCR_CKBLUE_SHIFT       (0U)
#define LTDC_LxCKCR_CKBLUE_MASK       (0xFFUL << LTDC_LxCKCR_CKBLUE_SHIFT)        /*!< 0x000000FF */
#define LTDC_LxCKCR_CKBLUE           LTDC_LxCKCR_CKBLUE_MASK                    /*!< Color Key Blue value  */
#define LTDC_LxCKCR_CKGREEN_SHIFT      (8U)
#define LTDC_LxCKCR_CKGREEN_MASK      (0xFFUL << LTDC_LxCKCR_CKGREEN_SHIFT)       /*!< 0x0000FF00 */
#define LTDC_LxCKCR_CKGREEN          LTDC_LxCKCR_CKGREEN_MASK                   /*!< Color Key Green value */
#define LTDC_LxCKCR_CKRED_SHIFT        (16U)
#define LTDC_LxCKCR_CKRED_MASK        (0xFFUL << LTDC_LxCKCR_CKRED_SHIFT)         /*!< 0x00FF0000 */
#define LTDC_LxCKCR_CKRED            LTDC_LxCKCR_CKRED_MASK                     /*!< Color Key Red value   */

/********************  Bit definition for LTDC_LxPFCR register  ***************/

#define LTDC_LxPFCR_PF_SHIFT           (0U)
#define LTDC_LxPFCR_PF_MASK           (0x7UL << LTDC_LxPFCR_PF_SHIFT)             /*!< 0x00000007 */
#define LTDC_LxPFCR_PF               LTDC_LxPFCR_PF_MASK                        /*!< Pixel Format */

/********************  Bit definition for LTDC_LxCACR register  ***************/

#define LTDC_LxCACR_CONSTA_SHIFT       (0U)
#define LTDC_LxCACR_CONSTA_MASK       (0xFFUL << LTDC_LxCACR_CONSTA_SHIFT)        /*!< 0x000000FF */
#define LTDC_LxCACR_CONSTA           LTDC_LxCACR_CONSTA_MASK                    /*!< Constant Alpha */

/********************  Bit definition for LTDC_LxDCCR register  ***************/

#define LTDC_LxDCCR_DCBLUE_SHIFT       (0U)
#define LTDC_LxDCCR_DCBLUE_MASK       (0xFFUL << LTDC_LxDCCR_DCBLUE_SHIFT)        /*!< 0x000000FF */
#define LTDC_LxDCCR_DCBLUE           LTDC_LxDCCR_DCBLUE_MASK                    /*!< Default Color Blue  */
#define LTDC_LxDCCR_DCGREEN_SHIFT      (8U)
#define LTDC_LxDCCR_DCGREEN_MASK      (0xFFUL << LTDC_LxDCCR_DCGREEN_SHIFT)       /*!< 0x0000FF00 */
#define LTDC_LxDCCR_DCGREEN          LTDC_LxDCCR_DCGREEN_MASK                   /*!< Default Color Green */
#define LTDC_LxDCCR_DCRED_SHIFT        (16U)
#define LTDC_LxDCCR_DCRED_MASK        (0xFFUL << LTDC_LxDCCR_DCRED_SHIFT)         /*!< 0x00FF0000 */
#define LTDC_LxDCCR_DCRED            LTDC_LxDCCR_DCRED_MASK                     /*!< Default Color Red   */
#define LTDC_LxDCCR_DCALPHA_SHIFT      (24U)
#define LTDC_LxDCCR_DCALPHA_MASK      (0xFFUL << LTDC_LxDCCR_DCALPHA_SHIFT)       /*!< 0xFF000000 */
#define LTDC_LxDCCR_DCALPHA          LTDC_LxDCCR_DCALPHA_MASK                   /*!< Default Color Alpha */

/********************  Bit definition for LTDC_LxBFCR register  ***************/

#define LTDC_LxBFCR_BF2_SHIFT          (0U)
#define LTDC_LxBFCR_BF2_MASK          (0x7UL << LTDC_LxBFCR_BF2_SHIFT)            /*!< 0x00000007 */
#define LTDC_LxBFCR_BF2              LTDC_LxBFCR_BF2_MASK                       /*!< Blending Factor 2 */
#define LTDC_LxBFCR_BF1_SHIFT          (8U)
#define LTDC_LxBFCR_BF1_MASK          (0x7UL << LTDC_LxBFCR_BF1_SHIFT)            /*!< 0x00000700 */
#define LTDC_LxBFCR_BF1              LTDC_LxBFCR_BF1_MASK                       /*!< Blending Factor 1 */

/********************  Bit definition for LTDC_LxCFBAR register  **************/

#define LTDC_LxCFBAR_CFBADD_SHIFT      (0U)
#define LTDC_LxCFBAR_CFBADD_MASK      (0xFFFFFFFFUL << LTDC_LxCFBAR_CFBADD_SHIFT) /*!< 0xFFFFFFFF */
#define LTDC_LxCFBAR_CFBADD          LTDC_LxCFBAR_CFBADD_MASK                   /*!< Color Frame Buffer Start Address */

/********************  Bit definition for LTDC_LxCFBLR register  **************/

#define LTDC_LxCFBLR_CFBLL_SHIFT       (0U)
#define LTDC_LxCFBLR_CFBLL_MASK       (0x1FFFUL << LTDC_LxCFBLR_CFBLL_SHIFT)      /*!< 0x00001FFF */
#define LTDC_LxCFBLR_CFBLL           LTDC_LxCFBLR_CFBLL_MASK                    /*!< Color Frame Buffer Line Length    */
#define LTDC_LxCFBLR_CFBP_SHIFT        (16U)
#define LTDC_LxCFBLR_CFBP_MASK        (0x1FFFUL << LTDC_LxCFBLR_CFBP_SHIFT)       /*!< 0x1FFF0000 */
#define LTDC_LxCFBLR_CFBP            LTDC_LxCFBLR_CFBP_MASK                     /*!< Color Frame Buffer Pitch in bytes */

/********************  Bit definition for LTDC_LxCFBLNR register  *************/

#define LTDC_LxCFBLNR_CFBLNBR_SHIFT    (0U)
#define LTDC_LxCFBLNR_CFBLNBR_MASK    (0x7FFUL << LTDC_LxCFBLNR_CFBLNBR_SHIFT)    /*!< 0x000007FF */
#define LTDC_LxCFBLNR_CFBLNBR        LTDC_LxCFBLNR_CFBLNBR_MASK                 /*!< Frame Buffer Line Number */

/********************  Bit definition for LTDC_LxCLUTWR register  *************/

#define LTDC_LxCLUTWR_BLUE_SHIFT       (0U)
#define LTDC_LxCLUTWR_BLUE_MASK       (0xFFUL << LTDC_LxCLUTWR_BLUE_SHIFT)        /*!< 0x000000FF */
#define LTDC_LxCLUTWR_BLUE           LTDC_LxCLUTWR_BLUE_MASK                    /*!< Blue value   */
#define LTDC_LxCLUTWR_GREEN_SHIFT      (8U)
#define LTDC_LxCLUTWR_GREEN_MASK      (0xFFUL << LTDC_LxCLUTWR_GREEN_SHIFT)       /*!< 0x0000FF00 */
#define LTDC_LxCLUTWR_GREEN          LTDC_LxCLUTWR_GREEN_MASK                   /*!< Green value  */
#define LTDC_LxCLUTWR_RED_SHIFT        (16U)
#define LTDC_LxCLUTWR_RED_MASK        (0xFFUL << LTDC_LxCLUTWR_RED_SHIFT)         /*!< 0x00FF0000 */
#define LTDC_LxCLUTWR_RED            LTDC_LxCLUTWR_RED_MASK                     /*!< Red value    */
#define LTDC_LxCLUTWR_CLUTADD_SHIFT    (24U)
#define LTDC_LxCLUTWR_CLUTADD_MASK    (0xFFUL << LTDC_LxCLUTWR_CLUTADD_SHIFT)     /*!< 0xFF000000 */
#define LTDC_LxCLUTWR_CLUTADD        LTDC_LxCLUTWR_CLUTADD_MASK                 /*!< CLUT address */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/** @defgroup LTDC_Layer LTDC Layer
  * @{
  */
#define LTDC_LAYER_1                      0x00000000U   /*!< LTDC Layer 1 */
#define LTDC_LAYER_2                      0x00000001U   /*!< LTDC Layer 2 */
/**
  * @}
  */

/** @defgroup LTDC_HS_POLARITY LTDC HS POLARITY
  * @{
  */
#define LTDC_HSPOLARITY_AL                0x00000000U   /*!< Horizontal Synchronization is active low. */
#define LTDC_HSPOLARITY_AH                LTDC_GCR_HSPOL            /*!< Horizontal Synchronization is active high. */
/**
  * @}
  */

/** @defgroup LTDC_VS_POLARITY LTDC VS POLARITY
  * @{
  */
#define LTDC_VSPOLARITY_AL                0x00000000U   /*!< Vertical Synchronization is active low. */
#define LTDC_VSPOLARITY_AH                LTDC_GCR_VSPOL            /*!< Vertical Synchronization is active high. */
/**
  * @}
  */

/** @defgroup LTDC_DE_POLARITY LTDC DE POLARITY
  * @{
  */
#define LTDC_DEPOLARITY_AL                0x00000000U   /*!< Data Enable, is active low. */
#define LTDC_DEPOLARITY_AH                LTDC_GCR_DEPOL            /*!< Data Enable, is active high. */
/**
  * @}
  */

/** @defgroup LTDC_PC_POLARITY LTDC PC POLARITY
  * @{
  */
#define LTDC_PCPOLARITY_IPC               0x00000000U   /*!< input pixel clock. */
#define LTDC_PCPOLARITY_IIPC              LTDC_GCR_PCPOL            /*!< inverted input pixel clock. */
/**
  * @}
  */

/** @defgroup LTDC_SYNC LTDC SYNC
  * @{
  */
#define LTDC_HORIZONTALSYNC               (LTDC_SSCR_HSW >> 16U)    /*!< Horizontal synchronization width. */
#define LTDC_VERTICALSYNC                 LTDC_SSCR_VSH             /*!< Vertical synchronization height. */
/**
  * @}
  */

/** @defgroup LTDC_BACK_COLOR LTDC BACK COLOR
  * @{
  */
#define LTDC_COLOR                        0x000000FFU   /*!< Color mask */
/**
  * @}
  */

/** @defgroup LTDC_BlendingFactor1 LTDC Blending Factor1
  * @{
  */
#define LTDC_BLENDING_FACTOR1_CA          0x00000400U   /*!< Blending factor : Cte Alpha */
#define LTDC_BLENDING_FACTOR1_PAxCA       0x00000600U   /*!< Blending factor : Cte Alpha x Pixel Alpha*/
/**
  * @}
  */

/** @defgroup LTDC_BlendingFactor2 LTDC Blending Factor2
  * @{
  */
#define LTDC_BLENDING_FACTOR2_CA          0x00000005U   /*!< Blending factor : Cte Alpha */
#define LTDC_BLENDING_FACTOR2_PAxCA       0x00000007U   /*!< Blending factor : Cte Alpha x Pixel Alpha*/
/**
  * @}
  */

/** @defgroup LTDC_Pixelformat LTDC Pixel format
  * @{
  */
#define LTDC_PIXEL_FORMAT_ARGB8888        0x00000000U   /*!< ARGB8888 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_RGB888          0x00000001U   /*!< RGB888 LTDC pixel format   */
#define LTDC_PIXEL_FORMAT_RGB565          0x00000002U   /*!< RGB565 LTDC pixel format   */
#define LTDC_PIXEL_FORMAT_ARGB1555        0x00000003U   /*!< ARGB1555 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_ARGB4444        0x00000004U   /*!< ARGB4444 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_L8              0x00000005U   /*!< L8 LTDC pixel format       */
#define LTDC_PIXEL_FORMAT_AL44            0x00000006U   /*!< AL44 LTDC pixel format     */
#define LTDC_PIXEL_FORMAT_AL88            0x00000007U   /*!< AL88 LTDC pixel format     */
/**
  * @}
  */

/** @defgroup LTDC_Alpha LTDC Alpha
  * @{
  */
#define LTDC_ALPHA                        LTDC_LxCACR_CONSTA        /*!< LTDC Constant Alpha mask */
/**
  * @}
  */

/** @defgroup LTDC_LAYER_Config LTDC LAYER Config
  * @{
  */
#define LTDC_STOPPOSITION                 (LTDC_LxWHPCR_WHSPPOS >> 16U) /*!< LTDC Layer stop position  */
#define LTDC_STARTPOSITION                LTDC_LxWHPCR_WHSTPOS          /*!< LTDC Layer start position */

#define LTDC_COLOR_FRAME_BUFFER           LTDC_LxCFBLR_CFBLL            /*!< LTDC Layer Line length    */
#define LTDC_LINE_NUMBER                  LTDC_LxCFBLNR_CFBLNBR         /*!< LTDC Layer Line number    */
/**
  * @}
  */

/** @defgroup LTDC_Interrupts LTDC Interrupts
  * @{
  */
#define LTDC_IT_LI                        LTDC_IER_LIE              /*!< LTDC Line Interrupt            */
#define LTDC_IT_FU                        LTDC_IER_FUIE             /*!< LTDC FIFO Underrun Interrupt   */
#define LTDC_IT_TE                        LTDC_IER_TERRIE           /*!< LTDC Transfer Error Interrupt  */
#define LTDC_IT_RR                        LTDC_IER_RRIE             /*!< LTDC Register Reload Interrupt */
/**
  * @}
  */

/** @defgroup LTDC_Flags LTDC Flags
  * @{
  */
#define LTDC_FLAG_LI                      LTDC_ISR_LIF              /*!< LTDC Line Interrupt Flag            */
#define LTDC_FLAG_FU                      LTDC_ISR_FUIF             /*!< LTDC FIFO Underrun interrupt Flag   */
#define LTDC_FLAG_TE                      LTDC_ISR_TERRIF           /*!< LTDC Transfer Error interrupt Flag  */
#define LTDC_FLAG_RR                      LTDC_ISR_RRIF             /*!< LTDC Register Reload interrupt Flag */
/**
  * @}
  */

/** @defgroup LTDC_Reload_Type LTDC Reload Type
  * @{
  */
#define LTDC_RELOAD_IMMEDIATE             LTDC_SRCR_IMR             /*!< Immediate Reload */
#define LTDC_RELOAD_VERTICAL_BLANKING     LTDC_SRCR_VBR             /*!< Vertical Blanking Reload */
/**
  * @}
  */


#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_LTDC_H */
