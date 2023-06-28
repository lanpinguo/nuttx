/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_dsi.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_DDSI_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_DDSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DSIHOST Register Offsets ****************************************************/

#define STM32_DSI_VR_OFFSET                 0x00      /*!< DSI Host Version Register,                                 Address offset: 0x00      */
#define STM32_DSI_CR_OFFSET                 0x04      /*!< DSI Host Control Register,                                 Address offset: 0x04      */
#define STM32_DSI_CCR_OFFSET                0x08      /*!< DSI HOST Clock Control Register,                           Address offset: 0x08      */
#define STM32_DSI_LVCIDR_OFFSET             0x0C      /*!< DSI Host LTDC VCID Register,                               Address offset: 0x0C      */
#define STM32_DSI_LCOLCR_OFFSET             0x10      /*!< DSI Host LTDC Color Coding Register,                       Address offset: 0x10      */
#define STM32_DSI_LPCR_OFFSET               0x14      /*!< DSI Host LTDC Polarity Configuration Register,             Address offset: 0x14      */
#define STM32_DSI_LPMCR_OFFSET              0x18      /*!< DSI Host Low-Power Mode Configuration Register,            Address offset: 0x18      */
                                                      /*!< Reserved, 0x1C - 0x2B                                                                */
#define STM32_DSI_PCR_OFFSET                0x2C      /*!< DSI Host Protocol Configuration Register,                  Address offset: 0x2C      */
#define STM32_DSI_GVCIDR_OFFSET             0x30      /*!< DSI Host Generic VCID Register,                            Address offset: 0x30      */
#define STM32_DSI_MCR_OFFSET                0x34      /*!< DSI Host Mode Configuration Register,                      Address offset: 0x34      */
#define STM32_DSI_VMCR_OFFSET               0x38      /*!< DSI Host Video Mode Configuration Register,                Address offset: 0x38      */
#define STM32_DSI_VPCR_OFFSET               0x3C      /*!< DSI Host Video Packet Configuration Register,              Address offset: 0x3C      */
#define STM32_DSI_VCCR_OFFSET               0x40      /*!< DSI Host Video Chunks Configuration Register,              Address offset: 0x40      */
#define STM32_DSI_VNPCR_OFFSET              0x44      /*!< DSI Host Video Null Packet Configuration Register,         Address offset: 0x44      */
#define STM32_DSI_VHSACR_OFFSET             0x48      /*!< DSI Host Video HSA Configuration Register,                 Address offset: 0x48      */
#define STM32_DSI_VHBPCR_OFFSET             0x4C      /*!< DSI Host Video HBP Configuration Register,                 Address offset: 0x4C      */
#define STM32_DSI_VLCR_OFFSET               0x50      /*!< DSI Host Video Line Configuration Register,                Address offset: 0x50      */
#define STM32_DSI_VVSACR_OFFSET             0x54      /*!< DSI Host Video VSA Configuration Register,                 Address offset: 0x54      */
#define STM32_DSI_VVBPCR_OFFSET             0x58      /*!< DSI Host Video VBP Configuration Register,                 Address offset: 0x58      */
#define STM32_DSI_VVFPCR_OFFSET             0x5C      /*!< DSI Host Video VFP Configuration Register,                 Address offset: 0x5C      */
#define STM32_DSI_VVACR_OFFSET              0x60      /*!< DSI Host Video VA Configuration Register,                  Address offset: 0x60      */
#define STM32_DSI_LCCR_OFFSET               0x64      /*!< DSI Host LTDC Command Configuration Register,              Address offset: 0x64      */
#define STM32_DSI_CMCR_OFFSET               0x68      /*!< DSI Host Command Mode Configuration Register,              Address offset: 0x68      */
#define STM32_DSI_GHCR_OFFSET               0x6C      /*!< DSI Host Generic Header Configuration Register,            Address offset: 0x6C      */
#define STM32_DSI_GPDR_OFFSET               0x70      /*!< DSI Host Generic Payload Data Register,                    Address offset: 0x70      */
#define STM32_DSI_GPSR_OFFSET               0x74      /*!< DSI Host Generic Packet Status Register,                   Address offset: 0x74      */
#define STM32_DSI_TCCR_OFFSET               0x78      /*!< DSI Host Timeout Counter Configuration Register,           Address offset: 0x78-0x8F */
#define STM32_DSI_TDCR_OFFSET               0x90      /*!< DSI Host 3D Configuration Register,                        Address offset: 0x90      */
#define STM32_DSI_CLCR_OFFSET               0x94      /*!< DSI Host Clock Lane Configuration Register,                Address offset: 0x94      */
#define STM32_DSI_CLTCR_OFFSET              0x98      /*!< DSI Host Clock Lane Timer Configuration Register,          Address offset: 0x98      */
#define STM32_DSI_DLTCR_OFFSET              0x9C      /*!< DSI Host Data Lane Timer Configuration Register,           Address offset: 0x9C      */
#define STM32_DSI_PCTLR_OFFSET              0xA0      /*!< DSI Host PHY Control Register,                             Address offset: 0xA0      */
#define STM32_DSI_PCONFR_OFFSET             0xA4      /*!< DSI Host PHY Configuration Register,                       Address offset: 0xA4      */
#define STM32_DSI_PUCR_OFFSET               0xA8      /*!< DSI Host PHY ULPS Control Register,                        Address offset: 0xA8      */
#define STM32_DSI_PTTCR_OFFSET              0xAC      /*!< DSI Host PHY TX Triggers Configuration Register,           Address offset: 0xAC      */
#define STM32_DSI_PSR_OFFSET                0xB0      /*!< DSI Host PHY Status Register,                              Address offset: 0xB0      */
                                                      /*!< Reserved, 0xB4 - 0xBB                                                                */
#define STM32_DSI_ISR_OFFSET                0xBC      /*!< DSI Host Interrupt & Status Register,                      Address offset: 0xBC-0xC3 */
#define STM32_DSI_IER_OFFSET                0xC4      /*!< DSI Host Interrupt Enable Register,                        Address offset: 0xC4-0xCB */
                                                      /*!< Reserved, 0xD0 - 0xD7                                                                */
#define STM32_DSI_FIR_OFFSET                0xD8      /*!< DSI Host Force Interrupt Register,                         Address offset: 0xD8-0xDF */
                                                      /*!< Reserved, 0xE0 - 0xFF                                                                */
#define STM32_DSI_VSCR_OFFSET               0x100     /*!< DSI Host Video Shadow Control Register,                    Address offset: 0x100     */
                                                      /*!< Reserved, 0x104 - 0x10B                                                              */
#define STM32_DSI_LCVCIDR_OFFSET            0x10C     /*!< DSI Host LTDC Current VCID Register,                       Address offset: 0x10C     */
#define STM32_DSI_LCCCR_OFFSET              0x110     /*!< DSI Host LTDC Current Color Coding Register,               Address offset: 0x110     */
                                                      /*!< Reserved, 0x114                                                                      */
#define STM32_DSI_LPMCCR_OFFSET             0x118     /*!< DSI Host Low-power Mode Current Configuration Register,    Address offset: 0x118     */
                                                      /*!< Reserved, 0x11C - 0x137                                                              */
#define STM32_DSI_VMCCR_OFFSET              0x138     /*!< DSI Host Video Mode Current Configuration Register,        Address offset: 0x138     */
#define STM32_DSI_VPCCR_OFFSET              0x13C     /*!< DSI Host Video Packet Current Configuration Register,      Address offset: 0x13C     */
#define STM32_DSI_VCCCR_OFFSET              0x140     /*!< DSI Host Video Chunks Current Configuration Register,     Address offset: 0x140     */
#define STM32_DSI_VNPCCR_OFFSET             0x144     /*!< DSI Host Video Null Packet Current Configuration Register, Address offset: 0x144     */
#define STM32_DSI_VHSACCR_OFFSET            0x148     /*!< DSI Host Video HSA Current Configuration Register,         Address offset: 0x148     */
#define STM32_DSI_VHBPCCR_OFFSET            0x14C     /*!< DSI Host Video HBP Current Configuration Register,         Address offset: 0x14C     */
#define STM32_DSI_VLCCR_OFFSET              0x150     /*!< DSI Host Video Line Current Configuration Register,        Address offset: 0x150     */
#define STM32_DSI_VVSACCR_OFFSET            0x154     /*!< DSI Host Video VSA Current Configuration Register,         Address offset: 0x154     */
#define STM32_DSI_VVBPCCR_OFFSET            0x158     /*!< DSI Host Video VBP Current Configuration Register,         Address offset: 0x158     */
#define STM32_DSI_VVFPCCR_OFFSET            0x15C     /*!< DSI Host Video VFP Current Configuration Register,         Address offset: 0x15C     */
#define STM32_DSI_VVACCR_OFFSET             0x160     /*!< DSI Host Video VA Current Configuration Register,          Address offset: 0x160     */
                                                      /*!< Reserved, 0x164 - 0x18F                                                              */
#define STM32_DSI_TDCCR_OFFSET              0x190     /*!< DSI Host 3D Current Configuration Register,                Address offset: 0x190     */
                                                      /*!< Reserved, 0x194 - 0x3FF                                                                */
#define STM32_DSI_WCFGR_OFFSET              0x400     /*!< DSI Wrapper Configuration Register,                        Address offset: 0x400       */
#define STM32_DSI_WCR_OFFSET                0x404     /*!< DSI Wrapper Control Register,                              Address offset: 0x404       */
#define STM32_DSI_WIER_OFFSET               0x408     /*!< DSI Wrapper Interrupt Enable Register,                     Address offset: 0x408       */
#define STM32_DSI_WISR_OFFSET               0x40C     /*!< DSI Wrapper Interrupt and Status Register,                 Address offset: 0x40C       */
#define STM32_DSI_WIFCR_OFFSET              0x410     /*!< DSI Wrapper Interrupt Flag Clear Register,                 Address offset: 0x410       */
                                                      /*!< Reserved, 0x414                                                                        */
#define STM32_DSI_WPCR_OFFSET               0x418     /*!< DSI Wrapper PHY Configuration Register,                    Address offset: 0x418-0x42B */
                                                      /*!< Reserved, 0x42C                                                                        */
#define STM32_DSI_WRPCR_OFFSET              0x430     /*!< DSI Wrapper Regulator and PLL Control Register, Address offset: 0x430                  */


#define STM32_DSI_VR                        (STM32_DSIHOST_BASE + STM32_DSI_VR_OFFSET)            
#define STM32_DSI_CR                        (STM32_DSIHOST_BASE + STM32_DSI_CR_OFFSET)            
#define STM32_DSI_CCR                       (STM32_DSIHOST_BASE + STM32_DSI_CCR_OFFSET)           
#define STM32_DSI_LVCIDR                    (STM32_DSIHOST_BASE + STM32_DSI_LVCIDR_OFFSET)        
#define STM32_DSI_LCOLCR                    (STM32_DSIHOST_BASE + STM32_DSI_LCOLCR_OFFSET)        
#define STM32_DSI_LPCR                      (STM32_DSIHOST_BASE + STM32_DSI_LPCR_OFFSET)          
#define STM32_DSI_LPMCR                     (STM32_DSIHOST_BASE + STM32_DSI_LPMCR_OFFSET)         
                                                                                                    
#define STM32_DSI_PCR                       (STM32_DSIHOST_BASE + STM32_DSI_PCR_OFFSET)           
#define STM32_DSI_GVCIDR                    (STM32_DSIHOST_BASE + STM32_DSI_GVCIDR_OFFSET)        
#define STM32_DSI_MCR                       (STM32_DSIHOST_BASE + STM32_DSI_MCR_OFFSET)           
#define STM32_DSI_VMCR                      (STM32_DSIHOST_BASE + STM32_DSI_VMCR_OFFSET)          
#define STM32_DSI_VPCR                      (STM32_DSIHOST_BASE + STM32_DSI_VPCR_OFFSET)          
#define STM32_DSI_VCCR                      (STM32_DSIHOST_BASE + STM32_DSI_VCCR_OFFSET)          
#define STM32_DSI_VNPCR                     (STM32_DSIHOST_BASE + STM32_DSI_VNPCR_OFFSET)         
#define STM32_DSI_VHSACR                    (STM32_DSIHOST_BASE + STM32_DSI_VHSACR_OFFSET)        
#define STM32_DSI_VHBPCR                    (STM32_DSIHOST_BASE + STM32_DSI_VHBPCR_OFFSET)        
#define STM32_DSI_VLCR                      (STM32_DSIHOST_BASE + STM32_DSI_VLCR_OFFSET)          
#define STM32_DSI_VVSACR                    (STM32_DSIHOST_BASE + STM32_DSI_VVSACR_OFFSET)        
#define STM32_DSI_VVBPCR                    (STM32_DSIHOST_BASE + STM32_DSI_VVBPCR_OFFSET)        
#define STM32_DSI_VVFPCR                    (STM32_DSIHOST_BASE + STM32_DSI_VVFPCR_OFFSET)        
#define STM32_DSI_VVACR                     (STM32_DSIHOST_BASE + STM32_DSI_VVACR_OFFSET)         
#define STM32_DSI_LCCR                      (STM32_DSIHOST_BASE + STM32_DSI_LCCR_OFFSET)          
#define STM32_DSI_CMCR                      (STM32_DSIHOST_BASE + STM32_DSI_CMCR_OFFSET)          
#define STM32_DSI_GHCR                      (STM32_DSIHOST_BASE + STM32_DSI_GHCR_OFFSET)          
#define STM32_DSI_GPDR                      (STM32_DSIHOST_BASE + STM32_DSI_GPDR_OFFSET)          
#define STM32_DSI_GPSR                      (STM32_DSIHOST_BASE + STM32_DSI_GPSR_OFFSET)          
#define STM32_DSI_TCCR                      (STM32_DSIHOST_BASE + STM32_DSI_TCCR_OFFSET)          
#define STM32_DSI_TDCR                      (STM32_DSIHOST_BASE + STM32_DSI_TDCR_OFFSET)          
#define STM32_DSI_CLCR                      (STM32_DSIHOST_BASE + STM32_DSI_CLCR_OFFSET)          
#define STM32_DSI_CLTCR                     (STM32_DSIHOST_BASE + STM32_DSI_CLTCR_OFFSET)         
#define STM32_DSI_DLTCR                     (STM32_DSIHOST_BASE + STM32_DSI_DLTCR_OFFSET)         
#define STM32_DSI_PCTLR                     (STM32_DSIHOST_BASE + STM32_DSI_PCTLR_OFFSET)         
#define STM32_DSI_PCONFR                    (STM32_DSIHOST_BASE + STM32_DSI_PCONFR_OFFSET)        
#define STM32_DSI_PUCR                      (STM32_DSIHOST_BASE + STM32_DSI_PUCR_OFFSET)          
#define STM32_DSI_PTTCR                     (STM32_DSIHOST_BASE + STM32_DSI_PTTCR_OFFSET)         
#define STM32_DSI_PSR                       (STM32_DSIHOST_BASE + STM32_DSI_PSR_OFFSET)           
                                                                                                    
#define STM32_DSI_ISR                       (STM32_DSIHOST_BASE + STM32_DSI_ISR_OFFSET)           
#define STM32_DSI_IER                       (STM32_DSIHOST_BASE + STM32_DSI_IER_OFFSET)           
                                                                                                    
#define STM32_DSI_FIR                       (STM32_DSIHOST_BASE + STM32_DSI_FIR_OFFSET)           
                                                                                                    
#define STM32_DSI_VSCR                      (STM32_DSIHOST_BASE + STM32_DSI_VSCR_OFFSET)          
                                                                                                    
#define STM32_DSI_LCVCIDR                   (STM32_DSIHOST_BASE + STM32_DSI_LCVCIDR_OFFSET)       
#define STM32_DSI_LCCCR                     (STM32_DSIHOST_BASE + STM32_DSI_LCCCR_OFFSET)         
                                                                                                    
#define STM32_DSI_LPMCCR                    (STM32_DSIHOST_BASE + STM32_DSI_LPMCCR_OFFSET)        
                                                                                                    
#define STM32_DSI_VMCCR                     (STM32_DSIHOST_BASE + STM32_DSI_VMCCR_OFFSET)         
#define STM32_DSI_VPCCR                     (STM32_DSIHOST_BASE + STM32_DSI_VPCCR_OFFSET)         
#define STM32_DSI_VCCCR                     (STM32_DSIHOST_BASE + STM32_DSI_VCCCR_OFFSET)         
#define STM32_DSI_VNPCCR                    (STM32_DSIHOST_BASE + STM32_DSI_VNPCCR_OFFSET)        
#define STM32_DSI_VHSACCR                   (STM32_DSIHOST_BASE + STM32_DSI_VHSACCR_OFFSET)       
#define STM32_DSI_VHBPCCR                   (STM32_DSIHOST_BASE + STM32_DSI_VHBPCCR_OFFSET)       
#define STM32_DSI_VLCCR                     (STM32_DSIHOST_BASE + STM32_DSI_VLCCR_OFFSET)         
#define STM32_DSI_VVSACCR                   (STM32_DSIHOST_BASE + STM32_DSI_VVSACCR_OFFSET)       
#define STM32_DSI_VVBPCCR                   (STM32_DSIHOST_BASE + STM32_DSI_VVBPCCR_OFFSET)       
#define STM32_DSI_VVFPCCR                   (STM32_DSIHOST_BASE + STM32_DSI_VVFPCCR_OFFSET)       
#define STM32_DSI_VVACCR                    (STM32_DSIHOST_BASE + STM32_DSI_VVACCR_OFFSET)        
                                                                                                    
#define STM32_DSI_TDCCR                     (STM32_DSIHOST_BASE + STM32_DSI_TDCCR_OFFSET)         
                                                                                                    
#define STM32_DSI_WCFGR                     (STM32_DSIHOST_BASE + STM32_DSI_WCFGR_OFFSET)         
#define STM32_DSI_WCR                       (STM32_DSIHOST_BASE + STM32_DSI_WCR_OFFSET)           
#define STM32_DSI_WIER                      (STM32_DSIHOST_BASE + STM32_DSI_WIER_OFFSET)          
#define STM32_DSI_WISR                      (STM32_DSIHOST_BASE + STM32_DSI_WISR_OFFSET)          
#define STM32_DSI_WIFCR                     (STM32_DSIHOST_BASE + STM32_DSI_WIFCR_OFFSET)         
                                                                                                    
#define STM32_DSI_WPCR                      (STM32_DSIHOST_BASE + STM32_DSI_WPCR_OFFSET)          
                                                                                                    
#define STM32_DSI_WRPCR                     (STM32_DSIHOST_BASE + STM32_DSI_WRPCR_OFFSET)         
								            

/******************************************************************************/
/*                                                                            */
/*                     Display Serial Interface (DSI)                         */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for DSI_VR register  *****************/
#define DSI_VR_Pos                    (1U)
#define DSI_VR_Msk                    (0x18999815UL << DSI_VR_Pos)             /*!< 0x3133302A */
#define DSI_VR                        DSI_VR_Msk                               /*!< DSI Host Version */

/*******************  Bit definition for DSI_CR register  *****************/
#define DSI_CR_EN_Pos                 (0U)
#define DSI_CR_EN_Msk                 (0x1UL << DSI_CR_EN_Pos)                 /*!< 0x00000001 */
#define DSI_CR_EN                     DSI_CR_EN_Msk                            /*!< DSI Host power up and reset */

/*******************  Bit definition for DSI_CCR register  ****************/
#define DSI_CCR_TXECKDIV_Pos          (0U)
#define DSI_CCR_TXECKDIV_Msk          (0xFFUL << DSI_CCR_TXECKDIV_Pos)         /*!< 0x000000FF */
#define DSI_CCR_TXECKDIV              DSI_CCR_TXECKDIV_Msk                     /*!< TX Escape Clock Division */
#define DSI_CCR_TXECKDIV0_Pos         (0U)
#define DSI_CCR_TXECKDIV0_Msk         (0x1UL << DSI_CCR_TXECKDIV0_Pos)         /*!< 0x00000001 */
#define DSI_CCR_TXECKDIV0             DSI_CCR_TXECKDIV0_Msk
#define DSI_CCR_TXECKDIV1_Pos         (1U)
#define DSI_CCR_TXECKDIV1_Msk         (0x1UL << DSI_CCR_TXECKDIV1_Pos)         /*!< 0x00000002 */
#define DSI_CCR_TXECKDIV1             DSI_CCR_TXECKDIV1_Msk
#define DSI_CCR_TXECKDIV2_Pos         (2U)
#define DSI_CCR_TXECKDIV2_Msk         (0x1UL << DSI_CCR_TXECKDIV2_Pos)         /*!< 0x00000004 */
#define DSI_CCR_TXECKDIV2             DSI_CCR_TXECKDIV2_Msk
#define DSI_CCR_TXECKDIV3_Pos         (3U)
#define DSI_CCR_TXECKDIV3_Msk         (0x1UL << DSI_CCR_TXECKDIV3_Pos)         /*!< 0x00000008 */
#define DSI_CCR_TXECKDIV3             DSI_CCR_TXECKDIV3_Msk
#define DSI_CCR_TXECKDIV4_Pos         (4U)
#define DSI_CCR_TXECKDIV4_Msk         (0x1UL << DSI_CCR_TXECKDIV4_Pos)         /*!< 0x00000010 */
#define DSI_CCR_TXECKDIV4             DSI_CCR_TXECKDIV4_Msk
#define DSI_CCR_TXECKDIV5_Pos         (5U)
#define DSI_CCR_TXECKDIV5_Msk         (0x1UL << DSI_CCR_TXECKDIV5_Pos)         /*!< 0x00000020 */
#define DSI_CCR_TXECKDIV5             DSI_CCR_TXECKDIV5_Msk
#define DSI_CCR_TXECKDIV6_Pos         (6U)
#define DSI_CCR_TXECKDIV6_Msk         (0x1UL << DSI_CCR_TXECKDIV6_Pos)         /*!< 0x00000040 */
#define DSI_CCR_TXECKDIV6             DSI_CCR_TXECKDIV6_Msk
#define DSI_CCR_TXECKDIV7_Pos         (7U)
#define DSI_CCR_TXECKDIV7_Msk         (0x1UL << DSI_CCR_TXECKDIV7_Pos)         /*!< 0x00000080 */
#define DSI_CCR_TXECKDIV7             DSI_CCR_TXECKDIV7_Msk

#define DSI_CCR_TOCKDIV_Pos           (8U)
#define DSI_CCR_TOCKDIV_Msk           (0xFFUL << DSI_CCR_TOCKDIV_Pos)          /*!< 0x0000FF00 */
#define DSI_CCR_TOCKDIV               DSI_CCR_TOCKDIV_Msk                      /*!< Timeout Clock Division */
#define DSI_CCR_TOCKDIV0_Pos          (8U)
#define DSI_CCR_TOCKDIV0_Msk          (0x1UL << DSI_CCR_TOCKDIV0_Pos)          /*!< 0x00000100 */
#define DSI_CCR_TOCKDIV0              DSI_CCR_TOCKDIV0_Msk
#define DSI_CCR_TOCKDIV1_Pos          (9U)
#define DSI_CCR_TOCKDIV1_Msk          (0x1UL << DSI_CCR_TOCKDIV1_Pos)          /*!< 0x00000200 */
#define DSI_CCR_TOCKDIV1              DSI_CCR_TOCKDIV1_Msk
#define DSI_CCR_TOCKDIV2_Pos          (10U)
#define DSI_CCR_TOCKDIV2_Msk          (0x1UL << DSI_CCR_TOCKDIV2_Pos)          /*!< 0x00000400 */
#define DSI_CCR_TOCKDIV2              DSI_CCR_TOCKDIV2_Msk
#define DSI_CCR_TOCKDIV3_Pos          (11U)
#define DSI_CCR_TOCKDIV3_Msk          (0x1UL << DSI_CCR_TOCKDIV3_Pos)          /*!< 0x00000800 */
#define DSI_CCR_TOCKDIV3              DSI_CCR_TOCKDIV3_Msk
#define DSI_CCR_TOCKDIV4_Pos          (12U)
#define DSI_CCR_TOCKDIV4_Msk          (0x1UL << DSI_CCR_TOCKDIV4_Pos)          /*!< 0x00001000 */
#define DSI_CCR_TOCKDIV4              DSI_CCR_TOCKDIV4_Msk
#define DSI_CCR_TOCKDIV5_Pos          (13U)
#define DSI_CCR_TOCKDIV5_Msk          (0x1UL << DSI_CCR_TOCKDIV5_Pos)          /*!< 0x00002000 */
#define DSI_CCR_TOCKDIV5              DSI_CCR_TOCKDIV5_Msk
#define DSI_CCR_TOCKDIV6_Pos          (14U)
#define DSI_CCR_TOCKDIV6_Msk          (0x1UL << DSI_CCR_TOCKDIV6_Pos)          /*!< 0x00004000 */
#define DSI_CCR_TOCKDIV6              DSI_CCR_TOCKDIV6_Msk
#define DSI_CCR_TOCKDIV7_Pos          (15U)
#define DSI_CCR_TOCKDIV7_Msk          (0x1UL << DSI_CCR_TOCKDIV7_Pos)          /*!< 0x00008000 */
#define DSI_CCR_TOCKDIV7              DSI_CCR_TOCKDIV7_Msk

/*******************  Bit definition for DSI_LVCIDR register  *************/
#define DSI_LVCIDR_VCID_Pos           (0U)
#define DSI_LVCIDR_VCID_Msk           (0x3UL << DSI_LVCIDR_VCID_Pos)           /*!< 0x00000003 */
#define DSI_LVCIDR_VCID               DSI_LVCIDR_VCID_Msk                      /*!< Virtual Channel ID */
#define DSI_LVCIDR_VCID0_Pos          (0U)
#define DSI_LVCIDR_VCID0_Msk          (0x1UL << DSI_LVCIDR_VCID0_Pos)          /*!< 0x00000001 */
#define DSI_LVCIDR_VCID0              DSI_LVCIDR_VCID0_Msk
#define DSI_LVCIDR_VCID1_Pos          (1U)
#define DSI_LVCIDR_VCID1_Msk          (0x1UL << DSI_LVCIDR_VCID1_Pos)          /*!< 0x00000002 */
#define DSI_LVCIDR_VCID1              DSI_LVCIDR_VCID1_Msk

/*******************  Bit definition for DSI_LCOLCR register  *************/
#define DSI_LCOLCR_COLC_Pos           (0U)
#define DSI_LCOLCR_COLC_Msk           (0xFUL << DSI_LCOLCR_COLC_Pos)           /*!< 0x0000000F */
#define DSI_LCOLCR_COLC               DSI_LCOLCR_COLC_Msk                      /*!< Color Coding */
#define DSI_LCOLCR_COLC0_Pos          (0U)
#define DSI_LCOLCR_COLC0_Msk          (0x1UL << DSI_LCOLCR_COLC0_Pos)          /*!< 0x00000001 */
#define DSI_LCOLCR_COLC0              DSI_LCOLCR_COLC0_Msk
#define DSI_LCOLCR_COLC1_Pos          (5U)
#define DSI_LCOLCR_COLC1_Msk          (0x1UL << DSI_LCOLCR_COLC1_Pos)          /*!< 0x00000020 */
#define DSI_LCOLCR_COLC1              DSI_LCOLCR_COLC1_Msk
#define DSI_LCOLCR_COLC2_Pos          (6U)
#define DSI_LCOLCR_COLC2_Msk          (0x1UL << DSI_LCOLCR_COLC2_Pos)          /*!< 0x00000040 */
#define DSI_LCOLCR_COLC2              DSI_LCOLCR_COLC2_Msk
#define DSI_LCOLCR_COLC3_Pos          (7U)
#define DSI_LCOLCR_COLC3_Msk          (0x1UL << DSI_LCOLCR_COLC3_Pos)          /*!< 0x00000080 */
#define DSI_LCOLCR_COLC3              DSI_LCOLCR_COLC3_Msk

#define DSI_LCOLCR_LPE_Pos            (8U)
#define DSI_LCOLCR_LPE_Msk            (0x1UL << DSI_LCOLCR_LPE_Pos)            /*!< 0x00000100 */
#define DSI_LCOLCR_LPE                DSI_LCOLCR_LPE_Msk                       /*!< Loosely Packet Enable */

/*******************  Bit definition for DSI_LPCR register  ***************/
#define DSI_LPCR_DEP_Pos              (0U)
#define DSI_LPCR_DEP_Msk              (0x1UL << DSI_LPCR_DEP_Pos)              /*!< 0x00000001 */
#define DSI_LPCR_DEP                  DSI_LPCR_DEP_Msk                         /*!< Data Enable Polarity */
#define DSI_LPCR_VSP_Pos              (1U)
#define DSI_LPCR_VSP_Msk              (0x1UL << DSI_LPCR_VSP_Pos)              /*!< 0x00000002 */
#define DSI_LPCR_VSP                  DSI_LPCR_VSP_Msk                         /*!< VSYNC Polarity */
#define DSI_LPCR_HSP_Pos              (2U)
#define DSI_LPCR_HSP_Msk              (0x1UL << DSI_LPCR_HSP_Pos)              /*!< 0x00000004 */
#define DSI_LPCR_HSP                  DSI_LPCR_HSP_Msk                         /*!< HSYNC Polarity */

/*******************  Bit definition for DSI_LPMCR register  **************/
#define DSI_LPMCR_VLPSIZE_Pos         (0U)
#define DSI_LPMCR_VLPSIZE_Msk         (0xFFUL << DSI_LPMCR_VLPSIZE_Pos)        /*!< 0x000000FF */
#define DSI_LPMCR_VLPSIZE             DSI_LPMCR_VLPSIZE_Msk                    /*!< VACT Largest Packet Size */
#define DSI_LPMCR_VLPSIZE0_Pos        (0U)
#define DSI_LPMCR_VLPSIZE0_Msk        (0x1UL << DSI_LPMCR_VLPSIZE0_Pos)        /*!< 0x00000001 */
#define DSI_LPMCR_VLPSIZE0            DSI_LPMCR_VLPSIZE0_Msk
#define DSI_LPMCR_VLPSIZE1_Pos        (1U)
#define DSI_LPMCR_VLPSIZE1_Msk        (0x1UL << DSI_LPMCR_VLPSIZE1_Pos)        /*!< 0x00000002 */
#define DSI_LPMCR_VLPSIZE1            DSI_LPMCR_VLPSIZE1_Msk
#define DSI_LPMCR_VLPSIZE2_Pos        (2U)
#define DSI_LPMCR_VLPSIZE2_Msk        (0x1UL << DSI_LPMCR_VLPSIZE2_Pos)        /*!< 0x00000004 */
#define DSI_LPMCR_VLPSIZE2            DSI_LPMCR_VLPSIZE2_Msk
#define DSI_LPMCR_VLPSIZE3_Pos        (3U)
#define DSI_LPMCR_VLPSIZE3_Msk        (0x1UL << DSI_LPMCR_VLPSIZE3_Pos)        /*!< 0x00000008 */
#define DSI_LPMCR_VLPSIZE3            DSI_LPMCR_VLPSIZE3_Msk
#define DSI_LPMCR_VLPSIZE4_Pos        (4U)
#define DSI_LPMCR_VLPSIZE4_Msk        (0x1UL << DSI_LPMCR_VLPSIZE4_Pos)        /*!< 0x00000010 */
#define DSI_LPMCR_VLPSIZE4            DSI_LPMCR_VLPSIZE4_Msk
#define DSI_LPMCR_VLPSIZE5_Pos        (5U)
#define DSI_LPMCR_VLPSIZE5_Msk        (0x1UL << DSI_LPMCR_VLPSIZE5_Pos)        /*!< 0x00000020 */
#define DSI_LPMCR_VLPSIZE5            DSI_LPMCR_VLPSIZE5_Msk
#define DSI_LPMCR_VLPSIZE6_Pos        (6U)
#define DSI_LPMCR_VLPSIZE6_Msk        (0x1UL << DSI_LPMCR_VLPSIZE6_Pos)        /*!< 0x00000040 */
#define DSI_LPMCR_VLPSIZE6            DSI_LPMCR_VLPSIZE6_Msk
#define DSI_LPMCR_VLPSIZE7_Pos        (7U)
#define DSI_LPMCR_VLPSIZE7_Msk        (0x1UL << DSI_LPMCR_VLPSIZE7_Pos)        /*!< 0x00000080 */
#define DSI_LPMCR_VLPSIZE7            DSI_LPMCR_VLPSIZE7_Msk

#define DSI_LPMCR_LPSIZE_Pos          (16U)
#define DSI_LPMCR_LPSIZE_Msk          (0xFFUL << DSI_LPMCR_LPSIZE_Pos)         /*!< 0x00FF0000 */
#define DSI_LPMCR_LPSIZE              DSI_LPMCR_LPSIZE_Msk                     /*!< Largest Packet Size */
#define DSI_LPMCR_LPSIZE0_Pos         (16U)
#define DSI_LPMCR_LPSIZE0_Msk         (0x1UL << DSI_LPMCR_LPSIZE0_Pos)         /*!< 0x00010000 */
#define DSI_LPMCR_LPSIZE0             DSI_LPMCR_LPSIZE0_Msk
#define DSI_LPMCR_LPSIZE1_Pos         (17U)
#define DSI_LPMCR_LPSIZE1_Msk         (0x1UL << DSI_LPMCR_LPSIZE1_Pos)         /*!< 0x00020000 */
#define DSI_LPMCR_LPSIZE1             DSI_LPMCR_LPSIZE1_Msk
#define DSI_LPMCR_LPSIZE2_Pos         (18U)
#define DSI_LPMCR_LPSIZE2_Msk         (0x1UL << DSI_LPMCR_LPSIZE2_Pos)         /*!< 0x00040000 */
#define DSI_LPMCR_LPSIZE2             DSI_LPMCR_LPSIZE2_Msk
#define DSI_LPMCR_LPSIZE3_Pos         (19U)
#define DSI_LPMCR_LPSIZE3_Msk         (0x1UL << DSI_LPMCR_LPSIZE3_Pos)         /*!< 0x00080000 */
#define DSI_LPMCR_LPSIZE3             DSI_LPMCR_LPSIZE3_Msk
#define DSI_LPMCR_LPSIZE4_Pos         (20U)
#define DSI_LPMCR_LPSIZE4_Msk         (0x1UL << DSI_LPMCR_LPSIZE4_Pos)         /*!< 0x00100000 */
#define DSI_LPMCR_LPSIZE4             DSI_LPMCR_LPSIZE4_Msk
#define DSI_LPMCR_LPSIZE5_Pos         (21U)
#define DSI_LPMCR_LPSIZE5_Msk         (0x1UL << DSI_LPMCR_LPSIZE5_Pos)         /*!< 0x00200000 */
#define DSI_LPMCR_LPSIZE5             DSI_LPMCR_LPSIZE5_Msk
#define DSI_LPMCR_LPSIZE6_Pos         (22U)
#define DSI_LPMCR_LPSIZE6_Msk         (0x1UL << DSI_LPMCR_LPSIZE6_Pos)         /*!< 0x00400000 */
#define DSI_LPMCR_LPSIZE6             DSI_LPMCR_LPSIZE6_Msk
#define DSI_LPMCR_LPSIZE7_Pos         (23U)
#define DSI_LPMCR_LPSIZE7_Msk         (0x1UL << DSI_LPMCR_LPSIZE7_Pos)         /*!< 0x00800000 */
#define DSI_LPMCR_LPSIZE7             DSI_LPMCR_LPSIZE7_Msk

/*******************  Bit definition for DSI_PCR register  ****************/
#define DSI_PCR_ETTXE_Pos             (0U)
#define DSI_PCR_ETTXE_Msk             (0x1UL << DSI_PCR_ETTXE_Pos)             /*!< 0x00000001 */
#define DSI_PCR_ETTXE                 DSI_PCR_ETTXE_Msk                        /*!< EoTp Transmission Enable */
#define DSI_PCR_ETRXE_Pos             (1U)
#define DSI_PCR_ETRXE_Msk             (0x1UL << DSI_PCR_ETRXE_Pos)             /*!< 0x00000002 */
#define DSI_PCR_ETRXE                 DSI_PCR_ETRXE_Msk                        /*!< EoTp Reception Enable */
#define DSI_PCR_BTAE_Pos              (2U)
#define DSI_PCR_BTAE_Msk              (0x1UL << DSI_PCR_BTAE_Pos)              /*!< 0x00000004 */
#define DSI_PCR_BTAE                  DSI_PCR_BTAE_Msk                         /*!< Bus Turn Around Enable */
#define DSI_PCR_ECCRXE_Pos            (3U)
#define DSI_PCR_ECCRXE_Msk            (0x1UL << DSI_PCR_ECCRXE_Pos)            /*!< 0x00000008 */
#define DSI_PCR_ECCRXE                DSI_PCR_ECCRXE_Msk                       /*!< ECC Reception Enable */
#define DSI_PCR_CRCRXE_Pos            (4U)
#define DSI_PCR_CRCRXE_Msk            (0x1UL << DSI_PCR_CRCRXE_Pos)            /*!< 0x00000010 */
#define DSI_PCR_CRCRXE                DSI_PCR_CRCRXE_Msk                       /*!< CRC Reception Enable */

/*******************  Bit definition for DSI_GVCIDR register  *************/
#define DSI_GVCIDR_VCID_Pos           (0U)
#define DSI_GVCIDR_VCID_Msk           (0x3UL << DSI_GVCIDR_VCID_Pos)           /*!< 0x00000003 */
#define DSI_GVCIDR_VCID               DSI_GVCIDR_VCID_Msk                      /*!< Virtual Channel ID */
#define DSI_GVCIDR_VCID0_Pos          (0U)
#define DSI_GVCIDR_VCID0_Msk          (0x1UL << DSI_GVCIDR_VCID0_Pos)          /*!< 0x00000001 */
#define DSI_GVCIDR_VCID0              DSI_GVCIDR_VCID0_Msk
#define DSI_GVCIDR_VCID1_Pos          (1U)
#define DSI_GVCIDR_VCID1_Msk          (0x1UL << DSI_GVCIDR_VCID1_Pos)          /*!< 0x00000002 */
#define DSI_GVCIDR_VCID1              DSI_GVCIDR_VCID1_Msk

/*******************  Bit definition for DSI_MCR register  ****************/
#define DSI_MCR_CMDM_Pos              (0U)
#define DSI_MCR_CMDM_Msk              (0x1UL << DSI_MCR_CMDM_Pos)              /*!< 0x00000001 */
#define DSI_MCR_CMDM                  DSI_MCR_CMDM_Msk                         /*!< Command Mode */

/*******************  Bit definition for DSI_VMCR register  ***************/
#define DSI_VMCR_VMT_Pos              (0U)
#define DSI_VMCR_VMT_Msk              (0x3UL << DSI_VMCR_VMT_Pos)              /*!< 0x00000003 */
#define DSI_VMCR_VMT                  DSI_VMCR_VMT_Msk                         /*!< Video Mode Type */
#define DSI_VMCR_VMT0_Pos             (0U)
#define DSI_VMCR_VMT0_Msk             (0x1UL << DSI_VMCR_VMT0_Pos)             /*!< 0x00000001 */
#define DSI_VMCR_VMT0                 DSI_VMCR_VMT0_Msk
#define DSI_VMCR_VMT1_Pos             (1U)
#define DSI_VMCR_VMT1_Msk             (0x1UL << DSI_VMCR_VMT1_Pos)             /*!< 0x00000002 */
#define DSI_VMCR_VMT1                 DSI_VMCR_VMT1_Msk

#define DSI_VMCR_LPVSAE_Pos           (8U)
#define DSI_VMCR_LPVSAE_Msk           (0x1UL << DSI_VMCR_LPVSAE_Pos)           /*!< 0x00000100 */
#define DSI_VMCR_LPVSAE               DSI_VMCR_LPVSAE_Msk                      /*!< Low-Power Vertical Sync Active Enable */
#define DSI_VMCR_LPVBPE_Pos           (9U)
#define DSI_VMCR_LPVBPE_Msk           (0x1UL << DSI_VMCR_LPVBPE_Pos)           /*!< 0x00000200 */
#define DSI_VMCR_LPVBPE               DSI_VMCR_LPVBPE_Msk                      /*!< Low-power Vertical Back-Porch Enable */
#define DSI_VMCR_LPVFPE_Pos           (10U)
#define DSI_VMCR_LPVFPE_Msk           (0x1UL << DSI_VMCR_LPVFPE_Pos)           /*!< 0x00000400 */
#define DSI_VMCR_LPVFPE               DSI_VMCR_LPVFPE_Msk                      /*!< Low-power Vertical Front-porch Enable */
#define DSI_VMCR_LPVAE_Pos            (11U)
#define DSI_VMCR_LPVAE_Msk            (0x1UL << DSI_VMCR_LPVAE_Pos)            /*!< 0x00000800 */
#define DSI_VMCR_LPVAE                DSI_VMCR_LPVAE_Msk                       /*!< Low-Power Vertical Active Enable */
#define DSI_VMCR_LPHBPE_Pos           (12U)
#define DSI_VMCR_LPHBPE_Msk           (0x1UL << DSI_VMCR_LPHBPE_Pos)           /*!< 0x00001000 */
#define DSI_VMCR_LPHBPE               DSI_VMCR_LPHBPE_Msk                      /*!< Low-Power Horizontal Back-Porch Enable */
#define DSI_VMCR_LPHFPE_Pos           (13U)
#define DSI_VMCR_LPHFPE_Msk           (0x1UL << DSI_VMCR_LPHFPE_Pos)           /*!< 0x00002000 */
#define DSI_VMCR_LPHFPE               DSI_VMCR_LPHFPE_Msk                      /*!< Low-Power Horizontal Front-Porch Enable */
#define DSI_VMCR_FBTAAE_Pos           (14U)
#define DSI_VMCR_FBTAAE_Msk           (0x1UL << DSI_VMCR_FBTAAE_Pos)           /*!< 0x00004000 */
#define DSI_VMCR_FBTAAE               DSI_VMCR_FBTAAE_Msk                      /*!< Frame Bus-Turn-Around Acknowledge Enable */
#define DSI_VMCR_LPCE_Pos             (15U)
#define DSI_VMCR_LPCE_Msk             (0x1UL << DSI_VMCR_LPCE_Pos)             /*!< 0x00008000 */
#define DSI_VMCR_LPCE                 DSI_VMCR_LPCE_Msk                        /*!< Low-Power Command Enable */
#define DSI_VMCR_PGE_Pos              (16U)
#define DSI_VMCR_PGE_Msk              (0x1UL << DSI_VMCR_PGE_Pos)              /*!< 0x00010000 */
#define DSI_VMCR_PGE                  DSI_VMCR_PGE_Msk                         /*!< Pattern Generator Enable */
#define DSI_VMCR_PGM_Pos              (20U)
#define DSI_VMCR_PGM_Msk              (0x1UL << DSI_VMCR_PGM_Pos)              /*!< 0x00100000 */
#define DSI_VMCR_PGM                  DSI_VMCR_PGM_Msk                         /*!< Pattern Generator Mode */
#define DSI_VMCR_PGO_Pos              (24U)
#define DSI_VMCR_PGO_Msk              (0x1UL << DSI_VMCR_PGO_Pos)              /*!< 0x01000000 */
#define DSI_VMCR_PGO                  DSI_VMCR_PGO_Msk                         /*!< Pattern Generator Orientation */

/*******************  Bit definition for DSI_VPCR register  ***************/
#define DSI_VPCR_VPSIZE_Pos           (0U)
#define DSI_VPCR_VPSIZE_Msk           (0x3FFFUL << DSI_VPCR_VPSIZE_Pos)        /*!< 0x00003FFF */
#define DSI_VPCR_VPSIZE               DSI_VPCR_VPSIZE_Msk                      /*!< Video Packet Size */
#define DSI_VPCR_VPSIZE0_Pos          (0U)
#define DSI_VPCR_VPSIZE0_Msk          (0x1UL << DSI_VPCR_VPSIZE0_Pos)          /*!< 0x00000001 */
#define DSI_VPCR_VPSIZE0              DSI_VPCR_VPSIZE0_Msk
#define DSI_VPCR_VPSIZE1_Pos          (1U)
#define DSI_VPCR_VPSIZE1_Msk          (0x1UL << DSI_VPCR_VPSIZE1_Pos)          /*!< 0x00000002 */
#define DSI_VPCR_VPSIZE1              DSI_VPCR_VPSIZE1_Msk
#define DSI_VPCR_VPSIZE2_Pos          (2U)
#define DSI_VPCR_VPSIZE2_Msk          (0x1UL << DSI_VPCR_VPSIZE2_Pos)          /*!< 0x00000004 */
#define DSI_VPCR_VPSIZE2              DSI_VPCR_VPSIZE2_Msk
#define DSI_VPCR_VPSIZE3_Pos          (3U)
#define DSI_VPCR_VPSIZE3_Msk          (0x1UL << DSI_VPCR_VPSIZE3_Pos)          /*!< 0x00000008 */
#define DSI_VPCR_VPSIZE3              DSI_VPCR_VPSIZE3_Msk
#define DSI_VPCR_VPSIZE4_Pos          (4U)
#define DSI_VPCR_VPSIZE4_Msk          (0x1UL << DSI_VPCR_VPSIZE4_Pos)          /*!< 0x00000010 */
#define DSI_VPCR_VPSIZE4              DSI_VPCR_VPSIZE4_Msk
#define DSI_VPCR_VPSIZE5_Pos          (5U)
#define DSI_VPCR_VPSIZE5_Msk          (0x1UL << DSI_VPCR_VPSIZE5_Pos)          /*!< 0x00000020 */
#define DSI_VPCR_VPSIZE5              DSI_VPCR_VPSIZE5_Msk
#define DSI_VPCR_VPSIZE6_Pos          (6U)
#define DSI_VPCR_VPSIZE6_Msk          (0x1UL << DSI_VPCR_VPSIZE6_Pos)          /*!< 0x00000040 */
#define DSI_VPCR_VPSIZE6              DSI_VPCR_VPSIZE6_Msk
#define DSI_VPCR_VPSIZE7_Pos          (7U)
#define DSI_VPCR_VPSIZE7_Msk          (0x1UL << DSI_VPCR_VPSIZE7_Pos)          /*!< 0x00000080 */
#define DSI_VPCR_VPSIZE7              DSI_VPCR_VPSIZE7_Msk
#define DSI_VPCR_VPSIZE8_Pos          (8U)
#define DSI_VPCR_VPSIZE8_Msk          (0x1UL << DSI_VPCR_VPSIZE8_Pos)          /*!< 0x00000100 */
#define DSI_VPCR_VPSIZE8              DSI_VPCR_VPSIZE8_Msk
#define DSI_VPCR_VPSIZE9_Pos          (9U)
#define DSI_VPCR_VPSIZE9_Msk          (0x1UL << DSI_VPCR_VPSIZE9_Pos)          /*!< 0x00000200 */
#define DSI_VPCR_VPSIZE9              DSI_VPCR_VPSIZE9_Msk
#define DSI_VPCR_VPSIZE10_Pos         (10U)
#define DSI_VPCR_VPSIZE10_Msk         (0x1UL << DSI_VPCR_VPSIZE10_Pos)         /*!< 0x00000400 */
#define DSI_VPCR_VPSIZE10             DSI_VPCR_VPSIZE10_Msk
#define DSI_VPCR_VPSIZE11_Pos         (11U)
#define DSI_VPCR_VPSIZE11_Msk         (0x1UL << DSI_VPCR_VPSIZE11_Pos)         /*!< 0x00000800 */
#define DSI_VPCR_VPSIZE11             DSI_VPCR_VPSIZE11_Msk
#define DSI_VPCR_VPSIZE12_Pos         (12U)
#define DSI_VPCR_VPSIZE12_Msk         (0x1UL << DSI_VPCR_VPSIZE12_Pos)         /*!< 0x00001000 */
#define DSI_VPCR_VPSIZE12             DSI_VPCR_VPSIZE12_Msk
#define DSI_VPCR_VPSIZE13_Pos         (13U)
#define DSI_VPCR_VPSIZE13_Msk         (0x1UL << DSI_VPCR_VPSIZE13_Pos)         /*!< 0x00002000 */
#define DSI_VPCR_VPSIZE13             DSI_VPCR_VPSIZE13_Msk

/*******************  Bit definition for DSI_VCCR register  ***************/
#define DSI_VCCR_NUMC_Pos             (0U)
#define DSI_VCCR_NUMC_Msk             (0x1FFFUL << DSI_VCCR_NUMC_Pos)          /*!< 0x00001FFF */
#define DSI_VCCR_NUMC                 DSI_VCCR_NUMC_Msk                        /*!< Number of Chunks */
#define DSI_VCCR_NUMC0_Pos            (0U)
#define DSI_VCCR_NUMC0_Msk            (0x1UL << DSI_VCCR_NUMC0_Pos)            /*!< 0x00000001 */
#define DSI_VCCR_NUMC0                DSI_VCCR_NUMC0_Msk
#define DSI_VCCR_NUMC1_Pos            (1U)
#define DSI_VCCR_NUMC1_Msk            (0x1UL << DSI_VCCR_NUMC1_Pos)            /*!< 0x00000002 */
#define DSI_VCCR_NUMC1                DSI_VCCR_NUMC1_Msk
#define DSI_VCCR_NUMC2_Pos            (2U)
#define DSI_VCCR_NUMC2_Msk            (0x1UL << DSI_VCCR_NUMC2_Pos)            /*!< 0x00000004 */
#define DSI_VCCR_NUMC2                DSI_VCCR_NUMC2_Msk
#define DSI_VCCR_NUMC3_Pos            (3U)
#define DSI_VCCR_NUMC3_Msk            (0x1UL << DSI_VCCR_NUMC3_Pos)            /*!< 0x00000008 */
#define DSI_VCCR_NUMC3                DSI_VCCR_NUMC3_Msk
#define DSI_VCCR_NUMC4_Pos            (4U)
#define DSI_VCCR_NUMC4_Msk            (0x1UL << DSI_VCCR_NUMC4_Pos)            /*!< 0x00000010 */
#define DSI_VCCR_NUMC4                DSI_VCCR_NUMC4_Msk
#define DSI_VCCR_NUMC5_Pos            (5U)
#define DSI_VCCR_NUMC5_Msk            (0x1UL << DSI_VCCR_NUMC5_Pos)            /*!< 0x00000020 */
#define DSI_VCCR_NUMC5                DSI_VCCR_NUMC5_Msk
#define DSI_VCCR_NUMC6_Pos            (6U)
#define DSI_VCCR_NUMC6_Msk            (0x1UL << DSI_VCCR_NUMC6_Pos)            /*!< 0x00000040 */
#define DSI_VCCR_NUMC6                DSI_VCCR_NUMC6_Msk
#define DSI_VCCR_NUMC7_Pos            (7U)
#define DSI_VCCR_NUMC7_Msk            (0x1UL << DSI_VCCR_NUMC7_Pos)            /*!< 0x00000080 */
#define DSI_VCCR_NUMC7                DSI_VCCR_NUMC7_Msk
#define DSI_VCCR_NUMC8_Pos            (8U)
#define DSI_VCCR_NUMC8_Msk            (0x1UL << DSI_VCCR_NUMC8_Pos)            /*!< 0x00000100 */
#define DSI_VCCR_NUMC8                DSI_VCCR_NUMC8_Msk
#define DSI_VCCR_NUMC9_Pos            (9U)
#define DSI_VCCR_NUMC9_Msk            (0x1UL << DSI_VCCR_NUMC9_Pos)            /*!< 0x00000200 */
#define DSI_VCCR_NUMC9                DSI_VCCR_NUMC9_Msk
#define DSI_VCCR_NUMC10_Pos           (10U)
#define DSI_VCCR_NUMC10_Msk           (0x1UL << DSI_VCCR_NUMC10_Pos)           /*!< 0x00000400 */
#define DSI_VCCR_NUMC10               DSI_VCCR_NUMC10_Msk
#define DSI_VCCR_NUMC11_Pos           (11U)
#define DSI_VCCR_NUMC11_Msk           (0x1UL << DSI_VCCR_NUMC11_Pos)           /*!< 0x00000800 */
#define DSI_VCCR_NUMC11               DSI_VCCR_NUMC11_Msk
#define DSI_VCCR_NUMC12_Pos           (12U)
#define DSI_VCCR_NUMC12_Msk           (0x1UL << DSI_VCCR_NUMC12_Pos)           /*!< 0x00001000 */
#define DSI_VCCR_NUMC12               DSI_VCCR_NUMC12_Msk

/*******************  Bit definition for DSI_VNPCR register  **************/
#define DSI_VNPCR_NPSIZE_Pos          (0U)
#define DSI_VNPCR_NPSIZE_Msk          (0x1FFFUL << DSI_VNPCR_NPSIZE_Pos)       /*!< 0x00001FFF */
#define DSI_VNPCR_NPSIZE              DSI_VNPCR_NPSIZE_Msk                     /*!< Null Packet Size */
#define DSI_VNPCR_NPSIZE0_Pos         (0U)
#define DSI_VNPCR_NPSIZE0_Msk         (0x1UL << DSI_VNPCR_NPSIZE0_Pos)         /*!< 0x00000001 */
#define DSI_VNPCR_NPSIZE0             DSI_VNPCR_NPSIZE0_Msk
#define DSI_VNPCR_NPSIZE1_Pos         (1U)
#define DSI_VNPCR_NPSIZE1_Msk         (0x1UL << DSI_VNPCR_NPSIZE1_Pos)         /*!< 0x00000002 */
#define DSI_VNPCR_NPSIZE1             DSI_VNPCR_NPSIZE1_Msk
#define DSI_VNPCR_NPSIZE2_Pos         (2U)
#define DSI_VNPCR_NPSIZE2_Msk         (0x1UL << DSI_VNPCR_NPSIZE2_Pos)         /*!< 0x00000004 */
#define DSI_VNPCR_NPSIZE2             DSI_VNPCR_NPSIZE2_Msk
#define DSI_VNPCR_NPSIZE3_Pos         (3U)
#define DSI_VNPCR_NPSIZE3_Msk         (0x1UL << DSI_VNPCR_NPSIZE3_Pos)         /*!< 0x00000008 */
#define DSI_VNPCR_NPSIZE3             DSI_VNPCR_NPSIZE3_Msk
#define DSI_VNPCR_NPSIZE4_Pos         (4U)
#define DSI_VNPCR_NPSIZE4_Msk         (0x1UL << DSI_VNPCR_NPSIZE4_Pos)         /*!< 0x00000010 */
#define DSI_VNPCR_NPSIZE4             DSI_VNPCR_NPSIZE4_Msk
#define DSI_VNPCR_NPSIZE5_Pos         (5U)
#define DSI_VNPCR_NPSIZE5_Msk         (0x1UL << DSI_VNPCR_NPSIZE5_Pos)         /*!< 0x00000020 */
#define DSI_VNPCR_NPSIZE5             DSI_VNPCR_NPSIZE5_Msk
#define DSI_VNPCR_NPSIZE6_Pos         (6U)
#define DSI_VNPCR_NPSIZE6_Msk         (0x1UL << DSI_VNPCR_NPSIZE6_Pos)         /*!< 0x00000040 */
#define DSI_VNPCR_NPSIZE6             DSI_VNPCR_NPSIZE6_Msk
#define DSI_VNPCR_NPSIZE7_Pos         (7U)
#define DSI_VNPCR_NPSIZE7_Msk         (0x1UL << DSI_VNPCR_NPSIZE7_Pos)         /*!< 0x00000080 */
#define DSI_VNPCR_NPSIZE7             DSI_VNPCR_NPSIZE7_Msk
#define DSI_VNPCR_NPSIZE8_Pos         (8U)
#define DSI_VNPCR_NPSIZE8_Msk         (0x1UL << DSI_VNPCR_NPSIZE8_Pos)         /*!< 0x00000100 */
#define DSI_VNPCR_NPSIZE8             DSI_VNPCR_NPSIZE8_Msk
#define DSI_VNPCR_NPSIZE9_Pos         (9U)
#define DSI_VNPCR_NPSIZE9_Msk         (0x1UL << DSI_VNPCR_NPSIZE9_Pos)         /*!< 0x00000200 */
#define DSI_VNPCR_NPSIZE9             DSI_VNPCR_NPSIZE9_Msk
#define DSI_VNPCR_NPSIZE10_Pos        (10U)
#define DSI_VNPCR_NPSIZE10_Msk        (0x1UL << DSI_VNPCR_NPSIZE10_Pos)        /*!< 0x00000400 */
#define DSI_VNPCR_NPSIZE10            DSI_VNPCR_NPSIZE10_Msk
#define DSI_VNPCR_NPSIZE11_Pos        (11U)
#define DSI_VNPCR_NPSIZE11_Msk        (0x1UL << DSI_VNPCR_NPSIZE11_Pos)        /*!< 0x00000800 */
#define DSI_VNPCR_NPSIZE11            DSI_VNPCR_NPSIZE11_Msk
#define DSI_VNPCR_NPSIZE12_Pos        (12U)
#define DSI_VNPCR_NPSIZE12_Msk        (0x1UL << DSI_VNPCR_NPSIZE12_Pos)        /*!< 0x00001000 */
#define DSI_VNPCR_NPSIZE12            DSI_VNPCR_NPSIZE12_Msk

/*******************  Bit definition for DSI_VHSACR register  *************/
#define DSI_VHSACR_HSA_Pos            (0U)
#define DSI_VHSACR_HSA_Msk            (0xFFFUL << DSI_VHSACR_HSA_Pos)          /*!< 0x00000FFF */
#define DSI_VHSACR_HSA                DSI_VHSACR_HSA_Msk                       /*!< Horizontal Synchronism Active duration */
#define DSI_VHSACR_HSA0_Pos           (0U)
#define DSI_VHSACR_HSA0_Msk           (0x1UL << DSI_VHSACR_HSA0_Pos)           /*!< 0x00000001 */
#define DSI_VHSACR_HSA0               DSI_VHSACR_HSA0_Msk
#define DSI_VHSACR_HSA1_Pos           (1U)
#define DSI_VHSACR_HSA1_Msk           (0x1UL << DSI_VHSACR_HSA1_Pos)           /*!< 0x00000002 */
#define DSI_VHSACR_HSA1               DSI_VHSACR_HSA1_Msk
#define DSI_VHSACR_HSA2_Pos           (2U)
#define DSI_VHSACR_HSA2_Msk           (0x1UL << DSI_VHSACR_HSA2_Pos)           /*!< 0x00000004 */
#define DSI_VHSACR_HSA2               DSI_VHSACR_HSA2_Msk
#define DSI_VHSACR_HSA3_Pos           (3U)
#define DSI_VHSACR_HSA3_Msk           (0x1UL << DSI_VHSACR_HSA3_Pos)           /*!< 0x00000008 */
#define DSI_VHSACR_HSA3               DSI_VHSACR_HSA3_Msk
#define DSI_VHSACR_HSA4_Pos           (4U)
#define DSI_VHSACR_HSA4_Msk           (0x1UL << DSI_VHSACR_HSA4_Pos)           /*!< 0x00000010 */
#define DSI_VHSACR_HSA4               DSI_VHSACR_HSA4_Msk
#define DSI_VHSACR_HSA5_Pos           (5U)
#define DSI_VHSACR_HSA5_Msk           (0x1UL << DSI_VHSACR_HSA5_Pos)           /*!< 0x00000020 */
#define DSI_VHSACR_HSA5               DSI_VHSACR_HSA5_Msk
#define DSI_VHSACR_HSA6_Pos           (6U)
#define DSI_VHSACR_HSA6_Msk           (0x1UL << DSI_VHSACR_HSA6_Pos)           /*!< 0x00000040 */
#define DSI_VHSACR_HSA6               DSI_VHSACR_HSA6_Msk
#define DSI_VHSACR_HSA7_Pos           (7U)
#define DSI_VHSACR_HSA7_Msk           (0x1UL << DSI_VHSACR_HSA7_Pos)           /*!< 0x00000080 */
#define DSI_VHSACR_HSA7               DSI_VHSACR_HSA7_Msk
#define DSI_VHSACR_HSA8_Pos           (8U)
#define DSI_VHSACR_HSA8_Msk           (0x1UL << DSI_VHSACR_HSA8_Pos)           /*!< 0x00000100 */
#define DSI_VHSACR_HSA8               DSI_VHSACR_HSA8_Msk
#define DSI_VHSACR_HSA9_Pos           (9U)
#define DSI_VHSACR_HSA9_Msk           (0x1UL << DSI_VHSACR_HSA9_Pos)           /*!< 0x00000200 */
#define DSI_VHSACR_HSA9               DSI_VHSACR_HSA9_Msk
#define DSI_VHSACR_HSA10_Pos          (10U)
#define DSI_VHSACR_HSA10_Msk          (0x1UL << DSI_VHSACR_HSA10_Pos)          /*!< 0x00000400 */
#define DSI_VHSACR_HSA10              DSI_VHSACR_HSA10_Msk
#define DSI_VHSACR_HSA11_Pos          (11U)
#define DSI_VHSACR_HSA11_Msk          (0x1UL << DSI_VHSACR_HSA11_Pos)          /*!< 0x00000800 */
#define DSI_VHSACR_HSA11              DSI_VHSACR_HSA11_Msk

/*******************  Bit definition for DSI_VHBPCR register  *************/
#define DSI_VHBPCR_HBP_Pos            (0U)
#define DSI_VHBPCR_HBP_Msk            (0xFFFUL << DSI_VHBPCR_HBP_Pos)          /*!< 0x00000FFF */
#define DSI_VHBPCR_HBP                DSI_VHBPCR_HBP_Msk                       /*!< Horizontal Back-Porch duration */
#define DSI_VHBPCR_HBP0_Pos           (0U)
#define DSI_VHBPCR_HBP0_Msk           (0x1UL << DSI_VHBPCR_HBP0_Pos)           /*!< 0x00000001 */
#define DSI_VHBPCR_HBP0               DSI_VHBPCR_HBP0_Msk
#define DSI_VHBPCR_HBP1_Pos           (1U)
#define DSI_VHBPCR_HBP1_Msk           (0x1UL << DSI_VHBPCR_HBP1_Pos)           /*!< 0x00000002 */
#define DSI_VHBPCR_HBP1               DSI_VHBPCR_HBP1_Msk
#define DSI_VHBPCR_HBP2_Pos           (2U)
#define DSI_VHBPCR_HBP2_Msk           (0x1UL << DSI_VHBPCR_HBP2_Pos)           /*!< 0x00000004 */
#define DSI_VHBPCR_HBP2               DSI_VHBPCR_HBP2_Msk
#define DSI_VHBPCR_HBP3_Pos           (3U)
#define DSI_VHBPCR_HBP3_Msk           (0x1UL << DSI_VHBPCR_HBP3_Pos)           /*!< 0x00000008 */
#define DSI_VHBPCR_HBP3               DSI_VHBPCR_HBP3_Msk
#define DSI_VHBPCR_HBP4_Pos           (4U)
#define DSI_VHBPCR_HBP4_Msk           (0x1UL << DSI_VHBPCR_HBP4_Pos)           /*!< 0x00000010 */
#define DSI_VHBPCR_HBP4               DSI_VHBPCR_HBP4_Msk
#define DSI_VHBPCR_HBP5_Pos           (5U)
#define DSI_VHBPCR_HBP5_Msk           (0x1UL << DSI_VHBPCR_HBP5_Pos)           /*!< 0x00000020 */
#define DSI_VHBPCR_HBP5               DSI_VHBPCR_HBP5_Msk
#define DSI_VHBPCR_HBP6_Pos           (6U)
#define DSI_VHBPCR_HBP6_Msk           (0x1UL << DSI_VHBPCR_HBP6_Pos)           /*!< 0x00000040 */
#define DSI_VHBPCR_HBP6               DSI_VHBPCR_HBP6_Msk
#define DSI_VHBPCR_HBP7_Pos           (7U)
#define DSI_VHBPCR_HBP7_Msk           (0x1UL << DSI_VHBPCR_HBP7_Pos)           /*!< 0x00000080 */
#define DSI_VHBPCR_HBP7               DSI_VHBPCR_HBP7_Msk
#define DSI_VHBPCR_HBP8_Pos           (8U)
#define DSI_VHBPCR_HBP8_Msk           (0x1UL << DSI_VHBPCR_HBP8_Pos)           /*!< 0x00000100 */
#define DSI_VHBPCR_HBP8               DSI_VHBPCR_HBP8_Msk
#define DSI_VHBPCR_HBP9_Pos           (9U)
#define DSI_VHBPCR_HBP9_Msk           (0x1UL << DSI_VHBPCR_HBP9_Pos)           /*!< 0x00000200 */
#define DSI_VHBPCR_HBP9               DSI_VHBPCR_HBP9_Msk
#define DSI_VHBPCR_HBP10_Pos          (10U)
#define DSI_VHBPCR_HBP10_Msk          (0x1UL << DSI_VHBPCR_HBP10_Pos)          /*!< 0x00000400 */
#define DSI_VHBPCR_HBP10              DSI_VHBPCR_HBP10_Msk
#define DSI_VHBPCR_HBP11_Pos          (11U)
#define DSI_VHBPCR_HBP11_Msk          (0x1UL << DSI_VHBPCR_HBP11_Pos)          /*!< 0x00000800 */
#define DSI_VHBPCR_HBP11              DSI_VHBPCR_HBP11_Msk

/*******************  Bit definition for DSI_VLCR register  ***************/
#define DSI_VLCR_HLINE_Pos            (0U)
#define DSI_VLCR_HLINE_Msk            (0x7FFFUL << DSI_VLCR_HLINE_Pos)         /*!< 0x00007FFF */
#define DSI_VLCR_HLINE                DSI_VLCR_HLINE_Msk                       /*!< Horizontal Line duration */
#define DSI_VLCR_HLINE0_Pos           (0U)
#define DSI_VLCR_HLINE0_Msk           (0x1UL << DSI_VLCR_HLINE0_Pos)           /*!< 0x00000001 */
#define DSI_VLCR_HLINE0               DSI_VLCR_HLINE0_Msk
#define DSI_VLCR_HLINE1_Pos           (1U)
#define DSI_VLCR_HLINE1_Msk           (0x1UL << DSI_VLCR_HLINE1_Pos)           /*!< 0x00000002 */
#define DSI_VLCR_HLINE1               DSI_VLCR_HLINE1_Msk
#define DSI_VLCR_HLINE2_Pos           (2U)
#define DSI_VLCR_HLINE2_Msk           (0x1UL << DSI_VLCR_HLINE2_Pos)           /*!< 0x00000004 */
#define DSI_VLCR_HLINE2               DSI_VLCR_HLINE2_Msk
#define DSI_VLCR_HLINE3_Pos           (3U)
#define DSI_VLCR_HLINE3_Msk           (0x1UL << DSI_VLCR_HLINE3_Pos)           /*!< 0x00000008 */
#define DSI_VLCR_HLINE3               DSI_VLCR_HLINE3_Msk
#define DSI_VLCR_HLINE4_Pos           (4U)
#define DSI_VLCR_HLINE4_Msk           (0x1UL << DSI_VLCR_HLINE4_Pos)           /*!< 0x00000010 */
#define DSI_VLCR_HLINE4               DSI_VLCR_HLINE4_Msk
#define DSI_VLCR_HLINE5_Pos           (5U)
#define DSI_VLCR_HLINE5_Msk           (0x1UL << DSI_VLCR_HLINE5_Pos)           /*!< 0x00000020 */
#define DSI_VLCR_HLINE5               DSI_VLCR_HLINE5_Msk
#define DSI_VLCR_HLINE6_Pos           (6U)
#define DSI_VLCR_HLINE6_Msk           (0x1UL << DSI_VLCR_HLINE6_Pos)           /*!< 0x00000040 */
#define DSI_VLCR_HLINE6               DSI_VLCR_HLINE6_Msk
#define DSI_VLCR_HLINE7_Pos           (7U)
#define DSI_VLCR_HLINE7_Msk           (0x1UL << DSI_VLCR_HLINE7_Pos)           /*!< 0x00000080 */
#define DSI_VLCR_HLINE7               DSI_VLCR_HLINE7_Msk
#define DSI_VLCR_HLINE8_Pos           (8U)
#define DSI_VLCR_HLINE8_Msk           (0x1UL << DSI_VLCR_HLINE8_Pos)           /*!< 0x00000100 */
#define DSI_VLCR_HLINE8               DSI_VLCR_HLINE8_Msk
#define DSI_VLCR_HLINE9_Pos           (9U)
#define DSI_VLCR_HLINE9_Msk           (0x1UL << DSI_VLCR_HLINE9_Pos)           /*!< 0x00000200 */
#define DSI_VLCR_HLINE9               DSI_VLCR_HLINE9_Msk
#define DSI_VLCR_HLINE10_Pos          (10U)
#define DSI_VLCR_HLINE10_Msk          (0x1UL << DSI_VLCR_HLINE10_Pos)          /*!< 0x00000400 */
#define DSI_VLCR_HLINE10              DSI_VLCR_HLINE10_Msk
#define DSI_VLCR_HLINE11_Pos          (11U)
#define DSI_VLCR_HLINE11_Msk          (0x1UL << DSI_VLCR_HLINE11_Pos)          /*!< 0x00000800 */
#define DSI_VLCR_HLINE11              DSI_VLCR_HLINE11_Msk
#define DSI_VLCR_HLINE12_Pos          (12U)
#define DSI_VLCR_HLINE12_Msk          (0x1UL << DSI_VLCR_HLINE12_Pos)          /*!< 0x00001000 */
#define DSI_VLCR_HLINE12              DSI_VLCR_HLINE12_Msk
#define DSI_VLCR_HLINE13_Pos          (13U)
#define DSI_VLCR_HLINE13_Msk          (0x1UL << DSI_VLCR_HLINE13_Pos)          /*!< 0x00002000 */
#define DSI_VLCR_HLINE13              DSI_VLCR_HLINE13_Msk
#define DSI_VLCR_HLINE14_Pos          (14U)
#define DSI_VLCR_HLINE14_Msk          (0x1UL << DSI_VLCR_HLINE14_Pos)          /*!< 0x00004000 */
#define DSI_VLCR_HLINE14              DSI_VLCR_HLINE14_Msk

/*******************  Bit definition for DSI_VVSACR register  *************/
#define DSI_VVSACR_VSA_Pos            (0U)
#define DSI_VVSACR_VSA_Msk            (0x3FFUL << DSI_VVSACR_VSA_Pos)          /*!< 0x000003FF */
#define DSI_VVSACR_VSA                DSI_VVSACR_VSA_Msk                       /*!< Vertical Synchronism Active duration */
#define DSI_VVSACR_VSA0_Pos           (0U)
#define DSI_VVSACR_VSA0_Msk           (0x1UL << DSI_VVSACR_VSA0_Pos)           /*!< 0x00000001 */
#define DSI_VVSACR_VSA0               DSI_VVSACR_VSA0_Msk
#define DSI_VVSACR_VSA1_Pos           (1U)
#define DSI_VVSACR_VSA1_Msk           (0x1UL << DSI_VVSACR_VSA1_Pos)           /*!< 0x00000002 */
#define DSI_VVSACR_VSA1               DSI_VVSACR_VSA1_Msk
#define DSI_VVSACR_VSA2_Pos           (2U)
#define DSI_VVSACR_VSA2_Msk           (0x1UL << DSI_VVSACR_VSA2_Pos)           /*!< 0x00000004 */
#define DSI_VVSACR_VSA2               DSI_VVSACR_VSA2_Msk
#define DSI_VVSACR_VSA3_Pos           (3U)
#define DSI_VVSACR_VSA3_Msk           (0x1UL << DSI_VVSACR_VSA3_Pos)           /*!< 0x00000008 */
#define DSI_VVSACR_VSA3               DSI_VVSACR_VSA3_Msk
#define DSI_VVSACR_VSA4_Pos           (4U)
#define DSI_VVSACR_VSA4_Msk           (0x1UL << DSI_VVSACR_VSA4_Pos)           /*!< 0x00000010 */
#define DSI_VVSACR_VSA4               DSI_VVSACR_VSA4_Msk
#define DSI_VVSACR_VSA5_Pos           (5U)
#define DSI_VVSACR_VSA5_Msk           (0x1UL << DSI_VVSACR_VSA5_Pos)           /*!< 0x00000020 */
#define DSI_VVSACR_VSA5               DSI_VVSACR_VSA5_Msk
#define DSI_VVSACR_VSA6_Pos           (6U)
#define DSI_VVSACR_VSA6_Msk           (0x1UL << DSI_VVSACR_VSA6_Pos)           /*!< 0x00000040 */
#define DSI_VVSACR_VSA6               DSI_VVSACR_VSA6_Msk
#define DSI_VVSACR_VSA7_Pos           (7U)
#define DSI_VVSACR_VSA7_Msk           (0x1UL << DSI_VVSACR_VSA7_Pos)           /*!< 0x00000080 */
#define DSI_VVSACR_VSA7               DSI_VVSACR_VSA7_Msk
#define DSI_VVSACR_VSA8_Pos           (8U)
#define DSI_VVSACR_VSA8_Msk           (0x1UL << DSI_VVSACR_VSA8_Pos)           /*!< 0x00000100 */
#define DSI_VVSACR_VSA8               DSI_VVSACR_VSA8_Msk
#define DSI_VVSACR_VSA9_Pos           (9U)
#define DSI_VVSACR_VSA9_Msk           (0x1UL << DSI_VVSACR_VSA9_Pos)           /*!< 0x00000200 */
#define DSI_VVSACR_VSA9               DSI_VVSACR_VSA9_Msk

/*******************  Bit definition for DSI_VVBPCR register  *************/
#define DSI_VVBPCR_VBP_Pos            (0U)
#define DSI_VVBPCR_VBP_Msk            (0x3FFUL << DSI_VVBPCR_VBP_Pos)          /*!< 0x000003FF */
#define DSI_VVBPCR_VBP                DSI_VVBPCR_VBP_Msk                       /*!< Vertical Back-Porch duration */
#define DSI_VVBPCR_VBP0_Pos           (0U)
#define DSI_VVBPCR_VBP0_Msk           (0x1UL << DSI_VVBPCR_VBP0_Pos)           /*!< 0x00000001 */
#define DSI_VVBPCR_VBP0               DSI_VVBPCR_VBP0_Msk
#define DSI_VVBPCR_VBP1_Pos           (1U)
#define DSI_VVBPCR_VBP1_Msk           (0x1UL << DSI_VVBPCR_VBP1_Pos)           /*!< 0x00000002 */
#define DSI_VVBPCR_VBP1               DSI_VVBPCR_VBP1_Msk
#define DSI_VVBPCR_VBP2_Pos           (2U)
#define DSI_VVBPCR_VBP2_Msk           (0x1UL << DSI_VVBPCR_VBP2_Pos)           /*!< 0x00000004 */
#define DSI_VVBPCR_VBP2               DSI_VVBPCR_VBP2_Msk
#define DSI_VVBPCR_VBP3_Pos           (3U)
#define DSI_VVBPCR_VBP3_Msk           (0x1UL << DSI_VVBPCR_VBP3_Pos)           /*!< 0x00000008 */
#define DSI_VVBPCR_VBP3               DSI_VVBPCR_VBP3_Msk
#define DSI_VVBPCR_VBP4_Pos           (4U)
#define DSI_VVBPCR_VBP4_Msk           (0x1UL << DSI_VVBPCR_VBP4_Pos)           /*!< 0x00000010 */
#define DSI_VVBPCR_VBP4               DSI_VVBPCR_VBP4_Msk
#define DSI_VVBPCR_VBP5_Pos           (5U)
#define DSI_VVBPCR_VBP5_Msk           (0x1UL << DSI_VVBPCR_VBP5_Pos)           /*!< 0x00000020 */
#define DSI_VVBPCR_VBP5               DSI_VVBPCR_VBP5_Msk
#define DSI_VVBPCR_VBP6_Pos           (6U)
#define DSI_VVBPCR_VBP6_Msk           (0x1UL << DSI_VVBPCR_VBP6_Pos)           /*!< 0x00000040 */
#define DSI_VVBPCR_VBP6               DSI_VVBPCR_VBP6_Msk
#define DSI_VVBPCR_VBP7_Pos           (7U)
#define DSI_VVBPCR_VBP7_Msk           (0x1UL << DSI_VVBPCR_VBP7_Pos)           /*!< 0x00000080 */
#define DSI_VVBPCR_VBP7               DSI_VVBPCR_VBP7_Msk
#define DSI_VVBPCR_VBP8_Pos           (8U)
#define DSI_VVBPCR_VBP8_Msk           (0x1UL << DSI_VVBPCR_VBP8_Pos)           /*!< 0x00000100 */
#define DSI_VVBPCR_VBP8               DSI_VVBPCR_VBP8_Msk
#define DSI_VVBPCR_VBP9_Pos           (9U)
#define DSI_VVBPCR_VBP9_Msk           (0x1UL << DSI_VVBPCR_VBP9_Pos)           /*!< 0x00000200 */
#define DSI_VVBPCR_VBP9               DSI_VVBPCR_VBP9_Msk

/*******************  Bit definition for DSI_VVFPCR register  *************/
#define DSI_VVFPCR_VFP_Pos            (0U)
#define DSI_VVFPCR_VFP_Msk            (0x3FFUL << DSI_VVFPCR_VFP_Pos)          /*!< 0x000003FF */
#define DSI_VVFPCR_VFP                DSI_VVFPCR_VFP_Msk                       /*!< Vertical Front-Porch duration */
#define DSI_VVFPCR_VFP0_Pos           (0U)
#define DSI_VVFPCR_VFP0_Msk           (0x1UL << DSI_VVFPCR_VFP0_Pos)           /*!< 0x00000001 */
#define DSI_VVFPCR_VFP0               DSI_VVFPCR_VFP0_Msk
#define DSI_VVFPCR_VFP1_Pos           (1U)
#define DSI_VVFPCR_VFP1_Msk           (0x1UL << DSI_VVFPCR_VFP1_Pos)           /*!< 0x00000002 */
#define DSI_VVFPCR_VFP1               DSI_VVFPCR_VFP1_Msk
#define DSI_VVFPCR_VFP2_Pos           (2U)
#define DSI_VVFPCR_VFP2_Msk           (0x1UL << DSI_VVFPCR_VFP2_Pos)           /*!< 0x00000004 */
#define DSI_VVFPCR_VFP2               DSI_VVFPCR_VFP2_Msk
#define DSI_VVFPCR_VFP3_Pos           (3U)
#define DSI_VVFPCR_VFP3_Msk           (0x1UL << DSI_VVFPCR_VFP3_Pos)           /*!< 0x00000008 */
#define DSI_VVFPCR_VFP3               DSI_VVFPCR_VFP3_Msk
#define DSI_VVFPCR_VFP4_Pos           (4U)
#define DSI_VVFPCR_VFP4_Msk           (0x1UL << DSI_VVFPCR_VFP4_Pos)           /*!< 0x00000010 */
#define DSI_VVFPCR_VFP4               DSI_VVFPCR_VFP4_Msk
#define DSI_VVFPCR_VFP5_Pos           (5U)
#define DSI_VVFPCR_VFP5_Msk           (0x1UL << DSI_VVFPCR_VFP5_Pos)           /*!< 0x00000020 */
#define DSI_VVFPCR_VFP5               DSI_VVFPCR_VFP5_Msk
#define DSI_VVFPCR_VFP6_Pos           (6U)
#define DSI_VVFPCR_VFP6_Msk           (0x1UL << DSI_VVFPCR_VFP6_Pos)           /*!< 0x00000040 */
#define DSI_VVFPCR_VFP6               DSI_VVFPCR_VFP6_Msk
#define DSI_VVFPCR_VFP7_Pos           (7U)
#define DSI_VVFPCR_VFP7_Msk           (0x1UL << DSI_VVFPCR_VFP7_Pos)           /*!< 0x00000080 */
#define DSI_VVFPCR_VFP7               DSI_VVFPCR_VFP7_Msk
#define DSI_VVFPCR_VFP8_Pos           (8U)
#define DSI_VVFPCR_VFP8_Msk           (0x1UL << DSI_VVFPCR_VFP8_Pos)           /*!< 0x00000100 */
#define DSI_VVFPCR_VFP8               DSI_VVFPCR_VFP8_Msk
#define DSI_VVFPCR_VFP9_Pos           (9U)
#define DSI_VVFPCR_VFP9_Msk           (0x1UL << DSI_VVFPCR_VFP9_Pos)           /*!< 0x00000200 */
#define DSI_VVFPCR_VFP9               DSI_VVFPCR_VFP9_Msk

/*******************  Bit definition for DSI_VVACR register  **************/
#define DSI_VVACR_VA_Pos              (0U)
#define DSI_VVACR_VA_Msk              (0x3FFFUL << DSI_VVACR_VA_Pos)           /*!< 0x00003FFF */
#define DSI_VVACR_VA                  DSI_VVACR_VA_Msk                         /*!< Vertical Active duration */
#define DSI_VVACR_VA0_Pos             (0U)
#define DSI_VVACR_VA0_Msk             (0x1UL << DSI_VVACR_VA0_Pos)             /*!< 0x00000001 */
#define DSI_VVACR_VA0                 DSI_VVACR_VA0_Msk
#define DSI_VVACR_VA1_Pos             (1U)
#define DSI_VVACR_VA1_Msk             (0x1UL << DSI_VVACR_VA1_Pos)             /*!< 0x00000002 */
#define DSI_VVACR_VA1                 DSI_VVACR_VA1_Msk
#define DSI_VVACR_VA2_Pos             (2U)
#define DSI_VVACR_VA2_Msk             (0x1UL << DSI_VVACR_VA2_Pos)             /*!< 0x00000004 */
#define DSI_VVACR_VA2                 DSI_VVACR_VA2_Msk
#define DSI_VVACR_VA3_Pos             (3U)
#define DSI_VVACR_VA3_Msk             (0x1UL << DSI_VVACR_VA3_Pos)             /*!< 0x00000008 */
#define DSI_VVACR_VA3                 DSI_VVACR_VA3_Msk
#define DSI_VVACR_VA4_Pos             (4U)
#define DSI_VVACR_VA4_Msk             (0x1UL << DSI_VVACR_VA4_Pos)             /*!< 0x00000010 */
#define DSI_VVACR_VA4                 DSI_VVACR_VA4_Msk
#define DSI_VVACR_VA5_Pos             (5U)
#define DSI_VVACR_VA5_Msk             (0x1UL << DSI_VVACR_VA5_Pos)             /*!< 0x00000020 */
#define DSI_VVACR_VA5                 DSI_VVACR_VA5_Msk
#define DSI_VVACR_VA6_Pos             (6U)
#define DSI_VVACR_VA6_Msk             (0x1UL << DSI_VVACR_VA6_Pos)             /*!< 0x00000040 */
#define DSI_VVACR_VA6                 DSI_VVACR_VA6_Msk
#define DSI_VVACR_VA7_Pos             (7U)
#define DSI_VVACR_VA7_Msk             (0x1UL << DSI_VVACR_VA7_Pos)             /*!< 0x00000080 */
#define DSI_VVACR_VA7                 DSI_VVACR_VA7_Msk
#define DSI_VVACR_VA8_Pos             (8U)
#define DSI_VVACR_VA8_Msk             (0x1UL << DSI_VVACR_VA8_Pos)             /*!< 0x00000100 */
#define DSI_VVACR_VA8                 DSI_VVACR_VA8_Msk
#define DSI_VVACR_VA9_Pos             (9U)
#define DSI_VVACR_VA9_Msk             (0x1UL << DSI_VVACR_VA9_Pos)             /*!< 0x00000200 */
#define DSI_VVACR_VA9                 DSI_VVACR_VA9_Msk
#define DSI_VVACR_VA10_Pos            (10U)
#define DSI_VVACR_VA10_Msk            (0x1UL << DSI_VVACR_VA10_Pos)            /*!< 0x00000400 */
#define DSI_VVACR_VA10                DSI_VVACR_VA10_Msk
#define DSI_VVACR_VA11_Pos            (11U)
#define DSI_VVACR_VA11_Msk            (0x1UL << DSI_VVACR_VA11_Pos)            /*!< 0x00000800 */
#define DSI_VVACR_VA11                DSI_VVACR_VA11_Msk
#define DSI_VVACR_VA12_Pos            (12U)
#define DSI_VVACR_VA12_Msk            (0x1UL << DSI_VVACR_VA12_Pos)            /*!< 0x00001000 */
#define DSI_VVACR_VA12                DSI_VVACR_VA12_Msk
#define DSI_VVACR_VA13_Pos            (13U)
#define DSI_VVACR_VA13_Msk            (0x1UL << DSI_VVACR_VA13_Pos)            /*!< 0x00002000 */
#define DSI_VVACR_VA13                DSI_VVACR_VA13_Msk

/*******************  Bit definition for DSI_LCCR register  ***************/
#define DSI_LCCR_CMDSIZE_Pos          (0U)
#define DSI_LCCR_CMDSIZE_Msk          (0xFFFFUL << DSI_LCCR_CMDSIZE_Pos)       /*!< 0x0000FFFF */
#define DSI_LCCR_CMDSIZE              DSI_LCCR_CMDSIZE_Msk                     /*!< Command Size */
#define DSI_LCCR_CMDSIZE0_Pos         (0U)
#define DSI_LCCR_CMDSIZE0_Msk         (0x1UL << DSI_LCCR_CMDSIZE0_Pos)         /*!< 0x00000001 */
#define DSI_LCCR_CMDSIZE0             DSI_LCCR_CMDSIZE0_Msk
#define DSI_LCCR_CMDSIZE1_Pos         (1U)
#define DSI_LCCR_CMDSIZE1_Msk         (0x1UL << DSI_LCCR_CMDSIZE1_Pos)         /*!< 0x00000002 */
#define DSI_LCCR_CMDSIZE1             DSI_LCCR_CMDSIZE1_Msk
#define DSI_LCCR_CMDSIZE2_Pos         (2U)
#define DSI_LCCR_CMDSIZE2_Msk         (0x1UL << DSI_LCCR_CMDSIZE2_Pos)         /*!< 0x00000004 */
#define DSI_LCCR_CMDSIZE2             DSI_LCCR_CMDSIZE2_Msk
#define DSI_LCCR_CMDSIZE3_Pos         (3U)
#define DSI_LCCR_CMDSIZE3_Msk         (0x1UL << DSI_LCCR_CMDSIZE3_Pos)         /*!< 0x00000008 */
#define DSI_LCCR_CMDSIZE3             DSI_LCCR_CMDSIZE3_Msk
#define DSI_LCCR_CMDSIZE4_Pos         (4U)
#define DSI_LCCR_CMDSIZE4_Msk         (0x1UL << DSI_LCCR_CMDSIZE4_Pos)         /*!< 0x00000010 */
#define DSI_LCCR_CMDSIZE4             DSI_LCCR_CMDSIZE4_Msk
#define DSI_LCCR_CMDSIZE5_Pos         (5U)
#define DSI_LCCR_CMDSIZE5_Msk         (0x1UL << DSI_LCCR_CMDSIZE5_Pos)         /*!< 0x00000020 */
#define DSI_LCCR_CMDSIZE5             DSI_LCCR_CMDSIZE5_Msk
#define DSI_LCCR_CMDSIZE6_Pos         (6U)
#define DSI_LCCR_CMDSIZE6_Msk         (0x1UL << DSI_LCCR_CMDSIZE6_Pos)         /*!< 0x00000040 */
#define DSI_LCCR_CMDSIZE6             DSI_LCCR_CMDSIZE6_Msk
#define DSI_LCCR_CMDSIZE7_Pos         (7U)
#define DSI_LCCR_CMDSIZE7_Msk         (0x1UL << DSI_LCCR_CMDSIZE7_Pos)         /*!< 0x00000080 */
#define DSI_LCCR_CMDSIZE7             DSI_LCCR_CMDSIZE7_Msk
#define DSI_LCCR_CMDSIZE8_Pos         (8U)
#define DSI_LCCR_CMDSIZE8_Msk         (0x1UL << DSI_LCCR_CMDSIZE8_Pos)         /*!< 0x00000100 */
#define DSI_LCCR_CMDSIZE8             DSI_LCCR_CMDSIZE8_Msk
#define DSI_LCCR_CMDSIZE9_Pos         (9U)
#define DSI_LCCR_CMDSIZE9_Msk         (0x1UL << DSI_LCCR_CMDSIZE9_Pos)         /*!< 0x00000200 */
#define DSI_LCCR_CMDSIZE9             DSI_LCCR_CMDSIZE9_Msk
#define DSI_LCCR_CMDSIZE10_Pos        (10U)
#define DSI_LCCR_CMDSIZE10_Msk        (0x1UL << DSI_LCCR_CMDSIZE10_Pos)        /*!< 0x00000400 */
#define DSI_LCCR_CMDSIZE10            DSI_LCCR_CMDSIZE10_Msk
#define DSI_LCCR_CMDSIZE11_Pos        (11U)
#define DSI_LCCR_CMDSIZE11_Msk        (0x1UL << DSI_LCCR_CMDSIZE11_Pos)        /*!< 0x00000800 */
#define DSI_LCCR_CMDSIZE11            DSI_LCCR_CMDSIZE11_Msk
#define DSI_LCCR_CMDSIZE12_Pos        (12U)
#define DSI_LCCR_CMDSIZE12_Msk        (0x1UL << DSI_LCCR_CMDSIZE12_Pos)        /*!< 0x00001000 */
#define DSI_LCCR_CMDSIZE12            DSI_LCCR_CMDSIZE12_Msk
#define DSI_LCCR_CMDSIZE13_Pos        (13U)
#define DSI_LCCR_CMDSIZE13_Msk        (0x1UL << DSI_LCCR_CMDSIZE13_Pos)        /*!< 0x00002000 */
#define DSI_LCCR_CMDSIZE13            DSI_LCCR_CMDSIZE13_Msk
#define DSI_LCCR_CMDSIZE14_Pos        (14U)
#define DSI_LCCR_CMDSIZE14_Msk        (0x1UL << DSI_LCCR_CMDSIZE14_Pos)        /*!< 0x00004000 */
#define DSI_LCCR_CMDSIZE14            DSI_LCCR_CMDSIZE14_Msk
#define DSI_LCCR_CMDSIZE15_Pos        (15U)
#define DSI_LCCR_CMDSIZE15_Msk        (0x1UL << DSI_LCCR_CMDSIZE15_Pos)        /*!< 0x00008000 */
#define DSI_LCCR_CMDSIZE15            DSI_LCCR_CMDSIZE15_Msk

/*******************  Bit definition for DSI_CMCR register  ***************/
#define DSI_CMCR_TEARE_Pos            (0U)
#define DSI_CMCR_TEARE_Msk            (0x1UL << DSI_CMCR_TEARE_Pos)            /*!< 0x00000001 */
#define DSI_CMCR_TEARE                DSI_CMCR_TEARE_Msk                       /*!< Tearing Effect Acknowledge Request Enable */
#define DSI_CMCR_ARE_Pos              (1U)
#define DSI_CMCR_ARE_Msk              (0x1UL << DSI_CMCR_ARE_Pos)              /*!< 0x00000002 */
#define DSI_CMCR_ARE                  DSI_CMCR_ARE_Msk                         /*!< Acknowledge Request Enable */
#define DSI_CMCR_GSW0TX_Pos           (8U)
#define DSI_CMCR_GSW0TX_Msk           (0x1UL << DSI_CMCR_GSW0TX_Pos)           /*!< 0x00000100 */
#define DSI_CMCR_GSW0TX               DSI_CMCR_GSW0TX_Msk                      /*!< Generic Short Write Zero parameters Transmission */
#define DSI_CMCR_GSW1TX_Pos           (9U)
#define DSI_CMCR_GSW1TX_Msk           (0x1UL << DSI_CMCR_GSW1TX_Pos)           /*!< 0x00000200 */
#define DSI_CMCR_GSW1TX               DSI_CMCR_GSW1TX_Msk                      /*!< Generic Short Write One parameters Transmission */
#define DSI_CMCR_GSW2TX_Pos           (10U)
#define DSI_CMCR_GSW2TX_Msk           (0x1UL << DSI_CMCR_GSW2TX_Pos)           /*!< 0x00000400 */
#define DSI_CMCR_GSW2TX               DSI_CMCR_GSW2TX_Msk                      /*!< Generic Short Write Two parameters Transmission */
#define DSI_CMCR_GSR0TX_Pos           (11U)
#define DSI_CMCR_GSR0TX_Msk           (0x1UL << DSI_CMCR_GSR0TX_Pos)           /*!< 0x00000800 */
#define DSI_CMCR_GSR0TX               DSI_CMCR_GSR0TX_Msk                      /*!< Generic Short Read Zero parameters Transmission */
#define DSI_CMCR_GSR1TX_Pos           (12U)
#define DSI_CMCR_GSR1TX_Msk           (0x1UL << DSI_CMCR_GSR1TX_Pos)           /*!< 0x00001000 */
#define DSI_CMCR_GSR1TX               DSI_CMCR_GSR1TX_Msk                      /*!< Generic Short Read One parameters Transmission */
#define DSI_CMCR_GSR2TX_Pos           (13U)
#define DSI_CMCR_GSR2TX_Msk           (0x1UL << DSI_CMCR_GSR2TX_Pos)           /*!< 0x00002000 */
#define DSI_CMCR_GSR2TX               DSI_CMCR_GSR2TX_Msk                      /*!< Generic Short Read Two parameters Transmission */
#define DSI_CMCR_GLWTX_Pos            (14U)
#define DSI_CMCR_GLWTX_Msk            (0x1UL << DSI_CMCR_GLWTX_Pos)            /*!< 0x00004000 */
#define DSI_CMCR_GLWTX                DSI_CMCR_GLWTX_Msk                       /*!< Generic Long Write Transmission */
#define DSI_CMCR_DSW0TX_Pos           (16U)
#define DSI_CMCR_DSW0TX_Msk           (0x1UL << DSI_CMCR_DSW0TX_Pos)           /*!< 0x00010000 */
#define DSI_CMCR_DSW0TX               DSI_CMCR_DSW0TX_Msk                      /*!< DCS Short Write Zero parameter Transmission */
#define DSI_CMCR_DSW1TX_Pos           (17U)
#define DSI_CMCR_DSW1TX_Msk           (0x1UL << DSI_CMCR_DSW1TX_Pos)           /*!< 0x00020000 */
#define DSI_CMCR_DSW1TX               DSI_CMCR_DSW1TX_Msk                      /*!< DCS Short Read One parameter Transmission */
#define DSI_CMCR_DSR0TX_Pos           (18U)
#define DSI_CMCR_DSR0TX_Msk           (0x1UL << DSI_CMCR_DSR0TX_Pos)           /*!< 0x00040000 */
#define DSI_CMCR_DSR0TX               DSI_CMCR_DSR0TX_Msk                      /*!< DCS Short Read Zero parameter Transmission */
#define DSI_CMCR_DLWTX_Pos            (19U)
#define DSI_CMCR_DLWTX_Msk            (0x1UL << DSI_CMCR_DLWTX_Pos)            /*!< 0x00080000 */
#define DSI_CMCR_DLWTX                DSI_CMCR_DLWTX_Msk                       /*!< DCS Long Write Transmission */
#define DSI_CMCR_MRDPS_Pos            (24U)
#define DSI_CMCR_MRDPS_Msk            (0x1UL << DSI_CMCR_MRDPS_Pos)            /*!< 0x01000000 */
#define DSI_CMCR_MRDPS                DSI_CMCR_MRDPS_Msk                       /*!< Maximum Read Packet Size */

/*******************  Bit definition for DSI_GHCR register  ***************/
#define DSI_GHCR_DT_Pos               (0U)
#define DSI_GHCR_DT_Msk               (0x3FUL << DSI_GHCR_DT_Pos)              /*!< 0x0000003F */
#define DSI_GHCR_DT                   DSI_GHCR_DT_Msk                          /*!< Type */
#define DSI_GHCR_DT0_Pos              (0U)
#define DSI_GHCR_DT0_Msk              (0x1UL << DSI_GHCR_DT0_Pos)              /*!< 0x00000001 */
#define DSI_GHCR_DT0                  DSI_GHCR_DT0_Msk
#define DSI_GHCR_DT1_Pos              (1U)
#define DSI_GHCR_DT1_Msk              (0x1UL << DSI_GHCR_DT1_Pos)              /*!< 0x00000002 */
#define DSI_GHCR_DT1                  DSI_GHCR_DT1_Msk
#define DSI_GHCR_DT2_Pos              (2U)
#define DSI_GHCR_DT2_Msk              (0x1UL << DSI_GHCR_DT2_Pos)              /*!< 0x00000004 */
#define DSI_GHCR_DT2                  DSI_GHCR_DT2_Msk
#define DSI_GHCR_DT3_Pos              (3U)
#define DSI_GHCR_DT3_Msk              (0x1UL << DSI_GHCR_DT3_Pos)              /*!< 0x00000008 */
#define DSI_GHCR_DT3                  DSI_GHCR_DT3_Msk
#define DSI_GHCR_DT4_Pos              (4U)
#define DSI_GHCR_DT4_Msk              (0x1UL << DSI_GHCR_DT4_Pos)              /*!< 0x00000010 */
#define DSI_GHCR_DT4                  DSI_GHCR_DT4_Msk
#define DSI_GHCR_DT5_Pos              (5U)
#define DSI_GHCR_DT5_Msk              (0x1UL << DSI_GHCR_DT5_Pos)              /*!< 0x00000020 */
#define DSI_GHCR_DT5                  DSI_GHCR_DT5_Msk

#define DSI_GHCR_VCID_Pos             (6U)
#define DSI_GHCR_VCID_Msk             (0x3UL << DSI_GHCR_VCID_Pos)             /*!< 0x000000C0 */
#define DSI_GHCR_VCID                 DSI_GHCR_VCID_Msk                        /*!< Channel */
#define DSI_GHCR_VCID0_Pos            (6U)
#define DSI_GHCR_VCID0_Msk            (0x1UL << DSI_GHCR_VCID0_Pos)            /*!< 0x00000040 */
#define DSI_GHCR_VCID0                DSI_GHCR_VCID0_Msk
#define DSI_GHCR_VCID1_Pos            (7U)
#define DSI_GHCR_VCID1_Msk            (0x1UL << DSI_GHCR_VCID1_Pos)            /*!< 0x00000080 */
#define DSI_GHCR_VCID1                DSI_GHCR_VCID1_Msk

#define DSI_GHCR_WCLSB_Pos            (8U)
#define DSI_GHCR_WCLSB_Msk            (0xFFUL << DSI_GHCR_WCLSB_Pos)           /*!< 0x0000FF00 */
#define DSI_GHCR_WCLSB                DSI_GHCR_WCLSB_Msk                       /*!< WordCount LSB */
#define DSI_GHCR_WCLSB0_Pos           (8U)
#define DSI_GHCR_WCLSB0_Msk           (0x1UL << DSI_GHCR_WCLSB0_Pos)           /*!< 0x00000100 */
#define DSI_GHCR_WCLSB0               DSI_GHCR_WCLSB0_Msk
#define DSI_GHCR_WCLSB1_Pos           (9U)
#define DSI_GHCR_WCLSB1_Msk           (0x1UL << DSI_GHCR_WCLSB1_Pos)           /*!< 0x00000200 */
#define DSI_GHCR_WCLSB1               DSI_GHCR_WCLSB1_Msk
#define DSI_GHCR_WCLSB2_Pos           (10U)
#define DSI_GHCR_WCLSB2_Msk           (0x1UL << DSI_GHCR_WCLSB2_Pos)           /*!< 0x00000400 */
#define DSI_GHCR_WCLSB2               DSI_GHCR_WCLSB2_Msk
#define DSI_GHCR_WCLSB3_Pos           (11U)
#define DSI_GHCR_WCLSB3_Msk           (0x1UL << DSI_GHCR_WCLSB3_Pos)           /*!< 0x00000800 */
#define DSI_GHCR_WCLSB3               DSI_GHCR_WCLSB3_Msk
#define DSI_GHCR_WCLSB4_Pos           (12U)
#define DSI_GHCR_WCLSB4_Msk           (0x1UL << DSI_GHCR_WCLSB4_Pos)           /*!< 0x00001000 */
#define DSI_GHCR_WCLSB4               DSI_GHCR_WCLSB4_Msk
#define DSI_GHCR_WCLSB5_Pos           (13U)
#define DSI_GHCR_WCLSB5_Msk           (0x1UL << DSI_GHCR_WCLSB5_Pos)           /*!< 0x00002000 */
#define DSI_GHCR_WCLSB5               DSI_GHCR_WCLSB5_Msk
#define DSI_GHCR_WCLSB6_Pos           (14U)
#define DSI_GHCR_WCLSB6_Msk           (0x1UL << DSI_GHCR_WCLSB6_Pos)           /*!< 0x00004000 */
#define DSI_GHCR_WCLSB6               DSI_GHCR_WCLSB6_Msk
#define DSI_GHCR_WCLSB7_Pos           (15U)
#define DSI_GHCR_WCLSB7_Msk           (0x1UL << DSI_GHCR_WCLSB7_Pos)           /*!< 0x00008000 */
#define DSI_GHCR_WCLSB7               DSI_GHCR_WCLSB7_Msk

#define DSI_GHCR_WCMSB_Pos            (16U)
#define DSI_GHCR_WCMSB_Msk            (0xFFUL << DSI_GHCR_WCMSB_Pos)           /*!< 0x00FF0000 */
#define DSI_GHCR_WCMSB                DSI_GHCR_WCMSB_Msk                       /*!< WordCount MSB */
#define DSI_GHCR_WCMSB0_Pos           (16U)
#define DSI_GHCR_WCMSB0_Msk           (0x1UL << DSI_GHCR_WCMSB0_Pos)           /*!< 0x00010000 */
#define DSI_GHCR_WCMSB0               DSI_GHCR_WCMSB0_Msk
#define DSI_GHCR_WCMSB1_Pos           (17U)
#define DSI_GHCR_WCMSB1_Msk           (0x1UL << DSI_GHCR_WCMSB1_Pos)           /*!< 0x00020000 */
#define DSI_GHCR_WCMSB1               DSI_GHCR_WCMSB1_Msk
#define DSI_GHCR_WCMSB2_Pos           (18U)
#define DSI_GHCR_WCMSB2_Msk           (0x1UL << DSI_GHCR_WCMSB2_Pos)           /*!< 0x00040000 */
#define DSI_GHCR_WCMSB2               DSI_GHCR_WCMSB2_Msk
#define DSI_GHCR_WCMSB3_Pos           (19U)
#define DSI_GHCR_WCMSB3_Msk           (0x1UL << DSI_GHCR_WCMSB3_Pos)           /*!< 0x00080000 */
#define DSI_GHCR_WCMSB3               DSI_GHCR_WCMSB3_Msk
#define DSI_GHCR_WCMSB4_Pos           (20U)
#define DSI_GHCR_WCMSB4_Msk           (0x1UL << DSI_GHCR_WCMSB4_Pos)           /*!< 0x00100000 */
#define DSI_GHCR_WCMSB4               DSI_GHCR_WCMSB4_Msk
#define DSI_GHCR_WCMSB5_Pos           (21U)
#define DSI_GHCR_WCMSB5_Msk           (0x1UL << DSI_GHCR_WCMSB5_Pos)           /*!< 0x00200000 */
#define DSI_GHCR_WCMSB5               DSI_GHCR_WCMSB5_Msk
#define DSI_GHCR_WCMSB6_Pos           (22U)
#define DSI_GHCR_WCMSB6_Msk           (0x1UL << DSI_GHCR_WCMSB6_Pos)           /*!< 0x00400000 */
#define DSI_GHCR_WCMSB6               DSI_GHCR_WCMSB6_Msk
#define DSI_GHCR_WCMSB7_Pos           (23U)
#define DSI_GHCR_WCMSB7_Msk           (0x1UL << DSI_GHCR_WCMSB7_Pos)           /*!< 0x00800000 */
#define DSI_GHCR_WCMSB7               DSI_GHCR_WCMSB7_Msk

/*******************  Bit definition for DSI_GPDR register  ***************/
#define DSI_GPDR_DATA1_Pos            (0U)
#define DSI_GPDR_DATA1_Msk            (0xFFUL << DSI_GPDR_DATA1_Pos)           /*!< 0x000000FF */
#define DSI_GPDR_DATA1                DSI_GPDR_DATA1_Msk                       /*!< Payload Byte 1 */
#define DSI_GPDR_DATA1_0              (0x01UL << DSI_GPDR_DATA1_Pos)            /*!< 0x00000001 */
#define DSI_GPDR_DATA1_1              (0x02UL << DSI_GPDR_DATA1_Pos)            /*!< 0x00000002 */
#define DSI_GPDR_DATA1_2              (0x04UL << DSI_GPDR_DATA1_Pos)            /*!< 0x00000004 */
#define DSI_GPDR_DATA1_3              (0x08UL << DSI_GPDR_DATA1_Pos)            /*!< 0x00000008 */
#define DSI_GPDR_DATA1_4              (0x10UL << DSI_GPDR_DATA1_Pos)            /*!< 0x00000010 */
#define DSI_GPDR_DATA1_5              (0x20UL << DSI_GPDR_DATA1_Pos)            /*!< 0x00000020 */
#define DSI_GPDR_DATA1_6              (0x40UL << DSI_GPDR_DATA1_Pos)            /*!< 0x00000040 */
#define DSI_GPDR_DATA1_7              (0x80UL << DSI_GPDR_DATA1_Pos)            /*!< 0x00000080 */

#define DSI_GPDR_DATA2_Pos            (8U)
#define DSI_GPDR_DATA2_Msk            (0xFFUL << DSI_GPDR_DATA2_Pos)           /*!< 0x0000FF00 */
#define DSI_GPDR_DATA2                DSI_GPDR_DATA2_Msk                       /*!< Payload Byte 2 */
#define DSI_GPDR_DATA2_0              (0x01UL << DSI_GPDR_DATA2_Pos)            /*!< 0x00000100 */
#define DSI_GPDR_DATA2_1              (0x02UL << DSI_GPDR_DATA2_Pos)            /*!< 0x00000200 */
#define DSI_GPDR_DATA2_2              (0x04UL << DSI_GPDR_DATA2_Pos)            /*!< 0x00000400 */
#define DSI_GPDR_DATA2_3              (0x08UL << DSI_GPDR_DATA2_Pos)            /*!< 0x00000800 */
#define DSI_GPDR_DATA2_4              (0x10UL << DSI_GPDR_DATA2_Pos)            /*!< 0x00001000 */
#define DSI_GPDR_DATA2_5              (0x20UL << DSI_GPDR_DATA2_Pos)            /*!< 0x00002000 */
#define DSI_GPDR_DATA2_6              (0x40UL << DSI_GPDR_DATA2_Pos)            /*!< 0x00004000 */
#define DSI_GPDR_DATA2_7              (0x80UL << DSI_GPDR_DATA2_Pos)            /*!< 0x00008000 */

#define DSI_GPDR_DATA3_Pos            (16U)
#define DSI_GPDR_DATA3_Msk            (0xFFUL << DSI_GPDR_DATA3_Pos)           /*!< 0x00FF0000 */
#define DSI_GPDR_DATA3                DSI_GPDR_DATA3_Msk                       /*!< Payload Byte 3 */
#define DSI_GPDR_DATA3_0              (0x01UL << DSI_GPDR_DATA3_Pos)            /*!< 0x00010000 */
#define DSI_GPDR_DATA3_1              (0x02UL << DSI_GPDR_DATA3_Pos)            /*!< 0x00020000 */
#define DSI_GPDR_DATA3_2              (0x04UL << DSI_GPDR_DATA3_Pos)            /*!< 0x00040000 */
#define DSI_GPDR_DATA3_3              (0x08UL << DSI_GPDR_DATA3_Pos)            /*!< 0x00080000 */
#define DSI_GPDR_DATA3_4              (0x10UL << DSI_GPDR_DATA3_Pos)            /*!< 0x00100000 */
#define DSI_GPDR_DATA3_5              (0x20UL << DSI_GPDR_DATA3_Pos)            /*!< 0x00200000 */
#define DSI_GPDR_DATA3_6              (0x40UL << DSI_GPDR_DATA3_Pos)            /*!< 0x00400000 */
#define DSI_GPDR_DATA3_7              (0x80UL << DSI_GPDR_DATA3_Pos)            /*!< 0x00800000 */

#define DSI_GPDR_DATA4_Pos            (24U)
#define DSI_GPDR_DATA4_Msk            (0xFFUL << DSI_GPDR_DATA4_Pos)           /*!< 0xFF000000 */
#define DSI_GPDR_DATA4                DSI_GPDR_DATA4_Msk                       /*!< Payload Byte 4 */
#define DSI_GPDR_DATA4_0              (0x01UL << DSI_GPDR_DATA4_Pos)            /*!< 0x01000000 */
#define DSI_GPDR_DATA4_1              (0x02UL << DSI_GPDR_DATA4_Pos)            /*!< 0x02000000 */
#define DSI_GPDR_DATA4_2              (0x04UL << DSI_GPDR_DATA4_Pos)            /*!< 0x04000000 */
#define DSI_GPDR_DATA4_3              (0x08UL << DSI_GPDR_DATA4_Pos)            /*!< 0x08000000 */
#define DSI_GPDR_DATA4_4              (0x10UL << DSI_GPDR_DATA4_Pos)            /*!< 0x10000000 */
#define DSI_GPDR_DATA4_5              (0x20UL << DSI_GPDR_DATA4_Pos)            /*!< 0x20000000 */
#define DSI_GPDR_DATA4_6              (0x40UL << DSI_GPDR_DATA4_Pos)            /*!< 0x40000000 */
#define DSI_GPDR_DATA4_7              (0x80UL << DSI_GPDR_DATA4_Pos)            /*!< 0x80000000 */

/*******************  Bit definition for DSI_GPSR register  ***************/
#define DSI_GPSR_CMDFE_Pos            (0U)
#define DSI_GPSR_CMDFE_Msk            (0x1UL << DSI_GPSR_CMDFE_Pos)            /*!< 0x00000001 */
#define DSI_GPSR_CMDFE                DSI_GPSR_CMDFE_Msk                       /*!< Command FIFO Empty */
#define DSI_GPSR_CMDFF_Pos            (1U)
#define DSI_GPSR_CMDFF_Msk            (0x1UL << DSI_GPSR_CMDFF_Pos)            /*!< 0x00000002 */
#define DSI_GPSR_CMDFF                DSI_GPSR_CMDFF_Msk                       /*!< Command FIFO Full */
#define DSI_GPSR_PWRFE_Pos            (2U)
#define DSI_GPSR_PWRFE_Msk            (0x1UL << DSI_GPSR_PWRFE_Pos)            /*!< 0x00000004 */
#define DSI_GPSR_PWRFE                DSI_GPSR_PWRFE_Msk                       /*!< Payload Write FIFO Empty */
#define DSI_GPSR_PWRFF_Pos            (3U)
#define DSI_GPSR_PWRFF_Msk            (0x1UL << DSI_GPSR_PWRFF_Pos)            /*!< 0x00000008 */
#define DSI_GPSR_PWRFF                DSI_GPSR_PWRFF_Msk                       /*!< Payload Write FIFO Full */
#define DSI_GPSR_PRDFE_Pos            (4U)
#define DSI_GPSR_PRDFE_Msk            (0x1UL << DSI_GPSR_PRDFE_Pos)            /*!< 0x00000010 */
#define DSI_GPSR_PRDFE                DSI_GPSR_PRDFE_Msk                       /*!< Payload Read FIFO Empty */
#define DSI_GPSR_PRDFF_Pos            (5U)
#define DSI_GPSR_PRDFF_Msk            (0x1UL << DSI_GPSR_PRDFF_Pos)            /*!< 0x00000020 */
#define DSI_GPSR_PRDFF                DSI_GPSR_PRDFF_Msk                       /*!< Payload Read FIFO Full */
#define DSI_GPSR_RCB_Pos              (6U)
#define DSI_GPSR_RCB_Msk              (0x1UL << DSI_GPSR_RCB_Pos)              /*!< 0x00000040 */
#define DSI_GPSR_RCB                  DSI_GPSR_RCB_Msk                         /*!< Read Command Busy */

/*******************  Bit definition for DSI_TCCR0 register  **************/
#define DSI_TCCR0_LPRX_TOCNT_Pos      (0U)
#define DSI_TCCR0_LPRX_TOCNT_Msk      (0xFFFFUL << DSI_TCCR0_LPRX_TOCNT_Pos)   /*!< 0x0000FFFF */
#define DSI_TCCR0_LPRX_TOCNT          DSI_TCCR0_LPRX_TOCNT_Msk                 /*!< Low-power Reception Timeout Counter */
#define DSI_TCCR0_LPRX_TOCNT0_Pos     (0U)
#define DSI_TCCR0_LPRX_TOCNT0_Msk     (0x1UL << DSI_TCCR0_LPRX_TOCNT0_Pos)     /*!< 0x00000001 */
#define DSI_TCCR0_LPRX_TOCNT0         DSI_TCCR0_LPRX_TOCNT0_Msk
#define DSI_TCCR0_LPRX_TOCNT1_Pos     (1U)
#define DSI_TCCR0_LPRX_TOCNT1_Msk     (0x1UL << DSI_TCCR0_LPRX_TOCNT1_Pos)     /*!< 0x00000002 */
#define DSI_TCCR0_LPRX_TOCNT1         DSI_TCCR0_LPRX_TOCNT1_Msk
#define DSI_TCCR0_LPRX_TOCNT2_Pos     (2U)
#define DSI_TCCR0_LPRX_TOCNT2_Msk     (0x1UL << DSI_TCCR0_LPRX_TOCNT2_Pos)     /*!< 0x00000004 */
#define DSI_TCCR0_LPRX_TOCNT2         DSI_TCCR0_LPRX_TOCNT2_Msk
#define DSI_TCCR0_LPRX_TOCNT3_Pos     (3U)
#define DSI_TCCR0_LPRX_TOCNT3_Msk     (0x1UL << DSI_TCCR0_LPRX_TOCNT3_Pos)     /*!< 0x00000008 */
#define DSI_TCCR0_LPRX_TOCNT3         DSI_TCCR0_LPRX_TOCNT3_Msk
#define DSI_TCCR0_LPRX_TOCNT4_Pos     (4U)
#define DSI_TCCR0_LPRX_TOCNT4_Msk     (0x1UL << DSI_TCCR0_LPRX_TOCNT4_Pos)     /*!< 0x00000010 */
#define DSI_TCCR0_LPRX_TOCNT4         DSI_TCCR0_LPRX_TOCNT4_Msk
#define DSI_TCCR0_LPRX_TOCNT5_Pos     (5U)
#define DSI_TCCR0_LPRX_TOCNT5_Msk     (0x1UL << DSI_TCCR0_LPRX_TOCNT5_Pos)     /*!< 0x00000020 */
#define DSI_TCCR0_LPRX_TOCNT5         DSI_TCCR0_LPRX_TOCNT5_Msk
#define DSI_TCCR0_LPRX_TOCNT6_Pos     (6U)
#define DSI_TCCR0_LPRX_TOCNT6_Msk     (0x1UL << DSI_TCCR0_LPRX_TOCNT6_Pos)     /*!< 0x00000040 */
#define DSI_TCCR0_LPRX_TOCNT6         DSI_TCCR0_LPRX_TOCNT6_Msk
#define DSI_TCCR0_LPRX_TOCNT7_Pos     (7U)
#define DSI_TCCR0_LPRX_TOCNT7_Msk     (0x1UL << DSI_TCCR0_LPRX_TOCNT7_Pos)     /*!< 0x00000080 */
#define DSI_TCCR0_LPRX_TOCNT7         DSI_TCCR0_LPRX_TOCNT7_Msk
#define DSI_TCCR0_LPRX_TOCNT8_Pos     (8U)
#define DSI_TCCR0_LPRX_TOCNT8_Msk     (0x1UL << DSI_TCCR0_LPRX_TOCNT8_Pos)     /*!< 0x00000100 */
#define DSI_TCCR0_LPRX_TOCNT8         DSI_TCCR0_LPRX_TOCNT8_Msk
#define DSI_TCCR0_LPRX_TOCNT9_Pos     (9U)
#define DSI_TCCR0_LPRX_TOCNT9_Msk     (0x1UL << DSI_TCCR0_LPRX_TOCNT9_Pos)     /*!< 0x00000200 */
#define DSI_TCCR0_LPRX_TOCNT9         DSI_TCCR0_LPRX_TOCNT9_Msk
#define DSI_TCCR0_LPRX_TOCNT10_Pos    (10U)
#define DSI_TCCR0_LPRX_TOCNT10_Msk    (0x1UL << DSI_TCCR0_LPRX_TOCNT10_Pos)    /*!< 0x00000400 */
#define DSI_TCCR0_LPRX_TOCNT10        DSI_TCCR0_LPRX_TOCNT10_Msk
#define DSI_TCCR0_LPRX_TOCNT11_Pos    (11U)
#define DSI_TCCR0_LPRX_TOCNT11_Msk    (0x1UL << DSI_TCCR0_LPRX_TOCNT11_Pos)    /*!< 0x00000800 */
#define DSI_TCCR0_LPRX_TOCNT11        DSI_TCCR0_LPRX_TOCNT11_Msk
#define DSI_TCCR0_LPRX_TOCNT12_Pos    (12U)
#define DSI_TCCR0_LPRX_TOCNT12_Msk    (0x1UL << DSI_TCCR0_LPRX_TOCNT12_Pos)    /*!< 0x00001000 */
#define DSI_TCCR0_LPRX_TOCNT12        DSI_TCCR0_LPRX_TOCNT12_Msk
#define DSI_TCCR0_LPRX_TOCNT13_Pos    (13U)
#define DSI_TCCR0_LPRX_TOCNT13_Msk    (0x1UL << DSI_TCCR0_LPRX_TOCNT13_Pos)    /*!< 0x00002000 */
#define DSI_TCCR0_LPRX_TOCNT13        DSI_TCCR0_LPRX_TOCNT13_Msk
#define DSI_TCCR0_LPRX_TOCNT14_Pos    (14U)
#define DSI_TCCR0_LPRX_TOCNT14_Msk    (0x1UL << DSI_TCCR0_LPRX_TOCNT14_Pos)    /*!< 0x00004000 */
#define DSI_TCCR0_LPRX_TOCNT14        DSI_TCCR0_LPRX_TOCNT14_Msk
#define DSI_TCCR0_LPRX_TOCNT15_Pos    (15U)
#define DSI_TCCR0_LPRX_TOCNT15_Msk    (0x1UL << DSI_TCCR0_LPRX_TOCNT15_Pos)    /*!< 0x00008000 */
#define DSI_TCCR0_LPRX_TOCNT15        DSI_TCCR0_LPRX_TOCNT15_Msk

#define DSI_TCCR0_HSTX_TOCNT_Pos      (16U)
#define DSI_TCCR0_HSTX_TOCNT_Msk      (0xFFFFUL << DSI_TCCR0_HSTX_TOCNT_Pos)   /*!< 0xFFFF0000 */
#define DSI_TCCR0_HSTX_TOCNT          DSI_TCCR0_HSTX_TOCNT_Msk                 /*!< High-Speed Transmission Timeout Counter */
#define DSI_TCCR0_HSTX_TOCNT0_Pos     (16U)
#define DSI_TCCR0_HSTX_TOCNT0_Msk     (0x1UL << DSI_TCCR0_HSTX_TOCNT0_Pos)     /*!< 0x00010000 */
#define DSI_TCCR0_HSTX_TOCNT0         DSI_TCCR0_HSTX_TOCNT0_Msk
#define DSI_TCCR0_HSTX_TOCNT1_Pos     (17U)
#define DSI_TCCR0_HSTX_TOCNT1_Msk     (0x1UL << DSI_TCCR0_HSTX_TOCNT1_Pos)     /*!< 0x00020000 */
#define DSI_TCCR0_HSTX_TOCNT1         DSI_TCCR0_HSTX_TOCNT1_Msk
#define DSI_TCCR0_HSTX_TOCNT2_Pos     (18U)
#define DSI_TCCR0_HSTX_TOCNT2_Msk     (0x1UL << DSI_TCCR0_HSTX_TOCNT2_Pos)     /*!< 0x00040000 */
#define DSI_TCCR0_HSTX_TOCNT2         DSI_TCCR0_HSTX_TOCNT2_Msk
#define DSI_TCCR0_HSTX_TOCNT3_Pos     (19U)
#define DSI_TCCR0_HSTX_TOCNT3_Msk     (0x1UL << DSI_TCCR0_HSTX_TOCNT3_Pos)     /*!< 0x00080000 */
#define DSI_TCCR0_HSTX_TOCNT3         DSI_TCCR0_HSTX_TOCNT3_Msk
#define DSI_TCCR0_HSTX_TOCNT4_Pos     (20U)
#define DSI_TCCR0_HSTX_TOCNT4_Msk     (0x1UL << DSI_TCCR0_HSTX_TOCNT4_Pos)     /*!< 0x00100000 */
#define DSI_TCCR0_HSTX_TOCNT4         DSI_TCCR0_HSTX_TOCNT4_Msk
#define DSI_TCCR0_HSTX_TOCNT5_Pos     (21U)
#define DSI_TCCR0_HSTX_TOCNT5_Msk     (0x1UL << DSI_TCCR0_HSTX_TOCNT5_Pos)     /*!< 0x00200000 */
#define DSI_TCCR0_HSTX_TOCNT5         DSI_TCCR0_HSTX_TOCNT5_Msk
#define DSI_TCCR0_HSTX_TOCNT6_Pos     (22U)
#define DSI_TCCR0_HSTX_TOCNT6_Msk     (0x1UL << DSI_TCCR0_HSTX_TOCNT6_Pos)     /*!< 0x00400000 */
#define DSI_TCCR0_HSTX_TOCNT6         DSI_TCCR0_HSTX_TOCNT6_Msk
#define DSI_TCCR0_HSTX_TOCNT7_Pos     (23U)
#define DSI_TCCR0_HSTX_TOCNT7_Msk     (0x1UL << DSI_TCCR0_HSTX_TOCNT7_Pos)     /*!< 0x00800000 */
#define DSI_TCCR0_HSTX_TOCNT7         DSI_TCCR0_HSTX_TOCNT7_Msk
#define DSI_TCCR0_HSTX_TOCNT8_Pos     (24U)
#define DSI_TCCR0_HSTX_TOCNT8_Msk     (0x1UL << DSI_TCCR0_HSTX_TOCNT8_Pos)     /*!< 0x01000000 */
#define DSI_TCCR0_HSTX_TOCNT8         DSI_TCCR0_HSTX_TOCNT8_Msk
#define DSI_TCCR0_HSTX_TOCNT9_Pos     (25U)
#define DSI_TCCR0_HSTX_TOCNT9_Msk     (0x1UL << DSI_TCCR0_HSTX_TOCNT9_Pos)     /*!< 0x02000000 */
#define DSI_TCCR0_HSTX_TOCNT9         DSI_TCCR0_HSTX_TOCNT9_Msk
#define DSI_TCCR0_HSTX_TOCNT10_Pos    (26U)
#define DSI_TCCR0_HSTX_TOCNT10_Msk    (0x1UL << DSI_TCCR0_HSTX_TOCNT10_Pos)    /*!< 0x04000000 */
#define DSI_TCCR0_HSTX_TOCNT10        DSI_TCCR0_HSTX_TOCNT10_Msk
#define DSI_TCCR0_HSTX_TOCNT11_Pos    (27U)
#define DSI_TCCR0_HSTX_TOCNT11_Msk    (0x1UL << DSI_TCCR0_HSTX_TOCNT11_Pos)    /*!< 0x08000000 */
#define DSI_TCCR0_HSTX_TOCNT11        DSI_TCCR0_HSTX_TOCNT11_Msk
#define DSI_TCCR0_HSTX_TOCNT12_Pos    (28U)
#define DSI_TCCR0_HSTX_TOCNT12_Msk    (0x1UL << DSI_TCCR0_HSTX_TOCNT12_Pos)    /*!< 0x10000000 */
#define DSI_TCCR0_HSTX_TOCNT12        DSI_TCCR0_HSTX_TOCNT12_Msk
#define DSI_TCCR0_HSTX_TOCNT13_Pos    (29U)
#define DSI_TCCR0_HSTX_TOCNT13_Msk    (0x1UL << DSI_TCCR0_HSTX_TOCNT13_Pos)    /*!< 0x20000000 */
#define DSI_TCCR0_HSTX_TOCNT13        DSI_TCCR0_HSTX_TOCNT13_Msk
#define DSI_TCCR0_HSTX_TOCNT14_Pos    (30U)
#define DSI_TCCR0_HSTX_TOCNT14_Msk    (0x1UL << DSI_TCCR0_HSTX_TOCNT14_Pos)    /*!< 0x40000000 */
#define DSI_TCCR0_HSTX_TOCNT14        DSI_TCCR0_HSTX_TOCNT14_Msk
#define DSI_TCCR0_HSTX_TOCNT15_Pos    (31U)
#define DSI_TCCR0_HSTX_TOCNT15_Msk    (0x1UL << DSI_TCCR0_HSTX_TOCNT15_Pos)    /*!< 0x80000000 */
#define DSI_TCCR0_HSTX_TOCNT15        DSI_TCCR0_HSTX_TOCNT15_Msk

/*******************  Bit definition for DSI_TCCR1 register  **************/
#define DSI_TCCR1_HSRD_TOCNT_Pos      (0U)
#define DSI_TCCR1_HSRD_TOCNT_Msk      (0xFFFFUL << DSI_TCCR1_HSRD_TOCNT_Pos)   /*!< 0x0000FFFF */
#define DSI_TCCR1_HSRD_TOCNT          DSI_TCCR1_HSRD_TOCNT_Msk                 /*!< High-Speed Read Timeout Counter */
#define DSI_TCCR1_HSRD_TOCNT0_Pos     (0U)
#define DSI_TCCR1_HSRD_TOCNT0_Msk     (0x1UL << DSI_TCCR1_HSRD_TOCNT0_Pos)     /*!< 0x00000001 */
#define DSI_TCCR1_HSRD_TOCNT0         DSI_TCCR1_HSRD_TOCNT0_Msk
#define DSI_TCCR1_HSRD_TOCNT1_Pos     (1U)
#define DSI_TCCR1_HSRD_TOCNT1_Msk     (0x1UL << DSI_TCCR1_HSRD_TOCNT1_Pos)     /*!< 0x00000002 */
#define DSI_TCCR1_HSRD_TOCNT1         DSI_TCCR1_HSRD_TOCNT1_Msk
#define DSI_TCCR1_HSRD_TOCNT2_Pos     (2U)
#define DSI_TCCR1_HSRD_TOCNT2_Msk     (0x1UL << DSI_TCCR1_HSRD_TOCNT2_Pos)     /*!< 0x00000004 */
#define DSI_TCCR1_HSRD_TOCNT2         DSI_TCCR1_HSRD_TOCNT2_Msk
#define DSI_TCCR1_HSRD_TOCNT3_Pos     (3U)
#define DSI_TCCR1_HSRD_TOCNT3_Msk     (0x1UL << DSI_TCCR1_HSRD_TOCNT3_Pos)     /*!< 0x00000008 */
#define DSI_TCCR1_HSRD_TOCNT3         DSI_TCCR1_HSRD_TOCNT3_Msk
#define DSI_TCCR1_HSRD_TOCNT4_Pos     (4U)
#define DSI_TCCR1_HSRD_TOCNT4_Msk     (0x1UL << DSI_TCCR1_HSRD_TOCNT4_Pos)     /*!< 0x00000010 */
#define DSI_TCCR1_HSRD_TOCNT4         DSI_TCCR1_HSRD_TOCNT4_Msk
#define DSI_TCCR1_HSRD_TOCNT5_Pos     (5U)
#define DSI_TCCR1_HSRD_TOCNT5_Msk     (0x1UL << DSI_TCCR1_HSRD_TOCNT5_Pos)     /*!< 0x00000020 */
#define DSI_TCCR1_HSRD_TOCNT5         DSI_TCCR1_HSRD_TOCNT5_Msk
#define DSI_TCCR1_HSRD_TOCNT6_Pos     (6U)
#define DSI_TCCR1_HSRD_TOCNT6_Msk     (0x1UL << DSI_TCCR1_HSRD_TOCNT6_Pos)     /*!< 0x00000040 */
#define DSI_TCCR1_HSRD_TOCNT6         DSI_TCCR1_HSRD_TOCNT6_Msk
#define DSI_TCCR1_HSRD_TOCNT7_Pos     (7U)
#define DSI_TCCR1_HSRD_TOCNT7_Msk     (0x1UL << DSI_TCCR1_HSRD_TOCNT7_Pos)     /*!< 0x00000080 */
#define DSI_TCCR1_HSRD_TOCNT7         DSI_TCCR1_HSRD_TOCNT7_Msk
#define DSI_TCCR1_HSRD_TOCNT8_Pos     (8U)
#define DSI_TCCR1_HSRD_TOCNT8_Msk     (0x1UL << DSI_TCCR1_HSRD_TOCNT8_Pos)     /*!< 0x00000100 */
#define DSI_TCCR1_HSRD_TOCNT8         DSI_TCCR1_HSRD_TOCNT8_Msk
#define DSI_TCCR1_HSRD_TOCNT9_Pos     (9U)
#define DSI_TCCR1_HSRD_TOCNT9_Msk     (0x1UL << DSI_TCCR1_HSRD_TOCNT9_Pos)     /*!< 0x00000200 */
#define DSI_TCCR1_HSRD_TOCNT9         DSI_TCCR1_HSRD_TOCNT9_Msk
#define DSI_TCCR1_HSRD_TOCNT10_Pos    (10U)
#define DSI_TCCR1_HSRD_TOCNT10_Msk    (0x1UL << DSI_TCCR1_HSRD_TOCNT10_Pos)    /*!< 0x00000400 */
#define DSI_TCCR1_HSRD_TOCNT10        DSI_TCCR1_HSRD_TOCNT10_Msk
#define DSI_TCCR1_HSRD_TOCNT11_Pos    (11U)
#define DSI_TCCR1_HSRD_TOCNT11_Msk    (0x1UL << DSI_TCCR1_HSRD_TOCNT11_Pos)    /*!< 0x00000800 */
#define DSI_TCCR1_HSRD_TOCNT11        DSI_TCCR1_HSRD_TOCNT11_Msk
#define DSI_TCCR1_HSRD_TOCNT12_Pos    (12U)
#define DSI_TCCR1_HSRD_TOCNT12_Msk    (0x1UL << DSI_TCCR1_HSRD_TOCNT12_Pos)    /*!< 0x00001000 */
#define DSI_TCCR1_HSRD_TOCNT12        DSI_TCCR1_HSRD_TOCNT12_Msk
#define DSI_TCCR1_HSRD_TOCNT13_Pos    (13U)
#define DSI_TCCR1_HSRD_TOCNT13_Msk    (0x1UL << DSI_TCCR1_HSRD_TOCNT13_Pos)    /*!< 0x00002000 */
#define DSI_TCCR1_HSRD_TOCNT13        DSI_TCCR1_HSRD_TOCNT13_Msk
#define DSI_TCCR1_HSRD_TOCNT14_Pos    (14U)
#define DSI_TCCR1_HSRD_TOCNT14_Msk    (0x1UL << DSI_TCCR1_HSRD_TOCNT14_Pos)    /*!< 0x00004000 */
#define DSI_TCCR1_HSRD_TOCNT14        DSI_TCCR1_HSRD_TOCNT14_Msk
#define DSI_TCCR1_HSRD_TOCNT15_Pos    (15U)
#define DSI_TCCR1_HSRD_TOCNT15_Msk    (0x1UL << DSI_TCCR1_HSRD_TOCNT15_Pos)    /*!< 0x00008000 */
#define DSI_TCCR1_HSRD_TOCNT15        DSI_TCCR1_HSRD_TOCNT15_Msk

/*******************  Bit definition for DSI_TCCR2 register  **************/
#define DSI_TCCR2_LPRD_TOCNT_Pos      (0U)
#define DSI_TCCR2_LPRD_TOCNT_Msk      (0xFFFFUL << DSI_TCCR2_LPRD_TOCNT_Pos)   /*!< 0x0000FFFF */
#define DSI_TCCR2_LPRD_TOCNT          DSI_TCCR2_LPRD_TOCNT_Msk                 /*!< Low-Power Read Timeout Counter */
#define DSI_TCCR2_LPRD_TOCNT0_Pos     (0U)
#define DSI_TCCR2_LPRD_TOCNT0_Msk     (0x1UL << DSI_TCCR2_LPRD_TOCNT0_Pos)     /*!< 0x00000001 */
#define DSI_TCCR2_LPRD_TOCNT0         DSI_TCCR2_LPRD_TOCNT0_Msk
#define DSI_TCCR2_LPRD_TOCNT1_Pos     (1U)
#define DSI_TCCR2_LPRD_TOCNT1_Msk     (0x1UL << DSI_TCCR2_LPRD_TOCNT1_Pos)     /*!< 0x00000002 */
#define DSI_TCCR2_LPRD_TOCNT1         DSI_TCCR2_LPRD_TOCNT1_Msk
#define DSI_TCCR2_LPRD_TOCNT2_Pos     (2U)
#define DSI_TCCR2_LPRD_TOCNT2_Msk     (0x1UL << DSI_TCCR2_LPRD_TOCNT2_Pos)     /*!< 0x00000004 */
#define DSI_TCCR2_LPRD_TOCNT2         DSI_TCCR2_LPRD_TOCNT2_Msk
#define DSI_TCCR2_LPRD_TOCNT3_Pos     (3U)
#define DSI_TCCR2_LPRD_TOCNT3_Msk     (0x1UL << DSI_TCCR2_LPRD_TOCNT3_Pos)     /*!< 0x00000008 */
#define DSI_TCCR2_LPRD_TOCNT3         DSI_TCCR2_LPRD_TOCNT3_Msk
#define DSI_TCCR2_LPRD_TOCNT4_Pos     (4U)
#define DSI_TCCR2_LPRD_TOCNT4_Msk     (0x1UL << DSI_TCCR2_LPRD_TOCNT4_Pos)     /*!< 0x00000010 */
#define DSI_TCCR2_LPRD_TOCNT4         DSI_TCCR2_LPRD_TOCNT4_Msk
#define DSI_TCCR2_LPRD_TOCNT5_Pos     (5U)
#define DSI_TCCR2_LPRD_TOCNT5_Msk     (0x1UL << DSI_TCCR2_LPRD_TOCNT5_Pos)     /*!< 0x00000020 */
#define DSI_TCCR2_LPRD_TOCNT5         DSI_TCCR2_LPRD_TOCNT5_Msk
#define DSI_TCCR2_LPRD_TOCNT6_Pos     (6U)
#define DSI_TCCR2_LPRD_TOCNT6_Msk     (0x1UL << DSI_TCCR2_LPRD_TOCNT6_Pos)     /*!< 0x00000040 */
#define DSI_TCCR2_LPRD_TOCNT6         DSI_TCCR2_LPRD_TOCNT6_Msk
#define DSI_TCCR2_LPRD_TOCNT7_Pos     (7U)
#define DSI_TCCR2_LPRD_TOCNT7_Msk     (0x1UL << DSI_TCCR2_LPRD_TOCNT7_Pos)     /*!< 0x00000080 */
#define DSI_TCCR2_LPRD_TOCNT7         DSI_TCCR2_LPRD_TOCNT7_Msk
#define DSI_TCCR2_LPRD_TOCNT8_Pos     (8U)
#define DSI_TCCR2_LPRD_TOCNT8_Msk     (0x1UL << DSI_TCCR2_LPRD_TOCNT8_Pos)     /*!< 0x00000100 */
#define DSI_TCCR2_LPRD_TOCNT8         DSI_TCCR2_LPRD_TOCNT8_Msk
#define DSI_TCCR2_LPRD_TOCNT9_Pos     (9U)
#define DSI_TCCR2_LPRD_TOCNT9_Msk     (0x1UL << DSI_TCCR2_LPRD_TOCNT9_Pos)     /*!< 0x00000200 */
#define DSI_TCCR2_LPRD_TOCNT9         DSI_TCCR2_LPRD_TOCNT9_Msk
#define DSI_TCCR2_LPRD_TOCNT10_Pos    (10U)
#define DSI_TCCR2_LPRD_TOCNT10_Msk    (0x1UL << DSI_TCCR2_LPRD_TOCNT10_Pos)    /*!< 0x00000400 */
#define DSI_TCCR2_LPRD_TOCNT10        DSI_TCCR2_LPRD_TOCNT10_Msk
#define DSI_TCCR2_LPRD_TOCNT11_Pos    (11U)
#define DSI_TCCR2_LPRD_TOCNT11_Msk    (0x1UL << DSI_TCCR2_LPRD_TOCNT11_Pos)    /*!< 0x00000800 */
#define DSI_TCCR2_LPRD_TOCNT11        DSI_TCCR2_LPRD_TOCNT11_Msk
#define DSI_TCCR2_LPRD_TOCNT12_Pos    (12U)
#define DSI_TCCR2_LPRD_TOCNT12_Msk    (0x1UL << DSI_TCCR2_LPRD_TOCNT12_Pos)    /*!< 0x00001000 */
#define DSI_TCCR2_LPRD_TOCNT12        DSI_TCCR2_LPRD_TOCNT12_Msk
#define DSI_TCCR2_LPRD_TOCNT13_Pos    (13U)
#define DSI_TCCR2_LPRD_TOCNT13_Msk    (0x1UL << DSI_TCCR2_LPRD_TOCNT13_Pos)    /*!< 0x00002000 */
#define DSI_TCCR2_LPRD_TOCNT13        DSI_TCCR2_LPRD_TOCNT13_Msk
#define DSI_TCCR2_LPRD_TOCNT14_Pos    (14U)
#define DSI_TCCR2_LPRD_TOCNT14_Msk    (0x1UL << DSI_TCCR2_LPRD_TOCNT14_Pos)    /*!< 0x00004000 */
#define DSI_TCCR2_LPRD_TOCNT14        DSI_TCCR2_LPRD_TOCNT14_Msk
#define DSI_TCCR2_LPRD_TOCNT15_Pos    (15U)
#define DSI_TCCR2_LPRD_TOCNT15_Msk    (0x1UL << DSI_TCCR2_LPRD_TOCNT15_Pos)    /*!< 0x00008000 */
#define DSI_TCCR2_LPRD_TOCNT15        DSI_TCCR2_LPRD_TOCNT15_Msk

/*******************  Bit definition for DSI_TCCR3 register  **************/
#define DSI_TCCR3_HSWR_TOCNT_Pos      (0U)
#define DSI_TCCR3_HSWR_TOCNT_Msk      (0xFFFFUL << DSI_TCCR3_HSWR_TOCNT_Pos)   /*!< 0x0000FFFF */
#define DSI_TCCR3_HSWR_TOCNT          DSI_TCCR3_HSWR_TOCNT_Msk                 /*!< High-Speed Write Timeout Counter */
#define DSI_TCCR3_HSWR_TOCNT0_Pos     (0U)
#define DSI_TCCR3_HSWR_TOCNT0_Msk     (0x1UL << DSI_TCCR3_HSWR_TOCNT0_Pos)     /*!< 0x00000001 */
#define DSI_TCCR3_HSWR_TOCNT0         DSI_TCCR3_HSWR_TOCNT0_Msk
#define DSI_TCCR3_HSWR_TOCNT1_Pos     (1U)
#define DSI_TCCR3_HSWR_TOCNT1_Msk     (0x1UL << DSI_TCCR3_HSWR_TOCNT1_Pos)     /*!< 0x00000002 */
#define DSI_TCCR3_HSWR_TOCNT1         DSI_TCCR3_HSWR_TOCNT1_Msk
#define DSI_TCCR3_HSWR_TOCNT2_Pos     (2U)
#define DSI_TCCR3_HSWR_TOCNT2_Msk     (0x1UL << DSI_TCCR3_HSWR_TOCNT2_Pos)     /*!< 0x00000004 */
#define DSI_TCCR3_HSWR_TOCNT2         DSI_TCCR3_HSWR_TOCNT2_Msk
#define DSI_TCCR3_HSWR_TOCNT3_Pos     (3U)
#define DSI_TCCR3_HSWR_TOCNT3_Msk     (0x1UL << DSI_TCCR3_HSWR_TOCNT3_Pos)     /*!< 0x00000008 */
#define DSI_TCCR3_HSWR_TOCNT3         DSI_TCCR3_HSWR_TOCNT3_Msk
#define DSI_TCCR3_HSWR_TOCNT4_Pos     (4U)
#define DSI_TCCR3_HSWR_TOCNT4_Msk     (0x1UL << DSI_TCCR3_HSWR_TOCNT4_Pos)     /*!< 0x00000010 */
#define DSI_TCCR3_HSWR_TOCNT4         DSI_TCCR3_HSWR_TOCNT4_Msk
#define DSI_TCCR3_HSWR_TOCNT5_Pos     (5U)
#define DSI_TCCR3_HSWR_TOCNT5_Msk     (0x1UL << DSI_TCCR3_HSWR_TOCNT5_Pos)     /*!< 0x00000020 */
#define DSI_TCCR3_HSWR_TOCNT5         DSI_TCCR3_HSWR_TOCNT5_Msk
#define DSI_TCCR3_HSWR_TOCNT6_Pos     (6U)
#define DSI_TCCR3_HSWR_TOCNT6_Msk     (0x1UL << DSI_TCCR3_HSWR_TOCNT6_Pos)     /*!< 0x00000040 */
#define DSI_TCCR3_HSWR_TOCNT6         DSI_TCCR3_HSWR_TOCNT6_Msk
#define DSI_TCCR3_HSWR_TOCNT7_Pos     (7U)
#define DSI_TCCR3_HSWR_TOCNT7_Msk     (0x1UL << DSI_TCCR3_HSWR_TOCNT7_Pos)     /*!< 0x00000080 */
#define DSI_TCCR3_HSWR_TOCNT7         DSI_TCCR3_HSWR_TOCNT7_Msk
#define DSI_TCCR3_HSWR_TOCNT8_Pos     (8U)
#define DSI_TCCR3_HSWR_TOCNT8_Msk     (0x1UL << DSI_TCCR3_HSWR_TOCNT8_Pos)     /*!< 0x00000100 */
#define DSI_TCCR3_HSWR_TOCNT8         DSI_TCCR3_HSWR_TOCNT8_Msk
#define DSI_TCCR3_HSWR_TOCNT9_Pos     (9U)
#define DSI_TCCR3_HSWR_TOCNT9_Msk     (0x1UL << DSI_TCCR3_HSWR_TOCNT9_Pos)     /*!< 0x00000200 */
#define DSI_TCCR3_HSWR_TOCNT9         DSI_TCCR3_HSWR_TOCNT9_Msk
#define DSI_TCCR3_HSWR_TOCNT10_Pos    (10U)
#define DSI_TCCR3_HSWR_TOCNT10_Msk    (0x1UL << DSI_TCCR3_HSWR_TOCNT10_Pos)    /*!< 0x00000400 */
#define DSI_TCCR3_HSWR_TOCNT10        DSI_TCCR3_HSWR_TOCNT10_Msk
#define DSI_TCCR3_HSWR_TOCNT11_Pos    (11U)
#define DSI_TCCR3_HSWR_TOCNT11_Msk    (0x1UL << DSI_TCCR3_HSWR_TOCNT11_Pos)    /*!< 0x00000800 */
#define DSI_TCCR3_HSWR_TOCNT11        DSI_TCCR3_HSWR_TOCNT11_Msk
#define DSI_TCCR3_HSWR_TOCNT12_Pos    (12U)
#define DSI_TCCR3_HSWR_TOCNT12_Msk    (0x1UL << DSI_TCCR3_HSWR_TOCNT12_Pos)    /*!< 0x00001000 */
#define DSI_TCCR3_HSWR_TOCNT12        DSI_TCCR3_HSWR_TOCNT12_Msk
#define DSI_TCCR3_HSWR_TOCNT13_Pos    (13U)
#define DSI_TCCR3_HSWR_TOCNT13_Msk    (0x1UL << DSI_TCCR3_HSWR_TOCNT13_Pos)    /*!< 0x00002000 */
#define DSI_TCCR3_HSWR_TOCNT13        DSI_TCCR3_HSWR_TOCNT13_Msk
#define DSI_TCCR3_HSWR_TOCNT14_Pos    (14U)
#define DSI_TCCR3_HSWR_TOCNT14_Msk    (0x1UL << DSI_TCCR3_HSWR_TOCNT14_Pos)    /*!< 0x00004000 */
#define DSI_TCCR3_HSWR_TOCNT14        DSI_TCCR3_HSWR_TOCNT14_Msk
#define DSI_TCCR3_HSWR_TOCNT15_Pos    (15U)
#define DSI_TCCR3_HSWR_TOCNT15_Msk    (0x1UL << DSI_TCCR3_HSWR_TOCNT15_Pos)    /*!< 0x00008000 */
#define DSI_TCCR3_HSWR_TOCNT15        DSI_TCCR3_HSWR_TOCNT15_Msk

#define DSI_TCCR3_PM_Pos              (24U)
#define DSI_TCCR3_PM_Msk              (0x1UL << DSI_TCCR3_PM_Pos)              /*!< 0x01000000 */
#define DSI_TCCR3_PM                  DSI_TCCR3_PM_Msk                         /*!< Presp Mode */

/*******************  Bit definition for DSI_TCCR4 register  **************/
#define DSI_TCCR4_LPWR_TOCNT_Pos      (0U)
#define DSI_TCCR4_LPWR_TOCNT_Msk      (0xFFFFUL << DSI_TCCR4_LPWR_TOCNT_Pos)   /*!< 0x0000FFFF */
#define DSI_TCCR4_LPWR_TOCNT          DSI_TCCR4_LPWR_TOCNT_Msk                 /*!< Low-Power Write Timeout Counter */
#define DSI_TCCR4_LPWR_TOCNT0_Pos     (0U)
#define DSI_TCCR4_LPWR_TOCNT0_Msk     (0x1UL << DSI_TCCR4_LPWR_TOCNT0_Pos)     /*!< 0x00000001 */
#define DSI_TCCR4_LPWR_TOCNT0         DSI_TCCR4_LPWR_TOCNT0_Msk
#define DSI_TCCR4_LPWR_TOCNT1_Pos     (1U)
#define DSI_TCCR4_LPWR_TOCNT1_Msk     (0x1UL << DSI_TCCR4_LPWR_TOCNT1_Pos)     /*!< 0x00000002 */
#define DSI_TCCR4_LPWR_TOCNT1         DSI_TCCR4_LPWR_TOCNT1_Msk
#define DSI_TCCR4_LPWR_TOCNT2_Pos     (2U)
#define DSI_TCCR4_LPWR_TOCNT2_Msk     (0x1UL << DSI_TCCR4_LPWR_TOCNT2_Pos)     /*!< 0x00000004 */
#define DSI_TCCR4_LPWR_TOCNT2         DSI_TCCR4_LPWR_TOCNT2_Msk
#define DSI_TCCR4_LPWR_TOCNT3_Pos     (3U)
#define DSI_TCCR4_LPWR_TOCNT3_Msk     (0x1UL << DSI_TCCR4_LPWR_TOCNT3_Pos)     /*!< 0x00000008 */
#define DSI_TCCR4_LPWR_TOCNT3         DSI_TCCR4_LPWR_TOCNT3_Msk
#define DSI_TCCR4_LPWR_TOCNT4_Pos     (4U)
#define DSI_TCCR4_LPWR_TOCNT4_Msk     (0x1UL << DSI_TCCR4_LPWR_TOCNT4_Pos)     /*!< 0x00000010 */
#define DSI_TCCR4_LPWR_TOCNT4         DSI_TCCR4_LPWR_TOCNT4_Msk
#define DSI_TCCR4_LPWR_TOCNT5_Pos     (5U)
#define DSI_TCCR4_LPWR_TOCNT5_Msk     (0x1UL << DSI_TCCR4_LPWR_TOCNT5_Pos)     /*!< 0x00000020 */
#define DSI_TCCR4_LPWR_TOCNT5         DSI_TCCR4_LPWR_TOCNT5_Msk
#define DSI_TCCR4_LPWR_TOCNT6_Pos     (6U)
#define DSI_TCCR4_LPWR_TOCNT6_Msk     (0x1UL << DSI_TCCR4_LPWR_TOCNT6_Pos)     /*!< 0x00000040 */
#define DSI_TCCR4_LPWR_TOCNT6         DSI_TCCR4_LPWR_TOCNT6_Msk
#define DSI_TCCR4_LPWR_TOCNT7_Pos     (7U)
#define DSI_TCCR4_LPWR_TOCNT7_Msk     (0x1UL << DSI_TCCR4_LPWR_TOCNT7_Pos)     /*!< 0x00000080 */
#define DSI_TCCR4_LPWR_TOCNT7         DSI_TCCR4_LPWR_TOCNT7_Msk
#define DSI_TCCR4_LPWR_TOCNT8_Pos     (8U)
#define DSI_TCCR4_LPWR_TOCNT8_Msk     (0x1UL << DSI_TCCR4_LPWR_TOCNT8_Pos)     /*!< 0x00000100 */
#define DSI_TCCR4_LPWR_TOCNT8         DSI_TCCR4_LPWR_TOCNT8_Msk
#define DSI_TCCR4_LPWR_TOCNT9_Pos     (9U)
#define DSI_TCCR4_LPWR_TOCNT9_Msk     (0x1UL << DSI_TCCR4_LPWR_TOCNT9_Pos)     /*!< 0x00000200 */
#define DSI_TCCR4_LPWR_TOCNT9         DSI_TCCR4_LPWR_TOCNT9_Msk
#define DSI_TCCR4_LPWR_TOCNT10_Pos    (10U)
#define DSI_TCCR4_LPWR_TOCNT10_Msk    (0x1UL << DSI_TCCR4_LPWR_TOCNT10_Pos)    /*!< 0x00000400 */
#define DSI_TCCR4_LPWR_TOCNT10        DSI_TCCR4_LPWR_TOCNT10_Msk
#define DSI_TCCR4_LPWR_TOCNT11_Pos    (11U)
#define DSI_TCCR4_LPWR_TOCNT11_Msk    (0x1UL << DSI_TCCR4_LPWR_TOCNT11_Pos)    /*!< 0x00000800 */
#define DSI_TCCR4_LPWR_TOCNT11        DSI_TCCR4_LPWR_TOCNT11_Msk
#define DSI_TCCR4_LPWR_TOCNT12_Pos    (12U)
#define DSI_TCCR4_LPWR_TOCNT12_Msk    (0x1UL << DSI_TCCR4_LPWR_TOCNT12_Pos)    /*!< 0x00001000 */
#define DSI_TCCR4_LPWR_TOCNT12        DSI_TCCR4_LPWR_TOCNT12_Msk
#define DSI_TCCR4_LPWR_TOCNT13_Pos    (13U)
#define DSI_TCCR4_LPWR_TOCNT13_Msk    (0x1UL << DSI_TCCR4_LPWR_TOCNT13_Pos)    /*!< 0x00002000 */
#define DSI_TCCR4_LPWR_TOCNT13        DSI_TCCR4_LPWR_TOCNT13_Msk
#define DSI_TCCR4_LPWR_TOCNT14_Pos    (14U)
#define DSI_TCCR4_LPWR_TOCNT14_Msk    (0x1UL << DSI_TCCR4_LPWR_TOCNT14_Pos)    /*!< 0x00004000 */
#define DSI_TCCR4_LPWR_TOCNT14        DSI_TCCR4_LPWR_TOCNT14_Msk
#define DSI_TCCR4_LPWR_TOCNT15_Pos    (15U)
#define DSI_TCCR4_LPWR_TOCNT15_Msk    (0x1UL << DSI_TCCR4_LPWR_TOCNT15_Pos)    /*!< 0x00008000 */
#define DSI_TCCR4_LPWR_TOCNT15        DSI_TCCR4_LPWR_TOCNT15_Msk

/*******************  Bit definition for DSI_TCCR5 register  **************/
#define DSI_TCCR5_BTA_TOCNT_Pos       (0U)
#define DSI_TCCR5_BTA_TOCNT_Msk       (0xFFFFUL << DSI_TCCR5_BTA_TOCNT_Pos)    /*!< 0x0000FFFF */
#define DSI_TCCR5_BTA_TOCNT           DSI_TCCR5_BTA_TOCNT_Msk                  /*!< Bus-Turn-Around Timeout Counter */
#define DSI_TCCR5_BTA_TOCNT0_Pos      (0U)
#define DSI_TCCR5_BTA_TOCNT0_Msk      (0x1UL << DSI_TCCR5_BTA_TOCNT0_Pos)      /*!< 0x00000001 */
#define DSI_TCCR5_BTA_TOCNT0          DSI_TCCR5_BTA_TOCNT0_Msk
#define DSI_TCCR5_BTA_TOCNT1_Pos      (1U)
#define DSI_TCCR5_BTA_TOCNT1_Msk      (0x1UL << DSI_TCCR5_BTA_TOCNT1_Pos)      /*!< 0x00000002 */
#define DSI_TCCR5_BTA_TOCNT1          DSI_TCCR5_BTA_TOCNT1_Msk
#define DSI_TCCR5_BTA_TOCNT2_Pos      (2U)
#define DSI_TCCR5_BTA_TOCNT2_Msk      (0x1UL << DSI_TCCR5_BTA_TOCNT2_Pos)      /*!< 0x00000004 */
#define DSI_TCCR5_BTA_TOCNT2          DSI_TCCR5_BTA_TOCNT2_Msk
#define DSI_TCCR5_BTA_TOCNT3_Pos      (3U)
#define DSI_TCCR5_BTA_TOCNT3_Msk      (0x1UL << DSI_TCCR5_BTA_TOCNT3_Pos)      /*!< 0x00000008 */
#define DSI_TCCR5_BTA_TOCNT3          DSI_TCCR5_BTA_TOCNT3_Msk
#define DSI_TCCR5_BTA_TOCNT4_Pos      (4U)
#define DSI_TCCR5_BTA_TOCNT4_Msk      (0x1UL << DSI_TCCR5_BTA_TOCNT4_Pos)      /*!< 0x00000010 */
#define DSI_TCCR5_BTA_TOCNT4          DSI_TCCR5_BTA_TOCNT4_Msk
#define DSI_TCCR5_BTA_TOCNT5_Pos      (5U)
#define DSI_TCCR5_BTA_TOCNT5_Msk      (0x1UL << DSI_TCCR5_BTA_TOCNT5_Pos)      /*!< 0x00000020 */
#define DSI_TCCR5_BTA_TOCNT5          DSI_TCCR5_BTA_TOCNT5_Msk
#define DSI_TCCR5_BTA_TOCNT6_Pos      (6U)
#define DSI_TCCR5_BTA_TOCNT6_Msk      (0x1UL << DSI_TCCR5_BTA_TOCNT6_Pos)      /*!< 0x00000040 */
#define DSI_TCCR5_BTA_TOCNT6          DSI_TCCR5_BTA_TOCNT6_Msk
#define DSI_TCCR5_BTA_TOCNT7_Pos      (7U)
#define DSI_TCCR5_BTA_TOCNT7_Msk      (0x1UL << DSI_TCCR5_BTA_TOCNT7_Pos)      /*!< 0x00000080 */
#define DSI_TCCR5_BTA_TOCNT7          DSI_TCCR5_BTA_TOCNT7_Msk
#define DSI_TCCR5_BTA_TOCNT8_Pos      (8U)
#define DSI_TCCR5_BTA_TOCNT8_Msk      (0x1UL << DSI_TCCR5_BTA_TOCNT8_Pos)      /*!< 0x00000100 */
#define DSI_TCCR5_BTA_TOCNT8          DSI_TCCR5_BTA_TOCNT8_Msk
#define DSI_TCCR5_BTA_TOCNT9_Pos      (9U)
#define DSI_TCCR5_BTA_TOCNT9_Msk      (0x1UL << DSI_TCCR5_BTA_TOCNT9_Pos)      /*!< 0x00000200 */
#define DSI_TCCR5_BTA_TOCNT9          DSI_TCCR5_BTA_TOCNT9_Msk
#define DSI_TCCR5_BTA_TOCNT10_Pos     (10U)
#define DSI_TCCR5_BTA_TOCNT10_Msk     (0x1UL << DSI_TCCR5_BTA_TOCNT10_Pos)     /*!< 0x00000400 */
#define DSI_TCCR5_BTA_TOCNT10         DSI_TCCR5_BTA_TOCNT10_Msk
#define DSI_TCCR5_BTA_TOCNT11_Pos     (11U)
#define DSI_TCCR5_BTA_TOCNT11_Msk     (0x1UL << DSI_TCCR5_BTA_TOCNT11_Pos)     /*!< 0x00000800 */
#define DSI_TCCR5_BTA_TOCNT11         DSI_TCCR5_BTA_TOCNT11_Msk
#define DSI_TCCR5_BTA_TOCNT12_Pos     (12U)
#define DSI_TCCR5_BTA_TOCNT12_Msk     (0x1UL << DSI_TCCR5_BTA_TOCNT12_Pos)     /*!< 0x00001000 */
#define DSI_TCCR5_BTA_TOCNT12         DSI_TCCR5_BTA_TOCNT12_Msk
#define DSI_TCCR5_BTA_TOCNT13_Pos     (13U)
#define DSI_TCCR5_BTA_TOCNT13_Msk     (0x1UL << DSI_TCCR5_BTA_TOCNT13_Pos)     /*!< 0x00002000 */
#define DSI_TCCR5_BTA_TOCNT13         DSI_TCCR5_BTA_TOCNT13_Msk
#define DSI_TCCR5_BTA_TOCNT14_Pos     (14U)
#define DSI_TCCR5_BTA_TOCNT14_Msk     (0x1UL << DSI_TCCR5_BTA_TOCNT14_Pos)     /*!< 0x00004000 */
#define DSI_TCCR5_BTA_TOCNT14         DSI_TCCR5_BTA_TOCNT14_Msk
#define DSI_TCCR5_BTA_TOCNT15_Pos     (15U)
#define DSI_TCCR5_BTA_TOCNT15_Msk     (0x1UL << DSI_TCCR5_BTA_TOCNT15_Pos)     /*!< 0x00008000 */
#define DSI_TCCR5_BTA_TOCNT15         DSI_TCCR5_BTA_TOCNT15_Msk

/*******************  Bit definition for DSI_TDCR register  ***************/
#define DSI_TDCR_3DM                  ((uint32_t)0x00000003U)                  /*!< 3D Mode */
#define DSI_TDCR_3DM0                 ((uint32_t)0x00000001U)
#define DSI_TDCR_3DM1                 ((uint32_t)0x00000002U)

#define DSI_TDCR_3DF                  ((uint32_t)0x0000000CU)                  /*!< 3D Format */
#define DSI_TDCR_3DF0                 ((uint32_t)0x00000004U)
#define DSI_TDCR_3DF1                 ((uint32_t)0x00000008U)

#define DSI_TDCR_SVS_Pos              (4U)
#define DSI_TDCR_SVS_Msk              (0x1UL << DSI_TDCR_SVS_Pos)              /*!< 0x00000010 */
#define DSI_TDCR_SVS                  DSI_TDCR_SVS_Msk                         /*!< Second VSYNC */
#define DSI_TDCR_RF_Pos               (5U)
#define DSI_TDCR_RF_Msk               (0x1UL << DSI_TDCR_RF_Pos)               /*!< 0x00000020 */
#define DSI_TDCR_RF                   DSI_TDCR_RF_Msk                          /*!< Right First */
#define DSI_TDCR_S3DC_Pos             (16U)
#define DSI_TDCR_S3DC_Msk             (0x1UL << DSI_TDCR_S3DC_Pos)             /*!< 0x00010000 */
#define DSI_TDCR_S3DC                 DSI_TDCR_S3DC_Msk                        /*!< Send 3D Control */

/*******************  Bit definition for DSI_CLCR register  ***************/
#define DSI_CLCR_DPCC_Pos             (0U)
#define DSI_CLCR_DPCC_Msk             (0x1UL << DSI_CLCR_DPCC_Pos)             /*!< 0x00000001 */
#define DSI_CLCR_DPCC                 DSI_CLCR_DPCC_Msk                        /*!< D-PHY Clock Control */
#define DSI_CLCR_ACR_Pos              (1U)
#define DSI_CLCR_ACR_Msk              (0x1UL << DSI_CLCR_ACR_Pos)              /*!< 0x00000002 */
#define DSI_CLCR_ACR                  DSI_CLCR_ACR_Msk                         /*!< Automatic Clocklane Control */

/*******************  Bit definition for DSI_CLTCR register  **************/
#define DSI_CLTCR_LP2HS_TIME_Pos      (0U)
#define DSI_CLTCR_LP2HS_TIME_Msk      (0x3FFUL << DSI_CLTCR_LP2HS_TIME_Pos)    /*!< 0x000003FF */
#define DSI_CLTCR_LP2HS_TIME          DSI_CLTCR_LP2HS_TIME_Msk                 /*!< Low-Power to High-Speed Time */
#define DSI_CLTCR_LP2HS_TIME0_Pos     (0U)
#define DSI_CLTCR_LP2HS_TIME0_Msk     (0x1UL << DSI_CLTCR_LP2HS_TIME0_Pos)     /*!< 0x00000001 */
#define DSI_CLTCR_LP2HS_TIME0         DSI_CLTCR_LP2HS_TIME0_Msk
#define DSI_CLTCR_LP2HS_TIME1_Pos     (1U)
#define DSI_CLTCR_LP2HS_TIME1_Msk     (0x1UL << DSI_CLTCR_LP2HS_TIME1_Pos)     /*!< 0x00000002 */
#define DSI_CLTCR_LP2HS_TIME1         DSI_CLTCR_LP2HS_TIME1_Msk
#define DSI_CLTCR_LP2HS_TIME2_Pos     (2U)
#define DSI_CLTCR_LP2HS_TIME2_Msk     (0x1UL << DSI_CLTCR_LP2HS_TIME2_Pos)     /*!< 0x00000004 */
#define DSI_CLTCR_LP2HS_TIME2         DSI_CLTCR_LP2HS_TIME2_Msk
#define DSI_CLTCR_LP2HS_TIME3_Pos     (3U)
#define DSI_CLTCR_LP2HS_TIME3_Msk     (0x1UL << DSI_CLTCR_LP2HS_TIME3_Pos)     /*!< 0x00000008 */
#define DSI_CLTCR_LP2HS_TIME3         DSI_CLTCR_LP2HS_TIME3_Msk
#define DSI_CLTCR_LP2HS_TIME4_Pos     (4U)
#define DSI_CLTCR_LP2HS_TIME4_Msk     (0x1UL << DSI_CLTCR_LP2HS_TIME4_Pos)     /*!< 0x00000010 */
#define DSI_CLTCR_LP2HS_TIME4         DSI_CLTCR_LP2HS_TIME4_Msk
#define DSI_CLTCR_LP2HS_TIME5_Pos     (5U)
#define DSI_CLTCR_LP2HS_TIME5_Msk     (0x1UL << DSI_CLTCR_LP2HS_TIME5_Pos)     /*!< 0x00000020 */
#define DSI_CLTCR_LP2HS_TIME5         DSI_CLTCR_LP2HS_TIME5_Msk
#define DSI_CLTCR_LP2HS_TIME6_Pos     (6U)
#define DSI_CLTCR_LP2HS_TIME6_Msk     (0x1UL << DSI_CLTCR_LP2HS_TIME6_Pos)     /*!< 0x00000040 */
#define DSI_CLTCR_LP2HS_TIME6         DSI_CLTCR_LP2HS_TIME6_Msk
#define DSI_CLTCR_LP2HS_TIME7_Pos     (7U)
#define DSI_CLTCR_LP2HS_TIME7_Msk     (0x1UL << DSI_CLTCR_LP2HS_TIME7_Pos)     /*!< 0x00000080 */
#define DSI_CLTCR_LP2HS_TIME7         DSI_CLTCR_LP2HS_TIME7_Msk
#define DSI_CLTCR_LP2HS_TIME8_Pos     (8U)
#define DSI_CLTCR_LP2HS_TIME8_Msk     (0x1UL << DSI_CLTCR_LP2HS_TIME8_Pos)     /*!< 0x00000100 */
#define DSI_CLTCR_LP2HS_TIME8         DSI_CLTCR_LP2HS_TIME8_Msk
#define DSI_CLTCR_LP2HS_TIME9_Pos     (9U)
#define DSI_CLTCR_LP2HS_TIME9_Msk     (0x1UL << DSI_CLTCR_LP2HS_TIME9_Pos)     /*!< 0x00000200 */
#define DSI_CLTCR_LP2HS_TIME9         DSI_CLTCR_LP2HS_TIME9_Msk

#define DSI_CLTCR_HS2LP_TIME_Pos      (16U)
#define DSI_CLTCR_HS2LP_TIME_Msk      (0x3FFUL << DSI_CLTCR_HS2LP_TIME_Pos)    /*!< 0x03FF0000 */
#define DSI_CLTCR_HS2LP_TIME          DSI_CLTCR_HS2LP_TIME_Msk                 /*!< High-Speed to Low-Power Time */
#define DSI_CLTCR_HS2LP_TIME0_Pos     (16U)
#define DSI_CLTCR_HS2LP_TIME0_Msk     (0x1UL << DSI_CLTCR_HS2LP_TIME0_Pos)     /*!< 0x00010000 */
#define DSI_CLTCR_HS2LP_TIME0         DSI_CLTCR_HS2LP_TIME0_Msk
#define DSI_CLTCR_HS2LP_TIME1_Pos     (17U)
#define DSI_CLTCR_HS2LP_TIME1_Msk     (0x1UL << DSI_CLTCR_HS2LP_TIME1_Pos)     /*!< 0x00020000 */
#define DSI_CLTCR_HS2LP_TIME1         DSI_CLTCR_HS2LP_TIME1_Msk
#define DSI_CLTCR_HS2LP_TIME2_Pos     (18U)
#define DSI_CLTCR_HS2LP_TIME2_Msk     (0x1UL << DSI_CLTCR_HS2LP_TIME2_Pos)     /*!< 0x00040000 */
#define DSI_CLTCR_HS2LP_TIME2         DSI_CLTCR_HS2LP_TIME2_Msk
#define DSI_CLTCR_HS2LP_TIME3_Pos     (19U)
#define DSI_CLTCR_HS2LP_TIME3_Msk     (0x1UL << DSI_CLTCR_HS2LP_TIME3_Pos)     /*!< 0x00080000 */
#define DSI_CLTCR_HS2LP_TIME3         DSI_CLTCR_HS2LP_TIME3_Msk
#define DSI_CLTCR_HS2LP_TIME4_Pos     (20U)
#define DSI_CLTCR_HS2LP_TIME4_Msk     (0x1UL << DSI_CLTCR_HS2LP_TIME4_Pos)     /*!< 0x00100000 */
#define DSI_CLTCR_HS2LP_TIME4         DSI_CLTCR_HS2LP_TIME4_Msk
#define DSI_CLTCR_HS2LP_TIME5_Pos     (21U)
#define DSI_CLTCR_HS2LP_TIME5_Msk     (0x1UL << DSI_CLTCR_HS2LP_TIME5_Pos)     /*!< 0x00200000 */
#define DSI_CLTCR_HS2LP_TIME5         DSI_CLTCR_HS2LP_TIME5_Msk
#define DSI_CLTCR_HS2LP_TIME6_Pos     (22U)
#define DSI_CLTCR_HS2LP_TIME6_Msk     (0x1UL << DSI_CLTCR_HS2LP_TIME6_Pos)     /*!< 0x00400000 */
#define DSI_CLTCR_HS2LP_TIME6         DSI_CLTCR_HS2LP_TIME6_Msk
#define DSI_CLTCR_HS2LP_TIME7_Pos     (23U)
#define DSI_CLTCR_HS2LP_TIME7_Msk     (0x1UL << DSI_CLTCR_HS2LP_TIME7_Pos)     /*!< 0x00800000 */
#define DSI_CLTCR_HS2LP_TIME7         DSI_CLTCR_HS2LP_TIME7_Msk
#define DSI_CLTCR_HS2LP_TIME8_Pos     (24U)
#define DSI_CLTCR_HS2LP_TIME8_Msk     (0x1UL << DSI_CLTCR_HS2LP_TIME8_Pos)     /*!< 0x01000000 */
#define DSI_CLTCR_HS2LP_TIME8         DSI_CLTCR_HS2LP_TIME8_Msk
#define DSI_CLTCR_HS2LP_TIME9_Pos     (25U)
#define DSI_CLTCR_HS2LP_TIME9_Msk     (0x1UL << DSI_CLTCR_HS2LP_TIME9_Pos)     /*!< 0x02000000 */
#define DSI_CLTCR_HS2LP_TIME9         DSI_CLTCR_HS2LP_TIME9_Msk

/*******************  Bit definition for DSI_DLTCR register  **************/
#define DSI_DLTCR_MRD_TIME_Pos        (0U)
#define DSI_DLTCR_MRD_TIME_Msk        (0x7FFFUL << DSI_DLTCR_MRD_TIME_Pos)     /*!< 0x00007FFF */
#define DSI_DLTCR_MRD_TIME            DSI_DLTCR_MRD_TIME_Msk                   /*!< Maximum Read Time */
#define DSI_DLTCR_MRD_TIME0_Pos       (0U)
#define DSI_DLTCR_MRD_TIME0_Msk       (0x1UL << DSI_DLTCR_MRD_TIME0_Pos)       /*!< 0x00000001 */
#define DSI_DLTCR_MRD_TIME0           DSI_DLTCR_MRD_TIME0_Msk
#define DSI_DLTCR_MRD_TIME1_Pos       (1U)
#define DSI_DLTCR_MRD_TIME1_Msk       (0x1UL << DSI_DLTCR_MRD_TIME1_Pos)       /*!< 0x00000002 */
#define DSI_DLTCR_MRD_TIME1           DSI_DLTCR_MRD_TIME1_Msk
#define DSI_DLTCR_MRD_TIME2_Pos       (2U)
#define DSI_DLTCR_MRD_TIME2_Msk       (0x1UL << DSI_DLTCR_MRD_TIME2_Pos)       /*!< 0x00000004 */
#define DSI_DLTCR_MRD_TIME2           DSI_DLTCR_MRD_TIME2_Msk
#define DSI_DLTCR_MRD_TIME3_Pos       (3U)
#define DSI_DLTCR_MRD_TIME3_Msk       (0x1UL << DSI_DLTCR_MRD_TIME3_Pos)       /*!< 0x00000008 */
#define DSI_DLTCR_MRD_TIME3           DSI_DLTCR_MRD_TIME3_Msk
#define DSI_DLTCR_MRD_TIME4_Pos       (4U)
#define DSI_DLTCR_MRD_TIME4_Msk       (0x1UL << DSI_DLTCR_MRD_TIME4_Pos)       /*!< 0x00000010 */
#define DSI_DLTCR_MRD_TIME4           DSI_DLTCR_MRD_TIME4_Msk
#define DSI_DLTCR_MRD_TIME5_Pos       (5U)
#define DSI_DLTCR_MRD_TIME5_Msk       (0x1UL << DSI_DLTCR_MRD_TIME5_Pos)       /*!< 0x00000020 */
#define DSI_DLTCR_MRD_TIME5           DSI_DLTCR_MRD_TIME5_Msk
#define DSI_DLTCR_MRD_TIME6_Pos       (6U)
#define DSI_DLTCR_MRD_TIME6_Msk       (0x1UL << DSI_DLTCR_MRD_TIME6_Pos)       /*!< 0x00000040 */
#define DSI_DLTCR_MRD_TIME6           DSI_DLTCR_MRD_TIME6_Msk
#define DSI_DLTCR_MRD_TIME7_Pos       (7U)
#define DSI_DLTCR_MRD_TIME7_Msk       (0x1UL << DSI_DLTCR_MRD_TIME7_Pos)       /*!< 0x00000080 */
#define DSI_DLTCR_MRD_TIME7           DSI_DLTCR_MRD_TIME7_Msk
#define DSI_DLTCR_MRD_TIME8_Pos       (8U)
#define DSI_DLTCR_MRD_TIME8_Msk       (0x1UL << DSI_DLTCR_MRD_TIME8_Pos)       /*!< 0x00000100 */
#define DSI_DLTCR_MRD_TIME8           DSI_DLTCR_MRD_TIME8_Msk
#define DSI_DLTCR_MRD_TIME9_Pos       (9U)
#define DSI_DLTCR_MRD_TIME9_Msk       (0x1UL << DSI_DLTCR_MRD_TIME9_Pos)       /*!< 0x00000200 */
#define DSI_DLTCR_MRD_TIME9           DSI_DLTCR_MRD_TIME9_Msk
#define DSI_DLTCR_MRD_TIME10_Pos      (10U)
#define DSI_DLTCR_MRD_TIME10_Msk      (0x1UL << DSI_DLTCR_MRD_TIME10_Pos)      /*!< 0x00000400 */
#define DSI_DLTCR_MRD_TIME10          DSI_DLTCR_MRD_TIME10_Msk
#define DSI_DLTCR_MRD_TIME11_Pos      (11U)
#define DSI_DLTCR_MRD_TIME11_Msk      (0x1UL << DSI_DLTCR_MRD_TIME11_Pos)      /*!< 0x00000800 */
#define DSI_DLTCR_MRD_TIME11          DSI_DLTCR_MRD_TIME11_Msk
#define DSI_DLTCR_MRD_TIME12_Pos      (12U)
#define DSI_DLTCR_MRD_TIME12_Msk      (0x1UL << DSI_DLTCR_MRD_TIME12_Pos)      /*!< 0x00001000 */
#define DSI_DLTCR_MRD_TIME12          DSI_DLTCR_MRD_TIME12_Msk
#define DSI_DLTCR_MRD_TIME13_Pos      (13U)
#define DSI_DLTCR_MRD_TIME13_Msk      (0x1UL << DSI_DLTCR_MRD_TIME13_Pos)      /*!< 0x00002000 */
#define DSI_DLTCR_MRD_TIME13          DSI_DLTCR_MRD_TIME13_Msk
#define DSI_DLTCR_MRD_TIME14_Pos      (14U)
#define DSI_DLTCR_MRD_TIME14_Msk      (0x1UL << DSI_DLTCR_MRD_TIME14_Pos)      /*!< 0x00004000 */
#define DSI_DLTCR_MRD_TIME14          DSI_DLTCR_MRD_TIME14_Msk

#define DSI_DLTCR_LP2HS_TIME_Pos      (16U)
#define DSI_DLTCR_LP2HS_TIME_Msk      (0xFFUL << DSI_DLTCR_LP2HS_TIME_Pos)     /*!< 0x00FF0000 */
#define DSI_DLTCR_LP2HS_TIME          DSI_DLTCR_LP2HS_TIME_Msk                 /*!< Low-Power To High-Speed Time */
#define DSI_DLTCR_LP2HS_TIME0_Pos     (16U)
#define DSI_DLTCR_LP2HS_TIME0_Msk     (0x1UL << DSI_DLTCR_LP2HS_TIME0_Pos)     /*!< 0x00010000 */
#define DSI_DLTCR_LP2HS_TIME0         DSI_DLTCR_LP2HS_TIME0_Msk
#define DSI_DLTCR_LP2HS_TIME1_Pos     (17U)
#define DSI_DLTCR_LP2HS_TIME1_Msk     (0x1UL << DSI_DLTCR_LP2HS_TIME1_Pos)     /*!< 0x00020000 */
#define DSI_DLTCR_LP2HS_TIME1         DSI_DLTCR_LP2HS_TIME1_Msk
#define DSI_DLTCR_LP2HS_TIME2_Pos     (18U)
#define DSI_DLTCR_LP2HS_TIME2_Msk     (0x1UL << DSI_DLTCR_LP2HS_TIME2_Pos)     /*!< 0x00040000 */
#define DSI_DLTCR_LP2HS_TIME2         DSI_DLTCR_LP2HS_TIME2_Msk
#define DSI_DLTCR_LP2HS_TIME3_Pos     (19U)
#define DSI_DLTCR_LP2HS_TIME3_Msk     (0x1UL << DSI_DLTCR_LP2HS_TIME3_Pos)     /*!< 0x00080000 */
#define DSI_DLTCR_LP2HS_TIME3         DSI_DLTCR_LP2HS_TIME3_Msk
#define DSI_DLTCR_LP2HS_TIME4_Pos     (20U)
#define DSI_DLTCR_LP2HS_TIME4_Msk     (0x1UL << DSI_DLTCR_LP2HS_TIME4_Pos)     /*!< 0x00100000 */
#define DSI_DLTCR_LP2HS_TIME4         DSI_DLTCR_LP2HS_TIME4_Msk
#define DSI_DLTCR_LP2HS_TIME5_Pos     (21U)
#define DSI_DLTCR_LP2HS_TIME5_Msk     (0x1UL << DSI_DLTCR_LP2HS_TIME5_Pos)     /*!< 0x00200000 */
#define DSI_DLTCR_LP2HS_TIME5         DSI_DLTCR_LP2HS_TIME5_Msk
#define DSI_DLTCR_LP2HS_TIME6_Pos     (22U)
#define DSI_DLTCR_LP2HS_TIME6_Msk     (0x1UL << DSI_DLTCR_LP2HS_TIME6_Pos)     /*!< 0x00400000 */
#define DSI_DLTCR_LP2HS_TIME6         DSI_DLTCR_LP2HS_TIME6_Msk
#define DSI_DLTCR_LP2HS_TIME7_Pos     (23U)
#define DSI_DLTCR_LP2HS_TIME7_Msk     (0x1UL << DSI_DLTCR_LP2HS_TIME7_Pos)     /*!< 0x00800000 */
#define DSI_DLTCR_LP2HS_TIME7         DSI_DLTCR_LP2HS_TIME7_Msk

#define DSI_DLTCR_HS2LP_TIME_Pos      (24U)
#define DSI_DLTCR_HS2LP_TIME_Msk      (0xFFUL << DSI_DLTCR_HS2LP_TIME_Pos)     /*!< 0xFF000000 */
#define DSI_DLTCR_HS2LP_TIME          DSI_DLTCR_HS2LP_TIME_Msk                 /*!< High-Speed To Low-Power Time */
#define DSI_DLTCR_HS2LP_TIME0_Pos     (24U)
#define DSI_DLTCR_HS2LP_TIME0_Msk     (0x1UL << DSI_DLTCR_HS2LP_TIME0_Pos)     /*!< 0x01000000 */
#define DSI_DLTCR_HS2LP_TIME0         DSI_DLTCR_HS2LP_TIME0_Msk
#define DSI_DLTCR_HS2LP_TIME1_Pos     (25U)
#define DSI_DLTCR_HS2LP_TIME1_Msk     (0x1UL << DSI_DLTCR_HS2LP_TIME1_Pos)     /*!< 0x02000000 */
#define DSI_DLTCR_HS2LP_TIME1         DSI_DLTCR_HS2LP_TIME1_Msk
#define DSI_DLTCR_HS2LP_TIME2_Pos     (26U)
#define DSI_DLTCR_HS2LP_TIME2_Msk     (0x1UL << DSI_DLTCR_HS2LP_TIME2_Pos)     /*!< 0x04000000 */
#define DSI_DLTCR_HS2LP_TIME2         DSI_DLTCR_HS2LP_TIME2_Msk
#define DSI_DLTCR_HS2LP_TIME3_Pos     (27U)
#define DSI_DLTCR_HS2LP_TIME3_Msk     (0x1UL << DSI_DLTCR_HS2LP_TIME3_Pos)     /*!< 0x08000000 */
#define DSI_DLTCR_HS2LP_TIME3         DSI_DLTCR_HS2LP_TIME3_Msk
#define DSI_DLTCR_HS2LP_TIME4_Pos     (28U)
#define DSI_DLTCR_HS2LP_TIME4_Msk     (0x1UL << DSI_DLTCR_HS2LP_TIME4_Pos)     /*!< 0x10000000 */
#define DSI_DLTCR_HS2LP_TIME4         DSI_DLTCR_HS2LP_TIME4_Msk
#define DSI_DLTCR_HS2LP_TIME5_Pos     (29U)
#define DSI_DLTCR_HS2LP_TIME5_Msk     (0x1UL << DSI_DLTCR_HS2LP_TIME5_Pos)     /*!< 0x20000000 */
#define DSI_DLTCR_HS2LP_TIME5         DSI_DLTCR_HS2LP_TIME5_Msk
#define DSI_DLTCR_HS2LP_TIME6_Pos     (30U)
#define DSI_DLTCR_HS2LP_TIME6_Msk     (0x1UL << DSI_DLTCR_HS2LP_TIME6_Pos)     /*!< 0x40000000 */
#define DSI_DLTCR_HS2LP_TIME6         DSI_DLTCR_HS2LP_TIME6_Msk
#define DSI_DLTCR_HS2LP_TIME7_Pos     (31U)
#define DSI_DLTCR_HS2LP_TIME7_Msk     (0x1UL << DSI_DLTCR_HS2LP_TIME7_Pos)     /*!< 0x80000000 */
#define DSI_DLTCR_HS2LP_TIME7         DSI_DLTCR_HS2LP_TIME7_Msk

/*******************  Bit definition for DSI_PCTLR register  **************/
#define DSI_PCTLR_DEN_Pos             (1U)
#define DSI_PCTLR_DEN_Msk             (0x1UL << DSI_PCTLR_DEN_Pos)             /*!< 0x00000002 */
#define DSI_PCTLR_DEN                 DSI_PCTLR_DEN_Msk                        /*!< Digital Enable */
#define DSI_PCTLR_CKE_Pos             (2U)
#define DSI_PCTLR_CKE_Msk             (0x1UL << DSI_PCTLR_CKE_Pos)             /*!< 0x00000004 */
#define DSI_PCTLR_CKE                 DSI_PCTLR_CKE_Msk                        /*!< Clock Enable */

/*******************  Bit definition for DSI_PCONFR register  *************/
#define DSI_PCONFR_NL_Pos             (0U)
#define DSI_PCONFR_NL_Msk             (0x3UL << DSI_PCONFR_NL_Pos)             /*!< 0x00000003 */
#define DSI_PCONFR_NL                 DSI_PCONFR_NL_Msk                        /*!< Number of Lanes */
#define DSI_PCONFR_NL0_Pos            (0U)
#define DSI_PCONFR_NL0_Msk            (0x1UL << DSI_PCONFR_NL0_Pos)            /*!< 0x00000001 */
#define DSI_PCONFR_NL0                DSI_PCONFR_NL0_Msk
#define DSI_PCONFR_NL1_Pos            (1U)
#define DSI_PCONFR_NL1_Msk            (0x1UL << DSI_PCONFR_NL1_Pos)            /*!< 0x00000002 */
#define DSI_PCONFR_NL1                DSI_PCONFR_NL1_Msk

#define DSI_PCONFR_SW_TIME_Pos        (8U)
#define DSI_PCONFR_SW_TIME_Msk        (0xFFUL << DSI_PCONFR_SW_TIME_Pos)       /*!< 0x0000FF00 */
#define DSI_PCONFR_SW_TIME            DSI_PCONFR_SW_TIME_Msk                   /*!< Stop Wait Time */
#define DSI_PCONFR_SW_TIME0_Pos       (8U)
#define DSI_PCONFR_SW_TIME0_Msk       (0x1UL << DSI_PCONFR_SW_TIME0_Pos)       /*!< 0x00000100 */
#define DSI_PCONFR_SW_TIME0           DSI_PCONFR_SW_TIME0_Msk
#define DSI_PCONFR_SW_TIME1_Pos       (9U)
#define DSI_PCONFR_SW_TIME1_Msk       (0x1UL << DSI_PCONFR_SW_TIME1_Pos)       /*!< 0x00000200 */
#define DSI_PCONFR_SW_TIME1           DSI_PCONFR_SW_TIME1_Msk
#define DSI_PCONFR_SW_TIME2_Pos       (10U)
#define DSI_PCONFR_SW_TIME2_Msk       (0x1UL << DSI_PCONFR_SW_TIME2_Pos)       /*!< 0x00000400 */
#define DSI_PCONFR_SW_TIME2           DSI_PCONFR_SW_TIME2_Msk
#define DSI_PCONFR_SW_TIME3_Pos       (11U)
#define DSI_PCONFR_SW_TIME3_Msk       (0x1UL << DSI_PCONFR_SW_TIME3_Pos)       /*!< 0x00000800 */
#define DSI_PCONFR_SW_TIME3           DSI_PCONFR_SW_TIME3_Msk
#define DSI_PCONFR_SW_TIME4_Pos       (12U)
#define DSI_PCONFR_SW_TIME4_Msk       (0x1UL << DSI_PCONFR_SW_TIME4_Pos)       /*!< 0x00001000 */
#define DSI_PCONFR_SW_TIME4           DSI_PCONFR_SW_TIME4_Msk
#define DSI_PCONFR_SW_TIME5_Pos       (13U)
#define DSI_PCONFR_SW_TIME5_Msk       (0x1UL << DSI_PCONFR_SW_TIME5_Pos)       /*!< 0x00002000 */
#define DSI_PCONFR_SW_TIME5           DSI_PCONFR_SW_TIME5_Msk
#define DSI_PCONFR_SW_TIME6_Pos       (14U)
#define DSI_PCONFR_SW_TIME6_Msk       (0x1UL << DSI_PCONFR_SW_TIME6_Pos)       /*!< 0x00004000 */
#define DSI_PCONFR_SW_TIME6           DSI_PCONFR_SW_TIME6_Msk
#define DSI_PCONFR_SW_TIME7_Pos       (15U)
#define DSI_PCONFR_SW_TIME7_Msk       (0x1UL << DSI_PCONFR_SW_TIME7_Pos)       /*!< 0x00008000 */
#define DSI_PCONFR_SW_TIME7           DSI_PCONFR_SW_TIME7_Msk

/*******************  Bit definition for DSI_PUCR register  ***************/
#define DSI_PUCR_URCL_Pos             (0U)
#define DSI_PUCR_URCL_Msk             (0x1UL << DSI_PUCR_URCL_Pos)             /*!< 0x00000001 */
#define DSI_PUCR_URCL                 DSI_PUCR_URCL_Msk                        /*!< ULPS Request on Clock Lane */
#define DSI_PUCR_UECL_Pos             (1U)
#define DSI_PUCR_UECL_Msk             (0x1UL << DSI_PUCR_UECL_Pos)             /*!< 0x00000002 */
#define DSI_PUCR_UECL                 DSI_PUCR_UECL_Msk                        /*!< ULPS Exit on Clock Lane */
#define DSI_PUCR_URDL_Pos             (2U)
#define DSI_PUCR_URDL_Msk             (0x1UL << DSI_PUCR_URDL_Pos)             /*!< 0x00000004 */
#define DSI_PUCR_URDL                 DSI_PUCR_URDL_Msk                        /*!< ULPS Request on Data Lane */
#define DSI_PUCR_UEDL_Pos             (3U)
#define DSI_PUCR_UEDL_Msk             (0x1UL << DSI_PUCR_UEDL_Pos)             /*!< 0x00000008 */
#define DSI_PUCR_UEDL                 DSI_PUCR_UEDL_Msk                        /*!< ULPS Exit on Data Lane */

/*******************  Bit definition for DSI_PTTCR register  **************/
#define DSI_PTTCR_TX_TRIG_Pos         (0U)
#define DSI_PTTCR_TX_TRIG_Msk         (0xFUL << DSI_PTTCR_TX_TRIG_Pos)         /*!< 0x0000000F */
#define DSI_PTTCR_TX_TRIG             DSI_PTTCR_TX_TRIG_Msk                    /*!< Transmission Trigger */
#define DSI_PTTCR_TX_TRIG0_Pos        (0U)
#define DSI_PTTCR_TX_TRIG0_Msk        (0x1UL << DSI_PTTCR_TX_TRIG0_Pos)        /*!< 0x00000001 */
#define DSI_PTTCR_TX_TRIG0            DSI_PTTCR_TX_TRIG0_Msk
#define DSI_PTTCR_TX_TRIG1_Pos        (1U)
#define DSI_PTTCR_TX_TRIG1_Msk        (0x1UL << DSI_PTTCR_TX_TRIG1_Pos)        /*!< 0x00000002 */
#define DSI_PTTCR_TX_TRIG1            DSI_PTTCR_TX_TRIG1_Msk
#define DSI_PTTCR_TX_TRIG2_Pos        (2U)
#define DSI_PTTCR_TX_TRIG2_Msk        (0x1UL << DSI_PTTCR_TX_TRIG2_Pos)        /*!< 0x00000004 */
#define DSI_PTTCR_TX_TRIG2            DSI_PTTCR_TX_TRIG2_Msk
#define DSI_PTTCR_TX_TRIG3_Pos        (3U)
#define DSI_PTTCR_TX_TRIG3_Msk        (0x1UL << DSI_PTTCR_TX_TRIG3_Pos)        /*!< 0x00000008 */
#define DSI_PTTCR_TX_TRIG3            DSI_PTTCR_TX_TRIG3_Msk

/*******************  Bit definition for DSI_PSR register  ****************/
#define DSI_PSR_PD_Pos                (1U)
#define DSI_PSR_PD_Msk                (0x1UL << DSI_PSR_PD_Pos)                /*!< 0x00000002 */
#define DSI_PSR_PD                    DSI_PSR_PD_Msk                           /*!< PHY Direction */
#define DSI_PSR_PSSC_Pos              (2U)
#define DSI_PSR_PSSC_Msk              (0x1UL << DSI_PSR_PSSC_Pos)              /*!< 0x00000004 */
#define DSI_PSR_PSSC                  DSI_PSR_PSSC_Msk                         /*!< PHY Stop State Clock lane */
#define DSI_PSR_UANC_Pos              (3U)
#define DSI_PSR_UANC_Msk              (0x1UL << DSI_PSR_UANC_Pos)              /*!< 0x00000008 */
#define DSI_PSR_UANC                  DSI_PSR_UANC_Msk                         /*!< ULPS Active Not Clock lane */
#define DSI_PSR_PSS0_Pos              (4U)
#define DSI_PSR_PSS0_Msk              (0x1UL << DSI_PSR_PSS0_Pos)              /*!< 0x00000010 */
#define DSI_PSR_PSS0                  DSI_PSR_PSS0_Msk                         /*!< PHY Stop State lane 0 */
#define DSI_PSR_UAN0_Pos              (5U)
#define DSI_PSR_UAN0_Msk              (0x1UL << DSI_PSR_UAN0_Pos)              /*!< 0x00000020 */
#define DSI_PSR_UAN0                  DSI_PSR_UAN0_Msk                         /*!< ULPS Active Not lane 0 */
#define DSI_PSR_RUE0_Pos              (6U)
#define DSI_PSR_RUE0_Msk              (0x1UL << DSI_PSR_RUE0_Pos)              /*!< 0x00000040 */
#define DSI_PSR_RUE0                  DSI_PSR_RUE0_Msk                         /*!< RX ULPS Escape lane 0 */
#define DSI_PSR_PSS1_Pos              (7U)
#define DSI_PSR_PSS1_Msk              (0x1UL << DSI_PSR_PSS1_Pos)              /*!< 0x00000080 */
#define DSI_PSR_PSS1                  DSI_PSR_PSS1_Msk                         /*!< PHY Stop State lane 1 */
#define DSI_PSR_UAN1_Pos              (8U)
#define DSI_PSR_UAN1_Msk              (0x1UL << DSI_PSR_UAN1_Pos)              /*!< 0x00000100 */
#define DSI_PSR_UAN1                  DSI_PSR_UAN1_Msk                         /*!< ULPS Active Not lane 1 */

/*******************  Bit definition for DSI_ISR0 register  ***************/
#define DSI_ISR0_AE0_Pos              (0U)
#define DSI_ISR0_AE0_Msk              (0x1UL << DSI_ISR0_AE0_Pos)              /*!< 0x00000001 */
#define DSI_ISR0_AE0                  DSI_ISR0_AE0_Msk                         /*!< Acknowledge Error 0 */
#define DSI_ISR0_AE1_Pos              (1U)
#define DSI_ISR0_AE1_Msk              (0x1UL << DSI_ISR0_AE1_Pos)              /*!< 0x00000002 */
#define DSI_ISR0_AE1                  DSI_ISR0_AE1_Msk                         /*!< Acknowledge Error 1 */
#define DSI_ISR0_AE2_Pos              (2U)
#define DSI_ISR0_AE2_Msk              (0x1UL << DSI_ISR0_AE2_Pos)              /*!< 0x00000004 */
#define DSI_ISR0_AE2                  DSI_ISR0_AE2_Msk                         /*!< Acknowledge Error 2 */
#define DSI_ISR0_AE3_Pos              (3U)
#define DSI_ISR0_AE3_Msk              (0x1UL << DSI_ISR0_AE3_Pos)              /*!< 0x00000008 */
#define DSI_ISR0_AE3                  DSI_ISR0_AE3_Msk                         /*!< Acknowledge Error 3 */
#define DSI_ISR0_AE4_Pos              (4U)
#define DSI_ISR0_AE4_Msk              (0x1UL << DSI_ISR0_AE4_Pos)              /*!< 0x00000010 */
#define DSI_ISR0_AE4                  DSI_ISR0_AE4_Msk                         /*!< Acknowledge Error 4 */
#define DSI_ISR0_AE5_Pos              (5U)
#define DSI_ISR0_AE5_Msk              (0x1UL << DSI_ISR0_AE5_Pos)              /*!< 0x00000020 */
#define DSI_ISR0_AE5                  DSI_ISR0_AE5_Msk                         /*!< Acknowledge Error 5 */
#define DSI_ISR0_AE6_Pos              (6U)
#define DSI_ISR0_AE6_Msk              (0x1UL << DSI_ISR0_AE6_Pos)              /*!< 0x00000040 */
#define DSI_ISR0_AE6                  DSI_ISR0_AE6_Msk                         /*!< Acknowledge Error 6 */
#define DSI_ISR0_AE7_Pos              (7U)
#define DSI_ISR0_AE7_Msk              (0x1UL << DSI_ISR0_AE7_Pos)              /*!< 0x00000080 */
#define DSI_ISR0_AE7                  DSI_ISR0_AE7_Msk                         /*!< Acknowledge Error 7 */
#define DSI_ISR0_AE8_Pos              (8U)
#define DSI_ISR0_AE8_Msk              (0x1UL << DSI_ISR0_AE8_Pos)              /*!< 0x00000100 */
#define DSI_ISR0_AE8                  DSI_ISR0_AE8_Msk                         /*!< Acknowledge Error 8 */
#define DSI_ISR0_AE9_Pos              (9U)
#define DSI_ISR0_AE9_Msk              (0x1UL << DSI_ISR0_AE9_Pos)              /*!< 0x00000200 */
#define DSI_ISR0_AE9                  DSI_ISR0_AE9_Msk                         /*!< Acknowledge Error 9 */
#define DSI_ISR0_AE10_Pos             (10U)
#define DSI_ISR0_AE10_Msk             (0x1UL << DSI_ISR0_AE10_Pos)             /*!< 0x00000400 */
#define DSI_ISR0_AE10                 DSI_ISR0_AE10_Msk                        /*!< Acknowledge Error 10 */
#define DSI_ISR0_AE11_Pos             (11U)
#define DSI_ISR0_AE11_Msk             (0x1UL << DSI_ISR0_AE11_Pos)             /*!< 0x00000800 */
#define DSI_ISR0_AE11                 DSI_ISR0_AE11_Msk                        /*!< Acknowledge Error 11 */
#define DSI_ISR0_AE12_Pos             (12U)
#define DSI_ISR0_AE12_Msk             (0x1UL << DSI_ISR0_AE12_Pos)             /*!< 0x00001000 */
#define DSI_ISR0_AE12                 DSI_ISR0_AE12_Msk                        /*!< Acknowledge Error 12 */
#define DSI_ISR0_AE13_Pos             (13U)
#define DSI_ISR0_AE13_Msk             (0x1UL << DSI_ISR0_AE13_Pos)             /*!< 0x00002000 */
#define DSI_ISR0_AE13                 DSI_ISR0_AE13_Msk                        /*!< Acknowledge Error 13 */
#define DSI_ISR0_AE14_Pos             (14U)
#define DSI_ISR0_AE14_Msk             (0x1UL << DSI_ISR0_AE14_Pos)             /*!< 0x00004000 */
#define DSI_ISR0_AE14                 DSI_ISR0_AE14_Msk                        /*!< Acknowledge Error 14 */
#define DSI_ISR0_AE15_Pos             (15U)
#define DSI_ISR0_AE15_Msk             (0x1UL << DSI_ISR0_AE15_Pos)             /*!< 0x00008000 */
#define DSI_ISR0_AE15                 DSI_ISR0_AE15_Msk                        /*!< Acknowledge Error 15 */
#define DSI_ISR0_PE0_Pos              (16U)
#define DSI_ISR0_PE0_Msk              (0x1UL << DSI_ISR0_PE0_Pos)              /*!< 0x00010000 */
#define DSI_ISR0_PE0                  DSI_ISR0_PE0_Msk                         /*!< PHY Error 0 */
#define DSI_ISR0_PE1_Pos              (17U)
#define DSI_ISR0_PE1_Msk              (0x1UL << DSI_ISR0_PE1_Pos)              /*!< 0x00020000 */
#define DSI_ISR0_PE1                  DSI_ISR0_PE1_Msk                         /*!< PHY Error 1 */
#define DSI_ISR0_PE2_Pos              (18U)
#define DSI_ISR0_PE2_Msk              (0x1UL << DSI_ISR0_PE2_Pos)              /*!< 0x00040000 */
#define DSI_ISR0_PE2                  DSI_ISR0_PE2_Msk                         /*!< PHY Error 2 */
#define DSI_ISR0_PE3_Pos              (19U)
#define DSI_ISR0_PE3_Msk              (0x1UL << DSI_ISR0_PE3_Pos)              /*!< 0x00080000 */
#define DSI_ISR0_PE3                  DSI_ISR0_PE3_Msk                         /*!< PHY Error 3 */
#define DSI_ISR0_PE4_Pos              (20U)
#define DSI_ISR0_PE4_Msk              (0x1UL << DSI_ISR0_PE4_Pos)              /*!< 0x00100000 */
#define DSI_ISR0_PE4                  DSI_ISR0_PE4_Msk                         /*!< PHY Error 4 */

/*******************  Bit definition for DSI_ISR1 register  ***************/
#define DSI_ISR1_TOHSTX_Pos           (0U)
#define DSI_ISR1_TOHSTX_Msk           (0x1UL << DSI_ISR1_TOHSTX_Pos)           /*!< 0x00000001 */
#define DSI_ISR1_TOHSTX               DSI_ISR1_TOHSTX_Msk                      /*!< Timeout High-Speed Transmission */
#define DSI_ISR1_TOLPRX_Pos           (1U)
#define DSI_ISR1_TOLPRX_Msk           (0x1UL << DSI_ISR1_TOLPRX_Pos)           /*!< 0x00000002 */
#define DSI_ISR1_TOLPRX               DSI_ISR1_TOLPRX_Msk                      /*!< Timeout Low-Power Reception */
#define DSI_ISR1_ECCSE_Pos            (2U)
#define DSI_ISR1_ECCSE_Msk            (0x1UL << DSI_ISR1_ECCSE_Pos)            /*!< 0x00000004 */
#define DSI_ISR1_ECCSE                DSI_ISR1_ECCSE_Msk                       /*!< ECC Single-bit Error */
#define DSI_ISR1_ECCME_Pos            (3U)
#define DSI_ISR1_ECCME_Msk            (0x1UL << DSI_ISR1_ECCME_Pos)            /*!< 0x00000008 */
#define DSI_ISR1_ECCME                DSI_ISR1_ECCME_Msk                       /*!< ECC Multi-bit Error */
#define DSI_ISR1_CRCE_Pos             (4U)
#define DSI_ISR1_CRCE_Msk             (0x1UL << DSI_ISR1_CRCE_Pos)             /*!< 0x00000010 */
#define DSI_ISR1_CRCE                 DSI_ISR1_CRCE_Msk                        /*!< CRC Error */
#define DSI_ISR1_PSE_Pos              (5U)
#define DSI_ISR1_PSE_Msk              (0x1UL << DSI_ISR1_PSE_Pos)              /*!< 0x00000020 */
#define DSI_ISR1_PSE                  DSI_ISR1_PSE_Msk                         /*!< Packet Size Error */
#define DSI_ISR1_EOTPE_Pos            (6U)
#define DSI_ISR1_EOTPE_Msk            (0x1UL << DSI_ISR1_EOTPE_Pos)            /*!< 0x00000040 */
#define DSI_ISR1_EOTPE                DSI_ISR1_EOTPE_Msk                       /*!< EoTp Error */
#define DSI_ISR1_LPWRE_Pos            (7U)
#define DSI_ISR1_LPWRE_Msk            (0x1UL << DSI_ISR1_LPWRE_Pos)            /*!< 0x00000080 */
#define DSI_ISR1_LPWRE                DSI_ISR1_LPWRE_Msk                       /*!< LTDC Payload Write Error */
#define DSI_ISR1_GCWRE_Pos            (8U)
#define DSI_ISR1_GCWRE_Msk            (0x1UL << DSI_ISR1_GCWRE_Pos)            /*!< 0x00000100 */
#define DSI_ISR1_GCWRE                DSI_ISR1_GCWRE_Msk                       /*!< Generic Command Write Error */
#define DSI_ISR1_GPWRE_Pos            (9U)
#define DSI_ISR1_GPWRE_Msk            (0x1UL << DSI_ISR1_GPWRE_Pos)            /*!< 0x00000200 */
#define DSI_ISR1_GPWRE                DSI_ISR1_GPWRE_Msk                       /*!< Generic Payload Write Error */
#define DSI_ISR1_GPTXE_Pos            (10U)
#define DSI_ISR1_GPTXE_Msk            (0x1UL << DSI_ISR1_GPTXE_Pos)            /*!< 0x00000400 */
#define DSI_ISR1_GPTXE                DSI_ISR1_GPTXE_Msk                       /*!< Generic Payload Transmit Error */
#define DSI_ISR1_GPRDE_Pos            (11U)
#define DSI_ISR1_GPRDE_Msk            (0x1UL << DSI_ISR1_GPRDE_Pos)            /*!< 0x00000800 */
#define DSI_ISR1_GPRDE                DSI_ISR1_GPRDE_Msk                       /*!< Generic Payload Read Error */
#define DSI_ISR1_GPRXE_Pos            (12U)
#define DSI_ISR1_GPRXE_Msk            (0x1UL << DSI_ISR1_GPRXE_Pos)            /*!< 0x00001000 */
#define DSI_ISR1_GPRXE                DSI_ISR1_GPRXE_Msk                       /*!< Generic Payload Receive Error */

/*******************  Bit definition for DSI_IER0 register  ***************/
#define DSI_IER0_AE0IE_Pos            (0U)
#define DSI_IER0_AE0IE_Msk            (0x1UL << DSI_IER0_AE0IE_Pos)            /*!< 0x00000001 */
#define DSI_IER0_AE0IE                DSI_IER0_AE0IE_Msk                       /*!< Acknowledge Error 0 Interrupt Enable */
#define DSI_IER0_AE1IE_Pos            (1U)
#define DSI_IER0_AE1IE_Msk            (0x1UL << DSI_IER0_AE1IE_Pos)            /*!< 0x00000002 */
#define DSI_IER0_AE1IE                DSI_IER0_AE1IE_Msk                       /*!< Acknowledge Error 1 Interrupt Enable */
#define DSI_IER0_AE2IE_Pos            (2U)
#define DSI_IER0_AE2IE_Msk            (0x1UL << DSI_IER0_AE2IE_Pos)            /*!< 0x00000004 */
#define DSI_IER0_AE2IE                DSI_IER0_AE2IE_Msk                       /*!< Acknowledge Error 2 Interrupt Enable */
#define DSI_IER0_AE3IE_Pos            (3U)
#define DSI_IER0_AE3IE_Msk            (0x1UL << DSI_IER0_AE3IE_Pos)            /*!< 0x00000008 */
#define DSI_IER0_AE3IE                DSI_IER0_AE3IE_Msk                       /*!< Acknowledge Error 3 Interrupt Enable */
#define DSI_IER0_AE4IE_Pos            (4U)
#define DSI_IER0_AE4IE_Msk            (0x1UL << DSI_IER0_AE4IE_Pos)            /*!< 0x00000010 */
#define DSI_IER0_AE4IE                DSI_IER0_AE4IE_Msk                       /*!< Acknowledge Error 4 Interrupt Enable */
#define DSI_IER0_AE5IE_Pos            (5U)
#define DSI_IER0_AE5IE_Msk            (0x1UL << DSI_IER0_AE5IE_Pos)            /*!< 0x00000020 */
#define DSI_IER0_AE5IE                DSI_IER0_AE5IE_Msk                       /*!< Acknowledge Error 5 Interrupt Enable */
#define DSI_IER0_AE6IE_Pos            (6U)
#define DSI_IER0_AE6IE_Msk            (0x1UL << DSI_IER0_AE6IE_Pos)            /*!< 0x00000040 */
#define DSI_IER0_AE6IE                DSI_IER0_AE6IE_Msk                       /*!< Acknowledge Error 6 Interrupt Enable */
#define DSI_IER0_AE7IE_Pos            (7U)
#define DSI_IER0_AE7IE_Msk            (0x1UL << DSI_IER0_AE7IE_Pos)            /*!< 0x00000080 */
#define DSI_IER0_AE7IE                DSI_IER0_AE7IE_Msk                       /*!< Acknowledge Error 7 Interrupt Enable */
#define DSI_IER0_AE8IE_Pos            (8U)
#define DSI_IER0_AE8IE_Msk            (0x1UL << DSI_IER0_AE8IE_Pos)            /*!< 0x00000100 */
#define DSI_IER0_AE8IE                DSI_IER0_AE8IE_Msk                       /*!< Acknowledge Error 8 Interrupt Enable */
#define DSI_IER0_AE9IE_Pos            (9U)
#define DSI_IER0_AE9IE_Msk            (0x1UL << DSI_IER0_AE9IE_Pos)            /*!< 0x00000200 */
#define DSI_IER0_AE9IE                DSI_IER0_AE9IE_Msk                       /*!< Acknowledge Error 9 Interrupt Enable */
#define DSI_IER0_AE10IE_Pos           (10U)
#define DSI_IER0_AE10IE_Msk           (0x1UL << DSI_IER0_AE10IE_Pos)           /*!< 0x00000400 */
#define DSI_IER0_AE10IE               DSI_IER0_AE10IE_Msk                      /*!< Acknowledge Error 10 Interrupt Enable */
#define DSI_IER0_AE11IE_Pos           (11U)
#define DSI_IER0_AE11IE_Msk           (0x1UL << DSI_IER0_AE11IE_Pos)           /*!< 0x00000800 */
#define DSI_IER0_AE11IE               DSI_IER0_AE11IE_Msk                      /*!< Acknowledge Error 11 Interrupt Enable */
#define DSI_IER0_AE12IE_Pos           (12U)
#define DSI_IER0_AE12IE_Msk           (0x1UL << DSI_IER0_AE12IE_Pos)           /*!< 0x00001000 */
#define DSI_IER0_AE12IE               DSI_IER0_AE12IE_Msk                      /*!< Acknowledge Error 12 Interrupt Enable */
#define DSI_IER0_AE13IE_Pos           (13U)
#define DSI_IER0_AE13IE_Msk           (0x1UL << DSI_IER0_AE13IE_Pos)           /*!< 0x00002000 */
#define DSI_IER0_AE13IE               DSI_IER0_AE13IE_Msk                      /*!< Acknowledge Error 13 Interrupt Enable */
#define DSI_IER0_AE14IE_Pos           (14U)
#define DSI_IER0_AE14IE_Msk           (0x1UL << DSI_IER0_AE14IE_Pos)           /*!< 0x00004000 */
#define DSI_IER0_AE14IE               DSI_IER0_AE14IE_Msk                      /*!< Acknowledge Error 14 Interrupt Enable */
#define DSI_IER0_AE15IE_Pos           (15U)
#define DSI_IER0_AE15IE_Msk           (0x1UL << DSI_IER0_AE15IE_Pos)           /*!< 0x00008000 */
#define DSI_IER0_AE15IE               DSI_IER0_AE15IE_Msk                      /*!< Acknowledge Error 15 Interrupt Enable */
#define DSI_IER0_PE0IE_Pos            (16U)
#define DSI_IER0_PE0IE_Msk            (0x1UL << DSI_IER0_PE0IE_Pos)            /*!< 0x00010000 */
#define DSI_IER0_PE0IE                DSI_IER0_PE0IE_Msk                       /*!< PHY Error 0 Interrupt Enable */
#define DSI_IER0_PE1IE_Pos            (17U)
#define DSI_IER0_PE1IE_Msk            (0x1UL << DSI_IER0_PE1IE_Pos)            /*!< 0x00020000 */
#define DSI_IER0_PE1IE                DSI_IER0_PE1IE_Msk                       /*!< PHY Error 1 Interrupt Enable */
#define DSI_IER0_PE2IE_Pos            (18U)
#define DSI_IER0_PE2IE_Msk            (0x1UL << DSI_IER0_PE2IE_Pos)            /*!< 0x00040000 */
#define DSI_IER0_PE2IE                DSI_IER0_PE2IE_Msk                       /*!< PHY Error 2 Interrupt Enable */
#define DSI_IER0_PE3IE_Pos            (19U)
#define DSI_IER0_PE3IE_Msk            (0x1UL << DSI_IER0_PE3IE_Pos)            /*!< 0x00080000 */
#define DSI_IER0_PE3IE                DSI_IER0_PE3IE_Msk                       /*!< PHY Error 3 Interrupt Enable */
#define DSI_IER0_PE4IE_Pos            (20U)
#define DSI_IER0_PE4IE_Msk            (0x1UL << DSI_IER0_PE4IE_Pos)            /*!< 0x00100000 */
#define DSI_IER0_PE4IE                DSI_IER0_PE4IE_Msk                       /*!< PHY Error 4 Interrupt Enable */

/*******************  Bit definition for DSI_IER1 register  ***************/
#define DSI_IER1_TOHSTXIE_Pos         (0U)
#define DSI_IER1_TOHSTXIE_Msk         (0x1UL << DSI_IER1_TOHSTXIE_Pos)         /*!< 0x00000001 */
#define DSI_IER1_TOHSTXIE             DSI_IER1_TOHSTXIE_Msk                    /*!< Timeout High-Speed Transmission Interrupt Enable */
#define DSI_IER1_TOLPRXIE_Pos         (1U)
#define DSI_IER1_TOLPRXIE_Msk         (0x1UL << DSI_IER1_TOLPRXIE_Pos)         /*!< 0x00000002 */
#define DSI_IER1_TOLPRXIE             DSI_IER1_TOLPRXIE_Msk                    /*!< Timeout Low-Power Reception Interrupt Enable */
#define DSI_IER1_ECCSEIE_Pos          (2U)
#define DSI_IER1_ECCSEIE_Msk          (0x1UL << DSI_IER1_ECCSEIE_Pos)          /*!< 0x00000004 */
#define DSI_IER1_ECCSEIE              DSI_IER1_ECCSEIE_Msk                     /*!< ECC Single-bit Error Interrupt Enable */
#define DSI_IER1_ECCMEIE_Pos          (3U)
#define DSI_IER1_ECCMEIE_Msk          (0x1UL << DSI_IER1_ECCMEIE_Pos)          /*!< 0x00000008 */
#define DSI_IER1_ECCMEIE              DSI_IER1_ECCMEIE_Msk                     /*!< ECC Multi-bit Error Interrupt Enable */
#define DSI_IER1_CRCEIE_Pos           (4U)
#define DSI_IER1_CRCEIE_Msk           (0x1UL << DSI_IER1_CRCEIE_Pos)           /*!< 0x00000010 */
#define DSI_IER1_CRCEIE               DSI_IER1_CRCEIE_Msk                      /*!< CRC Error Interrupt Enable */
#define DSI_IER1_PSEIE_Pos            (5U)
#define DSI_IER1_PSEIE_Msk            (0x1UL << DSI_IER1_PSEIE_Pos)            /*!< 0x00000020 */
#define DSI_IER1_PSEIE                DSI_IER1_PSEIE_Msk                       /*!< Packet Size Error Interrupt Enable */
#define DSI_IER1_EOTPEIE_Pos          (6U)
#define DSI_IER1_EOTPEIE_Msk          (0x1UL << DSI_IER1_EOTPEIE_Pos)          /*!< 0x00000040 */
#define DSI_IER1_EOTPEIE              DSI_IER1_EOTPEIE_Msk                     /*!< EoTp Error Interrupt Enable */
#define DSI_IER1_LPWREIE_Pos          (7U)
#define DSI_IER1_LPWREIE_Msk          (0x1UL << DSI_IER1_LPWREIE_Pos)          /*!< 0x00000080 */
#define DSI_IER1_LPWREIE              DSI_IER1_LPWREIE_Msk                     /*!< LTDC Payload Write Error Interrupt Enable */
#define DSI_IER1_GCWREIE_Pos          (8U)
#define DSI_IER1_GCWREIE_Msk          (0x1UL << DSI_IER1_GCWREIE_Pos)          /*!< 0x00000100 */
#define DSI_IER1_GCWREIE              DSI_IER1_GCWREIE_Msk                     /*!< Generic Command Write Error Interrupt Enable */
#define DSI_IER1_GPWREIE_Pos          (9U)
#define DSI_IER1_GPWREIE_Msk          (0x1UL << DSI_IER1_GPWREIE_Pos)          /*!< 0x00000200 */
#define DSI_IER1_GPWREIE              DSI_IER1_GPWREIE_Msk                     /*!< Generic Payload Write Error Interrupt Enable */
#define DSI_IER1_GPTXEIE_Pos          (10U)
#define DSI_IER1_GPTXEIE_Msk          (0x1UL << DSI_IER1_GPTXEIE_Pos)          /*!< 0x00000400 */
#define DSI_IER1_GPTXEIE              DSI_IER1_GPTXEIE_Msk                     /*!< Generic Payload Transmit Error Interrupt Enable */
#define DSI_IER1_GPRDEIE_Pos          (11U)
#define DSI_IER1_GPRDEIE_Msk          (0x1UL << DSI_IER1_GPRDEIE_Pos)          /*!< 0x00000800 */
#define DSI_IER1_GPRDEIE              DSI_IER1_GPRDEIE_Msk                     /*!< Generic Payload Read Error Interrupt Enable */
#define DSI_IER1_GPRXEIE_Pos          (12U)
#define DSI_IER1_GPRXEIE_Msk          (0x1UL << DSI_IER1_GPRXEIE_Pos)          /*!< 0x00001000 */
#define DSI_IER1_GPRXEIE              DSI_IER1_GPRXEIE_Msk                     /*!< Generic Payload Receive Error Interrupt Enable */

/*******************  Bit definition for DSI_FIR0 register  ***************/
#define DSI_FIR0_FAE0_Pos             (0U)
#define DSI_FIR0_FAE0_Msk             (0x1UL << DSI_FIR0_FAE0_Pos)             /*!< 0x00000001 */
#define DSI_FIR0_FAE0                 DSI_FIR0_FAE0_Msk                        /*!< Force Acknowledge Error 0 */
#define DSI_FIR0_FAE1_Pos             (1U)
#define DSI_FIR0_FAE1_Msk             (0x1UL << DSI_FIR0_FAE1_Pos)             /*!< 0x00000002 */
#define DSI_FIR0_FAE1                 DSI_FIR0_FAE1_Msk                        /*!< Force Acknowledge Error 1 */
#define DSI_FIR0_FAE2_Pos             (2U)
#define DSI_FIR0_FAE2_Msk             (0x1UL << DSI_FIR0_FAE2_Pos)             /*!< 0x00000004 */
#define DSI_FIR0_FAE2                 DSI_FIR0_FAE2_Msk                        /*!< Force Acknowledge Error 2 */
#define DSI_FIR0_FAE3_Pos             (3U)
#define DSI_FIR0_FAE3_Msk             (0x1UL << DSI_FIR0_FAE3_Pos)             /*!< 0x00000008 */
#define DSI_FIR0_FAE3                 DSI_FIR0_FAE3_Msk                        /*!< Force Acknowledge Error 3 */
#define DSI_FIR0_FAE4_Pos             (4U)
#define DSI_FIR0_FAE4_Msk             (0x1UL << DSI_FIR0_FAE4_Pos)             /*!< 0x00000010 */
#define DSI_FIR0_FAE4                 DSI_FIR0_FAE4_Msk                        /*!< Force Acknowledge Error 4 */
#define DSI_FIR0_FAE5_Pos             (5U)
#define DSI_FIR0_FAE5_Msk             (0x1UL << DSI_FIR0_FAE5_Pos)             /*!< 0x00000020 */
#define DSI_FIR0_FAE5                 DSI_FIR0_FAE5_Msk                        /*!< Force Acknowledge Error 5 */
#define DSI_FIR0_FAE6_Pos             (6U)
#define DSI_FIR0_FAE6_Msk             (0x1UL << DSI_FIR0_FAE6_Pos)             /*!< 0x00000040 */
#define DSI_FIR0_FAE6                 DSI_FIR0_FAE6_Msk                        /*!< Force Acknowledge Error 6 */
#define DSI_FIR0_FAE7_Pos             (7U)
#define DSI_FIR0_FAE7_Msk             (0x1UL << DSI_FIR0_FAE7_Pos)             /*!< 0x00000080 */
#define DSI_FIR0_FAE7                 DSI_FIR0_FAE7_Msk                        /*!< Force Acknowledge Error 7 */
#define DSI_FIR0_FAE8_Pos             (8U)
#define DSI_FIR0_FAE8_Msk             (0x1UL << DSI_FIR0_FAE8_Pos)             /*!< 0x00000100 */
#define DSI_FIR0_FAE8                 DSI_FIR0_FAE8_Msk                        /*!< Force Acknowledge Error 8 */
#define DSI_FIR0_FAE9_Pos             (9U)
#define DSI_FIR0_FAE9_Msk             (0x1UL << DSI_FIR0_FAE9_Pos)             /*!< 0x00000200 */
#define DSI_FIR0_FAE9                 DSI_FIR0_FAE9_Msk                        /*!< Force Acknowledge Error 9 */
#define DSI_FIR0_FAE10_Pos            (10U)
#define DSI_FIR0_FAE10_Msk            (0x1UL << DSI_FIR0_FAE10_Pos)            /*!< 0x00000400 */
#define DSI_FIR0_FAE10                DSI_FIR0_FAE10_Msk                       /*!< Force Acknowledge Error 10 */
#define DSI_FIR0_FAE11_Pos            (11U)
#define DSI_FIR0_FAE11_Msk            (0x1UL << DSI_FIR0_FAE11_Pos)            /*!< 0x00000800 */
#define DSI_FIR0_FAE11                DSI_FIR0_FAE11_Msk                       /*!< Force Acknowledge Error 11 */
#define DSI_FIR0_FAE12_Pos            (12U)
#define DSI_FIR0_FAE12_Msk            (0x1UL << DSI_FIR0_FAE12_Pos)            /*!< 0x00001000 */
#define DSI_FIR0_FAE12                DSI_FIR0_FAE12_Msk                       /*!< Force Acknowledge Error 12 */
#define DSI_FIR0_FAE13_Pos            (13U)
#define DSI_FIR0_FAE13_Msk            (0x1UL << DSI_FIR0_FAE13_Pos)            /*!< 0x00002000 */
#define DSI_FIR0_FAE13                DSI_FIR0_FAE13_Msk                       /*!< Force Acknowledge Error 13 */
#define DSI_FIR0_FAE14_Pos            (14U)
#define DSI_FIR0_FAE14_Msk            (0x1UL << DSI_FIR0_FAE14_Pos)            /*!< 0x00004000 */
#define DSI_FIR0_FAE14                DSI_FIR0_FAE14_Msk                       /*!< Force Acknowledge Error 14 */
#define DSI_FIR0_FAE15_Pos            (15U)
#define DSI_FIR0_FAE15_Msk            (0x1UL << DSI_FIR0_FAE15_Pos)            /*!< 0x00008000 */
#define DSI_FIR0_FAE15                DSI_FIR0_FAE15_Msk                       /*!< Force Acknowledge Error 15 */
#define DSI_FIR0_FPE0_Pos             (16U)
#define DSI_FIR0_FPE0_Msk             (0x1UL << DSI_FIR0_FPE0_Pos)             /*!< 0x00010000 */
#define DSI_FIR0_FPE0                 DSI_FIR0_FPE0_Msk                        /*!< Force PHY Error 0 */
#define DSI_FIR0_FPE1_Pos             (17U)
#define DSI_FIR0_FPE1_Msk             (0x1UL << DSI_FIR0_FPE1_Pos)             /*!< 0x00020000 */
#define DSI_FIR0_FPE1                 DSI_FIR0_FPE1_Msk                        /*!< Force PHY Error 1 */
#define DSI_FIR0_FPE2_Pos             (18U)
#define DSI_FIR0_FPE2_Msk             (0x1UL << DSI_FIR0_FPE2_Pos)             /*!< 0x00040000 */
#define DSI_FIR0_FPE2                 DSI_FIR0_FPE2_Msk                        /*!< Force PHY Error 2 */
#define DSI_FIR0_FPE3_Pos             (19U)
#define DSI_FIR0_FPE3_Msk             (0x1UL << DSI_FIR0_FPE3_Pos)             /*!< 0x00080000 */
#define DSI_FIR0_FPE3                 DSI_FIR0_FPE3_Msk                        /*!< Force PHY Error 3 */
#define DSI_FIR0_FPE4_Pos             (20U)
#define DSI_FIR0_FPE4_Msk             (0x1UL << DSI_FIR0_FPE4_Pos)             /*!< 0x00100000 */
#define DSI_FIR0_FPE4                 DSI_FIR0_FPE4_Msk                        /*!< Force PHY Error 4 */

/*******************  Bit definition for DSI_FIR1 register  ***************/
#define DSI_FIR1_FTOHSTX_Pos          (0U)
#define DSI_FIR1_FTOHSTX_Msk          (0x1UL << DSI_FIR1_FTOHSTX_Pos)          /*!< 0x00000001 */
#define DSI_FIR1_FTOHSTX              DSI_FIR1_FTOHSTX_Msk                     /*!< Force Timeout High-Speed Transmission */
#define DSI_FIR1_FTOLPRX_Pos          (1U)
#define DSI_FIR1_FTOLPRX_Msk          (0x1UL << DSI_FIR1_FTOLPRX_Pos)          /*!< 0x00000002 */
#define DSI_FIR1_FTOLPRX              DSI_FIR1_FTOLPRX_Msk                     /*!< Force Timeout Low-Power Reception */
#define DSI_FIR1_FECCSE_Pos           (2U)
#define DSI_FIR1_FECCSE_Msk           (0x1UL << DSI_FIR1_FECCSE_Pos)           /*!< 0x00000004 */
#define DSI_FIR1_FECCSE               DSI_FIR1_FECCSE_Msk                      /*!< Force ECC Single-bit Error */
#define DSI_FIR1_FECCME_Pos           (3U)
#define DSI_FIR1_FECCME_Msk           (0x1UL << DSI_FIR1_FECCME_Pos)           /*!< 0x00000008 */
#define DSI_FIR1_FECCME               DSI_FIR1_FECCME_Msk                      /*!< Force ECC Multi-bit Error */
#define DSI_FIR1_FCRCE_Pos            (4U)
#define DSI_FIR1_FCRCE_Msk            (0x1UL << DSI_FIR1_FCRCE_Pos)            /*!< 0x00000010 */
#define DSI_FIR1_FCRCE                DSI_FIR1_FCRCE_Msk                       /*!< Force CRC Error */
#define DSI_FIR1_FPSE_Pos             (5U)
#define DSI_FIR1_FPSE_Msk             (0x1UL << DSI_FIR1_FPSE_Pos)             /*!< 0x00000020 */
#define DSI_FIR1_FPSE                 DSI_FIR1_FPSE_Msk                        /*!< Force Packet Size Error */
#define DSI_FIR1_FEOTPE_Pos           (6U)
#define DSI_FIR1_FEOTPE_Msk           (0x1UL << DSI_FIR1_FEOTPE_Pos)           /*!< 0x00000040 */
#define DSI_FIR1_FEOTPE               DSI_FIR1_FEOTPE_Msk                      /*!< Force EoTp Error */
#define DSI_FIR1_FLPWRE_Pos           (7U)
#define DSI_FIR1_FLPWRE_Msk           (0x1UL << DSI_FIR1_FLPWRE_Pos)           /*!< 0x00000080 */
#define DSI_FIR1_FLPWRE               DSI_FIR1_FLPWRE_Msk                      /*!< Force LTDC Payload Write Error */
#define DSI_FIR1_FGCWRE_Pos           (8U)
#define DSI_FIR1_FGCWRE_Msk           (0x1UL << DSI_FIR1_FGCWRE_Pos)           /*!< 0x00000100 */
#define DSI_FIR1_FGCWRE               DSI_FIR1_FGCWRE_Msk                      /*!< Force Generic Command Write Error */
#define DSI_FIR1_FGPWRE_Pos           (9U)
#define DSI_FIR1_FGPWRE_Msk           (0x1UL << DSI_FIR1_FGPWRE_Pos)           /*!< 0x00000200 */
#define DSI_FIR1_FGPWRE               DSI_FIR1_FGPWRE_Msk                      /*!< Force Generic Payload Write Error */
#define DSI_FIR1_FGPTXE_Pos           (10U)
#define DSI_FIR1_FGPTXE_Msk           (0x1UL << DSI_FIR1_FGPTXE_Pos)           /*!< 0x00000400 */
#define DSI_FIR1_FGPTXE               DSI_FIR1_FGPTXE_Msk                      /*!< Force Generic Payload Transmit Error */
#define DSI_FIR1_FGPRDE_Pos           (11U)
#define DSI_FIR1_FGPRDE_Msk           (0x1UL << DSI_FIR1_FGPRDE_Pos)           /*!< 0x00000800 */
#define DSI_FIR1_FGPRDE               DSI_FIR1_FGPRDE_Msk                      /*!< Force Generic Payload Read Error */
#define DSI_FIR1_FGPRXE_Pos           (12U)
#define DSI_FIR1_FGPRXE_Msk           (0x1UL << DSI_FIR1_FGPRXE_Pos)           /*!< 0x00001000 */
#define DSI_FIR1_FGPRXE               DSI_FIR1_FGPRXE_Msk                      /*!< Force Generic Payload Receive Error */

/*******************  Bit definition for DSI_VSCR register  ***************/
#define DSI_VSCR_EN_Pos               (0U)
#define DSI_VSCR_EN_Msk               (0x1UL << DSI_VSCR_EN_Pos)               /*!< 0x00000001 */
#define DSI_VSCR_EN                   DSI_VSCR_EN_Msk                          /*!< Enable */
#define DSI_VSCR_UR_Pos               (8U)
#define DSI_VSCR_UR_Msk               (0x1UL << DSI_VSCR_UR_Pos)               /*!< 0x00000100 */
#define DSI_VSCR_UR                   DSI_VSCR_UR_Msk                          /*!< Update Register */

/*******************  Bit definition for DSI_LCVCIDR register  ************/
#define DSI_LCVCIDR_VCID_Pos          (0U)
#define DSI_LCVCIDR_VCID_Msk          (0x3UL << DSI_LCVCIDR_VCID_Pos)          /*!< 0x00000003 */
#define DSI_LCVCIDR_VCID              DSI_LCVCIDR_VCID_Msk                     /*!< Virtual Channel ID */
#define DSI_LCVCIDR_VCID0_Pos         (0U)
#define DSI_LCVCIDR_VCID0_Msk         (0x1UL << DSI_LCVCIDR_VCID0_Pos)         /*!< 0x00000001 */
#define DSI_LCVCIDR_VCID0             DSI_LCVCIDR_VCID0_Msk
#define DSI_LCVCIDR_VCID1_Pos         (1U)
#define DSI_LCVCIDR_VCID1_Msk         (0x1UL << DSI_LCVCIDR_VCID1_Pos)         /*!< 0x00000002 */
#define DSI_LCVCIDR_VCID1             DSI_LCVCIDR_VCID1_Msk

/*******************  Bit definition for DSI_LCCCR register  **************/
#define DSI_LCCCR_COLC_Pos            (0U)
#define DSI_LCCCR_COLC_Msk            (0xFUL << DSI_LCCCR_COLC_Pos)            /*!< 0x0000000F */
#define DSI_LCCCR_COLC                DSI_LCCCR_COLC_Msk                       /*!< Color Coding */
#define DSI_LCCCR_COLC0_Pos           (0U)
#define DSI_LCCCR_COLC0_Msk           (0x1UL << DSI_LCCCR_COLC0_Pos)           /*!< 0x00000001 */
#define DSI_LCCCR_COLC0               DSI_LCCCR_COLC0_Msk
#define DSI_LCCCR_COLC1_Pos           (1U)
#define DSI_LCCCR_COLC1_Msk           (0x1UL << DSI_LCCCR_COLC1_Pos)           /*!< 0x00000002 */
#define DSI_LCCCR_COLC1               DSI_LCCCR_COLC1_Msk
#define DSI_LCCCR_COLC2_Pos           (2U)
#define DSI_LCCCR_COLC2_Msk           (0x1UL << DSI_LCCCR_COLC2_Pos)           /*!< 0x00000004 */
#define DSI_LCCCR_COLC2               DSI_LCCCR_COLC2_Msk
#define DSI_LCCCR_COLC3_Pos           (3U)
#define DSI_LCCCR_COLC3_Msk           (0x1UL << DSI_LCCCR_COLC3_Pos)           /*!< 0x00000008 */
#define DSI_LCCCR_COLC3               DSI_LCCCR_COLC3_Msk

#define DSI_LCCCR_LPE_Pos             (8U)
#define DSI_LCCCR_LPE_Msk             (0x1UL << DSI_LCCCR_LPE_Pos)             /*!< 0x00000100 */
#define DSI_LCCCR_LPE                 DSI_LCCCR_LPE_Msk                        /*!< Loosely Packed Enable */

/*******************  Bit definition for DSI_LPMCCR register  *************/
#define DSI_LPMCCR_VLPSIZE_Pos        (0U)
#define DSI_LPMCCR_VLPSIZE_Msk        (0xFFUL << DSI_LPMCCR_VLPSIZE_Pos)       /*!< 0x000000FF */
#define DSI_LPMCCR_VLPSIZE            DSI_LPMCCR_VLPSIZE_Msk                   /*!< VACT Largest Packet Size */
#define DSI_LPMCCR_VLPSIZE0_Pos       (0U)
#define DSI_LPMCCR_VLPSIZE0_Msk       (0x1UL << DSI_LPMCCR_VLPSIZE0_Pos)       /*!< 0x00000001 */
#define DSI_LPMCCR_VLPSIZE0           DSI_LPMCCR_VLPSIZE0_Msk
#define DSI_LPMCCR_VLPSIZE1_Pos       (1U)
#define DSI_LPMCCR_VLPSIZE1_Msk       (0x1UL << DSI_LPMCCR_VLPSIZE1_Pos)       /*!< 0x00000002 */
#define DSI_LPMCCR_VLPSIZE1           DSI_LPMCCR_VLPSIZE1_Msk
#define DSI_LPMCCR_VLPSIZE2_Pos       (2U)
#define DSI_LPMCCR_VLPSIZE2_Msk       (0x1UL << DSI_LPMCCR_VLPSIZE2_Pos)       /*!< 0x00000004 */
#define DSI_LPMCCR_VLPSIZE2           DSI_LPMCCR_VLPSIZE2_Msk
#define DSI_LPMCCR_VLPSIZE3_Pos       (3U)
#define DSI_LPMCCR_VLPSIZE3_Msk       (0x1UL << DSI_LPMCCR_VLPSIZE3_Pos)       /*!< 0x00000008 */
#define DSI_LPMCCR_VLPSIZE3           DSI_LPMCCR_VLPSIZE3_Msk
#define DSI_LPMCCR_VLPSIZE4_Pos       (4U)
#define DSI_LPMCCR_VLPSIZE4_Msk       (0x1UL << DSI_LPMCCR_VLPSIZE4_Pos)       /*!< 0x00000010 */
#define DSI_LPMCCR_VLPSIZE4           DSI_LPMCCR_VLPSIZE4_Msk
#define DSI_LPMCCR_VLPSIZE5_Pos       (5U)
#define DSI_LPMCCR_VLPSIZE5_Msk       (0x1UL << DSI_LPMCCR_VLPSIZE5_Pos)       /*!< 0x00000020 */
#define DSI_LPMCCR_VLPSIZE5           DSI_LPMCCR_VLPSIZE5_Msk
#define DSI_LPMCCR_VLPSIZE6_Pos       (6U)
#define DSI_LPMCCR_VLPSIZE6_Msk       (0x1UL << DSI_LPMCCR_VLPSIZE6_Pos)       /*!< 0x00000040 */
#define DSI_LPMCCR_VLPSIZE6           DSI_LPMCCR_VLPSIZE6_Msk
#define DSI_LPMCCR_VLPSIZE7_Pos       (7U)
#define DSI_LPMCCR_VLPSIZE7_Msk       (0x1UL << DSI_LPMCCR_VLPSIZE7_Pos)       /*!< 0x00000080 */
#define DSI_LPMCCR_VLPSIZE7           DSI_LPMCCR_VLPSIZE7_Msk

#define DSI_LPMCCR_LPSIZE_Pos         (16U)
#define DSI_LPMCCR_LPSIZE_Msk         (0xFFUL << DSI_LPMCCR_LPSIZE_Pos)        /*!< 0x00FF0000 */
#define DSI_LPMCCR_LPSIZE             DSI_LPMCCR_LPSIZE_Msk                    /*!< Largest Packet Size */
#define DSI_LPMCCR_LPSIZE0_Pos        (16U)
#define DSI_LPMCCR_LPSIZE0_Msk        (0x1UL << DSI_LPMCCR_LPSIZE0_Pos)        /*!< 0x00010000 */
#define DSI_LPMCCR_LPSIZE0            DSI_LPMCCR_LPSIZE0_Msk
#define DSI_LPMCCR_LPSIZE1_Pos        (17U)
#define DSI_LPMCCR_LPSIZE1_Msk        (0x1UL << DSI_LPMCCR_LPSIZE1_Pos)        /*!< 0x00020000 */
#define DSI_LPMCCR_LPSIZE1            DSI_LPMCCR_LPSIZE1_Msk
#define DSI_LPMCCR_LPSIZE2_Pos        (18U)
#define DSI_LPMCCR_LPSIZE2_Msk        (0x1UL << DSI_LPMCCR_LPSIZE2_Pos)        /*!< 0x00040000 */
#define DSI_LPMCCR_LPSIZE2            DSI_LPMCCR_LPSIZE2_Msk
#define DSI_LPMCCR_LPSIZE3_Pos        (19U)
#define DSI_LPMCCR_LPSIZE3_Msk        (0x1UL << DSI_LPMCCR_LPSIZE3_Pos)        /*!< 0x00080000 */
#define DSI_LPMCCR_LPSIZE3            DSI_LPMCCR_LPSIZE3_Msk
#define DSI_LPMCCR_LPSIZE4_Pos        (20U)
#define DSI_LPMCCR_LPSIZE4_Msk        (0x1UL << DSI_LPMCCR_LPSIZE4_Pos)        /*!< 0x00100000 */
#define DSI_LPMCCR_LPSIZE4            DSI_LPMCCR_LPSIZE4_Msk
#define DSI_LPMCCR_LPSIZE5_Pos        (21U)
#define DSI_LPMCCR_LPSIZE5_Msk        (0x1UL << DSI_LPMCCR_LPSIZE5_Pos)        /*!< 0x00200000 */
#define DSI_LPMCCR_LPSIZE5            DSI_LPMCCR_LPSIZE5_Msk
#define DSI_LPMCCR_LPSIZE6_Pos        (22U)
#define DSI_LPMCCR_LPSIZE6_Msk        (0x1UL << DSI_LPMCCR_LPSIZE6_Pos)        /*!< 0x00400000 */
#define DSI_LPMCCR_LPSIZE6            DSI_LPMCCR_LPSIZE6_Msk
#define DSI_LPMCCR_LPSIZE7_Pos        (23U)
#define DSI_LPMCCR_LPSIZE7_Msk        (0x1UL << DSI_LPMCCR_LPSIZE7_Pos)        /*!< 0x00800000 */
#define DSI_LPMCCR_LPSIZE7            DSI_LPMCCR_LPSIZE7_Msk

/*******************  Bit definition for DSI_VMCCR register  **************/
#define DSI_VMCCR_VMT_Pos             (0U)
#define DSI_VMCCR_VMT_Msk             (0x3UL << DSI_VMCCR_VMT_Pos)             /*!< 0x00000003 */
#define DSI_VMCCR_VMT                 DSI_VMCCR_VMT_Msk                        /*!< Video Mode Type */
#define DSI_VMCCR_VMT0_Pos            (0U)
#define DSI_VMCCR_VMT0_Msk            (0x1UL << DSI_VMCCR_VMT0_Pos)            /*!< 0x00000001 */
#define DSI_VMCCR_VMT0                DSI_VMCCR_VMT0_Msk
#define DSI_VMCCR_VMT1_Pos            (1U)
#define DSI_VMCCR_VMT1_Msk            (0x1UL << DSI_VMCCR_VMT1_Pos)            /*!< 0x00000002 */
#define DSI_VMCCR_VMT1                DSI_VMCCR_VMT1_Msk

#define DSI_VMCCR_LPVSAE_Pos          (8U)
#define DSI_VMCCR_LPVSAE_Msk          (0x1UL << DSI_VMCCR_LPVSAE_Pos)          /*!< 0x00000100 */
#define DSI_VMCCR_LPVSAE              DSI_VMCCR_LPVSAE_Msk                     /*!< Low-power Vertical Sync time Enable */
#define DSI_VMCCR_LPVBPE_Pos          (9U)
#define DSI_VMCCR_LPVBPE_Msk          (0x1UL << DSI_VMCCR_LPVBPE_Pos)          /*!< 0x00000200 */
#define DSI_VMCCR_LPVBPE              DSI_VMCCR_LPVBPE_Msk                     /*!< Low-power Vertical Back-porch Enable */
#define DSI_VMCCR_LPVFPE_Pos          (10U)
#define DSI_VMCCR_LPVFPE_Msk          (0x1UL << DSI_VMCCR_LPVFPE_Pos)          /*!< 0x00000400 */
#define DSI_VMCCR_LPVFPE              DSI_VMCCR_LPVFPE_Msk                     /*!< Low-power Vertical Front-porch Enable */
#define DSI_VMCCR_LPVAE_Pos           (11U)
#define DSI_VMCCR_LPVAE_Msk           (0x1UL << DSI_VMCCR_LPVAE_Pos)           /*!< 0x00000800 */
#define DSI_VMCCR_LPVAE               DSI_VMCCR_LPVAE_Msk                      /*!< Low-power Vertical Active Enable */
#define DSI_VMCCR_LPHBPE_Pos          (12U)
#define DSI_VMCCR_LPHBPE_Msk          (0x1UL << DSI_VMCCR_LPHBPE_Pos)          /*!< 0x00001000 */
#define DSI_VMCCR_LPHBPE              DSI_VMCCR_LPHBPE_Msk                     /*!< Low-power Horizontal Back-porch Enable */
#define DSI_VMCCR_LPHFE_Pos           (13U)
#define DSI_VMCCR_LPHFE_Msk           (0x1UL << DSI_VMCCR_LPHFE_Pos)           /*!< 0x00002000 */
#define DSI_VMCCR_LPHFE               DSI_VMCCR_LPHFE_Msk                      /*!< Low-power Horizontal Front-porch Enable */
#define DSI_VMCCR_FBTAAE_Pos          (14U)
#define DSI_VMCCR_FBTAAE_Msk          (0x1UL << DSI_VMCCR_FBTAAE_Pos)          /*!< 0x00004000 */
#define DSI_VMCCR_FBTAAE              DSI_VMCCR_FBTAAE_Msk                     /*!< Frame BTA Acknowledge Enable */
#define DSI_VMCCR_LPCE_Pos            (15U)
#define DSI_VMCCR_LPCE_Msk            (0x1UL << DSI_VMCCR_LPCE_Pos)            /*!< 0x00008000 */
#define DSI_VMCCR_LPCE                DSI_VMCCR_LPCE_Msk                       /*!< Low-power Command Enable */

/*******************  Bit definition for DSI_VPCCR register  **************/
#define DSI_VPCCR_VPSIZE_Pos          (0U)
#define DSI_VPCCR_VPSIZE_Msk          (0x3FFFUL << DSI_VPCCR_VPSIZE_Pos)       /*!< 0x00003FFF */
#define DSI_VPCCR_VPSIZE              DSI_VPCCR_VPSIZE_Msk                     /*!< Video Packet Size */
#define DSI_VPCCR_VPSIZE0_Pos         (0U)
#define DSI_VPCCR_VPSIZE0_Msk         (0x1UL << DSI_VPCCR_VPSIZE0_Pos)         /*!< 0x00000001 */
#define DSI_VPCCR_VPSIZE0             DSI_VPCCR_VPSIZE0_Msk
#define DSI_VPCCR_VPSIZE1_Pos         (1U)
#define DSI_VPCCR_VPSIZE1_Msk         (0x1UL << DSI_VPCCR_VPSIZE1_Pos)         /*!< 0x00000002 */
#define DSI_VPCCR_VPSIZE1             DSI_VPCCR_VPSIZE1_Msk
#define DSI_VPCCR_VPSIZE2_Pos         (2U)
#define DSI_VPCCR_VPSIZE2_Msk         (0x1UL << DSI_VPCCR_VPSIZE2_Pos)         /*!< 0x00000004 */
#define DSI_VPCCR_VPSIZE2             DSI_VPCCR_VPSIZE2_Msk
#define DSI_VPCCR_VPSIZE3_Pos         (3U)
#define DSI_VPCCR_VPSIZE3_Msk         (0x1UL << DSI_VPCCR_VPSIZE3_Pos)         /*!< 0x00000008 */
#define DSI_VPCCR_VPSIZE3             DSI_VPCCR_VPSIZE3_Msk
#define DSI_VPCCR_VPSIZE4_Pos         (4U)
#define DSI_VPCCR_VPSIZE4_Msk         (0x1UL << DSI_VPCCR_VPSIZE4_Pos)         /*!< 0x00000010 */
#define DSI_VPCCR_VPSIZE4             DSI_VPCCR_VPSIZE4_Msk
#define DSI_VPCCR_VPSIZE5_Pos         (5U)
#define DSI_VPCCR_VPSIZE5_Msk         (0x1UL << DSI_VPCCR_VPSIZE5_Pos)         /*!< 0x00000020 */
#define DSI_VPCCR_VPSIZE5             DSI_VPCCR_VPSIZE5_Msk
#define DSI_VPCCR_VPSIZE6_Pos         (6U)
#define DSI_VPCCR_VPSIZE6_Msk         (0x1UL << DSI_VPCCR_VPSIZE6_Pos)         /*!< 0x00000040 */
#define DSI_VPCCR_VPSIZE6             DSI_VPCCR_VPSIZE6_Msk
#define DSI_VPCCR_VPSIZE7_Pos         (7U)
#define DSI_VPCCR_VPSIZE7_Msk         (0x1UL << DSI_VPCCR_VPSIZE7_Pos)         /*!< 0x00000080 */
#define DSI_VPCCR_VPSIZE7             DSI_VPCCR_VPSIZE7_Msk
#define DSI_VPCCR_VPSIZE8_Pos         (8U)
#define DSI_VPCCR_VPSIZE8_Msk         (0x1UL << DSI_VPCCR_VPSIZE8_Pos)         /*!< 0x00000100 */
#define DSI_VPCCR_VPSIZE8             DSI_VPCCR_VPSIZE8_Msk
#define DSI_VPCCR_VPSIZE9_Pos         (9U)
#define DSI_VPCCR_VPSIZE9_Msk         (0x1UL << DSI_VPCCR_VPSIZE9_Pos)         /*!< 0x00000200 */
#define DSI_VPCCR_VPSIZE9             DSI_VPCCR_VPSIZE9_Msk
#define DSI_VPCCR_VPSIZE10_Pos        (10U)
#define DSI_VPCCR_VPSIZE10_Msk        (0x1UL << DSI_VPCCR_VPSIZE10_Pos)        /*!< 0x00000400 */
#define DSI_VPCCR_VPSIZE10            DSI_VPCCR_VPSIZE10_Msk
#define DSI_VPCCR_VPSIZE11_Pos        (11U)
#define DSI_VPCCR_VPSIZE11_Msk        (0x1UL << DSI_VPCCR_VPSIZE11_Pos)        /*!< 0x00000800 */
#define DSI_VPCCR_VPSIZE11            DSI_VPCCR_VPSIZE11_Msk
#define DSI_VPCCR_VPSIZE12_Pos        (12U)
#define DSI_VPCCR_VPSIZE12_Msk        (0x1UL << DSI_VPCCR_VPSIZE12_Pos)        /*!< 0x00001000 */
#define DSI_VPCCR_VPSIZE12            DSI_VPCCR_VPSIZE12_Msk
#define DSI_VPCCR_VPSIZE13_Pos        (13U)
#define DSI_VPCCR_VPSIZE13_Msk        (0x1UL << DSI_VPCCR_VPSIZE13_Pos)        /*!< 0x00002000 */
#define DSI_VPCCR_VPSIZE13            DSI_VPCCR_VPSIZE13_Msk

/*******************  Bit definition for DSI_VCCCR register  **************/
#define DSI_VCCCR_NUMC_Pos            (0U)
#define DSI_VCCCR_NUMC_Msk            (0x1FFFUL << DSI_VCCCR_NUMC_Pos)         /*!< 0x00001FFF */
#define DSI_VCCCR_NUMC                DSI_VCCCR_NUMC_Msk                       /*!< Number of Chunks */
#define DSI_VCCCR_NUMC0_Pos           (0U)
#define DSI_VCCCR_NUMC0_Msk           (0x1UL << DSI_VCCCR_NUMC0_Pos)           /*!< 0x00000001 */
#define DSI_VCCCR_NUMC0               DSI_VCCCR_NUMC0_Msk
#define DSI_VCCCR_NUMC1_Pos           (1U)
#define DSI_VCCCR_NUMC1_Msk           (0x1UL << DSI_VCCCR_NUMC1_Pos)           /*!< 0x00000002 */
#define DSI_VCCCR_NUMC1               DSI_VCCCR_NUMC1_Msk
#define DSI_VCCCR_NUMC2_Pos           (2U)
#define DSI_VCCCR_NUMC2_Msk           (0x1UL << DSI_VCCCR_NUMC2_Pos)           /*!< 0x00000004 */
#define DSI_VCCCR_NUMC2               DSI_VCCCR_NUMC2_Msk
#define DSI_VCCCR_NUMC3_Pos           (3U)
#define DSI_VCCCR_NUMC3_Msk           (0x1UL << DSI_VCCCR_NUMC3_Pos)           /*!< 0x00000008 */
#define DSI_VCCCR_NUMC3               DSI_VCCCR_NUMC3_Msk
#define DSI_VCCCR_NUMC4_Pos           (4U)
#define DSI_VCCCR_NUMC4_Msk           (0x1UL << DSI_VCCCR_NUMC4_Pos)           /*!< 0x00000010 */
#define DSI_VCCCR_NUMC4               DSI_VCCCR_NUMC4_Msk
#define DSI_VCCCR_NUMC5_Pos           (5U)
#define DSI_VCCCR_NUMC5_Msk           (0x1UL << DSI_VCCCR_NUMC5_Pos)           /*!< 0x00000020 */
#define DSI_VCCCR_NUMC5               DSI_VCCCR_NUMC5_Msk
#define DSI_VCCCR_NUMC6_Pos           (6U)
#define DSI_VCCCR_NUMC6_Msk           (0x1UL << DSI_VCCCR_NUMC6_Pos)           /*!< 0x00000040 */
#define DSI_VCCCR_NUMC6               DSI_VCCCR_NUMC6_Msk
#define DSI_VCCCR_NUMC7_Pos           (7U)
#define DSI_VCCCR_NUMC7_Msk           (0x1UL << DSI_VCCCR_NUMC7_Pos)           /*!< 0x00000080 */
#define DSI_VCCCR_NUMC7               DSI_VCCCR_NUMC7_Msk
#define DSI_VCCCR_NUMC8_Pos           (8U)
#define DSI_VCCCR_NUMC8_Msk           (0x1UL << DSI_VCCCR_NUMC8_Pos)           /*!< 0x00000100 */
#define DSI_VCCCR_NUMC8               DSI_VCCCR_NUMC8_Msk
#define DSI_VCCCR_NUMC9_Pos           (9U)
#define DSI_VCCCR_NUMC9_Msk           (0x1UL << DSI_VCCCR_NUMC9_Pos)           /*!< 0x00000200 */
#define DSI_VCCCR_NUMC9               DSI_VCCCR_NUMC9_Msk
#define DSI_VCCCR_NUMC10_Pos          (10U)
#define DSI_VCCCR_NUMC10_Msk          (0x1UL << DSI_VCCCR_NUMC10_Pos)          /*!< 0x00000400 */
#define DSI_VCCCR_NUMC10              DSI_VCCCR_NUMC10_Msk
#define DSI_VCCCR_NUMC11_Pos          (11U)
#define DSI_VCCCR_NUMC11_Msk          (0x1UL << DSI_VCCCR_NUMC11_Pos)          /*!< 0x00000800 */
#define DSI_VCCCR_NUMC11              DSI_VCCCR_NUMC11_Msk
#define DSI_VCCCR_NUMC12_Pos          (12U)
#define DSI_VCCCR_NUMC12_Msk          (0x1UL << DSI_VCCCR_NUMC12_Pos)          /*!< 0x00001000 */
#define DSI_VCCCR_NUMC12              DSI_VCCCR_NUMC12_Msk

/*******************  Bit definition for DSI_VNPCCR register  *************/
#define DSI_VNPCCR_NPSIZE_Pos         (0U)
#define DSI_VNPCCR_NPSIZE_Msk         (0x1FFFUL << DSI_VNPCCR_NPSIZE_Pos)      /*!< 0x00001FFF */
#define DSI_VNPCCR_NPSIZE             DSI_VNPCCR_NPSIZE_Msk                    /*!< Number of Chunks */
#define DSI_VNPCCR_NPSIZE0_Pos        (0U)
#define DSI_VNPCCR_NPSIZE0_Msk        (0x1UL << DSI_VNPCCR_NPSIZE0_Pos)        /*!< 0x00000001 */
#define DSI_VNPCCR_NPSIZE0            DSI_VNPCCR_NPSIZE0_Msk
#define DSI_VNPCCR_NPSIZE1_Pos        (1U)
#define DSI_VNPCCR_NPSIZE1_Msk        (0x1UL << DSI_VNPCCR_NPSIZE1_Pos)        /*!< 0x00000002 */
#define DSI_VNPCCR_NPSIZE1            DSI_VNPCCR_NPSIZE1_Msk
#define DSI_VNPCCR_NPSIZE2_Pos        (2U)
#define DSI_VNPCCR_NPSIZE2_Msk        (0x1UL << DSI_VNPCCR_NPSIZE2_Pos)        /*!< 0x00000004 */
#define DSI_VNPCCR_NPSIZE2            DSI_VNPCCR_NPSIZE2_Msk
#define DSI_VNPCCR_NPSIZE3_Pos        (3U)
#define DSI_VNPCCR_NPSIZE3_Msk        (0x1UL << DSI_VNPCCR_NPSIZE3_Pos)        /*!< 0x00000008 */
#define DSI_VNPCCR_NPSIZE3            DSI_VNPCCR_NPSIZE3_Msk
#define DSI_VNPCCR_NPSIZE4_Pos        (4U)
#define DSI_VNPCCR_NPSIZE4_Msk        (0x1UL << DSI_VNPCCR_NPSIZE4_Pos)        /*!< 0x00000010 */
#define DSI_VNPCCR_NPSIZE4            DSI_VNPCCR_NPSIZE4_Msk
#define DSI_VNPCCR_NPSIZE5_Pos        (5U)
#define DSI_VNPCCR_NPSIZE5_Msk        (0x1UL << DSI_VNPCCR_NPSIZE5_Pos)        /*!< 0x00000020 */
#define DSI_VNPCCR_NPSIZE5            DSI_VNPCCR_NPSIZE5_Msk
#define DSI_VNPCCR_NPSIZE6_Pos        (6U)
#define DSI_VNPCCR_NPSIZE6_Msk        (0x1UL << DSI_VNPCCR_NPSIZE6_Pos)        /*!< 0x00000040 */
#define DSI_VNPCCR_NPSIZE6            DSI_VNPCCR_NPSIZE6_Msk
#define DSI_VNPCCR_NPSIZE7_Pos        (7U)
#define DSI_VNPCCR_NPSIZE7_Msk        (0x1UL << DSI_VNPCCR_NPSIZE7_Pos)        /*!< 0x00000080 */
#define DSI_VNPCCR_NPSIZE7            DSI_VNPCCR_NPSIZE7_Msk
#define DSI_VNPCCR_NPSIZE8_Pos        (8U)
#define DSI_VNPCCR_NPSIZE8_Msk        (0x1UL << DSI_VNPCCR_NPSIZE8_Pos)        /*!< 0x00000100 */
#define DSI_VNPCCR_NPSIZE8            DSI_VNPCCR_NPSIZE8_Msk
#define DSI_VNPCCR_NPSIZE9_Pos        (9U)
#define DSI_VNPCCR_NPSIZE9_Msk        (0x1UL << DSI_VNPCCR_NPSIZE9_Pos)        /*!< 0x00000200 */
#define DSI_VNPCCR_NPSIZE9            DSI_VNPCCR_NPSIZE9_Msk
#define DSI_VNPCCR_NPSIZE10_Pos       (10U)
#define DSI_VNPCCR_NPSIZE10_Msk       (0x1UL << DSI_VNPCCR_NPSIZE10_Pos)       /*!< 0x00000400 */
#define DSI_VNPCCR_NPSIZE10           DSI_VNPCCR_NPSIZE10_Msk
#define DSI_VNPCCR_NPSIZE11_Pos       (11U)
#define DSI_VNPCCR_NPSIZE11_Msk       (0x1UL << DSI_VNPCCR_NPSIZE11_Pos)       /*!< 0x00000800 */
#define DSI_VNPCCR_NPSIZE11           DSI_VNPCCR_NPSIZE11_Msk
#define DSI_VNPCCR_NPSIZE12_Pos       (12U)
#define DSI_VNPCCR_NPSIZE12_Msk       (0x1UL << DSI_VNPCCR_NPSIZE12_Pos)       /*!< 0x00001000 */
#define DSI_VNPCCR_NPSIZE12           DSI_VNPCCR_NPSIZE12_Msk

/*******************  Bit definition for DSI_VHSACCR register  ************/
#define DSI_VHSACCR_HSA_Pos           (0U)
#define DSI_VHSACCR_HSA_Msk           (0xFFFUL << DSI_VHSACCR_HSA_Pos)         /*!< 0x00000FFF */
#define DSI_VHSACCR_HSA               DSI_VHSACCR_HSA_Msk                      /*!< Horizontal Synchronism Active duration */
#define DSI_VHSACCR_HSA0_Pos          (0U)
#define DSI_VHSACCR_HSA0_Msk          (0x1UL << DSI_VHSACCR_HSA0_Pos)          /*!< 0x00000001 */
#define DSI_VHSACCR_HSA0              DSI_VHSACCR_HSA0_Msk
#define DSI_VHSACCR_HSA1_Pos          (1U)
#define DSI_VHSACCR_HSA1_Msk          (0x1UL << DSI_VHSACCR_HSA1_Pos)          /*!< 0x00000002 */
#define DSI_VHSACCR_HSA1              DSI_VHSACCR_HSA1_Msk
#define DSI_VHSACCR_HSA2_Pos          (2U)
#define DSI_VHSACCR_HSA2_Msk          (0x1UL << DSI_VHSACCR_HSA2_Pos)          /*!< 0x00000004 */
#define DSI_VHSACCR_HSA2              DSI_VHSACCR_HSA2_Msk
#define DSI_VHSACCR_HSA3_Pos          (3U)
#define DSI_VHSACCR_HSA3_Msk          (0x1UL << DSI_VHSACCR_HSA3_Pos)          /*!< 0x00000008 */
#define DSI_VHSACCR_HSA3              DSI_VHSACCR_HSA3_Msk
#define DSI_VHSACCR_HSA4_Pos          (4U)
#define DSI_VHSACCR_HSA4_Msk          (0x1UL << DSI_VHSACCR_HSA4_Pos)          /*!< 0x00000010 */
#define DSI_VHSACCR_HSA4              DSI_VHSACCR_HSA4_Msk
#define DSI_VHSACCR_HSA5_Pos          (5U)
#define DSI_VHSACCR_HSA5_Msk          (0x1UL << DSI_VHSACCR_HSA5_Pos)          /*!< 0x00000020 */
#define DSI_VHSACCR_HSA5              DSI_VHSACCR_HSA5_Msk
#define DSI_VHSACCR_HSA6_Pos          (6U)
#define DSI_VHSACCR_HSA6_Msk          (0x1UL << DSI_VHSACCR_HSA6_Pos)          /*!< 0x00000040 */
#define DSI_VHSACCR_HSA6              DSI_VHSACCR_HSA6_Msk
#define DSI_VHSACCR_HSA7_Pos          (7U)
#define DSI_VHSACCR_HSA7_Msk          (0x1UL << DSI_VHSACCR_HSA7_Pos)          /*!< 0x00000080 */
#define DSI_VHSACCR_HSA7              DSI_VHSACCR_HSA7_Msk
#define DSI_VHSACCR_HSA8_Pos          (8U)
#define DSI_VHSACCR_HSA8_Msk          (0x1UL << DSI_VHSACCR_HSA8_Pos)          /*!< 0x00000100 */
#define DSI_VHSACCR_HSA8              DSI_VHSACCR_HSA8_Msk
#define DSI_VHSACCR_HSA9_Pos          (9U)
#define DSI_VHSACCR_HSA9_Msk          (0x1UL << DSI_VHSACCR_HSA9_Pos)          /*!< 0x00000200 */
#define DSI_VHSACCR_HSA9              DSI_VHSACCR_HSA9_Msk
#define DSI_VHSACCR_HSA10_Pos         (10U)
#define DSI_VHSACCR_HSA10_Msk         (0x1UL << DSI_VHSACCR_HSA10_Pos)         /*!< 0x00000400 */
#define DSI_VHSACCR_HSA10             DSI_VHSACCR_HSA10_Msk
#define DSI_VHSACCR_HSA11_Pos         (11U)
#define DSI_VHSACCR_HSA11_Msk         (0x1UL << DSI_VHSACCR_HSA11_Pos)         /*!< 0x00000800 */
#define DSI_VHSACCR_HSA11             DSI_VHSACCR_HSA11_Msk

/*******************  Bit definition for DSI_VHBPCCR register  ************/
#define DSI_VHBPCCR_HBP_Pos           (0U)
#define DSI_VHBPCCR_HBP_Msk           (0xFFFUL << DSI_VHBPCCR_HBP_Pos)         /*!< 0x00000FFF */
#define DSI_VHBPCCR_HBP               DSI_VHBPCCR_HBP_Msk                      /*!< Horizontal Back-Porch duration */
#define DSI_VHBPCCR_HBP0_Pos          (0U)
#define DSI_VHBPCCR_HBP0_Msk          (0x1UL << DSI_VHBPCCR_HBP0_Pos)          /*!< 0x00000001 */
#define DSI_VHBPCCR_HBP0              DSI_VHBPCCR_HBP0_Msk
#define DSI_VHBPCCR_HBP1_Pos          (1U)
#define DSI_VHBPCCR_HBP1_Msk          (0x1UL << DSI_VHBPCCR_HBP1_Pos)          /*!< 0x00000002 */
#define DSI_VHBPCCR_HBP1              DSI_VHBPCCR_HBP1_Msk
#define DSI_VHBPCCR_HBP2_Pos          (2U)
#define DSI_VHBPCCR_HBP2_Msk          (0x1UL << DSI_VHBPCCR_HBP2_Pos)          /*!< 0x00000004 */
#define DSI_VHBPCCR_HBP2              DSI_VHBPCCR_HBP2_Msk
#define DSI_VHBPCCR_HBP3_Pos          (3U)
#define DSI_VHBPCCR_HBP3_Msk          (0x1UL << DSI_VHBPCCR_HBP3_Pos)          /*!< 0x00000008 */
#define DSI_VHBPCCR_HBP3              DSI_VHBPCCR_HBP3_Msk
#define DSI_VHBPCCR_HBP4_Pos          (4U)
#define DSI_VHBPCCR_HBP4_Msk          (0x1UL << DSI_VHBPCCR_HBP4_Pos)          /*!< 0x00000010 */
#define DSI_VHBPCCR_HBP4              DSI_VHBPCCR_HBP4_Msk
#define DSI_VHBPCCR_HBP5_Pos          (5U)
#define DSI_VHBPCCR_HBP5_Msk          (0x1UL << DSI_VHBPCCR_HBP5_Pos)          /*!< 0x00000020 */
#define DSI_VHBPCCR_HBP5              DSI_VHBPCCR_HBP5_Msk
#define DSI_VHBPCCR_HBP6_Pos          (6U)
#define DSI_VHBPCCR_HBP6_Msk          (0x1UL << DSI_VHBPCCR_HBP6_Pos)          /*!< 0x00000040 */
#define DSI_VHBPCCR_HBP6              DSI_VHBPCCR_HBP6_Msk
#define DSI_VHBPCCR_HBP7_Pos          (7U)
#define DSI_VHBPCCR_HBP7_Msk          (0x1UL << DSI_VHBPCCR_HBP7_Pos)          /*!< 0x00000080 */
#define DSI_VHBPCCR_HBP7              DSI_VHBPCCR_HBP7_Msk
#define DSI_VHBPCCR_HBP8_Pos          (8U)
#define DSI_VHBPCCR_HBP8_Msk          (0x1UL << DSI_VHBPCCR_HBP8_Pos)          /*!< 0x00000100 */
#define DSI_VHBPCCR_HBP8              DSI_VHBPCCR_HBP8_Msk
#define DSI_VHBPCCR_HBP9_Pos          (9U)
#define DSI_VHBPCCR_HBP9_Msk          (0x1UL << DSI_VHBPCCR_HBP9_Pos)          /*!< 0x00000200 */
#define DSI_VHBPCCR_HBP9              DSI_VHBPCCR_HBP9_Msk
#define DSI_VHBPCCR_HBP10_Pos         (10U)
#define DSI_VHBPCCR_HBP10_Msk         (0x1UL << DSI_VHBPCCR_HBP10_Pos)         /*!< 0x00000400 */
#define DSI_VHBPCCR_HBP10             DSI_VHBPCCR_HBP10_Msk
#define DSI_VHBPCCR_HBP11_Pos         (11U)
#define DSI_VHBPCCR_HBP11_Msk         (0x1UL << DSI_VHBPCCR_HBP11_Pos)         /*!< 0x00000800 */
#define DSI_VHBPCCR_HBP11             DSI_VHBPCCR_HBP11_Msk

/*******************  Bit definition for DSI_VLCCR register  **************/
#define DSI_VLCCR_HLINE_Pos           (0U)
#define DSI_VLCCR_HLINE_Msk           (0x7FFFUL << DSI_VLCCR_HLINE_Pos)        /*!< 0x00007FFF */
#define DSI_VLCCR_HLINE               DSI_VLCCR_HLINE_Msk                      /*!< Horizontal Line duration */
#define DSI_VLCCR_HLINE0_Pos          (0U)
#define DSI_VLCCR_HLINE0_Msk          (0x1UL << DSI_VLCCR_HLINE0_Pos)          /*!< 0x00000001 */
#define DSI_VLCCR_HLINE0              DSI_VLCCR_HLINE0_Msk
#define DSI_VLCCR_HLINE1_Pos          (1U)
#define DSI_VLCCR_HLINE1_Msk          (0x1UL << DSI_VLCCR_HLINE1_Pos)          /*!< 0x00000002 */
#define DSI_VLCCR_HLINE1              DSI_VLCCR_HLINE1_Msk
#define DSI_VLCCR_HLINE2_Pos          (2U)
#define DSI_VLCCR_HLINE2_Msk          (0x1UL << DSI_VLCCR_HLINE2_Pos)          /*!< 0x00000004 */
#define DSI_VLCCR_HLINE2              DSI_VLCCR_HLINE2_Msk
#define DSI_VLCCR_HLINE3_Pos          (3U)
#define DSI_VLCCR_HLINE3_Msk          (0x1UL << DSI_VLCCR_HLINE3_Pos)          /*!< 0x00000008 */
#define DSI_VLCCR_HLINE3              DSI_VLCCR_HLINE3_Msk
#define DSI_VLCCR_HLINE4_Pos          (4U)
#define DSI_VLCCR_HLINE4_Msk          (0x1UL << DSI_VLCCR_HLINE4_Pos)          /*!< 0x00000010 */
#define DSI_VLCCR_HLINE4              DSI_VLCCR_HLINE4_Msk
#define DSI_VLCCR_HLINE5_Pos          (5U)
#define DSI_VLCCR_HLINE5_Msk          (0x1UL << DSI_VLCCR_HLINE5_Pos)          /*!< 0x00000020 */
#define DSI_VLCCR_HLINE5              DSI_VLCCR_HLINE5_Msk
#define DSI_VLCCR_HLINE6_Pos          (6U)
#define DSI_VLCCR_HLINE6_Msk          (0x1UL << DSI_VLCCR_HLINE6_Pos)          /*!< 0x00000040 */
#define DSI_VLCCR_HLINE6              DSI_VLCCR_HLINE6_Msk
#define DSI_VLCCR_HLINE7_Pos          (7U)
#define DSI_VLCCR_HLINE7_Msk          (0x1UL << DSI_VLCCR_HLINE7_Pos)          /*!< 0x00000080 */
#define DSI_VLCCR_HLINE7              DSI_VLCCR_HLINE7_Msk
#define DSI_VLCCR_HLINE8_Pos          (8U)
#define DSI_VLCCR_HLINE8_Msk          (0x1UL << DSI_VLCCR_HLINE8_Pos)          /*!< 0x00000100 */
#define DSI_VLCCR_HLINE8              DSI_VLCCR_HLINE8_Msk
#define DSI_VLCCR_HLINE9_Pos          (9U)
#define DSI_VLCCR_HLINE9_Msk          (0x1UL << DSI_VLCCR_HLINE9_Pos)          /*!< 0x00000200 */
#define DSI_VLCCR_HLINE9              DSI_VLCCR_HLINE9_Msk
#define DSI_VLCCR_HLINE10_Pos         (10U)
#define DSI_VLCCR_HLINE10_Msk         (0x1UL << DSI_VLCCR_HLINE10_Pos)         /*!< 0x00000400 */
#define DSI_VLCCR_HLINE10             DSI_VLCCR_HLINE10_Msk
#define DSI_VLCCR_HLINE11_Pos         (11U)
#define DSI_VLCCR_HLINE11_Msk         (0x1UL << DSI_VLCCR_HLINE11_Pos)         /*!< 0x00000800 */
#define DSI_VLCCR_HLINE11             DSI_VLCCR_HLINE11_Msk
#define DSI_VLCCR_HLINE12_Pos         (12U)
#define DSI_VLCCR_HLINE12_Msk         (0x1UL << DSI_VLCCR_HLINE12_Pos)         /*!< 0x00001000 */
#define DSI_VLCCR_HLINE12             DSI_VLCCR_HLINE12_Msk
#define DSI_VLCCR_HLINE13_Pos         (13U)
#define DSI_VLCCR_HLINE13_Msk         (0x1UL << DSI_VLCCR_HLINE13_Pos)         /*!< 0x00002000 */
#define DSI_VLCCR_HLINE13             DSI_VLCCR_HLINE13_Msk
#define DSI_VLCCR_HLINE14_Pos         (14U)
#define DSI_VLCCR_HLINE14_Msk         (0x1UL << DSI_VLCCR_HLINE14_Pos)         /*!< 0x00004000 */
#define DSI_VLCCR_HLINE14             DSI_VLCCR_HLINE14_Msk

/*******************  Bit definition for DSI_VVSACCR register  ***************/
#define DSI_VVSACCR_VSA_Pos           (0U)
#define DSI_VVSACCR_VSA_Msk           (0x3FFUL << DSI_VVSACCR_VSA_Pos)         /*!< 0x000003FF */
#define DSI_VVSACCR_VSA               DSI_VVSACCR_VSA_Msk                      /*!< Vertical Synchronism Active duration */
#define DSI_VVSACCR_VSA0_Pos          (0U)
#define DSI_VVSACCR_VSA0_Msk          (0x1UL << DSI_VVSACCR_VSA0_Pos)          /*!< 0x00000001 */
#define DSI_VVSACCR_VSA0              DSI_VVSACCR_VSA0_Msk
#define DSI_VVSACCR_VSA1_Pos          (1U)
#define DSI_VVSACCR_VSA1_Msk          (0x1UL << DSI_VVSACCR_VSA1_Pos)          /*!< 0x00000002 */
#define DSI_VVSACCR_VSA1              DSI_VVSACCR_VSA1_Msk
#define DSI_VVSACCR_VSA2_Pos          (2U)
#define DSI_VVSACCR_VSA2_Msk          (0x1UL << DSI_VVSACCR_VSA2_Pos)          /*!< 0x00000004 */
#define DSI_VVSACCR_VSA2              DSI_VVSACCR_VSA2_Msk
#define DSI_VVSACCR_VSA3_Pos          (3U)
#define DSI_VVSACCR_VSA3_Msk          (0x1UL << DSI_VVSACCR_VSA3_Pos)          /*!< 0x00000008 */
#define DSI_VVSACCR_VSA3              DSI_VVSACCR_VSA3_Msk
#define DSI_VVSACCR_VSA4_Pos          (4U)
#define DSI_VVSACCR_VSA4_Msk          (0x1UL << DSI_VVSACCR_VSA4_Pos)          /*!< 0x00000010 */
#define DSI_VVSACCR_VSA4              DSI_VVSACCR_VSA4_Msk
#define DSI_VVSACCR_VSA5_Pos          (5U)
#define DSI_VVSACCR_VSA5_Msk          (0x1UL << DSI_VVSACCR_VSA5_Pos)          /*!< 0x00000020 */
#define DSI_VVSACCR_VSA5              DSI_VVSACCR_VSA5_Msk
#define DSI_VVSACCR_VSA6_Pos          (6U)
#define DSI_VVSACCR_VSA6_Msk          (0x1UL << DSI_VVSACCR_VSA6_Pos)          /*!< 0x00000040 */
#define DSI_VVSACCR_VSA6              DSI_VVSACCR_VSA6_Msk
#define DSI_VVSACCR_VSA7_Pos          (7U)
#define DSI_VVSACCR_VSA7_Msk          (0x1UL << DSI_VVSACCR_VSA7_Pos)          /*!< 0x00000080 */
#define DSI_VVSACCR_VSA7              DSI_VVSACCR_VSA7_Msk
#define DSI_VVSACCR_VSA8_Pos          (8U)
#define DSI_VVSACCR_VSA8_Msk          (0x1UL << DSI_VVSACCR_VSA8_Pos)          /*!< 0x00000100 */
#define DSI_VVSACCR_VSA8              DSI_VVSACCR_VSA8_Msk
#define DSI_VVSACCR_VSA9_Pos          (9U)
#define DSI_VVSACCR_VSA9_Msk          (0x1UL << DSI_VVSACCR_VSA9_Pos)          /*!< 0x00000200 */
#define DSI_VVSACCR_VSA9              DSI_VVSACCR_VSA9_Msk

/*******************  Bit definition for DSI_VVBPCCR register  ************/
#define DSI_VVBPCCR_VBP_Pos           (0U)
#define DSI_VVBPCCR_VBP_Msk           (0x3FFUL << DSI_VVBPCCR_VBP_Pos)         /*!< 0x000003FF */
#define DSI_VVBPCCR_VBP               DSI_VVBPCCR_VBP_Msk                      /*!< Vertical Back-Porch duration */
#define DSI_VVBPCCR_VBP0_Pos          (0U)
#define DSI_VVBPCCR_VBP0_Msk          (0x1UL << DSI_VVBPCCR_VBP0_Pos)          /*!< 0x00000001 */
#define DSI_VVBPCCR_VBP0              DSI_VVBPCCR_VBP0_Msk
#define DSI_VVBPCCR_VBP1_Pos          (1U)
#define DSI_VVBPCCR_VBP1_Msk          (0x1UL << DSI_VVBPCCR_VBP1_Pos)          /*!< 0x00000002 */
#define DSI_VVBPCCR_VBP1              DSI_VVBPCCR_VBP1_Msk
#define DSI_VVBPCCR_VBP2_Pos          (2U)
#define DSI_VVBPCCR_VBP2_Msk          (0x1UL << DSI_VVBPCCR_VBP2_Pos)          /*!< 0x00000004 */
#define DSI_VVBPCCR_VBP2              DSI_VVBPCCR_VBP2_Msk
#define DSI_VVBPCCR_VBP3_Pos          (3U)
#define DSI_VVBPCCR_VBP3_Msk          (0x1UL << DSI_VVBPCCR_VBP3_Pos)          /*!< 0x00000008 */
#define DSI_VVBPCCR_VBP3              DSI_VVBPCCR_VBP3_Msk
#define DSI_VVBPCCR_VBP4_Pos          (4U)
#define DSI_VVBPCCR_VBP4_Msk          (0x1UL << DSI_VVBPCCR_VBP4_Pos)          /*!< 0x00000010 */
#define DSI_VVBPCCR_VBP4              DSI_VVBPCCR_VBP4_Msk
#define DSI_VVBPCCR_VBP5_Pos          (5U)
#define DSI_VVBPCCR_VBP5_Msk          (0x1UL << DSI_VVBPCCR_VBP5_Pos)          /*!< 0x00000020 */
#define DSI_VVBPCCR_VBP5              DSI_VVBPCCR_VBP5_Msk
#define DSI_VVBPCCR_VBP6_Pos          (6U)
#define DSI_VVBPCCR_VBP6_Msk          (0x1UL << DSI_VVBPCCR_VBP6_Pos)          /*!< 0x00000040 */
#define DSI_VVBPCCR_VBP6              DSI_VVBPCCR_VBP6_Msk
#define DSI_VVBPCCR_VBP7_Pos          (7U)
#define DSI_VVBPCCR_VBP7_Msk          (0x1UL << DSI_VVBPCCR_VBP7_Pos)          /*!< 0x00000080 */
#define DSI_VVBPCCR_VBP7              DSI_VVBPCCR_VBP7_Msk
#define DSI_VVBPCCR_VBP8_Pos          (8U)
#define DSI_VVBPCCR_VBP8_Msk          (0x1UL << DSI_VVBPCCR_VBP8_Pos)          /*!< 0x00000100 */
#define DSI_VVBPCCR_VBP8              DSI_VVBPCCR_VBP8_Msk
#define DSI_VVBPCCR_VBP9_Pos          (9U)
#define DSI_VVBPCCR_VBP9_Msk          (0x1UL << DSI_VVBPCCR_VBP9_Pos)          /*!< 0x00000200 */
#define DSI_VVBPCCR_VBP9              DSI_VVBPCCR_VBP9_Msk

/*******************  Bit definition for DSI_VVFPCCR register  ************/
#define DSI_VVFPCCR_VFP_Pos           (0U)
#define DSI_VVFPCCR_VFP_Msk           (0x3FFUL << DSI_VVFPCCR_VFP_Pos)         /*!< 0x000003FF */
#define DSI_VVFPCCR_VFP               DSI_VVFPCCR_VFP_Msk                      /*!< Vertical Front-Porch duration */
#define DSI_VVFPCCR_VFP0_Pos          (0U)
#define DSI_VVFPCCR_VFP0_Msk          (0x1UL << DSI_VVFPCCR_VFP0_Pos)          /*!< 0x00000001 */
#define DSI_VVFPCCR_VFP0              DSI_VVFPCCR_VFP0_Msk
#define DSI_VVFPCCR_VFP1_Pos          (1U)
#define DSI_VVFPCCR_VFP1_Msk          (0x1UL << DSI_VVFPCCR_VFP1_Pos)          /*!< 0x00000002 */
#define DSI_VVFPCCR_VFP1              DSI_VVFPCCR_VFP1_Msk
#define DSI_VVFPCCR_VFP2_Pos          (2U)
#define DSI_VVFPCCR_VFP2_Msk          (0x1UL << DSI_VVFPCCR_VFP2_Pos)          /*!< 0x00000004 */
#define DSI_VVFPCCR_VFP2              DSI_VVFPCCR_VFP2_Msk
#define DSI_VVFPCCR_VFP3_Pos          (3U)
#define DSI_VVFPCCR_VFP3_Msk          (0x1UL << DSI_VVFPCCR_VFP3_Pos)          /*!< 0x00000008 */
#define DSI_VVFPCCR_VFP3              DSI_VVFPCCR_VFP3_Msk
#define DSI_VVFPCCR_VFP4_Pos          (4U)
#define DSI_VVFPCCR_VFP4_Msk          (0x1UL << DSI_VVFPCCR_VFP4_Pos)          /*!< 0x00000010 */
#define DSI_VVFPCCR_VFP4              DSI_VVFPCCR_VFP4_Msk
#define DSI_VVFPCCR_VFP5_Pos          (5U)
#define DSI_VVFPCCR_VFP5_Msk          (0x1UL << DSI_VVFPCCR_VFP5_Pos)          /*!< 0x00000020 */
#define DSI_VVFPCCR_VFP5              DSI_VVFPCCR_VFP5_Msk
#define DSI_VVFPCCR_VFP6_Pos          (6U)
#define DSI_VVFPCCR_VFP6_Msk          (0x1UL << DSI_VVFPCCR_VFP6_Pos)          /*!< 0x00000040 */
#define DSI_VVFPCCR_VFP6              DSI_VVFPCCR_VFP6_Msk
#define DSI_VVFPCCR_VFP7_Pos          (7U)
#define DSI_VVFPCCR_VFP7_Msk          (0x1UL << DSI_VVFPCCR_VFP7_Pos)          /*!< 0x00000080 */
#define DSI_VVFPCCR_VFP7              DSI_VVFPCCR_VFP7_Msk
#define DSI_VVFPCCR_VFP8_Pos          (8U)
#define DSI_VVFPCCR_VFP8_Msk          (0x1UL << DSI_VVFPCCR_VFP8_Pos)          /*!< 0x00000100 */
#define DSI_VVFPCCR_VFP8              DSI_VVFPCCR_VFP8_Msk
#define DSI_VVFPCCR_VFP9_Pos          (9U)
#define DSI_VVFPCCR_VFP9_Msk          (0x1UL << DSI_VVFPCCR_VFP9_Pos)          /*!< 0x00000200 */
#define DSI_VVFPCCR_VFP9              DSI_VVFPCCR_VFP9_Msk

/*******************  Bit definition for DSI_VVACCR register  *************/
#define DSI_VVACCR_VA_Pos             (0U)
#define DSI_VVACCR_VA_Msk             (0x3FFFUL << DSI_VVACCR_VA_Pos)          /*!< 0x00003FFF */
#define DSI_VVACCR_VA                 DSI_VVACCR_VA_Msk                        /*!< Vertical Active duration */
#define DSI_VVACCR_VA0_Pos            (0U)
#define DSI_VVACCR_VA0_Msk            (0x1UL << DSI_VVACCR_VA0_Pos)            /*!< 0x00000001 */
#define DSI_VVACCR_VA0                DSI_VVACCR_VA0_Msk
#define DSI_VVACCR_VA1_Pos            (1U)
#define DSI_VVACCR_VA1_Msk            (0x1UL << DSI_VVACCR_VA1_Pos)            /*!< 0x00000002 */
#define DSI_VVACCR_VA1                DSI_VVACCR_VA1_Msk
#define DSI_VVACCR_VA2_Pos            (2U)
#define DSI_VVACCR_VA2_Msk            (0x1UL << DSI_VVACCR_VA2_Pos)            /*!< 0x00000004 */
#define DSI_VVACCR_VA2                DSI_VVACCR_VA2_Msk
#define DSI_VVACCR_VA3_Pos            (3U)
#define DSI_VVACCR_VA3_Msk            (0x1UL << DSI_VVACCR_VA3_Pos)            /*!< 0x00000008 */
#define DSI_VVACCR_VA3                DSI_VVACCR_VA3_Msk
#define DSI_VVACCR_VA4_Pos            (4U)
#define DSI_VVACCR_VA4_Msk            (0x1UL << DSI_VVACCR_VA4_Pos)            /*!< 0x00000010 */
#define DSI_VVACCR_VA4                DSI_VVACCR_VA4_Msk
#define DSI_VVACCR_VA5_Pos            (5U)
#define DSI_VVACCR_VA5_Msk            (0x1UL << DSI_VVACCR_VA5_Pos)            /*!< 0x00000020 */
#define DSI_VVACCR_VA5                DSI_VVACCR_VA5_Msk
#define DSI_VVACCR_VA6_Pos            (6U)
#define DSI_VVACCR_VA6_Msk            (0x1UL << DSI_VVACCR_VA6_Pos)            /*!< 0x00000040 */
#define DSI_VVACCR_VA6                DSI_VVACCR_VA6_Msk
#define DSI_VVACCR_VA7_Pos            (7U)
#define DSI_VVACCR_VA7_Msk            (0x1UL << DSI_VVACCR_VA7_Pos)            /*!< 0x00000080 */
#define DSI_VVACCR_VA7                DSI_VVACCR_VA7_Msk
#define DSI_VVACCR_VA8_Pos            (8U)
#define DSI_VVACCR_VA8_Msk            (0x1UL << DSI_VVACCR_VA8_Pos)            /*!< 0x00000100 */
#define DSI_VVACCR_VA8                DSI_VVACCR_VA8_Msk
#define DSI_VVACCR_VA9_Pos            (9U)
#define DSI_VVACCR_VA9_Msk            (0x1UL << DSI_VVACCR_VA9_Pos)            /*!< 0x00000200 */
#define DSI_VVACCR_VA9                DSI_VVACCR_VA9_Msk
#define DSI_VVACCR_VA10_Pos           (10U)
#define DSI_VVACCR_VA10_Msk           (0x1UL << DSI_VVACCR_VA10_Pos)           /*!< 0x00000400 */
#define DSI_VVACCR_VA10               DSI_VVACCR_VA10_Msk
#define DSI_VVACCR_VA11_Pos           (11U)
#define DSI_VVACCR_VA11_Msk           (0x1UL << DSI_VVACCR_VA11_Pos)           /*!< 0x00000800 */
#define DSI_VVACCR_VA11               DSI_VVACCR_VA11_Msk
#define DSI_VVACCR_VA12_Pos           (12U)
#define DSI_VVACCR_VA12_Msk           (0x1UL << DSI_VVACCR_VA12_Pos)           /*!< 0x00001000 */
#define DSI_VVACCR_VA12               DSI_VVACCR_VA12_Msk
#define DSI_VVACCR_VA13_Pos           (13U)
#define DSI_VVACCR_VA13_Msk           (0x1UL << DSI_VVACCR_VA13_Pos)           /*!< 0x00002000 */
#define DSI_VVACCR_VA13               DSI_VVACCR_VA13_Msk

/*******************  Bit definition for DSI_TDCCR register  **************/
#define DSI_TDCCR_3DM                 ((uint32_t)0x00000003U)                  /*!< 3D Mode */
#define DSI_TDCCR_3DM0                ((uint32_t)0x00000001U)
#define DSI_TDCCR_3DM1                ((uint32_t)0x00000002U)

#define DSI_TDCCR_3DF                 ((uint32_t)0x0000000CU)                  /*!< 3D Format */
#define DSI_TDCCR_3DF0                ((uint32_t)0x00000004U)
#define DSI_TDCCR_3DF1                ((uint32_t)0x00000008U)

#define DSI_TDCCR_SVS_Pos             (4U)
#define DSI_TDCCR_SVS_Msk             (0x1UL << DSI_TDCCR_SVS_Pos)             /*!< 0x00000010 */
#define DSI_TDCCR_SVS                 DSI_TDCCR_SVS_Msk                        /*!< Second VSYNC */
#define DSI_TDCCR_RF_Pos              (5U)
#define DSI_TDCCR_RF_Msk              (0x1UL << DSI_TDCCR_RF_Pos)              /*!< 0x00000020 */
#define DSI_TDCCR_RF                  DSI_TDCCR_RF_Msk                         /*!< Right First */
#define DSI_TDCCR_S3DC_Pos            (16U)
#define DSI_TDCCR_S3DC_Msk            (0x1UL << DSI_TDCCR_S3DC_Pos)            /*!< 0x00010000 */
#define DSI_TDCCR_S3DC                DSI_TDCCR_S3DC_Msk                       /*!< Send 3D Control */

/*******************  Bit definition for DSI_WCFGR register  ***************/
#define DSI_WCFGR_DSIM_Pos            (0U)
#define DSI_WCFGR_DSIM_Msk            (0x1UL << DSI_WCFGR_DSIM_Pos)            /*!< 0x00000001 */
#define DSI_WCFGR_DSIM                DSI_WCFGR_DSIM_Msk                       /*!< DSI Mode */
#define DSI_WCFGR_COLMUX_Pos          (1U)
#define DSI_WCFGR_COLMUX_Msk          (0x7UL << DSI_WCFGR_COLMUX_Pos)          /*!< 0x0000000E */
#define DSI_WCFGR_COLMUX              DSI_WCFGR_COLMUX_Msk                     /*!< Color Multiplexing */
#define DSI_WCFGR_COLMUX0_Pos         (1U)
#define DSI_WCFGR_COLMUX0_Msk         (0x1UL << DSI_WCFGR_COLMUX0_Pos)         /*!< 0x00000002 */
#define DSI_WCFGR_COLMUX0             DSI_WCFGR_COLMUX0_Msk
#define DSI_WCFGR_COLMUX1_Pos         (2U)
#define DSI_WCFGR_COLMUX1_Msk         (0x1UL << DSI_WCFGR_COLMUX1_Pos)         /*!< 0x00000004 */
#define DSI_WCFGR_COLMUX1             DSI_WCFGR_COLMUX1_Msk
#define DSI_WCFGR_COLMUX2_Pos         (3U)
#define DSI_WCFGR_COLMUX2_Msk         (0x1UL << DSI_WCFGR_COLMUX2_Pos)         /*!< 0x00000008 */
#define DSI_WCFGR_COLMUX2             DSI_WCFGR_COLMUX2_Msk

#define DSI_WCFGR_TESRC_Pos           (4U)
#define DSI_WCFGR_TESRC_Msk           (0x1UL << DSI_WCFGR_TESRC_Pos)           /*!< 0x00000010 */
#define DSI_WCFGR_TESRC               DSI_WCFGR_TESRC_Msk                      /*!< Tearing Effect Source */
#define DSI_WCFGR_TEPOL_Pos           (5U)
#define DSI_WCFGR_TEPOL_Msk           (0x1UL << DSI_WCFGR_TEPOL_Pos)           /*!< 0x00000020 */
#define DSI_WCFGR_TEPOL               DSI_WCFGR_TEPOL_Msk                      /*!< Tearing Effect Polarity */
#define DSI_WCFGR_AR_Pos              (6U)
#define DSI_WCFGR_AR_Msk              (0x1UL << DSI_WCFGR_AR_Pos)              /*!< 0x00000040 */
#define DSI_WCFGR_AR                  DSI_WCFGR_AR_Msk                         /*!< Automatic Refresh */
#define DSI_WCFGR_VSPOL_Pos           (7U)
#define DSI_WCFGR_VSPOL_Msk           (0x1UL << DSI_WCFGR_VSPOL_Pos)           /*!< 0x00000080 */
#define DSI_WCFGR_VSPOL               DSI_WCFGR_VSPOL_Msk                      /*!< VSync Polarity */

/*******************  Bit definition for DSI_WCR register  *****************/
#define DSI_WCR_COLM_Pos              (0U)
#define DSI_WCR_COLM_Msk              (0x1UL << DSI_WCR_COLM_Pos)              /*!< 0x00000001 */
#define DSI_WCR_COLM                  DSI_WCR_COLM_Msk                         /*!< Color Mode */
#define DSI_WCR_SHTDN_Pos             (1U)
#define DSI_WCR_SHTDN_Msk             (0x1UL << DSI_WCR_SHTDN_Pos)             /*!< 0x00000002 */
#define DSI_WCR_SHTDN                 DSI_WCR_SHTDN_Msk                        /*!< Shutdown */
#define DSI_WCR_LTDCEN_Pos            (2U)
#define DSI_WCR_LTDCEN_Msk            (0x1UL << DSI_WCR_LTDCEN_Pos)            /*!< 0x00000004 */
#define DSI_WCR_LTDCEN                DSI_WCR_LTDCEN_Msk                       /*!< LTDC Enable */
#define DSI_WCR_DSIEN_Pos             (3U)
#define DSI_WCR_DSIEN_Msk             (0x1UL << DSI_WCR_DSIEN_Pos)             /*!< 0x00000008 */
#define DSI_WCR_DSIEN                 DSI_WCR_DSIEN_Msk                        /*!< DSI Enable */

/*******************  Bit definition for DSI_WIER register  ****************/
#define DSI_WIER_TEIE_Pos             (0U)
#define DSI_WIER_TEIE_Msk             (0x1UL << DSI_WIER_TEIE_Pos)             /*!< 0x00000001 */
#define DSI_WIER_TEIE                 DSI_WIER_TEIE_Msk                        /*!< Tearing Effect Interrupt Enable */
#define DSI_WIER_ERIE_Pos             (1U)
#define DSI_WIER_ERIE_Msk             (0x1UL << DSI_WIER_ERIE_Pos)             /*!< 0x00000002 */
#define DSI_WIER_ERIE                 DSI_WIER_ERIE_Msk                        /*!< End of Refresh Interrupt Enable */
#define DSI_WIER_PLLLIE_Pos           (9U)
#define DSI_WIER_PLLLIE_Msk           (0x1UL << DSI_WIER_PLLLIE_Pos)           /*!< 0x00000200 */
#define DSI_WIER_PLLLIE               DSI_WIER_PLLLIE_Msk                      /*!< PLL Lock Interrupt Enable */
#define DSI_WIER_PLLUIE_Pos           (10U)
#define DSI_WIER_PLLUIE_Msk           (0x1UL << DSI_WIER_PLLUIE_Pos)           /*!< 0x00000400 */
#define DSI_WIER_PLLUIE               DSI_WIER_PLLUIE_Msk                      /*!< PLL Unlock Interrupt Enable */
#define DSI_WIER_RRIE_Pos             (13U)
#define DSI_WIER_RRIE_Msk             (0x1UL << DSI_WIER_RRIE_Pos)             /*!< 0x00002000 */
#define DSI_WIER_RRIE                 DSI_WIER_RRIE_Msk                        /*!< Regulator Ready Interrupt Enable */

/*******************  Bit definition for DSI_WISR register  ****************/
#define DSI_WISR_TEIF_Pos             (0U)
#define DSI_WISR_TEIF_Msk             (0x1UL << DSI_WISR_TEIF_Pos)             /*!< 0x00000001 */
#define DSI_WISR_TEIF                 DSI_WISR_TEIF_Msk                        /*!< Tearing Effect Interrupt Flag */
#define DSI_WISR_ERIF_Pos             (1U)
#define DSI_WISR_ERIF_Msk             (0x1UL << DSI_WISR_ERIF_Pos)             /*!< 0x00000002 */
#define DSI_WISR_ERIF                 DSI_WISR_ERIF_Msk                        /*!< End of Refresh Interrupt Flag */
#define DSI_WISR_BUSY_Pos             (2U)
#define DSI_WISR_BUSY_Msk             (0x1UL << DSI_WISR_BUSY_Pos)             /*!< 0x00000004 */
#define DSI_WISR_BUSY                 DSI_WISR_BUSY_Msk                        /*!< Busy Flag */
#define DSI_WISR_PLLLS_Pos            (8U)
#define DSI_WISR_PLLLS_Msk            (0x1UL << DSI_WISR_PLLLS_Pos)            /*!< 0x00000100 */
#define DSI_WISR_PLLLS                DSI_WISR_PLLLS_Msk                       /*!< PLL Lock Status */
#define DSI_WISR_PLLLIF_Pos           (9U)
#define DSI_WISR_PLLLIF_Msk           (0x1UL << DSI_WISR_PLLLIF_Pos)           /*!< 0x00000200 */
#define DSI_WISR_PLLLIF               DSI_WISR_PLLLIF_Msk                      /*!< PLL Lock Interrupt Flag */
#define DSI_WISR_PLLUIF_Pos           (10U)
#define DSI_WISR_PLLUIF_Msk           (0x1UL << DSI_WISR_PLLUIF_Pos)           /*!< 0x00000400 */
#define DSI_WISR_PLLUIF               DSI_WISR_PLLUIF_Msk                      /*!< PLL Unlock Interrupt Flag */
#define DSI_WISR_RRS_Pos              (12U)
#define DSI_WISR_RRS_Msk              (0x1UL << DSI_WISR_RRS_Pos)              /*!< 0x00001000 */
#define DSI_WISR_RRS                  DSI_WISR_RRS_Msk                         /*!< Regulator Ready Flag */
#define DSI_WISR_RRIF_Pos             (13U)
#define DSI_WISR_RRIF_Msk             (0x1UL << DSI_WISR_RRIF_Pos)             /*!< 0x00002000 */
#define DSI_WISR_RRIF                 DSI_WISR_RRIF_Msk                        /*!< Regulator Ready Interrupt Flag */

/*******************  Bit definition for DSI_WIFCR register  ***************/
#define DSI_WIFCR_CTEIF_Pos           (0U)
#define DSI_WIFCR_CTEIF_Msk           (0x1UL << DSI_WIFCR_CTEIF_Pos)           /*!< 0x00000001 */
#define DSI_WIFCR_CTEIF               DSI_WIFCR_CTEIF_Msk                      /*!< Clear Tearing Effect Interrupt Flag */
#define DSI_WIFCR_CERIF_Pos           (1U)
#define DSI_WIFCR_CERIF_Msk           (0x1UL << DSI_WIFCR_CERIF_Pos)           /*!< 0x00000002 */
#define DSI_WIFCR_CERIF               DSI_WIFCR_CERIF_Msk                      /*!< Clear End of Refresh Interrupt Flag */
#define DSI_WIFCR_CPLLLIF_Pos         (9U)
#define DSI_WIFCR_CPLLLIF_Msk         (0x1UL << DSI_WIFCR_CPLLLIF_Pos)         /*!< 0x00000200 */
#define DSI_WIFCR_CPLLLIF             DSI_WIFCR_CPLLLIF_Msk                    /*!< Clear PLL Lock Interrupt Flag */
#define DSI_WIFCR_CPLLUIF_Pos         (10U)
#define DSI_WIFCR_CPLLUIF_Msk         (0x1UL << DSI_WIFCR_CPLLUIF_Pos)         /*!< 0x00000400 */
#define DSI_WIFCR_CPLLUIF             DSI_WIFCR_CPLLUIF_Msk                    /*!< Clear PLL Unlock Interrupt Flag */
#define DSI_WIFCR_CRRIF_Pos           (13U)
#define DSI_WIFCR_CRRIF_Msk           (0x1UL << DSI_WIFCR_CRRIF_Pos)           /*!< 0x00002000 */
#define DSI_WIFCR_CRRIF               DSI_WIFCR_CRRIF_Msk                      /*!< Clear Regulator Ready Interrupt Flag */

/*******************  Bit definition for DSI_WPCR0 register  ***************/
#define DSI_WPCR0_UIX4_Pos            (0U)
#define DSI_WPCR0_UIX4_Msk            (0x3FUL << DSI_WPCR0_UIX4_Pos)           /*!< 0x0000003F */
#define DSI_WPCR0_UIX4                DSI_WPCR0_UIX4_Msk                       /*!< Unit Interval multiplied by 4 */
#define DSI_WPCR0_UIX4_0              (0x01UL << DSI_WPCR0_UIX4_Pos)            /*!< 0x00000001 */
#define DSI_WPCR0_UIX4_1              (0x02UL << DSI_WPCR0_UIX4_Pos)            /*!< 0x00000002 */
#define DSI_WPCR0_UIX4_2              (0x04UL << DSI_WPCR0_UIX4_Pos)            /*!< 0x00000004 */
#define DSI_WPCR0_UIX4_3              (0x08UL << DSI_WPCR0_UIX4_Pos)            /*!< 0x00000008 */
#define DSI_WPCR0_UIX4_4              (0x10UL << DSI_WPCR0_UIX4_Pos)            /*!< 0x00000010 */
#define DSI_WPCR0_UIX4_5              (0x20UL << DSI_WPCR0_UIX4_Pos)            /*!< 0x00000020 */

#define DSI_WPCR0_SWCL_Pos            (6U)
#define DSI_WPCR0_SWCL_Msk            (0x1UL << DSI_WPCR0_SWCL_Pos)            /*!< 0x00000040 */
#define DSI_WPCR0_SWCL                DSI_WPCR0_SWCL_Msk                       /*!< Swap pins on clock lane */
#define DSI_WPCR0_SWDL0_Pos           (7U)
#define DSI_WPCR0_SWDL0_Msk           (0x1UL << DSI_WPCR0_SWDL0_Pos)           /*!< 0x00000080 */
#define DSI_WPCR0_SWDL0               DSI_WPCR0_SWDL0_Msk                      /*!< Swap pins on data lane 1 */
#define DSI_WPCR0_SWDL1_Pos           (8U)
#define DSI_WPCR0_SWDL1_Msk           (0x1UL << DSI_WPCR0_SWDL1_Pos)           /*!< 0x00000100 */
#define DSI_WPCR0_SWDL1               DSI_WPCR0_SWDL1_Msk                      /*!< Swap pins on data lane 2 */
#define DSI_WPCR0_HSICL_Pos           (9U)
#define DSI_WPCR0_HSICL_Msk           (0x1UL << DSI_WPCR0_HSICL_Pos)           /*!< 0x00000200 */
#define DSI_WPCR0_HSICL               DSI_WPCR0_HSICL_Msk                      /*!< Invert the high-speed data signal on clock lane */
#define DSI_WPCR0_HSIDL0_Pos          (10U)
#define DSI_WPCR0_HSIDL0_Msk          (0x1UL << DSI_WPCR0_HSIDL0_Pos)          /*!< 0x00000400 */
#define DSI_WPCR0_HSIDL0              DSI_WPCR0_HSIDL0_Msk                     /*!< Invert the high-speed data signal on lane 1 */
#define DSI_WPCR0_HSIDL1_Pos          (11U)
#define DSI_WPCR0_HSIDL1_Msk          (0x1UL << DSI_WPCR0_HSIDL1_Pos)          /*!< 0x00000800 */
#define DSI_WPCR0_HSIDL1              DSI_WPCR0_HSIDL1_Msk                     /*!< Invert the high-speed data signal on lane 2 */
#define DSI_WPCR0_FTXSMCL_Pos         (12U)
#define DSI_WPCR0_FTXSMCL_Msk         (0x1UL << DSI_WPCR0_FTXSMCL_Pos)         /*!< 0x00001000 */
#define DSI_WPCR0_FTXSMCL             DSI_WPCR0_FTXSMCL_Msk                    /*!< Force clock lane in TX stop mode */
#define DSI_WPCR0_FTXSMDL_Pos         (13U)
#define DSI_WPCR0_FTXSMDL_Msk         (0x1UL << DSI_WPCR0_FTXSMDL_Pos)         /*!< 0x00002000 */
#define DSI_WPCR0_FTXSMDL             DSI_WPCR0_FTXSMDL_Msk                    /*!< Force data lanes in TX stop mode */
#define DSI_WPCR0_CDOFFDL_Pos         (14U)
#define DSI_WPCR0_CDOFFDL_Msk         (0x1UL << DSI_WPCR0_CDOFFDL_Pos)         /*!< 0x00004000 */
#define DSI_WPCR0_CDOFFDL             DSI_WPCR0_CDOFFDL_Msk                    /*!< Contention detection OFF */
#define DSI_WPCR0_TDDL_Pos            (16U)
#define DSI_WPCR0_TDDL_Msk            (0x1UL << DSI_WPCR0_TDDL_Pos)            /*!< 0x00010000 */
#define DSI_WPCR0_TDDL                DSI_WPCR0_TDDL_Msk                       /*!< Turn Disable Data Lanes */
#define DSI_WPCR0_PDEN_Pos            (18U)
#define DSI_WPCR0_PDEN_Msk            (0x1UL << DSI_WPCR0_PDEN_Pos)            /*!< 0x00040000 */
#define DSI_WPCR0_PDEN                DSI_WPCR0_PDEN_Msk                       /*!< Pull-Down Enable */
#define DSI_WPCR0_TCLKPREPEN_Pos      (19U)
#define DSI_WPCR0_TCLKPREPEN_Msk      (0x1UL << DSI_WPCR0_TCLKPREPEN_Pos)      /*!< 0x00080000 */
#define DSI_WPCR0_TCLKPREPEN          DSI_WPCR0_TCLKPREPEN_Msk                 /*!< Timer for t-CLKPREP Enable */
#define DSI_WPCR0_TCLKZEROEN_Pos      (20U)
#define DSI_WPCR0_TCLKZEROEN_Msk      (0x1UL << DSI_WPCR0_TCLKZEROEN_Pos)      /*!< 0x00100000 */
#define DSI_WPCR0_TCLKZEROEN          DSI_WPCR0_TCLKZEROEN_Msk                 /*!< Timer for t-CLKZERO Enable */
#define DSI_WPCR0_THSPREPEN_Pos       (21U)
#define DSI_WPCR0_THSPREPEN_Msk       (0x1UL << DSI_WPCR0_THSPREPEN_Pos)       /*!< 0x00200000 */
#define DSI_WPCR0_THSPREPEN           DSI_WPCR0_THSPREPEN_Msk                  /*!< Timer for t-HSPREP Enable */
#define DSI_WPCR0_THSTRAILEN_Pos      (22U)
#define DSI_WPCR0_THSTRAILEN_Msk      (0x1UL << DSI_WPCR0_THSTRAILEN_Pos)      /*!< 0x00400000 */
#define DSI_WPCR0_THSTRAILEN          DSI_WPCR0_THSTRAILEN_Msk                 /*!< Timer for t-HSTRAIL Enable */
#define DSI_WPCR0_THSZEROEN_Pos       (23U)
#define DSI_WPCR0_THSZEROEN_Msk       (0x1UL << DSI_WPCR0_THSZEROEN_Pos)       /*!< 0x00800000 */
#define DSI_WPCR0_THSZEROEN           DSI_WPCR0_THSZEROEN_Msk                  /*!< Timer for t-HSZERO Enable */
#define DSI_WPCR0_TLPXDEN_Pos         (24U)
#define DSI_WPCR0_TLPXDEN_Msk         (0x1UL << DSI_WPCR0_TLPXDEN_Pos)         /*!< 0x01000000 */
#define DSI_WPCR0_TLPXDEN             DSI_WPCR0_TLPXDEN_Msk                    /*!< Timer for t-LPXD Enable */
#define DSI_WPCR0_THSEXITEN_Pos       (25U)
#define DSI_WPCR0_THSEXITEN_Msk       (0x1UL << DSI_WPCR0_THSEXITEN_Pos)       /*!< 0x02000000 */
#define DSI_WPCR0_THSEXITEN           DSI_WPCR0_THSEXITEN_Msk                  /*!< Timer for t-HSEXIT Enable */
#define DSI_WPCR0_TLPXCEN_Pos         (26U)
#define DSI_WPCR0_TLPXCEN_Msk         (0x1UL << DSI_WPCR0_TLPXCEN_Pos)         /*!< 0x04000000 */
#define DSI_WPCR0_TLPXCEN             DSI_WPCR0_TLPXCEN_Msk                    /*!< Timer for t-LPXC Enable */
#define DSI_WPCR0_TCLKPOSTEN_Pos      (27U)
#define DSI_WPCR0_TCLKPOSTEN_Msk      (0x1UL << DSI_WPCR0_TCLKPOSTEN_Pos)      /*!< 0x08000000 */
#define DSI_WPCR0_TCLKPOSTEN          DSI_WPCR0_TCLKPOSTEN_Msk                 /*!< Timer for t-CLKPOST Enable */

/*******************  Bit definition for DSI_WPCR1 register  ***************/
#define DSI_WPCR1_HSTXDCL_Pos         (0U)
#define DSI_WPCR1_HSTXDCL_Msk         (0x3UL << DSI_WPCR1_HSTXDCL_Pos)         /*!< 0x00000003 */
#define DSI_WPCR1_HSTXDCL             DSI_WPCR1_HSTXDCL_Msk                    /*!< High-Speed Transmission Delay on Clock Lane */
#define DSI_WPCR1_HSTXDCL0_Pos        (0U)
#define DSI_WPCR1_HSTXDCL0_Msk        (0x1UL << DSI_WPCR1_HSTXDCL0_Pos)        /*!< 0x00000001 */
#define DSI_WPCR1_HSTXDCL0            DSI_WPCR1_HSTXDCL0_Msk
#define DSI_WPCR1_HSTXDCL1_Pos        (1U)
#define DSI_WPCR1_HSTXDCL1_Msk        (0x1UL << DSI_WPCR1_HSTXDCL1_Pos)        /*!< 0x00000002 */
#define DSI_WPCR1_HSTXDCL1            DSI_WPCR1_HSTXDCL1_Msk

#define DSI_WPCR1_HSTXDDL_Pos         (2U)
#define DSI_WPCR1_HSTXDDL_Msk         (0x3UL << DSI_WPCR1_HSTXDDL_Pos)         /*!< 0x0000000C */
#define DSI_WPCR1_HSTXDDL             DSI_WPCR1_HSTXDDL_Msk                    /*!< High-Speed Transmission Delay on Data Lane */
#define DSI_WPCR1_HSTXDDL0_Pos        (2U)
#define DSI_WPCR1_HSTXDDL0_Msk        (0x1UL << DSI_WPCR1_HSTXDDL0_Pos)        /*!< 0x00000004 */
#define DSI_WPCR1_HSTXDDL0            DSI_WPCR1_HSTXDDL0_Msk
#define DSI_WPCR1_HSTXDDL1_Pos        (3U)
#define DSI_WPCR1_HSTXDDL1_Msk        (0x1UL << DSI_WPCR1_HSTXDDL1_Pos)        /*!< 0x00000008 */
#define DSI_WPCR1_HSTXDDL1            DSI_WPCR1_HSTXDDL1_Msk

#define DSI_WPCR1_LPSRCCL_Pos         (6U)
#define DSI_WPCR1_LPSRCCL_Msk         (0x3UL << DSI_WPCR1_LPSRCCL_Pos)         /*!< 0x000000C0 */
#define DSI_WPCR1_LPSRCCL             DSI_WPCR1_LPSRCCL_Msk                    /*!< Low-Power transmission Slew Rate Compensation on Clock Lane */
#define DSI_WPCR1_LPSRCCL0_Pos        (6U)
#define DSI_WPCR1_LPSRCCL0_Msk        (0x1UL << DSI_WPCR1_LPSRCCL0_Pos)        /*!< 0x00000040 */
#define DSI_WPCR1_LPSRCCL0            DSI_WPCR1_LPSRCCL0_Msk
#define DSI_WPCR1_LPSRCCL1_Pos        (7U)
#define DSI_WPCR1_LPSRCCL1_Msk        (0x1UL << DSI_WPCR1_LPSRCCL1_Pos)        /*!< 0x00000080 */
#define DSI_WPCR1_LPSRCCL1            DSI_WPCR1_LPSRCCL1_Msk

#define DSI_WPCR1_LPSRCDL_Pos         (8U)
#define DSI_WPCR1_LPSRCDL_Msk         (0x3UL << DSI_WPCR1_LPSRCDL_Pos)         /*!< 0x00000300 */
#define DSI_WPCR1_LPSRCDL             DSI_WPCR1_LPSRCDL_Msk                    /*!< Low-Power transmission Slew Rate Compensation on Data Lane */
#define DSI_WPCR1_LPSRCDL0_Pos        (8U)
#define DSI_WPCR1_LPSRCDL0_Msk        (0x1UL << DSI_WPCR1_LPSRCDL0_Pos)        /*!< 0x00000100 */
#define DSI_WPCR1_LPSRCDL0            DSI_WPCR1_LPSRCDL0_Msk
#define DSI_WPCR1_LPSRCDL1_Pos        (9U)
#define DSI_WPCR1_LPSRCDL1_Msk        (0x1UL << DSI_WPCR1_LPSRCDL1_Pos)        /*!< 0x00000200 */
#define DSI_WPCR1_LPSRCDL1            DSI_WPCR1_LPSRCDL1_Msk

#define DSI_WPCR1_SDDC_Pos            (12U)
#define DSI_WPCR1_SDDC_Msk            (0x1UL << DSI_WPCR1_SDDC_Pos)            /*!< 0x00001000 */
#define DSI_WPCR1_SDDC                DSI_WPCR1_SDDC_Msk                       /*!< SDD Control */

#define DSI_WPCR1_LPRXVCDL_Pos        (14U)
#define DSI_WPCR1_LPRXVCDL_Msk        (0x3UL << DSI_WPCR1_LPRXVCDL_Pos)        /*!< 0x0000C000 */
#define DSI_WPCR1_LPRXVCDL            DSI_WPCR1_LPRXVCDL_Msk                   /*!< Low-Power Reception V-IL Compensation on Data Lanes */
#define DSI_WPCR1_LPRXVCDL0_Pos       (14U)
#define DSI_WPCR1_LPRXVCDL0_Msk       (0x1UL << DSI_WPCR1_LPRXVCDL0_Pos)       /*!< 0x00004000 */
#define DSI_WPCR1_LPRXVCDL0           DSI_WPCR1_LPRXVCDL0_Msk
#define DSI_WPCR1_LPRXVCDL1_Pos       (15U)
#define DSI_WPCR1_LPRXVCDL1_Msk       (0x1UL << DSI_WPCR1_LPRXVCDL1_Pos)       /*!< 0x00008000 */
#define DSI_WPCR1_LPRXVCDL1           DSI_WPCR1_LPRXVCDL1_Msk

#define DSI_WPCR1_HSTXSRCCL_Pos       (16U)
#define DSI_WPCR1_HSTXSRCCL_Msk       (0x3UL << DSI_WPCR1_HSTXSRCCL_Pos)       /*!< 0x00030000 */
#define DSI_WPCR1_HSTXSRCCL           DSI_WPCR1_HSTXSRCCL_Msk                  /*!< High-Speed Transmission Delay on Clock Lane */
#define DSI_WPCR1_HSTXSRCCL0_Pos      (16U)
#define DSI_WPCR1_HSTXSRCCL0_Msk      (0x1UL << DSI_WPCR1_HSTXSRCCL0_Pos)      /*!< 0x00010000 */
#define DSI_WPCR1_HSTXSRCCL0          DSI_WPCR1_HSTXSRCCL0_Msk
#define DSI_WPCR1_HSTXSRCCL1_Pos      (17U)
#define DSI_WPCR1_HSTXSRCCL1_Msk      (0x1UL << DSI_WPCR1_HSTXSRCCL1_Pos)      /*!< 0x00020000 */
#define DSI_WPCR1_HSTXSRCCL1          DSI_WPCR1_HSTXSRCCL1_Msk

#define DSI_WPCR1_HSTXSRCDL_Pos       (18U)
#define DSI_WPCR1_HSTXSRCDL_Msk       (0x3UL << DSI_WPCR1_HSTXSRCDL_Pos)       /*!< 0x000C0000 */
#define DSI_WPCR1_HSTXSRCDL           DSI_WPCR1_HSTXSRCDL_Msk                  /*!< High-Speed Transmission Delay on Data Lane */
#define DSI_WPCR1_HSTXSRCDL0_Pos      (18U)
#define DSI_WPCR1_HSTXSRCDL0_Msk      (0x1UL << DSI_WPCR1_HSTXSRCDL0_Pos)      /*!< 0x00040000 */
#define DSI_WPCR1_HSTXSRCDL0          DSI_WPCR1_HSTXSRCDL0_Msk
#define DSI_WPCR1_HSTXSRCDL1_Pos      (19U)
#define DSI_WPCR1_HSTXSRCDL1_Msk      (0x1UL << DSI_WPCR1_HSTXSRCDL1_Pos)      /*!< 0x00080000 */
#define DSI_WPCR1_HSTXSRCDL1          DSI_WPCR1_HSTXSRCDL1_Msk

#define DSI_WPCR1_FLPRXLPM_Pos        (22U)
#define DSI_WPCR1_FLPRXLPM_Msk        (0x1UL << DSI_WPCR1_FLPRXLPM_Pos)        /*!< 0x00400000 */
#define DSI_WPCR1_FLPRXLPM            DSI_WPCR1_FLPRXLPM_Msk                   /*!< Forces LP Receiver in Low-Power Mode */

#define DSI_WPCR1_LPRXFT_Pos          (25U)
#define DSI_WPCR1_LPRXFT_Msk          (0x3UL << DSI_WPCR1_LPRXFT_Pos)          /*!< 0x06000000 */
#define DSI_WPCR1_LPRXFT              DSI_WPCR1_LPRXFT_Msk                     /*!< Low-Power RX low-pass Filtering Tuning */
#define DSI_WPCR1_LPRXFT0_Pos         (25U)
#define DSI_WPCR1_LPRXFT0_Msk         (0x1UL << DSI_WPCR1_LPRXFT0_Pos)         /*!< 0x02000000 */
#define DSI_WPCR1_LPRXFT0             DSI_WPCR1_LPRXFT0_Msk
#define DSI_WPCR1_LPRXFT1_Pos         (26U)
#define DSI_WPCR1_LPRXFT1_Msk         (0x1UL << DSI_WPCR1_LPRXFT1_Pos)         /*!< 0x04000000 */
#define DSI_WPCR1_LPRXFT1             DSI_WPCR1_LPRXFT1_Msk

/*******************  Bit definition for DSI_WPCR2 register  ***************/
#define DSI_WPCR2_TCLKPREP_Pos        (0U)
#define DSI_WPCR2_TCLKPREP_Msk        (0xFFUL << DSI_WPCR2_TCLKPREP_Pos)       /*!< 0x000000FF */
#define DSI_WPCR2_TCLKPREP            DSI_WPCR2_TCLKPREP_Msk                   /*!< t-CLKPREP */
#define DSI_WPCR2_TCLKPREP0_Pos       (0U)
#define DSI_WPCR2_TCLKPREP0_Msk       (0x1UL << DSI_WPCR2_TCLKPREP0_Pos)       /*!< 0x00000001 */
#define DSI_WPCR2_TCLKPREP0           DSI_WPCR2_TCLKPREP0_Msk
#define DSI_WPCR2_TCLKPREP1_Pos       (1U)
#define DSI_WPCR2_TCLKPREP1_Msk       (0x1UL << DSI_WPCR2_TCLKPREP1_Pos)       /*!< 0x00000002 */
#define DSI_WPCR2_TCLKPREP1           DSI_WPCR2_TCLKPREP1_Msk
#define DSI_WPCR2_TCLKPREP2_Pos       (2U)
#define DSI_WPCR2_TCLKPREP2_Msk       (0x1UL << DSI_WPCR2_TCLKPREP2_Pos)       /*!< 0x00000004 */
#define DSI_WPCR2_TCLKPREP2           DSI_WPCR2_TCLKPREP2_Msk
#define DSI_WPCR2_TCLKPREP3_Pos       (3U)
#define DSI_WPCR2_TCLKPREP3_Msk       (0x1UL << DSI_WPCR2_TCLKPREP3_Pos)       /*!< 0x00000008 */
#define DSI_WPCR2_TCLKPREP3           DSI_WPCR2_TCLKPREP3_Msk
#define DSI_WPCR2_TCLKPREP4_Pos       (4U)
#define DSI_WPCR2_TCLKPREP4_Msk       (0x1UL << DSI_WPCR2_TCLKPREP4_Pos)       /*!< 0x00000010 */
#define DSI_WPCR2_TCLKPREP4           DSI_WPCR2_TCLKPREP4_Msk
#define DSI_WPCR2_TCLKPREP5_Pos       (5U)
#define DSI_WPCR2_TCLKPREP5_Msk       (0x1UL << DSI_WPCR2_TCLKPREP5_Pos)       /*!< 0x00000020 */
#define DSI_WPCR2_TCLKPREP5           DSI_WPCR2_TCLKPREP5_Msk
#define DSI_WPCR2_TCLKPREP6_Pos       (6U)
#define DSI_WPCR2_TCLKPREP6_Msk       (0x1UL << DSI_WPCR2_TCLKPREP6_Pos)       /*!< 0x00000040 */
#define DSI_WPCR2_TCLKPREP6           DSI_WPCR2_TCLKPREP6_Msk
#define DSI_WPCR2_TCLKPREP7_Pos       (7U)
#define DSI_WPCR2_TCLKPREP7_Msk       (0x1UL << DSI_WPCR2_TCLKPREP7_Pos)       /*!< 0x00000080 */
#define DSI_WPCR2_TCLKPREP7           DSI_WPCR2_TCLKPREP7_Msk

#define DSI_WPCR2_TCLKZERO_Pos        (8U)
#define DSI_WPCR2_TCLKZERO_Msk        (0xFFUL << DSI_WPCR2_TCLKZERO_Pos)       /*!< 0x0000FF00 */
#define DSI_WPCR2_TCLKZERO            DSI_WPCR2_TCLKZERO_Msk                   /*!< t-CLKZERO */
#define DSI_WPCR2_TCLKZERO0_Pos       (8U)
#define DSI_WPCR2_TCLKZERO0_Msk       (0x1UL << DSI_WPCR2_TCLKZERO0_Pos)       /*!< 0x00000100 */
#define DSI_WPCR2_TCLKZERO0           DSI_WPCR2_TCLKZERO0_Msk
#define DSI_WPCR2_TCLKZERO1_Pos       (9U)
#define DSI_WPCR2_TCLKZERO1_Msk       (0x1UL << DSI_WPCR2_TCLKZERO1_Pos)       /*!< 0x00000200 */
#define DSI_WPCR2_TCLKZERO1           DSI_WPCR2_TCLKZERO1_Msk
#define DSI_WPCR2_TCLKZERO2_Pos       (10U)
#define DSI_WPCR2_TCLKZERO2_Msk       (0x1UL << DSI_WPCR2_TCLKZERO2_Pos)       /*!< 0x00000400 */
#define DSI_WPCR2_TCLKZERO2           DSI_WPCR2_TCLKZERO2_Msk
#define DSI_WPCR2_TCLKZERO3_Pos       (11U)
#define DSI_WPCR2_TCLKZERO3_Msk       (0x1UL << DSI_WPCR2_TCLKZERO3_Pos)       /*!< 0x00000800 */
#define DSI_WPCR2_TCLKZERO3           DSI_WPCR2_TCLKZERO3_Msk
#define DSI_WPCR2_TCLKZERO4_Pos       (12U)
#define DSI_WPCR2_TCLKZERO4_Msk       (0x1UL << DSI_WPCR2_TCLKZERO4_Pos)       /*!< 0x00001000 */
#define DSI_WPCR2_TCLKZERO4           DSI_WPCR2_TCLKZERO4_Msk
#define DSI_WPCR2_TCLKZERO5_Pos       (13U)
#define DSI_WPCR2_TCLKZERO5_Msk       (0x1UL << DSI_WPCR2_TCLKZERO5_Pos)       /*!< 0x00002000 */
#define DSI_WPCR2_TCLKZERO5           DSI_WPCR2_TCLKZERO5_Msk
#define DSI_WPCR2_TCLKZERO6_Pos       (14U)
#define DSI_WPCR2_TCLKZERO6_Msk       (0x1UL << DSI_WPCR2_TCLKZERO6_Pos)       /*!< 0x00004000 */
#define DSI_WPCR2_TCLKZERO6           DSI_WPCR2_TCLKZERO6_Msk
#define DSI_WPCR2_TCLKZERO7_Pos       (15U)
#define DSI_WPCR2_TCLKZERO7_Msk       (0x1UL << DSI_WPCR2_TCLKZERO7_Pos)       /*!< 0x00008000 */
#define DSI_WPCR2_TCLKZERO7           DSI_WPCR2_TCLKZERO7_Msk

#define DSI_WPCR2_THSPREP_Pos         (16U)
#define DSI_WPCR2_THSPREP_Msk         (0xFFUL << DSI_WPCR2_THSPREP_Pos)        /*!< 0x00FF0000 */
#define DSI_WPCR2_THSPREP             DSI_WPCR2_THSPREP_Msk                    /*!< t-HSPREP */
#define DSI_WPCR2_THSPREP0_Pos        (16U)
#define DSI_WPCR2_THSPREP0_Msk        (0x1UL << DSI_WPCR2_THSPREP0_Pos)        /*!< 0x00010000 */
#define DSI_WPCR2_THSPREP0            DSI_WPCR2_THSPREP0_Msk
#define DSI_WPCR2_THSPREP1_Pos        (17U)
#define DSI_WPCR2_THSPREP1_Msk        (0x1UL << DSI_WPCR2_THSPREP1_Pos)        /*!< 0x00020000 */
#define DSI_WPCR2_THSPREP1            DSI_WPCR2_THSPREP1_Msk
#define DSI_WPCR2_THSPREP2_Pos        (18U)
#define DSI_WPCR2_THSPREP2_Msk        (0x1UL << DSI_WPCR2_THSPREP2_Pos)        /*!< 0x00040000 */
#define DSI_WPCR2_THSPREP2            DSI_WPCR2_THSPREP2_Msk
#define DSI_WPCR2_THSPREP3_Pos        (19U)
#define DSI_WPCR2_THSPREP3_Msk        (0x1UL << DSI_WPCR2_THSPREP3_Pos)        /*!< 0x00080000 */
#define DSI_WPCR2_THSPREP3            DSI_WPCR2_THSPREP3_Msk
#define DSI_WPCR2_THSPREP4_Pos        (20U)
#define DSI_WPCR2_THSPREP4_Msk        (0x1UL << DSI_WPCR2_THSPREP4_Pos)        /*!< 0x00100000 */
#define DSI_WPCR2_THSPREP4            DSI_WPCR2_THSPREP4_Msk
#define DSI_WPCR2_THSPREP5_Pos        (21U)
#define DSI_WPCR2_THSPREP5_Msk        (0x1UL << DSI_WPCR2_THSPREP5_Pos)        /*!< 0x00200000 */
#define DSI_WPCR2_THSPREP5            DSI_WPCR2_THSPREP5_Msk
#define DSI_WPCR2_THSPREP6_Pos        (22U)
#define DSI_WPCR2_THSPREP6_Msk        (0x1UL << DSI_WPCR2_THSPREP6_Pos)        /*!< 0x00400000 */
#define DSI_WPCR2_THSPREP6            DSI_WPCR2_THSPREP6_Msk
#define DSI_WPCR2_THSPREP7_Pos        (23U)
#define DSI_WPCR2_THSPREP7_Msk        (0x1UL << DSI_WPCR2_THSPREP7_Pos)        /*!< 0x00800000 */
#define DSI_WPCR2_THSPREP7            DSI_WPCR2_THSPREP7_Msk

#define DSI_WPCR2_THSTRAIL_Pos        (24U)
#define DSI_WPCR2_THSTRAIL_Msk        (0xFFUL << DSI_WPCR2_THSTRAIL_Pos)       /*!< 0xFF000000 */
#define DSI_WPCR2_THSTRAIL            DSI_WPCR2_THSTRAIL_Msk                   /*!< t-HSTRAIL */
#define DSI_WPCR2_THSTRAIL0_Pos       (24U)
#define DSI_WPCR2_THSTRAIL0_Msk       (0x1UL << DSI_WPCR2_THSTRAIL0_Pos)       /*!< 0x01000000 */
#define DSI_WPCR2_THSTRAIL0           DSI_WPCR2_THSTRAIL0_Msk
#define DSI_WPCR2_THSTRAIL1_Pos       (25U)
#define DSI_WPCR2_THSTRAIL1_Msk       (0x1UL << DSI_WPCR2_THSTRAIL1_Pos)       /*!< 0x02000000 */
#define DSI_WPCR2_THSTRAIL1           DSI_WPCR2_THSTRAIL1_Msk
#define DSI_WPCR2_THSTRAIL2_Pos       (26U)
#define DSI_WPCR2_THSTRAIL2_Msk       (0x1UL << DSI_WPCR2_THSTRAIL2_Pos)       /*!< 0x04000000 */
#define DSI_WPCR2_THSTRAIL2           DSI_WPCR2_THSTRAIL2_Msk
#define DSI_WPCR2_THSTRAIL3_Pos       (27U)
#define DSI_WPCR2_THSTRAIL3_Msk       (0x1UL << DSI_WPCR2_THSTRAIL3_Pos)       /*!< 0x08000000 */
#define DSI_WPCR2_THSTRAIL3           DSI_WPCR2_THSTRAIL3_Msk
#define DSI_WPCR2_THSTRAIL4_Pos       (28U)
#define DSI_WPCR2_THSTRAIL4_Msk       (0x1UL << DSI_WPCR2_THSTRAIL4_Pos)       /*!< 0x10000000 */
#define DSI_WPCR2_THSTRAIL4           DSI_WPCR2_THSTRAIL4_Msk
#define DSI_WPCR2_THSTRAIL5_Pos       (29U)
#define DSI_WPCR2_THSTRAIL5_Msk       (0x1UL << DSI_WPCR2_THSTRAIL5_Pos)       /*!< 0x20000000 */
#define DSI_WPCR2_THSTRAIL5           DSI_WPCR2_THSTRAIL5_Msk
#define DSI_WPCR2_THSTRAIL6_Pos       (30U)
#define DSI_WPCR2_THSTRAIL6_Msk       (0x1UL << DSI_WPCR2_THSTRAIL6_Pos)       /*!< 0x40000000 */
#define DSI_WPCR2_THSTRAIL6           DSI_WPCR2_THSTRAIL6_Msk
#define DSI_WPCR2_THSTRAIL7_Pos       (31U)
#define DSI_WPCR2_THSTRAIL7_Msk       (0x1UL << DSI_WPCR2_THSTRAIL7_Pos)       /*!< 0x80000000 */
#define DSI_WPCR2_THSTRAIL7           DSI_WPCR2_THSTRAIL7_Msk

/*******************  Bit definition for DSI_WPCR3 register  ***************/
#define DSI_WPCR3_THSZERO_Pos         (0U)
#define DSI_WPCR3_THSZERO_Msk         (0xFFUL << DSI_WPCR3_THSZERO_Pos)        /*!< 0x000000FF */
#define DSI_WPCR3_THSZERO             DSI_WPCR3_THSZERO_Msk                    /*!< t-HSZERO */
#define DSI_WPCR3_THSZERO0_Pos        (0U)
#define DSI_WPCR3_THSZERO0_Msk        (0x1UL << DSI_WPCR3_THSZERO0_Pos)        /*!< 0x00000001 */
#define DSI_WPCR3_THSZERO0            DSI_WPCR3_THSZERO0_Msk
#define DSI_WPCR3_THSZERO1_Pos        (1U)
#define DSI_WPCR3_THSZERO1_Msk        (0x1UL << DSI_WPCR3_THSZERO1_Pos)        /*!< 0x00000002 */
#define DSI_WPCR3_THSZERO1            DSI_WPCR3_THSZERO1_Msk
#define DSI_WPCR3_THSZERO2_Pos        (2U)
#define DSI_WPCR3_THSZERO2_Msk        (0x1UL << DSI_WPCR3_THSZERO2_Pos)        /*!< 0x00000004 */
#define DSI_WPCR3_THSZERO2            DSI_WPCR3_THSZERO2_Msk
#define DSI_WPCR3_THSZERO3_Pos        (3U)
#define DSI_WPCR3_THSZERO3_Msk        (0x1UL << DSI_WPCR3_THSZERO3_Pos)        /*!< 0x00000008 */
#define DSI_WPCR3_THSZERO3            DSI_WPCR3_THSZERO3_Msk
#define DSI_WPCR3_THSZERO4_Pos        (4U)
#define DSI_WPCR3_THSZERO4_Msk        (0x1UL << DSI_WPCR3_THSZERO4_Pos)        /*!< 0x00000010 */
#define DSI_WPCR3_THSZERO4            DSI_WPCR3_THSZERO4_Msk
#define DSI_WPCR3_THSZERO5_Pos        (5U)
#define DSI_WPCR3_THSZERO5_Msk        (0x1UL << DSI_WPCR3_THSZERO5_Pos)        /*!< 0x00000020 */
#define DSI_WPCR3_THSZERO5            DSI_WPCR3_THSZERO5_Msk
#define DSI_WPCR3_THSZERO6_Pos        (6U)
#define DSI_WPCR3_THSZERO6_Msk        (0x1UL << DSI_WPCR3_THSZERO6_Pos)        /*!< 0x00000040 */
#define DSI_WPCR3_THSZERO6            DSI_WPCR3_THSZERO6_Msk
#define DSI_WPCR3_THSZERO7_Pos        (7U)
#define DSI_WPCR3_THSZERO7_Msk        (0x1UL << DSI_WPCR3_THSZERO7_Pos)        /*!< 0x00000080 */
#define DSI_WPCR3_THSZERO7            DSI_WPCR3_THSZERO7_Msk

#define DSI_WPCR3_TLPXD_Pos           (8U)
#define DSI_WPCR3_TLPXD_Msk           (0xFFUL << DSI_WPCR3_TLPXD_Pos)          /*!< 0x0000FF00 */
#define DSI_WPCR3_TLPXD               DSI_WPCR3_TLPXD_Msk                      /*!< t-LPXD */
#define DSI_WPCR3_TLPXD0_Pos          (8U)
#define DSI_WPCR3_TLPXD0_Msk          (0x1UL << DSI_WPCR3_TLPXD0_Pos)          /*!< 0x00000100 */
#define DSI_WPCR3_TLPXD0              DSI_WPCR3_TLPXD0_Msk
#define DSI_WPCR3_TLPXD1_Pos          (9U)
#define DSI_WPCR3_TLPXD1_Msk          (0x1UL << DSI_WPCR3_TLPXD1_Pos)          /*!< 0x00000200 */
#define DSI_WPCR3_TLPXD1              DSI_WPCR3_TLPXD1_Msk
#define DSI_WPCR3_TLPXD2_Pos          (10U)
#define DSI_WPCR3_TLPXD2_Msk          (0x1UL << DSI_WPCR3_TLPXD2_Pos)          /*!< 0x00000400 */
#define DSI_WPCR3_TLPXD2              DSI_WPCR3_TLPXD2_Msk
#define DSI_WPCR3_TLPXD3_Pos          (11U)
#define DSI_WPCR3_TLPXD3_Msk          (0x1UL << DSI_WPCR3_TLPXD3_Pos)          /*!< 0x00000800 */
#define DSI_WPCR3_TLPXD3              DSI_WPCR3_TLPXD3_Msk
#define DSI_WPCR3_TLPXD4_Pos          (12U)
#define DSI_WPCR3_TLPXD4_Msk          (0x1UL << DSI_WPCR3_TLPXD4_Pos)          /*!< 0x00001000 */
#define DSI_WPCR3_TLPXD4              DSI_WPCR3_TLPXD4_Msk
#define DSI_WPCR3_TLPXD5_Pos          (13U)
#define DSI_WPCR3_TLPXD5_Msk          (0x1UL << DSI_WPCR3_TLPXD5_Pos)          /*!< 0x00002000 */
#define DSI_WPCR3_TLPXD5              DSI_WPCR3_TLPXD5_Msk
#define DSI_WPCR3_TLPXD6_Pos          (14U)
#define DSI_WPCR3_TLPXD6_Msk          (0x1UL << DSI_WPCR3_TLPXD6_Pos)          /*!< 0x00004000 */
#define DSI_WPCR3_TLPXD6              DSI_WPCR3_TLPXD6_Msk
#define DSI_WPCR3_TLPXD7_Pos          (15U)
#define DSI_WPCR3_TLPXD7_Msk          (0x1UL << DSI_WPCR3_TLPXD7_Pos)          /*!< 0x00008000 */
#define DSI_WPCR3_TLPXD7              DSI_WPCR3_TLPXD7_Msk

#define DSI_WPCR3_THSEXIT_Pos         (16U)
#define DSI_WPCR3_THSEXIT_Msk         (0xFFUL << DSI_WPCR3_THSEXIT_Pos)        /*!< 0x00FF0000 */
#define DSI_WPCR3_THSEXIT             DSI_WPCR3_THSEXIT_Msk                    /*!< t-HSEXIT */
#define DSI_WPCR3_THSEXIT0_Pos        (16U)
#define DSI_WPCR3_THSEXIT0_Msk        (0x1UL << DSI_WPCR3_THSEXIT0_Pos)        /*!< 0x00010000 */
#define DSI_WPCR3_THSEXIT0            DSI_WPCR3_THSEXIT0_Msk
#define DSI_WPCR3_THSEXIT1_Pos        (17U)
#define DSI_WPCR3_THSEXIT1_Msk        (0x1UL << DSI_WPCR3_THSEXIT1_Pos)        /*!< 0x00020000 */
#define DSI_WPCR3_THSEXIT1            DSI_WPCR3_THSEXIT1_Msk
#define DSI_WPCR3_THSEXIT2_Pos        (18U)
#define DSI_WPCR3_THSEXIT2_Msk        (0x1UL << DSI_WPCR3_THSEXIT2_Pos)        /*!< 0x00040000 */
#define DSI_WPCR3_THSEXIT2            DSI_WPCR3_THSEXIT2_Msk
#define DSI_WPCR3_THSEXIT3_Pos        (19U)
#define DSI_WPCR3_THSEXIT3_Msk        (0x1UL << DSI_WPCR3_THSEXIT3_Pos)        /*!< 0x00080000 */
#define DSI_WPCR3_THSEXIT3            DSI_WPCR3_THSEXIT3_Msk
#define DSI_WPCR3_THSEXIT4_Pos        (20U)
#define DSI_WPCR3_THSEXIT4_Msk        (0x1UL << DSI_WPCR3_THSEXIT4_Pos)        /*!< 0x00100000 */
#define DSI_WPCR3_THSEXIT4            DSI_WPCR3_THSEXIT4_Msk
#define DSI_WPCR3_THSEXIT5_Pos        (21U)
#define DSI_WPCR3_THSEXIT5_Msk        (0x1UL << DSI_WPCR3_THSEXIT5_Pos)        /*!< 0x00200000 */
#define DSI_WPCR3_THSEXIT5            DSI_WPCR3_THSEXIT5_Msk
#define DSI_WPCR3_THSEXIT6_Pos        (22U)
#define DSI_WPCR3_THSEXIT6_Msk        (0x1UL << DSI_WPCR3_THSEXIT6_Pos)        /*!< 0x00400000 */
#define DSI_WPCR3_THSEXIT6            DSI_WPCR3_THSEXIT6_Msk
#define DSI_WPCR3_THSEXIT7_Pos        (23U)
#define DSI_WPCR3_THSEXIT7_Msk        (0x1UL << DSI_WPCR3_THSEXIT7_Pos)        /*!< 0x00800000 */
#define DSI_WPCR3_THSEXIT7            DSI_WPCR3_THSEXIT7_Msk

#define DSI_WPCR3_TLPXC_Pos           (24U)
#define DSI_WPCR3_TLPXC_Msk           (0xFFUL << DSI_WPCR3_TLPXC_Pos)          /*!< 0xFF000000 */
#define DSI_WPCR3_TLPXC               DSI_WPCR3_TLPXC_Msk                      /*!< t-LPXC */
#define DSI_WPCR3_TLPXC0_Pos          (24U)
#define DSI_WPCR3_TLPXC0_Msk          (0x1UL << DSI_WPCR3_TLPXC0_Pos)          /*!< 0x01000000 */
#define DSI_WPCR3_TLPXC0              DSI_WPCR3_TLPXC0_Msk
#define DSI_WPCR3_TLPXC1_Pos          (25U)
#define DSI_WPCR3_TLPXC1_Msk          (0x1UL << DSI_WPCR3_TLPXC1_Pos)          /*!< 0x02000000 */
#define DSI_WPCR3_TLPXC1              DSI_WPCR3_TLPXC1_Msk
#define DSI_WPCR3_TLPXC2_Pos          (26U)
#define DSI_WPCR3_TLPXC2_Msk          (0x1UL << DSI_WPCR3_TLPXC2_Pos)          /*!< 0x04000000 */
#define DSI_WPCR3_TLPXC2              DSI_WPCR3_TLPXC2_Msk
#define DSI_WPCR3_TLPXC3_Pos          (27U)
#define DSI_WPCR3_TLPXC3_Msk          (0x1UL << DSI_WPCR3_TLPXC3_Pos)          /*!< 0x08000000 */
#define DSI_WPCR3_TLPXC3              DSI_WPCR3_TLPXC3_Msk
#define DSI_WPCR3_TLPXC4_Pos          (28U)
#define DSI_WPCR3_TLPXC4_Msk          (0x1UL << DSI_WPCR3_TLPXC4_Pos)          /*!< 0x10000000 */
#define DSI_WPCR3_TLPXC4              DSI_WPCR3_TLPXC4_Msk
#define DSI_WPCR3_TLPXC5_Pos          (29U)
#define DSI_WPCR3_TLPXC5_Msk          (0x1UL << DSI_WPCR3_TLPXC5_Pos)          /*!< 0x20000000 */
#define DSI_WPCR3_TLPXC5              DSI_WPCR3_TLPXC5_Msk
#define DSI_WPCR3_TLPXC6_Pos          (30U)
#define DSI_WPCR3_TLPXC6_Msk          (0x1UL << DSI_WPCR3_TLPXC6_Pos)          /*!< 0x40000000 */
#define DSI_WPCR3_TLPXC6              DSI_WPCR3_TLPXC6_Msk
#define DSI_WPCR3_TLPXC7_Pos          (31U)
#define DSI_WPCR3_TLPXC7_Msk          (0x1UL << DSI_WPCR3_TLPXC7_Pos)          /*!< 0x80000000 */
#define DSI_WPCR3_TLPXC7              DSI_WPCR3_TLPXC7_Msk

/*******************  Bit definition for DSI_WPCR4 register  ***************/
#define DSI_WPCR4_TCLKPOST_Pos        (0U)
#define DSI_WPCR4_TCLKPOST_Msk        (0xFFUL << DSI_WPCR4_TCLKPOST_Pos)       /*!< 0x000000FF */
#define DSI_WPCR4_TCLKPOST            DSI_WPCR4_TCLKPOST_Msk                   /*!< t-CLKPOST */
#define DSI_WPCR4_TCLKPOST0_Pos       (0U)
#define DSI_WPCR4_TCLKPOST0_Msk       (0x1UL << DSI_WPCR4_TCLKPOST0_Pos)       /*!< 0x00000001 */
#define DSI_WPCR4_TCLKPOST0           DSI_WPCR4_TCLKPOST0_Msk
#define DSI_WPCR4_TCLKPOST1_Pos       (1U)
#define DSI_WPCR4_TCLKPOST1_Msk       (0x1UL << DSI_WPCR4_TCLKPOST1_Pos)       /*!< 0x00000002 */
#define DSI_WPCR4_TCLKPOST1           DSI_WPCR4_TCLKPOST1_Msk
#define DSI_WPCR4_TCLKPOST2_Pos       (2U)
#define DSI_WPCR4_TCLKPOST2_Msk       (0x1UL << DSI_WPCR4_TCLKPOST2_Pos)       /*!< 0x00000004 */
#define DSI_WPCR4_TCLKPOST2           DSI_WPCR4_TCLKPOST2_Msk
#define DSI_WPCR4_TCLKPOST3_Pos       (3U)
#define DSI_WPCR4_TCLKPOST3_Msk       (0x1UL << DSI_WPCR4_TCLKPOST3_Pos)       /*!< 0x00000008 */
#define DSI_WPCR4_TCLKPOST3           DSI_WPCR4_TCLKPOST3_Msk
#define DSI_WPCR4_TCLKPOST4_Pos       (4U)
#define DSI_WPCR4_TCLKPOST4_Msk       (0x1UL << DSI_WPCR4_TCLKPOST4_Pos)       /*!< 0x00000010 */
#define DSI_WPCR4_TCLKPOST4           DSI_WPCR4_TCLKPOST4_Msk
#define DSI_WPCR4_TCLKPOST5_Pos       (5U)
#define DSI_WPCR4_TCLKPOST5_Msk       (0x1UL << DSI_WPCR4_TCLKPOST5_Pos)       /*!< 0x00000020 */
#define DSI_WPCR4_TCLKPOST5           DSI_WPCR4_TCLKPOST5_Msk
#define DSI_WPCR4_TCLKPOST6_Pos       (6U)
#define DSI_WPCR4_TCLKPOST6_Msk       (0x1UL << DSI_WPCR4_TCLKPOST6_Pos)       /*!< 0x00000040 */
#define DSI_WPCR4_TCLKPOST6           DSI_WPCR4_TCLKPOST6_Msk
#define DSI_WPCR4_TCLKPOST7_Pos       (7U)
#define DSI_WPCR4_TCLKPOST7_Msk       (0x1UL << DSI_WPCR4_TCLKPOST7_Pos)       /*!< 0x00000080 */
#define DSI_WPCR4_TCLKPOST7           DSI_WPCR4_TCLKPOST7_Msk

/*******************  Bit definition for DSI_WRPCR register  ***************/
#define DSI_WRPCR_PLLEN_Pos           (0U)
#define DSI_WRPCR_PLLEN_Msk           (0x1UL << DSI_WRPCR_PLLEN_Pos)           /*!< 0x00000001 */
#define DSI_WRPCR_PLLEN               DSI_WRPCR_PLLEN_Msk                      /*!< PLL Enable */
#define DSI_WRPCR_PLL_NDIV_Pos        (2U)
#define DSI_WRPCR_PLL_NDIV_Msk        (0x7FUL << DSI_WRPCR_PLL_NDIV_Pos)       /*!< 0x000001FC */
#define DSI_WRPCR_PLL_NDIV            DSI_WRPCR_PLL_NDIV_Msk                   /*!< PLL Loop Division Factor */
#define DSI_WRPCR_PLL_NDIV0_Pos       (2U)
#define DSI_WRPCR_PLL_NDIV0_Msk       (0x1UL << DSI_WRPCR_PLL_NDIV0_Pos)       /*!< 0x00000004 */
#define DSI_WRPCR_PLL_NDIV0           DSI_WRPCR_PLL_NDIV0_Msk
#define DSI_WRPCR_PLL_NDIV1_Pos       (3U)
#define DSI_WRPCR_PLL_NDIV1_Msk       (0x1UL << DSI_WRPCR_PLL_NDIV1_Pos)       /*!< 0x00000008 */
#define DSI_WRPCR_PLL_NDIV1           DSI_WRPCR_PLL_NDIV1_Msk
#define DSI_WRPCR_PLL_NDIV2_Pos       (4U)
#define DSI_WRPCR_PLL_NDIV2_Msk       (0x1UL << DSI_WRPCR_PLL_NDIV2_Pos)       /*!< 0x00000010 */
#define DSI_WRPCR_PLL_NDIV2           DSI_WRPCR_PLL_NDIV2_Msk
#define DSI_WRPCR_PLL_NDIV3_Pos       (5U)
#define DSI_WRPCR_PLL_NDIV3_Msk       (0x1UL << DSI_WRPCR_PLL_NDIV3_Pos)       /*!< 0x00000020 */
#define DSI_WRPCR_PLL_NDIV3           DSI_WRPCR_PLL_NDIV3_Msk
#define DSI_WRPCR_PLL_NDIV4_Pos       (6U)
#define DSI_WRPCR_PLL_NDIV4_Msk       (0x1UL << DSI_WRPCR_PLL_NDIV4_Pos)       /*!< 0x00000040 */
#define DSI_WRPCR_PLL_NDIV4           DSI_WRPCR_PLL_NDIV4_Msk
#define DSI_WRPCR_PLL_NDIV5_Pos       (7U)
#define DSI_WRPCR_PLL_NDIV5_Msk       (0x1UL << DSI_WRPCR_PLL_NDIV5_Pos)       /*!< 0x00000080 */
#define DSI_WRPCR_PLL_NDIV5           DSI_WRPCR_PLL_NDIV5_Msk
#define DSI_WRPCR_PLL_NDIV6_Pos       (8U)
#define DSI_WRPCR_PLL_NDIV6_Msk       (0x1UL << DSI_WRPCR_PLL_NDIV6_Pos)       /*!< 0x00000100 */
#define DSI_WRPCR_PLL_NDIV6           DSI_WRPCR_PLL_NDIV6_Msk

#define DSI_WRPCR_PLL_IDF_Pos         (11U)
#define DSI_WRPCR_PLL_IDF_Msk         (0xFUL << DSI_WRPCR_PLL_IDF_Pos)         /*!< 0x00007800 */
#define DSI_WRPCR_PLL_IDF             DSI_WRPCR_PLL_IDF_Msk                    /*!< PLL Input Division Factor */
#define DSI_WRPCR_PLL_IDF0_Pos        (11U)
#define DSI_WRPCR_PLL_IDF0_Msk        (0x1UL << DSI_WRPCR_PLL_IDF0_Pos)        /*!< 0x00000800 */
#define DSI_WRPCR_PLL_IDF0            DSI_WRPCR_PLL_IDF0_Msk
#define DSI_WRPCR_PLL_IDF1_Pos        (12U)
#define DSI_WRPCR_PLL_IDF1_Msk        (0x1UL << DSI_WRPCR_PLL_IDF1_Pos)        /*!< 0x00001000 */
#define DSI_WRPCR_PLL_IDF1            DSI_WRPCR_PLL_IDF1_Msk
#define DSI_WRPCR_PLL_IDF2_Pos        (13U)
#define DSI_WRPCR_PLL_IDF2_Msk        (0x1UL << DSI_WRPCR_PLL_IDF2_Pos)        /*!< 0x00002000 */
#define DSI_WRPCR_PLL_IDF2            DSI_WRPCR_PLL_IDF2_Msk
#define DSI_WRPCR_PLL_IDF3_Pos        (14U)
#define DSI_WRPCR_PLL_IDF3_Msk        (0x1UL << DSI_WRPCR_PLL_IDF3_Pos)        /*!< 0x00004000 */
#define DSI_WRPCR_PLL_IDF3            DSI_WRPCR_PLL_IDF3_Msk

#define DSI_WRPCR_PLL_ODF_Pos         (16U)
#define DSI_WRPCR_PLL_ODF_Msk         (0x3UL << DSI_WRPCR_PLL_ODF_Pos)         /*!< 0x00030000 */
#define DSI_WRPCR_PLL_ODF             DSI_WRPCR_PLL_ODF_Msk                    /*!< PLL Output Division Factor */
#define DSI_WRPCR_PLL_ODF0_Pos        (16U)
#define DSI_WRPCR_PLL_ODF0_Msk        (0x1UL << DSI_WRPCR_PLL_ODF0_Pos)        /*!< 0x00010000 */
#define DSI_WRPCR_PLL_ODF0            DSI_WRPCR_PLL_ODF0_Msk
#define DSI_WRPCR_PLL_ODF1_Pos        (17U)
#define DSI_WRPCR_PLL_ODF1_Msk        (0x1UL << DSI_WRPCR_PLL_ODF1_Pos)        /*!< 0x00020000 */
#define DSI_WRPCR_PLL_ODF1            DSI_WRPCR_PLL_ODF1_Msk

#define DSI_WRPCR_REGEN_Pos           (24U)
#define DSI_WRPCR_REGEN_Msk           (0x1UL << DSI_WRPCR_REGEN_Pos)           /*!< 0x01000000 */
#define DSI_WRPCR_REGEN               DSI_WRPCR_REGEN_Msk                      /*!< Regulator Enable */


/* Exported constants --------------------------------------------------------*/
/** @defgroup DSI_Exported_Constants DSI Exported Constants
  * @{
  */
/** @defgroup DSI_DCS_Command DSI DCS Command
  * @{
  */
#define DSI_ENTER_IDLE_MODE       0x39U
#define DSI_ENTER_INVERT_MODE     0x21U
#define DSI_ENTER_NORMAL_MODE     0x13U
#define DSI_ENTER_PARTIAL_MODE    0x12U
#define DSI_ENTER_SLEEP_MODE      0x10U
#define DSI_EXIT_IDLE_MODE        0x38U
#define DSI_EXIT_INVERT_MODE      0x20U
#define DSI_EXIT_SLEEP_MODE       0x11U
#define DSI_GET_3D_CONTROL        0x3FU
#define DSI_GET_ADDRESS_MODE      0x0BU
#define DSI_GET_BLUE_CHANNEL      0x08U
#define DSI_GET_DIAGNOSTIC_RESULT 0x0FU
#define DSI_GET_DISPLAY_MODE      0x0DU
#define DSI_GET_GREEN_CHANNEL     0x07U
#define DSI_GET_PIXEL_FORMAT      0x0CU
#define DSI_GET_POWER_MODE        0x0AU
#define DSI_GET_RED_CHANNEL       0x06U
#define DSI_GET_SCANLINE          0x45U
#define DSI_GET_SIGNAL_MODE       0x0EU
#define DSI_NOP                   0x00U
#define DSI_READ_DDB_CONTINUE     0xA8U
#define DSI_READ_DDB_START        0xA1U
#define DSI_READ_MEMORY_CONTINUE  0x3EU
#define DSI_READ_MEMORY_START     0x2EU
#define DSI_SET_3D_CONTROL        0x3DU
#define DSI_SET_ADDRESS_MODE      0x36U
#define DSI_SET_COLUMN_ADDRESS    0x2AU
#define DSI_SET_DISPLAY_OFF       0x28U
#define DSI_SET_DISPLAY_ON        0x29U
#define DSI_SET_GAMMA_CURVE       0x26U
#define DSI_SET_PAGE_ADDRESS      0x2BU
#define DSI_SET_PARTIAL_COLUMNS   0x31U
#define DSI_SET_PARTIAL_ROWS      0x30U
#define DSI_SET_PIXEL_FORMAT      0x3AU
#define DSI_SET_SCROLL_AREA       0x33U
#define DSI_SET_SCROLL_START      0x37U
#define DSI_SET_TEAR_OFF          0x34U
#define DSI_SET_TEAR_ON           0x35U
#define DSI_SET_TEAR_SCANLINE     0x44U
#define DSI_SET_VSYNC_TIMING      0x40U
#define DSI_SOFT_RESET            0x01U
#define DSI_WRITE_LUT             0x2DU
#define DSI_WRITE_MEMORY_CONTINUE 0x3CU
#define DSI_WRITE_MEMORY_START    0x2CU
/**
  * @}
  */

/** @defgroup DSI_Video_Mode_Type DSI Video Mode Type
  * @{
  */
#define DSI_VID_MODE_NB_PULSES    0U
#define DSI_VID_MODE_NB_EVENTS    1U
#define DSI_VID_MODE_BURST        2U
/**
  * @}
  */

/** @defgroup DSI_Color_Mode DSI Color Mode
  * @{
  */
#define DSI_COLOR_MODE_FULL       0x00000000U
#define DSI_COLOR_MODE_EIGHT      DSI_WCR_COLM
/**
  * @}
  */

/** @defgroup DSI_ShutDown DSI ShutDown
  * @{
  */
#define DSI_DISPLAY_ON            0x00000000U
#define DSI_DISPLAY_OFF           DSI_WCR_SHTDN
/**
  * @}
  */

/** @defgroup DSI_LP_Command DSI LP Command
  * @{
  */
#define DSI_LP_COMMAND_DISABLE    0x00000000U
#define DSI_LP_COMMAND_ENABLE     DSI_VMCR_LPCE
/**
  * @}
  */

/** @defgroup DSI_LP_HFP DSI LP HFP
  * @{
  */
#define DSI_LP_HFP_DISABLE        0x00000000U
#define DSI_LP_HFP_ENABLE         DSI_VMCR_LPHFPE
/**
  * @}
  */

/** @defgroup DSI_LP_HBP DSI LP HBP
  * @{
  */
#define DSI_LP_HBP_DISABLE        0x00000000U
#define DSI_LP_HBP_ENABLE         DSI_VMCR_LPHBPE
/**
  * @}
  */

/** @defgroup DSI_LP_VACT DSI LP VACT
  * @{
  */
#define DSI_LP_VACT_DISABLE       0x00000000U
#define DSI_LP_VACT_ENABLE        DSI_VMCR_LPVAE
/**
  * @}
  */

/** @defgroup DSI_LP_VFP DSI LP VFP
  * @{
  */
#define DSI_LP_VFP_DISABLE       0x00000000U
#define DSI_LP_VFP_ENABLE        DSI_VMCR_LPVFPE
/**
  * @}
  */

/** @defgroup DSI_LP_VBP DSI LP VBP
  * @{
  */
#define DSI_LP_VBP_DISABLE       0x00000000U
#define DSI_LP_VBP_ENABLE        DSI_VMCR_LPVBPE
/**
  * @}
  */

/** @defgroup DSI_LP_VSYNC DSI LP VSYNC
  * @{
  */
#define DSI_LP_VSYNC_DISABLE     0x00000000U
#define DSI_LP_VSYNC_ENABLE      DSI_VMCR_LPVSAE
/**
  * @}
  */

/** @defgroup DSI_FBTA_acknowledge DSI FBTA Acknowledge
  * @{
  */
#define DSI_FBTAA_DISABLE        0x00000000U
#define DSI_FBTAA_ENABLE         DSI_VMCR_FBTAAE
/**
  * @}
  */

/** @defgroup DSI_TearingEffectSource DSI Tearing Effect Source
  * @{
  */
#define DSI_TE_DSILINK           0x00000000U
#define DSI_TE_EXTERNAL          DSI_WCFGR_TESRC
/**
  * @}
  */

/** @defgroup DSI_TearingEffectPolarity DSI Tearing Effect Polarity
  * @{
  */
#define DSI_TE_RISING_EDGE       0x00000000U
#define DSI_TE_FALLING_EDGE      DSI_WCFGR_TEPOL
/**
  * @}
  */

/** @defgroup DSI_Vsync_Polarity DSI Vsync Polarity
  * @{
  */
#define DSI_VSYNC_FALLING        0x00000000U
#define DSI_VSYNC_RISING         DSI_WCFGR_VSPOL
/**
  * @}
  */

/** @defgroup DSI_AutomaticRefresh DSI Automatic Refresh
  * @{
  */
#define DSI_AR_DISABLE           0x00000000U
#define DSI_AR_ENABLE            DSI_WCFGR_AR
/**
  * @}
  */

/** @defgroup DSI_TE_AcknowledgeRequest DSI TE Acknowledge Request
  * @{
  */
#define DSI_TE_ACKNOWLEDGE_DISABLE 0x00000000U
#define DSI_TE_ACKNOWLEDGE_ENABLE  DSI_CMCR_TEARE
/**
  * @}
  */

/** @defgroup DSI_AcknowledgeRequest DSI Acknowledge Request
  * @{
  */
#define DSI_ACKNOWLEDGE_DISABLE   0x00000000U
#define DSI_ACKNOWLEDGE_ENABLE    DSI_CMCR_ARE
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortWriteNoP DSI LP LPGen Short Write NoP
  * @{
  */
#define DSI_LP_GSW0P_DISABLE     0x00000000U
#define DSI_LP_GSW0P_ENABLE      DSI_CMCR_GSW0TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortWriteOneP DSI LP LPGen Short Write OneP
  * @{
  */
#define DSI_LP_GSW1P_DISABLE     0x00000000U
#define DSI_LP_GSW1P_ENABLE      DSI_CMCR_GSW1TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortWriteTwoP DSI LP LPGen Short Write TwoP
  * @{
  */
#define DSI_LP_GSW2P_DISABLE     0x00000000U
#define DSI_LP_GSW2P_ENABLE      DSI_CMCR_GSW2TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortReadNoP DSI LP LPGen Short Read NoP
  * @{
  */
#define DSI_LP_GSR0P_DISABLE     0x00000000U
#define DSI_LP_GSR0P_ENABLE      DSI_CMCR_GSR0TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortReadOneP DSI LP LPGen Short Read OneP
  * @{
  */
#define DSI_LP_GSR1P_DISABLE     0x00000000U
#define DSI_LP_GSR1P_ENABLE      DSI_CMCR_GSR1TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortReadTwoP DSI LP LPGen Short Read TwoP
  * @{
  */
#define DSI_LP_GSR2P_DISABLE     0x00000000U
#define DSI_LP_GSR2P_ENABLE      DSI_CMCR_GSR2TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenLongWrite DSI LP LPGen LongWrite
  * @{
  */
#define DSI_LP_GLW_DISABLE       0x00000000U
#define DSI_LP_GLW_ENABLE        DSI_CMCR_GLWTX
/**
  * @}
  */

/** @defgroup DSI_LP_LPDcsShortWriteNoP DSI LP LPDcs Short Write NoP
  * @{
  */
#define DSI_LP_DSW0P_DISABLE     0x00000000U
#define DSI_LP_DSW0P_ENABLE      DSI_CMCR_DSW0TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPDcsShortWriteOneP DSI LP LPDcs Short Write OneP
  * @{
  */
#define DSI_LP_DSW1P_DISABLE     0x00000000U
#define DSI_LP_DSW1P_ENABLE      DSI_CMCR_DSW1TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPDcsShortReadNoP DSI LP LPDcs Short Read NoP
  * @{
  */
#define DSI_LP_DSR0P_DISABLE     0x00000000U
#define DSI_LP_DSR0P_ENABLE      DSI_CMCR_DSR0TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPDcsLongWrite DSI LP LPDcs Long Write
  * @{
  */
#define DSI_LP_DLW_DISABLE       0x00000000U
#define DSI_LP_DLW_ENABLE        DSI_CMCR_DLWTX
/**
  * @}
  */

/** @defgroup DSI_LP_LPMaxReadPacket DSI LP LPMax Read Packet
  * @{
  */
#define DSI_LP_MRDP_DISABLE      0x00000000U
#define DSI_LP_MRDP_ENABLE       DSI_CMCR_MRDPS
/**
  * @}
  */

/** @defgroup DSI_HS_PrespMode DSI HS Presp Mode
  * @{
  */
#define DSI_HS_PM_DISABLE        0x00000000U
#define DSI_HS_PM_ENABLE         DSI_TCCR3_PM
/**
  * @}
  */


/** @defgroup DSI_Automatic_Clk_Lane_Control DSI Automatic Clk Lane Control
  * @{
  */
#define DSI_AUTO_CLK_LANE_CTRL_DISABLE 0x00000000U
#define DSI_AUTO_CLK_LANE_CTRL_ENABLE  DSI_CLCR_ACR
/**
  * @}
  */

/** @defgroup DSI_Number_Of_Lanes DSI Number Of Lanes
  * @{
  */
#define DSI_ONE_DATA_LANE          0U
#define DSI_TWO_DATA_LANES         1U
/**
  * @}
  */

/** @defgroup DSI_FlowControl DSI Flow Control
  * @{
  */
#define DSI_FLOW_CONTROL_CRC_RX    DSI_PCR_CRCRXE
#define DSI_FLOW_CONTROL_ECC_RX    DSI_PCR_ECCRXE
#define DSI_FLOW_CONTROL_BTA       DSI_PCR_BTAE
#define DSI_FLOW_CONTROL_EOTP_RX   DSI_PCR_ETRXE
#define DSI_FLOW_CONTROL_EOTP_TX   DSI_PCR_ETTXE
#define DSI_FLOW_CONTROL_ALL       (DSI_FLOW_CONTROL_CRC_RX | DSI_FLOW_CONTROL_ECC_RX | \
                                    DSI_FLOW_CONTROL_BTA | DSI_FLOW_CONTROL_EOTP_RX | \
                                    DSI_FLOW_CONTROL_EOTP_TX)
/**
  * @}
  */

/** @defgroup DSI_Color_Coding DSI Color Coding
  * @{
  */
#define DSI_RGB565                 0x00000000U /*!< The values 0x00000001 and 0x00000002 can also be used for the RGB565 color mode configuration */
#define DSI_RGB666                 0x00000003U /*!< The value 0x00000004 can also be used for the RGB666 color mode configuration                 */
#define DSI_RGB888                 0x00000005U
/**
  * @}
  */

/** @defgroup DSI_LooselyPacked DSI Loosely Packed
  * @{
  */
#define DSI_LOOSELY_PACKED_ENABLE  DSI_LCOLCR_LPE
#define DSI_LOOSELY_PACKED_DISABLE 0x00000000U
/**
  * @}
  */

/** @defgroup DSI_HSYNC_Polarity DSI HSYNC Polarity
  * @{
  */
#define DSI_HSYNC_ACTIVE_HIGH       0x00000000U
#define DSI_HSYNC_ACTIVE_LOW        DSI_LPCR_HSP
/**
  * @}
  */

/** @defgroup DSI_VSYNC_Active_Polarity DSI VSYNC Active Polarity
  * @{
  */
#define DSI_VSYNC_ACTIVE_HIGH       0x00000000U
#define DSI_VSYNC_ACTIVE_LOW        DSI_LPCR_VSP
/**
  * @}
  */

/** @defgroup DSI_DATA_ENABLE_Polarity DSI DATA ENABLE Polarity
  * @{
  */
#define DSI_DATA_ENABLE_ACTIVE_HIGH 0x00000000U
#define DSI_DATA_ENABLE_ACTIVE_LOW  DSI_LPCR_DEP
/**
  * @}
  */

/** @defgroup DSI_PLL_IDF DSI PLL IDF
  * @{
  */
#define DSI_PLL_IN_DIV1             0x00000001U
#define DSI_PLL_IN_DIV2             0x00000002U
#define DSI_PLL_IN_DIV3             0x00000003U
#define DSI_PLL_IN_DIV4             0x00000004U
#define DSI_PLL_IN_DIV5             0x00000005U
#define DSI_PLL_IN_DIV6             0x00000006U
#define DSI_PLL_IN_DIV7             0x00000007U
/**
  * @}
  */

/** @defgroup DSI_PLL_ODF DSI PLL ODF
  * @{
  */
#define DSI_PLL_OUT_DIV1            0x00000000U
#define DSI_PLL_OUT_DIV2            0x00000001U
#define DSI_PLL_OUT_DIV4            0x00000002U
#define DSI_PLL_OUT_DIV8            0x00000003U
/**
  * @}
  */

/** @defgroup DSI_Flags DSI Flags
  * @{
  */
#define DSI_FLAG_TE                 DSI_WISR_TEIF
#define DSI_FLAG_ER                 DSI_WISR_ERIF
#define DSI_FLAG_BUSY               DSI_WISR_BUSY
#define DSI_FLAG_PLLLS              DSI_WISR_PLLLS
#define DSI_FLAG_PLLL               DSI_WISR_PLLLIF
#define DSI_FLAG_PLLU               DSI_WISR_PLLUIF
#define DSI_FLAG_RRS                DSI_WISR_RRS
#define DSI_FLAG_RR                 DSI_WISR_RRIF
/**
  * @}
  */

/** @defgroup DSI_Interrupts DSI Interrupts
  * @{
  */
#define DSI_IT_TE                   DSI_WIER_TEIE
#define DSI_IT_ER                   DSI_WIER_ERIE
#define DSI_IT_PLLL                 DSI_WIER_PLLLIE
#define DSI_IT_PLLU                 DSI_WIER_PLLUIE
#define DSI_IT_RR                   DSI_WIER_RRIE
/**
  * @}
  */

/** @defgroup DSI_SHORT_WRITE_PKT_Data_Type DSI SHORT WRITE PKT Data Type
  * @{
  */
#define DSI_DCS_SHORT_PKT_WRITE_P0  0x00000005U /*!< DCS short write, no parameters      */
#define DSI_DCS_SHORT_PKT_WRITE_P1  0x00000015U /*!< DCS short write, one parameter      */
#define DSI_GEN_SHORT_PKT_WRITE_P0  0x00000003U /*!< Generic short write, no parameters  */
#define DSI_GEN_SHORT_PKT_WRITE_P1  0x00000013U /*!< Generic short write, one parameter  */
#define DSI_GEN_SHORT_PKT_WRITE_P2  0x00000023U /*!< Generic short write, two parameters */
/**
  * @}
  */

/** @defgroup DSI_LONG_WRITE_PKT_Data_Type DSI LONG WRITE PKT Data Type
  * @{
  */
#define DSI_DCS_LONG_PKT_WRITE      0x00000039U /*!< DCS long write     */
#define DSI_GEN_LONG_PKT_WRITE      0x00000029U /*!< Generic long write */
/**
  * @}
  */

/** @defgroup DSI_SHORT_READ_PKT_Data_Type DSI SHORT READ PKT Data Type
  * @{
  */
#define DSI_DCS_SHORT_PKT_READ      0x00000006U /*!< DCS short read                     */
#define DSI_GEN_SHORT_PKT_READ_P0   0x00000004U /*!< Generic short read, no parameters  */
#define DSI_GEN_SHORT_PKT_READ_P1   0x00000014U /*!< Generic short read, one parameter  */
#define DSI_GEN_SHORT_PKT_READ_P2   0x00000024U /*!< Generic short read, two parameters */
/**
  * @}
  */

/** @defgroup DSI_Error_Data_Type DSI Error Data Type
  * @{
  */
#define HAL_DSI_ERROR_NONE              0U
#define HAL_DSI_ERROR_ACK               0x00000001U /*!< acknowledge errors          */
#define HAL_DSI_ERROR_PHY               0x00000002U /*!< PHY related errors          */
#define HAL_DSI_ERROR_TX                0x00000004U /*!< transmission error          */
#define HAL_DSI_ERROR_RX                0x00000008U /*!< reception error             */
#define HAL_DSI_ERROR_ECC               0x00000010U /*!< ECC errors                  */
#define HAL_DSI_ERROR_CRC               0x00000020U /*!< CRC error                   */
#define HAL_DSI_ERROR_PSE               0x00000040U /*!< Packet Size error           */
#define HAL_DSI_ERROR_EOT               0x00000080U /*!< End Of Transmission error   */
#define HAL_DSI_ERROR_OVF               0x00000100U /*!< FIFO overflow error         */
#define HAL_DSI_ERROR_GEN               0x00000200U /*!< Generic FIFO related errors */
/**
  * @}
  */

/** @defgroup DSI_Lane_Group DSI Lane Group
  * @{
  */
#define DSI_CLOCK_LANE              0x00000000U
#define DSI_DATA_LANES              0x00000001U
/**
  * @}
  */

/** @defgroup DSI_Communication_Delay DSI Communication Delay
  * @{
  */
#define DSI_SLEW_RATE_HSTX          0x00000000U
#define DSI_SLEW_RATE_LPTX          0x00000001U
#define DSI_HS_DELAY                0x00000002U
/**
  * @}
  */

/** @defgroup DSI_CustomLane DSI CustomLane
  * @{
  */
#define DSI_SWAP_LANE_PINS          0x00000000U
#define DSI_INVERT_HS_SIGNAL        0x00000001U
/**
  * @}
  */

/** @defgroup DSI_Lane_Select DSI Lane Select
  * @{
  */
#define DSI_CLK_LANE                0x00000000U
#define DSI_DATA_LANE0              0x00000001U
#define DSI_DATA_LANE1              0x00000002U
/**
  * @}
  */

/** @defgroup DSI_PHY_Timing DSI PHY Timing
  * @{
  */
#define DSI_TCLK_POST               0x00000000U
#define DSI_TLPX_CLK                0x00000001U
#define DSI_THS_EXIT                0x00000002U
#define DSI_TLPX_DATA               0x00000003U
#define DSI_THS_ZERO                0x00000004U
#define DSI_THS_TRAIL               0x00000005U
#define DSI_THS_PREPARE             0x00000006U
#define DSI_TCLK_ZERO               0x00000007U
#define DSI_TCLK_PREPARE            0x00000008U


#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_DDSI_H */
