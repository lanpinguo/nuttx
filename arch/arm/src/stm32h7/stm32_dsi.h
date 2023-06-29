/****************************************************************************
 * arch/arm/src/stm32h7/stm32_dsi.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_DSI_H
#define __ARCH_ARM_SRC_STM32H7_STM32_DSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/adc.h>
#include "chip.h"
#include "hardware/stm32_dsi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
  * @brief  Enables the DSI host.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define STM32_DSI_ENABLE(__HANDLE__) do { \
                                          volatile uint32_t tmpreg = 0x00U; \
                                          modreg32(DSI_CR_EN, DSI_CR_EN, STM32_DSI_CR);\
                                          /* Delay after an DSI Host enabling */ \
                                          tmpreg = getreg32(STM32_DSI_CR);\
                                          UNUSED(tmpreg); \
                                        } while(0U)

/**
  * @brief  Disables the DSI host.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define STM32_DSI_DISABLE(__HANDLE__) do { \
                                          volatile uint32_t tmpreg = 0x00U; \
                                          modreg32(~DSI_CR_EN, DSI_CR_EN, STM32_DSI_CR);\
                                          /* Delay after an DSI Host disabling */ \
                                          tmpreg = getreg32(STM32_DSI_CR);\
                                          UNUSED(tmpreg); \
                                         } while(0U)

/**
  * @brief  Enables the DSI wrapper.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define STM32_DSI_WRAPPER_ENABLE(__HANDLE__) do { \
                                                 volatile uint32_t tmpreg = 0x00U; \
                                                 modreg32(DSI_WCR_DSIEN, DSI_WCR_DSIEN, STM32_DSI_WCR);\
                                                 /* Delay after an DSI warpper enabling */ \
                                                 tmpreg = getreg32(STM32_DSI_WCR);\
                                                 UNUSED(tmpreg); \
                                                } while(0U)

/**
  * @brief  Disable the DSI wrapper.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define STM32_DSI_WRAPPER_DISABLE(__HANDLE__) do { \
                                                  volatile uint32_t tmpreg = 0x00U; \
                                                  modreg32(~DSI_WCR_DSIEN, DSI_WCR_DSIEN, STM32_DSI_WCR);\
                                                  /* Delay after an DSI warpper disabling*/ \
                                                  tmpreg = getreg32(STM32_DSI_WCR);\
                                                  UNUSED(tmpreg); \
                                                 } while(0U)

/**
  * @brief  Enables the DSI PLL.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define STM32_DSI_PLL_ENABLE(__HANDLE__) do { \
                                             volatile uint32_t tmpreg = 0x00U; \
                                              modreg32(DSI_WRPCR_PLLEN, DSI_WRPCR_PLLEN, STM32_DSI_WRPCR);\
                                             /* Delay after an DSI PLL enabling */ \
                                             tmpreg = getreg32(STM32_DSI_WRPCR);\
                                             UNUSED(tmpreg); \
                                            } while(0U)

/**
  * @brief  Disables the DSI PLL.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define STM32_DSI_PLL_DISABLE(__HANDLE__) do { \
                                              volatile uint32_t tmpreg = 0x00U; \
                                              modreg32(~DSI_WRPCR_PLLEN, DSI_WRPCR_PLLEN, STM32_DSI_WRPCR);\
                                             /* Delay after an DSI PLL disabling */ \
                                              tmpreg = getreg32(STM32_DSI_WRPCR);\
                                              UNUSED(tmpreg); \
                                             } while(0U)

/**
  * @brief  Enables the DSI regulator.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define STM32_DSI_REG_ENABLE(__HANDLE__) do { \
                                              volatile uint32_t tmpreg = 0x00U; \
                                              modreg32(DSI_WRPCR_REGEN, DSI_WRPCR_REGEN, STM32_DSI_WRPCR);\
                                              /* Delay after an DSI regulator enabling */ \
                                              tmpreg = getreg32(STM32_DSI_WRPCR);\
                                              UNUSED(tmpreg); \
                                            } while(0U)

/**
  * @brief  Disables the DSI regulator.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define STM32_DSI_REG_DISABLE(__HANDLE__) do { \
                                              volatile uint32_t tmpreg = 0x00U; \
                                              modreg32(~DSI_WRPCR_REGEN, DSI_WRPCR_REGEN, STM32_DSI_WRPCR);\
                                              /* Delay after an DSI regulator disabling */ \
                                              tmpreg = getreg32(STM32_DSI_WRPCR);\
                                              UNUSED(tmpreg); \
                                             } while(0U)

/**
  * @brief  Get the DSI pending flags.
  * @param  __HANDLE__  DSI handle.
  * @param  __FLAG__  Get the specified flag.
  *          This parameter can be any combination of the following values:
  *            @arg DSI_FLAG_TE   : Tearing Effect Interrupt Flag
  *            @arg DSI_FLAG_ER   : End of Refresh Interrupt Flag
  *            @arg DSI_FLAG_BUSY : Busy Flag
  *            @arg DSI_FLAG_PLLLS: PLL Lock Status
  *            @arg DSI_FLAG_PLLL : PLL Lock Interrupt Flag
  *            @arg DSI_FLAG_PLLU : PLL Unlock Interrupt Flag
  *            @arg DSI_FLAG_RRS  : Regulator Ready Flag
  *            @arg DSI_FLAG_RR   : Regulator Ready Interrupt Flag
  * @retval The state of FLAG (SET or RESET).
  */
#define STM32_DSI_GET_FLAG(__HANDLE__, __FLAG__) (getreg32(STM32_DSI_WISR) & (__FLAG__))

/**
  * @brief  Clears the DSI pending flags.
  * @param  __HANDLE__  DSI handle.
  * @param  __FLAG__  specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DSI_FLAG_TE   : Tearing Effect Interrupt Flag
  *            @arg DSI_FLAG_ER   : End of Refresh Interrupt Flag
  *            @arg DSI_FLAG_PLLL : PLL Lock Interrupt Flag
  *            @arg DSI_FLAG_PLLU : PLL Unlock Interrupt Flag
  *            @arg DSI_FLAG_RR   : Regulator Ready Interrupt Flag
  * @retval None
  */
#define STM32_DSI_CLEAR_FLAG(__HANDLE__, __FLAG__) (putreg32((__FLAG__), STM32_DSI_WIFCR))


#define RCC_D1CCIPR_DSISEL_Pos                 (8U)
#define RCC_D1CCIPR_DSISEL_Msk                 (0x1UL << RCC_D1CCIPR_DSISEL_Pos) /*!< 0x00000100 */
#define RCC_D1CCIPR_DSISEL                     RCC_D1CCIPR_DSISEL_Msk


/** @brief  macro to configure the DSI clock source.
  *
  * @param  __DSICLKSource__ specifies the DSI clock source.
  *            @arg RCC_RCC_DSICLKSOURCE_PHY:DSI clock from PHY is selected as DSI byte lane clock
  *            @arg RCC_RCC_DSICLKSOURCE_PLL2   : PLL2_Q Clock clock is selected as DSI byte lane clock
  */
#define STM32_RCC_DSI_CONFIG(__DSICLKSource__) do{ \
        volatile uint32_t regval = 0x0U; \
        regval = getreg32(STM32_RCC_D1CCIPR); \
        regval &= ~RCC_D1CCIPR_DSISEL; \
        regval |= (__DSICLKSource__); \
        putreg32(regval, STM32_RCC_D1CCIPR); \
      }while(0)


/** @brief  macro to get the DSI clock source.
  * @retval The clock source can be one of the following values:
  *            @arg RCC_RCC_DSICLKSOURCE_PHY: DSI clock from PHY is selected as DSI byte lane clock
  *            @arg RCC_RCC_DSICLKSOURCE_PLL2: PLL2_Q Clock clock is selected as DSI byte lane clock
  */
#define STM32_RCC_GET_DSI_SOURCE() ((uint32_t)(getreg32(STM32_RCC_D1CCIPR) & RCC_D1CCIPR_DSISEL)


#define RCC_DSICLKSOURCE_PHY       (0x00000000U)
#define RCC_DSICLKSOURCE_PLL2       RCC_D1CCIPR_DSISEL


/* Private constants ---------------------------------------------------------*/
/** @defgroup DSI_Private_Constants DSI Private Constants
  * @{
  */
#define DSI_MAX_RETURN_PKT_SIZE (0x00000037U) /*!< Maximum return packet configuration */



/* Configuration ************************************************************/
/**
  * @brief  DSI Init Structure definition
  */
typedef struct DSI_Init_s
{
  uint32_t AutomaticClockLaneControl;    /*!< Automatic clock lane control
                                              This parameter can be any value of @ref DSI_Automatic_Clk_Lane_Control */

  uint32_t TXEscapeCkdiv;                /*!< TX Escape clock division
                                              The values 0 and 1 stop the TX_ESC clock generation                    */

  uint32_t NumberOfLanes;                /*!< Number of lanes
                                              This parameter can be any value of @ref DSI_Number_Of_Lanes            */

} DSI_Init_t;


typedef struct DSI_PLL_Init_s
{
  uint32_t PLLNDIV;                      /*!< PLL Loop Division Factor
                                              This parameter must be a value between 10 and 125   */

  uint32_t PLLIDF;                       /*!< PLL Input Division Factor
                                              This parameter can be any value of @ref DSI_PLL_IDF */

  uint32_t PLLODF;                       /*!< PLL Output Division Factor
                                              This parameter can be any value of @ref DSI_PLL_ODF */

} DSI_PLL_Init_t;



/**
  * @brief  DSI Video mode configuration
  */
typedef struct DSI_VidCfg_s
{
  uint32_t VirtualChannelID;             /*!< Virtual channel ID                                                 */

  uint32_t ColorCoding;                  /*!< Color coding for LTDC interface
                                              This parameter can be any value of @ref DSI_Color_Coding           */

  uint32_t LooselyPacked;                /*!< Enable or disable loosely packed stream (needed only when using
                                              18-bit configuration).
                                              This parameter can be any value of @ref DSI_LooselyPacked          */

  uint32_t Mode;                         /*!< Video mode type
                                              This parameter can be any value of @ref DSI_Video_Mode_Type        */

  uint32_t PacketSize;                   /*!< Video packet size                                                  */

  uint32_t NumberOfChunks;               /*!< Number of chunks                                                   */

  uint32_t NullPacketSize;               /*!< Null packet size                                                   */

  uint32_t HSPolarity;                   /*!< HSYNC pin polarity
                                              This parameter can be any value of @ref DSI_HSYNC_Polarity         */

  uint32_t VSPolarity;                   /*!< VSYNC pin polarity
                                              This parameter can be any value of @ref DSI_VSYNC_Active_Polarity  */

  uint32_t DEPolarity;                   /*!< Data Enable pin polarity
                                              This parameter can be any value of @ref DSI_DATA_ENABLE_Polarity   */

  uint32_t HorizontalSyncActive;         /*!< Horizontal synchronism active duration (in lane byte clock cycles) */

  uint32_t HorizontalBackPorch;          /*!< Horizontal back-porch duration (in lane byte clock cycles)         */

  uint32_t HorizontalLine;               /*!< Horizontal line duration (in lane byte clock cycles)               */

  uint32_t VerticalSyncActive;           /*!< Vertical synchronism active duration                               */

  uint32_t VerticalBackPorch;            /*!< Vertical back-porch duration                                       */

  uint32_t VerticalFrontPorch;           /*!< Vertical front-porch duration                                      */

  uint32_t VerticalActive;               /*!< Vertical active duration                                           */

  uint32_t LPCommandEnable;              /*!< Low-power command enable
                                              This parameter can be any value of @ref DSI_LP_Command             */

  uint32_t LPLargestPacketSize;          /*!< The size, in bytes, of the low power largest packet that
                                              can fit in a line during VSA, VBP and VFP regions                  */

  uint32_t LPVACTLargestPacketSize;      /*!< The size, in bytes, of the low power largest packet that
                                              can fit in a line during VACT region                               */

  uint32_t LPHorizontalFrontPorchEnable; /*!< Low-power horizontal front-porch enable
                                              This parameter can be any value of @ref DSI_LP_HFP                 */

  uint32_t LPHorizontalBackPorchEnable;  /*!< Low-power horizontal back-porch enable
                                              This parameter can be any value of @ref DSI_LP_HBP                 */

  uint32_t LPVerticalActiveEnable;       /*!< Low-power vertical active enable
                                              This parameter can be any value of @ref DSI_LP_VACT                */

  uint32_t LPVerticalFrontPorchEnable;   /*!< Low-power vertical front-porch enable
                                              This parameter can be any value of @ref DSI_LP_VFP                 */

  uint32_t LPVerticalBackPorchEnable;    /*!< Low-power vertical back-porch enable
                                              This parameter can be any value of @ref DSI_LP_VBP                 */

  uint32_t LPVerticalSyncActiveEnable;   /*!< Low-power vertical sync active enable
                                              This parameter can be any value of @ref DSI_LP_VSYNC               */

  uint32_t FrameBTAAcknowledgeEnable;    /*!< Frame bus-turn-around acknowledge enable
                                              This parameter can be any value of @ref DSI_FBTA_acknowledge       */

} DSI_VidCfg_t;

/**
  * @brief  DSI Adapted command mode configuration
  */
typedef struct DSI_CmdCfg_s
{
  uint32_t VirtualChannelID;             /*!< Virtual channel ID                                                */

  uint32_t ColorCoding;                  /*!< Color coding for LTDC interface
                                              This parameter can be any value of @ref DSI_Color_Coding          */

  uint32_t CommandSize;                  /*!< Maximum allowed size for an LTDC write memory command, measured in
                                              pixels. This parameter can be any value between 0x00 and 0xFFFFU   */

  uint32_t TearingEffectSource;          /*!< Tearing effect source
                                              This parameter can be any value of @ref DSI_TearingEffectSource   */

  uint32_t TearingEffectPolarity;        /*!< Tearing effect pin polarity
                                              This parameter can be any value of @ref DSI_TearingEffectPolarity */

  uint32_t HSPolarity;                   /*!< HSYNC pin polarity
                                              This parameter can be any value of @ref DSI_HSYNC_Polarity        */

  uint32_t VSPolarity;                   /*!< VSYNC pin polarity
                                              This parameter can be any value of @ref DSI_VSYNC_Active_Polarity */

  uint32_t DEPolarity;                   /*!< Data Enable pin polarity
                                              This parameter can be any value of @ref DSI_DATA_ENABLE_Polarity  */

  uint32_t VSyncPol;                     /*!< VSync edge on which the LTDC is halted
                                              This parameter can be any value of @ref DSI_Vsync_Polarity        */

  uint32_t AutomaticRefresh;             /*!< Automatic refresh mode
                                              This parameter can be any value of @ref DSI_AutomaticRefresh      */

  uint32_t TEAcknowledgeRequest;         /*!< Tearing Effect Acknowledge Request Enable
                                              This parameter can be any value of @ref DSI_TE_AcknowledgeRequest */

} DSI_CmdCfg_t;

/**
  * @brief  DSI command transmission mode configuration
  */
typedef struct DSI_LPCmd_s
{
  uint32_t LPGenShortWriteNoP;           /*!< Generic Short Write Zero parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortWriteNoP  */

  uint32_t LPGenShortWriteOneP;          /*!< Generic Short Write One parameter Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortWriteOneP */

  uint32_t LPGenShortWriteTwoP;          /*!< Generic Short Write Two parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortWriteTwoP */

  uint32_t LPGenShortReadNoP;            /*!< Generic Short Read Zero parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortReadNoP   */

  uint32_t LPGenShortReadOneP;           /*!< Generic Short Read One parameter Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortReadOneP  */

  uint32_t LPGenShortReadTwoP;           /*!< Generic Short Read Two parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortReadTwoP  */

  uint32_t LPGenLongWrite;               /*!< Generic Long Write Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenLongWrite      */

  uint32_t LPDcsShortWriteNoP;           /*!< DCS Short Write Zero parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPDcsShortWriteNoP  */

  uint32_t LPDcsShortWriteOneP;          /*!< DCS Short Write One parameter Transmission
                                              This parameter can be any value of @ref DSI_LP_LPDcsShortWriteOneP */

  uint32_t LPDcsShortReadNoP;            /*!< DCS Short Read Zero parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPDcsShortReadNoP   */

  uint32_t LPDcsLongWrite;               /*!< DCS Long Write Transmission
                                              This parameter can be any value of @ref DSI_LP_LPDcsLongWrite      */

  uint32_t LPMaxReadPacket;              /*!< Maximum Read Packet Size Transmission
                                              This parameter can be any value of @ref DSI_LP_LPMaxReadPacket     */

  uint32_t AcknowledgeRequest;           /*!< Acknowledge Request Enable
                                              This parameter can be any value of @ref DSI_AcknowledgeRequest     */

} DSI_LPCmd_t;

/**
  * @brief  DSI PHY Timings definition
  */
typedef struct DSI_PHY_Timer_s
{
  uint32_t ClockLaneHS2LPTime;           /*!< The maximum time that the D-PHY clock lane takes to go from high-speed
                                              to low-power transmission                                              */

  uint32_t ClockLaneLP2HSTime;           /*!< The maximum time that the D-PHY clock lane takes to go from low-power
                                              to high-speed transmission                                             */

  uint32_t DataLaneHS2LPTime;            /*!< The maximum time that the D-PHY data lanes takes to go from high-speed
                                              to low-power transmission                                              */

  uint32_t DataLaneLP2HSTime;            /*!< The maximum time that the D-PHY data lanes takes to go from low-power
                                              to high-speed transmission                                             */

  uint32_t DataLaneMaxReadTime;          /*!< The maximum time required to perform a read command */

  uint32_t StopWaitTime;                 /*!< The minimum wait period to request a High-Speed transmission after the
                                              Stop state                                                             */

} DSI_PHY_Timer_t;

/**
  * @brief  DSI HOST Timeouts definition
  */
typedef struct DSI_HOST_Timeout_s
{
  uint32_t TimeoutCkdiv;                 /*!< Time-out clock division                                  */

  uint32_t HighSpeedTransmissionTimeout; /*!< High-speed transmission time-out                         */

  uint32_t LowPowerReceptionTimeout;     /*!< Low-power reception time-out                             */

  uint32_t HighSpeedReadTimeout;         /*!< High-speed read time-out                                 */

  uint32_t LowPowerReadTimeout;          /*!< Low-power read time-out                                  */

  uint32_t HighSpeedWriteTimeout;        /*!< High-speed write time-out                                */

  uint32_t HighSpeedWritePrespMode;      /*!< High-speed write presp mode
                                              This parameter can be any value of @ref DSI_HS_PrespMode */

  uint32_t LowPowerWriteTimeout;         /*!< Low-speed write time-out                                 */

  uint32_t BTATimeout;                   /*!< BTA time-out                                             */

} DSI_HOST_Timeout_t;



/* Structs needed for interacting with the NuttX host dsi  */

typedef struct Host_DSI_s
{
  uint32_t                  base;         /*!< Register base address      */
  DSI_Init_t                Init;         /*!< DSI required parameters    */
  sem_t                     Lock;         /*!< DSI peripheral status      */
  volatile uint32_t         State;        /*!< DSI communication state    */
  volatile uint32_t         ErrorCode;    /*!< DSI Error code             */
  uint32_t                  ErrorMsk;     /*!< DSI Error monitoring mask  */
} Host_DSI_t;

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32_dsi_init
 *
 * Description:
 *   Initializes the DSI according to the specified
 *         parameters in the DSI_Init_t and create the associated handle.
 *
 * Input Parameters:
 *   intf      - Could be {1,2,3} for ADC1, ADC2, or ADC3
 *   chanlist  - The list of channels
 *   nchannels - Number of channels
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

int stm32_dsi_init(struct Host_DSI_s *hdsi, struct DSI_PLL_Init_s *PLLInit);

int stm32_dsi_config_adapted_command_mode(struct Host_DSI_s *hdsi, DSI_CmdCfg_t *CmdCfg);

int stm32_dsi_config_command(struct Host_DSI_s *hdsi, DSI_LPCmd_t *LPCmd);

int stm32_dsi_short_write(struct Host_DSI_s *hdsi,
                            uint32_t ChannelID,
                            uint32_t Mode,
                            uint32_t Param1,
                            uint32_t Param2);

int stm32_dsi_start(struct Host_DSI_s *hdsi);

int stm32_dsi_config_flow_control(struct Host_DSI_s *hdsi, uint32_t FlowControl);

int stm32_dsi_force_rx_lowPower(struct Host_DSI_s *hdsi, uint32_t State);

int stm32_dsi_pattern_generator_start(struct Host_DSI_s *hdsi, uint32_t Mode, uint32_t Orientation);

int stm32_dsi_config_phy_timer(struct Host_DSI_s *hdsi, DSI_PHY_Timer_t *PhyTimers);

int stm32_dsi_long_write(Host_DSI_t *hdsi,
                          uint32_t ChannelID,
                          uint32_t Mode,
                          uint32_t NbParams,
                          uint32_t Param1,
                          uint8_t *ParametersTable);

int stm32_dsi_read(Host_DSI_t *hdsi,
                    uint32_t ChannelNbr,
                    uint8_t *Array,
                    uint32_t Size,
                    uint32_t Mode,
                    uint32_t DCSCmd,
                    uint8_t *ParametersTable);

int stm32_dsi_wrapper_set(struct Host_DSI_s *hdsi, bool enable);

int stm32_dsi_refresh(struct Host_DSI_s *hdsi);

void stm32_dsi_reset(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32H7_STM32_DSI_H */
