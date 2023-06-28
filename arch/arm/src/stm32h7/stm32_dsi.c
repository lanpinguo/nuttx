/****************************************************************************
 * arch/arm/src/STM32H7/stm32_dsi.c
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

/* References:
 *   STM32H7x7xx Technical Reference Manual and Data Sheet
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <sys/param.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/video/fb.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/stm32_dsi.h"

#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_dsi.h"



/* Private types -------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @addtogroup DSI_Private_Constants
  * @{
  */
#define DSI_TIMEOUT_VALUE ((uint32_t)1000U)  /* 1s */

#define DSI_ERROR_ACK_MASK (DSI_ISR0_AE0 | DSI_ISR0_AE1 | DSI_ISR0_AE2 | DSI_ISR0_AE3 | \
                            DSI_ISR0_AE4 | DSI_ISR0_AE5 | DSI_ISR0_AE6 | DSI_ISR0_AE7 | \
                            DSI_ISR0_AE8 | DSI_ISR0_AE9 | DSI_ISR0_AE10 | DSI_ISR0_AE11 | \
                            DSI_ISR0_AE12 | DSI_ISR0_AE13 | DSI_ISR0_AE14 | DSI_ISR0_AE15)
#define DSI_ERROR_PHY_MASK (DSI_ISR0_PE0 | DSI_ISR0_PE1 | DSI_ISR0_PE2 | DSI_ISR0_PE3 | DSI_ISR0_PE4)
#define DSI_ERROR_TX_MASK  DSI_ISR1_TOHSTX
#define DSI_ERROR_RX_MASK  DSI_ISR1_TOLPRX
#define DSI_ERROR_ECC_MASK (DSI_ISR1_ECCSE | DSI_ISR1_ECCME)
#define DSI_ERROR_CRC_MASK DSI_ISR1_CRCE
#define DSI_ERROR_PSE_MASK DSI_ISR1_PSE
#define DSI_ERROR_EOT_MASK DSI_ISR1_EOTPE
#define DSI_ERROR_OVF_MASK DSI_ISR1_LPWRE
#define DSI_ERROR_GEN_MASK (DSI_ISR1_GCWRE | DSI_ISR1_GPWRE | DSI_ISR1_GPTXE | DSI_ISR1_GPRDE | DSI_ISR1_GPRXE)



/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Generic DSI packet header configuration
  * @param  DSIx  Pointer to DSI register base
  * @param  ChannelID Virtual channel ID of the header packet
  * @param  DataType  Packet data type of the header packet
  *                   This parameter can be any value of :
  *                      @arg DSI_SHORT_WRITE_PKT_Data_Type
  *                      @arg DSI_LONG_WRITE_PKT_Data_Type
  *                      @arg DSI_SHORT_READ_PKT_Data_Type
  *                      @arg DSI_MAX_RETURN_PKT_SIZE
  * @param  Data0  Word count LSB
  * @param  Data1  Word count MSB
  * @retval None
  */
static void dsi_config_pkt_hdr(    uint32_t ChannelID,
                                   uint32_t DataType,
                                   uint32_t Data0,
                                   uint32_t Data1)
{
  /* Update the DSI packet header with new information */
  putreg32 (DataType | (ChannelID << 6U) | (Data0 << 8U) | (Data1 << 16U), STM32_DSI_GHCR);
}


/**
  * @brief  write short DCS or short Generic command
  * @param  hdsi  pointer to a struct Host_DSI_s structure that contains
  *               the configuration information for the DSI.
  * @param  ChannelID  Virtual channel ID.
  * @param  Mode  DSI short packet data type.
  *               This parameter can be any value of @arg DSI_SHORT_WRITE_PKT_Data_Type.
  * @param  Param1  DSC command or first generic parameter.
  *                 This parameter can be any value of @arg DSI_DCS_Command or a
  *                 generic command code.
  * @param  Param2  DSC parameter or second generic parameter.
  * @retval HAL status
  */
int stm32_dsi_short_write(struct Host_DSI_s *hdsi,
                            uint32_t ChannelID,
                            uint32_t Mode,
                            uint32_t Param1,
                            uint32_t Param2)
{
  clock_t tickstart;

  /* Get tick */
  tickstart = clock_systime_ticks();

  /* Wait for Command FIFO Empty */
  while((getreg32(STM32_DSI_GPSR) & DSI_GPSR_CMDFE) == 0U)
  {
    /* Check for the Timeout */
    if((clock_systime_ticks() - tickstart ) > DSI_TIMEOUT_VALUE)
    {
      return -ETIME;
    }
  }

  /* Configure the packet to send a short DCS command with 0 or 1 parameter */
  /* Update the DSI packet header with new information */
  putreg32(Mode | (ChannelID << 6U) | (Param1 << 8U) | (Param2 << 16U), STM32_DSI_GHCR);

  return OK;
}

/**
  * @brief  Select adapted command mode and configure the corresponding parameters
  * @param  hdsi  pointer to a struct Host_DSI_s structure that contains
  *               the configuration information for the DSI.
  * @param  CmdCfg  pointer to a DSI_CmdCfg_t structure that contains
  *                 the DSI command mode configuration parameters
  * @retval HAL status
  */
int stm32_dsi_config_adapted_command_mode(struct Host_DSI_s *hdsi, DSI_CmdCfg_t *CmdCfg)
{

  /* Select command mode by setting CMDM and DSIM bits */
  modifyreg32(STM32_DSI_MCR, 0, DSI_MCR_CMDM);
  modifyreg32(STM32_DSI_WCFGR, DSI_WCFGR_DSIM, DSI_WCFGR_DSIM);

  /* Select the virtual channel for the LTDC interface traffic */
  modifyreg32(STM32_DSI_LVCIDR, DSI_LVCIDR_VCID, CmdCfg->VirtualChannelID);

  /* Configure the polarity of control signals */
  modifyreg32(STM32_DSI_LPCR, (DSI_LPCR_DEP | DSI_LPCR_VSP | DSI_LPCR_HSP), \
              (CmdCfg->DEPolarity | CmdCfg->VSPolarity | CmdCfg->HSPolarity));

  /* Select the color coding for the host */
  modifyreg32(STM32_DSI_LCOLCR, DSI_LCOLCR_COLC, CmdCfg->ColorCoding);

  /* Select the color coding for the wrapper */
  modifyreg32(STM32_DSI_WCFGR, DSI_WCFGR_COLMUX, ((CmdCfg->ColorCoding) << 1U));

  /* Configure the maximum allowed size for write memory command */
  modifyreg32(STM32_DSI_LCCR, DSI_LCCR_CMDSIZE, CmdCfg->CommandSize);

  /* Configure the tearing effect source and polarity and select the refresh mode */
  modifyreg32(STM32_DSI_WCFGR, (DSI_WCFGR_TESRC | DSI_WCFGR_TEPOL | DSI_WCFGR_AR | DSI_WCFGR_VSPOL), \
              (CmdCfg->TearingEffectSource | CmdCfg->TearingEffectPolarity | CmdCfg->AutomaticRefresh |
              CmdCfg->VSyncPol));

  /* Configure the tearing effect acknowledge request */
  modifyreg32(STM32_DSI_CMCR, DSI_CMCR_TEARE, CmdCfg->TEAcknowledgeRequest);

  /* Enable the Tearing Effect interrupt */
  modifyreg32(STM32_DSI_WIER, 0, DSI_IT_TE);

  /* Enable the End of Refresh interrupt */
  modifyreg32(STM32_DSI_WIER, 0, DSI_IT_ER);


  return OK;
}

/**
  * @brief  Configure command transmission mode: High-speed or Low-power
  *         and enable/disable acknowledge request after packet transmission
  * @param  hdsi  pointer to a struct Host_DSI_s structure that contains
  *               the configuration information for the DSI.
  * @param  LPCmd  pointer to a DSI_LPCmd_t structure that contains
  *                the DSI command transmission mode configuration parameters
  * @retval HAL status
  */
int stm32_dsi_config_command(struct Host_DSI_s *hdsi, DSI_LPCmd_t *LPCmd)
{
  uint32_t clrbits;
  uint32_t setbits;

  /* Select High-speed or Low-power for command transmission */
  clrbits = (DSI_CMCR_GSW0TX | \
            DSI_CMCR_GSW1TX | \
            DSI_CMCR_GSW2TX | \
            DSI_CMCR_GSR0TX | \
            DSI_CMCR_GSR1TX | \
            DSI_CMCR_GSR2TX | \
            DSI_CMCR_GLWTX  | \
            DSI_CMCR_DSW0TX | \
            DSI_CMCR_DSW1TX | \
            DSI_CMCR_DSR0TX | \
            DSI_CMCR_DLWTX  | \
            DSI_CMCR_MRDPS);
  setbits = (LPCmd->LPGenShortWriteNoP  | \
            LPCmd->LPGenShortWriteOneP | \
            LPCmd->LPGenShortWriteTwoP | \
            LPCmd->LPGenShortReadNoP   | \
            LPCmd->LPGenShortReadOneP  | \
            LPCmd->LPGenShortReadTwoP  | \
            LPCmd->LPGenLongWrite      | \
            LPCmd->LPDcsShortWriteNoP  | \
            LPCmd->LPDcsShortWriteOneP | \
            LPCmd->LPDcsShortReadNoP   | \
            LPCmd->LPDcsLongWrite      | \
            LPCmd->LPMaxReadPacket);
  modifyreg32(STM32_DSI_CMCR, clrbits, setbits);                            

  /* Configure the acknowledge request after each packet transmission */
  modifyreg32(STM32_DSI_CMCR, DSI_CMCR_ARE, LPCmd->AcknowledgeRequest);                            


  return OK;
}


/**
  * @brief  Start the DSI module
  * @param  hdsi  pointer to a struct Host_DSI_s structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
int stm32_dsi_start(struct Host_DSI_s *hdsi)
{

  /* Enable the DSI host */
  STM32_DSI_ENABLE(hdsi);

  /* Enable the DSI wrapper */
  STM32_DSI_WRAPPER_ENABLE(hdsi);


  return OK;
}

/**
  * @brief  set the DSI wrapper module
  * @param  hdsi  pointer to a struct Host_DSI_s structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
int stm32_dsi_wrapper_set(struct Host_DSI_s *hdsi, bool enable)
{

  if(enable){
    /* Enable the DSI wrapper */
    STM32_DSI_WRAPPER_ENABLE(hdsi);
  }
  else{
    /* Disable the DSI wrapper */
    STM32_DSI_WRAPPER_DISABLE(hdsi);
  }


  return OK;
}


/**
  * @brief  Refresh the display in command mode
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
int stm32_dsi_refresh(struct Host_DSI_s *hdsi)
{

  /* Update the display */
  modifyreg32(STM32_DSI_WCR, 0, DSI_WCR_LTDCEN);

  return OK;
}


/**
  * @brief  Configure the flow control parameters
  * @param  hdsi  pointer to a struct Host_DSI_s structure that contains
  *               the configuration information for the DSI.
  * @param  FlowControl  flow control feature(s) to be enabled.
  *                      This parameter can be any combination of @arg DSI_FlowControl.
  * @retval HAL status
  */
int stm32_dsi_config_flow_control(struct Host_DSI_s *hdsi, uint32_t FlowControl)
{


  /* Set the DSI Host Protocol Configuration Register */
  modifyreg32(STM32_DSI_PCR, DSI_FLOW_CONTROL_ALL, FlowControl);


  return OK;
}

/**
  * @brief  Force LP Receiver in Low-Power Mode
  * @param  hdsi  pointer to a struct Host_DSI_s structure that contains
  *               the configuration information for the DSI.
  * @param  State  ENABLE or DISABLE
  * @retval HAL status
  */
int stm32_dsi_force_rx_lowPower(struct Host_DSI_s *hdsi, uint32_t State)
{


  /* Force/Unforce LP Receiver in Low-Power Mode */
  modifyreg32(STM32_DSI_WPCR + 4, DSI_WPCR1_FLPRXLPM, ((uint32_t)State << 22U));


  return OK;
}

/**
  * @brief  Start test pattern generation
  * @param  hdsi  pointer to a struct Host_DSI_s structure that contains
  *               the configuration information for the DSI.
  * @param  Mode  Pattern generator mode
  *          This parameter can be one of the following values:
  *           0 : Color bars (horizontal or vertical)
  *           1 : BER pattern (vertical only)
  * @param  Orientation  Pattern generator orientation
  *          This parameter can be one of the following values:
  *           0 : Vertical color bars
  *           1 : Horizontal color bars
  * @retval HAL status
  */
int stm32_dsi_pattern_generator_start(struct Host_DSI_s *hdsi, uint32_t Mode, uint32_t Orientation)
{

  /* Configure pattern generator mode and orientation */
  modifyreg32(STM32_DSI_VMCR, (DSI_VMCR_PGM | DSI_VMCR_PGO), 
      ((Mode << 20U) | (Orientation << 24U)));

  /* Enable pattern generator by setting PGE bit */
  modifyreg32(STM32_DSI_VMCR, 0, DSI_VMCR_PGE);


  return OK;
}


/**
  * @brief  Configure the DSI PHY timer parameters
  * @param  hdsi  pointer to a struct Host_DSI_s structure that contains
  *               the configuration information for the DSI.
  * @param  PhyTimers  DSI_PHY_Timer_t structure that contains
  *                    the DSI PHY timing parameters
  * @retval HAL status
  */
int stm32_dsi_config_phy_timer(struct Host_DSI_s *hdsi, DSI_PHY_Timer_t *PhyTimers)
{
  uint32_t maxTime;

  maxTime = (PhyTimers->ClockLaneLP2HSTime > PhyTimers->ClockLaneHS2LPTime) ? PhyTimers->ClockLaneLP2HSTime :
            PhyTimers->ClockLaneHS2LPTime;

  /* Clock lane timer configuration */

  /* In Automatic Clock Lane control mode, the DSI Host can turn off the clock lane between two
     High-Speed transmission.
     To do so, the DSI Host calculates the time required for the clock lane to change from HighSpeed
     to Low-Power and from Low-Power to High-Speed.
     This timings are configured by the HS2LP_TIME and LP2HS_TIME in the DSI Host Clock Lane Timer Configuration Register (DSI_CLTCR).
     But the DSI Host is not calculating LP2HS_TIME + HS2LP_TIME but 2 x HS2LP_TIME.

     Workaround : Configure HS2LP_TIME and LP2HS_TIME with the same value being the max of HS2LP_TIME or LP2HS_TIME.
    */
  modifyreg32(STM32_DSI_CLTCR, (DSI_CLTCR_LP2HS_TIME | DSI_CLTCR_HS2LP_TIME), \
      (maxTime | ((maxTime) << 16U)));

  /* Data lane timer configuration */
  modifyreg32(STM32_DSI_DLTCR, (DSI_DLTCR_MRD_TIME | DSI_DLTCR_LP2HS_TIME | DSI_DLTCR_HS2LP_TIME), \
      (PhyTimers->DataLaneMaxReadTime | ((PhyTimers->DataLaneLP2HSTime) << 16U) | 
        ((PhyTimers->DataLaneHS2LPTime) << 24U)));

  /* Configure the wait period to request HS transmission after a stop state */
  modifyreg32(STM32_DSI_PCONFR, DSI_PCONFR_SW_TIME, ((PhyTimers->StopWaitTime) << 8U));


  return OK;
}



/**
  * @brief  write long DCS or long Generic command
  * @param  hdsi  pointer to a Host_DSI_t structure that contains
  *               the configuration information for the DSI.
  * @param  ChannelID  Virtual channel ID.
  * @param  Mode  DSI long packet data type.
  *               This parameter can be any value of @arg DSI_LONG_WRITE_PKT_Data_Type.
  * @param  NbParams  Number of parameters.
  * @param  Param1  DSC command or first generic parameter.
  *                 This parameter can be any value of @arg DSI_DCS_Command or a
  *                 generic command code
  * @param  ParametersTable  Pointer to parameter values table.
  * @retval HAL status
  */
int stm32_dsi_long_write(Host_DSI_t *hdsi,
                          uint32_t ChannelID,
                          uint32_t Mode,
                          uint32_t NbParams,
                          uint32_t Param1,
                          uint8_t *ParametersTable)
{
  uint32_t uicounter, nbBytes, count;
  uint32_t tickstart;
  uint32_t fifoword;
  uint8_t *pparams = ParametersTable;


  /* Get tick */
  tickstart = clock_systime_ticks();

  /* Wait for Command FIFO Empty */
  while ((getreg32(STM32_DSI_GPSR) & DSI_GPSR_CMDFE) == 0U)
  {
    /* Check for the Timeout */
    if ((clock_systime_ticks() - tickstart) > DSI_TIMEOUT_VALUE)
    {
      return -ETIME;
    }
  }

  /* Set the DCS code on payload byte 1, and the other parameters on the write FIFO command*/
  fifoword = Param1;
  nbBytes = (NbParams < 3U) ? NbParams : 3U;

  for (count = 0U; count < nbBytes; count++)
  {
    fifoword |= (((uint32_t)(*(pparams + count))) << (8U + (8U * count)));
  }
  putreg32(fifoword, STM32_DSI_GPDR);

  uicounter = NbParams - nbBytes;
  pparams += nbBytes;
  /* Set the Next parameters on the write FIFO command*/
  while (uicounter != 0U)
  {
    nbBytes = (uicounter < 4U) ? uicounter : 4U;
    fifoword = 0U;
    for (count = 0U; count < nbBytes; count++)
    {
      fifoword |= (((uint32_t)(*(pparams + count))) << (8U * count));
    }
    putreg32(fifoword, STM32_DSI_GPDR);

    uicounter -= nbBytes;
    pparams += nbBytes;
  }

  /* Configure the packet to send a long DCS command */
  dsi_config_pkt_hdr( ChannelID,
                         Mode,
                         ((NbParams + 1U) & 0x00FFU),
                         (((NbParams + 1U) & 0xFF00U) >> 8U));


  return OK;
}

/**
  * @brief  Read command (DCS or generic)
  * @param  hdsi  pointer to a Host_DSI_t structure that contains
  *               the configuration information for the DSI.
  * @param  ChannelNbr  Virtual channel ID
  * @param  Array pointer to a buffer to store the payload of a read back operation.
  * @param  Size  Data size to be read (in byte).
  * @param  Mode  DSI read packet data type.
  *               This parameter can be any value of @arg DSI_SHORT_READ_PKT_Data_Type.
  * @param  DCSCmd  DCS get/read command.
  * @param  ParametersTable  Pointer to parameter values table.
  * @retval HAL status
  */
int stm32_dsi_read(Host_DSI_t *hdsi,
                    uint32_t ChannelNbr,
                    uint8_t *Array,
                    uint32_t Size,
                    uint32_t Mode,
                    uint32_t DCSCmd,
                    uint8_t *ParametersTable)
{
  uint32_t tickstart;
  uint8_t *pdata = Array;
  uint32_t datasize = Size;
  uint32_t fifoword;
  uint32_t nbbytes;
  uint32_t count;



  if (datasize > 2U)
  {
    /* set max return packet size */
    if (stm32_dsi_short_write(hdsi, ChannelNbr, DSI_MAX_RETURN_PKT_SIZE, ((datasize) & 0xFFU),
                           (((datasize) >> 8U) & 0xFFU)) != OK)
    {
      return -EIO;
    }
  }

  /* Configure the packet to read command */
  if (Mode == DSI_DCS_SHORT_PKT_READ)
  {
    dsi_config_pkt_hdr( ChannelNbr, Mode, DCSCmd, 0U);
  }
  else if (Mode == DSI_GEN_SHORT_PKT_READ_P0)
  {
    dsi_config_pkt_hdr( ChannelNbr, Mode, 0U, 0U);
  }
  else if (Mode == DSI_GEN_SHORT_PKT_READ_P1)
  {
    dsi_config_pkt_hdr( ChannelNbr, Mode, ParametersTable[0U], 0U);
  }
  else if (Mode == DSI_GEN_SHORT_PKT_READ_P2)
  {
    dsi_config_pkt_hdr( ChannelNbr, Mode, ParametersTable[0U], ParametersTable[1U]);
  }
  else
  {
    return -EIO;
  }

  /* Get tick */
  tickstart = clock_systime_ticks();

  /* If DSI fifo is not empty, read requested bytes */
  while (((int32_t)(datasize)) > 0)
  {
    if ((getreg32(STM32_DSI_GPSR) & DSI_GPSR_PRDFE) == 0U)
    {
      fifoword = getreg32(STM32_DSI_GPDR);
      nbbytes = (datasize < 4U) ? datasize : 4U;

      for (count = 0U; count < nbbytes; count++)
      {
        *pdata = (uint8_t)(fifoword >> (8U * count));
        pdata++;
        datasize--;
      }
    }

    /* Check for the Timeout */
    if ((clock_systime_ticks() - tickstart) > DSI_TIMEOUT_VALUE)
    {
      return -ETIME;
    }
  }


  return OK;
}

/**
  * @brief  Enter the ULPM (Ultra Low Power Mode) with the D-PHY PLL running
  *         (only data lanes are in ULPM)
  * @param  hdsi  pointer to a Host_DSI_t structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
int stm32_dsi_enter_ulpm_data(Host_DSI_t *hdsi)
{
  uint32_t tickstart;


  /* ULPS Request on Data Lanes */
  modifyreg32(STM32_DSI_PUCR, 0, DSI_PUCR_URDL);

  /* Get tick */
  tickstart = clock_systime_ticks();

  /* Wait until the D-PHY active lanes enter into ULPM */
  if ((getreg32(STM32_DSI_PCONFR) & DSI_PCONFR_NL) == DSI_ONE_DATA_LANE)
  {
    while ((getreg32(STM32_DSI_PSR) & DSI_PSR_UAN0) != 0U)
    {
      /* Check for the Timeout */
      if ((clock_systime_ticks() - tickstart) > DSI_TIMEOUT_VALUE)
      {
        return -ETIME;
      }
    }
  }
  else if ((getreg32(STM32_DSI_PCONFR) & DSI_PCONFR_NL) == DSI_TWO_DATA_LANES)
  {
    while ((getreg32(STM32_DSI_PSR) & (DSI_PSR_UAN0 | DSI_PSR_UAN1)) != 0U)
    {
      /* Check for the Timeout */
      if ((clock_systime_ticks() - tickstart) > DSI_TIMEOUT_VALUE)
      {

        return -ETIME;
      }
    }
  }
  else
  {
    return -EIO;
  }


  return OK;
}

/**
  * @brief  Exit the ULPM (Ultra Low Power Mode) with the D-PHY PLL running
  *         (only data lanes are in ULPM)
  * @param  hdsi  pointer to a Host_DSI_t structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
int stm32_dsi_exit_ulpm_data(Host_DSI_t *hdsi)
{
  uint32_t tickstart;


  /* Exit ULPS on Data Lanes */
  modifyreg32(STM32_DSI_PUCR, 0, DSI_PUCR_UEDL);

  /* Get tick */
  tickstart = clock_systime_ticks();

  /* Wait until all active lanes exit ULPM */
  if ((getreg32(STM32_DSI_PCONFR) & DSI_PCONFR_NL) == DSI_ONE_DATA_LANE)
  {
    while ((getreg32(STM32_DSI_PSR) & DSI_PSR_UAN0) != DSI_PSR_UAN0)
    {
      /* Check for the Timeout */
      if ((clock_systime_ticks() - tickstart) > DSI_TIMEOUT_VALUE)
      {
        return -ETIME;
      }
    }
  }
  else if ((getreg32(STM32_DSI_PCONFR) & DSI_PCONFR_NL) == DSI_TWO_DATA_LANES)
  {
    while ((getreg32(STM32_DSI_PSR) & (DSI_PSR_UAN0 | DSI_PSR_UAN1)) != (DSI_PSR_UAN0 | DSI_PSR_UAN1))
    {
      /* Check for the Timeout */
      if ((clock_systime_ticks() - tickstart) > DSI_TIMEOUT_VALUE)
      {
        return -ETIME;
      }
    }
  }
  else
  {
    return -EIO;
  }

  /* wait for 1 ms*/
  usleep(1000);

  /* De-assert the ULPM requests and the ULPM exit bits */
  putreg32(0U, STM32_DSI_PUCR);


  return OK;
}

/**
  * @brief  Enter the ULPM (Ultra Low Power Mode) with the D-PHY PLL turned off
  *         (both data and clock lanes are in ULPM)
  * @param  hdsi  pointer to a Host_DSI_t structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
int stm32_dsi_enter_ulpm(Host_DSI_t *hdsi)
{
  uint32_t tickstart;


  /* Clock lane configuration: no more HS request */
  modifyreg32(STM32_DSI_CLCR, DSI_CLCR_DPCC, 0);

  /* Use system PLL as byte lane clock source before stopping DSIPHY clock source */
  STM32_RCC_DSI_CONFIG(RCC_DSICLKSOURCE_PLL2);

  /* ULPS Request on Clock and Data Lanes */
  modifyreg32(STM32_DSI_PUCR, 0, (DSI_PUCR_URCL | DSI_PUCR_URDL));

  /* Get tick */
  tickstart = clock_systime_ticks();

  /* Wait until all active lanes exit ULPM */
  if ((getreg32(STM32_DSI_PCONFR) & DSI_PCONFR_NL) == DSI_ONE_DATA_LANE)
  {
    while ((getreg32(STM32_DSI_PSR) & (DSI_PSR_UAN0 | DSI_PSR_UANC)) != 0U)
    {
      /* Check for the Timeout */
      if ((clock_systime_ticks() - tickstart) > DSI_TIMEOUT_VALUE)
      {
        return -ETIME;
      }
    }
  }
  else if ((getreg32(STM32_DSI_PCONFR) & DSI_PCONFR_NL) == DSI_TWO_DATA_LANES)
  {
    while ((getreg32(STM32_DSI_PSR) & (DSI_PSR_UAN0 | DSI_PSR_UAN1 | DSI_PSR_UANC)) != 0U)
    {
      /* Check for the Timeout */
      if ((clock_systime_ticks() - tickstart) > DSI_TIMEOUT_VALUE)
      {
        return -ETIME;
      }
    }
  }
  else
  {

    return -EIO;
  }

  /* Turn off the DSI PLL */
  STM32_DSI_PLL_DISABLE(hdsi);


  return OK;
}

/**
  * @brief  Exit the ULPM (Ultra Low Power Mode) with the D-PHY PLL turned off
  *         (both data and clock lanes are in ULPM)
  * @param  hdsi  pointer to a Host_DSI_t structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
int stm32_dsi_exit_ulpm(Host_DSI_t *hdsi)
{
  uint32_t tickstart;


  /* Turn on the DSI PLL */
  STM32_DSI_PLL_ENABLE(hdsi);

  /* Get tick */
  tickstart = clock_systime_ticks();

  /* Wait for the lock of the PLL */
  while (STM32_DSI_GET_FLAG(hdsi, DSI_FLAG_PLLLS) == 0U)
  {
    /* Check for the Timeout */
    if ((clock_systime_ticks() - tickstart) > DSI_TIMEOUT_VALUE)
    {
      return -ETIME;
    }
  }

  /* Exit ULPS on Clock and Data Lanes */
  modifyreg32(STM32_DSI_PUCR, 0, (DSI_PUCR_UECL | DSI_PUCR_UEDL));

  /* Get tick */
  tickstart = clock_systime_ticks();

  /* Wait until all active lanes exit ULPM */
  if ((getreg32(STM32_DSI_PCONFR) & DSI_PCONFR_NL) == DSI_ONE_DATA_LANE)
  {
    while ((getreg32(STM32_DSI_PSR) & (DSI_PSR_UAN0 | DSI_PSR_UANC)) != (DSI_PSR_UAN0 | DSI_PSR_UANC))
    {
      /* Check for the Timeout */
      if ((clock_systime_ticks() - tickstart) > DSI_TIMEOUT_VALUE)
      {
        return -ETIME;
      }
    }
  }
  else if ((getreg32(STM32_DSI_PCONFR) & DSI_PCONFR_NL) == DSI_TWO_DATA_LANES)
  {
    while ((getreg32(STM32_DSI_PSR) & (DSI_PSR_UAN0 | DSI_PSR_UAN1 | DSI_PSR_UANC)) != (DSI_PSR_UAN0 | DSI_PSR_UAN1 |
                                                                                    DSI_PSR_UANC))
    {
      /* Check for the Timeout */
      if ((clock_systime_ticks() - tickstart) > DSI_TIMEOUT_VALUE)
      {
        return -ETIME;
      }
    }
  }
  else
  {
    return -EIO;
  }

  /* wait for 1 ms */
  usleep(1000);

  /* De-assert the ULPM requests and the ULPM exit bits */
  putreg32(0, STM32_DSI_PUCR);

  /* Switch the lanbyteclock source in the RCC from system PLL to D-PHY */
  STM32_RCC_DSI_CONFIG(RCC_DSICLKSOURCE_PHY);

  /* Restore clock lane configuration to HS */
  modifyreg32(STM32_DSI_CLCR, 0, DSI_CLCR_DPCC);


  return OK;
}




/** @defgroup DSI_Group1 Initialization and Configuration functions
  *  @brief   Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
                ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize and configure the DSI
      (+) De-initialize the DSI

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the DSI according to the specified
  *         parameters in the DSI_Init_t and create the associated handle.
  * @param  hdsi  pointer to a struct Host_DSI_s structure that contains
  *               the configuration information for the DSI.
  * @param  PLLInit  pointer to a DSI_PLLInit_t structure that contains
  *                  the PLL Clock structure definition for the DSI.
  * @retval HAL status
  */
int stm32_dsi_init(struct Host_DSI_s *hdsi, struct DSI_PLL_Init_s *PLLInit)
{
  clock_t tickstart;
  uint32_t unitIntervalx4;
  uint32_t tempIDF;
  uint32_t regval;

  /* Check the DSI handle allocation */
  if (hdsi == NULL)
  {
    return -EINVAL;
  }




  /**************** Turn on the regulator and enable the DSI PLL ****************/

  /* Enable the regulator */
  STM32_DSI_REG_ENABLE(hdsi);

  /* Get tick */
  tickstart = clock_systime_ticks();

  /* Wait until the regulator is ready */
  while (STM32_DSI_GET_FLAG(hdsi, DSI_FLAG_RRS) == 0U)
  {
    /* Check for the Timeout */
    if ((clock_systime_ticks() - tickstart) > DSI_TIMEOUT_VALUE)
    {
      return -ETIME;
    }
  }

  /* Set the PLL division factors */
  regval = getreg32(STM32_DSI_WRPCR);
  regval &= ~(DSI_WRPCR_PLL_NDIV | DSI_WRPCR_PLL_IDF | DSI_WRPCR_PLL_ODF);
  regval |= (((PLLInit->PLLNDIV) << 2U) | ((PLLInit->PLLIDF) << 11U) | ((PLLInit->PLLODF) << 16U));
  putreg32(regval, STM32_DSI_WRPCR);

  /* Enable the DSI PLL */
  STM32_DSI_PLL_ENABLE(hdsi);

  /* Get tick */
  tickstart = clock_systime_ticks();

  /* Wait for the lock of the PLL */
  while (STM32_DSI_GET_FLAG(hdsi, DSI_FLAG_PLLLS) == 0U)
  {
    /* Check for the Timeout */
    if ((clock_systime_ticks() - tickstart) > DSI_TIMEOUT_VALUE)
    {
      return -ETIME;
    }
  }

  /*************************** Set the PHY parameters ***************************/

  /* D-PHY clock and digital enable*/
  modifyreg32(STM32_DSI_PCTLR, 0, DSI_PCTLR_CKE | DSI_PCTLR_DEN);
  
  /* Clock lane configuration */
  modifyreg32(STM32_DSI_CLCR, DSI_CLCR_DPCC | DSI_CLCR_ACR, 0);
  modifyreg32(STM32_DSI_CLCR, 0, DSI_CLCR_DPCC | hdsi->Init.AutomaticClockLaneControl);

  /* Configure the number of active data lanes */
  modifyreg32(STM32_DSI_PCONFR, DSI_PCONFR_NL, 0);
  modifyreg32(STM32_DSI_PCONFR, 0 , hdsi->Init.NumberOfLanes);

  /************************ Set the DSI clock parameters ************************/

  /* Set the TX escape clock division factor */
  modifyreg32(STM32_DSI_CCR, DSI_CCR_TXECKDIV, 0);
  modifyreg32(STM32_DSI_CCR, 0, hdsi->Init.TXEscapeCkdiv);

  /* Calculate the bit period in high-speed mode in unit of 0.25 ns (UIX4) */
  /* The equation is : UIX4 = IntegerPart( (1000/F_PHY_Mhz) * 4 )          */
  /* Where : F_PHY_Mhz = (NDIV * HSE_Mhz) / (IDF * ODF)                    */
  tempIDF = (PLLInit->PLLIDF > 0U) ? PLLInit->PLLIDF : 1U;
  unitIntervalx4 = (4000000U * tempIDF * ((1UL << (0x3U & PLLInit->PLLODF)))) / ((STM32_HSE_FREQUENCY / 1000U) * PLLInit->PLLNDIV);

  /* Set the bit period in high-speed mode */
  modifyreg32(STM32_DSI_WPCR, DSI_WPCR0_UIX4, 0);
  modifyreg32(STM32_DSI_WPCR, 0, unitIntervalx4);

  /****************************** Error management *****************************/

  /* Disable all error interrupts and reset the Error Mask */
  putreg32(0U, STM32_DSI_IER);
  putreg32(0U, STM32_DSI_IER + 4);



  return OK;
}


/****************************************************************************
 * Name: stm32_dsireset
 *
 * Description:
 *   Reset DSI via APB3RSTR
 *
 ****************************************************************************/
void stm32_dsireset(void)
{
  uint32_t regval = getreg32(STM32_RCC_APB3RSTR);
  putreg32(regval | RCC_APB3RSTR_DSIRST, STM32_RCC_APB3RSTR);
  putreg32(regval & ~RCC_APB3RSTR_DSIRST, STM32_RCC_APB3RSTR);
}
