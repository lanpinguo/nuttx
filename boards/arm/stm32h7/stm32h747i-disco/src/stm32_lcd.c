/****************************************************************************
 * boards/arm/stm32f7/stm32f746g-disco/src/stm32_lcd.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/video/fb.h>

#include <arch/board/board.h>


#ifdef CONFIG_STM32H7_LTDC

#include "arm_internal.h"
#include "stm32h747i-disco.h"
#include "stm32_gpio.h"
#include "stm32_ltdc.h"
#include "hardware/stm32_dsi.h"
#include "stm32_dsi.h"
#include "otm8009a.h"


/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_lcd_initialize(void);



/****************************************************************************
 * Private variables
 ****************************************************************************/

/* This structure describes the state of this driver */
static bool g_initialized;

Host_DSI_t hlcd_dsi;
static OTM8009A_Object_t   OTM8009AObj;



/****************************************************************************
 * Private Functions
 ****************************************************************************/


int32_t DSI_IO_GetTick(void)
{
  return (int32_t)clock_systime_ticks();
}

/**
  * @brief  DCS or Generic short/long write command
  * @param  ChannelNbr Virtual channel ID
  * @param  Reg Register to be written
  * @param  pData pointer to a buffer of data to be write
  * @param  Size To precise command to be used (short or long)
  * @retval BSP status
  */
static int32_t DSI_IO_Write(uint16_t ChannelNbr, uint16_t Reg, uint8_t *pData, uint16_t Size)
{
  int32_t ret = OK;

  if(Size <= 1U)
  {
    if(stm32_dsi_short_write(&hlcd_dsi, ChannelNbr, DSI_DCS_SHORT_PKT_WRITE_P1, Reg, (uint32_t)pData[Size]) != OK)
    {
      ret = -EBADE;
    }
  }
  else
  {
    if(stm32_dsi_long_write(&hlcd_dsi, ChannelNbr, DSI_DCS_LONG_PKT_WRITE, Size, (uint32_t)Reg, pData) != OK)
    {
      ret = -EBADE;
    }
  }

  return ret;
}

/**
  * @brief  DCS or Generic read command
  * @param  ChannelNbr Virtual channel ID
  * @param  Reg Register to be read
  * @param  pData pointer to a buffer to store the payload of a read back operation.
  * @param  Size  Data size to be read (in byte).
  * @retval BSP status
  */
static int32_t DSI_IO_Read(uint16_t ChannelNbr, uint16_t Reg, uint8_t *pData, uint16_t Size)
{
  int32_t ret = OK;

  if(stm32_dsi_read(&hlcd_dsi, ChannelNbr, pData, Size, DSI_DCS_SHORT_PKT_READ, Reg, pData) != OK)
  {
    ret = -EBADE;
  }

  return ret;
}

#if 1
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer video hardware associated with the display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  int ret = OK;

  /* Custom LCD display with RGB interface */

  ret = board_lcd_initialize();

  return  ret;
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane of the specified plane.
 *   Many OSDs support multiple planes of video.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *   vplane - Identifies the plane being queried.
 *
 * Returned Value:
 *   A non-NULL pointer to the frame buffer access structure is returned on
 *   success; NULL is returned on any failure.
 *
 ****************************************************************************/

struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  return stm32_ltdcgetvplane(vplane);
}

/****************************************************************************
 * Name: up_fbuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer support for the specified display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_fbuninitialize(int display)
{
  stm32_ltdcuninitialize();
}

#endif


/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.
 *   The initial state of the LCD is fully initialized, display memory
 *   cleared, and the LCD ready to use, but with the power setting at 0
 *   (full off).
 *
 ****************************************************************************/
int board_lcd_initialize(void)
{
  int ret = OK;

  DSI_PLL_Init_t dsiPllInit;
  DSI_CmdCfg_t CmdCfg;
  DSI_LPCmd_t LPCmd;
  DSI_PHY_Timer_t  PhyTimings;

  if (g_initialized == true)
  {
    return ret;
  }


  ginfo("Initializing\n");

  stm32_configgpio(GPIO_LCD_RESET);
  stm32_configgpio(GPIO_LCD_BL_CTL);

  stm32_gpiowrite(GPIO_LCD_BL_CTL, false);

  /* Activate XRES active low */
  stm32_gpiowrite(GPIO_LCD_RESET, false);
  usleep(20 * 1000); /* wait 20 ms */
  stm32_gpiowrite(GPIO_LCD_RESET, true);
  usleep(10 * 1000); /* Wait for 10ms after releasing XRES before sending commands */


  stm32_ltdc_reset();


  dsiPllInit.PLLNDIV  = 100;
  dsiPllInit.PLLIDF   = DSI_PLL_IN_DIV5;
  dsiPllInit.PLLODF  = DSI_PLL_OUT_DIV1;  

  hlcd_dsi.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
  hlcd_dsi.Init.TXEscapeCkdiv = 0x4;
  
  stm32_dsi_reset();
  stm32_dsi_init(&(hlcd_dsi), &(dsiPllInit));

  /* Configure the DSI for Command mode */
  CmdCfg.VirtualChannelID      = 0;
  CmdCfg.HSPolarity            = DSI_HSYNC_ACTIVE_HIGH;
  CmdCfg.VSPolarity            = DSI_VSYNC_ACTIVE_HIGH;
  CmdCfg.DEPolarity            = DSI_DATA_ENABLE_ACTIVE_HIGH;
  CmdCfg.ColorCoding           = DSI_RGB888;
  CmdCfg.CommandSize           = BOARD_LTDC_WIDTH;
  CmdCfg.TearingEffectSource   = DSI_TE_DSILINK;
  CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
  CmdCfg.VSyncPol              = DSI_VSYNC_FALLING;
  CmdCfg.AutomaticRefresh      = DSI_AR_DISABLE;
  CmdCfg.TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_ENABLE;
  stm32_dsi_config_adapted_command_mode(&hlcd_dsi, &CmdCfg);
  
  LPCmd.LPGenShortWriteNoP    = DSI_LP_GSW0P_ENABLE;
  LPCmd.LPGenShortWriteOneP   = DSI_LP_GSW1P_ENABLE;
  LPCmd.LPGenShortWriteTwoP   = DSI_LP_GSW2P_ENABLE;
  LPCmd.LPGenShortReadNoP     = DSI_LP_GSR0P_ENABLE;
  LPCmd.LPGenShortReadOneP    = DSI_LP_GSR1P_ENABLE;
  LPCmd.LPGenShortReadTwoP    = DSI_LP_GSR2P_ENABLE;
  LPCmd.LPGenLongWrite        = DSI_LP_GLW_ENABLE;
  LPCmd.LPDcsShortWriteNoP    = DSI_LP_DSW0P_ENABLE;
  LPCmd.LPDcsShortWriteOneP   = DSI_LP_DSW1P_ENABLE;
  LPCmd.LPDcsShortReadNoP     = DSI_LP_DSR0P_ENABLE;
  LPCmd.LPDcsLongWrite        = DSI_LP_DLW_ENABLE;
  stm32_dsi_config_command(&hlcd_dsi, &LPCmd);

  /* Initialize LTDC */
  lcdinfo("Configure lcd periphery\n");
  stm32_ltdc_init();  


  /* Start DSI */
  stm32_dsi_start(&(hlcd_dsi));

  /* Configure DSI PHY HS2LP and LP2HS timings */
  PhyTimings.ClockLaneHS2LPTime = 35;
  PhyTimings.ClockLaneLP2HSTime = 35;
  PhyTimings.DataLaneHS2LPTime = 35;
  PhyTimings.DataLaneLP2HSTime = 35;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 10;
  stm32_dsi_config_phy_timer(&hlcd_dsi, &PhyTimings);   

  OTM8009A_IO_t  IOCtx;  
  /* Initialize the OTM8009A LCD Display IC Driver (KoD LCD IC Driver) */
  IOCtx.Address     = 0;
  IOCtx.GetTick     = DSI_IO_GetTick;
  IOCtx.WriteReg    = DSI_IO_Write;
  IOCtx.ReadReg     = DSI_IO_Read;
  OTM8009A_RegisterBusIO(&OTM8009AObj, &IOCtx);
  OTM8009A_Init(&OTM8009AObj, OTM8009A_COLMOD_RGB888, LCD_ORIENTATION_LANDSCAPE);
  
  
  LPCmd.LPGenShortWriteNoP    = DSI_LP_GSW0P_DISABLE;
  LPCmd.LPGenShortWriteOneP   = DSI_LP_GSW1P_DISABLE;
  LPCmd.LPGenShortWriteTwoP   = DSI_LP_GSW2P_DISABLE;
  LPCmd.LPGenShortReadNoP     = DSI_LP_GSR0P_DISABLE;
  LPCmd.LPGenShortReadOneP    = DSI_LP_GSR1P_DISABLE;
  LPCmd.LPGenShortReadTwoP    = DSI_LP_GSR2P_DISABLE;
  LPCmd.LPGenLongWrite        = DSI_LP_GLW_DISABLE;
  LPCmd.LPDcsShortWriteNoP    = DSI_LP_DSW0P_DISABLE;
  LPCmd.LPDcsShortWriteOneP   = DSI_LP_DSW1P_DISABLE;
  LPCmd.LPDcsShortReadNoP     = DSI_LP_DSR0P_DISABLE;
  LPCmd.LPDcsLongWrite        = DSI_LP_DLW_DISABLE;
  stm32_dsi_config_command(&hlcd_dsi, &LPCmd);
  
  stm32_dsi_config_flow_control(&hlcd_dsi, DSI_FLOW_CONTROL_BTA);
  stm32_dsi_force_rx_lowPower(&hlcd_dsi, 1);  
  

  // stm32_dsi_pattern_generator_start(&hlcd_dsi, 0, 1);

  stm32_dsi_wrapper_set(&hlcd_dsi, false);
  stm32_ltdc_layer_init();
  stm32_dsi_wrapper_set(&hlcd_dsi, true);

  memset(0xd0000000, 0xFF, 480 * 800 * 4);

  stm32_dsi_refresh(&hlcd_dsi);

  g_initialized = true;

  return 0;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.
 *    This allows support for multiple LCD devices.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  return stm32_ltdc_getdev(0);
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{

  /* Turn the display off */

}



#endif /* CONFIG_STM32H7_LTDC */
