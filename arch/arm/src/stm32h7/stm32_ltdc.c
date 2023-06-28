/****************************************************************************
 * arch/arm/src/STM32H7/stm32_ltdc.c
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

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/stm32_ltdc.h"
#include "hardware/stm32_dma2d.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_ltdc.h"
#include "stm32_dma2d.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register definition ******************************************************/

#ifndef BOARD_LTDC_WIDTH
#  error BOARD_LTDC_WIDTH must be defined in the board.h header file
#endif

#ifndef BOARD_LTDC_HEIGHT
#  error BOARD_LTDC_HEIGHT must be defined in the board.h header file
#endif

#define STM32_LTDC_HEIGHT           BOARD_LTDC_HEIGHT
#define STM32_LTDC_WIDTH            BOARD_LTDC_WIDTH

/* Configure LTDC register */

/* Configure LTDC register */

#define STM32_LTDC_ACCUMULATED_HBP  (BOARD_LTDC_HSYNC + BOARD_LTDC_HBP)
#define STM32_LTDC_ACCUMULATED_VBP  (BOARD_LTDC_VSYNC + BOARD_LTDC_VBP)


#define STM32_LTDC_ACCUM_ACT_W      (BOARD_LTDC_HSYNC + BOARD_LTDC_HBP + BOARD_LTDC_WIDTH)
#define STM32_LTDC_ACCUM_ACT_H      (BOARD_LTDC_VSYNC + BOARD_LTDC_VBP + BOARD_LTDC_HEIGHT)


#define STM32_LTDC_TOTAL_W          (BOARD_LTDC_HSYNC + BOARD_LTDC_HBP + BOARD_LTDC_WIDTH + BOARD_LTDC_HFP)
#define STM32_LTDC_TOTAL_H          (BOARD_LTDC_VSYNC + BOARD_LTDC_VBP + BOARD_LTDC_HEIGHT + BOARD_LTDC_VFP)

#define STM32_LTDC_BKG_COLOR_R      0
#define STM32_LTDC_BKG_COLOR_G      0
#define STM32_LTDC_BKG_COLOR_B      0

void stm32_ltdc_init(void)
{
  uint32_t regval;


    /* Configure the HS, VS, DE and PC polarity */
  modifyreg32(STM32_LTDC_GCR,
        (LTDC_GCR_HSPOL | LTDC_GCR_VSPOL | LTDC_GCR_DEPOL | LTDC_GCR_PCPOL), 
        (BOARD_LTDC_GCR_HSPOL | BOARD_LTDC_GCR_VSPOL | BOARD_LTDC_GCR_DEPOL | BOARD_LTDC_GCR_PCPOL));

  /* Set Synchronization size */
  modifyreg32(STM32_LTDC_SSCR, (LTDC_SSCR_VSH | LTDC_SSCR_HSW), 
      (BOARD_LTDC_HSYNC << 16U) | BOARD_LTDC_VSYNC );

  /* Set Accumulated Back porch */
  modifyreg32(STM32_LTDC_BPCR, (LTDC_BPCR_AVBP | LTDC_BPCR_AHBP), 
      (STM32_LTDC_ACCUMULATED_HBP << 16U) | STM32_LTDC_ACCUMULATED_VBP);

  /* Set Accumulated Active Width */
  modifyreg32(STM32_LTDC_AWCR, (LTDC_AWCR_AAH | LTDC_AWCR_AAW),
      (STM32_LTDC_ACCUM_ACT_W<< 16U) | STM32_LTDC_ACCUM_ACT_H);

  /* Set Total Width */
  modifyreg32(STM32_LTDC_TWCR, (LTDC_TWCR_TOTALH | LTDC_TWCR_TOTALW),
      (STM32_LTDC_TOTAL_W << 16U) | STM32_LTDC_TOTAL_H);

  /* Set the background color value */

  modifyreg32(STM32_LTDC_BCCR, (LTDC_BCCR_BCBLUE | LTDC_BCCR_BCGREEN | LTDC_BCCR_BCRED), \
      (STM32_LTDC_BKG_COLOR_G << 8U) | (STM32_LTDC_BKG_COLOR_R << 16U) | \
       STM32_LTDC_BKG_COLOR_R );


  /* Enable the Transfer Error and FIFO underrun interrupts */
  //__HAL_LTDC_ENABLE_IT(hltdc, LTDC_IT_TE | LTDC_IT_FU);

  modifyreg32(STM32_LTDC_GCR, 0, LTDC_GCR_LTDCEN);

}


/****************************************************************************
 * Name: stm32_ltdcreset
 *
 * Description:
 *   Reset LTDC via APB3RSTR
 *
 ****************************************************************************/

void stm32_ltdcreset(void)
{
  uint32_t regval = getreg32(STM32_RCC_APB3RSTR);
  putreg32(regval | RCC_APB3RSTR_LTDCRST, STM32_RCC_APB3RSTR);
  putreg32(regval & ~RCC_APB3RSTR_LTDCRST, STM32_RCC_APB3RSTR);
  
  // dummy read
  regval = getreg32(STM32_RCC_APB3RSTR);

}


/** @defgroup LTDC_Private_Functions LTDC Private Functions
  * @{
  */

/**
  * @brief  Configure the LTDC peripheral
  * @param  hltdc     Pointer to a LTDC_HandleTypeDef structure that contains
  *                   the configuration information for the LTDC.
  * @param  pLayerCfg Pointer LTDC Layer Configuration structure
  * @param  LayerIdx  LTDC Layer index.
  *                   This parameter can be one of the following values: LTDC_LAYER_1 (0) or LTDC_LAYER_2 (1)
  * @retval None
  */
static void stm32_ltdc_set_config(LTDC_LayerCfg_t *pLayerCfg, uint32_t LayerIdx)
{
  uint32_t tmp;
  uint32_t tmp1;
  uint32_t tmp2;

  if(LayerIdx == 0){
    /* Configure the horizontal start and stop position */
    tmp = ((pLayerCfg->WindowX1 + ((getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AHBP) >> 16U)) << 16U);
    modifyreg32(STM32_LTDC_L1WHPCR, (LTDC_LxWHPCR_WHSTPOS | LTDC_LxWHPCR_WHSPPOS), 
                ((pLayerCfg->WindowX0 + ((getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AHBP) >> 16U) + 1U) | tmp));

    /* Configure the vertical start and stop position */
    tmp = ((pLayerCfg->WindowY1 + (getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AVBP)) << 16U);
    modifyreg32(STM32_LTDC_L1WVPCR, (LTDC_LxWVPCR_WVSTPOS | LTDC_LxWVPCR_WVSPPOS),
                ((pLayerCfg->WindowY0 + (getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AVBP) + 1U) | tmp));

    /* Specifies the pixel format */
    modifyreg32(STM32_LTDC_L1PFCR, LTDC_LxPFCR_PF, 
                pLayerCfg->PixelFormat);

    /* Configure the default color values */
    tmp = ((uint32_t)(pLayerCfg->Backcolor.Green) << 8U);
    tmp1 = ((uint32_t)(pLayerCfg->Backcolor.Red) << 16U);
    tmp2 = (pLayerCfg->Alpha0 << 24U);
    modifyreg32(STM32_LTDC_L1DCCR, (LTDC_LxDCCR_DCBLUE | LTDC_LxDCCR_DCGREEN | LTDC_LxDCCR_DCRED | LTDC_LxDCCR_DCALPHA), 
                (pLayerCfg->Backcolor.Blue | tmp | tmp1 | tmp2));

    /* Specifies the constant alpha value */
    modifyreg32(STM32_LTDC_L1CACR, LTDC_LxCACR_CONSTA, pLayerCfg->Alpha);

    /* Specifies the blending factors */
    modifyreg32(STM32_LTDC_L1BFCR, (LTDC_LxBFCR_BF2 | LTDC_LxBFCR_BF1), 
                (pLayerCfg->BlendingFactor1 | pLayerCfg->BlendingFactor2));

    /* Configure the color frame buffer start address */
    modifyreg32(STM32_LTDC_L1CFBAR, (LTDC_LxCFBAR_CFBADD), 
                (pLayerCfg->FBStartAdress));

    if (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
    {
      tmp = 4U;
    }
    else if (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB888)
    {
      tmp = 3U;
    }
    else if ((pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB4444) || \
            (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB565)   || \
            (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB1555) || \
            (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_AL88))
    {
      tmp = 2U;
    }
    else
    {
      tmp = 1U;
    }

    /* Configure the color frame buffer pitch in byte */
    modifyreg32(STM32_LTDC_L1CFBLR, (LTDC_LxCFBLR_CFBLL | LTDC_LxCFBLR_CFBP), 
                (((pLayerCfg->ImageWidth * tmp) << 16U) | (((pLayerCfg->WindowX1 - pLayerCfg->WindowX0) * tmp)  + 7U)));
    /* Configure the frame buffer line number */
    modifyreg32(STM32_LTDC_L1CFBLNR, (LTDC_LxCFBLNR_CFBLNBR), pLayerCfg->ImageHeight);

    /* Enable LTDC_Layer by setting LEN bit */
    modifyreg32(STM32_LTDC_L1CR, 0, (uint32_t)LTDC_LxCR_LEN);

  }
  else{
    /* Configure the horizontal start and stop position */
    tmp = ((pLayerCfg->WindowX1 + ((getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AHBP) >> 16U)) << 16U);
    modifyreg32(STM32_LTDC_L2WHPCR, (LTDC_LxWHPCR_WHSTPOS | LTDC_LxWHPCR_WHSPPOS), 
                ((pLayerCfg->WindowX0 + ((getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AHBP) >> 16U) + 1U) | tmp));

    /* Configure the vertical start and stop position */
    tmp = ((pLayerCfg->WindowY1 + (getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AVBP)) << 16U);
    modifyreg32(STM32_LTDC_L2WVPCR, (LTDC_LxWVPCR_WVSTPOS | LTDC_LxWVPCR_WVSPPOS),
                ((pLayerCfg->WindowY0 + (getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AVBP) + 1U) | tmp));

    /* Specifies the pixel format */
    modifyreg32(STM32_LTDC_L2PFCR, LTDC_LxPFCR_PF, 
                pLayerCfg->PixelFormat);

    /* Configure the default color values */
    tmp = ((uint32_t)(pLayerCfg->Backcolor.Green) << 8U);
    tmp1 = ((uint32_t)(pLayerCfg->Backcolor.Red) << 16U);
    tmp2 = (pLayerCfg->Alpha0 << 24U);
    modifyreg32(STM32_LTDC_L2DCCR, (LTDC_LxDCCR_DCBLUE | LTDC_LxDCCR_DCGREEN | LTDC_LxDCCR_DCRED | LTDC_LxDCCR_DCALPHA), 
                (pLayerCfg->Backcolor.Blue | tmp | tmp1 | tmp2));

    /* Specifies the constant alpha value */
    modifyreg32(STM32_LTDC_L2CACR, LTDC_LxCACR_CONSTA, pLayerCfg->Alpha);

    /* Specifies the blending factors */
    modifyreg32(STM32_LTDC_L2BFCR, (LTDC_LxBFCR_BF2 | LTDC_LxBFCR_BF1), 
                (pLayerCfg->BlendingFactor1 | pLayerCfg->BlendingFactor2));

    /* Configure the color frame buffer start address */
    modifyreg32(STM32_LTDC_L2CFBAR, (LTDC_LxCFBAR_CFBADD), 
                (pLayerCfg->FBStartAdress));

    if (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
    {
      tmp = 4U;
    }
    else if (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB888)
    {
      tmp = 3U;
    }
    else if ((pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB4444) || \
            (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB565)   || \
            (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB1555) || \
            (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_AL88))
    {
      tmp = 2U;
    }
    else
    {
      tmp = 1U;
    }

    /* Configure the color frame buffer pitch in byte */
    modifyreg32(STM32_LTDC_L2CFBLR, (LTDC_LxCFBLR_CFBLL | LTDC_LxCFBLR_CFBP), 
                (((pLayerCfg->ImageWidth * tmp) << 16U) | (((pLayerCfg->WindowX1 - pLayerCfg->WindowX0) * tmp)  + 7U)));
    /* Configure the frame buffer line number */
    modifyreg32(STM32_LTDC_L2CFBLNR, (LTDC_LxCFBLNR_CFBLNBR), pLayerCfg->ImageHeight);

    /* Enable LTDC_Layer by setting LEN bit */
    modifyreg32(STM32_LTDC_L2CR, 0, (uint32_t)LTDC_LxCR_LEN);

  }
}



/**
  * @brief  Configure the LTDC Layer according to the specified
  *         parameters in the LTDC_InitTypeDef and create the associated handle.
  * @param  hltdc      pointer to a LTDC_HandleTypeDef structure that contains
  *                    the configuration information for the LTDC.
  * @param  pLayerCfg  pointer to a LTDC_LayerCfg_t structure that contains
  *                    the configuration information for the Layer.
  * @param  LayerIdx  LTDC Layer index.
  *                    This parameter can be one of the following values:
  *                    LTDC_LAYER_1 (0) or LTDC_LAYER_2 (1)
  * @retval HAL status
  */
int stm32_ltdc_config_layer( LTDC_LayerCfg_t *pLayerCfg, uint32_t LayerIdx)
{




  /* Configure the LTDC Layer */
  stm32_ltdc_set_config(pLayerCfg, LayerIdx);

  /* Set the Immediate Reload type */
  putreg32(LTDC_SRCR_IMR, STM32_LTDC_SRCR);



  return OK;
}





void stm32_ltdc_layer_init(void)
{
  LTDC_LayerCfg_t  layercfg;

  /* Layer Init */
  layercfg.WindowX0 = 0;
  layercfg.WindowX1 = 800 ;
  layercfg.WindowY0 = 0;
  layercfg.WindowY1 = 480; 
  layercfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  layercfg.FBStartAdress = 0xD0000000;
  layercfg.Alpha = 255;
  layercfg.Alpha0 = 0;
  layercfg.Backcolor.Blue = 0;
  layercfg.Backcolor.Green = 0;
  layercfg.Backcolor.Red = 0;
  layercfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  layercfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  layercfg.ImageWidth = 800;
  layercfg.ImageHeight = 480;

  stm32_ltdc_config_layer(&layercfg, 0); 

}