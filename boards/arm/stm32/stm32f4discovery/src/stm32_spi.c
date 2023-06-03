/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_spi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"

#include "stm32f4discovery.h"

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3)


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f4discovery
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void)
{

    stm32_configgpio(GPIO_WIRELESS_INT);
    stm32_configgpio(GPIO_WIRELESS_CS_0);
    stm32_configgpio(GPIO_WIRELESS_CS_1);
    stm32_configgpio(GPIO_WIRELESS_CS_2);
    stm32_configgpio(GPIO_WIRELESS_CS_3);
    stm32_configgpio(GPIO_WIRELESS_CS_4);
    stm32_configgpio(GPIO_WIRELESS_CS_5);
    stm32_configgpio(GPIO_WIRELESS_CS_6);
    stm32_configgpio(GPIO_WIRELESS_CS_7);
    stm32_configgpio(GPIO_WIRELESS_CS_8);
    stm32_configgpio(GPIO_WIRELESS_CS_9);
    stm32_configgpio(GPIO_WIRELESS_CS_10);
    stm32_configgpio(GPIO_WIRELESS_CS_11);
    stm32_configgpio(GPIO_WIRELESS_CS_12);
    stm32_configgpio(GPIO_WIRELESS_CS_13);
    stm32_configgpio(GPIO_WIRELESS_CS_14);
    stm32_configgpio(GPIO_WIRELESS_CS_15);


}

/****************************************************************************
 * Name:  stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   stm32_spibus_initialize()) are provided by common STM32 logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board
 *      is configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI1
void stm32_spi1select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" :
          "de-assert");

  switch (devid)
  {
  case SPIDEV_IEEE802154(0):
    stm32_gpiowrite(GPIO_WIRELESS_CS_0, !selected);
    break;
  case SPIDEV_IEEE802154(1):
    stm32_gpiowrite(GPIO_WIRELESS_CS_1, !selected);
    break;
  case SPIDEV_IEEE802154(2):
    stm32_gpiowrite(GPIO_WIRELESS_CS_2, !selected);
    break;
  case SPIDEV_IEEE802154(3):
    stm32_gpiowrite(GPIO_WIRELESS_CS_3, !selected);
    break;
  case SPIDEV_IEEE802154(4):
    stm32_gpiowrite(GPIO_WIRELESS_CS_4, !selected);
    break;
  case SPIDEV_IEEE802154(5):
    stm32_gpiowrite(GPIO_WIRELESS_CS_5, !selected);
    break;
  case SPIDEV_IEEE802154(6):
    stm32_gpiowrite(GPIO_WIRELESS_CS_6, !selected);
    break;
  case SPIDEV_IEEE802154(7):
    stm32_gpiowrite(GPIO_WIRELESS_CS_7, !selected);
    break;
  case SPIDEV_IEEE802154(8):
    stm32_gpiowrite(GPIO_WIRELESS_CS_8, !selected);
    break;
  case SPIDEV_IEEE802154(9):
    stm32_gpiowrite(GPIO_WIRELESS_CS_9, !selected);
    break;
  case SPIDEV_IEEE802154(10):
    stm32_gpiowrite(GPIO_WIRELESS_CS_10, !selected);
    break;
  case SPIDEV_IEEE802154(11):
    stm32_gpiowrite(GPIO_WIRELESS_CS_11, !selected);
    break;
  case SPIDEV_IEEE802154(12):
    stm32_gpiowrite(GPIO_WIRELESS_CS_12, !selected);
    break;
  case SPIDEV_IEEE802154(13):
    stm32_gpiowrite(GPIO_WIRELESS_CS_13, !selected);
    break;
  case SPIDEV_IEEE802154(14):
    stm32_gpiowrite(GPIO_WIRELESS_CS_14, !selected);
    break;
  case SPIDEV_IEEE802154(15):
    stm32_gpiowrite(GPIO_WIRELESS_CS_15, !selected);
    break;
  
  default:
    break;
  };


}

uint8_t stm32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

#ifdef CONFIG_LPWAN_SX127X
  if (devid == SPIDEV_LPWAN(0))
    {
      status |= SPI_STATUS_PRESENT;
    }
#endif

  return status;
}
#endif

#ifdef CONFIG_STM32_SPI2
void stm32_spi2select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" :
          "de-assert");

#if defined(CONFIG_SENSORS_MAX31855)
  if (devid == SPIDEV_TEMPERATURE(0))
    {
      stm32_gpiowrite(GPIO_MAX31855_CS, !selected);
    }
#endif

#if defined(CONFIG_SENSORS_MAX6675)
  if (devid == SPIDEV_TEMPERATURE(0))
    {
      stm32_gpiowrite(GPIO_MAX6675_CS, !selected);
    }
#endif

#if defined(CONFIG_MMCSD_SPI)
  if (devid == SPIDEV_MMCSD(0))
    {
      stm32_gpiowrite(GPIO_MMCSD_NSS, !selected);
    }
#endif
}

uint8_t stm32_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;
#if defined(CONFIG_MMCSD_SPI)
  if (devid == SPIDEV_MMCSD(0))
    {
      /* Note: SD_DET is pulled high when there's no SD card present. */

      ret = stm32_gpioread(GPIO_MMCSD_NCD) ? 0 : 1;
    }
#endif

  return ret;
}
#endif

#ifdef CONFIG_STM32_SPI3
void stm32_spi3select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" :
          "de-assert");

#if defined(CONFIG_WL_GS2200M)
  if (devid == SPIDEV_WIRELESS(0))
    {
      stm32_gpiowrite(GPIO_GS2200M_CS, !selected);
    }
#endif
}

uint8_t stm32_spi3status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_spi1cmddata
 *
 * Description:
 *   Set or clear the SH1101A A0 or SD1306 D/C n bit to select data (true)
 *   or command (false). This function must be provided by platform-specific
 *   logic. This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_STM32_SPI1
int stm32_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#if defined(CONFIG_LCD_ST7567) || defined(CONFIG_LCD_ST7789)
  if (devid == SPIDEV_DISPLAY(0))
    {
      /*  This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       */

      stm32_gpiowrite(STM32_LCD_RS, !cmd);

      return OK;
    }
#endif

#if defined(CONFIG_LCD_UG2864AMBAG01) || defined(CONFIG_LCD_UG2864HSWEG01) || \
    defined(CONFIG_LCD_SSD1351)
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* "This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       *
       *  A0 = "H": the inputs at D0 to D7 are treated as display data.
       *  A0 = "L": the inputs at D0 to D7 are transferred to the command
       *       registers."
       */

# if defined(CONFIG_LCD_UG2864AMBAG01)
      stm32_gpiowrite(GPIO_OLED_A0, !cmd);
# endif
# if defined(CONFIG_LCD_UG2864HSWEG01) || defined(CONFIG_LCD_SSD1351)
      stm32_gpiowrite(GPIO_OLED_DC, !cmd);
# endif
      return OK;
    }
#endif

  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32_SPI2
int stm32_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32_SPI3
int stm32_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif /* CONFIG_SPI_CMDDATA */

#endif /* CONFIG_STM32_SPI1 || CONFIG_STM32_SPI2 */
