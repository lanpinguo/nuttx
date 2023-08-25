/****************************************************************************
 * boards/arm/stm32l4/nucleo-l452re/src/stm32_bringup.c
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

#include <sys/types.h>
#include <debug.h>

#include <nuttx/input/buttons.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#if defined(CONFIG_SENSORS_SHT21)
#include <nuttx/sensors/sht21.h>
#endif

#if defined(CONFIG_SENSORS_BH1750FVI)
#include <nuttx/sensors/bh1750fvi.h>
#endif

#ifdef CONFIG_RELAY_GPIO
uint32_t board_relay_io_initialize(void);
#endif

#include "stm32l4_i2c.h"
#include "nucleo-l452re.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_I2C_DRIVER
#if defined(CONFIG_STM32L4_I2C1) && defined(CONFIG_I2C_DRIVER)
#  define HAVE_I2C_DRIVER 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
#ifdef HAVE_I2C_DRIVER
  struct i2c_master_s *i2c;
#endif
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
#ifdef CONFIG_INPUT_BUTTONS_LOWER
  iinfo("Initializing button driver\n");

  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      ierr("ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#else
  /* Enable BUTTON support for some other purpose */

  board_button_initialize();
#endif
#endif /* CONFIG_INPUT_BUTTONS */


#if defined(CONFIG_USERLED) && !defined(CONFIG_ARCH_LEDS)
#ifdef CONFIG_USERLED_LOWER
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#else
  /* Enable USER LED support for some other purpose */

  board_userled_initialize();
#endif /* CONFIG_USERLED_LOWER */
#endif /* CONFIG_USERLED && !CONFIG_ARCH_LEDS */

#ifdef CONFIG_RELAY_GPIO
  board_relay_io_initialize();
#endif

#ifdef HAVE_I2C_DRIVER
  /* Get the I2C lower half instance */

  i2c = stm32l4_i2cbus_initialize(1);
  if (i2c == NULL)
    {
      i2cerr("ERROR: Initialize I2C1: %d\n", ret);
    }
  else
    {
      /* Register the I2C character driver */

      ret = i2c_register(i2c, 1);
      if (ret < 0)
        {
          i2cerr("ERROR: Failed to register I2C1 device: %d\n", ret);
        }
    }


  #if defined(CONFIG_SENSORS_SHT21)
  sht21_register("/dev/xht21", i2c, 0x40);
  #endif
  
  #if defined(CONFIG_SENSORS_BH1750FVI)
  bh1750fvi_register("/dev/bh1750", i2c, 0x23);
  #endif

#endif

#ifdef CONFIG_DAC
  ainfo("Initializing DAC\n");

  stm32l4_dac_setup();
#endif

#ifdef CONFIG_ADC
  ainfo("Initializing ADC\n");

  stm32l4_adc_setup();
#endif

#ifdef CONFIG_STM32L4_IRTIM
  ainfo("Initializing IRTIM\n");

  stm32l4_ir_setup();
#endif

#ifdef CONFIG_IEEE802154_CC2520
  /* Configure CC2520 wireless */

  ret = stm32_cc2520_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_cc2520_initialize() failed:"
                      " %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
