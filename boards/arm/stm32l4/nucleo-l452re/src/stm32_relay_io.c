/****************************************************************************
 * boards/arm/stm32l4/nucleo-l452re/src/stm32_userleds.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/power/pm.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32l4_gpio.h"
#include "nucleo-l452re.h"

#include <arch/board/board.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/power/relay.h>


#ifdef CONFIG_RELAY_GPIO

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
  static int stm32_relay_io_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           bool value);
  static int stm32_relay_io_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          FAR bool *value);




/****************************************************************************
 * Private Data
 ****************************************************************************/
const struct ioexpander_ops_s relay_ops = {
  .ioe_readpin = stm32_relay_io_readpin,
  .ioe_writepin = stm32_relay_io_writepin,
};

FAR struct ioexpander_dev_s iodev = {
  .ops = &relay_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int stm32_relay_io_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          bool value)
{
  switch (pin)
  {
  case 0:
    stm32l4_gpiowrite(GPIO_RELAY_10A_0, value);
    break;
  case 1:
    stm32l4_gpiowrite(GPIO_RELAY_16A_0, value);
    break;

  default:
    break;
  }

  return OK;

}


static int stm32_relay_io_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value)
{
  switch (pin)
  {
  case 0:
    *value = stm32l4_gpioread(GPIO_RELAY_10A_0);
    break;
  case 1:
    *value = stm32l4_gpioread(GPIO_RELAY_16A_0);
    break;

  default:
    break;
  }

  return OK;
}                        



/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_relay_io_initialize
 ****************************************************************************/

uint32_t board_relay_io_initialize(void)
{
  int ret;
  /* Configure relay GPIO for output */

  stm32l4_configgpio(GPIO_RELAY_10A_0);
  ret = relay_gpio_register(&iodev, 0,
                        false, "/dev/ac-10A");

  stm32l4_configgpio(GPIO_RELAY_16A_0);
  ret = relay_gpio_register(&iodev, 1,
                        false, "/dev/ac-16A");

  UNUSED(ret);
  return 2;
}



#endif /* CONFIG_RELAY_GPIO */
