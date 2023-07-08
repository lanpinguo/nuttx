/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_ethernet.c
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


#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

#include "stm32_gpio.h"
#include "stm32_ethernet.h"

#include "stm32h747i-disco.h"

#if defined(CONFIG_STM32H7_ETHMAC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/* Debug ********************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_phy_boardinitialize(int intf)
{
  //   for(int addr = 0; addr < 31; addr++){
  //   ret = stm32_phyread(addr, 0x12, &phyval);
  //     if((phyval & 0x1f) == addr)
  //     {
  //       ninfo(" detected PHY: %d\n", addr);
  //       break;
  //     }
  // }

  return 0;
}

#endif /* CONFIG_STM32H7_ETHMAC */
