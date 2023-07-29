/****************************************************************************
 * boards/arm/stm32h7/stm32h747i-disco/src/stm32_bringup.c
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
#include <syslog.h>
#include <errno.h>

#include "stm32h747i-disco.h"

#include <nuttx/fs/fs.h>

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32_rtc.h"
#endif


#ifdef CONFIG_VIDEO_FB
#include <nuttx/video/fb.h>
#endif

#ifdef CONFIG_STM32H7_QUADSPI

#include <nuttx/mtd/mtd.h>
#include <nuttx/spi/qspi.h>

#include "stm32_qspi.h"

#ifdef CONFIG_FS_FAT
#include <sys/mount.h>
#include <nuttx/fs/fat.h>
#endif

extern FAR struct qspi_dev_s *stm32h7_qspi_initialize(int intf);
extern FAR struct mtd_dev_s *mt25qlxxx_initialize(FAR struct qspi_dev_s *qspi,
                                           bool unprotect);

#endif

#ifdef CONFIG_INPUT_FT6X06
int stm32_tsc_setup(int minor);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
          stm32_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: stm32_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2ctool(void)
{
#ifdef CONFIG_STM32H7_I2C1
  stm32_i2c_register(1);
#endif
#ifdef CONFIG_STM32H7_I2C2
  stm32_i2c_register(2);
#endif
#ifdef CONFIG_STM32H7_I2C3
  stm32_i2c_register(3);
#endif
#ifdef CONFIG_STM32H7_I2C4
  stm32_i2c_register(4);
#endif
}
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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y &&
 *   CONFIG_NSH_ARCHINIT:
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;
#ifdef HAVE_RTC_DRIVER
  struct rtc_lowerhalf_s *lower;
#endif

  UNUSED(ret);

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
  stm32_i2ctool();
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n", ret);
    }
#endif /* CONFIG_FS_PROCFS */

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32 lower-half RTC driver */

  lower = stm32_rtc_lowerhalf();
  if (!lower)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to instantiate the RTC lower-half driver\n");
      return -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, lower);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }
    }
#endif


#ifdef CONFIG_STM32H7_QUADSPI

  FAR struct qspi_dev_s *qspi;
  FAR struct mtd_dev_s *mtd;
  struct qspi_meminfo_s meminfo;

  qspi = stm32h7_qspi_initialize(0);
  if (!qspi){
    syslog(LOG_ERR, "ERROR: stm32h7_qspi_initialize failed\n");
  }

  mtd = mt25qlxxx_initialize(qspi, true);
  if (!mtd){
    syslog(LOG_ERR, "ERROR: mt25qlxxx_initialize failed\n");
  }

// #ifndef CONFIG_FS_NXFFS

//   ret = ftl_initialize(0, mtd);
//   if (ret < 0)
//     {
//       ferr("ERROR: Initialize the FTL layer\n");
//     }

// #endif

  /* memory-mapped fast read mode with 4-byte addresses and 10 dummy cycles (for read only) */
  meminfo.flags = QSPIMEM_READ | QSPIMEM_QUADIO | QSPIMEM_IQUAD;
  meminfo.addrlen = 4;
  meminfo.dummies = 10;
  meminfo.cmd = 0xec; /* MT25QLXXX_FAST_READ_QUADIO */
  meminfo.addr = 0;
  meminfo.buflen = 0;
  meminfo.buffer = NULL;

  stm32h7_qspi_enter_memorymapped(qspi, &meminfo, 4096);

//     stm32_mpu_uheap((uintptr_t)0x90000000, 0x4000000);
#endif


#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif



#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif /* CONFIG_INPUT_BUTTONS */

#ifdef CONFIG_INPUT_FT6X06
  /* Initialize the touchscreen */

  ret = stm32_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_tsc_setup failed: %d\n", ret);
    }
#endif


#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif /* CONFIG_ADC */

#if defined(CONFIG_FAT_DMAMEMORY)
  if (stm32_dma_alloc_init() < 0)
    {
      syslog(LOG_ERR, "DMA alloc FAILED");
    }
#endif

#ifdef HAVE_SDIO
  /* Initialize the SDIO block driver */

  ret = stm32_sdio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
    }
#endif

  return OK;
}
