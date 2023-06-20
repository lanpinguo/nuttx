/****************************************************************************
 * boards/arm/stm32/clicker2-stm32/src/stm32_mrf24j40.c
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

#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/cc2520.h>

#include "stm32_gpio.h"
#include "stm32_exti.h"
#include "stm32_spi.h"

#include "stm32.h"
#include "stm32f4discovery.h"


#ifdef CONFIG_IEEE802154_CC2520

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DRIVERS_WIRELESS
#  error Wireless support requires CONFIG_DRIVERS_WIRELESS
#endif


#ifdef CONFIG_CLICKER2_STM32_MB1_BEE
#  ifndef CONFIG_STM32_SPI3
#    error Mikroe BEE on mikroBUS1 requires CONFIG_STM32_SPI3
#  endif
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_BEE
#  ifndef CONFIG_STM32_SPI2
#    error Mikroe BEE on mikroBUS1 requires CONFIG_STM32_SPI2
#  endif
#endif


/* Poll timeout */

#define POLLTIMEOUT MSEC2TICK(100)


/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_priv_s
{
    struct cc2520_lower_s dev;
    xcpt_t handler;
    void *arg;
    uint32_t intcfg;
    uint32_t rstcfg;
    uint8_t spidev;

    /* Timing */
    struct wdog_s         poll_wd;     /* poll timeout timer */


    struct work_s irqwork;   /* Workqueue for FIFOP */
    struct work_s poll_work;   /* Workqueue for FIFOP */
    struct ieee802154_radio_s * radio_lst[16];

};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
 * to isolate the CC2520 driver from differences in GPIO interrupt handling
 * varying boards and MCUs.
 *
 *   irq_attach - Attach the CC2520 interrupt handler to the GPIO
 *                interrupt
 *   irq_enable - Enable or disable the GPIO interrupt
 */

static int  stm32_attach_irq(const struct cc2520_lower_s *lower,
                             xcpt_t handler, void *arg);
static void stm32_enable_irq(const struct cc2520_lower_s *lower,
                             bool state);
static int  stm32_cc2520_devsetup(struct stm32_priv_s *priv);

static int stm32_interrupt_handler(int irq, FAR void *context, FAR void *arg);

static void cc2520_poll_timeout(wdparm_t arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the CC2520
 * driver.  This structure provides information about the configuration
 * of the CC2520 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct stm32_priv_s g_cc2520_mb1_priv =
{
  .dev.attach  = stm32_attach_irq,
  .dev.enable  = stm32_enable_irq,
  .handler     = NULL,
  .arg         = NULL,
  .intcfg      = GPIO_WIRELESS_INT,
  .rstcfg      = GPIO_WIRELESS_RST,
  .spidev      = 1,

};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the CC2520 driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   irq_attach       - Attach the CC2520 interrupt handler to the GPIO
 *                      interrupt
 *   irq_enable       - Enable or disable the GPIO interrupt
 */

static int stm32_attach_irq(const struct cc2520_lower_s *lower,
                            xcpt_t handler, void *arg)
{
  struct stm32_priv_s *priv = (struct stm32_priv_s *)lower;

  DEBUGASSERT(priv != NULL);

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  priv->arg     = arg;
  return OK;
}

static void stm32_enable_irq(const struct cc2520_lower_s *lower,
                             bool state)
{
  struct stm32_priv_s *priv = (struct stm32_priv_s *)lower;

  /* The caller should not attempt to enable interrupts if the handler
   * has not yet been 'attached'
   */

  DEBUGASSERT(priv != NULL && (priv->handler != NULL || !state));

  wlinfo("state:%d\n", (int)state);

  /* Attach and enable, or detach and disable */

  if (state)
    {
      stm32_gpiosetevent(priv->intcfg, false, true, true,
                         priv->handler, priv->arg);
    }
  else
    {
      stm32_gpiosetevent(priv->intcfg, false, false, false,
                         NULL, NULL);
    }
}

void stm32_irqworker(FAR void *arg)
{
    FAR struct stm32_priv_s *priv = (FAR struct stm32_priv_s *)arg;

    DEBUGASSERT(priv);


    wlinfo("irq interrupt\n");

    /* Get exclusive access to the driver */
    for(int minor = 0; minor < 16; minor++){
        cc2520_irqwork_rx(priv->radio_lst[minor]);
    }


    /* Re-enable GPIO interrupts */
    priv->dev.enable(&priv->dev, true);
}

void stm32_poll_worker(FAR void *arg)
{
    FAR struct stm32_priv_s *priv = (FAR struct stm32_priv_s *)arg;

    DEBUGASSERT(priv);


    /* Get exclusive access to the driver */
    for(int minor = 0; minor < 16; minor++){
        cc2520_irqwork_rx(priv->radio_lst[minor]);
    }

    wd_start(&priv->poll_wd, POLLTIMEOUT, cc2520_poll_timeout, (wdparm_t)priv);


}


static int stm32_interrupt_handler(int irq, FAR void *context, FAR void *arg)
{
    FAR struct stm32_priv_s *priv = (FAR struct stm32_priv_s *)arg;

    DEBUGASSERT(priv != NULL);

    /* In complex environments, we cannot do SPI transfers from the interrupt
    * handler because semaphores are probably used to lock the SPI bus.  In
    * this case, we will defer processing to the worker thread.  This is also
    * much kinder in the use of system resources and is, therefore, probably
    * a good thing to do in any event.
    */

    DEBUGASSERT(work_available(&priv->irqwork));

    /* Notice that further GPIO interrupts are disabled until the work is
    * actually performed.  This is to prevent overrun of the worker thread.
    * Interrupts are re-enabled in enc_irqworker() when the work is completed.
    */

    priv->dev.enable(&priv->dev, false);
    return work_queue(HPWORK, &priv->irqwork,
                    stm32_irqworker, (FAR void *)arg, 0);
}

static void cc2520_poll_timeout(wdparm_t arg)
{
    int ret;

    FAR struct stm32_priv_s *priv = (FAR struct stm32_priv_s *)arg;

    DEBUGASSERT(priv != NULL);

    /* In complex environments, we cannot do SPI transfers from the interrupt
    * handler because semaphores are probably used to lock the SPI bus.  In
    * this case, we will defer processing to the worker thread.  This is also
    * much kinder in the use of system resources and is, therefore, probably
    * a good thing to do in any event.
    */

    DEBUGASSERT(work_available(&priv->poll_work));

    ret = work_queue(HPWORK, &priv->poll_work, stm32_poll_worker, (FAR void *)arg, 0);
    DEBUGASSERT(ret == OK);
    UNUSED(ret);
}


/****************************************************************************
 * Name: stm32_cc2520_devsetup
 *
 * Description:
 *   Initialize one the CC2520 device in one mikroBUS slot
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static int stm32_cc2520_devsetup(struct stm32_priv_s *priv)
{
    struct ieee802154_radio_s *radio;
    MACHANDLE mac;
    struct spi_dev_s *spi;
    int ret;

    /* Configure the interrupt pin */
#if 0
    stm32_configgpio(priv->intcfg);
#endif
    stm32_configgpio(priv->rstcfg);

    /* Tdres ≥ 0.1 ms */
    stm32_gpiowrite(priv->rstcfg, 0);
    up_udelay(200);
    stm32_gpiowrite(priv->rstcfg, 1);

    /* wait XOSC startup time*/
    up_udelay(300);

    /* Initialize the SPI bus and get an instance of the SPI interface */

    spi = stm32_spibus_initialize(priv->spidev);
    if (spi == NULL)
    {
        wlerr("ERROR: Failed to initialize SPI bus %d\n", priv->spidev);
        return -ENODEV;
    }

    /* Re-configure spi mode */
    SPI_SETMODE(spi, SPIDEV_MODE0);
    SPI_SETFREQUENCY(spi, 8000000);


#ifdef CONFIG_SPI_DRIVER
    /* Register the SPI1 character driver */
    ret = spi_register(spi, priv->spidev);
    if (ret < 0)
    {
        spierr("ERROR: Failed to register SPI1 device: %d\n", ret);
    }
#endif

    /* Initialize and register the SPI CC2520 device */
    for(int32_t minor = 0 ; minor < 16; minor++){

        radio = cc2520_init(spi, &priv->dev, minor);
        if (radio == NULL)
        {
            wlerr("ERROR: Failed to initialize cc2520 bus %ld\n", minor);
            continue;
        }

        /* restore for irq polling devices*/
        priv->radio_lst[minor] = radio;


        /* Create a 802.15.4 MAC device from a 802.15.4 compatible radio device. */
        mac = mac802154_create(radio);
        if (mac == NULL)
        {
            wlerr("ERROR: Failed to initialize IEEE802.15.4 MAC\n");
            return -ENODEV;
        }


#ifdef CONFIG_IEEE802154_NETDEV
        /* Use the IEEE802.15.4 MAC interface instance to create a 6LoWPAN
        * network interface by wrapping the MAC intrface instance in a
        * network device driver via mac802154dev_register().
        */

        ret = mac802154netdev_register(mac);
        if (ret < 0)
        {
            wlerr("ERROR: Failed to register the MAC network driver wpan%d: %d\n",
                0, ret);
            return ret;
        }
#endif

#ifdef CONFIG_IEEE802154_MACDEV
        /* If want to call these APIs from userspace, you have to wrap the MAC
        * interface in a character device via mac802154dev_register().
        */

        ret = mac802154dev_register(mac, minor);
        if (ret < 0)
        {
            wlerr(" Failed to register the MAC character driver /dev/ieee%d: %d\n",
                0, ret);
            return ret;
        }
#endif


    }


    wd_start(&priv->poll_wd, POLLTIMEOUT, cc2520_poll_timeout, (wdparm_t)priv);


#if 0
    /* Attach irq to first dev instance*/
    if (priv->dev.attach(&priv->dev, stm32_interrupt_handler, priv) != OK)
    {
        wlerr(" Failed to attach irq\n");
    }

    priv->dev.enable(&priv->dev, true);
#endif
    UNUSED(ret);
    return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_cc2520_initialize
 *
 * Description:
 *   Initialize the CC2520 device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_cc2520_initialize(void)
{
  int ret;

  wlinfo("Configuring BEE in spi-1\n");


  ret = stm32_cc2520_devsetup(&g_cc2520_mb1_priv);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to initialize BD in spi-1: %d\n", ret);
    }

  UNUSED(ret);
  return OK;
}
#endif /* CONFIG_IEEE802154_CC2520 */
