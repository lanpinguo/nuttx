// SPDX-License-Identifier: GPL-2.0-or-later
/* Driver for TI CC2520 802.15.4 Wireless-PAN Networking controller
 *
 * Copyright (C) 2014 Varka Bhadram <varkab@cdac.in>
 *		      Md.Jamal Mohiuddin <mjmohiuddin@cdac.in>
 *		      P Sowjanya <sowjanyap@cdac.in>
 */
#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <sys/types.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/mm/iob.h>

#include <nuttx/wireless/ieee802154/cc2520.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>


#include "cc2520.h"
#include "cc2520_radif.h"
#include "cc2520_cdev.h"


FAR struct ieee802154_radio_s *
cc2520_init(FAR struct spi_dev_s *spi, FAR const struct cc2520_lower_s *lower, int32_t minor)
{
    FAR struct cc2520_radio_s *dev;
    char devname[18];


    if(cc2520_probe(spi, minor) != OK){
        wlerr("CC2520 minor %d not exist\n", minor);
        return NULL;
    }

    dev = kmm_zalloc(sizeof(struct cc2520_radio_s));
    if (dev == NULL)
    {
        return NULL;
    }

    dev->buf = kmm_zalloc(SPI_COMMAND_BUFFER);
    if (dev->buf == NULL)
    {
        kmm_free(dev);
        return NULL;
    }


    /* Allow exclusive access to the privmac struct */

    nxmutex_init(&dev->lock);

    dev->radio.bind         = cc2520_bind;
    dev->radio.reset        = cc2520_reset;
    dev->radio.getattr      = cc2520_getattr;
    dev->radio.setattr      = cc2520_setattr;
    dev->radio.txnotify     = cc2520_txnotify;
    dev->radio.txdelayed    = cc2520_txdelayed;
    dev->radio.rxenable     = cc2520_rxenable;
    dev->radio.energydetect = cc2520_energydetect;
    dev->radio.beaconstart  = cc2520_beaconstart;
    dev->radio.beaconupdate = cc2520_beaconupdate;
    dev->radio.beaconstop   = cc2520_beaconstop;
    dev->radio.sfupdate     = cc2520_sfupdate;

    dev->lower              = lower;
    dev->spi                = spi;
    dev->minor              = minor;





    snprintf(devname, sizeof(devname), "/dev/snif%u", (unsigned int)minor);

    if (cc2520_cdev_driver_register(devname, dev) != OK)
    {
        wlerr("Failed to register character device\n");
    }



    return &dev->radio;
}
