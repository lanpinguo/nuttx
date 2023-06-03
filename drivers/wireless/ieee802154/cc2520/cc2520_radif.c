/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/cc2520_radif.c
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

#include <assert.h>
#include <debug.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "cc2520.h"
#include "cc2520_reg.h"
#include "cc2520_regops.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/


/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cc2520_bind(FAR struct ieee802154_radio_s *radio,
                  FAR struct ieee802154_radiocb_s *radiocb)
{
    FAR struct cc2520_radio_s *dev = (FAR struct cc2520_radio_s *)radio;

    DEBUGASSERT(dev != NULL);
    dev->radiocb = radiocb;

    return OK;
}

/****************************************************************************
 * Function: cc2520_txnotify
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   radio  - Reference to the radio driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *
 ****************************************************************************/

int cc2520_txnotify(FAR struct ieee802154_radio_s *radio, bool gts)
{
  return OK;
}

/****************************************************************************
 * Function: cc2520_txdelayed
 *
 * Description:
 *   Transmit a packet without regard to supeframe structure after a certain
 *   number of symbols.  This function is used to send Data Request
 *   responses.  It can also be used to send data immediately if the delay
 *   is set to 0.
 *
 * Input Parameters:
 *   radio  - Reference to the radio driver state structure
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *
 ****************************************************************************/

int cc2520_txdelayed(FAR struct ieee802154_radio_s *radio,
                              FAR struct ieee802154_txdesc_s *txdesc,
                              uint32_t symboldelay)
{
  return OK;
}

/****************************************************************************
 * Name: cc2520_rxenable
 *
 * Description:
 *  Enable/Disable receiver.
 *
 ****************************************************************************/

int cc2520_rxenable(FAR struct ieee802154_radio_s *radio, bool enable)
{
    FAR struct cc2520_radio_s *dev = (FAR struct cc2520_radio_s *)radio;
    uint8_t reg;

    dev->rxenabled = enable;
    if (enable)
    {
        /* Disable packet reception. */


        /* Enable rx int */


        /* Purge the RX buffer */


        /* Re-enable packet reception. */


    }
    else
    {
        /* Disable rx int */


    }

    return OK;
}

int cc2520_energydetect(FAR struct ieee802154_radio_s *radio,
                          uint32_t nsymbols)
{
    return -ENOTTY;
}



int cc2520_reset(FAR struct ieee802154_radio_s *radio)
{
	int ret;


    ret = cc2520_hw_init((struct cc2520_radio_s *)radio);
    
    ret |= cc2520_set_promiscuous_mode((struct cc2520_radio_s *)radio, true);

    return ret;
}

int cc2520_getattr(FAR struct ieee802154_radio_s *radio,
                     enum ieee802154_attr_e attr,
                     FAR union ieee802154_attr_u *attrval)
{
    FAR struct cc2520_radio_s *dev = (FAR struct cc2520_radio_s *)radio;
    int ret;

    switch (attr)
    {
        case IEEE802154_ATTR_MAC_EADDR:
        {
            memcpy(&attrval->mac.eaddr[0], &dev->addr.eaddr[0], 8);
            ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

        case IEEE802154_ATTR_MAC_MAX_FRAME_WAITTIME:
        {
            attrval->mac.max_frame_waittime = dev->max_frame_waittime;
            ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

        case IEEE802154_ATTR_PHY_SYMBOL_DURATION:
        {
            attrval->phy.symdur_picosec = CC2520_SYMBOL_DURATION_PS;
            ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

        case IEEE802154_ATTR_PHY_CHAN:
        {
            attrval->phy.chan = dev->chan;
            ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

        case IEEE802154_ATTR_PHY_FCS_LEN:
        {
            attrval->phy.fcslen = 2;
            ret = IEEE802154_STATUS_SUCCESS;
        }
        break;

        default:
        ret = IEEE802154_STATUS_UNSUPPORTED_ATTRIBUTE;
    }

    return ret;

}

int cc2520_setattr(FAR struct ieee802154_radio_s *radio,
                            enum ieee802154_attr_e attr,
                            FAR const union ieee802154_attr_u *attrval)
{
    FAR struct cc2520_radio_s *dev = (FAR struct cc2520_radio_s *)radio;
    int ret = IEEE802154_STATUS_SUCCESS;

    switch (attr)
    {
        case IEEE802154_ATTR_MAC_PANID:
        {
            // mrf24j40_setpanid(dev, attrval->mac.panid);
        }
        break;

        case IEEE802154_ATTR_MAC_SADDR:
        {
            // mrf24j40_setsaddr(dev, attrval->mac.saddr);
        }
        break;

        case IEEE802154_ATTR_MAC_EADDR:
        {
            // mrf24j40_seteaddr(dev, attrval->mac.eaddr);
        }
        break;

        case IEEE802154_ATTR_MAC_COORD_SADDR:
        {
            // mrf24j40_setcoordsaddr(dev, attrval->mac.coordsaddr);
        }
        break;

        case IEEE802154_ATTR_MAC_COORD_EADDR:
        {
            // mrf24j40_setcoordeaddr(dev, attrval->mac.coordeaddr);
        }
        break;

        case IEEE802154_ATTR_MAC_PROMISCUOUS_MODE:
        {
            cc2520_set_promiscuous_mode(dev, attrval->mac.promisc_mode);
        }
        break;

        case IEEE802154_ATTR_PHY_CHAN:
        {
            cc2520_set_channel(dev, 0, attrval->phy.chan);
        }
        break;

        case IEEE802154_ATTR_MAC_DEVMODE:
        {
            // mrf24j40_setdevmode(dev, attrval->mac.devmode);
        }
        break;

        default:
        ret = IEEE802154_STATUS_UNSUPPORTED_ATTRIBUTE;
        break;
    }

    return ret;
}

int cc2520_beaconstart(FAR struct ieee802154_radio_s *radio,
               FAR const struct ieee802154_superframespec_s *sfspec,
               FAR struct ieee802154_beaconframe_s *beacon)
{
  return OK;
}

int cc2520_beaconupdate(FAR struct ieee802154_radio_s *radio,
               FAR struct ieee802154_beaconframe_s *beacon)
{
  FAR struct cc2520_radio_s *dev = (FAR struct cc2520_radio_s *)radio;


  return OK;
}

int cc2520_beaconstop(FAR struct ieee802154_radio_s *radio)
{
    return -ENOTTY;
}

int cc2520_sfupdate(FAR struct ieee802154_radio_s *radio,
             FAR const struct ieee802154_superframespec_s *sfspec)
{

    return OK;
}
