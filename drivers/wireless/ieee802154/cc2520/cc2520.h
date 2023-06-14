/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/mrf24j40.h
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

#ifndef __DRIVERS_WIRELESS_IEEE802154_CC2520_CC2520_H
#define __DRIVERS_WIRELESS_IEEE802154_CC2520_CC2520_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/spi/spi.h>

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/* Formula for calculating default macMaxFrameWaitTime is on pg. 130
 *
 * For PHYs other than CSS and UWB, the attribute phyMaxFrameDuration
 * is given by:
 *
 * phyMaxFrameDuration = phySHRDuration +
 *                       ceiling([aMaxPHYPacketSize + 1] x
 *                       phySymbolsPerOctet)
 *
 * where ceiling() is a function that returns the smallest integer value
 * greater than or equal to its argument value. [1] pg. 158
 */


/* Clock configuration macros */


/* Configuration ************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
#define	SPI_COMMAND_BUFFER	3
#define	HIGH			    1
#define	LOW			        0
#define	STATE_IDLE		    0
#define	RSSI_VALID		    0
#define	RSSI_OFFSET		    78


#define CC2520_DEFAULT_MAX_FRAME_WAITTIME   1824

#define CC2520_SYMBOL_DURATION_PS           16000000

struct cc2520_radio_stat_s {
    uint32_t flush_cnt;
    uint32_t rx_frm_cnt;
    uint32_t tx_frm_cnt;
};


/* Driver private information */
struct cc2520_radio_s {
    struct ieee802154_radio_s radio;            /* The public device instance */
    FAR struct ieee802154_radiocb_s *radiocb;   /* Registered callbacks */

    /* MAC Attributes */
    struct ieee802154_addr_s addr;

    uint8_t minor;               /* mapping to spi cs 0~15 */
    uint8_t chan;                /* 11 to 26 for the 2.4 GHz band */
    uint8_t devmode;             /* device mode: device, coord, pancoord */
    uint8_t paenabled;           /* enable usage of PA */
    uint8_t rxmode;              /* Reception mode: Main, no CRC, promiscuous */
    int32_t txpower;             /* TX power in mBm = dBm/100 */
    struct ieee802154_cca_s cca; /* Clear channel assessement method */
    uint32_t slpclkper;          /* Sleep clock period (nanoseconds) */

    uint32_t max_frame_waittime;


    bool txdelayed_busy : 1;
    bool csma_busy : 1;
    bool reschedule_csma : 1;

    bool rxenabled : 1;

    FAR const struct cc2520_lower_s *lower;
	FAR struct spi_dev_s *spi;		            /* SPI device structure */
	uint8_t *buf;			                    /* SPI TX/Rx data buffer */
	bool amplified;			                    /* Flag for CC2591 */
	int fifo_pin;			                    /* FIFO GPIO pin number */
	struct work_s irqwork;                      /* Workqueue for FIFOP */
	mutex_t lock;		                        /* Lock for is_tx*/
	bool promiscuous;                           /* Flag for promiscuous mode */

    struct cc2520_radio_stat_s stat;
};


/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int  cc2520_interrupt(int irq, FAR void *context, FAR void *arg);
void cc2520_irqworker(FAR void *arg);

#endif /* __DRIVERS_WIRELESS_IEEE802154_CC2520_CC2520_H */
