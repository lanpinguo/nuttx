/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/cc2520_interrupt.c
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

#include <nuttx/mm/iob.h>

#include <nuttx/wireless/ieee802154/cc2520.h>

#include "cc2520.h"
#include "cc2520_reg.h"
#include "cc2520_regops.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void cc2520_single_irqwork_rx(FAR struct cc2520_radio_s *dev);
static void cc2520_irqwork_txnorm(FAR struct cc2520_radio_s *dev);
static void cc2520_irqwork_txgts(FAR struct cc2520_radio_s *dev,
              uint8_t gts_num);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cc2520_irqwork_txnorm
 *
 * Description:
 *   Manage completion of packet transmission.
 *
 ****************************************************************************/

static void cc2520_irqwork_txnorm(FAR struct cc2520_radio_s *dev)
{

}

/****************************************************************************
 * Name: cc2520_irqwork_gts
 *
 * Description:
 *   Manage completion of packet transmission.
 *
 ****************************************************************************/

static void cc2520_irqwork_txgts(FAR struct cc2520_radio_s *dev,
                                   uint8_t gts)
{
    uint8_t txstat;

    /* Disable tx int */

}

void cc2520_irqwork_rx(FAR struct ieee802154_radio_s *radio)
{
    int32_t ret = 0;
    FAR struct cc2520_radio_s *dev = (FAR struct cc2520_radio_s *)radio;
    FAR struct ieee802154_primitive_s *primitive;
    FAR struct ieee802154_data_ind_s *ind;
	uint8_t len = 0, lqi = 0, bytes = 0;
    uint8_t reg_val = 0;
    int8_t rxfifo_cnt;
    uint8_t rxfifo_1st;
    uint8_t excflag0;
    uint8_t excflag0_new;


    if(dev == NULL){
        return;
    }

	ret = cc2520_read_register(dev, CC2520_EXCFLAG0, &excflag0);
	if (ret)
		return;
	ret = cc2520_read_register(dev, CC2520_EXCFLAG1, &reg_val);
	if (ret)
		return;
    /* clear interrupt status */
	cc2520_write_register(dev, CC2520_EXCFLAG0, 0);
	cc2520_write_register(dev, CC2520_EXCFLAG1, 0);
	cc2520_write_register(dev, CC2520_EXCFLAG2, 0);


    do{
        uint8_t len = 0, lqi = 0;
        uint8_t reg_val;

        rxfifo_cnt = 0;
        cc2520_read_register(dev, CC2520_RXFIFOCNT, &rxfifo_cnt);
        if(rxfifo_cnt == 0){
            //wlwarn("FIFO-CNT: 0x%02x\n", rxfifo_cnt);
            goto done;
        }

        ret = cc2520_read_register(dev, CC2520_RXFIRST, &rxfifo_1st);
        if (ret)
            goto done;

        /*assume 1st byte is phy length */
        if(rxfifo_1st == 0 || rxfifo_1st > rxfifo_cnt){
            /* no more one complete frame data in fifo, return */
            goto done;
        }

        len = rxfifo_1st;

        /* read firt byte in fifo */
        cc2520_read_rxfifo(dev, &bytes, 1);
        if(bytes != len || len > 128){
            /* there is a malformed pkt */
            wlwarn("Frame len: 0x%02x\n", bytes);
            goto flush;
        }

        /* Allocate a data_ind to put the frame in */
        primitive = ieee802154_primitive_allocate();
        ind = (FAR struct ieee802154_data_ind_s *)primitive;
        if (ind == NULL)
        {
            wlerr("ERROR: Unable to allocate data_ind. Discarding frame\n");
            goto flush;
        }

        primitive->type = IEEE802154_PRIMITIVE_IND_DATA;

        /* Allocate an IOB to put the frame into */
        ind->frame = iob_tryalloc(false);
        if (ind->frame == NULL) {
            wlwarn("no more memory \n");
            ieee802154_primitive_free(primitive);
            goto flush;
        }


        if (cc2520_read_rxfifo(dev, ind->frame->io_data, len)) {
            wlerr("frame reception failed\n");
            iob_free(ind->frame);
            ieee802154_primitive_free(primitive);
            goto flush;
        }

        ind->frame->io_len = len;


        /* In promiscuous mode, we configure the radio to include the
        * CRC (AUTOCRC==0) and we pass on the packet unconditionally. If not
        * in promiscuous mode, we check the CRC here, but leave the
        * RSSI/LQI/CRC_OK bytes as they will get removed in the mac layer.
        */
        if (!dev->promiscuous) {
            bool crc_ok;

            /* Check if the CRC is valid. With AUTOCRC set, the most
            * significant bit of the last byte returned from the CC2520
            * is CRC_OK flag. See section 20.3.4 of the datasheet.
            */
            crc_ok = ind->frame->io_data[len - 1] & BIT(7);

            /* If we failed CRC drop the packet in the driver layer. */
            if (!crc_ok) {
                wlerr("CRC check failed\n");
                iob_free(ind->frame);
                ieee802154_primitive_free(primitive);
                goto flush;
            }

            /* To calculate LQI, the lower 7 bits of the last byte (the
            * correlation value provided by the radio) must be scaled to
            * the range 0-255. According to section 20.6, the correlation
            * value ranges from 50-110. Ideally this would be calibrated
            * per hardware design, but we use roughly the datasheet values
            * to get close enough while avoiding floating point.
            */
            lqi = ind->frame->io_data[len - 1] & 0x7f;
            if (lqi < 50)
                lqi = 50;
            else if (lqi > 113)
                lqi = 113;
            lqi = (lqi - 50) * 4;
        }

        ind->lqi = lqi;

        /* Callback the receiver in the next highest layer */
        wlinfo("recv: %d on chl %d\n", len, dev->chan);
        dev->radiocb->rxframe(dev->radiocb, ind);
    }while(rxfifo_cnt > 0);

done:
	ret = cc2520_read_register(dev, CC2520_EXCFLAG0, &excflag0_new);
    if(((excflag0 | excflag0_new) & ((1<<6) | (1<<5))) != 0){
        /* SFLUSHRX
            The workaround consists of flushing the RX FIFO twice instead of once.
            The second flush operation removes the unwanted byte from the RX FIFO
        */
flush:
        cc2520_cmd_strobe(dev, CC2520_CMD_SFLUSHRX);
        cc2520_cmd_strobe(dev, CC2520_CMD_SFLUSHRX);
        dev->stat.flush_cnt++;
    }

    return;
}



/****************************************************************************
 * Name: cc2520_irqwork_rx
 *
 * Description:
 *   Manage packet reception.
 *
 ****************************************************************************/

static void cc2520_single_irqwork_rx(FAR struct cc2520_radio_s *dev)
{
    FAR struct ieee802154_primitive_s *primitive;
    FAR struct ieee802154_data_ind_s *ind;
	uint8_t len = 0, lqi = 0, bytes = 1;

    wlinfo("RX interrupt\n");

	/* Read single length byte from the radio. */
	cc2520_read_rxfifo(dev, &len, bytes);


    /* Allocate a data_ind to put the frame in */
    primitive = ieee802154_primitive_allocate();
    ind = (FAR struct ieee802154_data_ind_s *)primitive;
    if (ind == NULL)
    {
        wlerr("ERROR: Unable to allocate data_ind. Discarding frame\n");
        goto done;
    }

    primitive->type = IEEE802154_PRIMITIVE_IND_DATA;

    /* Allocate an IOB to put the frame into */

    ind->frame = iob_alloc(false);
    DEBUGASSERT(ind->frame != NULL);

	if (cc2520_read_rxfifo(dev, ind->frame->io_data, len)) {
		wlerr("frame reception failed\n");
		iob_free(ind->frame);
		ieee802154_primitive_free(primitive);
	}

    ind->frame->io_len = len;


	/* In promiscuous mode, we configure the radio to include the
	 * CRC (AUTOCRC==0) and we pass on the packet unconditionally. If not
	 * in promiscuous mode, we check the CRC here, but leave the
	 * RSSI/LQI/CRC_OK bytes as they will get removed in the mac layer.
	 */
	if (!dev->promiscuous) {
		bool crc_ok;

		/* Check if the CRC is valid. With AUTOCRC set, the most
		 * significant bit of the last byte returned from the CC2520
		 * is CRC_OK flag. See section 20.3.4 of the datasheet.
		 */
		crc_ok = ind->frame->io_data[len - 1] & BIT(7);

		/* If we failed CRC drop the packet in the driver layer. */
		if (!crc_ok) {
			wlerr("CRC check failed\n");
            iob_free(ind->frame);
            ieee802154_primitive_free(primitive);
            goto done;
		}

		/* To calculate LQI, the lower 7 bits of the last byte (the
		 * correlation value provided by the radio) must be scaled to
		 * the range 0-255. According to section 20.6, the correlation
		 * value ranges from 50-110. Ideally this would be calibrated
		 * per hardware design, but we use roughly the datasheet values
		 * to get close enough while avoiding floating point.
		 */
		lqi = ind->frame->io_data[len - 1] & 0x7f;
		if (lqi < 50)
			lqi = 50;
		else if (lqi > 113)
			lqi = 113;
		lqi = (lqi - 50) * 4;
	}

    ind->lqi = lqi;
	wlinfo("RXFIFO: %x %x\n", len, lqi);

    /* Callback the receiver in the next highest layer */
    dev->radiocb->rxframe(dev->radiocb, ind);

done:
    /* Enable reception of next packet by flushing the fifo. */
	cc2520_cmd_strobe(dev, CC2520_CMD_SFLUSHRX);
	cc2520_cmd_strobe(dev, CC2520_CMD_SFLUSHRX);

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cc2520_irqworker
 *
 * Description:
 *   Perform interrupt handling logic outside of the interrupt handler (on
 *   the work queue thread).
 *
 * Input Parameters:
 *   arg     - The reference to the driver structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void cc2520_irqworker(FAR void *arg)
{
    FAR struct cc2520_radio_s *dev = (FAR struct cc2520_radio_s *)arg;
    uint8_t intstat;
    uint8_t reg;

    DEBUGASSERT(dev);
    DEBUGASSERT(dev->spi);

    wlinfo("irq interrupt\n");

    /* Get exclusive access to the driver */



    cc2520_single_irqwork_rx(dev);


    /* Re-enable GPIO interrupts */
    dev->lower->enable(dev->lower, true);
}

/****************************************************************************
 * Name: cc2520_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

int cc2520_interrupt(int irq, FAR void *context, FAR void *arg)
{
    FAR struct cc2520_radio_s *dev = (FAR struct cc2520_radio_s *)arg;

    DEBUGASSERT(dev != NULL);

    /* In complex environments, we cannot do SPI transfers from the interrupt
    * handler because semaphores are probably used to lock the SPI bus.  In
    * this case, we will defer processing to the worker thread.  This is also
    * much kinder in the use of system resources and is, therefore, probably
    * a good thing to do in any event.
    */

    DEBUGASSERT(work_available(&dev->irqwork));

    /* Notice that further GPIO interrupts are disabled until the work is
    * actually performed.  This is to prevent overrun of the worker thread.
    * Interrupts are re-enabled in enc_irqworker() when the work is completed.
    */

    dev->lower->enable(dev->lower, false);
    return work_queue(HPWORK, &dev->irqwork,
                    cc2520_irqworker, (FAR void *)dev, 0);
}
