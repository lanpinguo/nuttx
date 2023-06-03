/****************************************************************************
 * drivers/wireless/ieee802154/cc2520/cc2520_regops.c
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
#include <inttypes.h>
#include <stdio.h>

#include "cc2520.h"
#include "cc2520_reg.h"
#include "cc2520_regops.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int
cc2520_cmd_strobe(struct cc2520_radio_s *dev, uint8_t cmd)
{
	int ret = OK;
    int len = 0;
    uint8_t rx_buf[2] = {0};

    dev->spi->ops->lock(dev->spi, true);
	dev->buf[len++] = cmd;
	wlinfo("command strobe buf[0] = %02x\n",
		 dev->buf[0]);

    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), true);
    SPI_EXCHANGE(dev->spi, dev->buf, rx_buf, len);
    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), false);

	wlinfo("rx_buf[0] = %02x\n", rx_buf[0]);
    dev->spi->ops->lock(dev->spi, false);

	return ret;
}

int
cc2520_get_status(struct cc2520_radio_s *dev, uint8_t *status)
{
	int ret = OK;
    int len = 0;

    dev->spi->ops->lock(dev->spi, true);

	dev->buf[len++] = CC2520_CMD_SNOP;


    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), true);
    SPI_EXCHANGE(dev->spi, dev->buf, status, len);
    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), false);

	wlinfo("status = %02x\n", *status);

    dev->spi->ops->lock(dev->spi, false);

	return ret;
}

int
cc2520_write_register(struct cc2520_radio_s *dev, uint8_t reg, uint8_t value)
{
	int ret = OK;
    int len = 0;


    dev->spi->ops->lock(dev->spi, true);

	if (reg <= CC2520_FREG_MASK) {
		dev->buf[len++] = CC2520_CMD_REGISTER_WRITE | reg;
		dev->buf[len++] = value;
	} else {
		dev->buf[len++] = CC2520_CMD_MEMORY_WRITE;
		dev->buf[len++] = reg;
		dev->buf[len++] = value;
	}

    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), true);
    SPI_SNDBLOCK(dev->spi, dev->buf, len);
    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), false);

    dev->spi->ops->lock(dev->spi, false);

	return ret;
}

int
cc2520_write_ram(struct cc2520_radio_s *dev, uint16_t reg, uint8_t len, uint8_t *data)
{
	int ret = OK;
    int hdr_len = 0;


    dev->spi->ops->lock(dev->spi, true);
	dev->buf[hdr_len++] = (CC2520_CMD_MEMORY_WRITE |
						((reg >> 8) & 0xff));
	dev->buf[hdr_len++] = reg & 0xff;
    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), true);
    SPI_SNDBLOCK(dev->spi, dev->buf, hdr_len);

    SPI_SNDBLOCK(dev->spi, data, len);

    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), false);
    dev->spi->ops->lock(dev->spi, false);
	return ret;
}

int
cc2520_read_register(struct cc2520_radio_s *dev, uint8_t reg, uint8_t *data)
{
	int ret = OK;
    int hdr_len = 0;



    dev->spi->ops->lock(dev->spi, true);
    dev->buf[hdr_len++] = CC2520_CMD_REGISTER_READ | reg;
    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), true);

    SPI_SNDBLOCK(dev->spi, dev->buf, hdr_len);

    SPI_RECVBLOCK(dev->spi, data, 1);

    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), false);
    dev->spi->ops->lock(dev->spi, false);

	return ret;
}

int
cc2520_read_ram(struct cc2520_radio_s *dev, uint8_t reg, uint8_t len, uint8_t *data)
{
	int ret = OK;
    int hdr_len = 0;



    dev->spi->ops->lock(dev->spi, true);
	dev->buf[hdr_len++] = CC2520_CMD_MEMORY_READ;
	dev->buf[hdr_len++] = reg;
    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), true);

    SPI_SNDBLOCK(dev->spi, dev->buf, hdr_len);

    SPI_RECVBLOCK(dev->spi, data, len);

    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), false);
    dev->spi->ops->lock(dev->spi, false);

	return ret;
}


int
cc2520_write_txfifo(struct cc2520_radio_s *dev, uint8_t pkt_len, uint8_t *data, uint8_t len)
{
	int ret = OK;
    int hdr_len = 0;

	/* length byte must include FCS even
	 * if it is calculated in the hardware
	 */
	uint8_t len_byte = pkt_len;


    dev->spi->ops->lock(dev->spi, true);
	dev->buf[hdr_len++] = CC2520_CMD_TXBUF;
	wlinfo("TX_FIFO pkt_len = 0x%02x,  len= 0x%02x\n", pkt_len, len);

    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), true);
    SPI_SNDBLOCK(dev->spi, dev->buf, hdr_len);

    SPI_SNDBLOCK(dev->spi, &len_byte, 1);

    SPI_SNDBLOCK(dev->spi, data, len);

    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), false);
    dev->spi->ops->lock(dev->spi, false);

	return ret;
}

int
cc2520_write_txfifo_fcs(struct cc2520_radio_s *dev, uint8_t pkt_len, uint8_t *data, uint8_t len, uint16_t fcs)
{
	int ret = OK;
    int hdr_len = 0;

	/* length byte must include FCS even
	 * if it is calculated in the hardware
	 */
	uint8_t len_byte = pkt_len;


    dev->spi->ops->lock(dev->spi, true);
	dev->buf[hdr_len++] = CC2520_CMD_TXBUF;
	wlinfo("TX_FIFO pkt_len = 0x%02x,  len= 0x%02x\n", pkt_len, len);

    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), true);
    SPI_SNDBLOCK(dev->spi, dev->buf, hdr_len);

    SPI_SNDBLOCK(dev->spi, &len_byte, 1);

    SPI_SNDBLOCK(dev->spi, data, len);

    SPI_SNDBLOCK(dev->spi, &fcs, 2);

    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), false);
    dev->spi->ops->lock(dev->spi, false);

	return ret;
}

int
cc2520_read_rxfifo(struct cc2520_radio_s *dev, uint8_t *data, uint8_t len)
{
	int ret = OK;
    int hdr_len = 0;



    dev->spi->ops->lock(dev->spi, true);

	dev->buf[hdr_len++] = CC2520_CMD_RXBUF;

    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), true);
    SPI_SNDBLOCK(dev->spi, dev->buf, hdr_len);

    SPI_RECVBLOCK(dev->spi, data, len);
    SPI_SELECT(dev->spi, SPIDEV_IEEE802154(dev->minor), false);


	wlinfo("length buf[0] = %02x\n", data[0]);

    dev->spi->ops->lock(dev->spi, false);

	return ret;
}


int
cc2520_set_channel(struct cc2520_radio_s *dev, uint8_t page, uint8_t channel)
{
	int ret;

	wlinfo("trying to set channel\n");

	// WARN_ON(page != 0);
	// WARN_ON(channel < CC2520_MINCHANNEL);
	// WARN_ON(channel > CC2520_MAXCHANNEL);

	ret = cc2520_write_register(dev, CC2520_FREQCTRL,
				    11 + 5 * (channel - 11));

	return ret;
}

int
cc2520_set_promiscuous_mode(struct cc2520_radio_s *dev, bool on)
{
	uint8_t frmfilt0;

	wlinfo("mode %d\n", on);

	dev->promiscuous = on;

	cc2520_read_register(dev, CC2520_FRMFILT0, &frmfilt0);

	if (on) {
		/* Disable automatic ACK, automatic CRC, and frame filtering. */
		cc2520_write_register(dev, CC2520_FRMCTRL0, 0);
		frmfilt0 &= ~FRMFILT0_FRAME_FILTER_EN;
	} else {
		cc2520_write_register(dev, CC2520_FRMCTRL0, FRMCTRL0_AUTOACK |
							     FRMCTRL0_AUTOCRC);
		frmfilt0 |= FRMFILT0_FRAME_FILTER_EN;
	}
	return cc2520_write_register(dev, CC2520_FRMFILT0, frmfilt0);
}


int
cc2520_probe(FAR struct spi_dev_s *spi, int32_t minor)
{
	int ret = OK;
    int len = 0;
    uint8_t chipid = 0;
    uint8_t buf[4] = {0};



	buf[len++] = CC2520_CMD_MEMORY_READ;
	buf[len++] = CC2520_CHIPID;


    spi->ops->lock(spi, true);

    SPI_SELECT(spi, SPIDEV_IEEE802154(minor), true);

    SPI_SNDBLOCK(spi, buf, len);
    SPI_RECVBLOCK(spi, &chipid, 1);

    SPI_SELECT(spi, SPIDEV_IEEE802154(minor), false);

    spi->ops->lock(spi, false);

    if(chipid != 0x84){
        return ERROR;
    }

	return OK;
}


int cc2520_hw_init(struct cc2520_radio_s *priv)
{
	uint8_t status = 0, state = 0xff;
	int ret;
	int timeout = 100;



	do {
		ret = cc2520_get_status(priv, &status);
		if (ret)
			goto err_ret;

		if (timeout-- <= 0) {
			wlerr("oscillator start failed!\n");
			return ret;
		}
		up_udelay(1);
	} while (!(status & CC2520_STATUS_XOSC32M_STABLE));


	ret = cc2520_read_register(priv, CC2520_FSMSTAT1, &state);
	if (ret)
		goto err_ret;

	if (state != STATE_IDLE){
        wlerr("cc2520 state error!\n");
		return -EINVAL;
    }

	wlinfo("oscillator brought up\n");

	/* If the CC2520 is connected to a CC2591 amplifier, we must both
	 * configure GPIOs on the CC2520 to correctly configure the CC2591
	 * and change a couple settings of the CC2520 to work with the
	 * amplifier. See section 8 page 17 of TI application note AN065.
	 * http://www.ti.com/lit/an/swra229a/swra229a.pdf
	 */
	if (priv->amplified) {
		ret = cc2520_write_register(priv, CC2520_AGCCTRL1, 0x16);
		if (ret)
			goto err_ret;

		ret = cc2520_write_register(priv, CC2520_GPIOCTRL0, 0x46);
		if (ret)
			goto err_ret;

		ret = cc2520_write_register(priv, CC2520_GPIOCTRL5, 0x47);
		if (ret)
			goto err_ret;

		ret = cc2520_write_register(priv, CC2520_GPIOPOLARITY, 0x1e);
		if (ret)
			goto err_ret;

		ret = cc2520_write_register(priv, CC2520_TXCTRL, 0xc1);
		if (ret)
			goto err_ret;
	} else {
		ret = cc2520_write_register(priv, CC2520_AGCCTRL1, 0x11);
		if (ret)
			goto err_ret;
	}

	/* Registers default value: section 28.1 in Datasheet */

	/* Set the CCA threshold to -50 dBm. This seems to have been copied
	 * from the TinyOS CC2520 driver and is much higher than the -84 dBm
	 * threshold suggested in the datasheet.
	 */
	ret = cc2520_write_register(priv, CC2520_CCACTRL0, 0x1A);
	if (ret)
		goto err_ret;

	ret = cc2520_write_register(priv, CC2520_MDMCTRL0, 0x85);
	if (ret)
		goto err_ret;

	ret = cc2520_write_register(priv, CC2520_MDMCTRL1, 0x14);
	if (ret)
		goto err_ret;

	ret = cc2520_write_register(priv, CC2520_RXCTRL, 0x3f);
	if (ret)
		goto err_ret;

	ret = cc2520_write_register(priv, CC2520_FSCTRL, 0x5a);
	if (ret)
		goto err_ret;

	ret = cc2520_write_register(priv, CC2520_FSCAL1, 0x2b);
	if (ret)
		goto err_ret;

	ret = cc2520_write_register(priv, CC2520_ADCTEST0, 0x10);
	if (ret)
		goto err_ret;

	ret = cc2520_write_register(priv, CC2520_ADCTEST1, 0x0e);
	if (ret)
		goto err_ret;

	ret = cc2520_write_register(priv, CC2520_ADCTEST2, 0x03);
	if (ret)
		goto err_ret;

	/* Configure registers correctly for this driver. */
	ret = cc2520_write_register(priv, CC2520_FRMCTRL1,
				    FRMCTRL1_SET_RXENMASK_ON_TX |
				    FRMCTRL1_IGNORE_TX_UNDERF);
	if (ret)
		goto err_ret;

	ret = cc2520_write_register(priv, CC2520_FIFOPCTRL, 127);
	if (ret)
		goto err_ret;

	/* Configure cc2520 tx power. */
    /* Table 3: Suggested TXPOWER register settings for different temperatures */
	ret = cc2520_write_register(priv, CC2520_TXPOWER, 0xF7);
	if (ret)
		goto err_ret;


	/* Configure cc2520 GPIO. */
    /* Negative polarity. Level indication is active low. When input, falling edge is active.*/
	ret = cc2520_write_register(priv, CC2520_GPIOPOLARITY, 0x00);
	if (ret)
		goto err_ret;

    /* GPIO2 as output RX_FRM_DONE exception */
	ret = cc2520_write_register(priv, CC2520_GPIOCTRL2, GPIO_OUT_EXC_RX_FRM_DONE);
	if (ret)
		goto err_ret;

    /* GPIO3 as input with tied to ground*/
	ret = cc2520_write_register(priv, CC2520_GPIOCTRL3, (1<<7) | 0x10);
	if (ret)
		goto err_ret;

    /* GPIO4 as input with tied to ground*/
	ret = cc2520_write_register(priv, CC2520_GPIOCTRL4, (1<<7) | 0x10);
	if (ret)
		goto err_ret;


	return 0;

err_ret:
    wlerr("cc2520 hw init failed!\n");
	return ret;
}


