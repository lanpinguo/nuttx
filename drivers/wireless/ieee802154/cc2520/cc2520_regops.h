/****************************************************************************
 * drivers/wireless/ieee802154/cc2520/cc2520_regops.h
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

#ifndef __DRIVERS_WIRELESS_IEEE802154_CC2520_REGOPS_H
#define __DRIVERS_WIRELESS_IEEE802154_CC2520_REGOPS_H

int
cc2520_probe(FAR struct spi_dev_s *spi, int32_t minor);

int
cc2520_cmd_strobe(struct cc2520_radio_s *dev, uint8_t cmd);

int
cc2520_get_status(struct cc2520_radio_s *dev, uint8_t *status);

int
cc2520_read_register(struct cc2520_radio_s *dev, uint8_t reg, uint8_t *data);

int
cc2520_write_register(struct cc2520_radio_s *dev, uint8_t reg, uint8_t value);

int
cc2520_read_ram(struct cc2520_radio_s *dev, uint8_t reg, uint8_t len, uint8_t *data);

int
cc2520_write_ram(struct cc2520_radio_s *dev, uint16_t reg, uint8_t len, uint8_t *data);

int
cc2520_write_txfifo(struct cc2520_radio_s *dev, uint8_t pkt_len, uint8_t *data, uint8_t len);

int
cc2520_write_txfifo_fcs(struct cc2520_radio_s *dev, uint8_t pkt_len, uint8_t *data, uint8_t len, uint16_t fcs);

int
cc2520_read_rxfifo(struct cc2520_radio_s *dev, uint8_t *data, uint8_t len);

int
cc2520_set_channel(struct cc2520_radio_s *dev, uint8_t page, uint8_t channel);

int 
cc2520_hw_init(struct cc2520_radio_s *priv);

#endif /* __DRIVERS_WIRELESS_IEEE802154_CC2520_REGOPS_H */
