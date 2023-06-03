/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/cc2520_radif.h
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

#ifndef __DRIVERS_WIRELESS_IEEE802154_CC2520__RADIF_H
#define __DRIVERS_WIRELESS_IEEE802154_CC2520__RADIF_H

int cc2520_bind(FAR struct ieee802154_radio_s *radio,
                  FAR struct ieee802154_radiocb_s *radiocb);

int cc2520_reset(FAR struct ieee802154_radio_s *radio);

int cc2520_getattr(FAR struct ieee802154_radio_s *radio,
                     enum ieee802154_attr_e attr,
                     FAR union ieee802154_attr_u *attrval);

int cc2520_setattr(FAR struct ieee802154_radio_s *radio,
                     enum ieee802154_attr_e attr,
                     FAR const union ieee802154_attr_u *attrval);

int cc2520_txnotify(FAR struct ieee802154_radio_s *radio, bool gts);

int cc2520_txdelayed(FAR struct ieee802154_radio_s *radio,
                      FAR struct ieee802154_txdesc_s *txdesc,
                      uint32_t symboldelay);

int cc2520_rxenable(FAR struct ieee802154_radio_s *radio, bool enable);

int cc2520_energydetect(FAR struct ieee802154_radio_s *radio,
                          uint32_t nsymbols);

int cc2520_beaconstart(FAR struct ieee802154_radio_s *radio,
                     FAR const struct ieee802154_superframespec_s *sfspec,
                     FAR struct ieee802154_beaconframe_s *beacon);

int cc2520_beaconupdate(FAR struct ieee802154_radio_s *radio,
                          FAR struct ieee802154_beaconframe_s *beacon);

int cc2520_beaconstop(FAR struct ieee802154_radio_s *radio);

int cc2520_sfupdate(FAR struct ieee802154_radio_s *radio,
                      FAR const struct ieee802154_superframespec_s *sfspec);

#endif /* __DRIVERS_WIRELESS_IEEE802154_CC2520__RADIF_H */
