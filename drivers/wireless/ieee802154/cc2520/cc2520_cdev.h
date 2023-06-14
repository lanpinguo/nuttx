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

#ifndef __DRIVERS_WIRELESS_IEEE802154_CC2520_CDEV__RADIF_H
#define __DRIVERS_WIRELESS_IEEE802154_CC2520_CDEV__RADIF_H

int 
cc2520_cdev_register(FAR const char *devpath, struct cc2520_radio_s *dev);

#endif /* __DRIVERS_WIRELESS_IEEE802154_CC2520_CDEV__RADIF_H */
