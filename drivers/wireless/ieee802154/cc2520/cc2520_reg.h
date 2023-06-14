/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/mrf24j40_reg.h
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

#ifndef __DRIVERS_WIRELESS_IEEE802154_CC2520__REG_H
#define __DRIVERS_WIRELESS_IEEE802154_CC2520__REG_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BIT(n)          ((1UL) << (n))
#define BIT64(n)        ((1ULL) << (n))


/****************************************************************************
 * Public Types
 ****************************************************************************/
#define	CC2520_RAM_SIZE		640
#define	CC2520_FIFO_SIZE	128

#define	CC2520RAM_TXFIFO	0x100
#define	CC2520RAM_RXFIFO	0x180
#define	CC2520RAM_IEEEADDR	0x3EA
#define	CC2520RAM_PANID		0x3F2
#define	CC2520RAM_SHORTADDR	0x3F4

#define	CC2520_FREG_MASK	0x3F

/* status byte values */
#define	CC2520_STATUS_XOSC32M_STABLE	BIT(7)
#define	CC2520_STATUS_RSSI_VALID	BIT(6)
#define	CC2520_STATUS_TX_UNDERFLOW	BIT(3)

/* IEEE-802.15.4 defined constants (2.4 GHz logical channels) */
#define	CC2520_MINCHANNEL		11
#define	CC2520_MAXCHANNEL		26
#define	CC2520_CHANNEL_SPACING		5

/* command strobes */
#define	CC2520_CMD_SNOP			0x00
#define	CC2520_CMD_IBUFLD		0x02
#define	CC2520_CMD_SIBUFEX		0x03
#define	CC2520_CMD_SSAMPLECCA		0x04
#define	CC2520_CMD_SRES			0x0f
#define	CC2520_CMD_MEMORY_MASK		0x0f
#define	CC2520_CMD_MEMORY_READ		0x10
#define	CC2520_CMD_MEMORY_WRITE		0x20
#define	CC2520_CMD_RXBUF		0x30
#define	CC2520_CMD_RXBUFCP		0x38
#define	CC2520_CMD_RXBUFMOV		0x32
#define	CC2520_CMD_TXBUF		0x3A
#define	CC2520_CMD_TXBUFCP		0x3E
#define	CC2520_CMD_RANDOM		0x3C
#define	CC2520_CMD_SXOSCON		0x40
#define	CC2520_CMD_STXCAL		0x41
#define	CC2520_CMD_SRXON		0x42
#define	CC2520_CMD_STXON		0x43
#define	CC2520_CMD_STXONCCA		0x44
#define	CC2520_CMD_SRFOFF		0x45
#define	CC2520_CMD_SXOSCOFF		0x46
#define	CC2520_CMD_SFLUSHRX		0x47
#define	CC2520_CMD_SFLUSHTX		0x48
#define	CC2520_CMD_SACK			0x49
#define	CC2520_CMD_SACKPEND		0x4A
#define	CC2520_CMD_SNACK		0x4B
#define	CC2520_CMD_SRXMASKBITSET	0x4C
#define	CC2520_CMD_SRXMASKBITCLR	0x4D
#define	CC2520_CMD_RXMASKAND		0x4E
#define	CC2520_CMD_RXMASKOR		0x4F
#define	CC2520_CMD_MEMCP		0x50
#define	CC2520_CMD_MEMCPR		0x52
#define	CC2520_CMD_MEMXCP		0x54
#define	CC2520_CMD_MEMXWR		0x56
#define	CC2520_CMD_BCLR			0x58
#define	CC2520_CMD_BSET			0x59
#define	CC2520_CMD_CTR_UCTR		0x60
#define	CC2520_CMD_CBCMAC		0x64
#define	CC2520_CMD_UCBCMAC		0x66
#define	CC2520_CMD_CCM			0x68
#define	CC2520_CMD_UCCM			0x6A
#define	CC2520_CMD_ECB			0x70
#define	CC2520_CMD_ECBO			0x72
#define	CC2520_CMD_ECBX			0x74
#define	CC2520_CMD_INC			0x78
#define	CC2520_CMD_ABORT		0x7F
#define	CC2520_CMD_REGISTER_READ	0x80
#define	CC2520_CMD_REGISTER_WRITE	0xC0

/* status registers */
#define	CC2520_CHIPID			0x40
#define	CC2520_VERSION			0x42
#define	CC2520_EXTCLOCK			0x44
#define	CC2520_MDMCTRL0			0x46
#define	CC2520_MDMCTRL1			0x47
#define	CC2520_FREQEST			0x48
#define	CC2520_RXCTRL			0x4A
#define	CC2520_FSCTRL			0x4C
#define	CC2520_FSCAL0			0x4E
#define	CC2520_FSCAL1			0x4F
#define	CC2520_FSCAL2			0x50
#define	CC2520_FSCAL3			0x51
#define	CC2520_AGCCTRL0			0x52
#define	CC2520_AGCCTRL1			0x53
#define	CC2520_AGCCTRL2			0x54
#define	CC2520_AGCCTRL3			0x55
#define	CC2520_ADCTEST0			0x56
#define	CC2520_ADCTEST1			0x57
#define	CC2520_ADCTEST2			0x58
#define	CC2520_MDMTEST0			0x5A
#define	CC2520_MDMTEST1			0x5B
#define	CC2520_DACTEST0			0x5C
#define	CC2520_DACTEST1			0x5D
#define	CC2520_ATEST			0x5E
#define	CC2520_DACTEST2			0x5F
#define	CC2520_PTEST0			0x60
#define	CC2520_PTEST1			0x61
#define	CC2520_RESERVED			0x62
#define	CC2520_DPUBIST			0x7A
#define	CC2520_ACTBIST			0x7C
#define	CC2520_RAMBIST			0x7E

/* frame registers */
#define	CC2520_FRMFILT0			0x00
#define	CC2520_FRMFILT1			0x01
#define	CC2520_SRCMATCH			0x02
#define	CC2520_SRCSHORTEN0		0x04
#define	CC2520_SRCSHORTEN1		0x05
#define	CC2520_SRCSHORTEN2		0x06
#define	CC2520_SRCEXTEN0		0x08
#define	CC2520_SRCEXTEN1		0x09
#define	CC2520_SRCEXTEN2		0x0A
#define	CC2520_FRMCTRL0			0x0C
#define	CC2520_FRMCTRL1			0x0D
#define	CC2520_RXENABLE0		0x0E
#define	CC2520_RXENABLE1		0x0F
#define	CC2520_EXCFLAG0			0x10
#define	CC2520_EXCFLAG1			0x11
#define	CC2520_EXCFLAG2			0x12
#define	CC2520_EXCMASKA0		0x14
#define	CC2520_EXCMASKA1		0x15
#define	CC2520_EXCMASKA2		0x16
#define	CC2520_EXCMASKB0		0x18
#define	CC2520_EXCMASKB1		0x19
#define	CC2520_EXCMASKB2		0x1A
#define	CC2520_EXCBINDX0		0x1C
#define	CC2520_EXCBINDX1		0x1D
#define	CC2520_EXCBINDY0		0x1E
#define	CC2520_EXCBINDY1		0x1F
#define	CC2520_GPIOCTRL0		0x20
#define	CC2520_GPIOCTRL1		0x21
#define	CC2520_GPIOCTRL2		0x22
#define	CC2520_GPIOCTRL3		0x23
#define	CC2520_GPIOCTRL4		0x24
#define	CC2520_GPIOCTRL5		0x25
#define	CC2520_GPIOPOLARITY		0x26
#define	CC2520_GPIOCTRL			0x28
#define	CC2520_DPUCON			0x2A
#define	CC2520_DPUSTAT			0x2C
#define	CC2520_FREQCTRL			0x2E
#define	CC2520_FREQTUNE			0x2F
#define	CC2520_TXPOWER			0x30
#define	CC2520_TXCTRL			0x31
#define	CC2520_FSMSTAT0			0x32
#define	CC2520_FSMSTAT1			0x33
#define	CC2520_FIFOPCTRL		0x34
#define	CC2520_FSMCTRL			0x35
#define	CC2520_CCACTRL0			0x36
#define	CC2520_CCACTRL1			0x37
#define	CC2520_RSSI			    0x38
#define	CC2520_RSSISTAT			0x39
#define	CC2520_RXFIRST			0x3C
#define	CC2520_RXFIFOCNT		0x3E
#define	CC2520_TXFIFOCNT		0x3F

/* CC2520_FRMFILT0 */
#define FRMFILT0_FRAME_FILTER_EN	BIT(0)
#define FRMFILT0_PAN_COORDINATOR	BIT(1)

/* CC2520_FRMCTRL0 */
#define FRMCTRL0_AUTOACK		BIT(5)
#define FRMCTRL0_AUTOCRC		BIT(6)

/* CC2520_FRMCTRL1 */
#define FRMCTRL1_SET_RXENMASK_ON_TX	BIT(0)
#define FRMCTRL1_IGNORE_TX_UNDERF	BIT(1)

#define GPIO_OUT_EXC_RX_FRM_DONE    0x09
#define GPIO_OUT_EXC_FIFOP          0x0D
#define GPIO_OUT_EXC_CHL_A          0x21

#endif /* __DRIVERS_WIRELESS_IEEE802154_CC2520__REG_H */
