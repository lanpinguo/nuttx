/****************************************************************************
 * drivers/mtd/mt25qlxxx.c
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

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <inttypes.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* QuadSPI Mode.  Per data sheet, either Mode 0 or Mode 3 may be used. */

#ifndef CONFIG_MT25QLXXX_QSPIMODE
#define CONFIG_MT25QLXXX_QSPIMODE QSPIDEV_MODE0
#endif

/* QuadSPI Frequency per data sheet:
 *
 * In this implementation, only "Quad" reads are performed.
 */

#ifndef CONFIG_MT25QLXXX_QSPI_FREQUENCY
/* If you haven't specified frequency, default to 100 MHz which will work
 * with all commands. up to 133MHz.
 */

#define CONFIG_MT25QLXXX_QSPI_FREQUENCY 100000000
#endif

#ifndef CONFIG_MT25QLXXX_DUMMIES
/* If you haven't specified the number of dummy cycles for quad reads,
 * provide a reasonable default.  The actual number of dummies needed is
 * clock and IO command dependent.(four to six times according to data sheet)
 */

#define CONFIG_MT25QLXXX_DUMMIES 6
#endif

/* MT25QLXXX Commands *******************************************************/

/* Configuration, Status, Erase, Program Commands ***************************
 *      Command                  Value    Description:                      *
 *                                          Data sequence                   *
 */

                                       /* Read status registers-1/2/3: */
#define MT25QLXXX_READ_STATUS   	  0x05 /* SRP|SEC|TB |BP2|BP1|BP0|WEL|BUSY       */
#define MT25QLXXX_READ_FLAG_STATUS	0x70 /*       */

/* Write status registers-1/2/3:          */
#define MT25QLXXX_WRITE_STATUS  	  0x01 /* SRP|SEC|TB |BP2|BP1|BP0|WEL|BUSY       */
#define MT25QLXXX_WRITE_ENABLE    	0x06 /* Write enable                           */
#define MT25QLXXX_WRITE_DISABLE   	0x04 /* Write disable                          */
#define MT25QLXXX_PAGE_PROGRAM    	0x02 /* Page Program:                          *
											                    * 0x02 | A23-A16 | A15-A8 | A7-A0 | data */
#define MT25QLXXX_SECTOR_ERASE    	0xD8 /* Sector Erase ()                    *
										                      *  0x20 | A23-A16 | A15-A8 | A7-A0     */
#define MT25QLXXX_BLOCK_ERASE_32K 	0x52 /* Block Erase (32 KB)                    *
											                    *  0x52 | A23-A16 | A15-A8 | A7-A0     */
#define MT25QLXXX_BLOCK_ERASE_4K 	  0x20 /* Block Erase (4 KB)                    *
											                    *  0xd8 | A23-A16 | A15-A8 | A7-A0       */
#define MT25QLXXX_CHIP_ERASE      	0x60 /* Chip Erase:                            *
											                    *  0xc7 or 0x60                          */
#define MT25QLXXX_ENTER_4BT_MODE  	0xB7 /* Enter 4-byte address mode              */
#define MT25QLXXX_EXIT_4BT_MODE   	0xE9 /* Exit 4-byte address mode               */
	
#define MT25QLXXX_ENTER_QPI_MODE  	0x35 /* Enter QUAD PROTOCOL Operations         */
#define MT25QLXXX_EXIT_QPI_MODE   	0xF5 /* Exit QUAD PROTOCOL Operations          */
	
#define MT25QLXXX_WR_CFG_REG      	0x81 /* Write Volatile Conf. Reg.              */
#define MT25QLXXX_ENHANCED_CFG_REG  0x61 /* Write Enhanced Conf. Reg             */

/* Read Commands ************************************************************
 *      Command                  Value    Description:                      *
 *                                          Data sequence                   *
 */

#define MT25QLXXX_FAST_READ_QUADIO 0xec  /* Fast Read Quad I/O:             *
                                          *   0xeb | ADDR | data...         */

/* Reset Commands ***********************************************************
 *      Command                  Value    Description:                      *
 *                                          Data sequence                   *
 */

#define MT25QLXXX_RESET_ENABLE    	0x66  /* Enable Reset                     */
#define MT25QLXXX_DEVICE_RESET    	0x99  /* Reset Device                     */

/* ID/Security Commands *****************************************************
 *      Command                  Value    Description:                      *
 *                                            Data sequence                 *
 */
#define MT25QLXXX_JEDEC_ID        	0x9F  /* JEDEC ID:                        *
                                         * 0x9f | Manufacturer | MemoryType | *
                                         * Capacity                         */

#define MT25QLXXX_MULT_IO_JEDEC_ID  0xAF  /* JEDEC ID:                        *
                                         * MULTIPLE I/O READ ID     */


/* MT25QLXXX JEDIC IDs */

#define MT25QLXXXQ_JEDEC_DEVICE_3V  0xba  
#define MT25QLXXXQ_JEDEC_DEVICE_1V8 0xbb  


#define MT25Q016_JEDEC_CAPACITY    0x15  /* MT25Q016 (2 MB) memory capacity */
#define MT25Q032_JEDEC_CAPACITY    0x16  /* MT25Q032 (4 MB) memory capacity */
#define MT25Q064_JEDEC_CAPACITY    0x17  /* MT25Q064 (8 MB) memory capacity */
#define MT25Q128_JEDEC_CAPACITY    0x18  /* MT25Q128 (16 MB) memory capacity */
#define MT25Q256_JEDEC_CAPACITY    0x19  /* MT25Q256 (32 MB) memory capacity */
#define MT25Q512_JEDEC_CAPACITY    0x20  /* MT25Q512 (64 MB) memory capacity */
#define MT25Q01_JEDEC_CAPACITY     0x21  /* MT25Q01 (128 MB) memory capacity */

/* MT25QLXXX Registers ******************************************************/

/* Status register 1 bit definitions                                      */

#define STATUS_BUSY_MASK     (1 << 0) /* Bit 0: Device ready/busy status  */
#define STATUS_READY         (0 << 0) /*   0 = Not Busy                   */
#define STATUS_BUSY          (1 << 0) /*   1 = Busy                       */
#define STATUS_WEL_MASK      (1 << 1) /* Bit 1: Write enable latch status */
#define STATUS_WEL_DISABLED  (0 << 1) /*   0 = Not Write Enabled          */
#define STATUS_WEL_ENABLED   (1 << 1) /*   1 = Write Enabled              */
#define STATUS_BP_SHIFT      (2)      /* Bits 2-4: Block protect bits     */
#define STATUS_BP_MASK       (7 << STATUS_BP_SHIFT)
#define STATUS_BP_NONE       (0 << STATUS_BP_SHIFT)
#define STATUS_BP_ALL        (15 << STATUS_BP_SHIFT)
#define STATUS_TB_MASK       (1 << 5) /* Bit 5: Top / Bottom Protect      */
#define STATUS_TB_TOP        (0 << 5) /*   0 = BP2-BP0 protect Top down   */
#define STATUS_TB_BOTTOM     (1 << 5) /*   1 = BP2-BP0 protect Bottom up  */
#define STATUS_SEC_MASK      (1 << 6) /* Bit 6: SEC                       */
#define STATUS_SEC_64KB      (0 << 6) /*   0 = Protect 64KB Blocks        */
#define STATUS_SEC_4KB       (1 << 6) /*   1 = Protect 4KB Sectors        */
#define STATUS_SRP_MASK      (1 << 7) /* Bit 7: Status register protect 0 */
#define STATUS_SRP_UNLOCKED  (0 << 7) /*   see blow for details           */
#define STATUS_SRP_LOCKED    (1 << 7) /*   see blow for details           */

/* Status register 2 bit definitions                                      */

#define STATUS2_QE_MASK      (1 << 1) /* Bit 1: Quad Enable (QE)          */
#define STATUS2_QE_DISABLED  (0 << 1) /*  0 = Standard/Dual SPI modes     */
#define STATUS2_QE_ENABLED   (1 << 1) /*  1 = Standard/Dual/Quad modes    */

/* Some chips have four protect bits                                      */

/* Bits 2-5: Block protect bits                                           */
#define STATUS_BP_4_MASK     (15 << STATUS_BP_SHIFT)
/* Some chips have top/bottom bit at sixth bit                            */
#define STATUS_TB_6_MASK     (1 << 6) /* Bit 6: Top / Bottom Protect      */

/* Status Register Protect (SRP, SRL)
 * SRL SRP /WP Status Register         Description
 *  0   0   X  Software Protection     [Factory Default] /WP pin has no
 *                                     control.
 *                                     The Status register can be written
 *                                     to after a Write Enable instruction,
 *                                     WEL=1.
 *  0   1   0  Hardware Protected      When /WP pin is low the Status
 *                                     Register locked and cannot be
 *                                     written to.
 *  0   1   1  Hardware Unprotected    When /WP pin is high the Status
 *                                     register is unlocked and can be
 *                                     written to after a Write Enable
 *                                     instruction, WEL=1.
 *  1   X   X  Power Supply Lock-Down  Status Register is protected and
 *                                     cannot be written to again until
 *                                     the next power-down, power-up cycle.
 *                                     (When SRL =1 , a power-down, power-
 *                                     up cycle will change SRL =0 state.)
 *  1   X   X  One Time Program        Status Register is permanently
 *                                     protected and cannot be written to.
 *                                     (enabled by adding prefix command
 *                                     aah, 55h)
 */

/* Chip Geometries **********************************************************/

/* All members of the family support uniform 4K-byte 'sub sectors'; they also
 * support 64k (and sometimes 32k) 'sectors' proper, but we won't be using
 * those here.
 */

/* MT25Q016 (2 MB) memory capacity */

#define MT25Q016_SECTOR_SIZE         (4 * 1024)
#define MT25Q016_SECTOR_SHIFT        (12)
#define MT25Q016_SECTOR_COUNT        (512)
#define MT25Q016_PAGE_SIZE           (256)
#define MT25Q016_PAGE_SHIFT          (8)

/* MT25Q032 (4 MB) memory capacity */

#define MT25Q032_SECTOR_SIZE         (4 * 1024)
#define MT25Q032_SECTOR_SHIFT        (12)
#define MT25Q032_SECTOR_COUNT        (1024)
#define MT25Q032_PAGE_SIZE           (256)
#define MT25Q032_PAGE_SHIFT          (8)

/* MT25Q064 (8 MB) memory capacity */

#define MT25Q064_SECTOR_SIZE         (4 * 1024)
#define MT25Q064_SECTOR_SHIFT        (12)
#define MT25Q064_SECTOR_COUNT        (2048)
#define MT25Q064_PAGE_SIZE           (256)
#define MT25Q064_PAGE_SHIFT          (8)

/* MT25Q128 (16 MB) memory capacity */

#define MT25Q128_SECTOR_SIZE         (4 * 1024)
#define MT25Q128_SECTOR_SHIFT        (12)
#define MT25Q128_SECTOR_COUNT        (4096)
#define MT25Q128_PAGE_SIZE           (256)
#define MT25Q128_PAGE_SHIFT          (8)

/* MT25Q256 (32 MB) memory capacity */

#define MT25Q256_SECTOR_SIZE         (4 * 1024)
#define MT25Q256_SECTOR_SHIFT        (12)
#define MT25Q256_SECTOR_COUNT        (8192)
#define MT25Q256_PAGE_SIZE           (256)
#define MT25Q256_PAGE_SHIFT          (8)

/* MT25Q512 (64 MB) memory capacity */

#define MT25Q512_SECTOR_SIZE         (4 * 1024)
#define MT25Q512_SECTOR_SHIFT        (12)
#define MT25Q512_SECTOR_COUNT        (16384)
#define MT25Q512_PAGE_SIZE           (256)
#define MT25Q512_PAGE_SHIFT          (8)

/* MT25Q01 (128 MB) memory capacity */

#define MT25Q01_SECTOR_SIZE          (4 * 1024)
#define MT25Q01_SECTOR_SHIFT         (12)
#define MT25Q01_SECTOR_COUNT         (32768)
#define MT25Q01_PAGE_SIZE            (256)
#define MT25Q01_PAGE_SHIFT           (8)

/* Cache flags **************************************************************/

#define MT25QLXXX_CACHE_VALID       (1 << 0)  /* 1=Cache has valid data */
#define MT25QLXXX_CACHE_DIRTY       (1 << 1)  /* 1=Cache is dirty */
#define MT25QLXXX_CACHE_ERASED      (1 << 2)  /* 1=Backing FLASH is erased */

#define IS_VALID(p)                 ((((p)->flags) & MT25QLXXX_CACHE_VALID) != 0)
#define IS_DIRTY(p)                 ((((p)->flags) & MT25QLXXX_CACHE_DIRTY) != 0)
#define IS_ERASED(p)                ((((p)->flags) & MT25QLXXX_CACHE_ERASED) != 0)

#define SET_VALID(p)                do { (p)->flags |= MT25QLXXX_CACHE_VALID; } while (0)
#define SET_DIRTY(p)                do { (p)->flags |= MT25QLXXX_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)               do { (p)->flags |= MT25QLXXX_CACHE_ERASED; } while (0)

#define CLR_VALID(p)                do { (p)->flags &= ~MT25QLXXX_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)                do { (p)->flags &= ~MT25QLXXX_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)               do { (p)->flags &= ~MT25QLXXX_CACHE_ERASED; } while (0)

/* 512 byte sector support **************************************************/

#define MT25QLXXX_SECTOR512_SHIFT     9
#define MT25QLXXX_SECTOR512_SIZE      (1 << 9)
#define MT25QLXXX_ERASED_STATE        0xff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device. The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct mt25qlxxx_dev_s.
 */

struct mt25qlxxx_dev_s
{
  struct mtd_dev_s       mtd;         /* MTD interface */
  FAR struct qspi_dev_s *qspi;        /* Saved QuadSPI interface instance */
  uint16_t               nsectors;    /* Number of erase sectors */
  uint8_t                sectorshift; /* Log2 of sector size */
  uint8_t                pageshift;   /* Log2 of page size */
  uint8_t                addresslen;  /* Length of address 3 or 4 bytes */
  uint8_t                protectmask; /* Mask for protect bits in status register */
  uint8_t                tbmask;      /* Mask for top/bottom bit in status register */
  FAR uint8_t           *cmdbuf;      /* Allocated command buffer */
  FAR uint8_t           *readbuf;     /* Allocated status read buffer */
  FAR uint32_t           dual_flash;  /* indicate dual-flash mode */

#ifdef CONFIG_MT25QLXXX_SECTOR512
  uint8_t                flags;       /* Buffered sector flags */
  uint16_t               esectno;     /* Erase sector number in the cache */
  FAR uint8_t           *sector;      /* Allocated sector data */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Locking */

static void mt25qlxxx_lock(FAR struct qspi_dev_s *qspi);
static inline void mt25qlxxx_unlock(FAR struct qspi_dev_s *qspi);

/* Low-level message helpers */

static int mt25qlxxx_command_single(FAR struct qspi_dev_s *qspi, uint8_t cmd);
static int mt25qlxxx_command_quad(FAR struct qspi_dev_s *qspi, uint8_t cmd);
static int  mt25qlxxx_command_address(FAR struct qspi_dev_s *qspi,
                                      uint8_t cmd,
                                      off_t addr,
                                      uint8_t addrlen);
static int  mt25qlxxx_command_quad_read(FAR struct qspi_dev_s *qspi,
                                   uint8_t cmd,
                                   FAR void *buffer,
                                   size_t buflen);
static int  mt25qlxxx_command_quad_write(FAR struct qspi_dev_s *qspi,
                                    uint8_t cmd,
                                    FAR const void *buffer,
                                    size_t buflen);
static int  mt25qlxxx_command_single_read(FAR struct qspi_dev_s *qspi,
                                   uint8_t cmd,
                                   FAR void *buffer,
                                   size_t buflen);
static int  mt25qlxxx_command_single_write(FAR struct qspi_dev_s *qspi,
                                    uint8_t cmd,
                                    FAR const void *buffer,
                                    size_t buflen);
static uint8_t mt25qlxxx_read_status(FAR struct mt25qlxxx_dev_s *priv);
static void mt25qlxxx_write_status(FAR struct mt25qlxxx_dev_s *priv);
#if 0
static uint8_t mt25qlxxx_read_volcfg(FAR struct mt25qlxxx_dev_s *priv);
static void mt25qlxxx_write_volcfg(FAR struct mt25qlxxx_dev_s *priv);
#endif
static void mt25qlxxx_write_enable(FAR struct mt25qlxxx_dev_s *priv);
static void mt25qlxxx_write_disable(FAR struct mt25qlxxx_dev_s *priv);

static int  mt25qlxxx_readid(FAR struct mt25qlxxx_dev_s *priv);
static int  mt25qlxxx_protect(FAR struct mt25qlxxx_dev_s *priv,
              off_t startblock, size_t nblocks);
static int  mt25qlxxx_unprotect(FAR struct mt25qlxxx_dev_s *priv,
              off_t startblock, size_t nblocks);
static bool mt25qlxxx_isprotected(FAR struct mt25qlxxx_dev_s *priv,
              uint8_t status, off_t address);
static int  mt25qlxxx_erase_sector(FAR struct mt25qlxxx_dev_s *priv,
                                   off_t offset);
static int  mt25qlxxx_erase_chip(FAR struct mt25qlxxx_dev_s *priv);
static int  mt25qlxxx_read_byte(FAR struct mt25qlxxx_dev_s *priv,
                                FAR uint8_t *buffer,
                                off_t address,
                                size_t nbytes);
static int  mt25qlxxx_write_page(FAR struct mt25qlxxx_dev_s *priv,
                                 FAR const uint8_t *buffer,
                                 off_t address,
                                 size_t nbytes);
#ifdef CONFIG_MT25QLXXX_SECTOR512
static int  mt25qlxxx_flush_cache(struct mt25qlxxx_dev_s *priv);
static FAR uint8_t *mt25qlxxx_read_cache(struct mt25qlxxx_dev_s *priv,
                                         off_t sector);
static void mt25qlxxx_erase_cache(struct mt25qlxxx_dev_s *priv,
                                  off_t sector);
static int  mt25qlxxx_write_cache(FAR struct mt25qlxxx_dev_s *priv,
                                  FAR const uint8_t *buffer,
                                  off_t sector,
                                  size_t nsectors);
#endif

/* MTD driver methods */

static int  mt25qlxxx_erase(FAR struct mtd_dev_s *dev,
                            off_t startblock,
                            size_t nblocks);
static ssize_t mt25qlxxx_bread(FAR struct mtd_dev_s *dev,
                               off_t startblock,
                               size_t nblocks,
                               FAR uint8_t *buf);
static ssize_t mt25qlxxx_bwrite(FAR struct mtd_dev_s *dev,
                                off_t startblock,
                                size_t nblocks,
                                FAR const uint8_t *buf);
static ssize_t mt25qlxxx_read(FAR struct mtd_dev_s *dev,
                              off_t offset,
                              size_t nbytes,
                              FAR uint8_t *buffer);
static int  mt25qlxxx_ioctl(FAR struct mtd_dev_s *dev,
                            int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mt25qlxxx_lock
 ****************************************************************************/

static void mt25qlxxx_lock(FAR struct qspi_dev_s *qspi)
{
  /* On QuadSPI buses where there are multiple devices, it will be necessary
   * to lock QuadSPI to have exclusive access to the buses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusive
   * access to the QuadSPI bus.  We will retain that exclusive access until
   * the bus is unlocked.
   */

  (void)QSPI_LOCK(qspi, true);

  /* After locking the QuadSPI bus, the we also need call the setfrequency,
   * setbits, and setmode methods to make sure that the QuadSPI is properly
   * configured for the device. If the QuadSPI bus is being shared, then it
   * may have been left in an incompatible state.
   */

  QSPI_SETMODE(qspi, CONFIG_MT25QLXXX_QSPIMODE);
  QSPI_SETBITS(qspi, 8);
  (void)QSPI_SETFREQUENCY(qspi, CONFIG_MT25QLXXX_QSPI_FREQUENCY);
}

/****************************************************************************
 * Name: mt25qlxxx_unlock
 ****************************************************************************/

static inline void mt25qlxxx_unlock(FAR struct qspi_dev_s *qspi)
{
  (void)QSPI_LOCK(qspi, false);
}

/****************************************************************************
 * Name: mt25qlxxx_command_single
 ****************************************************************************/

static int mt25qlxxx_command_single(FAR struct qspi_dev_s *qspi, uint8_t cmd)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x\n", cmd);

  cmdinfo.flags   = 0;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = NULL;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: mt25qlxxx_command_quad
 ****************************************************************************/

static int mt25qlxxx_command_quad(FAR struct qspi_dev_s *qspi, uint8_t cmd)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x\n", cmd);

  cmdinfo.flags   = QSPICMD_IQUAD;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = NULL;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: mt25qlxxx_command_address
 ****************************************************************************/

static int mt25qlxxx_command_address(FAR struct qspi_dev_s *qspi,
                                     uint8_t cmd,
                                     off_t addr,
                                     uint8_t addrlen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x Address: %04lx addrlen=%d\n",
         cmd,
         (unsigned long)addr,
          addrlen);

  cmdinfo.flags   = QSPICMD_ADDRESS;
  cmdinfo.addrlen = addrlen;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = addr;
  cmdinfo.buffer  = NULL;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: mt25qlxxx_command_single_read
 ****************************************************************************/

static int mt25qlxxx_command_single_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                  FAR void *buffer, size_t buflen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x buflen: %lu\n", cmd, (unsigned long)buflen);

  cmdinfo.flags   = QSPICMD_READDATA;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = buflen;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = buffer;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: mt25qlxxx_command_quad_read
 ****************************************************************************/

static int mt25qlxxx_command_quad_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                  FAR void *buffer, size_t buflen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x buflen: %lu\n", cmd, (unsigned long)buflen);

  cmdinfo.flags   = QSPICMD_READDATA | QSPICMD_IQUAD | QSPIMEM_QUADIO;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = buflen;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = buffer;

  return QSPI_COMMAND(qspi, &cmdinfo);
}


/****************************************************************************
 * Name: mt25qlxxx_command_single_write
 ****************************************************************************/

static int mt25qlxxx_command_single_write(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                   FAR const void *buffer, size_t buflen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x buflen: %lu\n", cmd, (unsigned long)buflen);

  cmdinfo.flags   = QSPICMD_WRITEDATA;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = buflen;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = (FAR void *)buffer;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: mt25qlxxx_command_quad_write
 ****************************************************************************/

static int mt25qlxxx_command_quad_write(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                   FAR const void *buffer, size_t buflen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x buflen: %lu\n", cmd, (unsigned long)buflen);

  cmdinfo.flags   = QSPICMD_WRITEDATA | QSPICMD_IQUAD | QSPIMEM_QUADIO;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = buflen;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = (FAR void *)buffer;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/****************************************************************************
 * Name: mt25qlxxx_read_status
 ****************************************************************************/

static uint8_t mt25qlxxx_read_status(FAR struct mt25qlxxx_dev_s *priv)
{
  DEBUGVERIFY(mt25qlxxx_command_quad_read(priv->qspi, MT25QLXXX_READ_STATUS,
                                     (FAR void *)&priv->readbuf[0], 1));
  return priv->readbuf[0];
}

/****************************************************************************
 * Name:  mt25qlxxx_write_status
 ****************************************************************************/

static void mt25qlxxx_write_status(FAR struct mt25qlxxx_dev_s *priv)
{
  mt25qlxxx_write_enable(priv);

  /* Keep in Software Protection */

  priv->cmdbuf[0] &= ~STATUS_SRP_MASK;

  mt25qlxxx_command_quad_write(priv->qspi, MT25QLXXX_WRITE_STATUS,
                       (FAR const void *)priv->cmdbuf, 1);
  mt25qlxxx_write_disable(priv);
}

/****************************************************************************
 * Name:  mt25qlxxx_write_enable
 ****************************************************************************/

static void mt25qlxxx_write_enable(FAR struct mt25qlxxx_dev_s *priv)
{
  uint8_t status;

  do
    {
      mt25qlxxx_command_quad(priv->qspi, MT25QLXXX_WRITE_ENABLE);
      status = mt25qlxxx_read_status(priv);
    }
  while ((status & STATUS_WEL_MASK) != STATUS_WEL_ENABLED);
}

/****************************************************************************
 * Name:  mt25qlxxx_write_disable
 ****************************************************************************/

static void mt25qlxxx_write_disable(FAR struct mt25qlxxx_dev_s *priv)
{
  uint8_t status;

  do
    {
      mt25qlxxx_command_quad(priv->qspi, MT25QLXXX_WRITE_DISABLE);
      status = mt25qlxxx_read_status(priv);
    }
  while ((status & STATUS_WEL_MASK) != STATUS_WEL_DISABLED);
}


/****************************************************************************
 * Name: mt25qlxxx_readid
 ****************************************************************************/

static inline int mt25qlxxx_readid(struct mt25qlxxx_dev_s *priv)
{
  /* Lock the QuadSPI bus and configure the bus. */
  uint8_t dev_manufacturer = 0;
  uint8_t dev_type = 0;
  uint8_t dev_capacity = 0;

  mt25qlxxx_lock(priv->qspi);

  /* Read the JEDEC ID */
  if(priv->dual_flash){
    mt25qlxxx_command_quad_read(priv->qspi, MT25QLXXX_MULT_IO_JEDEC_ID, priv->cmdbuf, 2 * 3);
    /* only fetch flash-1 info */
    dev_manufacturer = priv->cmdbuf[0];
    dev_type = priv->cmdbuf[2];
    dev_capacity = priv->cmdbuf[4];
  }
  else{
    mt25qlxxx_command_quad_read(priv->qspi, MT25QLXXX_MULT_IO_JEDEC_ID, priv->cmdbuf, 3);

    dev_manufacturer = priv->cmdbuf[0];
    dev_type = priv->cmdbuf[1];
    dev_capacity = priv->cmdbuf[2];
  }

  /* Unlock the bus */

  mt25qlxxx_unlock(priv->qspi);

  finfo("Manufacturer: %02x Device Type %02x, Capacity: %02x\n",
        dev_manufacturer, dev_type, dev_capacity);

  /* Check for a recognized memory device type */

  if ((dev_type != MT25QLXXXQ_JEDEC_DEVICE_3V) &&
      (dev_type != MT25QLXXXQ_JEDEC_DEVICE_1V8))
    {
      ferr("ERROR: Unrecognized device type: 0x%02x\n", dev_type);
      return -ENODEV;
    }

  /* Check for a supported capacity */

  switch (dev_capacity)
    {
      case MT25Q016_JEDEC_CAPACITY:
        priv->sectorshift = MT25Q016_SECTOR_SHIFT;
        priv->pageshift   = MT25Q016_PAGE_SHIFT;
        priv->nsectors    = MT25Q016_SECTOR_COUNT;
        priv->addresslen  = 3;
        priv->protectmask = STATUS_BP_MASK;
        priv->tbmask      = STATUS_TB_MASK;
        break;

      case MT25Q032_JEDEC_CAPACITY:
        priv->sectorshift = MT25Q032_SECTOR_SHIFT;
        priv->pageshift   = MT25Q032_PAGE_SHIFT;
        priv->nsectors    = MT25Q032_SECTOR_COUNT;
        priv->addresslen  = 3;
        priv->protectmask = STATUS_BP_MASK;
        priv->tbmask      = STATUS_TB_MASK;
        break;

      case MT25Q064_JEDEC_CAPACITY:
        priv->sectorshift = MT25Q064_SECTOR_SHIFT;
        priv->pageshift   = MT25Q064_PAGE_SHIFT;
        priv->nsectors    = MT25Q064_SECTOR_COUNT;
        priv->addresslen  = 3;
        priv->protectmask = STATUS_BP_4_MASK;
        priv->tbmask      = STATUS_TB_6_MASK;
        break;

      case MT25Q128_JEDEC_CAPACITY:
        priv->sectorshift = MT25Q128_SECTOR_SHIFT;
        priv->pageshift   = MT25Q128_PAGE_SHIFT;
        priv->nsectors    = MT25Q128_SECTOR_COUNT;
        priv->addresslen  = 3;
        priv->protectmask = STATUS_BP_MASK;
        priv->tbmask      = STATUS_TB_MASK;
        break;

      case MT25Q256_JEDEC_CAPACITY:
        priv->sectorshift = MT25Q256_SECTOR_SHIFT;
        priv->pageshift   = MT25Q256_PAGE_SHIFT;
        priv->nsectors    = MT25Q256_SECTOR_COUNT;
        priv->addresslen  = 4;
        priv->protectmask = STATUS_BP_4_MASK;
        priv->tbmask      = STATUS_TB_6_MASK;
        break;

      case MT25Q512_JEDEC_CAPACITY:
        priv->sectorshift = MT25Q512_SECTOR_SHIFT;
        priv->pageshift   = MT25Q512_PAGE_SHIFT;
        priv->nsectors    = MT25Q512_SECTOR_COUNT;
        priv->addresslen  = 4;
        priv->protectmask = STATUS_BP_4_MASK;
        priv->tbmask      = STATUS_TB_6_MASK;
        break;

      case MT25Q01_JEDEC_CAPACITY:
        priv->sectorshift = MT25Q01_SECTOR_SHIFT;
        priv->pageshift   = MT25Q01_PAGE_SHIFT;
        priv->nsectors    = MT25Q01_SECTOR_COUNT;
        priv->addresslen  = 4;
        priv->protectmask = STATUS_BP_4_MASK;
        priv->tbmask      = STATUS_TB_6_MASK;
        break;

      /* Support for this part is not implemented yet */

      default:
        ferr("ERROR: Unsupported memory capacity: %02x\n", dev_capacity);
        return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: mt25qlxxx_protect
 ****************************************************************************/

static int mt25qlxxx_protect(FAR struct mt25qlxxx_dev_s *priv,
                             off_t startblock, size_t nblocks)
{
  /* Get the status register value to check the current protection */

  priv->cmdbuf[0] = mt25qlxxx_read_status(priv);

  if ((priv->cmdbuf[0] & priv->protectmask) ==
                           (STATUS_BP_ALL & priv->protectmask))
    {
      /* Protection already enabled */

      return 0;
    }

  /* set the BP bits as necessary to protect the range of sectors. */

  priv->cmdbuf[0] |= (STATUS_BP_ALL & priv->protectmask);
  mt25qlxxx_write_status(priv);

  /* Check the new status */

  priv->cmdbuf[0] = mt25qlxxx_read_status(priv);
  if ((priv->cmdbuf[0] & priv->protectmask) !=
                            (STATUS_BP_ALL & priv->protectmask))
    {
      return -EACCES;
    }

  return OK;
}

/****************************************************************************
 * Name: mt25qlxxx_unprotect
 ****************************************************************************/

static int mt25qlxxx_unprotect(FAR struct mt25qlxxx_dev_s *priv,
                               off_t startblock, size_t nblocks)
{
  /* Get the status register value to check the current protection */

  priv->cmdbuf[0] = mt25qlxxx_read_status(priv);

  if ((priv->cmdbuf[0] & priv->protectmask) == STATUS_BP_NONE)
    {
      /* Protection already disabled */

      return 0;
    }

  /* Set the protection mask to zero (and not complemented).
   * REVISIT:  This logic should really just re-write the BP bits as
   * necessary to unprotect the range of sectors.
   */

  priv->cmdbuf[0] &= ~priv->protectmask;
  mt25qlxxx_write_status(priv);

  /* Check the new status */

  priv->cmdbuf[0] = mt25qlxxx_read_status(priv);
  if ((priv->cmdbuf[0] & (STATUS_SRP_MASK | priv->protectmask)) != 0)
    {
      return -EACCES;
    }

  return OK;
}

/****************************************************************************
 * Name: mt25qlxxx_isprotected
 ****************************************************************************/

static bool mt25qlxxx_isprotected(FAR struct mt25qlxxx_dev_s *priv,
                                  uint8_t status,
                                  off_t address)
{
  off_t protstart;
  off_t protend;
  off_t protsize;
  unsigned int bp;

  /* The BP field is spread across non-contiguous bits */

  bp = (status & priv->protectmask) >> STATUS_BP_SHIFT;

  /* the BP field is essentially the power-of-two of the number of 64k
   * sectors, saturated to the device size.
   */

  if (0 == bp)
    {
      return false;
    }

  protsize = 0x00010000;
  protsize <<= (protsize << (bp - 1));
  protend = (1 << priv->sectorshift) * priv->nsectors;
  if (protsize > protend)
    {
      protsize = protend;
    }

  /* The final protection range then depends on if the protection region is
   * configured top-down or bottom up  (assuming CMP=0).
   */

  if ((status & priv->tbmask) != 0)
    {
      protstart = 0x00000000;
      protend   = protstart + protsize;
    }
  else
    {
      protstart = protend - protsize;

      /* protend already computed above */
    }

  return (address >= protstart && address < protend);
}

/****************************************************************************
 * Name:  mt25qlxxx_erase_sector
 ****************************************************************************/

static int mt25qlxxx_erase_sector(FAR struct mt25qlxxx_dev_s *priv,
                                  off_t sector)
{
  off_t address;
  uint8_t status;

  finfo("sector: %08lx\n", (unsigned long)sector);

  /* Check that the flash is ready and unprotected */

  status = mt25qlxxx_read_status(priv);
  if ((status & STATUS_BUSY_MASK) != STATUS_READY)
    {
      ferr("ERROR: Flash busy: %02x", status);
      return -EBUSY;
    }

  /* Get the address associated with the sector */

  address = (off_t)sector << priv->sectorshift;

  if ((status & priv->protectmask) != 0 &&
       mt25qlxxx_isprotected(priv, status, address))
    {
      ferr("ERROR: Flash protected: %02x", status);
      return -EACCES;
    }

  /* Send the sector erase command */

  mt25qlxxx_write_enable(priv);
  mt25qlxxx_command_address(priv->qspi,
                            MT25QLXXX_SECTOR_ERASE,
                            address, priv->addresslen);

  /* Wait for erasure to finish */

  while ((mt25qlxxx_read_status(priv) & STATUS_BUSY_MASK) != 0);

  return OK;
}

/****************************************************************************
 * Name:  mt25qlxxx_erase_chip
 ****************************************************************************/

static int mt25qlxxx_erase_chip(FAR struct mt25qlxxx_dev_s *priv)
{
  uint8_t status;

  /* Check if the FLASH is protected */

  status = mt25qlxxx_read_status(priv);
  if ((status & priv->protectmask) != 0)
    {
      ferr("ERROR: FLASH is Protected: %02x", status);
      return -EACCES;
    }

  /* Erase the whole chip */

  mt25qlxxx_write_enable(priv);
  mt25qlxxx_command_quad(priv->qspi, MT25QLXXX_CHIP_ERASE);

  /* Wait for the erasure to complete */

  status = mt25qlxxx_read_status(priv);
  while ((status & STATUS_BUSY_MASK) != 0)
    {
      nxsig_usleep(200  *1000);
      status = mt25qlxxx_read_status(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: mt25qlxxx_read_byte
 ****************************************************************************/

static int mt25qlxxx_read_byte(FAR struct mt25qlxxx_dev_s *priv,
                               FAR uint8_t *buffer,
                               off_t address, size_t buflen)
{
  struct qspi_meminfo_s meminfo;

  finfo("address: %08lx nbytes: %d\n", (long)address, (int)buflen);

  meminfo.flags   = QSPIMEM_READ | QSPIMEM_QUADIO;
  meminfo.addrlen = priv->addresslen;
  meminfo.dummies = CONFIG_MT25QLXXX_DUMMIES;
  meminfo.buflen  = buflen;
  meminfo.cmd     = MT25QLXXX_FAST_READ_QUADIO;
  meminfo.addr    = address;
  meminfo.buffer  = buffer;

  return QSPI_MEMORY(priv->qspi, &meminfo);
}

/****************************************************************************
 * Name:  mt25qlxxx_write_page
 ****************************************************************************/

static int mt25qlxxx_write_page(struct mt25qlxxx_dev_s *priv,
                                FAR const uint8_t *buffer,
                                off_t address, size_t buflen)
{
  struct qspi_meminfo_s meminfo;
  unsigned int pagesize;
  unsigned int npages;
  int ret;
  int i;

  finfo("address: %08lx buflen: %u\n",
        (unsigned long)address,
        (unsigned)buflen);

  npages   = (buflen >> priv->pageshift);
  pagesize = (1 << priv->pageshift);

  /* Set up non-varying parts of transfer description */

  meminfo.flags   = QSPIMEM_WRITE;
  meminfo.cmd     = MT25QLXXX_PAGE_PROGRAM;
  meminfo.addrlen = priv->addresslen;
  meminfo.buflen  = pagesize;
  meminfo.dummies = 0;

  /* Then write each page */

  for (i = 0; i < npages; i++)
    {
      /* Set up varying parts of the transfer description */

      meminfo.addr   = address;
      meminfo.buffer = (void *)buffer;

      /* Write one page */

      mt25qlxxx_write_enable(priv);
      ret = QSPI_MEMORY(priv->qspi, &meminfo);
      mt25qlxxx_write_disable(priv);

      if (ret < 0)
        {
          ferr("ERROR: QSPI_MEMORY failed writing address=%06"PRIxOFF"\n",
               address);
          return ret;
        }

      /* Update for the next time through the loop */

      buffer  += pagesize;
      address += pagesize;
      buflen  -= pagesize;
    }

  /* The transfer should always be an even number of sectors and hence also
   * pages.  There should be no remainder.
   */

  DEBUGASSERT(buflen == 0);

  return OK;
}

/****************************************************************************
 * Name: mt25qlxxx_flush_cache
 ****************************************************************************/

#ifdef CONFIG_MT25QLXXX_SECTOR512
static int mt25qlxxx_flush_cache(struct mt25qlxxx_dev_s *priv)
{
  int ret = OK;

  /* If the cache is dirty (meaning that it no longer matches the old FLASH
   * contents) or was erased (with the cache containing the correct FLASH
   * contents), then write the cached erase block to FLASH.
   */

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      off_t address;

      /* Convert the erase sector number into a FLASH address */

      address = (off_t)priv->esectno << priv->sectorshift;

      /* Write entire erase block to FLASH */

      ret = mt25qlxxx_write_page(priv,
                                 priv->sector,
                                 address,
                                 1 << priv->sectorshift);
      if (ret < 0)
        {
          ferr("ERROR: mt25qlxxx_write_page failed: %d\n", ret);
        }

      /* The cache is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: mt25qlxxx_read_cache
 ****************************************************************************/

#ifdef CONFIG_MT25QLXXX_SECTOR512
static FAR uint8_t *mt25qlxxx_read_cache(struct mt25qlxxx_dev_s *priv,
                                         off_t sector)
{
  off_t esectno;
  int   shift;
  int   index;
  int   ret;

  /* Convert from the 512 byte sector to the erase sector size of the device.
   * For example, if the actual erase sector size is 4Kb (1 << 12), then we
   * first shift to the right by 3 to get the sector number in 4096
   * increments.
   */

  shift    = priv->sectorshift - MT25QLXXX_SECTOR512_SHIFT;
  esectno  = sector >> shift;
  finfo("sector: %ld esectno: %d shift=%d\n", sector, esectno, shift);

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || esectno != priv->esectno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      ret = mt25qlxxx_flush_cache(priv);
      if (ret < 0)
        {
          ferr("ERROR: mt25qlxxx_flush_cache failed: %d\n", ret);
          return NULL;
        }

      /* Read the erase block into the cache */

      ret = mt25qlxxx_read_byte(priv, priv->sector,
                             (esectno << priv->sectorshift),
                             (1 << priv->sectorshift));
      if (ret < 0)
        {
          ferr("ERROR: mt25qlxxx_read_byte failed: %d\n", ret);
          return NULL;
        }

      /* Mark the sector as cached */

      priv->esectno = esectno;

      SET_VALID(priv);          /* The data in the cache is valid */
      CLR_DIRTY(priv);          /* It should match the FLASH contents */
      CLR_ERASED(priv);         /* The underlying FLASH has not been erased */
    }

  /* Get the index to the 512 sector in the erase block that holds the
   * argument
   */

  index = sector & ((1 << shift) - 1);

  /* Return the address in the cache that holds this sector */

  return &priv->sector[index << MT25QLXXX_SECTOR512_SHIFT];
}
#endif

/****************************************************************************
 * Name: mt25qlxxx_erase_cache
 ****************************************************************************/

#ifdef CONFIG_MT25QLXXX_SECTOR512
static void mt25qlxxx_erase_cache(struct mt25qlxxx_dev_s *priv, off_t sector)
{
  FAR uint8_t *dest;

  /* First, make sure that the erase block containing the 512 byte sector is
   * in the cache.
   */

  dest = mt25qlxxx_read_cache(priv, sector);

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase
   * sector is read into the cache and set here when we erase the block.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >>
                      (priv->sectorshift - MT25QLXXX_SECTOR512_SHIFT);
      finfo("sector: %ld esectno: %d\n", sector, esectno);

      DEBUGVERIFY(mt25qlxxx_erase_sector(priv, esectno));
      SET_ERASED(priv);
    }

  /* Put the cached sector data into the erase state and mark the cache as
   * dirty(but don't update the FLASH yet.  The caller will do that at a
   * more optimal time).
   */

  memset(dest, MT25QLXXX_ERASED_STATE, MT25QLXXX_SECTOR512_SIZE);
  SET_DIRTY(priv);
}
#endif

/****************************************************************************
 * Name: mt25qlxxx_write_cache
 ****************************************************************************/

#ifdef CONFIG_MT25QLXXX_SECTOR512
static int mt25qlxxx_write_cache(FAR struct mt25qlxxx_dev_s *priv,
                                 FAR const uint8_t *buffer, off_t sector,
                                 size_t nsectors)
{
  FAR uint8_t *dest;
  int ret;

  for (; nsectors > 0; nsectors--)
    {
      /* First, make sure that the erase block containing 512 byte sector is
       * in memory.
       */

      dest = mt25qlxxx_read_cache(priv, sector);

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicated will be cleared when the data from the erase
       * sector is read into the cache and set here when we erase the sector.
       */

      if (!IS_ERASED(priv))
        {
          off_t esectno  = sector >>
                           (priv->sectorshift - MT25QLXXX_SECTOR512_SHIFT);
          finfo("sector: %ld esectno: %d\n", sector, esectno);

          ret = mt25qlxxx_erase_sector(priv, esectno);
          if (ret < 0)
            {
              ferr("ERROR: mt25qlxxx_erase_sector failed: %d\n", ret);
              return ret;
            }

          SET_ERASED(priv);
        }

      /* Copy the new sector data into cached erase block */

      memcpy(dest, buffer, MT25QLXXX_SECTOR512_SIZE);
      SET_DIRTY(priv);

      /* Set up for the next 512 byte sector */

      buffer += MT25QLXXX_SECTOR512_SIZE;
      sector++;
    }

  /* Flush the last erase block left in the cache */

  return mt25qlxxx_flush_cache(priv);
}
#endif

/****************************************************************************
 * Name: mt25qlxxx_erase
 ****************************************************************************/

static int mt25qlxxx_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks)
{
  FAR struct mt25qlxxx_dev_s *priv = (FAR struct mt25qlxxx_dev_s *)dev;
  size_t blocksleft = nblocks;
#ifdef CONFIG_MT25QLXXX_SECTOR512
  int ret;
#endif

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  mt25qlxxx_lock(priv->qspi);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

#ifdef CONFIG_MT25QLXXX_SECTOR512
      mt25qlxxx_erase_cache(priv, startblock);
#else
      mt25qlxxx_erase_sector(priv, startblock);
#endif
      startblock++;
    }

#ifdef CONFIG_MT25QLXXX_SECTOR512
  /* Flush the last erase block left in the cache */

  ret = mt25qlxxx_flush_cache(priv);
  if (ret < 0)
    {
      nblocks = ret;
    }
#endif

  mt25qlxxx_unlock(priv->qspi);

  return (int)nblocks;
}

/****************************************************************************
 * Name: mt25qlxxx_bread
 ****************************************************************************/

static ssize_t mt25qlxxx_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                               size_t nblocks, FAR uint8_t *buffer)
{
#ifndef CONFIG_MT25QLXXX_SECTOR512
  FAR struct mt25qlxxx_dev_s *priv = (FAR struct mt25qlxxx_dev_s *)dev;
#endif
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

#ifdef CONFIG_MT25QLXXX_SECTOR512
  nbytes = mt25qlxxx_read(dev, startblock << MT25QLXXX_SECTOR512_SHIFT,
                       nblocks << MT25QLXXX_SECTOR512_SHIFT, buffer);
  if (nbytes > 0)
    {
      nbytes >>= MT25QLXXX_SECTOR512_SHIFT;
    }
#else
  nbytes = mt25qlxxx_read(dev, startblock << priv->pageshift,
                       nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      nbytes >>= priv->pageshift;
    }
#endif

  return nbytes;
}

/****************************************************************************
 * Name: mt25qlxxx_bwrite
 ****************************************************************************/

static ssize_t mt25qlxxx_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                                size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct mt25qlxxx_dev_s *priv = (FAR struct mt25qlxxx_dev_s *)dev;
  int ret = (int)nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the QuadSPI bus and write all of the pages to FLASH */

  mt25qlxxx_lock(priv->qspi);

#if defined(CONFIG_MT25QLXXX_SECTOR512)
  ret = mt25qlxxx_write_cache(priv, buffer, startblock, nblocks);
  if (ret < 0)
    {
      ferr("ERROR: mt25qlxxx_write_cache failed: %d\n", ret);
    }

#else
  ret = mt25qlxxx_write_page(priv, buffer, startblock << priv->pageshift,
                          nblocks << priv->pageshift);
  if (ret < 0)
    {
      ferr("ERROR: mt25qlxxx_write_page failed: %d\n", ret);
    }
#endif

  mt25qlxxx_unlock(priv->qspi);

  return ret < 0 ? ret : nblocks;
}

/****************************************************************************
 * Name: mt25qlxxx_read
 ****************************************************************************/

static ssize_t mt25qlxxx_read(FAR struct mtd_dev_s *dev,
                              off_t offset,
                              size_t nbytes,
                              FAR uint8_t *buffer)
{
  FAR struct mt25qlxxx_dev_s *priv = (FAR struct mt25qlxxx_dev_s *)dev;
  int ret;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the QuadSPI bus and select this FLASH part */

  mt25qlxxx_lock(priv->qspi);
  ret = mt25qlxxx_read_byte(priv, buffer, offset, nbytes);
  mt25qlxxx_unlock(priv->qspi);

  if (ret < 0)
    {
      ferr("ERROR: mt25qlxxx_read_byte returned: %d\n", ret);
      return (ssize_t)ret;
    }

  finfo("return nbytes: %d\n", (int)nbytes);
  return (ssize_t)nbytes;
}

/****************************************************************************
 * Name: mt25qlxxx_ioctl
 ****************************************************************************/

static int mt25qlxxx_ioctl(FAR struct mtd_dev_s *dev,
                           int cmd,
                           unsigned long arg)
{
  FAR struct mt25qlxxx_dev_s *priv = (FAR struct mt25qlxxx_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the
               * client will expect the device logic to do whatever is
               * necessary to make it appear so.
               */

#ifdef CONFIG_MT25QLXXX_SECTOR512
              geo->blocksize    = (1 << MT25QLXXX_SECTOR512_SHIFT);
              geo->erasesize    = (1 << MT25QLXXX_SECTOR512_SHIFT);
              geo->neraseblocks = priv->nsectors <<
                                  (priv->sectorshift -
                                   MT25QLXXX_SECTOR512_SHIFT);
#else
              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;
#endif
              ret               = OK;

              finfo("blocksize: %lu erasesize: %lu neraseblocks: %lu\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)arg;
          if (info != NULL)
            {
#ifdef CONFIG_MT25QLXXX_SECTOR512
              info->numsectors  = priv->nsectors <<
                             (priv->sectorshift - MT25QLXXX_SECTOR512_SHIFT);
              info->sectorsize  = 1 << MT25QLXXX_SECTOR512_SHIFT;
#else
              info->numsectors  = priv->nsectors <<
                                  (priv->sectorshift - priv->pageshift);
              info->sectorsize  = 1 << priv->pageshift;
#endif
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          mt25qlxxx_lock(priv->qspi);
          ret = mt25qlxxx_erase_chip(priv);
          mt25qlxxx_unlock(priv->qspi);
        }
        break;

      case MTDIOC_PROTECT:
        {
          FAR const struct mtd_protect_s *prot =
            (FAR const struct mtd_protect_s *)((uintptr_t)arg);

          DEBUGASSERT(prot);
          ret = mt25qlxxx_protect(priv, prot->startblock, prot->nblocks);
        }
        break;

      case MTDIOC_UNPROTECT:
        {
          FAR const struct mtd_protect_s *prot =
            (FAR const struct mtd_protect_s *)((uintptr_t)arg);

          DEBUGASSERT(prot);
          ret = mt25qlxxx_unprotect(priv, prot->startblock, prot->nblocks);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = MT25QLXXX_ERASED_STATE;

          ret = OK;
        }
        break;

      default:
        ret = -ENOTTY; /* Bad/unsupported command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mt25qlxxx_initialize
 *
 * Description:
 *   Create an initialize MTD device instance for the QuadSPI-based MT25QxxxJV
 *   FLASH part.
 *
 *   MTD devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *mt25qlxxx_initialize(FAR struct qspi_dev_s *qspi,
                                           bool unprotect)
{
  FAR struct mt25qlxxx_dev_s *priv;
  int ret;

  finfo("qspi: %p\n", qspi);
  DEBUGASSERT(qspi != NULL);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per QuadSPI
   * device (only because of the QSPIDEV_FLASH(0) definition) and so would
   * have to be extended to handle multiple FLASH parts on the same QuadSPI
   * bus.
   */

  priv = (FAR struct mt25qlxxx_dev_s *)
         kmm_zalloc(sizeof(struct mt25qlxxx_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = mt25qlxxx_erase;
      priv->mtd.bread  = mt25qlxxx_bread;
      priv->mtd.bwrite = mt25qlxxx_bwrite;
      priv->mtd.read   = mt25qlxxx_read;
      priv->mtd.ioctl  = mt25qlxxx_ioctl;
      priv->mtd.name   = "mt25qlxxx";
      priv->qspi       = qspi;

#if CONFIG_MT25QLXXX_QSPI_MODE_DUAL      
      priv->dual_flash = true;
#else
      priv->dual_flash = false;
#endif
      /* Allocate a 4-byte buffer per flash to support DMA-able command data */

      priv->cmdbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, priv->dual_flash ? 8 : 4);
      if (priv->cmdbuf == NULL)
        {
          ferr("ERROR Failed to allocate command buffer\n");
          goto errout_with_priv;
        }

      /* Allocate a one-byte buffer to support DMA-able status read data */

      priv->readbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, priv->dual_flash? 2 : 1);
      if (priv->readbuf == NULL)
        {
          ferr("ERROR Failed to allocate read buffer\n");
          goto errout_with_cmdbuf;
        }

      mt25qlxxx_lock(priv->qspi);


      /* software reset memory */
      ret = mt25qlxxx_command_quad(priv->qspi, MT25QLXXX_RESET_ENABLE);
      if (ret != OK)
        {
          ferr("ERROR: Failed to exit qpi mode\n");
        }
      ret = mt25qlxxx_command_quad(priv->qspi, MT25QLXXX_DEVICE_RESET);
      if (ret != OK)
        {
          ferr("ERROR: Failed to exit qpi mode\n");
        }

      ret = mt25qlxxx_command_single(priv->qspi, MT25QLXXX_RESET_ENABLE);
      if (ret != OK)
        {
          ferr("ERROR: Failed to exit qpi mode\n");
        }
      ret = mt25qlxxx_command_single(priv->qspi, MT25QLXXX_DEVICE_RESET);
      if (ret != OK)
        {
          ferr("ERROR: Failed to exit qpi mode\n");
        }

      up_udelay(1000);


      ret = mt25qlxxx_command_single(priv->qspi, MT25QLXXX_WRITE_ENABLE);
      if (ret != OK)
        {
          ferr("ERROR: Failed to enable write\n");
        }

      /* 0xAB 0xAB for 10 dummy clocks */
      priv->cmdbuf[0] = 0xAB;
      priv->cmdbuf[1] = 0xAB;
      ret = mt25qlxxx_command_single_write(priv->qspi, MT25QLXXX_WR_CFG_REG, priv->cmdbuf, 2);
      if (ret != OK)
        {
          ferr("ERROR: Failed to write cfg\n");
        }
      up_udelay(1000);

      ret = mt25qlxxx_command_single(priv->qspi, MT25QLXXX_WRITE_ENABLE);
      if (ret != OK)
        {
          ferr("ERROR: Failed to enable write\n");
        }

      /* Enable QPI mode via enhanced volatile configuration register */
      priv->cmdbuf[0] = 0x3F;
      priv->cmdbuf[1] = 0x3F;
      ret = mt25qlxxx_command_single_write(priv->qspi, MT25QLXXX_ENHANCED_CFG_REG, priv->cmdbuf, 2);
      if (ret != OK)
        {
          ferr("ERROR: Failed to write cfg\n");
        }

      up_udelay(1000);

      ret = mt25qlxxx_command_single(priv->qspi, MT25QLXXX_ENTER_QPI_MODE);
      if (ret != OK)
        {
          ferr("ERROR: Failed to wnter qpi mode\n");
        }

      up_udelay(1000);


      mt25qlxxx_unlock(priv->qspi);


      /* Identify the FLASH chip and get its capacity */

      ret = mt25qlxxx_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and
           * return NULL
           */

          ferr("ERROR Unrecognized QSPI device\n");
          goto errout_with_readbuf;
        }

      /* Enter 4-byte address mode if chip is 4-byte addressable */

      if (priv->addresslen == 4)
        {
          mt25qlxxx_lock(priv->qspi);
          ret = mt25qlxxx_command_quad(priv->qspi, MT25QLXXX_ENTER_4BT_MODE);
          if (ret != OK)
            {
              ferr("ERROR: Failed to enter 4 byte mode\n");
            }

          mt25qlxxx_unlock(priv->qspi);
        }

      /* Unprotect FLASH sectors if so requested. */

      if (unprotect)
        {
          ret = mt25qlxxx_unprotect(priv, 0, priv->nsectors - 1);
          if (ret < 0)
            {
              ferr("ERROR: Sector unprotect failed\n");
            }
        }


#ifdef CONFIG_MT25QLXXX_SECTOR512  /* Simulate a 512 byte sector */
      /* Allocate a buffer for the erase block cache */

      priv->sector = (FAR uint8_t *)QSPI_ALLOC(qspi, 1 << priv->sectorshift);
      if (priv->sector == NULL)
        {
          /* Allocation failed! Discard all of that work we just did and
           * return NULL
           */

          ferr("ERROR: Sector allocation failed\n");
          goto errout_with_readbuf;
        }
#endif
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;

errout_with_readbuf:
  QSPI_FREE(qspi, priv->readbuf);

errout_with_cmdbuf:
  QSPI_FREE(qspi, priv->cmdbuf);

errout_with_priv:
  kmm_free(priv);
  return NULL;
}
