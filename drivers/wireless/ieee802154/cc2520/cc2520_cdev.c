/****************************************************************************
 * drivers/rf/cc2520_cdev_driver.c
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/* CC2520 Test Driver */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/rf/ioctl.h>
#include <nuttx/rf/attenuator.h>
#include <nuttx/crc16.h>

#include "cc2520.h"
#include "cc2520_reg.h"
#include "cc2520_regops.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cc2520_cdev_driver_dev_s
{
    off_t offset;
    struct cc2520_radio_s *dev;
    mutex_t lock;		                        /* Lock for dev data*/

};


typedef enum CC2520_CFG_E{
    CC2520_CFG_CHL = 1,
    CC2520_CFG_PROMIS,
    CC2520_CFG_STROBE,
}CC2520_CFG_e;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int 
cc2520_cdev_driver_open(FAR struct file *filep);

static int 
cc2520_cdev_driver_close(FAR struct file *filep);

static ssize_t 
cc2520_cdev_driver_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);

static ssize_t 
cc2520_cdev_driver_write(FAR struct file *filep,
                            FAR const char *buffer, size_t buflen);

static int 
cc2520_cdev_driver_ioctl(FAR struct file *filep, int cmd,
                        unsigned long arg);

static off_t 
cc2520_cdev_driver_seek(FAR struct file *filep, off_t offset, int whence);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_cc2520_cdev_driver_fops =
{
    cc2520_cdev_driver_open,
    cc2520_cdev_driver_close,
    cc2520_cdev_driver_read,
    cc2520_cdev_driver_write,
    cc2520_cdev_driver_seek,  
    cc2520_cdev_driver_ioctl,
    NULL   /* Poll not implemented */
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: cc2520_cdev_driver_open
 *
 * Description:
 *   This function is called whenever the device is opened.
 *
 ****************************************************************************/

static int cc2520_cdev_driver_open(FAR struct file *filep)
{
    _info("\n");
    DEBUGASSERT(filep != NULL);
    return OK;
}

/****************************************************************************
 * Name: cc2520_cdev_driver_close
 *
 * Description:
 *   This function is called whenever the device is closed.
 *
 ****************************************************************************/

static int cc2520_cdev_driver_close(FAR struct file *filep)
{
    _info("\n");
    DEBUGASSERT(filep != NULL);
    return OK;
}

/****************************************************************************
 * Name: cc2520_cdev_driver_write
 *
 * Description:
 *   Write the buffer to the device.
 ****************************************************************************/

static ssize_t cc2520_cdev_driver_write(FAR struct file *filep,
                            FAR const char *buffer,
                            size_t buflen)
{
	int ret = OK;
    uint8_t status;
    uint32_t pkt_len;
    uint16_t fcs; 

    _info("buflen=%u\n", buflen);
    DEBUGASSERT(buffer != NULL);
    DEBUGASSERT(filep  != NULL);


    FAR struct inode *inode = filep->f_inode;
    DEBUGASSERT(inode != NULL);
    FAR struct cc2520_cdev_driver_dev_s *priv = inode->i_private;
    DEBUGASSERT(priv != NULL);

    if((priv->offset >= CC2520RAM_TXFIFO) && 
        (priv->offset < CC2520RAM_TXFIFO + CC2520_FIFO_SIZE)){
            

        pkt_len = buflen + 2; /* add 2 bytes FCS */

        ret = cc2520_cmd_strobe(priv->dev, CC2520_CMD_SFLUSHTX);
        if (ret)
            goto err_tx;

        if (priv->dev->promiscuous) {
            fcs = crc16ccitt( buffer, buflen);
            ret = cc2520_write_txfifo_fcs(priv->dev, pkt_len, (uint8_t*)buffer, buflen, fcs);
            if (ret < 0)
            {
                wlerr("cc2520_write_txfifo failed: %d\n", ret);
                return 0;
            }

        }
        else{
            ret = cc2520_write_txfifo(priv->dev, pkt_len, (uint8_t*)buffer, buflen);
            if (ret < 0)
            {
                wlerr("cc2520_write_txfifo failed: %d\n", ret);
                return 0;
            }

        }

        ret = cc2520_get_status(priv->dev, &status);
        if (ret)
            goto err_tx;

        if (status & CC2520_STATUS_TX_UNDERFLOW) {
            wlerr("cc2520 tx underflow exception\n");
            goto err_tx;
        }

        /* trigger tx */
        ret = cc2520_cmd_strobe(priv->dev, CC2520_CMD_STXONCCA);
        if (ret)
            goto err_tx;

    }

err_tx:

    return buflen;
}

/****************************************************************************
 * Name: cc2520_cdev_driver_read
 *
 * Description:
 *   Return the data received from the device.
 ****************************************************************************/

static ssize_t cc2520_cdev_driver_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
	int ret = OK;


    _info("buflen=%u\n", buflen);
    DEBUGASSERT(buflen < 0x400);
    DEBUGASSERT(filep  != NULL);
    DEBUGASSERT(buffer != NULL);


    FAR struct inode *inode = filep->f_inode;
    DEBUGASSERT(inode != NULL);
    FAR struct cc2520_cdev_driver_dev_s *priv = inode->i_private;
    DEBUGASSERT(priv != NULL);

    if((priv->offset >= CC2520RAM_RXFIFO) && 
        (priv->offset < CC2520RAM_RXFIFO + CC2520_FIFO_SIZE)){

        ret = cc2520_read_rxfifo(priv->dev, (uint8_t*)buffer, buflen);
        if (ret < 0)
        {
            wlerr("cc2520_read_rxfifo failed: %d\n", ret);
            return 0;
        }
    }
    else{
        ret = cc2520_read_ram(priv->dev, priv->offset, buflen, (uint8_t*)buffer);
        if (ret < 0)
        {
            wlerr("cc2520_read_ram failed: %d\n", ret);
            return 0;
        }
        else{
            priv->offset += buflen;
            if(priv->offset > 0x400){
                priv->offset = 0;
            }
        }
    }


    return buflen;
}

/****************************************************************************
 * Name: cc2520_cdev_driver_seek
 *
 * Description:
 *   This function is called for lseek API.
 *
 ****************************************************************************/
static off_t cc2520_cdev_driver_seek(FAR struct file *filep, off_t offset, int whence)
{
    _info("offset=0x%x\n", offset);
    DEBUGASSERT(filep != NULL);

    FAR struct inode *inode = filep->f_inode;
    DEBUGASSERT(inode != NULL);
    FAR struct cc2520_cdev_driver_dev_s *priv = inode->i_private;
    DEBUGASSERT(priv != NULL);
    
    if(whence == SEEK_SET){
        priv->offset = offset;
        wlinfo("info: seek pos to: %d\n", offset);
    }

    return OK;
}


/****************************************************************************
 * Name: cc2520_cdev_driver_ioctl
 *
 * Description:
 *   Execute ioctl commands for the device.
 ****************************************************************************/

static int cc2520_cdev_driver_ioctl(FAR struct file *filep,
                        int cmd,
                        unsigned long arg)
{
    int ret = OK;
    uint8_t buf[128] = {0};
    uint8_t len = 0;

    _info("cmd=0x%x, arg=0x%lx\n", cmd, arg);
    DEBUGASSERT(filep != NULL);
    FAR struct inode *inode = filep->f_inode;
    DEBUGASSERT(inode != NULL);
    FAR struct cc2520_cdev_driver_dev_s *priv = inode->i_private;
    DEBUGASSERT(priv != NULL);

    switch (cmd)
    {
        /* TODO: Handle ioctl commands */
    case CC2520_CFG_CHL:
        ret = cc2520_set_channel(priv->dev, 1, arg);
        break;
    case CC2520_CFG_STROBE:
        ret = cc2520_cmd_strobe(priv->dev, arg & 0xFF);
        break;
    default:
        wlinfo("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

    return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cc2520_cdev_driver_register
 *
 * Description:
 *   Register the cc2520_cdev_driver character device as 'devpath' during NuttX startup.
 *
 ****************************************************************************/

int 
cc2520_cdev_driver_register(FAR const char *devpath, struct cc2520_radio_s *dev)
{
    _info("devpath=%s\n", devpath);
    FAR struct cc2520_cdev_driver_dev_s *priv;
    int ret;

    /* Sanity check */

    DEBUGASSERT(devpath != NULL);
    DEBUGASSERT(dev != NULL);

    /* Initialize the device structure */

    priv = (FAR struct cc2520_cdev_driver_dev_s *)
        kmm_malloc(sizeof(struct cc2520_cdev_driver_dev_s));
    if (priv == NULL)
    {
        wlerr("ERROR: Failed to allocate instance\n");
        return -ENOMEM;
    }


    nxmutex_init(&priv->lock);
    priv->offset = 0;
    priv->dev = dev;
    /* Register the character driver */
    ret = register_driver(devpath, &g_cc2520_cdev_driver_fops, 0666, priv);
    if (ret < 0)
    {
        wlerr("ERROR: Failed to register driver: %d\n", ret);
        kmm_free(priv);
    }

    return ret;
}

