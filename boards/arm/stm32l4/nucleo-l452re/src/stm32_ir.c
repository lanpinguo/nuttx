/****************************************************************************
 * boards/arm/stm32l4/nucleo-l452re/src/stm32_adc.c
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

#include <errno.h>
#include <debug.h>
#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>

#include <nuttx/random.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/analog/ioctl.h>
#include <arch/stm32l4/chip.h>

#include "arm_internal.h"
#include "hardware/stm32l4_syscfg.h"
#include "stm32l4_gpio.h"
#include "stm32l4_tim.h"
#include "nucleo-l452re.h"

#include <nuttx/rc/lirc_dev.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>


#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_L4_IR_MAX_EVENT_SIZE        512
#define STM32_IRTIM_WORK_PERIOD           SEC2TICK(1)
#define STM32_L4_IR_MAX_BUF_SIZE          8

/****************************************************************************
 * Private
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  stm32_irtim_open(FAR struct lirc_lowerhalf_s *lower);
static void stm32_irtim_close(FAR struct lirc_lowerhalf_s *lower);
static int  stm32_irtim_s_tx_mask(FAR struct lirc_lowerhalf_s *lower,
                            unsigned int mask);
static int  stm32_irtim_s_tx_carrier(FAR struct lirc_lowerhalf_s *lower,
                               unsigned int carrier);
static int  stm32_irtim_s_tx_duty_cycle(FAR struct lirc_lowerhalf_s *lower,
                                  unsigned int duty_cycle);
static int  stm32_irtim_s_rx_carrier_range(FAR struct lirc_lowerhalf_s *lower,
                                     unsigned int min, unsigned int max);
static int  stm32_irtim_tx_ir(FAR struct lirc_lowerhalf_s *lower,
                        FAR unsigned int *txbuf, unsigned int n);
static int  stm32_irtim_tx_scancode(FAR struct lirc_lowerhalf_s *lower,
                              FAR struct lirc_scancode *txbuf);
static int  stm32_irtim_s_learning_mode(FAR struct lirc_lowerhalf_s *lower,
                                  int enable);
static int  stm32_irtim_s_carrier_report(FAR struct lirc_lowerhalf_s *lower,
                                   int enable);
static int  stm32_irtim_s_timeout(FAR struct lirc_lowerhalf_s *lower,
                            unsigned int timeout);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_irtim_dev_s
{
  struct lirc_lowerhalf_s lower;
  struct work_s work;
  unsigned test_sample;
  struct stm32l4_tim_dev_s* irtim[2];
  uint32_t bitsSndCnt;
  uint32_t wordBitsSndCnt;
  uint32_t cursor;
  uint32_t globalFrameLen;
  uint32_t sndOpRdyFlag;
  uint32_t sndOpCompleteFlag;
  uint32_t frameFmt[STM32_L4_IR_MAX_BUF_SIZE];

};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct lirc_ops_s g_stm32_irtim_ops =
{
  LIRC_DRIVER_IR_RAW,           /* driver_type */
  stm32_irtim_open,                   /* open */
  stm32_irtim_close,                  /* close */
  stm32_irtim_s_tx_mask,              /* s_tx_mask */
  stm32_irtim_s_tx_carrier,           /* s_tx_carrier */
  stm32_irtim_s_tx_duty_cycle,        /* s_tx_duty_cycle */
  stm32_irtim_s_rx_carrier_range,     /* s_rx_carrier_range */
  stm32_irtim_tx_ir,                  /* tx_ir */
  stm32_irtim_tx_scancode,            /* tx_scancode */
  stm32_irtim_s_learning_mode,        /* s_learning_mode */
  stm32_irtim_s_carrier_report,       /* s_carrier_report */
  stm32_irtim_s_timeout               /* s_timeout */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/


static int irtim_isr(int irq, FAR void *context, FAR void *arg)
{
  struct stm32_irtim_dev_s *ir_dev = arg;
  uint32_t bit_msg = 0;
  uint32_t frameFmt =0;

  ir_dev->irtim[1]->ops->ackint(ir_dev->irtim[1], 0);

  /* dummy 1 tick */
  if(ir_dev->sndOpRdyFlag == 0){

    ir_dev->sndOpRdyFlag = 1;
    return 0;
  }

  if ((ir_dev->sndOpRdyFlag == 1) && (ir_dev->bitsSndCnt <= (ir_dev->globalFrameLen)))
  {
    ir_dev->sndOpCompleteFlag = 0x00;

    frameFmt = ir_dev->frameFmt[ir_dev->cursor];
    bit_msg = (frameFmt >> ir_dev->wordBitsSndCnt) & 1;


    if (bit_msg == 1)
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_HI);
    }
    else
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_LO);  
    }

    ir_dev->bitsSndCnt++;
    ir_dev->wordBitsSndCnt++;
    if(ir_dev->wordBitsSndCnt >= 32){
      ir_dev->cursor++;
      ir_dev->wordBitsSndCnt = 0;
    }
  }
  else
  {
    ir_dev->sndOpCompleteFlag = 0x01;

    /* TIM IT Disable */
    ir_dev->irtim[1]->ops->disableint(ir_dev->irtim[1], 0);
    ir_dev->sndOpRdyFlag = 0;
    ir_dev->bitsSndCnt = 0;
    ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_DISABLED);  

    /* TIM Disable */
    ir_dev->irtim[1]->ops->disable(ir_dev->irtim[1]);
  }

  return 0;
}


static void stm32_irtim_worker(FAR void *arg)
{
  struct stm32_irtim_dev_s *dummy = arg;
  unsigned sample = dummy->test_sample;

  if (sample % 2 == 0)
    {
      sample = LIRC_SPACE(sample);
    }
  else
    {
      sample = LIRC_PULSE(sample);
    }

  lirc_sample_event(&dummy->lower, sample);

  rcinfo("Dummy RC read Raw Data from device:%d\n", sample);
  dummy->test_sample++;

  work_queue(LPWORK, &dummy->work, stm32_irtim_worker, dummy,
             STM32_IRTIM_WORK_PERIOD);
}

static int stm32_irtim_open(FAR struct lirc_lowerhalf_s *lower)
{
  struct stm32_irtim_dev_s *dummy = (struct stm32_irtim_dev_s *)lower;

  rcinfo("Called %s\n", __func__);

  dummy->test_sample = 0;
  return OK;
}

static void stm32_irtim_close(FAR struct lirc_lowerhalf_s *lower)
{
  struct stm32_irtim_dev_s *dummy = (struct stm32_irtim_dev_s *)lower;

  rcinfo("Called %s\n", __func__);

  work_cancel(LPWORK, &dummy->work);
}

static int stm32_irtim_s_tx_mask(FAR struct lirc_lowerhalf_s *lower,
                           unsigned int mask)
{
  rcinfo("Called %s, mask:%u\n", __func__, mask);
  return 0;
}

static int stm32_irtim_s_tx_carrier(FAR struct lirc_lowerhalf_s *lower,
                              unsigned int carrier)
{
  rcinfo("Called %s, carrier:%u\n", __func__, carrier);
  return 0;
}

static int stm32_irtim_s_tx_duty_cycle(FAR struct lirc_lowerhalf_s *lower,
                                 unsigned int duty_cycle)
{
  rcinfo("Called %s, duty_cycle:%u\n", __func__, duty_cycle);
  return 0;
}

static int stm32_irtim_s_rx_carrier_range(FAR struct lirc_lowerhalf_s *lower,
                                    unsigned int min, unsigned int max)
{
  rcinfo("Called %s, min:%u, max:%u\n", __func__, min, max);
  return 0;
}

static int stm32_irtim_tx_ir(FAR struct lirc_lowerhalf_s *lower,
                       unsigned *txbuf, unsigned int n)
{

  uint32_t period_reg;

  struct stm32_irtim_dev_s *ir_dev = (struct stm32_irtim_dev_s *)lower;

  if(n > STM32_L4_IR_MAX_BUF_SIZE){
    return -EINVAL;
  }

  rcinfo("Dummy RC send raw data:%d(size:%d) to device\n", *txbuf, n);

  ir_dev->bitsSndCnt = 0;
  ir_dev->globalFrameLen = n * 32;
  ir_dev->sndOpRdyFlag = 0;
  ir_dev->sndOpCompleteFlag = 0;
  ir_dev->cursor = 0;
  ir_dev->wordBitsSndCnt = 0;

  for(int i = 0; i < n && i < STM32_L4_IR_MAX_BUF_SIZE; i++){
    ir_dev->frameFmt[i] = txbuf[i];
  }

  ir_dev->irtim[0]->ops->setmode(ir_dev->irtim[0], STM32L4_TIM_MODE_UP);
  ir_dev->irtim[0]->ops->setfreq(ir_dev->irtim[0], 38000);
  period_reg = ir_dev->irtim[0]->ops->getperiod(ir_dev->irtim[0]);
  ir_dev->irtim[0]->ops->setchannel(ir_dev->irtim[0], 1, STM32L4_TIM_CH_OUTPWM);  
  ir_dev->irtim[0]->ops->setcompare(ir_dev->irtim[0], 1, period_reg / 3);  
  ir_dev->irtim[0]->ops->enable(ir_dev->irtim[0]);

  ir_dev->irtim[1]->ops->setisr(ir_dev->irtim[1], irtim_isr, ir_dev, 0);  
  ir_dev->irtim[1]->ops->enableint(ir_dev->irtim[1], 0);
  ir_dev->irtim[1]->ops->setmode(ir_dev->irtim[1], STM32L4_TIM_MODE_UP);
  ir_dev->irtim[1]->ops->setfreq(ir_dev->irtim[1], 1125);
  ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_LO);  
  // ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_OUTPWM);  
  period_reg = ir_dev->irtim[1]->ops->getperiod(ir_dev->irtim[1]);
  ir_dev->irtim[1]->ops->setcompare(ir_dev->irtim[1], 1, period_reg);  
  ir_dev->irtim[1]->ops->enable(ir_dev->irtim[1]);


  for(int i = 0 ; i < n ; i++){
    rcinfo("Send IR-Code: %08x \n", ir_dev->frameFmt[i]);
  }

  return n;
}

static int stm32_irtim_tx_scancode(FAR struct lirc_lowerhalf_s *lower,
                             FAR struct lirc_scancode *txbuf)
{
  rcinfo("Dummy RC send scancode data:%" PRIu64 " to device\n",
         txbuf->scancode);
  return 0;
}

static int stm32_irtim_s_learning_mode(FAR struct lirc_lowerhalf_s *lower,
                                 int enable)
{
  rcinfo("Called %s, enable:%d\n", __func__, enable);
  return 0;
}

static int stm32_irtim_s_carrier_report(FAR struct lirc_lowerhalf_s *lower,
                                  int enable)
{
  rcinfo("Called %s, enable:%d\n", __func__, enable);
  return 0;
}

static int stm32_irtim_s_timeout(FAR struct lirc_lowerhalf_s *lower,
                           unsigned int timeout)
{
  rcinfo("Called %s, timeout:%u\n", __func__, timeout);
  return 0;
}




/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_irtim_irtim_initialize(int devno)
{
  uint32_t reg;
  struct stm32_irtim_dev_s *ir_dev;

  ir_dev = kmm_malloc(sizeof(*ir_dev));
  if (!ir_dev)
    {
      rcerr("failed to alloc memory for ir_dev\n");
      return -ENOMEM;
    }


#if 1


  /* activate the high current sink capability */

  reg  = getreg32(STM32L4_SYSCFG_CFGR1);
  reg |= SYSCFG_CFGR1_I2C_PB9_FMP;
  putreg32(reg, STM32L4_SYSCFG_CFGR1);

  stm32l4_configgpio(GPIO_IR_OUT);
  stm32l4_configgpio(GPIO_TIM15_CH1OUT);
  stm32l4_configgpio(GPIO_TIM16_CH1OUT);


  ir_dev->irtim[0] = stm32l4_tim_init(15);
  ir_dev->irtim[1] = stm32l4_tim_init(16);


#endif



  ir_dev->lower.ops = &g_stm32_irtim_ops;
  ir_dev->lower.buffer_bytes = STM32_L4_IR_MAX_EVENT_SIZE * sizeof(unsigned);

  ir_dev->work.worker = NULL;
  ir_dev->test_sample = 0;

  return lirc_register(&ir_dev->lower, devno);
}

/****************************************************************************
 * Name: stm32l4_ir_setup
 ****************************************************************************/

int stm32l4_ir_setup(void)
{
  int ret;
  static bool initialized = false;
  int devno = 0;


  if (!initialized)
    {


      ret = stm32_irtim_irtim_initialize(devno);
      if (ret < 0)
        {
          snerr("ERROR: stm32_irtim_initialize failed\n");
          // goto drv_err;
        }


      initialized = true;
    }

  return OK;
}
