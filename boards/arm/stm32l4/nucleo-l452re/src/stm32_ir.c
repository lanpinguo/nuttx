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

uint32_t flag = 0;

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

#if 1
    if (bit_msg == 1)
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_HI);
    }
    else
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_LO);  
    }

#else
    if (flag == 1)
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_HI);
      flag = 0;
    }
    else
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_LO); 
      flag = 1; 
    }
#endif

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

#define PHILIPS_MODE   1
// #define HAIR_MODE   1
// #define GREE_MODE   1

#if defined(PHILIPS_MODE)
#define RAW_DATA_LEN     52 
uint32_t test_ir_raw[RAW_DATA_LEN][2] = {
		{ 3594, 1 }, 
		{ 7143, 0 }, 
		{ 526, 1 }, 
		{ 701, 0 }, 
		{ 568, 1 }, 
		{ 1648, 0 }, 
		{ 616, 1 }, 
		{ 628, 0 }, 
		{ 601, 1 }, 
		{ 622, 0 }, 
		{ 577, 1 }, 
		{ 650, 0 }, 
		{ 548, 1 }, 
		{ 1683, 0 }, 
		{ 600, 1 }, 
		{ 588, 0 }, 
		{ 641, 1 }, 
		{ 622, 0 }, 
		{ 575, 1 }, 
		{ 652, 0 }, 
		{ 525, 1 }, 
		{ 700, 0 }, 
		{ 569, 1 }, 
		{ 658, 0 }, 
		{ 595, 1 }, 
		{ 1639, 0 }, 
		{ 599, 1 }, 
		{ 628, 0 }, 
		{ 598, 1 }, 
		{ 1630, 0 }, 
		{ 603, 1 }, 
		{ 604, 0 }, 
		{ 621, 1 }, 
		{ 628, 0 }, 
		{ 600, 1 }, 
		{ 1626, 0 }, 
		{ 604, 1 }, 
		{ 604, 0 }, 
		{ 624, 1 }, 
		{ 1626, 0 }, 
		{ 509, 1 }, 
		{ 631, 0 }, 
		{ 625, 1 }, 
		{ 602, 0 }, 
		{ 624, 1 }, 
		{ 626, 0 }, 
		{ 602, 1 }, 
		{ 625, 0 }, 
		{ 602, 1 }, 
		{ 621, 0 }, 
		{ 213, 1 }, 
		{ 50, 0 }, 
};

static void stm32_irtim_txworker(FAR void *arg)
{
  irqstate_t flags = 0;
  uint32_t period_reg;

  struct stm32_irtim_dev_s *ir_dev = (struct stm32_irtim_dev_s *)arg;

  rcinfo("txworker RC send raw data len: %d\n", RAW_DATA_LEN);




  // ir_dev->irtim[1]->ops->setisr(ir_dev->irtim[1], irtim_isr, ir_dev, 0);  
  // ir_dev->irtim[1]->ops->enableint(ir_dev->irtim[1], 0);
  ir_dev->irtim[1]->ops->setmode(ir_dev->irtim[1], STM32L4_TIM_MODE_UP);
  // ir_dev->irtim[1]->ops->setfreq(ir_dev->irtim[1], 3000);
  ir_dev->irtim[1]->ops->setclock(ir_dev->irtim[1], 2*1000*1000); // 2MHz --> t=0.5 micro-second
  ir_dev->irtim[1]->ops->setperiod(ir_dev->irtim[1], 333*2); // 333 micro-senond
  ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_LO);  
  period_reg = ir_dev->irtim[1]->ops->getperiod(ir_dev->irtim[1]);
  ir_dev->irtim[1]->ops->setcompare(ir_dev->irtim[1], 1, period_reg);  
  ir_dev->irtim[1]->ops->enable(ir_dev->irtim[1]);

  ir_dev->irtim[0]->ops->setmode(ir_dev->irtim[0], STM32L4_TIM_MODE_UP);
  ir_dev->irtim[0]->ops->setfreq(ir_dev->irtim[0], 42000);
  period_reg = ir_dev->irtim[0]->ops->getperiod(ir_dev->irtim[0]);
  ir_dev->irtim[0]->ops->setchannel(ir_dev->irtim[0], 1, STM32L4_TIM_CH_OUTPWM);  
  ir_dev->irtim[0]->ops->setcompare(ir_dev->irtim[0], 1, period_reg * 0.4);  
  ir_dev->irtim[0]->ops->enable(ir_dev->irtim[0]);

  flags = enter_critical_section();

  for(int i = 0; i < RAW_DATA_LEN; i++){
    if (test_ir_raw[i][1])
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_HI);
    }
    else
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_LO);  
    }
    up_udelay(test_ir_raw[i][0]);
  }

  leave_critical_section(flags);

  rcinfo("txworker RC send done\n");

  ir_dev->sndOpCompleteFlag = 0x01;

  /* TIM IT Disable */
  ir_dev->irtim[1]->ops->disableint(ir_dev->irtim[1], 0);
  ir_dev->sndOpRdyFlag = 0;
  ir_dev->bitsSndCnt = 0;
  ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_DISABLED);  

  /* TIM Disable */
  ir_dev->irtim[1]->ops->disable(ir_dev->irtim[1]);

}

#elif defined(MIDEA_MODE)
#define RAW_DATA_LEN     100 
uint32_t test_ir_raw[RAW_DATA_LEN][2] = {
		{ 8455, 1 }, 
		{ 4167, 0 }, 
		{ 560, 1 }, 
		{ 1537, 0 }, 
		{ 563, 1 }, 
		{ 485, 0 }, 
		{ 563, 1 }, 
		{ 485, 0 }, 
		{ 563, 1 }, 
		{ 484, 0 }, 
		{ 563, 1 }, 
		{ 486, 0 }, 
		{ 563, 1 }, 
		{ 485, 0 }, 
		{ 563, 1 }, 
		{ 486, 0 }, 
		{ 562, 1 }, 
		{ 485, 0 }, 
		{ 564, 1 }, 
		{ 485, 0 }, 
		{ 563, 1 }, 
		{ 1533, 0 }, 
		{ 564, 1 }, 
		{ 1534, 0 }, 
		{ 563, 1 }, 
		{ 1533, 0 }, 
		{ 563, 1 }, 
		{ 1533, 0 }, 
		{ 563, 1 }, 
		{ 1534, 0 }, 
		{ 563, 1 }, 
		{ 1533, 0 }, 
		{ 562, 1 }, 
		{ 1534, 0 }, 
		{ 563, 1 }, 
		{ 1533, 0 }, 
		{ 562, 1 }, 
		{ 1534, 0 }, 
		{ 563, 1 }, 
		{ 485, 0 }, 
		{ 563, 1 }, 
		{ 484, 0 }, 
		{ 563, 1 }, 
		{ 484, 0 }, 
		{ 563, 1 }, 
		{ 484, 0 }, 
		{ 553, 1 }, 
		{ 497, 0 }, 
		{ 557, 1 }, 
		{ 490, 0 }, 
		{ 546, 1 }, 
		{ 502, 0 }, 
		{ 563, 1 }, 
		{ 485, 0 }, 
		{ 563, 1 }, 
		{ 1533, 0 }, 
		{ 563, 1 }, 
		{ 1534, 0 }, 
		{ 562, 1 }, 
		{ 1535, 0 }, 
		{ 561, 1 }, 
		{ 1533, 0 }, 
		{ 564, 1 }, 
		{ 1534, 0 }, 
		{ 562, 1 }, 
		{ 1534, 0 }, 
		{ 550, 1 }, 
		{ 1546, 0 }, 
		{ 563, 1 }, 
		{ 1533, 0 }, 
		{ 563, 1 }, 
		{ 486, 0 }, 
		{ 541, 1 }, 
		{ 505, 0 }, 
		{ 563, 1 }, 
		{ 485, 0 }, 
		{ 562, 1 }, 
		{ 486, 0 }, 
		{ 562, 1 }, 
		{ 485, 0 }, 
		{ 547, 1 }, 
		{ 501, 0 }, 
		{ 562, 1 }, 
		{ 485, 0 }, 
		{ 540, 1 }, 
		{ 508, 0 }, 
		{ 540, 1 }, 
		{ 1556, 0 }, 
		{ 541, 1 }, 
		{ 1557, 0 }, 
		{ 540, 1 }, 
		{ 1557, 0 }, 
		{ 540, 1 }, 
		{ 1556, 0 }, 
		{ 562, 1 }, 
		{ 1533, 0 }, 
		{ 541, 1 }, 
		{ 1557, 0 }, 
		{ 540, 1 }, 
		{ 50, 0 }, 
};



static void stm32_irtim_txworker(FAR void *arg)
{
  irqstate_t flags = 0;
  uint32_t period_reg;

  struct stm32_irtim_dev_s *ir_dev = (struct stm32_irtim_dev_s *)arg;

  rcinfo("txworker RC send raw data len: %d\n", RAW_DATA_LEN);




  // ir_dev->irtim[1]->ops->setisr(ir_dev->irtim[1], irtim_isr, ir_dev, 0);  
  // ir_dev->irtim[1]->ops->enableint(ir_dev->irtim[1], 0);
  ir_dev->irtim[1]->ops->setmode(ir_dev->irtim[1], STM32L4_TIM_MODE_UP);
  // ir_dev->irtim[1]->ops->setfreq(ir_dev->irtim[1], 3000);
  ir_dev->irtim[1]->ops->setclock(ir_dev->irtim[1], 2*1000*1000); // 2MHz --> t=0.5 micro-second
  ir_dev->irtim[1]->ops->setperiod(ir_dev->irtim[1], 333*2); // 333 micro-senond
  ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_LO);  
  period_reg = ir_dev->irtim[1]->ops->getperiod(ir_dev->irtim[1]);
  ir_dev->irtim[1]->ops->setcompare(ir_dev->irtim[1], 1, period_reg);  
  ir_dev->irtim[1]->ops->enable(ir_dev->irtim[1]);

  ir_dev->irtim[0]->ops->setmode(ir_dev->irtim[0], STM32L4_TIM_MODE_UP);
  ir_dev->irtim[0]->ops->setfreq(ir_dev->irtim[0], 38000);
  period_reg = ir_dev->irtim[0]->ops->getperiod(ir_dev->irtim[0]);
  ir_dev->irtim[0]->ops->setchannel(ir_dev->irtim[0], 1, STM32L4_TIM_CH_OUTPWM);  
  ir_dev->irtim[0]->ops->setcompare(ir_dev->irtim[0], 1, period_reg / 2);  
  ir_dev->irtim[0]->ops->enable(ir_dev->irtim[0]);

  flags = enter_critical_section();

  for(int i = 0; i < RAW_DATA_LEN; i++){
    if (test_ir_raw[i][1])
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_HI);
    }
    else
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_LO);  
    }
    up_udelay(test_ir_raw[i][0]);
  }

  leave_critical_section(flags);

  rcinfo("txworker RC send done\n");

  ir_dev->sndOpCompleteFlag = 0x01;

  /* TIM IT Disable */
  ir_dev->irtim[1]->ops->disableint(ir_dev->irtim[1], 0);
  ir_dev->sndOpRdyFlag = 0;
  ir_dev->bitsSndCnt = 0;
  ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_DISABLED);  

  /* TIM Disable */
  ir_dev->irtim[1]->ops->disable(ir_dev->irtim[1]);

}

#elif  defined(GREE_MODE)
#define RAW_DATA_LEN     280 
uint32_t test_ir_raw[RAW_DATA_LEN][2] = {
		{ 8509, 1 }, 
		{ 4229, 0 }, 
		{ 617, 1 }, 
		{ 1535, 0 }, 
		{ 616, 1 }, 
		{ 495, 0 }, 
		{ 617, 1 }, 
		{ 520, 0 }, 
		{ 617, 1 }, 
		{ 1535, 0 }, 
		{ 616, 1 }, 
		{ 495, 0 }, 
		{ 617, 1 }, 
		{ 520, 0 }, 
		{ 617, 1 }, 
		{ 495, 0 }, 
		{ 616, 1 }, 
		{ 496, 0 }, 
		{ 617, 1 }, 
		{ 521, 0 }, 
		{ 616, 1 }, 
		{ 496, 0 }, 
		{ 616, 1 }, 
		{ 496, 0 }, 
		{ 617, 1 }, 
		{ 1535, 0 }, 
		{ 616, 1 }, 
		{ 521, 0 }, 
		{ 616, 1 }, 
		{ 496, 0 }, 
		{ 616, 1 }, 
		{ 495, 0 }, 
		{ 616, 1 }, 
		{ 521, 0 }, 
		{ 616, 1 }, 
		{ 495, 0 }, 
		{ 617, 1 }, 
		{ 496, 0 }, 
		{ 616, 1 }, 
		{ 521, 0 }, 
		{ 616, 1 }, 
		{ 496, 0 }, 
		{ 616, 1 }, 
		{ 496, 0 }, 
		{ 616, 1 }, 
		{ 1560, 0 }, 
		{ 616, 1 }, 
		{ 496, 0 }, 
		{ 616, 1 }, 
		{ 496, 0 }, 
		{ 616, 1 }, 
		{ 523, 0 }, 
		{ 614, 1 }, 
		{ 497, 0 }, 
		{ 592, 1 }, 
		{ 521, 0 }, 
		{ 591, 1 }, 
		{ 573, 0 }, 
		{ 563, 1 }, 
		{ 1586, 0 }, 
		{ 564, 1 }, 
		{ 548, 0 }, 
		{ 564, 1 }, 
		{ 1612, 0 }, 
		{ 543, 1 }, 
		{ 569, 0 }, 
		{ 566, 1 }, 
		{ 547, 0 }, 
		{ 568, 1 }, 
		{ 1583, 0 }, 
		{ 595, 1 }, 
		{ 543, 0 }, 
		{ 595, 1 }, 
		{ 18792, 0 }, 
		{ 624, 1 }, 
		{ 539, 0 }, 
		{ 598, 1 }, 
		{ 515, 0 }, 
		{ 595, 1 }, 
		{ 517, 0 }, 
		{ 594, 1 }, 
		{ 543, 0 }, 
		{ 592, 1 }, 
		{ 497, 0 }, 
		{ 613, 1 }, 
		{ 522, 0 }, 
		{ 588, 1 }, 
		{ 522, 0 }, 
		{ 615, 1 }, 
		{ 497, 0 }, 
		{ 615, 1 }, 
		{ 496, 0 }, 
		{ 614, 1 }, 
		{ 522, 0 }, 
		{ 614, 1 }, 
		{ 498, 0 }, 
		{ 593, 1 }, 
		{ 519, 0 }, 
		{ 596, 1 }, 
		{ 540, 0 }, 
		{ 613, 1 }, 
		{ 499, 0 }, 
		{ 611, 1 }, 
		{ 1539, 0 }, 
		{ 590, 1 }, 
		{ 547, 0 }, 
		{ 590, 1 }, 
		{ 523, 0 }, 
		{ 588, 1 }, 
		{ 523, 0 }, 
		{ 588, 1 }, 
		{ 549, 0 }, 
		{ 588, 1 }, 
		{ 525, 0 }, 
		{ 587, 1 }, 
		{ 525, 0 }, 
		{ 586, 1 }, 
		{ 550, 0 }, 
		{ 586, 1 }, 
		{ 527, 0 }, 
		{ 585, 1 }, 
		{ 528, 0 }, 
		{ 584, 1 }, 
		{ 553, 0 }, 
		{ 584, 1 }, 
		{ 528, 0 }, 
		{ 583, 1 }, 
		{ 530, 0 }, 
		{ 582, 1 }, 
		{ 555, 0 }, 
		{ 581, 1 }, 
		{ 1588, 0 }, 
		{ 562, 1 }, 
		{ 1589, 0 }, 
		{ 561, 1 }, 
		{ 1590, 0 }, 
		{ 561, 1 }, 
		{ 1614, 0 }, 
		{ 561, 1 }, 
		{ 37650, 0 }, 
		{ 8452, 1 }, 
		{ 4286, 0 }, 
		{ 560, 1 }, 
		{ 1590, 0 }, 
		{ 560, 1 }, 
		{ 552, 0 }, 
		{ 560, 1 }, 
		{ 577, 0 }, 
		{ 559, 1 }, 
		{ 1591, 0 }, 
		{ 560, 1 }, 
		{ 552, 0 }, 
		{ 560, 1 }, 
		{ 577, 0 }, 
		{ 559, 1 }, 
		{ 553, 0 }, 
		{ 559, 1 }, 
		{ 553, 0 }, 
		{ 559, 1 }, 
		{ 578, 0 }, 
		{ 559, 1 }, 
		{ 553, 0 }, 
		{ 559, 1 }, 
		{ 553, 0 }, 
		{ 558, 1 }, 
		{ 1592, 0 }, 
		{ 559, 1 }, 
		{ 578, 0 }, 
		{ 559, 1 }, 
		{ 553, 0 }, 
		{ 558, 1 }, 
		{ 554, 0 }, 
		{ 558, 1 }, 
		{ 579, 0 }, 
		{ 558, 1 }, 
		{ 554, 0 }, 
		{ 557, 1 }, 
		{ 555, 0 }, 
		{ 558, 1 }, 
		{ 579, 0 }, 
		{ 558, 1 }, 
		{ 554, 0 }, 
		{ 558, 1 }, 
		{ 555, 0 }, 
		{ 557, 1 }, 
		{ 1618, 0 }, 
		{ 557, 1 }, 
		{ 555, 0 }, 
		{ 557, 1 }, 
		{ 556, 0 }, 
		{ 557, 1 }, 
		{ 580, 0 }, 
		{ 556, 1 }, 
		{ 556, 0 }, 
		{ 556, 1 }, 
		{ 556, 0 }, 
		{ 556, 1 }, 
		{ 580, 0 }, 
		{ 557, 1 }, 
		{ 1594, 0 }, 
		{ 556, 1 }, 
		{ 1596, 0 }, 
		{ 555, 1 }, 
		{ 1596, 0 }, 
		{ 555, 1 }, 
		{ 581, 0 }, 
		{ 556, 1 }, 
		{ 556, 0 }, 
		{ 555, 1 }, 
		{ 1596, 0 }, 
		{ 555, 1 }, 
		{ 583, 0 }, 
		{ 554, 1 }, 
		{ 18860, 0 }, 
		{ 555, 1 }, 
		{ 582, 0 }, 
		{ 554, 1 }, 
		{ 558, 0 }, 
		{ 554, 1 }, 
		{ 558, 0 }, 
		{ 553, 1 }, 
		{ 584, 0 }, 
		{ 553, 1 }, 
		{ 559, 0 }, 
		{ 553, 1 }, 
		{ 559, 0 }, 
		{ 553, 1 }, 
		{ 584, 0 }, 
		{ 553, 1 }, 
		{ 560, 0 }, 
		{ 551, 1 }, 
		{ 560, 0 }, 
		{ 552, 1 }, 
		{ 586, 0 }, 
		{ 552, 1 }, 
		{ 560, 0 }, 
		{ 552, 1 }, 
		{ 561, 0 }, 
		{ 551, 1 }, 
		{ 586, 0 }, 
		{ 551, 1 }, 
		{ 562, 0 }, 
		{ 550, 1 }, 
		{ 561, 0 }, 
		{ 550, 1 }, 
		{ 587, 0 }, 
		{ 551, 1 }, 
		{ 561, 0 }, 
		{ 551, 1 }, 
		{ 561, 0 }, 
		{ 551, 1 }, 
		{ 586, 0 }, 
		{ 551, 1 }, 
		{ 561, 0 }, 
		{ 551, 1 }, 
		{ 561, 0 }, 
		{ 551, 1 }, 
		{ 586, 0 }, 
		{ 551, 1 }, 
		{ 561, 0 }, 
		{ 551, 1 }, 
		{ 1600, 0 }, 
		{ 550, 1 }, 
		{ 561, 0 }, 
		{ 551, 1 }, 
		{ 586, 0 }, 
		{ 551, 1 }, 
		{ 561, 0 }, 
		{ 551, 1 }, 
		{ 561, 0 }, 
		{ 551, 1 }, 
		{ 1625, 0 }, 
		{ 550, 1 }, 
		{ 1622, 0 }, 
		{ 529, 1 }, 
		{ 584, 0 }, 
		{ 528, 1 }, 
		{ 609, 0 }, 
		{ 529, 1 }, 
		{ 50, 0 }, 
};





static void stm32_irtim_txworker(FAR void *arg)
{
  irqstate_t flags = 0;
  uint32_t period_reg;

  struct stm32_irtim_dev_s *ir_dev = (struct stm32_irtim_dev_s *)arg;

  rcinfo("txworker RC send raw data len: %d\n", RAW_DATA_LEN);




  // ir_dev->irtim[1]->ops->setisr(ir_dev->irtim[1], irtim_isr, ir_dev, 0);  
  // ir_dev->irtim[1]->ops->enableint(ir_dev->irtim[1], 0);
  ir_dev->irtim[1]->ops->setmode(ir_dev->irtim[1], STM32L4_TIM_MODE_UP);
  // ir_dev->irtim[1]->ops->setfreq(ir_dev->irtim[1], 3000);
  ir_dev->irtim[1]->ops->setclock(ir_dev->irtim[1], 2*1000*1000); // 2MHz --> t=0.5 micro-second
  ir_dev->irtim[1]->ops->setperiod(ir_dev->irtim[1], 333*2); // 333 micro-senond
  ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_LO);  
  period_reg = ir_dev->irtim[1]->ops->getperiod(ir_dev->irtim[1]);
  ir_dev->irtim[1]->ops->setcompare(ir_dev->irtim[1], 1, period_reg);  
  ir_dev->irtim[1]->ops->enable(ir_dev->irtim[1]);

  ir_dev->irtim[0]->ops->setmode(ir_dev->irtim[0], STM32L4_TIM_MODE_UP);
  ir_dev->irtim[0]->ops->setfreq(ir_dev->irtim[0], 38000);
  period_reg = ir_dev->irtim[0]->ops->getperiod(ir_dev->irtim[0]);
  ir_dev->irtim[0]->ops->setchannel(ir_dev->irtim[0], 1, STM32L4_TIM_CH_OUTPWM);  
  ir_dev->irtim[0]->ops->setcompare(ir_dev->irtim[0], 1, period_reg / 2);  
  ir_dev->irtim[0]->ops->enable(ir_dev->irtim[0]);

  flags = enter_critical_section();

  for(int i = 0; i < RAW_DATA_LEN; i++){
    if (test_ir_raw[i][1])
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_HI);
    }
    else
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_LO);  
    }
    up_udelay(test_ir_raw[i][0]);
  }

  leave_critical_section(flags);

  rcinfo("txworker RC send done\n");

  ir_dev->sndOpCompleteFlag = 0x01;

  /* TIM IT Disable */
  ir_dev->irtim[1]->ops->disableint(ir_dev->irtim[1], 0);
  ir_dev->sndOpRdyFlag = 0;
  ir_dev->bitsSndCnt = 0;
  ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_DISABLED);  

  /* TIM Disable */
  ir_dev->irtim[1]->ops->disable(ir_dev->irtim[1]);

}

#elif defined(HAIR_MODE)
#define RAW_DATA_LEN     68 
uint32_t test_ir_raw[RAW_DATA_LEN][2] = {
		{ 8508, 1 }, 
		{ 4287, 0 }, 
		{ 548, 1 }, 
		{ 534, 0 }, 
		{ 574, 1 }, 
		{ 1612, 0 }, 
		{ 548, 1 }, 
		{ 527, 0 }, 
		{ 571, 1 }, 
		{ 558, 0 }, 
		{ 547, 1 }, 
		{ 536, 0 }, 
		{ 573, 1 }, 
		{ 1614, 0 }, 
		{ 544, 1 }, 
		{ 538, 0 }, 
		{ 571, 1 }, 
		{ 559, 0 }, 
		{ 547, 1 }, 
		{ 1589, 0 }, 
		{ 573, 1 }, 
		{ 558, 0 }, 
		{ 549, 1 }, 
		{ 1590, 0 }, 
		{ 572, 1 }, 
		{ 1618, 0 }, 
		{ 546, 1 }, 
		{ 1590, 0 }, 
		{ 573, 1 }, 
		{ 557, 0 }, 
		{ 546, 1 }, 
		{ 1590, 0 }, 
		{ 571, 1 }, 
		{ 1616, 0 }, 
		{ 546, 1 }, 
		{ 536, 0 }, 
		{ 572, 1 }, 
		{ 1614, 0 }, 
		{ 546, 1 }, 
		{ 1590, 0 }, 
		{ 573, 1 }, 
		{ 558, 0 }, 
		{ 547, 1 }, 
		{ 1589, 0 }, 
		{ 551, 1 }, 
		{ 1634, 0 }, 
		{ 547, 1 }, 
		{ 536, 0 }, 
		{ 571, 1 }, 
		{ 1615, 0 }, 
		{ 546, 1 }, 
		{ 1589, 0 }, 
		{ 544, 1 }, 
		{ 584, 0 }, 
		{ 516, 1 }, 
		{ 552, 0 }, 
		{ 541, 1 }, 
		{ 1640, 0 }, 
		{ 545, 1 }, 
		{ 536, 0 }, 
		{ 569, 1 }, 
		{ 562, 0 }, 
		{ 545, 1 }, 
		{ 1590, 0 }, 
		{ 570, 1 }, 
		{ 562, 0 }, 
		{ 517, 1 }, 
		{ 50, 0 }, 
};



static void stm32_irtim_txworker(FAR void *arg)
{
  irqstate_t flags = 0;
  uint32_t period_reg;

  struct stm32_irtim_dev_s *ir_dev = (struct stm32_irtim_dev_s *)arg;

  rcinfo("txworker RC send raw data len: %d\n", RAW_DATA_LEN);




  // ir_dev->irtim[1]->ops->setisr(ir_dev->irtim[1], irtim_isr, ir_dev, 0);  
  // ir_dev->irtim[1]->ops->enableint(ir_dev->irtim[1], 0);
  ir_dev->irtim[1]->ops->setmode(ir_dev->irtim[1], STM32L4_TIM_MODE_UP);
  // ir_dev->irtim[1]->ops->setfreq(ir_dev->irtim[1], 3000);
  ir_dev->irtim[1]->ops->setclock(ir_dev->irtim[1], 2*1000*1000); // 2MHz --> t=0.5 micro-second
  ir_dev->irtim[1]->ops->setperiod(ir_dev->irtim[1], 333*2); // 333 micro-senond
  ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_LO);  
  period_reg = ir_dev->irtim[1]->ops->getperiod(ir_dev->irtim[1]);
  ir_dev->irtim[1]->ops->setcompare(ir_dev->irtim[1], 1, period_reg);  
  ir_dev->irtim[1]->ops->enable(ir_dev->irtim[1]);

  ir_dev->irtim[0]->ops->setmode(ir_dev->irtim[0], STM32L4_TIM_MODE_UP);
  ir_dev->irtim[0]->ops->setfreq(ir_dev->irtim[0], 38000);
  period_reg = ir_dev->irtim[0]->ops->getperiod(ir_dev->irtim[0]);
  ir_dev->irtim[0]->ops->setchannel(ir_dev->irtim[0], 1, STM32L4_TIM_CH_OUTPWM);  
  ir_dev->irtim[0]->ops->setcompare(ir_dev->irtim[0], 1, period_reg / 2);  
  ir_dev->irtim[0]->ops->enable(ir_dev->irtim[0]);

  flags = enter_critical_section();

  for(int i = 0; i < RAW_DATA_LEN; i++){
    if (test_ir_raw[i][1])
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_HI);
    }
    else
    {
      ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_LO);  
    }
    up_udelay(test_ir_raw[i][0]);
  }

  leave_critical_section(flags);

  rcinfo("txworker RC send done\n");

  ir_dev->sndOpCompleteFlag = 0x01;

  /* TIM IT Disable */
  ir_dev->irtim[1]->ops->disableint(ir_dev->irtim[1], 0);
  ir_dev->sndOpRdyFlag = 0;
  ir_dev->bitsSndCnt = 0;
  ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_DISABLED);  

  /* TIM Disable */
  ir_dev->irtim[1]->ops->disable(ir_dev->irtim[1]);

}


#endif

static int stm32_irtim_tx_ir(FAR struct lirc_lowerhalf_s *lower,
                       unsigned *txbuf, unsigned int n)
{

  uint32_t period_reg;

  struct stm32_irtim_dev_s *ir_dev = (struct stm32_irtim_dev_s *)lower;

  if(n > STM32_L4_IR_MAX_BUF_SIZE){
    return -EINVAL;
  }

  rcinfo("Dummy RC send raw data:%d(size:%d) to device\n", *txbuf, n);

#if 0  
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
  ir_dev->irtim[0]->ops->setfreq(ir_dev->irtim[0], 43300);
  period_reg = ir_dev->irtim[0]->ops->getperiod(ir_dev->irtim[0]);
  ir_dev->irtim[0]->ops->setchannel(ir_dev->irtim[0], 1, STM32L4_TIM_CH_OUTPWM);  
  ir_dev->irtim[0]->ops->setcompare(ir_dev->irtim[0], 1, period_reg / 2);  
  ir_dev->irtim[0]->ops->enable(ir_dev->irtim[0]);

  ir_dev->irtim[1]->ops->setisr(ir_dev->irtim[1], irtim_isr, ir_dev, 0);  
  ir_dev->irtim[1]->ops->enableint(ir_dev->irtim[1], 0);
  ir_dev->irtim[1]->ops->setmode(ir_dev->irtim[1], STM32L4_TIM_MODE_UP);
  // ir_dev->irtim[1]->ops->setfreq(ir_dev->irtim[1], 3000);
  ir_dev->irtim[1]->ops->setclock(ir_dev->irtim[1], 2*1000*1000); // 2MHz --> t=0.5 micro-second
  ir_dev->irtim[1]->ops->setperiod(ir_dev->irtim[1], 333*2); // 333 micro-senond
  ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_FORCE_LO);  
  // ir_dev->irtim[1]->ops->setchannel(ir_dev->irtim[1], 1, STM32L4_TIM_CH_OUTPWM);  
  period_reg = ir_dev->irtim[1]->ops->getperiod(ir_dev->irtim[1]);
  ir_dev->irtim[1]->ops->setcompare(ir_dev->irtim[1], 1, period_reg);  
  ir_dev->irtim[1]->ops->enable(ir_dev->irtim[1]);


  for(int i = 0 ; i < n ; i++){
    rcinfo("Send IR-Code: %08x \n", ir_dev->frameFmt[i]);
  }

#else
    DEBUGASSERT(work_available(&ir_dev->work));

    return work_queue(HPWORK, &ir_dev->work,
                    stm32_irtim_txworker, (FAR void *)ir_dev, 0);

#endif
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
//   stm32l4_configgpio(GPIO_TIM15_CH1OUT);
//   stm32l4_configgpio(GPIO_TIM16_CH1OUT);


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
