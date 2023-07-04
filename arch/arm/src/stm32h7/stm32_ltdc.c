/****************************************************************************
 * arch/arm/src/STM32H7/stm32_ltdc.c
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

/* References:
 *   STM32H7x7xx Technical Reference Manual and Data Sheet
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <sys/param.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/video/fb.h>
#include <nuttx/lcd/lcd.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/stm32_ltdc.h"
#include "hardware/stm32_dma2d.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_ltdc.h"
#include "stm32_dma2d.h"
#include "stm32_dsi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register definition ******************************************************/

#ifndef BOARD_LTDC_WIDTH
#  error BOARD_LTDC_WIDTH must be defined in the board.h header file
#endif

#ifndef BOARD_LTDC_HEIGHT
#  error BOARD_LTDC_HEIGHT must be defined in the board.h header file
#endif

#define STM32_LTDC_HEIGHT           BOARD_LTDC_HEIGHT
#define STM32_LTDC_WIDTH            BOARD_LTDC_WIDTH

/* Configure LTDC register */

/* Configure LTDC register */

#define STM32_LTDC_ACCUMULATED_HBP  (BOARD_LTDC_HSYNC + BOARD_LTDC_HBP)
#define STM32_LTDC_ACCUMULATED_VBP  (BOARD_LTDC_VSYNC + BOARD_LTDC_VBP)


#define STM32_LTDC_ACCUM_ACT_W      (BOARD_LTDC_HSYNC + BOARD_LTDC_HBP + BOARD_LTDC_WIDTH)
#define STM32_LTDC_ACCUM_ACT_H      (BOARD_LTDC_VSYNC + BOARD_LTDC_VBP + BOARD_LTDC_HEIGHT)


#define STM32_LTDC_TOTAL_W          (BOARD_LTDC_HSYNC + BOARD_LTDC_HBP + BOARD_LTDC_WIDTH + BOARD_LTDC_HFP)
#define STM32_LTDC_TOTAL_H          (BOARD_LTDC_VSYNC + BOARD_LTDC_VBP + BOARD_LTDC_HEIGHT + BOARD_LTDC_VFP)

#define STM32_LTDC_BKG_COLOR_R      0
#define STM32_LTDC_BKG_COLOR_G      0
#define STM32_LTDC_BKG_COLOR_B      0


/* Hardware acceleration support */

/* Acceleration support for LTDC overlays */

#ifdef CONFIG_STM32H7_LTDC_L1_CHROMAKEYEN
#  define STM32_LTDC_L1_CHROMAEN    true
#  define STM32_LTDC_L1_CHROMAKEY   CONFIG_STM32H7_LTDC_L1_CHROMAKEY
#  define LTDC_LTDC_ACCL_L1         FB_ACCL_TRANSP | FB_ACCL_CHROMA
#else
#  define STM32_LTDC_L1_CHROMAEN    false
#  define STM32_LTDC_L1_CHROMAKEY   0
#  define LTDC_LTDC_ACCL_L1         FB_ACCL_TRANSP
#endif

#ifdef CONFIG_STM32H7_LTDC_L2_CHROMAKEYEN
#  define STM32_LTDC_L2_CHROMAEN    true
#  define STM32_LTDC_L2_CHROMAKEY   CONFIG_STM32H7_LTDC_L2_CHROMAKEY
#  define LTDC_LTDC_ACCL_L2         FB_ACCL_TRANSP | FB_ACCL_CHROMA
#else
#  define STM32_LTDC_L2_CHROMAEN    false
#  define STM32_LTDC_L2_CHROMAKEY   0
#  define LTDC_LTDC_ACCL_L2         FB_ACCL_TRANSP
#endif

#ifdef CONFIG_STM32H7_DMA2D
#  ifdef CONFIG_FB_OVERLAY_BLIT
#    ifdef CONFIG_STM32H7_FB_CMAP
#      define LTDC_BLIT_ACCL        FB_ACCL_BLIT
#    else
#      define LTDC_BLIT_ACCL        FB_ACCL_BLIT | FB_ACCL_BLEND
#    endif /* CONFIG_STM32H7_FB_CMAP */
#  else
#    define LTDC_BLIT_ACCL          0
#  endif /* CONFIG_FB_OVERLAY_BLIT */

#  ifdef CONFIG_STM32H7_FB_CMAP
#    define LTDC_DMA2D_ACCL         LTDC_BLIT_ACCL
#  else
#    define LTDC_DMA2D_ACCL         FB_ACCL_COLOR | LTDC_BLIT_ACCL
#  endif /* CONFIG_STM32H7_FB_CMAP */
#else
#  define LTDC_DMA2D_ACCL           0
#endif /* CONFIG_STM32H7_DMA2D */

#define LTDC_L1_ACCL                LTDC_LTDC_ACCL_L1 | LTDC_DMA2D_ACCL
#ifdef CONFIG_STM32H7_LTDC_L2
#  define LTDC_L2_ACCL              LTDC_LTDC_ACCL_L2 | LTDC_DMA2D_ACCL
#endif

/* Acceleration support for DMA2D overlays */

#ifdef CONFIG_STM32H7_FB_CMAP
#  ifdef CONFIG_FB_OVERLAY_BLIT
#    define DMA2D_ACCL              FB_ACCL_BLIT | FB_ACCL_AREA
#  else
#    define DMA2D_ACCL              FB_ACCL_AREA
#  endif
#else
#  ifdef CONFIG_FB_OVERLAY_BLIT
#    define DMA2D_ACCL              FB_ACCL_AREA  | \
                                    FB_ACCL_TRANSP | \
                                    FB_ACCL_COLOR | \
                                    FB_ACCL_BLIT | \
                                    FB_ACCL_BLEND
#  else
#    define DMA2D_ACCL              FB_ACCL_AREA  | \
                                    FB_ACCL_TRANSP | \
                                    FB_ACCL_COLOR
#  endif
#endif



/* Color/video formats */

/* Layer 1 format */

#if defined(CONFIG_STM32H7_LTDC_L1_L8)
#  define STM32_LTDC_L1_BPP         8
#  define STM32_LTDC_L1_COLOR_FMT   FB_FMT_RGB8
#  define STM32_LTDC_L1PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_L8)
#  define STM32_LTDC_L1_DMA2D_PF    DMA2D_PF_L8
#  define STM32_LTDC_L1CMAP
#elif defined(CONFIG_STM32H7_LTDC_L1_RGB565)
#  define STM32_LTDC_L1_BPP         16
#  define STM32_LTDC_L1_COLOR_FMT   FB_FMT_RGB16_565
#  define STM32_LTDC_L1PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_RGB565)
#  define STM32_LTDC_L1_DMA2D_PF    DMA2D_PF_RGB565
#elif defined(CONFIG_STM32H7_LTDC_L1_RGB888)
#  define STM32_LTDC_L1_BPP         24
#  define STM32_LTDC_L1_COLOR_FMT   FB_FMT_RGB24
#  define STM32_LTDC_L1PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_RGB888)
#  define STM32_LTDC_L1_DMA2D_PF    DMA2D_PF_RGB888
#elif defined(CONFIG_STM32H7_LTDC_L1_ARGB8888)
#  define STM32_LTDC_L1_BPP         32
#  define STM32_LTDC_L1_COLOR_FMT   FB_FMT_RGB32
#  define STM32_LTDC_L1PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_ARGB8888)
#  define STM32_LTDC_L1_DMA2D_PF    DMA2D_PF_ARGB8888
#else
#  error "LTDC pixel format not supported"
#endif

/* Layer 2 format */

#ifdef CONFIG_STM32H7_LTDC_L2
#  if defined(CONFIG_STM32H7_LTDC_L2_L8)
#   define STM32_LTDC_L2_BPP         8
#   define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB8
#   define STM32_LTDC_L2PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_L8)
#   define STM32_LTDC_L2_DMA2D_PF    DMA2D_PF_L8
#   define STM32_LTDC_L2CMAP
#  elif defined(CONFIG_STM32H7_LTDC_L2_RGB565)
#   define STM32_LTDC_L2_BPP         16
#   define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB16_565
#   define STM32_LTDC_L2PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_RGB565)
#   define STM32_LTDC_L2_DMA2D_PF    DMA2D_PF_RGB565
#  elif defined(CONFIG_STM32H7_LTDC_L2_RGB888)
#   define STM32_LTDC_L2_BPP         24
#   define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB24
#   define STM32_LTDC_L2PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_RGB888)
#   define STM32_LTDC_L2_DMA2D_PF    DMA2D_PF_RGB888
#  elif defined(CONFIG_STM32H7_LTDC_L2_ARGB8888)
#   define STM32_LTDC_L2_BPP         32
#   define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB32
#   define STM32_LTDC_L2PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_ARGB8888)
#   define STM32_LTDC_L2_DMA2D_PF    DMA2D_PF_ARGB8888
#  else
#   error "LTDC pixel format not supported"
#  endif
#endif /* CONFIG_STM32H7_LTDC_L2 */


/* Framebuffer sizes in bytes */

#if STM32_LTDC_L1_BPP == 8
#  define STM32_LTDC_L1_STRIDE      (STM32_LTDC_WIDTH)
#elif STM32_LTDC_L1_BPP == 16
#  define STM32_LTDC_L1_STRIDE      ((STM32_LTDC_WIDTH * 16 + 7) / 8)
#elif STM32_LTDC_L1_BPP == 24
#  define STM32_LTDC_L1_STRIDE      ((STM32_LTDC_WIDTH * 24 + 7) / 8)
#elif STM32_LTDC_L1_BPP == 32
#  define STM32_LTDC_L1_STRIDE      ((STM32_LTDC_WIDTH * 32 + 7) / 8)
#else
#  error Undefined or unrecognized base resolution
#endif


#define STM32_LTDC_L1_FBSIZE        (STM32_LTDC_L1_STRIDE * STM32_LTDC_HEIGHT)

#ifdef CONFIG_STM32H7_LTDC_L2
#  ifndef CONFIG_STM32H7_LTDC_L2_WIDTH
#    define CONFIG_STM32H7_LTDC_L2_WIDTH STM32_LTDC_WIDTH
#  endif

#  if CONFIG_STM32H7_LTDC_L2_WIDTH > STM32_LTDC_WIDTH
#    error Width of Layer 2 exceeds the width of the display
#  endif

#  ifndef CONFIG_STM32H7_LTDC_L2_HEIGHT
#    define CONFIG_STM32H7_LTDC_L2_HEIGHT STM32_LTDC_HEIGHT
#  endif

#  if CONFIG_STM32H7_LTDC_L2_HEIGHT > STM32_LTDC_HEIGHT
#    error Height of Layer 2 exceeds the height of the display
#  endif

#  if STM32_LTDC_L2_BPP == 8
#    define STM32_LTDC_L2_STRIDE    (CONFIG_STM32H7_LTDC_L2_WIDTH)
#  elif STM32_LTDC_L2_BPP == 16
#    define STM32_LTDC_L2_STRIDE    ((CONFIG_STM32H7_LTDC_L2_WIDTH * 16 + 7) / 8)
#  elif STM32_LTDC_L2_BPP == 24
#    define STM32_LTDC_L2_STRIDE    ((CONFIG_STM32H7_LTDC_L2_WIDTH * 24 + 7) / 8)
#  elif STM32_LTDC_L2_BPP == 32
#    define STM32_LTDC_L2_STRIDE    ((CONFIG_STM32H7_LTDC_L2_WIDTH * 32 + 7) / 8)
#  else
#    error Undefined or unrecognized base resolution
#  endif

#  define STM32_LTDC_L2_FBSIZE      (STM32_LTDC_L2_STRIDE * \
                                     CONFIG_STM32H7_LTDC_L2_HEIGHT)

#else
#  define STM32_LTDC_L2_FBSIZE (0)
#  define STM32_LTDC_L2_BPP 0
#endif

/* Total memory used for framebuffers */

#define STM32_LTDC_TOTAL_FBSIZE     (STM32_LTDC_L1_FBSIZE + \
                                     STM32_LTDC_L2_FBSIZE)

/* Debug option */

#ifdef CONFIG_STM32H7_LTDC_REGDEBUG
#  define regerr       lcderr
#  define reginfo      lcdinfo
#else
#  define regerr(x...)
#  define reginfo(x...)
#endif


/* Preallocated LTDC framebuffers */

/* Position the framebuffer memory in the center of the memory set aside.
 * We will use any skirts before or after the framebuffer memory as a guard
 * against wild framebuffer writes.
 */

#define STM32_LTDC_BUFFER_SIZE      CONFIG_STM32H7_LTDC_FB_SIZE
#define STM32_LTDC_BUFFER_FREE      (STM32_LTDC_BUFFER_SIZE - \
                                    STM32_LTDC_TOTAL_FBSIZE)
#define STM32_LTDC_BUFFER_START     (CONFIG_STM32H7_LTDC_FB_BASE)

#if STM32_LTDC_BUFFER_FREE < 0
#  error "STM32_LTDC_BUFFER_SIZE not large enough for frame buffers"
#endif

/* Layer frame buffer */

#define STM32_LTDC_BUFFER_L1        STM32_LTDC_BUFFER_START
#define STM32_LTDC_ENDBUF_L1        (STM32_LTDC_BUFFER_L1 + \
                                     STM32_LTDC_L1_FBSIZE)

#ifdef CONFIG_STM32H7_LTDC_L2
#  define STM32_LTDC_BUFFER_L2      STM32_LTDC_ENDBUF_L1
#  define STM32_LTDC_ENDBUF_L2      (STM32_LTDC_BUFFER_L2 + \
                                     STM32_LTDC_L2_FBSIZE)
#else
#  define STM32_LTDC_ENDBUF_L2      STM32_LTDC_ENDBUF_L1
#endif


/* LTDC layer */

#define STM32_IRQ_LTDCINT     (STM32_IRQ_FIRST + 88)  /*  88: LCD-TFT global interrupt */
#define STM32_IRQ_LTDCERRINT  (STM32_IRQ_FIRST + 89)  /*  89: LCD-TFT global Error interrupt */
#define STM32_IRQ_DMA2D       (STM32_IRQ_FIRST + 90)  /*  90: DMA2D global interrupt */

#define STM32_IRQ_DSI         (STM32_IRQ_FIRST + 107)  /*  107: DMA2D global interrupt */


#ifdef CONFIG_STM32H7_LTDC_L2
#  define LTDC_NLAYERS 2
#else
#  define LTDC_NLAYERS 1
#endif

/* DMA2D layer */

#ifdef CONFIG_STM32H7_DMA2D
#  define DMA2D_NLAYERS             CONFIG_STM32H7_DMA2D_NLAYERS
#  if DMA2D_NLAYERS < 1
#    error "DMA2D must at least support 1 overlay"
#  endif

#define STM32_DMA2D_WIDTH           CONFIG_STM32H7_DMA2D_LAYER_PPLINE

#  if defined(CONFIG_STM32H7_DMA2D_L8)
#    define STM32_DMA2D_STRIDE      (STM32_DMA2D_WIDTH)
#    define STM32_DMA2D_BPP         8
#    define STM32_DMA2D_COLOR_FMT   DMA2D_PF_L8
#  elif defined(CONFIG_STM32H7_DMA2D_RGB565)
#    define STM32_DMA2D_STRIDE      ((STM32_DMA2D_WIDTH * 16 + 7) / 8)
#    define STM32_DMA2D_BPP         16
#    define STM32_DMA2D_COLOR_FMT   DMA2D_PF_RGB565
#  elif defined(CONFIG_STM32H7_DMA2D_RGB888)
#    define STM32_DMA2D_STRIDE      ((STM32_DMA2D_WIDTH * 24 + 7) / 8)
#    define STM32_DMA2D_BPP         24
#    define STM32_DMA2D_COLOR_FMT   DMA2D_PF_RGB888
#  elif defined(CONFIG_STM32H7_DMA2D_ARGB8888)
#    define STM32_DMA2D_STRIDE      ((STM32_DMA2D_WIDTH * 32 + 7) / 8)
#    define STM32_DMA2D_BPP         32
#    define STM32_DMA2D_COLOR_FMT   DMA2D_PF_ARGB8888
#  else
#    error "DMA2D pixel format not supported"
#  endif

#  ifdef CONFIG_STM32H7_DMA2D_LAYER_SHARED
#    define STM32_DMA2D_FBSIZE      CONFIG_STM32H7_DMA2D_FB_SIZE
#    define STM32_DMA2D_LAYER_SIZE  0
#  else
#    define STM32_DMA2D_FBSIZE      CONFIG_STM32H7_DMA2D_FB_SIZE / DMA2D_NLAYERS
#    define STM32_DMA2D_LAYER_SIZE  STM32_DMA2D_FBSIZE
#    if STM32_DMA2D_FBSIZE * DMA2D_NLAYERS > CONFIG_STM32H7_DMA2D_FB_SIZE
#      error "DMA2D framebuffer size to small for configured number of overlays"
#    endif
#  endif /* CONFIG_STM32H7_DMA2D_LAYER_SHARED */

#  define STM32_DMA2D_HEIGHT         STM32_DMA2D_FBSIZE / STM32_DMA2D_STRIDE

#  define STM32_DMA2D_BUFFER_START   CONFIG_STM32H7_DMA2D_FB_BASE
#else
#  define DMA2D_NLAYERS              0
#endif /* CONFIG_STM32H7_DMA2D */

#define LTDC_NOVERLAYS              LTDC_NLAYERS + DMA2D_NLAYERS


/* LTDC_TWCR register */

#define STM32_LTDC_TWCR_TOTALH      LTDC_TWCR_TOTALH(BOARD_LTDC_VSYNC + \
                                    BOARD_LTDC_VBP + \
                                    STM32_LTDC_HEIGHT + BOARD_LTDC_VFP - 1)
#define STM32_LTDC_TWCR_TOTALW      LTDC_TWCR_TOTALW(BOARD_LTDC_HSYNC + \
                                    BOARD_LTDC_HBP + \
                                    STM32_LTDC_WIDTH + BOARD_LTDC_HFP - 1)

/* LIPCR register */

#define STM32_LTDC_LIPCR_LIPOS      LTDC_LIPCR_LIPOS(STM32_LTDC_TWCR_TOTALW)

#define STM32_LTDC_DEV_ENABLE       0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This enumeration names each layer supported by the hardware */

enum stm32_layer_e
{
  LTDC_LAYER_L1 = 0,       /* LCD Layer 1 */
  LTDC_LAYER_L2,           /* LCD Layer 2 */
};

/* LTDC General layer information */

struct stm32_ltdc_s
{
  int layerno;                                /* layer number */

#ifdef CONFIG_FB_OVERLAY
  struct   fb_overlayinfo_s oinfo;            /* Overlay info */
#endif

#ifdef CONFIG_STM32H7_DMA2D
  struct stm32_dma2d_overlay_s dma2dinfo;     /* Overlay info for DMA2D */
#endif

  mutex_t *lock;                                /* Layer exclusive access */
};

/* This structure provides the overall state of the LTDC layer */

struct stm32_ltdcdev_s
{
#if STM32_LTDC_DEV_ENABLE
  /* The instance of the LCD driver */
  struct lcd_dev_s      dev;   
#endif

  /* Framebuffer interface */

  struct fb_vtable_s    vtable;

  /* Framebuffer video information */

  struct fb_videoinfo_s vinfo;

  /* Framebuffer plane information */

  struct fb_planeinfo_s pinfo;

  /* Cmap information */

#ifdef CONFIG_STM32H7_FB_CMAP
  struct fb_cmap_s cmap;
#endif

  /* Layer information */

  struct stm32_ltdc_s layer[LTDC_NOVERLAYS];

#ifdef CONFIG_STM32H7_DMA2D
  /* Interface to the dma2d controller */

  struct dma2d_layer_s *dma2d;
#endif
};

/* Interrupt handling */

struct stm32_interrupt_s
{
  int   irq;        /* irq number */
  int   dsi_irq;    /* dsi irq number */
  int   error;        /* Interrupt error */
  sem_t *sem;       /* Semaphore for waiting for irq */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_ltdcirq(int irq, void *context, void *arg);
static int stm32_ltdc_waitforirq(void);
static int stm32_ltdc_reload(uint8_t value, bool waitvblank);

/* Framebuffer interface */

static int stm32_getvideoinfo(struct fb_vtable_s *vtable,
                              struct fb_videoinfo_s *vinfo);
static int stm32_getplaneinfo(struct fb_vtable_s *vtable,
                              int planeno,
                              struct fb_planeinfo_s *pinfo);
static int stm32_updatearea(FAR struct fb_vtable_s *vtable,
                            FAR const struct fb_area_s *area);

static int stm32_ltdc_putrun(FAR struct lcd_dev_s *dev,
                          fb_coord_t row, fb_coord_t col,
                          FAR const uint8_t *buffer,
                          size_t npixels);

static int stm32_ltdc_getrun(FAR struct lcd_dev_s *dev,
                          fb_coord_t row, fb_coord_t col,
                          FAR uint8_t *buffer,
                          size_t npixels);

/* The following is provided only if the video hardware supports RGB color
 * mapping
 */

#ifdef CONFIG_STM32H7_FB_CMAP
static int stm32_getcmap(struct fb_vtable_s *vtable,
                         struct fb_cmap_s *cmap);
static int stm32_putcmap(struct fb_vtable_s *vtable,
                         const struct fb_cmap_s *cmap);
#endif

/* The following is provided only if the video hardware signals vertical
 * synchronisation
 */

#ifdef CONFIG_FB_SYNC
static int stm32_waitforvsync(struct fb_vtable_s *vtable);
#endif

/* The following is provided only if the video hardware supports overlays */

#ifdef CONFIG_FB_OVERLAY
static int stm32_getoverlayinfo(struct fb_vtable_s *vtable,
                                int overlayno,
                                struct fb_overlayinfo_s *oinfo);
static int stm32_settransp(struct fb_vtable_s *vtable,
                           const struct fb_overlayinfo_s *oinfo);
static int stm32_setchromakey(struct fb_vtable_s *vtable,
                              const struct fb_overlayinfo_s *oinfo);
static int stm32_setcolor(struct fb_vtable_s *vtable,
                          const struct fb_overlayinfo_s *oinfo);
static int stm32_setblank(struct fb_vtable_s *vtable,
                          const struct fb_overlayinfo_s *oinfo);
static int stm32_setarea(struct fb_vtable_s *vtable,
                         const struct fb_overlayinfo_s *oinfo);

/* The following is provided only if the video hardware supports blit and
 * blend operation
 */

#  ifdef CONFIG_FB_OVERLAY_BLIT
static int stm32_blit(struct fb_vtable_s *vtable,
                      const struct fb_overlayblit_s *blit);
static int stm32_blend(struct fb_vtable_s *vtable,
                       const struct fb_overlayblend_s *blend);
#  endif /* CONFIG_FB_OVERLAY_BLIT */
#endif /* CONFIG_FB_OVERLAY */


static int stm32_ltdc_getvideoinfo(FAR struct lcd_dev_s *dev,
                                 FAR struct fb_videoinfo_s *vinfo);

static int stm32_ltdc_getplaneinfo(FAR struct lcd_dev_s *dev,
                                unsigned int planeno,
                                FAR struct lcd_planeinfo_s *pinfo);



/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_ltdc_initialized;

/* The LTDC mutex that enforces mutually exclusive access */

static mutex_t g_lock = NXMUTEX_INITIALIZER;

/* The semaphore for interrupt handling */

static sem_t g_semirq = SEM_INITIALIZER(0);

/* This structure provides irq handling */

static struct stm32_interrupt_s g_interrupt =
{
  .irq     = STM32_IRQ_LTDCINT,
  .dsi_irq = STM32_IRQ_DSI,
  .error   = OK,
  .sem     = &g_semirq
};

/* This structure provides the internal interface */

static struct stm32_ltdcdev_s g_vtable =
{
#if STM32_LTDC_DEV_ENABLE
  .dev = {
      .getvideoinfo    = stm32_ltdc_getvideoinfo,
      .getplaneinfo    = stm32_ltdc_getplaneinfo
  },
#endif

  .vtable =
    {
      .getvideoinfo    = stm32_getvideoinfo,
      .getplaneinfo    = stm32_getplaneinfo
#if defined(CONFIG_FB_UPDATE) || defined(CONFIG_NX_UPDATE)
      ,
      .updatearea     = stm32_updatearea
#endif      
#ifdef CONFIG_FB_SYNC
      ,
      .waitforvsync    = stm32_waitforvsync
#endif

#ifdef CONFIG_STM32H7_FB_CMAP
      ,
      .getcmap         = stm32_getcmap,
      .putcmap         = stm32_putcmap
#endif

#ifdef CONFIG_FB_OVERLAY
      ,
      .getoverlayinfo  = stm32_getoverlayinfo,
      .settransp       = stm32_settransp,
      .setchromakey    = stm32_setchromakey,
      .setcolor        = stm32_setcolor,
      .setblank        = stm32_setblank,
      .setarea         = stm32_setarea
#  ifdef CONFIG_FB_OVERLAY_BLIT
      ,
      .blit            = stm32_blit,
      .blend           = stm32_blend
#  endif
#endif /* CONFIG_FB_OVERLAY */
  },
#ifdef CONFIG_STM32H7_LTDC_L2
  .pinfo =
    {
      .fbmem           = (uint8_t *)STM32_LTDC_BUFFER_L2,
      .fblen           = STM32_LTDC_L2_FBSIZE,
      .stride          = STM32_LTDC_L2_STRIDE,
      .display         = 0,
      .bpp             = STM32_LTDC_L2_BPP
    },
  .vinfo =
    {
      .fmt             = STM32_LTDC_L2_COLOR_FMT,
      .xres            = STM32_LTDC_WIDTH,
      .yres            = STM32_LTDC_HEIGHT,
      .nplanes         = 1,
#  ifdef CONFIG_FB_OVERLAY
      .noverlays       = LTDC_NOVERLAYS
#  endif
    }
#else
  .pinfo =
    {
      .fbmem           = (uint8_t *)STM32_LTDC_BUFFER_L1,
      .fblen           = STM32_LTDC_L1_FBSIZE,
      .stride          = STM32_LTDC_L1_STRIDE,
      .display         = 0,
      .bpp             = STM32_LTDC_L1_BPP
    },
  .vinfo =
    {
      .fmt             = STM32_LTDC_L1_COLOR_FMT,
      .xres            = STM32_LTDC_WIDTH,
      .yres            = STM32_LTDC_HEIGHT,
      .nplanes         = 1,
#  ifdef CONFIG_FB_OVERLAY
      .noverlays       = LTDC_NOVERLAYS
#  endif
    }
#endif /* CONFIG_STM32H7_LTDC_L2 */
  ,
#ifdef CONFIG_STM32H7_FB_CMAP
  .cmap =
    {
      .first           = 0,
      .len             = STM32_LTDC_NCLUT,
      .red             = g_redclut,
      .green           = g_greenclut,
      .blue            = g_blueclut,
#  ifdef CONFIG_STM32H7_FB_TRANSPARENCY
      .transp          = g_transpclut
#  endif
    }
  ,
#endif
  .layer[LTDC_LAYER_L1] =
    {
      .layerno = LTDC_LAYER_L1,
#ifdef CONFIG_FB_OVERLAY
      .oinfo =
        {
          .fbmem            = (uint8_t *)STM32_LTDC_BUFFER_L1,
          .fblen            = STM32_LTDC_L1_FBSIZE,
          .stride           = STM32_LTDC_L1_STRIDE,
          .overlay          = LTDC_LAYER_L1,
          .bpp              = STM32_LTDC_L1_BPP,
          .blank            = 0,
          .chromakey        = 0,
          .color            = 0,
          .transp =
            {
              .transp       = 0xff,
              .transp_mode  = FB_CONST_ALPHA
            },
          .sarea =
            {
              .x            = 0,
              .y            = 0,
              .w            = STM32_LTDC_WIDTH,
              .h            = STM32_LTDC_HEIGHT
            },
          .accl             = LTDC_L1_ACCL
        },
#endif

#ifdef CONFIG_STM32H7_DMA2D
      .dma2dinfo =
        {
            .fmt            = STM32_LTDC_L1_DMA2D_PF,
            .transp_mode    = STM32_DMA2D_PFCCR_AM_NONE,
            .xres           = STM32_LTDC_WIDTH,
            .yres           = STM32_LTDC_HEIGHT,
            .oinfo          = &g_vtable.layer[LTDC_LAYER_L1].oinfo
        },
#endif
      .lock = &g_lock
    }
#ifdef CONFIG_STM32H7_LTDC_L2
  ,
  .layer[LTDC_LAYER_L2] =
    {
      .layerno = LTDC_LAYER_L2,
#ifdef CONFIG_FB_OVERLAY
      .oinfo =
        {
          .overlay          = LTDC_LAYER_L2,
          .fbmem            = (uint8_t *)STM32_LTDC_BUFFER_L2,
          .fblen            = STM32_LTDC_L2_FBSIZE,
          .stride           = STM32_LTDC_L2_STRIDE,
          .bpp              = STM32_LTDC_L2_BPP,
          .blank            = 0,
          .chromakey        = 0,
          .color            = 0,
          .transp =
            {
              .transp       = 0xff,
              .transp_mode  = FB_CONST_ALPHA
            },
          .sarea =
            {
              .x            = 0,
              .y            = 0,
              .w            = STM32_LTDC_WIDTH,
              .h            = STM32_LTDC_HEIGHT
            },
          .accl             = LTDC_L2_ACCL
        },
#endif

#ifdef CONFIG_STM32H7_DMA2D
      .dma2dinfo =
        {
            .fmt            = STM32_LTDC_L2_DMA2D_PF,
            .transp_mode    = STM32_DMA2D_PFCCR_AM_NONE,
            .xres           = STM32_LTDC_WIDTH,
            .yres           = STM32_LTDC_HEIGHT,
            .oinfo          = &g_vtable.layer[LTDC_LAYER_L2].oinfo
        },
#endif
      .lock = &g_lock
    }
#endif
};









/****************************************************************************
 * Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ltdc_irqctrl
 *
 * Description:
 *   Control  interrupts generated by the ltdc controller
 *
 * Input Parameters:
 *   setirqs  - set interrupt mask
 *   clrirqs  - clear interrupt mask
 *
 ****************************************************************************/

static void stm32_ltdc_irqctrl(uint32_t setirqs, uint32_t clrirqs)
{
  uint32_t regval;

  regval = getreg32(STM32_LTDC_IER);
  regval &= ~clrirqs;
  regval |= setirqs;
  reginfo("set LTDC_IER=%08x\n", regval);
  putreg32(regval, STM32_LTDC_IER);
  reginfo("configured LTDC_IER=%08x\n", getreg32(STM32_LTDC_IER));
}



/****************************************************************************
 * Name: stm32_ltdc_linepos
 *
 * Description:
 *   Configures line position register
 *
 ****************************************************************************/

static void stm32_ltdc_linepos(void)
{
  /* Configure LTDC_LIPCR */

  reginfo("set LTDC_LIPCR=%08x\n", STM32_LTDC_LIPCR_LIPOS);
  putreg32(STM32_LTDC_LIPCR_LIPOS, STM32_LTDC_LIPCR);
  reginfo("configured LTDC_LIPCR=%08x\n", getreg32(STM32_LTDC_LIPCR));
}

/****************************************************************************
 * Name: stm32_dsiirq
 *
 * Description:
 *   DSI interrupt handler
 *
 ****************************************************************************/

static int stm32_dsiirq(int irq, void *context, void *arg)
{

  reginfo("irq = %d\n", irq);

  return OK;
}

/****************************************************************************
 * Name: stm32_ltdcirq
 *
 * Description:
 *   LTDC interrupt handler
 *
 ****************************************************************************/

static int stm32_ltdcirq(int irq, void *context, void *arg)
{
  int ret;
  struct stm32_interrupt_s *priv = &g_interrupt;
  uint32_t regval = getreg32(STM32_LTDC_ISR);

  reginfo("irq = %d, regval = %08x\n", irq, regval);

  if (regval & LTDC_ISR_RRIF)
    {
      /* Register reload interrupt */

      /* Clear the interrupt status register */

      reginfo("Register reloaded\n");
      putreg32(LTDC_ICR_CRRIF, STM32_LTDC_ICR);
      priv->error = OK;
    }
  else if (regval & LTDC_IER_LIE)
    {
      /* Line interrupt */

      /* Clear the interrupt status register */

      reginfo("Line interrupt\n");
      putreg32(LTDC_ICR_CLIF, STM32_LTDC_ICR);
      priv->error = OK;
    }
  else if (regval & LTDC_IER_TERRIE)
    {
      /* Transfer error interrupt */

      /* Clear the interrupt status register */

      reginfo("Error transfer\n");
      putreg32(LTDC_ICR_CTERRIF, STM32_LTDC_ICR);
      priv->error = -ECANCELED;
    }
  else if (regval & LTDC_IER_FUIE)
    {
      /* Fifo underrun error interrupt */

      /* Clear the interrupt status register */

      reginfo("Error fifo underrun\n");
      putreg32(LTDC_ICR_CFUIF, STM32_LTDC_ICR);
      priv->error = -ECANCELED;
    }
  else
    {
      DEBUGASSERT("Unknown interrupt");
    }

  /* Unlock the semaphore if locked */

  ret = nxsem_post(priv->sem);

  if (ret < 0)
    {
      lcderr("ERROR: nxsem_post() failed\n");
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_ltdc_waitforirq
 *
 * Description:
 *   Helper waits until the ltdc irq occurs. In the current design That means
 *   that a register reload was been completed.
 *   Note! The caller must use this function within a critical section.
 *
 * Returned Value:
 *   OK - On success otherwise ERROR
 *
 ****************************************************************************/

static int stm32_ltdc_waitforirq(void)
{
  int ret = OK;
  struct stm32_interrupt_s *priv = &g_interrupt;

  ret = nxsem_wait(priv->sem);

  if (ret < 0)
    {
      lcderr("ERROR: nxsem_wait() failed\n");
    }

  ret = priv->error;

  return ret;
}

/****************************************************************************
 * Name: stm32_ltdc_reload
 *
 * Description:
 *   Reload the layer shadow register and make layer changes visible.
 *   Note! The caller must ensure that a previous register reloading has been
 *   completed.
 *
 * Input Parameters:
 *   value      - Reload flag (e.g. upon vertical blank or immediately)
 *   waitvblank - Wait until register reload is finished
 *
 ****************************************************************************/

static int stm32_ltdc_reload(uint8_t value, bool waitvblank)
{
  int ret = OK;

  /* Reloads the shadow register.
   * Note! This will not trigger an register reload interrupt if
   * immediately reload is set.
   */

  reginfo("set LTDC_SRCR=%08x\n", value);
  putreg32(value, STM32_LTDC_SRCR);
  reginfo("configured LTDC_SRCR=%08x\n", getreg32(STM32_LTDC_SRCR));

  if (value == LTDC_SRCR_VBR && waitvblank)
    {
      /* Wait upon vertical blanking period */

      ret = stm32_ltdc_waitforirq();
    }
  else
    {
      /* Wait until register reload hase been done */

      while (getreg32(STM32_LTDC_SRCR) & value);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_ltdc_irqconfig
 *
 * Description:
 *   Configure interrupts
 *
 ****************************************************************************/

static void stm32_ltdc_irqconfig(void)
{

  /* Attach DSI interrupt vector */

  irq_attach(g_interrupt.dsi_irq, stm32_dsiirq, NULL);


  /* Attach LTDC interrupt vector */

  irq_attach(g_interrupt.irq, stm32_ltdcirq, NULL);

  /* Enable the IRQ at the NVIC */

  up_enable_irq(g_interrupt.irq);

  /* Enable interrupts expect line interrupt */

  stm32_ltdc_irqctrl(LTDC_IER_RRIE |
                     LTDC_IER_TERRIE |
                     LTDC_IER_FUIE,
                     LTDC_IER_LIE);

  /* Configure line interrupt */

  stm32_ltdc_linepos();
}


/****************************************************************************
 * Name: stm32_ltdc_ltransp
 *
 * Description:
 *   Change layer transparency.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   transp - Transparency
 *   mode   - Transparency mode
 *
 ****************************************************************************/

static void stm32_ltdc_ltransp(struct stm32_ltdc_s *layer,
                               uint8_t transp, uint32_t mode)
{

}

/****************************************************************************
 * Name: stm32_ltdc_enable
 *
 * Description:
 *   Disable the LCD peripheral
 *
 * Input Parameters:
 *   enable - Enable or disable
 *
 ****************************************************************************/

static void stm32_ltdc_enable(bool enable)
{
  uint32_t    regval;

  regval = getreg32(STM32_LTDC_GCR);
  reginfo("get LTDC_GCR=%08x\n", regval);

  if (enable == true)
    {
      regval |= LTDC_GCR_LTDCEN;
    }
  else
    {
      regval &= ~LTDC_GCR_LTDCEN;
    }

  reginfo("set LTDC_GCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_GCR);
  reginfo("configured LTDC_GCR=%08x\n", getreg32(STM32_LTDC_GCR));
}


/****************************************************************************
 * Name: stm32_ltdc_lenable
 *
 * Description:
 *   Enable or disable layer.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   enable - Enable or disable layer
 *
 ****************************************************************************/

static void stm32_ltdc_lenable(struct stm32_ltdc_s *layer, bool enable)
{
  // uint32_t   regval;
  // DEBUGASSERT(layer->layerno < LTDC_NLAYERS);

  // regval = getreg32(stm32_cr_layer_t[layer->layerno]);

  // if (enable == true)
  //   {
  //     regval |= LTDC_LxCR_LEN;
  //   }
  // else
  //   {
  //     regval &= ~LTDC_LxCR_LEN;
  //   }

  // /* Enable/Disable layer */

  // reginfo("set LTDC_L%dCR=%08x\n", layer->layerno + 1, regval);
  // putreg32(regval, stm32_cr_layer_t[layer->layerno]);

  // /* Reload shadow register */

  // stm32_ltdc_reload(LTDC_SRCR_IMR, false);
}

/****************************************************************************
 * Name: stm32_ltdc_lchromakey
 *
 * Description:
 *   Change layer chromakey.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   chroma - chromakey
 *
 ****************************************************************************/

static void stm32_ltdc_lchromakey(struct stm32_ltdc_s *layer,
                                  uint32_t chroma)
{
//   uint32_t rgb;
//   DEBUGASSERT(layer->layerno < LTDC_NLAYERS);

//   reginfo("%08x\n", getreg32(stm32_cr_layer_t[layer->layerno]));

//   /* Set chromakey */

// #ifdef CONFIG_STM32F7_FB_CMAP
//   uint8_t r = g_vtable.cmap.red[chroma];
//   uint8_t g = g_vtable.cmap.green[chroma];
//   uint8_t b = g_vtable.cmap.blue[chroma];
//   rgb = ((r << 16) | (g << 8) | b);
// #else
//   rgb = ARGB8888(chroma);
// #endif

//   reginfo("set LTDC_L%dCKCR=%08x\n", layer->layerno + 1, rgb);
//   putreg32(rgb, stm32_ckcr_layer_t[layer->layerno]);

//   /* Reload shadow register */

//   stm32_ltdc_reload(LTDC_SRCR_IMR, false);
}



void stm32_ltdc_init(void)
{
  // uint32_t regval;


    /* Configure the HS, VS, DE and PC polarity */
  modifyreg32(STM32_LTDC_GCR,
        (LTDC_GCR_HSPOL | LTDC_GCR_VSPOL | LTDC_GCR_DEPOL | LTDC_GCR_PCPOL), 
        (BOARD_LTDC_GCR_HSPOL | BOARD_LTDC_GCR_VSPOL | BOARD_LTDC_GCR_DEPOL | BOARD_LTDC_GCR_PCPOL));

  /* Set Synchronization size */
  modifyreg32(STM32_LTDC_SSCR, (LTDC_SSCR_VSH | LTDC_SSCR_HSW), 
      (BOARD_LTDC_HSYNC << 16U) | BOARD_LTDC_VSYNC );

  /* Set Accumulated Back porch */
  modifyreg32(STM32_LTDC_BPCR, (LTDC_BPCR_AVBP | LTDC_BPCR_AHBP), 
      (STM32_LTDC_ACCUMULATED_HBP << 16U) | STM32_LTDC_ACCUMULATED_VBP);

  /* Set Accumulated Active Width */
  modifyreg32(STM32_LTDC_AWCR, (LTDC_AWCR_AAH | LTDC_AWCR_AAW),
      (STM32_LTDC_ACCUM_ACT_W<< 16U) | STM32_LTDC_ACCUM_ACT_H);

  /* Set Total Width */
  modifyreg32(STM32_LTDC_TWCR, (LTDC_TWCR_TOTALH_MASK | LTDC_TWCR_TOTALW_MASK),
      (STM32_LTDC_TOTAL_W << 16U) | STM32_LTDC_TOTAL_H);

  /* Set the background color value */

  modifyreg32(STM32_LTDC_BCCR, (LTDC_BCCR_BCBLUE | LTDC_BCCR_BCGREEN | LTDC_BCCR_BCRED), \
      (STM32_LTDC_BKG_COLOR_G << 8U) | (STM32_LTDC_BKG_COLOR_R << 16U) | \
       STM32_LTDC_BKG_COLOR_R );


  /* Enable the Transfer Error and FIFO underrun interrupts */
  //modifyreg32(STM32_LTDC_IER, 0, LTDC_IT_TE | LTDC_IT_FU);
  stm32_ltdc_irqconfig();

  // modifyreg32(STM32_LTDC_GCR, 0, LTDC_GCR_LTDCEN);
  stm32_ltdc_enable(true);

}

/****************************************************************************
 * Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ltdc_reset
 *
 * Description:
 *   Reset LTDC via APB3RSTR
 *
 ****************************************************************************/

void stm32_ltdc_reset(void)
{
  uint32_t regval = getreg32(STM32_RCC_APB3RSTR);

  putreg32(regval | RCC_APB3RSTR_LTDCRST, STM32_RCC_APB3RSTR);

  putreg32(regval & ~RCC_APB3RSTR_LTDCRST, STM32_RCC_APB3RSTR);
  
  // dummy read
  regval = getreg32(STM32_RCC_APB3RSTR);

}


/** @defgroup LTDC_Private_Functions LTDC Private Functions
  * @{
  */

/**
  * @brief  Configure the LTDC peripheral
  * @param  hltdc     Pointer to a LTDC_HandleTypeDef structure that contains
  *                   the configuration information for the LTDC.
  * @param  pLayerCfg Pointer LTDC Layer Configuration structure
  * @param  LayerIdx  LTDC Layer index.
  *                   This parameter can be one of the following values: LTDC_LAYER_1 (0) or LTDC_LAYER_2 (1)
  * @retval None
  */
static void stm32_ltdc_set_config(LTDC_LayerCfg_t *pLayerCfg, uint32_t LayerIdx)
{
  uint32_t tmp;
  uint32_t tmp1;
  uint32_t tmp2;

  if(LayerIdx == 0){
    /* Configure the horizontal start and stop position */
    tmp = ((pLayerCfg->WindowX1 + ((getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AHBP) >> 16U)) << 16U);
    modifyreg32(STM32_LTDC_L1WHPCR, (LTDC_LxWHPCR_WHSTPOS | LTDC_LxWHPCR_WHSPPOS), 
                ((pLayerCfg->WindowX0 + ((getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AHBP) >> 16U) + 1U) | tmp));

    /* Configure the vertical start and stop position */
    tmp = ((pLayerCfg->WindowY1 + (getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AVBP)) << 16U);
    modifyreg32(STM32_LTDC_L1WVPCR, (LTDC_LxWVPCR_WVSTPOS | LTDC_LxWVPCR_WVSPPOS),
                ((pLayerCfg->WindowY0 + (getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AVBP) + 1U) | tmp));

    /* Specifies the pixel format */
    modifyreg32(STM32_LTDC_L1PFCR, LTDC_LxPFCR_PF, 
                pLayerCfg->PixelFormat);

    /* Configure the default color values */
    tmp = ((uint32_t)(pLayerCfg->Backcolor.Green) << 8U);
    tmp1 = ((uint32_t)(pLayerCfg->Backcolor.Red) << 16U);
    tmp2 = (pLayerCfg->Alpha0 << 24U);
    modifyreg32(STM32_LTDC_L1DCCR, (LTDC_LxDCCR_DCBLUE | LTDC_LxDCCR_DCGREEN | LTDC_LxDCCR_DCRED | LTDC_LxDCCR_DCALPHA), 
                (pLayerCfg->Backcolor.Blue | tmp | tmp1 | tmp2));

    /* Specifies the constant alpha value */
    modifyreg32(STM32_LTDC_L1CACR, LTDC_LxCACR_CONSTA, pLayerCfg->Alpha);

    /* Specifies the blending factors */
    modifyreg32(STM32_LTDC_L1BFCR, (LTDC_LxBFCR_BF2 | LTDC_LxBFCR_BF1), 
                (pLayerCfg->BlendingFactor1 | pLayerCfg->BlendingFactor2));

    /* Configure the color frame buffer start address */
    modifyreg32(STM32_LTDC_L1CFBAR, (LTDC_LxCFBAR_CFBADD), 
                (pLayerCfg->FBStartAdress));

    if (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
    {
      tmp = 4U;
    }
    else if (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB888)
    {
      tmp = 3U;
    }
    else if ((pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB4444) || \
            (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB565)   || \
            (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB1555) || \
            (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_AL88))
    {
      tmp = 2U;
    }
    else
    {
      tmp = 1U;
    }

    /* Configure the color frame buffer pitch in byte */
    modifyreg32(STM32_LTDC_L1CFBLR, (LTDC_LxCFBLR_CFBLL | LTDC_LxCFBLR_CFBP), 
                (((pLayerCfg->ImageWidth * tmp) << 16U) | (((pLayerCfg->WindowX1 - pLayerCfg->WindowX0) * tmp)  + 7U)));
    /* Configure the frame buffer line number */
    modifyreg32(STM32_LTDC_L1CFBLNR, (LTDC_LxCFBLNR_CFBLNBR), pLayerCfg->ImageHeight);

    /* Enable LTDC_Layer by setting LEN bit */
    modifyreg32(STM32_LTDC_L1CR, 0, (uint32_t)LTDC_LxCR_LEN);

  }
  else{
    /* Configure the horizontal start and stop position */
    tmp = ((pLayerCfg->WindowX1 + ((getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AHBP) >> 16U)) << 16U);
    modifyreg32(STM32_LTDC_L2WHPCR, (LTDC_LxWHPCR_WHSTPOS | LTDC_LxWHPCR_WHSPPOS), 
                ((pLayerCfg->WindowX0 + ((getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AHBP) >> 16U) + 1U) | tmp));

    /* Configure the vertical start and stop position */
    tmp = ((pLayerCfg->WindowY1 + (getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AVBP)) << 16U);
    modifyreg32(STM32_LTDC_L2WVPCR, (LTDC_LxWVPCR_WVSTPOS | LTDC_LxWVPCR_WVSPPOS),
                ((pLayerCfg->WindowY0 + (getreg32(STM32_LTDC_BPCR) & LTDC_BPCR_AVBP) + 1U) | tmp));

    /* Specifies the pixel format */
    modifyreg32(STM32_LTDC_L2PFCR, LTDC_LxPFCR_PF, 
                pLayerCfg->PixelFormat);

    /* Configure the default color values */
    tmp = ((uint32_t)(pLayerCfg->Backcolor.Green) << 8U);
    tmp1 = ((uint32_t)(pLayerCfg->Backcolor.Red) << 16U);
    tmp2 = (pLayerCfg->Alpha0 << 24U);
    modifyreg32(STM32_LTDC_L2DCCR, (LTDC_LxDCCR_DCBLUE | LTDC_LxDCCR_DCGREEN | LTDC_LxDCCR_DCRED | LTDC_LxDCCR_DCALPHA), 
                (pLayerCfg->Backcolor.Blue | tmp | tmp1 | tmp2));

    /* Specifies the constant alpha value */
    modifyreg32(STM32_LTDC_L2CACR, LTDC_LxCACR_CONSTA, pLayerCfg->Alpha);

    /* Specifies the blending factors */
    modifyreg32(STM32_LTDC_L2BFCR, (LTDC_LxBFCR_BF2 | LTDC_LxBFCR_BF1), 
                (pLayerCfg->BlendingFactor1 | pLayerCfg->BlendingFactor2));

    /* Configure the color frame buffer start address */
    modifyreg32(STM32_LTDC_L2CFBAR, (LTDC_LxCFBAR_CFBADD), 
                (pLayerCfg->FBStartAdress));

    if (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
    {
      tmp = 4U;
    }
    else if (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB888)
    {
      tmp = 3U;
    }
    else if ((pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB4444) || \
            (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB565)   || \
            (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB1555) || \
            (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_AL88))
    {
      tmp = 2U;
    }
    else
    {
      tmp = 1U;
    }

    /* Configure the color frame buffer pitch in byte */
    modifyreg32(STM32_LTDC_L2CFBLR, (LTDC_LxCFBLR_CFBLL | LTDC_LxCFBLR_CFBP), 
                (((pLayerCfg->ImageWidth * tmp) << 16U) | (((pLayerCfg->WindowX1 - pLayerCfg->WindowX0) * tmp)  + 7U)));
    /* Configure the frame buffer line number */
    modifyreg32(STM32_LTDC_L2CFBLNR, (LTDC_LxCFBLNR_CFBLNBR), pLayerCfg->ImageHeight);

    /* Enable LTDC_Layer by setting LEN bit */
    modifyreg32(STM32_LTDC_L2CR, 0, (uint32_t)LTDC_LxCR_LEN);

  }
}



/**
  * @brief  Configure the LTDC Layer according to the specified
  *         parameters in the LTDC_InitTypeDef and create the associated handle.
  * @param  hltdc      pointer to a LTDC_HandleTypeDef structure that contains
  *                    the configuration information for the LTDC.
  * @param  pLayerCfg  pointer to a LTDC_LayerCfg_t structure that contains
  *                    the configuration information for the Layer.
  * @param  LayerIdx  LTDC Layer index.
  *                    This parameter can be one of the following values:
  *                    LTDC_LAYER_1 (0) or LTDC_LAYER_2 (1)
  * @retval HAL status
  */
int stm32_ltdc_config_layer( LTDC_LayerCfg_t *pLayerCfg, uint32_t LayerIdx)
{




  /* Configure the LTDC Layer */
  stm32_ltdc_set_config(pLayerCfg, LayerIdx);

  /* Set the Immediate Reload type */
  putreg32(LTDC_SRCR_IMR, STM32_LTDC_SRCR);



  return OK;
}





void stm32_ltdc_layer_init(void)
{
  LTDC_LayerCfg_t  layercfg;

  uint32_t base_addr = 0;

  base_addr = (uint32_t)malloc(STM32_LTDC_TOTAL_FBSIZE);
  if (base_addr == NULL)
    {
      lcderr("failed to malloc draw buffer");
      return;
    }

  /* Layer Init */
  layercfg.WindowX0 = 0;
  layercfg.WindowX1 = STM32_LTDC_WIDTH ;
  layercfg.WindowY0 = 0;
  layercfg.WindowY1 = STM32_LTDC_HEIGHT; 
  layercfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  layercfg.FBStartAdress = base_addr;
  layercfg.Alpha = 255;
  layercfg.Alpha0 = 0;
  layercfg.Backcolor.Blue = 0;
  layercfg.Backcolor.Green = 0;
  layercfg.Backcolor.Red = 0;
  layercfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  layercfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  layercfg.ImageWidth = 800;
  layercfg.ImageHeight = 480;

  stm32_ltdc_config_layer(&layercfg, 0); 

  g_vtable.pinfo.fbmem = base_addr;

}

/****************************************************************************
 * Name: stm32_getvideoinfo
 *
 * Description:
 *   Entrypoint ioctl FBIOGET_VIDEOINFO
 *   Get the videoinfo for the framebuffer
 *
 * Input Parameters:
 *   vtable - The framebuffer driver object
 *   vinfo  - the videoinfo object
 *
 * Returned Value:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int stm32_getvideoinfo(struct fb_vtable_s *vtable,
                              struct fb_videoinfo_s *vinfo)
{
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  lcdinfo("vtable=%p vinfo=%p\n", vtable, vinfo);
  DEBUGASSERT(vtable != NULL && priv == &g_vtable && vinfo != NULL);

  memcpy(vinfo, &priv->vinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/****************************************************************************
 * Name: stm32_getplaneinfo
 *
 * Description:
 *   Entrypoint ioctl FBIOGET_PLANEINFO
 *   Get the planeinfo for the framebuffer
 *
 * Input Parameters:
 *   vtable - The framebuffer driver object
 *   pinfo  - the planeinfo object
 *
 * Returned Value:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int stm32_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                              struct fb_planeinfo_s *pinfo)
{
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable);
  lcdinfo("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);

  if (planeno == 0)
    {
      memcpy(pinfo, &priv->pinfo, sizeof(struct fb_planeinfo_s));
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}



/****************************************************************************
 * Name: stm32_getoverlayinfo
 * Description:
 *   Entrypoint ioctl FBIOGET_OVERLAYINFO
 ****************************************************************************/

#ifdef CONFIG_FB_OVERLAY
static int stm32_getoverlayinfo(struct fb_vtable_s *vtable,
                                int overlayno,
                                struct fb_overlayinfo_s *oinfo)
{
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  lcdinfo("vtable=%p overlay=%d oinfo=%p\n", vtable, overlayno, oinfo);
  DEBUGASSERT(vtable != NULL && priv == &g_vtable);

  if (overlayno < LTDC_NOVERLAYS)
    {
      struct stm32_ltdc_s *layer = &priv->layer[overlayno];
      memcpy(oinfo, &layer->oinfo, sizeof(struct fb_overlayinfo_s));
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_settransp
 * Description:
 *   Entrypoint ioctl FBIOSET_TRANSP
 ****************************************************************************/

static int stm32_settransp(struct fb_vtable_s *vtable,
                           const struct fb_overlayinfo_s *oinfo)
{
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable);
  lcdinfo("vtable=%p, overlay=%d, transp=%02x, transp_mode=%02x\n", vtable,
          oinfo->overlay, oinfo->transp.transp, oinfo->transp.transp_mode);

  if (oinfo->transp.transp_mode > 1)
    {
      lcderr("ERROR: Returning ENOSYS, transparency mode not supported\n");
      return -ENOSYS;
    }

  if (oinfo->overlay < LTDC_NOVERLAYS)
    {
      struct stm32_ltdc_s *layer = &priv->layer[oinfo->overlay];

      nxmutex_lock(layer->lock);
      layer->oinfo.transp.transp      = oinfo->transp.transp;
      layer->oinfo.transp.transp_mode = oinfo->transp.transp_mode;

#  ifdef CONFIG_STM32H7_DMA2D
      if (layer->oinfo.transp.transp_mode == 0)
        {
          layer->dma2dinfo.transp_mode = STM32_DMA2D_PFCCR_AM_CONST;
        }
      else if (layer->oinfo.transp.transp_mode == 1)
        {
          layer->dma2dinfo.transp_mode = STM32_DMA2D_PFCCR_AM_PIXEL;
        }

      if (oinfo->overlay < LTDC_NLAYERS)
#  endif
        {
          /* Set LTDC blendmode and alpha value */

          stm32_ltdc_ltransp(layer, layer->oinfo.transp.transp,
                             layer->oinfo.transp.transp_mode);
        }

      nxmutex_unlock(layer->lock);
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_setchromakey
 * Description:
 *   Entrypoint ioctl FBIOSET_CHROMAKEY
 ****************************************************************************/

static int stm32_setchromakey(struct fb_vtable_s *vtable,
                              const struct fb_overlayinfo_s *oinfo)
{
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable && oinfo != NULL);
  lcdinfo("vtable=%p, overlay=%d, chromakey=%08x\n", vtable,
          oinfo->overlay, oinfo->chromakey);

  if (oinfo->overlay < LTDC_NLAYERS)
    {
      int ret;
      struct stm32_ltdc_s *layer = &priv->layer[oinfo->overlay];

#  ifndef CONFIG_STM32H7_LTDC_L1_CHROMAKEY
      if (oinfo->overlay == LTDC_LAYER_L1)
        {
          return -ENOSYS;
        }
#  endif

#  ifndef CONFIG_STM32H7_LTDC_L2_CHROMAKEY
      if (oinfo->overlay == LTDC_LAYER_L2)
        {
          return -ENOSYS;
        }
#  endif

      nxmutex_lock(layer->lock);
#  ifdef CONFIG_STM32H7_FB_CMAP
      if (oinfo->chromakey >= g_vtable.cmap.len)
        {
          lcderr("ERROR: Clut index %d is out of range\n", oinfo->chromakey);
          ret = -EINVAL;
        }
      else
#  endif
        {
          layer->oinfo.chromakey = oinfo->chromakey;

          /* Set chromakey */

          stm32_ltdc_lchromakey(layer, layer->oinfo.chromakey);
          ret = OK;
        }

      nxmutex_unlock(layer->lock);
      return ret;
    }
#  ifdef CONFIG_STM32H7_DMA2D
  else if (oinfo->overlay < LTDC_NOVERLAYS)
    {
      /* Chromakey not supported by DMA2D */

      return -ENOSYS;
    }
#  endif

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_setcolor
 * Description:
 *   Entrypoint ioctl FBIOSET_COLOR
 ****************************************************************************/

static int stm32_setcolor(struct fb_vtable_s *vtable,
                          const struct fb_overlayinfo_s *oinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable && oinfo != NULL);
  lcdinfo("vtable=%p, overlay=%d, color=%08x\n", vtable, oinfo->color);

  if (oinfo->overlay < LTDC_NOVERLAYS)
    {
#  ifdef CONFIG_STM32H7_DMA2D

      /* Set color within the active overlay is not supported by LTDC. So use
       * DMA2D controller instead when configured.
       */

      int ret;
      struct stm32_ltdcdev_s *priv =
        (struct stm32_ltdcdev_s *)vtable;
      struct stm32_ltdc_s *layer = &priv->layer[oinfo->overlay];
      struct fb_overlayinfo_s *poverlay = layer->dma2dinfo.oinfo;

      DEBUGASSERT(&layer->oinfo == poverlay);

      nxmutex_lock(layer->lock);
      poverlay->color = oinfo->color;
      ret = priv->dma2d->fillcolor(&layer->dma2dinfo, &poverlay->sarea,
                                   poverlay->color);
      nxmutex_unlock(layer->lock);

      return ret;
#  else
      /* Coloring not supported by LTDC */

      return -ENOSYS;
#  endif
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_setblank
 * Description:
 *   Entrypoint ioctl FBIOSET_BLANK
 ****************************************************************************/

static int stm32_setblank(struct fb_vtable_s *vtable,
                          const struct fb_overlayinfo_s *oinfo)
{
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable && oinfo != NULL);
  lcdinfo("vtable=%p, overlay=%d, blank=%02x\n", vtable, oinfo->blank);

  if (oinfo->overlay < LTDC_NLAYERS)
    {
      struct stm32_ltdc_s *layer = &priv->layer[oinfo->overlay];

      nxmutex_lock(layer->lock);
      layer->oinfo.blank = oinfo->blank;

      /* Enable or disable layer */

      stm32_ltdc_lenable(layer, (layer->oinfo.blank == 0));
      nxmutex_unlock(layer->lock);

      return OK;
    }
#  ifdef CONFIG_STM32H7_DMA2D
  else if (oinfo->overlay < LTDC_NOVERLAYS)
    {
      /* DMA2D overlays are non visible */

      return OK;
    }
#  endif

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_setarea
 * Description:
 *   Entrypoint ioctl FBIOSET_AREA
 ****************************************************************************/

static int stm32_setarea(struct fb_vtable_s *vtable,
                         const struct fb_overlayinfo_s *oinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable && oinfo != NULL);
  lcdinfo("vtable=%p, overlay=%d, x=%d, y=%d, w=%d, h=%d\n", vtable,
          oinfo->overlay, oinfo->sarea.x, oinfo->sarea.y, oinfo->sarea.w,
          oinfo->sarea.h);

  if (oinfo->overlay < LTDC_NLAYERS)
    {
      /* LTDC area is defined by the overlay size (display resolution) only */

      return -ENOSYS;
    }

#  ifdef CONFIG_STM32H7_DMA2D
  if (oinfo->overlay < LTDC_NOVERLAYS)
    {
      struct stm32_ltdcdev_s *priv =
        (struct stm32_ltdcdev_s *)vtable;
      struct stm32_ltdc_s *layer =
        &priv->layer[oinfo->overlay];

      nxmutex_lock(layer->lock);
      memcpy(&layer->oinfo.sarea, &oinfo->sarea, sizeof(struct fb_area_s));
      nxmutex_unlock(layer->lock);

      return OK;
    }
#  endif

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_blit
 * Description:
 *   Entrypoint ioctl FBIOSET_BLIT
 ****************************************************************************/

#  ifdef CONFIG_FB_OVERLAY_BLIT
static int stm32_blit(struct fb_vtable_s *vtable,
                      const struct fb_overlayblit_s *blit)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable && blit != NULL);
  lcdinfo("vtable = %p, blit = %p\n", vtable, blit);

  if (blit->dest.overlay < LTDC_NOVERLAYS &&
      blit->src.overlay < LTDC_NOVERLAYS)
    {
#    ifdef CONFIG_STM32H7_DMA2D
      int ret;
      struct fb_area_s sarea;
      const struct fb_area_s *darea = &blit->dest.area;
      struct stm32_ltdcdev_s *priv =
        (struct stm32_ltdcdev_s *)vtable;
      struct stm32_ltdc_s *dlayer =
        &priv->layer[blit->dest.overlay];
      struct stm32_ltdc_s *slayer =
        &priv->layer[blit->src.overlay];

      DEBUGASSERT(&dlayer->oinfo == dlayer->dma2dinfo.oinfo &&
                  &slayer->oinfo == slayer->dma2dinfo.oinfo);

      /* DMA2D doesn't support image scale, so set to the smallest area */

      memcpy(&sarea, &blit->src.area, sizeof(struct fb_area_s));

      /* Check if area is within the entire overlay */

      if (!stm32_ltdc_lvalidate(dlayer, darea) ||
          !stm32_ltdc_lvalidate(slayer, &sarea))
        {
          return -EINVAL;
        }

      sarea.w = MIN(darea->w, sarea.w);
      sarea.h = MIN(darea->h, sarea.h);

      nxmutex_lock(dlayer->lock);
      ret = priv->dma2d->blit(&dlayer->dma2dinfo, darea->x, darea->y,
                              &slayer->dma2dinfo, &sarea);
      nxmutex_unlock(dlayer->lock);

      return ret;
#    else
      /* LTDC doesn't support blit transfer */

      return -ENOSYS;
#    endif
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_blend
 * Description:
 *   Entrypoint ioctl FBIOSET_BLEND
 ****************************************************************************/

static int stm32_blend(struct fb_vtable_s *vtable,
                       const struct fb_overlayblend_s *blend)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable && blend != NULL);
  lcdinfo("vtable = %p, blend = %p\n", vtable, blend);

  if (blend->dest.overlay < LTDC_NOVERLAYS &&
      blend->foreground.overlay < LTDC_NOVERLAYS &&
      blend->background.overlay < LTDC_NOVERLAYS)
    {
#    ifdef CONFIG_STM32H7_DMA2D
      int ret;
      struct fb_area_s barea;
      const struct fb_area_s *darea = &blend->dest.area;
      const struct fb_area_s *farea = &blend->foreground.area;
      struct stm32_ltdcdev_s *priv =
        (struct stm32_ltdcdev_s *)vtable;
      struct stm32_ltdc_s *dlayer = &priv->layer[blend->dest.overlay];
      struct stm32_ltdc_s *flayer =
        &priv->layer[blend->foreground.overlay];
      struct stm32_ltdc_s *blayer =
        &priv->layer[blend->background.overlay];

      DEBUGASSERT(&dlayer->oinfo == dlayer->dma2dinfo.oinfo &&
                  &flayer->oinfo == flayer->dma2dinfo.oinfo &&
                  &blayer->oinfo == blayer->dma2dinfo.oinfo);

      /* DMA2D doesn't support image scale, so set to the smallest area */

      memcpy(&barea, &blend->background.area, sizeof(struct fb_area_s));

      /* Check if area is within the entire overlay */

      if (!stm32_ltdc_lvalidate(dlayer, darea) ||
          !stm32_ltdc_lvalidate(flayer, farea) ||
          !stm32_ltdc_lvalidate(blayer, &barea))
        {
          lcderr("ERROR: Returning EINVAL\n");
          return -EINVAL;
        }

      barea.w = MIN(darea->w, barea.w);
      barea.h = MIN(darea->h, barea.h);
      barea.w = MIN(farea->w, barea.w);
      barea.h = MIN(farea->h, barea.h);

      nxmutex_lock(dlayer->lock);
      ret = priv->dma2d->blend(&dlayer->dma2dinfo, darea->x, darea->y,
                               &flayer->dma2dinfo, farea->x, farea->y,
                               &blayer->dma2dinfo, &barea);
      nxmutex_unlock(dlayer->lock);

      return ret;
#    else
      /* LTDC doesn't support blend transfer */

      return -ENOSYS;
#    endif
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}
#  endif /* CONFIG_FB_OVERLAY_BLIT */
#endif /* CONFIG_FB_OVERLAY */

/****************************************************************************
 * Name: stm32_ltdcgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Reference to the framebuffer object (NULL on failure)
 *
 ****************************************************************************/

struct fb_vtable_s *stm32_ltdcgetvplane(int vplane)
{
  lcdinfo("vplane: %d\n", vplane);

  if (vplane == 0)
    {
      return &g_vtable.vtable;
    }

  return NULL;
}


/****************************************************************************
 * Name: stm32_updatearea
 *
 * Description:
 *   stm32 update area !
 *
 ****************************************************************************/

static int stm32_updatearea(FAR struct fb_vtable_s *vtable,
                            FAR const struct fb_area_s *area)
{
  stm32_dsi_refresh(0);
  return OK;
}



/****************************************************************************
 * Name: stm32_ltdcuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer driver.  Bad things will happen if you
 *   call this without first calling fb_initialize()!
 *
 ****************************************************************************/

void stm32_ltdcuninitialize(void)
{
  /* Disable all ltdc interrupts */

  stm32_ltdc_irqctrl(0, LTDC_IER_RRIE | LTDC_IER_TERRIE |
                        LTDC_IER_FUIE | LTDC_IER_LIE);

  up_disable_irq(g_interrupt.irq);
  irq_detach(g_interrupt.irq);

  /* Disable the LCD controller */

  stm32_ltdc_enable(false);

  /* Set initialized state */

  g_ltdc_initialized = false;
}


#if STM32_LTDC_DEV_ENABLE
/****************************************************************************
 * Name:  stm32_ltdc_getdev
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

struct lcd_dev_s * stm32_ltdc_getdev( unsigned int planeno )
{
  return &g_vtable.dev;
}

/****************************************************************************
 * Name:  stm32_ltdc_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 *   dev     - The lcd device
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int stm32_ltdc_putrun(FAR struct lcd_dev_s *dev,
                          fb_coord_t row, fb_coord_t col,
                          FAR const uint8_t *buffer,
                          size_t npixels)
{
  return OK;
}

/****************************************************************************
 * Name:  stm32_ltdc_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD:
 *
 *  dev     - The lcd device
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int stm32_ltdc_getrun(FAR struct lcd_dev_s *dev,
                          fb_coord_t row, fb_coord_t col,
                          FAR uint8_t *buffer,
                          size_t npixels)
{
    return OK;
}




/****************************************************************************
 * Name:  stm32_ltdc_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int stm32_ltdc_getvideoinfo(FAR struct lcd_dev_s *dev,
                                 FAR struct fb_videoinfo_s *vinfo)
{
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)dev;


  memcpy(vinfo, &priv->vinfo, sizeof(struct fb_videoinfo_s));

  return OK;
}

/****************************************************************************
 * Name:  stm32_ltdc_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int stm32_ltdc_getplaneinfo(FAR struct lcd_dev_s *dev,
                                unsigned int planeno,
                                FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct stm32_ltdcdev_s *priv = (FAR struct stm32_ltdcdev_s *)dev;

  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdinfo("planeno: %d bpp: %d\n", planeno, STM32_LTDC_L1_BPP);

  pinfo->putrun = stm32_ltdc_putrun;                 /* Put a run into LCD memory */
  pinfo->getrun = stm32_ltdc_getrun;                 /* Get a run from LCD memory */
  pinfo->buffer = (FAR uint8_t *)g_vtable.pinfo.fbmem; /* Run scratch buffer */
  pinfo->bpp    = STM32_LTDC_L1_BPP;                    /* Bits-per-pixel */
  pinfo->dev    = dev;                            /* The lcd device */
  
  return OK;
}

/****************************************************************************
 * Name:  stm32_ltdc_getpower
 *
 * Description:
 *   Get the LCD panel power status
 *   (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int stm32_ltdc_getpower(FAR struct lcd_dev_s *dev)
{
  lcdinfo("power: %d\n", 0);
  return 0;
}

/****************************************************************************
 * Name:  stm32_ltdc_poweroff
 *
 * Description:
 *   Enable/disable LCD panel power
 *   (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int stm32_ltdc_poweroff(FAR struct stm32_ltdc_lcd_s *lcd)
{
  return OK;
}

/****************************************************************************
 * Name:  stm32_ltdc_setpower
 *
 * Description:
 *   Enable/disable LCD panel power
 *   (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int stm32_ltdc_setpower(FAR struct lcd_dev_s *dev, int power)
{

  return OK;
}

/****************************************************************************
 * Name:  stm32_ltdc_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int stm32_ltdc_getcontrast(FAR struct lcd_dev_s *dev)
{
  lcdinfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  stm32_ltdc_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int stm32_ltdc_setcontrast(FAR struct lcd_dev_s *dev,
                               unsigned int contrast)
{
  lcdinfo("contrast: %d\n", contrast);
  return -ENOSYS;
}

#endif

