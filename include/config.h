#pragma once

#define NEOC_NOF_COLORS        (3)  /* number of colors: 3 for RGB, 4 for RGBW */

#define NEOC_PIN_START         (14) /* start of physical GPIO pin number: GPIO2 */
#define NEOC_NOF_LANES         (1)  /* number of lanes (and pins) to be used */

#define NEOC_LANE_START        (0)  /* start bit number in lane (logical) data: for Kinetis DMA, this has to match physical NEOC_PIN_START! */
#define NEOC_LANE_END          (NEOC_LANE_START+NEOC_NOF_LANES-1)  /* this is the last GPIO pin used: GPIOx */

#define NEOC_NOF_LEDS_IN_LANE   (32) /* number of LEDs in each lane */

#ifndef NEOC_NOF_COLORS
  #define NEOC_NOF_COLORS (3)  /* default number of colors: 3 for RGB, 4 for RGBW */
#endif

#if (NEOC_NOF_LANES>8)
  #error "can only handle up to 8 lanes (8bit port)"
#endif

#define NEOC_NOF_PIXEL            ((NEOC_NOF_LANES)*(NEOC_NOF_LEDS_IN_LANE)) /* total number of pixels in the system */

#define NEOC_USE_DMA              (1) /* if using DMA. Otherwise the bits are sent in a loop */

#define NEOC_PIO_32BIT_PIXELS     (1) /* pixels are organized as 32bit values, optimized for the RP2040 PIO, supporting 1 and up to 8 lanes */


#include "types.h"

