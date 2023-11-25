#include <stdio.h>
#include <array>
#include "pico/stdlib.h"
#include "../include/config.h"
extern "C"
{
#include "../lib/NeoPixel.h"
#include "../lib/ws2812.h"
}


static constexpr PinNr internalLED = 16;
static constexpr PinNr externalStrip = 14;

static constexpr ColorIntensity max_brightness = 0xA0;
static constexpr uint16_t num_leds = 20;


static std::array<Pixel, num_leds> transmitBuf;

int main() {
    setup_default_uart();
    stdio_init_all();

    NEO_Init();
    WS2812_Init();

    while (true)
    {
      printf("Hello, world!\n");
      for (ColorIntensity cnt = 0; cnt < max_brightness; cnt++)
      {
        for (uint8_t i = 0; i < NEOC_NOF_LEDS_IN_LANE; i++)
        {
          if (NEO_SetPixelRGB(0, i, cnt, cnt/2, max_brightness - cnt) != Neopixel_ReturnCode::ERR_OK)
          {
            printf("setPixel returned unsuccessful\n");
          }
        }
        sleep_ms(3);
      }
    }

    return 0;
}