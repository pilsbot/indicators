#include <stdio.h>
#include <array>
#include "pico/stdlib.h"
#include "../include/config.h"
#include "../lib/ws2812.h"


static std::array<Pixel, num_leds> transmitBuf;

int main() {
    setup_default_uart();
    stdio_init_all();

    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ColorIntensity cnt = 0;

    ws2812_program_init(pio, sm, offset, externalStrip, 800000, true);

    while (true)
    {
      printf("Hello, world!\n");
      for (cnt = 0; cnt < max_brightness; cnt++)
      {
          put_rgb(cnt, max_brightness - cnt, 0);
          sleep_ms(3);
      }
      for (cnt = 0; cnt < max_brightness; cnt++)
      {
          put_rgb(max_brightness - cnt, 0, cnt);
          sleep_ms(3);
      }
      for (cnt = 0; cnt < max_brightness; cnt++)
      {
          put_rgb(0, cnt, max_brightness - cnt);
          sleep_ms(3);
      }
    }

    return 0;
}