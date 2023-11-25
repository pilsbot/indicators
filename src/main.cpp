#include <stdio.h>
#include <array>
#include "pico/stdlib.h"
#include <config.hpp>
#include "lights.hpp"

void do_static_fun();

int main() {
    setup_default_uart();
    stdio_init_all();

    Lights lights;
    lights.setParty(true);
    printf("set party mode\n");

    while (true)
    {
        auto elapsed_seconds = to_ms_since_boot(get_absolute_time()) / 1000;
        if (true) // (elapsed_seconds / 10) & 1)
        {
            // so, every even number of 10 seconds multiples...?
            switch (elapsed_seconds % 10)
            {
                case 0:
                    lights.setIndicatorLeft();
                    printf("Indicator on\n");
                    break;
                case 5:
                    lights.setIndicatorLeft(false);
                    lights.setHeadlight();
                    printf("Headlight on\n");
                    break;
                case 9:
                    lights.setHeadlight(false);
                    printf("nearly all off\n");
                    break;
            }
            lights.update();
            sleep_ms(15);   // dummy serial connection
        }
        else
        {

            do_static_fun();
        }
    }

    return 0;
}


extern "C"
{
#include "../lib/NeoPixel.h"
}

void do_static_fun()
{
    uint8_t cnts = 10;
    static constexpr ColorIntensity max_brightness = 0xA0;

    printf("\nHello, neopixel ");

    // some duff
    for (uint8_t cnt = 0; cnt < cnts; cnt++)
    {
        for (ColorIntensity intensity = 0; intensity < max_brightness; intensity++)
        {
            for (uint8_t i = 0; i < NEOC_NOF_LEDS_IN_LANE; i++)
            {
                if (NEO_SetPixelRGB(0, i, intensity, intensity/2, max_brightness - intensity) != Neopixel_ReturnCode::ERR_OK)
                {
                    printf("setPixel returned unsuccessful\n");
                }
            }
            NEO_TransferPixels();
            sleep_ms(3);
        }
        printf("%d ", cnt);
    }
    printf("\nthe ol' runlight ");

    for (uint8_t cnt = 0; cnt < cnts; cnt++)
    {
        for (uint8_t run = 0; run < NEOC_NOF_LEDS_IN_LANE; run++)
        {
            ColorIntensity r = cnt < cnts ? max_brightness / 2 : max_brightness;
            ColorIntensity g = cnt < cnts / 2 ? 0 : max_brightness;
            ColorIntensity b = cnt > cnts / 3 ? 0 : max_brightness;
            for (uint8_t i = 0; i < NEOC_NOF_LEDS_IN_LANE; i++)
            {
                ColorIntensity isCurrent = i == run ? 1 : 0;
                if (NEO_SetPixelRGB(0, i, r * isCurrent, g * isCurrent, b * isCurrent) != Neopixel_ReturnCode::ERR_OK)
                {
                    printf("setPixel returned unsuccessful\n");
                }
            }
            NEO_TransferPixels();
            sleep_ms(10);
        }
        printf("%d ", cnt);
    }
}