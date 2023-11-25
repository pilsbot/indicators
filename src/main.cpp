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
static constexpr PinNr externalStrip = NEOC_PIN_START;

static constexpr ColorIntensity max_brightness = 0xA0;
static constexpr uint16_t num_leds = 20;


static std::array<Pixel, num_leds> transmitBuf;

int main() {
    setup_default_uart();
    stdio_init_all();

    NEO_Init();
    WS2812_Init();

    uint8_t cnts = 10;

    while (true)
    {
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

    return 0;
}