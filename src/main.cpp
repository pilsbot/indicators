#include <config.hpp>
#include "lights.hpp"

#include <stdio.h>
#include <array>
#include "pico/stdlib.h"
#include <protocol.hpp>
#include <pico/multicore.h>
#include <optional>


// let's see whether this survives multicore
static Lights lights;

void
do_static_fun();

std::optional<Command>
try_read_command();

void
lightsUpdateService()
{
    while(true)
    {
        const auto vorher = get_absolute_time();
        lights.update();
        sleep_until(delayed_by_ms(vorher, 1000 / lights.getDesiredTicksPerSecond()));
    }
}

int main() {
    setup_default_uart();
    stdio_init_all();

    lights.init();
    lights.setParty(true);
    // printf("set party mode\n");

    multicore_launch_core1(lightsUpdateService);

    while (true)
    {
        // const auto vorher = get_absolute_time();
        const auto command = try_read_command();
        if(command)
        {
            const auto& value = command->value;
            switch (command->type)
            {
                case Command::Type::headlight:
                    lights.setHeadlight(value);
                    break;
                case Command::Type::indicator:
                    if (command->value & 0b10)
                        lights.setIndicatorLeft(value & 0b01);
                    else
                        lights.setIndicatorRight(value & 0b01);
                    break;
                case Command::Type::brake:
                    lights.setBrake(value);
                    break;
                case Command::Type::party:
                    lights.setParty(value);
                    break;
                default:
                    // nothing
                    printf("Got invalid command %d\n", command->type);
                    break;
            }
        }
        // lights.update();
        // sleep_until(delayed_by_ms(vorher, 1000 / lights.getDesiredTicksPerSecond()));
    };

    return 0;
}

std::optional<Command>
try_read_command()
{
    std::array <uint8_t, sizeof(Command)> buf  = {{}};
    uint8_t p = 0;

    while (p < buf.size())
    {
        auto maybeChar = getchar_timeout_us(1000 * 100);
        if (maybeChar == PICO_ERROR_TIMEOUT)
            return std::nullopt;

        buf[p] = maybeChar;
        p++;
    }

    return *reinterpret_cast<Command*>(&buf);
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