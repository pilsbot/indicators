#pragma once

#include <config.hpp>

class Lights
{
    struct State
    {
        ColorIntensity indicator_left = 0;
        ColorIntensity indicator_right = 0;
        ColorIntensity brake = 0;
        ColorIntensity headlight = 0;
        ColorIntensity party = 0;
    } state;

    enum class Direction
    {
        forward,
        reverse
    };

    struct Color
    {
        ColorIntensity r;
        ColorIntensity g;
        ColorIntensity b;

        Color
        operator* (const ColorIntensity x) const;
    };

    static constexpr Color headlight = {0xFF, 0xFF, 0xFF};
    static constexpr Color taillight = {0x30,    0,    0};
    static constexpr Color indicator = {0xFF, 0xA0,    0};
    static constexpr Color brake     = {0xFF,    0,    0};
    static constexpr Color party     = {0xAA,    0, 0xBB};
    static constexpr Color off       = {   0,    0,    0};

    static constexpr uint8_t ticks_per_indication = 0b0100000;

    uint8_t tick = 0;
public:

    void init();
    void clear();
    void blinkInfo(const Color color, const bool reverse = false);

    static constexpr uint16_t
    getDesiredTicksPerSecond()
    {
        return ticks_per_indication * 2;
    }

    void setIndicatorLeft(ColorIntensity val = 0xFF);
    void setIndicatorRight(ColorIntensity val = 0xFF);
    void setBrake(ColorIntensity val = 0xFF);
    void setHeadlight(ColorIntensity val = 0xFF);
    void setParty(ColorIntensity val = 0xFF);

    void update();
private:
    void setColorSide(const LightPos& pos, const Color color, const Direction dir, const uint8_t num = 0xFF, bool overwrite_rest = false);
};