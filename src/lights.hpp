#pragma once

#include <config.hpp>

class Lights
{
    struct State
    {
        bool indicator_left = false;
        bool indicator_right = false;
        bool brake = false;
        bool headlight = false;
        bool party = false;
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
    };

    static constexpr Color headlight = {0xFF, 0xFF, 0xFF};
    static constexpr Color taillight = {0x30,    0,    0};
    static constexpr Color indicator = {0xFF, 0xFF,    0};
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


    void setIndicatorLeft(bool val = true);
    void setIndicatorRight(bool val = true);
    void setBrake(bool val = true);
    void setHeadlight(bool val = true);
    void setParty(bool val = true);

    void update();
private:
    void setColorSide(const LightPos& pos, const Color color, const Direction dir, const uint8_t num = 0xFF, bool overwrite_rest = false);
};