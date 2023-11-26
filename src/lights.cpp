#include "lights.hpp"
extern "C"
{
#include "../lib/NeoPixel.h"
}
#include <limits> // no limits

Lights::Color
Lights::Color::operator* (const ColorIntensity x) const
{
    return Color {
        static_cast<ColorIntensity>((r * x) / std::numeric_limits<ColorIntensity>::max()),
        static_cast<ColorIntensity>((g * x) / std::numeric_limits<ColorIntensity>::max()),
        static_cast<ColorIntensity>((b * x) / std::numeric_limits<ColorIntensity>::max())
    };
}

void Lights::init()
{
    // should be a singleton, but you pay attention, do you??
    NEO_Init();
}

void Lights::clear()
{
    tick = 0;
    NEO_ClearAllPixel();
    NEO_TransferPixels();
}

void Lights::blinkInfo(const Color color, const bool reverse)
{
    if(reverse)
        NEO_SetPixelRGB(0, front_left.len - (tick+1), color.r, color.g, color.b);
    else
        NEO_SetPixelRGB(0, tick, color.r, color.g, color.b);

    if(++tick > front_left.len)
    {
        tick = 0;
        for(uint8_t i = 0; i <= front_left.len; i++)
        {
            NEO_ClearPixel(0, i);
        }
    }

    NEO_TransferPixels();
}

void Lights::setIndicatorLeft(ColorIntensity val)
{
    tick = 0; state.indicator_left = val;
};
void Lights::setIndicatorRight(ColorIntensity val)
{
    tick = 0; state.indicator_right = val;
};
void Lights::setBrake(ColorIntensity val)
{
    state.brake = val;
};
void Lights::setHeadlight(ColorIntensity val)
{
    state.headlight = val;
};
void Lights::setParty(ColorIntensity val)
{
    state.party = val;
};

void Lights::update()
{
    bool filling = (tick & ticks_per_indication);
    //indicators
    if(filling)
    {
        setColorSide(front_left , indicator * state.indicator_left,
                     Direction::reverse,  (tick << 1) & 0b111111);
        setColorSide(rear_left  , indicator * state.indicator_left,
                     Direction::forward,  (tick << 1) & 0b111111);

        setColorSide(front_right, indicator * state.indicator_right,
                     Direction::forward, (tick << 1) & 0b111111);
        setColorSide(rear_right , indicator * state.indicator_right,
                     Direction::reverse, (tick << 1) & 0b111111);
    }
    else
    {   //pause between fills or "not indicating"
        setColorSide(front_left, off, Direction::reverse);
        setColorSide(rear_left , off, Direction::forward);

        setColorSide(front_right, off, Direction::forward);
        setColorSide(rear_right , off, Direction::reverse);
    }


    if(state.headlight)
    {   //For headlights, turning signal has priority
        if(!state.indicator_left)
        {
            setColorSide(front_left, headlight * state.headlight,
                         Direction::reverse, front_left.len/2);
            setColorSide(rear_left,  taillight * state.headlight,
                         Direction::forward, rear_left.len/2);
        }
        if(!state.indicator_right)
        {
            setColorSide(front_right, headlight * state.headlight,
                         Direction::forward, front_right.len/2);
            setColorSide(rear_right,  taillight * state.headlight,
                         Direction::reverse, rear_right.len/2);
        }
    }

    if(state.brake)
    {   //brake has prio
        setColorSide(rear_right, brake * state.brake,
                     Direction::reverse, rear_right.len/2, false);
        setColorSide(rear_left , brake * state.brake,
                     Direction::forward, rear_left.len/2, false);
    }

    if(state.party)
    {
        //Wohoo, party!
        const auto scaled_party = party * state.party;
        NEO_SetPixelRGB(0, tick % num_pixels, scaled_party.r, scaled_party.g, scaled_party.b);
    }

    tick++;

    NEO_TransferPixels();
}


void Lights::setColorSide(
    const LightPos& pos,
    const Color color,
    const Direction dir,
    const uint8_t num,
    bool overwrite_rest)
{
    uint8_t len = num > pos.len ? pos.len : num;
    if(dir == Direction::forward)
    {
        for(uint8_t i = pos.offs; i < pos.offs + len; i++) {
            NEO_SetPixelRGB(0, i, color.r, color.g, color.b);
        }
        if(overwrite_rest)
        {
            for(uint8_t i = pos.offs + len; i < pos.offs + pos.len; i++) {
                NEO_ClearPixel(0, i);
            }
        }
    }
    else if(dir == Direction::reverse)
    {
        for(int16_t i = pos.offs + (pos.len - 1); i >= pos.offs + (pos.len - len); i--) {
            NEO_SetPixelRGB(0, i, color.r, color.g, color.b);
        }
        if(overwrite_rest)
        {
            for(int16_t i = pos.offs + (pos.len - len); i >= pos.offs; i--) {
                NEO_SetPixelRGB(0, i, color.r, color.g, color.b);
            }
        }
    }
    // not transferring yet! (but might as well, lol)
}