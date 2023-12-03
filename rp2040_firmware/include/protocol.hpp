#pragma once
#include "types.h"
#include <stddef.h>

struct __attribute__((packed)) Command
{
    enum Type : uint8_t
    {
        invalid = 0,
        headlight,
        indicator,
        brake,
        party,
    } type;
    uint8_t value;

    Command() = default;
    Command(Type type, uint8_t value) : type(type), value(value){};
};

namespace cmd
{
// TODO: finish this? Lot of beautiful overhead for such a simple protocol
struct Headlight : public Command
{
    Headlight(const ColorIntensity intensity) : Command(Type::headlight, intensity){};

    uint8_t getIntensity()
    {
        return value;
    }
};

struct IndicatorLeft : public Command
{
    // last bit determines indicator side
    /**
     * \param[in] on zero is off, everything else on with given intensity
    */
    IndicatorLeft(const ColorIntensity intensity) : Command(Type::indicator, intensity | 0b1){};
};

struct IndicatorRight : public Command
{
    // last bit determines indicator side
    /**
     * \param[in] on zero is off, everything else on with given intensity
    */
    IndicatorRight(const ColorIntensity intensity) : Command(Type::indicator, intensity & ~0b1){};
};

struct Brake : public Command
{
    // last bit determines indicator side
    Brake(const ColorIntensity intensity) : Command(Type::brake, intensity){};
};

struct Party : public Command
{
    Party(const ColorIntensity intensity) : Command(Type::party, intensity){};
};
}