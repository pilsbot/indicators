#pragma once
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
    Headlight(uint8_t intensity) : Command(Type::headlight, intensity){};

    uint8_t getIntensity()
    {
        return value;
    }
};

struct IndicatorLeft : public Command
{
    IndicatorLeft(bool activate) : Command(Type::indicator, activate ? 0b11 : 0b10){};
};

struct IndicatorRight : public Command
{
    IndicatorRight(bool activate) : Command(Type::indicator, activate ? 0b01 : 0b00){};
};

}