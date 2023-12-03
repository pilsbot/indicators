#pragma once

#include <array>

namespace indicators
{

enum class Topic
{
    indicatorLeft = 0,
    indicatorRight,
    brake,
    headlight,
    party,
};

// ugly with that 5
static constexpr std::array<Topic, 5> topics {
    Topic::indicatorLeft,
    Topic::indicatorRight,
    Topic::brake,
    Topic::headlight,
    Topic::party,
};

constexpr const char*
getTopicName(const Topic& topic)
{
    switch (topic)
    {
        case Topic::indicatorLeft:
            return "lighting/indicator/left";
        case Topic::indicatorRight:
            return "lighting/indicator/right";
        case Topic::brake:
            return "lighting/brake";
        case Topic::headlight:
            return "lighting/headlight";
        case Topic::party:
            return "lighting/party";
        default:
            return "lighting/invalid_shit_happend_AUTCH";
    }
}

}