#pragma once

#include "types.h"

extern "C"
{
#include "neo_config.h"
}

struct LightPos
{
    uint8_t offs;
    uint8_t len;
};

static constexpr LightPos front_left  = {0, 32};
static constexpr LightPos front_right = {front_left.offs + front_left.len, 32};
static constexpr LightPos rear_right  = {front_right.offs + front_right.len, 32};
static constexpr LightPos rear_left   = {rear_right.offs + rear_right.len, 32};

static constexpr uint8_t num_pixels = rear_left.offs + rear_left.len;

static_assert(num_pixels <= NEOC_NOF_LEDS_IN_LANE, "Ugly C define is not correct");