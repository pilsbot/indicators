#include <inttypes.h>

typedef uint8_t PinNr;
typedef uint8_t ColorIntensity;
typedef uint32_t Pixel;

static constexpr PinNr internalLED = 16;
static constexpr PinNr externalStrip = 14;
static constexpr ColorIntensity max_brightness = 0xA0;