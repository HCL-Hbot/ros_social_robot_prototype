#include "rgb_led.hpp"

RGBLed::RGBLed(uint8_t pin)
: strip(1, pin, NEO_GRB + NEO_KHZ800)
{
    strip.begin();
    strip.setPixelColor(0, 0); // Set the color to black (off)
    strip.show();
}

void RGBLed::setColor(uint8_t r, uint8_t g, uint8_t b)
{
    strip.setPixelColor(0, strip.Color(r, g, b));
}

void RGBLed::setBrightness(uint8_t brightness)
{
    strip.setBrightness(brightness);
}

void RGBLed::show()
{
    strip.show();
}
