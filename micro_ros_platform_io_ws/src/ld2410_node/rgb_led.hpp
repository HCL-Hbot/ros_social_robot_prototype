#ifndef RGB_LED_HPP_
#define RGB_LED_HPP_

#include <Adafruit_NeoPixel.h>

/**
 * @brief Class to control a RGB LED
 * 
 */
class RGBLed
{
public:
    /**
     * @brief Construct a new RGBLed object
     * 
     * @param pin  The pin number where the RGB LED is connected
     */
    RGBLed(uint8_t pin);
    
    /**
     * @brief Set the Color object
     * 
     * @param r     Red value
     * @param g     Green value
     * @param b     Blue value
     */
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    
    /**
     * @brief Set the Brightness object
     * 
     * @param brightness    Brightness value 
     */
    void setBrightness(uint8_t brightness);
    
    /**
     * @brief Show the color
     * 
     */
    void show();

private:
    Adafruit_NeoPixel strip;
};

#endif // RGB_LED_HPP_
