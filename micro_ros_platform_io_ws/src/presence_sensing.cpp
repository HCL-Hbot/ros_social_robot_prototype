#include <Arduino.h>

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH); // LED aan
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);  // LED uit
    delay(1000);
}
