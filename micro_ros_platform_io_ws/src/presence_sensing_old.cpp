#include <Arduino.h>
#include <ld2410.h>

// See e.g. https://community.element14.com/technologies/embedded/b/blog/posts/experimenting-with-microwave-based-sensors-for-presence-detection
// https://github.com/ncmreynolds/ld2410/blob/main/examples/basicSensor/basicSensor.ino
// Pico pinout: https://www.14core.com/wp-content/uploads/2022/11/Raspberry-Pi-PICO-Pinout-Diagram.jpeg

ld2410 radar;
uint32_t lastReading = 0;

//links
#define L_RADAR_RX 40
#define L_RADAR_TX 39
//rechts
#define R_RADAR_RX 38
#define R_RADAR_TX 37

//https://github.com/2Grey/s3km1110/blob/main/examples/main.cpp

void setup(void)
{
  Serial.begin(115200); // USB
  radar.debug(Serial); //Uncomment to show debug information from the library on the Serial Monitor. By default this does not show sensor reads as they are very frequent.
 
  // UART2, with pins 16 (RX) and 17 (TX), is the only UART that can be used for external communication with other devices 
  //Serial2.begin(256000, SERIAL_8N1, L_RADAR_RX, L_RADAR_TX);
  Serial2.begin(256000, SERIAL_8N1, L_RADAR_RX, L_RADAR_TX);

  delay(500);
  Serial.print(F("\nLD2410 radar sensor initialising: "));
  if(radar.begin(Serial2))
  {
    Serial.println(F("OK"));
  }
  else
  {
    Serial.println(F("not connected"));
  }
}


void loop()
{
  radar.read();
  if(radar.isConnected() && millis() - lastReading > 500)  //Report every 1000ms
  {
    lastReading = millis();
    
    Serial.println("Frame Data");
    FrameData fd = radar.getFrameData();

    for(int i = 0; i<fd.length; ++i)
    {
      if(fd.data[i] < 0x10)
			{
				Serial.print('0');
			}
      Serial.print(fd.data[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println("Done reading");
    
    if(radar.presenceDetected())
    {
      Serial.println("Check target");
      if(radar.stationaryTargetDetected())
      {
        Serial.print(F("Stationary target: "));
        Serial.print(radar.stationaryTargetDistance()); //het blijkt dat hierin distance target in zit..
        Serial.print(F("cm energy:"));
        Serial.println(radar.stationaryTargetEnergy());
      }
      if(radar.movingTargetDetected())
      {
        Serial.print(F("Moving target: "));
        Serial.print(radar.movingTargetDistance()); //het blijkt dat hierin eigenlijk stationary in zit
        Serial.print(F("cm energy:"));
        Serial.println(radar.movingTargetEnergy());
      }


    }
    else
    {
      Serial.println(F("No target"));
    }
  }
}