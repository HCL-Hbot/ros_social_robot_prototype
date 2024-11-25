#include <Arduino.h>
#include <ld2410.h>


ld2410 radar;
uint32_t lastReading = 0;

//links
#define L_RADAR_RX 40
#define L_RADAR_TX 39
//rechts
#define R_RADAR_RX 38
#define R_RADAR_TX 37

void setup(void)
{
  Serial.begin(115200); // USB
  //radar.debug(Serial); //Uncomment to show debug information from the library on the Serial Monitor. By default this does not show sensor reads as they are very frequent.

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
  if(radar.isConnected() && millis() - lastReading > 1000)  //Report every 1000ms
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

    if(fd.data[8] == 0) Serial.println("No target");
    else if(fd.data[8] == 1) Serial.println("Moving only");
    else if(fd.data[8] == 2) Serial.println("Stationary only");
    else if(fd.data[8] == 3) Serial.println("Both moving and stationary");

    Serial.println("Check target");

    Serial.print(F("Stationary target: "));
    Serial.print(fd.data[12] | (fd.data[13] << 8));

    //Serial.print(radar.stationaryTargetDistance()); //het blijkt dat hierin distance target in zit..
    Serial.print(F("cm energy:"));
    Serial.println(fd.data[14]);

    Serial.print(F("Moving target: "));

    Serial.print(fd.data[9] | (fd.data[10] << 8));
    //Serial.print(radar.movingTargetDistance()); //het blijkt dat hierin eigenlijk stationary in zit
    Serial.print(F("cm energy:"));
    Serial.println(fd.data[11]);

    Serial.print(F("Distance: "));
    Serial.print(fd.data[15] | (fd.data[16] << 8));
    Serial.println("cm");

  }
}