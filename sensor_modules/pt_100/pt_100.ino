#include <Adafruit_MAX31865.h>

Adafruit_MAX31865 rtdSensor = Adafruit_MAX31865(10, 11, 12, 13);

#define RREF 430.0
#define RNOMINAL 100.0   

unsigned long lastReadTime = 0;
const unsigned long readInterval = 1000; 

void setup() 
{
  Serial.begin(9600);
  while (!Serial) delay(10);

  Serial.println("System Booting...");
  rtdSensor.begin(MAX31865_3WIRE);
  Serial.println("MAX31865 RTD Sensor Initialized.");
}

void loop() 
{
  unsigned long now = millis();

  if (now - lastReadTime >= readInterval) 
  {
    float temperatureC = rtdSensor.temperature(RNOMINAL, RREF);
    float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;

    uint16_t rtdVal = rtdSensor.readRTD();

    Serial.print("RTD Raw Value: ");
    Serial.println(rtdVal);
    Serial.print("Temperature: ");
    Serial.print(temperatureC);
    Serial.print(" °C  |  ");
    Serial.print(temperatureF);
    Serial.println(" °F");

    if (isnan(temperatureC)) 
    {
      Serial.println("Failed to read temperature from RTD sensor.");
    }

    lastReadTime = now;
  }
}
