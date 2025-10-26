#include <Adafruit_MAX31865.h>

#define RREF 431.0     
#define RNOMINAL 100.0 

Adafruit_MAX31865 rtd = Adafruit_MAX31865(10, 11, 12, 13);

void setup() 
{
  Serial.begin(9600);
  Serial.println("System Booting...");
  rtd.begin(MAX31865_3WIRE); 
  Serial.println("MAX31865 RTD Sensor Initialized.");
}

void loop() 
{
  uint16_t rtdRaw = rtd.readRTD();   
  Serial.print("RTD Raw Value: ");
  Serial.println(rtdRaw);

  float ratio = rtdRaw / 32768.0;     
  float resistance = (RREF * ratio) - 318;
  Serial.print("Ratio = ");
  Serial.println(ratio, 8);
  Serial.print("Resistance = ");
  Serial.print(resistance, 2);
  Serial.println(" ohms");

  float temperature = rtd.temperature(RNOMINAL, (RREF - 318) + 45);
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  delay(1000);
}
