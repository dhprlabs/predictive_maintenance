#define ANALOG_PIN A0

const float ADC_MAX = 1023.0;
const float PRESSURE_MAX_BAR = 10.0;

unsigned long lastRead = 0;
const unsigned long READ_INTERVAL = 5000;

void setup() 
{
  Serial.begin(9600);
  while (!Serial) delay(10);
  Serial.println("Pressure Sensor (A0) - Started");
  Serial.println("Note: Calibrate ADC->pressure using known points for your sensor.");
}

void loop() 
{
  if (millis() - lastRead < READ_INTERVAL) return;
  lastRead = millis();

  int adcValue = analogRead(ANALOG_PIN);
  float pressureBar = (float)adcValue * (PRESSURE_MAX_BAR / ADC_MAX);

  Serial.print("ADC: "); Serial.print(adcValue);
  Serial.print("  Pressure (bar): "); Serial.println(pressureBar, 3);
  Serial.println("---");
}
