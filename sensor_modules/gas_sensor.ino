#define MQ135_PIN A1

const float ADC_MAX = 1023.0;
const float VCC = 5.0; 

const float RL_VALUE = 10.0; 

const bool AUTO_CALIBRATE = false;
float R0 = 10.0; 

unsigned long lastRead = 0;
const unsigned long READ_INTERVAL = 5000;

float adcToVoltage(int adc) 
{
  return ((float)adc / ADC_MAX) * VCC;
}

float adcToRs(int adc) 
{
  float vrl = adcToVoltage(adc);
  if (vrl <= 0.0) return NAN;
  float rs = (VCC - vrl) * (RL_VALUE) / vrl; 
  return rs;
}

float calibrateR0(int samples = 50) 
{
  Serial.println("Calibrating R0 (assumes clean air)...");
  double acc = 0;

  for (int i = 0; i < samples; ++i) 
  {
    int adc = analogRead(MQ135_PIN);
    float rs = adcToRs(adc);
    if (!isnan(rs)) acc += rs;
    delay(100);
  }

  float avg = (float)(acc / samples);
  Serial.print("Calibration complete. R0 = "); Serial.print(avg, 3); Serial.println(" kOhm");
  return avg;
}

void setup() 
{
  Serial.begin(9600);
  while (!Serial) delay(10);
  Serial.println("MQ135 Gas Sensor (A1) - Started");
  Serial.println("Note: Raw ADC and Rs (kOhm) are printed. To get PPM you must calibrate and apply the sensor curve.");

  if (AUTO_CALIBRATE) 
  {
    R0 = calibrateR0();
  }
}

void loop() 
{
  if (millis() - lastRead < READ_INTERVAL) return;
  lastRead = millis();

  int adc = analogRead(MQ135_PIN);
  float v = adcToVoltage(adc);
  float rs = adcToRs(adc);

  Serial.print("ADC: "); Serial.print(adc);
  Serial.print("  V: "); Serial.print(v, 3);
  Serial.print(" V  Rs: "); Serial.print(rs, 3); Serial.print(" kOhm");
  
  if (!isnan(R0) && R0 > 0) 
  {
    Serial.print("  Rs/R0: "); Serial.print(rs / R0, 3);
  }
  
  Serial.println();
  Serial.println("Estimated PPM: N/A - implement calibration curve using sensor datasheet");
  Serial.println("---");
}
