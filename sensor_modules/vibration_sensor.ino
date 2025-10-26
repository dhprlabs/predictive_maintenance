#include <SoftwareSerial.h>
#include <ModbusMaster.h>
#include <math.h>

#define RE_DE 4

SoftwareSerial RS485Serial(2, 3); 
ModbusMaster vibrationNode;

#define VIBRATION_SLAVE_ID 0x50 

#define VIB_TEMP_REG 0x40
#define X_ACCEL_REG 0x34
#define X_SPEED_REG 0x3A
#define X_DISPL_REG 0x41

const float ACCEL_SCALING_MULTIPLIER = 0.00048828125;
const float SPEED_SCALING_DIVISOR = 10.0;
const float DISPL_SCALING_DIVISOR = 10.0;

unsigned long lastRead = 0;
const unsigned long READ_INTERVAL = 5000;

void preTransmission() 
{
  digitalWrite(RE_DE, HIGH);
}

void postTransmission() 
{
  delayMicroseconds(50);
  digitalWrite(RE_DE, LOW);
}

float readModbusShort(ModbusMaster &node, uint16_t reg) 
{
  float result_val = NAN;
  uint8_t result = node.readHoldingRegisters(reg, 1);

  if (result == node.ku8MBSuccess) 
  {
    int16_t raw = (int16_t)node.getResponseBuffer(0);
    result_val = (float)raw;
  } 
  else 
  {
    Serial.print("Modbus read short failed at reg ");
    Serial.print(reg);
    Serial.print(" with error: 0x");
    Serial.println(result, HEX);
  }

  delay(250);
  return result_val;
}

void setup() 
{
  Serial.begin(9600);
  while (!Serial) delay(10);
  Serial.println("Vibration Sensor (Modbus) - Started");

  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW);

  RS485Serial.begin(9600);
  vibrationNode.begin(VIBRATION_SLAVE_ID, RS485Serial);
  vibrationNode.preTransmission(preTransmission);
  vibrationNode.postTransmission(postTransmission);
  Serial.println("Modbus RS485 initialized for Vibration Sensor (ID 80)");
}

void loop() 
{
  if (millis() - lastRead < READ_INTERVAL) return;
  lastRead = millis();

  Serial.println("--- Querying Vibration Sensor ---");
  float vib_temp = readModbusShort(vibrationNode, VIB_TEMP_REG) / 100.0;
  float x_accel = readModbusShort(vibrationNode, X_ACCEL_REG) * ACCEL_SCALING_MULTIPLIER;
  float x_speed = readModbusShort(vibrationNode, X_SPEED_REG) / SPEED_SCALING_DIVISOR;
  float x_displ = readModbusShort(vibrationNode, X_DISPL_REG) / DISPL_SCALING_DIVISOR;

  if (isnan(vib_temp)) Serial.println("Vib Temp: ERR"); else { Serial.print("Vib Temp (C): "); Serial.println(vib_temp, 2); }
  if (isnan(x_accel)) Serial.println("X Accel: ERR"); else { Serial.print("X Accel (g): "); Serial.println(x_accel, 4); }
  if (isnan(x_speed)) Serial.println("X Speed: ERR"); else { Serial.print("X Speed: "); Serial.println(x_speed, 4); }
  if (isnan(x_displ)) Serial.println("X Displ: ERR"); else { Serial.print("X Displ: "); Serial.println(x_displ, 4); }

  Serial.println("---");
}