#include <ModbusMaster.h>
#include <SoftwareSerial.h>

#define RE_DE 4
#define SLAVE_ID 1 

#define L12_VOLTAGE_REG   28 
#define L23_VOLTAGE_REG   32 
#define L31_VOLTAGE_REG   36 
#define L1_CURRENT_REG    44
#define L2_CURRENT_REG    48
#define L3_CURRENT_REG    52
#define SYS_FREQUENCY_REG 70
#define SYS_PF_REG        66
#define TOTAL_KW_REG      84
#define TOTAL_KVA_REG     100

SoftwareSerial RS485Serial(2, 3); 
ModbusMaster node;

void preTransmission() 
{
  digitalWrite(RE_DE, HIGH);
}

void postTransmission() 
{
  digitalWrite(RE_DE, LOW);
}

uint8_t readModbusFloat(uint16_t registerAddress, float &destinationFloat) 
{
  uint8_t result = node.readHoldingRegisters(registerAddress, 2);

  if (result == node.ku8MBSuccess) 
  {
    uint32_t raw_value = ((uint32_t)node.getResponseBuffer(1) << 16) | node.getResponseBuffer(0);
    memcpy(&destinationFloat, &raw_value, sizeof(destinationFloat));
  }

  return result;
}

void setup() 
{
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW);

  Serial.begin(9600);
  RS485Serial.begin(9600);
  node.begin(SLAVE_ID, RS485Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  Serial.println("Starting MFM 13-M1 Comprehensive Modbus Reader...");
}

void loop() 
{
  uint8_t result;
  float value;

  Serial.println("----------------------------------------");
  delay(250);

  result = readModbusFloat(L12_VOLTAGE_REG, value);
  if (result == node.ku8MBSuccess) { Serial.print("Voltage (L1-L2):   "); Serial.print(value); Serial.println(" V"); } 
  else { Serial.print("Failed to read L1-L2 Voltage. Error: 0x"); Serial.println(result, HEX); }
  delay(250);

  result = readModbusFloat(L23_VOLTAGE_REG, value);
  if (result == node.ku8MBSuccess) { Serial.print("Voltage (L2-L3):   "); Serial.print(value); Serial.println(" V"); }
  else { Serial.print("Failed to read L2-L3 Voltage. Error: 0x"); Serial.println(result, HEX); }
  delay(250);

  result = readModbusFloat(L31_VOLTAGE_REG, value);
  if (result == node.ku8MBSuccess) { Serial.print("Voltage (L3-L1):   "); Serial.print(value); Serial.println(" V"); }
  else { Serial.print("Failed to read L3-L1 Voltage. Error: 0x"); Serial.println(result, HEX); }
  delay(250);

  Serial.println();

  result = readModbusFloat(L1_CURRENT_REG, value);
  if (result == node.ku8MBSuccess) { Serial.print("Current (L1):      "); Serial.print(value, 3); Serial.println(" A"); }
  else { Serial.print("Failed to read L1 Current. Error: 0x"); Serial.println(result, HEX); }
  delay(250);

  result = readModbusFloat(L2_CURRENT_REG, value);
  if (result == node.ku8MBSuccess) { Serial.print("Current (L2):      "); Serial.print(value, 3); Serial.println(" A"); }
  else { Serial.print("Failed to read L2 Current. Error: 0x"); Serial.println(result, HEX); }
  delay(250);
  
  result = readModbusFloat(L3_CURRENT_REG, value);
  if (result == node.ku8MBSuccess) { Serial.print("Current (L3):      "); Serial.print(value, 3); Serial.println(" A"); }
  else { Serial.print("Failed to read L3 Current. Error: 0x"); Serial.println(result, HEX); }
  delay(250);

  Serial.println();

  result = readModbusFloat(SYS_FREQUENCY_REG, value);
  if (result == node.ku8MBSuccess) { Serial.print("Frequency:         "); Serial.print(value); Serial.println(" Hz"); }
  else { Serial.print("Failed to read Frequency. Error: 0x"); Serial.println(result, HEX); }
  delay(250);

  result = readModbusFloat(SYS_PF_REG, value);
  if (result == node.ku8MBSuccess) { Serial.print("System PF:         "); Serial.println(value, 3); }
  else { Serial.print("Failed to read System PF. Error: 0x"); Serial.println(result, HEX); }
  delay(250);
  
  result = readModbusFloat(TOTAL_KW_REG, value);
  if (result == node.ku8MBSuccess) { Serial.print("Total Power (kW):  "); Serial.println(value, 3); }
  else { Serial.print("Failed to read Total kW. Error: 0x"); Serial.println(result, HEX); }
  delay(250);

  result = readModbusFloat(TOTAL_KVA_REG, value);
  if (result == node.ku8MBSuccess) { Serial.print("Total Power (kVA): "); Serial.println(value, 3); }
  else { Serial.print("Failed to read Total kVA. Error: 0x"); Serial.println(result, HEX); }

  delay(5000);
}