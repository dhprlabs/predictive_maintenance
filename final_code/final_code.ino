//==============================================================================
// 1. LIBRARIES
//==============================================================================
#include <Adafruit_MAX31865.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <math.h> 

//==============================================================================
// 2. GLOBAL CONFIGURATION & CONSTANTS
//==============================================================================

// --- LoRa Configuration ---
#define LORA_BAUDRATE 9600
#define LoraSerial Serial1
const int LORA_TARGET_ADDRESS = 2;        
const unsigned long SEND_INTERVAL = 15000; 

// --- PT100 Temperature Sensor Configuration ---
#define RREF 431.0      
#define RNOMINAL 100.0  
Adafruit_MAX31865 rtd = Adafruit_MAX31865(10, 11, 12, 13);

// --- Analog Pressure Sensor Configuration ---
#define ANALOG_PIN A0
const float ADC_MIN = 0.0;
const float ADC_MAX = 1023.0;
const float PRESSURE_MIN = 0.0;
const float PRESSURE_MAX = 10.0; 

// --- MQ135 Gas Sensor Configuration ---
#define MQ135_PIN A1 

// --- Modbus (Shared) Configuration ---
#define RE_DE 4 
SoftwareSerial RS485Serial(2, 3); 
ModbusMaster powerMeterNode;  
ModbusMaster vibrationNode;   

// --- MFM 13-M1 (Power Meter) Modbus Configuration ---
#define POWER_METER_SLAVE_ID 1
#define L12_VOLTAGE_REG 28
#define L1_CURRENT_REG 44
#define SYS_FREQUENCY_REG 70
#define SYS_PF_REG 66
#define TOTAL_KW_REG 84

// --- Vibration Sensor Modbus Configuration ---
#define VIBRATION_SLAVE_ID 0x50 
#define ACCEL_SCALING_MULTIPLIER 0.00048828125
#define SPEED_SCALING_DIVISOR 10.0
#define DISPL_SCALING_DIVISOR 10.0
#define VIB_TEMP_REG 0x40
#define X_ACCEL_REG 0x34
#define X_SPEED_REG 0x3A
#define X_DISPL_REG 0x41

//==============================================================================
// 3. GLOBAL VARIABLES
//==============================================================================
unsigned long lastSendTime = 0;

//==============================================================================
// 4. SETUP FUNCTION
//==============================================================================
void setup() 
{
    Serial.begin(9600);
    while (!Serial) delay(10);
    Serial.println("--- System Booting ---");

    rtd.begin(MAX31865_3WIRE);
    Serial.println("MAX31865 RTD Sensor Initialized.");

    pinMode(RE_DE, OUTPUT);
    digitalWrite(RE_DE, LOW);
    RS485Serial.begin(9600);
    
    powerMeterNode.begin(POWER_METER_SLAVE_ID, RS485Serial);
    vibrationNode.begin(VIBRATION_SLAVE_ID, RS485Serial);
    
    powerMeterNode.preTransmission(preTransmission);
    powerMeterNode.postTransmission(postTransmission);
    vibrationNode.preTransmission(preTransmission);
    vibrationNode.postTransmission(postTransmission);
    
    Serial.println("Modbus Initialized for multiple slaves.");

    LoraSerial.begin(LORA_BAUDRATE);
    delay(100);
    Serial.println("Configuring LoRa Module...");
    sendAT("AT+OPMODE=1", 500);
    sendAT("ATZ", 2000);
    sendAT("AT+ADDRESS=100", 500);
    sendAT("AT+BAND=865000000", 500);
    sendAT("AT+PARAMETER=9,7,1,12", 500);
    Serial.println("--- Initialization Complete. Starting main loop. ---");
}

//==============================================================================
// 5. MAIN LOOP
//==============================================================================
void loop() 
{
    if (millis() - lastSendTime >= SEND_INTERVAL) 
    {
        lastSendTime = millis();
        collectAndSendData();
    }

    while (LoraSerial.available()) 
    {
        String resp = LoraSerial.readStringUntil('\n');
        resp.trim();
        if (resp.length()) {
            Serial.print("[LoRa RX] ");
            Serial.println(resp);
        }
    }
}

//==============================================================================
// 6. SENSOR DATA COLLECTION & TRANSMISSION
//==============================================================================
void collectAndSendData() 
{
    char dataPayload[512]; 
    char tempBuffer[256];   
    uint8_t fault = rtd.readFault();

    if (fault) 
    {
      Serial.print("PT100 Fault 0x"); Serial.println(fault, HEX);
      if (fault & MAX31865_FAULT_HIGHTHRESH) Serial.println("RTD High Threshold");
      if (fault & MAX31865_FAULT_LOWTHRESH) Serial.println("RTD Low Threshold");
      if (fault & MAX31865_FAULT_REFINLOW) Serial.println("REFIN- > 0.85 x Bias");
      if (fault & MAX31865_FAULT_REFINHIGH) Serial.println("REFIN- < 0.85 x Bias");
      if (fault & MAX31865_FAULT_RTDINLOW) Serial.println("RTDIN- < 0.85 x Bias");
      if (fault & MAX31865_FAULT_OVUV) Serial.println("Under/Over voltage");
      rtd.clearFault();
    }
    
    float pt100_temp = rtd.temperature(RNOMINAL, RREF);
    int adcValue = analogRead(ANALOG_PIN);
    float pressure = (float)adcValue * (PRESSURE_MAX / ADC_MAX);
    int mq135_raw = analogRead(MQ135_PIN);
    char pt100_str[8], pres_str[8];
    dtostrf(pt100_temp, 4, 2, pt100_str);
    dtostrf(pressure, 4, 2, pres_str);

    snprintf(dataPayload, sizeof(dataPayload), "PT100:%s,PRES:%s,MQ135:%d",
        (isnan(pt100_temp) || fault) ? "ERR" : pt100_str,
        isnan(pressure) ? "ERR" : pres_str,
        mq135_raw
    );
    
    Serial.println("\n--- Reading Sensor Data ---");
    Serial.print("Payload so far: ");
    Serial.println(dataPayload);

    Serial.println("--- Querying Power Meter (ID: 1) ---");
    float v12 = readModbusFloat(powerMeterNode, L12_VOLTAGE_REG);
    float i1 = readModbusFloat(powerMeterNode, L1_CURRENT_REG);
    float freq = readModbusFloat(powerMeterNode, SYS_FREQUENCY_REG);
    float pf = readModbusFloat(powerMeterNode, SYS_PF_REG);
    float kw = readModbusFloat(powerMeterNode, TOTAL_KW_REG);

    char v12_str[8], i1_str[8], freq_str[8], pf_str[8], kw_str[8];
    dtostrf(v12, 4, 1, v12_str);
    dtostrf(i1, 4, 2, i1_str);
    dtostrf(freq, 4, 1, freq_str);
    dtostrf(pf, 3, 2, pf_str);
    dtostrf(kw, 4, 2, kw_str);
    
    snprintf(tempBuffer, sizeof(tempBuffer), ",V12:%s,I1:%s,HZ:%s,PF:%s,KW:%s",
        isnan(v12) ? "ERR" : v12_str,
        isnan(i1) ? "ERR" : i1_str,
        isnan(freq) ? "ERR" : freq_str,
        isnan(pf) ? "ERR" : pf_str,
        isnan(kw) ? "ERR" : kw_str
    );

    strcat(dataPayload, tempBuffer);
    Serial.print("Payload so far: ");
    Serial.println(dataPayload);

    Serial.println("--- Querying Vibration Sensor (ID: 80) ---");
    float vib_temp = readModbusShort(vibrationNode, VIB_TEMP_REG) / 100.0;
    float x_accel = readModbusShort(vibrationNode, X_ACCEL_REG) * ACCEL_SCALING_MULTIPLIER;
    float x_speed = readModbusShort(vibrationNode, X_SPEED_REG) / SPEED_SCALING_DIVISOR;
    float x_displ = readModbusShort(vibrationNode, X_DISPL_REG) / DISPL_SCALING_DIVISOR;
    
    char vib_t_str[8], x_a_str[8], x_s_str[8], x_d_str[8];
    dtostrf(vib_temp, 4, 1, vib_t_str);
    dtostrf(x_accel, 3, 2, x_a_str);
    dtostrf(x_speed, 4, 2, x_s_str);
    dtostrf(x_displ, 4, 2, x_d_str);

    snprintf(tempBuffer, sizeof(tempBuffer), ",VIB_T:%s,X_A:%s,X_S:%s,X_D:%s",
        isnan(vib_temp) ? "ERR" : vib_t_str,
        isnan(x_accel) ? "ERR" : x_a_str,
        isnan(x_speed) ? "ERR" : x_s_str,
        isnan(x_displ) ? "ERR" : x_d_str
    );

    strcat(dataPayload, tempBuffer);

    Serial.println("---------------------------------");
    Serial.print("Final Payload: ");
    Serial.println(dataPayload);
    sendData(LORA_TARGET_ADDRESS, dataPayload);
    Serial.println("---------------------------------");
}

//==============================================================================
// 7. MODBUS HELPER FUNCTIONS
//==============================================================================
void preTransmission() 
{
    digitalWrite(RE_DE, HIGH);
}

void postTransmission() 
{
    delayMicroseconds(50);
    digitalWrite(RE_DE, LOW);
}

float readModbusFloat(ModbusMaster &node, uint16_t reg) 
{
    float result_val = NAN;
    uint8_t result = node.readHoldingRegisters(reg, 2);

    if (result == node.ku8MBSuccess) 
    {
        uint32_t raw_value = ((uint32_t)node.getResponseBuffer(1) << 16) | node.getResponseBuffer(0);
        memcpy(&result_val, &raw_value, sizeof(result_val));
    } 
    else 
    {
        Serial.print("Modbus read float failed at reg ");
        Serial.print(reg);
        Serial.print(" with error: 0x");
        Serial.println(result, HEX);
    }

    delay(250); 
    return result_val;
}

float readModbusShort(ModbusMaster &node, uint16_t reg) 
{
    float result_val = NAN;
    uint8_t result = node.readHoldingRegisters(reg, 1);
    if (result == node.ku8MBSuccess) 
    {
        result_val = node.getResponseBuffer(0);
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

//==============================================================================
// 8. LORA HELPER FUNCTIONS
//==============================================================================
void sendAT(const char* cmd, int delayMs) 
{
    LoraSerial.print(cmd);
    LoraSerial.print("\r\n");
    Serial.print("[AT CMD] ");
    Serial.println(cmd);

    long startTime = millis();

    while (millis() - startTime < delayMs) 
    {
        if (LoraSerial.available()) 
        {
            String r = LoraSerial.readStringUntil('\n');
            r.trim();
            if (r.length()) 
            {
                Serial.print("↳ [AT RESP] ");
                Serial.println(r);
            }
        }
    }
}

void sendData(int address, const char* data) 
{
    int len = strlen(data);
    LoraSerial.print("AT+SEND=");
    LoraSerial.print(address);
    LoraSerial.print(",");
    LoraSerial.print(len);
    LoraSerial.print(",");
    LoraSerial.println(data);

    Serial.print("[LoRa TX] addr=");
    Serial.print(address);
    Serial.print(" len=");
    Serial.print(len);
    Serial.print(" msg=\"");
    Serial.print(data);
    Serial.println("\"");

    long startTime = millis();
    
    while (millis() - startTime < 1000) 
    {
        if (LoraSerial.available()) 
        {
            String r = LoraSerial.readStringUntil('\n');
            r.trim();
            if (r.length()) 
            {
                Serial.print("↳ [LoRa TX RESP] ");
                Serial.println(r);
            }
        }
    }
}