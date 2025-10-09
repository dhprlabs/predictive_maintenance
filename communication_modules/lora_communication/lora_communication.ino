#define LORA_BAUDRATE 9600
#define LoraSerial Serial1  

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 10000;
const int targetAddress = 2;

void setup() 
{
  Serial.begin(9600);
  while (!Serial) delay(10);

  LoraSerial.begin(LORA_BAUDRATE);
  delay(100);

  Serial.println("Configuring LoRa Module...");
  sendAT("AT+OPMODE=1", 500);       
  sendAT("ATZ", 2000);             
  sendAT("AT+ADDRESS=100", 500);   
  sendAT("AT+BAND=865000000", 500);
  sendAT("AT+PARAMETER=9,7,1,12", 500);

  Serial.println("LoRa Communication Ready!");
}

void loop() 
{
  unsigned long now = millis();

  if (now - lastSendTime >= sendInterval) 
  {
    sendData(targetAddress, "hello world :)");
    lastSendTime = now;
  }

  if (Serial.available()) 
  {
    String in = Serial.readStringUntil('\n');
    in.trim();
    int sep = in.indexOf('#');

    if (sep > 0) 
    {
      int addr = in.substring(0, sep).toInt();
      String msg = in.substring(sep + 1);
      sendData(addr, msg.c_str());
    } 
    else if (in.length() > 0) 
    {
      Serial.println("Invalid format. Use: ADDRESS#MESSAGE");
    }
  }

  while (LoraSerial.available()) 
  {
    String resp = LoraSerial.readStringUntil('\n');
    resp.trim();

    if (resp.length()) 
    {
      Serial.print("[LoRa RX] ");
      Serial.println(resp);
    }
  }
}

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
