#include <Arduino.h>
#include <ModbusMaster.h>
#define RS485_TXEN 5

ModbusMaster mbus;

void preTransmission()
{
  digitalWrite(RS485_TXEN, HIGH);
}

void postTransmission()
{
  digitalWrite(RS485_TXEN, LOW);
}


void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  pinMode(RS485_TXEN, OUTPUT);
  digitalWrite(RS485_TXEN, LOW); 

  mbus.begin(1, Serial2);

  // pakai ini jika modul RS585 bukan mode AutoDirection (memiliki pin DE dan RE)
  mbus.preTransmission(preTransmission);
  mbus.postTransmission(postTransmission);
}

void loop() {
  uint8_t result = mbus.readHoldingRegisters(0x00, 8);
  if (result == mbus.ku8MBSuccess)
  {
      // printf("Data: ");
      // for (int i=0; i<8; i++)
      //   Serial.printf("%d ", mbus.getResponseBuffer(i));
      // Serial.println();  
    int nData0    = mbus.getResponseBuffer(0);
    int nData1    = mbus.getResponseBuffer(1);
    float fMoisture = (float(mbus.getResponseBuffer(2))/10.0f);
    float fTemperature = (float(mbus.getResponseBuffer(3))/10.0f);

    Serial.printf("SoilSensor Data: %d %d Moisture: %2.1f %% Temp: %2.1f C\r\n",
     nData0, nData1, fMoisture, fTemperature);    
  }
  else
  {
    Serial.print("Read failed, result: ");
    Serial.println(result, HEX);
  }

  delay(2000);
}