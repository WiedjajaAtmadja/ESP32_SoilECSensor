/*
Untuk emailnya ini pak
Email : mebinusmalang@gmail.com
Pass :Me10062019
*/
#include <Arduino.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <ThingsBoard.h>
#include "NonBlockingOutput.h"

#define PIN_RS485_TXEN 5 // D5
#define PIN_WATER_PUMP 4 // D4

#define TIMER_SEND_DATA_INTERVAL 2000
#define SOIL_MOISTURE_THRESHOLD_LOW 50.0f
#define SOIL_MOISTURE_THRESHOLD_HIGH 80.0f

/*
PIN IO34 dan IO35 hanya bisa digunakan sebagai input

const int relay1 = 34;          //Output Relay 1 ke Arnuino Pin D0
const int relay2 = 35;          //Output Relay 2 ke Arnuino Pin D1
const int relay3 = 32;          //Output Relay 3 ke Arnuino Pin D2
const int relay4 = 33;          //Output Relay 4 ke Arnuino Pin D3
const int relay5 = 25;          //Output Relay 5 ke Arnuino Pin D4
const int relay6 = 26;          //Output Relay 6 ke Arnuino Pin D5
const int relay7 = 27;          //Output Relay 7 ke Arnuino Pin D6
const int relay8 = 14;          //Output Relay 8 ke Arnuino Pin D7*/

#define NUM_OUTPUS 8
// ini adalah pin yang digunakan untuk output
const uint8_t arOutputsPin[NUM_OUTPUS] = {13, 12, 14, 27, 26, 25, 33, 32};
// ini adalah durasi output dalam milisecond
uint16_t arOutputOnDuration[NUM_OUTPUS] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

NonBlockingOutput nbOutput;

constexpr uint16_t MAX_MESSAGE_SIZE = 128U;
constexpr char WIFI_SSID[] = "";
constexpr char WIFI_PASSWORD[] = "";
constexpr char THINGSBOARD_SERVER[] = "demo.thingsboard.io";
constexpr char THINGSBOARD_TOKEN[] = "";
constexpr uint16_t THINGSBOARD_PORT = 80U;
constexpr char TEMPERATURE_KEY[] = "temperature";
constexpr char HUMIDITY_KEY[] = "moisture";

ModbusMaster mbus;
WiFiClient espClient;
ThingsBoard tb(espClient);

void preTransmission()
{
  digitalWrite(PIN_RS485_TXEN, HIGH);
}

void postTransmission()
{
  digitalWrite(PIN_RS485_TXEN, LOW);
}

void OnReadSensor()
{
    uint8_t result = mbus.readHoldingRegisters(0x00, 8);
    if (result == mbus.ku8MBSuccess)
    {
      digitalWrite(LED_BUILTIN, HIGH);
        // printf("Data: ");
        // for (int i=0; i<8; i++)
        //   Serial.printf("%d ", mbus.getResponseBuffer(i));
        // Serial.println();  
      int nData0    = mbus.getResponseBuffer(0);
      int nData1    = mbus.getResponseBuffer(1);
      float fSoilMoisture = (float(mbus.getResponseBuffer(2))/10.0f);
      float fTemperature = (float(mbus.getResponseBuffer(3))/10.0f);

      Serial.printf("SoilSensor Data: %d %d Moisture: %2.1f %% Temp: %2.1f C\r\n",
      nData0, nData1, fSoilMoisture, fTemperature);    
      if (tb.connected())
      {
        Serial.println("Sending data to thingsboard");
        tb.sendTelemetryData("Temperature", fTemperature);
        tb.sendTelemetryData("SoilMoisture", fSoilMoisture);
      }
      else
      {
        Serial.println("Thingsboard not connected");
      }

      if (fSoilMoisture<=SOIL_MOISTURE_THRESHOLD_LOW)
      {
        Serial.println("Soil moisture is low, turn on water pump");
        digitalWrite(LED_BUILTIN, HIGH);
        nbOutput.start();
      }

      if (fSoilMoisture>=SOIL_MOISTURE_THRESHOLD_HIGH)
      {
        Serial.println("Soil moisture is high, turn off water pump");
        digitalWrite(LED_BUILTIN, LOW);
      }
        
      digitalWrite(LED_BUILTIN, LOW);
    }
    else
      Serial.printf("Read failed, result: 0x%02X\r\n", result);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(PIN_RS485_TXEN, OUTPUT);
  digitalWrite(PIN_RS485_TXEN, LOW); 

  pinMode(PIN_WATER_PUMP, OUTPUT);
  digitalWrite(PIN_WATER_PUMP, LOW);
  for (uint8_t i = 0; i < NUM_OUTPUS; i++) {
    nbOutput.addOutput(arOutputsPin[i], arOutputOnDuration[i]);
  }
  nbOutput.begin();

  mbus.begin(1, Serial2);

  // pakai ini jika modul RS585 bukan mode AutoDirection (memiliki pin DE dan RE)
  mbus.preTransmission(preTransmission);
  mbus.postTransmission(postTransmission);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  digitalWrite(LED_BUILTIN, HIGH);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
  Serial.println("WiFi Connection Failed! Rebooting...");
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(4500);
  ESP.restart();
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("System connected with IP Address: ");
  Serial.println(WiFi.localIP());
    if (tb.connect(THINGSBOARD_SERVER, THINGSBOARD_TOKEN)){
    Serial.println("Connected to thingsboard.");
  }
  else
  {
      Serial.println("Failed to connect thingsBoard");
  }
}


unsigned long lastMillis = 0;
void loop() {
  unsigned long nNow = millis();
  if (nNow-lastMillis >= TIMER_SEND_DATA_INTERVAL)
  {
    lastMillis = nNow;
    OnReadSensor();
/*}
    if(HUMIDITY_KEY < 50)
   {
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, HIGH);
    digitalWrite(relay3, LOW);
    digitalWrite(relay4, HIGH);
    digitalWrite(relay5, LOW);
    digitalWrite(relay6, HIGH);
    digitalWrite(relay7, LOW);
    digitalWrite(relay8, HIGH);
   }
    else
  {
    digitalWrite(relay1, HIGH);
    digitalWrite(relay2, HIGH);
    digitalWrite(relay3, HIGH);
    digitalWrite(relay4, HIGH);
    digitalWrite(relay5, HIGH);
    digitalWrite(relay6, HIGH);
    digitalWrite(relay7, HIGH);
    digitalWrite(relay8, HIGH);*/
  }
  tb.loop();
  nbOutput.update();
}