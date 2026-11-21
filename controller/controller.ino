#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "PCF8575.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define NSS D4 //LoRa
#define RST D8 
#define DI0 D0 
#define BAND 433E6 
#define SCK D5
#define MISO D6
#define MOSI D7
#define SDA D2 
#define SCL D1
#define ACDC 0x48 //PCF8591
#define J1_X 0x40
#define J1_Y 0x41 
#define J2_X 0x43
#define J2_Y 0x42
PCF8575 io(0x20); //PCF8575

const char *ssid = "featherweight"; //wifi
const char *password = "p@$$Word";
IPAddress broadcastIP(192, 168, 137, 1);
const int udpPort = 4210;
WiFiUDP Udp;

float camp = 40.0; //vrx
float camy = 45.0;
float initcamp = 0.0;
float initcamy = 0.0;

long lastSendTime = 0; 
bool LandingGear = true; 
bool stabilization = true;
bool StartOk = false;

bool wasConnected = false;
unsigned long lastPacketReceivedTime = 0;
unsigned long lastBatteryWarn = 0;
unsigned long lastAnalogReadTime = 0;

String incomingTelemetry = "";
float planeBatteryV = 0.0;

struct ToneNote {
  int durationMs;
  int freqDelayUs;
  int pauseMs;
};

const ToneNote BOOT_MELODY[] = { {70, 350, 10}, {70, 280, 10}, {70, 220, 10}, {120, 150, 0} }; 
const ToneNote CONNECT_MELODY[] = { {80, 280, 25}, {100, 200, 25}, {180, 150, 0} };               
const ToneNote DISCON_MELODY[] = { {150, 150, 30}, {300, 400, 0} };                              
const ToneNote BATT_WARN_MELODY[] = { {80, 200, 50}, {80, 200, 50}, {80, 200, 0} };                 
const ToneNote WARN_BEEP[] = { {120, 220, 0} };

const ToneNote* currentSeq = nullptr;
int currentSeqLen = 0;
int currentStep = 0;
unsigned long stepStartTime = 0;
bool inPause = false;
bool buzzerActive = false;

void playSequence(const ToneNote* seq, int length) {
  currentSeq = seq;
  currentSeqLen = length;
  currentStep = 0;
  stepStartTime = millis();
  inPause = false;
  buzzerActive = true;
}

void updateBuzzer() {
  if (currentSeq == nullptr) {
    if (buzzerActive) {
      io.digitalWrite(P11, HIGH); 
      buzzerActive = false;
    }
    return;
  }

  unsigned long nowMs = millis();
  unsigned long elapsedMs = nowMs - stepStartTime;
  ToneNote note = currentSeq[currentStep];
  if (!inPause) {
    if (elapsedMs < note.durationMs) {
      unsigned long noteStartUs = micros();
      unsigned long noteTargetUs = note.durationMs * 1000UL;
      int safeDelay = max(note.freqDelayUs, 150); 
      while ((micros() - noteStartUs) < noteTargetUs) {
        io.digitalWrite(P11, LOW);  
        delayMicroseconds(safeDelay);
        io.digitalWrite(P11, HIGH); 
        delayMicroseconds(safeDelay);
      }

      inPause = true;
      stepStartTime = millis();
    }
  } else {
    if (elapsedMs >= note.pauseMs) {
      currentStep++;
      if (currentStep >= currentSeqLen) {
        currentSeq = nullptr; 
        io.digitalWrite(P11, HIGH);
        buzzerActive = false;
      } else {
        inPause = false;
        stepStartTime = millis();
      }
    }
  }
}

int read_pcf8591(byte control_byte) {
  Wire.beginTransmission(ACDC);
  Wire.write(control_byte); 
  Wire.endTransmission();
  Wire.requestFrom(ACDC, 2); 
  if (Wire.available()) {
    Wire.read(); 
    if (Wire.available()) {
      return Wire.read();  
    }
  }
  return -1; 
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) { 
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int mapJ1X(int val) {
  if (val == -1) return 128; 
  if (val <= 197) return map(val, 0, 197, 0, 128);
  else return map(val, 197, 255, 128, 255);
}
int mapJ1Y(int val) {
  if (val == -1) return 128; 
  if (val <= 43) return 0;
  if (val <= 203) return map(val, 43, 203, 0, 128);
  else return map(val, 203, 255, 128, 255);
}
int mapJ2X(int val) {
  if (val == -1) return 128;  
  if (val <= 132) return map(val, 0, 132, 0, 128);
  else return map(val, 132, 255, 128, 255);
}
int mapJ2Y(int val) {
  if (val == -1) return 128; 
  if (val <= 125) return map(val, 0, 125, 0, 128);
  else return map(val, 125, 255, 128, 255);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA, SCL); 
  SPI.begin();
  LoRa.setPins(NSS, RST, DI0);
  if (!LoRa.begin(BAND)) {
    while (1);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5); 
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);

  pinMode(D3,INPUT);
  io.pinMode(P0, INPUT); 
  io.pinMode(P1, INPUT);
  io.pinMode(P4, INPUT);
  io.pinMode(P5, INPUT);
  io.pinMode(P6, INPUT);
  io.pinMode(P7, INPUT);
  io.pinMode(P8, INPUT);
  io.pinMode(P9, INPUT);
  io.pinMode(P10, OUTPUT);
  io.pinMode(P11, OUTPUT);
  io.pinMode(P12, INPUT);
  io.pinMode(P13, INPUT);
  io.pinMode(P14, INPUT);
  io.pinMode(P15, INPUT);
  io.begin(); 
  io.digitalWrite(P10,LOW);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  Udp.begin(udpPort);
  io.digitalWrite(P10, HIGH);
  playSequence(BOOT_MELODY, 4);
}

void loop() {
  updateBuzzer();

  int toggle10 = LOW; //j2y+
  int toggle11 = LOW; //j2y-
  int toggle12 = LOW; //br button
  int toggle13 = LOW; //bm button
  int toggle14 = LOW; //bl button
  int toggle15 = LOW; //j2x-
  int toggle16 = LOW; //j2x+
  int toggle17 = LOW; //tr button
  if (millis() - lastAnalogReadTime >= 30) {
    lastAnalogReadTime = millis();
    int button = analogRead(A0);
    if (button >=5 && button<=25){
      toggle10 = HIGH;
    } if (button >=540 && button<=560){
      toggle11 = HIGH;
    } if (button >=715 && button<=735){
      toggle12 = HIGH;
    } if (button >=805 && button<=830){
      toggle13 = HIGH;
    } if (button >=855 && button<=880){
      toggle14 = HIGH;
    } if (button >=890 && button<=915){
      toggle15 = HIGH;
    } if (button >=920 && button<=940){
      toggle16 = HIGH;
    } if (button >=950 && button<=1023){
      toggle17 = HIGH;
    }
  }

  int packetSize = Udp.parsePacket(); //get head tracking
  if (packetSize > 0) {
    char incomingBuffer[128];
    int len = Udp.read(incomingBuffer, 127);
    if (len > 0) {
      incomingBuffer[len] = '\0';
      String udpStr = String(incomingBuffer);
      
      if (udpStr.startsWith("@")) {
        udpStr = udpStr.substring(1); 
        int commaIndex = udpStr.indexOf(',');
        if (commaIndex != -1) {
          camp = udpStr.substring(0, commaIndex).toFloat();
          camy = udpStr.substring(commaIndex + 1).toFloat();

          if (initcamp == 0.0 || toggle13 == HIGH){
            initcamp = abs(camp);
            initcamy = camy;
          } else {
            camp = camp - initcamp;
            camy = camy - initcamy;
          }

          if (camp > -360.0 && camp < -230.0){
            camp = mapf(camp, -360.0, -270.0, 40.0, 90.0);
          } else if(camp < 0.0 && camp >-90.0){
            camp = mapf(camp, 0.0, -90.0, 40.0, 0.0);
          } else if(camp > 0.0){
            camp = 40.0;
          }

          if (camy > 0){
            camy = mapf(camy, 90.0, 0.0, 0.0, 45.0);
          } else if (camy < 0.0){
            camy = mapf(camy, 0.0, -90.0, 45.0, 90.0);
          }
          if (camy == 0.0){camy = 45.0;}

          camp = constrain(camp, 20.0, 90.0);
          camy = 90.0 - constrain(camy, 0.0, 90.0);
        }
      }
    }
  }

  if (millis() - lastSendTime > 100) { 
    lastSendTime = millis(); 

    int percentage = map(mapJ1Y(read_pcf8591(J1_Y)), 1, 255, 0, 100); 
    int j1_x = mapJ1X(read_pcf8591(J1_X));
    int j2_x = mapJ2X(read_pcf8591(J2_X));
    int j2_y = mapJ2Y(read_pcf8591(J2_Y));

    int speed = (percentage*6.5) + 1350;
    int toggle1 = io.digitalRead(P0); //j1x+
    int toggle2 = io.digitalRead(P1); //j1y+
    int toggle3 = io.digitalRead(P8); //tl button
    int toggle4 = io.digitalRead(P9); //j1x-
    int toggle5 = io.digitalRead(P12); //li u toggle
    int toggle6 = io.digitalRead(P13); //lo u toggle
    int toggle7 = io.digitalRead(P14); //stabilization
    int toggle8 = io.digitalRead(P15); //plane mains
    int toggle9 = digitalRead(D3); //j1y-
    int toggle10 = io.digitalRead(P4); //ro b toggle
    int toggle11 = io.digitalRead(P5); //ro u toggle
    int toggle12 = io.digitalRead(P6); //lo b toggle
    int toggle13 = io.digitalRead(P7); //gear
    
    if (toggle8 == HIGH) { 
      if (speed <= 1360){
        speed = 1;
        StartOk = true;
      } 
      if (!StartOk){
        playSequence(WARN_BEEP, 1);
        speed = 0; 
      }
    } else {
      speed = 0;
      StartOk = false;
    }
    if (toggle7 == LOW) {
      stabilization = true;
    } else {
      stabilization = false;
    }
    if (toggle13 == HIGH) {
      LandingGear = false;
    } else {
      LandingGear = true;
    }

    LoRa.beginPacket(); 
    LoRa.print(speed); LoRa.print(",");
    LoRa.print(j1_x); LoRa.print(",");
    LoRa.print(j2_y); LoRa.print(",");
    LoRa.print(j2_x); LoRa.print(",");
    LoRa.print(LandingGear); LoRa.print(",");
    LoRa.print(stabilization); LoRa.print(",");
    LoRa.print(camp); LoRa.print(",");
    LoRa.print(camy);
    LoRa.endPacket();
    delay(1);

    unsigned long waitStart = millis(); 
    bool gotPacket = false;
    uint8_t loraBuffer[64];
    int loraByteCount = 0;
    
    while (millis() - waitStart < 50) { //get telemetry
      if (int(LoRa.parsePacket()) > 0) { 
        lastPacketReceivedTime = millis();
        gotPacket = true;
        
        int ByteCount = 0;
        String VoltStr = "";
        incomingTelemetry = "";
        while (LoRa.available() && loraByteCount < 64) {
          char c = (char)LoRa.read();
          loraBuffer[loraByteCount++] = (uint8_t)c;
          incomingTelemetry += c;
          if (ByteCount >= 14) { 
            if ((c >= '0' && c <= '9') || c == '.') VoltStr += c;
          }
          ByteCount++;
        }

        if (VoltStr.length() > 0){
          planeBatteryV = VoltStr.toFloat();      
          if (planeBatteryV > 1.0 && planeBatteryV < 10.6 && planeBatteryV != 10.00 && ((millis() - lastBatteryWarn) > 10000)) { 
            playSequence(BATT_WARN_MELODY, 3);
            lastBatteryWarn = millis();
          } 
        }  
        break;
      }
    }

    Udp.beginPacket(broadcastIP, udpPort); //send telemetry
    if (gotPacket && loraByteCount > 0) {
      Udp.write(loraBuffer, loraByteCount);
    }
    String telemetryStr = "#" + String(speed) + "," + String(LoRa.packetRssi()) + "\n";
    Udp.write(telemetryStr.c_str());
    Udp.endPacket();

    if (gotPacket && !wasConnected) { 
      playSequence(CONNECT_MELODY, 3);
      wasConnected = true;
      io.digitalWrite(P10, LOW);
    } else if (wasConnected && ((millis() - lastPacketReceivedTime) > 1000)) {
      playSequence(DISCON_MELODY, 2);
      wasConnected = false;
      io.digitalWrite(P10, HIGH);
    }
  }
}