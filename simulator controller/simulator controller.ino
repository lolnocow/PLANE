#include <Wire.h>
#include "PCF8575.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define SDA D2
#define SCL D1
#define ACDC 0x48            // PCF8591
#define J1_X 0x40
#define J1_Y 0x41
#define J2_X 0x43
#define J2_Y 0x42
PCF8575 io(0x20);

const char *ssid = "featherweight";
const char *password = "p@$$Word";
IPAddress pcIP(192, 168, 137, 1);   // PC (Windows hotspot gateway)
const int udpPort = 4210;
WiFiUDP Udp;

const unsigned long SEND_INTERVAL_MS = 20;   // 50 Hz
unsigned long lastSendTime = 0;
unsigned long lastWarn = 0;
unsigned long seqNum = 0;
bool StartOk = false;

// ---------- buzzer ----------
struct ToneNote { int durationMs; int freqDelayUs; int pauseMs; };
const ToneNote BOOT_MELODY[] = { {70, 350, 10}, {70, 280, 10}, {70, 220, 10}, {120, 150, 0} };
const ToneNote WARN_BEEP[]   = { {120, 220, 0} };

const ToneNote* currentSeq = nullptr;
int currentSeqLen = 0, currentStep = 0;
unsigned long stepStartTime = 0;
bool inPause = false, buzzerActive = false;

void playSequence(const ToneNote* seq, int length) {
  currentSeq = seq; currentSeqLen = length; currentStep = 0;
  stepStartTime = millis(); inPause = false; buzzerActive = true;
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

// ---------- inputs ----------
int read_pcf8591(byte control_byte) {
  Wire.beginTransmission(ACDC);
  Wire.write(control_byte);
  Wire.endTransmission();
  Wire.requestFrom(ACDC, 2);
  if (Wire.available()) {
    Wire.read();
    if (Wire.available()) return Wire.read();
  }
  return -1;
}

// calibrated so each stick's physical centre maps to 128
int mapAxis(int val, int centre) {
  if (val == -1) return 128;
  if (val <= centre) return map(val, 0, centre, 0, 128);
  return map(val, centre, 255, 128, 255);
}

int mapJ1Y(int val) {
  if (val == -1) return 128; 
  if (val <= 43) return 0;
  if (val <= 203) return map(val, 43, 203, 0, 128);
  else return map(val, 203, 255, 128, 255);
}

// A0 resistor ladder -> bitmask (bit0..bit7)
uint8_t readButtons() {
  int b = analogRead(A0);
  if (b >= 5   && b <= 25)   return 1 << 0;  // j2y+
  if (b >= 540 && b <= 560)  return 1 << 1;  // j2y-
  if (b >= 715 && b <= 735)  return 1 << 2;  // br
  if (b >= 805 && b <= 830)  return 1 << 3;  // bm
  if (b >= 855 && b <= 880)  return 1 << 4;  // bl
  if (b >= 890 && b <= 915)  return 1 << 5;  // j2x-
  if (b >= 920 && b <= 940)  return 1 << 6;  // j2x+
  if (b >= 950)              return 1 << 7;  // tr
  return 0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA, SCL); 

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
  io.digitalWrite(P11, HIGH); 

  if (WiFi.status() != WL_CONNECTED) {    // ESP auto-reconnects
    io.digitalWrite(P10, HIGH);
    return;
  }
  io.digitalWrite(P10, LOW);

  if (millis() - lastSendTime < SEND_INTERVAL_MS) return;
  lastSendTime = millis();

  int throttle = map(mapJ1Y(read_pcf8591(J1_Y)), 0 ,255 , 0, 100);
  throttle = constrain(throttle, 0 ,100);
  int yaw   = mapAxis(read_pcf8591(J1_X), 197);
  int pitch = mapAxis(read_pcf8591(J2_Y), 125);
  int roll  = mapAxis(read_pcf8591(J2_X), 132);

  bool armed   = io.digitalRead(P15) == HIGH;   // plane mains switch
  bool stab    = io.digitalRead(P14) == LOW;
  bool gearUp  = io.digitalRead(P7) == HIGH;
  uint8_t btn  = readButtons();

  // arm only if throttle is at idle
  if (armed) {
    if (throttle <= 2) StartOk = true;
    if (!StartOk) {
      throttle = 0;
    }
  } else {
    StartOk = false;
    throttle = 0;
  }
  bool live = armed && StartOk;

  char msg[64];
  snprintf(msg, sizeof(msg), "%lu,%d,%d,%d,%d,%d,%d,%d,%u",
           seqNum++, throttle, yaw, pitch, roll, gearUp ? 1 : 0, stab ? 1 : 0, live ? 1 : 0, btn);
  Udp.beginPacket(pcIP, udpPort);
  Udp.write((const uint8_t*)msg, strlen(msg));
  Udp.endPacket();
}