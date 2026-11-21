#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <LoRa.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define NSS D0 //LoRa
#define RST D4 
#define DI0 D8 
#define BAND 433E6 
#define SDA D2   
#define SCL D3   
#define BATT A0 //Battery
#define SERVOMIN  150 //Servo
#define SERVOMAX  600 
#define RUDDER 0
#define ELEVATOR 1
#define FLAPSR 2
#define FLAPSL 3
#define AILERONSR 4
#define AILERONSL 5
#define CAMP 6
#define CAMY 7
#define GEARR 8
#define GEARL 9
#define ESC1 14 //A2212
#define ESC2 15

int motor = 0; //servo
int rudder = 0;
int elevator = 0;
int flaps = 0;
int ailerons = 0;
int gear = 0;
bool stabilization = true;
float camp = 0.0;
float camy = 0.0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); //mpu
MPU6050 mpu;

bool dmpReady = false; //teapot
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 
Quaternion q;           
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

VectorFloat gravity; //stabilization
float ypr[3];

int us_to_ticks(int us) { //servo driver
  return (int)(us * 4096.0 / 20000.0);
}

int angleToPulse(int angle) {
  return map(angle, 0, 90, SERVOMIN, SERVOMAX);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA, SCL); 
  Wire.setClock(100000);
  pwm.begin();
  pwm.setPWMFreq(50); 
  
  int arm_ticks = us_to_ticks(1100); 
  pwm.setPWM(ESC1, 0, arm_ticks);
  pwm.setPWM(ESC2, 0, arm_ticks);
  delay(5000); 
  Serial.println("motor");
  int test_ticks = us_to_ticks(1300);
  pwm.setPWM(ESC1, 0, test_ticks);
  pwm.setPWM(ESC2, 0, test_ticks);
  delay(100);

  pwm.setPWM(RUDDER, 0, angleToPulse(0)); //servo
  pwm.setPWM(ELEVATOR, 0, angleToPulse(0));
  pwm.setPWM(FLAPSR, 0, angleToPulse(0));
  pwm.setPWM(FLAPSL, 0, angleToPulse(0));
  pwm.setPWM(AILERONSR, 0, angleToPulse(0));
  pwm.setPWM(AILERONSL, 0, angleToPulse(0));
  pwm.setPWM(CAMP, 0, angleToPulse(40));
  pwm.setPWM(CAMY, 0, angleToPulse(45));
  pwm.setPWM(GEARR, 0, angleToPulse(0));
  pwm.setPWM(GEARL, 0, angleToPulse(0));

  LoRa.setPins(NSS, RST, DI0); //LoRa
  if (!LoRa.begin(BAND)) {
    while (1);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5); 
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXAccelOffset(-3638);
  mpu.setYAccelOffset(-6126);
  mpu.setZAccelOffset(9574); 
  mpu.setXGyroOffset(19);
  mpu.setYGyroOffset(-35);
  mpu.setZGyroOffset(40);
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
}

void loop() { 
  int packetSize = LoRa.parsePacket();
  int rawADC = analogRead(BATT); //battery
  float pinVoltage = rawADC * (3.3 / 1024.0);
  float batteryV = pinVoltage * 4.164;
  if (packetSize > 0) {
    String recieved = "";
    while (LoRa.available()){
      recieved += (char)LoRa.read();
    }
    int values[8];
    int count = 0;
    int startIndex = 0;
    for (int i = 0; i < recieved.length(); i++) {
      if (recieved.charAt(i) == ',' || i == recieved.length() - 1) {
        String segment = (i == recieved.length() - 1) ? recieved.substring(startIndex) : recieved.substring(startIndex, i);
        if (count < 8) {
          values[count++] = segment.toInt();
        }
        startIndex = i + 1;
      }
    }

    if (count == 8) {
      motor = values[0];
      rudder = map(values[1], 0, 255, 0 ,90);
      elevator = map(values[2], 0, 255, 0 ,90);
      ailerons = map(values[3], 0, 255, 0 ,90);
      gear = values[4]; 
      stabilization = values[5];
      camp = values[6];
      camy = values[7];
    }
    
    if (motor != 0){
      int pulse_us = motor;
      if (gear){
        pwm.setPWM(GEARR, 0, angleToPulse(0));
        pwm.setPWM(GEARL, 0, angleToPulse(0));
      } else{
        pwm.setPWM(GEARR, 0, angleToPulse(90));
        pwm.setPWM(GEARL, 0, angleToPulse(90));
      }

      if (pulse_us <= 2000) {
        int pulse_ticks = us_to_ticks(pulse_us); //motor
        pwm.setPWM(ESC1, 0, pulse_ticks);
        pwm.setPWM(ESC2, 0, pulse_ticks);
        pwm.setPWM(CAMP, 0, angleToPulse(camp));
        pwm.setPWM(CAMY, 0, angleToPulse(camy));

        // if (!dmpReady) return;

        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { //teapot
          mpu.dmpGetQuaternion(&q, fifoBuffer);

          if (stabilization) {
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            float rollDeg  = ypr[2] * 180/M_PI;
            float pitchDeg = ypr[1] * 180/M_PI;
            float Kp = 1.2;
                            
            float targetRoll = map(ailerons, 0, 90, -30, 30); 
            float rollError  = targetRoll - rollDeg;
            int correctedAilerons = ailerons + (rollError * Kp);

            float targetPitch = map(elevator, 0, 90, -30, 30);
            float pitchError  = targetPitch - pitchDeg;
            int correctedElevator = elevator + (pitchError * Kp);

            pwm.setPWM(ELEVATOR, 0, angleToPulse(constrain(correctedElevator, 0, 90)));
            pwm.setPWM(AILERONSR, 0, angleToPulse(constrain(correctedAilerons, 0, 90)));
            pwm.setPWM(AILERONSL, 0, angleToPulse(constrain(correctedAilerons, 0, 90)));

          } else {
            
          }

          int16_t w = q.w * 16384;
          int16_t x = q.x * 16384;
          int16_t y = q.y * 16384;
          int16_t z = q.z * 16384;

          teapotPacket[2] = (uint8_t)(w >> 8);
          teapotPacket[3] = (uint8_t)(w & 0xFF);
          teapotPacket[4] = (uint8_t)(x >> 8);
          teapotPacket[5] = (uint8_t)(x & 0xFF);
          teapotPacket[6] = (uint8_t)(y >> 8);
          teapotPacket[7] = (uint8_t)(y & 0xFF);
          teapotPacket[8] = (uint8_t)(z >> 8);
          teapotPacket[9] = (uint8_t)(z & 0xFF);
        }

        pwm.setPWM(ELEVATOR, 0, angleToPulse(elevator));//put in else of stabilization
        pwm.setPWM(AILERONSR, 0, angleToPulse(ailerons));
        pwm.setPWM(AILERONSL, 0, angleToPulse(ailerons));
        
        pwm.setPWM(RUDDER, 0, angleToPulse(rudder)); 
        pwm.setPWM(FLAPSR, 0, angleToPulse(flaps));
        pwm.setPWM(FLAPSL, 0, angleToPulse(90-flaps));

        LoRa.idle();
        LoRa.beginPacket(); //LoRa send
        LoRa.write(teapotPacket, 14);
        LoRa.print(batteryV);     
        LoRa.print("\n");
        LoRa.endPacket();
        mpu.resetFIFO();
      }
    } else{ //off state
      pwm.setPWM(ESC1, 0, 0); 
      pwm.setPWM(ESC2, 0, 0);
      pwm.setPWM(RUDDER, 0, angleToPulse(0));
      pwm.setPWM(ELEVATOR, 0, angleToPulse(0));
      pwm.setPWM(FLAPSR, 0, angleToPulse(0));
      pwm.setPWM(FLAPSL, 0, angleToPulse(0));
      pwm.setPWM(AILERONSR, 0, angleToPulse(0));
      pwm.setPWM(AILERONSL, 0, angleToPulse(0));
      pwm.setPWM(GEARR, 0, angleToPulse(0));
      pwm.setPWM(GEARL, 0, angleToPulse(0));
      pwm.setPWM(CAMP, 0, angleToPulse(40));
      pwm.setPWM(CAMY, 0, angleToPulse(45));
    }
    LoRa.receive();
  } else { //remove this for final plane code
    pwm.setPWM(ESC1, 0, 0); 
    pwm.setPWM(ESC2, 0, 0);
  }
}