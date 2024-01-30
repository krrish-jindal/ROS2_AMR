#include <Adafruit_MotorShield.h>
#include <Arduino.h>
#include <Wire.h>
#define AUTO_STOP_INTERVAL 10000   // Stop the robot if it hasn't received a movement command in this number of milliseconds

unsigned long currentMillis = 0;
long lastMotorCommand = AUTO_STOP_INTERVAL; // Convert the rate into an interval
int motorSpeed = 50;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x40);

#define IFDAC_ENABLED 1

#define FRONTLEFT 0
#define FRONTRIGHT 1
#define BACKRIGHT 2
#define BACKLEFT 3

Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(4);

void initMotorController() {
  if (IFDAC_ENABLED) {
    // DAC configuration code (commented out for now)
    // ...
  }

  AFMS.begin(); // create with the default frequency 1.6KHz

  frontLeftMotor->setSpeed(100);
  backLeftMotor->setSpeed(100);
  frontRightMotor->setSpeed(100);
  backRightMotor->setSpeed(100);

  // set motors forward to initiate
  frontLeftMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  frontRightMotor->run(FORWARD);
  backRightMotor->run(FORWARD);

  // release all motors
  frontLeftMotor->run(RELEASE);
  backLeftMotor->run(RELEASE);
  frontRightMotor->run(RELEASE);
  backRightMotor->run(RELEASE);
}

void setMotorSpeed(int motor, int spd) {
  switch (motor) {
    case FRONTLEFT:
      if (spd == 0)
        frontLeftMotor->run(RELEASE);
      else
        spd > 0 ? frontLeftMotor->run(FORWARD) : frontLeftMotor->run(BACKWARD);
      frontLeftMotor->setSpeed(abs(spd));
      break;
    case FRONTRIGHT:
      if (spd == 0)
        frontRightMotor->run(RELEASE);
      else
        spd > 0 ? frontRightMotor->run(FORWARD) : frontRightMotor->run(BACKWARD);
      frontRightMotor->setSpeed(abs(spd));
      break;
    case BACKRIGHT:
      if (spd == 0)
        backRightMotor->run(RELEASE);
      else
        spd > 0 ? backRightMotor->run(FORWARD) : backRightMotor->run(BACKWARD);
      backRightMotor->setSpeed(abs(spd));
      break;
    case BACKLEFT:
      if (spd == 0)
        backLeftMotor->run(RELEASE);
      else
        spd > 0 ? backLeftMotor->run(FORWARD) : backLeftMotor->run(BACKWARD);
      backLeftMotor->setSpeed(abs(spd));
      break;
  }
}

void setMotorSpeeds(int frontLeftSpeed, int frontRightSpeed, int backRightSpeed, int backLeftSpeed) {
  setMotorSpeed(FRONTLEFT, frontLeftSpeed);
  setMotorSpeed(FRONTRIGHT, frontRightSpeed);
  setMotorSpeed(BACKRIGHT, backRightSpeed);
  setMotorSpeed(BACKLEFT, backLeftSpeed);
}

void setup(void) {
  Serial.begin(115200);
  Wire.begin (33, 32);   // sda= 33 /scl= 32 for sense and 22, 21 for ttgo_motor controller
  initMotorController();
  setMotorSpeeds(0, 0, 0, 0);   // give time for reboot/program upload to stabilize
  delay(1000);
  setMotorSpeeds(motorSpeed, motorSpeed, -motorSpeed, -motorSpeed);
  Serial.println("Started");
}

void loop(void) {
  currentMillis = millis();
  if (currentMillis > AUTO_STOP_INTERVAL) { // Check to see if we have exceeded the auto-stop interval
    setMotorSpeeds(0, 0, 0, 0);
  }
}
