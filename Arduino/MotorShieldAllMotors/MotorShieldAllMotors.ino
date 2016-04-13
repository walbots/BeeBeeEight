//INSPIRED BY:
/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->  http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *motor1 = AFMS.getMotor(1); //M1
Adafruit_DCMotor *motor2 = AFMS.getMotor(2); //M2
Adafruit_DCMotor *motor3 = AFMS.getMotor(3); //M3
Adafruit_DCMotor *motor4 = AFMS.getMotor(4); //M4

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("STARTING MotorShieldAllMotors");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
}

#define WALBOTS_MAX_SPEED 255
#define WALBOTS_DELAY      10
#define WALBOTS_INCREMENT   1

void cycleMotor (Adafruit_DCMotor *someMotor) {
  uint8_t i;

  for (i = 0; i < WALBOTS_MAX_SPEED; i += WALBOTS_INCREMENT) {
    someMotor->setSpeed(i);
    delay(WALBOTS_DELAY);
  }

  for (i = WALBOTS_MAX_SPEED; i > 0; i -= WALBOTS_INCREMENT) {
    someMotor->setSpeed(i);
    delay(WALBOTS_DELAY);
  }
}

void testMotor (Adafruit_DCMotor *someMotor, uint8_t motorId) {
  uint8_t i;

  Serial.print("tick ");
  Serial.println(motorId);

  someMotor->run(FORWARD);

  cycleMotor(someMotor);

  Serial.print("tock ");
  Serial.println(motorId);

  someMotor->run(BACKWARD);

  cycleMotor(someMotor);

  Serial.print("tech ");
  Serial.println(motorId);

  someMotor->run(RELEASE);
  delay(WALBOTS_DELAY);
}

void loop() {
  testMotor(motor1, 1);
  testMotor(motor2, 2);
  testMotor(motor3, 3);
  testMotor(motor4, 4);
}

