// Bluetooth shtuff

#include <SoftwareSerial.h>   //Software Serial Port
#define RxD 3
#define TxD 2

#define DEBUG_ENABLED  1

#define PIN_TEMP    A5

SoftwareSerial blueToothSerial(RxD,TxD);


// Motor shtuff

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

// Motors will be mounted to drive omniwheels directly.
// Motor shafts will run parallel to the plane of the platform, mounted underneath with brackets.
// Motor axles will be oriented outwards from the center of the platform, into the omniwheels.
// Omniwheels will be oriented perpendicular to the plane of the platform.
// Motors 1 & 2 will act as the "forward" and "back" drivers.
// Motors 3 & 4 will act as the "left" and "right" drivers.
// Each motor-pair will act in concert, with the directions for the motors in opposition (because their mount orientations are mirrored).
// Omniwheels are necessary to avoid drag in this situation.

Adafruit_DCMotor *motors_1[]   = { motor1 };
Adafruit_DCMotor *motors_2[]   = { motor2 };
Adafruit_DCMotor *motors_3[]   = { motor3 };
Adafruit_DCMotor *motors_4[]   = { motor4 };
Adafruit_DCMotor *motors_1_2[] = { motor1, motor2 };
Adafruit_DCMotor *motors_1_3[] = { motor1, motor3 };
Adafruit_DCMotor *motors_1_4[] = { motor1, motor4 };
Adafruit_DCMotor *motors_2_3[] = { motor2, motor3 };
Adafruit_DCMotor *motors_2_4[] = { motor2, motor4 };
Adafruit_DCMotor *motors_3_4[] = { motor3, motor4 };
Adafruit_DCMotor *motors_1_2_3_4[] = { motor1, motor2, motor3, motor4 };


uint8_t motor_ids_1[] = { 1 };
uint8_t motor_ids_2[] = { 2 };
uint8_t motor_ids_3[] = { 3 };
uint8_t motor_ids_4[] = { 4 };
uint8_t motor_ids_1_3[] = { 1, 3 };
uint8_t motor_ids_1_4[] = { 1, 4 };
uint8_t motor_ids_2_3[] = { 2, 3 };
uint8_t motor_ids_2_4[] = { 2, 4 };

// custom shtuff

#define WALBOTS_MAX_MOTORS   4
#define WALBOTS_MAX_SPEED  255
#define WALBOTS_DELAY       10
#define WALBOTS_INCREMENT    1

void printMotorIds (uint8_t motorIds[], uint8_t numMotors) {
  uint8_t i;

  for (i = 0; i < numMotors; i++ ) {
    Serial.print(" ");
    Serial.print(motorIds[i]);
  }
}

void directMotors (Adafruit_DCMotor *someMotors[], uint8_t numMotors, uint8_t d) {
  uint8_t i;

  for (i = 0; i < numMotors; i++) {
    someMotors[i]->run(d);
  }
}

void directMotorsBackward () {
  directMotors(motors_1, 1, BACKWARD);
  directMotors(motors_2, 1, FORWARD);
}

void directMotorsForward () {
  directMotors(motors_1, 1, FORWARD);
  directMotors(motors_2, 1, BACKWARD);
}

void directMotorsLeft () {
  directMotors(motors_3, 1, FORWARD);
  directMotors(motors_4, 1, BACKWARD);
}

void directMotorsRight () {
  directMotors(motors_3, 1, BACKWARD);
  directMotors(motors_4, 1, FORWARD);
}

void speedMotors (Adafruit_DCMotor *someMotors[], uint8_t numMotors, uint8_t s) {
  uint8_t i;

  for (i = 0; i < numMotors; i++) {
    someMotors[i]->setSpeed(s);
  }
}

void cycleMotors (Adafruit_DCMotor *someMotors[], uint8_t numMotors) {
  uint8_t s;

  for (s = 0; s < WALBOTS_MAX_SPEED; s += WALBOTS_INCREMENT) {
    speedMotors(someMotors, numMotors, s);
    delay(WALBOTS_DELAY);
  }

  for (s = WALBOTS_MAX_SPEED; s > 0; s -= WALBOTS_INCREMENT) {
    speedMotors(someMotors, numMotors, s);
    delay(WALBOTS_DELAY);
  }
}

void testMotors (Adafruit_DCMotor *someMotors[], uint8_t numMotors, uint8_t motorIds[]) {
  uint8_t i;

  Serial.print("tick");
  printMotorIds(motorIds, numMotors);
  Serial.println("");
  directMotors(someMotors, numMotors, FORWARD);
  cycleMotors(someMotors, numMotors);

  Serial.print("tock");
  printMotorIds(motorIds, numMotors);
  Serial.println("");
  directMotors(someMotors, numMotors, BACKWARD);
  cycleMotors(someMotors, numMotors);

  Serial.print("tech");
  printMotorIds(motorIds, numMotors);
  Serial.println("");
  directMotors(someMotors, numMotors, RELEASE);

  delay(WALBOTS_DELAY);
}

void releaseMotors (Adafruit_DCMotor *someMotors[], uint8_t numMotors, uint8_t motorIds[]) {
  Serial.print("MOTORS RELEASE");
  printMotorIds(motorIds, 

  directMotors(motors_1_2_3_4, 4, RELEASE);
}

int getTemp ()
{
    int a = analogRead(PIN_TEMP);
    int B=3975;
    float resistance = (float)(1023-a)*10000/a;
    float temperature = temperature=1/(log(resistance/10000)/B+1/298.15)-273.15;
    
    return (int)temperature;
    
}

void setupBlueToothConnection ()
{          
  blueToothSerial.begin(9600);  
  
  blueToothSerial.print("AT\r\n");
  delay(400); 

  blueToothSerial.print("AT+DEFAULT\r\n");             // Restore all setup value to factory setup
  delay(2000); 
  
  blueToothSerial.print("AT+NAMESeeedBTSlave\r\n");    // set the bluetooth name as "SeeedBTSlave" ,the length of bluetooth name must less than 12 characters.
  delay(400);
  
  blueToothSerial.print("AT+PIN0000\r\n");             // set the pair code to connect 
  delay(400);
  
  blueToothSerial.print("AT+AUTH1\r\n");             //
  delay(400);    

  blueToothSerial.flush();
}


// lifecycle shtuff

void setup ()
{
  Serial.begin(9600);
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);

  Serial.println("RCCarWhassup");
  Serial.println("setting up bluetooth");
  setupBlueToothConnection();
  Serial.println("done setting up bluetooth");



  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
}

void loop ()
{
  char recvChar;

  while (1)
  {
      if (blueToothSerial.available())
      {//check if there's any data sent from the remote bluetooth shield
          recvChar = blueToothSerial.read();

          Serial.print("received: ");
          Serial.println(recvChar);
          blueToothSerial.print("received: ");
          blueToothSerial.println(recvChar);

//RDW FIXME need to ignore input while "action" is happening
          switch (recvChar) {
            case 'S':
            case 's':
              // STAY
              releaseMotors();
              break;
            case 'F':
            case 'f':
              // FORWARD
              //releaseMotors();
              directMotorsForward();
              //testMotors(motors_1, 1, motor_ids_1);
              break;
            case 'B':
            case 'b':
              // BACKWARD
              releaseMotors();
              directMotorsBackward();
              //testMotors(motors_2, 1, motor_ids_2);
              break;
            case 'L':
            case 'l':
              // LEFT
              releaseMotors();
              //testMotors(motors_3, 1, motor_ids_3);
              break;
            case 'R':
            case 'r':
              // RIGHT
              releaseMotors();
              //testMotors(motors_4, 1, motor_ids_4);
              break;
            case 'G':
            case 'g':
              // LEFT FORWARD
              releaseMotors();
              //testMotors(motors_1_3, 2, motor_ids_1_3);
              break;
            case 'I':
            case 'i':
              // RIGHT FORWARD
              releaseMotors();
              //testMotors(motors_1_4, 2, motor_ids_1_4);
              break;
            case 'H':
            case 'h':
              // LEFT BACKWARD
              releaseMotors();
              //testMotors(motors_2_3, 2, motor_ids_2_3);
              break;
            case 'J':
            case 'j':
              // RIGHT BACKWARD
              releaseMotors();
              //testMotors(motors_2_4, 2, motor_ids_2_4);
              break;
            case '0':
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
            case 'q':
              // slider values
              break;
            default:
              Serial.print("unhandled: ");
              Serial.println(recvChar);
              break;
          }
              
          
//          if(recvChar == 't' || recvChar == 'T')
//          {
//              blueToothSerial.print("temperature: ");
//              blueToothSerial.println(getTemp());
//          }
      }
//      if(Serial.available())
//      {//check if there's any data sent from t he local serial terminal, you can add the other applications here
//          recvChar  = Serial.read();
//          blueToothSerial.print(recvChar);
//      }
  }
}

