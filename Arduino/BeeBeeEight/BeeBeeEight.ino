// Bluetooth shtuff

#include <SoftwareSerial.h>   //Software Serial Port

// custom constants

#define DEBUG_ENABLED  1

#define PIN_TEMP    A5

#define RxD 3
#define TxD 2

#define WALBOTS_DELAY            10
#define WALBOTS_HEAD_SERVO_RANGE 180
#define WALBOTS_HEAD_DEFAULT     (WALBOTS_HEAD_SERVO_RANGE / 2)
#define WALBOTS_HEAD_INCREMENT   (WALBOTS_HEAD_SERVO_RANGE / 10)
#define WALBOTS_INCREMENT        3
#define WALBOTS_MAX_MOTORS       4
#define WALBOTS_MAX_SPEED        128
#define WALBOTS_MIN_SPEED        -WALBOTS_MAX_SPEED

SoftwareSerial blueToothSerial(RxD,TxD);

// Motor shtuff

#include <Wire.h>
#include <Servo.h>
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

Servo servoHead;

uint8_t motor_ids_1[] = { 1 };
uint8_t motor_ids_2[] = { 2 };
uint8_t motor_ids_3[] = { 3 };
uint8_t motor_ids_4[] = { 4 };
uint8_t motor_ids_1_3[] = { 1, 3 };
uint8_t motor_ids_1_4[] = { 1, 4 };
uint8_t motor_ids_2_3[] = { 2, 3 };
uint8_t motor_ids_2_4[] = { 2, 4 };
uint8_t motor_ids_1_2_3_4[] = { 1, 2, 3, 4 };

int16_t forwardBackwardSpeed = 0;
int16_t leftRightSpeed = 0;

char lastRecvChar = 0;
int16_t lastForwardBackwardSpeed = 0;
int16_t lastLeftRightSpeed = 0;

int16_t headPosition = WALBOTS_HEAD_DEFAULT;

uint8_t absolute_value (int16_t in)
{
  int16_t out = in;
  uint8_t result;

  if (in < 0)
  {
    out = -in;
  }

  if (out < 0)
  {
    Serial.print("BOGUS ABSOLUTE VALUE: ");
    Serial.println(out);
  }

  if (out > WALBOTS_MAX_SPEED)
  {
    Serial.print("SPEED OUT OF BOUNDS: ");
    Serial.println(out);
  }

  result = (uint8_t) out;

  return result;
}

void printMotorIds (uint8_t motorIds[], uint8_t numMotors)
{
  uint8_t i;

  for (i = 0; i < numMotors; i++)
  {
    Serial.print(" ");
    Serial.print(motorIds[i]);
  }
}

void directMotors (Adafruit_DCMotor *someMotors[], uint8_t numMotors, uint8_t d)
{
  uint8_t i;

  for (i = 0; i < numMotors; i++)
  {
    someMotors[i]->run(d);
  }
}

void directMotorsBackward ()
{
  directMotors(motors_1, 1, BACKWARD);
  directMotors(motors_2, 1, FORWARD);
}

void directMotorsForward ()
{
  directMotors(motors_1, 1, FORWARD);
  directMotors(motors_2, 1, BACKWARD);
}

void directMotorsLeft ()
{
  directMotors(motors_3, 1, FORWARD);
  directMotors(motors_4, 1, BACKWARD);
}

void directMotorsRight ()
{
  directMotors(motors_3, 1, BACKWARD);
  directMotors(motors_4, 1, FORWARD);
}

void releaseMotorsForwardBackward ()
{
  if (lastForwardBackwardSpeed != forwardBackwardSpeed)
  {
    Serial.println("MOTORS FORWARD/BACKWARD RELEASE");
  }

  directMotors(motors_1_2, 2, RELEASE);
}

void releaseMotorsLeftRight ()
{
  if (lastLeftRightSpeed != leftRightSpeed)
  {
    Serial.println("MOTORS LEFT/RIGHT RELEASE");
  }

  directMotors(motors_3_4, 2, RELEASE);
}

void directMotors ()
{
  if (forwardBackwardSpeed > 0)
  {
    directMotorsForward();
  }
  else if (forwardBackwardSpeed < 0)
  {
    directMotorsBackward();
  }
  else
  {
    releaseMotorsForwardBackward();
  }

  if (leftRightSpeed > 0)
  {
    directMotorsRight();
  }
  else if (leftRightSpeed < 0)
  {
    directMotorsLeft();
  }
  else
  {
    releaseMotorsLeftRight();
  }
}

void releaseMotors ()
{
  Serial.println("MOTORS ALL RELEASE");
  printMotorIds(motor_ids_1_2_3_4, 4);
  directMotors(motors_1_2_3_4, 4, RELEASE);
}

void speedMotors (Adafruit_DCMotor *someMotors[], uint8_t numMotors, uint8_t s)
{
  uint8_t i;

  for (i = 0; i < numMotors; i++)
  {
    someMotors[i]->setSpeed(s);
  }
}

void speedMotors ()
{
  if (lastForwardBackwardSpeed != forwardBackwardSpeed)
  {
    Serial.print("FWD/BCK SPEED: ");
    Serial.print(forwardBackwardSpeed);
    Serial.print(" ABS: ");
    Serial.println(absolute_value(forwardBackwardSpeed));
    lastForwardBackwardSpeed = forwardBackwardSpeed;
  }

  speedMotors(motors_1_2, 2, absolute_value(forwardBackwardSpeed));

  if (lastLeftRightSpeed != leftRightSpeed)
  {
    Serial.print("LT/RT SPEED: ");
    Serial.print(leftRightSpeed);
    Serial.print(" ABS: ");
    Serial.println(absolute_value(leftRightSpeed));
    lastLeftRightSpeed = leftRightSpeed;
  }

  speedMotors(motors_3_4, 2, absolute_value(leftRightSpeed));
}

int16_t increaseSpeed (int16_t someSpeed)
{
  int16_t result = someSpeed + WALBOTS_INCREMENT;

  if (someSpeed > WALBOTS_MAX_SPEED)
  {
    result = WALBOTS_MAX_SPEED;
  }

  return result;
}

int16_t decreaseSpeed (int16_t someSpeed)
{
  int16_t result = someSpeed - WALBOTS_INCREMENT;

  if (someSpeed < WALBOTS_MIN_SPEED)
  {
    someSpeed = WALBOTS_MIN_SPEED;
  }

  return result;
}

int16_t stillSpeed (int16_t someSpeed)
{
  int16_t result;

  if (someSpeed < 0)
  {
    result = increaseSpeed(someSpeed);
  }
  else if (someSpeed > 0)
  {
    result = decreaseSpeed(someSpeed);
  }
  else
  {
    result = 0;
  }

  return result;
}

void changeSpeed (char input)
{
  switch (input) {
    case 'S':
    case 's':
      // STAY
      forwardBackwardSpeed = stillSpeed(forwardBackwardSpeed);
      leftRightSpeed = stillSpeed(leftRightSpeed);
      break;
    case 'F':
    case 'f':
      // FORWARD
      forwardBackwardSpeed = increaseSpeed(forwardBackwardSpeed);
      leftRightSpeed = stillSpeed(leftRightSpeed);
      break;
    case 'B':
    case 'b':
      // BACKWARD
      forwardBackwardSpeed = decreaseSpeed(forwardBackwardSpeed);
      leftRightSpeed = stillSpeed(leftRightSpeed);
      break;
    case 'L':
    case 'l':
      // LEFT
      forwardBackwardSpeed = stillSpeed(forwardBackwardSpeed);
      leftRightSpeed = decreaseSpeed(leftRightSpeed);
      break;
    case 'R':
    case 'r':
      // RIGHT
      forwardBackwardSpeed = stillSpeed(forwardBackwardSpeed);
      leftRightSpeed = increaseSpeed(leftRightSpeed);
      break;
    case 'G':
    case 'g':
      // LEFT FORWARD
      forwardBackwardSpeed = increaseSpeed(forwardBackwardSpeed);
      leftRightSpeed = decreaseSpeed(leftRightSpeed);
      break;
    case 'I':
    case 'i':
      // RIGHT FORWARD
      forwardBackwardSpeed = increaseSpeed(forwardBackwardSpeed);
      leftRightSpeed = increaseSpeed(leftRightSpeed);
      break;
    case 'H':
    case 'h':
      // LEFT BACKWARD
      forwardBackwardSpeed = decreaseSpeed(forwardBackwardSpeed);
      leftRightSpeed = decreaseSpeed(leftRightSpeed);
      break;
    case 'J':
    case 'j':
      // RIGHT BACKWARD
      forwardBackwardSpeed = decreaseSpeed(forwardBackwardSpeed);
      leftRightSpeed = increaseSpeed(leftRightSpeed);
      break;
    default:
      Serial.print("NO SUCH DRIVE INPUT: ");
      Serial.println(input);
      break;
  }
}

void driveMotors (char input)
{
  changeSpeed(input);
  directMotors();
  speedMotors();
}

void cycleMotors (Adafruit_DCMotor *someMotors[], uint8_t numMotors)
{
  uint8_t s;

  for (s = 0; s < WALBOTS_MAX_SPEED; s += WALBOTS_INCREMENT)
  {
    speedMotors(someMotors, numMotors, s);
    delay(WALBOTS_DELAY);
  }

  for (s = WALBOTS_MAX_SPEED; s > 0; s -= WALBOTS_INCREMENT)
  {
    speedMotors(someMotors, numMotors, s);
    delay(WALBOTS_DELAY);
  }
}

void testMotors (Adafruit_DCMotor *someMotors[], uint8_t numMotors, uint8_t motorIds[])
{
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

void turnHead (char recvChar)
{
  char charToUse = recvChar;
  uint8_t charValue;

  if (charToUse == 'q')
  {
    charToUse = '0';
  }

  charValue = charToUse - 48;

  Serial.print("HD VALUE: ");
  Serial.println(charValue);

  Serial.print("HD POSITION: ");
  Serial.println(charValue * WALBOTS_HEAD_INCREMENT);

  servoHead.write(charValue * WALBOTS_HEAD_INCREMENT);
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

  blueToothSerial.print("AT+DEFAULT\r\n");          // Restore all setup value to factory setup
  delay(2000);

  blueToothSerial.print("AT+NAMESeeedBTSlave\r\n"); // set the bluetooth name as "SeeedBTSlave" ,the length of bluetooth name must less than 12 characters.
  delay(400);

  blueToothSerial.print("AT+PIN0000\r\n");          // set the pair code to connect
  delay(400);

  blueToothSerial.print("AT+AUTH1\r\n");            //
  delay(400);

  blueToothSerial.flush();
}

// lifecycle shtuff

void setup ()
{
  Serial.begin(250000);
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);

  Serial.println("BeeBeeEight");
  Serial.println("setting up bluetooth");
  setupBlueToothConnection();
  Serial.println("done setting up bluetooth");

  // The Servo_1 and Servo_2 ports on the Adafruit MotorShield are just pins 9 and 10, respectively,
  // from the main Arduino board wired through to two different 3-pin ports (1 control pin, and +/- power pins).

  servoHead.attach(9);    // Servo_1
  servoHead.write(WALBOTS_HEAD_DEFAULT);

  AFMS.begin();       // create with the default frequency 1.6KHz
  //AFMS.begin(1000); // OR with a different frequency, say 1KHz
}

void loop ()
{
  char recvChar;

  while (1)
  {
    if (blueToothSerial.available())
    {
      // Check if there's any data sent from the remote bluetooth shield.

      recvChar = blueToothSerial.read();

      // Log what we got.

      if (lastRecvChar != recvChar)
      {
        Serial.print("received: ");
        Serial.println(recvChar);
        blueToothSerial.print("received: ");
        blueToothSerial.println(recvChar);
      }

      lastRecvChar = recvChar;

      // Process what we got.

      switch (recvChar)
      {
        case 'S': // STAY
        case 's':
        case 'F': // FORWARD
        case 'f':
        case 'B': // BACKWARD
        case 'b':
        case 'L': // LEFT
        case 'l':
        case 'R': // RIGHT
        case 'r':
        case 'G': // LEFT FORWARD
        case 'g':
        case 'I': // RIGHT FORWARD
        case 'i':
        case 'H': // LEFT BACKWARD
        case 'h':
        case 'J': // RIGHT BACKWARD
        case 'j':
          driveMotors(recvChar);
          break;

        case '0': // slider values
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
          turnHead(recvChar);
          break;

        default:
          //Serial.print("UNHANDLED INPUT: ");
          //Serial.println(recvChar);
          break;
      }

//      if (recvChar == 't' || recvChar == 'T')
//      {
//          blueToothSerial.print("temperature: ");
//          blueToothSerial.println(getTemp());
//      }
    }
//    if (Serial.available())
//    {
//      // Check if there's any data sent from the local serial terminal.
//      // You can add the other applications here.
//      recvChar = Serial.read();
//      blueToothSerial.print(recvChar);
//    }
  }
}

