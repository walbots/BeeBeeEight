// This is a combination of the Adafruit DCMotorTest sketch and the SeeedStudio Slave_temperature sketch.

// Seeed

#include <SoftwareSerial.h>   //Software Serial Port

// AdaFruit

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Seeed

#define RxD 3
#define TxD 2

// AdaFruit

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

// Seeed

#define DEBUG_ENABLED  1

#define PIN_TEMP    A5


SoftwareSerial blueToothSerial(RxD,TxD);

/***************************************************************************
 * Function Name: getTemp
 * Description:  get the temperature data
 * Parameters:  
 * Return: 
***************************************************************************/

int getTemp()
{
    int a = analogRead(PIN_TEMP);
    int B=3975;
    float resistance = (float)(1023-a)*10000/a;
    float temperature = temperature=1/(log(resistance/10000)/B+1/298.15)-273.15;
    
    return (int)temperature;
    
}

void setupBlueToothConnection()
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


void setup() {
  // AdaFruit

  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);

  // Seeed

  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);
  setupBlueToothConnection();
}

void loop() {

  // Seeed

  char recvChar;
  while(1)
  {
      if(blueToothSerial.available())
      {//check if there's any data sent from the remote bluetooth shield
          recvChar = blueToothSerial.read();
          Serial.print(recvChar);
          
          if(recvChar == 't' || recvChar == 'T')
          {
              blueToothSerial.print("temperature: ");
              blueToothSerial.println(getTemp());

              // AdaFruit
            
              uint8_t i;
              
              Serial.print("tick");
            
              myMotor->run(FORWARD);
              for (i=0; i<255; i++) {
                myMotor->setSpeed(i);  
                delay(10);
              }
              for (i=255; i!=0; i--) {
                myMotor->setSpeed(i);  
                delay(10);
              }
              
              Serial.print("tock");
            
              myMotor->run(BACKWARD);
              for (i=0; i<255; i++) {
                myMotor->setSpeed(i);  
                delay(10);
              }
              for (i=255; i!=0; i--) {
                myMotor->setSpeed(i);  
                delay(10);
              }
            
              Serial.print("tech");
              myMotor->run(RELEASE);
              delay(1000);
          }
      }
      if(Serial.available())
      {//check if there's any data sent from t he local serial terminal, you can add the other applications here
          recvChar  = Serial.read();
          blueToothSerial.print(recvChar);
      }
  }
}

