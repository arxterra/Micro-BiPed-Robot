/* The following two libraries are included with the Arduino IDE  */
#include <SPI.h>                        // supports SPI serial communication and defines analog pins
#include <EEPROM.h>
#include <Servo.h>
#include <Wire.h>                       // I2C support (currently not implemented)
//#include <L3G4200D.h>                   // 3-axis Gyro (currently not implemented)
#include <SoftwareSerial.h>             // import the serial library
#include "Pinouts_and_Define_ROFIA.h"   // Pinout for microBiPed and microSpider

//Comment in if the microBiPed is being used

#include <avr/pgmspace.h>
#include "table_biped.h"
#include "Tlc5940.h"
#include "tlc_servos_atmega_32_u.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

//Comment in if the microSpiderbot is being used
/*
#include <Adafruit_PWMServoDriver.h>
#include <PCA9685.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
*/
/**************** Robot Configuration **************
 * Set Rover, Pinouts, Connection Type, and Pinger *
 ***************************************************/
#define FALSE 0               // LOW
#define TRUE  1               // HIGH
#define microBiPed TRUE
#define microSpider FALSE
#define bluetooth TRUE       // Leonardo class rovers - Serial used for USB, Serial1 for bluetooth (i.e., USART)
#define debug  TRUE

/**************** MPU varibles ****************
 ***************************************************/
 #if microBiPed
 
MPU6050 mpu;
 
int16_t ax, ay, az;
int16_t gx, gy, gz;

#endif

void setup()
{

    
  #if bluetooth
  Serial1.begin(9600);
  #endif 
  Serial.begin(9600);                  // legacy rovers operated at 57600
  
 
  timer = millis();                     // telemetry  
  
  
  #if microBiPed
  
  setup_MicroBiPed();
  
 mpu.initialize();
 
  #elif microSpider
  
  spider_setup();

  //Serial.println("done set-up");
  
  #endif
}

void loop()
{

  #if bluetooth
  if(Serial1.available() ) commandDecoder();      // note: Leonardo does not support serialEvent() handler
  #else
  if(Serial.available() ) commandDecoder();
  #endif
  
  // future: replace with watchdog timer interrupt  ****
  if (millis() > getNextPing())
  {
    sendWordPacket(EMERGENCY_ID,WATCHDOG_TIMEOUT);
    
    #if bluetooth                                 // if packet sent over USART=>bluetooth,  
    Serial.print("Emergency exception 0x0");      // send duplicate data as text to
    Serial.println(WATCHDOG_TIMEOUT,HEX);         // USB=>Arduino IDE Serial Monitor.
    #endif 
    
    // ****** SLEEP PAPERBOT *******
    // ##### JEFF CHANGED
    // Until the above-mentioned "SLEEP PAPERBOT" is implemented,
    // let's reset to allow another ping interval to avoid the
    // constant barrage of an exception message on every loop.
    updateNextPing();
    // #####
  }
  #if microBiPed
  
    balance(); //controls orientation
  
  #endif
}


