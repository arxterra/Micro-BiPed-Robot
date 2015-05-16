
void move_BiPed(uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6)
{
  #if microBiPed
  
  // Distance detected by the ultrasonic sensor
  unsigned long cm = DetectDistance(pingPin);
  
  #endif
  
  sendData(); 
   
  /***********************************
  * motion command = 0x01
  * motordata[3]   left run    (FORWARD = index 1, BACKWARD = index 2, BRAKE = index 3, RELEASE = index 4)
  * motordata[4]   left speed  0 - 255
  * motordata[5]   right run   (FORWARD, BACKWARD, BRAKE, RELEASE) 
  * motordata[6]   right speed 0 - 255
  * example
  * forward half speed  0x01, 0x01, 0x80, 0x01, 0x80 0101800180
  ***********************************/

  if (data3 == 1 && data5 == 1)
  {
    #if microBiPed
    // The robot will walk forward, but will turn right if there is an obstacle 30 cm or less in front of it
    if(cm >= 43)
    {
      PlayFrames(numberOfFramesForward, playbackDelayForward, "forward");
//      Serial.println("Walk Forward");
    }
    else
    {
      for (int i = 0; i < 4; i++)
      {
        PlayFrames(numberOfFramesRight, playbackDelayRight, "right");
//        Serial.println("Turn Rignt");
      }
    }
    PlayFrames(numberOfFramesNeutral, playbackDelayNeutral, "home"); PlayFrames(numberOfFramesForward, playbackDelayForward, "forward");
    
    #elif microSpider
    
    for (int i = 0; i < 4; i++)
    {
    walkfwd();
    Serial.println("Forward");
    }
    stand();
    
    #endif  
     

  }
  else if (data3 == 4 && data5 == 4 && data4 == 0 && data6 == 0)
  {
    // The robot will go back to home position
    #if microBiPed
    
    PlayFrames(numberOfFramesNeutral, playbackDelayNeutral, "home");
//    Serial.println("Stay Stand");

    #elif microSpider
    
    stand();
    Serial.println("Stay Stand");
    
    #endif

   
  }
  else if ((data3 == 1 && data5 != 1)||(data3 == 1 && data5 == 2))
  {
     // The robot move right
    #if microBiPed 
    
    for (int i = 0; i < 4; i++)
    {
      PlayFrames(numberOfFramesRight, playbackDelayRight, "right");
//      Serial.println("Turn Rignt"); 
    }
    PlayFrames(numberOfFramesNeutral, playbackDelayNeutral, "home");
    
    #elif microSpider
    
    turnright();
    Serial.println("Turn Right");
    
    #endif
     
  }
  else if ((data3 != 1 && data5 == 1)||(data3 == 2 && data5 == 1))
  {
    // The robot move left
   #if microBiPed
   for (int i = 0; i < 4; i++)
    {
       PlayFrames(numberOfFramesLeft, playbackDelayLeft, "left");
//      Serial.println("Turn Leftt");
    }
    PlayFrames(numberOfFramesNeutral, playbackDelayNeutral, "home");
    #elif microSpider
    turnleft();
    Serial.println("Turn Leftt");
    #endif
    
  }
  else if (data3 == 2 && data5 == 2)
  #if microBiPed
  
   Serial.println("walk back");
   
  #elif microSpider
      for (int i = 0; i < 4; i++)
    {
    walkbwd();
    Serial.println("Walk Back");
    }
    stand();  
  #endif
  
  delay(2000);  
}

#if microBiPed

void setup_MicroBiPed(){
     //initialize the servos
   tlc_initServos();  // Note: this will drop the PWM freqency down to 50Hz
   
   //initialize MPU 6050
   Wire.begin();
   mpu.initialize();
  
  //initializeServos();
  PlayFrames(numberOfFramesNeutral, playbackDelayNeutral, "home");
  
   // Delay to give the user time to set the robot down after the legs straighten out
  delay(2000);
}

void PlayFrames(int numberOfFrames, int playbackDelay, String dir)
{
  // Angle value of the servo
  int value;
  // This for loop determines the animation frame the robot is playing through
  for (int framesRowNumber = 0; framesRowNumber < numberOfFrames; framesRowNumber++)
  {
    // This for loop adjusts the position of each servo
    for (int servo = 0; servo < numberOfServos; servo++)
    {
      // each servo position is sent as a 2 byte value (high byte, low byte) integer (from -32,768 to 32,767)
      // this number is encoding the angle of the servo. The number is 100 * the servo angle.  This allows for the
      // storage of 2 significant digits(i.e. the value can be from -60.00 to 60.00 and every value in between).
      // Also remember that the servos have a range of 120 degrees. The angle is written in positions
      // which range from a minimum of 800 (-60 degrees) and go to a maximum of 2200 (60 degrees)
      // This branch determines whether or not the robot is walking forward or turning left based on
      // the parameters passed in the main loop
      if (numberOfFrames == numberOfFramesForward)      
      {
        value = framesForward[framesRowNumber][servo];
      }
      else if (numberOfFrames == numberOfFramesNeutral) 
      {
        value = framesNeutral[framesRowNumber][servo];
      }
      else if (numberOfFrames == numberOfFramesRight && dir == "right")   
      {
        value = framesRight[framesRowNumber][servo];
      }
      else if (numberOfFrames == numberOfFramesLeft && dir == "left")    
      {
        value = framesLeft[framesRowNumber][servo];
      }
      else if (numberOfFrames == numberOfFramesLeftLean && dir == "leftLean")   
      {
        value = framesLeftLean[framesRowNumber][servo];
      }
      else if (numberOfFrames == numberOfFramesRightLean && dir == "rightLean")  
      {
        value = framesRightLean[framesRowNumber][servo];
      }
      else 
      {
        value = framesForward[framesRowNumber][servo];
      }

      // flip for the left leg.
      if(servo >= numberOfServos/2) value = map(value, -6000,6000,6000,-6000);

      // tell servo to go to position in variable 'pos'
      tlc_setServo(servo, map(value, -6000,6000,800,2200));
      Tlc.update();
      //delay(20);
    }
  // This delay controls the delay between each frame
  // This will vary based on the animation and may need to be changed if you make your own
  // animation with a different speed.
  delay(playbackDelay);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Ultrasonic Sensor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
long DetectDistance(int pingPin)
{
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  long duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  long cm = duration / ultrasonicConstant;
  return cm;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Balance
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void balance() 
{
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 
    val = map(ax, -17000, 17000, 0, 179);
    val1 = map(ay, 17000, -17000, 0, 179);
    Serial.println(val);
    if (val != prevVal)
    {
      tlc_setServo(0, val);
      Tlc.update();
      delay(20);
      tlc_setServo(1, val);
      Tlc.update();
      delay(20);
      prevVal = val;
    }
    if (val1 != prevVal1)
    {
      tlc_setServo(3, val1);
      Tlc.update();
      delay(20);
      tlc_setServo(4, val1);
      Tlc.update();
      delay(20);
      prevVal1 = val1;
    }
 
    delay(50);
}

#elif microSpider

void spider_setup() 
{
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  pwm1.begin();
  pwm1.setPWMFreq(50);
  stand();
  
}

void stand(){
  //Front Left
  pwm.setPWM(8,0,200);
  pwm.setPWM(7,0,300);
  pwm.setPWM(6,0,480);
  
 // Back Left 
 pwm.setPWM(2,0,175);
 pwm.setPWM(1,0,300);
 pwm.setPWM(0,0,450);
 
 //Middle Right
 pwm1.setPWM(5,0,200);
 pwm1.setPWM(4,0,350);
 pwm1.setPWM(3,0,125);
 
 //Front Right
 pwm1.setPWM(8,0,175);
 pwm1.setPWM(7,0,350);
 pwm1.setPWM(6,0,150);
 
 //Back Right
 pwm1.setPWM(2,0,225);
 pwm1.setPWM(1,0,350);
 pwm1.setPWM(0,0,150);
 
 //Middle Left
 pwm.setPWM(5,0,200);
 pwm.setPWM(4,0,300);
 pwm.setPWM(3,0,475);
}

void walkfwd(){
 tri1();
 tri2();
 tri3();
 tri4();
}

void walkbwd(){
  tri4();
  tri3();
  tri2();
  tri1();
}

void turnright(){
  right1();
  right2();
  right3();
  right4();
}

void turnleft(){
  left1();
  left2();
  left3();
  left4();
}

void tibia(){
    //TIBIA 90
  //LEFT SIDE
  pwm.setPWM(6,0,450);
  pwm.setPWM(0,0,450);
  pwm1.setPWM(3,0,125);
  //RIGHT SIDE
  pwm1.setPWM(6,0,150);
  pwm1.setPWM(0,0,125);
  pwm.setPWM(3,0,450);
}

void tri1(){
  tibia();
  //FEMUR UP
  pwm.setPWM(1,0,350);
  pwm.setPWM(1,0,350);
  pwm1.setPWM(4,0,300);
  
  pwm1.setPWM(7,0,350);
  pwm1.setPWM(1,0,350);
  pwm.setPWM(4,0,300);
  delay(500);
}

void tri2(){
  tibia();
  //HIP FORWARD
  pwm.setPWM(8,0,150);
  pwm.setPWM(2,0,150);
  pwm1.setPWM(5,0,275);
  
  pwm1.setPWM(8,0,150);
  pwm1.setPWM(2,0,200);
  pwm.setPWM(5,0,225);
  delay(500);
}

void tri3(){
  tibia();
  //FEMUR DOWN
  pwm.setPWM(7,0,300);
  pwm.setPWM(1,0,300);
  pwm1.setPWM(4,0,350);
  
  pwm1.setPWM(7,0,300);
  pwm1.setPWM(1,0,300);
  pwm.setPWM(4,0,350);
  delay(500);
}

void tri4(){
  tibia();
  //HIP BACK
  pwm.setPWM(8,0,225);
  pwm.setPWM(2,0,225);
  pwm1.setPWM(5,0,200);
  
  pwm1.setPWM(8,0,225);
  pwm1.setPWM(2,0,275);
  pwm.setPWM(5,0,150);
  delay(500);
}

void right1(){
  tibia();
  pwm.setPWM(7,0,350);
  pwm.setPWM(1,0,350);
  pwm1.setPWM(4,0,350);
  
  pwm1.setPWM(7,0,300);
  pwm1.setPWM(1,0,300);
  pwm.setPWM(4,0,300);
  delay(500);
}
void right2(){
  tibia();
  pwm.setPWM(8,0,150);
  pwm.setPWM(2,0,150);
  pwm1.setPWM(5,0,275);
  
  pwm1.setPWM(8,0,150);
  pwm1.setPWM(2,0,200);
  pwm.setPWM(5,0,225);
  delay(500);
}

void right3(){
  tibia();
  pwm1.setPWM(7,0,300);
  pwm1.setPWM(1,0,300);
  pwm1.setPWM(4,0,300);
  
  pwm1.setPWM(7,0,350);
  pwm1.setPWM(1,0,350);
  pwm.setPWM(4,0,350);
  delay(500);
}

void right4(){
  tibia();
  pwm1.setPWM(8,0,225);
  pwm1.setPWM(2,0,225);
  pwm1.setPWM(5,0,200);
  
  pwm1.setPWM(8,0,225);
  pwm1.setPWM(2,0,275);
  pwm.setPWM(5,0,150);
  delay(500);
}

void left1(){
  //FEMUR UP
  tibia();
  pwm1.setPWM(7,0,300);
  pwm1.setPWM(1,0,300);
  pwm1.setPWM(4,0,300);
  
  pwm1.setPWM(7,0,350);
  pwm1.setPWM(1,0,350);
  pwm.setPWM(4,0,350);
  delay(500);
}

void left2(){
  tibia();
  pwm1.setPWM(8,0,150);
  pwm1.setPWM(2,0,150);
  pwm1.setPWM(5,0,275);
  
  pwm1.setPWM(8,0,150);
  pwm1.setPWM(2,0,200);
  pwm.setPWM(5,0,225);
  delay(500);
}

void left3(){
  tibia();
  pwm1.setPWM(7,0,350);
  pwm1.setPWM(1,0,350);
  pwm1.setPWM(4,0,350);
  
  pwm1.setPWM(7,0,300);
  pwm1.setPWM(1,0,300);
  pwm.setPWM(4,0,300);
  delay(500);
}

void left4(){
  tibia();
  pwm1.setPWM(8,0,225);
  pwm1.setPWM(2,0,225);
  pwm1.setPWM(5,0,200);
  
  pwm1.setPWM(8,0,225);
  pwm1.setPWM(2,0,275);
  pwm.setPWM(5,0,150);
  delay(500);
}
 

#endif

