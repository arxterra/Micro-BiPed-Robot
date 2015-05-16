/******************* Packet IDs ****************
 * source: Arxterra
 ***************************************************/
#define COMMAND_PACKET_ID    0xA5
#define TELEMETRY_PACKET_ID  0xCA

/******************* Robot Commands ****************
 * source: Arxterra
 ***************************************************/
// Arxterra Commands to Numeric Value Mapping
//               Data[0] =     CMD TYPE | Qual | Arguments
//                           bit  7654321   0    Bytes    N = 1 + bytes 
#define MOVE            0x01   // 0000000   1     4      
#define CAMERA_MOVE     0x02   // 0000001   0     4
#define CAMERA_HOME     0x04   // 0000010   0     0
#define CAMERA_RESET    0x05   // 0000010   1     0
#define READ_EEPROM     0x06   // 0000011   0     3 
#define WRITE_EEPROM    0x07   // 0000011   1     3 + b
#define SAFE_ROVER      0x08   // 0000100   0     0
#define SLEEP           0x0A   // 0000101   0     0
#define WAKEUP          0x0B   // 0000101   1     0
#define PING_INTERVAL   0x10   // 0001000   0
#define PING            0x11   // 0001000   1     1
#define HEADING         0x12   // 0001001   0
#define CURRENT_COORD   0x13   // 0001001   1         (float) Latitude, (float) Longitude
#define WAYPOINT_COORD  0x14   // 0001010   0
#define WAYPOINT_DELETE 0x16   // 0001011   0
#define WAYPOINT_MOVE   0x17   // 0001011   1
#define WAYPOINTS_OFF   0x18   // 0001100   0
#define WAYPOINTS_ON    0x19   // 0001100   1
#define WAYPOINTS_CLEAR 0x1A   // 0001101   0
// 0x40 - 0x5F  32 Custom Commands, 1 - 2 byte arguments

/******************* Robot Telemetry ***************
 * source: Arxterra
 ***************************************************/
// Telemetry Identifiers to Numeric Value Mapping 
#define  RANGE_LEFT_ID      0x04           // ultrasonic range 1 is left
#define  PAN_POSITION_ID    0x08           // originally defined as pan and tilt
#define  TILT_POSITION_ID   0x09           // not in original definition
#define  EEPROM_RESPONSE_ID 0x0A           // sent in response to EEPROM Read Command
#define  EMERGENCY_ID       0x0B
#define  COMMAND_ECHO_ID    0x0D
#define  EXCEPTION_ID       0x0E           // arduino error code exception
#define  PONG_ID            0x11
#define  ROUTE_STATUS_ID    0x12
#define  WAYPOINT_ARRIVE_ID 0x13

 /********************** Exception Codes ********************
 *   01    Start byte 0xA5 expected
 *   02    Packet length out of range 1 - 20
 *   03    LRC checksum error
 *   04    Undefined command decoder FSM state
 *   05    Array out of range i >= 23
 ************************************************************/
 
 /********************** Emergency Codes ********************
 *   0x0100    Watchdog timeout 0x01 with ancillary byte 0x00  
 ************************************************************/
#define WATCHDOG_TIMEOUT   0x0100


// **** Make into an Object ****
// Heartbeat
unsigned long ping_interval = 10000;  // 10 seconds
unsigned long next_ping = millis() + ping_interval;


// note: #define is a text substitution pre-processor directive
// Pintout Defined for BiPed ROFIA
//    Poser action playback
//      This program was generated by Project Biped Poser to playback a specific action.
//    Use it as the starting point for your projects!
//
//    Copyright (C) 2012  Jonathan Dowdall, Project Biped (www.projectbiped.com)
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constants
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const int led = 13;  // WARNING: assumes use of Arduino UNO not Sparfun Pro Micro
const int numberOfServos             = 12;    // the number of servos
const int numberOfJoints             = 12;
const int numberOfHeadServos         = 2;
const int numberOfFramesForward      = 126;   // Frames for moving forward
const int playbackDelayForward       = 10;    // Delay between the forward walk frames to achieve a stable speed
const int numberOfFramesNeutral      = 5;     // Frames for moving to home position
const int playbackDelayNeutral       = 20;    // Delay between the nuetral frames to achieve a stable speed
const int numberOfFramesRight        = 30;    // Delay between the right turn frames to achieve a stable speed
const int playbackDelayRight         = 25;    // Delay between the right turn frames to achieve a stable speed
const int numberOfFramesLeft         = 30;    // Frames for turning left
const int playbackDelayLeft          = 25;    // Delay between the left turn frames to achieve a stable speed
const int numberOfFramesLeftLean     = 6;     // Frames for balance the BiPed afer a force pushes left
const int numberOfFramesRightLean    = 7;     // Frames for balance the BiPed afer a force pushes right
const int playbackDelayBalance       = 15;    // Delay between the balance to the home position
const int ultrasonicConstant         = 58;    // Calculation for the distance detected by the ultrasonic sensor
                                              // Speed of sound is about 340 m/s -> 29 microseconds/cm, divide
                                              // the time taken for the ultrasonic sensor to detect by 2 to get
                                              // the time taken by the pulse to reach the object, then divide by
                                              // the ultrasonic constant to get the distance in centimeters.
const int pingPin                    = 4;     // Ultrasonic sensor pin
//const int MPU                        =0x68;   // I2C address of the MPU-6050

 
/*
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Balance System
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t stand;                                // Accelerometer Sensor Variable at Stand Position
const int leftLean                   = 1200;   // BiPed is pushed left by an external force
const int rightLean                  = -1200;  // BiPed is pushed right by an external force
const int debounceDelay              = 1200;   // Debounce Delay
unsigned long lastDebounceTime = 0;           // Time for the previous action take*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Action Frames
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Joints positions are in degrees * 100 (home position is 0 degrees)
/////////////////////////////////////////////
//JOINT INDEXES
//
//RIGHT LEG JOINTS
// 0 Right Ankle (roll)
// 1 Right Lower Leg 
// 2 Right Knee 
// 3 Right Middle Leg 
// 4 Right Upper Leg 
// 5 Right Hip (roll)
//LEFT LEG JOINTS
// 6 Left Ankle (roll)
// 7 Left Lower Leg 
// 8 Left Knee 
// 9 Left Middle Leg 
// 10 Left Upper Leg 
// 11 Left Hip (roll)
/////////////////////////////////////////////
int servoPins[numberOfServos] = {7, 8, 9, 10, 11, 12, 1, 2, 3, 4, 5, 6};  // the pin for each servo


/******************* Robot Sensors ****************
* source: Found in CommunicationRobotPilot Folder
 ***************************************************/
uint16_t sensor_value;              // Used for Current Sensor
                                    // last sensor value sent to control panel
                                    // used for comparison with current sensor value
                                    // if different data packet is sent to the control panel
uint16_t  positionScan=0;
uint16_t  positionTilt=0;

/**************** Global Variables ****************
 * source: Found in CommunicationRobotPilot Folder  *
 ***************************************************/
const int16_t FLAG = -128;            // value returned by readFuelGauge when called
                                   // by sendDataon undervoltage condition
// Timer Variable
unsigned long timer;               // unsigned 32-bit

/**************** MPU varibles ****************
 ***************************************************/
 
int val;
int prevVal;
int val1;
int prevVal1; 