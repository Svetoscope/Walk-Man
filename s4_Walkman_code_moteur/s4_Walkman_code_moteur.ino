/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/** Please refer to each DYNAMIXEL eManual(http://emanual.robotis.com) for supported Operating Mode
 * Operating Mode
 *  1. OP_POSITION                (Position Mode in protocol2.0, Joint Mode in protocol1.0)
 *  2. OP_VELOCITY                (Velocity Mode in protocol2.0, Speed Mode in protocol1.0)
 *  3. OP_PWM                     (PWM Mode in protocol2.0)
 *  4. OP_EXTENDED_POSITION       (Extended Position Mode in protocol2.0, Multi-turn Mode(only MX series) in protocol1.0)
 *  5. OP_CURRENT                 (Current Mode in protocol2.0, Torque Mode(only MX64,MX106) in protocol1.0)
 *  6. OP_CURRENT_BASED_POSITION  (Current Based Postion Mode in protocol2.0 (except MX28, XL430))
 */


/*
  Message read from Pi: <(letter describing movement to do)>
  Feedback message (To Pi): <(current movement letter)>
  Possible letters for movements:
    W: Walking
    S: stand
    N: no movement
    F: feedback requested
    T: launch local test
*/


#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

// Constant variables 
const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const uint8_t DXL_ID3 = 3;
const uint8_t SWITCH_LEG1 = 6; //pin number for limit switch on right leg
const uint8_t SWITCH_LEG2 = 7; //pin number for limit switch on left leg
const float HOME_DXL1 = 345; // Home position for DXL_ID1. To calibrate 
const float HOME_DXL2 = 2839; // Home position for DXL_ID2. To calibrate
const float MIDDLE_DXL3 = 2607;
const float DESIRED_SPEED = 60; // Desired speed for the motor in motion. Adjust to liking 
const float DXL_PROTOCOL_VERSION = 2.0;
const uint8_t MAXCHARS = 30;
const float LIMIT = 110; //Current limit for pendulum motor
const int PEND_LEG1 = 1;
const int PEND_LEG2 = 2;
const int INIT_POS = 3;

 
// Global variables
bool init_pos = true; // True until all motors have been initialised and are at home position
bool first = false; // True for the first cycle of loop only
bool leg1_walk = false;
bool leg2_walk = false;
bool readInProgress = false;
bool walking = false;
float dxl3_pos1 = 0; // Position of DXL_ID3 when the pendulum is on the side of leg 2. To calibrate
float dxl3_pos2 = 0;
float dxl3_middle = 0;
float leg_speed = 0;
char moveQueue[MAXCHARS];
char state = 'P';
int queueLength = 0;
int counter = 0;


Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // Use Serial to debug.
  DEBUG_SERIAL.begin(9600);

  Serial.begin(115200); // ***** 

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Get DYNAMIXEL information
  dxl.ping(DXL_ID1);
  dxl.ping(DXL_ID2);
  dxl.ping(DXL_ID3);

  // Puts limit switches pins in input_pullup mode
  pinMode(SWITCH_LEG1, INPUT_PULLUP);
  pinMode(SWITCH_LEG2, INPUT_PULLUP);
}

void loop() {
  // Only does this function once
  if(init_pos){
    position_init(); // Sets motors to their home position
  }
  if(queueLength != 0){
    state = moveQueue[0];
    removeFirstInQueue();
    counter = 0;
  }

  switch(state){
    case 'P':
      // Stop the robot
      break;

    case 'A':
      // Do one complete cycle of walk at slow speed
      leg_speed = DESIRED_SPEED;
      pendulum_leg1();
      leg1_walking();
      delay(50);
      leg1_walking();
      pendulum_leg2();
      leg2_walking();
      delay(50);
      leg2_walking();
      break;

    case 'B':
      // Do one complete cycle of walk at medium speed
      leg_speed = DESIRED_SPEED + 20*1;
      pendulum_leg1();
      leg1_walking();
      delay(50);
      leg1_walking();
      pendulum_leg2();
      leg2_walking();
      delay(50);
      leg2_walking();
      break;

    case 'C':
      // Do one complete cycle of walk at fast speed
      leg_speed = DESIRED_SPEED + 20*2;
      pendulum_leg1();
      leg1_walking();
      delay(50);
      leg1_walking();
      pendulum_leg2();
      leg2_walking();
      delay(50);
      leg2_walking();
      break;

    case 'N':
      // Standing position is the position of initialization 
      leg_speed = DESIRED_SPEED;
      pendulum_leg1();
      leg1_walking();
      state = 'P';
      break;

    case 'T':
      // Launch test sequence
      doTest();
      break;

    default:
      break;
  }

  if(Serial.available()){
    readSerial();
  }
}

void position_init(){
  // Used to set the motors to their home position
  dxl.torqueOff(DXL_ID1);
  dxl.setOperatingMode(DXL_ID1, OP_POSITION); //Sets motor to position mode
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 60); // Sets maximum speed of motor
  dxl.torqueOn(DXL_ID1);
  // Sends motor to a certain position (home in this case)
  if(dxl.setGoalPosition(DXL_ID1, HOME_DXL1)){
    delay(1000);
  }

  dxl.torqueOff(DXL_ID2);
  dxl.setOperatingMode(DXL_ID2, OP_POSITION); // Sets motor to position mode
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 60); // Sets maximum speed of motor
  dxl.torqueOn(DXL_ID2);
  // Sends motor to a certain position (home in this case)
  if(dxl.setGoalPosition(DXL_ID2, HOME_DXL2)){
    delay(1000);
  }

  dxl.torqueOff(DXL_ID3);
  dxl.setOperatingMode(DXL_ID3, OP_VELOCITY); // Sets motor to velocity mode
  // Pendulum initialization if it's over leg 1
  if(digitalRead(SWITCH_LEG1) == LOW){
    dxl3_pos1 = dxl.getPresentPosition(DXL_ID3);
    while(digitalRead(SWITCH_LEG2) != LOW){
      if(dxl.getPresentCurrent(DXL_ID3) < LIMIT){
        dxl.torqueOn(DXL_ID3);
        dxl.setGoalVelocity(DXL_ID3, 20);
      }else{
        errorMessage(INIT_POS);
      }
    }
    dxl.torqueOff(DXL_ID3);
    dxl3_pos2 = dxl.getPresentPosition(DXL_ID3); // Used to find vertical position of pendulum
  }
  // Pendulum initialization if it's over leg 2
  else if(digitalRead(SWITCH_LEG2) == LOW){
    dxl3_pos2 = dxl.getPresentPosition(DXL_ID3);
    while(digitalRead(SWITCH_LEG1) != LOW){
      if(dxl.getPresentCurrent(DXL_ID3) < LIMIT){
        dxl.torqueOn(DXL_ID3);
        dxl.setGoalVelocity(DXL_ID3, -20);
      }else{
        errorMessage(INIT_POS);
      }
    }
    dxl.torqueOff(DXL_ID3);
    dxl3_pos1 = dxl.getPresentPosition(DXL_ID3); // Used to find vertical position of pendulum
  }
  else{
    dxl.torqueOff(DXL_ID3);
  }

  // Set motors to velocity mode 
  dxl.torqueOff(DXL_ID1);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 1000);
  dxl.writeControlTableItem(VELOCITY_LIMIT, DXL_ID1, 100);
  dxl.setOperatingMode(DXL_ID1, OP_VELOCITY);
  dxl.torqueOn(DXL_ID1);

  dxl.torqueOff(DXL_ID2);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 1000);
  dxl.writeControlTableItem(VELOCITY_LIMIT, DXL_ID2, 100);
  dxl.setOperatingMode(DXL_ID2, OP_VELOCITY);
  dxl.torqueOn(DXL_ID2);

  dxl.torqueOff(DXL_ID3);
  dxl.setOperatingMode(DXL_ID3, OP_VELOCITY);
  dxl.torqueOn(DXL_ID3);

  init_pos = false; // Disables this function
  DEBUG_SERIAL.println("end init");

  
  if(dxl3_pos1 < 0 || dxl3_pos2 < 0){
    dxl3_pos1 = dxl3_pos1 + 4095;
    dxl3_pos2 = dxl3_pos2 + 4095;
  }
  float ecart = dxl3_pos2-dxl3_pos1;
  // Finds the vertical position of pendulum
  if((int(ecart) % 2) == 0){
    dxl3_middle = ecart/2;
  }else{
    dxl3_middle = (ecart-1)/2;
  }
  counter++;
}

void leg1_walking(){
  DEBUG_SERIAL.println("leg1");
  // Used to complete the walking cycle of leg1
  first = true;
  walking = true;
  int pos_start;
  int pos_end;
  while(walking){
    if(first){
      pos_end = dxl.getPresentPosition(DXL_ID1) + 4095; // End position after one complete cycle
      first = false;
    }
    // End of cycle conditions
    if((dxl.getPresentPosition(DXL_ID1) > (pos_end - 100)) && (dxl.getPresentPosition(DXL_ID1) < (pos_end + 100))){
      dxl.setGoalVelocity(DXL_ID1, 0);
      walking = false;
      break;
    }
    dxl.torqueOn(DXL_ID1);
    dxl.setGoalVelocity(DXL_ID1, leg_speed);
  }
}

void leg2_walking(){
  DEBUG_SERIAL.println("leg2");
  // Used to complete the walking cycle of leg2
  first = true;
  walking = true;
  int pos_start;
  int pos_end;
  while(walking){
    if(first){
      pos_end = dxl.getPresentPosition(DXL_ID2) - 4095; // End position after one complete cycle
      first = false;
    }
    // End of cycle conditions
    if((dxl.getPresentPosition(DXL_ID2) > (pos_end - 100)) && (dxl.getPresentPosition(DXL_ID2) < (pos_end + 100))){
      //dxl.torqueOff(DXL_ID2);
      dxl.setGoalVelocity(DXL_ID2, 0);
      walking = false;
      break;
    }
    dxl.torqueOn(DXL_ID2);
    dxl.setGoalVelocity(DXL_ID2, -1*leg_speed); // Negative speed for counter clock wise rotation
  }
}

void pendulum_leg1(){
  DEBUG_SERIAL.println("pendulum1");
  if(digitalRead(6) == LOW){
    while(digitalRead(7) != LOW){
      if(dxl.getPresentCurrent(DXL_ID3) < LIMIT){ // Condition for current limit in motor
        dxl.torqueOn(DXL_ID3);
        dxl.setGoalVelocity(DXL_ID3, getPendulumSpeed(dxl.getPresentPosition(DXL_ID3))); // Speed of pendulum proportionnal to its position
      }else{
        errorMessage(PEND_LEG1);
      }

    }
    dxl.torqueOff(DXL_ID3);
  }
}

void pendulum_leg2(){
  // Used to place the pendulum over leg1
  DEBUG_SERIAL.println("pendulum2");
  if(digitalRead(7) == LOW){
    while(digitalRead(6) != LOW){
      if(dxl.getPresentCurrent(DXL_ID3) < LIMIT){ // Condition for current limit in pendulum
        dxl.torqueOn(DXL_ID3);
        dxl.setGoalVelocity(DXL_ID3, getPendulumSpeed(dxl.getPresentPosition(DXL_ID3)) * -1); // Speed of pendulum proportionnal to its position
      }else{
        errorMessage(PEND_LEG2);
      }
    }
    dxl.torqueOff(DXL_ID3);
  }
}

float getPendulumSpeed(float pos){
  // Returns a speed proportionnal to position that is in argument
  float speed = (abs(dxl3_middle - pos)/dxl3_middle + 1) * 20;
   
  return speed;
}

void addToQueue(char charac) {
  moveQueue[queueLength] = charac;
  queueLength++;
}

void readSerial(){
  char startMarker = '<';
  char endMarker = '>';
  char lastCharRead;
  char msgToSend[3] = {'<', 'N', '>'};

  while(Serial.available()){
    lastCharRead = Serial.read();
  
    if(readInProgress){
      if(lastCharRead != endMarker){
        
        if(lastCharRead == 'F'){
          msgToSend[1] = 'R';
          state = 'P';
        }else{
          msgToSend[1] = lastCharRead;
          addToQueue(lastCharRead);
          DEBUG_SERIAL.print("adding to queue: ");
          DEBUG_SERIAL.println(lastCharRead);
        }
        DEBUG_SERIAL.print("msg to send: ");
        DEBUG_SERIAL.println(msgToSend);
        Serial.println(msgToSend);
      } 
    }

    if(lastCharRead == startMarker){
      readInProgress = true;
    }
    if(lastCharRead == endMarker){
      break;
    }
  }
  readInProgress = false;
}

void removeFirstInQueue() {
  for(int i = 0; i < queueLength-1; i++){
    moveQueue[i] = moveQueue[i+1];
  }
  queueLength--;
  moveQueue[queueLength] = 'N';
}


void doTest(){

}

void errorMessage(int location){
  bool messageWritten = false;
  while(true){
    dxl.torqueOff(DXL_ID1);
    dxl.torqueOff(DXL_ID2);
    dxl.torqueOff(DXL_ID3);

    if(!messageWritten){
      switch(location){
        case PEND_LEG1:
          Serial.println("Motor 3 has reached the current limit during the pendulum for leg 1 movement (pendulum_leg1) and is now in emergency stop");
          messageWritten = true;
          break;

        case PEND_LEG2:
          Serial.println("Motor 3 has reached the current limit during the pendulum for leg 2 movement (pendulum_leg2) and is now in emergency stop");
          messageWritten = true;
          break;

        case INIT_POS:
          Serial.println("Motor 3 has reached the current limit during the initialization (position_init) and is now in emergency stop");
          messageWritten = true;
          break;

        default:
          break;
      }
    }
  }
}








