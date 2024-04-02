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

const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const uint8_t DXL_ID3 = 3;
const float DXL_PROTOCOL_VERSION = 2.0;
 
bool init_pos = true;
bool first = false;
bool leg1_walk = false;
bool leg2_walk = false;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // Use Serial to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID1);
  dxl.ping(DXL_ID2);
  dxl.ping(DXL_ID3);
}

void loop() {
  position_init();
  pendulum_leg1();
  leg1_walking();
  pendulum_leg2();
  leg2_walking();
}

void position_init(){
  if (init_pos){
    dxl.torqueOff(DXL_ID1);
    dxl.setOperatingMode(DXL_ID1, OP_POSITION);
    dxl.torqueOn(DXL_ID1);
    if(dxl.setGoalPosition(DXL_ID1, 1840)){
      delay(1000);
    }

    dxl.torqueOff(DXL_ID2);
    dxl.setOperatingMode(DXL_ID2, OP_POSITION);
    dxl.torqueOn(DXL_ID2);
    if(dxl.setGoalPosition(DXL_ID2, 1750)){
      delay(1000);
    }

    dxl.torqueOff(DXL_ID1);
    dxl.setOperatingMode(DXL_ID1, OP_VELOCITY);
    dxl.torqueOn(DXL_ID1);

    dxl.torqueOff(DXL_ID2);
    dxl.setOperatingMode(DXL_ID2, OP_VELOCITY);
    dxl.torqueOn(DXL_ID2);

    dxl.torqueOff(DXL_ID3);
    dxl.setOperatingMode(DXL_ID3, OP_VELOCITY);
    dxl.torqueOn(DXL_ID3);

    Serial.print("Position 1:"); Serial.println(dxl.getPresentPosition(DXL_ID1));
    Serial.print("Position 2:"); Serial.println(dxl.getPresentPosition(DXL_ID2));
    Serial.print("Position 3:"); Serial.println(dxl.getPresentPosition(DXL_ID3));

    init_pos = false;
  }
}

void leg1_walking(){
  first = true;
  int pos_start;
  int pos_end;
  while(leg1_walk && !leg2_walk){
    if(first){
      pos_end = dxl.getPresentPosition(DXL_ID1) + 4095;
      first = false;
    }
   

    if((dxl.getPresentPosition(DXL_ID1) > (pos_end - 100)) && (dxl.getPresentPosition(DXL_ID1) < (pos_end + 100))){
      dxl.torqueOff(DXL_ID1);
      leg1_walk = false;
      leg2_walk = false;
      break;
    }
    dxl.torqueOn(DXL_ID1);
    dxl.setGoalVelocity(DXL_ID1, 60);
  }
}

void leg2_walking(){
  first = true;
  int pos_start;
  int pos_end;
  while(!leg1_walk && leg2_walk){
    if(first){
      pos_end = dxl.getPresentPosition(DXL_ID2) - 4095;
      first = false;
    }

    if((dxl.getPresentPosition(DXL_ID2) > (pos_end - 100)) && (dxl.getPresentPosition(DXL_ID2) < (pos_end + 100))){
      dxl.torqueOff(DXL_ID2);
      leg1_walk = false;
      leg2_walk = false;
      break;
    }
    dxl.torqueOn(DXL_ID2);
    dxl.setGoalVelocity(DXL_ID2, -60);
  }
}

void pendulum_leg1(){
  
  if(!leg1_walk && !leg2_walk){
    
    dxl.torqueOn(DXL_ID3);
    dxl.setGoalVelocity(DXL_ID3, -100);
    Serial.println("1");
    leg1_walk = true;
    leg2_walk = false;
  }
  while(!((dxl.getPresentPosition(DXL_ID3) > (1300 - 100)) && (dxl.getPresentPosition(DXL_ID3) < (1300 + 100)))){
    
  }
  dxl.torqueOff(DXL_ID3);
}

void pendulum_leg2(){
  
  if(!leg1_walk && !leg2_walk){
    
    dxl.torqueOn(DXL_ID3);
    dxl.setGoalVelocity(DXL_ID3, 100);
    Serial.println("2");
    leg1_walk = false;
    leg2_walk = true;
  }
  while(!((dxl.getPresentPosition(DXL_ID3) > (5950 - 100)) && (dxl.getPresentPosition(DXL_ID3) < (5950 + 100)))){
    
  }
  dxl.torqueOff(DXL_ID3);
}











