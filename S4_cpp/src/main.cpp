#include "Arduino.h"
#include <Servo.h>
/*
  Message read from Pi: <(letter describing movement to do)>
  Feedback message (To Pi): <(current movement letter)>
  Possible letters for movements:
    L: move left leg
    R: move right leg
    S: stand
    N: no movement
    F: feedback requested
*/
#define button1_PIN 10
#define button2_PIN 11

const int maxChars = 30;
char moveQueue[maxChars];
int queueLength = 0;
bool readInProgress = false;
int compteur;
Servo leftHip;
Servo rightHip;



//Headers
void removeFirstInQueue();
void addToQueue(char charac);
void doNextMovement();
void moveLeftLeg();
void moveRightLeg();
void standUp();


void setup() {
  Serial.begin(9600);
  moveQueue[0] = 'N';
  leftHip.attach(12);
  rightHip.attach(13);
  pinMode(button1_PIN, INPUT_PULLUP);
  pinMode(button2_PIN, INPUT_PULLUP);
}

void loop() {
  delay(50);
}

void removeFirstInQueue() {
  for(int i = 0; i < queueLength-1; i++){
    moveQueue[i] = moveQueue[i+1];
  }
  queueLength--;
  moveQueue[queueLength] = 'N';
}

void addToQueue(char charac) {
  moveQueue[queueLength] = charac;
  queueLength++;
}

void serialEvent() {
  char startMarker = '<';
  char endMarker = '>';
  char lastCharRead;
  char msgToSend[3] = {'<', 'N', '>'};

  while(Serial.available()){
    delay(50);
    lastCharRead = Serial.read();
  
    if(readInProgress){
      //Serial.println("Read in is true");
      if(lastCharRead != endMarker){
        
        if(lastCharRead == 'F'){
          msgToSend[1] = moveQueue[0];
        }
        else{
          //Serial.println("Une lettre autre que F");
          msgToSend[1] = lastCharRead;
          addToQueue(lastCharRead);
        }
        Serial.println(msgToSend);
      } 
    }

    if(lastCharRead == startMarker){
      readInProgress = true;
    }
  }
  readInProgress = false;
  
}

void doNextMovement(){
  switch(moveQueue[0]){
    case 'R':
      moveRightLeg();
      removeFirstInQueue();
      break;
    case 'L':
      moveLeftLeg();
      removeFirstInQueue();
      break;
    case 'S':
      standUp();
      removeFirstInQueue();
      break;
    case 'N':
      break;
    default:
      break;    
  }
  
}

void moveRightLeg(){
  rightHip.write(90);
}

void moveLeftLeg(){
  leftHip.write(90);
}

void standUp(){
  leftHip.write(0);
  rightHip.write(0);
}