#include <IRremote.h>
#include <Servo.h>
#include "remote_control.h"

#define DIR_1_L_PIN      2    //Motor direction
#define DIR_2_L_PIN      4    //Motor direction
#define SPEED_L_PIN      6    // Needs to be a PWM pin to be able to control motor speed
#define DIR_1_R_PIN      7    //Motor direction
#define DIR_2_R_PIN      8    //Motor direction
#define SPEED_R_PIN      5    // Needs to be a PWM pin to be able to control motor speed

#define IR_PORT          3

#define ECHO_PIN         12   // Ultrasonic Echo pin connect to D11
#define TRIG_PIN         13   // Ultrasonic Trig pin connect to D12

#define SERVO_PIN        9    //servo connect to D9

#define MIN_MOVING_SPEED  90
#define MAX_MOVING_SPEED  250

#define STOPPED                 0
#define MOVING_FORWARD          1
#define MOVING_BACKWARD         2
#define MOVING_FORWARD_LEFT     3
#define MOVING_FORWARD_RIGHT    4
#define MOVING_BACKWARD_LEFT    5
#define MOVING_BACKWARD_RIGHT   6
#define SPINNING_LEFT           7
#define SPINNING_RIGHT          8


Servo head;

IRrecv irrecv(IR_PORT);
decode_results irResults;
unsigned long lastSelectedKey = 0;

// States
int movingStatus = STOPPED;
int delayCounter = 0;
int vel = MIN_MOVING_SPEED;

void setMovingSpeed(int movingSpeed) {
  setMotorSpeed(movingSpeed, movingSpeed);
}

void setMotorSpeed(int speedL,int speedR) {
//    Serial.print("Motor speed: ");
//    Serial.print(abs(speedL));
//    Serial.print("  -   ");
//    Serial.println(abs(speedR));
    analogWrite(SPEED_L_PIN, abs(speedL)); 
    analogWrite(SPEED_R_PIN, abs(speedR));   
}

void moveForward(void) {
  setMovingSpeed(vel);
  digitalWrite(DIR_1_L_PIN, HIGH);
  digitalWrite(DIR_2_L_PIN, LOW);
  digitalWrite(DIR_1_R_PIN, HIGH);
  digitalWrite(DIR_2_R_PIN, LOW);
  movingStatus = MOVING_FORWARD;
  delayCounter = 0;
}

void moveBackward() {
    setMovingSpeed(vel);
    digitalWrite(DIR_1_L_PIN, LOW);
    digitalWrite(DIR_2_L_PIN, HIGH);
    digitalWrite(DIR_1_R_PIN, LOW);
    digitalWrite(DIR_2_R_PIN, HIGH);
    movingStatus = MOVING_BACKWARD;
    delayCounter = 0;
}

void moveStop() {
    digitalWrite(DIR_1_L_PIN, LOW);
    digitalWrite(DIR_2_L_PIN, LOW);
    digitalWrite(DIR_1_R_PIN, LOW);
    digitalWrite(DIR_2_R_PIN, LOW);
    setMovingSpeed(MIN_MOVING_SPEED);
    movingStatus = STOPPED;
    vel = 0;
    delayCounter = 0;
}

void spinLeft() {
    setMovingSpeed(MAX_MOVING_SPEED);
    digitalWrite(DIR_1_L_PIN, HIGH);
    digitalWrite(DIR_2_L_PIN, LOW);
    digitalWrite(DIR_1_R_PIN, LOW);
    digitalWrite(DIR_2_R_PIN, HIGH);
    movingStatus = SPINNING_LEFT;
    delayCounter = 0;
}

void spinRight() {
    setMovingSpeed(MAX_MOVING_SPEED);
    digitalWrite(DIR_1_L_PIN, LOW);
    digitalWrite(DIR_2_L_PIN, HIGH);
    digitalWrite(DIR_1_R_PIN, HIGH);
    digitalWrite(DIR_2_R_PIN, LOW);
    movingStatus = SPINNING_RIGHT;
    delayCounter = 0;
}

void turnLeftForward() {
    setMovingSpeed(MAX_MOVING_SPEED);
    digitalWrite(DIR_1_L_PIN, HIGH);
    digitalWrite(DIR_2_L_PIN, LOW);
    digitalWrite(DIR_1_R_PIN, LOW);
    digitalWrite(DIR_2_R_PIN, LOW);
    movingStatus = MOVING_FORWARD_LEFT;
    delayCounter = 0;
}

void turnRightForward() {
    setMovingSpeed(MAX_MOVING_SPEED);
    digitalWrite(DIR_1_L_PIN, LOW);
    digitalWrite(DIR_2_L_PIN, LOW);
    digitalWrite(DIR_1_R_PIN, HIGH);
    digitalWrite(DIR_2_R_PIN, LOW);
    movingStatus = MOVING_FORWARD_RIGHT;
    delayCounter = 0;
}

void turnLeftBackward() {
    setMovingSpeed(MAX_MOVING_SPEED);
    digitalWrite(DIR_1_L_PIN, LOW);
    digitalWrite(DIR_2_L_PIN, HIGH);
    digitalWrite(DIR_1_R_PIN, LOW);
    digitalWrite(DIR_2_R_PIN, LOW);
    movingStatus = MOVING_BACKWARD_LEFT;
    delayCounter = 0;
}

void turnRightBackward() {
    setMovingSpeed(MAX_MOVING_SPEED);
    digitalWrite(DIR_1_L_PIN, LOW);
    digitalWrite(DIR_2_L_PIN, LOW);
    digitalWrite(DIR_1_R_PIN, LOW);
    digitalWrite(DIR_2_R_PIN, HIGH);
    movingStatus = MOVING_BACKWARD_RIGHT;
    delayCounter = 0;
}

byte readIrKey(){
  if (irrecv.decode(&irResults)){
      if (irResults.value == 0XFFFFFFFF || irResults.value == 0XFF) {
          irrecv.resume();
          if (*irResults.rawbuf < 1000){
            return 0;
          } else {
            return lastSelectedKey;
          }
      }
      switch(irResults.value){
        case IR_KEY_UP_1:
        case IR_KEY_UP_2:
          lastSelectedKey = KEY_UP;
          break;
        case IR_KEY_DOWN_1:
        case IR_KEY_DOWN_2:
          lastSelectedKey = KEY_DOWN;
          break;
        case IR_KEY_RIGHT_1:
        case IR_KEY_RIGHT_2:
          lastSelectedKey = KEY_RIGHT;
          break;
        case IR_KEY_LEFT_1:
        case IR_KEY_LEFT_2:
          lastSelectedKey = KEY_LEFT;
          break;
        case IR_KEY_STOP_1:
        case IR_KEY_STOP_2:
          lastSelectedKey = KEY_STOP;
          break;
      }
      irrecv.resume();
      return lastSelectedKey;
  }
  return 0;
}

void accelerate(int increment){
  vel += increment;
//  Serial.print("Vel: ");
//  Serial.print(vel);
//  Serial.print(" Status: ");
//  Serial.print(movingStatus);
//  Serial.print(" increment: ");
//  Serial.println(increment);
  if (-MIN_MOVING_SPEED < vel && vel < MIN_MOVING_SPEED) {
    if (movingStatus == MOVING_FORWARD || movingStatus == MOVING_BACKWARD){
      moveStop();
      return;
    }
    vel = (increment > 0)? MIN_MOVING_SPEED : -MIN_MOVING_SPEED;
  }
  if (vel > MAX_MOVING_SPEED) vel = MAX_MOVING_SPEED;
  if (vel < -MAX_MOVING_SPEED) vel = -MAX_MOVING_SPEED;
  if (vel > 0){
    moveForward();
  } else {
    moveBackward();
  }
}

void setup() {
  Serial.begin(9600);

  // Init IR
  irrecv.enableIRIn();
  irrecv.blink13(true);

  // Init Servo
//  head.attach(SERVO_PIN); 
//  head.write(90);
  
  // Init Motors
  setMovingSpeed(MIN_MOVING_SPEED);
  pinMode(DIR_1_L_PIN, OUTPUT); 
  pinMode(DIR_2_L_PIN, OUTPUT); 
  pinMode(SPEED_L_PIN, OUTPUT);  
  pinMode(DIR_1_R_PIN, OUTPUT);
  pinMode(DIR_2_R_PIN, OUTPUT); 
  pinMode(SPEED_R_PIN, OUTPUT);

  moveStop();
}

void loop() {
//  Serial.print("Status: ");
//  Serial.print(movingStatus);
//  Serial.print(" delayCounter: ");
//  Serial.println(delayCounter);
  int irKey = readIrKey();
  switch(irKey){
    case KEY_UP:
      accelerate(+20);
      break;
    case KEY_DOWN:
      accelerate(-20);
      break;
    case KEY_LEFT:
      if (movingStatus == STOPPED || movingStatus == SPINNING_LEFT){
        spinLeft();
      } else {
        if (movingStatus == MOVING_BACKWARD || movingStatus == MOVING_BACKWARD_LEFT){
          turnLeftBackward();
        } else {
          turnLeftForward();
        }
      }
      break;
    case KEY_RIGHT:
      if (movingStatus == STOPPED || movingStatus == SPINNING_RIGHT){
        spinRight();
      } else {
        if (movingStatus == MOVING_BACKWARD || movingStatus == MOVING_BACKWARD_RIGHT){
          turnRightBackward();
        } else {
          turnRightForward();
        }
      }
      break;
    case KEY_STOP:
      moveStop();
      break;
  }
  switch(movingStatus){
    case STOPPED:
      break;
    case MOVING_FORWARD:
      break;
    case MOVING_FORWARD_LEFT:
      if (++delayCounter > 8) moveForward();
      break;
    case MOVING_FORWARD_RIGHT:
      if (++delayCounter > 8) moveForward();
      break;
    case MOVING_BACKWARD_LEFT:
      if (++delayCounter > 8) moveBackward();
      break;
    case MOVING_BACKWARD_RIGHT:
      if (++delayCounter > 8) moveBackward();
      break;
    case SPINNING_LEFT:
      if (++delayCounter > 12) moveStop();
      break;
    case SPINNING_RIGHT:
      if (++delayCounter > 12) moveStop();
      break;
  }
  delay(20);
}
