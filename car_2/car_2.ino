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
#define MOVING_SPEED_DIFF (MAX_MOVING_SPEED - MIN_MOVING_SPEED)

#define MOVING_STOPPED          0
#define MOVING_FORWARD          1
#define MOVING_BACKWARD         2
#define MOVING_FORWARD_LEFT     3
#define MOVING_FORWARD_RIGHT    4
#define MOVING_BACKWARD_LEFT    5
#define MOVING_BACKWARD_RIGHT   6
#define SPINNING_LEFT           7
#define SPINNING_RIGHT          8


#define HEAD_STOPPED            0
#define HEAD_MIDDLE             1
#define HEAD_LEFT_CCW           2
#define HEAD_LEFT_CW            3
#define HEAD_RIGHT_CCW          4
#define HEAD_RIGHT_CW           5


#define MAX_DISTANCE 100
#define MIN_DISTANCE 4

#define MAX_MOVING_DISTANCE 30
#define MIN_MOVING_DISTANCE 10
#define MOVING_DISTANCE_DIFF (MAX_MOVING_DISTANCE - MIN_MOVING_DISTANCE)

#define MIN_HEAD_ANGLE 0
#define MAX_HEAD_ANGLE 180
#define HEAD_SPEED 10
#define HEAD_STEPS ((MAX_HEAD_ANGLE - MIN_HEAD_ANGLE) / HEAD_SPEED)

Servo head;

IRrecv irrecv(IR_PORT);
decode_results irResults;
unsigned long lastSelectedKey = 0;

// States
byte movingStatus = MOVING_STOPPED;
byte delayMovingCounter = 0;
int vel = MIN_MOVING_SPEED;

// Distance states
byte headStatus = HEAD_STOPPED;
byte headAngle = 90;
byte delayHeadCounter = 0;
byte distances[HEAD_STEPS + 1];

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
  delayMovingCounter = 0;
}

void moveBackward() {
    setMovingSpeed(vel);
    digitalWrite(DIR_1_L_PIN, LOW);
    digitalWrite(DIR_2_L_PIN, HIGH);
    digitalWrite(DIR_1_R_PIN, LOW);
    digitalWrite(DIR_2_R_PIN, HIGH);
    movingStatus = MOVING_BACKWARD;
    delayMovingCounter = 0;
}

void moveStop() {
    digitalWrite(DIR_1_L_PIN, LOW);
    digitalWrite(DIR_2_L_PIN, LOW);
    digitalWrite(DIR_1_R_PIN, LOW);
    digitalWrite(DIR_2_R_PIN, LOW);
    setMovingSpeed(MIN_MOVING_SPEED);
    movingStatus = MOVING_STOPPED;
    vel = 0;
    delayMovingCounter = 0;
}

void spinLeft() {
    setMovingSpeed(MAX_MOVING_SPEED);
    digitalWrite(DIR_1_L_PIN, HIGH);
    digitalWrite(DIR_2_L_PIN, LOW);
    digitalWrite(DIR_1_R_PIN, LOW);
    digitalWrite(DIR_2_R_PIN, HIGH);
    movingStatus = SPINNING_LEFT;
    delayMovingCounter = 0;
}

void spinRight() {
    setMovingSpeed(MAX_MOVING_SPEED);
    digitalWrite(DIR_1_L_PIN, LOW);
    digitalWrite(DIR_2_L_PIN, HIGH);
    digitalWrite(DIR_1_R_PIN, HIGH);
    digitalWrite(DIR_2_R_PIN, LOW);
    movingStatus = SPINNING_RIGHT;
    delayMovingCounter = 0;
}

void turnLeftForward() {
    setMovingSpeed(MAX_MOVING_SPEED);
    digitalWrite(DIR_1_L_PIN, HIGH);
    digitalWrite(DIR_2_L_PIN, LOW);
    digitalWrite(DIR_1_R_PIN, LOW);
    digitalWrite(DIR_2_R_PIN, LOW);
    movingStatus = MOVING_FORWARD_LEFT;
    delayMovingCounter = 0;
}

void turnRightForward() {
    setMovingSpeed(MAX_MOVING_SPEED);
    digitalWrite(DIR_1_L_PIN, LOW);
    digitalWrite(DIR_2_L_PIN, LOW);
    digitalWrite(DIR_1_R_PIN, HIGH);
    digitalWrite(DIR_2_R_PIN, LOW);
    movingStatus = MOVING_FORWARD_RIGHT;
    delayMovingCounter = 0;
}

void turnLeftBackward() {
    setMovingSpeed(MAX_MOVING_SPEED);
    digitalWrite(DIR_1_L_PIN, LOW);
    digitalWrite(DIR_2_L_PIN, HIGH);
    digitalWrite(DIR_1_R_PIN, LOW);
    digitalWrite(DIR_2_R_PIN, LOW);
    movingStatus = MOVING_BACKWARD_LEFT;
    delayMovingCounter = 0;
}

void turnRightBackward() {
    setMovingSpeed(MAX_MOVING_SPEED);
    digitalWrite(DIR_1_L_PIN, LOW);
    digitalWrite(DIR_2_L_PIN, LOW);
    digitalWrite(DIR_1_R_PIN, LOW);
    digitalWrite(DIR_2_R_PIN, HIGH);
    movingStatus = MOVING_BACKWARD_RIGHT;
    delayMovingCounter = 0;
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

byte detectDistance(){
    long duration;
    byte distance;
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(15);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH, 5000);
    if (duration <= 5){
      return MAX_DISTANCE;
    }
    distance = round(duration * 0.01657);
    if (distance > MAX_DISTANCE) return MAX_DISTANCE;
    if (distance < MIN_DISTANCE) return MIN_DISTANCE;
    return distance;
}

void setup() {
  Serial.begin(9600);

  // Init IR
  irrecv.enableIRIn();
  irrecv.blink13(true);

  // Init Servo
  head.attach(SERVO_PIN); 
  head.write(headAngle);
  
  // Init Motors
  setMovingSpeed(MIN_MOVING_SPEED);
  pinMode(DIR_1_L_PIN, OUTPUT); 
  pinMode(DIR_2_L_PIN, OUTPUT); 
  pinMode(SPEED_L_PIN, OUTPUT);  
  pinMode(DIR_1_R_PIN, OUTPUT);
  pinMode(DIR_2_R_PIN, OUTPUT); 
  pinMode(SPEED_R_PIN, OUTPUT);

  moveStop();

  for (byte i = 0; i <= HEAD_STEPS; i++) distances[i] = MAX_DISTANCE;

}

void loop() {

  // IR Key
  int irKey = readIrKey();
  switch(irKey){
    case KEY_UP:
      accelerate(+20);
      break;
    case KEY_DOWN:
      accelerate(-20);
      break;
    case KEY_LEFT:
      if (movingStatus == MOVING_STOPPED || movingStatus == SPINNING_LEFT){
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
      if (movingStatus == MOVING_STOPPED || movingStatus == SPINNING_RIGHT){
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
//  Serial.print(" HeadStatus: ");
//  Serial.println(headStatus);

  // Head/Distance
  if (movingStatus == MOVING_FORWARD || movingStatus == MOVING_FORWARD_RIGHT || movingStatus == MOVING_FORWARD_LEFT) {
    switch(headStatus){
      case HEAD_STOPPED:
        headStatus = HEAD_LEFT_CCW;
        break;
      case HEAD_LEFT_CCW:
        headAngle -= HEAD_SPEED;
        if (headAngle <= MIN_HEAD_ANGLE) {
          headAngle = MIN_HEAD_ANGLE;
          headStatus = HEAD_LEFT_CW;
        }
        break;
      case HEAD_LEFT_CW:
        if (headAngle == 90) {
          delayHeadCounter++;
          if (delayHeadCounter > 10){
            delayHeadCounter = 0;
            headStatus = HEAD_RIGHT_CW;
          }
        } else {
          headAngle += HEAD_SPEED;
        }
        break;
      case HEAD_RIGHT_CW:
        headAngle += HEAD_SPEED;
        if (headAngle >= MAX_HEAD_ANGLE) {
          headStatus = HEAD_RIGHT_CCW;
        }
        break;
      case HEAD_RIGHT_CCW:
        if (headAngle == 90) {
          delayHeadCounter++;
          if (delayHeadCounter > 10){
            delayHeadCounter = 0;
            headStatus = HEAD_LEFT_CCW;
          }
        } else {
          headAngle -= HEAD_SPEED;
        }
        break;
    }
    head.write(headAngle);
    byte distance = detectDistance();
    distances[headAngle / HEAD_SPEED] = distance;
    byte hitDistance = 0;
    if (headAngle >= 70 && headAngle <= 110){
      float proportionalVel = (float) (vel - MIN_MOVING_SPEED) / (float) MOVING_SPEED_DIFF;
      hitDistance = MIN_MOVING_DISTANCE + (MOVING_DISTANCE_DIFF * proportionalVel);
    } else {
      // Discard smaller value
      hitDistance = MIN_MOVING_DISTANCE;
    }
    
//    Serial.print("Vel: ");
//    Serial.print(vel);
//    Serial.print(" hitDistance: ");
//    Serial.print(hitDistance);
//    Serial.print(" distance: ");
//    Serial.println(distance);
//   
    if (distance < hitDistance){
      moveStop();
    }
//    Serial.print("HeadAngle: ");
//    Serial.println(headAngle);
//    Serial.print("Distances: ");
//    for (byte i = 0; i <= HEAD_STEPS; i++) {
//      Serial.print(i * HEAD_SPEED);
//      Serial.print(": ");
//      Serial.print(distances[i]);
//      Serial.print(", ");
//      distances[i];
//    }
//    Serial.println("");
  } else {
    headStatus = HEAD_STOPPED;
    if (headAngle != 90){
      headAngle = 90;
      head.write(headAngle);
    }
  }
  
  // Status
  switch(movingStatus){
    case MOVING_FORWARD_LEFT:
      if (++delayMovingCounter > 8) moveForward();
      break;
    case MOVING_FORWARD_RIGHT:
      if (++delayMovingCounter > 8) moveForward();
      break;
    case MOVING_BACKWARD_LEFT:
      if (++delayMovingCounter > 8) moveBackward();
      break;
    case MOVING_BACKWARD_RIGHT:
      if (++delayMovingCounter > 8) moveBackward();
      break;
    case SPINNING_LEFT:
      if (++delayMovingCounter > 8) moveStop();
      break;
    case SPINNING_RIGHT:
      if (++delayMovingCounter > 8) moveStop();
      break;
  }
  delay(20);
}
