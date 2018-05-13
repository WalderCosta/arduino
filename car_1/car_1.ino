#include <IRremote.h>
#include <Servo.h>

#define dir1PinL  2    //Motor direction
#define dir2PinL  4    //Motor direction
#define speedPinL 6    // Needs to be a PWM pin to be able to control motor speed

#define dir1PinR  7    //Motor direction
#define dir2PinR  8   //Motor direction
#define speedPinR 5    // Needs to be a PWM pin to be able to control motor speed

#define IR_PORT       3


#define Echo_PIN      12 // Ultrasonic Echo pin connect to D11
#define Trig_PIN      13  // Ultrasonic Trig pin connect to D12

#define SERVO_PIN     9  //servo connect to D9

#define IR_ADVANCE       0x00FF18E7 
#define IR_ADVANCE2      0x3D9AE3F7
#define IR_BACK          0x00FF4AB5
#define IR_BACK2         0x1BC0157B
#define IR_RIGHT         0x00FF5AA5
#define IR_RIGHT2        0x0449E79F
#define IR_LEFT          0x00FF10EF
#define IR_LEFT2         0x8C22657B    
#define IR_STOP          0x00FF38C7
#define IR_STOP2         0x488F3CBB

#define MIN_SPEED  90
#define MAX_SPEED  250

#define MAX_DISTANCE 100
#define MIN_DISTANCE 4

#define HEAD_SPEED 10
#define MIN_HEAD_ANGLE 0
#define MAX_HEAD_ANGLE 180

//#define HEAD_STEPS (180 / HEAD_SPEED)
//#define HEAD_STEPS_LEFT ((90 / HEAD_SPEED) - 1)
//#define HEAD_STEPS_RIGHT ((90 / HEAD_SPEED) + 1)

//byte distances[HEAD_STEPS + 1];

IRrecv irrecv(IR_PORT);
decode_results results;
unsigned long lastValue = 0;
Servo head;
int vel = 0;

byte headAngle = 90;
int headDirection = -1;
byte headCount = 0;


void moveForward(void) {
  setMotorSpeed(vel, vel);
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL, LOW);
  digitalWrite(dir1PinR, HIGH);
  digitalWrite(dir2PinR, LOW);
}

void spinLeft() {
    setMotorSpeed(MAX_SPEED, MAX_SPEED);
    digitalWrite(dir1PinL, HIGH);
    digitalWrite(dir2PinL, LOW);
    digitalWrite(dir1PinR, LOW);
    digitalWrite(dir2PinR, HIGH);
    delay(250);
    moveStop();
}

void spinRight() {
    setMotorSpeed(MAX_SPEED, MAX_SPEED);
    digitalWrite(dir1PinL, LOW);
    digitalWrite(dir2PinL, HIGH);
    digitalWrite(dir1PinR, HIGH);
    digitalWrite(dir2PinR, LOW);
    delay(250);
    moveStop();
}


void turnLeftForward() {
    setMotorSpeed(MAX_SPEED, MAX_SPEED);
    digitalWrite(dir1PinL, HIGH);
    digitalWrite(dir2PinL, LOW);
    digitalWrite(dir1PinR, LOW);
    digitalWrite(dir2PinR, LOW);
    delay(150);
    setMotorSpeed(vel, vel);
    moveForward();
}

void turnRightForward() {
    setMotorSpeed(MAX_SPEED, MAX_SPEED);
    digitalWrite(dir1PinL, LOW);
    digitalWrite(dir2PinL, LOW);
    digitalWrite(dir1PinR, HIGH);
    digitalWrite(dir2PinR, LOW);
    delay(150);
    setMotorSpeed(vel, vel);
    moveForward();
}

void turnLeftBackward() {
    setMotorSpeed(MAX_SPEED, MAX_SPEED);
    digitalWrite(dir1PinL, LOW);
    digitalWrite(dir2PinL, HIGH);
    digitalWrite(dir1PinR, LOW);
    digitalWrite(dir2PinR, LOW);
    delay(150);
    setMotorSpeed(vel, vel);
    moveBackward();
}


void turnRightBackward() {
    setMotorSpeed(MAX_SPEED, MAX_SPEED);
    digitalWrite(dir1PinL, LOW);
    digitalWrite(dir2PinL, LOW);
    digitalWrite(dir1PinR, LOW);
    digitalWrite(dir2PinR, HIGH);
    delay(150);
    setMotorSpeed(vel, vel);
    moveBackward();
}

void moveBackward() {
    setMotorSpeed(vel, vel);
    digitalWrite(dir1PinL, LOW);
    digitalWrite(dir2PinL, HIGH);
    digitalWrite(dir1PinR, LOW);
    digitalWrite(dir2PinR, HIGH);
}

void moveStop() {
    digitalWrite(dir1PinL, LOW);
    digitalWrite(dir2PinL, LOW);
    digitalWrite(dir1PinR, LOW);
    digitalWrite(dir2PinR, LOW);
    setMotorSpeed(MIN_SPEED, MIN_SPEED);
    vel = 0;
}

void setMotorSpeed(int speed_L,int speed_R) {
//    Serial.print("Speed: ");
//    Serial.println(speed_L);
    analogWrite(abs(speedPinL), abs(speed_L)); 
    analogWrite(abs(speedPinR), abs(speed_R));   
}

byte detectDistance(){
    long duration;
    byte distance;
    digitalWrite(Trig_PIN, LOW);
    delayMicroseconds(5);                                                                              
    digitalWrite(Trig_PIN, HIGH);
    delayMicroseconds(15);
    digitalWrite(Trig_PIN, LOW);
    duration = pulseIn(Echo_PIN, HIGH, 5000);
    if (duration <= 5){
      return MAX_DISTANCE;
    }
//    Serial.print("duration: ");
//    Serial.println(duration);
    distance = round(duration * 0.01657);
    if (distance > MAX_DISTANCE) return MAX_DISTANCE;
    if (distance < MIN_DISTANCE) return MIN_DISTANCE;
    return distance;
}

byte frontDistance = MAX_DISTANCE;
byte leftDistance = MAX_DISTANCE; 
byte rightDistance = MAX_DISTANCE;
bool isScanning = false;

void fullScan(){
  head.write(MIN_HEAD_ANGLE);
  delay(200);
  leftDistance = detectDistance();
  head.write(MAX_HEAD_ANGLE);
  delay(200);
  rightDistance = detectDistance();
  head.write(90);
  delay(200);
  frontDistance = detectDistance();
}


void setup() {
    Serial.begin(9600);

    // Init IR
    irrecv.enableIRIn();
    irrecv.blink13(true);
    
    // init Distance Sensor (HC-SR04)
    pinMode(Trig_PIN, OUTPUT); 
    pinMode(Echo_PIN,INPUT);

    /*init Servo*/
    head.attach(SERVO_PIN); 
    head.write(90);

    // Init Motors
    setMotorSpeed(MIN_SPEED, MIN_SPEED);
    pinMode(dir1PinL, OUTPUT); 
    pinMode(dir2PinL, OUTPUT); 
    pinMode(speedPinL, OUTPUT);  
    pinMode(dir1PinR, OUTPUT);
    pinMode(dir2PinR, OUTPUT); 
    pinMode(speedPinR, OUTPUT); 

//    initializeDistances();
    moveStop();
}


void loop() {

//  if (!isScanning){
//    headCount++;
//    if (headAngle != 90){
//      headAngle = 90;
//      head.write(headAngle);
//    }
//    if (headCount > 10 && vel != 0){
//      isScanning = true;
//      if (headDirection > 0) {
//        leftDistance = MAX_DISTANCE;
//      } else {
//        rightDistance = MAX_DISTANCE;
//      }
//    }
//  } else {
//    if (headAngle >= MAX_HEAD_ANGLE || headAngle <= MIN_HEAD_ANGLE){
//      headDirection *= -1;
//    }
//    headAngle += (HEAD_SPEED * headDirection);
//    head.write(headAngle);
//
//    if (headAngle == 90){
//      isScanning = false;
//      headCount = 0;
//    }    
//  }

//  Serial.print("Angle: ");
//  Serial.println(headAngle);
  delay(15);
//  byte distance = detectDistance();
//  if (isScanning){
//    if (headAngle < 80){
//      if (distance < leftDistance){
//        leftDistance = distance;
//      }
//    } else if (headAngle > 100) {
//      if (distance < rightDistance){
//        rightDistance = distance;
//      }
//    } else {
//      frontDistance = distance;
//    }
//  }
  
  if (irrecv.decode(&results)){
//      Serial.print("IR key: ");
//      Serial.println(results.value, HEX);
      if ((results.value == 0XFFFFFFFF || results.value == 0XFF) && *results.rawbuf < 10000) {
          results.value = lastValue;
      }
      switch(results.value){
        case IR_ADVANCE:
        case IR_ADVANCE2:
          vel += 20;
          if (-MIN_SPEED < vel && vel < MIN_SPEED) {
            vel =  MIN_SPEED; 
          }
          if (vel > MAX_SPEED) vel = MAX_SPEED;
          moveForward();
          break;
        case IR_BACK:
        case IR_BACK2:
          vel -= 20;
          if (-MIN_SPEED < vel && vel < MIN_SPEED) {
            vel =  -MIN_SPEED; 
          }
          if (vel < -MAX_SPEED) vel = -MAX_SPEED;
          moveBackward();
          break;
        case IR_RIGHT:
        case IR_RIGHT2:
          if (vel == 0){
            spinRight();
          } else if (vel > 0){
            turnRightForward();
          } else {
            turnRightBackward();
          }
          break;
        case IR_LEFT:
        case IR_LEFT2:
          if (vel == 0){
            spinLeft();
          } else if (vel > 0){
            turnLeftForward();
          } else {
            turnLeftBackward();
          }
          break;
        case IR_STOP:
        case IR_STOP2:
          moveStop();
          break;
      }      
      lastValue = results.value;
      irrecv.resume();
  }

//  Serial.print("leftDistance: ");
//  Serial.print(leftDistance);
//  Serial.print(" frontDistance: ");
//  Serial.print(distance);
//  Serial.print(" leftDistance: ");
//  Serial.println(rightDistance);
  
//  if (vel > 0){
//   if (leftDistance < 15){
//      turnLeftForward();
//      leftDistance = MAX_DISTANCE;
//      head.write(90);
//      delay(50);
//    } else if (rightDistance < 10){
//      turnRightForward();
//      rightDistance = MAX_DISTANCE;
//      head.write(90);
//      delay(50);
//    } if (frontDistance < 10){
//      byte oldVel = vel;
//      vel = MIN_SPEED;
//      moveBackward();
//      delay(200);
//      moveStop();
//      fullScan();
//      bool shouldSpinLeft = leftDistance > rightDistance && leftDistance > 5;
//      byte spinCounter = 0;
//      while(frontDistance < 10 && spinCounter++ < 5){
//        if(shouldSpinLeft){
//          spinLeft();
//        } else {
//          spinRight();
//        }
//       frontDistance = detectDistance();
//      }
//      if (spinCounter >= 5){
//        moveStop();
//      } else {
//        vel = oldVel;
//        moveForward();
//      }
//    }
//  }
}
