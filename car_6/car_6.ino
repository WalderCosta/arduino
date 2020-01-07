#include <Servo.h>

#define SERVO_PIN 9 

Servo head;
byte headAngle = 90;

void setup() {
    head.attach(SERVO_PIN); 
    head.write(headAngle);

}

void loop() {
  delay(2000);

  headAngle = (headAngle + 20) % 180;

  head.write(headAngle);

  
}
