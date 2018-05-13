const int TRIG_PIN = 9;
const int ECHO_PIN = 10;
const int SOUND_PIN = 3;

const float SOUND_SPEED = 0.034 / 2;
const int MAX_DELAY_COUNTER = 20;

long duration;
int distance;
int delayCounter = MAX_DELAY_COUNTER;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SOUND_PIN, OUTPUT);

  Serial.begin(9600);
}

int counter = 0;
void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);

  distance = duration * SOUND_SPEED;

  delayCounter = MAX_DELAY_COUNTER * (distance / 60.0);

  if (counter >= delayCounter){
      beep(50);
      counter = 0;
  } else {
    delay(50);
    counter++;
  }


  Serial.print("Distance: ");
  Serial.println(distance);
}

void beep(unsigned char delayms){
  analogWrite(SOUND_PIN, 40);
  delay(delayms);
  analogWrite(SOUND_PIN, 0);
  delay(delayms);
}  
