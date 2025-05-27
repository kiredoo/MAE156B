int PWM1 = 5;
int INA1 = 4;
int INB1 = 2;
bool running = true;
bool dir = 0;
long prevSteps = 0;
long pos = 0;
long prevPos = 0;
long steps = 0;
bool homeFlag = 0;
float pulseperin = 441.96;
unsigned long prevTimer = 0;
unsigned long lastStepTime = 0;
int trigDelay = 3000; // in microseconds



void setup() {
  digitalWrite(PWM1, LOW);
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, LOW);

  pinMode(PWM1, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(3), countPulse, RISING);
  delay(200);  // Let driver settle

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (running) {
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);
    Serial.println("initialize");
    delay(2000);
    analogWrite(PWM1, 255);
    delay(15000);
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, LOW);
    delay(1000);
    // digitalWrite(INA1, HIGH);
    // digitalWrite(INB1, LOW);
    // analogWrite(PWM1, 255);
    // delay(5000);
    // digitalWrite(INA1, LOW);
    // digitalWrite(INB1, LOW);
    // analogWrite(PWM1, 0);
    // // if (homeFlag == 0){
    // //   homeActuator();
    // //   }
    // // extend();
    // // digitalWrite(INA1, LOW);
    // // digitalWrite(INB1, LOW);
    // // delay(1000);
    // // retract();
    running = false;
    Serial.println("done");
  }
}

void extend() {
  bool extended = 0;
  Serial.println("extending");
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, HIGH);
  analogWrite(PWM1, 255);
  while (extended == 0){
    if(millis() - prevTimer > 100){
      updatePosition();
      prevTimer = millis();
      if(pos == prevPos || pos == 883){
        pos = 883;
        extended = 0;
        }
      else {
        prevPos = pos;
      }
      Serial.println(convertToInches(pos));
    }
  }
  Serial.println("finished extending");
}

void retract() {
  bool retracted = 0;
  Serial.println("retracting");
  digitalWrite(INA1, HIGH);
  digitalWrite(INB1, LOW);
  analogWrite(PWM1, 255);
  while (retracted == 0){
    if(millis() - prevTimer > 100){
      updatePosition();
      prevTimer = millis();
      if(pos == prevPos || pos == 0){
        pos = 0;
        retracted = 0;
        }
      else {
        prevPos = pos;
      }
      Serial.println(convertToInches(pos));
    }
  }
  Serial.println("finished retracting");
}

void homing(void){
  Serial.println("homing");
  digitalWrite(INA1, HIGH);
  digitalWrite(INB1, LOW);
  analogWrite(PWM1, 255);
  delay(5000);
  Serial.println("finished homing");
}

void countPulse(void) {
  if((micros()-lastStepTime > trigDelay)){
    steps++;
    lastStepTime = micros();
    Serial.println(steps);
  }
}

void homeActuator(void) {
  prevTimer = millis();
  while(homeFlag == 0){
    digitalWrite(INA1, HIGH);
    digitalWrite(INB1, LOW);
    analogWrite(PWM1, 255);
    if (prevSteps == steps){
      if(millis()-prevTimer > 100){
        digitalWrite(INA1, LOW);
        digitalWrite(INB1, LOW);
        analogWrite(PWM1, 0);
        steps = 0;
        homeFlag = 1;
      }
    }
    else {
      prevSteps = steps;
      prevTimer = millis();
    }
  }
}

void updatePosition(void){
  if(dir == 1){
    pos = pos + steps;
    steps = 0;
  }
  else{
    pos = pos - steps;
    steps = 0;
  }
}

float convertToInches(long pos) {
  return pos/pulseperin;
}