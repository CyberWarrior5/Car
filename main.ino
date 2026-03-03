#include <Wire.h>

int LED = 2;
int IN1 = 12;
int IN2 = 32;
int IN3 = 33;
int IN4 = 25;
int ENA = 27;
int ENB = 26;
int f = 20000;
int r = 8;
float gyroOffset = 0;
int current_ENA = 0;
int current_ENB = 0;
int trig = 18;
int echo = 19;
int servopin = 16;

void servo(int angle) {
  angle = constrain(angle, 0, 180);
  int dutyCycle = map(angle, 0, 180, 1638, 8192);
  ledcWrite(servopin, dutyCycle);
}

float getDistance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);
  if (duration == 0) {
    return 999.9;
  }

  float distance = (duration * 0.034) / 2;

  return distance;
}


float GetAngularVelocity() {

  Wire.beginTransmission(0x68);
  Wire.write(0x47);
  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 2);

  if (Wire.available() >= 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();

    int16_t rawZ = (highByte << 8) | lowByte;
    return rawZ / 131.0;
  }
}

float GetAverageAvel(int time) {
  unsigned long startTime = millis();

  float total = 0;
  int counter = 0;

  while (millis() - startTime < time) {
    float avel = GetAngularVelocity();
    total += avel;
    counter += 1;
    delay(1);
  }

  if (counter == 0) {
    Serial.println("Counter is 0");
    return 0;
  }

  float average_avel = total / counter;
  return average_avel;
  
}

void calibrate() {
  Serial.println("Calibrating, hold still...");
  gyroOffset = GetAverageAvel(500);
}



void StartDriving(int speed, bool reverse = false) { 

  ledcWrite(ENA, speed);
  ledcWrite(ENB, speed);
  current_ENA = speed;
  current_ENB = speed;

  if (!reverse) {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

void driveStraight(float distance_check, int distance_inverse, bool reverse = false) {
  StartDriving(200, false);
  int driver = ENA;
  int inverse = ENB;
  if (reverse) {
    driver = ENB;
    inverse = ENA;
  } 
  float total_avel = 0.0;
  float base_speed = 200;
  int Kp = 3;
  float distance = getDistance();
  
  while ((distance * distance_inverse) > (distance_check * distance_inverse)) {
    float avel = GetAngularVelocity() - gyroOffset;
    if (abs(avel) < 1) {
      avel = 0;
    }
    float error = Kp * avel;
    float change = constrain(base_speed - error, 0,255);
    float inverse_change = constrain(base_speed + error, 0 ,255);
    ledcWrite(driver, change);
    ledcWrite(inverse, inverse_change);
    Serial.print(change);
    Serial.print(" ");
    Serial.println(inverse_change);

    distance = getDistance();
  }
  //Brake and clean up
  //The idea for the car is to complete a track, so it will have to only drive straight for a set amount of time, i will edit the trigger for drive straight to change it from time to something else later tho, if that doesnt work out i will remove the breaking system
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, 255);
  ledcWrite(ENB, 255);

  delay(500);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);

}


void turn(int degrees) {
  int driver;
  int inverse;
  int INX;
  int INY;
  int INA;
  int INB;

  if (degrees > 0) {
    driver  = ENB;
    inverse = ENA;
    INX = IN3;
    INY = IN4;
    INA = IN1;
    INB = IN2;
  } else {
    driver  = ENA;
    inverse = ENB;
    INX = IN1;
    INY = IN2;
    INA = IN3;
    INB = IN4;
  }

  digitalWrite(INA, HIGH);
  digitalWrite(INB, HIGH);

  digitalWrite(INX, LOW);
  digitalWrite(INY, HIGH);

  ledcWrite(driver, 0);
  ledcWrite(inverse, 255);

  float total_adis = 0;
  float current_speed;
  float base_speed = 150;
  float Kp = 2;
  float averageAvel;

  unsigned long startTime = millis();
  unsigned long startTime2 = millis();

  while ((abs(degrees - total_adis ) > 0.3 || abs(averageAvel) > 0.1) && millis() - startTime < 2000) {
    
    averageAvel = GetAverageAvel(10) - gyroOffset;
    float adis = averageAvel * ((millis() - startTime2) / 1000.0);
    total_adis += adis;

    float error = abs(degrees - total_adis);
    current_speed = constrain((error * Kp) + base_speed, 0, 255);

    if (abs(total_adis) < abs(degrees)) {
      digitalWrite(INX, LOW);
      digitalWrite(INY, HIGH);
      ledcWrite(driver, current_speed);
    } else {
      digitalWrite(INX, HIGH);
      digitalWrite(INY, LOW);
      ledcWrite(driver, current_speed);
    }
    startTime2 = millis();

  }
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);

  ledcWrite(ENA, 255);
  ledcWrite(ENB, 255);

  delay(500);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}



void parralel_park() {
  servo(180);
  delay(200);
  driveStraight(15, 1, false);
  turn(-90);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(13, 14);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);

  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: MPU6050 not found! Check SDA/SCL wiring.");
  } else {
    Serial.println("MPU6050 Found. Starting Calibration...");
    calibrate();
    Serial.println("Calibration Successful.");
  }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, f, r);
  ledcAttach(ENB, f, r);
  ledcAttach(servopin, 50, 16);
  servo(90);
  delay(500);
  pinMode(LED, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  Wire.setClock(400000);

}


void loop() {
  driveStraight(15, 1, false);
  
}



