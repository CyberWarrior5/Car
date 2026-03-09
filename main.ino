#include <Wire.h>
#include <algorithm>

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

float GetAverageAvel(unsigned long time) {
  unsigned long startTime = micros();

  float total = 0;
  long counter = 0;

  while (micros() - startTime < time) { 
    total += GetAngularVelocity();
    counter += 1;
  }

  if (counter > 0) {
    return total / float(counter);
  } 
  return 0;
  
}

void calibrate() {
  gyroOffset = GetAverageAvel(1000000);
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

float driveStraight(float distance_check, float baseSpeed = 200, int distance_inverse = 1, bool reverse = false, int slowdownnum = 20) {
  float slowdown = distance_check + (slowdownnum * distance_inverse);
  int driver = ENA;
  int inverse = ENB;
  if (reverse) {
    driver = ENB;
    inverse = ENA;
  } 
  float total_avel = 0.0;
  float total_adis = 0.0;
  int Kp = 10;
  float Ki = 20;
  float integral = 0;
  float distance = getDistance();
  unsigned long startTime = micros();
  unsigned long currentTime = micros();
  unsigned long stopTimer = micros();
  float deadzone = 0.1;
  float Kd = 1;

  StartDriving(baseSpeed, reverse);
  float speed = baseSpeed - 100;
  bool passed = false;
  float increments = 0;
  while (true) {
    speed = constrain(speed + 3, 0, baseSpeed);
    float avel = GetAngularVelocity() - gyroOffset;
    if (abs(avel) < deadzone) {
      avel = 0;
    }
    
    currentTime = micros();
    float dt = ((currentTime - startTime) / 1000000.0);
    startTime = currentTime;
    

    float adis = avel * dt;
    total_adis += adis;
    integral += (Ki * total_adis * dt);
    integral = constrain(integral, -50.0, 50.0);
    float error = (Kp * total_adis) + integral + (Kd * avel);

    float change = constrain(speed - error, 0,255);
    float inverse_change = constrain(speed + error , 0 ,255);
    ledcWrite(driver, change);
    ledcWrite(inverse, inverse_change);

    
    // if (micros() - stopTimer > 3000000) {

    //   total_adis = 0;
    //   total_inc = 0;
    //   ledcWrite(ENA, 0);
    //   ledcWrite(ENB, 0);

    //   delay(500);

    //   calibrate();
    //   stopTimer = micros();
    //   startTime = micros();

    //   ledcWrite(driver, baseSpeed);
    //   ledcWrite(inverse, baseSpeed);


    // }
    float raw = 100;

    if (raw > 2.0 && raw < 400.0) {
      distance = raw;
    }

  
    if (!((distance * distance_inverse) > (slowdown * distance_inverse))) {
      if (!passed) {
        int pass = 0;
        for (int i = 0; i < 3; i++) {
          delay(2);
          float tdist = getDistance();
          if (!((tdist * distance_inverse) > (slowdown * distance_inverse))) {
            pass += 1;
          }
        }
        if (pass >= 3) {
          passed = true;
          baseSpeed = speed;
          increments = baseSpeed / 100;
          Serial.println("Slowdown started");
          digitalWrite(LED, HIGH);
        }
      }
    }
      
    if (passed) {
      baseSpeed = constrain(baseSpeed - increments, 180, 255);
    }
      
    if (!((distance * distance_inverse) > (distance_check * distance_inverse))) {
      int pass = 0;
      for (int i = 0; i < 3; i++) {
        delay(2);
        float tdist = getDistance();
        if (!((tdist * distance_inverse) > (distance_check * distance_inverse))) {
          pass += 1;
        }
      }
      if (pass >= 3) {
        Serial.println("Stopping...");
        digitalWrite(LED, LOW);
        break;
      }
    }
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
  return total_adis;
}

void StartTurning(int speed, int degrees) {
  if (degrees > 0) { // Right turn
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Left Forward
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); // Right Backward
  } else { // Left turn
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); // Left Backward
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // Right Forward
  }
  ledcWrite(ENA, speed);
  ledcWrite(ENB, speed);
}


void turn(int degrees, float baseSpeed = 200) {
  int driver = ENA;
  int inverse = ENB;
  int INX = IN1;
  int INY = IN2;
  int INA = IN3;
  int INB = IN4;



  float deadzone = 0.15;
  float total_adis = 0;
  float Kp = 2;
  float Ki = 30;
  float integral = 0;
  float decel = 0;


  StartTurning(baseSpeed, degrees);

  bool passed = false;

  unsigned long currentTime = micros();
  unsigned long startTime = micros();
  while (true) {
    float avel = GetAngularVelocity() - gyroOffset;

    if (abs(avel) < deadzone) {
      avel = 0;
    }

    currentTime = micros();
    float dt = ((currentTime - startTime) / 1000000.0);
    

    float adis = avel * dt;
    total_adis += adis;

    float current_error = degrees - total_adis; 
    integral += (Ki * abs(current_error) * dt);
    integral = constrain(integral, -45, 45);

    float change = (abs(current_error) * Kp) + integral;
    change = constrain(change, 160, baseSpeed);

    if (current_error > 0.5) {
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); 
    } else if (current_error < -0.5) {
      digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); 
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); 
    }
      ledcWrite(ENA, change);
      ledcWrite(ENB, change);


    if (abs(total_adis - degrees) < 1 && abs(avel) < 6) {
      break;
    } 
    startTime = currentTime;
  }
  digitalWrite(LED, LOW);
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

float getDistanceSafe() {
  float distance = 0;
  int pass = 0;
  while (true) {
    pass = 0;
    distance = getDistance();
    for (int i = 0; i < 10; i++) {
      delay(2);
      float tdist = getDistance();
      if (abs(distance - tdist) < 0.5) {
        pass += 1;
      }
    }
    if (pass == 10) {
      break;
    }
  }
  return distance;
}


void parralel_park(int side) {
  //Identify space
  side = constrain(side, -1, 1);
  servo(90 + (90 * side));
  delay(500);
  float currentDistance = getDistanceSafe();
  driveStraight(currentDistance + 3, 200, -1, false, 0);
  delay(1000);
  turn(180, 190);

}

void setup() {
  Serial.begin(115200);
  Wire.begin(13, 14);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  delay(2000);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: MPU6050 not found! Check SDA/SCL wiring.");
  } else {
    calibrate();
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
  // parralel_park(1);
  float total_adis = driveStraight(30, 250, 1, false, 20);
  delay(1000);
  float new_adis = driveStraight(10, 250, 1, true, 0);

  while (true) {
    Serial.println(total_adis);
    delay(1000);
  }
}


