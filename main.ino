#include <Wire.h>

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


void setup() {
  Serial.begin(115200);
  Wire.begin(13, 14);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  calibrate();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, f, r);
  ledcAttach(ENB, f, r);


}

void StartDriving(int speed) { 

  ledcWrite(ENA, speed);
  ledcWrite(ENB, speed);
  current_ENA = speed;
  current_ENB = speed;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

}

void DriveStraight(int time) {
  StartDriving(200);

  unsigned long startTime = millis();
  float total_adis = 0;
  int Kp = 5;
  while (millis() - startTime < time) {
    float averageAvel = GetAverageAvel(50) - gyroOffset;
    float adis = averageAvel * 0.05;
    total_adis += adis;

    if (abs(total_adis) > 3) {
      if (total_adis < 0) {
        if (!(current_ENA == 200)) {
          current_ENA = 200;
          ledcWrite(ENA, current_ENA);
        }
        
        float change = Kp * abs(total_adis);

        current_ENB = constrain(200 + change, 0, 255)
        ledcWrite(ENB, current_ENB);
        
      } else {

        if (!(current_ENB == 200)) {
          current_ENB = 200;
          ledcWrite(ENB, current_ENB);
        }
        float change = Kp * abs(total_adis);
        if (200 + change <= 255) {
          current_ENA = constrain(200 + change, 0, 255)
          ledcWrite(ENA, current_ENA);
        }
      }
    } else {
      ledcWrite(ENA, 200);
      ledcWrite(ENB, 200);
      current_ENA = 200;
      current_ENB = 200;
    }

  }
}

void loop() {
  delay(3000)
  DriveStraight(3000);
}




