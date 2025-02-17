/*
   TUBES FIRMWARE - STM32F411CEU6
   Kontrol jarak menggunakan PID dan ESC
*/

#include "modules/PID.h"
#include <Servo.h>

// Bagian Sensor Ultrasonik (HC-SR04)
const int trigPin = PA0;  // I/O
const int echoPin = PA1;  // I/O

// Bagian Kontrol Motor ESC
Servo esc;
const int escPin = PA8; // generate sinyal PWM

// Target jarak yang diinginkan
float setpoint = 175;

// Penggunaan PID pada program utama
PID pid(1.0, 0.05, 0.1);

void setup() {
  Serial.begin(9600);
  Serial.println("Sistem Kendali Jarak dengan PID");

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // penggunaan ESC
  esc.attach(escPin);
  esc.writeMicroseconds(1500);
  delay(5000);
}

void loop() {
  long duration;
  long distance_cm;
  
  // Baca jarak menggunakan sensor ultrasonik
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance_cm = microsecondsToCentimeters(duration);
  
  // Hitung output PID
  float output = pid.compute(setpoint, distance_cm);
  
  // Konversi output PID ke sinyal ESC (1000-2000μs)
  int escSignal;
  if (output > 0) {
    // Mundur: 1000-1500μs (jarak < setpoint)
    escSignal = map(output, 0, 1000, 1500, 1000); 
  } else {
    // Maju: 1500-2000μs (jarak > setpoint)
    escSignal = map(abs(output), 0, 1000, 1500, 2000);
  }
  
  escSignal = constrain(escSignal, 1000, 2000);
  esc.writeMicroseconds(escSignal);

  // Tuning PID via Serial Monitor
  if (Serial.available() >= 12) {
    float Kp = Serial.parseFloat();
    float Ki = Serial.parseFloat();
    float Kd = Serial.parseFloat();
    pid.setGains(Kp, Ki, Kd);
    Serial.print("PID Updated: Kp=");
    Serial.print(Kp);
    Serial.print(", Ki=");
    Serial.print(Ki);
    Serial.print(", Kd=");
    Serial.println(Kd);
  }
  
  delay(50);
}

long microsecondsToCentimeters(long microseconds) {
  // Rumus: jarak (cm) = (durasi / 29) / 2, yang setara dengan durasi / 58
  return microseconds / 29 / 2;
}
