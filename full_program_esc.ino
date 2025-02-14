#include <Servo.h>

// Bagian Sensor Ultrasonik (HC-SR04)
const int trigPin = 7;
const int echoPin = 6;

// Bagian Kontrol Motor ESC
Servo esc;
const int escPin = 9;

// Bagian PID Controller
class PID {
  public:
    PID(float Kp, float Ki, float Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {
      reset();
    }

    void setGains(float Kp, float Ki, float Kd) {
      this->Kp = Kp;
      this->Ki = Ki;
      this->Kd = Kd;
      reset();
    }

    float compute(float setpoint, float input) {
      float error = setpoint - input;
      integral += error * dt;
      float derivative = (error - prevError) / dt;
      prevError = error;
      return Kp * error + Ki * integral + Kd * derivative;
    }

    void reset() {
      integral = 0;
      prevError = 0;
    }
  
  private:
    float Kp, Ki, Kd;
    float integral = 0;
    float prevError = 0;
    const float dt = 0.1;
  };

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
