// Bagian Sensor Ultrasonik (HC-SR04)
const int trigPin = 7;
const int echoPin = 6;


// Bagian Kontrol Motor (H-Bridge)
const int forwardpin = 12;
const int backwardpin = 10;
const int speedpin = 11;


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

void setup()
{
  Serial.begin(9600);
  Serial.println("Sistem Kendali Jarak dengan PID");
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(forwardpin, OUTPUT);
  pinMode(backwardpin, OUTPUT);

}

void loop()
{
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
  
  // Kontrol motor berdasarkan output PID
  if (output > 0) {
    // Jarak < setpoint maka outputnya mundur
    digitalWrite(forwardpin, LOW);
    digitalWrite(backwardpin, HIGH);
  } else {
    // Jarak > setpoint maka outputnya maju
    digitalWrite(forwardpin, HIGH);
    digitalWrite(backwardpin, LOW);
  }
  
  
  // Konversi output PID ke PWM (0-255)
  int pwm = constrain(abs(output), 0, 255);
  analogWrite(speedpin, pwm);

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
  
  // Graph di Tinkercad
  Serial.print(distance_cm);  // Menampilkan jarak
  Serial.print(",");
  Serial.println(pwm); // Menampilkan kecepatan motor
  
  delay(50);
}

// Fungsi baca jarak HC-SR04
float readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // Rumus: jarak (cm) = (durasi / 29) / 2, yang setara dengan durasi / 58
  return microseconds / 29 / 2;
}