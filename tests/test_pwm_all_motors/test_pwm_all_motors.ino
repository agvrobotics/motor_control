// ----- Pin Definitions -----
// Front Motors (Driver 1)
#define ENA_F 5   // Front Left Enable (PWM)
#define ENB_F 6   // Front Right Enable (PWM)
#define IN1_F 22  // FL IN1
#define IN2_F 23  // FL IN2
#define IN3_F 24  // FR IN3
#define IN4_F 25  // FR IN4

// Rear Motors (Driver 2)
#define ENA_R 7   // Rear Left Enable (PWM)
#define ENB_R 8   // Rear Right Enable (PWM)
#define IN1_R 26  // RL IN1
#define IN2_R 27  // RL IN2
#define IN3_R 28  // RR IN3
#define IN4_R 29  // RR IN4

void setup() {
  // Front motor pins
  pinMode(ENA_F, OUTPUT);
  pinMode(ENB_F, OUTPUT);
  pinMode(IN1_F, OUTPUT);
  pinMode(IN2_F, OUTPUT);
  pinMode(IN3_F, OUTPUT);
  pinMode(IN4_F, OUTPUT);

  // Rear motor pins
  pinMode(ENA_R, OUTPUT);
  pinMode(ENB_R, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(IN3_R, OUTPUT);
  pinMode(IN4_R, OUTPUT);

  // Stop all motors at startup
  stopAllMotors();
}

void loop() {
  // ----- Forward -----
  forwardAll(200);   // speed 200/255
  delay(5000);

  stopAllMotors();
  delay(1000);

  // ----- Reverse -----
  reverseAll(200);
  delay(5000);

  stopAllMotors();
  delay(2000);
}

// ----- Motor Control Functions -----
void forwardAll(int speed) {
  // Front
  digitalWrite(IN1_F, HIGH);
  digitalWrite(IN2_F, LOW);
  digitalWrite(IN3_F, HIGH);
  digitalWrite(IN4_F, LOW);
  analogWrite(ENA_F, speed);
  analogWrite(ENB_F, speed);

  // Rear
  digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
  digitalWrite(IN3_R, HIGH);
  digitalWrite(IN4_R, LOW);
  analogWrite(ENA_R, speed);
  analogWrite(ENB_R, speed);
}

void reverseAll(int speed) {
  // Front
  digitalWrite(IN1_F, LOW);
  digitalWrite(IN2_F, HIGH);
  digitalWrite(IN3_F, LOW);
  digitalWrite(IN4_F, HIGH);
  analogWrite(ENA_F, speed);
  analogWrite(ENB_F, speed);

  // Rear
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, HIGH);
  digitalWrite(IN3_R, LOW);
  digitalWrite(IN4_R, HIGH);
  analogWrite(ENA_R, speed);
  analogWrite(ENB_R, speed);
}

void stopAllMotors() {
  // Front
  analogWrite(ENA_F, 0);
  analogWrite(ENB_F, 0);
  digitalWrite(IN1_F, LOW);
  digitalWrite(IN2_F, LOW);
  digitalWrite(IN3_F, LOW);
  digitalWrite(IN4_F, LOW);

  // Rear
  analogWrite(ENA_R, 0);
  analogWrite(ENB_R, 0);
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);
  digitalWrite(IN3_R, LOW);
  digitalWrite(IN4_R, LOW);
}
