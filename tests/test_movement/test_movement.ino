// ----- Pin Definitions -----
// Front Motors (Driver 1)
#define ENA_F 5   // Front Left Enable (PWM)
#define ENB_F 6   // Front Right Enable (PWM)
#define IN1_F 22
#define IN2_F 23
#define IN3_F 24
#define IN4_F 25

// Rear Motors (Driver 2)
#define ENA_R 7   // Rear Left Enable (PWM)
#define ENB_R 8   // Rear Right Enable (PWM)
#define IN1_R 26
#define IN2_R 27
#define IN3_R 28
#define IN4_R 29

void setup() {
  // Set all pins as outputs
  pinMode(ENA_F, OUTPUT);
  pinMode(ENB_F, OUTPUT);
  pinMode(IN1_F, OUTPUT);
  pinMode(IN2_F, OUTPUT);
  pinMode(IN3_F, OUTPUT);
  pinMode(IN4_F, OUTPUT);

  pinMode(ENA_R, OUTPUT);
  pinMode(ENB_R, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(IN3_R, OUTPUT);
  pinMode(IN4_R, OUTPUT);

  // ----- Forward All Motors -----
  // Front
  digitalWrite(IN1_F, HIGH);
  digitalWrite(IN2_F, LOW);
  digitalWrite(IN3_F, HIGH);
  digitalWrite(IN4_F, LOW);
  analogWrite(ENA_F, 200);  // speed (0–255)
  analogWrite(ENB_F, 200);

  // Rear
  digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
  digitalWrite(IN3_R, HIGH);
  digitalWrite(IN4_R, LOW);
  analogWrite(ENA_R, 200);
  analogWrite(ENB_R, 200);
}

void loop() {
  // Nothing here → motors keep running forward
}
