// ------------------- Encoder pins -------------------
#define ENCA_FL 18
#define ENCB_FL 19

#define ENCA_FR 20
#define ENCB_FR 21

#define ENCA_RR 2
#define ENCB_RR 3

// ------------------- Motor pins -------------------
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

// ------------------- Counters -------------------
volatile long countFL = 0;
volatile long countFR = 0;
volatile long countRR = 0;

void setup() {
  // --- Motor pins ---
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

  // --- Encoder pins ---
  pinMode(ENCA_FL, INPUT);
  pinMode(ENCB_FL, INPUT);
  pinMode(ENCA_FR, INPUT);
  pinMode(ENCB_FR, INPUT);
  pinMode(ENCA_RR, INPUT);
  pinMode(ENCB_RR, INPUT);

  // Attach interrupts for FL
  attachInterrupt(digitalPinToInterrupt(ENCA_FL), readEncoderFL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_FL), readEncoderFL_B, CHANGE);

  // Attach interrupts for FR
  attachInterrupt(digitalPinToInterrupt(ENCA_FR), readEncoderFR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_FR), readEncoderFR_B, CHANGE);

  // Attach interrupts for RR
  attachInterrupt(digitalPinToInterrupt(ENCA_RR), readEncoderRR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_RR), readEncoderRR_B, CHANGE);

  Serial.begin(115200);

  // --- Start all motors forward ---
  // Front
  digitalWrite(IN1_F, HIGH);
  digitalWrite(IN2_F, LOW);
  digitalWrite(IN3_F, HIGH);
  digitalWrite(IN4_F, LOW);
  analogWrite(ENA_F, 200);
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
  static long lastCountFL = 0;
  static long lastCountFR = 0;
  static long lastCountRR = 0;
  static unsigned long lastTime = 0;

  if (millis() - lastTime >= 200) {  // update every 200 ms
    unsigned long now = millis();

    long deltaFL = countFL - lastCountFL;
    long deltaFR = countFR - lastCountFR;
    long deltaRR = countRR - lastCountRR;

    String dirFL = (deltaFL > 0) ? "Forward" : (deltaFL < 0) ? "Reverse" : "Stopped";
    String dirFR = (deltaFR > 0) ? "Forward" : (deltaFR < 0) ? "Reverse" : "Stopped";
    String dirRR = (deltaRR > 0) ? "Forward" : (deltaRR < 0) ? "Reverse" : "Stopped";

    Serial.print("FL: "); Serial.print(countFL); Serial.print(" (Δ"); Serial.print(deltaFL); Serial.print(", "); Serial.print(dirFL); Serial.print(") | ");
    Serial.print("FR: "); Serial.print(countFR); Serial.print(" (Δ"); Serial.print(deltaFR); Serial.print(", "); Serial.print(dirFR); Serial.print(") | ");
    Serial.print("RR: "); Serial.print(countRR); Serial.print(" (Δ"); Serial.print(deltaRR); Serial.print(", "); Serial.print(dirRR); Serial.println(")");

    lastCountFL = countFL;
    lastCountFR = countFR;
    lastCountRR = countRR;
    lastTime = now;
  }
}

// ------------------- FL Encoder ISRs -------------------
void readEncoderFL_A() {
  int a = digitalRead(ENCA_FL);
  int b = digitalRead(ENCB_FL);
  if (a == b) countFL++;
  else countFL--;
}
void readEncoderFL_B() {
  int a = digitalRead(ENCA_FL);
  int b = digitalRead(ENCB_FL);
  if (a != b) countFL++;
  else countFL--;
}

// ------------------- FR Encoder ISRs -------------------
void readEncoderFR_A() {
  int a = digitalRead(ENCA_FR);
  int b = digitalRead(ENCB_FR);
  if (a == b) countFR++;
  else countFR--;
}
void readEncoderFR_B() {
  int a = digitalRead(ENCA_FR);
  int b = digitalRead(ENCB_FR);
  if (a != b) countFR++;
  else countFR--;
}

// ------------------- RR Encoder ISRs -------------------
void readEncoderRR_A() {
  int a = digitalRead(ENCA_RR);
  int b = digitalRead(ENCB_RR);
  if (a == b) countRR++;
  else countRR--;
}
void readEncoderRR_B() {
  int a = digitalRead(ENCA_RR);
  int b = digitalRead(ENCB_RR);
  if (a != b) countRR++;
  else countRR--;
}
