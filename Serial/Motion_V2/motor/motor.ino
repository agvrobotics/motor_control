// ------------------- Encoder pins -------------------
// Front Right
#define ENCA_FR 2   // Green
#define ENCB_FR 3   // Yellow

// Rear Left
#define ENCA_RL 18  // Green
#define ENCB_RL 19  // Yellow

// Rear Right
#define ENCA_RR 20  // Yellow
#define ENCB_RR 21  // Green


// ------------------- Motor pins -------------------
// Front Motors (Driver 1)
#define ENA_F 5
#define ENB_F 6
#define IN1_F 22
#define IN2_F 23
#define IN3_F 24
#define IN4_F 25

// Rear Motors (Driver 2)
#define ENA_R 7
#define ENB_R 8
#define IN1_R 26
#define IN2_R 27
#define IN3_R 28
#define IN4_R 29


// ------------------- Encoder counters -------------------
volatile long countFR = 0;
volatile long countRL = 0;
volatile long countRR = 0;

// ------------------- Timing -------------------
unsigned long lastReport = 0;
const unsigned long reportInterval = 20; // 50 Hz

void setup() {
  // Motor pins
  pinMode(ENA_F, OUTPUT); pinMode(ENB_F, OUTPUT);
  pinMode(IN1_F, OUTPUT); pinMode(IN2_F, OUTPUT);
  pinMode(IN3_F, OUTPUT); pinMode(IN4_F, OUTPUT);
  pinMode(ENA_R, OUTPUT); pinMode(ENB_R, OUTPUT);
  pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT);
  pinMode(IN3_R, OUTPUT); pinMode(IN4_R, OUTPUT);

  // Encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA_FR), readEncoderFR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_FR), readEncoderFR_B, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENCA_RL), readEncoderRL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_RL), readEncoderRL_B, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENCA_RR), readEncoderRR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_RR), readEncoderRR_B, CHANGE);

  Serial.begin(115200);
}

// ------------------- Main loop -------------------
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    float linear = 0, angular = 0;

    int commaIndex = cmd.indexOf(',');
    if (commaIndex > 0) {
      linear = cmd.substring(0, commaIndex).toFloat();
      angular = cmd.substring(commaIndex + 1).toFloat();
      setMotorPWM(linear, angular);
    }
  }
  // Report encoder counts at fixed rate
  if (millis() - lastReport >= reportInterval) {
    lastReport = millis();
    Serial.print(countFR);
    Serial.print(",");
    Serial.print(countRL);
    Serial.print(",");
    Serial.print(countRR);
    Serial.println();
  }
}

// ------------------- Convert cmd_vel to PWM -------------------
void setMotorPWM(float linear, float angular) {
  //wheelbase is the distance between the left and right wheel contact points
  const float WHEEL_BASE = 0.2325;
  const int MAX_PWM = 255;

  float v_left  = linear - (angular * WHEEL_BASE / 2.0);
  float v_right = linear + (angular * WHEEL_BASE / 2.0);

  // Scale linear velocity to PWM (example mapping, adjust as needed)
  int pwmFL = constrain(int(v_left * 1000), -MAX_PWM, MAX_PWM);
  int pwmFR = constrain(int(v_right * 1000), -MAX_PWM, MAX_PWM);

  int pwmRL = pwmFL;
  int pwmRR = pwmFR; 

  // Front Left
  if (pwmFL >= 0) { digitalWrite(IN1_F,HIGH); digitalWrite(IN2_F,LOW); }
  else { digitalWrite(IN1_F,LOW); digitalWrite(IN2_F,HIGH); pwmFL = -pwmFL; }
  analogWrite(ENA_F, pwmFL);

  // Front Right
  if (pwmFR >= 0) { digitalWrite(IN3_F,HIGH); digitalWrite(IN4_F,LOW); }
  else { digitalWrite(IN3_F,LOW); digitalWrite(IN4_F,HIGH); pwmFR = -pwmFR; }
  analogWrite(ENB_F, pwmFR);

  // Rear Left
  if (pwmRL >= 0) { digitalWrite(IN3_R,HIGH); digitalWrite(IN4_R,LOW); }
  else { digitalWrite(IN3_R,LOW); digitalWrite(IN4_R,HIGH); pwmRL = -pwmRL; }
  analogWrite(ENA_R, pwmRL);

  // Rear Right
  if (pwmRR >= 0) { digitalWrite(IN1_R,HIGH); digitalWrite(IN2_R,LOW); }
  else { digitalWrite(IN1_R,LOW); digitalWrite(IN2_R,HIGH); pwmRR = -pwmRR; }
  analogWrite(ENB_R, pwmRR);

}

// ------------------- Encoder ISRs -------------------
void readEncoderFR_A() { int a=digitalRead(ENCA_FR), b=digitalRead(ENCB_FR); if(a==b) countFR++; else countFR--; }
void readEncoderFR_B() { int a=digitalRead(ENCA_FR), b=digitalRead(ENCB_FR); if(a!=b) countFR++; else countFR--; }

void readEncoderRL_A() { int a=digitalRead(ENCA_RL), b=digitalRead(ENCB_RL); if(a==b) countRL++; else countRL--; }
void readEncoderRL_B() { int a=digitalRead(ENCA_RL), b=digitalRead(ENCB_RL); if(a!=b) countRL++; else countRL--; }

void readEncoderRR_A() { int a=digitalRead(ENCA_RR), b=digitalRead(ENCB_RR); if(a==b) countRR++; else countRR--; }
void readEncoderRR_B() { int a=digitalRead(ENCA_RR), b=digitalRead(ENCB_RR); if(a!=b) countRR++; else countRR--; }
