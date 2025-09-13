// ------------------- Encoder pins -------------------
#define ENCA_FL 18
#define ENCB_FL 19
#define ENCA_FR 20
#define ENCB_FR 21
#define ENCA_RR 2
#define ENCB_RR 3

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

// ------------------- Counters -------------------
volatile long countFL = 0;
volatile long countFR = 0;
volatile long countRR = 0;

// ------------------- Timing -------------------
unsigned long lastReport = 0;
const unsigned long reportInterval = 20; // 50 Hz

// ------------------- PID constants -------------------
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.1;

// ------------------- PID state -------------------
float prevErrorFL = 0, prevErrorFR = 0, prevErrorRR = 0;
float integralFL = 0, integralFR = 0, integralRR = 0;

// ------------------- Target speeds (in encoder counts per interval) -------------------
float targetFL = 0;
float targetFR = 0;
float targetRR = 0;

// Robot parameters
const float WHEEL_RADIUS = 0.03;  // meters
const float WHEEL_BASE = 0.2;     // meters distance between left/right wheels

void setup() {
  // Motor pins
  pinMode(ENA_F, OUTPUT); pinMode(ENB_F, OUTPUT);
  pinMode(IN1_F, OUTPUT); pinMode(IN2_F, OUTPUT);
  pinMode(IN3_F, OUTPUT); pinMode(IN4_F, OUTPUT);
  pinMode(ENA_R, OUTPUT); pinMode(ENB_R, OUTPUT);
  pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT);
  pinMode(IN3_R, OUTPUT); pinMode(IN4_R, OUTPUT);

  // Encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA_FL), readEncoderFL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_FL), readEncoderFL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_FR), readEncoderFR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_FR), readEncoderFR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_RR), readEncoderRR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_RR), readEncoderRR_B, CHANGE);

  Serial.begin(115200);
}

// ------------------- Main loop -------------------
void loop() {
  // --- Read cmd_vel from ROS ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    float v = 0, w = 0;

    // Expect CSV: "v,omega"
    int commaIndex = cmd.indexOf(',');
    if (commaIndex > 0) {
      v = cmd.substring(0, commaIndex).toFloat();
      w = cmd.substring(commaIndex + 1).toFloat();
      setTargetWheelSpeeds(v, w);
    }
  }

  // --- Update motors via PID ---
  updateMotorPID();

  // --- Report encoder counts ---
  unsigned long now = millis();
  if (now - lastReport >= reportInterval) {
    lastReport = now;

    long fl, fr, rr;
    noInterrupts();
      fl = countFL; fr = countFR; rr = countRR;
    interrupts();

    char dirFL = (fl > 0) ? 'F' : (fl < 0) ? 'R' : 'S';
    char dirFR = (fr > 0) ? 'F' : (fr < 0) ? 'R' : 'S';
    char dirRR = (rr > 0) ? 'F' : (rr < 0) ? 'R' : 'S';

    Serial.print("ENC,");
    Serial.print(fl); Serial.print(","); Serial.print(dirFL); Serial.print(",");
    Serial.print(fr); Serial.print(","); Serial.print(dirFR); Serial.print(",");
    Serial.print(rr); Serial.print(","); Serial.println(dirRR);
  }
}

// ------------------- Convert cmd_vel to wheel targets -------------------
void setTargetWheelSpeeds(float linear, float angular) {
  // Differential drive kinematics
  float v_left = linear - (angular * WHEEL_BASE / 2.0);
  float v_right = linear + (angular * WHEEL_BASE / 2.0);

  // Convert m/s to encoder counts per interval
  const float COUNTS_PER_METER = 1000.0; // Example, calibrate with your encoder
  targetFL = v_left * COUNTS_PER_METER * (reportInterval / 1000.0);
  targetFR = v_right * COUNTS_PER_METER * (reportInterval / 1000.0);
  targetRR = targetFR;          // Use FR encoder for RR PID
}

// ------------------- PID control -------------------
void updateMotorPID() {
  noInterrupts();
    long encFL = countFL;
    long encFR = countFR;
    long encRR = countRR;
  interrupts();

  // Calculate errors
  float errorFL = targetFL - encFL;
  float errorFR = targetFR - encFR;
  float errorRR = targetRR - encRR;

  // Integral
  integralFL += errorFL;
  integralFR += errorFR;
  integralRR += errorRR;

  // Derivative
  float derivFL = errorFL - prevErrorFL;
  float derivFR = errorFR - prevErrorFR;
  float derivRR = errorRR - prevErrorRR;

  prevErrorFL = errorFL;
  prevErrorFR = errorFR;
  prevErrorRR = errorRR;

  // PID output to PWM
  int pwmFL = constrain(Kp*errorFL + Ki*integralFL + Kd*derivFL, -255, 255);
  int pwmFR = constrain(Kp*errorFR + Ki*integralFR + Kd*derivFR, -255, 255);
  int pwmRR = constrain(Kp*errorRR + Ki*integralRR + Kd*derivRR, -255, 255);

  setMotorPWM(pwmFL, pwmFR, pwmRR);
}

// ------------------- Motor driver function -------------------
void setMotorPWM(int pwmFL, int pwmFR, int pwmRR) {
  // Front Left
  if (pwmFL >= 0) { digitalWrite(IN1_F,HIGH); digitalWrite(IN2_F,LOW); }
  else { digitalWrite(IN1_F,LOW); digitalWrite(IN2_F,HIGH); pwmFL = -pwmFL; }
  analogWrite(ENA_F, pwmFL);

  // Front Right
  if (pwmFR >= 0) { digitalWrite(IN3_F,HIGH); digitalWrite(IN4_F,LOW); }
  else { digitalWrite(IN3_F,LOW); digitalWrite(IN4_F,HIGH); pwmFR = -pwmFR; }
  analogWrite(ENB_F, pwmFR);

  // Rear Right
  if (pwmRR >= 0) { digitalWrite(IN3_R,HIGH); digitalWrite(IN4_R,LOW); }
  else { digitalWrite(IN3_R,LOW); digitalWrite(IN4_R,HIGH); pwmRR = -pwmRR; }
  analogWrite(ENB_R, pwmRR);

  // Approximate Rear Left with FL
  if (pwmFL >= 0) { digitalWrite(IN1_R,HIGH); digitalWrite(IN2_R,LOW); }
  else { digitalWrite(IN1_R,LOW); digitalWrite(IN2_R,HIGH); }
  analogWrite(ENA_R, pwmFL);
}

// ------------------- Encoder ISRs (unchanged) -------------------
void readEncoderFL_A() { int a=digitalRead(ENCA_FL), b=digitalRead(ENCB_FL); if(a==b) countFL++; else countFL--; }
void readEncoderFL_B() { int a=digitalRead(ENCA_FL), b=digitalRead(ENCB_FL); if(a!=b) countFL++; else countFL--; }
void readEncoderFR_A() { int a=digitalRead(ENCA_FR), b=digitalRead(ENCB_FR); if(a==b) countFR++; else countFR--; }
void readEncoderFR_B() { int a=digitalRead(ENCA_FR), b=digitalRead(ENCB_FR); if(a!=b) countFR++; else countFR--; }
void readEncoderRR_A() { int a=digitalRead(ENCA_RR), b=digitalRead(ENCB_RR); if(a==b) countRR++; else countRR--; }
void readEncoderRR_B() { int a=digitalRead(ENCA_RR), b=digitalRead(ENCB_RR); if(a!=b) countRR++; else countRR--; }
