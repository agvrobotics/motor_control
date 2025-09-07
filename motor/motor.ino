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

// ------------------- Report timing -------------------
unsigned long lastReport = 0;
const unsigned long reportInterval = 20; // use 10 to 50hz

// ------------------- PID constants -------------------
float Kp = 1.0;    // Proportional gain
float Ki = 0.0;    // Integral gain
float Kd = 0.1;    // Derivative gain

// PID state
long prevErrorFR = 0;
long prevErrorRR = 0;
long integralFR = 0;
long integralRR = 0;
// Set point
int targetPWM_FL = 0;
int targetPWM_FR = 0;
int targetPWM_RR = 0;

String Direction = "STOP";

void setup() {
  // Set motor pins as outputs
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

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA_FL), readEncoderFL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_FL), readEncoderFL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_FR), readEncoderFR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_FR), readEncoderFR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_RR), readEncoderRR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_RR), readEncoderRR_B, CHANGE);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "FORWARD_LOW") setMotors("FORWARD", "LOW");
    else if (cmd == "FORWARD_HIGH") setMotors("FORWARD", "HIGH");
    else if (cmd == "REVERSE") setMotors("REVERSE", "LOW");
    else if (cmd == "STOP") stopMotors();
  }

  // --- Encoder Report---
  unsigned long now = millis();
  if (now - lastReport >= reportInterval) {
    lastReport = now;

    long fl, fr, rr;
    noInterrupts();
      fl = countFL;
      fr = countFR;
      rr = countRR;
    interrupts();

    static long lastFl = 0;
    static long lastFr = 0;
    static long lastRr = 0;

    long dfl = fl - lastFl;
    long dfr = fr - lastFr;
    long drr = rr - lastRr;

    // Direction: 'F' = Forward (positive delta), 'R' = Reverse (negative), 'S' = Stopped (zero)
    char dirFL = (dfl > 0) ? 'F' : (dfl < 0) ? 'R' : 'S';
    char dirFR = (dfr > 0) ? 'F' : (dfr < 0) ? 'R' : 'S';
    char dirRR = (drr > 0) ? 'F' : (drr < 0) ? 'R' : 'S';

    // CSV format: ENC,<FL_total>,<FL_dir>,<FR_total>,<FR_dir>,<RR_total>,<RR_dir>
    // Serial.print("ENC,");
    // Serial.print(fl); Serial.print(","); Serial.print(dirFL); Serial.print(",");
    // Serial.print(fr); Serial.print(","); Serial.print(dirFR); Serial.print(",");
    // Serial.print(rr); Serial.print(","); Serial.print(dirRR); Serial.println();

    // save for next interval
    lastFl = fl;
    lastFr = fr;
    lastRr = rr;
  }
  updatePID();
}


// Motor control
void setMotors(String direction, String speedLevel) {
  int pwmValue;

  if (speedLevel == "HIGH") pwmValue = 255;
  else if (speedLevel == "LOW") pwmValue = 150;
  else pwmValue = 0;

  targetPWM_FL = pwmValue;
  targetPWM_FR = pwmValue;
  targetPWM_RR = pwmValue;
  Direction = direction;

  // Determine pin HIGH/LOW based on direction
  int fwdA = (direction == "FORWARD") ? HIGH : (direction == "REVERSE") ? LOW : LOW;
  int fwdB = (direction == "FORWARD") ? LOW  : (direction == "REVERSE") ? HIGH : LOW;

  // --- Front Motors ---
  digitalWrite(IN1_F, fwdA);
  digitalWrite(IN2_F, fwdB);
  digitalWrite(IN3_F, fwdA);
  digitalWrite(IN4_F, fwdB);

  // --- Rear Motors ---
  digitalWrite(IN1_R, fwdA);
  digitalWrite(IN2_R, fwdB);
  digitalWrite(IN3_R, fwdA);
  digitalWrite(IN4_R, fwdB);

  // analogWrite(ENA_F, pwmValue);
  // analogWrite(ENB_F, pwmValue);
  // analogWrite(ENA_R, pwmValue);
  // analogWrite(ENB_R, pwmValue);

}
// Stop function - smooth stop - adjust delay for choice
void stopMotors() {
  for (int pwm = targetPWM_FL; pwm >= 0; pwm -= 10) {
    analogWrite(ENA_F, pwm);
    analogWrite(ENB_F, pwm);
    analogWrite(ENA_R, pwm);
    analogWrite(ENB_R, pwm);
    delay(50);
  }
  setMotors("STOP", "NULL");
}

// ------------------- PID -------------------
void updatePID() {
    noInterrupts();
    long fl = countFL;
    long fr = countFR;
    long rr = countRR;
    interrupts();

    // Calculate errors relative to FL
    long errorFR, errorRR;
    if (Direction == "FORWARD") {
      errorFR = fl - fr;
      errorRR = fl - rr;
    } else if (Direction == "REVERSE") {
      // In reverse, counts decrease, so flip the subtraction
      errorFR = fr - fl;
      errorRR = rr - fl;
    } else {
      errorFR = 0;
      errorRR = 0;
    }

    // Integral
    integralFR += errorFR;
    integralRR += errorRR;

    // Derivative
    long derivativeFR = errorFR - prevErrorFR;
    long derivativeRR = errorRR - prevErrorRR;

    // PID output (adjust PWM)
    int adjustFR = Kp * errorFR + Ki * integralFR + Kd * derivativeFR;
    int adjustRR = Kp * errorRR + Ki * integralRR + Kd * derivativeRR;

    // Save previous error
    prevErrorFR = errorFR;
    prevErrorRR = errorRR;

    // --- Apply adjustments ---
    analogWrite(ENA_F, targetPWM_FL); // reference motor stays target PWM
    analogWrite(ENB_F, constrain(targetPWM_FR + adjustFR, 0, 255));
    analogWrite(ENB_R, constrain(targetPWM_RR + adjustRR, 0, 255));
}

// ------------------- Encoder ISRs -------------------
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


