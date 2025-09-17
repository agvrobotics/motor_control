#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(9); 

  for (int pos = 0; pos <= 180; pos++) {
    myServo.write(pos);
    delay(5);  
  }
  
  delay(3000);

  for (int pos = 180; pos >= 0; pos--) {
    myServo.write(pos);
    delay(3);
  }

  while (true) {
  }
}

void loop() {
}
