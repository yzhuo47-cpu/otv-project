// ===== Pins and settings (change to match your setup) =====
// One L298N drives all four motors:
//   Right two motors -> OUT1/OUT2, controlled by ENA, IN1, IN2
//   Left two motors  -> OUT3/OUT4, controlled by ENB, IN3, IN4
// Pins match the current wiring (ENA and ENB must be on PWM pins)
const int RIGHT_EN  = 10;   // L298N ENA (PWM, sets right side speed)
const int RIGHT_IN1 = 9;    // L298N IN1
const int RIGHT_IN2 = 8;    // L298N IN2
const int LEFT_IN1  = 6;    // L298N IN4 (left motors are mirrored, so IN3/IN4 are swapped)
const int LEFT_IN2  = 7;    // L298N IN3
const int LEFT_EN   = 5;    // L298N ENB (PWM, sets left side speed)

const int LEFT_SPEED  = 255;   // Left side PWM (lower it a little if the OTV curves right)
const int RIGHT_SPEED = 255;   // Right side PWM (lower it a little if the OTV curves left)
const unsigned long DRIVE_TIME = 75000;   // Time (ms) to cover 3.4 m -- measure and adjust this

// Drive one side: positive = forward, negative = backward, 0 = stop
void setSide(int en, int in1, int in2, int speed) {
  if (speed > 0) {          // Forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {   // Backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {                  // Stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(en, abs(speed));   // Speed is set through the enable pin (PWM)
}

// Drive both sides at once
void drive(int left, int right) {
  setSide(LEFT_EN, LEFT_IN1, LEFT_IN2, left);
  setSide(RIGHT_EN, RIGHT_IN1, RIGHT_IN2, right);
}

void setup() {
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_EN, OUTPUT);
  drive(0, 0);   // Make sure all motors are off at start

  delay(3000);   // Time to set the OTV down after turning it on

  // Drive straight forward for DRIVE_TIME, then stop
  drive(LEFT_SPEED, RIGHT_SPEED);
  delay(DRIVE_TIME);
  drive(0, 0);
}

void loop() {
  // Nothing to do after driving forward
}
