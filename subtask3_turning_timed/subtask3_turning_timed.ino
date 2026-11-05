// ===== Pins and settings (change to match your setup) =====
// One L298N drives all four motors:
//   Right two motors -> OUT1/OUT2, controlled by ENA, IN1, IN2
//   Left two motors  -> OUT3/OUT4, controlled by ENB, IN3, IN4
// Pins match the current wiring (ENA and ENB must be on PWM pins)
const int RIGHT_EN  = 10;   // L298N ENA (PWM, sets right side speed)
const int RIGHT_IN1 = 9;    // L298N IN1
const int RIGHT_IN2 = 8;    // L298N IN2
const int LEFT_IN1  = 7;    // L298N IN3
const int LEFT_IN2  = 6;    // L298N IN4
const int LEFT_EN   = 5;    // L298N ENB (PWM, sets left side speed)

const int SPEED = 255;        // Full speed: the 10 RPM motors are slow and need full torque
const int TURN_TIME = 5000;   // Time (ms) for a 90-degree turn -- measure and adjust this
const int PAUSE = 2000;       // Pause (ms) between turns

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

// Turn 90 degrees left in place: left side backward, right side forward, for a fixed time
void turn90() {
  drive(-SPEED, SPEED);
  delay(TURN_TIME);
  drive(0, 0);
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

  // Turn 3 times in a row
  for (int i = 0; i < 3; i++) {
    turn90();
    delay(PAUSE);
  }
}

void loop() {
  // Nothing to do after the 3 turns
}
