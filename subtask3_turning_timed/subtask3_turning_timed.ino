// ===== Pins and settings (change to match your setup) =====
const int LEFT_A  = 5;    // Left L298N IN1 + IN3
const int LEFT_B  = 6;    // Left L298N IN2 + IN4
const int RIGHT_A = 3;    // Right L298N IN1 + IN3
const int RIGHT_B = 11;   // Right L298N IN2 + IN4

const int SPEED = 255;        // Full speed: the 10 RPM motors are slow and need full torque
const int TURN_TIME = 5000;   // Time (ms) for a 90-degree turn -- measure and adjust this
const int PAUSE = 2000;       // Pause (ms) between turns

// Drive one side: positive = forward, negative = backward, 0 = stop
void setSide(int pinA, int pinB, int speed) {
  if (speed > 0) {
    analogWrite(pinA, speed);
    digitalWrite(pinB, LOW);
  } else if (speed < 0) {
    digitalWrite(pinA, LOW);
    analogWrite(pinB, -speed);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
}

// Drive both sides at once
void drive(int left, int right) {
  setSide(LEFT_A, LEFT_B, left);
  setSide(RIGHT_A, RIGHT_B, right);
}

// Turn 90 degrees left in place: left side backward, right side forward, for a fixed time
void turn90() {
  drive(-SPEED, SPEED);
  delay(TURN_TIME);
  drive(0, 0);
}

void setup() {
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);

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
