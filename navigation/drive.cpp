#include "drive.h"
#include "config.h"

namespace {

const uint8_t N_LEFT  = sizeof(LEFT_CHANNELS)  / sizeof(LEFT_CHANNELS[0]);
const uint8_t N_RIGHT = sizeof(RIGHT_CHANNELS) / sizeof(RIGHT_CHANNELS[0]);

void setupChannel(const MotorChannel& c) {
  pinMode(c.pinA, OUTPUT);
  pinMode(c.pinB, OUTPUT);
  digitalWrite(c.pinA, LOW);
  digitalWrite(c.pinB, LOW);
  if (c.pinEn >= 0) {
    pinMode(c.pinEn, OUTPUT);
    analogWrite(c.pinEn, 0);
  }
}

// speed: -255..255
void writeChannel(const MotorChannel& c, int speed) {
  speed = constrain(speed, -255, 255);
  if (c.inverted) speed = -speed;
  int mag = abs(speed);

  if (c.pinEn < 0) {
    // Style (a): PWM straight into IN1 / IN2, ENA jumper on.
    if (speed > 0)      { analogWrite(c.pinA, mag); digitalWrite(c.pinB, LOW); }
    else if (speed < 0) { digitalWrite(c.pinA, LOW); analogWrite(c.pinB, mag); }
    else                { digitalWrite(c.pinA, LOW); digitalWrite(c.pinB, LOW); }
  } else {
    // Style (b): direction on IN1 / IN2, PWM on ENA.
    digitalWrite(c.pinA, speed > 0 ? HIGH : LOW);
    digitalWrite(c.pinB, speed < 0 ? HIGH : LOW);
    analogWrite(c.pinEn, mag);
  }
}

}  // namespace

namespace drive {

void begin() {
  for (uint8_t i = 0; i < N_LEFT; i++)  setupChannel(LEFT_CHANNELS[i]);
  for (uint8_t i = 0; i < N_RIGHT; i++) setupChannel(RIGHT_CHANNELS[i]);
  stop();
}

void set(int left, int right) {
  for (uint8_t i = 0; i < N_LEFT; i++)  writeChannel(LEFT_CHANNELS[i], left);
  for (uint8_t i = 0; i < N_RIGHT; i++) writeChannel(RIGHT_CHANNELS[i], right);
}

void stop() { set(0, 0); }

uint8_t channelCount(char side) { return side == 'L' ? N_LEFT : N_RIGHT; }

void testChannel(char side, uint8_t index, int pwm) {
  stop();
  if (side == 'L' && index < N_LEFT)  writeChannel(LEFT_CHANNELS[index], pwm);
  if (side == 'R' && index < N_RIGHT) writeChannel(RIGHT_CHANNELS[index], pwm);
}

}  // namespace drive
