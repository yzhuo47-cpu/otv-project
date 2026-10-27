// drive.h - skid-steer motor control through the L298N drivers.
#pragma once
#include <Arduino.h>

namespace drive {

void begin();

// left / right speed: -255 (full reverse) .. 0 .. +255 (full forward)
void set(int left, int right);
void stop();

// Bench test helper: run ONE channel of one side, the other channels stay off.
// side: 'L' or 'R', index: position in LEFT_CHANNELS / RIGHT_CHANNELS.
void testChannel(char side, uint8_t index, int pwm);
uint8_t channelCount(char side);

}  // namespace drive
