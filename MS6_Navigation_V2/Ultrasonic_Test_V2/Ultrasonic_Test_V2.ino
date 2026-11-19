// Ultrasonic_Test (V2): checks the HC-SR04 by itself (no Vision System, no motors)
// Upload, open Serial Monitor at 9600 baud, put your hand 10 / 30 / 100 cm in front of the sensor.

const int TRIG = 12;      // same pins as Nav2
const int ECHO = 11;

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(9600);
}

void loop() {
  // One measurement: a 10 us pulse on TRIG, then time how long ECHO stays HIGH
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long t = pulseIn(ECHO, HIGH, 30000);   // echo time in us; 0 = no echo within 30 ms

  // Print the result
  if (t == 0) {
    Serial.println("no echo");
  } else {
    float cm = t * 0.0343 / 2;   // sound: 0.0343 cm/us, divide by 2 for there and back
    Serial.print(cm); Serial.println(" cm");
  }
  delay(200);   // about 5 readings per second
}
