#include <Wire.h>
#include <PID_v1.h>

#define ENCA 0  // Quadrature encoder A pin (C1)
#define ENCB 1  // Quadrature encoder B pin (C2)
#define M1 9          // PWM outputs to L298N H-Bridge motor driver module
#define M2 10

double kp = 1, ki = 20, kd = 0;  // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
unsigned long lastTime, now;

volatile long encoderPos = 0, last_pos = 0, lastPos = 0;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {
  pinMode(ENCA, INPUT);     // quadrature encoder input A
  pinMode(ENCB, INPUT);     // quadrature encoder input B
  attachInterrupt(digitalPinToInterrupt(ENCA), encoder, RISING);  // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;      // To prevent Motor Noise

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);

  Wire.begin(8);                 // join i2c bus with address #8 for Right Motor
  Wire.onRequest(requestEvent);  // register events
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
}

void loop() {
  now = millis();
  int timeChange = (now - lastTime);
  if (timeChange >= 500) {
    input = (360.0 * 1000 * (encoderPos - last_pos)) / (1856.0 * (now - lastTime));
    lastTime = now;
    last_pos = encoderPos;
  }

  myPID.Compute();  // calculate new output

  Serial.println(encoderPos);

  if (setpoint == 0) {
    myPID.SetMode(MANUAL);
    output = 0;
    myPID.SetMode(AUTOMATIC);
  }

  pwmOut(output);  // drive L298N H-Bridge module

  delay(10);
}

void pwmOut(int out) {  // to H-Bridge board
  if (out > 0) {
    analogWrite(M1, out);  // drive motor CW
    analogWrite(M2, 0);
  } else {
    analogWrite(M1, 0);
    analogWrite(M2, abs(out));  // drive motor CCW
  }
}

void encoder() {  // pulse and direction, direct port reading to save cycles
  if (digitalRead(ENCB)) encoderPos++;  // if(digitalRead(ENCB)==HIGH)   count --;
  else encoderPos--;  // if(digitalRead(ENCB)==LOW)   count ++;
}

void requestEvent() {
  int8_t s;

  s = (360.0 * (encoderPos - lastPos)) / 1856.0;  //change in position in degrees of the wheel

  lastPos = encoderPos;

  Wire.write(s);  // respond with message of 6 bytes
}

void receiveEvent(int howMany) {
  uint8_t a, b;
  a = Wire.read();
  b = Wire.read();
  setpoint = (double)((b << 8) | a);
}
