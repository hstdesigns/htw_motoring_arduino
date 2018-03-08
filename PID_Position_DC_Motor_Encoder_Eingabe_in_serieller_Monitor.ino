// PID motor position control.
// Eingabe der Sollposition im seriellen Monitor und senden

#include <PinChangeInt.h>
#include <PID_v1.h>
#define encodPinA1      2                       // Quadrature encoder A pin
#define encodPinB1      8                       // Quadrature encoder B pin
#define M1              9                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              10
double kp = 5 , ki = 50 , kd = 0.01;            // modify for optimal performance
double input = 0, output = 0, setpoint = 0;

volatile long number =0;
volatile long Differenz = 0;
volatile long encoderPos = 0;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

void setup() {
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(0, encoder, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  Serial.begin (115200);                              // for debugging
}

void loop() {

    
    setpoint = number;                       // analogRead(0) * 30 modify to fit motor and encoder characteristics, potmeter connected to A0
    input = encoderPos ;                     // data from encoder
    Serial.println(encoderPos);              // monitor motor position
    myPID.Compute();                         // calculate new output
    pwmOut(output);
    
    while (Serial.available() > 0) // if there's any serial available, read it
    {  
    // look for the next valid integer in the incoming serial stream:
    number = Serial.parseInt();
    // look for the newline. That's the end of your sentence:
    if (Serial.read() == '\n') {
    }
    Serial.println(number);
    } 
 
}
void pwmOut(int out) {                                // to H-Bridge board
  if (out > 0) {
    analogWrite(M1, out);                             // drive motor CW
    analogWrite(M2, 0);
  }
  else {
    analogWrite(M1, 0);
    analogWrite(M2, abs(out));                        // drive motor CCW
  }
}

void encoder()  {                                     // pulse and direction, direct port reading to save cycles
  if (PINB & 0b00000001)    encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      encoderPos--;             // if(digitalRead(encodPinB1)==LOW)   count --;
}
