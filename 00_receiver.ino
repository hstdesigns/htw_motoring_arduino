/*                Receiver
 For more details see: http://projectsfromtech.blogspot.com/
 */
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <PinChangeInt.h>
#define encodPinA1      2                       // Quadrature encoder A pin 2
#define encodPinB1      8                       // Quadrature encoder B pin 8 weil Interruptfaehig

#include <PID_v1.h>
#define M1              9                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              10                      // PWM outputs to L298N H-Bridge motor driver module

double kp = 5 , ki = 0 , kd = 0.1;            // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
volatile long encoderPos = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

int Differenz = 0;

LiquidCrystal_I2C lcd(0x20,16,4);

int Encoder_Master = 0;
int Encoder_Slave = 0;
char zeichenStart;

void setup()
{
    pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
    pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
    attachInterrupt(0, encoder, FALLING);               // update encoder position via Interrupt
  
    lcd.init();
    lcd.setCursor(0,0);

    TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(1);
    myPID.SetOutputLimits(-255, 255);

    Serial.begin(9600);
}

void loop()
{    
  while (Differenz >=10) 
  {
    setpoint = Encoder_Master;          // modify to fit motor and encoder characteristics, potmeter connected to A0
    input = encoderPos;              // data from encoder
    //Serial.println(encoderPos);         // monitor motor position
    myPID.Compute();                    // calculate new output
    pwmOut(output);
  }
    /*lcd.clear();
    
    lcd.setCursor(0,0);
    lcd.print("Master:");
    lcd.setCursor(8,0);
    lcd.print(Encoder_Master);
    
    lcd.setCursor(0,1);
    lcd.print("Slave:");
    lcd.setCursor(8,1);
    lcd.print(Encoder_Slave);
  
    //delay (50);*/   
    
  
  while (Serial.available() > 0)  // wenn Daten empfangen
  {  
    zeichenStart = Serial.read(); // wenn Startzeichen (Header)
      switch (zeichenStart)
      {
      case 'A' : Encoder_Master = Serial.parseInt(); break;  // wenn A wird der folgende wert in den int Encoder_Master übertragen
      default: break;
      }

    input = encoderPos ;                // Encoder auslesen (Input ist als float definiert
    Encoder_Slave = input;              // aus dem float wird hier ein int
  }   
  
  
  Differenz = abs(Encoder_Master - Encoder_Slave);


     
  
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



