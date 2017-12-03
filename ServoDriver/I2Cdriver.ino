// Servo control module with I2C interface.
// This will be a slave device using a pot for reference
// and either a PWM output signal  or a H-Bridge output

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>


// IO Asignments
/*
 * A7
 * A6 Ref Signal for analog mode
 * A5 SCL
 * A4 SDA
 * A3
 * A2
 * A1 Motor Load monitor
 * A0 Potion Signal In
 * D0 USB RX
 * D1 USB TX
 * D2 Addr Sel 0
 * D3 Addr Sel 1 - PWM/OC2B
 * D4 Addr Sel 2
 * D5 PWM/OC0B
 * D6 H-Bridge PWM output - PWM/OC0A
 * D7 Addr Sel 3
 * D8 Mode Select. 1 = Analog Mode, 0 = I2C Mode
 * D9 ESC PWM Output - PWM/OC1A/Hi-Res
 * D10 PWM/OC1B/Hi-Res
 * D11 PWM/OC2A
 * D12 H-Bridge Direction
 * D13 LED Status
 */

#define DefaultBaseAddr 10
#define Addr0 2
#define Addr1 3
#define Addr2 4
#define Addr3 7
#define Mode0 8
#define ESCoutput 9
#define HBridgePWM 6
#define HBridgeDir 12
#define PositionInput A0
#define MotorLoadInput A1
#define ReferenceInput A6
#define DefaultAnalogFilter 10
#define DefaultKp 0.1
#define DefaultKi 0.1
#define DefaultKd 0.0
#define DefaultMinPos 0
#define DefaultMaxPos 1023
#define DefaultDeadBand 5
#define DefaultMaxSpeed 255
#define DefaultMotorLoad 102    // Aproximately 0.1 Amp = 1
#define Timeout 2000     // in mS

// Variable Declaration
typedef struct  {
  float P;
  float I;
  float D;
  float AnalogFilter = DefaultAnalogFilter;
  int MinPos = DefaultMinPos;
  int MaxPos = DefaultMaxPos;
  int DeadBand = DefaultDeadBand;
  int MaxSpeed = DefaultMaxSpeed;
  int MaxMotorLoad = DefaultMotorLoad;
} PID_Settings;

typedef struct {
  float CurentPos;
  int SetPos;
  int MotorLoad;
  bool Direction;                   // true = Reverse
  PID_Settings K;
} Status_Structure;

union MainStruct {
  float test;
  Status_Structure Values;
//  byte Data[];
};

Servo ESCservo;                     // create servo object to control a servo

//PID_Settings K;                     // PID settings
Status_Structure MotorStatus;       // Structure to hold all of the motor status
//MainStruct MotorStatus2;       // Structure to hold all of the motor status
int MinCurrent=0, MaxCurrent=1023;  // Analog Input range
int WireIDNumber = DefaultBaseAddr; // This needs to be programable
float PID_Output;                   // this will be a percentage ?
int eeAddress = 0;
bool SingnalOK = false;             // Set the coms state to inactive
unsigned long LastSignal = 0;       //
unsigned long LastTime;             // Used to calculate the PID loop Time,

void setup() {
  // Configer the IO pins
  pinMode(Addr0, INPUT_PULLUP);
  pinMode(Addr1, INPUT_PULLUP);
  pinMode(Addr2, INPUT_PULLUP);
  pinMode(Addr3, INPUT_PULLUP);
  pinMode(Mode0, INPUT_PULLUP);
  pinMode(PositionInput, INPUT);
  pinMode(MotorLoadInput, INPUT);
  pinMode(ReferenceInput, INPUT);
  pinMode(HBridgePWM, OUTPUT);
  pinMode(HBridgeDir, OUTPUT);
  pinMode(ESCoutput, OUTPUT);
  // Determine the I2C Address
  WireIDNumber = DefaultBaseAddr;
  if (digitalRead(Addr0) == 0){
    WireIDNumber = WireIDNumber + 1;
  }
  if (digitalRead(Addr1) == 0){
    WireIDNumber = WireIDNumber + 2;
  }
  if (digitalRead(Addr2) == 0){
    WireIDNumber = WireIDNumber + 4;
  }
  if (digitalRead(Addr3) == 0){
    WireIDNumber = WireIDNumber + 8;
  }
  ESCservo.attach(ESCoutput);  // attaches the servo on ESCoutput to the servo object
  // Configer the I2C BUS
  Wire.begin(WireIDNumber);             // join i2c bus with address #8
  Wire.onReceive(receiveCommandEvent);  // register event
  Wire.onRequest(requestStatusEvent);   // register event

  Serial.begin(9600);                   // start serial for output
  // Load PID Settings
  EEPROM.get(eeAddress, MotorStatus.K);
  if (MotorStatus.K.P == NAN or MotorStatus.K.I == NAN or MotorStatus.K.D == NAN){
    MotorStatus.K.P = DefaultKp;
    MotorStatus.K.I = DefaultKi;
    MotorStatus.K.D = DefaultKd;
    EEPROM.put(eeAddress, MotorStatus.K);
  }
  LastTime = millis();
}

void loop() {
  float Error;
  float Intergral = 0.0;
  float PreviousError = 0.0;
  float DeltaTime;
  unsigned long CurrentTime;
  
  // Process timing Control
  CurrentTime = millis();
  DeltaTime = float(CurrentTime - LastTime);
  LastTime = CurrentTime;
  if (LastSignal < (CurrentTime - Timeout)) {
    SingnalOK = true;
  } else {
    SingnalOK = false;
  }
  
  // Get Current Position and apply some filtering
  MotorStatus.CurentPos = ((MotorStatus.CurentPos*MotorStatus.K.AnalogFilter)+float(analogRead(PositionInput)))/(MotorStatus.K.AnalogFilter+1);

  // Get current Reference Position
  if (digitalRead(Mode0) == 0){
    // Analog input Mode need to read the input and apply filtering.
    MotorStatus.SetPos = ((MotorStatus.SetPos*MotorStatus.K.AnalogFilter)+float(analogRead(ReferenceInput)))/(MotorStatus.K.AnalogFilter+1);
  } else {
    // I2C Mode
    if (SingnalOK) {
      // I2C is OK
    } else {
      // I2C is not OK
      MotorStatus.SetPos = MotorStatus.CurentPos;
    }
  }
  
  // Calculate the PID Output
  Error = float(MotorStatus.SetPos) - MotorStatus.CurentPos;
  Intergral = Intergral + (Error * DeltaTime);
  PID_Output = (Error * MotorStatus.K.P) + (Intergral * MotorStatus.K.I) + (((Error - PreviousError)/DeltaTime) * MotorStatus.K.D);
  
  // Process the output to drive the motor
  if (PID_Output < 0){
    PID_Output = PID_Output * -1;
    MotorStatus.Direction = true;
  } else {
    MotorStatus.Direction = false;
  }
  if (PID_Output > DefaultMaxSpeed) PID_Output = DefaultMaxSpeed; 
  if (PID_Output > MotorStatus.K.DeadBand){
    if(MotorStatus.Direction){
      ESCservo.write(map(PID_Output, 0, 255, 90, 180));
    }else{
      ESCservo.write(map(PID_Output, 255, 0, 0, 90));
    }
  }else{
    ESCservo.write(90);
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveCommandEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}


// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestStatusEvent() {
//  Wire.write(MotorStatus, sizeof(MotorStatus)); // respond with current motor Status structure
  // as expected by master
}

