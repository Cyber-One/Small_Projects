/*
 * I2C based dual high power motor controller
 * The motor drivers come in a variety of type and methods of control.
 * This driver supports the following types:
 * Separate outputs used for the PWM drive forward and reverse
 * A single PWM signal for motor drive and one digital output for direction control
 * A single PWM signal for motor drive and complimentry output for direction control.
 * 
 * Created by Ray Edgley in colabaration with Bartosz Scencelek
 * for the Inmoov Walking Robot Project 
 */
#include <EEPROM.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Servo.h>

// Define Pin IO. Note D0 and D1 reserved for Debug Serial
#define ServoPin 13  // used to help with debugging or status or a servo output
// Define Pin IO. Note A4 and A5 reserved for I2C 
#define i2cAddr0 2  // used to set the I2C device address
#define i2cAddr1 4  // With 3 lines 8 addresses are possible
#define i2cAddr2 12
// Define Pin IO Motor 1
#define Motor1PWM1 5
#define Motor1PWM2 6
#define Motor1Dir1 7
#define Motor1Dir2 8
#define Motor1Position A0
// Define Pin IO Motor 2
#define Motor2PWM1 3
#define Motor2PWM2 11
#define Motor2Dir1 9
#define Motor2Dir2 10
#define Motor2Position A1
// That leaves A2, A3, A6 and A7 unused. 
// A6 and A7 are Analog only inputs so i could use A2 and A3 as input and output type select.
#define Motor1SetPoinTest A6
#define Motor2SetPoinTest A7
#define OutputType A2
#define INPUTtype A3

// Define the status bits
#define PacketType0   0
#define PacketType1   1
#define PacketType2   2
#define I2CADDR1      3
#define I2CADDR2      4
#define I2CADDR3      5
#define SERVOENABELED 6
#define NoAutoSleep   7
#define MOTOR1STOP    8
#define MOTOR1FWD     9
#define MOTOR1REV    10
#define MOTOR2STOP   11
#define MOTOR2FWD    12
#define MOTOR2REV    13
#define MOTOR1PAUSED 14
#define MOTOR2PAUSED 15
//Define the Command Dword
#define ReturnStatusBit0 0  // these bits are used in both command and status words.
#define ReturnStatusBit1 1
#define ReturnStatusBit2 2
#define UPDATE_PID  3
#define SAVE_TO_EEPROM 4  // This is proposed
#define RESET_STATISTICS 5

// Define PID constant Settings
#define M1SpeedKp 1.5
#define M1SpeedKi 0.0
#define M1SpeedKd 0.0
#define M1PowerKp 1.0
#define M1PowerKi 0.1
#define M1PowerKd 0.0
#define M1MinPos 150
#define M1MaxPos 650
#define M1MinSpeed -200
#define M1MaxSpeed 200
#define M1MinPower -255
#define M1MaxPower 255
#define MotorKick 0
#define AnalogFilter 1
#define AceptableERROR 3.0
#define TimeOut 1000
#define ServoAutoSleepTime 2000


// Other Defined Constants
#define CyclePeriod 10    // Time in mS the calculation are done for both motors
#define DisplayPeriod 100 // Used for the Debugging inforation.
#define I2CbaseAddess 50   // This is the first address on the I2C bus these devise will use

union Data {
  double d;
  unsigned long dw;
  byte b[4];
};

union Command {
  int u;
  byte b[2];
};

struct MotorConfig {
  double SpeedKp;
  double SpeedKi;
  double SpeedKd;
  double PowerKp;
  double PowerKi;
  double PowerKd;
  double MinPos;
  double MaxPos;
  double MinSpeed;
  double MaxSpeed;
  double MinPower;
  double MaxPower;
  unsigned long Timeout;
  double AcceptableError;
  double Kick;
  unsigned long Filter;
};

struct ModuleConfig {
  unsigned long ServoTimeout;
  MotorConfig Motor[2];
};

union CombinedModuleConfig {
  ModuleConfig fig;
  byte B[];
};

// Variables Declaration
union CombinedModuleConfig Con;
double Setpoint1, CurrentPos1, LastPos1, Speed1, SetSpeed1, ReqSpeed1, MotorPower1, Motor1PWM;
double Setpoint2, CurrentPos2, LastPos2, Speed2, SetSpeed2, ReqSpeed2, MotorPower2, Motor2PWM;
unsigned long CommandDWord, StatusDWord, LastNotAceptable1, LastNotAceptable2;
unsigned long CycleTime, LastCycle, LastDisplay, ServoUpdate;
int I2Caddress, ServoPos, CycleCounts, LastCounts;
bool RunCycle, RunDisplay, PWMoutputType, InputType;
double MaxMotor1Speed, MinMotor1Speed, MaxMotor1Power;
double MaxMotor2Speed, MinMotor2Speed, MaxMotor2Power;
bool I2CtestFlag1, I2CtestFlag2;
String DebugInfo;

// Start Library Functions
PID motor1Speed(&CurrentPos1, &SetSpeed1, &Setpoint1, M1SpeedKp, M1SpeedKi, M1SpeedKd, DIRECT);
PID motor1Power(&Speed1, &MotorPower1, &SetSpeed1, M1PowerKp, M1PowerKi, M1PowerKd, DIRECT);
PID motor2Speed(&CurrentPos2, &SetSpeed2, &Setpoint2, M1SpeedKp, M1SpeedKi, M1SpeedKd, DIRECT);
PID motor2Power(&Speed2, &MotorPower2, &SetSpeed2, M1PowerKp, M1PowerKi, M1PowerKd, DIRECT);
Servo RobotServo;

void setup() {
  // Setup logging output
  Serial.begin(115200);
  Serial.println("Startup Servo Driver.");
  Serial.flush();
  DebugInfo.reserve(200);
  
  // Configure the IO Pins
  pinMode(Motor1SetPoinTest, INPUT);
  pinMode(Motor2SetPoinTest, INPUT);
  pinMode(Motor1Position, INPUT);
  pinMode(Motor2Position, INPUT);
  pinMode(Motor1PWM1, OUTPUT);
  pinMode(Motor1PWM2, OUTPUT);
  pinMode(Motor1Dir1, OUTPUT);
  pinMode(Motor1Dir2, OUTPUT);
  pinMode(Motor2PWM1, OUTPUT);
  pinMode(Motor2PWM2, OUTPUT);
  pinMode(Motor2Dir1, OUTPUT);
  pinMode(Motor2Dir2, OUTPUT);
  pinMode(ServoPin, OUTPUT);
  pinMode(i2cAddr0, INPUT_PULLUP);
  pinMode(i2cAddr1, INPUT_PULLUP);
  pinMode(i2cAddr2, INPUT_PULLUP);
  pinMode(OutputType, INPUT_PULLUP);
  pinMode(INPUTtype, INPUT_PULLUP);

  // Setup startup values
  StatusDWord = 0;
  CommandDWord = 0;
  I2CtestFlag1 = false;
  I2CtestFlag2 = false;
  SetDefaults();
  PWMoutputType = digitalRead(OutputType);
  InputType = digitalRead(INPUTtype);
  CurrentPos1 = analogRead(Motor1Position);
  LastPos1 = CurrentPos1;
  Setpoint1 = CurrentPos1;    // for startup default to the current pos as setpoint
  SetSpeed1 = 0;
  CurrentPos2 = analogRead(Motor2Position);
  LastPos2 = CurrentPos2;
  Setpoint2 = CurrentPos2;    // for startup default to the current pos as setpoint
  SetSpeed2 = 0;

  // Setup the I2C bus
  I2Caddress = I2CbaseAddess;
  if (!digitalRead(i2cAddr0)){ 
    I2Caddress +=1;
    bitSet(StatusDWord, 3);
  }
  if (!digitalRead(i2cAddr1)) {
    I2Caddress +=2;
    bitSet(StatusDWord, 4);
  }
  if (!digitalRead(i2cAddr2)) {
    I2Caddress +=4;
    bitSet(StatusDWord, 5);
  }
  Wire.begin(I2Caddress);    // Join the I2C bus with e calculated address
  Wire.onReceive(receiveEvent); // register a receive event
  Wire.onRequest(requestEvent); // register a request event
  Serial.print("Controller Address: ");
  Serial.println(I2Caddress);
  // Configure the PWM settings
  updateConfig();
  // Initilise the timer control loops
  CycleTime = millis();
  LastCycle = CycleTime;
  LastNotAceptable1 = CycleTime;
  LastNotAceptable2 = CycleTime;
  RunCycle = false;
  CycleCounts = 0;
  LastCounts = 0;
}

void loop() {
  // Calculate the period for code processing speed calc and logging data
  CycleTime = millis();
  CycleCounts++;    // This is used to see how many time the loop is run between PID updates
  if (CycleTime > (LastCycle + CyclePeriod)){
    RunCycle = true;
    LastCycle = CycleTime;
    LastCounts = CycleCounts;
    CycleCounts = 0;
  } else RunCycle = false;
  if (CycleTime > (LastDisplay + DisplayPeriod)){
    RunDisplay = false;;
    if (I2CtestFlag1){
      I2CtestFlag1 = false;
      Serial.print("I2C Data Requested. ");
      Serial.println(DebugInfo);
    }
    if (I2CtestFlag2){
      I2CtestFlag2 = false;
      Serial.print("I2C Data Sent. ");
      Serial.println(DebugInfo);
      DebugInfo = "";
    }
    LastDisplay = CycleTime;
  } else RunDisplay = false;
  if (CycleTime > (ServoUpdate + Con.fig.ServoTimeout)){
    RobotServo.detach();
    bitClear(StatusDWord, SERVOENABELED);
  }
  // if setpoint type is debug read analog inputs
  if (!InputType){
    Setpoint1 = map(analogRead(Motor1SetPoinTest), 0, 1023, Con.fig.Motor[0].MinPos, Con.fig.Motor[0].MaxPos);
    Setpoint2 = map(analogRead(Motor2SetPoinTest), 0, 1023, Con.fig.Motor[1].MinPos, Con.fig.Motor[1].MaxPos);
  }
  // Get the input signal from position sensor
  if (Con.fig.Motor[0].Filter > 0){
    CurrentPos1 = ((CurrentPos1 * float(Con.fig.Motor[0].Filter)) + float(analogRead(Motor1Position)))/(float(Con.fig.Motor[0].Filter) + 1.0);
  } else {
    CurrentPos1 = analogRead(Motor1Position);
  }
  if (Con.fig.Motor[1].Filter > 0){
    CurrentPos2 = ((CurrentPos2 * float(Con.fig.Motor[1].Filter)) + float(analogRead(Motor2Position)))/(float(Con.fig.Motor[1].Filter) + 1.0);
  } else {
    CurrentPos2 = analogRead(Motor2Position);
  }
//  CurrentPos2 = analogRead(Motor2Position);
  if (RunCycle){
    Speed1 = (CurrentPos1 - LastPos1)*10;
    LastPos1 = CurrentPos1;
    if (MaxMotor1Speed < Speed1) MaxMotor1Speed = Speed1;
    if (MinMotor1Speed > Speed1) MinMotor1Speed = Speed1;
    Speed2 = (CurrentPos2 - LastPos2)*10;
    LastPos2 = CurrentPos2;
    if (MaxMotor2Speed < Speed2) MaxMotor2Speed = Speed2;
    if (MinMotor2Speed > Speed2) MinMotor2Speed = Speed2;
  }
  if (CurrentPos1-Setpoint1 >Con.fig.Motor[0].AcceptableError or CurrentPos1-Setpoint1< -Con.fig.Motor[0].AcceptableError) {
    LastNotAceptable1 = CycleTime;
    motor1Power.SetMode(AUTOMATIC);
    bitClear(StatusDWord, MOTOR1PAUSED);
  } else {
    if (LastNotAceptable1+TimeOut<CycleTime){
      motor1Power.SetMode(MANUAL);
      bitSet(StatusDWord, MOTOR1PAUSED);
      MotorPower1 = 0;
    }
  }
  if (CurrentPos2-Setpoint2 >Con.fig.Motor[1].AcceptableError or CurrentPos2-Setpoint2< -Con.fig.Motor[1].AcceptableError) {
    LastNotAceptable2 = CycleTime;
    motor2Power.SetMode(AUTOMATIC);
    bitClear(StatusDWord, MOTOR2PAUSED);
  } else {
    if (LastNotAceptable2+TimeOut<CycleTime){
      motor2Power.SetMode(MANUAL);
      bitSet(StatusDWord, MOTOR2PAUSED);
      MotorPower2 = 0;
    }
  }

  // Run the PID Process
  motor1Speed.Compute();
  motor1Power.Compute();
  motor2Speed.Compute();
  motor2Power.Compute();

  // Output the motor control
  if (MotorPower1 > 0){ // This is where we would add deadband control Fwd
    digitalWrite(Motor1Dir1, true);
    digitalWrite(Motor1Dir2, false);
    Motor1PWM = MotorPower1;
    if(Con.fig.Motor[0].Kick > 0) {
      if (Motor1PWM < Con.fig.Motor[0].Kick && Speed1 == 0){
        Motor1PWM = Con.fig.Motor[0].Kick;
      }
    }
    if (PWMoutputType){
      analogWrite(Motor1PWM1, Motor1PWM);
      analogWrite(Motor1PWM2, Motor1PWM);
    } else {
      analogWrite(Motor1PWM1, Motor1PWM);
      analogWrite(Motor1PWM2, 0);
    }
    bitClear(StatusDWord, MOTOR1STOP);
    bitSet(StatusDWord, MOTOR1FWD);
    bitClear(StatusDWord, MOTOR1REV);
  } else if (MotorPower1 < 0){ // This is where we would add deadband control Rev
    digitalWrite(Motor1Dir1, false);
    digitalWrite(Motor1Dir2, true);
    Motor1PWM = MotorPower1 * -1;
    if(Con.fig.Motor[0].Kick > 0) {
      if (Motor1PWM < Con.fig.Motor[0].Kick && Speed1 == 0){
        Motor1PWM = Con.fig.Motor[0].Kick;
      }
    }
    if (PWMoutputType){
      analogWrite(Motor1PWM1, Motor1PWM);
      analogWrite(Motor1PWM2, Motor1PWM);
    } else {
      analogWrite(Motor1PWM1, 0);
      analogWrite(Motor1PWM2, Motor1PWM);
    }
    bitClear(StatusDWord, MOTOR1STOP);
    bitClear(StatusDWord, MOTOR1FWD);
    bitSet(StatusDWord, MOTOR1REV);
  } else {
    digitalWrite(Motor1Dir1, false);
    digitalWrite(Motor1Dir2, false);
    Motor1PWM = 0;
    analogWrite(Motor1PWM1, 0);
    analogWrite(Motor1PWM2, 0);
    bitSet(StatusDWord, MOTOR1STOP);
    bitClear(StatusDWord, MOTOR1FWD);
    bitClear(StatusDWord, MOTOR1REV);
  }
  if(MaxMotor1Power < Motor1PWM){
    MaxMotor1Power = Motor1PWM;
  }
  if (MotorPower2 > 0){ // This is where we would add deadband control Fwd
    digitalWrite(Motor2Dir1, true);
    digitalWrite(Motor2Dir2, false);
    Motor2PWM = MotorPower2;
    if(Con.fig.Motor[1].Kick > 0) {
      if (Motor2PWM < Con.fig.Motor[1].Kick && Speed2 == 0){
        Motor2PWM = Con.fig.Motor[1].Kick;
      }
    }
    if (PWMoutputType){
      analogWrite(Motor2PWM1, Motor2PWM);
      analogWrite(Motor2PWM2, Motor2PWM);
    } else {
      analogWrite(Motor2PWM1, Motor2PWM);
      analogWrite(Motor2PWM2, 0);
    }
    bitClear(StatusDWord, MOTOR2STOP);
    bitSet(StatusDWord, MOTOR2FWD);
    bitClear(StatusDWord, MOTOR2REV);
  } else if (MotorPower2 < 0){ // This is where we would add deadband control Rev
    digitalWrite(Motor2Dir1, false);
    digitalWrite(Motor2Dir2, true);
    Motor2PWM = MotorPower2 * -1;
    if(Con.fig.Motor[1].Kick > 0) {
      if (Motor2PWM < Con.fig.Motor[1].Kick && Speed2 == 0){
        Motor2PWM = Con.fig.Motor[1].Kick;
      }
    }
    if (PWMoutputType){
      analogWrite(Motor2PWM1, Motor2PWM);
      analogWrite(Motor2PWM2, Motor2PWM);
    } else {
      analogWrite(Motor2PWM1, 0);
      analogWrite(Motor2PWM2, Motor2PWM);
    }
    bitClear(StatusDWord, MOTOR2STOP);
    bitClear(StatusDWord, MOTOR2FWD);
    bitSet(StatusDWord, MOTOR2REV);
  } else {
    digitalWrite(Motor2Dir1, false);
    digitalWrite(Motor2Dir2, false);
    Motor2PWM = 0;
    analogWrite(Motor2PWM1, 0);
    analogWrite(Motor2PWM2, 0);
    bitSet(StatusDWord, MOTOR2STOP);
    bitClear(StatusDWord, MOTOR2FWD);
    bitClear(StatusDWord, MOTOR2REV);
  }
  if(MaxMotor2Power < Motor2PWM){
    MaxMotor2Power = Motor2PWM;
  }
  // Print debug log mesages
  if (RunDisplay){
    Serial.print(CurrentPos1);
    Serial.print("\t");
    Serial.print(CurrentPos2);
    Serial.print("\t");
    Serial.print(ServoPos);
    Serial.print("\t");
    Serial.print(Setpoint1);
    Serial.print("\t");
    Serial.print(Setpoint2);
/*    Serial.print(Speed1);
    Serial.print("\t");
    Serial.print(MotorPower1);
    Serial.print("\t");
//    Serial.print(Motor1PWM);
//    Serial.print("\t");
    Serial.print(Motor1MinPos);
    Serial.print("\t");
    Serial.print(Motor1MaxPos);
//    Serial.print("\t");
//    Serial.print(MaxMotor1Speed);
//    Serial.print("\t");
//    Serial.print(MinMotor1Speed);
//    Serial.print("\t");
//    Serial.print(MaxMotor1Power);
*/    Serial.println();
  }
}

//########################################################
//  This section loads the Config Data to EEPROM and compares
//  CRC cecksum with the saved CRC checksum so the data 
//  integerity can be checked.
//########################################################
void SetDefaults() {
  // These are the default values that may be changed via I2C
  // The plan is to save these to EEPROM and reload at startup
  unsigned long SavedCRC, CalcCRC;
   
  EEPROM.get( 0, Con );
  EEPROM.get(sizeof(Con), SavedCRC);
  CalcCRC = Config_crc();
  Serial.print("CRC Numbers, Calc [");
  Serial.print(CalcCRC, HEX);
  Serial.print("] Saved [");
  Serial.print(SavedCRC, HEX);
  Serial.println("]");
  if (SavedCRC != CalcCRC){ 
    Con.fig.ServoTimeout = ServoAutoSleepTime;
    // Motors Settings
    for (int x=0; x<2; x++){
      Con.fig.Motor[x].SpeedKp = M1SpeedKp;
      Con.fig.Motor[x].SpeedKi = M1SpeedKi;
      Con.fig.Motor[x].SpeedKd = M1SpeedKd;
      Con.fig.Motor[x].PowerKp = M1PowerKp;
      Con.fig.Motor[x].PowerKi = M1PowerKi;
      Con.fig.Motor[x].PowerKd = M1PowerKd;
      Con.fig.Motor[x].MinPos = M1MinPos;
      Con.fig.Motor[x].MaxPos = M1MaxPos;
      Con.fig.Motor[x].MinSpeed = M1MinSpeed;
      Con.fig.Motor[x].MaxSpeed = M1MaxSpeed;
      Con.fig.Motor[x].MinPower = M1MinPower;
      Con.fig.Motor[x].MaxPower = M1MaxPower;
      Con.fig.Motor[x].Timeout = TimeOut;
      Con.fig.Motor[x].AcceptableError = AceptableERROR;
      Con.fig.Motor[x].Filter = AnalogFilter;
      Con.fig.Motor[x].Kick = MotorKick;
    }
    EEPROM.put(0, Con);
    CalcCRC = Config_crc();
    EEPROM.put(sizeof(Con), CalcCRC);
  }
}

//########################################################
//  This section saves the Config Data to EEPROM with a
//  CRC cecksum so the data integerity can be checked.
//  After updating the PID Loop controllers.
//########################################################
void updateConfig(){
  unsigned long CalcCRC;
  // Configure the PWM settings
  motor1Speed.SetOutputLimits(Con.fig.Motor[0].MinSpeed, Con.fig.Motor[0].MaxSpeed); // Min < Max
  motor1Power.SetOutputLimits(Con.fig.Motor[0].MinPower, Con.fig.Motor[0].MaxPower); // Min < Max
  motor1Speed.SetTunings(Con.fig.Motor[0].SpeedKp, Con.fig.Motor[0].SpeedKi, Con.fig.Motor[0].SpeedKd); // Kp, Ki, Kd
  motor1Power.SetTunings(Con.fig.Motor[0].PowerKp, Con.fig.Motor[0].PowerKi, Con.fig.Motor[0].PowerKd); // Kp, Ki, Kd
  motor1Speed.SetControllerDirection(DIRECT); // DIRECT or REVERSE
  motor1Power.SetControllerDirection(DIRECT); // DIRECT or REVERSE
  motor1Speed.SetSampleTime(CyclePeriod);  // Must be greater than 0ms
  motor1Power.SetSampleTime(CyclePeriod);  // Must be greater than 0ms
  motor1Speed.SetMode(AUTOMATIC); // AUTOMATIC or MANUAL
  motor1Power.SetMode(AUTOMATIC); // AUTOMATIC or MANUAL
  motor2Speed.SetOutputLimits(Con.fig.Motor[1].MinSpeed, Con.fig.Motor[1].MaxSpeed); // Min < Max
  motor2Power.SetOutputLimits(Con.fig.Motor[1].MinPower, Con.fig.Motor[1].MaxPower); // Min < Max
  motor2Speed.SetTunings(Con.fig.Motor[1].SpeedKp, Con.fig.Motor[1].SpeedKi, Con.fig.Motor[1].SpeedKd); // Kp, Ki, Kd
  motor2Power.SetTunings(Con.fig.Motor[1].PowerKp, Con.fig.Motor[1].PowerKi, Con.fig.Motor[1].PowerKd); // Kp, Ki, Kd
  motor2Speed.SetControllerDirection(DIRECT); // DIRECT or REVERSE
  motor2Power.SetControllerDirection(DIRECT); // DIRECT or REVERSE
  motor2Speed.SetSampleTime(CyclePeriod);  // Must be greater than 0ms
  motor2Power.SetSampleTime(CyclePeriod);  // Must be greater than 0ms
  motor2Speed.SetMode(AUTOMATIC); // AUTOMATIC or MANUAL
  motor2Power.SetMode(AUTOMATIC); // AUTOMATIC or MANUAL
  EEPROM.put(0, Con);
  CalcCRC = Config_crc();
  EEPROM.put(sizeof(Con), CalcCRC);
  bitClear(CommandDWord, UPDATE_PID);
}

//########################################################
//  This section receives a command from the I2C Bus master
//  and processes the data supplied.
//########################################################
void receiveEvent(int howMany){
  union Command cmd;
  union Data data;
  cmd.b[0] = Wire.read();
  cmd.b[1] = Wire.read();
  data.b[0] = Wire.read();
  data.b[1] = Wire.read();
  data.b[2] = Wire.read();
  data.b[3] = Wire.read();
  I2CtestFlag2 = true;
  DebugInfo = "Command [";
  DebugInfo.concat(cmd.u);
  DebugInfo.concat("] Data [");
  DebugInfo.concat(data.dw);
  DebugInfo.concat("]");
  switch (cmd.u) {
    case 0:   // Invalid Command.
      break;
    case 1:
      Setpoint1 = data.d;
      if(Setpoint1 < Con.fig.Motor[0].MinPos) Setpoint1 = Con.fig.Motor[0].MinPos;
      if(Setpoint1 > Con.fig.Motor[0].MaxPos) Setpoint1 = Con.fig.Motor[0].MaxPos;
      break;
    case 2:
      Setpoint2 = data.d;
      if(Setpoint2 < Con.fig.Motor[1].MinPos) Setpoint2 = Con.fig.Motor[1].MinPos;
      if(Setpoint2 > Con.fig.Motor[1].MaxPos) Setpoint2 = Con.fig.Motor[1].MaxPos;
      break;
    case 3:
      ServoPos = data.d;
      RobotServo.attach(ServoPin);
      RobotServo.write(ServoPos);
      bitSet(StatusDWord, SERVOENABELED);
      ServoUpdate = CycleTime;
      break;
    case 4:
      Con.fig.ServoTimeout = data.dw;
      break;
    case 5:
      CommandDWord = data.dw;
      StatusDWord &= ~7;  // Clears the 3 bit for the status packet to be returned 
      StatusDWord |= (CommandDWord & 7); // Set the correct packet from the last command
      break;
    case 6:   // Reset Stats
      MaxMotor1Power = 0;
      MinMotor1Speed = 0;
      MaxMotor1Speed = 0;
      MaxMotor2Power = 0;
      MinMotor2Speed = 0;
      MaxMotor2Speed = 0;
      break;
    case 7:   // Update and Save Config
      updateConfig();
      break;
    case 8:   // Reserved
      break;
      
    case 10:
      Con.fig.Motor[0].MinPos = data.d;
      break;
    case 11:
      Con.fig.Motor[0].MaxPos = data.d;
      break;
    case 12:
      Con.fig.Motor[0].MinSpeed = data.d;
      break;
    case 13:
      Con.fig.Motor[0].MaxSpeed = data.d;
      break;
    case 14:
      Con.fig.Motor[0].MinPower = data.d;
      break;
    case 15:
      Con.fig.Motor[0].MaxPower = data.d;
      break;
    case 16:
      Con.fig.Motor[0].Timeout = data.dw;
      break;
    case 17:
      Con.fig.Motor[0].AcceptableError = data.d;
      break;
    case 18:
      Con.fig.Motor[0].Kick = data.d;
      break;
    case 19:
      Con.fig.Motor[0].Filter = data.dw;
      break;
    case 20:
      Con.fig.Motor[0].SpeedKp = data.d;
      break;
    case 21:
      Con.fig.Motor[0].SpeedKi = data.d;
      break;
    case 22:
      Con.fig.Motor[0].SpeedKd = data.d;
      break;
    case 23:
      Con.fig.Motor[0].PowerKp = data.d;
      break;
    case 24:
      Con.fig.Motor[0].PowerKi = data.d;
      break;
    case 25:
      Con.fig.Motor[0].PowerKd = data.d;
      break;
      
    case 30:
      Con.fig.Motor[1].MinPos = data.d;
      break;
    case 31:
      Con.fig.Motor[1].MaxPos = data.d;
      break;
    case 32:
      Con.fig.Motor[1].MinSpeed = data.d;
      break;
    case 33:
      Con.fig.Motor[1].MaxSpeed = data.d;
      break;
    case 34:
      Con.fig.Motor[1].MinPower = data.d;
      break;
    case 35:
      Con.fig.Motor[1].MaxPower = data.d;
      break;
    case 36:
      Con.fig.Motor[1].Timeout = data.dw;
      break;
    case 37:
      Con.fig.Motor[1].AcceptableError = data.d;
      break;
    case 38:
      Con.fig.Motor[1].Kick = data.d;
      break;
    case 39:
      Con.fig.Motor[1].Filter = data.dw;
      break;
    case 40:
      Con.fig.Motor[1].SpeedKp = data.d;
      break;
    case 41:
      Con.fig.Motor[1].SpeedKi = data.d;
      break;
    case 42:
      Con.fig.Motor[1].SpeedKd = data.d;
      break;
    case 43:
      Con.fig.Motor[1].PowerKp = data.d;
      break;
    case 44:
      Con.fig.Motor[1].PowerKi = data.d;
      break;
    case 45:
      Con.fig.Motor[1].PowerKd = data.d;
      break;
    
  }
}

//########################################################
//  This section process a request for data from the I2C
//  bus Master controller.
//########################################################
void requestEvent() {
//  union Data data;
//  StatusDWord &= !7;  // Clears the 3 bit for the status packet to be returned 
//  StatusDWord |= (CommandDWord & 3); // Set the correct packet from the last command
//  data.dw = StatusDWord;
  I2CtestFlag1 = true;
  DebugInfo = "Sending Packet [";
  DebugInfo.concat(StatusDWord & 7);
  DebugInfo.concat("] Status [");
  DebugInfo.concat(StatusDWord);
  DebugInfo.concat("]");

  switch (StatusDWord & 7) {
    case 0: // Base Information
      writeDword(StatusDWord);
      writeDouble(CurrentPos1);
      writeDouble(CurrentPos2);
      writeDouble(Speed1);
      writeDouble(Speed2);
      writeDouble(MotorPower1);
      writeDouble(MotorPower2);
      break;
    case 1: // Setpoints Information
      writeDword(StatusDWord);
      writeDword(CommandDWord);
      writeDouble(ServoPos);
      writeDouble(Setpoint1);
      writeDouble(Setpoint2);
      writeDouble(0.0);   // Reserved for future use
      writeDouble(0.0);   // Reserved for future use
      break;
    case 2: // Setpoint Motor1 Information
      writeDword(StatusDWord);
      writeDouble(Con.fig.Motor[0].MinPos);
      writeDouble(Con.fig.Motor[0].MaxPos);
      writeDouble(Con.fig.Motor[0].MinSpeed);
      writeDouble(Con.fig.Motor[0].MaxSpeed);
      writeDouble(Con.fig.Motor[0].MinPower);
      writeDouble(Con.fig.Motor[0].MaxPower);
      break;
    case 3: // Setpoint Motor2 Information
      writeDword(StatusDWord);
      writeDouble(Con.fig.Motor[1].MinPos);
      writeDouble(Con.fig.Motor[1].MaxPos);
      writeDouble(Con.fig.Motor[1].MinSpeed);
      writeDouble(Con.fig.Motor[1].MaxSpeed);
      writeDouble(Con.fig.Motor[1].MinPower);
      writeDouble(Con.fig.Motor[1].MaxPower);
      break;
    case 4: // PID's Motor1 Information
      writeDword(StatusDWord);
      writeDouble(Con.fig.Motor[0].SpeedKp);
      writeDouble(Con.fig.Motor[0].SpeedKi);
      writeDouble(Con.fig.Motor[0].SpeedKd);
      writeDouble(Con.fig.Motor[0].PowerKp);
      writeDouble(Con.fig.Motor[0].PowerKi);
      writeDouble(Con.fig.Motor[0].PowerKd);
      break;
    case 5: // PID's Motor2 Information
      writeDword(StatusDWord);
      writeDouble(Con.fig.Motor[1].SpeedKp);
      writeDouble(Con.fig.Motor[1].SpeedKi);
      writeDouble(Con.fig.Motor[1].SpeedKd);
      writeDouble(Con.fig.Motor[1].PowerKp);
      writeDouble(Con.fig.Motor[1].PowerKi);
      writeDouble(Con.fig.Motor[1].PowerKd);
      break;
    case 6: // Motor Power/Speed Statistic
      writeDword(StatusDWord);
      writeDouble(MaxMotor1Power);
      writeDouble(MaxMotor2Power);
      writeDouble(MinMotor1Speed);
      writeDouble(MinMotor2Speed);
      writeDouble(MaxMotor1Speed);
      writeDouble(MaxMotor2Speed);
      break;
    case 7: // Reserved
      writeDword(StatusDWord);
      writeDouble(Con.fig.Motor[0].Kick);
      writeDouble(Con.fig.Motor[0].Filter);
      writeDouble(Con.fig.Motor[1].Kick);
      writeDouble(Con.fig.Motor[1].Filter);
      writeDouble(0.0);
      writeDouble(0.0);
      break;
  }
}

//########################################################
//  This section takes the unsigned 32 bit (dword) value and
//  writes it out to the I2C bus
//########################################################
void writeDword(unsigned long dword){
  union Data data;
  data.dw = dword;
  Wire.write(data.b[0]);
  Wire.write(data.b[1]);
  Wire.write(data.b[2]);
  Wire.write(data.b[3]);
}

//########################################################
//  This section takes the floating piont double value and
//  writes it out to the I2C bus
//########################################################
void writeDouble(double value) {
  union Data data;
  data.d = value;
  Wire.write(data.b[0]);
  Wire.write(data.b[1]);
  Wire.write(data.b[2]);
  Wire.write(data.b[3]);
}

//########################################################
//  This section calculates the CRC for the Config Data
//  straucture and returns it as an unsigned long or dword
//########################################################
unsigned long Config_crc(void) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (int index = 0; index < sizeof(Con); ++index) {
    crc = crc_table[(crc ^ Con.B[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (Con.B[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

