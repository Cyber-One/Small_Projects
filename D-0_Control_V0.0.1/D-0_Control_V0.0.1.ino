/*
 * D-0 control system V 0.0.1
 *  Use: 
 *  FS-iA6B receiver to Serial1.
 *  Mini MP3 Player
 *  Adafruit PCA9685 I2C 16ch servo driver
 *  
 *  Created by Ray Edgley
 */

#include <FlySkyiBus.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <DFMiniMp3.h>
#include "MP3.h"

// software serial: RX = digital pin 8, TX = digital pin 9
// on the Mega, use other pins instead, since 8 and 9 don't work on the Mega
// on the mega, your better off using hardware serial, it has 4 of them.
SoftwareSerial MP3Serial(8, 9);

// Both the software serial FlySky iBus library and theHardware version work the same.
FlySkyiBus iBus(10, 10);  //rx, tx
// The FlySky iBus returns the output to servos in uS pulse duration
#define RX_MIN 1000
#define RX_MAX 2000
/*
 *  Channel 0 =  Left / Right.
 *  Channel 1 =  Forward / Reverse.
 *  Channel 2 =  Nod Bar.
 *  Channel 3 =  Head Rotate.
 *  Channel 4 =  Head Tilt.
 *  Channel 5 =  Main Arm.
 *  Channel 6 =  Select  Track.
 *  Channel 7 =  Enable Servos
 *  Channel 8 =  
 *  Channel 9 =  Button / Play Track.
 */
 // Lets setup some servo limits.  The range is 1000uS to 2000uS
 #define NOD_BAR_MIN    1000
 #define NOD_BAR_MAX    2000
 #define HEAD_ROT_MIN   1000
 #define HEAD_ROT_MAX   2000
 #define HEAD_TILT_MIN  1250
 #define HEAD_TILT_MAX  1750
 #define MAIN_ARM_MIN   1000
 #define MAIN_ARM_MAX   2000

 // The Adafruit_PWMServoDriver module is a full PWM module.
 // In this  application we are using the last 6 outputs as Servo outputs
 // however thereis nothing stopping us from using the rest for full PWM LED drivers.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define PWM_STEPS   4096
#define FREQUENCY   50                              // 50Hz has a 20mS period.
#define PWM_MIN     (PWM_STEPS*(FREQUENCY/10))/100  // This will give us the count in the PWM for 1000uS.
#define PWM_MAX     PWM_MIN*2                       // This will give us the count in the PWM for 2000uS.
#define PWM_OFFSET  12                              // Account for an error in the clock on the PCA9685

DFMiniMp3<SoftwareSerial, Mp3Notify> mp3(MP3Serial);

unsigned int Servo_Last[10];                    // Used to hold the output value of each of the PWM channels.
unsigned int RC_Last[10];                       // The value of the last received data for each channel from the FlySky receiver.
bool RC_Changed[10];
int MP3track;

void setup() {
  // The hardware Serial is used for debugging and programming.
  Serial.begin(115200);
  Serial.println("Starup of the D-0 Robot control system");
  Serial.print("PWM_Min (");
  Serial.print(PWM_MIN, DEC);
  Serial.print(")\t PWM_MAX (");
  Serial.print(PWM_MAX, DEC);
  Serial.print(")\t FREQUENCY (");
  Serial.print(FREQUENCY, DEC);
  Serial.println(")");
  Serial.print("RAW_PWM = ");
  Serial.println(PWM_STEPS*FREQUENCY, DEC);
  Serial.flush();

  // The wire library is the I2C bus controller.
  // This is required to be running before we start the Adafruit_PWMServoDriver.
  Wire.begin();
  Serial.println("I2C bus started");
  Serial.flush();

  // I found through trial and error, you need to start the MP3 player module before the
  // FlySky iBus moduleor there is a conflict thestops the iBus module.
  MP3track = 1;
  mp3.begin();
  Serial.println("MP3 Started.");
  Serial.flush();

  // Start the FlySky iBus module after any other software serial devices.
  iBus.begin(115200);
  Serial.println("Ibus started.");
  Serial.flush();

  // This is the Adafruit_PWMServoDriver module.
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  Serial.println("I2C PCA9685 Controller Started.");
  Serial.flush();
  
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  Serial.println("PCA9685 enable pin configured and set to high"); 
  Serial.flush();

  mp3.playMp3FolderTrack(MP3track);  // sd:/mp3/0001.mp3

}

void loop() {
  // put your main code here, to run repeatedly:
  bool RCchanged;
  
  RCchanged = false;
  iBus.read_serial();
  for (int x=0; x<10; x++){
    if(RC_Last[x] != iBus.get_channel(x)) {
      RC_Last[x] = iBus.get_channel(x);
      RCchanged = true;
      RC_Changed[x] = true;
    } else RC_Changed[x] = false;
  }
  if(RC_Changed[0] || RC_Changed[1]) {    // drive system controls
    int LeftRight = map(RC_Last[0], RX_MIN, RX_MAX, -100, 100);
    int FwdRev    = map(RC_Last[1], RX_MIN, RX_MAX, 100, -100);
    Servo_Last[0] =  map(constrain(FwdRev+LeftRight, -100, 100), -100, 100, PWM_MIN, PWM_MAX)+PWM_OFFSET;
    Servo_Last[1] =  map(constrain(FwdRev-LeftRight, -100, 100), -100, 100, PWM_MAX, PWM_MIN)+PWM_OFFSET;
    pwm.setPWM(15, 0, Servo_Last[0]); // Left Wheel.
    pwm.setPWM(14, 0, Servo_Last[1]); // Right Wheel.
  }
  /*  
   *  There are a couple of ways of resricting the range of motion from a servo.
   *  We can contrain the input from the RC to the limits we set above and be done with it.
   *  but that would reduce the level of control we have.
   *  The other way is to map the full input range down to the smaller range, this is what we will do.
   */
  
  if(RC_Changed[2]) {                     // Nod Bar Controls
    Servo_Last[2] = map(map(RC_Last[2], RX_MIN, RX_MAX, NOD_BAR_MIN, NOD_BAR_MAX), RX_MIN, RX_MAX, PWM_MIN, PWM_MAX)+PWM_OFFSET;
    pwm.setPWM(12, 0, Servo_Last[2]); // Nod Bar.    
  }
  if(RC_Changed[3]) {                     // Head Rotate Controls
    Servo_Last[3] = map(map(RC_Last[3], RX_MIN, RX_MAX, HEAD_ROT_MIN, HEAD_ROT_MAX), RX_MIN, RX_MAX, PWM_MIN, PWM_MAX)+PWM_OFFSET;
    pwm.setPWM(10, 0, Servo_Last[3]); // Head Rotate.  
  }
  if(RC_Changed[4]) {                     // Head Tilt Controls
    Servo_Last[4] = map(map(RC_Last[4], RX_MIN, RX_MAX, HEAD_TILT_MIN, HEAD_TILT_MAX), RX_MIN, RX_MAX, PWM_MIN, PWM_MAX)+PWM_OFFSET;
     pwm.setPWM(11, 0, Servo_Last[4]); // Head Tilt.    
  }
  if(RC_Changed[5]) {                     // Main Arm Controls
    Servo_Last[5] = map(map(RC_Last[5], RX_MIN, RX_MAX, MAIN_ARM_MIN, MAIN_ARM_MAX), RX_MIN, RX_MAX, PWM_MIN, PWM_MAX)+PWM_OFFSET;
    pwm.setPWM(13, 0, Servo_Last[5]); // Main Arm.    
  }
  if(RC_Changed[7]) {                     // Servo Enable Control
    if (RC_Last[7] <  1500) {             // When the switch is up, its will be at 1000, down will be 2000.
      digitalWrite(2, HIGH);
    } else {
      digitalWrite(2, LOW);
    }
  }
  if (RC_Changed[9]){                     // Button on back.
    if (RC_Last[9] > 1500) {              // When the button is released it will be at 1000, when pressed at 2000.
      MP3track = modeSelector(RC_Last[6]);  // Mode Switch Posistions.
      RC_Changed[9] = false;
      mp3.playMp3FolderTrack(MP3track);
    }
  }
  if (RCchanged){
    PrintRCvalues();
  }
  
}

int modeSelector(int x) {
  switch (x) {
    case 1000:
      return 1;
      break;
    case 1100:
      return 2;
      break;
    case 1300:
      return 3;
      break;
    case 1400:
      return 4;
      break;
    case 1500:
      return 5;
      break;
    case 1600:
      return 6;
      break;
    case 1700:
      return 7;
      break;
    case 1800:
      return 8;
      break;
    case 1900:
      return 9;
      break;
    default:
      return 1;
  }
}

void PrintRCvalues(){
  for (int y=0; y<10; y++){
    Serial.print(RC_Last[y], DEC);
    Serial.print(" (");
    Serial.print(Servo_Last[y], DEC);
    Serial.print(")\t");
  }
  Serial.println();
  Serial.flush();
}
