
Dual Motor Controller

The dual motor controller is based around the Arduino Nano using PWM output to control up to two DC motors and one standard type servo.

NOTE: the servo will derive its power from the I2C bus, so make sure there is sufficient power available on the bus to power all the servos connected, in this case a total of 8 servos assuming you have 8 dual motor controller units installed.

Proportional Integral Derivative (PID) loops is the term given to a type of control loop normally associated with power control.  PID is used extensively throughout industry for the control of pressure, temperature, flow rate and level control.
PID loops typically have a set-point, an input and the output.  The loop comprises 3 separate formula to control an output that adjust the input to be closer to the set-point based on the error between the set-point and the input.
The three formula can be expressed in simple mathematical terms as the following:

- Proportional. Output = Error * Kp
- Integral. Output = Integral * Ki, Integral = Integral + Error * DeltaTime
- Derivative. Output =Derivative * Kd, Derivative = (PrevousError - Error) / DeltaTime

The three outputs are summed together to give a Control Output.
Kp, Ki and Kd are the PID tuning constants.
DeltaTime is the change in time since the last time the loop ran.

For most applications the PID loop on its own works reasonable well, however in motor positioning systems as we are implementing, the integral component winds up over longer travel distances and results in overshoot.  A proportional only system is also not ideal, particularly for smaller changes.  If the Kp value is set high enough for the smaller changes to work, the loop overshoots on the larger changes.  In this situation the Integral component would normally be used, however when set high enough to have an effect at smaller distances, the wind-up becomes problematic for the larger distances.

Enter the the Dual PID Motor controller. 

The system works by using a PID to set a speed request based on the error derived from the set-point position and the current position, and a second PID to control the power sent to the motor to achieve the required speed.  
The first PID control is normally configured as a proportional only loop with a maximum speed setting based on something the motor can achieve without any trouble.
The second PID loop is used to set the power output to the motor via Pulse Width Modulation (PWM) based on the error of current speed vs request speed.  
As the max speed that can be set by the first PID loop should be well within the motors capability, the motor should achieve the speed without too much Integral error, and also produce a breaking action if required when the requested speed falls below the current speed.
This dual PID operation gives us both fast motor speed even at small distance changes, and very little, if any, over shoot that can result in damage.

Even with the dual PID operation, when errors are very small, the speed request from the first loop can be very small, resulting in small power outputs to the motor.  In these cases static friction which is normally greater than non-static friction in the mechanical system will resist in allowing the motor to move until the integral component builds to a level that is great enough for the motor to overcome the static friction.  This can result in the motor overshooting the set-point by a small amount and the whole process repeating its self again.   For this reason a feature known as the Kick has been added.
When set to a value greater then 0 (the default setting) and while the motor speed is at 0 and when the speed request is greater than 0, the program will over ride the PID power output with the Kick value.  As soon as speed is detected, the output power is set back to the PID output level.  This gives the motor a little kick as it were to get the motor moving, breaking the static friction.  In my setup for the knee joint, I know the motor does not start to turn until the power reaches at least 12 to 13 in a range of 0 - 255.  This range is the PWM output range as dictated by the micro-controller.

In most systems the PID loop is run on a reasonably low rate, some heating applications can have the loop cycle as low as once per second, most however run at the more reasonable rate of once every 100 mS.  In motor positioning systems this can be too slow.  In the Dual PID motor controller, we run both the PID loops at 10 mS intervals so as to get the fastest possible response times.

https://youtu.be/3vIJzb_Ovzk?t=29s
As can be seen from the above video, the speed and accuracy of the control system is quite impressive.  The long delays between operations was for two reasons:

1. To allow you to see the position was stable.
2. It is hard to record video with a tablet and type on the keyboard of the computer 🙂 
It should be noted that apart for the position set-point limits, the motor controller was operating on default setting with the Kick function turned off.  There was also a second controller connected to the I2C bus that was also being scanned.

On power up the motor controller sets the set-point for each of the motors to match that of the current position.  This is to prevent unexpected motor movement during system startup before the bus-master can give the motor controller instructions.  It is strongly advised that the bus-master not issue and commands until after it has obtained a full system status from the motor controller.  Also during startup, the PID and set-point limits are also loaded from EEPROM and applied.  If you make any changes to the set-point limits or the PID loop settings, then it is strongly advised to execute the save command to save the settings to EEPROM.  

Note: PID setting will not be applied until the save command is issued.

The address of the motor controller is set by connecting a combination of D2, D4 and D12 to ground.  This forms a bit field to select the address with D2 being bit 0, D4 as bit 1 and D12 as bit 2.  With this combination a possible of 8 different address can be selected. The base address is 50 dec.
A2 is used to select the output type, if you require two PWM signals per motor, one for forward the other reverse, then connect A2 to ground.

Connection details

| Pin | Output Mode 0                                           | Output Mode 1                                           |
| --- | ------------------------------------------------------- | ------------------------------------------------------- |
| D0  | USB RX                                                  | USB RX                                                  |
| D1  | USB TX                                                  | USB TX                                                  |
| D2  | I2C Address set bit 0                                   | I2C Address set bit 0                                   |
| D3  | PWM Motor 2                                             | PWM Motor 2 FWD                                         |
| D4  | I2C Address set bit 1                                   | I2C Address set bit 1                                   |
| D5  | PWM Motor 1                                             | PWM Motor 1 FWD                                         |
| D6  | PWM Motor 1                                             | PWM Motor 1 REV                                         |
| D7  | Motor 1 Direction FWD                                   | Motor 1 Direction FWD                                   |
| D8  | Motor 1 Direction REV                                   | Motor 1 Direction REV                                   |
| D9  | Motor 2 Direction FWD                                   | Motor 2 Direction FWD                                   |
| D10 | Motor 2 Direction REV                                   | Motor 2 Direction REV                                   |
| D11 | PWM Motor 2                                             | PWM Motor 2 REV                                         |
| D12 | I2C Address Set bit 2                                   | I2C Address Set bit 2                                   |
| D13 | Servo Output                                            | Servo Output                                            |
| A0  | Motor 1 Position Feed Back                              | Motor 1 Position Feed Back                              |
| A1  | Motor 2 Position Feed Back                              | Motor 2 Position Feed Back                              |
| A2  | Output Mode 0 select open circuit                       | Output Mode 1 select Connected to ground                |
| A3  | Test Input, Connect to ground to use A6 A7 as setpoints | Test Input, Connect to ground to use A6 A7 as setpoints |
| A4  | I2C SDA                                                 | I2C SDA                                                 |
| A5  | I2C CLK                                                 | I2C CLK                                                 |
| A6  | Motor 1 Analog test set point                           | Motor 1 Analog test set point                           |
| A7  | Motor 2 Analog test set point                           | Motor 2 Analog test set point                           |


This controller uses an I2C command 6 bytes long, and returns 28 bytes of data when polled.
The 6 byte command is actually two values, a word or 2 bytes which forms the command WORD and 4 bytes that is the DWORD  data.
The Command word tells the motor controller what to do with the data, most of the time the data will be a float cast to a DWORD.

List of Commands

| CMD | Name                          | Data Type | Default Value                      | Description                                                                                                                                                                                                                                                                                                                                                                                                                   |
| --- | ----------------------------- | --------- | ---------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0   | Not Used                      |           |                                    | Not used and is ignored                                                                                                                                                                                                                                                                                                                                                                                                       |
| 1   | Set Point 1                   | Float     | Motor 1 CurrentPos                 | The data is used to update the first motor requested position and has a range of Motor1MinPos - Motor1MaxPos                                                                                                                                                                                                                                                                                                                  |
| 2   | Set Point 2                   | Float     | Motor 2 CurrentPos                 | The data is used to update the Second motor requested position and has a range of Motor2MinPos - Motor2MaxPos                                                                                                                                                                                                                                                                                                                 |
| 3   | Servo SetPos                  | Float     | Not Defined. Remains off until set | The data is used to set the servo position. Note: this is rounded to an integer and has a range of 0 - 180                                                                                                                                                                                                                                                                                                                    |
| 4   | Servo Timeout                 | Dword     | 2000 mS                            | The data is used to set how long after the last Servo SetPos command before the servo is turned off.  This allows for power saving and helps to prevent servo burnout.                                                                                                                                                                                                                                                        |
| 5   | Set Page Returned             | Dword     | 0                                  | The data has a range of 0 - 7. There are up to 8 pages of data that can be returned, this selects which page will be returned on the next data request.  See returned data for more detail.                                                                                                                                                                                                                                   |
| 6   | Reset Statistics              |           |                                    | The data is ignored.  The Motor controller records the maximum power sent to each motor and the maximum speed forward and revers for each motor. This command set the statistics back to zero.  Useful when setting the PID loops.                                                                                                                                                                                            |
| 7   | Update & Save Config          |           |                                    | The data is ignored.  While set-point limits are applied immediately, PID values are not updated until this command is executed. Also data for the set-points is not saved until after this command. Once saved, the current config will be reloaded on power up.                                                                                                                                                             |
| 8   | Reserved                      |           |                                    |                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 9   | Reserved                      |           |                                    |                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 10  | Set Motor 1 Min Position      | Float     | 150                                | This sets the minimum position you can set the set point to. If you try and set the set-point to a value less than this, it will be set to this value.   Range 0 - 1022                                                                                                                                                                                                                                                       |
| 11  | Set Motor 1 Max Position      | Float     | 650                                | This sets the maximum position you can set the set point to. If you try and set the set-point to a value greater than this, it will be set to this value.  Range 1 - 1023                                                                                                                                                                                                                                                     |
| 12  | Set Motor 1 Max Reverse Speed | Float     | -200                               | The controller will not try and run the motor any faster than this in the reverse direction.                                                                                                                                                                                                                                                                                                                                  |
| 13  | Set Motor 1 Max Forward Speed | Float     | 200                                | The controller will not try and run the motor any faster than this in the forward direction.                                                                                                                                                                                                                                                                                                                                  |
| 14  | Set Motor 1 Max Reverse Power | Float     | -255                               | The controller will not exceed this level of power in the reverse direction.
Range: -255 - 0                                                                                                                                                                                                                                                                                                                                  |
| 15  | Set Motor 1 Max Forward Power | Float     | 255                                | The controller will not exceed this level of power in the forward direction.
Range: 0 - 255                                                                                                                                                                                                                                                                                                                                   |
| 16  | Set Motor 1 Timeout           | Dword     | 1000 mS                            | Once the motor falls within the acceptable range the time out timer starts. once the timer is exceeded, the power to the motor will be shut down to save energy and to prevent the motor or motor driver from over heating.  The value is in mili seconds                                                                                                                                                                     |
| 17  | Set Motor 1 Acceptable Error  | Float     | 3.0                                | This is the allowable error in positioning the motor, once within this range, the motor will continue to try for the exact position until the timeout has occurred as set above.                                                                                                                                                                                                                                              |
| 18  | Motor 1 Kick                  | Float     | 0                                  | The default value is 0.  When Kick is set to a value greater than 0 and the current speed = 0 and  the requested power if greater than 0 and less than Kick, then the Kick value is applied to the output power, over riding the PID power request.  On the next loop, if the speed is greater than 0 then Kick is not applied and the normal PID output is.  This can be useful when high static friction loads are present. |
| 19  | Motor 1 Input Filter          | Dword     | 1                                  | A setting of 0 turns it off.  This function can smooth out a noisy analog input allowing the motor to go to sleep.                                                                                                                                                                                                                                                                                                            |
| 20  | Motor 1 Speed PID             | Float     | 1.5                                | This sets the Kp (proportional) value of the Speed PID for Motor 1.
Note: this value is not used until the data s saved with command 07.                                                                                                                                                                                                                                                                                      |
| 21  | Motor 1 Speed PID             | Float     | 0.0                                | This sets the Ki (integral) value of the Speed PID for Motor 1.
Note: this value is not used until the data s saved with command 07.                                                                                                                                                                                                                                                                                          |
| 22  | Motor 1 Speed PID             | Float     | 0.0                                | This sets the Kd (derivative) value of the Speed PID for Motor 1.
Note: this value is not used until the data s saved with command 07.                                                                                                                                                                                                                                                                                        |
| 23  | Motor 1 Power PID             | Float     | 1.0                                | This sets the Kp (proportional) value of the Power PID for Motor 1.
Note: this value is not used until the data s saved with command 07.                                                                                                                                                                                                                                                                                      |
| 24  | Motor 1 Power PID             | Float     | 0.1                                | This sets the Ki (integral) value of the Power PID for Motor 1.
Note: this value is not used until the data s saved with command 07.                                                                                                                                                                                                                                                                                          |
| 25  | Motor 1 Power PID             | Float     | 0.0                                | This sets the Kd (derivative) value of the Power PID for Motor 1.
Note: this value is not used until the data s saved with command 07.                                                                                                                                                                                                                                                                                        |
| 30  | Set Motor 2 Min Position      | Float     | 150                                | This set the minimum position you can set the set point to. If you try and set the set-point to a value less than this, it will be set to this value.                                                                                                                                                                                                                                                                         |
| 31  | Set Motor 2 Max Position      | Float     | 650                                | This set the maximum position you can set the set point to. If you try and set the set-point to a value greater than this, it will be set to this value.                                                                                                                                                                                                                                                                      |
| 32  | Set Motor 2 Max Reverse Speed | Float     | -200                               | The controller will not try and run the motor any faster than this in the reverse direction.                                                                                                                                                                                                                                                                                                                                  |
| 33  | Set Motor 2 Max Forward Speed | Float     | 200                                | The controller will not try and run the motor any faster than this in the forward direction.                                                                                                                                                                                                                                                                                                                                  |
| 34  | Set Motor 2 Max Reverse Power | Float     | -255                               | The controller will not exceed this level of power in the reverse direction.                                                                                                                                                                                                                                                                                                                                                  |
| 35  | Set Motor 2 Max Forward Power | Float     | 255                                | The controller will not exceed this level of power in the forward direction.                                                                                                                                                                                                                                                                                                                                                  |
| 36  | Set Motor 2 Timeout           | Dword     | 1000 mS                            | Once the motor falls within the acceptable range the time out timer starts. once the timer is exceeded, the power to the motor will be shut down to save energy and to prevent the motor or motor driver from over heating.  The value is in mili seconds                                                                                                                                                                     |
| 37  | Set Motor 2 Acceptable Error  | Float     | 3.0                                | This is the allowable error in positioning the motor, once within this range, the motor will continue to try for the exact position until the timeout has occurred as set above.                                                                                                                                                                                                                                              |
| 38  | Motor 2 Kick                  | Float     | 0                                  | The default value is 0.  When Kick is set to a value greater than 0 and the current speed = 0 and  the requested power if greater than 0 and less than Kick, then the Kick value is applied to the output power, over riding the PID power request.  On the next loop, if the speed is greater than 0 then Kick is not applied and the normal PID output is.  This can be useful when high static friction loads are present. |
| 39  | Motor 2 Input Filter          | Dword     | 1                                  | The default value for the filter is 1, a setting of 0 turns it off.  This function can smooth out noisy analog input allowing the motor to go to sleep.                                                                                                                                                                                                                                                                       |
| 40  | Motor 2 Speed PID             | Float     | 1.5                                | This sets the Kp (proportional) value of the Speed PID for Motor 2.
Note: this value is not used until the data s saved with command 07.                                                                                                                                                                                                                                                                                      |
| 41  | Motor 2 Speed PID             | Float     | 0.0                                | This sets the Ki (integral) value of the Speed PID for Motor 2.
Note: this value is not used until the data s saved with command 07.                                                                                                                                                                                                                                                                                          |
| 42  | Motor 2 Speed PID             | Float     | 0.0                                | This sets the Kd (derivative) value of the Speed PID for Motor 2.
Note: this value is not used until the data s saved with command 07.                                                                                                                                                                                                                                                                                        |
| 43  | Motor 2 Power PID             | Float     | 1.0                                | This sets the Kp (proportional) value of the Power PID for Motor 2.
Note: this value is not used until the data s saved with command 07.                                                                                                                                                                                                                                                                                      |
| 44  | Motor 2 Power PID             | Float     | 0.1                                | This sets the Ki (integral) value of the Power PID for Motor 2.
Note: this value is not used until the data s saved with command 07.                                                                                                                                                                                                                                                                                          |
| 45  | Motor 2 Power PID             | Float     | 0.0                                | This sets the Kd (derivative) value of the Power PID for Motor 2.
Note: this value is not used until the data s saved with command 07.                                                                                                                                                                                                                                                                                        |


Returned Data
The data is returned as a 28 byte packet divided into 7 dwords.
The dwords are either used as dwords, a bit field or cast into a floating point number.
As the packet or page can be selected by the controller, it is suggested the only the first two be aggressively scanned with the remainder being scanned at intervals as great as 5 seconds apart.

| Page | DWORD | Data type | Description                         |
| ---- | ----- | --------- | ----------------------------------- |
| 0    | 0     | dword     | Device Status as a bit field        |
|      | 1     | float     | Current position of Motor 1         |
|      | 2     | float     | Current position of Motor 2         |
|      | 3     | float     | Current speed of Motor 1            |
|      | 4     | float     | Current speed of Motor 2            |
|      | 5     | float     | Current Power being sent to Motor 1 |
|      | 6     | float     | Current Power being sent to Motor 2 |
| 1    | 0     | dword     | Device Status as a bit field        |
|      | 1     | dword     | ?????                               |
|      | 2     | float     | Servo Set point                     |
|      | 3     | float     | Motor 1 Set point                   |
|      | 4     | float     | Motor 2 Set point                   |
|      | 5     |           | Reserved returns 0                  |
|      | 6     |           | Reserved returns 0                  |
| 2    | 0     | dword     | Device Status as a bit field        |
|      | 1     | float     | Motor 1 Min Position setting        |
|      | 2     | float     | Motor 1 Max Position setting        |
|      | 3     | float     | Motor 1 Reverse Max Speed           |
|      | 4     | float     | Motor 1 Forward Max Speed           |
|      | 5     | float     | Motor 1 Reverse Max Power           |
|      | 6     | float     | Motor 1 Forward Max Power           |
| 3    | 0     | dword     | Device Status as a bit field        |
|      | 1     | float     | Motor 2 Min Position setting        |
|      | 2     | float     | Motor 2 Max Position setting        |
|      | 3     | float     | Motor 2 Reverse Max Speed           |
|      | 4     | float     | Motor 2 Forward Max Speed           |
|      | 5     | float     | Motor 2 Reverse Max Power           |
|      | 6     | float     | Motor 2 Forward Max Power           |
| 4    | 0     | dword     | Device Status as a bit field        |
|      | 1     | float     | Motor 1 Speed PID Kp setting        |
|      | 2     | float     | Motor 1 Speed PID Ki setting        |
|      | 3     | float     | Motor 1 Speed PID Kd setting        |
|      | 4     | float     | Motor 1 Power PID Kp setting        |
|      | 5     | float     | Motor 1 Power PID Ki setting        |
|      | 6     | float     | Motor 1 Power PID Kd setting        |
| 5    | 0     | dword     | Device Status as a bit field        |
|      | 1     | float     | Motor 2 Speed PID Kp setting        |
|      | 2     | float     | Motor 2 Speed PID Ki setting        |
|      | 3     | float     | Motor 2 Speed PID Kd setting        |
|      | 4     | float     | Motor 2 Power PID Kp setting        |
|      | 5     | float     | Motor 2 Power PID Ki setting        |
|      | 6     | float     | Motor 2 Power PID Kd setting        |
| 6    | 0     | dword     | Device Status as a bit field        |
|      | 1     | float     | Motor 1 Max power used              |
|      | 2     | float     | Motor 2 Max power used              |
|      | 3     | float     | Motor 1 Max reverse speed achieved  |
|      | 4     | float     | Motor 2 Max reverse speed achieved  |
|      | 5     | float     | Motor 1 Max forward speed achieved  |
|      | 6     | float     | Motor 2 Max forward speed achieved  |
| 7    | 0     | dword     | Device Status as a bit field        |
|      | 1     | Float     | Motor 1 Kick setting                |
|      | 2     | dword     | Motor 1 Filter for the analog input |
|      | 3     | Float     | Motor 2 Kick setting                |
|      | 4     | dword     | Motor 2 Filter for the analog input |
|      | 5     |           | Reserved for future use             |
|      | 6     |           | Reserved for future use             |


Device Status Bit Field
This is a 32 bit bit field, however not all have been defined at this time.

| Bit No. | Description                                                                       |
| ------- | --------------------------------------------------------------------------------- |
| 0       | Returned packet type bit 0                                                        |
| 1       | Returned packet type bit 1                                                        |
| 2       | Returned packet type bit 2                                                        |
| 3       | Controller ID bit 0                                                               |
| 4       | Controller ID bit 1                                                               |
| 5       | Controller ID bit 2                                                               |
| 6       | Servo Enabled                                                                     |
| 7       | Servo Disable Sleep (Proposed)                                                    |
| 8       | Motor 1 Stopped                                                                   |
| 9       | Motor 1 Running Forward                                                           |
| 10      | Motor 1 Running Reverse                                                           |
| 11      | Motor 2 Stopped                                                                   |
| 12      | Motor 2 Running Forward                                                           |
| 13      | Motor 2 Running Reverse                                                           |
| 14      | Motor 1 Paused.  (Position is deemed close enough to target after timeout period) |
| 15      | Motor 2 Paused.  (Position is deemed close enough to target after timeout period) |


