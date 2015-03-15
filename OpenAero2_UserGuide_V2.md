# Basic User Guide #
## Factory reset ##
This guide presumes all values have been reset to their factory default values, for a quick and easy setup, from a 'factory reset' to 'ready to fly'.

  1. Hold down all four buttons while powering up the KK2 board,
  1. A "**Reset**" message will briefly appear on the screen
  1. Release all buttons once the red LED has blinked,
  1. OpenAero2 will continue to display the idle screen.

Please note that at every power-up you are required to hold the model still while the message "Hold steady" is displayed. This is to ensure accurate calibration of the gyros.
If you fail to hold the model still, the message "Sensor error" will be displayed on the menu screen. In this case, you must repower the board.

## Idle Screen ##
After the default 10 seconds, the status screen will time-out to display the idle screen.

![http://i.imgur.com/966nbo0.jpg](http://i.imgur.com/966nbo0.jpg)

## Status Screen ##
The status screen allows OpenAero2 to provide current status information.

![http://i.imgur.com/xYFNAVl.jpg](http://i.imgur.com/xYFNAVl.jpg)

  * Mode - Selected Model Type.
  * Version - The version of OpenAero2 running on the board.
  * RX sync - Selected receiver type.
  * Profile - Current flight Profile - Replaces older stability/autolevel displays.
  * Free RAM - Displays lowest amount of processor memory ever measured.
  * Battery icon - Graphical representation of the flight pack voltage.
  * Battery voltage - Numerical representation of the flight pack voltage.

## Main Menu ##
Select the 'Menu', button 1 on the left, to display the main menu.

![http://i.imgur.com/cM9dmLt.jpg](http://i.imgur.com/cM9dmLt.jpg)

## 1. General ##
![http://i.imgur.com/CbIhAew.jpg](http://i.imgur.com/CbIhAew.jpg)

### Model Type ###
OpenAero2 has been designed with selectable model presets, for easy setup to control both single and dual aileron aeroplanes or flying-wings.

  * Aero - Default preset for both **single** and **dual aileron** aeroplanes.
  * F.Wing - Selectable preset for **flying-wings**.
  * CamStab - For standalone [camera stabilization](https://code.google.com/p/nextcopterplus/wiki/OpenAero2_CamStab_Setup_Part_1) & with  [RC control](https://code.google.com/p/nextcopterplus/wiki/OpenAero2_CamStab_Setup_Part_2) only.

Unlike other solutions, OpenAero2's on-board mixers are completely flexible to allow control of up to 12 individual channels or 8 directly connected servos.

Please refer to the ['Advanced User Guide'](OpenAero2_Adv_UserGuide_V2.md) for more information on how to customize the mixers.

### Board Orientation ###
OpenAero2 supports the following five mounting orientations.

  * Forward - Default horizontal orientation, buttons facing **aft** with screen to the **front**.
  * Aft - Horizontal, buttons facing **front** with screen to the **aft**.
  * Inverted - **Inverted** horizontal, buttons facing **aft** with screen to the **front**.
  * Vertical - **Upright**, buttons **down** with the screen on **top**.
  * Sideways - Horizontal, buttons to the right.

**Forward**

![http://i.imgur.com/OMxdRqn.jpg](http://i.imgur.com/OMxdRqn.jpg)

**Reverse**

![http://i.imgur.com/RfM475w.jpg](http://i.imgur.com/RfM475w.jpg)

## 2. Receiver setup ##

![http://i.imgur.com/WoRjK7u.jpg](http://i.imgur.com/WoRjK7u.jpg)

### Receiver Sync ###
While the KK2 board is limited to 5 PWM channels from a multi-wire receivers, OpenAero2 can support 8 CPPM channels from a single-wire receiver or via serial protocols such as S.Bus, Spektrum Satellite and XPS Xtreme.

  * Rudder - Default for most multi-wire PWM receivers.
  * CPPM - Listens for a compatible multi-channel stream on the **rudder** input only.
  * S.Bus - Serial input on the **Throttle** input via an S.Bus adapter cable.
  * Satellite - Serial input on the **Throttle** input via a Satellite adapter cable.
  * Xtreme - Serial input on the **Throttle** input

### Ch. order ###
With a compatible CPPM receiver attached to the rudder or serial RX on the throttle input, OpenAero2 supports either **JR/Spk**, **Futaba** or **Satellite** channel order.

The order of the channels will be defined by the brand of your transmitter. OpenAero2 supports the JR/Spektrum, Fubata and Spektrum Satellite channel allocations.

|**JR/Spektrum** | **Futaba**| **Satellite** |
|:---------------|:----------|:--------------|
|CH1: Throttle |CH1: Aileron #1| CH1: Flap |
|CH2: Aileron #1 |CH2: Elevator| CH2: Aileron |
|CH3: Elevator |CH3: Throttle| CH3: Elevator |
|CH4: Rudder |CH4: Rudder| CH4: Rudder |
|CH5: Gear switch |CH5: Gear switch| CH5: Gear |
|CH6: Aux 1 |CH6: Aileron #2| CH6: Throttle |
|CH7: Aux 2 |CH7: Aux 1| CH7: AUX1 |
|CH8: Aux 3  |CH8: Aux 2 | CH8: AUX2 |

### Profile channel ###
Selection of stability and autolevel is now managed by flight "Profiles". This gives far more flexibility than the older mode settings.

You must select which channel you are using to select the flight profile here.

You can then set up each of the three flight profiles to your liking.

### 2nd Aileron ###
In addition to an aileron on output 6, OpenAero2 can control a second aileron on output 7. Three methods are supported.

  * **None** - M7 remains unassigned and will **not** operate a second aileron
  * **Aileron** - Commonly used with PWM receivers to control a second aileron on output 7 as a copy of the single aileron channel.
  * **Any other channel** - If you have a second aileron signal from your TX and connected to the KK2 board, selecting it here will allow OpenAero2 to use it.

![http://i.imgur.com/aHbuCXV.jpg](http://i.imgur.com/aHbuCXV.jpg)

### Dynamic gain channel ###
Dynamic gain allows you to modify the gyro gains from the TX while flying or at the field by associating it with a TX channel
Set the channel number here.

### Dynamic gain ###
The amount of dynamic gain attenuation is determined here.
Setting dynamic gain to 100 allows you to completely dampen all gyro gain - which is the same as pass-through mode.
Setting dynamic gain to 0 gives you no dynamic gain adjustment at all and the model will use the maximum amount of gyro gain set in the flight profile.
Setting dynamic gain to 50 will allow you to reduce the gyro gain to half at the highest setting of the dynamic gain channel.

![http://i.imgur.com/obPCBLV.jpg](http://i.imgur.com/obPCBLV.jpg)

### Differential ###
If you have independently-controllable ailerons, you can set the amount of differential here.
Differential will cause the ailerons to move farther "UP" than "DOWN".
Setting differential to zero will cause the ailerons to behave as normal.
Setting differential to 50% will cause the ailerons to move half as much downwards as upwards.
Setting differential to 100% will cause the ailerons to not move downwards at all.

### Flap speed ###
OpenAero2 now allows you to set the deployment speed of flaperons.
It will not affect separate flap channels that you may route through OpenAero2.
Setting flap speed to zero will result in normal full-speed movement.
Setting flap speed to 20 will result in very slow flap deployment.

### Lock rate ###
When a profile that includes axis-locked gyros is selected, the stick input rate that will affect the gyros is set here.
Setting the number lower increases the stick input and higher reduces it.

### Deadband ###
When using axis-locked gyros, small errors or drift in the RC input will cause the servos to quickly swing to one side or the other.
Increase the deadband to reduce this. The deadband is the RC input value below which will be ignored.

## 3. Stick Polarity ##
OpenAero2 has been designed to unify the polarity of stick inputs for all flight modes. Failure to set the stick polarity correctly will result in a failure of OpenAero2 to control the model while stabilised.

![http://i.imgur.com/OF6LTkQ.jpg](http://i.imgur.com/OF6LTkQ.jpg)

The TX and RX must be powered up and transmitting data. Once the 'Stick polarity' menu has been entered, promptly complete the following actions.

  1. Move both sticks forward (away from you) and into top right corners,
  1. Wait till 'done' is displayed and the red light blinks,
  1. The main menu will be displayed upon completion.

## 4. Receiver Inputs ##
The 'Receiver Inputs' screen shows the current receiver values of up to eight channels. Note that a conventional multi-wire PWM receiver is only cable of connecting five channels. A CPPM or serial receiver can use up to eight.

![http://i.imgur.com/ceG7fDL.jpg](http://i.imgur.com/ceG7fDL.jpg)

### Stick Calibration ###
Center stick calibration allows OpenAero2 to center the receiver's input, to ensure correct roll rate and selection of flight modes.

Before pressing 'cal', ensure both throttle stick and any 3-position switches are centered. Set any two-position switches to the lowest position.

### Stick Polarity Check ###
Prior to setting up the flight modes, complete the following checks, ensuring **positive** values are displayed.

  1. Move the rudder stick right, check the rudder polarity.
  1. Move the aileron stick right, check the aileron polarity.
  1. Move the aileron stick right, check the polarity of the second aileron, define by '2nd aileron'.
  1. Push the elevator stick forwards / away, check the elevator polarity.
  1. Move the throttle stick to full throttle,  check the throttle polarity.

Once the above has been completed, all positive values should be displayed when both sticks are pushed forwards / away and to the right.

## 5 to 7. Profile Setup ##
OpenAero2 supports the remote activation of gyro stabilization modes via an RC channel. Either two or three position switches, or a rotary dial can be used.

![http://i.imgur.com/Q936zbt.jpg](http://i.imgur.com/Q936zbt.jpg)

### Profile Trigger ###
This setting will define at what position this profile will activate.

Profile 1 will be selected if neither of the triggers for Profiles 2 and 3 are exceeded. As such, normally this is set to -100% (lowest position).

For profile 2, this percentage will dictate the **lower** limit of the control input. For example, if this is set to 0% (mid position) then Profile 2 will activate from this point.

Similarly, the trigger for profile 3 is set higher still (say, 90%) so that it can be activated by the highest position of a 3-position switch.

### Stability ###
Selecting Stability **ON** will enable gyro stability in this Profile.
Note that unless PID values are set, the gyros will have no effect. The default values will however initially give the desired result.

### Autolevel ###
Selecting Autolevel **ON** will enable accelerometer-based stability in this Profile.
Note that unless PID values are set, the accelerometers will have no effect. The default values will however initially give the desired result.
Selecting Autolevel **Hands-free** will cause the autolevel function to enable only when your sticks are not being moved signifcantly.

### Gyro types ###
Each of the axis gyros (roll/pitch/yaw) can be set to rate mode or axis-lock mode.
In rate mode, the gyro simply provides a signal indicating rotation in the axis in question.
In axis lock mode, the gyro will continually signal any change from the position, or heading that it was in when initially activated.
You can use stick input to reset the "neutral" heading position at any time.

![http://i.imgur.com/ix6bNTy.jpg](http://i.imgur.com/ix6bNTy.jpg)

### Roll PID Settings ###
The default values of P, I and D for each axis are a good starting point, but they may well prove to be too small or large for your application. Please refer to other online guides for methods of optimising PID values.

### Roll I limit ###
When using an axis-locked gyro mode, this value will limit the servo throw of the I component. Setting this to 125% will allow full movement.

![http://i.imgur.com/VU6knHD.jpg](http://i.imgur.com/VU6knHD.jpg)

### Acc Roll P ###
The ACC P setting will directly set the autolevel pitch gain response. Increase this is the amount of servo movement is insufficient. Reduce this if there is unwanted oscillation.

### Roll Trim ###
Acc roll trim allows you to add a very small amount of roll trim for the autolevel. If you find that the autolevel has not been calibrated correctly, it may be easier to add some trim to allow the model to fly level.

![http://i.imgur.com/Ll4yn41.jpg](http://i.imgur.com/Ll4yn41.jpg)
![http://i.imgur.com/lSowpXj.jpg](http://i.imgur.com/lSowpXj.jpg)
![http://i.imgur.com/4usTNU7.jpg](http://i.imgur.com/4usTNU7.jpg)

## 8. Sensor Calibration ##
OpenAero2 uses both gyros and accelerometers to determine the attitude and rate of rotation of the aeroplane and add corrections.

![http://i.imgur.com/wC6ORmU.jpg](http://i.imgur.com/wC6ORmU.jpg)

Once the board has been securely mounted within the aeroplane, the sensors can be calibrated.

### Calibrate ###
This option calibrates the accelerometers only, so OpenAero2 can determine the angle parallel to the ground.

Prior selecting 'Cal', available on button 4, ensure the model is perfectly horizontal and in the upright position.

For best results, firmly prop up the model on a hard surface and use a spirit level to ensure the model is perfectly level.

### Inverted calibrate ###
To more accurately determine the inverted orientation, a second calibration is required.
Prior to selecting 'Inv', available on button 3, ensure the model is perfectly horizontal and in the inverted position.

For best results, firmly prop up the model on a hard surface and use a spirit level to ensure the model is perfectly level.

## 9. Level Meter ##
This is a visual representation of what OpenAero2 believes to be level, based on the successful calibration of the accelerometers.

![http://i.imgur.com/Or0D6Uy.jpg](http://i.imgur.com/Or0D6Uy.jpg)

## 10. Channel Mixing ##
For advanced users, OpenAero2 has the flexibility to mix multiple receiver and sensor inputs, to produce the desired outputs.

![http://i.imgur.com/oLcsXH3.jpg](http://i.imgur.com/oLcsXH3.jpg)
![http://i.imgur.com/6LcIg7x.jpg](http://i.imgur.com/6LcIg7x.jpg)

For typical single and dual aileron aeroplane's, the default settings will work successfully.

Please refer to the ['Advanced User Guide'](OpenAero2_Adv_UserGuide_V2.md) for more information.

## 11. Output Mixing ##
For advanced users, OpenAero2 has the flexibility to combine the movements of two outputs on a third individual output

![http://i.imgur.com/WjF1TMn.jpg](http://i.imgur.com/WjF1TMn.jpg)

Please refer to the ['Advanced User Guide'](OpenAero2_Adv_UserGuide_V2.md) for more information.

## 12. Servo Direction ##
OpenAero2 allows the direction any output to be reversed internally, rather than externally or mechanically.

![http://i.imgur.com/zIi0hX1.jpg](http://i.imgur.com/zIi0hX1.jpg)

In the event a control surface is moving in the wrong direction, simply reversed the affected output in OpenAero2 here rather than in the transmitter.

## 13. Servo Trim ##
OpenAero2 allows the neutral position of each servo to be updated in real-time.

![http://i.imgur.com/o34gGIO.jpg](http://i.imgur.com/o34gGIO.jpg)

In the event a control surface is slightly off neutral, simply increase or decrease the trim for the affected output OpenAero2 rather than in the transmitter.

## 14. Negative Servo Travel ##
OpenAero2 allows real-time configuration of the maximum negative servo travel to prevent binding.

![http://i.imgur.com/LFOCCxn.jpg](http://i.imgur.com/LFOCCxn.jpg)

This feature is particularly useful when stability and autoleveling is used with flaperons.

## 15. Positive Servo Travel ##
Similar to 'negative servo travel', OpenAero2 allows real-time configuration of the maximum positive servo travel to prevent binding.

![http://i.imgur.com/yE5a75i.jpg](http://i.imgur.com/yE5a75i.jpg)

Again, this feature is particularly useful when stability and autoleveling is used with flaperons.

## 16. Failsafe Settings ##
OpenAero2 has been designed with built-in failsafes, which have the potential to allow the pilot to regain control and save your aircraft.

![http://i.imgur.com/qbFeiNm.jpg](http://i.imgur.com/qbFeiNm.jpg)

**Mode** supports two options.

  * **Fixed** - Simple fix servo positions
  * **Advanced** - Automatically enables autoleveling, with a slight trim.

Please refer to the ['Advanced User Guide'](OpenAero2_Adv_UserGuide_V2.md) for more information.

## 17. Failsafe Positions ##
OpenAero2 allows the '**fixed**' failsafe positions to be configured, which become active in the event the receiver signal is lost.
Please note that if your receiver continues to transmit a signal, invalid or otherwise, OpenAero2 cannot switch into failsafe mode.
If your receiver does this, please set up the failsafe function **on the receiver**.

![http://i.imgur.com/oVlTxWV.jpg](http://i.imgur.com/oVlTxWV.jpg)

The initial position can be set by selecting 'Failsafe' in 'Receiver Inputs'

These settings will be ignored if 'advanced' mode in 'failsafe settings' is selected.

## 18. Battery Monitor ##
OpenAero2 has the ability to monitor a flight or receiver pack during flight.

![http://i.imgur.com/Kep5f5E.jpg](http://i.imgur.com/Kep5f5E.jpg)

Typically, only the number of **cells** has to be specified to suit your model.

The voltage of the pack is displayed both as a graphical & numerical representation on the 'status' screen and the piezo buzzer will emit a series of warning tone in the event the low voltage has been breached.