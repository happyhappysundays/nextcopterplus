[![](http://i.imgur.com/uaRIu.png)](https://sites.google.com/site/openaerowiki/home)

Please ensure you read the manuals in the following order.

  1. [Frequently Ask Questions](OpenAero2_FAQ.md)
  1. [Getting Started Guide](OpenAero2_Getting_Started.md)
  1. [Aeroplane - Quick Setup Guide](OpenAero2_UserGuide_V1.md)
  1. [Aeroplane - Advanced User Guide](OpenAero2_Adv_UserGuide_V1.md)
  1. [Camera - Standalone Guide](OpenAero2_CamStab_Setup_Part_1.md)
  1. [Camera - Receiver Control Guide](OpenAero2_CamStab_Setup_Part_2.md)

Please post any questions and tips on [RC Groups](http://www.rcgroups.com/forums/showthread.php?t=1708175).

# Aeroplane - Quick Setup guide #

### Factory reset ###
This guide presumes all values have been reset to their factory default values, for a quick and easy setup, from a 'factory reset' to 'ready to fly'.

  1. Hold down all four buttons while powering up the KK2 board,
  1. A "**Reset**" message will briefly appear on the screen after the OpenAero2 logo is displayed,
  1. Release all buttons once the red LED has blinked,
  1. OpenAero will continue to display the idle screen.

### Idle Screen ###
After the default 10 seconds, the status screen will time-out, to display the idle screen.

![http://i.imgur.com/966nbo0.jpg](http://i.imgur.com/966nbo0.jpg)

### Status Screen ###
The status screen allows OpenAero2 to provide current status information.

![http://i.imgur.com/TW36X8l.jpg](http://i.imgur.com/TW36X8l.jpg)

  * Mode - **1. General** - Selected **Model Type**.
  * Version - The version of OpenAero2 running on the board.
  * RX sync - **2. Receiver setup** - Selected receiver type.
  * Autolevel - **6. Autolevel Setup** - Status of autolevel mode.
  * Stability - **5. Stability Setup** - Status of stability mode.
  * Battery icon - **17. Battery Monitor** - Graphical representation of the flight pack voltage.
  * Battery voltage - **17. Battery Monitor** - Numerical representation of the flight pack voltage.
  * RAM: **Advanced feature** - Displays available processor memory after loading selected menus.

### Main Menu ###
Select the 'Menu', button 1 on the left, to display the main menu.

![http://i.imgur.com/cM9dmLt.jpg](http://i.imgur.com/cM9dmLt.jpg)

## 1. General ##
![http://i.imgur.com/CbIhAew.jpg](http://i.imgur.com/CbIhAew.jpg)

### Model Type ###
OpenAero2 has been designed with selectable model presets, for easy setup to control both single and dual aileron aeroplane's or flying-wing's.

  * Aero - Default preset for both **single** and **dual aileron** aeroplane's.
  * F.Wing - Selectable preset for **flying-wing**'s.
  * CamStab - For standalone [camera stabilization](https://code.google.com/p/nextcopterplus/wiki/OpenAero2_CamStab_Setup_Part_1) & with  [RC control](https://code.google.com/p/nextcopterplus/wiki/OpenAero2_CamStab_Setup_Part_2) only.

Unlike other solutions, OpenAero2's on-board mixers are completely flexible to allow control of up to 8 individual channels or directly connected servos.

Please refer to the ['Advanced User Guide'](OpenAero2_Adv_UserGuide_V1.md) for more information on how to customize the mixers

### Board Orientation ###
OpenAero2 supports the following four mounting orientations.

  * Forward - Default horizontal orientation, buttons facing **aft** with screen to the **front**.
  * Aft - Horizontal, buttons facing **front** with screen to the **aft**.
  * Inverted - **Inverted** horizontal, buttons facing **aft** with screen to the **front**.
  * Vertical - **Upright**, buttons **down** with the screen on **top**.

## 2. Receiver setup ##

![http://i.imgur.com/syjSafK.jpg](http://i.imgur.com/syjSafK.jpg)

### Receiver Sync ###
While the KK2 board is limited to 5 PWM channels from a multi-wire receivers, OpenAero2 can support 8 CPPM channels from a single-wire receiver.

  * Rudder - Default for most multi-wire PWM receivers.
  * CPPM - Listens for a compatible multi-channel stream on the **rudder** input only.

### CPPM order ###
With a compatible CPPM receiver attached to the rudder input only, OpenAero2 supports either **JR/Spk** or **Futaba** channel order.

The order of the channels will be define by the brand of your transmitter.

### Stability input ###
OpenAero2 allows any input channel to remotely enable or disable gyro stability.

By default, both PWM & CPPM setups commonly use the '**gear**' channel.

### Autolevel input ###
Similar to stability mode, OpenAero2 allows any input channel to remotely enable or disable auto-leveling.

  * **Gear** - Typically used by PWM receivers on the same 2 or 3 position flight switch.
  * **AUX1** - Commonly used by CPPM receivers on a second individual flight switch.

If the '**gear**' channel is used for both '**Stab input**' and '**Autolev input**', both stabilization and autoleveling will be switch on and off together.

### 2nd Aileron ###
In addition to an aileron on output 6, OpenAero2 can control a second aileron on output 7. Three methods are supported.

![http://i.imgur.com/dLFoASW.jpg](http://i.imgur.com/dLFoASW.jpg)

  * **None** - M7 remains unassigned and will **not** operate a second aileron
  * **Aileron** - Commonly used with PWM receivers to control a second aileron on output 7.

#### Flaperon ####
Allows both ailerons to be used as flaperons, where the flaps and ailerons are pre-mixed within the transmitter, prior to OpenAero2.

  * **Throttle** - PWM receivers only.
  * **AUX1** - CPPM receivers only.

## 3. Stick Polarity ##
OpenAero2 has been designed to unify the polarity of stick inputs for all flight modes.

![http://i.imgur.com/OF6LTkQ.jpg](http://i.imgur.com/OF6LTkQ.jpg)

Once the 'Stick polarity' menu has been entered, promptly complete the following actions.

  1. Move both sticks forward and into top right corners,
  1. Wait till 'done' is displayed and the red light blinks,
  1. The main menu will be displayed upon completion.

## 4. Receiver Inputs ##
The 'Receiver Inputs' screen shows the current receiver values of the first five channels from a PWM or CPPM receiver.

![http://i.imgur.com/0TWX9qA.jpg](http://i.imgur.com/0TWX9qA.jpg)

### Stick Calibration ###
Center stick calibration allows OpenAero2 to center both PWM & CPPM receivers input, to ensure correct roll rate and selection of flight modes.

When selecting 'cal', ensure both throttle stick and any 3-position flight switch are centered.

### Stick Polarity Check ###
Prior to setting up the flight modes, complete the following checks, ensuring **positive** values are displayed.

  1. Move the rudder stick right, check the rudder polarity.
  1. Move the aileron stick right, check the aileron polarity.
  1. Move the aileron stick right, check the polarity of the second aileron, define by '2nd aileron'.
  1. Push the elevator stick forwards / away, check the elevator polarity.
  1. Move the throttle stick to full throttle,  check the throttle polarity.

Once the above has been completed, all positive values should be displayed when both sticks are pushed forwards / away and to the right.

## 5. Stability Setup ##
OpenAero2 supports the remote activation of gyro stabilization either via a two or three position switch.

![http://i.imgur.com/fsmrG41.jpg](http://i.imgur.com/fsmrG41.jpg)

By default, '**StabChan**' allows a 2-position switch to activate gyro stabilization via the '**gear**' channel.

A 2-position flight switch will be limited to gyro stabilization **on** or **off**.

Ensure your 2-position stability switch is set to -100% & +100% for correct mode selection.

Alternatively, selecting '**3-pos**' allows a 3-position switch to alternate between all three flight modes upon the single channel.

Ensure your 3-position switch is set to -100%, 0%, +100% and the switch was centered when 'stick calibration' was ran.

## 6. Autolevel Setup ##
Similar to stability mode, OpenAero2 supports the remote activation of autoleveling either via a two or three position switch.

![http://i.imgur.com/gPbXhIr.jpg](http://i.imgur.com/gPbXhIr.jpg)

By default, the '**AutoChan**' option allows a 2-position switch to activate autoleveling via the '**gear**' channel. A 2-position flight switch will be limited to autoleveling **on** or **off**.

Ensure your 2-position stabilization switch is set to -100% & +100% for correct mode selection.

If the '**gear**' channel is used for both '**Stab input**' and '**Autolev input**', both stabilization and autoleveling will be switch on and off together.

Please refer to the ['Advanced User Guide'](OpenAero2_Adv_UserGuide_V1.md) for more information on 'Hands Free' mode.

Alternatively, selecting '**3-pos**' allows a 3-position switch to alternate between all three flight modes upon the single channel.

Make sure your 3-position switch is set to -100%, 0%, +100% and the switch was centered when 'stick calibration' was ran.

## 7. Sensor Calibration ##
OpenAero2 uses both gyro's and accelerometers to determine the angle of the aeroplane and add corrections.

![http://i.imgur.com/wC6ORmU.jpg](http://i.imgur.com/wC6ORmU.jpg)

Once the board has been securely mounted within the aeroplane, the sensors can be calibrating.

### Calibrate ###
This option calibrates the accelerometers only, so OpenAero2 can determine the angle horizontal to the ground.

Prior selecting 'Cal', available on button 4, ensure the model is perfectly horizontal in the upright position.

For best results, firmly prop up the model on a hard surface and use a spirt level to ensure the model is perfectly level.

### Inverted calibrate ###
Once calibrated successfully, OpenAero2 will be able to upright the aircraft from an inverted position.

This option will only become active once the above calibration has successfully been completed.

Prior selecting 'Inv', available on button 3, ensure the model is perfectly horizontal in the inverted position.

For best results, firmly prop up the model on a hard surface and use a spirt level to ensure the model is perfectly level.

## 8. Level Meter ##
This is a visual representation of what OpenAero2 believes to be level, based on the successful calibration of the accelerometers.

![http://i.imgur.com/Or0D6Uy.jpg](http://i.imgur.com/Or0D6Uy.jpg)

## 9. Channel Mixing ##
For advanced users, OpenAero2 has the flexibility to mix multiple receiver and sensor inputs, to produce the desired outs.

![http://i.imgur.com/pHJhhlr.jpg](http://i.imgur.com/pHJhhlr.jpg)

For typical single and dual aileron aeroplane's, the default settings will work successfully.

Please refer to the ['Advanced User Guide'](OpenAero2_Adv_UserGuide_V1.md) for more information.

## 10. Output Mixing ##
For advanced users, OpenAero2 has the flexibility to combine the movements of two outputs on a third individual output

![http://i.imgur.com/WjF1TMn.jpg](http://i.imgur.com/WjF1TMn.jpg)

Please refer to the ['Advanced User Guide'](OpenAero2_Adv_UserGuide_V1.md) for more information.

## 11. Servo Direction ##
OpenAero2 allows the direction any output to be reversed internally, rather than externally or mechanically.

![http://i.imgur.com/zIi0hX1.jpg](http://i.imgur.com/zIi0hX1.jpg)

In the event a control surface is moving in the wrong direction, simply reversed the affected output in OpenAero2 here rather than in the transmitter.

## 12. Servo Trim ##
OpenAero2 allows the neutral position of each servo to be updated in real-time.

![http://i.imgur.com/o34gGIO.jpg](http://i.imgur.com/o34gGIO.jpg)

In the event a control surface is slightly off neutral, simply increase or decrease the trim for the affected output OpenAero2 rather than in the transmitter.

## 13. Negative Servo Travel ##
OpenAero2 allows real-time configuration of the maximum negative servo travel to prevent binding.

![http://i.imgur.com/LFOCCxn.jpg](http://i.imgur.com/LFOCCxn.jpg)

This feature is particularly useful when stability and autoleveling is used with flaperons.

## 14. Positive Servo Travel ##
Similar to 'negative servo travel', OpenAero2 allows real-time configuration of the maximum positive servo travel to prevent binding.

![http://i.imgur.com/yE5a75i.jpg](http://i.imgur.com/yE5a75i.jpg)

Again, this feature is particularly useful when stability and autoleveling is used with flaperons.

## 15. Failsafe Settings ##
OpenAero2 has been designed with built-in failsafe's, which have the potential to allow the pilot to regain control and save your aircraft.

![http://i.imgur.com/qbFeiNm.jpg](http://i.imgur.com/qbFeiNm.jpg)

**Mode** supports two options.

  * **Fixed** - Simple fix positions
  * **Advanced** - Automatically enables autoleveling

Please refer to the ['Advanced User Guide'](OpenAero2_Adv_UserGuide_V1.md) for more information.

## 16. Failsafe Positions ##
OpenAero2 allows the '**fixed**' failsafe positions to be configured, which become active in the event the receiver signal is lost.

![http://i.imgur.com/oVlTxWV.jpg](http://i.imgur.com/oVlTxWV.jpg)

The initial position can be set by selecting 'Failsafe' in 'Receiver Inputs'

These settings will be ignored if 'advanced' mode in 'failsafe settings' is selected.

## 17. Battery Monitor ##
OpenAero2 has the ability to monitor a flight or receiver pack during flight.

![http://i.imgur.com/Kep5f5E.jpg](http://i.imgur.com/Kep5f5E.jpg)

Typically, only the number of **cells** has to be specified to suit your model.

The voltage of the pack is displayed both as a graphical & numerical representation on the 'status' screen and the piezo buzzer will emit a series of warning tone in the event the low voltage has been breached.