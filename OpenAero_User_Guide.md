# The OpenAero User Guide #
## What is OpenAero? ##
OpenAero is an aeroplane stabilising software for KK-type multicopter boards. You put the programmed KK board in between your receiver and the servos in your aircraft. When deactivated, OpenAero just passes the servo signals from your receiver to the servos. When activated, the gyros on your KK board are used to move the servos to counteract any unwanted movement of the aircraft.
## What does this guide cover? ##
This guide covers only the basic versions of OpenAero.
It does not cover versions with accelerometer or MEMS board add-ons for autoleveling.

## What versions of OpenAero are there? ##
### Standard ###
The standard PWM version of OpenAero has three stabilised outputs - Aileron, Elevator and Rudder.
For those needing a throttle control, you can just take the throttle output of your receiver directly to the ESC for your motor. The three-channel version of OpenAero provides finer control of servos.

The standard CPPM version of OpenAero has three stabilised outputs - Aileron, Elevator and Rudder, and outputs a copy of the throttle signal from the receiver out the THR pin of the KK board.

Read more about the differences between PWM and CPPM receivers elsewhere in this Wiki.
### Flying Wing ###
Flying wings normally have delta/flaperon mixing done by the transmitter. This allows you to set the aileron/elevator mixing as best suits you and your aircraft.
When these signals pass through the OpenAero software the gyro stabilising has to accommodate the flying wing channel mixing.
For instance, in response to pitch input, the standard version of Openaero would simply adjust the elevator servo. However the flying wing version adjusts both flaperons to have the desired effect.

The flying wing PWM version of OpenAero has three stabilised outputs - Left Flaperon, Right Flaperon and Rudder.

For those needing a throttle control, you can just take the throttle output of your receiver directly to the ESC for your motor. The three-channel version of OpenAero provides finer control of servos.

The flying wing CPPM version of OpenAero has three stabilised outputs - Left Flaperon, Right Flaperon and Rudder, and outputs a copy of the throttle signal from the receiver out the THR pin of the KK board.
### Flaperon ###
The flaperon version of OpenAero is designed for conventional aeroplanes that have individually controlled ailerons.
The aileron channels normally have flap signals mixed with them so that both ailerons can be lowered to act as flaps, or raised to act as spoilers. Having separate aileron control also allows the use of aileron differential, where the upward movement of the ailerons is larger than the downward.

These models have the flaperon mixing done by the transmitter. This allows you to set flap positions and differential as best suits you and your aircraft.
When these signals pass through the OpenAero software the gyro stabilising has to accommodate the flaperon mixing. To accommodate this, the flaperon version of OpenAero has four stabilised outputs - Left Flaperon, Right Flaperon, Elevator and Rudder. A copy of the throttle signal from the receiver is also output from the THR pin of the KK board.

The flaperon version requires a single-wire CPPM receiver and the CPPM version of the code OR the HYBRID\_PWM version of the code if a normal receiver is used. The HYBRID version allows the addition of a fifth PWM input into the KK board via one of the connectors normally used for an output.


### OpenAero Nano ###
There is also a version called OpenAero Nano, which is a cut-down version of OpenAero designed to fit into the smallest KK boards with only 4K of program memory.
You can read more about OpenAero Nano at the following link.

http://www.rcgroups.com/forums/showpost.php?p=20331503&postcount=2

## How do I connect up my model? ##
### PWM receiver connections ###
For a standard multi-wire PWM receiver you need to use the normal PWM inputs of the KK board. However, the THR input of the KK board has been repurposed to be the stability/autolevel switch input.

  * AIL: Aileron signal input (required)
  * ELE: Elevator signal input (optional)
  * THR: Stability switch input (required)
  * RUD: Rudder signal input (optional)

Please make careful note that for OpenAero to output servo signals, at least two inputs, including the stability switch input **MUST** be connected to the KK board.

Connect a switched input of your transmitter - normally the gear switch on channel 5 - to the Stability switch input.

Here is a diagram showing how to connect up a PWM receiver.
![http://i.imgur.com/PzWqc.jpg](http://i.imgur.com/PzWqc.jpg)
![http://i.imgur.com/sgalF.jpg](http://i.imgur.com/sgalF.jpg)

### HYBRID PWM receiver connections ###
In Hybrid PWM mode an additional PWM input is made on M3 for the second aileron channel of a flaperon or split aileron system.

  * AIL: Aileron signal input (required)
  * ELE: Elevator signal input (optional)
  * THR: Stability switch input (required)
  * RUD: Rudder signal input (optional)
  * M3:  Second aileron signal input (required)

Connect a switched input of your transmitter - normally the gear switch on channel 5 - to the Stability switch input.

Here is a diagram showing how to connect up a PWM receiver for the Hybrid code.
![http://i.imgur.com/EQA86.jpg](http://i.imgur.com/EQA86.jpg)

### CPPM receiver connections ###
**V1.11 and older**

For a single-wire CPPM receiver you need to connect to the ELE input pin of the KK board. All the required signals including the stability/autolevel switch inputs are conveyed over this wire.

  * AIL: Unused
  * ELE: CPPM signal input from receiver
  * THR: Throttle output
  * RUD: Unused

**V1.12 and newer**

For a single-wire CPPM receiver you need to connect to the M3 output pin of the KK board. All the required signals including the stability/autolevel switch inputs are conveyed over this wire.

  * AIL: Unused
  * ELE: Unused
  * THR: Throttle output
  * RUD: Unused
  * M3:  CPPM signal input from receiver

Here is a diagram showing how to connect up a CPPM receiver.
![http://i.imgur.com/Q3DiQ.jpg](http://i.imgur.com/Q3DiQ.jpg)
![http://i.imgur.com/wxQjY.jpg](http://i.imgur.com/wxQjY.jpg)

The CPPM versions require a receiver and a transmitter that can reallocate channels if not matching the JR/Spektrum channel order, or a CPPM converter board.
The OpenAero CPPM channel order is the same as the JR/Spektrum system:
  * CH1: Throttle
  * CH2: Aileron #1
  * CH3: Elevator
  * CH4: Rudder
  * CH5: Gear switch
  * CH6: Aileron #2
  * CH7: Aux 1
  * CH8: Aux 2 <-- Stability/Autolevel switch input
Note 1: The flaperon versions expect the second aileron channel to be found on Ch6.
Note 2: The Stability/Autolevel switch input is on Ch8.

### Servo connections ###
For the standard version the connections are as follows:
  * M1: Rudder
  * M2: Aileron
  * M3: CPPM input (if used)
  * M4: Elevator
  * M5: Unused
  * M6: Unused
  * THR: Throttle (CPPM versions only)

For the flying wing version the connections are as follows:
  * M1: Rudder
  * M2: Left flaperon
  * M3: CPPM input (if used)
  * M4: Right flaperon
  * M5: Unused
  * M6: Unused
  * THR: Throttle (CPPM versions only)

For the flaperon wing version the connections are as follows:
  * M1: Rudder
  * M2: Left flaperon
  * M3: CPPM input (if used)
  * M4: Elevator
  * M5: Right flaperon
  * M6: Unused
  * THR: Throttle (CPPM versions only)

### Extra connections ###
**Low battery alarm**

If you have a KK Plus board, OpenAero supports the detection of battery voltage via the attached power distribution pads.

**Buzzer/LED outputs**

If you have a KK Plus board, OpenAero supports the use of a piezo buzzer via the buzzer connections and LEDs via the LED driver pads.
The buzzer can be used as the audible signal for the lost model alarm or to signal that the battery voltage has dropped below the preset point.

## How do I set up OpenAero? ##
### Switching between stability modes ###
When used with a PWM receiver, the input connected to the THR pin controls stability.
When used with a CPPM receiver the switch input is taken from Ch.8. In order to share these inputs with autolevel mode, you can set it up to toggle stability switchable/alwayson and autolevel switchable/alwaysoff. So that gives you three options with a single channel switch:

  * 1. Autolevel always OFF, stability switchable ON/OFF (default)
  * 2. Stability always ON, Autolevel switchable ON/OFF
  * 3. Both stability and autolevel switchable ON/OFF

You set up the modes by using the yaw and roll TX sticks on power up and it's saved in eeprom. **Note:** For verions V1.13 and above, only the Yaw input is used for setting stability and autolevel modes.


Discussion about this will follow in the advanced user guide which includes autolevel.

### Pot positions and their use ###
**Roll Pot**

The roll pot is primarily used to adjust the roll and pitch gyro gain.
Increasing the gain will increase the amount that the control surfaces move in response to external movement.
Please note that adjusting the gain too far can lead to instability and oscillation.

The roll pot is also used to adjust the gyro polarity when turned to the minimum setting while being powered up. More on that later.


**Pitch Pot**

The pitch pot is used to adjust the roll and pitch PID I-term.
Increasing this value will increase the control surface's tendency to stay deflected until the model returns to its original position.
Please note that it is inadvisable to increase this above zero until much experimentation is done.

**Please leave this control at the minimum setting. If you find that the control surfaces slowly move to an unusual position when you first switch stability ON, then the Pitch pot is actually set at the maximum position and must be changed to the opposite position.**

**Yaw Pot**

The yaw pot is primarily used to adjust the yaw gyro gain.
Increasing the gain will increase the amount that the control surfaces move in response to external movement.
Please note that adjusting the gain too far can lead to instability and oscillation.

The yaw pot is also used to go into stick centering mode when turned to the minimum setting, and autotune mode when turned to the maximum setting while being powered up. More on that later.

### Initial setup and calibration ###
The first time you set up OpenAero you will have to adjust a few things to suit your radio gear and particular usage. While everything is adjustable via the LCD interface, for the moment we will just go over the most common ones that everyone can use.

**Setting failsafe mode (V1.13 and above)**

**(KK boards)**
To setup failsafe mode, set the control positions that you require, then simply short the M6 signal pin to ground. An RX bind plug linking the two opposite end pins is perfect for this job. M6 needs to be shorted for one second to trigger failsafe learning mode.
In failsafe learning mode the LED will start to flash. You can then remove the shorting jumper to stop the learning mode from restarting. When the LED has stopped flashing the failsafe has been set.

**(N6 boards)**
To setup failsafe mode, set the control positions that you require, then simply close switch 1 of the DIP switch. The switch needs to be shorted for one second to trigger failsafe learning mode.
In failsafe learning mode the LED will start to flash. You can then return the swithc to the original setting to stop the learning mode from restarting. When the LED has stopped flashing the failsafe has been set.

**Trimming your model**

For OpenAero to understand the normal trimmed radio settings, you have to ask it to learn the trimmed position.
Firstly though, your model has to be completely trimmed when OpenAero stability is OFF.
To be sure OpenAero stability is off, with the model powered up, move the model about and see if the control surfaces move. If so, stability is ON and you have to correct that before proceeding. Change the input that controls stability to the opposite state to switch off stability.
Now adjust all flying surfaces to the neutral position with all trims centered. Please adjust the mechanical servo connections and finally the computer-controlled trims in the transmitter until the model is trimmed.

**Stick centering**

To get OpenAero to learn the neutral stick position, all you have to do is power up the model (with transmitter on) while the Yaw pot is set to the correct position.
Different boards have the pot polarity reversed so there is no hard rule to determine which direction (clockwise or couter-clockwise) to turn the yaw pot.
What we need to do is set the Yaw pot to the minimum value. Turn the pot with a small screwdriver until it hits the stop in the minimum direction.
Power up the model and watch the LED on the KK board carefully.
If the Yaw pot is set to the correct position, the LED should blink rapidly three times then one long flash. If so the stick position has been successfully saved.
If the Yaw pot is set to the wrong position, the LED will blink about 12-13 times. Power off the model, reverse the Yaw pot and try again.
Once the stick position has been successfully saved, switch off and please remember to return the Yaw pot to the center of its travel.

**Checking the gyro direction**

An extremely important step is to confirm that the gyros are acting on your model as appropriate. If the gyro directions are reversed any motion of the model will be amplified in the same direction leading to an almost immediate crash.
Once you have the model trimmed correctly, switch on stabilisation and watch carefully how the control surfces move in response to moving the model.

Gyros respond to rotation, so the easiest way to check the gyro direction is to rotate the model on each axis one by one.
With the model held in front of you, rotate it along the roll axis and watch the ailerons move. They should move to oppose the movement. For example the falling wing's aileron should move DOWN and the rising wing's aileron should move UP. If this is not the case, the Roll gyro needs to be reversed.

With the model held in front of you, rotate it up and down along the pitch axis and watch the elevator move. It should move to oppose the movement. For example the elevator should move DOWN when you pitch the model's nose UP and DOWN when you pitch the model's nose UP. If this is not the case, the Pitch gyro needs to be reversed.

With the model held in front of you, rotate it left and right along the yaw axis and watch the rudder move. It should move to oppose the movement. For example the rudder should move LEFT when you yaw the model's nose RIGHT and RIGHT when you yaw the model's nose LEFT. If this is not the case, the Yaw gyro needs to be reversed.

**Adjusting the gyro direction**

To get OpenAero to change gyro direction, all you have to do is power up the model while the Roll pot is set to the correct position.
Different boards have the pot polarity reversed so there is no hard rule to determine which direction (clockwise or couter-clockwise) to turn the Roll pot.
What we need to do is set the Roll pot to the minimum value. Turn the pot with a small screwdriver until it hits the stop in the minimum direction.
Power up the model and watch the LED on the KK board carefully.
If the Roll pot is set to the correct position, the LED should blink rapidly **six** times, then stay ON. If so, we are in gyro reversing mode.

  * Move the transmitter roll stick left or right to set the roll gyro to the normal or reversed state. The LED on the KK board will flash briefly to indicate that the setting has changed.
  * Move the transmitter yaw stick left or right to set the yaw gyro to the normal or reversed state. The LED on the KK board will flash briefly to indicate that the setting has changed.
  * Move the transmitter pitch stick up or down to set the pitch gyro to the normal or reversed state. The LED on the KK board will flash briefly to indicate that the setting has changed.

Once the gyro direction has been successfully saved, switch off and please remember to return the Roll pot back to your preferred setting.

If you find that the gyro direction has not changed, try moving the transmitter stick in the **opposite** direction next time.

**Initial pot positions**

Try initially setting the Roll and Yaw pots to the center of its throw to set moderate gyro gain. Set the Pitch pot to minimum.

### Every time you power up OpenAero ###
There are some important things to note when starting OpenAero.

**Stick positions**

The stick positions that the transmitter is in as the model powers up are important to OpenAero. Unless you plan to alter the stability switch modes, make sure the transmitter sticks are centered while powering up your model.

**Gyro calibration**

The gyros are calibrated automatically when the model is powered up.
It is important that the model is held very still while the gyro calibration is taking place.

To make this easier, the gyro calibration is done after a short pause and just as the LED lights on the KK board. This allows two seconds or so to connect the battery and allow any movement to settle.


**Safety**

Always make sure your throttle is set to minimum before attempting to power up your model. **Warning:** Even with the transmitter throttle set to zero, there may be a momentary output from the KK board to the throttle channel which may cause the motor to turn very briefly. As such stay well away from the motor and propellor and sure your model when powering it up.

## Flying with OpenAero ##
Once you have confirmed that the gyro direction is correct you are ready to fly with OpenAero. Do initial test flights with stability switched OFF until you are sure that your model is aerodynamically trimmed and the control throws are set appropriately.
You can then, at a safe height, switch on stability and carefully observe the model.
You may find that the controls appear more sluggish than normal as the software dampens movement of the model.

## Using the LCD ##
OpenAero supports serial LCD modules for easy access to all settings in OpenAero. Having an LCD module allows you to make quick changes in the field and is invaluable during setup.

### Connecting an LCD ###
The KK Plus board has a dedicated LCD port that allows direct connection of an LCD module. For those that do not own a KK Plus board you will need to arrange connection to the following three signals.

  * MISO (PB4, pin 16 of the MPU) to LCD data
  * +5v
  * Ground


### Entering LCD mode ###
Once an LCD module is connected, and OpenAero powered up, you need to set the transmitter sticks to some extreme positions and hold them for at least two seconds.
If the correct stick positions have been held, the LCD module will display the OpenAero menu.
Due to channel reversing differences between tranmitter models, there is no hard and fast specification of the stick position but the official LCD entry position is:

  * Pitch up, Roll right and Yaw left

If you find that this does not cause OpenAero to go into LCD mode, then you will have to try all the variations of these position.

**LCD menu**

The LCD menu is navigated by moving the pitch stick up and down. The values can be changed by moving the roll stick and saved with the yaw stick.
The complete list of menu items is as follows:

  * Acc Trim (Roll): Used to fine tune the autolevel roll setting
  * Acc Trim (Pitch): Used to fine tune the autolevel pitch setting
  * PID Roll P-term: In EEPROM mode, used to adjust the Roll gain
  * PID Roll I-term: In EEPROM mode, used to adjust the Roll I-term
  * PID Pitch P-term: In EEPROM mode, used to adjust the Pitch gain
  * PID Pitch I-term: In EEPROM mode, used to adjust the Pitch I-term
  * PID Yaw P-term: In EEPROM mode, used to adjust the Yaw gain
  * PID Yaw I-term: In EEPROM mode, used to adjust the Yaw I-term
  * Gyr level P-term: Used to set the gyro gain in autolevel mode
  * Gyr level I-term: Used to set the gyro I-term in autolevel mode
  * Acc level P-term: Used to set the accelerometer gain in autolevel mode
  * Acc level I-term: Used to set the accelerometer I-term in autolevel mode
  * LVA voltage: Set the voltage at which the low voltage alarm (LVA) trips
  * LVA mode: Buzzer or LED
  * Battery voltage: Current battery voltage reading
  * Pot mode: EEPROM mode - use these settings, Pot mode - use pots
  * Roll gyro: Reverse roll gyro direction
  * Pitch gyro: Reverse pitch gyro direction
  * Yaw gyro: Reverse yaw gyro direction
  * Roll servo: Reverse roll servo direction
  * Pitch servo: Reverse pitch servo direction
  * Yaw servo: Reverse yaw servo direction
  * Calibrate Accel.: Start accelerometer calibration
  * Center sticks: Start stick centering
  * Stability mode: Switch between switchable and always ON
  * Autolevel mode: Switch between switchable and always OFF

Note that many of the above setting only apply to this with add-on accelerometers or MEMS boards.


### Exiting LCD mode ###
To exitLCD mode you need to set the transmitter sticks to some extreme positions and hold them for at least one second.
If the correct stick positions have been held, the LCD module will go blank and control return to the main program.
Due to channel reversing differences between tranmitter models, there is no hard and fast specification of the stick position but the official LCD exit position is:

  * Pitch up, Yaw right

Alternatively you can just reboot.

## Using the GUI ##
The OpenAero GUI is a great tool for visualising the sensors on your model and to not just set, but experiment with PID settings and simulate them on-screen. The limitations of the KK design mean that various KK boards will run at different speeds so it is likely that your board will have to be tuned. Fortunately, with the GUI this is relatively easy. You only have to do the Autotune once, unless you re-flash the board, or click on the "Defaults" button, which resets the eeProm values. The procedure to auto-tune is detailed later in this document.

Please note that the MultiWii GUI is Java-based and quite CPU intensive. Using the GUI while running too many applications might cuse it to slow.

Here's a screenshot of the new V0.2 GUI for the OpenAero. This example shows a model in Flaperon mode (two stabilised servos per wing).

![http://i.imgur.com/hSHbl.jpg](http://i.imgur.com/hSHbl.jpg)

### What do I need to get the GUI running? ###
You need a MKUSB-type serial interface for the GUI to communicate to the board.
Because of the limitations of the KK board, a non-standard cable is required.

The connections you need are as follows:
**For KK boards with an 8-pin header**

From: KK+         To: MKUSB X3 port (Note X3 port **NOT** X2 port)
  * Pin 8 receive      Pin 8 transmit
  * Pin 9 transmit     Pin 3 receive
  * Pin 10 ground     Pin 10 ground
  * Pin 2   +5V        Pin 2   +5V

**If you don't have an 8-pin header**

The 6-pin connector is missing the serial RXD signal that the KK+ has. It goes to pin 30 of the CPU. Looking at a Blackboard, for example, that pin is not connected at all. Unless you are super-keen to solder a fine wire to pin 30 and make up a special cable, you're out of luck with a Blackboard. However, if sufficiently motivated, here's the connections you'd need.
Note that on the HobbyKing boards, there is a test pad connected to this pin, making it much easier to access.

**For KK boards with an 6-pin header**

From: 6-pin KK To: MKUSB X3 port
  * Pin X RXD Pin 8 transmit
  * Pin 1 MISO to Pin 3 receive
  * Pin 6 Ground to Pin 10 ground
  * Pin 2 +5V to Pin 2 +5V

**Pin X goes to pin 30 of the Atmel CPU.**

Here is a picture of my very simple serial cable for 8-pin KK+ boards.
![http://i.imgur.com/MFTP5.jpg](http://i.imgur.com/MFTP5.jpg)

## Entering GUI mode ##
  * 1. Make sure your MKUSB and cable are connected to your PC and the KK+.
  * 2. Run the MultiWiiConf\_Next.exe file. Make sure your COM port is listed in the GUI.
  * 3. Click on the COM port and make sure it turns green.
  * 4. Power up the KK+ board.
  * 5. Within 10 seconds, click on the START button and data should start to be transferred to the GUI. The graph will start to move to the right and the gyro data should be displayed. Try moving the KK+ board and see the gyro outputs displayed on the graph.
  * 6. Click on READ under "Flight parameters". Now, all the PID etc data from the KK+ is transferred to the screen.
  * 7. You can modify the data by clicking on each box and moving the mouse left or right. To save any data, first STOP the graph, then click on WRITE.
  * 8. If you move the transmitter sticks you can see the values change in the GUI. The servo outputs are disabled when in GUI mode.

## Leaving GUI mode ##
To exit GUI mode simply switch off the model and power to the KK board.

## Autotuning your KK board ##
If you are seeing errors in the data being sent to the GI or are having difficulty communicating with the GUI you might need to autotune the KK board.

  * 1. Set the Yaw gain pot to the minimum value.
  * 2. Make sure your MKUSB and cable is connected to your PC and the KK+.
  * 3. Run the MultiWiiConf\_Next.exe file.
  * 4. When it opens, select the COM port from the list on the left top. The port number should turn green.
  * 5. Click on the button labelled "KK". If you move the mouse away from the button, you will notice that it has turned orange. Please check that the TX LED of the MKUSB is flashing very fast.
  * 6. Power up the KK+ board. The LED should flash 12-13 times.
  * 7. Click on the button labelled "KK". If you move the mouse away from the button, you will notice that it has turned back to green. Please check that the TX LED of the MKUSB has STOPPED flashing.
  * 8. Restore the Yaw gain pot to the original setting.
  * 9. Power off and restart

The KK board should now communicate with the GUI error-free.

## Additional features of the GUI ##
  * Stick centering: You can calibrate the stick center position with a single click on the "Calibrate Sticks" button.
  * Gyro calibration: You can calibrate the gyros with a single click on the "Calibrate Gyros" button.
  * Acc calibration: You can calibrate the accelerometers with a single click on the "Calibrate Acc" button.
  * LVA: You can see the current battery voltage, and set the low-voltage warning trigger level. You can also set it up to alert you via the buzzer output of via LEDs. In buzzer mode the buzzer is quiet normally and beeps rapidly when the battery voltage falls below the trigger level. In LED mode the output is ON normally and flashes slowly when the battery voltage falls below the trigger level.

### Using the LVA ###
The default mode for LVA is BUZZER mode and the trigger is 0 Volts.
While you can change the LVA mode and trigger at any time, the graph must be running to see the VBAT updating.
  * 1. Click and drag the LVA numbers until the trigger is higher than your VBAT value.
  * 2. Click on WRITE. The alarm should start beeping quickly.
  * 3. Reduce the LVA value to less than the VBAT.
  * 4. Click on WRITE. The alarm should stop beeping.
  * 5. Click on the MODE button to change the text to LED.
  * 6. Click on WRITE. The alarm will sound continually, or the LEDs will be ON.
  * 7. Click and drag the LVA numbers until the trigger is higher than your VBAT value.
  * 8. Click on WRITE. The alarm should start beeping at 1Hz, or the LEDs will be flashing.

# Donations #
OpenAero is a open-source project and is free, however it is also the result of many hundreds of hours of hard work.
If you feel motivated to donate to the project please feel free. I'm always happy to make special versions for people.

![http://i.imgur.com/uaRIu.png](http://i.imgur.com/uaRIu.png)

Yeah Google Code sucks and the button won't work. You can find a working button on the OpenAero Project page at the following link.
https://sites.google.com/site/openaerowiki/home