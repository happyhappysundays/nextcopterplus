# Aeroplane - Advanced User guide #

## The LCD Menu System ##

### The Status Screen ###
The status screen gives you a continually updated display of the battery status, flight modes and any error messages.
For performance reasons the status screen will switch back to the idle screen after a short interval. The interval defaults to 10 seconds but can be set to up to 60s in the General Menu.

**Status screen**

![http://i.imgur.com/TW36X8l.jpg](http://i.imgur.com/TW36X8l.jpg)

  * **Mode:** The current mixer preset (Aero, F.Wing or Camstab)
  * **Version:** The version number of the loaded firmware
  * **RX sync:** The receiver sync mode (CPPM, Rudder, Throttle or Gear)
  * **Profile:** The current flight profile selected (1 to 3)
  * **Battery icon:** The battery icon shows the remaining battery capacity. The limits of the capacity meter are calculated from the battery settings menu. For example, if a 3S LiPo is used and the minimum cell voltage is set to 3.7V (370) the graphic will show empty at a total battery voltage of 11.1V. If the maximum cell voltage is set to 4.2V (420) the graphic will show full at a total cell voltage of 12.6V.
  * **Battery voltage:** The battery voltage is displayed under the battery icon
  * **Free RAM:** The number of bytes of never-used RAM is shown to help debug potential stack overflow conditions. If this number ever reaches zero, a stack overflow has occurred.

Pressing button 1 goes to the main menu.

**Idle screen**

![http://i.imgur.com/966nbo0.jpg](http://i.imgur.com/966nbo0.jpg)

### Main Menu ###
![http://i.imgur.com/cM9dmLt.jpg](http://i.imgur.com/cM9dmLt.jpg)

**Main menu**
The main menu stops the loop. The main menu controls should be self-explanatory.
LEFT exits to the status menu
RIGHT goes down into a submenu
UD/DOWN moves through the menu list

**Sub menus**
LEFT exits to the main menu
RIGHT open up an item to edit
UD/DOWN moves through the sub menu list

**Menu item**
LEFT resets the item to its minimum value
RIGHT saves the item
UD/DOWN increments and decrements the item

### General Menu ###
![http://i.imgur.com/CbIhAew.jpg](http://i.imgur.com/CbIhAew.jpg)

  * **Mode:** Allows you to select from the input preset mixers. Currently there are presets for dual-aileron aeroplane (Aero), flying wing (F.Wing) and camera stability (Camstab).
  * **Orientation:** Remaps the sensors so that you can mount the board in one of four different ways.
  1. **Forward:** KK2.0 board flat with the buttons facing the rear of the model.
  1. **Vertical:** KK2.0 board up on edge with the buttons facing down and the LCD screen facing to the right side of the model.
  1. **Inverted:** KK2.0 board flat but upside down, with the buttons facing the rear of the model.
  1. **Aft:** KK2.0 board flat with the buttons facing the front of the model.
  * **Contrast:** Allows you to change the contrast of the LCD. See below.
  * **Status time:** Allows you to set the amount of time the status screen is displayed until it is replaced by the idle mode screen. Settable from 1 to 60 seconds.

![http://i.imgur.com/blhHKOa.jpg](http://i.imgur.com/blhHKOa.jpg)

  * **LMA timeout:**  Allows you to set the amount of time without RC activity before the Lost Model Alarm goes off. Setting to zero disables the alarm.
  * **Camstab.:** Setting this to ON allows the board to function without an external RC source. Use only for stand-alone camera stability.
  * **Cam. servo:** If Camera Stab is ON, you can increase the servo refresh rate on M1 to M4 to 300Hz.
  1. **LOW:** Servo rate is the same as the input RC (typically 50Hz)
  1. **HIGH:** Servo rate is approximately 300Hz
  * **Acc LPF:** Allows adjustment of the accelerometer Low-Pass Filter from 1 (infinite) to 64 (about 1 Hz). Increase this to remove noise into the accelerometers. This will reduce the accelerometer response.

![http://i.imgur.com/6P7aBnH.jpg](http://i.imgur.com/6P7aBnH.jpg)

  * **CF factor:** Controls the fusion of Accelerometer and Gyros. The higher the number the faster as gyro is favoured, the lower the more accurate but slower.
  * **Head. hold:** Heading hold mode. There are three options.
  1. **Normal:** Heading hold is processed normally when the I-terms and I-term limits are set correctly.
  1. **Auto:** Automatically centers all axis when in heading hold mode if they have drifted off-center.
  1. **3D:** Allows the sticks to re-center the I-term neutral when in heading hold mode. Use with great caution and test the mode out first before attempting to fly. Also, make absolutely sure you can switch off stability if required.
  * **Adv.IMU:** There are two options.
  1. **OFF:** As per V1.0 and earlier. Sticks over-ride gyro input but gyros dampen sticks.
  1. **ON:** Sticks taken into account. Sticks set gyro rotation rate in stability mode and so are much more responsive. Autolevel mode is the same as V1.0 but the IMU takes inverted attitudes into account.

### Adjusting LCD Contrast ###
![http://i.imgur.com/WzAeW.jpg](http://i.imgur.com/WzAeW.jpg)
![http://i.imgur.com/QvA6c.jpg](http://i.imgur.com/QvA6c.jpg)
![http://i.imgur.com/tjW92.jpg](http://i.imgur.com/tjW92.jpg)
  * Reducing the number reduces contrast.
  * Pressing button 1 "Def." returns the value to the default of 38.
  * Increasing the number increases contrast.

### Receiver Setup Menu ###
![http://i.imgur.com/syjSafK.jpg](http://i.imgur.com/syjSafK.jpg)

  * **RX sync:** Many users have previously experienced trouble with their receivers and OpenAero. For many it was because they did not use a rudder input. The rudder input was used by the software to signify that all channel data had been received. Now you have the option of syncing to the Rudder input, the Throttle input or the Gear input. If you have no servo jitter, then "Rudder" is working for you, but if not, please try "Gear" and "Throttle" to see what difference they make.
If you are lucky enough to own a CPPM-capable receiver, please select "CPPM".
  * **CPPM order:** When using CPPM input, this setting allows you to select either JR/Spektrum (TAER) or Futaba (AETR) channel order.
  * **Stability Ch.:** Set this to the channel number that you plan to use for stability control.
  * **Autolevel Ch.:** Set this to the channel number that you plan to use for autolevel input. It can be the same as for stability if you like.

![http://i.imgur.com/dLFoASW.jpg](http://i.imgur.com/dLFoASW.jpg)

  * **2nd Ail. Ch.:** Set this to the channel number that your radio uses for the second aileron channel. If you wish to use separate aileron servos but don't have a separate (second) aileron RC channel, select "Aileron" here.
If you only plan to use one servo, or two servos with a Y-cable, leave this as "None".
  * **Aileron pol:** If your TX cannot reverse channels to suit OpenAero2 then you can do this here. The Stick Polarity screen will over-write this automatically.
  * **2nd Ail. pol:** If your TX cannot reverse channels to suit OpenAero2 then you can do this here. The Stick Polarity screen will over-write this automatically.
  * **Elevator pol:** If your TX cannot reverse channels to suit OpenAero2 then you can do this here. The Stick Polarity screen will over-write this automatically.
  * **Rudder pol:** If your TX cannot reverse channels to suit OpenAero2 then you can do this here. The Stick Polarity screen will over-write this automatically.

![http://i.imgur.com/QLIgrrE.jpg](http://i.imgur.com/QLIgrrE.jpg)

  * **Differential:** If you wish to use aileron differential, do not set it up in the transmitter, use this setting. Setting 0% is no differential, 50% will limit the downward throw of ailerons to 50% and 100% will result in split ailerons (no downward travel).

### Stick polarity screen ###
![http://i.imgur.com/OF6LTkQ.jpg](http://i.imgur.com/OF6LTkQ.jpg)

In an age where many people willfully refuse to read manuals, I created this screen to help them sort through their problems automatically.
OpenAero2 **requires** specific RC channel polarity. While this is easy to set up, it is also very easy to NOT do...
When this screen is entered, it first checks for the required presence of an RC signal. If none is found the message "No RX signal!" is displayed. Please connect a valid signal to the KK2.0 board.
Once this has been done, the message will change to "Hold as shown". This indicates to the user that they must move the transmitter sticks away and to the right.
Once this has been done, the message "Done!" is displayed and the various RC input reversals changed and saved for you.
It is critical that you use this screen in the order indicated by the main menu. In other words, do this after completing the Receiver Setup screen.

### Receiver  inputs Screen ###
![http://i.imgur.com/0TWX9qA.jpg](http://i.imgur.com/0TWX9qA.jpg)

The RC inputs screen shows you the current values received by the system in either PWM or CPPM mode. In CPPM mode, only the first five channels are shown.

Pressing button 4 "Cal." performs a stick centering calibration. This should make all the numbers on the right-hand column drop to zero. Make sure all controls are centered **including the throttle** before calibration.

Pressing button 3 'Failsafe" saves the current stick positions to the OUT1 to OUT8 failsafe setting. The servos will adopt this position in the event of a loss of input signal.
Note that most receivers will still transmit the last known RC values if it loses the TX signal, so this is no test of failsafe. You need to physically disconnect the receiver.
It is recommended that receivers with built-in failsafe be set up and used.
Regardless of your plan to use failsafe, please center all sticks and **set the throttle to minimum** and press button 3. This will set a safe default failsafe.

### Stability Setup Menu ###
![http://i.imgur.com/fsmrG41.jpg](http://i.imgur.com/fsmrG41.jpg)

  * **Mode:** This setting determines how the stability is to be controlled. There six options.
  1. **OFF:** Always off
  1. **Autochan:** On when the channel set as "Autochan" in Receiver settings is more than the trigger setting.
  1. **Stabchan:** On when the channel set as "Stabchan" in Receiver  settings is more than the trigger setting.
  1. **3-pos:** Stability on when the channel set as "3-pos" in Receiver  settings is more than the stability trigger. Autolevel also on when the channel set as 3-pos in RC settings is more than the autolevel trigger. Make sure that your 3-position switch is set to -100%, 0% and 100% to reliably trigger with the default settings.
  1. **ON:** Always on
  * **Trigger:** Stick position to enable (-125% to 125%)
  * **Dyn.Gain Ch:** The channel used to influence the overall PID gain of all axis.
  * **Dyn.Gain:** The amount of attenuation you desire at the 100% point of the Dynamic gain channel source. Setting 100 will completely remove stability at 100% of the source channel's value.

![http://i.imgur.com/ucDpfYI.jpg](http://i.imgur.com/ucDpfYI.jpg)

  * **Roll P:** PID Proportional gain for the roll gyro
  * **Roll I:** PID I-term gain for the roll gyro
  * **Roll D:** PID Differential gain for the roll gyro
  * **Pitch P:** PID Proportional gain for the pitch gyro

![http://i.imgur.com/YuW9FHj.jpg](http://i.imgur.com/YuW9FHj.jpg)

  * **Pitch I:** PID I-term gain for the roll gyro
  * **Pitch D:** PID Differential gain for the pitch gyro
  * **Yaw P Gain:** PID Proportional gain for the yaw gyro
  * **Yaw I Gain:** PID I-term gain for the roll gyro

![http://i.imgur.com/fNoRt1C.jpg](http://i.imgur.com/fNoRt1C.jpg)

  * **Yaw D Gain:** PID Differential gain for the yaw gyro
  * **Roll I Limit:** Maximum influence of I-term (-125% to 125%)
  * **Pitch I Limit:** Maximum influence of I-term (-125% to 125%)
  * **Yaw I Limit:** Maximum influence of I-term (-125% to 125%)


**Note:** The function of PID gain controls is outside the scope of this User's Guide.

### Autolevel Setup Menu ###
![http://i.imgur.com/gPbXhIr.jpg](http://i.imgur.com/gPbXhIr.jpg)

  * **Mode:** This setting determines how the stability is to be controlled. There six options.
  1. **OFF:** Always off
  1. **Autochan:** On when the channel set as "Autochan" in RC settings is more than the trigger setting.
  1. **Stabchan:** On when the channel set as "Stabchan" in RC settings is more than the trigger setting.
  1. **3-pos:** Stability on when the channel set as "3-pos" in RC settings is more than the stability trigger. Autolevel also on when the channel set as 3-pos in RC settings is more than the autolevel trigger. Make sure that your 3-position switch is set to -100%, 0% and 100% to reliably trigger with the default settings.
  1. **ON:** Always on
  1. **Hands free:** On when the sticks are released. The margin to determine when the sticks are released is determined by the Trigger.
  * **Trigger:** Stick position to enable (-125% to 125%)
  * **Roll P:** PID Proportional gain for the roll accelerometer
  * **Pitch P:** PID Proportional gain for the pitch accelerometer

![http://i.imgur.com/H4ihWxL.jpg](http://i.imgur.com/H4ihWxL.jpg)

  * **Roll trim:** For fine roll tuning of the autolevel.
  * **Pitch trim:** For fine pitch tuning of the autolevel.
  * **Launch delay:** Enable/disable launch mode. When enabled, the autolevel will be disabled for 10 seconds after it is triggered by a throttle position higher than set by "THR.pos". Please note that you can only start the timer once per flight. To reset the timer you have to reboot the KK2.0 board.
  * **THR.pos:** Throttle stick position to enable launch timer.

### Sensor Calibration Screen ###
![http://i.imgur.com/wC6ORmU.jpg](http://i.imgur.com/wC6ORmU.jpg)

The sensor inputs screen shows you the current values of all sensors.
Pressing button 4 "Cal." calibrates the accelerometers. The gyros are calibrated at start-up.

Pressing button 3 "Inv." will initiate an extra inverted calibration step. After doing the normal calibration above, invert the model and press button 3. This helps the software to more accurately detect the inverted attitude.

Pressing button 2 "Gyro" will recalibrate the gyros on the fly. This is useful if you find they have drifted due to temperature.

### Level Meter Screen ###
![http://i.imgur.com/Or0D6Uy.jpg](http://i.imgur.com/Or0D6Uy.jpg)

The level meter screen shows you a graphical representation of where the board considers "down" or "level" to be.

### Channel Mixing Menu ###
This menu has changed quite a bit from earlier builds.
Now you first select the output number that you wish to modify with the "Setting for:" item.
Once this is set to the right output number you may continue down the settings.

![http://i.imgur.com/pHJhhlr.jpg](http://i.imgur.com/pHJhhlr.jpg)

There is a pair of RC mixers on each output channel.
You can do delta, taileron, elevon etc mixing with OpenAero2 without needing special transmitter mixing functions.
  * **Source A:** Set the primary channel source to be mixed.
  * **Volume (%):** Set the amount of channel A required.
  * **Source B:** Set the secondary channel source to be mixed.
  * **Volume(%):** Set the amount of channel B required.

![http://i.imgur.com/DS1vsQv.jpg](http://i.imgur.com/DS1vsQv.jpg)

  * **Source mix:** Copies the RC source through to the mixer. Normally OFF. Advanced users only.
  * **Roll gyro:** Roll gyro signal enable/disable (Off/On). If you wish this output to respond to roll, enable this.
  * **Direction:** If the response is backwards to what you need, set this to Reversed.

![http://i.imgur.com/H0gNL8G.jpg](http://i.imgur.com/H0gNL8G.jpg)

  * **Pitch gyro:** Pitch gyro signal enable/disable (Off/On). If you wish this output to respond to pitch, enable this.
  * **Direction:** If the response is backwards to what you need, set this to Reversed.
  * **Yaw gyro:** Yaw gyro signal enable/disable (Off/On). If you wish this output to respond to yaw, enable this.
  * **Direction:** If the response is backwards to what you need, set this to Reversed.

![http://i.imgur.com/D9036DF.jpg](http://i.imgur.com/D9036DF.jpg)

  * **Roll acc:** Roll accelerometer signal enable/disable (Off/On). If you wish this output to respond to the roll accelerometer, enable this.
  * **Direction:** If the response is backwards to what you need, set this to Reversed.
  * **Pitch acc:** Pitch accelerometer signal enable/disable (Off/On). If you wish this output to respond to the pitch accelerometer, enable this.
  * **Direction:** If the response is backwards to what you need, set this to Reversed.

### Output Mixing Menu ###
The output mixers can be used to set up CCPM mixing for flybarless helicopters and other complex mixers.
This menu has changed quite a bit from earlier builds.
Now you first select the output number that you wish to modify with the "Setting for:" item.
Once this is set to the right output number you may continue down the settings.

![http://i.imgur.com/WjF1TMn.jpg](http://i.imgur.com/WjF1TMn.jpg)

  * **From:** This is the second source for the output mixer. This will be mixed with the primary.
  * **Volume (%):** The amount of the second channel to be send to the servo/ESC.
  * **From:** This is the third source for the output mixer. This will be mixed with the primary.
  * **Volume (%):** The amount of the third channel to be send to the servo/ESC.

![http://i.imgur.com/A6JJtHc.jpg](http://i.imgur.com/A6JJtHc.jpg)

  * **From:** This is the fourth source for the output mixer. This will be mixed with the primary.
  * **Volume(%):** The amount of the fourth channel to be send to the servo/ESC.

### Servo Direction Menu ###
You can reverse the direction of the servos for each channel here.

![http://i.imgur.com/zIi0hX1.jpg](http://i.imgur.com/zIi0hX1.jpg)

### Servo Trim (%) Menu ###
You can set the neutral position of each servo from this menu. Note that now the servo updates the new postion in real-time so that you can see the effect immediately.

![http://i.imgur.com/o34gGIO.jpg](http://i.imgur.com/o34gGIO.jpg)

### Neg. Servo Trvl(%) Menu ###
You can set the maximum negative throw of each servo from this menu. Note that now the servo updates the new postion in real-time so that you can see the effect immediately.

![http://i.imgur.com/LFOCCxn.jpg](http://i.imgur.com/LFOCCxn.jpg)

### Pos. Servo Trvl(%) Menu ###
You can set the maximum positive throw of each servo from this menu. Note that now the servo updates the new postion in real-time so that you can see the effect immediately.

![http://i.imgur.com/yE5a75i.jpg](http://i.imgur.com/yE5a75i.jpg)

### Failsafe Menu ###
![http://i.imgur.com/qbFeiNm.jpg](http://i.imgur.com/qbFeiNm.jpg)

  * **Mode:** There are two options: **Fixed** which simply transfers the settings from the "Failsafe positions" menu to the servos, and **Adv.** which uses the settings below. In Advanced mode, autolevel is enabled automatically.
  * **THR.pos %:** Throttle position when in advanced failsafe (in %). Note that the throttle position ranges from -100% (min) to 100% (max).
  * **Elevator:** Elevator trim when in advanced failsafe.
  * **Aileron:** Aileron trim when in advanced failsafe.
  * **Rudder:** Rudder trim when in advanced failsafe.

### Failsafe Positions Menu ###
You can set here a specific servo position that the servos will move to in Failsafe mode when Failsafe is set to "fixed".
Note that pressing "FS" on the Receiver input screen will load the current values into these settings.
Note also that when Failsafe is set to "Advanced", these settinsg will not be used as the settings on the "Failsafe settings" menu will be used instead.

![http://i.imgur.com/oVlTxWV.jpg](http://i.imgur.com/oVlTxWV.jpg)

### Battery Menu ###
![http://i.imgur.com/Kep5f5E.jpg](http://i.imgur.com/Kep5f5E.jpg)

  * **Battery type:** LiPo or NiMh, Used internally to preset the min and max cell volatges.
  * **Cells:** Number of cells in your battery pack. When you change this value, OpenAero2 will automatically recalculate the LVA voltage based on the number of cells and the minimum cell voltage.
  * **Alarm voltage:** This is the low battery alarm (LVA) setting. You can change this value at any time, but changing either the number of cells or the minimum cell voltage will over-write the LVA. "1000" equales to 10.00 volts.
  * **Max. cell mV:** Set this to the maximum cell voltage that you like to use. Defaults to 420 (4.2V) for LiPo.
![http://i.imgur.com/uSzYqpd.jpg](http://i.imgur.com/uSzYqpd.jpg)
  * **Min. cell mV:** Set this to the minimum cell voltage that you like to use. defaults to 360 (3.6V) for LiPo. When you change this value, OpenAero2 will automatically recalculate the LVA voltage based on the number of cells and the minimum cell voltage.

## Error messages ##
If an error or alarm is detected the buzzer will sound and a message will be displayed on the screen. There are five possible screen messages.

![http://i.imgur.com/bnM8R.jpg](http://i.imgur.com/bnM8R.jpg)
![http://i.imgur.com/MEX6v.jpg](http://i.imgur.com/MEX6v.jpg)
![http://i.imgur.com/zz3pz.jpg](http://i.imgur.com/zz3pz.jpg)

**Sensor Error:**
When the board is powered up it must remain still so that the gyros can be accurately calibrated. It the boards detects movement during calibration, this error will be displayed. To clear this error, repower the board while keeping the board still.

**Battery Low:**
If you are using the board to monitor the battery voltage, when the current battery voltage falls below the set alarm point, this message is shown.
If you are not planning to monitor the battery voltage, please set the LVA to zero.

**No Signal:**
This message will be displayed when the board detects a loss of RC signal.

![http://i.imgur.com/Oynh2.jpg](http://i.imgur.com/Oynh2.jpg)
![http://i.imgur.com/v397I.jpg](http://i.imgur.com/v397I.jpg)

**Lost Model:**
After the time period set for "LMA time", if no RC input is detected, the Lost Model buzzer will sound and this message will be displayed. The alarm can be cancelled with any stick input.

**Throttle High:**
PWM does not require a throttle input in OpenAero2. In CPPM mode however, the throttle signal is passed through the board and appears as a PWM signal on M1.
As a safety measure, if the board is powered up with the throttle set to more than about 5%, the oupts will be disabled, an alarm will sound and this message will be displayed.