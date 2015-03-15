Introduction

---

The NeXtcopter open source code for the KK boards was just a starting point for my ultimate goal: bringing the much-requested **auto-level** feature to KK-level boards.
Doing so was a long journey! Over the last few months I have been working on bringing the most current open-sourced KK software up to date. Along the way a number of great new features, some unique to NeXtcopter Plus, have been added.
These include:
- PID loop control on all axis (Just P for auto-level)
- Support for accelerometers and auto-level functions
- Custom MultiWii GUI interface for use with serial adapters or potentially bluetooth
- Simple LCD menu system for on-field adjustments without a laptop
- Low Voltage Alarm support for LEDs or piezo buzzer
- Support for PPM receivers with up to 8 channels

A number of changes have been made to the way the code runs:
- Motor control no longer dictates how much processing time is available
- ESC rate varies depending on the flight mode
- RC inputs were originally compressed into 8 bits but now are processed without loss of resolution

Here is a picture of my hand-made acc board add-on. The final version will use Dan's module and be much neater.
![http://i.imgur.com/A153J.jpg](http://i.imgur.com/A153J.jpg)

Here is an early picture of the GUI showing the gyro and accelerometer outputs being graphed on screen.
![http://i.imgur.com/YNhDO.jpg](http://i.imgur.com/YNhDO.jpg)

The code compiles to about 64.7% capacity of the Atmega168 so there is still much scope to experiment and add features
In order to simplify the code, currently it only supports Quad and Quad-X multicopters.
Hopefully, others may add support for the other types in time.

I hope to release the source code after final testing is done.
I'm a firmware developer, not a pilot so the code is largely untested and has a lot of tuning to be done.
I have flown my quad using this software in Quad-X configuration and it seems to fly well.

I need people to try this software out and provide feedback on tuning and improvements.
Any feedback is more than welcome.
I hope that this release is of interest to people interested in the code and to those keen to experiment!
Please note that the NeXtcopter Plus code is protected under the GNU General Public License v3.0.
Feel free to use the code as you like as long as the resultant code remains open source.

A quick picture of the LCD menu splash screen
![http://i.imgur.com/hZFV5.jpg](http://i.imgur.com/hZFV5.jpg)

Hardware requirements

---

**Will this code run on a regular KK board?**
Not fully. You need a few things on the board to set it up and for the accelerometer functions to work.
> - LCD or GUI interface. The code outputs both the LCD and GUI serial data out the MISO line.
> > You need a board with access to that pin on the interface header.

> - 2-axis Accelerometer. Required for the auto-level to work.
> > I have connected the X and Y outputs of the accelerometer to the Pitch and Roll gain pot inputs.
> > You have to disconnect the pot circuitry from these inputs.

> - Atmega168 or larger

You'll have to remove a couple of resistors in order for the accelerometer outputs to be made available.
![http://i.imgur.com/si9Oh.jpg](http://i.imgur.com/si9Oh.jpg)

**What do I need to get the GUI running?**
You need a MKUSB-type serial interface for the GUI to communicate to the board.
Because of the limitations of the KK board, a non-standard cable is required.

The connections you need are as follows:
From: KK+         To: MKUSB X3 port (Note X3 port **NOT** X2 port)
Pin 8 receive      Pin 8 transmit
Pin 9 transmit     Pin 3 receive
Pin 10 ground     Pin 10 ground
Pin 2   +5V        Pin 2   +5V

Here is a shot of my very simple serial cable.
![http://i.imgur.com/MFTP5.jpg](http://i.imgur.com/MFTP5.jpg)

It's a very simple cable and once you've played with the GUI for a while you'll love it.
Dan's KK+ boards have both the LCD and serial ports available and has made accelerometer connection easy.
This is what I have used to develop the code.

**Why can't I change the gyro direction?**
Why would you? There's only one hardware platform this code will run on and the gyro direction for that is well known.
If you really want to change the direction, just change the source code (in gyros.c).

**What do the Gain pots do?**
Roll Gain:	Not used
Pitch Gain:	Not used
Yaw Gain:	used to select ESC cailbration mode or Stick centering mode at power-on


How to set up your KK+ board to fly for the first time

---

The following procedure must be carried out in a windless environment.
1. Place the copter on a level surface.
2. Select "Calibrate acc" from either the GUI or the LCD menu.
3. Select "Center sticks" from either the GUI or the LCD menu.
4. Fly in normal mode, trim with TX until close to stable. This trims out Gyro drifts.
5. Land, switch to auto-level mode.
6. Hover and take note of the direction that the copter tends towards when hands off.
7. Land, adjust the Acc trim amounts in very small increments for both PITCH and ROLL.
8. Hover in auto-level again and see how the adjustment went.
9. Repeat until the copter is optimally set up.


About the Auto-level function

---

An auto-level feature is not magic nor is it an auto-pilot.
It will not make your copter hover magically on the spot.

What it will do is keep your copter close to level in the absence of pilot input and to a certain extent even with wind gusts.
You will still need to keep your copter under the same close control as you would always do.

Think of it like stability control fitted to many modern cars.

Using the LCD menu

---

You can use the standard LCD module that Dan supplies, or the Sparkfun serial LCD module.
The module must support 9600bd. Please bear in mind that if you accidentally have the LCD module attached as well as the GUI running (via the serial cable) the LCD could become corrupted by the stream of serial data, and may even reprogram it to a different interface speed. Do try and make sure the LCD is only used **away** from your PC.

To enter the LCD menu, first make sure that your LCD module is connected, then power up your copter.
Hold PITCH UP and YAW RIGHT simultaneously to enter the LCD menu. The LCD will display "NeXtcopter Menu".

The LCD menu is fairly self-explanatory:
Pitch forward/backwards to move up/down menu items.
Roll left/right to increase/decrease values. Note that there are four speeds depending on the stick position.
Yaw left to save the setting to EEPROM.
You have to save each entry you change. If you move to a new menu item before saving, your changes will be forgotten.

To exit the LCD menu, hold PITCH DOWN and YAW RIGHT simultaneously. The LCD will go blank and you can now power off and disconnect it. If you like you can leave it connected and fly with it like I do :)

Here is a video of the LCD menu in action.
http://www.youtube.com/watch?v=IyXbdcODpho

Using the GUI

---

The MultiWii GUI is a great tool for visualising the sensors on your copter and to not just set, but experiment with PID settings and simulate them on-screen. The limitations of the KK design mean that I have to use a software UART to transmit data back to the GUI. Various KK boards will run at different speeds so it is likely that your board will have to be tuned. Fortunately, with the GUI this is relatively easy. You only have to do the Autotune once, unless you re-flash the board, or click on the "Defaults" button, which resets the eeProm values. The procedure to auto-tune is detailed later in this document.

A recent shot of the GUI updated with auto-level settings
![http://i.imgur.com/KW3Fr.jpg](http://i.imgur.com/KW3Fr.jpg)

Please note that the MultiWii GUI is Java-based and quite CPU intensive. Using the GUI while running too many applications might cuse it to slow.

1. Make sure your MKUSB and cable are connected to your PC and the KK+.
2. Power up the KK+ board.
3. Run the MultiWiiConf\_Next.exe file. Make sure your COM port is listed in the GUI.
4. Click on the COM port and make sure it turns green.
5. Click on the START button and data should start to be transferred to the GUI.
> The graph will start to move to the right and the "Cycle Time" will increase.
> Try moving the KK+ board and see the gyro and acc outputs displayed on the graph.
> Note that unless you have previsously calibrated the accelerometers, these will not be displayed correctly.
6. Click on READ under "Flight parameters". Now, all the PID etc data from the KK+ is transferred to the screen.
7. You can modify the data by clicking on each box and moving the mouse left or right.
> To save any data, first STOP the graph, then click on WRITE.
8. You can now ARM the KK+ with your transmitter.
9. Increase the throttle. Now you can see the motor outputs start up as if the KK+ is flying.
> The motor outputs are disabled when in GUI mode, but please take care when arming your copter at **any time**.
> If you are not actually in GUI mode, then your copter will then be armed and dangerous, so to speak.

How to auto-tune:
1. Make sure your MKUSB and cable is connected to your PC and the KK+.
2. Power up the KK+ board.
3. Run the MultiWiiConf\_Next.exe file.
4. When it opens, select the COM port from the list on the left top. The port number should turn green.
> Do NOT click on anything else yet.
5. Click on the button labelled "KK". If you move the mouse away from the button, you will notice that it has turned orange.
> Please check that the TX LED of the MKUSB is flashing very fast.
6. On the transmitter, go into AUTOTUNE mode by holding both ROLL and YAW left for a few seconds.
> If you have some channels reversed, this may be different.
> You can tell that you are in AUTOTUNE mode because the LED will flash about 12-13 times.
> When the autotune has finished, the LED will flash one long flash.
> The software will save the successful numbers into eeProm. The autotune process takes less than 10 seconds.
7. Click on the button labelled "KK". If you move the mouse away from the button, you will notice that it has turned back to green.
> Please check that the TX LED of the MKUSB has STOPPED flashing.

Additional features of the GUI:
Stick centering: You can calibrate the stick center position with a single click on the "Calibrate Sticks" button.
Gyro calibration: You can calibrate the gyros with a single click on the "Calibrate Gyros" button.
Acc calibration: You can calibrate the accelerometers with a single click on the "Calibrate Acc" button.
LVA: You can see the current battery voltage, and set the low-voltage warning trigger level. You can also set it up to alert you via the buzzer output of via LEDs. In buzzer mode the buzzer is quiet normally and beeps rapidly when the battery voltage falls below the trigger level. In LED mode the output is ON normally and flashes slowly when the battery voltage falls below the trigger level.

Using the LVA:
The default mode for LVA is BUZZER mode and the trigger is 0 Volts.
While you can change the LVA mode and trigger at any time, the graph must be running to see the VBAT updating.
1. Click and drag the LVA numbers until the trigger is higher than your VBAT value.
2. Click on WRITE. The alarm should start beeping quickly.
3. Reduce the LVA value to less than the VBAT.
4. Click on WRITE. The alarm should stop beeping
5. Click on the MODE button to change the text to LED.
6. Click on WRITE. The alarm will sound continually, or the LEDs will be ON.
7. Click and drag the LVA numbers until the trigger is higher than your VBAT value.
8. Click on WRITE. The alarm should start beeping at 1Hz, or the LEDs will be flashing.

If you'd like to try this firmware, PLEASE be sure to run through all the power-up settings (particularly ESC calibration) before attaching props to your multicopter.

Power-up settings

---

Stick Centering:
Set YAW pot to maximum. All TX sticks and trims to centre. Power up. LEDs flash 5 times then one long flash when the settings are saved. Power down. Return pot to previous settings.

ESC throttle calibration:
Set YAW pot to minimum. Set TX throttle to maximum. Power up. Wait for the ESC signal (double beep). Set the TX throttle to zero. Wait for the ESC confirmation signal. Power off. Return pot to previous settings.

Stick settings

---

Arming/disarming is done in the usual way. Hold minimum throttle and move the Yaw stick fully left/right to arm/disarm. Check your radio for possible stick reversal.

A number of preset stick sensitivity flight modes are available. These are selectable by moving the pitch stick forward or backwards while arming.

Acro mode:   Pitch forward while holding left to arm. LED will flash 5 times then turn ON.
UFO mode:    Pitch back while holding left to arm. LED will flash 3 times then turn ON.
Normal mode: Pitch neutral while holding left to arm. LED will flash once then turn ON.

There are four more special modes that are selected via sticks.

Autolevel mode: YAW LEFT + ROLL RIGHT
Autotune mode:  YAW LEFT + ROLL LEFT <-- Only for tuning your KK board to the GUI. Do not fly in this mode.
LCD menu mode:	YAW RIGHT + PITCH FWD
Exit LCD menu:	YAW RIGHT + PITCH BACK

Videos

---

Sorry, I've been too busy to take many videos. Hope to have more up soon.
Auto-level torture test:
http://www.youtube.com/watch?v=TCGdvMnb_SI


What's next?

---

After some bug fixing and maybe a few suggestions, that's it. It's time to move on to other things.