# The OpenAero32 User Guide (Alpha 1) #
## What is OpenAero32? ##
OpenAero32 is an aeroplane stabilising software for the Naze32 AfroMini STM32-based boards. You put the board in between your receiver and the servos in your aircraft. When deactivated, OpenAero32 just passes the servo signals from your receiver to the servos. When activated, the gyros and accelerometers are used to move the servos to counteract any unwanted movement of the aircraft.

OpenAero32 has eight output channels which can all mix from eight RC channels in CPPM mode.


## What models can OpenAero32 be used with? ##
OpenAero32 currently supports only conventional, fixed-wing aircraft, and flying wings.
### Aeroplane mode ###
In aeroplane mode there are up to six outputs - Throttle, Left Aileron, Right aileron, Elevator, Rudder and Flap.
### Flying Wing mode ###
In flying wing mode there are up to four outputs - Throttle, Left Elevon, Right Elevon, and Rudder.

## How do I connect up my model? ##
### Receiver connections ###
For a standard multi-wire PWM receiver you need to use the extended PWM inputs of the Naze32 board.
If you have a CPPM receiver, just connect the single wire to the CPPM input.
The following are the default channel allocations. many can be changed from the command-line interface (CLI).
See the Naze32 user guide for guidance on the use of CLI mode.

  * CH1: Aileron signal input
  * CH2: Elevator signal input
  * CH3: Throttle signal input
  * CH4: Rudder signal input
  * CH5: Second aileron signal input
  * CH6: Flap signal input
  * CH7: Stability switch input
  * CH8: Autolevel switch input

### CPPM channel order ###
For CPPM mode to work correctly, you have to know the channel order of your transmitter. Two examples are shown below.
You can configure the channel order via the CLI.

**JR/Spektrum**
  * CH1: Throttle
  * CH2: Aileron #1
  * CH3: Elevator
  * CH4: Rudder
  * CH5: Gear switch
  * CH6: Aileron #2
  * CH7: Aux 1
  * CH8: Aux 2
**Futaba**
  * CH1: Aileron #1
  * CH2: Elevator
  * CH3: Throttle
  * CH4: Rudder
  * CH5: Gear switch
  * CH6: Aileron #2
  * CH7: Aux 1
  * CH8: Aux 2

### Servo connections ###
For conventional aeroplanes the connections are as follows:

Naze32
  * M1: Throttle 1
  * M2: Throttle 2 (unused)
  * M3: Left aileron
  * M4: Right aileron
  * M5: Rudder
  * M6: Elevator

AfroMini
  * PPM: CPPM input
  * THR: Throttle output
  * S1: Left aileron
  * S2: Right aileron
  * S3: Rudder
  * S4: Elevator

## Set up ##
The setup procedure is extremely important and must be followed to get good results from OpenAero32.

### New CLI commands and variables ###
OpenAero32 supports most of the Naze32 CLI commands but has many more for aeroplane functionality.

The following new variables can be set using the "set" command.

**defaultrc** - 1200 to 1700. The default "neutral" RC input position. Not related to RC stick zeros.

**flapmode** - 0 to 2. For setting Basic Flap, Premixed Flap and Advanced Flap modes. See below for more info.

**flapchan** - 0 to 7. The RC channel carrying flap info or, the RC channel to carry dummy flap info.

**aileron2** - 0 to 7. The RC channel carrying the second aileron info.

**flapspeed** - 0 to 50. The flap deflection speed. 0 is full speed. 50 is very slow.

**flapstep** - 1 to 10. The flap movement increment. Larger moves in larger steps.

**dynPIDchan** - 0 to 7. The channel to be used as the source for Dynamic PID gain adjustment.

**dynPIDbp** - 0 to 1999. The point above which Dynamic PID gain becomes active for that channel.

**sX\_trim** - 1200 to 1700. The servo trim where "[X](X.md)" can be 0 to 7 for each of the eight channels. Use to adjust the flying surface positions.

**sX\_min** - 1000 to 2000. The minimum servo travel point where "[X](X.md)" can be 0 to 7 for each of the eight channels.

**sX\_max** - 1000 to 2000. The maximum servo travel point where "[X](X.md)" can be 0 to 7 for each of the eight channels.

**roll\_rev** - -1 to 1. The global polarity of the gyro/acc roll PID influence.

**pitch\_rev** - -1 to 1. The global polarity of the gyro/acc pitch PID influence.

**yaw\_rev** - -1 to 1. The global polarity of the gyro/acc pitch PID influence.

There is also a new CLI command for calibrating. The usage is described below.

Typing **"cal gyro"** will recalibrate the gyros.

Typing **"cal acc"** will recalibrate the accelerometers and therefore the autolevel position.

Typing **"cal mag"** will recalibrate the magnetometer. You have to move the model through all axis within 30 seconds after initiating a mag cal, as per the normal Naze32 operation. Untested.

Typing **"cal sticks"** will zero all the RC inputs. This is very important as a last step after trimming the model on the ground. openAero32 uses the stick zeros to accurately extract roll and flap info from premixed flaperon data.

### RC setup ###
First of all, in the CLI type "defaults" to reset the board to the factory defaults.

Using the CLI set the receiver mode(PWM or CPPM) using the "feature" command as such:
To enable **CPPM type** "feature ppm".
To enable Spektrum receiver support type "feature SPEKTRUM".

To set the **airplane type**, type "mixer airplane" or "mixer flying\_wing".

To set the **CPPM channel order**, type "map AETR1234" where each character is the channel name/number. Note that the code defaults to the JR channel order of "TAER1234".

To set the channel to be used for the **second aileron**, type "set aileron2 (channel)" where (channel) is the channel number (0 to 7) of the second aileron.

To set the channel to be used for the **dedicated flap** (if required), type "set flapchan (channel)" where (channel) is the channel number (0 to 7) of the flap signal.
It is important to set this to an usused channel number if you are using pre-mixed flaperons. Any channel will do, even if your TX does not support it.

Most importantly, set the **flap mode** to the method you plan to use.
There are three modes.

**Basic Flap:** Use this mode when you only have one aileron channel.
The one aileron channel will be copied to both ailerons. Type "set flapmode = 0" to set this mode.

**Premixed flap:** Use this mode when you have two aileron channels, mixed with flaperons in the TX. Type "set flapmode = 1" to set this mode. If you do not set this mode with flaperon mixing enabled in the TX, OpenAero32 will not be able to accurately decode the roll and flap components from the signals.

**Advanced flap:** Use this mode when you have two independent aileron channels and a flap channel. OpenAero32 will mix these into on-board flaperons. Type "set flapmode = 2" to set this mode.

Type "save" afterwards to store the settings permanently.

Next, using nicodh's GUI, you can set the stability/autolevel RC activation and also confirm that all the RC and sensors are working correctly.

## Required tools ##
Android and Windows-based GUI by nicodh. You must use this tool to set the new variables and the RC activation methods.

> Android version: http://www.rcgroups.com/forums/showthread.php?t=1667520

> Windows version: http://www.rcgroups.com/forums/showthread.php?t=1667516

Terminal emulator
I use Hyperterm myself but Hercules is also recommended.
Set the serial data to 112kbaud, 8 bits, no parity.