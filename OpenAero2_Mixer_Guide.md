# Mixer User Guide #
## Channel Mixers ##
OpenAero2 has 12 mixing channels named OUT1 to OUT8, and PSU9 to PSU12.
The eight primary mixing channels are connected to the eight physical outputs OUT1 to OUT8 (often historically referred to as M1 to M8).
Each of these mixing channels can take input from two RC channels and all five sensor axis.
Also, all 12 of the outputs of these channels can be mixed together.

### Mixing RC channels ###
The channel mixers allow the initial mixing of two RC channels. This is used in the Flying Wing preset to automatically mix aileron and elevator for elevon functions.
### Mix volumes ###
Mix volumes can range from -100% to 100%. Selecting neagtive volumes inverts the RC signal to be mixed. Setting a volume of 50% reduces an RC signal by half.
Often, when combining RC channels, one or both must be reduced to 50% so as to not produce signals larger than those acceptible to OpenAero2.
### Source mix ###
Source Mix is an important switch. For RC signals that are directly related to the primary axis (aileron = roll, elevator = pitch, rudder = yaw) OpenAero2 will provide a stabilised version of the RC signal when in any stabilised modes. When in pass-through (Manual) mode the RC signals are passed straight through.
However, for all other signals, OpenAero2 does not provide a stabilised signal. In this case, selecting "Source Mix" ON copies the RC signal through in both stabilised and non-stabilised modes.
If you find that a signal doesn't pass through in stabilised mode, switching source mix ON will solve this problem.
### Mixing sensors ###
Any mixing channel can have any combination of sensors mixed into it.
Gyro sensors are mixed in if in a Profile that has stability ON. Accelerometer sensors are mixed in if in a Profile that has autolevel ON. Sensors can be either OFF, ON or REVERSED.

Note that in some rare cases it may be required to reverse a sensor, but in 99% of the cases, the need to reverse a preset sensor means that an RC input has been incorrectly reversed.
### Pseudo channels ###
To do some complex mixing, you may need to process an RC channel separately to the physical OUT1 to OUT8 channels. For instance, if you wish to post-mix an RC channel (such as flap) into some stabilised channels (like ailerons) you first need to set up a spare channel for that input. An example is given in the Output Mixer section below.
Pseudo channels are never set up by any of the presets and must be set up by the user.
## Output Mixers ##
The output mixers allow mixing of any three other channels into the selected channel. The output mixers always assume that the current channel is also included 100% into the mix.
### Cross-mixing ###
You are able to choose three additional channels to mix in with the primary channel. The advantage of mixing channels here instead of in the Channel mixers is that the output channels can each be stabilised for a particular axis prior to mixing. This will allow you to create for instance, a CCPM 120 degree flybarless stabilised controller.
### Mix volumes ###
You can select from -100% to 100%. Selecting negative volumes will reverse that output's value. Selecting a volume of 50% will reduce that output's value by half.
While the primary channel is included by default at 100%, you can remove this if necessary by cross-mixing a copy of the primary channel at -100%, which will remove it from the mix.
### Example: CROW mixing ###
CROW mixing, otherwise known as "Butterfly" mixes flap with aileron to provide a highly-effective and stable braking effect. As flaps are deployed downwards, ailerons are moved upwards.
The TX will need to provide a dedicated flap signal, usually activated by a three-position switch.
OpenAero2 defaults to OUT6 for the left aileron and OUT7 for the right aileron channel.
In this example we will use PSU9 to process the flap signal.
  1. Ensure that the flap signal has been calibrated to produce a zero value as shown in the RC inputs screen when the flap is fully retracted.
  1. First we must use the Channel mixer to select the TX channel that contains the flap signal. In the "Source A" for PSU9, select the TX channel containing the flap signal, and set its volume to 100%.
  1. Make sure all the sensors are switched OFF.
  1. While in the PSU9 channel mixer, set the "Source Mix" to ON. The flap signal from the TX will be directly copied to the output of PSU9.
  1. Switch now to the Output Mixer screen.
  1. In the Output Mixers for both OUT6 and OUT7 (ailerons) add PSU9 at +50% for initial testing. This will mix half the flap signal to the ailerons.
  1. Exit to the Status screen so that the outputs become active.
  1. Deploy the flaps and observe the ailerons. If deploying the flaps moves an aileron downwards instead of upwards, reverse the volume of PSU9 mixed into that particular channel. If the ailerons move too far upwards, reduce the percentage of the PSU volume. Conversely, if the ailerons do not lift up far enough, increase the PSU volume.