# Frequently asked Questions #
The [Getting Started Guide](OpenAero2_Getting_Started.md) describes the process of how to load OpenAero2 and connect the KK2 board between your receiver and servos.

## Basics ##
### What is OpenAero2? ###
OpenAero2 is a fully-featured aeroplane and camera stabilization software for [Hobbyking's KK2 multi-rotor board](http://www.hobbyking.com/hobbyking/store/uh_viewitem.asp?idproduct=24723), replacing the manufacturer's original [multi-rotor firmware](http://www.rcgroups.com/forums/showthread.php?t=1675613).

OpenAero can take up to eight RC input channels and mix them into any of 12 mixer channels. Each of these mixer channels can take input from any or all sensors, then be cross-mixed with up to three other outputs.

![http://i.imgur.com/icW16LJ.jpg](http://i.imgur.com/icW16LJ.jpg)

### Where can I purchase a KK2 board from? ###
The [KK2 multi-rotor board](http://www.hobbyking.com/hobbyking/store/uh_viewitem.asp?idproduct=24723) is manufactured and sold only by [HobbyKing](http://www.hobbyking.com/hobbyking/store/uh_viewitem.asp?idproduct=24723).

Additionally, a [USBasp](http://www.hobbyking.com/hobbyking/store/uh_viewitem.asp?idproduct=21321) from [Hobbyking](http://www.hobbyking.com/hobbyking/store/uh_viewitem.asp?idproduct=21321) is required to load the OpenAero2 software onto the KK2 board.

## Features ##
### What models can I stabilize? ###
OpenAero2 has been designed with two selectable model types, for easy setup to control both single and dual aileron aeroplanes or flying-wings.
Unlike other solutions, OpenAero2's on-board mixers are completely flexible to allow control of up to 12 individual channels or 8 directly connected servos.
As such, by setting up each of the mixing channels, you can control virtually any type of model.

#### Aeroplane Mixers ####
Aeroplane mode has the ability to stabilize all three axis, support the use of flaps or pre-mixed flaperons, with any receiver.
The stabilized outputs are simply referred to as aileron, elevator and rudder.

#### Flying Wing Mixers ####
Mixing for flying-wings is now supported by the on-board mixers, allowing OpenAero2 to stabilize the left flaperon, right flaperon and rudder (if fitted).

#### Custom Mixers ####
OpenAero2 has eight output channels which can all mix from five PWM RC channels in PWM mode, and eight RC channels in the other modes.
Each of the eight output channels can have any or all sensors mixed together. Each of the output channels can mix any two RC inputs together. Additionally, each of the eight output channels can cross-mix up to three of the other channels.
There are also four fully-functional pseudo channels (PSU9 to PSU12) which can be used for even more complex mixing.

### What flight modes are supported? ###
OpenAero2 offers multiple unique flight modes for aeroplanes, interchangeable remotely from the transmitter during flight, through a designated receiver channel.

1 - **Manual mode** - Deactivates all assistance from OpenAero2, passing all servo control signals from the receiver to the servos.

2 - **Stabilization mode** - Uses OpenAero2's gyro stabilization to reduce the effects of wind and air turbulence on the aeroplane. Both rate mode gyros and axis-locked gyros can be configured.

3 - **Auto-levelling mode** - Uses OpenAero2's accelerometers to return the aeroplane to level flight.

4 - **Axis-locked mode** - Locks (Heading hold) any or all axis to a particular attitude. This can be adjusted via the sticks for incredible 3D effects.

## Receivers ##
### Which receivers are supported? ###
The KK2 board supports common multi-wire PWM receivers, single-wire CCPM receivers and serial receivers suchas Futaba S.Bus, Spektrum Satellite and XPS Xtreme.
S.Bus and Satellite receivers require an inexpensive interface cable.
Binding of Satellite receivers is also supported natively.

## Camera Stabilization ##
### What camera gimbals can I stabilize? ###
In Camera stability mode, both gyros and accelerometers are enabled to stabilise 3-axis camera mounts. See the separate guides for setting up camera stability.

## Problems ##
### Why don't my servos move? ###
You have failed to supply a second source of power to the M2-M8 outputs. See a detailed explanation in the following section.

The [Getting Started Guide](OpenAero2_Getting_Started.md) describes the process of how to load OpenAero2 and connect the KK2 board between your receiver and servos.