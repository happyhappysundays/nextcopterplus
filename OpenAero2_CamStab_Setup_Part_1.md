[![](http://i.imgur.com/uaRIu.png)](https://sites.google.com/site/openaerowiki/home)

Please ensure you read the manuals in the following order.

  1. [Frequently Ask Questions](OpenAero2_FAQ.md)
  1. [Getting Started Guide](OpenAero2_Getting_Started.md)
  1. [Aeroplane - Quick Setup Guide](OpenAero2_UserGuide_V1.md)
  1. [Aeroplane - Advanced User Guide](OpenAero2_Adv_UserGuide_V1.md)
  1. [Camera - Standalone Guide](OpenAero2_CamStab_Setup_Part_1.md)
  1. [Camera - Receiver Control Guide](OpenAero2_CamStab_Setup_Part_2.md)

Please post any questions and tips on [RC Groups](http://www.rcgroups.com/forums/showthread.php?t=1708175).

# OpenAero2 Camera Stabilisation Setup Guide #
# Part 1 - Stabilisation only #

This guide assumes that you only plan to use OpenAero2 for camera stability and not for controlling the flight of an aircraft.
While it is possible to do both, there are limitations. This will be covered in a separate guide.

There will also be a separate guide explaining how to set up OpenAero2 to allow the camera mount to be controlled during stabilisation.

**Note:** The features described in this document are only available in Openaero2 version V1.0. An updated document will cover V1.1.

## Step 1: Set Camstab mixer mode ##

There is a preset mixer stored in the software that will set up the majority of camera mixer settings for you.
In the General Menu, change "Mode" to "Camera Stab.".
Note that this will reset any manual mixer settings that you may have made.

## Step 2: Set up Camstab independent mode ##

For the software to run without an external RC inputs we first need to switch on Camera Stabilisation mode.
In the General Menu, change "Camera Stab." to ON.

## Step 3: Set servo speed ##

The default servo rate is 50Hz. This is suitable for all servos, and is the only rate you should use with analog servos.
If you have digital servos that can handle 300Hz update rate, you can increase the servo rate.
In the General Menu, change "Servo rate:" to High.
Note that using the "High" setting with analog servos may cause them to burn out.

## Step 4: Set Stability and Autolevel modes for testing the gyros. ##

As the camera stability is to be completely autonomous, we need to switch on both stability and autolevel modes manually.
Go into the "Stability control" menu and change the "Mode:" to ON.
We need to set up the gyros first so to switch off the accelerometers we need to turn Autolevel OFF.
Go into the "Autolevel control" menu and change the "Mode:" to OFF.


## Step 5: Set up gyros ##

Depending on how many axis your camera mount can control, the next couple of steps will vary.
We need to confirm that both the gyro and accelerometer that will be controlling this channel are in the correct polarity for your mount.

Connect ONE axis of the mount to the KK2 board according to the list below.
Note that M8 is closest to the corner of the board and the right-most button.

  * M2 Pitch (Tilt)
  * M3 Yaw	(Pan)
  * M4 Roll (Roll)

Once connected, the servo should move to approximately the center position.
Make sure you are in the Status Menu to test the servos.
If it is not in the desired position we will adjust that later on.

## Step 6: Test gyro polarity ##

If the Camstab mixer has been loaded, only one gyro per output has been enabled and all accelerometers are OFF.
This is to help setup, but is not sufficient to control the camera mount.

Assuming that there are valid P-term gain values set in the Stability control menu, moving the mount **in the axis being set up** will casue the mount to respond briefly. If the response is too small to determine the direction, increase the P-gain value for that gyro in the Stability control menu.
Move the mount swifty in **one** direction on the relevant axis. For example, if testing the roll axis, roll the mount abruptly to the right.
Take note of the direction the mount moves. If it moves in the **same** direction as your movement, the gyro will need to be reversed. If it moves in the **opposite** direction, then you may proceed with the next steps.

To reverse a gyro, go into the mixer setup menu for the channel to which you are connected (e.g. M2 for the Pitch/Tilt axis).
Change the polarity of the relevant gyro to "Reversed".
Return to the Status menu to confirm that the gyro direction is now correct.

Remove the servo connection that you have just set up and connect the next axis.
Repeat steps 5 and 6 above until the gyros of all axis have been tested and set up correctly.


## Step 7: Set up accelerometers ##

The next step is to set up the accelerometers for the pitch and roll axis.
To make this easier we have to switch off the gyros.

Go into the "Stability control" menu and change the "Mode:" to OFF.
Go into the "Autolevel control" menu and change the "Mode:" to ON.

Remove all servo connections and re-insert the first axis you wish to stabilise for accelerometers (Pitch or Roll).


## Step 8: Test accelerometer polarity ##

Assuming that there are valid P-term gain values set in the Autolevel control menu, tilting the mount **in the axis being set up** will casue the mount to move in one direction.
Take note of the direction the mount moves. If it moves in the **same** direction as your movement, the accelerometer will need to be reversed. If it moves in the **opposite** direction, then you may proceed with the next steps.

To reverse an accelerometer, go into the mixer setup menu for the channel to which you are connected (e.g. M2 for the Pitch/Tilt axis).
Change the polarity of the relevant ACC to "Reversed".
Return to the Status menu to confirm that the accelerometer direction is now correct.

Remove the servo connection that you have just set up and connect the next axis.
Repeat step 7 above until both axis have been tested and set up correctly.

Once this is done, reconnect ALL servos to the KK2 board.

## Step 9: Tuning the accelerometers ##

At this point, you have accelerometers set up but the gyros are off and will remain off until the accelerometers have been scaled correctly to suit your mount.
With the mount and camera connected rotate the mount for the axis in question and observe how the mount is moved to compensate.
It may help to remove the servo connection for the other axis while you tune this axis.
More than likely the mount will move either not far enough, or too far to compensate for the movement.
To adjust this, go into the "Autolevel control" menu and change the "Acc P:" setting.
Increasing the value will increase the amount of movement.
Adjust until moving the mount results in a movement that makes the mount stay level for that particular axis.
Repeat for any other accelerometer-controlled axis you may have (Pitch or Roll only).

## Step 10: Tuning the Gyros (Pitch and Roll) ##

Go into the "Stability control" menu and change the "Mode:" to ON.
This will re-enable the gyros.

The gyros are used to counter the slow action of the dampened accelerometers.
Using moderate amounts of gyro gain, you may be able to find a setting that moves quickly enough to counter fast movement and does not overshoot the target position.

It may help to remove the servo connection for the other axis while you tune this axis.

With the mount and camera connected rotate the mount for the axis in question and observe how the mount is moved to compensate.
The mount may initially move too far to compensate for the movement, or jerk forward suddenly.
To adjust this, go into the "Stability control" menu and change the "P Gain:" setting for the axis under test.
Reducing the value will reduce the amount of overshoot.

Adjust until moving the mount results in an optimal movement for that particular axis.
Repeat for any other accelerometer-controlled axis you may have (Pitch or Roll only).


## Step 11: Tuning the Gyros (Yaw) ##

There is no equivalent Yaw accelerometer so the Yaw or Pan axis is stabilised by gyro only.

With the mount and camera connected rotate the yaw axis and observe how the mount is moved to compensate.
To adjust, go into the "Stability control" menu and change the "P Gain:" setting for the yaw axis.
Reducing the value will reduce the amount of overshoot.

Adjust until moving the mount results in an optimal movement for the yaw axis.


## Step 12: Set presets ##

To point the camera at your desired position without RC inputs you have to set the roll, pitch and yaw servos to a preset position.

When you select Camstab mixer mode,
  * PRESET 1 is assigned to the Pitch (Tilt) output
  * PRESET 2 is assigned to the Yaw (Pan) output
  * PRESET 3 is assigned to the Roll output

A preset position of "0" should be the neutral servo position.

To alter the presets, go to the RC Setup menu and scroll down to the presets.
Select the preset that relates to the ouput you wish to change and set the preset within the range -125% to 125%.
You will have to return to the Status screen to see the results.


## Step 13: Completion ##

You have now set up basic stabilisation for all axis.
For advanced heading-hold settings, see the separate guide.