OpenAero2\_Transitional\_User\_Guide
# Transitional mixing in OpenAero2 V1.3 #

## Part 1 ##
You have separate servos for main tilt, and fractional tilt.
Due to the nature of OpenAero2's transition mode, you have to use OUT5, 6, 7, 8 for one profile and PSU9, 10, 11 and 12 for the other.
Just bear that in mind.
So let's start with OUT5 (Left motor speed).
Set the Channel mixing for OUT5 as per the .pdf file (80% throttle and 20% aileron).
Exit to the Status menu and with the ESC into OUT5, try it out.
The motor should increase speed with throttle and increase/reduce a little with aileron input.
Do exactly the same thing with OUT6 but -20% aileron.
OUT6 should behave the same with throttle but opposite with aileron input.
OUT7 is the left motor tilt servo. I assume that you know that you have to power OUT2 to OUT8 with a separate 5V supply for servos to work? Fortunately you have two motors (with ESCs) plugged into OUT5 and OUT6 so they will power your servos.
The .pdf does not show separate servos for tilt which is clearly a mistake.
OUT7 should have 50% Elevator and 50% Rudder.
Similarly, OUT8 should have 50% Elevator and -50% Rudder.
If these give too much movement, you might have to reduce the percentages.
So now, when you move the elevator the tilt servos should also move together slightly back and forward. If not, reverse the volume percentage until they do.
Also, when you move the rudder, the tilt servos should move opposite each other. If not, reverse the volume percentage until they do.
Once you've gotten the controls moving the right way you can start switching on the gyros.
In Profile 1 (Hover) you want the roll gyros to affect the motor speeds on OUT5 and OUT6.
You want the pitch gyros to control the common forward and back tilt on OUT7 and OUT8.
You want the yaw gyros to control the opposing forward and back tilt on OUT7 and OUT8.
To do this, go into Profile 1, switch Stability ON, all gyros in RATE mode, no I terms and default P terms.
Then go into the Channel Mix menu for OUT5 to OUT8 and switch on the gyros and test each one at a time.
Turning them all on will be confusing!
So, switch on OUT5 and OUT6's Roll gyro.
Power and throttle up and roll the model back and forth.
The motors should speed up/slow down.
What you want, of course is the falling motor to speed up and the rising one to slow down.
Play with the polarity of the roll gyro settings of OUT5 and OUT6 (they go OFF/ON/REV) until they do.
Next, switch on the Yaw gyros for OUT7 and OUT8.
Power and throttle up and yaw the model back and forth on the yaw axis.
The tilt servos will move as you yaw the model.
What you want, of course is the servos to tilt forward if the yaw is pulling it back, and backwards if the yaw is pushing it forward.
Play with the polarity of the yaw gyro settings for OUT7 and OUT8 until they do.
Finally, switch on the Pitch gyros for OUT7 and OUT8.
Power and throttle up and pitch the model back and forth on the pitch axis.
The tilt servos will move as you pitch the model.
To be honest I am not sure how bicopters work in pitch stability but they should move together to stabilise.
Play with the polarity of the pitch gyro settings for OUT7 and OUT8 until they work correctly.
Now, with a lot of fiddling, you should be able to hover and move about in HOVER mode (profile 1).
I imagine it will take a lot of adjustment to be stable.

## Part 2 ##
OK, once hover mode is working in Profile 1, you should try to set up FF mode in profile 1.
It's easier to test by just writing over the Hover settings in Profile 1
Transition mode needs the alternate mix to be set up in PSU9 to 12 but that makes it hard to test.
Firstly, copy everything you have done for hover mode down onto paper so you don't lose it.
Alright....
In FF mode OUT5 and 6 will respond mainly to throttle but also a little bit of rudder.
So for OUT5, use 80% throttle and 20% rudder, Source mix ON.
For OUT6 use 80% throttle and -20% rudder, source mix on.
With stability OFF test it out. The throttle will control the motors, but also rudder stick will make the model yaw a bit as the motors will speed up/down to yaw the model. 20% could be too much...
OUT7 and 8 are the motor tilts and will move together for pitch control and opposite for roll I imagine.
So, for OUT7, mix 20% elevator and 20% aileron.
For OUT8, mix 20% elevator and -20% aileron.
Try it out and reverse the volumes if the controls move the wrong way.
That's all the control part sorted, now the stability.
Turn on the gyros as follows:
OUT5 Yaw gyro ON
OUT6 Yaw gyro ON (maybe reversed)
OUT7 Both Pitch and Roll gyros ON
OUT8 Both Pitch and Roll gyros ON
Again you'll have to switch stability on and test each axis so that the motors and servos move to oppose the movement. Just watch the motors as yawing the model might cause one or both to spin up even with no throttle.
Once you've set the gyro polarities correct your model should be ready to fly horizontally under stability control.

## Part 3 ##
So finally we come to the cutting edge... the transition. Not very well tested and you are the first to actually attempt to use it fully :D
Make sure your profile switch is set up to switch between Profiles 1 and 2. Test it to be sure.
Then change the model type to "Transition".
Copy all the hover settings into Profile 1 for OUT5, 6, 7 and OUT8 as we did before.
Copy all the FF settings into Profile 2 but into the outputs as follows:
OUT5 becomes PSU9
OUT6 becomes PSU10
OUT7 becomes PSU11
OUT8 becomes PSU12
If the transition speed is set to zero (0) then you can control the transition manually with a knob that is linked to the Profile select channel. Otherwise you have to set the transition speed to a value and when you switch profiles, it will (over a period of a few seconds) go from one mix (hover) to the other (FF).
It's hard to see what is happening as the status display will show "Profile 3" the whole time.
So set the transition speed to 1 (the fastest) and switch between the two profiles while wiggling the rudder stick.
In hover, the rudder will rock the motor mounts back and forward but in FF the rudder will control the motor speeds. You should see the model fade from one to the other and back as you switch the modes.
Once you've gotten used to that I would recommend tying the actual tilt mechanism (not currently plugged into the KK2) into the profile switch so that the model will automatically transition as you tilt the wing. That's the whole point of setting the transition speed to 0 and tying it to an analog channel that controls the wing tilt.
Well it is complex but I hope most of the above makes sense.