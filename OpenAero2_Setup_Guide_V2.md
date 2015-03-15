# Setup Procedure #
## A word about Stability and Autolevel switching ##
Stability and autolevel modes are now selected via the Flight Profiles. There are three profiles to choose from. The default settings are as follows:

  * Profile 1: Pass-through. No stability or autolevel
  * Profile 2: Stability ON and autolevel OFF
  * Profile 3: Both stability and autolevel ON

Note that instead of the above, you could have three profiles, all with stability on, but with different PID values. Or you could have one profile with axis-locked gyros so that these can be switched mid-flight for 3D applications.

While any type of input is possible, a three-position switch is typically used to select the flight profile.
The way it works is this: For the switch position between -100% and the Profile 2 trigger, Profile 1 is selected. For the switch position between the Profile 2 trigger and the Profile 3 trigger, Profile 2 is selected. For the switch position above the Profile 3 trigger, Profile 3 is selected.

## Initial setup and calibration ##
The first time you set up OpenAero2 you will have to adjust a few things to suit your radio gear and particular usage.

### Step 1: Model type ###
  * **Mode:** Go into the General menu and change the Mode to that which suits your model. The default is Aeroplane. Note that if you choose instead "Flying Wing" you must disable flying wing mixing in your transmitter.
  * **Orientation:** Next set the Orientation to match the way your KK2 has be physically mounted in the model. See the Basic User Guide for the five possible orientations.

### Step 2: Receiver settings ###
  * **Mode:** Set the Rx mode to suit your radio. If you are using a conventional multi-wire receiver, start with "Rudder". If you have jitter issues or are unable to see any input data, please change this to the highest channel number that is connected to your receiver. CPPM users must select CPPM. Users that have Futaba S.Bus or Spektrum/OrangeRx Satellite receivers can connect these via a suitable cable. XPS Xtreme format is also supported directly.
  * **Ch.order.:** If you are using CPPM, also select the channel order between either JR/Spektrum or Futaba. If you are using a Satellite receiver, be sure to select "Satellite" as the channel order. Please remember that CPPM inputs use the Rudder input and the Serial formats use the Throttle input.
  * **Profile Chan.:** Select the RC channel to that which you plan to control the flight modes. Typically this will be a switch.
  * **2nd Ail. Chan.:** If your radio carries a second, unique aileron channel set that here too.

### Step 3: Stick polarity ###
  * Follow the on-screen instructions to have OpenAero2 automatically adjust the RC input polarity to suit.

### Step 4: RC check ###
  * Go to the Receiver inputs screen and switch on your transmitter and receiver. If set up correctly, you should immediately see values appearing in both columns next to each connected channel.
  * Make sure all the tranmitter channels are centered including the throttle, and press button 4, under the screen label "Cal." This performs a stick centering calibration.
  * The right-hand column should now all be zero and change +/-1000 or so at the extremes of throw.

**EXTREMELY IMPORTANT**
Make sure that when you have both sticks away from you and to the right, all values are POSITIVE.
If they are negative, then you have failed to use the "Stick polarity" screen to set them for you. Please go back and do Step 3.

### Step 5: Flight profiles ###
  * Initially you should use the default settings. Details on how to adjust these settings are discussed elsewhere.
  * You may need to adjust the **Profile trig.** settings of each profile so that you can reliably switch between them. The status menu displays the current profile number so you can very easily see which profile is selected in real time.

### Step 6: Sensor calibration ###
  * Go into the sensor screen where you can calibrate the autolevel.
  * Place the model on a flat surface and chock up the tail or nose until it is level and in its normal flying attitude.
  * Keeping the model perfectly still, carefully press button 4 under the screen marker "Cal." After a short pause the LED will flash.
  * If you wish the inverted performance to be optimal, at this point you can flip the model over into the inverted position and press button 3 "Inv.". You cannot redo this until you redo the initial calibration with button 4.

### Step 7: Level meter ###
  * If calibrated correctly the level meter will show the black circle inside the square when the model is held in its flying attitude.

### Step 8: Channel mixing ###
  * For now, leave these settings at their defaults.

### Step 9: Output mixing ###
  * For now, leave these settings at their defaults.

### Step 10: Servo direction ###
  * Once returned to the status menu, the servos should be activated and you should be able to control all outputs with the transmitter. Please make sure that stability and autolevel are both OFF (Profile 1) to continue. The selected profile can easily be seen in the status menu.
  * Move all surfaces and check that they move as expected. If they are reversed, you must reverse the channels in the Servo direction screen. **Important: Do not reverse the servos in the TX**.

### Step 11: Servo trim ###
  * You can now adjust the neutral point of every output from the Servo trim screen to trim each of the flying surfaces. The servo will move in real time to indicate the trim position.
  * Note that it is always advisable to mechanically adjust the flying surfaces as close as possible so that excessive electronic trimming is not required.

### Step 12: Neg. Servo trvl.(%) ###
  * If you like, you can now adjust the minimum point of every output from this screen. The servo will move in real time to indicate the travel limit. This is useful to prevent the servos from over-extending when stabilisation or autolevel movements are added to your RC control inputs.

### Step 13: Pos. Servo trvl.(%) ###
  * If you like, you can now adjust the maximum point of every output from this screen. The servo will move in real time to indicate the travel limit. This is useful to prevent the servos from over-extending when stabilisation or autolevel movements are added to your RC control inputs.

### Step 14: Stability check ###
  * Switch to flight profile 2 (Stability on).
  * Pitch the nose of the model up and down while watching the elevator. The elevator should move to counteract the pitching. Pitching the nose UP should make the elevator move DOWN.
  * Roll the model left and right while watching the ailerons. They should also move to counteract the rolling.
  * Finally, yaw the model left and right while observing the rudder.

### Step 15: Auto-level check ###
  * Switch to flight profile 3 (Stability and Autolevel on).
  * Tilt the model in pitch and roll while watching the respective surfaces. They should move slightly to compensate and remain in that position.
  * The completes the basic setup process.

## Every time you power up ##
There are some important things to note when starting OpenAero2.

### Gyro calibration ###
The gyros are calibrated automatically when the model is powered up.
It is important that the model is held very still while the gyro calibration is taking place.
If you move the model during calibration the "Sensor Error" message will appear with a warning beep. The only option is to repower the board.


### Safety ###
Always make sure your throttle is set to minimum before attempting to power up your model. **Warning:** Even with the transmitter throttle set to zero, there may be a momentary output from the KK board to the throttle channel which may cause the motor to turn very briefly. As such stay well away from the motor and propellor and sure your model when powering it up.