#include "board.h"
#include "mw.h"

uint8_t numberMotor = 0;
uint8_t useServo = 0;
int16_t motor[MAX_MOTORS];
int16_t servo[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

static motorMixer_t currentMixer[MAX_MOTORS];

static const motorMixer_t mixerTri[] = {
    { 1.0f,  0.0f,  1.333333f,  0.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f,  0.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f,  0.0f },     // LEFT
};

static const motorMixer_t mixerQuadP[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
    { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
    { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
};

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

static const motorMixer_t mixerBi[] = {
    { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
    { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
};

static const motorMixer_t mixerY6[] = {
    { 1.0f,  0.0f,  1.333333f,  1.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f, -1.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f, -1.0f },     // LEFT
    { 1.0f,  0.0f,  1.333333f, -1.0f },     // UNDER_REAR
    { 1.0f, -1.0f, -0.666667f,  1.0f },     // UNDER_RIGHT
    { 1.0f,  1.0f, -0.666667f,  1.0f },     // UNDER_LEFT
};

static const motorMixer_t mixerHex6P[] = {
    { 1.0f, -1.0f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -1.0f, -0.866025f, -1.0f },     // FRONT_R
    { 1.0f,  1.0f,  0.866025f,  1.0f },     // REAR_L
    { 1.0f,  1.0f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f, -0.866025f,  1.0f },     // FRONT
    { 1.0f,  0.0f, 0.866025f, -1.0f },     // REAR
};

static const motorMixer_t mixerY4[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
    { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
};

static const motorMixer_t mixerHex6X[] = {
    { 1.0f, -0.866025f,  1.0f,  1.0f },     // REAR_R
    { 1.0f, -0.866025f, -1.0f,  1.0f },     // FRONT_R
    { 1.0f,  0.866025f,  1.0f, -1.0f },     // REAR_L
    { 1.0f,  0.866025f, -1.0f, -1.0f },     // FRONT_L
    { 1.0f, -0.866025f,  0.0f, -1.0f },     // RIGHT
    { 1.0f,  0.866025f,  0.0f,  1.0f },     // LEFT
};

static const motorMixer_t mixerOctoX8[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f,  1.0f,  1.0f },          // UNDER_REAR_R
    { 1.0f, -1.0f, -1.0f, -1.0f },          // UNDER_FRONT_R
    { 1.0f,  1.0f,  1.0f, -1.0f },          // UNDER_REAR_L
    { 1.0f,  1.0f, -1.0f,  1.0f },          // UNDER_FRONT_L
};

static const motorMixer_t mixerOctoFlatP[] = {
    { 1.0f,  0.707107f, -0.707107f,  1.0f },    // FRONT_L
    { 1.0f, -0.707107f, -0.707107f,  1.0f },    // FRONT_R
    { 1.0f, -0.707107f,  0.707107f,  1.0f },    // REAR_R
    { 1.0f,  0.707107f,  0.707107f,  1.0f },    // REAR_L
    { 1.0f,  0.0f, -1.0f, -1.0f },              // FRONT
    { 1.0f, -1.0f,  0.0f, -1.0f },              // RIGHT
    { 1.0f,  0.0f,  1.0f, -1.0f },              // REAR
    { 1.0f,  1.0f,  0.0f, -1.0f },              // LEFT
};

static const motorMixer_t mixerOctoFlatX[] = {
    { 1.0f,  1.0f, -0.5f,  1.0f },          // MIDFRONT_L
    { 1.0f, -0.5f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f, -1.0f,  0.5f,  1.0f },          // MIDREAR_R
    { 1.0f,  0.5f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  0.5f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f, -0.5f, -1.0f },          // MIDFRONT_R
    { 1.0f, -0.5f,  1.0f, -1.0f },          // REAR_R
    { 1.0f,  1.0f,  0.5f, -1.0f },          // MIDREAR_L
};

static const motorMixer_t mixerVtail4[] = {
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -0.0f },          // FRONT_L
};

// Keep this synced with MultiType struct in mw.h!
const mixer_t mixers[] = {
//    numberMotor,useServo,motorMixer_t *motor
    { 0, 0, NULL },                // entry 0
    { 3, 1, mixerTri },            // MULTITYPE_TRI
    { 4, 0, mixerQuadP },          // MULTITYPE_QUADP
    { 4, 0, mixerQuadX },          // MULTITYPE_QUADX
    { 2, 1, mixerBi },             // MULTITYPE_BI
    { 0, 1, NULL },                // * MULTITYPE_GIMBAL
    { 6, 0, mixerY6 },             // MULTITYPE_Y6
    { 6, 0, mixerHex6P },          // MULTITYPE_HEX6
    { 2, 1, NULL },                // * MULTITYPE_FLYING_WING
    { 4, 0, mixerY4 },             // MULTITYPE_Y4
    { 6, 0, mixerHex6X },          // MULTITYPE_HEX6X
    { 8, 0, mixerOctoX8 },         // MULTITYPE_OCTOX8
    { 8, 0, mixerOctoFlatP },      // MULTITYPE_OCTOFLATP
    { 8, 0, mixerOctoFlatX },      // MULTITYPE_OCTOFLATX
    { 2, 1, NULL },                // * MULTITYPE_AIRPLANE
    { 0, 1, NULL },                // * MULTITYPE_HELI_120_CCPM
    { 0, 1, NULL },                // * MULTITYPE_HELI_90_DEG
    { 4, 0, mixerVtail4 },         // MULTITYPE_VTAIL4
    { 0, 0, NULL },                // MULTITYPE_CUSTOM
	{ 2, 1, NULL },                // * MULTITYPE_FW_DRAG
};

void mixerInit(void)
{
    int i;

    // enable servos for mixes that require them. note, this shifts motor counts.
    useServo = mixers[cfg.mixerConfiguration].useServo;
    // if we want camstab/trig, that also enables servos, even if mixer doesn't
    if (feature(FEATURE_SERVO_TILT))
        useServo = 1;

	// Copy from custom mixer if selected
    if (cfg.mixerConfiguration == MULTITYPE_CUSTOM) {
        // load custom mixer into currentMixer
        for (i = 0; i < MAX_MOTORS; i++) {
            // check if done
            if (cfg.customMixer[i].throttle == 0.0f)
                break;
            currentMixer[i] = cfg.customMixer[i];
            numberMotor++;
        }
    } 
	// Copy from non-NULL, non-Custom mixers
	else 
	{
        numberMotor = mixers[cfg.mixerConfiguration].numberMotor;
        // Copy motor-based mixers if defined (not NULL)
        if (mixers[cfg.mixerConfiguration].motor != NULL) {
            for (i = 0; i < numberMotor; i++)
                currentMixer[i] = mixers[cfg.mixerConfiguration].motor[i];
        }
    }
}

void mixerLoadMix(int index)
{
    int i;

    // we're 1-based
    index++;
    // clear existing
    for (i = 0; i < MAX_MOTORS; i++)
        cfg.customMixer[i].throttle = 0.0f;

    // do we have anything here to begin with?
    if (mixers[index].motor != NULL) {
        for (i = 0; i < mixers[index].numberMotor; i++)
            cfg.customMixer[i] = mixers[index].motor[i];
    }
}

void writeServos(void)
{
    if (!useServo)
        return;

    switch (cfg.mixerConfiguration) {
        case MULTITYPE_BI:
            pwmWriteServo(0, servo[4]);
            pwmWriteServo(1, servo[5]);
            break;

        case MULTITYPE_TRI:
            pwmWriteServo(0, servo[5]);
            break;

        case MULTITYPE_AIRPLANE:
            pwmWriteServo(0, servo[0]); // Left aileron
            pwmWriteServo(1, servo[1]); // Right aileron
            pwmWriteServo(2, servo[2]); // Rudder
            pwmWriteServo(3, servo[3]); // Elevator
            pwmWriteServo(4, servo[4]); // Flap
            break;

        case MULTITYPE_FW_DRAG:
            pwmWriteServo(0, servo[0]); // Left elevon
            pwmWriteServo(1, servo[1]); // Right elevon
            pwmWriteServo(2, servo[2]); // Left drag rudder
            pwmWriteServo(3, servo[3]); // Right drag rudder
			break;

        case MULTITYPE_FLYING_WING:
            pwmWriteServo(0, servo[0]); // Left elevon
            pwmWriteServo(1, servo[1]); // Right elevon
			pwmWriteServo(2, servo[2]); // Rudder
            break;	

        case MULTITYPE_GIMBAL:
            pwmWriteServo(0, servo[0]);
            pwmWriteServo(1, servo[1]);
            break;

        default:
            // Two servos for SERVO_TILT, if enabled
            if (feature(FEATURE_SERVO_TILT)) {
                pwmWriteServo(0, servo[0]);
                pwmWriteServo(1, servo[1]);
            }
            break;
    }
}

extern uint8_t cliMode;

void writeMotors(void)
{
    uint8_t i;

    for (i = 0; i < numberMotor; i++)
        pwmWriteMotor(i, motor[i]); // Put a breakpoint here and check numberMotor
}

void writeAllMotors(int16_t mc)
{
    uint8_t i;

    // Sends commands to all motors
    for (i = 0; i < numberMotor; i++)
        motor[i] = mc;
    writeMotors();
}

static void airplaneMixer(void)
{
	static int16_t flaperons = 0;
	static int16_t flap = 0;
	static int16_t left_roll = 0;
	static int16_t right_roll = 0;
	static int16_t slowFlaps = 0;
	static uint8_t flapskip;
	uint8_t speed;

	// Throttle is handled separately here
	motor[0] = rcData[THROTTLE];	  				// Send directly from RC for now
	motor[1] = rcData[THROTTLE];	  				// Copy to motor[0] for now (not fitted to Afro-mini)

	// Recover flap and aileron info
	switch(cfg.flapmode)
	{
		// No flaperons - one aileron channel copied to both
		case BASIC_FLAP:
			left_roll = rcCommand[ROLL];
			right_roll = left_roll;
			// Use RC flap channel if set
			if (cfg.flapchan != NOCHAN)
			{
				flap = rcCommand[cfg.flapchan];
			}
			// Zero if no RC flap channel
			else flap = 0;
			break;
		
		// Flaperons - two ailerons with flaps pre-mixed in the TX
		case PREMIXED_FLAP:
			// Select left/right aileron channels
			left_roll 	= rcCommand[ROLL];
			right_roll 	= rcCommand[cfg.aileron2];
			// Select flap signal decoded from flaperons
			flap = rcCommand[cfg.flapchan]; 	// Get flap data
			break;
		
		// Flaperons - two independant aileron channels + one flap input on cfg.flapchan
		case ADV_FLAP:
			left_roll = rcCommand[ROLL];
			right_roll = rcCommand[cfg.aileron2];
			// Ignore if no flap channel
			if (cfg.flapchan != NOCHAN)
			{
				flap = rcCommand[cfg.flapchan];
			}
			else flap = 0;
			break;
		
		default:
			break;
	}

	// Do flap speed control
	if (cfg.flapspeed) 
	{
		if (abs(slowFlaps - flap) >= cfg.flapstep)	// Difference larger than one step, so ok
		{
			speed = cfg.flapstep;					// Need to manipulate speed as target approaches										
		}
		else
		{
			speed = 1;								// Otherwise this will oscillate
		}

		if ((slowFlaps < flap) && (flapskip == cfg.flapspeed))
		{
			slowFlaps += speed;
		} 
		else if ((slowFlaps > flap) && (flapskip == cfg.flapspeed)) 
		{
			slowFlaps -= speed;
		}				
	} 
	// No speed control requested so copy flaps
	else
	{
	 	slowFlaps = flap;
	}

	flapskip++;
	if (flapskip > cfg.flapspeed) flapskip = 0;
		
	// Add in flap and reverse as necessary
	flaperons = (slowFlaps * cfg.servoreverse[cfg.flapchan]);	

	// Basic functions
    servo[0] = left_roll + flaperons;     				// Left flaperon or Aileron
    servo[1] = right_roll - flaperons; 					// Right flaperon
    servo[2] = rcCommand[YAW];                       	// Rudder
    servo[3] = rcCommand[PITCH];                     	// Elevator
	servo[4] = flaperons;								// Speed-controlled flap

	// Ignore if in pass-through mode
    if (!f.PASSTHRU_MODE)
	{
		servo[0] -= (cfg.rollPIDpol * axisPID[ROLL]);	// Stabilised left flaperon or Aileron
		servo[1] -= (cfg.rollPIDpol * axisPID[ROLL]); 	// Stabilised right flaperon
		servo[2] -= (cfg.yawPIDpol * axisPID[YAW]);    	// Stabilised Rudder
		servo[3] += (cfg.pitchPIDpol * axisPID[PITCH]); // Stabilised Elevator
	}
}																						 

void mixTable(void)
{
    uint32_t i;

    // Start by zeroing the servos
	for (i = 0; i < 8; i++) 
	{
		servo[i] = 0;
    }	

    // airplane / servo mixes
    switch (cfg.mixerConfiguration) {
        case MULTITYPE_BI:
            servo[4] = constrain(1500 + (cfg.yaw_direction * axisPID[YAW]) + axisPID[PITCH], 1020, 2000);   //LEFT
            servo[5] = constrain(1500 + (cfg.yaw_direction * axisPID[YAW]) - axisPID[PITCH], 1020, 2000);   //RIGHT
            break;

        case MULTITYPE_TRI:
            servo[5] = constrain(cfg.tri_yaw_middle + cfg.yaw_direction * axisPID[YAW], cfg.tri_yaw_min, cfg.tri_yaw_max); //REAR
            break;

        case MULTITYPE_GIMBAL:
            servo[0] = constrain(cfg.gimbal_pitch_mid + cfg.gimbal_pitch_gain * angle[PITCH] / 16 + rcCommand[PITCH], cfg.gimbal_pitch_min, cfg.gimbal_pitch_max);
            servo[1] = constrain(cfg.gimbal_roll_mid + cfg.gimbal_roll_gain * angle[ROLL] / 16 + rcCommand[ROLL], cfg.gimbal_roll_min, cfg.gimbal_roll_max);
            break;

        case MULTITYPE_AIRPLANE:
            airplaneMixer();
            break;

        case MULTITYPE_FLYING_WING:
			// Throttle
			motor[0] = rcData[THROTTLE];	  				// Send directly from RC for now
			motor[1] = rcData[THROTTLE];	  				// Copy to motor[0] for now (not fitted to Afro-mini)

			// Basic functions
			servo[0] = (rcCommand[PITCH] + rcCommand[ROLL]) >> 1; 	// Left elevon
			servo[1] = (rcCommand[PITCH] - rcCommand[ROLL]) >> 1; 	// Right elevon
			servo[2] = rcCommand[YAW];                     			// Rudder
		
			// Ignore if in pass-through mode
			if (!f.PASSTHRU_MODE)
			{
				// Stabilised left elevon
				servo[0] = servo[0] + (cfg.pitchPIDpol * axisPID[PITCH]) + (cfg.rollPIDpol * axisPID[ROLL]);   
				// Stabilised right elevon
				servo[1] = servo[1] + (cfg.pitchPIDpol * axisPID[PITCH]) - (cfg.rollPIDpol * axisPID[ROLL]);	
				// Stabilised Rudder
				servo[2] -= axisPID[YAW]; 
			}
            break;

		case MULTITYPE_FW_DRAG:
			// Throttle
			motor[0] = rcData[THROTTLE];	  				// Send directly from RC for now
			motor[1] = rcData[THROTTLE];	  				// Copy to motor[0] for now (not fitted to Afro-mini)

			// Basic functions
			servo[0] = (rcCommand[PITCH] + rcCommand[ROLL]) >> 1; 	// Left elevon
			servo[1] = (rcCommand[PITCH] - rcCommand[ROLL]) >> 1; 	// Right elevon
		
			if (rcCommand[YAW] >= 0)
			{
				servo[2] = rcCommand[YAW];                  // Left drag rudder
			}
			else 
			{
				servo[2] = 0;
			}
			
			if (rcCommand[YAW] <= 0)
			{
				servo[3] = rcCommand[YAW];                  // Right drag rudder
			}
			else 
			{
				servo[3] = 0;
			}
		
			// Ignore if in pass-through mode
			if (!f.PASSTHRU_MODE)
			{
				// Stabilised left elevon
				servo[0] = servo[0] + (cfg.pitchPIDpol * axisPID[PITCH]) - (cfg.rollPIDpol * axisPID[ROLL]);   
				// Stabilised right elevon
				servo[1] = servo[1] + (cfg.pitchPIDpol * axisPID[PITCH]) + (cfg.rollPIDpol * axisPID[ROLL]);	
				
				// Stabilised drag rudders (only for one side of movement)
				if (axisPID[YAW] < 0)
				{
					servo[2] -= axisPID[YAW]; 
				}
				if (axisPID[YAW] > 0)
				{
					servo[3] -= axisPID[YAW]; 
				}
			}
            break;
    }

    // do camstab
    if (feature(FEATURE_SERVO_TILT)) {
        uint16_t aux[2] = { 0, 0 };

        if ((cfg.gimbal_flags & GIMBAL_NORMAL) || (cfg.gimbal_flags & GIMBAL_TILTONLY))
            aux[0] = rcData[AUX3] - cfg.midrc[AUX3];
        if (!(cfg.gimbal_flags & GIMBAL_DISABLEAUX34))
            aux[1] = rcData[AUX4] - cfg.midrc[AUX4];

        servo[0] = cfg.gimbal_pitch_mid + aux[0];
        servo[1] = cfg.gimbal_roll_mid + aux[1];

        if (rcOptions[BOXCAMSTAB]) {
            if (cfg.gimbal_flags & GIMBAL_MIXTILT) 
			{
				servo[0] -= (-cfg.gimbal_pitch_gain) * angle[PITCH] / 16 - cfg.gimbal_roll_gain * angle[ROLL] / 16;
				servo[1] += (-cfg.gimbal_pitch_gain) * angle[PITCH] / 16 + cfg.gimbal_roll_gain * angle[ROLL] / 16;	
			}
			else 
			{
				servo[0] += cfg.gimbal_pitch_gain * angle[PITCH] / 16;
				servo[1] += cfg.gimbal_roll_gain * angle[ROLL]  / 16;
			}
        }

        servo[0] = constrain(servo[0], cfg.gimbal_pitch_min, cfg.gimbal_pitch_max);
        servo[1] = constrain(servo[1], cfg.gimbal_roll_min, cfg.gimbal_roll_max);
    }

    if (cfg.gimbal_flags & GIMBAL_FORWARDAUX) {
        int offset = 0;
        if (feature(FEATURE_SERVO_TILT))
            offset = 2;
        for (i = 0; i < 4; i++)
            pwmWriteServo(i + offset, rcData[AUX1 + i]);
    }

	// Kill motors when not armed
    for (i = 0; i < numberMotor; i++) 
	{
        if (!f.ARMED)
            motor[i] = cfg.mincommand;
    }
			
	// Reverse, offset, then check all servo outputs against endpoints
	for (i = 0; i < 8; i++) 
	{
		servo[i] = servo[i] * cfg.servoreverse[i];
		servo[i] = servo[i] + cfg.servotrim[i];
		servo[i] = constrain(servo[i], cfg.servoendpoint_low[i], cfg.servoendpoint_high[i]);
	}
	
	
}
