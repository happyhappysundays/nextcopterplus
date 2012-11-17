#include "board.h"
#include "mw.h"

static uint8_t numberMotor = 0;
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
    { 1.0f,  0.0f,  0.866025f,  1.0f },     // FRONT
    { 1.0f,  0.0f, -0.866025f, -1.0f },     // REAR
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
    { 1, 1, NULL },                // * MULTITYPE_FLYING_WING
    { 4, 0, mixerY4 },             // MULTITYPE_Y4
    { 6, 0, mixerHex6X },          // MULTITYPE_HEX6X
    { 8, 0, mixerOctoX8 },         // MULTITYPE_OCTOX8
    { 8, 0, mixerOctoFlatP },      // MULTITYPE_OCTOFLATP
    { 8, 0, mixerOctoFlatX },      // MULTITYPE_OCTOFLATX
    { 1, 1, NULL },                // * MULTITYPE_AIRPLANE
    { 0, 1, NULL },                // * MULTITYPE_HELI_120_CCPM
    { 0, 1, NULL },                // * MULTITYPE_HELI_90_DEG
    { 4, 0, mixerVtail4 },         // MULTITYPE_VTAIL4
    { 0, 0, NULL },                // MULTITYPE_CUSTOM
};

void mixerInit(void)
{
    int i;

    // enable servos for mixes that require them. note, this shifts motor counts.
    useServo = mixers[cfg.mixerConfiguration].useServo;
    // if we want camstab/trig, that also enables servos, even if mixer doesn't
    if (feature(FEATURE_SERVO_TILT))
        useServo = 1;

    if (cfg.mixerConfiguration == MULTITYPE_CUSTOM) {
        // load custom mixer into currentMixer
        for (i = 0; i < MAX_MOTORS; i++) {
            // check if done
            if (cfg.customMixer[i].throttle == 0.0f)
                break;
            currentMixer[i] = cfg.customMixer[i];
            numberMotor++;
        }
    } else {
        numberMotor = mixers[cfg.mixerConfiguration].numberMotor;
        // copy motor-based mixers
        if (mixers[cfg.mixerConfiguration].motor) {
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
            pwmWriteServo(0, servo[0]); // debug
            pwmWriteServo(1, servo[1]);
            pwmWriteServo(2, servo[2]);
            pwmWriteServo(3, servo[3]);
            pwmWriteServo(4, servo[4]);
            break;

        case MULTITYPE_FLYING_WING:
            pwmWriteServo(0, servo[0]); // debug
            pwmWriteServo(1, servo[1]);
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
        pwmWriteMotor(i, motor[i]);
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
	static int16_t slowFlaps = 0;
	static uint8_t flapskip;
		
	uint8_t i, speed;
	uint8_t roll_channel;
    uint16_t servomid[8];

    // Start by setting the mid points
	for (i = 0; i < 8; i++) 
	{
//		servomid[i] = 1500 + cfg.servotrim[i]; 		// cfg.servotrim[] are not set anywhere yet... duh!
		servomid[i] = 1500; 		// Servo center is 1500?
		servo[i] = 0; // Debug
    }

	// Reconstruct extra channels
	rcCommand[AUX1] = rcData[AUX1] - cfg.midrc;
	rcCommand[AUX2] = rcData[AUX2] - cfg.midrc;
	rcCommand[AUX3] = rcData[AUX3] - cfg.midrc;
	rcCommand[AUX4] = rcData[AUX4] - cfg.midrc;
	rcCommand[NOCHAN] = 0; // Debug
		
	// Throttle is handled separately here
    if (!f.ARMED)
	{
		motor[0] = cfg.mincommand; 					// Kill throttle when disarmed
    }
	else
	{
		motor[0] = rcData[THROTTLE];
	}
	// Do flap speed control
	if (cfg.flapchan != NOCHAN)						// Ignore if no flap channel
	{
		flap = rcCommand[cfg.flapchan];
	}
	
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
	else
	{
	 	slowFlaps = flap;
	}

	flapskip++;
	if (flapskip > cfg.flapspeed) flapskip = 0;
		
	// Add in flap and reverse as necessary
	flaperons = (slowFlaps * cfg.servoreverse[cfg.flapchan]);
	
	// Basic flaps - one separate channel
	if (cfg.flapmode == BASIC_FLAP)
	{
		roll_channel = ROLL;
	}
	
	// Flaperons - two idependant channels
	else
	{
		roll_channel = cfg.aileron2;
	}			

    if (f.PASSTHRU_MODE) { // More-or-less Direct passthru from RX
        servo[0] = (rcCommand[ROLL] + flaperons) * cfg.servoreverse[0];     	//   Reversible left flaperon or Aileron
        servo[1] = (rcCommand[roll_channel] + flaperons) * cfg.servoreverse[1]; //   Reversible right flaperon
        servo[2] = rcCommand[YAW] * cfg.servoreverse[2];                       	//   Reversible Rudder
        servo[3] = rcCommand[PITCH] * cfg.servoreverse[3];                     	//   Reversible Elevator
		servo[4] = rcCommand[cfg.flapchan] * cfg.servoreverse[4];     			//   Reversible flap
    }
	 
	else {
		if (f.HORIZON_MODE)
		{	// Autolevel
			servo[0] = cfg.gimbal_roll_gain * angle[ROLL] / 16;   // Create new gain variable
			servo[1] = cfg.gimbal_roll_gain * angle[ROLL] / 16;
	    	servo[3] = cfg.gimbal_pitch_gain * angle[PITCH] / 16;	
		}
			
		// Assisted modes (gyro only or gyro+acc according to AUX configuration in GUI)
        servo[0] += (axisPID[ROLL] + flaperons) * cfg.servoreverse[0];     	//   Reversible, stabilised left flaperon or Aileron
        servo[1] += (axisPID[ROLL] - rcCommand[ROLL] + rcCommand[roll_channel] + flaperons) * cfg.servoreverse[1];   //   Reversible, stabilised right flaperon
        servo[2] = axisPID[YAW] * cfg.servoreverse[2];                       	//   Reversible, stabilised Rudder
        servo[3] += axisPID[PITCH] * cfg.servoreverse[3];                     	//   Reversible, stabilised Elevator
		servo[4] = flaperons;        		 									//   Reversible, speed-controlled flap
	}
		
	// Offset, then check all servo outputs against endpoints
	for (i = 0; i < 8; i++) 
	{
		servo[i] = servo[i] + servomid[i];
		servo[i] = constrain(servo[i], cfg.servoendpoint_low[i], cfg.servoendpoint_high[i]);
	}

}

void mixTable(void)
{
    int16_t maxMotor;
    uint32_t i;

    if (numberMotor > 3) {
        // prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));
    }

    // motors for non-servo mixes
    if (numberMotor > 1)
        for (i = 0; i < numberMotor; i++)
            motor[i] = rcCommand[THROTTLE] * currentMixer[i].throttle + axisPID[PITCH] * currentMixer[i].pitch + axisPID[ROLL] * currentMixer[i].roll + cfg.yaw_direction * axisPID[YAW] * currentMixer[i].yaw;

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
            motor[0] = rcCommand[THROTTLE];
            if (f.PASSTHRU_MODE) { // do not use sensors for correction, simple 2 channel mixing
                int p = 0, r = 0;
                servo[0] = p * (rcData[PITCH] - cfg.midrc) + r * (rcData[ROLL] - cfg.midrc);
                servo[1] = p * (rcData[PITCH] - cfg.midrc) + r * (rcData[ROLL] - cfg.midrc);
            } else { // use sensors to correct (gyro only or gyro+acc)
                int p = 0, r = 0;
                servo[0] = p * axisPID[PITCH] + r * axisPID[ROLL];
                servo[1] = p * axisPID[PITCH] + r * axisPID[ROLL];
            }
            break;
    }

    // do camstab
    if (feature(FEATURE_SERVO_TILT)) {
        uint16_t aux[2] = { 0, 0 };

        if ((cfg.gimbal_flags & GIMBAL_NORMAL) || (cfg.gimbal_flags & GIMBAL_TILTONLY))
            aux[0] = rcData[AUX3] - cfg.midrc;
        if (!(cfg.gimbal_flags & GIMBAL_DISABLEAUX34))
            aux[1] = rcData[AUX4] - cfg.midrc;

        servo[0] = cfg.gimbal_pitch_mid + aux[0];
        servo[1] = cfg.gimbal_roll_mid + aux[1];

        if (rcOptions[BOXCAMSTAB]) {
            servo[0] += cfg.gimbal_pitch_gain * angle[PITCH] / 16;		
            servo[1] += cfg.gimbal_roll_gain * angle[ROLL]  / 16;
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

    maxMotor = motor[0];
    for (i = 1; i < numberMotor; i++)
        if (motor[i] > maxMotor)
            maxMotor = motor[i];
    for (i = 0; i < numberMotor; i++) {
        if (maxMotor > cfg.maxthrottle)     // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motor[i] -= maxMotor - cfg.maxthrottle;
        motor[i] = constrain(motor[i], cfg.minthrottle, cfg.maxthrottle);
        if ((rcData[THROTTLE]) < cfg.mincheck) {
            if (!feature(FEATURE_MOTOR_STOP))
                motor[i] = cfg.minthrottle;
            else
                motor[i] = cfg.mincommand;
        }
        if (!f.ARMED)
            motor[i] = cfg.mincommand;
    }
}
