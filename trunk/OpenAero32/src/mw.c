//*********************************************************************
//* mw.c
//*********************************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "board.h"
#include "mw.h"

//************************************************************
// Variables
//************************************************************
// Flags
flags_t f;
int16_t debug[4];
uint8_t toggleBeep = 0;
int16_t headFreeModeHold;

// Misc
uint32_t currentTime = 0;
uint32_t previousTime = 0;
uint16_t cycleTime = 0; 
int16_t annex650_overrun_count = 0;
int16_t telemTemperature1;      		// Gyro sensor temperature

// RC
int16_t failsafeCnt = 0;
int16_t failsafeEvents = 0;
//int16_t rcData[8] = { 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502 }; // Interval [1000;2000]
int16_t rcCommand[9];           		// Interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
int16_t lookupPitchRollRC[6];   		// Lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[11];  			// Lookup table for expo & mid THROTTLE
uint8_t rcOptions[CHECKBOXITEMS];
//rcReadRawDataPtr rcReadRawFunc = NULL;  // Function to receive data from default receiver driver
																								
// PID
uint8_t dynP8[3], dynI8[3], dynD8[3];
int16_t axisPID[3];

// GPS
int32_t GPS_coord[2];
int32_t GPS_home[2];
int32_t GPS_hold[2];
uint8_t GPS_numSat;
uint16_t GPS_distanceToHome;			// Distance to home point in meters
int16_t GPS_directionToHome;			// Direction to home or hol point in degrees
uint16_t GPS_altitude, GPS_speed;       // Altitude in 0.1m and speed in 0.1m/s
uint8_t GPS_update = 0;             	// Binary toogle to distinct a GPS position update
int16_t GPS_angle[2] = { 0, 0 };    	// Angles that must be applied for GPS correction
uint16_t GPS_ground_course = 0;     	// Degrees*10
uint8_t GPS_Present = 0;            	// Checksum from Gps serial
uint8_t GPS_Enable = 0;
int16_t nav[2];
int16_t nav_rated[2];               	// Adding a rate controller to the navigation to make it smoother
int8_t nav_mode = NAV_MODE_NONE;    	// Navigation mode

// Automatic ACC Offset Calibration
uint16_t InflightcalibratingA = 0;
int16_t  AccInflightCalibrationArmed;
uint16_t AccInflightCalibrationMeasurementDone = 0;
uint16_t AccInflightCalibrationSavetoEEProm = 0;
uint16_t AccInflightCalibrationActive = 0;

// Battery monitoring stuff
uint8_t vbat;                   	// Battery voltage in 0.1V steps
uint8_t batteryCellCount = 3;   	// Cell count
uint16_t batteryWarningVoltage;

//************************************************************
// Defines
//************************************************************

#define BREAKPOINT 1500	  // <-- make this a variable

//************************************************************
// Code
//************************************************************

void loop(void)
{
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    uint8_t axis, i;
    int16_t error, errorAngle;
    int16_t delta, deltaSum;
    int16_t PTerm = 0, ITerm = 0, PTermACC = 0, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0, DTerm = 0;
    static int16_t lastGyro[3] = { 0, 0, 0 };
    static int16_t delta1[3], delta2[3];
    static int16_t errorGyroI[3] = { 0, 0, 0 };
    static int16_t errorAngleI[2] = { 0, 0 };
    static uint32_t rcTime = 0;
    static int16_t initialThrottleHold;
    static uint32_t loopTime;
    uint16_t auxState = 0;
    int16_t prop;

    // This will return false if spektrum is disabled. shrug.
    if (spektrumFrameComplete())
	{
		computeRC();
	}
    
	// 50Hz RC loop
	if ((int32_t)(currentTime - rcTime) >= 0) 
	{ // 50Hz
        rcTime = currentTime + 20000;
        // TODO clean this up. computeRC should handle this check
        if (!feature(FEATURE_SPEKTRUM))
            computeRC();

        // Failsafe routine
        if (feature(FEATURE_FAILSAFE)) 
		{
            if (failsafeCnt > (5 * cfg.failsafe_delay) && f.ARMED) 
			{ // Stabilize, and set Throttle to specified level
                for (i = 0; i < 3; i++)
                {
					rcData[i] = cfg.midrc;      // after specified guard time after RC signal is lost (in 0.1sec)
                }
				rcData[THROTTLE] = cfg.failsafe_throttle;
                if (failsafeCnt > 5 * (cfg.failsafe_delay + cfg.failsafe_off_delay)) 
				{  // Turn OFF motors after specified Time (in 0.1sec)
                    f.ARMED = 0;  // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
                    f.OK_TO_ARM = 0;        // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
                }
                failsafeEvents++;
            }
            if (failsafeCnt > (5 * cfg.failsafe_delay) && !f.ARMED) 
			{  // Turn off "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
                f.ARMED = 0;        // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
                f.OK_TO_ARM = 0;    // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
            }
            failsafeCnt++;
        }

        if (rcData[THROTTLE] < cfg.mincheck) 
		{
            errorGyroI[ROLL] = 0;
            errorGyroI[PITCH] = 0;	// Reset I-terms
            errorGyroI[YAW] = 0;
            errorAngleI[ROLL] = 0;
            errorAngleI[PITCH] = 0;
            rcDelayCommand++;
            if (rcData[YAW] < cfg.mincheck && rcData[PITCH] < cfg.mincheck && !f.ARMED) 
			{
                if (rcDelayCommand == 20) 
				{
                    calibratingG = 1000;
                    if (feature(FEATURE_GPS))
					{
                        GPS_reset_home_position();
					}
                }
            } 
			else if (feature(FEATURE_INFLIGHT_ACC_CAL) && (!f.ARMED && rcData[YAW] < cfg.mincheck && rcData[PITCH] > cfg.maxcheck && rcData[ROLL] > cfg.maxcheck)) 
			{
                if (rcDelayCommand == 20) 
				{
                    if (AccInflightCalibrationMeasurementDone) 
					{   // trigger saving into eeprom after landing
                        AccInflightCalibrationMeasurementDone = 0;
                        AccInflightCalibrationSavetoEEProm = 1;
                    } 
					else 
					{
                        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
                        if (AccInflightCalibrationArmed) 
						{
                            toggleBeep = 2;
                        } 
						else 
						{
                            toggleBeep = 3;
                        }
                    }
                }
            } 
			// debug
			if (1)
			{
				f.ARMED = 1;
			}
/*
			else if (cfg.activate[BOXARM] > 0) 
			{
                if (rcOptions[BOXARM] && f.OK_TO_ARM) 
				{
                    // TODO: feature(FEATURE_FAILSAFE) && failsafeCnt == 0
                    f.ARMED = 1;
                    headFreeModeHold = heading;
                } 
				else if (f.ARMED)
				{
                    f.ARMED = 0;
                }
				rcDelayCommand = 0;
            } 
			else if ((rcData[YAW] < cfg.mincheck || (cfg.retarded_arm == 1 && rcData[ROLL] < cfg.mincheck)) && f.ARMED) 
			{
                if (rcDelayCommand == 20)
                {
					f.ARMED = 0;  // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
            	}
			} 
			else if ((rcData[YAW] > cfg.maxcheck || (rcData[ROLL] > cfg.maxcheck && cfg.retarded_arm == 1)) && rcData[PITCH] < cfg.maxcheck && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED) 
			{
                if (rcDelayCommand == 20) 
				{
                    f.ARMED = 1;
                    headFreeModeHold = heading;
                }
            } 
*/
			else
			{
                rcDelayCommand = 0;
        	}
		} 

		// Do ACC trim adjustment
		else if (rcData[THROTTLE] > cfg.maxcheck && !f.ARMED) 
		{
            if (rcData[YAW] < cfg.mincheck && rcData[PITCH] < cfg.mincheck) 
			{   // throttle=max, yaw=left, pitch=min
                if (rcDelayCommand == 20)
				{
                    calibratingA = 400;
                }
				rcDelayCommand++;
            } 
			else if (rcData[YAW] > cfg.maxcheck && rcData[PITCH] < cfg.mincheck) 
			{    // throttle=max, yaw=right, pitch=min
                if (rcDelayCommand == 20)
				{
                    f.CALIBRATE_MAG = 1;   // MAG calibration request
                }
				rcDelayCommand++;
            } 
			else if (rcData[PITCH] > cfg.maxcheck) 
			{
                cfg.angleTrim[PITCH] += 2;
				writeParams(1);
            } 
			else if (rcData[PITCH] < cfg.mincheck) 
			{
                cfg.angleTrim[PITCH] -= 2;
                writeParams(1);
            } 
			else if (rcData[ROLL] > cfg.maxcheck) 
			{
                cfg.angleTrim[ROLL] += 2;
                writeParams(1);
            } 
			else if (rcData[ROLL] < cfg.mincheck) 
			{
                cfg.angleTrim[ROLL] -= 2;
                writeParams(1);
            } 
			else 
			{
                rcDelayCommand = 0;
            }
        }

        if (feature(FEATURE_INFLIGHT_ACC_CAL)) 
		{
            if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > cfg.mincheck && !rcOptions[BOXARM]) 
			{   // Copter is airborne and you are turning it off via boxarm : start measurement
                InflightcalibratingA = 50;
                AccInflightCalibrationArmed = 0;
            }
            if (rcOptions[BOXPASSTHRU]) 
			{      // Use the Passthru Option to activate : Passthru = TRUE Meausrement started, Land and passtrhu = 0 measurement stored
                if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
				{
                    InflightcalibratingA = 50;
            	}
			} 
			else if (AccInflightCalibrationMeasurementDone && !f.ARMED) 
			{
                AccInflightCalibrationMeasurementDone = 0;
                AccInflightCalibrationSavetoEEProm = 1;
            }
        }

        // Set AUX1 to AUX4 states
		for(i = 0; i < 4; i++)
        {
		    auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | 									   // Low
						(1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) |	   // Mid
						(rcData[AUX1 + i] > 1700) << (3 * i + 2);								   // High
		}
        for(i = 0; i < CHECKBOXITEMS; i++)
		{
            rcOptions[i] = (auxState & cfg.activate[i]) > 0;
		}

        // Handle stability mode change
        if (rcOptions[BOXSTABILITY]) 
		{
            // bumpless transfer to Level mode
            if (!f.STABILITY_MODE) 
			{
                errorAngleI[ROLL] = 0;
                errorAngleI[PITCH] = 0;
                f.STABILITY_MODE = 1;
            }
        } 
		else 
		{
            f.STABILITY_MODE = 0;        // failsave support
        }

        // Handle autolevel mode change. Use autolevel in failsafe
		// note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
		if ((rcOptions[BOXAUTOLEVEL] || (failsafeCnt > 5 * cfg.failsafe_delay)) && (sensors(SENSOR_ACC))) 
		{
            if (!f.AUTOLEVEL_MODE) 
			{
                errorAngleI[ROLL] = 0; 	 // Reset I-terms
                errorAngleI[PITCH] = 0;
                f.AUTOLEVEL_MODE = 1;
            }
        } 
		else 
		{
            f.AUTOLEVEL_MODE = 0;
        }

        // Handle arming mode change
		if ((rcOptions[BOXARM]) == 0)
		{
            f.OK_TO_ARM = 1;
        }
		
		// Set LED if in Autolevel or Stability modes
		if (f.STABILITY_MODE || f.AUTOLEVEL_MODE) 
		{
            LED1_ON;
        } 
		else 
		{
            LED1_OFF;
        }

        // Handle Baro mode change
		if (sensors(SENSOR_BARO)) 
		{
            if (rcOptions[BOXBARO]) 
			{
                if (!f.BARO_MODE) 
				{
                    f.BARO_MODE = 1;
                    AltHold = EstAlt;
                    initialThrottleHold = rcCommand[THROTTLE];
                    errorAltitudeI = 0;
                    BaroPID = 0;
                }
            } 
			else
			{
                f.BARO_MODE = 0;
			}
        }

#ifdef  MAG							 	
		// Handle mag mode change
        if (sensors(SENSOR_MAG)) 
		{
            if (rcOptions[BOXMAG]) 
			{
                if (!f.MAG_MODE) 
				{
                    f.MAG_MODE = 1;
                    magHold = heading;
                }
            } 
			else
			{
                f.MAG_MODE = 0;
			}
            if (rcOptions[BOXHEADADJ]) 
			{
                headFreeModeHold = heading; // acquire new heading
            }
        }
#endif
		// Handle GPS navigation mode change
        if (sensors(SENSOR_GPS)) 
		{
            if (f.GPS_FIX && GPS_numSat >= 5) 
			{
                if (rcOptions[BOXGPSHOME]) 
				{
                    if (!f.GPS_HOME_MODE) 
					{
                        f.GPS_HOME_MODE = 1;
                        GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);
                        nav_mode = NAV_MODE_WP;
                    }
                } 
				else 
				{
                    f.GPS_HOME_MODE = 0;
                }
                if (rcOptions[BOXGPSHOLD]) 
				{
                    if (!f.GPS_HOLD_MODE) 
					{
                        f.GPS_HOLD_MODE = 1;
                        GPS_hold[LAT] = GPS_coord[LAT];
                        GPS_hold[LON] = GPS_coord[LON];
                        GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
                        nav_mode = NAV_MODE_POSHOLD;
                    }
                } 
				else 
				{
                    f.GPS_HOLD_MODE = 0;
                }
            }
        }

        // Handle pass-through mode change
		if (rcOptions[BOXPASSTHRU]) 
		{
            f.PASSTHRU_MODE = 1;
        } 
		else 
		{
            f.PASSTHRU_MODE = 0;
        }
        
    } 
	
	
	// Full-speed loop but not RC
	else 
	{                    // not in rc loop
        static int8_t taskOrder = 0;    // never call all function in the same loop, to avoid high delay spikes
        switch (taskOrder++ % 4) 
		{
        case 0:
#ifdef MAG
            if (sensors(SENSOR_MAG))
                Mag_getADC();
#endif
            break;
        case 1:
            if (sensors(SENSOR_BARO))
                Baro_update();
            break;
        case 2:
            if (sensors(SENSOR_BARO))
                getEstimatedAltitude();
            break;
        case 3:
            break;
        default:
            taskOrder = 0;
            break;
        }
    }

	// Full-speed loop

	// Do IMU etc if due
	currentTime = micros();
    if (cfg.looptime == 0 || (int32_t)(currentTime - loopTime) >= 0) 
	{
        loopTime = currentTime + cfg.looptime;

        computeIMU();
        // Measure loop rate just afer reading the sensors
        currentTime = micros();
        cycleTime = (int32_t)(currentTime - previousTime);
        previousTime = currentTime;
#ifdef MPU6050_DMP
        mpu6050DmpLoop();
#endif

#ifdef MAG
		// Handle mag hold
        if (sensors(SENSOR_MAG)) 
		{
            if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE) 
			{
                int16_t dif = heading - magHold;
                if (dif <= -180)
				{
                    dif += 360;
                }
				if (dif >= +180)
                {
				    dif -= 360;
                }
				if (f.SMALL_ANGLES_25)
                {
				    rcCommand[YAW] -= dif * cfg.P8[PIDMAG] / 30;    // 18 deg
				}
            } 
			else
			{
                magHold = heading;
			}
        }
#endif

		// Do alt hold	   <-- but for aeroplanes
        if (sensors(SENSOR_BARO)) 
		{
            if (f.BARO_MODE) 
			{
                if (abs(rcCommand[THROTTLE] - initialThrottleHold) > cfg.alt_hold_throttle_neutral) 
				{
					f.BARO_MODE = 0;   // so that a new althold reference is defined
                }
				rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
            }
        }
		
		// Do GPS navigation 	   <-- but for aeroplanes
        if (sensors(SENSOR_GPS)) 
		{
            // Check that we really need to navigate ?
            if ((!f.GPS_HOME_MODE && !f.GPS_HOLD_MODE) || !f.GPS_FIX_HOME) 
			{
                // If not. Reset nav loops and all nav related parameters
                GPS_reset_nav();
            } 
			else 
			{
                float sin_yaw_y = sinf(heading * 0.0174532925f);
                float cos_yaw_x = cosf(heading * 0.0174532925f);
                if (cfg.nav_slew_rate) 
				{
                    nav_rated[LON] += constrain(wrap_18000(nav[LON] - nav_rated[LON]), -cfg.nav_slew_rate, cfg.nav_slew_rate); // TODO check this on uint8
                    nav_rated[LAT] += constrain(wrap_18000(nav[LAT] - nav_rated[LAT]), -cfg.nav_slew_rate, cfg.nav_slew_rate);
                    GPS_angle[ROLL] = (nav_rated[LON] * cos_yaw_x - nav_rated[LAT] * sin_yaw_y) / 10;
                    GPS_angle[PITCH] = (nav_rated[LON] * sin_yaw_y + nav_rated[LAT] * cos_yaw_x) / 10;
                } 
				else 
				{
                    GPS_angle[ROLL] = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
                    GPS_angle[PITCH] = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
                }
            }
        }

		//*********************************************************************
		//* Flight PITCH & ROLL & YAW PID
		//********************************************************************/
  
        prop = max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])); // range [0;500]

        for (axis = 0; axis < 3; axis++) 
		{
            // Roll/Pitch accelerometers
			if (f.AUTOLEVEL_MODE && axis < 2) 
			{ 
                // Error
                errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -500, +500) - angle[axis] + cfg.angleTrim[axis]; // 50 degrees max inclination	    

				// P-term
                PTermACC = (int32_t)errorAngle * cfg.P8[PIDLEVEL] / 100; // 32 bits is needed for calculation
                PTermACC = constrain(PTermACC, -cfg.D8[PIDLEVEL] * 5, +cfg.D8[PIDLEVEL] * 5);	  // Limit the acc throw with the D variable <-- fix this bullshit 

                // I-term
				errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000); // Anti wind-up
                ITermACC = ((int32_t)errorAngleI[axis] * cfg.I8[PIDLEVEL]) >> 12;
            }

            // RPY Gyros in stability mode
			if (f.STABILITY_MODE) 
			{ 
				// Error
                error = (int32_t)rcCommand[axis] * 10 * 8 / cfg.P8[axis];
                error -= gyroData[axis];

                // P-term
	            // Add in throttle-based Dynamic P for gyro on all axis
				PTermGYRO -= (int32_t)gyroData[axis] * dynP8[axis] / 10 / 8; // 32 bits is needed for calculation

                // I-term
				errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000); // Anti wind-up
                if (abs(gyroData[axis]) > 640)
				{ 
                    errorGyroI[axis] = 0;		 // Trash the I-term if gyro data bottoms out
                }
				ITermGYRO = (errorGyroI[axis] / 125 * cfg.I8[axis]) >> 6;

	              // D-term for all axis
				delta = gyroData[axis] - lastGyro[axis]; // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
	            lastGyro[axis] = gyroData[axis];
	            deltaSum = delta1[axis] + delta2[axis] + delta;
	            delta2[axis] = delta1[axis];
	            delta1[axis] = delta;
	            DTerm = ((int32_t)deltaSum * dynD8[axis]) >> 5; // 32 bits is needed for calculation
            }

			// Dynamically vary the Roll/Pitch ACC and Gyro mix depending on the RC input (Autolevel+Stability mode)
            if (f.AUTOLEVEL_MODE && f.STABILITY_MODE && axis < 2) 
			{
                PTerm = ((int32_t)PTermACC * (500 - prop) + (int32_t)PTermGYRO * prop) / 500;
                ITerm = ((int32_t)ITermACC * (500 - prop) + (int32_t)ITermGYRO * prop) / 500;
            } 

			// Use accelerometers in Autolevel-only mode
            else if (f.AUTOLEVEL_MODE && !f.STABILITY_MODE && axis < 2) 
			{
                PTerm = PTermACC;
                ITerm = ITermACC;
            } 

			// Not in Autolevel so use gyros only (also for Yaw axis)
			else 
			{
                PTerm = PTermGYRO;
                ITerm = ITermGYRO;
            }
																								   
 			// PID sum
            axisPID[axis] =  PTerm + ITerm - DTerm;
			PTermGYRO = 0; //debug
			PTerm = 0; // Debug
			ITerm = 0; // Debug
			DTerm = 0; // Debug
        }

        mixTable();
        writeServos();
        writeMotors();
    }
}

// This code is executed at each loop and won't interfere with control loop if it lasts less than 650 microseconds
void annexCode(void)
{
	static uint32_t calibratedAccTime;
	uint16_t tmp, tmp2;
	static uint8_t buzzerFreq;  // Delay between buzzer ring
	static uint8_t vbatTimer = 0;
	uint8_t axis, prop1, prop2;
	static uint8_t ind = 0;
	uint16_t vbatRaw = 0;
	static uint16_t vbatRawArray[8];
	uint8_t i;
	
	// PITCH & ROLL dynamic PID adjustment, depending on throttle value
	if (rcData[THROTTLE] < BREAKPOINT) 
	{
		prop2 = 100;
	} 
	else 
	{
		if (rcData[THROTTLE] < 2000) 
		{
			prop2 = 100 - (uint16_t) cfg.dynThrPID * (rcData[THROTTLE] - BREAKPOINT) / (2000 - BREAKPOINT);
		} 
		else 
		{
			prop2 = 100 - cfg.dynThrPID;
		}
	}

	// Deadband, stick rates
	for (axis = 0; axis < 3; axis++) 
	{
		tmp = min(abs(rcData[axis] - cfg.midrc), 500);
		if (axis != 2) 
		{ // ROLL & PITCH
			if (cfg.deadband) 
			{
				if (tmp > cfg.deadband) 
				{
					tmp -= cfg.deadband;
				} 
				else 
				{
					tmp = 0;
				}
			}
		
		    tmp2 = tmp / 100;
		    rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
		    prop1 = 100 - (uint16_t) cfg.rollPitchRate * tmp / 500;
		    prop1 = (uint16_t) prop1 *prop2 / 100;
		} 
		else 
		{ // YAW
			if (cfg.yawdeadband) 
			{
				if (tmp > cfg.yawdeadband) 
				{
					tmp -= cfg.yawdeadband;
				} 
				else 
				{
					tmp = 0;
				}
			}
			rcCommand[axis] = tmp;
			prop1 = 100 - (uint16_t) cfg.yawRate * tmp / 500;
		}

		// Calculate dynamic P and D values
		dynP8[axis] = (uint16_t) cfg.P8[axis] * prop1 / 100;
		dynD8[axis] = (uint16_t) cfg.D8[axis] * prop1 / 100;

		if (rcData[axis] < cfg.midrc)
		{
			rcCommand[axis] = -rcCommand[axis];		 // This is stupid... but it will do for now
		}
	}

	// Throttle scaling
    tmp = constrain(rcData[THROTTLE], cfg.mincheck, 2000);
    tmp = (uint32_t) (tmp - cfg.mincheck) * 1000 / (2000 - cfg.mincheck);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

	// Battery alarm
    if (feature(FEATURE_VBAT)) 
	{
        if (!(++vbatTimer % VBATFREQ)) 
		{
            vbatRawArray[(ind++) % 8] = adcGetBattery();
            for (i = 0; i < 8; i++)
            {
				vbatRaw += vbatRawArray[i];
            }
			vbat = batteryAdcToVoltage(vbatRaw / 8);
        }
        if ((vbat > batteryWarningVoltage) || (vbat < cfg.vbatmincellvoltage)) 
		{ // VBAT ok, buzzer off
            buzzerFreq = 0;
        } 
		else
        {
			buzzerFreq = 4;     // low battery
		}
    }

	// Handle buzzer events
    buzzer(buzzerFreq);         // external buzzer routine that handles buzzer events globally now

	// Acc recalibration
	if ((calibratingA > 0 && sensors(SENSOR_ACC)) || (calibratingG > 0)) 
	{      // Calibration phases
		LED0_TOGGLE;
	} 
	else 
	{
		if (f.ACC_CALIBRATED)
		{
			LED0_OFF;
		}
		if (f.ARMED)
		{
			LED0_ON;
		}
		// This will switch to/from 9600 or 115200 baud depending on state. Of course, it should only do it on changes.
		if (feature(FEATURE_TELEMETRY))
		{
			initTelemetry(f.ARMED);
		}
	}

	if ((int32_t)(currentTime - calibratedAccTime) >= 0) 
	{
		if (!f.SMALL_ANGLES_25) 
		{
			f.ACC_CALIBRATED = 0; // the multi uses ACC and is not calibrated or is too much inclinated
			LED0_TOGGLE;
			calibratedAccTime = currentTime + 500000;
		} 
		else 
		{
			f.ACC_CALIBRATED = 1;
		}
	}

	// Hande serial events
	serialCom();
	
	// Flash LED when appropriate for GPS
	if (sensors(SENSOR_GPS)) 
	{
		static uint32_t GPSLEDTime;
		if ((int32_t)(currentTime - GPSLEDTime) >= 0 && (GPS_numSat >= 5)) 
		{
			GPSLEDTime = currentTime + 150000;
			LED1_TOGGLE;
		}
	}

    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature)
	{
		gyro.temperature(&telemTemperature1);
	}
    else 
	{
		// TODO MCU temp
    }
}

// Move this
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;

    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            LED0_TOGGLE;        // switch LEDPIN state
            BEEP_ON;
            delay(wait);
            BEEP_OFF;
        }
        delay(60);
    }
}

