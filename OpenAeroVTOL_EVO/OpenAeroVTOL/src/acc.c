//***********************************************************
//* acc.c
//*
//* Read and calibrate the accelerometers.
//* Manage the different board orientations.
//* accADC[] holds the raw values
//*
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "io_cfg.h"
#include "adc.h"
#include "eeprom.h"
#include "main.h"
#include "i2c.h"
#include "MPU6050.h"
#include "imu.h"
#include "menu_ext.h"
#include "mixer.h"
#include "rc.h"

//************************************************************
// Prototypes
//************************************************************

void ReadAcc(void);
void CalibrateAcc(int8_t type);
void get_raw_accs(void);

//************************************************************
// Defines
//************************************************************

#define ACCFS16G	0x18		// 16G full scale
#define ACCFS4G 	0x08		// 4G full scale
#define ACCFS2G		0x00		// 2G full scale

//***********************************************************
// ROLL, PITCH, YAW mapping for alternate orientation modes
//***********************************************************

// This is the translation required to return axis data in ROLL, PITCH, YAW order
const int8_t ACC_RPY_Order[NUMBEROFORIENTS][NUMBEROFAXIS] PROGMEM = 
{
// 	 ROLL, PITCH, YAW
	{ROLL, PITCH, YAW}, // Up/Back (Normal)			[Position 2] -> [Position 3]
	{PITCH, ROLL, YAW}, // Up/Left
	{ROLL, PITCH, YAW}, // Up/Front (Aft)			(Backwards-compatibility. Not for tail-sitters)
	{PITCH, ROLL, YAW}, // Up/Right (Sideways)		(Backwards-compatibility. Not for tail-sitters)
	
	{ROLL, YAW, PITCH}, // Back/Down (PitchUp)		[Position 1] -> [Position 2]
	{PITCH, YAW, ROLL}, // Back/Left
	{ROLL, YAW, PITCH}, // Back/Up
	{PITCH, YAW, ROLL}, // Back/Right
	
	{ROLL, PITCH, YAW}, // Down/Back (Upside down)	(Backwards-compatibility. Not for tail-sitters)
	{PITCH, ROLL, YAW}, // Down/Right
	{ROLL, PITCH, YAW}, // Down/Front				[Position 4] -> [Position 1]
	{PITCH, ROLL, YAW}, // Down/Left
		
	{ROLL, YAW, PITCH}, // Front/Down
	{PITCH, YAW, ROLL}, // Front/Right
	{ROLL, YAW, PITCH}, // Front/Up					[Position 3] -> [Position 4]
	{PITCH, YAW, ROLL}, // Front/Left
		
	{YAW, ROLL, PITCH}, // Left/Down				[Position 5] -> [Position 7]
	{YAW, PITCH, ROLL}, // Left/Front
	{YAW, ROLL, PITCH}, // Left/Up
	{YAW, PITCH, ROLL}, // Left/Back				[Position 7] (destination for Position 5 only)
		
	{YAW, ROLL, PITCH}, // Right/Down (Vertical)	[Position 6] -> [Position 8]
	{YAW, PITCH, ROLL}, // Right/Back				[Position 8] (destination for Position 6 only)
	{YAW, ROLL, PITCH}, // Right/Up
	{YAW, PITCH, ROLL}, // Right/Front
};

// These are the polarities to return them to the default
const int8_t Acc_Pol[NUMBEROFORIENTS][NUMBEROFAXIS] PROGMEM =
{
// 	 ROLL, PITCH, YAW
	{-1,-1,1},	// Up/Back (Normal)
	{1,-1,1},	// Up/Left
	{1,1,1},	// Up/Front (Aft)
	{-1,1,1},	// Up/Right (Sideways)
	
	{-1,-1,-1}, // Back/Down (PitchUp)
	{1,-1,-1},	// Back/Left
	{1,-1,1},	// Back/Up
	{-1,-1,1},	// Back/Right
	
	{1,-1,-1},	// Down/Back (Upside down)
	{-1,-1,-1},	// Down/Right
	{-1,1,-1},	// Down/Front
	{1,1,-1},	// Down/Left
	
	{1,1,-1},	// Front/Down
	{-1,1,-1},	// Front/Right
	{-1,1,1},	// Front/Up
	{1,1,1},	// Front/Left
	
	{1,-1,-1},	// Left/Down
	{1,1,-1},	// Left/Front
	{1,1,1},	// Left/Up
	{1,-1,1},	// Left/Back
	
	{-1,1,-1},	// Right/Down (Vertical)
	{-1,-1,-1},	// Right/Back
	{-1,-1,1},	// Right/Up
	{-1,1,1},	// Right/Front
};

//************************************************************
// Code
//************************************************************
int16_t accADC[NUMBEROFAXIS];		// Holds Acc ADC values - always in RPY order (Combined)
int16_t accADC_P1[NUMBEROFAXIS];	// Holds Acc ADC values - always in RPY order (P1)
int16_t accADC_P2[NUMBEROFAXIS];	// Holds Acc ADC values - always in RPY order (P2)
float accVertf = 0.0;				// Holds the level-zeroed Z-acc value. Used for height damping in hover only.

void ReadAcc()
{
	uint8_t i;
	int16_t temp1, temp2;

	get_raw_accs();					// Updates accADC_P1[] and accADC_P2[] (RPY)

	// P1
	// Use default Config.AccZero for Acc-Z if inverse calibration not done yet
	// Actual zero is held in Config.AccZeroNormZ waiting for inverse calibration
	if (!(Config.Main_flags & (1 << inv_cal_done_P1)))
	{
		Config.AccZero_P1[YAW] = 0;
	}
	// If inverted cal done, Config.AccZeroNormZ and Config.AccZeroDiff have valid values
	else
	{
		Config.AccZero_P1[YAW] = Config.AccZeroNormZ_P1 - Config.AccZeroDiff_P1;
	}
	
	// P2
	if (!(Config.Main_flags & (1 << inv_cal_done_P2)))
	{
		Config.AccZero_P2[YAW] = 0;
	}
	// If inverted cal done, Config.AccZeroNormZ and Config.AccZeroDiff have valid values
	else
	{
		Config.AccZero_P2[YAW] = Config.AccZeroNormZ_P2 - Config.AccZeroDiff_P2;
	}

	// Roll and Pitch are handled normally
	for (i = 0; i < (NUMBEROFAXIS - 1); i++)
	{
		// Only need to do this if the orientations differ
		if (Config.P1_Reference != NO_ORIENT)
		{
			// Change polarity - use the zeros from the appropriate calibrate
			temp1 = ((accADC_P1[i] - Config.AccZero_P1[i]) * (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation_P1][i]));
			temp2 = ((accADC_P2[i] - Config.AccZero_P2[i]) * (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation_P2][i]));
			
			// Get P1 value
			temp1 = scale32(temp1, (100 - transition));

			// Get P2 value
			temp2 = scale32(temp2, transition);

			// Sum the two values
			accADC[i] = temp1 + temp2;
		}
		else
		{
			accADC[i] = ((accADC_P2[i] - Config.AccZero_P2[i]) * (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation_P2][i]));
		}
	}
	
	// Z -axis requires special handling as the zeros are already polarity corrected
	// Only need to do this if the orientations differ
	if (Config.P1_Reference != NO_ORIENT)
	{
		// Change polarity - use the zeros from the appropriate calibrate
		temp1 = ((accADC_P1[YAW] * (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation_P1][YAW]) - Config.AccZero_P1[YAW]));
		temp2 = ((accADC_P2[YAW] * (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation_P2][YAW]) - Config.AccZero_P2[YAW]));
			
		// Get P1 value
		temp1 = scale32(temp1, (100 - transition));

		// Get P2 value
		temp2 = scale32(temp2, transition);

		// Sum the two values
		accADC[YAW] = temp1 + temp2;
	}
	else
	{
		accADC[YAW] = ((accADC_P2[YAW] * (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation_P2][YAW]) - Config.AccZero_P2[YAW]));
	}
		
	// Recalculate current accVertf using filtered acc value
	// Note that AccSmooth[YAW] is already zeroed around 1G so we have to re-add 
	// the zero back here so that Config.AccZeroNormZ subtracts the correct amount
	// accVertf = accSmooth[YAW] + (Config.AccZeroNormZ - Config.AccZero[YAW]);
	
	// Note also that accSmooth[] has already got the correct acc orientations, 
	// so only needs the zeroing value merged from one to the other.

	// Only need to do this if the orientations differ
	if (Config.P1_Reference != NO_ORIENT)
	{
		// Calculate the correct Z-axis data based on the orientation
		temp1 = accSmooth[YAW] + (Config.AccZeroNormZ_P1 - Config.AccZero_P1[YAW]); 
		temp2 = accSmooth[YAW] + (Config.AccZeroNormZ_P2 - Config.AccZero_P2[YAW]); 
	
		// Merge with transition
		temp1 = scale32(temp1, (100 - transition));
		temp2 = scale32(temp2, transition);
	 
		accVertf = (float)temp1 + temp2;
	}
	// Just use the P2 value
	else
	{
		// Calculate the correct Z-axis data based on the orientation
		accVertf = accSmooth[YAW] + (float)(Config.AccZeroNormZ_P2 - Config.AccZero_P2[YAW]);		
	}
}

//***************************************************************
// Fill accADC with RPY data appropriate to the board orientation
// Nice as it would be, we cannot remove zeros here as this is the
// routine used by the zeroing calibration routine. Chicken | Egg.
// We also cannot merge P1 and P2 here as each have their own zeros.
//***************************************************************

void get_raw_accs(void)
{
	int16_t RawADC[NUMBEROFAXIS];
	uint8_t i;

	// Get data from MPU6050
	uint8_t Accs[6];

	// Get the i2c data from the MPU6050
	readI2CbyteArray(MPU60X0_DEFAULT_ADDRESS,MPU60X0_RA_ACCEL_XOUT_H,(uint8_t *)Accs,6);

	// Reassemble data into accADC array and down sample to reduce resolution and noise.
	// This notation is true to the chip, but not the board orientation.
	RawADC[ROLL] = (Accs[0] << 8) + Accs[1];
	RawADC[PITCH] = -((Accs[2] << 8) + Accs[3]);
	RawADC[YAW] = (Accs[4] << 8) + Accs[5];

	// Reorient the data as per the board orientation	
	for (i = 0; i < NUMBEROFAXIS; i++)
	{
		// Rearrange the sensors for both orientations
		accADC_P1[i] = RawADC[(int8_t)pgm_read_byte(&ACC_RPY_Order[Config.Orientation_P1][i])] >> 6;
		accADC_P2[i] = RawADC[(int8_t)pgm_read_byte(&ACC_RPY_Order[Config.Orientation_P2][i])] >> 6;
	}
}

//***************************************************************
// Calibration routines
// For V1.2 onwards we have to be able to calibrate both
// orientations when there are more than one.
//***************************************************************

void CalibrateAcc(int8_t type)
{
	uint8_t i;
	int16_t accZero[NUMBEROFAXIS] = {0,0,0};	// Used for calibrating Accs on ground

	// Calibrate acc
	// P2
	if (type == NORMAL)
	{
		// Work out which orientation we are calibrating.
		// Only need to do this if the orientations differ.
		// Just do P2 if orientations the same.
		// Will not save new calibration when different and not firmly in P1 or p2.
		if ((transition > 95) || (Config.P1_Reference == NO_ORIENT))
		{
			// Get average zero value (over 32 readings)
			for (i = 0; i < 32; i++)
			{
				get_raw_accs();						// Updates accADC_P1[] and accADC_P2[] (RPY)
				accZero[ROLL] += accADC_P2[ROLL];
				accZero[PITCH] += accADC_P2[PITCH];
				accZero[YAW] += accADC_P2[YAW];
				_delay_ms(10);						// Get a better acc average over time
			}
			
			// Average
			for (i = 0; i < NUMBEROFAXIS; i++)		// For selected axis in RPY order
			{
				// Round and divide by 32
				accZero[i] = ((accZero[i] + 16) >> 5);
			}

			// Reset zeros to normal cal
			Config.AccZero_P2[ROLL] = accZero[ROLL];
			Config.AccZero_P2[PITCH] = accZero[PITCH];
			Config.AccZeroNormZ_P2 = accZero[YAW];
			
			// Correct polarity of AccZeroNormZ as per orientation
			Config.AccZeroNormZ_P2 *= (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation_P2][YAW]);
			
			// Flag that normal cal done
			Config.Main_flags |= (1 << normal_cal_done_P2);
			
			// Save new calibration and flash LED for confirmation
			Save_Config_to_EEPROM();
			LED1 = 1;
			_delay_ms(500);
			LED1 = 0;
		}
		// P1
		else if (transition <= 5)
		{
			// Get average zero value (over 32 readings)
			for (i = 0; i < 32; i++)
			{
				get_raw_accs();						// Updates accADC_P1[] and accADC_P2[] (RPY)
				accZero[ROLL] += accADC_P1[ROLL];
				accZero[PITCH] += accADC_P1[PITCH];
				accZero[YAW] += accADC_P1[YAW];
				_delay_ms(10);						// Get a better acc average over time
			}
			
			// Average
			for (i = 0; i < NUMBEROFAXIS; i++)		// For selected axis in RPY order
			{
				// Round and divide by 32
				accZero[i] = ((accZero[i] + 16) >> 5);
			}

			// Reset zeros to normal cal
			Config.AccZero_P1[ROLL] = accZero[ROLL];
			Config.AccZero_P1[PITCH] = accZero[PITCH];
			Config.AccZeroNormZ_P1 = accZero[YAW];
			
			// Correct polarity of AccZeroNormZ as per orientation
			Config.AccZeroNormZ_P1 *= (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation_P1][YAW]);
			
			// Flag that normal cal done
			Config.Main_flags |= (1 << normal_cal_done_P1);

			// Save new calibration and flash LED for confirmation
			Save_Config_to_EEPROM();
			LED1 = 1;
			_delay_ms(500);
			LED1 = 0;
		}
	}

	else
	// Calibrate inverted acc
	{
		// P2 or same
		if ((transition > 95) || (Config.P1_Reference == NO_ORIENT))
		{
			// Only update the inverted cal value if preceded by a normal calibration
			if (Config.Main_flags & (1 << normal_cal_done_P2))
			{
				// Get average zero value (over 32 readings)
				Config.AccZeroInvZ_P2 = 0;

				for (i = 0; i < 32; i++)
				{
					get_raw_accs();					// Updates accADC_P1[] and accADC_P2[] (RPY)
					Config.AccZeroInvZ_P2 += accADC_P2[YAW];
					_delay_ms(10);					// Get a better acc average over time
				}

				// Round and divide by 32
				Config.AccZeroInvZ_P2 = ((Config.AccZeroInvZ_P2 + 16) >> 5);		// Inverted zero point
				
				// Correct polarity of AccZeroInvZ as per orientation
				Config.AccZeroInvZ_P2 *= (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation_P2][YAW]);

				// Test if board is actually inverted relative to board orientation.
				if (Config.AccZeroInvZ_P2 < 0)
				{
					// Reset zero to halfway between min and max Z
					Config.AccZeroDiff_P2 = ((Config.AccZeroNormZ_P2 - Config.AccZeroInvZ_P2) >> 1);
					
					 // Config.AccZero_P2[YAW] is now half-way in between
					Config.AccZero_P2[YAW] = Config.AccZeroNormZ_P2 - Config.AccZeroDiff_P2;

					// Flag that inverted cal done
					Config.Main_flags |= (1 << inv_cal_done_P2);

					// Save new calibration and flash LED for confirmation
					Save_Config_to_EEPROM();
					LED1 = 1;
					_delay_ms(500);
					LED1 = 0;
					
					// Chirp as well. The LED might be hard to see.
					menu_beep(5);
				}
			}
		} // Orientation-specific code
		
		// P1
		else 
		{
			// Only update the inverted cal value if preceded by a normal calibration
			if (Config.Main_flags & (1 << normal_cal_done_P1))
			{
				// Get average zero value (over 32 readings)
				Config.AccZeroInvZ_P1 = 0;

				for (i = 0; i < 32; i++)
				{
					get_raw_accs();					// Updates accADC_P1[] and accADC_P2[] (RPY)
					Config.AccZeroInvZ_P1 += accADC_P1[YAW];
					_delay_ms(10);					// Get a better acc average over time
				}

				// Round and divide by 32
				Config.AccZeroInvZ_P1 = ((Config.AccZeroInvZ_P1 + 16) >> 5);		// Inverted zero point
			
				// Correct polarity of AccZeroInvZ as per orientation
				Config.AccZeroInvZ_P1 *= (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation_P1][YAW]);

				// Test if board is actually inverted relative to board orientation.
				if (Config.AccZeroInvZ_P1 < 0)
				{
					// Reset zero to halfway between min and max Z
					Config.AccZeroDiff_P1 = ((Config.AccZeroNormZ_P1 - Config.AccZeroInvZ_P1) >> 1);
					
					// Config.AccZero_P1[YAW] is now half-way in between
					Config.AccZero_P1[YAW] = Config.AccZeroNormZ_P1 - Config.AccZeroDiff_P1;

					// Flag that inverted cal done
					Config.Main_flags |= (1 << inv_cal_done_P1);

					// Save new calibration and flash LED for confirmation
					Save_Config_to_EEPROM();
					LED1 = 1;
					_delay_ms(500);
					LED1 = 0;
				
					// Chirp as well. The LED might be hard to see.
					menu_beep(5);
				}
			}			
		}

	} // Calibrate inverted acc
}

//***************************************************************
// Set up the MPU6050 (Acc)
//***************************************************************

void init_i2c_accs(void)
{
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_PWR_MGMT_1, 0x01); // Gyro X clock, awake
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_ACCEL_CONFIG, ACCFS4G); // 4G full scale
}

