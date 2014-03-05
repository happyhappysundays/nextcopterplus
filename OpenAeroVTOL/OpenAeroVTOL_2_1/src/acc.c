//***********************************************************
//* acc.c
//*
//* This fuction populates following vars:
//* accADC[] holds the raw ADC values
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
	{ROLL, PITCH, YAW}, // Normal
	{YAW, ROLL, PITCH}, // Vertical
	{ROLL, PITCH, YAW},	// Upside down
	{ROLL, PITCH, YAW},	// Aft
	{PITCH, ROLL, YAW},	// Sideways
	{ROLL, YAW, PITCH}, // Rear/bottom (PitchUp)
};

// These are the polarities to return them to the default
const int8_t Acc_Pol[NUMBEROFORIENTS][NUMBEROFAXIS] PROGMEM =
{
// 	 ROLL, PITCH, YAW
	{-1,-1,1},		// Forward 
	{-1,1,-1},		// Vertical 
	{1,-1,-1},		// Upside down 
	{1,1,1},		// Aft 
	{-1,1,1},		// Sideways 
	{-1,-1,-1},		// Rear/bottom (PitchUp)
};

//************************************************************
// Code
//************************************************************

int16_t accADC[NUMBEROFAXIS];	// Holds Acc ADC values - alwys in RPY order
int16_t accVert = 0;			// Holds the level-zeroed Z-acc value. Used for height damping in hover only.

void ReadAcc()
{
	uint8_t i;

	get_raw_accs();				// Updates accADC[] (RPY)

	// Use default Config.AccZero for Acc-Z if inverse calibration not done yet
	// Actual zero is held in Config.AccZeroNormZ waiting for inv calibration
	if (!(Config.Main_flags & (1 << inv_cal_done)))
	{
#ifdef KK21 
		Config.AccZero[YAW] = 0;
#else
		Config.AccZero[YAW] = 643;
#endif
	}

	// If inverted cal done, Config.AccZeroNormZ and Config.AccZeroDiff have
	// valid values
	else
	{
		Config.AccZero[YAW] = Config.AccZeroNormZ - Config.AccZeroDiff;
	}

	for (i=0; i<NUMBEROFAXIS; i++)
	{
		// Remove offsets from acc outputs
		accADC[i] -= Config.AccZero[i];

		// Change polarity
		accADC[i] *= (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][i]);
	}


	// Recalculate current accVert using filtered acc value
	// Note that AccSmooth[YAW] is already zeroed around 1G so we have to re-add 
	// the zero back here so that Config.AccZeroNormZ subtracts the correct amount
	 accVert = accSmooth[YAW] - Config.AccZero[YAW] + Config.AccZeroNormZ ;

}

//***************************************************************
// Fill accADC with RPY data appropriate to the board orientation
//***************************************************************

void get_raw_accs(void)
{
	int16_t RawADC[NUMBEROFAXIS];
	uint8_t i;

#ifdef KK21
	// Get data from MPU6050 for KK2.1
	uint8_t Accs[6];
	int16_t temp1, temp2;

	// For KK2.1 boards, use the i2c data from the MPU6050
	readI2CbyteArray(MPU60X0_DEFAULT_ADDRESS,MPU60X0_RA_ACCEL_XOUT_H,(uint8_t *)Accs,6);

	// Reassemble data into accADC array and down sample to reduce resolution and noise
	// This notation is true to the chip, but not the board orientation

	temp1 = Accs[0] << 8;					// Accel X
	temp2 = Accs[1];
	RawADC[ROLL] = (temp1 + temp2) >> 6;

	temp1 = Accs[2] << 8;					// Accel Y (note: reversed polarity wrt KK2.0
	temp2 = Accs[3];
	RawADC[PITCH] = -((temp1 + temp2) >> 6);

	temp1 = Accs[4] << 8;					// Accel Z
	temp2 = Accs[5];
	RawADC[YAW] = (temp1 + temp2) >> 6;

#else
	// Get data from analog inputs for KK2.0
	read_adc(AIN_Y_ACC);		// Read Y acc ADC4 (Roll)
	RawADC[ROLL] = ADCW;

	read_adc(AIN_X_ACC);		// Read X acc ADC5 (Pitch)
	RawADC[PITCH] = ADCW;

	read_adc(AIN_Z_ACC);		// Read Z acc ADC2 (Z)
	RawADC[YAW] = ADCW;

#endif

	// Reorient the data as per the board orientation	
	for (i=0; i<NUMBEROFAXIS; i++)
	{
		// Rearrange the sensors
		accADC[i] = RawADC[(int8_t)pgm_read_byte(&ACC_RPY_Order[Config.Orientation][i])];
	}
}

//***************************************************************
// Calibration routines
//***************************************************************

void CalibrateAcc(int8_t type)
{
	uint8_t i;
	int16_t accZero[NUMBEROFAXIS] = {0,0,0};	// Used for calibrating Accs on ground

	// Calibrate acc
	if (type == NORMAL)
	{
		// Get average zero value (over 32 readings)
		for (i=0; i<32; i++)
		{
			get_raw_accs();						// Updates accADC[] with signals reoriented into (RPY)

			accZero[ROLL] += accADC[ROLL];
			accZero[PITCH] += accADC[PITCH];						
			accZero[YAW] += accADC[YAW];		

			_delay_ms(10);						// Get a better acc average over time
		}

		for (i=0; i<NUMBEROFAXIS; i++)			// For selected axis in RPY order
		{
			// Round and divide by 32
			accZero[i] = ((accZero[i] + 16) >> 5);
		}

		// Reset zeros to normal cal
		Config.AccZero[ROLL] = accZero[ROLL]; 
		Config.AccZero[PITCH] = accZero[PITCH]; 
		Config.AccZeroNormZ = accZero[YAW]; 

		// Flag that normal cal done
		Config.Main_flags |= (1 << normal_cal_done);

		Save_Config_to_EEPROM();
	}

	else
	// Calibrate inverted acc
	{
		// Only update the inverted cal value if preceeded by a normal calibration
		if (Config.Main_flags & (1 << normal_cal_done))
		{
			// Get average zero value (over 32 readings)
			Config.AccZeroInvZ = 0;

			for (i=0; i<32; i++)
			{
				get_raw_accs();					// Updates gyroADC[] with reoriented vales
				Config.AccZeroInvZ += accADC[YAW];		
				_delay_ms(10);					// Get a better acc average over time
			}

			// Round and divide by 32
			Config.AccZeroInvZ = ((Config.AccZeroInvZ + 16) >> 5);		// Inverted zero point

			// Test if board is actually inverted relative to board orientation.
			// We have to do this as get_raw_accs() does not reorient the data
#ifdef KK21 
			if ((Config.AccZeroInvZ * (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][YAW])) < 0) // Upside down
#else
			if ((((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][YAW]) == 1) && (Config.AccZeroInvZ < Config.AccZero[YAW])) || // Forward, Aft and Sideways 
			    (((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][YAW]) == -1) && (Config.AccZeroInvZ > Config.AccZero[YAW])))  // Vertical and Upside down
#endif
			{

				// Reset zero to halfway between min and max Z
				Config.AccZeroDiff = ((Config.AccZeroNormZ - Config.AccZeroInvZ) >> 1);
				Config.AccZero[YAW] = Config.AccZeroNormZ - Config.AccZeroDiff; // Config.AccZero[YAW] is now valid to use

				// Flag that inverted cal done
				Config.Main_flags |= (1 << inv_cal_done);

				Save_Config_to_EEPROM();
			}
		}
	}
}

#ifdef KK21
//***************************************************************
// Set up the MPU6050 (Acc)
//***************************************************************

void init_i2c_accs(void)
{
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_PWR_MGMT_1, 0x01); // Gyro X clock, awake
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_ACCEL_CONFIG, ACCFS4G); // 4G full scale
}
#endif

