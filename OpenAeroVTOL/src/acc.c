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

//************************************************************
// Prototypes
//************************************************************

void ReadAcc(void);
void CalibrateAcc(int8_t type);
void get_raw_accs(void);

//************************************************************
// Defines
//************************************************************

// Polarity handling 
#ifdef KK21 
const int8_t Acc_Pol[5][3] PROGMEM =  // ROLL, PITCH, YAW
{
	{1,-1,1},		// Forward
	{1,-1,1},		// Vertical
	{-1,-1,-1},		// Upside down
	{-1,1,1},		// Aft
	{-1,-1,1},		// Sideways
};
#else
const int8_t Acc_Pol[5][3] PROGMEM =  // ROLL, PITCH, YAW
{
	{1,1,1},		// Forward
	{1,-1,-1},		// Vertical
	{-1,1,-1},		// Upside down
	{-1,-1,1},		// Aft
	{1,-1,1},		// Sideways
};
#endif

//************************************************************
// Code
//************************************************************

int16_t accADC[3];				// Holds Acc ADC values - alwys in RPY order
int16_t NormalAccZero;			// Holds Z acc in between normal and inverted cals
int16_t accVert = 0;			// Holds the level-zeroed Z-acc value. Used for height damping in hover only.

void ReadAcc()
{
	uint8_t i;

	get_raw_accs();				// Updates accADC[] (RPY)

	// Recalculate current accVert
	accVert = accADC[YAW] - Config.AccVertZero;

	// Use default zero for Acc-Z if inverse calibration not done yet
	// Actual zero is held in NormalAccZero waiting for inv calibration
	if (!(Main_flags & (1 << inv_cal_done)))
	{
#ifdef KK21 
		Config.AccZero[YAW] = 0;
#else
		Config.AccZero[YAW] = 643;
#endif
	}

	for (i=0;i<3;i++)			// For all axis (RPY)
	{
		// Remove offsets from acc outputs
		accADC[i] -= Config.AccZero[i];

		// Change polarity as per orientation mode
		accADC[i] *= (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][i]);
	}
}

void CalibrateAcc(int8_t type)
{
	uint8_t i;
	int16_t accZero[3] = {0,0,0};	// Used for calibrating Accs on ground
	int16_t temp;
	int16_t accZeroYaw = 0;

	// Calibrate acc
	if (type == NORMAL)
	{
		// Get average zero value (over 32 readings)
		for (i=0;i<32;i++)
		{
			get_raw_accs();			// Updates accADC[] (RPY - board)

			accZero[ROLL] += accADC[ROLL];
			accZero[PITCH] += accADC[PITCH];						
			accZero[YAW] += accADC[YAW];		

			_delay_ms(10);			// Get a better acc average over time
		}

		for (i=0;i<3;i++)			// For all axis in RPY order (always)
		{
			// Divide by 32
			accZero[i] = (accZero[i] >> 5);
			Config.AccZero[i] = accZero[i]; 
		}

		// Save upright Z-axis zero
		NormalAccZero = Config.AccZero[YAW];
		Config.AccVertZero = NormalAccZero;

		Main_flags |= (1 << normal_cal_done);
		Main_flags &= ~(1 << inv_cal_done);

		Save_Config_to_EEPROM();
	}

	else
	// Calibrate inverted acc
	{
		// Only update the cal value if preceeded by a normal calibration
		if (Main_flags & (1 << normal_cal_done))
		{
			// Get average zero value (over 32 readings)
			for (i=0;i<32;i++)
			{
				get_raw_accs();					// Updates gyroADC[] with board-oriented vales
				accZeroYaw += accADC[YAW];		
				_delay_ms(10);					// Get a better acc average over time
			}

			accZeroYaw = (accZeroYaw >> 5);		// Inverted zero point

			// Test if board is actually inverted relative to board orientation
#ifdef KK21 
			if ((accZeroYaw * (int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][YAW])) < 0) // Upside down
#else
			if ((((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][YAW]) == 1) && (accZeroYaw < Config.AccZero[YAW])) || // Forward, Aft and Sideways 
			    (((int8_t)pgm_read_byte(&Acc_Pol[Config.Orientation][YAW]) == -1) && (accZeroYaw > Config.AccZero[YAW])))  // Vertical and Upside down
#endif

			{
				// Reset zero to halfway between min and max Z
				temp = ((NormalAccZero - accZeroYaw) >> 1);
				Config.AccZero[YAW] = NormalAccZero - temp; // Config.AccZero[YAW] is now valid to use

				Main_flags |= (1 << inv_cal_done);
				Main_flags &= ~(1 << normal_cal_done);

				Save_Config_to_EEPROM();
			}
		}
	}
}

//***************************************************************
// Fill accADC with RPY data appropriate to the board orientation
//***************************************************************

void get_raw_accs(void)
{
// Get data from MPU6050 for KK2.1
#ifdef KK21
	uint8_t Accs[6];
	int16_t temp1, temp2;
	int16_t RawADC[3];

	// For KK2.1 boards, use the i2c data from the MPU6050
	readI2CbyteArray(MPU60X0_DEFAULT_ADDRESS,MPU60X0_RA_ACCEL_XOUT_H,(uint8_t *)Accs,6);

	// Reassemble data into accADC array and down sample to reduce resolution and noise
	// This notation is true to the chip, but not the board orientation

	temp1 = Accs[0] << 8;					// Accel X
	temp2 = Accs[1];
	RawADC[PITCH] = (temp1 + temp2) >> 7;

	temp1 = Accs[2] << 8;					// Accel Y
	temp2 = Accs[3];
	RawADC[ROLL] = (temp1 + temp2) >> 7;

	temp1 = Accs[4] << 8;					// Accel Z
	temp2 = Accs[5];
	RawADC[YAW] = (temp1 + temp2) >> 7;

	// Reorient the data as per the board orientation
	accADC[ROLL] 	= RawADC[(int8_t)pgm_read_byte(&ACC_RPY_Order[Config.Orientation][ROLL])];
	accADC[PITCH] 	= RawADC[(int8_t)pgm_read_byte(&ACC_RPY_Order[Config.Orientation][PITCH])];
	accADC[YAW]		= RawADC[(int8_t)pgm_read_byte(&ACC_RPY_Order[Config.Orientation][YAW])];

#else
	//	For the KK2.0, the order of the analog sensors is swapped in adc.c
	read_adc(AIN_Y_ACC);		// Read Y acc ADC4 (Roll)
	accADC[ROLL] = ADCW;

	read_adc(AIN_X_ACC);		// Read X acc ADC5 (Pitch)
	accADC[PITCH] = ADCW;

	read_adc(AIN_Z_ACC);		// Read Z acc ADC2 (inverted)
	accADC[YAW] = ADCW;

#endif
}

#ifdef KK21
void init_i2c_accs(void)
{
	// Configure accs
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_ACCEL_CONFIG, 0x18); 

	// Wake MPU6050
	writeI2Cbyte(MPU60X0_DEFAULT_ADDRESS, MPU60X0_RA_PWR_MGMT_1, 0x01); // Gyro X clock, awake
}
#endif
