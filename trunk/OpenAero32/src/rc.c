//*********************************************************************
//* rc.c
//*********************************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "board.h"
#include "mw.h"

//************************************************************
// Defines
//************************************************************

#define BREAKPOINT 1500	  // <-- make this a variable

//************************************************************
// Variables
//************************************************************

int16_t rcData[8] = { 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502 }; // Interval [1000;2000]

rcReadRawDataPtr rcReadRawFunc = NULL;  // Function to receive data from default receiver driver

//************************************************************
// Code
//************************************************************

uint16_t pwmReadRawRC(uint8_t chan)
{
    uint16_t data;

    data = pwmRead(cfg.rcmap[chan]);
    if (data < 750 || data > 2250)
	{
        data = cfg.defaultrc;
	}
    return data;
}

void computeRC(void)
{
    static int16_t rcData4Values[8][4], rcDataMean[8];
    static uint8_t rc4ValuesIndex = 0;
    uint8_t chan, a;

	// RC input corrections
	int16_t	roll_actual, flap_actual = 0;
	
   	static int16_t rcData4Scratch[4];		// Un-static these later...
	static uint16_t rcDataZeroScratch[4];
	static int32_t sticks[8];
	uint8_t axis, prop1, prop2;
	uint16_t tmp, tmp2;

	int16_t temp_chan;
	int16_t roll_raw_left, roll_raw_right;

    // Average RC data values
    rc4ValuesIndex++;
    for (chan = 0; chan < 8; chan++) 
	{
		rcData4Values[chan][rc4ValuesIndex % 4] = rcReadRawFunc(chan);
		rcDataMean[chan] = 0;
		for (a = 0; a < 4; a++)
		{
			rcDataMean[chan] += rcData4Values[chan][a];
		}
		rcDataMean[chan] = (rcDataMean[chan] + 2) / 4;
		if (rcDataMean[chan] < rcData[chan] - 3)
		{
			rcData[chan] = rcDataMean[chan] + 2;
		}
		if (rcDataMean[chan] > rcData[chan] + 3)
		{
			rcData[chan] = rcDataMean[chan] - 2;
		}
    }

	// PITCH & ROLL dynamic PID adjustment, depending on cfg.DynPIDchan value
	if (rcData[cfg.DynPIDchan] < cfg.DynPIDbreakpoint) //#define BREAKPOINT 1500	
	{
		prop2 = 100;
	} 
	else 
	{
		if (rcData[cfg.DynPIDchan] < 2000) 
		{
			prop2 = 100 - (uint16_t) cfg.dynThrPID * (rcData[cfg.DynPIDchan] - cfg.DynPIDbreakpoint) / (2000 - cfg.DynPIDbreakpoint);
		} 
		else 
		{
			prop2 = 100 - cfg.dynThrPID;
		}
	}

	// Reconstruct ailerons prior to rates and expo
	// to get actual roll stick input values
	// This requires accurate stick centering...
	roll_raw_left 	= rcData[ROLL] - cfg.midrc[ROLL];
	roll_raw_right 	= rcData[cfg.aileron2] - cfg.midrc[cfg.aileron2];

	// Un-mix ailerons from flaperons as required
	// Recover roll info based on flap config mode
	switch(cfg.flapmode)
	{
		// No flaperons - one aileron channel copied to both
		case BASIC_FLAP:
		 	// Aileron data
			rcData4Scratch[0] = rcData[ROLL];  			// Aileron	 
			rcData4Scratch[2] = rcData[ROLL];  			// Aileron2
			roll_actual = rcData[ROLL];
			// Flap data if set up
			if (cfg.flapchan != NOCHAN)
			{
				flap_actual = rcData[cfg.flapchan];		// Flap
			}
			else
			{
				flap_actual = cfg.defaultrc;			// Neutral flap
			}
			// Set up zeros for channels requiring scaling and expo
			rcDataZeroScratch[0] = cfg.midrc[ROLL]; 	// Roll
			rcDataZeroScratch[2] = cfg.midrc[ROLL];		// Aileron2
			break;

		// Flaperons - two ailerons with flaps pre-mixed in the TX
		case PREMIXED_FLAP:
			// Recreate actual roll signal from flaperons if available
			roll_actual = roll_raw_left + roll_raw_right;
			roll_actual = roll_actual >> 1;
			roll_actual += cfg.defaultrc;				// Restore offset using system center
			rcData4Scratch[0] = roll_actual;			// Aileron
			rcData4Scratch[2] = roll_actual;			// Aileron2

			// Set up zeros
			rcDataZeroScratch[0] = cfg.defaultrc; 		// Roll
			rcDataZeroScratch[2] = cfg.defaultrc;		// Aileron2
			
			// Recreate actual flap signal from flaperons if available
			flap_actual = roll_raw_left - roll_raw_right;
			flap_actual = flap_actual >> 1;
			flap_actual += cfg.defaultrc;				// Restore offset using system center
			break;
		
		// Flaperons - two independant aileron channels + one flap input on cfg.flapchan
		case ADV_FLAP:
		 	rcData4Scratch[0] = rcData[ROLL];  			// Aileron	 
			rcData4Scratch[2] = rcData[cfg.aileron2];  	// Aileron2	 
			roll_actual = rcData[ROLL];
			flap_actual = rcData[cfg.flapchan];			// From RC flap channel
			
			// Set up zeros
			rcDataZeroScratch[0] = cfg.midrc[ROLL]; 	// Roll
			rcDataZeroScratch[2] = cfg.midrc[cfg.aileron2];	// Aileron2
			break;
		
		default:
			break;
	}

	// Fiddle rcData so that second aileron channel handled properly for expo
	rcData4Scratch[1] = rcData[PITCH];					// Elevator
	rcData4Scratch[3] = rcData[YAW];					// Yaw

	// Set up zeros for remaining channels
	rcDataZeroScratch[1] = cfg.midrc[PITCH];			// Elevator
	rcDataZeroScratch[3] = cfg.midrc[YAW];				// Yaw
	
	// Deadband, stick rates and expo	[Roll, Elevator, Aileron2, Yaw]
	for (axis = 0; axis < 4; axis++) 
	{
		// RC offset is removed here
		tmp = min(abs(rcData4Scratch[axis] - rcDataZeroScratch[axis]), 500);
		if (axis != 3) 
		{ // ROLL, Aileron2 & PITCH
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

		if (rcData4Scratch[axis] < rcDataZeroScratch[axis])
		{
			rcCommand[axis] = -rcCommand[axis];		 // This is stupid... but it will do for now
		}
    }
 
	// Rearrange channels back into RC order
	temp_chan = rcCommand[2];						// Aileron 2
	rcCommand[YAW] = rcCommand[3];					// Yaw back to normal position

	// Recover second aileron channel for rcCommand[]
	if (cfg.aileron2 != NOCHAN)
	{
		rcCommand[cfg.aileron2] = temp_chan;
	}
	else 
	{
		rcCommand[cfg.aileron2] = 0;
	}
	
	// Recover flap channel for rcCommand[]
	switch(cfg.flapmode)
	{
		case BASIC_FLAP: 
			if (cfg.flapchan != NOCHAN)
			{
				rcCommand[cfg.flapchan] = flap_actual - cfg.midrc[cfg.flapchan];
			}
			else
			{
				rcCommand[cfg.flapchan] = 0;
			}
			break;
		case PREMIXED_FLAP:
			rcCommand[cfg.flapchan] = flap_actual - cfg.defaultrc;
			break;
		case ADV_FLAP:
			rcCommand[cfg.flapchan] = flap_actual - cfg.midrc[cfg.flapchan];
			break;			
		default:
			break;
	}
	
	// Throttle scaling
    tmp = constrain(rcData[THROTTLE], cfg.mincheck, 2000);
    tmp = (uint32_t) (tmp - cfg.mincheck) * 1000 / (2000 - cfg.mincheck);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
	// TODO note this only works for rcCommand[]
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

	// Calibrate sticks if requested
    if (calibratingS > 0) 
	{
		for (chan = 0; chan < 8; chan++) 
        {   // Reset sticks[chan] at start of calibration
            if (calibratingS == 50)
			{
                sticks[chan] = 0;
            }
			// Sum up 50 readings
            sticks[chan] += rcData[chan];
        }
		
        // Calculate average and store values in EEPROM at end of calibration
        if (calibratingS == 1) 
		{
			for (chan = 0; chan < 8; chan++) 
			{
				cfg.midrc[chan] = sticks[chan] / 50;
			}
            writeParams(1);      // write cfg.midrc in EEPROM
        }
		// Keep going...
        calibratingS--;
    }
	else
	{
		  calibratingS = 0;
	}	
}
