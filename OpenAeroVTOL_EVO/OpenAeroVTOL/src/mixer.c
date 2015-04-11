//***********************************************************
//* mixer.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include "compiledefs.h"
#include <string.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "typedefs.h"
#include "io_cfg.h"
#include "rc.h"
#include "isr.h"
#include "servos.h"
#include "pid.h"
#include "main.h"
#include <avr/pgmspace.h> 
#include "mixer.h"
#include "imu.h"
#include "init.h"

//************************************************************
// Prototypes
//************************************************************

void ProcessMixer(void);
void UpdateServos(void);
void UpdateLimits(void);
void get_preset_mix (const channel_t*);
int16_t scale32(int16_t value16, int16_t multiplier16);
int16_t scale_percent(int8_t value);
int16_t scale_percent_nooffset(int8_t value);

//************************************************************
// Defines
//************************************************************

#define MIX_OUTPUTS 8

//************************************************************
// Defines
//************************************************************

// Throttle volume curves
// Why 101 steps? Well, both 0% and 100% transition values are valid...

const int8_t SIN[101] PROGMEM = 
			{0,2,3,5,6,8,10,11,13,14,
			16,17,19,20,22,23,25,26,28,29,
			31,32,34,35,37,38,40,41,43,44,
			45,47,48,50,51,52,54,55,56,58,
			59,60,61,63,64,65,66,67,68,70,
			71,72,73,74,75,76,77,78,79,80,
			81,82,83,84,84,85,86,87,88,88,
			89,90,90,91,92,92,93,94,94,95,
			95,96,96,96,97,97,98,98,98,99,
			99,99,99,99,100,100,100,100,100,100,
			100};

const int8_t SQRTSIN[101] PROGMEM = 
			{0,13,18,22,25,28,31,33,35,38,
			40,41,43,45,47,48,50,51,53,54,
			56,57,58,59,61,62,63,64,65,66,
			67,68,69,70,71,72,73,74,75,76,
			77,77,78,79,80,81,81,82,83,83,
			84,85,85,86,87,87,88,88,89,89,
			90,90,91,91,92,92,93,93,94,94,
			94,95,95,95,96,96,96,97,97,97,
			98,98,98,98,98,99,99,99,99,99,
			99,99,100,100,100,100,100,100,100,100,
			100};

#define EXT_SOURCE 8	// Offset for indexing sensor sources

//************************************************************
// Code
//************************************************************

void ProcessMixer(void)
{
	uint8_t i = 0;
	uint8_t j = 0;
	int16_t P1_solution = 0;
	int16_t P2_solution = 0;

	int16_t temp1 = 0;
	int16_t temp2 = 0;
	int16_t	temp3 = 0;
	int16_t	Step1 = 0;
	int16_t	Step2 = 0;
	int8_t	itemp8 = 0;

	// Copy the sensor data to an array for easy indexing - acc data is from accSmooth, increased to reasonable rates
	temp1 = (int16_t)accSmooth[ROLL] << 3;
	temp2 = (int16_t)accSmooth[PITCH] << 3;
	int16_t	SensorDataP1[7] = {PID_Gyros[P1][ROLL], PID_Gyros[P1][PITCH], PID_Gyros[P1][YAW], temp1, temp2, PID_ACCs[P1][ROLL], PID_ACCs[P1][PITCH]};
	int16_t	SensorDataP2[7] = {PID_Gyros[P2][ROLL], PID_Gyros[P2][PITCH], PID_Gyros[P2][YAW], temp1, temp2, PID_ACCs[P2][ROLL], PID_ACCs[P2][PITCH]}; 

	//************************************************************
	// Main mix loop - sensors, RC inputs and other channels
	//************************************************************

	for (i = 0; i < MIX_OUTPUTS; i++)
	{
		//************************************************************
		// Zero each channel value to start
		//************************************************************

		P1_solution = 0;
		P2_solution = 0;

		//************************************************************
		// Mix in gyros
		//************************************************************ 

		// P1 gyros
		if (transition < 100)
		{
			switch (Config.Channel[i].P1_Roll_gyro) 
			{
				case OFF:
					break;
				case ON:
					if (Config.Channel[i].P1_aileron_volume < 0 )
					{
						P1_solution = P1_solution + PID_Gyros[P1][ROLL];		// Reverse if volume negative
					}
					else
					{
						P1_solution = P1_solution - PID_Gyros[P1][ROLL];
					}
					break;
				case SCALE:
					P1_solution = P1_solution - scale32(PID_Gyros[P1][ROLL], Config.Channel[i].P1_aileron_volume * 5); 
					break;
				default:
					break;	
			}

			switch (Config.Channel[i].P1_Pitch_gyro)
			{
				case OFF:
					break;
				case ON:
					if (Config.Channel[i].P1_elevator_volume < 0 )
					{
						P1_solution = P1_solution - PID_Gyros[P1][PITCH];		// Reverse if volume negative
					}
					else
					{
						P1_solution = P1_solution + PID_Gyros[P1][PITCH];
					}
					break;
				case SCALE:
					P1_solution = P1_solution + scale32(PID_Gyros[P1][PITCH], Config.Channel[i].P1_elevator_volume * 5);
					break;
				default:
					break;
			}

			switch (Config.Channel[i].P1_Yaw_gyro)
			{
				case OFF:
					break;
				case ON:
					if (Config.Channel[i].P1_rudder_volume < 0 )
					{
						P1_solution = P1_solution - PID_Gyros[P1][YAW];			// Reverse if volume negative
					}
					else
					{
						P1_solution = P1_solution + PID_Gyros[P1][YAW];
					}
					break;
				case SCALE:
					P1_solution = P1_solution + scale32(PID_Gyros[P1][YAW], Config.Channel[i].P1_rudder_volume * 5);
					break;
				default:
					break;
			}
		}

		// P2 gyros
		if (transition > 0)
		{
			switch (Config.Channel[i].P2_Roll_gyro)
			{
				case OFF:
					break;
				case ON:
					if (Config.Channel[i].P2_aileron_volume < 0 )
					{
						P2_solution = P2_solution + PID_Gyros[P2][ROLL];		// Reverse if volume negative
					}
					else
					{
						P2_solution = P2_solution - PID_Gyros[P2][ROLL];
					}
					break;
				case SCALE:
					P2_solution = P2_solution - scale32(PID_Gyros[P2][ROLL], Config.Channel[i].P2_aileron_volume * 5);
					break;
				default:
					break;
			}

			switch (Config.Channel[i].P2_Pitch_gyro)
			{
				case OFF:
					break;
				case ON:
					if (Config.Channel[i].P2_elevator_volume < 0 )
					{
						P2_solution = P2_solution - PID_Gyros[P2][PITCH];		// Reverse if volume negative
					}
					else
					{
						P2_solution = P2_solution + PID_Gyros[P2][PITCH];
					}
					break;
				case SCALE:
					P2_solution = P2_solution + scale32(PID_Gyros[P2][PITCH], Config.Channel[i].P2_elevator_volume * 5);
					break;
				default:
					break;
			}

			switch (Config.Channel[i].P2_Yaw_gyro)
			{
				case OFF:
					break;
				case ON:
					if (Config.Channel[i].P2_rudder_volume < 0 )
					{
						P2_solution = P2_solution - PID_Gyros[P2][YAW];			// Reverse if volume negative
					}
					else
					{
						P2_solution = P2_solution + PID_Gyros[P2][YAW];
					}
					break;
				case SCALE:
					P2_solution = P2_solution + scale32(PID_Gyros[P2][YAW], Config.Channel[i].P2_rudder_volume * 5);
					break;
				default:
					break;
			}
		}

		//************************************************************
		// Mix in accelerometers
		//************************************************************ 
		// P1
		if (transition < 100)
		{
			switch (Config.Channel[i].P1_Roll_acc)
			{
				case OFF:
					break;
				case ON:
					if (Config.Channel[i].P1_aileron_volume < 0 )
					{
						P1_solution = P1_solution + PID_ACCs[P1][ROLL];			// Reverse if volume negative
					}
					else
					{
						P1_solution = P1_solution - PID_ACCs[P1][ROLL];			// or simply add
					}
					break;
				case SCALE:
					P1_solution = P1_solution - scale32(PID_ACCs[P1][ROLL], Config.Channel[i].P1_aileron_volume * 5);
					break;
				default:
					break;
			}			

			switch (Config.Channel[i].P1_Pitch_acc)
			{
				case OFF:
					break;
				case ON:
					if (Config.Channel[i].P1_elevator_volume < 0 )
					{
						P1_solution = P1_solution - PID_ACCs[P1][PITCH];		// Reverse if volume negative
					}
					else
					{
						P1_solution = P1_solution + PID_ACCs[P1][PITCH];
					}
					break;
				case SCALE:
					P1_solution = P1_solution + scale32(PID_ACCs[P1][PITCH], Config.Channel[i].P1_elevator_volume * 5);
					break;
				default:
					break;
			}

			switch (Config.Channel[i].P1_Z_delta_acc)
			{
				case OFF:
					break;
				case ON:
					if (Config.Channel[i].P1_throttle_volume < 0 )
					{
						P1_solution = P1_solution + PID_ACCs[P1][YAW];			// Reverse if volume negative
					}
					else
					{
						P1_solution = P1_solution - PID_ACCs[P1][YAW];
					}
					break;
				case SCALE:
					P1_solution = P1_solution - scale32(PID_ACCs[P1][YAW], Config.Channel[i].P1_throttle_volume * 5);
					break;
				default:
					break;
			}
		}

		// P2
		if (transition > 0)
		{
			switch (Config.Channel[i].P2_Roll_acc)
			{
				case OFF:
					break;
				case ON:
					if (Config.Channel[i].P2_aileron_volume < 0 )
					{
						P2_solution = P2_solution + PID_ACCs[P2][ROLL];			// Reverse if volume negative
					}
					else
					{
						P2_solution = P2_solution - PID_ACCs[P2][ROLL];			// or simply add
					}
					break;
				case SCALE:
					P2_solution = P2_solution - scale32(PID_ACCs[P2][ROLL], Config.Channel[i].P2_aileron_volume * 5);
					break;
				default:
					break;
			}

			switch (Config.Channel[i].P2_Pitch_acc)
			{
				case OFF:
					break;
				case ON:
					if (Config.Channel[i].P2_elevator_volume < 0 )
					{

						P2_solution = P2_solution - PID_ACCs[P2][PITCH];		// Reverse if volume negative
					}
					else
					{
						P2_solution = P2_solution + PID_ACCs[P2][PITCH];
					}
					break;
				case SCALE:
					P2_solution = P2_solution + scale32(PID_ACCs[P2][PITCH], Config.Channel[i].P2_elevator_volume * 5);
					break;
				default:
					break;
			}

			switch (Config.Channel[i].P2_Z_delta_acc)
			{
				case OFF:
					break;
				case ON:
					if (Config.Channel[i].P2_throttle_volume < 0 )
					{
						P2_solution = P2_solution + PID_ACCs[P2][YAW];			// Reverse if volume negative
					}
					else
					{
						P2_solution = P2_solution - PID_ACCs[P2][YAW];
					}
					break;
				case SCALE:
					P2_solution = P2_solution - scale32(PID_ACCs[P2][YAW], Config.Channel[i].P2_throttle_volume * 5);
					break;
				default:
					break;
			}
		}

		//************************************************************
		// Process mixers
		//************************************************************ 

		// Mix in other outputs here (P1)
		if (transition < 100)
		{
			// Mix in dedicated RC sources - aileron, elevator and rudder
			if (Config.Channel[i].P1_aileron_volume != 0) 					// Mix in dedicated aileron
			{
				temp2 = scale32(RCinputs[AILERON], Config.Channel[i].P1_aileron_volume);
				P1_solution = P1_solution + temp2;
			}
			if (Config.Channel[i].P1_elevator_volume != 0) 					// Mix in dedicated elevator
			{
				temp2 = scale32(RCinputs[ELEVATOR], Config.Channel[i].P1_elevator_volume);
				P1_solution = P1_solution + temp2;
			}
			if (Config.Channel[i].P1_rudder_volume != 0) 					// Mix in dedicated rudder
			{
				temp2 = scale32(RCinputs[RUDDER], Config.Channel[i].P1_rudder_volume);
				P1_solution = P1_solution + temp2;
			}

			// Other sources
			if ((Config.Channel[i].P1_source_a_volume != 0) && (Config.Channel[i].P1_source_a != NOMIX)) // Mix in first extra source
			{
				// Is the source a sensor?
				if (Config.Channel[i].P1_source_a > (MAX_RC_CHANNELS - 1))
				{
					temp2 = SensorDataP1[Config.Channel[i].P1_source_a - EXT_SOURCE];
				}
				// Is the source an RC input?
				else
				{
					// Yes, calculate RC channel number from source number and return RC value
					temp2 = RCinputs[Config.Channel[i].P1_source_a];
				}

				temp2 = scale32(temp2, Config.Channel[i].P1_source_a_volume);
				P1_solution = P1_solution + temp2;
			}
			if ((Config.Channel[i].P1_source_b_volume != 0) && (Config.Channel[i].P1_source_b != NOMIX)) // Mix in second extra source
			{
				// Is the source a sensor?
				if (Config.Channel[i].P1_source_b > (MAX_RC_CHANNELS - 1))
				{
					temp2 = SensorDataP1[Config.Channel[i].P1_source_b - EXT_SOURCE];
				}
				// Is the source an RC input?
				else
				{
					temp2 = RCinputs[Config.Channel[i].P1_source_b];
				}

				temp2 = scale32(temp2, Config.Channel[i].P1_source_b_volume);
				P1_solution = P1_solution + temp2;
			}
		}

		// Mix in other outputs here (P2)
		if (transition > 0)	
		{
			// Mix in dedicated RC sources - aileron, elevator and rudder
			if (Config.Channel[i].P2_aileron_volume != 0) 					// Mix in dedicated aileron
			{
				temp2 = scale32(RCinputs[AILERON], Config.Channel[i].P2_aileron_volume);
				P2_solution = P2_solution + temp2;
			}
			if (Config.Channel[i].P2_elevator_volume != 0) 					// Mix in dedicated elevator
			{
				temp2 = scale32(RCinputs[ELEVATOR], Config.Channel[i].P2_elevator_volume);
				P2_solution = P2_solution + temp2;
			}
			if (Config.Channel[i].P2_rudder_volume != 0) 					// Mix in dedicated rudder
			{
				temp2 = scale32(RCinputs[RUDDER], Config.Channel[i].P2_rudder_volume);
				P2_solution = P2_solution + temp2;
			}

			// Other sources
			if ((Config.Channel[i].P2_source_a_volume != 0) && (Config.Channel[i].P2_source_a != NOMIX)) // Mix in first extra source
			{
				// Is the source a sensor?
				if (Config.Channel[i].P2_source_a > (MAX_RC_CHANNELS - 1))
				{
					temp2 = SensorDataP2[Config.Channel[i].P2_source_a - EXT_SOURCE];
				}
				// Is the source an RC input?
				else 
				{
					temp2 = RCinputs[Config.Channel[i].P2_source_a];
				}

				temp2 = scale32(temp2, Config.Channel[i].P2_source_a_volume);
				P2_solution = P2_solution + temp2;
			}
			if ((Config.Channel[i].P2_source_b_volume != 0) && (Config.Channel[i].P2_source_b != NOMIX)) // Mix in second extra source
			{
				// Is the source a sensor?
				if (Config.Channel[i].P2_source_b > (MAX_RC_CHANNELS - 1))
				{
					temp2 = SensorDataP2[Config.Channel[i].P2_source_b - EXT_SOURCE];
				}
				// Is the source an RC input?
				else
				{
					temp2 = RCinputs[Config.Channel[i].P2_source_b];
				}

				temp2 = scale32(temp2, Config.Channel[i].P2_source_b_volume);
				P2_solution = P2_solution + temp2;
			}
		}
			
		// Save solution for this channel. Note that this contains cross-mixed data from the *last* cycle
		Config.Channel[i].P1_value = P1_solution;
		Config.Channel[i].P2_value = P2_solution;

	} // Mixer loop: for (i = 0; i < MIX_OUTPUTS; i++)

	//************************************************************
	// Mixer transition code
	//************************************************************ 

	// Convert number to percentage (0 to 100%)
	if (Config.TransitionSpeed != 0) 
	{
		// transition_counter counts from 0 to 100 (101 steps)
		transition = transition_counter;
	}

	// Recalculate P1 values based on transition stage
	for (i = 0; i < MIX_OUTPUTS; i++)
	{
		// Speed up the easy ones :)
		if (transition == 0)
		{
			temp1 = Config.Channel[i].P1_value;
		}
		else if (transition >= 100)
		{
			temp1 = Config.Channel[i].P2_value;
		}
		else
		{
			// Get source channel value
			temp1 = Config.Channel[i].P1_value;
			temp1 = scale32(temp1, (100 - transition));

			// Get destination channel value
			temp2 = Config.Channel[i].P2_value;
			temp2 = scale32(temp2, transition);

			// Sum the mixers
			temp1 = temp1 + temp2;
		}
		// Save transitioned solution into P1
		Config.Channel[i].P1_value = temp1;
	}  

	//************************************************************
	// Groovy throttle curve handling. Must be after the transition.
	// Uses the transition value, but is not part of the transition
	// mixer. Linear or Sine curve. Reverse Sine done automatically
	//************************************************************ 

	for (i = 0; i < MIX_OUTPUTS; i++)
	{
		// Ignore if both throttle volumes are 0% (no throttle)
		if 	(!((Config.Channel[i].P1_throttle_volume == 0) && 
			(Config.Channel[i].P2_throttle_volume == 0)))
		{
			// Only process if there is a curve
			if (Config.Channel[i].P1_throttle_volume != Config.Channel[i].P2_throttle_volume)
			{
				// Calculate step difference in 1/100ths and round
				temp1 = (Config.Channel[i].P2_throttle_volume - Config.Channel[i].P1_throttle_volume);
				temp1 = temp1 << 7; 						// Multiply by 128 so divide gives reasonable step values
				Step1 = temp1 / 100;	

				// Set start (P1) point
				temp2 = Config.Channel[i].P1_throttle_volume; // Promote to 16 bits
				temp2 = temp2 << 7;

				// Linear vs. Sinusoidal calculation
				if (Config.Channel[i].Throttle_curve == LINEAR)
				{
					// Multiply [transition] steps (0 to 100)
					temp3 = temp2 + (Step1 * transition);
				}

				// SINE
				else if (Config.Channel[i].Throttle_curve == SINE)
				{
					// Choose between SINE and COSINE
					// If P2 less than P1, COSINE (reverse SINE) is the one we want
					if (Step1 < 0)
					{ 
						// Multiply SIN[100 - transition] steps (0 to 100)
						temp3 = 100 - (int8_t)pgm_read_byte(&SIN[100 - (int8_t)transition]);
					}
					// If P2 greater than P1, SINE is the one we want
					else
					{
						// Multiply SIN[transition] steps (0 to 100)
						temp3 = (int8_t)pgm_read_byte(&SIN[(int8_t)transition]);
					}

					// Get SINE% (temp2) of difference in volumes (Step1)
					// Step1 is already in 100ths of the difference * 128
					// temp1 is the start volume * 128
					temp3 = temp2 + (Step1 * temp3);
				}
				// SQRT SINE
				else
				{
					// Choose between SQRT SINE and SQRT COSINE
					// If P2 less than P1, COSINE (reverse SINE) is the one we want
					if (Step1 < 0)
					{ 
						// Multiply SQRTSIN[100 - transition] steps (0 to 100)
						temp3 = 100 - (int8_t)pgm_read_byte(&SQRTSIN[100 - (int8_t)transition]);
					}
					// If P2 greater than P1, SINE is the one we want
					else
					{
						// Multiply SQRTSIN[transition] steps (0 to 100)
						temp3 = (int8_t)pgm_read_byte(&SQRTSIN[(int8_t)transition]);
					}

					// Get SINE% (temp2) of difference in volumes (Step1)
					// Step1 is already in 100ths of the difference * 128
					// temp1 is the start volume * 128
					temp3 = temp2 + (Step1 * temp3);
				}

				// Round, then rescale to normal value
				temp3 = temp3 + 64;
				temp3 = temp3 >> 7;
			}
			
			// No curve
			else
			{
				// Just use the value of P1 volume as there is no curve
				temp3 = Config.Channel[i].P1_throttle_volume; // Promote to 16 bits
			}

			// Calculate actual throttle value to the curve
			temp3 = scale32(MonopolarThrottle, temp3);

			// At this point, the throttle values are 0 to 2500 (+/-150%)
			// Re-scale throttle values back to neutral-centered system values (+/-1250) 
			// and set the minimum throttle point to 1.1ms.
			// A THROTTLEMIN value of 1000 will result in 2750, or 1.1ms
			temp3 = temp3 - THROTTLEMIN;

			// Add offset to channel value
			Config.Channel[i].P1_value += temp3;

		} // No throttle
		
		// No throttles, so clamp to THROTTLEMIN if flagged as a motor
		else if (Config.Channel[i].Motor_marker == MOTOR)
		{
			Config.Channel[i].P1_value = -THROTTLEOFFSET; // 3750-1250 = 2500 = 1.0ms
		}
	}

	//************************************************************
	// Per-channel 3-point offset needs to be after the transition  
	// loop as it is non-linear, unlike the transition.
	//************************************************************ 

	for (i = 0; i < MIX_OUTPUTS; i++)
	{
		// Simplify if all are the same
		if (!((Config.Channel[i].P1_offset == Config.Channel[i].P1n_offset) &&
		 	 (Config.Channel[i].P2_offset == Config.Channel[i].P1n_offset)))
		{
			// Work out distance to cover over stage 1 (P1 to P1.n)
			temp1 = Config.Channel[i].P1n_offset - Config.Channel[i].P1_offset;
			temp1 = temp1 << 7; // Multiply by 128 so divide gives reasonable step values

			// Divide distance into steps
			temp2 = Config.Channel[i].P1n_position; 
			Step1 = ((temp1 + (temp2 >> 1)) / temp2) ; // Divide and round result
		
			// Work out distance to cover over stage 2 (P1.n to P2)
			temp2 = Config.Channel[i].P2_offset - Config.Channel[i].P1n_offset;
			temp2 = temp2 << 7;

			// Divide distance into steps
			temp1 = (100 - Config.Channel[i].P1n_position); 
			Step2 = ((temp2 + (temp1 >> 1)) / temp1) ; // Divide and round result	

			// Set start (P1) point
			temp3 = Config.Channel[i].P1_offset; // Promote to 16bits
			temp3 = temp3 << 7;

			// Count up transition steps of the appropriate step size
			for (j = 0; j < transition; j++)
			{
				// If in stage 1 use Step1 size
				if (j < Config.Channel[i].P1n_position)
				{
					temp3 += Step1;
				}
				// If in stage 2 use Step2 size
				else
				{
					temp3 += Step2;
				}
			}

			// Reformat into a system-compatible value
			itemp8 = (int8_t)((temp3 + 64) >> 7);							// Round then divide by 128
			P1_solution = scale_percent_nooffset(itemp8);	

		} // No curve, so just use one point for offset
		else
		{
			P1_solution = scale_percent_nooffset(Config.Channel[i].P1_offset);
		}

		// Add offset to channel value
		Config.Channel[i].P1_value += P1_solution;
	}

} // ProcessMixer()

//************************************************************
// Misc mixer code
//************************************************************

// Update actual limits value with that from the mix setting percentages
// This is only done at start-up and whenever the values are changed
// so as to reduce CPU loop load
void UpdateLimits(void)
{
	uint8_t i,j;
	int32_t temp32, gain32;

	int8_t limits[FLIGHT_MODES][NUMBEROFAXIS] = 
		{
			{Config.FlightMode[P1].Roll_limit, Config.FlightMode[P1].Pitch_limit, Config.FlightMode[P1].Yaw_limit},
			{Config.FlightMode[P2].Roll_limit, Config.FlightMode[P2].Pitch_limit, Config.FlightMode[P2].Yaw_limit}
		};

	int8_t gains[FLIGHT_MODES][NUMBEROFAXIS] = 
		{
			{Config.FlightMode[P1].Roll_I_mult, Config.FlightMode[P1].Pitch_I_mult, Config.FlightMode[P1].Yaw_I_mult},
			{Config.FlightMode[P2].Roll_I_mult, Config.FlightMode[P2].Pitch_I_mult, Config.FlightMode[P2].Yaw_I_mult}
		};

	// Update LVA trigger
	// Vbat is measured in units of 10mV, so PowerTriggerActual of 1270 equates to 12.7V
	switch (Config.PowerTrigger)
	{
		case 0:
			Config.PowerTriggerActual = 0;			// Off
			break;
		case 1:
			Config.PowerTriggerActual = 320; 		// 3.2V
			break;
		case 2:
			Config.PowerTriggerActual = 330; 		// 3.3V
			break;
		case 3:
			Config.PowerTriggerActual = 340;		// 3.4V
			break;
		case 4:
			Config.PowerTriggerActual = 350; 		// 3.5V
			break;
		case 5:
			Config.PowerTriggerActual = 360; 		// 3.6V
			break;
		case 6:
			Config.PowerTriggerActual = 370; 		// 3.7V
			break;
		case 7:
			Config.PowerTriggerActual = 380; 		// 3.8V
			break;
		case 8:
			Config.PowerTriggerActual = 390; 		// 3.9V
			break;
		default:
			Config.PowerTriggerActual = 0;			// Off
			break;
	}
			
	// Determine cell count and use to multiply trigger
	if (SystemVoltage >= 2150)										// 6S - 21.5V or at least 3.58V per cell
	{
		Config.PowerTriggerActual *= 6;
	}
	else if ((SystemVoltage >= 1730) && (SystemVoltage < 2150))		// 5S 17.3V to 21.5V or 4.32V(4S) to 3.58V(6S) per cell
	{
		Config.PowerTriggerActual *= 5;
	}
	else if ((SystemVoltage >= 1300) && (SystemVoltage < 1730))		// 4S 13.0V to 17.3V or 4.33V(3S) to 3.46V(5S) per cell
	{
		Config.PowerTriggerActual *= 4;
	}
	else if ((SystemVoltage >= 900) && (SystemVoltage < 1300))		// 3S 9.0V to 13.0V or 4.5V(2S) to 3.25V(4S) per cell
	{
		Config.PowerTriggerActual *= 3;
	}
	else if (SystemVoltage < 900)									// 2S Under 9.0V or 3.0V(3S) per cell
	{
		Config.PowerTriggerActual *= 2;
	}

	// Update I_term input constraints for all profiles
	for (j = 0; j < FLIGHT_MODES; j++)
	{
		for (i = 0; i < NUMBEROFAXIS; i++)
		{
			temp32 	= limits[j][i]; 						// Promote limit %

			// I-term output (throw). Convert from % to actual count
			// A value of 80,000 results in +/- 1250 or full throw at the output stage
			// This is because the maximum signal value is +/-1250 after division by 64. 1250 * 64 = 80,000
			Config.Raw_I_Limits[j][i] = temp32 * (int32_t)640;	// 80,000 / 125% = 640

			// I-term source limits. These have to be different due to the I-term gain setting
			// I-term = (gyro * gain) / 32, so the gyro count for a particular gain and limit is
			// Gyro = (I-term * 32) / gain :) 

			if (gains[j][i] != 0)
			{
				gain32 = gains[j][i];						// Promote gain value
				Config.Raw_I_Constrain[j][i] = (Config.Raw_I_Limits[j][i] << 5) / gain32;
			}
			else 
			{
				Config.Raw_I_Constrain[j][i] = 0;
			}
		}
	}

	// Update travel limits
	for (i = 0; i < MIX_OUTPUTS; i++)
	{
		Config.Limits[i].minimum = scale_percent(Config.min_travel[i]);
		Config.Limits[i].maximum = scale_percent(Config.max_travel[i]);
	}

	// Adjust trim to match 0.01 degree resolution
	// A value of 127 multiplied by 10 = 1270 which in 1/100ths of a degree equates to 12.7 degrees
	for (i = P1; i <= P2; i++)
	{
		Config.Rolltrim[i] = Config.FlightMode[i].AccRollZeroTrim * 10;
		Config.Pitchtrim[i] = Config.FlightMode[i].AccPitchZeroTrim * 10;
	}
}

// Update servos from the mixer Config.Channel[i].P1_value data, add offsets and enforce travel limits
void UpdateServos(void)
{
	uint8_t i;
	int16_t temp1 = 0; // Output value

	for (i = 0; i < MIX_OUTPUTS; i++)
	{
		// Servo reverse and trim for the eight physical outputs
		temp1 = Config.Channel[i].P1_value;

		// Reverse this channel for the eight physical outputs
		if ((i <= MIX_OUTPUTS) && (Config.Servo_reverse[i] == ON))
		{	
			temp1 = -temp1;
		}

		// Add offset value to restore to system compatible value
		temp1 += 3750;

		// Enforce min, max travel limits
		if (temp1 > Config.Limits[i].maximum)
		{
			ServoOut[i] = Config.Limits[i].maximum;
		}

		else if (temp1 < Config.Limits[i].minimum)
		{
			ServoOut[i] = Config.Limits[i].minimum;
		}

		// Transfer value to servo
		else
		{
			ServoOut[i] = temp1;
		}
	}
}

// 32 bit multiply/scale for broken GCC
// Returns immediately if multiplier is 100, 0 or -100
int16_t scale32(int16_t value16, int16_t multiplier16)
{
	int32_t temp32 = 0;
	int32_t mult32 = 0;

	// No change if 100% (no scaling)
	if (multiplier16 == 100)
	{
		return value16;
	}

	// Reverse if -100%
	else if (multiplier16 == -100)
	{
		return -value16;	
	}

	// Zero if 0%
	else if (multiplier16 == 0)
	{
		return 0;	
	}

	// Only do the scaling if necessary
	else
	{
		// GCC is broken bad regarding multiplying 32 bit numbers, hence all this crap...
		mult32 = multiplier16;
		temp32 = value16;
		temp32 = temp32 * mult32;

		// Divide by 100 and round to get scaled value
		temp32 = (temp32 + (int32_t)50) / (int32_t)100; // Constants need to be cast up to 32 bits
		value16 = (int16_t)temp32;
	}

	return value16;
}

// Scale percentages to position
int16_t scale_percent(int8_t value)
{
	int16_t temp16_1, temp16_2;

	temp16_1 = value; // Promote
	temp16_2 = ((temp16_1 * (int16_t)10) + 3750);

	return temp16_2;
}


// Scale percentages to relative position
int16_t scale_percent_nooffset(int8_t value)
{
	int16_t temp16_1, temp16_2;

	temp16_1 = value; // Promote
	temp16_2 = (temp16_1 * (int16_t)10);

	return temp16_2;
}
