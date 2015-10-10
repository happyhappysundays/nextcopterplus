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
int16_t scale_percent_nooffset_mono(int8_t value);
int16_t scale_throttle_curve_percent_bipolar(int8_t value);
int16_t scale_throttle_curve_percent_mono(int8_t value);
int16_t scale_micros(int8_t value);
int16_t Process_curve(uint8_t curve, uint8_t type, int16_t input_value);

//************************************************************
// Globals
//************************************************************

int16_t	P1_curve_C = 0;		// Generic curve C
int16_t	P2_curve_C = 0;		// Generic curve C
int16_t	P1_curve_D = 0;		// Generic curve D
int16_t	P2_curve_D = 0;		// Generic curve D

//************************************************************
// Defines
//************************************************************

#define MIX_OUTPUTS 8

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

//************************************************************
// Code
//************************************************************

void ProcessMixer(void)
{
	uint8_t i = 0;
	int16_t P1_solution = 0;
	int16_t P2_solution = 0;

	int16_t temp1 = 0;
	int16_t temp2 = 0;
	int16_t	temp3 = 0;
	int16_t	Step1 = 0;
	int16_t monothrottle = 0;
	
	int32_t e32temp1 = 0;
	int32_t e32temp2 = 0;
	int32_t e32temp3 = 0;
	int32_t	e32Step1 = 0;
	
	// Curve outputs
	int16_t	P1_throttle = 0;	// P1 Throttle curve
	int16_t	P2_throttle = 0;	// P2 Throttle curve
	int16_t	P1_collective = 0;	// P1 Collective curve
	int16_t	P2_collective = 0;	// P2 Collective curve
	
	int8_t	P1_acc_roll_volume_source = 0;
	int8_t	P1_gyro_roll_volume_source = 0;
	int8_t	P1_gyro_yaw_volume_source = 0;

	// Process curves
	P1_throttle = Process_curve(P1_THR_CURVE, MONOPOLAR, MonopolarThrottle);
	P2_throttle = Process_curve(P2_THR_CURVE, MONOPOLAR, MonopolarThrottle);
	P1_collective = Process_curve(P1_COLL_CURVE, BIPOLAR, RCinputs[THROTTLE]);
	P2_collective = Process_curve(P2_COLL_CURVE, BIPOLAR, RCinputs[THROTTLE]);

	// Copy the universal mixer inputs to an array for easy indexing - acc data is from accSmooth, increased to reasonable rates
	temp1 = (int16_t)accSmooth[ROLL] << 3;
	temp2 = (int16_t)accSmooth[PITCH] << 3;
		
	// THROTTLE, CURVE A, CURVE B, COLLECTIVE, THROTTLE, AILERON, ELEVATOR, RUDDER, GEAR, AUX1, AUX2, AUX3, ROLLGYRO, PITCHGYO, YAWGYRO, ACCSMOOTH, PITCHSMOOTH, ROLLACC, PITCHACC, AccZ, NONE
	int16_t	UniversalP1[NUMBEROFSOURCES] = 
		{P1_throttle, P1_curve_C, P1_curve_D, P1_collective, RCinputs[THROTTLE], RCinputs[AILERON], RCinputs[ELEVATOR], RCinputs[RUDDER], RCinputs[GEAR], RCinputs[AUX1], RCinputs[AUX2], RCinputs[AUX3],
		 PID_Gyros[P1][ROLL], PID_Gyros[P1][PITCH], PID_Gyros[P1][YAW], temp1, temp2, PID_ACCs[P1][ROLL], PID_ACCs[P1][PITCH],PID_ACCs[P1][YAW], 0};
		
	int16_t	UniversalP2[NUMBEROFSOURCES] = 
		{P2_throttle, P2_curve_C, P2_curve_D, P2_collective, RCinputs[THROTTLE], RCinputs[AILERON], RCinputs[ELEVATOR], RCinputs[RUDDER], RCinputs[GEAR], RCinputs[AUX1], RCinputs[AUX2], RCinputs[AUX3],
		 PID_Gyros[P2][ROLL], PID_Gyros[P2][PITCH], PID_Gyros[P2][YAW], temp1, temp2, PID_ACCs[P2][ROLL], PID_ACCs[P2][PITCH],PID_ACCs[P2][YAW], 0}; 

	//************************************************************
	// Generic curves
	//************************************************************
	
	// Only process generic curves if they have a source selected
	if (Config.Curve[GEN_CURVE_C].channel != NOMIX)
	{
		P1_curve_C = Process_curve(GEN_CURVE_C, BIPOLAR, UniversalP1[Config.Curve[GEN_CURVE_C].channel]);		
		P2_curve_C = Process_curve(GEN_CURVE_C, BIPOLAR, UniversalP2[Config.Curve[GEN_CURVE_C].channel]);	
	}
	else
	{
		P2_curve_C = 0;
	}
	
	if (Config.Curve[GEN_CURVE_D].channel != NOMIX)
	{
		P1_curve_D = Process_curve(GEN_CURVE_D, BIPOLAR, UniversalP1[Config.Curve[GEN_CURVE_D].channel]);
		P2_curve_D = Process_curve(GEN_CURVE_D, BIPOLAR, UniversalP2[Config.Curve[GEN_CURVE_D].channel]);
	}
	else
	{
		P2_curve_D = 0;
	}
	
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

		// If the user wants earth reference for tail-sitter hover, swap the related stick sources.
		// The secret is understanding WHICH STICK is controlling movement on the AXIS in the selected REFERENCE
		// Only need to do this if the orientations differ
		if (Config.P1_Reference != NO_ORIENT)
		{
			// EARTH-Referenced tail-sitter
			if (Config.P1_Reference == EARTH)
			{
				P1_acc_roll_volume_source = Config.Channel[i].P1_aileron_volume;
				P1_gyro_roll_volume_source = Config.Channel[i].P1_aileron_volume;	// These are always the same
				P1_gyro_yaw_volume_source = Config.Channel[i].P1_rudder_volume;		// These are always the same
			}
			// MODEL-Referenced tail-sitter
			else
			{
				P1_acc_roll_volume_source = Config.Channel[i].P1_rudder_volume;
				P1_gyro_roll_volume_source =  Config.Channel[i].P1_aileron_volume;
				P1_gyro_yaw_volume_source =  Config.Channel[i].P1_rudder_volume;			
			}
		}
		// Normal case
		else
		{
			P1_acc_roll_volume_source =  Config.Channel[i].P1_aileron_volume;
			P1_gyro_roll_volume_source =  Config.Channel[i].P1_aileron_volume;
			P1_gyro_yaw_volume_source =  Config.Channel[i].P1_rudder_volume;
		}
		
		// P1 gyros
		if (transition < 100)
		{
			switch (Config.Channel[i].P1_Roll_gyro) 
			{
				case OFF:
					break;
				case ON:
					if (P1_gyro_roll_volume_source < 0 )
					{
						P1_solution = P1_solution + PID_Gyros[P1][ROLL];		// Reverse if volume negative
					}
					else
					{
						P1_solution = P1_solution - PID_Gyros[P1][ROLL];
					}
					break;
				case SCALE:
					P1_solution = P1_solution - scale32(PID_Gyros[P1][ROLL], P1_gyro_roll_volume_source * 5); 
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
					if (P1_gyro_yaw_volume_source < 0 )
					{
						P1_solution = P1_solution - PID_Gyros[P1][YAW];			// Reverse if volume negative
					}
					else
					{
						P1_solution = P1_solution + PID_Gyros[P1][YAW];
					}
					break;
				case SCALE:
					P1_solution = P1_solution + scale32(PID_Gyros[P1][YAW], P1_gyro_yaw_volume_source * 5);
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
					if (P1_acc_roll_volume_source < 0 )
					{
						P1_solution = P1_solution + PID_ACCs[P1][ROLL];			// Reverse if volume negative
					}
					else
					{
						P1_solution = P1_solution - PID_ACCs[P1][ROLL];			// or simply add
					}
					break;
				case SCALE:
					P1_solution = P1_solution - scale32(PID_ACCs[P1][ROLL], P1_acc_roll_volume_source * 5);
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
				temp2 = UniversalP1[Config.Channel[i].P1_source_a];
				temp2 = scale32(temp2, Config.Channel[i].P1_source_a_volume);
				P1_solution = P1_solution + temp2;
			}
			if ((Config.Channel[i].P1_source_b_volume != 0) && (Config.Channel[i].P1_source_b != NOMIX)) // Mix in second extra source
			{
				temp2 = UniversalP1[Config.Channel[i].P1_source_b];
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
				temp2 = UniversalP2[Config.Channel[i].P2_source_a];
				temp2 = scale32(temp2, Config.Channel[i].P2_source_a_volume);
				P2_solution = P2_solution + temp2;
			}
			if ((Config.Channel[i].P2_source_b_volume != 0) && (Config.Channel[i].P2_source_b != NOMIX)) // Mix in second extra source
			{
				temp2 = UniversalP2[Config.Channel[i].P2_source_b];
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
	if (Config.TransitionSpeedOut != 0) 
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
	// P1/P2 throttle curve handling
	// Work out the resultant Monopolar throttle value based on
	// P1_throttle, P2_throttle and the transition number
	//************************************************************ 

	// Only process if there is a difference
	if (P1_throttle != P2_throttle)
	{
		// Speed up the easy ones :)
		if (transition == 0)
		{
			e32temp3 = P1_throttle;
		}
		else if (transition >= 100)
		{
			e32temp3 = P2_throttle;
		}
		else
		{
			// Calculate step difference in 1/100ths and round
			e32temp1 = (P2_throttle - P1_throttle);
			e32temp1 = e32temp1 << 16; 						// Multiply by 65536 so divide gives reasonable step values
			e32Step1 = e32temp1 / (int32_t)100;

			// Set start (P1) point
			e32temp2 = P1_throttle;							// Promote to 32 bits
			e32temp2 = e32temp2 << 16;

			// Multiply [transition] steps (0 to 100)
			e32temp3 = e32temp2 + (e32Step1 * transition);

			// Round, then rescale to normal value
			e32temp3 = e32temp3 + (int32_t)32768;
			e32temp3 = e32temp3 >> 16;			
		}
	}
			
	// No curve
	else
	{
		// Just use the value of P1_throttle as there is no curve
		e32temp3 = P1_throttle; // Promote to 16 bits
	}

	// Copy to monopolar throttle
	monothrottle = (int16_t)e32temp3;

	//************************************************************
	// Groovy transition curve handling. Must be after the transition.
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
			temp3 = scale32(monothrottle, temp3);

			// At this point, the throttle values are 0 to 2500 (+/-150%)
			// Re-scale throttle values back to neutral-centered system values (+/-1250) 
			// and set the minimum throttle point to 1.1ms.
			// A THROTTLEMIN value of -1000 will result in 2750, or 1.1ms
			temp3 = temp3 - THROTTLEMIN;

			// Add offset to channel value
			Config.Channel[i].P1_value += temp3;

		} // No throttle
		
		// No throttles, so clamp to THROTTLEMIN if flagged as a motor
		else if (Config.Channel[i].Motor_marker == MOTOR)
		{
			Config.Channel[i].P1_value = -THROTTLEOFFSET; // 3750-1250 = 2500 = 1.0ms. THROTTLEOFFSET = 1250
		}
	}

	//************************************************************
	// Per-channel 7-point offset needs to be after the transition  
	// loop as it is non-linear, unlike the transition.
	//************************************************************ 

	for (i = 0; i < MIX_OUTPUTS; i++)
	{
		// The input to the curves will be the transition number, altered to appear as -1000 to 1000.
		temp1 = (transition - 50) * 20; // 0 - 100 -> -1000 to 1000

		// Process as 7-point offset curve. All are BIPOLAR types.
		// Temporarily add NUMBEROFCURVES to the curve number to identify 
		// them to Process_curve() as being offsets, not the other curves.
		temp2 = Process_curve(i + NUMBEROFCURVES, BIPOLAR, temp1);
		
		// Add offset to channel value
		Config.Channel[i].P1_value += temp2;
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
		Config.Limits[i].minimum = scale_micros(Config.min_travel[i]);
		Config.Limits[i].maximum = scale_micros(Config.max_travel[i]);
	}

	// Adjust trim to match 0.01 degree resolution
	// A value of 127 multiplied by 100 = 12700 which in 1/100ths of a degree equates to potentially 127 degrees
	// In reality though, with a more realistic P gain of 10, this equates to potentially 12.7 degrees
	for (i = P1; i <= P2; i++)
	{
		Config.Rolltrim[i] = Config.FlightMode[i].AccRollZeroTrim * 100;
		Config.Pitchtrim[i] = Config.FlightMode[i].AccPitchZeroTrim * 100;
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

		// Transfer value to servo
		ServoOut[i] = temp1;
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

// Scale percentages to microsecond (position)
int16_t scale_micros(int8_t value)
{
	int16_t temp16;

	// 100% = 1000 to 2000
	temp16 = (int16_t)((value * 5) + SERVO_CENTER); // SERVO_CENTER = 1500

	return temp16;
}

// Scale percentages to internal units (position). Must not be further expanded.
// Input values are -125 to 125. Output values are 2187.5 to 5312.5 (875 to 2125us)
int16_t scale_percent(int8_t value)
{
	int16_t temp16;
	float tempf;

	// 100% = 2500 to 5000
	tempf = (float)value; // Promote
	temp16 = (int16_t)((tempf * 12.5f) + 3750.0f);

	return temp16;
}


// Scale percentages to relative position (internal units) with a non-expanded scale
// Input values are -125 to 125. Output values are -1250 to 1250 (1000 to 2000us)
int16_t scale_percent_nooffset(int8_t value)
{
	int16_t temp16;
	float tempf;

	tempf = (float)value; // Promote
	temp16 = (int16_t)(tempf * 10.0f);

	return temp16;
}

// Scale curve percentages to relative position (Bipolar internal units)
// Input values are -100 to 100. Output values are -1000 to 1000.
int16_t scale_throttle_curve_percent_bipolar(int8_t value)
{
	int16_t temp16;
	float tempf;

	tempf = (float)value; // Promote
	temp16 = (int16_t)(tempf * 10.0f);

	return temp16;
}

// Scale curve percentages to relative position (Monopolar internal units)
// Note that a curve percentage is a percentage of the input value.
// Input values are 0 to 100% of full throttle. Output values are 0 to 2000 (full throttle).
int16_t scale_throttle_curve_percent_mono(int8_t value)
{
	int16_t temp16;
	float tempf;

	tempf = (float)value; // Promote
	temp16 = (int16_t)(tempf * 20.0f);

	return temp16;
}

// Process curves. Maximum input values are +/-1000 for Bipolar curves and 0-2000 for monopolar curves.
// Curve number > NUMBEROFCURVES are the offset curves.
// Seven points 0, 17%, 33%, 50%, 67%, 83%, 100%	(Monopolar)
// Seven points -100, 67%, -33%, 0%, 33%, 67%, 100% (Bipolar)
int16_t Process_curve(uint8_t curve, uint8_t type, int16_t input_value)
{
	int16_t output_value = 0;
	int8_t zone = 0;
	int8_t start = 0;
	int8_t end = 0;
	int16_t start_pos = 0;
	int16_t bracket = 0;
	int16_t end_pos = 0;
	int32_t temp1 = 0;
	int32_t temp2 = 0;
	int32_t	Step1 = 0;

	if (type == BIPOLAR)
	{
		// Limit input value to +/-100% (+/-1000)
		if (input_value < -1000)
		{
			input_value = -1000;
		}
		if (input_value > 1000)
		{
			input_value = 1000;
		}
	}
	else // Monopolar
	{
		// Limit input value to 0 to 100% (0 to 2000)
		if (input_value < 0)
		{
			input_value = 0;
		}
		if (input_value > 2000)
		{
			input_value = 2000;
		}		
	}

	if (type == BIPOLAR)
	{
		// Work out which zone we are in
		if (input_value < -667)
		{
			zone = 0;
			bracket = -1000;			
		}
		else if (input_value < -333)
		{
			zone = 1;
			bracket = -667;
		}
		else if (input_value < 0)
		{
			zone = 2;
			bracket = -333;
		}	
		else if (input_value > 667)
		{
			zone = 5;
			bracket = 667;
		}		
		else if (input_value > 333)
		{
			zone = 4;
			bracket = 333;
		}
		else if (input_value >= 0)
		{
			zone = 3;
			bracket = 0;
		}
	}
	else // Monopolar
	{
		// Work out which zone we are in
		if (input_value < 333)
		{
			zone = 0;
			bracket = 0;
		}
		else if (input_value < 667)
		{
			zone = 1;
			bracket = 333;
		}
		else if (input_value < 1000)
		{
			zone = 2;
			bracket = 667;
		}
		else if (input_value > 1667)
		{
			zone = 5;
			bracket = 1667;
		}
		else if (input_value > 1333)
		{
			zone = 4;
			bracket = 1333;
		}
		else if (input_value >= 1000)
		{
			zone = 3;
			bracket = 1000;
		}	
	}

	// Find start/end points of zone 
	// Normal curves
	if (curve < NUMBEROFCURVES)
	{
		switch(zone)
		{
			case 0:
				start = Config.Curve[curve].Point1;
				end = Config.Curve[curve].Point2;
				break;
			case 1:	
				start = Config.Curve[curve].Point2;
				end = Config.Curve[curve].Point3;
				break;
			case 2:
				start = Config.Curve[curve].Point3;
				end = Config.Curve[curve].Point4;
				break;
			case 3:
				start = Config.Curve[curve].Point4;
				end = Config.Curve[curve].Point5;
				break;
			case 4:
				start = Config.Curve[curve].Point5;
				end = Config.Curve[curve].Point6;
				break;
			case 5:
				start = Config.Curve[curve].Point6;
				end = Config.Curve[curve].Point7;
				break;
			default:
				break;
		}
	}
	// Offsets
	else
	{
		// Correct curve number
		curve = curve - NUMBEROFCURVES;

		switch(zone)
		{
			case 0:
				start = Config.Offsets[curve].Point1;
				end = Config.Offsets[curve].Point2;
				break;
			case 1:
				start = Config.Offsets[curve].Point2;
				end = Config.Offsets[curve].Point3;
				break;
			case 2:
				start = Config.Offsets[curve].Point3;
				end = Config.Offsets[curve].Point4;
				break;
			case 3:
				start = Config.Offsets[curve].Point4;
				end = Config.Offsets[curve].Point5;
				break;
			case 4:
				start = Config.Offsets[curve].Point5;
				end = Config.Offsets[curve].Point6;
				break;
			case 5:
				start = Config.Offsets[curve].Point6;
				end = Config.Offsets[curve].Point7;
				break;
			default:
				break;
		}		
	}
	
	// Work out distance to cover
	// Convert percentages to positions
	if (type == BIPOLAR)
	{	
		start_pos = scale_throttle_curve_percent_bipolar(start);
		end_pos = scale_throttle_curve_percent_bipolar(end);
	}
	else
	{
		start_pos = scale_throttle_curve_percent_mono(start);
		end_pos = scale_throttle_curve_percent_mono(end);
	}

	// Upscale span for best resolution (x 65536)
	temp1 = (int32_t)(end_pos - start_pos);
	temp1 = temp1 << 16;

	// Divide distance into steps that cover the interval
	Step1 = (int32_t)(temp1 / (int32_t)334) ;

	temp2 = start_pos;
	temp2 = temp2 << 16;
	temp2 += ((input_value - bracket) * Step1);

	// Reformat into a system-compatible value
	// Divide by 65536
	output_value = (int16_t)(temp2 >> 16);

	return output_value;
}
