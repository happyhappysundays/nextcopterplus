//***********************************************************
//* display_rcinput.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdlib.h>
#include <stdbool.h>
#include "io_cfg.h"
#include "glcd_driver.h"
#include "mugui.h"
#include <avr/pgmspace.h>
#include "glcd_menu.h"
#include "main.h"
#include "isr.h"
#include <util/delay.h>
#include "acc.h"
#include "gyros.h"
#include "rc.h"
#include "menu_ext.h"
#include "mixer.h"
#include "eeprom.h"
#include "servos.h"
#include "imu.h"
#include "pid.h"

//************************************************************
// Prototypes
//************************************************************

void Display_in_out(void);

//************************************************************
// Code
//************************************************************

void Display_in_out(void)
{
	uint8_t i = 0;
	int16_t temp = 0;
	uint16_t utemp = 0;
	int8_t	pos1, pos2, pos3;
	mugui_size16_t size;
	int16_t	Outputs[MAX_OUTPUTS];
	int16_t	Inputs[MAX_RC_CHANNELS];
	uint16_t Save_TCNT1 = 0;
	uint16_t ticker_16 = 0;
	uint32_t interval = 0;			// IMU interval	
	float	tempf1 = 0.0;
	
	// Re-enable interrupts. High speed mode may have left them off
	init_int();
	
	// While back button not pressed
	while(BUTTON1 != 0)
	{
		RxGetChannels();
		ReadGyros();
		ReadAcc();

		//************************************************************
		//* Update IMU
		// TMR1 is a 16-bit counter that counts at 2.5MHz. Max interval is 26.2ms
		// TMR0 is an 8-bit counter that counts at 19.531kHz. Max interval is 13.1ms
		// These two are concatenated to create a virtual timer that can measure up to 
		// 256 x 26.2ms = 6.7072s at which point the "period" is 16,768,000, a 24-bit number
		//************************************************************
		
		// Safely get current value of TCNT1
		Save_TCNT1 = TIM16_ReadTCNT1();

		// Reset Timer0 count
		TCNT0 = 0;

		// Handle TCNT1 overflow correctly - this actually seems necessary...
		// ticker_16 will hold the most recent amount measured by TCNT1
		// Timer1 (16bit) - run @ 2.5MHz (400ns) - max 26.2ms
		if (Save_TCNT1 < LoopStartTCNT1)
		{
			ticker_16 = (65536 - LoopStartTCNT1) + Save_TCNT1;
		}
		else
		{
			ticker_16 = (Save_TCNT1 - LoopStartTCNT1);
		}
		
		interval = ticker_16; // uint16_t
		
		// Store old TCNT for next measurement
		LoopStartTCNT1 = Save_TCNT1;
		
		// Handle both Timer1 under- and over-run cases
		// If TMR0_counter is less than 2, ICNT1 has not overflowed
		if (TMR0_counter < 2)
		{
			interval = ticker_16; // uint16_t
		}
		
		// If TMR0_counter is 2 or more, then TCNT1 has overflowed
		// So we use chunks of TCNT0, counted during the loop interval
		// to work out the exact period.
		// Timer0 (8bit) - run @ 20MHz / 1024 = 19.531kHz or 51.2us - max 13.1ms
		else
		{
			interval = ticker_16 + (TMR0_counter * 32768);
		}

		TMR0_counter = 0;

		// Refresh accSmooth values and AccVert
		imu_update(interval);
		Sensor_PID(interval);
		Calculate_PID();		// Calculate PID values

		UpdateTransition();		// Update the transition variable
		ProcessMixer();			// Do all the mixer tasks
		UpdateServos();			// Transfer Config.Channel[i].value data to ServoOut[i]

		// Re-span numbers from internal values (2500 to 5000) to microseconds
		for (i = 0; i < MAX_OUTPUTS; i++)
		{
			temp = ServoOut[i];					// Promote to 16 bits

			// Check for motor marker and ignore if set
			if (Config.Channel[i].Motor_marker != MOTOR)
			{
				// Scale servo from 2500~5000 to 875~2125
				temp = ((temp - 3750) >> 1) + SERVO_CENTER; // SERVO_CENTER = 1500
			}
			else
			{
				// Scale motor from 2500~5000 to 1000~2000
				temp = ((temp << 2) + 5) / 10; 	// Round and convert
			}
		
			ServoOut[i] = (uint16_t)temp;
		}

		// Check limits in microsecond units.
		for (i = 0; i < MAX_OUTPUTS; i++)
		{
			temp = ServoOut[i];
			
			// Enforce min, max travel limits
			if (temp > Config.Limits[i].maximum)
			{
				temp = Config.Limits[i].maximum;
			}

			else if (temp < Config.Limits[i].minimum)
			{
				temp = Config.Limits[i].minimum;
			}
			
			ServoOut[i] = temp;
		}
		
		// Servos are now in microsecond units.
		
		// Check for motor flags if throttle is below arming minimum
		if (MonopolarThrottle < THROTTLEIDLE) // THROTTLEIDLE = 50
		{
			// For each output
			for (i = 0; i < MAX_OUTPUTS; i++)
			{
				// Check for motor marker
				if (Config.Channel[i].Motor_marker == MOTOR)
				{
					// Set output to minimum pulse width (1000us)
					ServoOut[i] = MOTORMIN;
				}
			}
		}

		// Convert outputs to percentages (center and divide by 4)
		for (i = 0; i < MAX_OUTPUTS; i++)
		{
			Outputs[i] = (int16_t)ServoOut[i];
			Outputs[i] = Outputs[i] - SERVO_CENTER;
			
			// Round correctly in both directions
			if (Outputs[i] >= 0)
			{
				Outputs[i] = (Outputs[i] + 2) / 5; // Convert to percentages +/-500 = +/-100%
			}
			else
			{
				Outputs[i] = (Outputs[i] - 2) / 5; 
			}
		}

		// Convert inputs to percentages (center and divide by 10)
		for (i = 0; i < MAX_RC_CHANNELS; i++)
		{
			tempf1 = (float)RCinputs[i];
			Inputs[i] = (int16_t)(tempf1 / 10.0f);
		}

		// Column 1
		LCD_Display_Text(478,(const unsigned char*)Verdana8,0,3);
		LCD_Display_Text(479,(const unsigned char*)Verdana8,0,13);
		LCD_Display_Text(480,(const unsigned char*)Verdana8,0,23);
		LCD_Display_Text(481,(const unsigned char*)Verdana8,0,33);
		LCD_Display_Text(482,(const unsigned char*)Verdana8,0,43);
		LCD_Display_Text(483,(const unsigned char*)Verdana8,0,53);
		
		mugui_lcd_puts(itoa((MonopolarThrottle / 20),pBuffer,10),(const unsigned char*)Verdana8,25,3);
		mugui_lcd_puts(itoa(Inputs[AILERON],pBuffer,10),(const unsigned char*)Verdana8,25,13);
		mugui_lcd_puts(itoa(Inputs[ELEVATOR],pBuffer,10),(const unsigned char*)Verdana8,25,23);
		mugui_lcd_puts(itoa(Inputs[RUDDER],pBuffer,10),(const unsigned char*)Verdana8,25,33);
		mugui_lcd_puts(itoa(Inputs[GEAR],pBuffer,10),(const unsigned char*)Verdana8,25,43);
		mugui_lcd_puts(itoa(Inputs[AUX1],pBuffer,10),(const unsigned char*)Verdana8,25,53);
		
		// Column 2
		LCD_Display_Text(484,(const unsigned char*)Verdana8,48,3);
		LCD_Display_Text(485,(const unsigned char*)Verdana8,48,13);
		LCD_Display_Text(477,(const unsigned char*)Verdana8,48,23); // OUT
		LCD_Display_Text(468,(const unsigned char*)Verdana8,48,33);
		LCD_Display_Text(469,(const unsigned char*)Verdana8,48,43);
		LCD_Display_Text(470,(const unsigned char*)Verdana8,48,53);		
		mugui_lcd_puts(itoa(Inputs[AUX2],pBuffer,10),(const unsigned char*)Verdana8,71,3);
		mugui_lcd_puts(itoa(Inputs[AUX3],pBuffer,10),(const unsigned char*)Verdana8,71,13);
		mugui_lcd_puts(itoa(Outputs[0],pBuffer,10),(const unsigned char*)Verdana8,57,33);
		mugui_lcd_puts(itoa(Outputs[1],pBuffer,10),(const unsigned char*)Verdana8,57,43);
		mugui_lcd_puts(itoa(Outputs[2],pBuffer,10),(const unsigned char*)Verdana8,57,53);
		
		// Column 3
		LCD_Display_Text(471,(const unsigned char*)Verdana8,94,3);
		LCD_Display_Text(472,(const unsigned char*)Verdana8,94,13);
		LCD_Display_Text(473,(const unsigned char*)Verdana8,94,23);
		LCD_Display_Text(474,(const unsigned char*)Verdana8,94,33);
		LCD_Display_Text(475,(const unsigned char*)Verdana8,94,43);
		LCD_Display_Text(476,(const unsigned char*)Verdana8,88,53); // Pn.


		mugui_lcd_puts(itoa(Outputs[3],pBuffer,10),(const unsigned char*)Verdana8,104,3);
		mugui_lcd_puts(itoa(Outputs[4],pBuffer,10),(const unsigned char*)Verdana8,104,13);
		mugui_lcd_puts(itoa(Outputs[5],pBuffer,10),(const unsigned char*)Verdana8,104,23);
		mugui_lcd_puts(itoa(Outputs[6],pBuffer,10),(const unsigned char*)Verdana8,104,33);
		mugui_lcd_puts(itoa(Outputs[7],pBuffer,10),(const unsigned char*)Verdana8,104,43);

		// Display the transition number as 1.00 to 2.00
		uint8_t x_loc = 104;		// X location of transition display
		uint8_t y_loc = 53;		// Y location of transition display

		utemp = transition + 100;
		temp = utemp/100;		// Display whole decimal part first
		mugui_text_sizestring(itoa(temp,pBuffer,10), (const unsigned char*)Verdana8, &size);
		mugui_lcd_puts(itoa(temp,pBuffer,10),(const unsigned char*)Verdana8,x_loc,y_loc);
		pos1 = size.x;

		utemp = utemp - (temp * 100); // Now display the parts to the right of the decimal point

		LCD_Display_Text(268,(const unsigned char*)Verdana8,(x_loc + pos1),y_loc);
		mugui_text_sizestring(".", (const unsigned char*)Verdana8, &size);
		pos3 = size.x;
		mugui_text_sizestring("0", (const unsigned char*)Verdana8, &size);
		pos2 = size.x;

		if (utemp >= 10)
		{
			mugui_lcd_puts(itoa(utemp,pBuffer,10),(const unsigned char*)Verdana8,(x_loc + pos1 + pos3),y_loc);
		}
		else
		{
			LCD_Display_Text(269,(const unsigned char*)Verdana8,(x_loc + pos1 + pos3),y_loc);
			mugui_lcd_puts(itoa(utemp,pBuffer,10),(const unsigned char*)Verdana8,(x_loc + pos1 + pos2 + pos3),y_loc);
		}

		// mugui_lcd_puts(itoa(transition,pBuffer,10),(const unsigned char*)Verdana8,104,53);		

		// Update buffer
		write_buffer(buffer);
		clear_buffer(buffer);
	}
}
