//***********************************************************
//* proximity.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#include "..\inc\io_cfg.h"
#include "..\inc\isr.h"

#ifdef PROX_MODULE
//************************************************************
// Prototypes
//************************************************************

void Ping(void);
uint16_t GetDistance(void);	

//************************************************************
// Code
//************************************************************

void Ping(void)
{
	PING = 1;		// Start 15us pulse
	_delay_us(15);
	PING = 0;
}

uint16_t GetDistance(void)
{	
	uint16_t distance;
	distance = EchoTime / 58;
	return(distance);
}
#endif
