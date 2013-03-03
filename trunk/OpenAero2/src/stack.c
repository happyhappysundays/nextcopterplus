//***********************************************************
//* stack.c
//* Copied from public code on AVRfreaks via Michael McTernan
//* http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=52249
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include <stdlib.h>

extern uint8_t __bss_end;
extern uint8_t __stack; 

//***********************************************************
//* Defines
//***********************************************************

#define STACK_CANARY	0xc5

//************************************************************
// Prototypes
//************************************************************

void StackPaint(void) __attribute__ ((naked)) __attribute__ ((section (".init3")));
uint16_t StackCount(void);	

//************************************************************
// Code
//************************************************************

uint16_t StackCount(void)
{
    const uint8_t *p = &__bss_end;
    uint16_t       c = 0;

    while(*p == STACK_CANARY && p <= &__stack)
    {
        p++;
        c++;
    }

    return c;
} 

void StackPaint(void)
{
#if 0
    uint8_t *p = &_end;

    while(p <= &__stack)
    {
        *p = STACK_CANARY;
        p++;
    }
#else
    __asm volatile ("    ldi r30,lo8(__bss_end)\n"
                    "    ldi r31,hi8(__bss_end)\n"
                    "    ldi r24,lo8(0xc5)\n" 
                    "    ldi r25,hi8(__stack)\n"
                    "    rjmp .cmp\n"
                    ".loop:\n"
                    "    st Z+,r24\n"
                    ".cmp:\n"
                    "    cpi r30,lo8(__stack)\n"
                    "    cpc r31,r25\n"
                    "    brlo .loop\n"
                    "    breq .loop"::);
#endif
} 
