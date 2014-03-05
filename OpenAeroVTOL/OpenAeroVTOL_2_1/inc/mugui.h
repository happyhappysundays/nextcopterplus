/*********************************************************************
 * mugui.h
 ********************************************************************/

#include <avr/pgmspace.h>

//***********************************************************
//* Externals
//***********************************************************

extern void mugui_lcd_puts(char *s, const unsigned char* font,uint16_t x, uint16_t y);
extern uint16_t mugui_lcd_putc(char c, const unsigned char* font,uint16_t x, uint16_t y);
extern void mugui_text_sizestring(char *s, const unsigned char* font, mugui_size16_t *size);
extern void pgm_mugui_lcd_puts(const unsigned char* s, const unsigned char* font,uint16_t x, uint16_t y);
extern void pgm_mugui_scopy(const char *s);

extern const unsigned char Verdana8[] PROGMEM;
extern const unsigned char Verdana14[] PROGMEM;
extern const unsigned char Wingdings[] PROGMEM;
