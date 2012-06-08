/*********************************************************************
 * lcd.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void LCDprint(uint8_t i);
extern void LCDgoTo(uint8_t position);
extern void LCDprint_line1(const char *s);
extern void LCDprint_line2(const char *s);
extern void LCDclear(void);
extern void LCDclearLine(uint8_t line);
extern void LCDprintstr(const char *s);
extern void set_menu_item(uint8_t menuitem, int16_t value);
extern int16_t get_menu_item(uint8_t menuitem);
extern void LCD_Display_Menu (uint8_t menuitem);
extern menu_range_t get_menu_range (uint8_t menuitem);
extern void LCD_fixBL(void);
