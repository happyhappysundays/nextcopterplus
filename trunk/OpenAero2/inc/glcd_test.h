/*********************************************************************
 * glcd_test.h
 ********************************************************************/

//***********************************************************
//* Defines
//***********************************************************

extern void testdrawbitmap(uint8_t *buff, const char *bitmap, uint8_t w, uint8_t h);
extern void testdrawchar(uint8_t *buff);
extern void testdrawcircle(uint8_t *buff);
extern void testdrawline(uint8_t *buff);
extern void testdrawrect(uint8_t *buff);
extern void testfillrect(uint8_t *buff);

extern uint8_t buffer[1024]; // debug
