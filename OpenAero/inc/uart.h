/*********************************************************************
 * uart.h
 ********************************************************************/

//***********************************************************
//* Externals
//***********************************************************

extern void init_uart(void);
extern void send_byte(uint8_t i);
extern void send_word(uint16_t i);
extern uint8_t rx_byte(void);
extern uint16_t rx_word(void);
extern void send_multwii_data(void);
extern void get_multwii_data(void);
extern void autotune(void);
extern void variable_delay(uint8_t count);



