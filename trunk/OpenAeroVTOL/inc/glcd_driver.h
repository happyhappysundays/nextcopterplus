/*********************************************************************
 * glcd_driver.h
 ********************************************************************/

//***********************************************************
//* LCD Defines
//***********************************************************

// ST7565 parameters
#define CMD_DISPLAY_OFF   0xAE
#define CMD_DISPLAY_ON    0xAF
#define CMD_SET_DISP_START_LINE  0x40
#define CMD_SET_PAGE  0xB0
#define CMD_SET_COLUMN_UPPER  0x10
#define CMD_SET_COLUMN_LOWER  0x00
#define CMD_SET_ADC_NORMAL  0xA0
#define CMD_SET_ADC_REVERSE 0xA1
#define CMD_SET_DISP_NORMAL 0xA6
#define CMD_SET_DISP_REVERSE 0xA7
#define CMD_SET_ALLPTS_NORMAL 0xA4
#define CMD_SET_ALLPTS_ON  0xA5
#define CMD_SET_BIAS_9 0xA2 
#define CMD_SET_BIAS_7 0xA3
#define CMD_RMW  0xE0
#define CMD_RMW_CLEAR 0xEE
#define CMD_INTERNAL_RESET  0xE2
#define CMD_SET_COM_NORMAL  0xC0
#define CMD_SET_COM_REVERSE  0xC8
#define CMD_SET_POWER_CONTROL  0x28
#define CMD_SET_RESISTOR_RATIO  0x24
#define CMD_SET_VOLUME_FIRST  0x81
#define CMD_SET_VOLUME_SECOND  0
#define CMD_SET_STATIC_OFF  0xAC
#define CMD_SET_STATIC_ON  0xAD
#define CMD_SET_STATIC_REG  0x0
#define CMD_SET_BOOSTER_FIRST  0xF8
#define CMD_SET_BOOSTER_234  0
#define CMD_SET_BOOSTER_5  1
#define CMD_SET_BOOSTER_6  3
#define CMD_NOP  0xE3
#define CMD_TEST  0xF0

// ST7565 project parameters
#define LCDWIDTH 128
#define LCDHEIGHT 64

// Serial data - PD1 for KK 2.0
#define SID_DDR DDRD
#define SID_PIN PIND
#define SID_PORT PORTD
#define SID 1

// Serial clock - PD4 for KK 2.0
#define SCLK_DDR DDRD
#define SCLK_PIN PIND
#define SCLK_PORT PORTD
#define SCLK 4

// A0 - PD7 for KK 2.0
#define A0_DDR DDRD
#define A0_PIN PIND
#define A0_PORT PORTD
#define A0 7

// Reset - PD6 for KK 2.0
#define RST_DDR DDRD
#define RST_PIN PIND
#define RST_PORT PORTD
#define RST 6

// Chip select - PD4 for KK 2.0
#define CS_DDR DDRD
#define CS_PIN PIND
#define CS_PORT PORTD
#define CS 5

// Misc
#define swap(a, b) { uint8_t t = a; a = b; b = t; }

//***********************************************************
//* Prototypes
//***********************************************************

void spiwrite(uint8_t c);
void st7565_command(uint8_t c) ;
void st7565_data(uint8_t c);
void st7565_init(void);
void st7565_set_brightness(uint8_t val);
void write_buffer(uint8_t *buffer);
void clear_buffer(uint8_t *buff);
void write_logo_buffer(uint8_t *buffer);

void setpixel(uint8_t *buff, uint8_t x, uint8_t y, uint8_t color);
void drawrect(uint8_t *buff, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void drawline(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
void fillcircle(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t r, uint8_t color) ;
void fillrect(uint8_t *buff, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);




