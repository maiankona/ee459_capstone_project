#ifndef LCD_H
#define LCD_H

#include <avr/io.h>

// LCD I2C address
#define LCD_ADDR 0x78  // 8-bit I2C address for the LCD

// Control bytes for LCD commands and data
#define LCD_CMD 0x80  // Command register
#define LCD_DATA 0x40 // Data register

void lcd_init(void);
void lcd_write_string(uint8_t *str, uint8_t len);

#endif
