#ifndef LCD_H
#define LCD_H

#include <avr/io.h>
#include <string.h>

void lcd_init();
void lcd_writebyte(uint8_t control, uint8_t *data, size_t len);
void lcd_writedata(uint8_t x);
void lcd_writecommand(uint8_t x);
void lcd_moveto(unsigned char, unsigned char);
void lcd_stringout(char *str);
void lcd_screen(char data[4][21]);

#endif