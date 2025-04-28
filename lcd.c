#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "i2c.h"
#include "debug.h"
#include <stdlib.h>

#define DEBUG

#define LCD_ADDR 0x3C // 8 bit addr (not shifted)
#define Co 7
#define A0 6

void lcd_writebyte(uint8_t control, uint8_t *data, size_t len)
{
  uint8_t *wbuf = malloc(sizeof(uint8_t) * (len + 1));

  // add control byte
  wbuf[0] = control;

  // add remaining bytes
  int i = 1;
  for (i; i < len + 1; i++)
  {
    wbuf[i] = data[i - 1];
  }

  uint8_t res = i2c_io((LCD_ADDR << 1), wbuf, len + 1, NULL, 0);
  free(wbuf);

#ifdef DEBUG
  char nbuf[100];
  snprintf(nbuf, 100, "[LCD] Control: %hhX; Data: %hhX; I2C Code: %hhX\n\r", control, data, res);
  debug_log(nbuf, 100);
#endif
}

void lcd_writedata(uint8_t x)
{
  uint8_t data[1] = {x};
  lcd_writebyte((1 << A0), data, 1);
};

void lcd_writecommand(uint8_t x)
{
  uint8_t data[1] = {x};
  lcd_writebyte(0x00, data, 1);
};

void lcd_moveto(unsigned char row, unsigned char col)
{
  unsigned char base[] = {0x00, 0x40, 0x14, 0x54};
  unsigned char pos = base[row - 1] + (col - 1);
  lcd_writecommand(0x80 | pos);
}

// /*
//   lcd_stringout - Print the contents of the character string "str"
//   at the current cursor position.
// */
void lcd_stringout(char *str)
{
  char ch;
  while ((ch = *str++) != '\0')
    lcd_writedata(ch);
}

void lcd_screen(char data[4][21])
{
  char rearranged_data[80];

  memcpy(rearranged_data, data, 20);
  memcpy(&rearranged_data[20], data[2], 20);
  memcpy(&rearranged_data[40], data[1], 20);
  memcpy(&rearranged_data[60], data[3], 20);

  //  move to begining
  lcd_moveto(1, 0);
  lcd_writebyte((1 << A0), rearranged_data, 80);
}

void lcd_init()
{
  i2c_init(BDIV); // init i2c for LCD

  _delay_ms(500); // delay for reset to finish

  lcd_writecommand(0x38); // function set 2 lines
  _delay_us(120);

  lcd_writecommand(0x0C); // display on, cursor on
  _delay_us(120);

  lcd_writecommand(0x01); // clear display
  _delay_ms(15);

  lcd_writecommand(0x06); // entry mode : cursor shifts right
  _delay_us(120);
};