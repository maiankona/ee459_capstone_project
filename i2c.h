#ifndef I2C_H
#define I2C_H

#include <avr/io.h>

#define FOSC 7372800                      // Clock frequency = Oscillator freq.
#define BDIV (FOSC / 100000 - 16) / 2 + 1 // Puts I2C rate just below 100kHz

void i2c_init(uint8_t bdiv);
uint8_t i2c_io(uint8_t device_addr, uint8_t *wp, uint16_t wn, uint8_t *rp, uint16_t rn);

#endif