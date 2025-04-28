#include <avr/io.h>
#include <util/delay.h>
#include "i2c.h"
#include "lcd.h"

int main(void) {
    uint8_t hello[] = {'H', 'e', 'l', 'l', 'o'};

    // Initialize I2C
    i2c_init(11);  // For 200kHz baud rate with a 7.3728MHz oscillator

    // Initialize the LCD
    lcd_init();

    // Clear the display
    lcd_send_command(0x01);  // Clear display
    _delay_ms(15);           // Ensure the screen has time to clear

    // Write a simple character to the screen
    uint8_t test[] = {'H'};
    lcd_write_string(test, 1);  // Write a single character

    // Infinite loop to keep the program running
    while (1) {}

    return 0;
}
