#include "lcd.h"
#include "i2c.h"
#include <util/delay.h>
#include <stddef.h>  // Include for NULL

// Function to send a command to the LCD
void lcd_send_command(uint8_t cmd) {
    uint8_t data[2];
    data[0] = LCD_CMD;  // Command byte
    data[1] = cmd;      // Command value
    i2c_io(LCD_ADDR, data, 2, NULL, 0);
    _delay_us(120); // Wait after command
}

// Function to initialize the LCD
void lcd_init(void) {
    _delay_ms(500); // Delay to ensure the LCD is powered up

    // Initialize the LCD to 2-line mode, 8-bit interface
    lcd_send_command(0x38);  // Function Set: 2 lines
    _delay_us(120);

    // Turn on the display and cursor
    lcd_send_command(0x0F);  // Display On, Cursor On, Blink On
    _delay_us(120);

    // Clear the display
    lcd_send_command(0x01);  // Clear display
    _delay_ms(15);           // Wait after clear

    // Set entry mode: cursor shifts right
    lcd_send_command(0x06);  // Entry Mode Set
    _delay_us(120);
}

// Function to write a string to the LCD
void lcd_write_string(uint8_t *str, uint8_t len) {
    uint8_t data[2];
    for (uint8_t i = 0; i < len; i++) {
        data[0] = LCD_DATA;   // Data byte
        data[1] = str[i];      // Character to display
        i2c_io(LCD_ADDR, data, 2, NULL, 0);
        _delay_us(120);        // Wait after each character
    }
}
