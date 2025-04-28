#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define F_CPU 16000000UL // Define CPU clock frequency (16 MHz for ATmega328P)

#define LCD_ADDR 0 // Change this to the actual I2C address of your LCD (check your LCD datasheet)

// Function to initialize the I2C communication
void I2C_init() {
    // Set the SCL and SDA pins to be outputs
    DDRC |= (1 << PC4) | (1 << PC5);
    // Set the bitrate (clock speed) for I2C communication
    TWBR = 72; // Adjust this value for 100kHz I2C speed with a 16MHz clock
}

// Function to start an I2C transmission
void I2C_start() {
    TWCR = (1 << TWSTA) | (1 << TWINT) | (1 << TWEN); // Send start condition
    while (!(TWCR & (1 << TWINT))); // Wait for transmission to complete
}

// Function to stop I2C communication
void I2C_stop() {
    TWCR = (1 << TWSTO) | (1 << TWINT) | (1 << TWEN); // Send stop condition
    while (TWCR & (1 << TWSTO)); // Wait for stop condition to finish
}

// Function to send a byte via I2C
void I2C_send(uint8_t data) {
    TWDR = data; // Load the data into the TWDR register
    TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
    while (!(TWCR & (1 << TWINT))); // Wait for transmission to complete
}

// Function to write data to the LCD
void LCD_write(uint8_t data) {
    I2C_start();
    I2C_send(LCD_ADDR << 1);  // Send the I2C address (write)
    I2C_send(0x00);           // Send control byte (command mode)
    I2C_send(data);           // Send the data byte
    I2C_stop();
}

// Function to initialize the LCD (assuming it's compatible with the standard LCD controller)
void LCD_init() {
    _delay_ms(50);  // Wait for LCD to power up
    LCD_write(0x38); // 8-bit mode, 2 lines
    LCD_write(0x0C); // Display on, cursor off
    LCD_write(0x06); // Entry mode (increment cursor)
    LCD_write(0x01); // Clear display
    _delay_ms(2);    // Wait for clear display to finish
}

// Function to send a string to the LCD
void LCD_send_string(char *str) {
    while (*str) {
        LCD_write(*str++);
    }
}

int main(void) {
    I2C_init();  // Initialize I2C
    LCD_init();  // Initialize the LCD

    // Print "Hello" on the LCD
    LCD_send_string("Hello");

    while (1) {
        // Loop forever
    }

    return 0;
}
