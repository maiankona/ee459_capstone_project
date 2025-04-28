/*********************************************************************
*       LCD Interface Code Functions
*
* Revision History
* Date     Author      Description
* 3/1/25   C. Nystrom  
*********************************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "i2c.h"
#include "lcd.h"
//#include "serial.h"

#define LCD_ADDR 0x78 

uint8_t LCD_init();
uint8_t LCD_send_command(uint8_t command);
uint8_t LCD_send_data(uint8_t data);
uint8_t LCD_write_string(const char* str, uint8_t row);


uint8_t LCD_init(){
    uint8_t status = 0;
    _delay_ms(500);

    status = LCD_send_command(0x38);  // 8-bit mode, 2 lines
    if (status != 0){
        //serial_out_string("LCD Initialization FAILURE\r\n");
        return status;
    }
    _delay_us(120);

    status = LCD_send_command(0x0F);  // Display ON, Cursor OFF
    if (status != 0){
        //serial_out_string("LCD Initialization FAILURE\r\n");
        return status;
    }
    _delay_us(120);

    status = LCD_send_command(0x01);  // Clear display
    if (status != 0){
        //serial_out_string("LCD Initialization FAILURE\r\n");
        return status;
    }
    _delay_ms(15);

    status = LCD_send_command(0x06);  // Entry mode: Increment, no shift
    if (status != 0){
        //serial_out_string("LCD Initialization FAILURE\r\n");
        return status;
    }
    _delay_us(120);

    //serial_out_string("LCD Initialization SUCCESS\r\n");
    return status;
}

// Send command to the LCD
uint8_t LCD_send_command(uint8_t command){
    uint8_t status = 0;
    uint8_t data[2];
    data[0] = 0x80;  // Control byte: 0x80 for command
    data[1] = command;   // Command to send

    status = i2c_io(LCD_ADDR, data, 2, NULL, 0);  // Write command
    if (status != 0){
        //serial_out_string("LCD command send FAILURE\r\n");
        return status;
    }

    _delay_us(120);  // Wait for command to execute
    return status;
}

// Send data (character) to the LCD
uint8_t LCD_send_data(uint8_t data){
    uint8_t status = 0;
    uint8_t send_data[2];
    send_data[0] = 0x40;  // Control byte: 0x40 for data
    send_data[1] = data;  // Character data to send

    status = i2c_io(LCD_ADDR, send_data, 2, NULL, 0);  // Write data
    if (status != 0){
        //serial_out_string("LCD command send FAILURE\r\n");
        return status;
    }
    _delay_us(50);  // Wait for data to be written

    return status;
}

// Write a string to the LCD
uint8_t LCD_write_string(const char* str, uint8_t row){
    uint8_t status = 0;
    uint8_t row_addresses[4] = {0x80, 0xC0, 0x94, 0xD4};  // Start addresses for 4 rows

    if (row > 3){
        return 1;
    }

    status = LCD_send_command(row_addresses[row]);

    for (int i = 0; i < 20; i++){
        status = LCD_send_data(' ');  // Write a space character to clear the line
    }

    status = LCD_send_command(row_addresses[row]);

    while (*str){
        status = LCD_send_data(*str);  // Send each character to the LCD
        if (status != 0){
            //serial_out_string("LCD Print FAILURE\r\n");
            return status;
        }
        str++;
    }

    return status;
}

void print_heart(){
    uint8_t status = 0;

    // Custom heart character in 5x8
    uint8_t heart[8] = {
        0b00000,
        0b01010,
        0b11111,
        0b11111,
        0b11111,
        0b01110,
        0b00100,
        0b00000
    };

    // Set CGRAM address to 0x00 (start of character 0)
    status = LCD_send_command(0x40);
    if (status != 0) return;

    // Write heart shape to CGRAM
    for (int i = 0; i < 8; i++) {
        status = LCD_send_data(heart[i]);
        if (status != 0) return;
    }

    // Set cursor to row 2, column 0
    status = LCD_send_command(0x94);  // Row 2 start address
    if (status != 0) return;

    // Display the heart character (custom char in CGRAM slot 0)
    LCD_send_data(0x00);
}