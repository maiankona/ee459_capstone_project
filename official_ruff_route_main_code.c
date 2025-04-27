/* Includes */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "i2c_lcd.h"   // (Assuming you have an I2C LCD library for Crystalfontz 20x4)
#include "gps.h"       // (Assuming you have a basic UART GPS library)

/* Motor Pins */
#define MOTOR_A_PIN1   PD6
#define MOTOR_A_PIN2   PD7
#define MOTOR_A_PWM    PB2

#define MOTOR_B_PIN1   PD4
#define MOTOR_B_PIN2   PD5
#define MOTOR_B_PWM    PB2  // ?? (same as A? -- assumed shared PWM)

/* Button Pins */
#define BUTTON_LEFT    PC1
#define BUTTON_SELECT  PC2
#define BUTTON_RIGHT   PC3

/* LCD Control Pin */
#define LCD_CSB        PD3

/* State Machine States */
typedef enum {
    STATE_INITIAL,
    STATE_ROUTE1,
    STATE_ROUTE2,
    STATE_SLOW,
    STATE_MEDIUM,
    STATE_FAST,
    STATE_LETSGO,
    STATE_CONGRATS
} State;

volatile State currentState = STATE_INITIAL;

/* Variables */
volatile uint32_t hoursElapsed = 0;
volatile uint8_t speed = 0; // 1 = slow, 2 = medium, 3 = fast

/* Function Prototypes */
void init_pins(void);
void motors_off(void);
void set_speed(uint8_t speed_level);
void lcd_display_initial(void);
void lcd_display_routes(void);
void lcd_display_speed(void);
void lcd_display_letsgo(void);
void lcd_display_congrats(void);
void update_miles_walked(void);
uint8_t button_pressed(void);

/* Main Program */
int main(void)
{
    init_pins();
    lcd_init();
    gps_init();

    motors_off();
    lcd_display_initial();

    sei(); // Enable interrupts (for timer if used)

    while (1)
    {
        switch (currentState)
        {
            case STATE_INITIAL:
                if (button_pressed())
                    currentState = STATE_ROUTE1;
                break;

            case STATE_ROUTE1:
                lcd_display_routes();
                if (PINC & (1 << BUTTON_LEFT) || PINC & (1 << BUTTON_RIGHT))
                    currentState = STATE_ROUTE2;
                else if (PINC & (1 << BUTTON_SELECT))
                    currentState = STATE_SLOW;
                break;

            case STATE_ROUTE2:
                lcd_display_routes();
                if (PINC & (1 << BUTTON_LEFT) || PINC & (1 << BUTTON_RIGHT))
                    currentState = STATE_ROUTE1;
                else if (PINC & (1 << BUTTON_SELECT))
                    currentState = STATE_SLOW;
                break;

            case STATE_SLOW:
                speed = 1;
                set_speed(speed);
                lcd_display_speed();
                if (PINC & (1 << BUTTON_SELECT))
                    currentState = STATE_LETSGO;
                else if (PINC & (1 << BUTTON_LEFT))
                    currentState = STATE_FAST;
                else if (PINC & (1 << BUTTON_RIGHT))
                    currentState = STATE_MEDIUM;
                break;

            case STATE_MEDIUM:
                speed = 2;
                set_speed(speed);
                lcd_display_speed();
                if (PINC & (1 << BUTTON_SELECT))
                    currentState = STATE_LETSGO;
                else if (PINC & (1 << BUTTON_LEFT))
                    currentState = STATE_SLOW;
                else if (PINC & (1 << BUTTON_RIGHT))
                    currentState = STATE_FAST;
                break;

            case STATE_FAST:
                speed = 3;
                set_speed(speed);
                lcd_display_speed();
                if (PINC & (1 << BUTTON_SELECT))
                    currentState = STATE_LETSGO;
                else if (PINC & (1 << BUTTON_LEFT))
                    currentState = STATE_MEDIUM;
                else if (PINC & (1 << BUTTON_RIGHT))
                    currentState = STATE_SLOW;
                break;

            case STATE_LETSGO:
                lcd_display_letsgo();
                update_miles_walked();
                if (gps_route_complete())
                    currentState = STATE_CONGRATS;
                break;

            case STATE_CONGRATS:
                lcd_display_congrats();
                _delay_ms(30000); // 30 seconds wait
                currentState = STATE_INITIAL;
                motors_off();
                lcd_display_initial();
                break;
        }
    }
}

/* Helper Functions */

void init_pins(void)
{
    // Motor pins as output
    DDRD |= (1 << MOTOR_A_PIN1) | (1 << MOTOR_A_PIN2) | (1 << MOTOR_B_PIN1) | (1 << MOTOR_B_PIN2) | (1 << LCD_CSB);
    DDRB |= (1 << MOTOR_A_PWM);

    // Buttons as input
    DDRC &= ~((1 << BUTTON_LEFT) | (1 << BUTTON_SELECT) | (1 << BUTTON_RIGHT));
    PORTC |= (1 << BUTTON_LEFT) | (1 << BUTTON_SELECT) | (1 << BUTTON_RIGHT); // enable pull-ups
}

void motors_off(void)
{
    PORTD &= ~((1 << MOTOR_A_PIN1) | (1 << MOTOR_A_PIN2) | (1 << MOTOR_B_PIN1) | (1 << MOTOR_B_PIN2));
    OCR1A = 0;  // PWM off
}

void set_speed(uint8_t speed_level)
{
    switch(speed_level)
    {
        case 1:
            OCR1A = 43;
            break;
        case 2:
            OCR1A = 86;
            break;
        case 3:
            OCR1A = 128;
            break;
    }
}

void lcd_display_initial(void)
{
    lcd_clear();
    lcd_set_cursor(0,0);
    lcd_write_string("WELCOME");
    lcd_set_cursor(1,0);
    lcd_write_string("Select your route");
    lcd_set_cursor(2,0);
    lcd_write_string("WOOF");
}

void lcd_display_routes(void)
{
    lcd_clear();
    lcd_set_cursor(0,0);
    lcd_write_string("Route 1");
    lcd_set_cursor(1,0);
    lcd_write_string("Route 2");
}

void lcd_display_speed(void)
{
    lcd_clear();
    lcd_set_cursor(0,0);
    lcd_write_string("Speed:");

    lcd_set_cursor(1,0);
    if (speed == 1) lcd_write_string("**slow**  medium  fast");
    else if (speed == 2) lcd_write_string("slow  **medium**  fast");
    else if (speed == 3) lcd_write_string("slow  medium  **fast**");
}

void lcd_display_letsgo(void)
{
    lcd_clear();
    lcd_set_cursor(0,0);
    lcd_write_string("LETS GO!");
    lcd_set_cursor(1,0);
    lcd_write_string("miles walked: 00");
}

void lcd_display_congrats(void)
{
    lcd_clear();
    lcd_set_cursor(0,0);
    lcd_write_string("Route finished--");
    lcd_set_cursor(1,0);
    lcd_write_string("CONGRATS");
}

void update_miles_walked(void)
{
    static uint16_t miles = 0;
    miles = gps_get_miles();

    lcd_set_cursor(1,13);
    lcd_write_number(miles); // Assuming your LCD lib can write numbers directly
}

uint8_t button_pressed(void)
{
    if (!(PINC & (1 << BUTTON_LEFT)) || !(PINC & (1 << BUTTON_SELECT)) || !(PINC & (1 << BUTTON_RIGHT)))
        return 1;
    return 0;
}
