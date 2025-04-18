/*
 * Basic Motor Control for Robot Dog using ATmega328P (Bare-metal AVR C)
 * This code provides basic motor control with safety features
 *
 * Hardware Setup:
 *  - ATmega328P running at 7.3728MHz
 *  - L298N Motor Driver
 *  - Two DC Motors
 *
 * Pin Connections:
 *  - Motor A Direction: PD2, PD3
 *  - Motor B Direction: PD4, PD5
 */

#define F_CPU 7372800UL    // Correct CPU frequency for your setup

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// Motor control pin definitions
#define MOTOR_A_PIN1    PD6
#define MOTOR_A_PIN2    PD7
#define MOTOR_B_PIN1    PD4
#define MOTOR_B_PIN2    PD5
#define MOTOR_PIN PB2

// Initialize motor control pins
void init_motors(void) {
    // Set direction pins as outputs
    DDRD |= (1 << MOTOR_A_PIN1) | (1 << MOTOR_A_PIN2) |
            (1 << MOTOR_B_PIN1) | (1 << MOTOR_B_PIN2);
}

// Set motor A direction
void set_motor_a(uint8_t direction) {
    // Safety check: prevent both direction pins being HIGH
    if (direction == 0) {  // Forward
        PORTD &= ~(1 << MOTOR_A_PIN2);
        PORTD |= (1 << MOTOR_A_PIN1);
    } else if (direction == 1) {  // Reverse
        PORTD &= ~(1 << MOTOR_A_PIN1);
        PORTD |= (1 << MOTOR_A_PIN2);
    } else {  // Stop
        PORTD &= ~((1 << MOTOR_A_PIN1) | (1 << MOTOR_A_PIN2));
    }
}

// Set motor B direction
void set_motor_b(uint8_t direction) {
    // Safety check: prevent both direction pins being HIGH
    if (direction == 0) {  // Forward
        PORTD &= ~(1 << MOTOR_B_PIN2);
        PORTD |= (1 << MOTOR_B_PIN1);
    } else if (direction == 1) {  // Reverse
        PORTD &= ~(1 << MOTOR_B_PIN1);
        PORTD |= (1 << MOTOR_B_PIN2);
    } else {  // Stop
        PORTD &= ~((1 << MOTOR_B_PIN1) | (1 << MOTOR_B_PIN2));
    }
}

// Emergency stop both motors
void emergency_stop(void) {
    // Disable both motors
    PORTD &= ~((1 << MOTOR_A_PIN1) | (1 << MOTOR_A_PIN2) |
               (1 << MOTOR_B_PIN1) | (1 << MOTOR_B_PIN2));
}


void init_pwm(void) {
    // Configure PB1 as output
    DDRB |= (1 << MOTOR_PIN);

    // Fast PWM, 8-bit, clear OC1A on compare match, prescaler=8
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11);
}

void set_motor_speed(uint8_t speed) {
    OCR1A = speed;
}

int main(void) {
    // Initialize motors
    init_motors();
    init_pwm();
    
    // Test sequence
    while (1) {
        // Forward
        set_motor_a(0);
        set_motor_b(0);
        set_motor_speed(255); // Set PWM to high
        _delay_ms(2000);
        
        // Stop
        set_motor_a(2);
        set_motor_b(2);
        set_motor_speed(0); // Stop the motor
        _delay_ms(1000);
        
        // Reverse
        set_motor_a(1);
        set_motor_b(1);
        set_motor_speed(255); // Set PWM to high
        _delay_ms(2000);
        
        // Stop
        set_motor_a(2);
        set_motor_b(2);
        set_motor_speed(0); // Stop the motor
        _delay_ms(1000);
        
        // Turn right
        set_motor_a(0);
        set_motor_b(1);
        set_motor_speed(255); // Set PWM to high
        _delay_ms(2000);
        
        // Stop
        set_motor_a(2);
        set_motor_b(2);
        set_motor_speed(0); // Stop the motor
        _delay_ms(1000);
        
        // Turn left
        set_motor_a(1);
        set_motor_b(0);
        set_motor_speed(255); // Set PWM to high
        _delay_ms(2000);
        
        // Stop
        set_motor_a(2);
        set_motor_b(2);
        set_motor_speed(0); // Stop the motor
        _delay_ms(1000);
    }
    
    return 0;
}
 