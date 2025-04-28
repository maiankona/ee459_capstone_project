#include "ruff_route.h"

void init_motors(void) {
    // Set direction pins as outputs
    DDRD |= (1 << MOTOR_A_DIR1) | (1 << MOTOR_A_DIR2) |
            (1 << MOTOR_B_DIR1) | (1 << MOTOR_B_DIR2);
}

void init_pwm(void) {
    // Timer1 for Motor A (OC1B on PB2)
    DDRB |= (1 << MOTOR_PWM);
    TCCR1A = (1 << COM1B1) | (1 << WGM10);    // Fast PWM, 8-bit, non-inverting
    TCCR1B = (1 << WGM12) | (1 << CS11);      // prescaler = 8
    OCR1B = 0;

    // Timer2 for Motor B (OC2B on PD3)
    DDRD |= (1 << MOTOR_B_DIR1);
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // Fast PWM, 8-bit, non-inverting
    TCCR2B = (1 << CS21);                         // prescaler = 8
    OCR2B = 0;
}

void set_motor_speed(uint8_t motor, uint8_t speed) {
    if (motor == 0) {  // Motor A
        OCR1B = speed;
    } else {  // Motor B
        OCR2B = speed;
    }
}

void motor_control(uint8_t motor, uint8_t direction, uint8_t speed) {
    if (motor == 0) {  // Motor A
        if (direction == 0) {  // Forward
            PORTD &= ~(1 << MOTOR_A_DIR2);
            PORTD |= (1 << MOTOR_A_DIR1);
        } else if (direction == 1) {  // Reverse
            PORTD &= ~(1 << MOTOR_A_DIR1);
            PORTD |= (1 << MOTOR_A_DIR2);
        } else {  // Stop
            PORTD &= ~((1 << MOTOR_A_DIR1) | (1 << MOTOR_A_DIR2));
        }
        set_motor_speed(0, speed);
    } else {  // Motor B
        if (direction == 0) {  // Forward
            PORTD &= ~(1 << MOTOR_B_DIR2);
            PORTD |= (1 << MOTOR_B_DIR1);
        } else if (direction == 1) {  // Reverse
            PORTD &= ~(1 << MOTOR_B_DIR1);
            PORTD |= (1 << MOTOR_B_DIR2);
        } else {  // Stop
            PORTD &= ~((1 << MOTOR_B_DIR1) | (1 << MOTOR_B_DIR2));
        }
        set_motor_speed(1, speed);
    }
}

void stop_motors(void) {
    motor_control(0, 2, 0);  // Stop Motor A
    motor_control(1, 2, 0);  // Stop Motor B
}

void forward(uint8_t speed) {
    motor_control(0, 0, speed);  // Motor A forward
    motor_control(1, 0, speed);  // Motor B forward
}

void reverse(uint8_t speed) {
    motor_control(0, 1, speed);  // Motor A reverse
    motor_control(1, 1, speed);  // Motor B reverse
}

void turn_left(uint8_t speed) {
    motor_control(0, 1, speed);  // Motor A reverse
    motor_control(1, 0, speed);  // Motor B forward
}

void turn_right(uint8_t speed) {
    motor_control(0, 0, speed);  // Motor A forward
    motor_control(1, 1, speed);  // Motor B reverse
} 