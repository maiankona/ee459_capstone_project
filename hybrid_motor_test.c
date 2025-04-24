/*
 * Dual-PWM Motor Control for ATmega328P
 *  - Motor A (Front Left) on PD6/PD7 with PWM on PB2 (OC1B)
 *  - Motor B (Front Right) on PD4/PD5 with PWM on PD3 (OC2B)
 *  - ATmega328P @ 7.3728 MHz
 */

#define F_CPU 7372800UL
#include <avr/io.h>
#include <util/delay.h>

// Motor A (Front Left) direction pins
#define MOTOR_A_PIN1   PD6
#define MOTOR_A_PIN2   PD7
// PWM pin for Motor A (OC1B)
#define MOTOR_A_PWM    PB2

// Motor B (Front Right) direction pins
#define MOTOR_B_PIN1   PD4
#define MOTOR_B_PIN2   PD5
// PWM pin for Motor B (OC2B)
#define MOTOR_B_PWM    PD3

void init_motors(void) {
    // Set Motor A and B direction pins as outputs
    DDRD |= (1 << MOTOR_A_PIN1)
          | (1 << MOTOR_A_PIN2)
          | (1 << MOTOR_B_PIN1)
          | (1 << MOTOR_B_PIN2);
}

void init_pwm(void) {
    // --- Motor A on OC1B (PB2) ---
    DDRB |= (1<<MOTOR_A_PWM);
    TCCR1A = (1<<COM1B1)|(1<<WGM10);    // Fast PWM, 8‑bit, non‑inverting
    TCCR1B = (1<<WGM12)|(1<<CS11);      // prescaler = 8
    OCR1B = 0;

    // --- Motor B on OC2B (PD3) ---
    DDRD |= (1<<MOTOR_B_PWM);
    TCCR2A = (1<<COM2B1)|(1<<WGM21)|(1<<WGM20); // Fast PWM, 8‑bit, non‑inverting
    TCCR2B = (1<<CS21);                         // prescaler = 8
    OCR2B = 0;
}


void set_motor_a_backward(void) {
    // Dir1 HIGH, Dir2 LOW
    PORTD |=  (1 << MOTOR_A_PIN1);
    PORTD &= ~(1 << MOTOR_A_PIN2);
}

void set_motor_a_forward(void) {
    PORTD &= ~(1 << MOTOR_A_PIN1);
    PORTD |=  (1 << MOTOR_A_PIN2);
}

void set_motor_a_stop(void) {
    // Both direction pins LOW, PWM = 0
    PORTD &= ~((1 << MOTOR_A_PIN1) | (1 << MOTOR_A_PIN2));
    OCR1B = 0;
}

void set_motor_a_speed(uint8_t speed) {
    OCR1B = speed;  // 0–255 duty cycle
}

void set_motor_b_backward(void) {
    PORTD |=  (1 << MOTOR_B_PIN1);
    PORTD &= ~(1 << MOTOR_B_PIN2);
}

void set_motor_b_forward(void) {
    PORTD &= ~(1 << MOTOR_B_PIN1);
    PORTD |=  (1 << MOTOR_B_PIN2);
}

void set_motor_b_stop(void) {
    PORTD &= ~((1 << MOTOR_B_PIN1) | (1 << MOTOR_B_PIN2));
    OCR2B = 0;
}

void set_motor_b_speed(uint8_t speed) {
    OCR2B = speed;  // 0–255 duty cycle
}

int main(void) {
    init_motors();
    init_pwm();

    _delay_ms(1000);  // let hardware settle

    while (1) {
        // --- Both motors forward at full speed ---
        /*set_motor_a_forward();
        set_motor_b_forward();
        set_motor_a_speed(128);  // full speed A
        set_motor_b_speed(128);  // full speed B
        _delay_ms(1000);*/
    }

    return 0;
}

