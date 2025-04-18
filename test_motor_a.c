#include <avr/io.h>
#include <util/delay.h>

#define MOTOR_A_PIN1 PD6
#define MOTOR_A_PIN2 PD7
#define MOTOR_PIN PB2

void init_motor_a(void) {
    // Set direction pins as outputs
    DDRD |= (1 << MOTOR_A_PIN1) | (1 << MOTOR_A_PIN2);
}

void init_pwm(void) {
    // Configure PB2 as output
    DDRB |= (1 << MOTOR_PIN);

    // Fast PWM, 8-bit, clear OC1A on compare match, prescaler=8
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11);
}

void set_motor_a_forward(void) {
    PORTD &= ~(1 << MOTOR_A_PIN2);
    PORTD |= (1 << MOTOR_A_PIN1);
}

void set_motor_speed(uint8_t speed) {
    OCR1A = speed;
}

int main(void) {
    init_motor_a();
    init_pwm();
    set_motor_a_forward();
    set_motor_speed(255); // Set PWM to high for full speed

    while (1) {
        // Motor A will continue to run forward indefinitely
        _delay_ms(1000); // Delay to prevent CPU from being too busy
    }
    return 0;
} 