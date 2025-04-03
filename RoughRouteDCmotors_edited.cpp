#include <avr/io.h>
#include <util/delay.h>

// ===== PWM TIMER1 (OC1A = PB1, OC1B = PB2) =====
void pwm_timer1_init() {
    // Set PB1 (OC1A) and PB2 (OC1B) as output
    DDRB |= (1 << PB1) | (1 << PB2);

    // Fast PWM 8-bit, non-inverting mode
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);  // Prescaler = 64
}

// ===== PWM TIMER2 (OC2A = PB3, OC2B = PD3) =====
void pwm_timer2_init() {
    // Set PB3 (OC2A) and PD3 (OC2B) as output
    DDRB |= (1 << PB3);
    DDRD |= (1 << PD3);

    // Fast PWM, non-inverting mode
    TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
    TCCR2B = (1 << CS22); // Prescaler = 64
}

// Set motor speed: 0-255
void set_motor1_speed(uint8_t speedA, uint8_t speedB) {
    OCR1A = speedA; // Motor1 A
    OCR1B = speedB; // Motor1 B
}

void set_motor2_speed(uint8_t speedA, uint8_t speedB) {
    OCR2A = speedA; // Motor2 A
    OCR2B = speedB; // Motor2 B
}

void init_all() {
    pwm_timer1_init();
    pwm_timer2_init();
}

int main() {
    init_all();

    while (1) {
        // Motor 1: Forward (PWM on A, 0 on B)
        set_motor1_speed(200, 0);
        // Motor 2: Backward (0 on A, PWM on B)
        set_motor2_speed(0, 200);
        _delay_ms(2000);

        // Stop both motors
        set_motor1_speed(0, 0);
        set_motor2_speed(0, 0);
        _delay_ms(1000);

        // Reverse direction
        set_motor1_speed(0, 200);
        set_motor2_speed(200, 0);
        _delay_ms(2000);

        // Stop
        set_motor1_speed(0, 0);
        set_motor2_speed(0, 0);
        _delay_ms(1000);
    }
}
