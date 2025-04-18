#include <avr/io.h>
#include <util/delay.h>

// Motor Definitions
#define MOTOR_A_PIN1 PD6  // Front Right Motor Direction
#define MOTOR_A_PIN2 PD7  // Back Right Motor Direction
#define MOTORA_PWM_PIN PB2  // PWM pin for Right Motors (OC1B - OCR1B)

#define MOTOR_B_PIN1 PD4  // Front Left Motor Direction
#define MOTOR_B_PIN2 PD5  // Back Left Motor Direction
#define MOTORB_PWM_PIN PD3  // PWM pin for Left Motors (OC2B - OCR2B)

// Initialize motor control pins
void init_motors(void) {
    DDRD |= (1 << MOTOR_A_PIN1) | (1 << MOTOR_A_PIN2) | (1 << MOTOR_B_PIN1) | (1 << MOTOR_B_PIN2);
    
    DDRB |= (1 << MOTORA_PWM_PIN);  // PB2 - Output (Right Motors PWM)
    DDRD |= (1 << MOTORB_PWM_PIN);  // PD3 - Output (Left Motors PWM)
}

// Initialize PWM on Timer1 (Right Motors) and Timer2 (Left Motors)
void init_pwm(void) {
    // Timer1 setup for Right Motors (OC1B on PB2)
    TCCR1A = (1 << COM1B1) | (1 << WGM10);  // Fast PWM 8-bit
    TCCR1B = (1 << WGM12) | (1 << CS11);    // Prescaler = 8

    // Timer2 setup for Left Motors (OC2B on PD3)
    TCCR2A = (1 << COM2B1) | (1 << WGM20);  // Fast PWM 8-bit
    TCCR2B = (1 << WGM22) | (1 << CS21);    // Prescaler = 8
}

// Set motors forward
void set_motors_forward(void) {
    // Right side forward
    PORTD |= (1 << MOTOR_A_PIN1);
    PORTD &= ~(1 << MOTOR_A_PIN2);

    // Left side forward
    PORTD |= (1 << MOTOR_B_PIN1);
    PORTD &= ~(1 << MOTOR_B_PIN2);
}

// Set speed for right motors (OCR1B)
void set_right_motor_speed(uint8_t speed) {
    OCR1B = speed;
}

// Set speed for left motors (OCR2B)
void set_left_motor_speed(uint8_t speed) {
    OCR2B = speed;
}

// Turn left: slow down left motors, keep right motors running
void turn_left(void) {
    set_right_motor_speed(200); // Right motors fast
    set_left_motor_speed(50);   // Left motors slow
}

// Turn right: slow down right motors, keep left motors running
void turn_right(void) {
    set_right_motor_speed(50);  // Right motors slow
    set_left_motor_speed(200);  // Left motors fast
}

int main(void) {
    init_motors();
    init_pwm();
    set_motors_forward();

    while (1) {
        // Full speed
        set_right_motor_speed(255);
        set_left_motor_speed(255);
        _delay_ms(2000);

        // Medium speed
        set_right_motor_speed(150);
        set_left_motor_speed(150);
        _delay_ms(2000);

        // Slow speed
        set_right_motor_speed(75);
        set_left_motor_speed(75);
        _delay_ms(2000);

        // Turn Left
        turn_left();
        _delay_ms(2000);

        // Turn Right
        turn_right();
        _delay_ms(2000);
    }

    return 0;
}
